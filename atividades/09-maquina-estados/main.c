#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "int_i2c.h"
#include "esp_log.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"

enum StateMachine {
  READ_TEMP,
  PLAY_ALARM,
  UPDATE_LCD,
  UPDATE_LEDS,
  WRITE_SD_CARD,
  TEST_SD_CARD
};

#define LED0 4
#define LED1 5
#define LED2 6
#define LED3 7
#define BOTAO_INCREMENTO 38
#define BOTAO_DECREMENTO 39
#define BUZZER_GPIO 14
#define I2C_SDA 2
#define I2C_SCL 3

#define SENSOR_GPIO 1
#define SENSOR_CHANNEL ADC1_CHANNEL_0
#define BETA 3950.0
#define ADC_MAX 4095.0
#define T0_KELVIN 298.15

#define PIN_NUM_MISO GPIO_NUM_13
#define PIN_NUM_MOSI GPIO_NUM_11
#define PIN_NUM_CLK GPIO_NUM_12
#define PIN_NUM_CS GPIO_NUM_10
#define MOUNT_POINT "/sdcard"
#define LOG_FILE_PATH MOUNT_POINT"/temp_log.csv"

#define TEMP_MINIMA 10
#define TEMP_MAXIMA 50
#define LOG_INTERVAL_MS 10000

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_ADDRESS 0x27
#define LCD_SIZE DISPLAY_16X02

static const char *TAG = "TEMPERATURE_MONITOR";

int temperatura_alarme = 25;
int temperatura_atual = 20;
bool alarme_ativado = false;
bool sd_card_ok = false;
uint64_t ultimo_log_time = 0;

lcd_i2c_handle_t lcd;

volatile bool botao_inc_flag = false;
volatile bool botao_dec_flag = false;
uint8_t last_inc_state = 1;
uint8_t last_dec_state = 1;
TickType_t last_inc_time = 0;
TickType_t last_dec_time = 0;
const TickType_t debounce_interval = pdMS_TO_TICKS(50);

static void IRAM_ATTR botao_incremento_isr_handler(void* arg);
static void IRAM_ATTR botao_decremento_isr_handler(void* arg);
void play_alarm();
void update_leds(int diff);
bool init_sd_card();
void log_temperature();
void create_log_header();
void read_log();
void tratar_botoes();

uint64_t millis() {
    return esp_timer_get_time() / 1000;
}

static void IRAM_ATTR botao_incremento_isr_handler(void* arg) {
    gpio_intr_disable(BOTAO_INCREMENTO);
    botao_inc_flag = true;
}

static void IRAM_ATTR botao_decremento_isr_handler(void* arg) {
    gpio_intr_disable(BOTAO_DECREMENTO);
    botao_dec_flag = true;
}

void gpio_setup() {
    gpio_config_t config_leds = {
        .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config_leds);

    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << BOTAO_INCREMENTO) | (1ULL << BOTAO_DECREMENTO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  
        .intr_type = GPIO_INTR_DISABLE 
    };
    gpio_config(&btn_config);

    gpio_set_intr_type(BOTAO_INCREMENTO, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(BOTAO_DECREMENTO, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO_INCREMENTO, botao_incremento_isr_handler, NULL);
    gpio_isr_handler_add(BOTAO_DECREMENTO, botao_decremento_isr_handler, NULL);
}

void ledc_setup() {
    ledc_timer_config_t pwm_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 2000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE
    };
    ledc_channel_config(&pwm_channel);
}

void i2c_setup() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void adc_setup() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_CHANNEL, ADC_ATTEN_DB_11);
}

void play_alarm() {
    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 2500);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    alarme_ativado = true;
    vTaskDelay(pdMS_TO_TICKS(100));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    alarme_ativado = false;
}

void update_leds(int diff) {
    bool alarme_ativo = diff <= 0;
    bool pisca = alarme_ativo && (esp_timer_get_time() / 500000) % 2;
    
    gpio_set_level(LED0, alarme_ativo ? pisca : (temperatura_atual <= temperatura_alarme - 2));
    gpio_set_level(LED1, alarme_ativo ? pisca : (temperatura_atual <= temperatura_alarme - 10));
    gpio_set_level(LED2, alarme_ativo ? pisca : (temperatura_atual <= temperatura_alarme - 15));
    gpio_set_level(LED3, alarme_ativo ? pisca : (temperatura_atual <= temperatura_alarme - 20));
}

float ler_temperatura_celsius() {
    int raw = adc1_get_raw(SENSOR_CHANNEL);
    float resistencia = 10000.0 / ((ADC_MAX / (float)raw) - 1.0);
    float temperatura_k = 1.0 / (log(resistencia / 10000.0) / BETA + (1.0 / T0_KELVIN));
    return temperatura_k - 273.15;
}

void update_display() {
    char buffer[20];
    lcd_i2c_cursor_set(&lcd, 0, 0);
    snprintf(buffer, sizeof(buffer), "Al: %d C", temperatura_alarme);
    lcd_i2c_print(&lcd, buffer);
    
    lcd_i2c_cursor_set(&lcd, 0, 1);
    snprintf(buffer, sizeof(buffer), "Temp: %d C", temperatura_atual);
    lcd_i2c_print(&lcd, buffer);
}

void tratar_botoes() {
    if (botao_inc_flag) {
        botao_inc_flag = false;
        uint8_t current_state = gpio_get_level(BOTAO_INCREMENTO);
        TickType_t now = xTaskGetTickCount();

        if (current_state == 0 && last_inc_state == 1) {
            if ((now - last_inc_time) >= debounce_interval) {
                if (temperatura_alarme < TEMP_MAXIMA) {
                    temperatura_alarme += 5;
                    ESP_LOGI(TAG, "Limite aumentado para: %d", temperatura_alarme);
                }
                last_inc_time = now;
            }
        }
        last_inc_state = current_state;
        gpio_intr_enable(BOTAO_INCREMENTO);
    }

    if (botao_dec_flag) {
        botao_dec_flag = false;
        uint8_t current_state = gpio_get_level(BOTAO_DECREMENTO);
        TickType_t now = xTaskGetTickCount();

        if (current_state == 0 && last_dec_state == 1) {
            if ((now - last_dec_time) >= debounce_interval) {
                if (temperatura_alarme > TEMP_MINIMA) {
                    temperatura_alarme -= 5;
                    ESP_LOGI(TAG, "Limite diminuido para: %d", temperatura_alarme);
                }
                last_dec_time = now;
            }
        }
        last_dec_state = current_state;
        gpio_intr_enable(BOTAO_DECREMENTO);
    }
}

bool init_sd_card() {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha no SPI: 0x%x", ret);
        return false;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na montagem: 0x%x", ret);
        return false;
    }
    
    create_log_header();
    return true;
}

void create_log_header() {
    struct stat st;
    if (stat(LOG_FILE_PATH, &st) == -1) {
        FILE *f = fopen(LOG_FILE_PATH, "w");
        if (f) {
            fprintf(f, "Data,Hora,Temperatura(C),Limite(C),Alarme\n");
            fclose(f);
        }
    }
}

void log_temperature() {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    FILE *f = fopen(LOG_FILE_PATH, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Erro ao abrir arquivo");
        return;
    }

    fprintf(f, "%04d-%02d-%02d,%02d:%02d:%02d,%d,%d,%s\n",
            timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
            temperatura_atual, temperatura_alarme, alarme_ativado ? "ON" : "OFF");
    
    fclose(f);
    ESP_LOGI(TAG, "Dados gravados no SD Card");
}

void read_log() {
    FILE *f = fopen(LOG_FILE_PATH, "r");
    if (f == NULL) return;

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        ESP_LOGI(TAG, "LOG: %s", line);
    }
    fclose(f);
}

void app_main() {
    ESP_LOGI(TAG, "Iniciando sistema");
    
    gpio_setup();
    ledc_setup();
    i2c_setup();
    adc_setup();
    
    // LCD
    lcd.address = LCD_ADDRESS;
    lcd.num = I2C_MASTER_NUM;
    lcd.backlight = 1;
    lcd.size = LCD_SIZE;
    lcd_i2c_init(&lcd);
    
    // SD Card
    sd_card_ok = init_sd_card();
    
    // Mensagem inicial
    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, "Monitor Temp");
    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, "Iniciando...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);

    // Variáveis da máquina de estados
    int state = READ_TEMP;
    int diff = 0;
    uint64_t last_display_update = 0;
    uint64_t last_log_read = 0;

    while (1) {
        tratar_botoes();

        switch (state) {
            case READ_TEMP:
                temperatura_atual = (int)ler_temperatura_celsius();
                diff = temperatura_alarme - temperatura_atual;
                state = PLAY_ALARM;
                break;
                
            case PLAY_ALARM:
                if (diff <= 0) {
                    play_alarm();
                } else {
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                    alarme_ativado = false;
                }
                state = UPDATE_LCD;
                break;
                
            case UPDATE_LCD:
                if (millis() - last_display_update > 500) {
                    last_display_update = millis();
                    update_display();
                }
                state = UPDATE_LEDS;
                break;
                
            case UPDATE_LEDS:
                update_leds(diff);
                state = WRITE_SD_CARD;
                break;
                
            case WRITE_SD_CARD:
                if (sd_card_ok && (millis() - ultimo_log_time >= LOG_INTERVAL_MS)) {
                    log_temperature();
                    ultimo_log_time = millis();
                }
                state = TEST_SD_CARD;
                break;
                
            case TEST_SD_CARD:
                if (millis() - last_log_read > 1500) {
                    last_log_read = millis();
                    if (sd_card_ok) {
                        read_log();
                    }
                }
                state = READ_TEMP;
                break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(15)); 
    }
}
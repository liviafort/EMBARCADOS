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

// Configurações do SD Card (adicionadas do código do amigo)
#define PIN_NUM_MISO GPIO_NUM_13
#define PIN_NUM_MOSI GPIO_NUM_11
#define PIN_NUM_CLK GPIO_NUM_12
#define PIN_NUM_CS GPIO_NUM_10
#define MOUNT_POINT "/sdcard"
#define LOG_FILE_PATH MOUNT_POINT"/temp_log.csv"

// Novas constantes para controle
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

// Funções para SD Card (adicionadas do código do amigo)
bool init_sd_card(void);
void log_temperature(float temp);
void create_log_header(void);
void check_sd_card(void);

// Função millis para controle de tempo
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

static void i2c_master_init() {
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

void configurar_botoes() {
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

void inicializar_pwm_buzzer() {
    ledc_timer_config_t pwm_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 2000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&pwm_channel);
}

void ligar_buzzer() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    alarme_ativado = true;
}

void desligar_buzzer() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    alarme_ativado = false;
}

float ler_temperatura_celsius() {
    int raw = adc1_get_raw(SENSOR_CHANNEL);
    float resistencia = 10000.0 / ((ADC_MAX / (float)raw) - 1.0);
    float temperatura_k = 1.0 / (log(resistencia / 10000.0) / BETA + (1.0 / T0_KELVIN));
    return temperatura_k - 273.15;
}

void atualizar_display() {
    char buffer[20];
    lcd_i2c_cursor_set(&lcd, 0, 0);
    snprintf(buffer, sizeof(buffer), "Al: %d C", temperatura_alarme);
    lcd_i2c_print(&lcd, buffer);
    lcd_i2c_cursor_set(&lcd, 0, 1);
    snprintf(buffer, sizeof(buffer), "Temp: %d C", temperatura_atual);
    lcd_i2c_print(&lcd, buffer);
}

void atualizar_leds(int temp, int alarme, bool alarme_ativo) {
    bool pisca = alarme_ativo && (esp_timer_get_time() / 500000) % 2;
    gpio_set_level(LED0, alarme_ativo ? pisca : (temp <= alarme - 2));
    gpio_set_level(LED1, alarme_ativo ? pisca : (temp <= alarme - 10));
    gpio_set_level(LED2, alarme_ativo ? pisca : (temp <= alarme - 15));
    gpio_set_level(LED3, alarme_ativo ? pisca : (temp <= alarme - 20));
}

bool init_sd_card() {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_card_t *card;
    
    ESP_LOGI(TAG, "Inicializando SD Card...");

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

    FILE *f = fopen(MOUNT_POINT"/test.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Falha no teste de escrita");
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        return false;
    }
    fprintf(f, "Teste SD Card\n");
    fclose(f);

    sdmmc_card_print_info(stdout, card);
    ESP_LOGI(TAG, "SD Card inicializado com sucesso");
    
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

void log_temperature(float temp) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    FILE *f = fopen(LOG_FILE_PATH, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Erro ao abrir arquivo");
        return;
    }

    fprintf(f, "%04d-%02d-%02d,%02d:%02d:%02d,%.1f,%d,%s\n",
            timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
            temp, temperatura_alarme, alarme_ativado ? "ON" : "OFF");
    
    fclose(f);
    ESP_LOGI(TAG, "Dados gravados no SD Card");
}

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando sistema");

    i2c_master_init();
    lcd.address = LCD_ADDRESS;
    lcd.num = I2C_MASTER_NUM;
    lcd.backlight = 1;
    lcd.size = LCD_SIZE;
    vTaskDelay(pdMS_TO_TICKS(100));
    lcd_i2c_init(&lcd);

    gpio_config_t cfg_leds = {
        .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&cfg_leds);

    configurar_botoes();
    inicializar_pwm_buzzer();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_CHANNEL, ADC_ATTEN_DB_11);

    // Inicialização do SD Card
    sd_card_ok = init_sd_card();
    if (!sd_card_ok) {
        lcd_i2c_cursor_set(&lcd, 0, 0);
        lcd_i2c_print(&lcd, "Erro SD Card!");
        lcd_i2c_cursor_set(&lcd, 0, 1);
        lcd_i2c_print(&lcd, "Verifique conexao");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, "Monitor Temp");
    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, "Iniciando...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);

    uint8_t last_inc_state = 1;
    uint8_t last_dec_state = 1;
    TickType_t last_inc_time = 0;
    TickType_t last_dec_time = 0;
    const TickType_t debounce_interval = pdMS_TO_TICKS(50);

    while (1) {
        // Tratamento dos botões
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

        // Leitura de temperatura e controle
        temperatura_atual = (int)ler_temperatura_celsius();

        bool alarme_ativo = temperatura_atual >= temperatura_alarme;
        if (alarme_ativo) {
            ligar_buzzer();
        } else {
            desligar_buzzer();
        }

        atualizar_leds(temperatura_atual, temperatura_alarme, alarme_ativo);
        atualizar_display();

        // Log de temperatura no SD Card
        if (sd_card_ok && (millis() - ultimo_log_time >= LOG_INTERVAL_MS)) {
            log_temperature(temperatura_atual);
            ultimo_log_time = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
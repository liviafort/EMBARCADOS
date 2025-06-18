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
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "int_i2c.h"
#include "esp_log.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"

// Definições de hardware
#define SEG_G 4
#define SEG_F 5
#define SEG_A 6
#define SEG_B 7
#define SEG_C 15
#define SEG_D 17
#define SEG_E 18

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

SemaphoreHandle_t xMutex;
int temperatura_alarme = 25;
int temperatura_atual = 20;
bool alarme_ativado = false;
bool sd_card_ok = false;
lcd_i2c_handle_t lcd;

QueueHandle_t button_queue;

static void IRAM_ATTR botao_incremento_isr_handler(void* arg) {
    int btn_id = 1;
    xQueueSendFromISR(button_queue, &btn_id, NULL);
}

static void IRAM_ATTR botao_decremento_isr_handler(void* arg) {
    int btn_id = 2;
    xQueueSendFromISR(button_queue, &btn_id, NULL);
}

void temp_task(void *pvParam);
void button_task(void *pvParam);
void alarm_task(void *pvParam);
void lcd_task(void *pvParam);
void display_task(void *pvParam);
void sd_task(void *pvParam);

void display_7seg(char digit);

void gpio_setup() {
    gpio_config_t config_seg = {
        .pin_bit_mask = (1ULL << SEG_A) | (1ULL << SEG_B) | (1ULL << SEG_C) |
                        (1ULL << SEG_D) | (1ULL << SEG_E) | (1ULL << SEG_F) |
                        (1ULL << SEG_G),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config_seg);

    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << BOTAO_INCREMENTO) | (1ULL << BOTAO_DECREMENTO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&btn_config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO_INCREMENTO, botao_incremento_isr_handler, NULL);
    gpio_isr_handler_add(BOTAO_DECREMENTO, botao_decremento_isr_handler, NULL);
    
    display_7seg(' ');
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
    adc1_config_channel_atten(SENSOR_CHANNEL, ADC_ATTEN_DB_12);
}

float ler_temperatura_celsius() {
    int raw = adc1_get_raw(SENSOR_CHANNEL);
    float resistencia = 10000.0 / ((ADC_MAX / (float)raw) - 1.0);
    float temperatura_k = 1.0 / (log(resistencia / 10000.0) / BETA + (1.0 / T0_KELVIN));
    return temperatura_k - 273.15;
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
    
    struct stat st;
    if (stat(LOG_FILE_PATH, &st) == -1) {
        FILE *f = fopen(LOG_FILE_PATH, "w");
        if (f) {
            fprintf(f, "Data,Hora,Temperatura(C),Limite(C),Alarme\n");
            fclose(f);
        }
    }
    return true;
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

// Implementação das threads
void temp_task(void *pvParam) {
    while(1) {
        float temp = ler_temperatura_celsius();
        
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            temperatura_atual = (int)temp;
            alarme_ativado = (temperatura_atual >= temperatura_alarme);
            xSemaphoreGive(xMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200)); // Leitura a cada 200ms
    }
}

bool botao_inc_flag = false;
bool botao_dec_flag = false;

static void IRAM_ATTR local_botao_incremento_isr_handler(void* arg) {
        gpio_intr_disable(BOTAO_INCREMENTO);
        botao_inc_flag = true;
    }

static void IRAM_ATTR local_botao_decremento_isr_handler(void* arg) {
        gpio_intr_disable(BOTAO_DECREMENTO);
        botao_dec_flag = true;
    }

void button_task(void *pvParam) {
    uint8_t last_inc_state = 1;
    uint8_t last_dec_state = 1;
    TickType_t last_inc_time = 0;
    TickType_t last_dec_time = 0;
    const TickType_t debounce_interval = pdMS_TO_TICKS(50);
    
    gpio_isr_handler_remove(BOTAO_INCREMENTO);
    gpio_isr_handler_remove(BOTAO_DECREMENTO);
    gpio_isr_handler_add(BOTAO_INCREMENTO, local_botao_incremento_isr_handler, NULL);
    gpio_isr_handler_add(BOTAO_DECREMENTO, local_botao_decremento_isr_handler, NULL);

    while(1) {
        if (botao_inc_flag) {
            botao_inc_flag = false;
            uint8_t current_state = gpio_get_level(BOTAO_INCREMENTO);
            TickType_t now = xTaskGetTickCount();

            if (current_state == 0 && last_inc_state == 1) {
                if ((now - last_inc_time) >= debounce_interval) {
                    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                        if (temperatura_alarme < TEMP_MAXIMA) {
                            temperatura_alarme += 5;
                            ESP_LOGI(TAG, "Limite aumentado para: %d", temperatura_alarme);
                        }
                        xSemaphoreGive(xMutex);
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
                    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                        if (temperatura_alarme > TEMP_MINIMA) {
                            temperatura_alarme -= 5;
                            ESP_LOGI(TAG, "Limite diminuido para: %d", temperatura_alarme);
                        }
                        xSemaphoreGive(xMutex);
                    }
                    last_dec_time = now;
                }
            }
            last_dec_state = current_state;
            gpio_intr_enable(BOTAO_DECREMENTO);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void alarm_task(void *pvParam) {
    while(1) {
        bool current_alarm;
        
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(100))) {
            current_alarm = alarme_ativado;
            xSemaphoreGive(xMutex);
        } else {
            continue;
        }
        
        if (current_alarm) {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 2500);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(100));
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(100)); // Pausa entre bipes
        } else {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(200)); // Verificação mais lenta
        }
    }
}

void lcd_task(void *pvParam) {
    char buffer[20];
    while(1) {
        int temp_atual, temp_alarme;
        
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(100))) {
            temp_atual = temperatura_atual;
            temp_alarme = temperatura_alarme;
            xSemaphoreGive(xMutex);
        } else {
            continue;
        }
        
        lcd_i2c_cursor_set(&lcd, 0, 0);
        snprintf(buffer, sizeof(buffer), "NTC: %d C", temp_atual);
        lcd_i2c_print(&lcd, buffer);
        
        lcd_i2c_cursor_set(&lcd, 0, 1);
        snprintf(buffer, sizeof(buffer), "Alarme: %d C", temp_alarme);
        lcd_i2c_print(&lcd, buffer);
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Atualização a cada 500ms
    }
}

// Função para controlar o display de 7 segmentos - CORRIGIDA para ânodo comum
void display_7seg(char digit) {
    // Para display de ânodo comum (COM no VCC):
    // - Segmento apagado = HIGH (1)
    // - Segmento aceso = LOW (0)
    
    // Primeiro, apagar todos os segmentos (colocar em HIGH)
    gpio_set_level(SEG_A, 1);
    gpio_set_level(SEG_B, 1);
    gpio_set_level(SEG_C, 1);
    gpio_set_level(SEG_D, 1);
    gpio_set_level(SEG_E, 1);
    gpio_set_level(SEG_F, 1);
    gpio_set_level(SEG_G, 1);

    // Acender os segmentos conforme o dígito (colocar em LOW)
    switch (digit) {
        case '0': // Dígito 0: A,B,C,D,E,F
            gpio_set_level(SEG_A, 0);
            gpio_set_level(SEG_B, 0);
            gpio_set_level(SEG_C, 0);
            gpio_set_level(SEG_D, 0);
            gpio_set_level(SEG_E, 0);
            gpio_set_level(SEG_F, 0);
            break;
            
        case '3': // Dígito 3: A,B,C,D,G
            gpio_set_level(SEG_A, 0);
            gpio_set_level(SEG_B, 0);
            gpio_set_level(SEG_C, 0);
            gpio_set_level(SEG_D, 0);
            gpio_set_level(SEG_G, 0);
            break;
            
        case '7': // Dígito 7: A,B,C
            gpio_set_level(SEG_A, 0);
            gpio_set_level(SEG_B, 0);
            gpio_set_level(SEG_C, 0);
            break;
            
        case 'D': // Dígito D: B,C,D,E,G
            gpio_set_level(SEG_B, 0);
            gpio_set_level(SEG_C, 0);
            gpio_set_level(SEG_D, 0);
            gpio_set_level(SEG_E, 0);
            gpio_set_level(SEG_G, 0);
            break;
            
        case 'F': // Dígito F: A,E,F,G
            gpio_set_level(SEG_A, 0);
            gpio_set_level(SEG_E, 0);
            gpio_set_level(SEG_F, 0);
            gpio_set_level(SEG_G, 0);
            break;
            
        default: // Apagar todos (já feito acima)
            break;
    }
}

void display_task(void *pvParam) {
    char digit_to_show = ' ';
    uint64_t last_blink_time = 0;
    bool display_on = true;
    
    while(1) {
        int temp_atual, temp_alarme;
        bool alarme;
        
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(100))) {
            temp_atual = temperatura_atual;
            temp_alarme = temperatura_alarme;
            alarme = alarme_ativado;
            xSemaphoreGive(xMutex);
        } else {
            continue;
        }
        
        int diff = temp_alarme - temp_atual;
        
        // Determinar qual dígito mostrar
        if (alarme) {
            digit_to_show = 'F';
        } else if (diff <= 2) {
            digit_to_show = 'D';
        } else if (diff <= 10) {
            digit_to_show = '7';
        } else if (diff <= 15) {
            digit_to_show = '3';
        } else if (diff <= 20) {
            digit_to_show = '0';
        } else {
            digit_to_show = ' '; // Apagar display
        }
        
        // Piscar o display quando em alarme
        if (alarme) {
            uint64_t now = esp_timer_get_time();
            if (now - last_blink_time > 500000) { // 500ms
                last_blink_time = now;
                display_on = !display_on;
            }
        } else {
            display_on = true;
        }
        
        // Atualizar display
        if (display_on) {
            display_7seg(digit_to_show);
        } else {
            display_7seg(' '); // Apagar display
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualização rápida para piscar
    }
}

void sd_task(void *pvParam) {
    uint64_t last_log = 0;
    
    while(1) {
        uint64_t now = esp_timer_get_time() / 1000;
        
        if ((now - last_log) >= LOG_INTERVAL_MS) {
            if (sd_card_ok) {
                log_temperature();
                last_log = now;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Verificação a cada 1s
    }
}

void app_main() {
    ESP_LOGI(TAG, "Iniciando sistema");
    
    // Inicializações
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

    // Cria recursos do FreeRTOS
    xMutex = xSemaphoreCreateMutex();
    button_queue = xQueueCreate(10, sizeof(int));
    
    // Cria tasks conforme requisitos do professor
    xTaskCreate(temp_task,      "temp_reader",  2048, NULL, 5, NULL); // Leitura de temperatura
    xTaskCreate(button_task,    "buttons",      2048, NULL, 4, NULL); // Botões (Parte A)
    xTaskCreate(alarm_task,     "alarm",        2048, NULL, 3, NULL); // PWM (Parte B)
    xTaskCreate(lcd_task,       "lcd",          2048, NULL, 2, NULL); // LCD (Parte C)
    xTaskCreate(display_task,   "display",      2048, NULL, 2, NULL); // Display 7 seg (Parte D)
    xTaskCreate(sd_task,        "sd_card",      4096, NULL, 1, NULL); // SD Card (Parte E)

    // Mantém a task principal ativa
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
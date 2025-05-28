#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include "driver/ledc.h"
#include "int_i2c.h"

#define BUTTON_A_PIN       GPIO_NUM_38
#define BUTTON_B_PIN       GPIO_NUM_39
#define LED1_PIN           GPIO_NUM_4 
#define LED2_PIN           GPIO_NUM_5
#define LED3_PIN           GPIO_NUM_6
#define LED4_PIN           GPIO_NUM_7
#define PWM_LED_PIN        GPIO_NUM_14
#define I2C_MASTER_SCL_IO  GPIO_NUM_1
#define I2C_MASTER_SDA_IO  GPIO_NUM_2
#define I2C_MASTER_NUM     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_ADDRESS        0x27      
#define LCD_SIZE           DISPLAY_16X02

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY      5000

static const char *TAG = "BIN_COUNTER";
static uint8_t counter = 0;
lcd_i2c_handle_t lcd;

volatile bool button_a_interrupt_flag = false;
volatile bool button_b_interrupt_flag = false;

static void IRAM_ATTR button_a_isr_handler(void* arg) {
    gpio_intr_disable(BUTTON_A_PIN);
    button_a_interrupt_flag = true;
}

static void IRAM_ATTR button_b_isr_handler(void* arg) {
    gpio_intr_disable(BUTTON_B_PIN);
    button_b_interrupt_flag = true;
}

static void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static void setup_pwm() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM_LED_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void update_pwm_led(uint8_t count) {
    uint32_t duty = (count * 255) / 15;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void update_lcd_display(uint8_t count) {
    char buffer[20];
    
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
    vTaskDelay(10 / portTICK_PERIOD_MS); 

    lcd_i2c_cursor_set(&lcd, 0, 0);
    snprintf(buffer, sizeof(buffer), "0x%X", count);
    lcd_i2c_print(&lcd, buffer);
    
    lcd_i2c_cursor_set(&lcd, 0, 1);
    snprintf(buffer, sizeof(buffer), "%d", count);
    lcd_i2c_print(&lcd, buffer);
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

static void setup_gpio() {
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << BUTTON_A_PIN) | (1ULL << BUTTON_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_config);

    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED1_PIN) | (1ULL << LED2_PIN) | 
                        (1ULL << LED3_PIN) | (1ULL << LED4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_config);

    gpio_set_intr_type(BUTTON_A_PIN, GPIO_INTR_NEGEDGE); 
    gpio_set_intr_type(BUTTON_B_PIN, GPIO_INTR_NEGEDGE); 
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A_PIN, button_a_isr_handler, NULL);
    gpio_isr_handler_add(BUTTON_B_PIN, button_b_isr_handler, NULL);
}

void app_main() {
    ESP_LOGI(TAG, "Iniciando sistema");
    setup_gpio();
    i2c_master_init();  
    setup_pwm();        
    
    lcd.address = LCD_ADDRESS;
    lcd.num = I2C_MASTER_NUM;
    lcd.backlight = 1; 
    lcd.size = LCD_SIZE;
    lcd_i2c_init(&lcd);
    
    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, "Contador Binario");
    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, "Iniciando...");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
    
    ESP_LOGI(TAG, "Sistema iniciado! Contador: 0 (0x0)");
    update_lcd_display(counter);

    uint8_t last_a_state = 1;
    uint8_t last_b_state = 1; 
    TickType_t last_a_press_time = 0; 
    TickType_t last_b_press_time = 0; 
    const TickType_t debounce_interval = pdMS_TO_TICKS(50);

    while (1) {
        uint8_t a_state = gpio_get_level(BUTTON_A_PIN);
        uint8_t b_state = gpio_get_level(BUTTON_B_PIN);
        TickType_t now = xTaskGetTickCount();

        if (button_a_interrupt_flag) {
            button_a_interrupt_flag = false;
            if (a_state == 0 && last_a_state == 1) {  
                if ((now - last_a_press_time) >= debounce_interval) {
                    counter = (counter + 1) % 16;
                    
                    ESP_LOGI(TAG, "Incremento: %d (0x%X)", counter, counter);
                    printf("[Contador] Valor atual: %d (0x%X)\n", counter, counter);
                    
                    gpio_set_level(LED1_PIN, (counter >> 0) & 0x01);
                    gpio_set_level(LED2_PIN, (counter >> 1) & 0x01);
                    gpio_set_level(LED3_PIN, (counter >> 2) & 0x01);
                    gpio_set_level(LED4_PIN, (counter >> 3) & 0x01);
                    
                  
                    update_pwm_led(counter);
                    
                    
                    update_lcd_display(counter);
                    
                    last_a_press_time = now;
                }
            }
            gpio_intr_enable(BUTTON_A_PIN);
        }

        if(button_b_interrupt_flag) {
            button_b_interrupt_flag = false;
            if (b_state == 0 && last_b_state == 1) { 
                if ((now - last_b_press_time) >= debounce_interval) {
                    counter = (counter - 1 + 16) % 16; 

                    ESP_LOGI(TAG, "Decremento: %d (0x%X)", counter, counter);
                    printf("[Contador] Valor atual: %d (0x%X)\n", counter, counter);
                    
                    gpio_set_level(LED1_PIN, (counter >> 0) & 0x01);
                    gpio_set_level(LED2_PIN, (counter >> 1) & 0x01);
                    gpio_set_level(LED3_PIN, (counter >> 2) & 0x01);
                    gpio_set_level(LED4_PIN, (counter >> 3) & 0x01);
                    
                    update_pwm_led(counter);
                    
                    update_lcd_display(counter);
                    
                    last_b_press_time = now;
                }
            }
            gpio_intr_enable(BUTTON_B_PIN);
        }

        last_a_state = a_state;
        last_b_state = b_state;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
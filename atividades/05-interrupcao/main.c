#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>

#define BUTTON_A_PIN       GPIO_NUM_38
#define BUTTON_B_PIN       GPIO_NUM_39
#define LED1_PIN           GPIO_NUM_4 
#define LED2_PIN           GPIO_NUM_5
#define LED3_PIN           GPIO_NUM_6
#define LED4_PIN           GPIO_NUM_7 

static const char *TAG = "BIN_COUNTER";
static uint8_t counter = 0;

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

//capturar interrupção do botão
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
    setup_gpio();
    printf("Sistema iniciado! Contador: 0 (0x0)\n");

    uint8_t last_a_state = 1;
    uint8_t last_b_state = 1; 
    TickType_t last_a_press_time = 0; 
    TickType_t last_b_press_time = 0; 
    const TickType_t debounce_interval = pdMS_TO_TICKS(50);
    static uint8_t step = 1; 

    while (1) {
        uint8_t a_state = gpio_get_level(BUTTON_A_PIN);
        uint8_t b_state = gpio_get_level(BUTTON_B_PIN);
        TickType_t now = xTaskGetTickCount();

        if (button_a_interrupt_flag) {
          button_a_interrupt_flag = false;
          if (a_state == 0 && last_a_state == 1) {  
              if ((now - last_a_press_time) >= debounce_interval) {
                  counter = (counter + step) % 16;
                  
                  printf("[AÇÃO] Incremento: %d + %d = %d (0x%X)\n", 
                        (counter - step + 16) % 16,
                        step,
                        counter,
                        counter);
                  
                  gpio_set_level(LED1_PIN, (counter >> 0) & 0x01);
                  gpio_set_level(LED2_PIN, (counter >> 1) & 0x01);
                  gpio_set_level(LED3_PIN, (counter >> 2) & 0x01);
                  gpio_set_level(LED4_PIN, (counter >> 3) & 0x01);
                  last_a_press_time = now;
              }
          }
          gpio_intr_enable(BUTTON_A_PIN);
        }

        if(button_b_interrupt_flag) {
          button_b_interrupt_flag = false;
          if (b_state == 0 && last_b_state == 1) { 
              if ((now - last_b_press_time) >= debounce_interval) {
                  step = (step == 1) ? 2 : 1;
                  printf("[CONFIG] Passo alterado para: %d\n", step);
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
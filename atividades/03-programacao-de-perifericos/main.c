#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED1_GPIO 4  
#define LED2_GPIO 7

int LED1 = 0;
int LED2 = 0;
int ultima_vez_led = 0;

void leds() {
  LED2 = !LED2;
  gpio_set_level(LED2_GPIO, LED2);

  if(ultima_vez_led % 5 == 0) {
    LED1 = !LED1;
    gpio_set_level(LED1_GPIO, LED1);
  } 

  ultima_vez_led++;
}

void app_main() {
    gpio_set_direction(LED1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_level(LED1_GPIO, gpio_get_level(LED1_GPIO));
    gpio_set_level(LED2_GPIO, gpio_get_level(LED2_GPIO));

    while(1) {
      leds();
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

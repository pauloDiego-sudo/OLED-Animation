#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <u8g2.h>

#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"
#include "bitmap.h"


// CLK - GPIO14
#define PIN_CLK 14

// MOSI - GPIO 13
#define PIN_MOSI 13

// RESET - GPIO 26
#define PIN_RESET 26

// DC - GPIO 27
#define PIN_DC 27

// CS - GPIO 15
#define PIN_CS 15
static char tag[] = "test_SSD1306";

void app_main(void) {
          
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.bus.spi.clk = PIN_CLK;
  u8g2_esp32_hal.bus.spi.mosi = PIN_MOSI;
  u8g2_esp32_hal.bus.spi.cs = PIN_CS;
  u8g2_esp32_hal.dc = PIN_DC;
  u8g2_esp32_hal.reset = PIN_RESET;
  u8g2_esp32_hal_init(u8g2_esp32_hal);

  u8g2_t u8g2;  // a structure which will contain all the data for one display
  u8g2_Setup_ssd1306_128x64_noname_f(
      &u8g2, U8G2_R0, u8g2_esp32_spi_byte_cb,
      u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure

  u8g2_InitDisplay(&u8g2);  // send init sequence to the display, display is in
                            // sleep mode after this,

  u8g2_SetPowerSave(&u8g2,0);  // wake up display

  for(;;){
    for(int i=0;i<reddit_robot_bitmap_allArray_LEN;i++){
      u8g2_ClearBuffer(&u8g2);
      u8g2_DrawXBM(&u8g2, 40,15, 50, 50, reddit_robot_bitmap_allArray[i]);
      u8g2_SendBuffer(&u8g2);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    u8g2_ClearBuffer(&u8g2);
  }

  ESP_LOGD(tag, "All done!");
  vTaskDelete(NULL);
}
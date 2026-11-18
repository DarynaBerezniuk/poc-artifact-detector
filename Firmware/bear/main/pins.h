#pragma once
#include "driver/gpio.h"

#define UWB_PIN_CS      GPIO_NUM_5
#define UWB_PIN_MOSI    GPIO_NUM_23
#define UWB_PIN_MISO    GPIO_NUM_19
#define UWB_PIN_CLK     GPIO_NUM_18
#define UWB_PIN_IRQ     GPIO_NUM_4

#define UWB_PIN_RST     (-1)

#define I2C_PIN_SDA     GPIO_NUM_21
#define I2C_PIN_SCL     GPIO_NUM_22

#define LCD_PIN_SCL     GPIO_NUM_14
#define LCD_PIN_SDA     GPIO_NUM_13
#define LCD_PIN_CS      GPIO_NUM_26
#define LCD_PIN_DC      GPIO_NUM_27
#define LCD_PIN_RST     GPIO_NUM_16

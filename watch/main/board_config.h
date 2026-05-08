#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// ESP32-C3 hardware wiring — same physical board as shock_watch.
// SDA=GPIO8, SCL=GPIO9, shock/buzzer=GPIO10, battery ADC=GPIO4.

#include "driver/i2c.h"

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    8
#define I2C_SCL_GPIO    9
#define I2C_FREQ_HZ     100000

#define OLED_ADDR       0x3C
#define MAX30102_ADDR   0x57

#define SHOCK_GPIO        10
#define SHOCK_ACTIVE_HIGH 1

#define BATTERY_ADC_GPIO  4

#endif

#ifndef VEML6030_H
#define VEML6030_H

#include "esp_err.h"
#include <stdint.h>

#define VEML6030_I2C_ADDR 0x48

esp_err_t veml6030_init(void);
esp_err_t veml6030_read_lux(float *lux);
esp_err_t veml6030_read_raw(uint16_t *raw);
esp_err_t veml6030_deinit(void);

#endif
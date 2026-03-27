#ifndef SHT30_H
#define SHT30_H

#include "esp_err.h"

esp_err_t sht30_init(void);
esp_err_t sht30_read(float *temperature, float *humidity);

#endif
#include "veml6030.h"

#include "driver/i2c.h"
#include "freertos/task.h"

#include <math.h>

#define I2C_MASTER_NUM      I2C_NUM_0

#define VEML6030_REG_ALS_CONF   0x00
#define VEML6030_REG_ALS        0x04

#define VEML6030_GAIN_1_8   (2 << 11)
#define VEML6030_IT_25MS    (12 << 6)

static esp_err_t write_reg(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {
        reg,
        value & 0xFF,
        (value >> 8) & 0xFF
    };

    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        VEML6030_I2C_ADDR,
        data,
        3,
        1000 / portTICK_PERIOD_MS
    );
}

static esp_err_t read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        VEML6030_I2C_ADDR,
        &reg, 1,
        data, 2,
        1000 / portTICK_PERIOD_MS
    );

    if (ret == ESP_OK) {
        *value = data[0] | (data[1] << 8);
    }

    return ret;
}

esp_err_t veml6030_init(void)
{
    uint16_t config = VEML6030_GAIN_1_8 | VEML6030_IT_25MS;

    esp_err_t ret = write_reg(VEML6030_REG_ALS_CONF, config);
    if (ret != ESP_OK) return ret;

    vTaskDelay(200 / portTICK_PERIOD_MS);

    return ESP_OK;
}

esp_err_t veml6030_read_raw(uint16_t *raw)
{
    if (!raw) return ESP_ERR_INVALID_ARG;
    return read_reg(VEML6030_REG_ALS, raw);
}

esp_err_t veml6030_read_lux(float *lux)
{
    if (!lux) return ESP_ERR_INVALID_ARG;

    uint16_t raw;
    if (veml6030_read_raw(&raw) != ESP_OK) {
        *lux = -1;
        return ESP_FAIL;
    }

    float val = raw * 1.8432f;

    if (val > 100.0f) {
        val = (6.0135e-13f * powf(val, 4)) -
              (9.3924e-9f  * powf(val, 3)) +
              (8.1488e-5f  * powf(val, 2)) +
              (1.0023f     * val);
    }

    *lux = val;
    return ESP_OK;
}

esp_err_t veml6030_deinit(void)
{
    return ESP_OK;
}
#include "sht30.h"
#include "driver/i2c.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM      I2C_NUM_0
#define SHT30_ADDR          0x44

#define CMD_MEASURE         0x2400

static esp_err_t write_cmd(uint16_t cmd)
{
    uint8_t data[2] = { cmd >> 8, cmd & 0xFF };

    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        SHT30_ADDR,
        data,
        2,
        1000 / portTICK_PERIOD_MS
    );
}

static esp_err_t read_data(uint8_t *data, size_t len)
{
    return i2c_master_read_from_device(
        I2C_MASTER_NUM,
        SHT30_ADDR,
        data,
        len,
        1000 / portTICK_PERIOD_MS
    );
}

esp_err_t sht30_init(void)
{
    return ESP_OK;
}

esp_err_t sht30_read(float *temperature, float *humidity)
{
    if (!temperature || !humidity) return ESP_ERR_INVALID_ARG;

    uint8_t data[6];

    if (write_cmd(CMD_MEASURE) != ESP_OK)
        return ESP_FAIL;

    vTaskDelay(20 / portTICK_PERIOD_MS);

    if (read_data(data, 6) != ESP_OK)
        return ESP_FAIL;

    uint16_t rawT = (data[0] << 8) | data[1];
    uint16_t rawH = (data[3] << 8) | data[4];

    *temperature = -45 + 175 * ((float)rawT / 65535.0f);
    *humidity    = 100 * ((float)rawH / 65535.0f);

    return ESP_OK;
}
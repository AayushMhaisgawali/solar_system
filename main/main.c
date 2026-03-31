#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "sht30.h"
#include "veml6030.h"
#include "ntc.h"

static const char *TAG = "MAIN";

/* ================= I2C CONFIG ================= */

#define I2C_MASTER_SDA_IO       8
#define I2C_MASTER_SCL_IO       9
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/* ================= LINEAR REGRESSION MODEL ================= */
/*
 * Model trained with sklearn LinearRegression, accuracy ~97.5%
 * Formula:  deltaT = MODEL_COEF * (lux / 120.0) + MODEL_INTERCEPT
 * Output:   deltaT = temperature rise above ambient (C)
 */

#define MODEL_COEF       0.02984201f
#define MODEL_INTERCEPT  0.08106460f
#define NUM_NTC          6

static float ml_predict(float lux, float ntc_temp, float ambient_temp)
{
    float irradiance = lux / 120.0f;
    return fabsf(ntc_temp - ambient_temp) + (MODEL_COEF * irradiance) + MODEL_INTERCEPT;
}

/* ================= MAIN ================= */

void app_main(void)
{
    ESP_LOGI(TAG, "System Starting...");

    i2c_master_init();

    if (sht30_init() != ESP_OK) {
        ESP_LOGE(TAG, "SHT30 init failed");
        return;
    }

    if (veml6030_init() != ESP_OK) {
        ESP_LOGE(TAG, "VEML6030 init failed");
        return;
    }

    ntc_init();

    ESP_LOGI(TAG, "All sensors initialized successfully");
    ESP_LOGI(TAG, "ML model ready (coef=%.8f, intercept=%.8f)",
             (double)MODEL_COEF, (double)MODEL_INTERCEPT);

    while (1)
    {
        float temperature = 0.0f;
        float humidity    = 0.0f;
        float lux         = 0.0f;
        uint16_t raw      = 0;

        printf("\n================ SENSOR DATA ================\n");

        /* ---- SHT30 ---- */
        if (sht30_read(&temperature, &humidity) == ESP_OK) {
            printf("SHT30  -> Temp: %.2f C | Humidity: %.2f %% | ",
                   temperature, humidity);
        } else {
            printf("SHT30  -> Read Failed | ");
        }

        /* ---- VEML6030 ---- */
        if (veml6030_read_lux(&lux) == ESP_OK) {
            veml6030_read_raw(&raw);
            printf("Lux: %.2f (Raw: %u)\n", lux, raw);
        } else {
            printf("Lux Read Failed\n");
        }

        printf("-------------------------------------------\n");

        /* ---- NTC (MUX) + per-NTC ML prediction ---- */
        for (int i = 0; i < NUM_NTC; i++)
        {
            uint32_t mv    = 0;
            float temp_ntc = ntc_read_temperature(i, &mv);

            if (isnan(temp_ntc)) {
                printf("NTC%d -> %lu mV | FAULT\n", i + 1, mv);
            } else {
                float deltaT         = ml_predict(lux, temp_ntc, temperature);
                float predicted_temp = temperature + deltaT;
                printf("NTC%d -> %lu mV | %.2f C | DeltaT: %.2f C | Pred: %.2f C\n",
                       i + 1, mv, temp_ntc, deltaT, predicted_temp);
            }
        }

        printf("===========================================\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
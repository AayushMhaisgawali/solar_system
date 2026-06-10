#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "nrf24_link.h" 
#include "ntc.h"
#include "sht30.h"
#include "veml6030.h"

static const char *TAG = "SOLAR_APP";

#define NRF_PACKET_TYPE_SOLAR_PANEL 0x51
#define SOLAR_I2C_PORT I2C_NUM_0
#define SOLAR_I2C_SDA_GPIO 8
#define SOLAR_I2C_SCL_GPIO 9
#define SOLAR_ARRAY_STC_POWER_W 6600.0f
#define SOLAR_PANEL_TEMP_RISE_PER_IRRADIANCE 0.025f

static esp_err_t solar_i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SOLAR_I2C_SDA_GPIO,
        .scl_io_num = SOLAR_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    esp_err_t ret = i2c_param_config(SOLAR_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_driver_install(SOLAR_I2C_PORT, conf.mode, 0, 0, 0);
    return (ret == ESP_ERR_INVALID_STATE) ? ESP_OK : ret;
}

static float clamp_float(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

static float calculate_dust_index(float measured_temp, float predicted_temp) {
    if (predicted_temp <= 0.0f || isnan(measured_temp) || isnan(predicted_temp)) {
        return 0.0f;
    }

    float dust = ((predicted_temp - measured_temp) / predicted_temp) * 100.0f;
    return clamp_float(dust, 0.0f, 100.0f);
}

void app_main(void) {
    esp_err_t i2c_ret = solar_i2c_init();
    ESP_LOGI(TAG, "I2C init: %s", esp_err_to_name(i2c_ret));

    esp_err_t sht30_ret = sht30_init();
    ESP_LOGI(TAG, "SHT30 init: %s", esp_err_to_name(sht30_ret));

    esp_err_t lux_ret = veml6030_init();
    ESP_LOGI(TAG, "VEML6030 init: %s", esp_err_to_name(lux_ret));

    ntc_init();
    ESP_LOGI(TAG, "NTC mux/ADC initialized");

    if (!nrf24_init_tx()) {
        ESP_LOGE(TAG, "Radio hardware subsystem failed to init!");
        return;
    }
    
    uint16_t seq = 0;

    while (1) {
        float ambient_t = 25.0f;
        float humidity = 0.0f;
        if (sht30_read(&ambient_t, &humidity) != ESP_OK) {
            ESP_LOGW(TAG, "SHT30 read failed, using fallback ambient");
        }

        float lux = 0.0f;
        if (veml6030_read_lux(&lux) != ESP_OK || lux < 0.0f) {
            ESP_LOGW(TAG, "VEML6030 read failed, using 0 lux");
            lux = 0.0f;
        }

        float irradiance_wm2 = clamp_float(lux / 120.0f, 0.0f, 1200.0f);
        float expected_rise = irradiance_wm2 * SOLAR_PANEL_TEMP_RISE_PER_IRRADIANCE;

        float measured_temps[NUM_NTC_SENSORS] = {0};
        float predicted_temps[NUM_NTC_SENSORS] = {0};
        float dust_indexes[NUM_NTC_SENSORS] = {0};
        uint32_t ntc_millivolts[NUM_NTC_SENSORS] = {0};
        uint8_t panel_flags[NUM_NTC_SENSORS] = {0};
        float avg_panel_temp = 0.0f;
        float avg_dust_index = 0.0f;
        int valid_panel_count = 0;

        for (uint8_t panel = 0; panel < NUM_NTC_SENSORS; panel++) {
            uint32_t ntc_mv = 0;
            float measured_t = ntc_read_temperature(panel, &ntc_mv);
            uint8_t flags = 0x00;

            if (isnan(measured_t)) {
                measured_t = ambient_t;
                flags |= 0x01;
            }

            float predicted_t = ambient_t + expected_rise;
            float dust = calculate_dust_index(measured_t, predicted_t);

            measured_temps[panel] = measured_t;
            predicted_temps[panel] = predicted_t;
            dust_indexes[panel] = dust;
            ntc_millivolts[panel] = ntc_mv;
            panel_flags[panel] = flags;

            if ((flags & 0x01) == 0) {
                avg_panel_temp += measured_t;
                avg_dust_index += dust;
                valid_panel_count++;
            }
        }

        if (valid_panel_count > 0) {
            avg_panel_temp /= valid_panel_count;
            avg_dust_index /= valid_panel_count;
        } else {
            avg_panel_temp = ambient_t;
            avg_dust_index = 0.0f;
        }

        float irradiance_factor = clamp_float(irradiance_wm2 / 1000.0f, 0.0f, 1.2f);
        float temp_factor = 1.0f - (0.0035f * (avg_panel_temp - 25.0f));
        float dust_factor = 1.0f - clamp_float(avg_dust_index / 100.0f, 0.0f, 1.0f);
        float solar_p_out = SOLAR_ARRAY_STC_POWER_W * irradiance_factor * temp_factor * dust_factor;
        solar_p_out = clamp_float(solar_p_out, 0.0f, SOLAR_ARRAY_STC_POWER_W * 1.2f);
        float panel_power_share = clamp_float(solar_p_out / NUM_NTC_SENSORS, 0.0f, 655.35f);

        ESP_LOGI(TAG, "Solar env Lux=%.2f lx Irr=%.2f W/m2 AvgT=%.2fC AvgDI=%.2f%% P_out=%.2fW",
                 lux, irradiance_wm2, avg_panel_temp, avg_dust_index, solar_p_out);

        for (uint8_t panel = 0; panel < NUM_NTC_SENSORS; panel++) {
            float measured_t = measured_temps[panel];
            float predicted_t = predicted_temps[panel];
            float dust = dust_indexes[panel];
            uint32_t ntc_mv = ntc_millivolts[panel];
            uint8_t flags = panel_flags[panel];

            nrf24_solar_panel_packet_t packet = {
                .type = NRF_PACKET_TYPE_SOLAR_PANEL,
                .panel_index = panel + 1,
                .sequence = ++seq,
                .flags = flags,
                .measured_temp_x100 = (int16_t)(measured_t * 100),
                .predicted_temp_x100 = (int16_t)(predicted_t * 100),
                .delta_temp_x100 = (int16_t)((predicted_t - measured_t) * 100),
                .dust_index_x100 = (uint16_t)(dust * 100),
                .panel_power_x100 = (uint16_t)(panel_power_share * 100),
                .sht30_temp_x100 = (int16_t)(ambient_t * 100),
                .sht30_humidity_x100 = (uint16_t)(clamp_float(humidity, 0.0f, 100.0f) * 100),
                .irradiance_x10 = (uint16_t)(irradiance_wm2 * 10),
            };

            if (nrf24_send_solar_panel_packet(&packet)) {
                ESP_LOGI(TAG, "TX panel=%u seq=%u T=%.2fC Pred=%.2fC DI=%.2f%% Lux=%.2f lx ADC=%lumV Pshare=%.2fW",
                         panel + 1, seq, measured_t, predicted_t, dust, lux,
                         (unsigned long)ntc_mv, panel_power_share);
            } else {
                ESP_LOGW(TAG, "TX Drop panel=%u seq=%u", panel + 1, seq);
            }

            vTaskDelay(pdMS_TO_TICKS(70));
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

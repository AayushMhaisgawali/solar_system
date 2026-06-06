#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nrf24_link.h"
#include "ntc.h"
#include "sht30.h"
#include "veml6030.h"

static const char *TAG = "SOLAR_TX";

#define I2C_MASTER_SDA_IO  8
#define I2C_MASTER_SCL_IO  9
#define I2C_MASTER_NUM     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define MODEL_COEF      0.02984201f
#define MODEL_INTERCEPT 0.08106460f

#define NUM_NTC NRF24_SOLAR_PANEL_COUNT

#define PANEL_RATED_POWER 6600.0f
#define TEMP_COEFFICIENT  0.00354f
#define STC_TEMPERATURE   25.0f

static uint16_t s_packet_sequence;

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int16_t scale_i16_x100(float value)
{
    return (int16_t)lrintf(value * 100.0f);
}

static uint16_t scale_u16_x100(float value)
{
    value = clamp_float(value, 0.0f, 655.35f);
    return (uint16_t)lrintf(value * 100.0f);
}

static uint16_t scale_u16_x10(float value)
{
    value = clamp_float(value, 0.0f, 6553.5f);
    return (uint16_t)lrintf(value * 10.0f);
}

static int32_t scale_i32_x100(float value)
{
    return (int32_t)lrintf(value * 100.0f);
}

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static float ml_predict(float lux, float ntc_temp, float ambient_temp)
{
    float irradiance = lux / 120.0f;
    return fabsf(ntc_temp - ambient_temp) + (MODEL_COEF * irradiance) + MODEL_INTERCEPT;
}

static float calculate_dust_index(float measured_temp, float predicted_temp)
{
    float delta = predicted_temp - measured_temp;
    if (delta < 0.0f) {
        delta = 0.0f;
    }
    return clamp_float(delta * 10.0f, 0.0f, 100.0f);
}

static float calculate_solar_power(float irradiance, float panel_temp, float dust_index)
{
    float irradiance_factor = clamp_float(irradiance / 1000.0f, 0.0f, 2.0f);
    float temp_factor = 1.0f - (TEMP_COEFFICIENT * (panel_temp - STC_TEMPERATURE));
    float dust_factor = 1.0f - clamp_float(dust_index / 100.0f, 0.0f, 1.0f);

    temp_factor = clamp_float(temp_factor, 0.0f, 1.5f);
    return PANEL_RATED_POWER * irradiance_factor * temp_factor * dust_factor;
}

static bool send_panel_packet(uint8_t panel_index,
                              uint8_t flags,
                              float measured_temp,
                              float predicted_temp,
                              float delta_t,
                              float dust_index,
                              float panel_power,
                              float ambient_temp,
                              float humidity,
                              float irradiance)
{
    nrf24_solar_panel_packet_t packet;
    memset(&packet, 0, sizeof(packet));

    packet.type = NRF24_PACKET_TYPE_SOLAR_PANEL;
    packet.sequence = ++s_packet_sequence;
    packet.panel_index = panel_index;
    packet.flags = flags;
    packet.measured_temp_x100 = scale_i16_x100(measured_temp);
    packet.predicted_temp_x100 = scale_i16_x100(predicted_temp);
    packet.delta_temp_x100 = scale_i16_x100(delta_t);
    packet.dust_index_x100 = scale_i16_x100(dust_index);
    packet.panel_power_x100 = scale_i32_x100(panel_power);
    packet.sht30_temp_x100 = scale_i16_x100(ambient_temp);
    packet.sht30_humidity_x100 = scale_u16_x100(humidity);
    packet.irradiance_x10 = scale_u16_x10(irradiance);

    bool ok = nrf24_send_solar_panel_packet(&packet);
    printf("TX Solar PV%u seq=%u %s\n", panel_index, packet.sequence, ok ? "OK" : "FAIL");
    vTaskDelay(pdMS_TO_TICKS(30));
    return ok;
}

void app_main(void)
{
    ESP_LOGI(TAG, "System Starting");

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

    if (!nrf24_init_tx()) {
        ESP_LOGE(TAG, "NRF24 init failed");
        return;
    }

    ESP_LOGI(TAG, "All modules initialized");

    while (1) {
        float ambient_temp = 0.0f;
        float humidity = 0.0f;
        float lux = 0.0f;
        float irradiance = 0.0f;
        float total_power = 0.0f;
        float total_dust = 0.0f;
        int valid_panels = 0;
        uint16_t raw = 0;

        printf("\n========================================\n");
        printf("         SOLAR PV TRANSMITTER\n");
        printf("========================================\n");

        if (sht30_read(&ambient_temp, &humidity) == ESP_OK) {
            printf("Ambient Temp : %.2f C\n", ambient_temp);
            printf("Humidity     : %.2f %%\n", humidity);
        } else {
            printf("SHT30 READ FAILED\n");
        }

        if (veml6030_read_lux(&lux) == ESP_OK) {
            veml6030_read_raw(&raw);
            irradiance = lux / 120.0f;
            printf("Lux          : %.2f\n", lux);
            printf("Irradiance   : %.2f W/m2\n", irradiance);
        } else {
            printf("VEML6030 READ FAILED\n");
        }

        printf("----------------------------------------\n");

        for (int i = 0; i < NUM_NTC; i++) {
            uint32_t mv = 0;
            float measured_temp = ntc_read_temperature(i, &mv);

            if (isnan(measured_temp)) {
                printf("PV%d : FAULT\n", i + 1);
                send_panel_packet((uint8_t)(i + 1),
                                  NRF24_SOLAR_FAULT_FLAG,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  ambient_temp,
                                  humidity,
                                  irradiance);
                continue;
            }

            float delta_t = ml_predict(lux, measured_temp, ambient_temp);
            float predicted_temp = ambient_temp + delta_t;
            float dust_index = calculate_dust_index(measured_temp, predicted_temp);
            float panel_power = calculate_solar_power(irradiance, measured_temp, dust_index);

            total_power += panel_power;
            total_dust += dust_index;
            valid_panels++;

            printf("PV%d : %.2f C | Pred %.2f C | dT %.2f C | Dust %.2f %% | P %.2f W\n",
                   i + 1,
                   measured_temp,
                   predicted_temp,
                   delta_t,
                   dust_index,
                   panel_power);

            send_panel_packet((uint8_t)(i + 1),
                              0x00,
                              measured_temp,
                              predicted_temp,
                              delta_t,
                              dust_index,
                              panel_power,
                              ambient_temp,
                              humidity,
                              irradiance);
        }

        float avg_dust = (valid_panels > 0) ? (total_dust / valid_panels) : 0.0f;

        printf("----------------------------------------\n");
        printf("Avg Dust Index        : %.2f %%\n", avg_dust);
        printf("P_out Solar           : %.2f W\n", total_power);
        printf("Valid Solar Probes    : %d / %d\n", valid_panels, NUM_NTC);
        printf("========================================\n");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

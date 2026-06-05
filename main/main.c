#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "sht30.h"
#include "veml6030.h"
#include "ntc.h"
#include "nrf24_link.h"


static const char *TAG = "SOLAR_TX";

/* =========================================================
 * I2C CONFIG
 * ========================================================= */

#define I2C_MASTER_SDA_IO       8
#define I2C_MASTER_SCL_IO       9
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000

/* =========================================================
 * ML MODEL
 * ========================================================= */

#define MODEL_COEF       0.02984201f
#define MODEL_INTERCEPT  0.08106460f

#define NUM_NTC          12

/* =========================================================
 * I2C INIT
 * ========================================================= */

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

    ESP_ERROR_CHECK(
        i2c_param_config(
            I2C_MASTER_NUM,
            &conf
        )
    );

    ESP_ERROR_CHECK(
        i2c_driver_install(
            I2C_MASTER_NUM,
            conf.mode,
            0,
            0,
            0
        )
    );
}

/* =========================================================
 * ML PREDICTION
 * ========================================================= */

static float ml_predict(float lux,
                        float ntc_temp,
                        float ambient_temp)
{
    float irradiance = lux / 120.0f;

    return fabsf(ntc_temp - ambient_temp)
           + (MODEL_COEF * irradiance)
           + MODEL_INTERCEPT;
}

/* =========================================================
 * SEND TEXT PACKET
 * ========================================================= */

static bool send_text_packet(uint8_t type,
                             uint8_t sequence,
                             uint8_t total,
                             const char *text)
{
    nrf24_text_packet_t pkt;

    memset(&pkt, 0, sizeof(pkt));

    pkt.type = type;
    pkt.sequence = sequence;
    pkt.total_packets = total;

    strncpy(pkt.text,
            text,
            NRF24_LABELED_TEXT_SIZE - 1);

    bool ok = nrf24_send_payload(
        (uint8_t *)&pkt,
        sizeof(pkt)
    );

    vTaskDelay(pdMS_TO_TICKS(50));

    return ok;
}

/* =========================================================
 * SEND ENVIRONMENT DATA
 * ========================================================= */

static void send_environment_data(float temp,
                                  float hum)
{
    char text[32];

    snprintf(text,
             sizeof(text),
             "AMB %.1fC HUM %.1f%%",
             temp,
             hum);

    send_text_packet(
        NRF24_PACKET_TYPE_SOLAR_ENV,
        1,
        1,
        text
    );
}

/* =========================================================
 * SEND LIGHT DATA
 * ========================================================= */

static void send_light_data(float lux,
                            float irradiance)
{
    char text[32];

    snprintf(text,
             sizeof(text),
             "LUX:%.0f IRR:%.1f",
             lux,
             irradiance);

    send_text_packet(
        NRF24_PACKET_TYPE_SOLAR_PV,
        1,
        15,
        text
    );
}

/* =========================================================
 * SEND NTC DATA
 * ========================================================= */

static void send_ntc_data(uint8_t index,
                          float ntc_temp,
                          float predicted_temp,
                          float delta_t,
                          float dust)
{
    char text[32];

    snprintf(text,
             sizeof(text),
             "PV%d %.1f %.1f %.1f %.1f",
             index,
             ntc_temp,
             predicted_temp,
             delta_t,
             dust);

    send_text_packet(
        NRF24_PACKET_TYPE_SOLAR_PV,
        index + 1,
        15,
        text
    );
}

/* =========================================================
 * SOLAR POWER MODEL
 * ========================================================= */

#define PANEL_RATED_POWER      6600.0f
#define TEMP_COEFFICIENT       0.00354f
#define STC_TEMPERATURE        25.0f

/* =========================================================
 * DUST INDEX
 * ========================================================= */

static void send_avg_dust(float avg_dust)
{
    char text[32];

    snprintf(text,
             sizeof(text),
             "AVG_DUST:%.1f",
             avg_dust);

    send_text_packet(
        NRF24_PACKET_TYPE_SOLAR_PV,
        14,
        15,
        text
    );
}

/* =========================================================
 * SOLAR POWER
 * ========================================================= */

static void send_total_power(float power)
{
    char text[32];

    snprintf(text,
             sizeof(text),
             "POWER:%.1fW",
             power);

    send_text_packet(
        NRF24_PACKET_TYPE_SOLAR_PV,
        15,
        15,
        text
    );
}

/* =========================================================
 * MAIN
 * ========================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "System Starting");

    /* ---------------- I2C ---------------- */

    i2c_master_init();

    /* ---------------- SHT30 ---------------- */

    if (sht30_init() != ESP_OK) {

        ESP_LOGE(TAG,
                 "SHT30 init failed");

        return;
    }

    /* ---------------- VEML6030 ---------------- */

    if (veml6030_init() != ESP_OK) {

        ESP_LOGE(TAG,
                 "VEML6030 init failed");

        return;
    }

    /* ---------------- NTC ---------------- */

    ntc_init();

    /* ---------------- NRF24 ---------------- */

    if (!nrf24_init_tx()) {

        ESP_LOGE(TAG,
                 "NRF24 init failed");

        return;
    }

    ESP_LOGI(TAG,
             "All modules initialized");

    /* =====================================================
     * LOOP
     * ===================================================== */

    while (1)
    {
        float ambient_temp = 0.0f;

        float humidity = 0.0f;

        float lux = 0.0f;

        float irradiance = 0.0f;

        uint16_t raw = 0;

        printf("\n");

        printf("========================================\n");
        printf("         SOLAR PV TRANSMITTER\n");
        printf("========================================\n");

        /* =====================================
         * SHT30
         * ===================================== */

        if (sht30_read(&ambient_temp,
                       &humidity) == ESP_OK)
        {
            printf("Ambient Temp : %.2f C\n",
                   ambient_temp);

            printf("Humidity     : %.2f %%\n",
                   humidity);

            send_environment_data(
                ambient_temp,
                humidity
            );
        }
        else
        {
            printf("SHT30 READ FAILED\n");
        }

        /* =====================================
         * VEML6030
         * ===================================== */

        if (veml6030_read_lux(&lux) == ESP_OK)
        {
            veml6030_read_raw(&raw);

            irradiance = lux / 120.0f;

            printf("Lux          : %.2f\n",
                   lux);

            printf("Irradiance   : %.2f\n",
                   irradiance);

            send_light_data(
                lux,
                irradiance
            );
        }
        else
        {
            printf("VEML6030 READ FAILED\n");
        }

        printf("----------------------------------------\n");

/* =====================================
 * NTC LOOP
 * ===================================== */

float total_power = 0.0f;
float total_dust  = 0.0f;

int valid_panels = 0;

float pv_temp[NUM_NTC];
float pv_pred[NUM_NTC];
float pv_delta[NUM_NTC];
float pv_dust[NUM_NTC];

printf("----------------------------------------\n");

for (int i = 0; i < NUM_NTC; i++)
{
    uint32_t mv = 0;

    float measured_temp =
        ntc_read_temperature(i, &mv);

    if (isnan(measured_temp))
    {
        pv_temp[i]  = NAN;
        pv_pred[i]  = NAN;
        pv_delta[i] = NAN;
        pv_dust[i]  = NAN;
        continue;
    }

    float deltaT =
        ml_predict(
            lux,
            measured_temp,
            ambient_temp
        );

    float predicted_temp =
        ambient_temp + deltaT;

    float dust_index =
        calculate_dust_index(
            measured_temp,
            predicted_temp
        );

    float panel_power =
        calculate_solar_power(
            irradiance,
            measured_temp,
            dust_index
        );

    pv_temp[i]  = measured_temp;
    pv_pred[i]  = predicted_temp;
    pv_delta[i] = deltaT;
    pv_dust[i]  = dust_index;

    total_power += panel_power;
    total_dust  += dust_index;

    valid_panels++;

    send_ntc_data(
    (uint8_t)(i + 1),
    measured_temp,
    predicted_temp,
    deltaT,
    dust_index
);
}
/* =====================================
 * OVERALL VALUES
 * ===================================== */

float avg_dust = 0.0f;
send_avg_dust(avg_dust);

send_total_power(total_power);
if (valid_panels > 0)
{
    avg_dust = total_dust / valid_panels;
}

printf("\n");

printf("SHT30 : Temperature = %.2f C, Humidity = %.2f %%\n",
       ambient_temp,
       humidity);

printf("Lux : %.2f , Irradiance : %.2f\n",
       lux,
       irradiance);

printf("Avg Dust Index : %.2f %%\n",
       avg_dust);

printf("Estimated Solar Power : %.2f W\n",
       total_power);

printf("\n");

for (int i = 0; i < NUM_NTC; i++)
{
    if (isnan(pv_temp[i]))
    {
        printf("PV%d = FAULT\n",
               i + 1);
    }
    else
    {
        printf(
            "PV%d = %.2f | Predicted = %.2f | Delta_T = %.2f | Dust = %.2f %%\n",
            i + 1,
            pv_temp[i],
            pv_pred[i],
            pv_delta[i],
            pv_dust[i]
        );
    }
}

printf("========================================\n");

vTaskDelay(pdMS_TO_TICKS(2000));

    }   /* while(1) */

}       /* app_main() */
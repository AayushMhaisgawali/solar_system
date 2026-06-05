#include "ntc.h"

#include <math.h>
#include <stdbool.h>

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_rom_sys.h"

/* =========================================================
 * MUX GPIO PINS
 * ========================================================= */

#define MUX_S0 GPIO_NUM_10
#define MUX_S1 GPIO_NUM_11
#define MUX_S2 GPIO_NUM_12
#define MUX_S3 GPIO_NUM_13

/* =========================================================
 * ADC CONFIG
 * ========================================================= */

#define ADC_UNIT_ID    ADC_UNIT_1
#define ADC_CHANNEL    ADC_CHANNEL_0
#define ADC_ATTEN      ADC_ATTEN_DB_12

/* =========================================================
 * HARDWARE CONFIG
 * ========================================================= */

#define VCC            3.3f
#define R_PULLUP       10000.0f
#define MUX_RON        70.0f

/* =========================================================
 * VALID NTC RANGE
 * ========================================================= */

#define NTC_MIN_RESISTANCE   1000.0f
#define NTC_MAX_RESISTANCE   100000.0f

/* =========================================================
 * ADC HANDLES
 * ========================================================= */

static adc_oneshot_unit_handle_t adc_handle;

static adc_cali_handle_t cali_handle = NULL;

static bool cali_enable = false;

/* =========================================================
 * INTERNAL FUNCTIONS
 * ========================================================= */

static void mux_select(uint8_t ch)
{
    gpio_set_level(MUX_S0, (ch >> 0) & 1);
    gpio_set_level(MUX_S1, (ch >> 1) & 1);
    gpio_set_level(MUX_S2, (ch >> 2) & 1);
    gpio_set_level(MUX_S3, (ch >> 3) & 1);

    /* Settling delay for 0.1uF capacitor */
    vTaskDelay(pdMS_TO_TICKS(30));
}

static uint32_t read_adc_mv(void)
{
    int raw = 0;

    /* Flush capacitor */
    for (int i = 0; i < 20; i++) {

        adc_oneshot_read(
            adc_handle,
            ADC_CHANNEL,
            &raw
        );

        esp_rom_delay_us(300);
    }

    /* Oversampling */
    int sum = 0;

    for (int i = 0; i < 64; i++) {

        adc_oneshot_read(
            adc_handle,
            ADC_CHANNEL,
            &raw
        );

        sum += raw;

        esp_rom_delay_us(200);
    }

    int avg = sum / 64;

    int mv = 0;

    if (cali_enable) {

        adc_cali_raw_to_voltage(
            cali_handle,
            avg,
            &mv
        );

    } else {

        mv = (int)((float)avg / 4095.0f * 3300.0f);
    }

    return (uint32_t)mv;
}

static float calc_resistance(float voltage_mv)
{
    if (voltage_mv < 10.0f ||
        voltage_mv > 3290.0f)
    {
        return INFINITY;
    }

    float resistance =
        (R_PULLUP * voltage_mv /
        (3300.0f - voltage_mv)) - MUX_RON;

    return resistance;
}

static float calc_temp(float resistance)
{
    if (resistance <= 0.0f ||
        isinf(resistance))
    {
        return NAN;
    }

    float log_r = logf(resistance);

    /* Steinhart-Hart Equation */

    float invT =
        0.001129148f +
        0.000234125f * log_r +
        0.0000000876741f *
        log_r * log_r * log_r;

    return (1.0f / invT) - 273.15f;
}

/* =========================================================
 * PUBLIC FUNCTIONS
 * ========================================================= */

void ntc_init(void)
{
    /* ---------------- MUX GPIO ---------------- */

    gpio_config_t io = {

        .pin_bit_mask =
            (1ULL << MUX_S0) |
            (1ULL << MUX_S1) |
            (1ULL << MUX_S2) |
            (1ULL << MUX_S3),

        .mode = GPIO_MODE_OUTPUT
    };

    gpio_config(&io);

    /* ---------------- ADC INIT ---------------- */

    adc_oneshot_unit_init_cfg_t unit = {
        .unit_id = ADC_UNIT_ID
    };

    adc_oneshot_new_unit(
        &unit,
        &adc_handle
    );

    adc_oneshot_chan_cfg_t chan = {

        .atten = ADC_ATTEN,

        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    adc_oneshot_config_channel(
        adc_handle,
        ADC_CHANNEL,
        &chan
    );

    /* ---------------- ADC CALIBRATION ---------------- */

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED

    adc_cali_curve_fitting_config_t cfg = {

        .unit_id = ADC_UNIT_ID,

        .chan = ADC_CHANNEL,

        .atten = ADC_ATTEN
    };

    if (adc_cali_create_scheme_curve_fitting(
            &cfg,
            &cali_handle) == ESP_OK)
    {
        cali_enable = true;
    }

#endif
}

float ntc_read_temperature(uint8_t mux_channel,
                           uint32_t *mv_out)
{
    mux_select(mux_channel);

    uint32_t mv = read_adc_mv();

    if (mv_out != NULL)
        *mv_out = mv;

    float resistance =
        calc_resistance((float)mv);

    if (isinf(resistance) ||
        resistance < NTC_MIN_RESISTANCE ||
        resistance > NTC_MAX_RESISTANCE)
    {
        return NAN;
    }

    return calc_temp(resistance);
}
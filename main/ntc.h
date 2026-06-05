#pragma once

#include <stdint.h>

/* ================= CONFIG ================= */

#define NUM_NTC_SENSORS 12

/* ================= API ================= */

/**
 * @brief Initialize MUX GPIO and ADC
 */
void ntc_init(void);

/**
 * @brief Read temperature from selected MUX channel
 *
 * @param mux_channel  MUX channel number (0-11)
 * @param mv_out       Optional pointer to store ADC voltage in mV
 *
 * @return Temperature in degree Celsius
 *         Returns NAN if fault detected
 */
float ntc_read_temperature(uint8_t mux_channel,
                           uint32_t *mv_out);
#pragma once
#include <stdint.h>

void ntc_init(void);
float ntc_read_temperature(uint8_t mux_channel, uint32_t *mv_out);
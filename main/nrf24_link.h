#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t panel_index;
    uint16_t sequence;
    uint8_t flags;
    int16_t measured_temp_x100;
    int16_t predicted_temp_x100;
    int16_t delta_temp_x100;
    uint16_t dust_index_x100;
    uint16_t panel_power_x100;
    int16_t sht30_temp_x100;
    uint16_t sht30_humidity_x100;
    uint16_t irradiance_x10;
} nrf24_solar_panel_packet_t;

_Static_assert(sizeof(nrf24_solar_panel_packet_t) <= 32, "solar packet must fit NRF24 payload");

bool nrf24_init_tx(void);
bool nrf24_send_solar_panel_packet(const nrf24_solar_panel_packet_t *packet);

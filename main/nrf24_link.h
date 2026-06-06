#ifndef NRF24_LINK_H_
#define NRF24_LINK_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF24_PACKET_TYPE_SOLAR_PANEL 0x01
#define NRF24_PAYLOAD_SIZE 32
#define NRF24_SOLAR_PANEL_COUNT 12
#define NRF24_SOLAR_FAULT_FLAG 0x80

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint16_t sequence;
    uint8_t panel_index;
    uint8_t flags;
    int16_t measured_temp_x100;
    int16_t predicted_temp_x100;
    int16_t delta_temp_x100;
    int16_t dust_index_x100;
    int32_t panel_power_x100;
    int16_t sht30_temp_x100;
    uint16_t sht30_humidity_x100;
    uint16_t irradiance_x10;
    uint8_t reserved[9];
} nrf24_solar_panel_packet_t;

_Static_assert(sizeof(nrf24_solar_panel_packet_t) == NRF24_PAYLOAD_SIZE,
               "NRF24 solar payload must be exactly 32 bytes");

bool nrf24_init_tx(void);
bool nrf24_send_payload(const uint8_t *data, uint8_t len);
bool nrf24_send_solar_panel_packet(const nrf24_solar_panel_packet_t *packet);

#ifdef __cplusplus
}
#endif

#endif

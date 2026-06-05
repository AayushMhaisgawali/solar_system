#ifndef NRF24_LINK_H_
#define NRF24_LINK_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF24_PACKET_TYPE_CHARGER 0x01

typedef struct __attribute__((packed)) {
    uint8_t  type;                  // Packet identifier (0x01)
    uint16_t sequence;              // Rolling packet counter
    uint8_t  panel_index;           // CHANGED: Holds 1 to 12 for individual panel tracking
    uint8_t  flags;                 // Status flags (0x80 for sensor fault)
    
    int16_t  measured_temp_x100;    // Individual panel NTC temp (measured_temp * 100)
    int16_t  predicted_temp_x100;   // Individual predicted ML temp (predicted_temp * 100)
    int16_t  delta_temp_x100;       // individual Delta T (deltaT * 100)
    int16_t  dust_index_x100;       // Individual Dust Index (dust_index * 100)
    int32_t  panel_power_x100;      // Individual Panel Calculated Power (panel_power * 100)
    
    int16_t  sht30_temp_x100;       // Global Ambient Temperature
    uint16_t sht30_humidity_x100;   // Global Humidity percentage
    uint16_t irradiance_x10;        // Global Solar Irradiance (lux / 120.0f)
    
    uint8_t  reserved[4];           // Padding bytes to maintain structure alignment
} nrf24_charger_packet_t;

bool nrf24_init_tx(void);
bool nrf24_send_payload(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* NRF24_LINK_H_ */
#ifndef NRF24_LINK_H
#define NRF24_LINK_H

#include <stdbool.h>
#include <stdint.h>

#define NRF24_PAYLOAD_SIZE 32

// Exact Struct Match for Receiver (Total 23 bytes)
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

bool nrf24_init_tx(void);
bool nrf24_send_payload(const uint8_t *data, uint8_t len);
bool nrf24_send_solar_panel_packet(const nrf24_solar_panel_packet_t *packet);

#endif // NRF24_LINK_H

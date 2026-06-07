#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "nrf24_link.h" 

static const char *TAG = "SOLAR_APP";

#define NRF_PACKET_TYPE_SOLAR_PANEL 0x01

static float clamp_float(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

static float calculate_dust_index(float measured_temp, float predicted_temp) {
    float delta = predicted_temp - measured_temp;
    if (delta < 0.0f) delta = 0.0f;
    return clamp_float(delta * 10.0f, 0.0f, 100.0f);
}

void app_main(void) {
    if (!nrf24_init_tx()) {
        ESP_LOGE(TAG, "Radio hardware subsystem failed to init!");
        return;
    }
    
    uint16_t seq = 0;

    while (1) {
        float measured_t = 32.5f;
        float predicted_t = 34.1f;
        float dust = calculate_dust_index(measured_t, predicted_t);
        
        float dust_factor = 1.0f - clamp_float(dust / 100.0f, 0.0f, 1.0f);
        float p_out = 15.0f * dust_factor; 

        nrf24_solar_panel_packet_t packet = {
            .type = NRF_PACKET_TYPE_SOLAR_PANEL,
            .panel_index = 1,
            .sequence = ++seq,
            .flags = 0x00,
            .measured_temp_x100 = (int16_t)(measured_t * 100),
            .predicted_temp_x100 = (int16_t)(predicted_t * 100),
            .delta_temp_x100 = (int16_t)((predicted_t - measured_t) * 100),
            .dust_index_x100 = (uint16_t)(dust * 100),
            .panel_power_x100 = (uint16_t)(p_out * 100),
            .sht30_temp_x100 = 2850,
            .sht30_humidity_x100 = 5500,
            .irradiance_x10 = 8505 
        };

        if (nrf24_send_solar_panel_packet(&packet)) {
            ESP_LOGI(TAG, "TX Success! Sent Packet %u, Power: %.2f W", seq, p_out);
        } else {
            ESP_LOGW(TAG, "TX Drop. Packet %u reached max retries without receiver ACK.", seq);
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
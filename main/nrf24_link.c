#include "nrf24_link.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NRF_CE_GPIO 42
#define NRF_CSN_GPIO 45
#define NRF_SCK_GPIO 39
#define NRF_MOSI_GPIO 40
#define NRF_MISO_GPIO 41

#define NRF_CHANNEL 76
#define NRF_PAYLOAD_LEN 32
#define NRF_SPI_CLOCK_HZ (500 * 1000)
#define NRF_RF_SETUP_250KBPS_REDUCED_PA 0x22
#define NRF_SETUP_RETR_DISABLED 0x00

#define CMD_R_REGISTER 0x00
#define CMD_W_REGISTER 0x20
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_FLUSH_TX 0xE1
#define CMD_NOP 0xFF

#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_RX_ADDR_P0 0x0A
#define REG_TX_ADDR 0x10
#define REG_RX_PW_P0 0x11
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D

#define STATUS_RX_DR BIT(6)
#define STATUS_TX_DS BIT(5)
#define STATUS_MAX_RT BIT(4)

static const char *TAG = "NRF24_SOLAR_TX";
static const uint8_t BESS_ADDR[5] = {'B', 'E', 'S', 'S', '1'};
static spi_device_handle_t s_spi;

static esp_err_t nrf_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(s_spi, &t);
}

static uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {CMD_R_REGISTER | (reg & 0x1F), CMD_NOP};
    uint8_t rx[2] = {0};
    (void)nrf_transfer(tx, rx, sizeof(tx));
    return rx[1];
}

static void nrf_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {CMD_W_REGISTER | (reg & 0x1F), value};
    (void)nrf_transfer(tx, NULL, sizeof(tx));
}

static void nrf_write_buf(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t tx[1 + 5] = {0};
    tx[0] = CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], data, len);
    (void)nrf_transfer(tx, NULL, len + 1);
}

static void nrf_command(uint8_t cmd)
{
    uint8_t tx = cmd;
    (void)nrf_transfer(&tx, NULL, 1);
}

bool nrf24_init_tx(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = NRF_MOSI_GPIO,
        .miso_io_num = NRF_MISO_GPIO,
        .sclk_io_num = NRF_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = NRF_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = NRF_CSN_GPIO,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return false;
    }
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(ret));
        return false;
    }

    gpio_config_t ce_cfg = {
        .pin_bit_mask = 1ULL << NRF_CE_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&ce_cfg));
    gpio_set_level(NRF_CE_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    nrf_write_reg(REG_CONFIG, 0x0C);
    vTaskDelay(pdMS_TO_TICKS(5));
    nrf_write_reg(REG_EN_AA, 0x00);
    nrf_write_reg(REG_EN_RXADDR, 0x01);
    nrf_write_reg(REG_SETUP_AW, 0x03);
    nrf_write_reg(REG_SETUP_RETR, NRF_SETUP_RETR_DISABLED);
    nrf_write_reg(REG_RF_CH, NRF_CHANNEL);
    nrf_write_reg(REG_RF_SETUP, NRF_RF_SETUP_250KBPS_REDUCED_PA);
    nrf_write_buf(REG_TX_ADDR, BESS_ADDR, sizeof(BESS_ADDR));
    nrf_write_buf(REG_RX_ADDR_P0, BESS_ADDR, sizeof(BESS_ADDR));
    nrf_write_reg(REG_RX_PW_P0, NRF_PAYLOAD_LEN);
    nrf_write_reg(REG_EN_AA, 0x00);
    nrf_write_reg(REG_FEATURE, 0x00);
    nrf_write_reg(REG_DYNPD, 0x00);
    nrf_write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
    nrf_command(CMD_FLUSH_TX);
    nrf_write_reg(REG_CONFIG, 0x0E);
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t config = nrf_read_reg(REG_CONFIG);
    uint8_t channel = nrf_read_reg(REG_RF_CH);
    uint8_t rf_setup = nrf_read_reg(REG_RF_SETUP);
    if (config == 0x00 || config == 0xFF || (config & 0x0E) != 0x0E ||
        channel != NRF_CHANNEL || rf_setup != NRF_RF_SETUP_250KBPS_REDUCED_PA) {
        ESP_LOGE(TAG, "NRF24 bad readback CONFIG=0x%02X CH=%u RF=0x%02X",
                 config, channel, rf_setup);
        return false;
    }

    ESP_LOGI(TAG, "TX channel %u, 250kbps, shared address, no ACK, SPI=%uHz",
             NRF_CHANNEL, NRF_SPI_CLOCK_HZ);
    return true;
}

bool nrf24_send_solar_panel_packet(const nrf24_solar_panel_packet_t *packet)
{
    if (packet == NULL) {
        return false;
    }

    uint8_t tx[1 + NRF_PAYLOAD_LEN] = {CMD_W_TX_PAYLOAD};
    memcpy(&tx[1], packet, sizeof(*packet));

    nrf_write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT);
    nrf_command(CMD_FLUSH_TX);
    if (nrf_transfer(tx, NULL, sizeof(tx)) != ESP_OK) {
        return false;
    }

    gpio_set_level(NRF_CE_GPIO, 1);
    esp_rom_delay_us(20);
    gpio_set_level(NRF_CE_GPIO, 0);

    for (int i = 0; i < 120; i++) {
        uint8_t status = nrf_read_reg(REG_STATUS);
        if ((status & STATUS_TX_DS) != 0) {
            nrf_write_reg(REG_STATUS, STATUS_TX_DS);
            return true;
        }
        if ((status & STATUS_MAX_RT) != 0) {
            nrf_write_reg(REG_STATUS, STATUS_MAX_RT);
            nrf_command(CMD_FLUSH_TX);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    nrf_command(CMD_FLUSH_TX);
    return false;
}

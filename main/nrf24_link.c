#include "nrf24_link.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "NRF24_SOLAR_TX";

#define NRF_RF_CHANNEL_LONG_RANGE 76
#define NRF_SETUP_RETR_LONG_RANGE 0xFF
#define NRF_RF_SETUP_250KBPS_LOW_POWER 0x26

#define NRF_CE_PIN GPIO_NUM_42
#define NRF_CSN_PIN GPIO_NUM_45
#define NRF_SCK_PIN GPIO_NUM_39
#define NRF_MOSI_PIN GPIO_NUM_40
#define NRF_MISO_PIN GPIO_NUM_41

#define NRF_SPI_HOST SPI2_HOST
#define NRF_SPI_CLOCK_HZ (1000 * 1000)

#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_RX_ADDR_P0  0x0A
#define REG_TX_ADDR     0x10
#define REG_RX_PW_P0    0x11
#define REG_DYNPD       0x1C
#define REG_FEATURE     0x1D

#define CMD_R_REGISTER    0x00
#define CMD_W_REGISTER    0x20
#define CMD_ACTIVATE      0x50
#define CMD_W_TX_PAYLOAD  0xA0
#define CMD_FLUSH_TX      0xE1
#define CMD_FLUSH_RX      0xE2
#define CMD_NOP           0xFF

#define CONFIG_EN_CRC     (1U << 3)
#define CONFIG_CRCO       (1U << 2)
#define CONFIG_PWR_UP     (1U << 1)

#define STATUS_RX_DR      (1U << 6)
#define STATUS_TX_DS      (1U << 5)
#define STATUS_MAX_RT     (1U << 4)
#define STATUS_CLEAR_MASK (STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT)

static const uint8_t pipe_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

static spi_device_handle_t s_nrf;
static bool s_initialized;

static inline void ce_high(void)
{
    gpio_set_level(NRF_CE_PIN, 1);
}

static inline void ce_low(void)
{
    gpio_set_level(NRF_CE_PIN, 0);
}

static esp_err_t spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t transaction = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    return spi_device_polling_transmit(s_nrf, &transaction);
}

static uint8_t read_reg(uint8_t reg)
{
    uint8_t tx[2] = {(uint8_t)(CMD_R_REGISTER | (reg & 0x1F)), CMD_NOP};
    uint8_t rx[2] = {0};
    spi_transfer(tx, rx, sizeof(tx));
    return rx[1];
}

static void write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(CMD_W_REGISTER | (reg & 0x1F)), value};
    uint8_t rx[2] = {0};
    spi_transfer(tx, rx, sizeof(tx));
}

static void write_reg_multi(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t tx[6] = {0};
    uint8_t rx[6] = {0};

    tx[0] = CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], data, len);
    spi_transfer(tx, rx, len + 1);
}

static void send_command(uint8_t command)
{
    uint8_t tx[1] = {command};
    uint8_t rx[1] = {0};
    spi_transfer(tx, rx, sizeof(tx));
}

static void flush_tx(void)
{
    send_command(CMD_FLUSH_TX);
}

static void flush_rx(void)
{
    send_command(CMD_FLUSH_RX);
}

static void clear_status(void)
{
    write_reg(REG_STATUS, STATUS_CLEAR_MASK);
}

static void activate_features(void)
{
    uint8_t tx[2] = {CMD_ACTIVATE, 0x73};
    uint8_t rx[2] = {0};
    spi_transfer(tx, rx, sizeof(tx));
}

static void write_tx_payload(const uint8_t *data)
{
    uint8_t tx[1 + NRF24_PAYLOAD_SIZE] = {0};
    uint8_t rx[1 + NRF24_PAYLOAD_SIZE] = {0};

    tx[0] = CMD_W_TX_PAYLOAD;
    memcpy(&tx[1], data, NRF24_PAYLOAD_SIZE);
    spi_transfer(tx, rx, sizeof(tx));
}

bool nrf24_init_tx(void)
{
    if (s_initialized) {
        return true;
    }

    gpio_config_t ce_config = {
        .pin_bit_mask = (1ULL << NRF_CE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ce_config));
    ce_low();

    spi_bus_config_t bus = {
        .mosi_io_num = NRF_MOSI_PIN,
        .miso_io_num = NRF_MISO_PIN,
        .sclk_io_num = NRF_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1 + NRF24_PAYLOAD_SIZE,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(NRF_SPI_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev = {
        .clock_speed_hz = NRF_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = NRF_CSN_PIN,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(NRF_SPI_HOST, &dev, &s_nrf));

    vTaskDelay(pdMS_TO_TICKS(100));

    flush_tx();
    flush_rx();
    clear_status();

    write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP);
    write_reg(REG_EN_AA, 0x01);
    write_reg(REG_EN_RXADDR, 0x01);
    write_reg(REG_SETUP_AW, 0x03);
    write_reg(REG_SETUP_RETR, NRF_SETUP_RETR_LONG_RANGE);
    write_reg(REG_RF_CH, NRF_RF_CHANNEL_LONG_RANGE);
    write_reg(REG_RF_SETUP, NRF_RF_SETUP_250KBPS_LOW_POWER);
    activate_features();
    write_reg(REG_DYNPD, 0x01);
    write_reg(REG_FEATURE, 0x04);
    write_reg_multi(REG_TX_ADDR, pipe_addr, sizeof(pipe_addr));
    write_reg_multi(REG_RX_ADDR_P0, pipe_addr, sizeof(pipe_addr));

    vTaskDelay(pdMS_TO_TICKS(10));

    s_initialized = true;

    ESP_LOGI(TAG,
             "NRF24 SOLAR TX ready CE=%d CSN=%d SCK=%d MOSI=%d MISO=%d SPI=%uHz CH=%u RF=0x%02X PW=%u ACK DPL",
             NRF_CE_PIN,
             NRF_CSN_PIN,
             NRF_SCK_PIN,
             NRF_MOSI_PIN,
             NRF_MISO_PIN,
             NRF_SPI_CLOCK_HZ,
             read_reg(REG_RF_CH),
             read_reg(REG_RF_SETUP),
             read_reg(REG_RX_PW_P0));

    return true;
}

bool nrf24_send_payload(const uint8_t *data, uint8_t len)
{
    uint8_t payload[NRF24_PAYLOAD_SIZE] = {0};
    uint8_t status = 0;
    int timeout_ms = 250;

    if (data == NULL) {
        return false;
    }

    if (len > NRF24_PAYLOAD_SIZE) {
        len = NRF24_PAYLOAD_SIZE;
    }

    memcpy(payload, data, len);

    ce_low();
    flush_tx();
    clear_status();
    write_tx_payload(payload);

    ce_high();
    esp_rom_delay_us(20);
    ce_low();

    do {
        status = read_reg(REG_STATUS);
        if (status & (STATUS_TX_DS | STATUS_MAX_RT)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (--timeout_ms > 0);

    clear_status();

    if (timeout_ms == 0 || (status & STATUS_MAX_RT)) {
        flush_tx();
        ESP_LOGW(TAG, "TX failed status=0x%02X", status);
        return false;
    }

    return true;
}

bool nrf24_send_solar_panel_packet(const nrf24_solar_panel_packet_t *packet)
{
    if (packet == NULL) {
        return false;
    }

    return nrf24_send_payload((const uint8_t *)packet, sizeof(*packet));
}

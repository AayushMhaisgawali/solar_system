# Solar System Firmware Architecture

## 1. Purpose

This firmware runs on the solar PV sender ESP32-S3. It measures solar panel temperature, ambient temperature, humidity, and light intensity, estimates irradiance, predicted panel temperature, dust index, and solar output power, then sends panel-wise telemetry to the BESS receiver over NRF24L01+PA+LNA.

The project is located at:

```text
D:\Solar_system
```

## 2. Firmware Module Layout

| File | Responsibility |
|---|---|
| `main/main.c` | Main application. Initializes I2C, sensors, NTC MUX/ADC, NRF24, computes solar metrics, sends packets. |
| `main/ntc.c/.h` | Reads 12 NTC temperature sensors through an analog multiplexer and ADC. |
| `main/sht30.c/.h` | Reads ambient temperature and humidity from SHT30 over I2C. |
| `main/veml6030.c/.h` | Reads lux from VEML6030 ambient light sensor over I2C. |
| `main/nrf24_link.c/.h` | Configures NRF24 radio and transmits solar panel packets. |
| `main/CMakeLists.txt` | Registers all firmware source files for ESP-IDF build. |

## 3. High-Level Data Flow

```text
SHT30 ambient sensor
  -> ambient temperature and humidity

VEML6030 light sensor
  -> lux
  -> estimated irradiance

12 NTC sensors through MUX
  -> measured panel temperatures

Solar algorithm
  -> predicted panel temperature
  -> delta temperature
  -> dust index
  -> estimated solar array power
  -> panel power share

NRF24 sender
  -> one packet per panel
  -> BESS receiver
```

## 4. Hardware Connections

### 4.1 Shared I2C Bus

SHT30 and VEML6030 share the same I2C bus.

| Signal | ESP32-S3 Pin | Purpose |
|---|---:|---|
| SDA | GPIO8 | I2C data |
| SCL | GPIO9 | I2C clock |
| VCC | 3V3 | Sensor power |
| GND | GND | Common ground |

Firmware settings:

```c
#define SOLAR_I2C_PORT     I2C_NUM_0
#define SOLAR_I2C_SDA_GPIO 8
#define SOLAR_I2C_SCL_GPIO 9
```

I2C speed:

```text
100 kHz
```

### 4.2 SHT30

| SHT30 Pin | ESP32-S3 Pin |
|---|---:|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO8 |
| SCL | GPIO9 |

SHT30 I2C address:

```c
#define SHT30_ADDR 0x44
```

Output:

```text
ambient temperature in C
relative humidity in %
```

### 4.3 VEML6030

| VEML6030 Pin | ESP32-S3 Pin |
|---|---:|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO8 |
| SCL | GPIO9 |

The firmware configures VEML6030 with:

```c
#define VEML6030_GAIN_1_8 (2 << 11)
#define VEML6030_IT_25MS  (12 << 6)
```

Output:

```text
lux
```

### 4.4 NTC Temperature Sensors With Analog MUX

The firmware supports 12 NTC sensors:

```c
#define NUM_NTC_SENSORS 12
```

MUX select pins:

| MUX Signal | ESP32-S3 Pin |
|---|---:|
| S0 | GPIO10 |
| S1 | GPIO11 |
| S2 | GPIO12 |
| S3 | GPIO13 |

ADC input:

| Signal | ESP32-S3 Pin |
|---|---:|
| MUX analog output | ADC1_CH0 |

Firmware ADC settings:

```c
#define ADC_UNIT_ID ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_0
#define ADC_ATTEN   ADC_ATTEN_DB_12
```

NTC divider settings:

```c
#define VCC      3.3f
#define R_PULLUP 10000.0f
#define MUX_RON  70.0f
```

### 4.5 NRF24L01+PA+LNA Solar Sender

| NRF24 Pin | ESP32-S3 Pin |
|---|---:|
| VCC | 3V3 |
| GND | GND |
| SCK | GPIO39 |
| MOSI | GPIO40 |
| MISO | GPIO41 |
| CE | GPIO42 |
| CSN | GPIO45 |
| IRQ | Not connected |

Firmware settings:

```c
#define NRF_CE_PIN   GPIO_NUM_42
#define NRF_CSN_PIN  GPIO_NUM_45
#define NRF_SCK_PIN  GPIO_NUM_39
#define NRF_MOSI_PIN GPIO_NUM_40
#define NRF_MISO_PIN GPIO_NUM_41
```

## 5. Startup Sequence

`app_main()` performs this sequence:

```text
1. Initialize I2C bus on GPIO8/GPIO9
2. Initialize SHT30
3. Initialize VEML6030
4. Initialize NTC MUX GPIO and ADC
5. Initialize NRF24 transmitter
6. Enter continuous measurement/transmit loop
```

Relevant code flow:

```c
solar_i2c_init();
sht30_init();
veml6030_init();
ntc_init();
nrf24_init_tx();
```

If NRF initialization fails, the firmware logs an error and returns from `app_main()`.

## 6. Main Loop Behavior

Each loop does:

```text
1. Read ambient temperature and humidity from SHT30
2. Read lux from VEML6030
3. Convert lux to approximate irradiance
4. Read all 12 NTC panel temperatures
5. Calculate predicted panel temperature for each panel
6. Calculate dust index for each panel
7. Calculate average panel temperature and average dust index
8. Estimate total solar array output power
9. Divide power across 12 panels as panel power share
10. Send one NRF packet per panel
11. Wait 2 seconds
```

Timing:

```text
70 ms delay between panel packets
2000 ms delay after all 12 panel packets
```

## 7. NTC Temperature Measurement

The NTC module reads one selected MUX channel at a time.

### 7.1 MUX Selection

For channel `ch`, the firmware sets:

```c
S0 = bit0 of ch
S1 = bit1 of ch
S2 = bit2 of ch
S3 = bit3 of ch
```

Then it waits:

```text
30 ms
```

This settling delay is for the analog path and capacitor.

### 7.2 ADC Sampling

The ADC read function does two stages:

```text
1. Flush capacitor using 20 ADC reads
2. Oversample using 64 ADC reads
```

Then it averages the 64 samples.

If ADC calibration is available, raw ADC is converted to millivolts using:

```c
adc_cali_raw_to_voltage()
```

Otherwise:

```text
mv = raw / 4095 * 3300
```

### 7.3 Resistance Calculation

The NTC resistance is calculated from the divider:

```text
R_ntc = (R_pullup * Vout / (3300 - Vout)) - MUX_RON
```

In code:

```c
resistance = (R_PULLUP * voltage_mv / (3300.0f - voltage_mv)) - MUX_RON;
```

Valid resistance range:

```c
#define NTC_MIN_RESISTANCE 1000.0f
#define NTC_MAX_RESISTANCE 100000.0f
```

If the resistance is outside this range, the sensor reading is treated as invalid and returns `NAN`.

### 7.4 Temperature Calculation

Temperature is calculated using Steinhart-Hart:

```text
1/T = A + B*ln(R) + C*(ln(R))^3
```

Coefficients:

```c
A = 0.001129148
B = 0.000234125
C = 0.0000000876741
```

Temperature output:

```text
T_C = (1 / invT) - 273.15
```

## 8. SHT30 Ambient Measurement

The SHT30 command used is:

```c
#define CMD_MEASURE 0x2400
```

After command write, firmware waits:

```text
20 ms
```

Temperature conversion:

```text
temperature = -45 + 175 * rawT / 65535
```

Humidity conversion:

```text
humidity = 100 * rawH / 65535
```

If SHT30 fails:

```text
ambient temperature fallback = 25.0 C
humidity fallback = 0.0 %
```

## 9. VEML6030 Lux Measurement

The VEML6030 reads raw ALS data from register:

```c
#define VEML6030_REG_ALS 0x04
```

Raw lux estimate:

```text
lux = raw * 1.8432
```

For values above 100 lux, a polynomial correction is applied:

```text
lux_corrected =
  6.0135e-13 * lux^4
- 9.3924e-9  * lux^3
+ 8.1488e-5  * lux^2
+ 1.0023     * lux
```

If VEML6030 read fails:

```text
lux = 0
```

## 10. Solar Estimation Algorithm

### 10.1 Lux To Irradiance

The firmware approximates irradiance from lux:

```text
irradiance_wm2 = lux / 120
```

Then clamps it:

```text
0 to 1200 W/m2
```

### 10.2 Expected Panel Temperature

Expected thermal rise:

```text
expected_rise = irradiance_wm2 * 0.025
```

Predicted panel temperature:

```text
predicted_temp = ambient_temp + expected_rise
```

Example:

```text
ambient_temp = 30 C
lux = 60000 lx
irradiance = 60000 / 120 = 500 W/m2
expected_rise = 500 * 0.025 = 12.5 C
predicted_temp = 30 + 12.5 = 42.5 C
```

### 10.3 Dust Index

Dust index compares predicted panel temperature and measured panel temperature:

```text
dust_index = ((predicted_temp - measured_temp) / predicted_temp) * 100
```

Then it is clamped:

```text
0 to 100 %
```

Example:

```text
predicted_temp = 42.5 C
measured_temp = 39.0 C
dust_index = ((42.5 - 39.0) / 42.5) * 100
dust_index = 8.24 %
```

If measured temperature is invalid:

```text
measured_temp = ambient_temp
flags |= 0x01
```

### 10.4 Average Panel Temperature And Dust

For all valid NTC panels:

```text
avg_panel_temp = average(measured_temps)
avg_dust_index = average(dust_indexes)
```

If all NTC readings are invalid:

```text
avg_panel_temp = ambient_temp
avg_dust_index = 0
```

### 10.5 Solar Output Power

Constants:

```c
#define SOLAR_ARRAY_STC_POWER_W 6600.0f
```

Irradiance factor:

```text
irradiance_factor = irradiance_wm2 / 1000
```

Clamped:

```text
0 to 1.2
```

Temperature factor:

```text
temp_factor = 1 - 0.0035 * (avg_panel_temp - 25)
```

Dust factor:

```text
dust_factor = 1 - avg_dust_index / 100
```

Solar output:

```text
solar_p_out = 6600 * irradiance_factor * temp_factor * dust_factor
```

Clamped:

```text
0 to 7920 W
```

Panel power share:

```text
panel_power_share = solar_p_out / 12
```

Clamped:

```text
0 to 655.35 W
```

Example:

```text
lux = 60000 lx
irradiance = 500 W/m2
avg_panel_temp = 40 C
avg_dust_index = 8 %

irradiance_factor = 500 / 1000 = 0.5
temp_factor = 1 - 0.0035 * (40 - 25) = 0.9475
dust_factor = 1 - 8/100 = 0.92

solar_p_out = 6600 * 0.5 * 0.9475 * 0.92
solar_p_out = 2876.61 W

panel_power_share = 2876.61 / 12
panel_power_share = 239.72 W
```

## 11. NRF24 Radio Architecture

Current NRF settings:

```c
#define NRF_RF_CHANNEL_LONG_RANGE 76
#define NRF_SETUP_RETR_LONG_RANGE 0xFF
#define NRF_RF_SETUP_250KBPS_LOW_POWER 0x22
#define NRF_SPI_CLOCK_HZ (1000 * 1000)
```

Radio address:

```c
static const uint8_t solar_pipe_addr[5] = {0xD7, 0x3A, 0x3A, 0x3A, 0x3A};
```

Payload behavior:

```text
Max payload size: 32 bytes
Packet struct size: 23 bytes
Payload is copied into a 32-byte zero-padded buffer
Dynamic payload is enabled on pipe 0
```

Current NRF configuration:

```text
Channel: 76
RF setup: 0x22
Auto acknowledgement: disabled
Setup retry: 0xFF configured, but ignored because auto ACK is disabled
Dynamic payload: enabled
CRC: enabled
Address width: 5 bytes
```

Important receiver requirement:

```text
Receiver must use the same channel, address, payload mode, packet type, and struct layout.
```

## 12. NRF Packet Format

Packet type:

```c
#define NRF_PACKET_TYPE_SOLAR_PANEL 0x51
```

The solar sender transmits one packet per panel:

```c
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
```

Payload fields:

| Field | Meaning | Decode |
|---|---|---|
| `type` | Solar packet identifier, `0x51` | Check before decoding |
| `panel_index` | Panel number | `1` to `12` |
| `sequence` | Packet counter | Incrementing |
| `flags` | Status flags | `0x01` means NTC fallback used |
| `measured_temp_x100` | NTC measured panel temperature | `/ 100.0` C |
| `predicted_temp_x100` | Predicted panel temperature | `/ 100.0` C |
| `delta_temp_x100` | Predicted minus measured temperature | `/ 100.0` C |
| `dust_index_x100` | Estimated dust index | `/ 100.0` % |
| `panel_power_x100` | Estimated panel power share | `/ 100.0` W |
| `sht30_temp_x100` | Ambient temperature | `/ 100.0` C |
| `sht30_humidity_x100` | Relative humidity | `/ 100.0` % |
| `irradiance_x10` | Estimated irradiance | `/ 10.0` W/m2 |

Example encoding:

```text
panel_index = 3
measured_temp = 39.00 C      -> 3900
predicted_temp = 42.50 C     -> 4250
delta_temp = 3.50 C          -> 350
dust_index = 8.24 %          -> 824
panel_power_share = 239.72 W -> 23972
ambient_temp = 30.00 C       -> 3000
humidity = 55.50 %           -> 5550
irradiance = 500.0 W/m2      -> 5000
```

Receiver decoding example:

```c
float measured_temp = packet.measured_temp_x100 / 100.0f;
float predicted_temp = packet.predicted_temp_x100 / 100.0f;
float delta_temp = packet.delta_temp_x100 / 100.0f;
float dust_index = packet.dust_index_x100 / 100.0f;
float panel_power = packet.panel_power_x100 / 100.0f;
float ambient_temp = packet.sht30_temp_x100 / 100.0f;
float humidity = packet.sht30_humidity_x100 / 100.0f;
float irradiance = packet.irradiance_x10 / 10.0f;
```

## 13. Logging Output

Environment summary log:

```text
Solar env Lux=<lux> lx Irr=<irradiance> W/m2 AvgT=<avg_panel_temp>C AvgDI=<avg_dust_index>% P_out=<solar_p_out>W
```

Per-panel transmit log:

```text
TX panel=<panel> seq=<sequence> T=<measured>C Pred=<predicted>C DI=<dust>% Lux=<lux> lx ADC=<ntc_mv>mV Pshare=<panel_power>W
```

Failure log:

```text
TX Drop panel=<panel> seq=<sequence>
```

## 14. Timing Summary

| Operation | Timing |
|---|---:|
| I2C speed | 100 kHz |
| SHT30 conversion wait | 20 ms |
| VEML6030 init wait | 200 ms |
| MUX settling delay | 30 ms per channel |
| ADC flush reads | 20 reads |
| ADC oversampling reads | 64 reads |
| NRF delay between panel packets | 70 ms |
| Main loop delay after 12 panels | 2000 ms |

## 15. Build Configuration

`main/CMakeLists.txt` registers:

```cmake
idf_component_register(SRCS "nrf24_link.c" "main.c"
                             "sht30.c"
                             "veml6030.c"
                             "ntc.c"
                    INCLUDE_DIRS ".")
```

Build command:

```text
idf.py build
```

## 16. Architecture Notes And Risks

### 16.1 Duplicate `app_main()` In NRF File

`nrf24_link.c` currently contains an example `solar_tx_task()` and an `app_main()` at the bottom of the file.

This is risky because the real application entry point is already in `main.c`.

Recommended cleanup:

```text
Remove or wrap the example solar_tx_task() and app_main() in nrf24_link.c.
Keep nrf24_link.c as a driver/module only.
```

### 16.2 NRF ACK Configuration

The code sets:

```c
write_reg(REG_EN_AA, 0x00);
```

So auto acknowledgement is disabled. This can simplify communication, but it also means the sender cannot know whether the receiver actually received the packet.

### 16.3 Dynamic Payload Mode

The code enables dynamic payload:

```c
write_reg(REG_FEATURE, 0x04);
write_reg(REG_DYNPD, 0x01);
```

Receiver must also support dynamic payload correctly, or use the same fixed 32-byte handling expected by the sender.

### 16.4 Packet Size Comment Mismatch

The header comment says:

```text
Exact Struct Match for Receiver (Total 23 bytes)
```

But the NRF send path copies it into a 32-byte payload buffer.

Receiver should be clear whether it reads:

```text
23-byte dynamic payload
```

or:

```text
32-byte fixed payload
```

Both sides must match.

## 17. End-To-End Example

Assume:

```text
SHT30 ambient temperature = 30.0 C
SHT30 humidity = 55.5 %
VEML6030 lux = 60000 lx
NTC panel 3 measured temperature = 39.0 C
```

Irradiance:

```text
irradiance = 60000 / 120 = 500 W/m2
```

Predicted panel temperature:

```text
expected_rise = 500 * 0.025 = 12.5 C
predicted_temp = 30.0 + 12.5 = 42.5 C
```

Delta temperature:

```text
delta_temp = predicted_temp - measured_temp
delta_temp = 42.5 - 39.0 = 3.5 C
```

Dust index:

```text
dust_index = (3.5 / 42.5) * 100
dust_index = 8.24 %
```

Power:

```text
irradiance_factor = 0.5
temp_factor = 0.9475
dust_factor = 0.9176
solar_p_out = 6600 * 0.5 * 0.9475 * 0.9176
solar_p_out = 2869.10 W
panel_power_share = 2869.10 / 12 = 239.09 W
```

Packet:

```text
type = 0x51
panel_index = 3
measured_temp_x100 = 3900
predicted_temp_x100 = 4250
delta_temp_x100 = 350
dust_index_x100 = 824
panel_power_x100 = 23909
sht30_temp_x100 = 3000
sht30_humidity_x100 = 5550
irradiance_x10 = 5000
```


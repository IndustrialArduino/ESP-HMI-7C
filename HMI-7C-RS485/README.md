# ESP-HMI-7C RS485 MODBUS Project

## Overview

This project implements a **human-machine interface (HMI)** on an **ESP32** with a 7-inch RGB display, touch panel, Modbus sensor integration, and various I/O peripherals. It reads **temperature and humidity** from a Modbus XY-MD02 sensor, displays values on an **LVGL-based GUI**, and visualizes the data using **vertical thermometer bars and humidity arcs**.

**Key features:**

- ESP32 with **Arduino-ESP32RGBPanel** driver for 1024×600 display
- Touchscreen support using **TAMC_GT911**
- **Modbus RTU over RS485** to read temperature/humidity data
- I2C GPIO expansion using **PCA9536 / PCA9538**
- **LVGL GUI** designed in SquareLine Studio
- Temperature and Humidity zones (**COLD/WARM/HOT**, **DRY/COMFORT/HUMID**)
- Display visualizations: **vertical thermometer, humidity arc, numeric values**
- Optional **Ethernet/UDP/NTP** support
- **ADS1115** analog sensor reading support
- SD card and SPI peripherals ready

---

## Hardware Setup

### ESP32 Pins

| Peripheral       | ESP32 Pin | Notes |
|-----------------|-----------|-------|
| SDA (I2C)       | 19        | PCA9536 / TAMC_GT911 |
| SCL (I2C)       | 20        | PCA9536 / TAMC_GT911 |
| RS485 RXD       | 2         | Serial1 RX |
| RS485 TXD       | 9         | Serial1 TX |
| UART_SEL        | 3         | I2C-controlled UART selection |
| GSM_RESET       | 2         | I2C-controlled GSM reset |
| TFT_RST         | 39        | Display reset |
| SPI SCLK        | 11        | SPI bus for SD/Ethernet |
| SPI MISO        | 12        | SPI bus for SD/Ethernet |
| SPI MOSI        | 13        | SPI bus for SD/Ethernet |
| DSP_CS          | 47        | Chip select for SPI devices |
| TFT_BL          | -1        | Backlight control (if defined) |
| Touch INT       | 42        | Interrupt from touch controller |

### I2C Peripherals

- **PCA9536** — GPIO expansion (4 inputs, 4 outputs)
- **PCA9538** — optional additional GPIO expansion
- **TAMC_GT911** — touch controller
- **ADS1115** — optional analog sensor input

---

## Software Components

### Libraries Used

- **Arduino_GFX_Library** — RGB display driver
- **lvgl** — GUI library (version 8.x recommended)
- **SquareLine UI** — LVGL GUI design
- **ModbusMaster** — Read Modbus RTU sensors
- **PCA9536D / PCA9538** — I2C GPIO expanders
- **Adafruit_ADS1X15** — Analog input
- **RTClib** — DS3231 RTC support
- **ArduinoJson** — JSON parsing for potential network data

---

## Code Structure

### `setup()`

1. Initialize **Serial** for debugging and **RS485** for Modbus.  
2. Reset and initialize **TFT display**.  
3. Initialize **I2C** and **PCA9536 GPIO expanders**.  
4. Start **ModbusMaster node** for XY-MD02 sensor.  
5. Initialize **touch panel** via `touch_init()`.  
6. Initialize **LVGL display**, draw buffer, and touch input driver.  
7. Optionally initialize **UI exported from SquareLine Studio**.  
8. Start a **1-second timer (`lv_timer_create`)** to read sensors and update the GUI.

### `GPIOCallback()` — Main Sensor & GUI Update Loop

1. Read temperature and humidity from **XY-MD02 Modbus sensor**:

```cpp
uint8_t result = node.readInputRegisters(0x0001, 2);


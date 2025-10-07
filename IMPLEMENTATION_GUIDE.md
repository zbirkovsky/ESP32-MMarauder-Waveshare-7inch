markdown# Complete Implementation Guide for ESP32 Marauder Waveshare 7-inch Port

## Mission Statement
Transform the ESP32 Marauder firmware into a standalone tablet-style penetration testing tool using the Waveshare ESP32-S3-Touch-LCD-7.0" display. This document contains all instructions, code, and configurations needed to complete this port.

## Repository Information
- **Repository**: https://github.com/zbirkovsky/ESP32-MMarauder-Waveshare-7inch
- **Target Hardware**: Waveshare ESP32-S3-Touch-LCD-7.0" (800×480 IPS with GT911 touch)
- **Base Project**: ESP32 Marauder by justcallmekoko
- **Reference Implementation**: CYD port by Fr4nkFletcher

## Phase 1: Repository Structure Creation

Create the following directory structure and files:

### Directory Structure
ESP32-MMarauder-Waveshare-7inch/
├── .github/
│   └── workflows/
│       └── build.yml
├── src/
├── lib/
├── include/
├── boards/
├── docs/
│   └── images/
├── hardware/
│   ├── case/
│   └── schematics/
├── releases/
├── test/
└── data/
└── web/

### File 1: README.md
```markdown
# ESP32 Marauder for Waveshare 7-inch Display

[![GitHub release](https://img.shields.io/github/release/zbirkovsky/ESP32-MMarauder-Waveshare-7inch.svg)](https://github.com/zbirkovsky/ESP32-MMarauder-Waveshare-7inch/releases)
[![License](https://img.shields.io/github/license/zbirkovsky/ESP32-MMarauder-Waveshare-7inch)](LICENSE)
[![Build Status](https://img.shields.io/github/actions/workflow/status/zbirkovsky/ESP32-MMarauder-Waveshare-7inch/build.yml)](https://github.com/zbirkovsky/ESP32-MMarauder-Waveshare-7inch/actions)

A standalone implementation of ESP32 Marauder firmware optimized for the Waveshare ESP32-S3-Touch-LCD-7.0" display, featuring a tablet-style interface for WiFi/Bluetooth penetration testing.

## ⚡ Features

- **7-inch IPS Touch Display** (800×480) with tablet-style UI
- **Full Marauder Functionality**: WiFi scanning, deauth attacks, Evil Portal, BLE tools
- **Enhanced Interface**: Optimized for large screen with multi-touch support
- **Dual-Core Performance**: WiFi operations on Core 0, GUI on Core 1
- **8MB PSRAM**: Smooth graphics and extensive packet capture
- **SD Card Support**: PCAP storage via high-speed SDIO interface
- **GPS Ready**: UART interface for GPS module integration
- **USB-C Powered**: Modern connectivity and power delivery

## 🚀 Quick Start

### Flash Pre-built Binary
```bash
esptool.py --chip esp32s3 --port /dev/ttyUSB0 --baud 921600 \
  write_flash -z 0x0 marauder-7inch.bin
Build from Source
bashgit clone --recursive https://github.com/zbirkovsky/ESP32-MMarauder-Waveshare-7inch
cd ESP32-MMarauder-Waveshare-7inch
pio run -e waveshare_7inch
pio run -e waveshare_7inch -t upload
📚 Documentation
See docs/ folder for detailed documentation.
⚠️ Disclaimer
This tool is for educational and authorized testing purposes only. Users are responsible for complying with all applicable laws and regulations.
📄 License
This project maintains compatibility with the original ESP32 Marauder license.

### File 2: .gitignore
```gitignore
# PlatformIO
.pio
.vscode/.browse.c_cpp.db*
.vscode/c_cpp_properties.json
.vscode/launch.json
.vscode/ipch

# Build outputs
build/
dist/
*.bin
*.elf
*.map

# IDE
.vscode/*
!.vscode/extensions.json
.idea/
*.swp
*.swo
*~

# OS
.DS_Store
Thumbs.db
desktop.ini

# Python
__pycache__/
*.pyc
.env
venv/
env/

# Temporary
*.tmp
*.bak
*.log
*.orig

# Credentials
wifi_credentials.h
secrets.h
config.json

# Dependencies
node_modules/
bower_components/
File 3: platformio.ini
ini; PlatformIO Project Configuration File
; Waveshare 7-inch ESP32-S3 Display Configuration

[platformio]
default_envs = waveshare_7inch
src_dir = src
lib_dir = lib
include_dir = include
boards_dir = boards

[env:waveshare_7inch]
platform = espressif32@6.5.0
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
upload_speed = 921600
monitor_filters = esp32_exception_decoder, colorize
board_build.mcu = esp32s3
board_build.f_cpu = 240000000L
board_build.flash_size = 16MB
board_build.partitions = partitions.csv
board_build.psram_type = opi
board_upload.flash_size = 16MB

build_flags = 
    ; System Configuration
    -DBOARD_HAS_PSRAM
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
    -DCORE_DEBUG_LEVEL=2
    
    ; Display Configuration
    -DWAVESHARE_7_INCH
    -DLCD_WIDTH=800
    -DLCD_HEIGHT=480
    -DLCD_COLOR_DEPTH=16
    -DLCD_PIXEL_CLOCK_HZ=16000000
    
    ; Display Pin Definitions
    -DLCD_PIN_NUM_VSYNC=41
    -DLCD_PIN_NUM_HSYNC=40
    -DLCD_PIN_NUM_DE=42
    -DLCD_PIN_NUM_PCLK=39
    -DLCD_PIN_NUM_DATA0=15  ; B3
    -DLCD_PIN_NUM_DATA1=7   ; B4
    -DLCD_PIN_NUM_DATA2=6   ; B5
    -DLCD_PIN_NUM_DATA3=5   ; B6
    -DLCD_PIN_NUM_DATA4=4   ; B7
    -DLCD_PIN_NUM_DATA5=9   ; G2
    -DLCD_PIN_NUM_DATA6=46  ; G3
    -DLCD_PIN_NUM_DATA7=3   ; G4
    -DLCD_PIN_NUM_DATA8=8   ; G5
    -DLCD_PIN_NUM_DATA9=16  ; G6
    -DLCD_PIN_NUM_DATA10=1  ; G7
    -DLCD_PIN_NUM_DATA11=14 ; R3
    -DLCD_PIN_NUM_DATA12=21 ; R4
    -DLCD_PIN_NUM_DATA13=47 ; R5
    -DLCD_PIN_NUM_DATA14=48 ; R6
    -DLCD_PIN_NUM_DATA15=45 ; R7
    -DLCD_PIN_NUM_DISP_EN=-1
    -DLCD_PIN_NUM_BK_LIGHT=2
    
    ; Touch Configuration
    -DTOUCH_GT911
    -DTOUCH_SDA=19
    -DTOUCH_SCL=20
    -DTOUCH_INT=18
    -DTOUCH_RST=38
    
    ; SD Card Configuration
    -DSD_CMD=10
    -DSD_CLK=11
    -DSD_D0=13
    -DSD_D1=12
    -DSD_D2=35
    -DSD_D3=36
    
    ; UART Configuration
    -DUART0_TX=43
    -DUART0_RX=44
    
    ; LVGL Configuration
    -DLV_CONF_INCLUDE_SIMPLE
    -DLV_MEM_CUSTOM=1
    -DLV_MEM_SIZE=65536
    -DLV_USE_PSRAM=1
    -DLV_DISP_DEF_REFR_PERIOD=30
    -DLV_TICK_CUSTOM=1
    
    ; Memory Optimization
    -DCONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM=4
    -DCONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM=8
    -DCONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM=8
    -DCONFIG_ESP32_WIFI_TX_BUFFER_TYPE=1
    -DCONFIG_ESP32_WIFI_CACHE_TX_BUFFER_NUM=16
    
    ; Marauder Features
    -DMARAUDER_VERSION=\"0.13.0\"
    -DHAS_SCREEN=1
    -DHAS_TOUCH=1
    -DHAS_SD=1
    -DHAS_GPS=1
    -DHAS_BT=1

lib_deps = 
    ; Display & Graphics
    lvgl/lvgl@^9.2.0
    https://github.com/esp-arduino-libs/ESP32_Display_Panel
    
    ; Touch Driver
    https://github.com/worthylyx/GT911
    
    ; Marauder Dependencies  
    bblanchon/ArduinoJson@^7.0.4
    h2zero/NimBLE-Arduino@^1.4.2
    me-no-dev/AsyncTCP@^1.1.1
    me-no-dev/ESP Async WebServer@^1.2.3
    
    ; Utilities
    Wire
    SPI
    SD
    FS
    SPIFFS
    Preferences

[env:waveshare_7inch_debug]
extends = env:waveshare_7inch
build_type = debug
build_flags = 
    ${env:waveshare_7inch.build_flags}
    -DCORE_DEBUG_LEVEL=5
    -DDEBUG_ESP_WIFI
    -DDEBUG_ESP_PORT=Serial
File 4: partitions.csv
csv# Name,   Type, SubType, Offset,  Size,     Flags
nvs,      data, nvs,     0x9000,  0x5000,
otadata,  data, ota,     0xe000,  0x2000,
app0,     app,  ota_0,   0x10000, 0x300000,
app1,     app,  ota_1,   0x310000,0x300000,
spiffs,   data, spiffs,  0x610000,0x9F0000,
Phase 2: Source Code Implementation
File 5: src/main.cpp
cpp/**
 * ESP32 Marauder for Waveshare 7-inch Display
 * Main application entry point
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <lvgl.h>

#include "Display.h"
#include "TouchDriver.h"
#include "WiFiManager.h"
#include "MarauderMenu.h"
#include "Settings.h"
#include "SDInterface.h"

// Task handles
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t guiTaskHandle = NULL;
TaskHandle_t scanTaskHandle = NULL;

// Semaphores
SemaphoreHandle_t lvglMutex;
SemaphoreHandle_t wifiMutex;

// Global objects
Display display;
TouchDriver touch;
WiFiManager wifiManager;
MarauderMenu menu;
Settings settings;
SDInterface sdCard;

// Function declarations
void wifi_task(void *parameter);
void gui_task(void *parameter);
void scan_task(void *parameter);
void init_hardware();
void init_display();
void init_touch();
void init_wifi();
void init_sd();

void setup() {
    Serial.begin(115200);
    Serial.println(F("\n\n================================="));
    Serial.println(F("ESP32 Marauder - Waveshare 7inch"));
    Serial.println(F("Version: " MARAUDER_VERSION));
    Serial.println(F("=================================\n"));
    
    // Create semaphores
    lvglMutex = xSemaphoreCreateMutex();
    wifiMutex = xSemaphoreCreateMutex();
    
    // Initialize hardware
    init_hardware();
    
    // Load settings from NVS
    settings.begin();
    
    // Initialize display first
    init_display();
    
    // Initialize touch
    init_touch();
    
    // Initialize SD card
    init_sd();
    
    // Initialize WiFi (after display to prevent conflicts)
    init_wifi();
    
    // Create tasks pinned to cores
    xTaskCreatePinnedToCore(
        wifi_task,           // Task function
        "WiFi_Task",        // Task name
        8192,               // Stack size
        NULL,               // Parameters
        5,                  // Priority
        &wifiTaskHandle,    // Task handle
        0                   // Core 0
    );
    
    xTaskCreatePinnedToCore(
        gui_task,           // Task function
        "GUI_Task",         // Task name
        16384,              // Stack size (larger for LVGL)
        NULL,               // Parameters
        3,                  // Priority
        &guiTaskHandle,     // Task handle
        1                   // Core 1
    );
    
    Serial.println(F("Setup complete!"));
}

void loop() {
    // Empty - everything runs in tasks
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void init_hardware() {
    Serial.println(F("Initializing hardware..."));
    
    // Configure I2C for touch controller
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    Wire.setClock(400000); // 400kHz I2C
    
    // Configure SPI for SD card
    SPI.begin(SD_CLK, SD_D0, SD_CMD);
    
    // Configure backlight PWM
    pinMode(LCD_PIN_NUM_BK_LIGHT, OUTPUT);
    ledcSetup(0, 5000, 8); // Channel 0, 5kHz, 8-bit
    ledcAttachPin(LCD_PIN_NUM_BK_LIGHT, 0);
    ledcWrite(0, 128); // 50% brightness initially
    
    Serial.println(F("Hardware initialized"));
}

void init_display() {
    Serial.println(F("Initializing display..."));
    
    if (!display.begin()) {
        Serial.println(F("Display initialization failed!"));
        esp_restart();
    }
    
    // Initialize LVGL
    lv_init();
    
    // Create display buffers in PSRAM
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(
        LCD_WIDTH * 40 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    static lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(
        LCD_WIDTH * 40 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_WIDTH * 40);
    
    // Initialize display driver
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = display.flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    
    Serial.println(F("Display initialized"));
}

void init_touch() {
    Serial.println(F("Initializing touch controller..."));
    
    if (!touch.begin()) {
        Serial.println(F("Touch initialization failed!"));
        // Continue without touch
        return;
    }
    
    // Register touch input device with LVGL
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch.read_cb;
    lv_indev_drv_register(&indev_drv);
    
    Serial.println(F("Touch initialized"));
}

void init_wifi() {
    Serial.println(F("Initializing WiFi..."));
    
    // Configure WiFi with reduced buffers
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.static_rx_buf_num = 4;
    cfg.dynamic_rx_buf_num = 8;
    cfg.dynamic_tx_buf_num = 8;
    
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    
    // Enable promiscuous mode
    esp_wifi_set_promiscuous(true);
    
    Serial.println(F("WiFi initialized"));
}

void init_sd() {
    Serial.println(F("Initializing SD card..."));
    
    if (!sdCard.begin()) {
        Serial.println(F("SD card not found - continuing without"));
        return;
    }
    
    Serial.print(F("SD Card Size: "));
    Serial.print(sdCard.getSize() / 1024 / 1024);
    Serial.println(F(" MB"));
}

void wifi_task(void *parameter) {
    Serial.println(F("WiFi task started on Core 0"));
    
    while (true) {
        // Handle WiFi operations
        if (xSemaphoreTake(wifiMutex, portMAX_DELAY)) {
            wifiManager.update();
            xSemaphoreGive(wifiMutex);
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void gui_task(void *parameter) {
    Serial.println(F("GUI task started on Core 1"));
    
    // Initialize menu
    menu.begin();
    
    while (true) {
        // Handle LVGL
        if (xSemaphoreTake(lvglMutex, portMAX_DELAY)) {
            lv_timer_handler();
            xSemaphoreGive(lvglMutex);
        }
        
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void scan_task(void *parameter) {
    // Implement scan task
    vTaskDelete(NULL);
}
File 6: include/Display.h
cpp#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <lvgl.h>
#include "ESP32_Display_Panel.h"

class Display {
private:
    ESP_Panel *panel;
    uint8_t brightness;
    bool initialized;
    
public:
    Display();
    ~Display();
    
    bool begin();
    void setBrightness(uint8_t level);
    void clear();
    void update();
    
    static void flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
};

#endif // DISPLAY_H
File 7: include/TouchDriver.h
cpp#ifndef TOUCHDRIVER_H
#define TOUCHDRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>

class TouchDriver {
private:
    const uint8_t GT911_ADDR = 0x5D;
    bool initialized;
    
    struct TouchPoint {
        uint16_t x;
        uint16_t y;
        bool touched;
    };
    
    TouchPoint lastTouch;
    
public:
    TouchDriver();
    ~TouchDriver();
    
    bool begin();
    bool read(uint16_t &x, uint16_t &y);
    void calibrate();
    
    static void read_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
};

#endif // TOUCHDRIVER_H
File 8: include/pins.h
cpp#ifndef PINS_H
#define PINS_H

// Display RGB Interface Pins
#define LCD_PIN_NUM_VSYNC    41
#define LCD_PIN_NUM_HSYNC    40
#define LCD_PIN_NUM_DE       42
#define LCD_PIN_NUM_PCLK     39

// Display Data Pins (RGB565)
#define LCD_PIN_NUM_DATA0    15  // B3
#define LCD_PIN_NUM_DATA1    7   // B4  
#define LCD_PIN_NUM_DATA2    6   // B5
#define LCD_PIN_NUM_DATA3    5   // B6
#define LCD_PIN_NUM_DATA4    4   // B7
#define LCD_PIN_NUM_DATA5    9   // G2
#define LCD_PIN_NUM_DATA6    46  // G3
#define LCD_PIN_NUM_DATA7    3   // G4
#define LCD_PIN_NUM_DATA8    8   // G5
#define LCD_PIN_NUM_DATA9    16  // G6
#define LCD_PIN_NUM_DATA10   1   // G7
#define LCD_PIN_NUM_DATA11   14  // R3
#define LCD_PIN_NUM_DATA12   21  // R4
#define LCD_PIN_NUM_DATA13   47  // R5
#define LCD_PIN_NUM_DATA14   48  // R6
#define LCD_PIN_NUM_DATA15   45  // R7

#define LCD_PIN_NUM_BK_LIGHT 2   // Backlight PWM

// Touch I2C Pins
#define TOUCH_SDA            19
#define TOUCH_SCL            20
#define TOUCH_INT            18
#define TOUCH_RST            38

// SD Card SDIO Pins
#define SD_CMD               10
#define SD_CLK               11
#define SD_D0                13
#define SD_D1                12
#define SD_D2                35
#define SD_D3                36

// UART Pins (for GPS)
#define UART0_TX             43
#define UART0_RX             44

// Additional I2C (for sensors)
#define I2C_SDA              17
#define I2C_SCL              18

// ADC Pin
#define ADC_PIN              8

#endif // PINS_H
Phase 3: UI Implementation
File 9: src/ui/MarauderMenu.cpp
cpp#include "MarauderMenu.h"
#include <lvgl.h>

// Screen objects
static lv_obj_t *screen_main;
static lv_obj_t *screen_scan;
static lv_obj_t *screen_attack;
static lv_obj_t *screen_sniff;
static lv_obj_t *screen_settings;

// Main menu tiles
static lv_obj_t *tile_wifi;
static lv_obj_t *tile_ble;
static lv_obj_t *tile_gps;
static lv_obj_t *tile_sd;

MarauderMenu::MarauderMenu() {
    currentScreen = SCREEN_MAIN;
}

void MarauderMenu::begin() {
    createMainScreen();
    createScanScreen();
    createAttackScreen();
    createSniffScreen();
    createSettingsScreen();
    
    // Show main screen
    lv_scr_load(screen_main);
}

void MarauderMenu::createMainScreen() {
    screen_main = lv_obj_create(NULL);
    
    // Create title bar
    lv_obj_t *title = lv_label_create(screen_main);
    lv_label_set_text(title, "ESP32 MARAUDER v" MARAUDER_VERSION);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    
    // Create grid for main menu buttons
    static lv_coord_t col_dsc[] = {200, 200, 200, 200, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {120, 120, 120, 120, LV_GRID_TEMPLATE_LAST};
    
    lv_obj_t *grid = lv_obj_create(screen_main);
    lv_obj_set_size(grid, 800, 400);
    lv_obj_center(grid);
    lv_obj_set_grid_dsc_array(grid, col_dsc, row_dsc);
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_opa(grid, LV_OPA_TRANSP, 0);
    
    // Create menu buttons
    const char *btn_labels[] = {
        LV_SYMBOL_WIFI " WiFi\nScan",
        LV_SYMBOL_BLUETOOTH " BLE\nScan", 
        LV_SYMBOL_WARNING " Attack\nMenu",
        LV_SYMBOL_EYE_OPEN " Packet\nSniffer",
        LV_SYMBOL_DOWNLOAD " Evil\nPortal",
        LV_SYMBOL_GPS " GPS\nLogger",
        LV_SYMBOL_SD_CARD " File\nManager",
        LV_SYMBOL_SETTINGS " Settings"
    };
    
    for (int i = 0; i < 8; i++) {
        lv_obj_t *btn = lv_btn_create(grid);
        lv_obj_set_grid_cell(btn, LV_GRID_ALIGN_STRETCH, i % 4, 1,
                             LV_GRID_ALIGN_STRETCH, i / 4, 1);
        
        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, btn_labels[i]);
        lv_obj_center(label);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        
        // Add event handler
        lv_obj_add_event_cb(btn, main_menu_event_cb, LV_EVENT_CLICKED, (void*)i);
    }
    
    // Create status bar at bottom
    createStatusBar(screen_main);
}

void MarauderMenu::createStatusBar(lv_obj_t *parent) {
    lv_obj_t *status_bar = lv_obj_create(parent);
    lv_obj_set_size(status_bar, 800, 40);
    lv_obj_align(status_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    
    // WiFi status
    lv_obj_t *wifi_status = lv_label_create(status_bar);
    lv_label_set_text(wifi_status, LV_SYMBOL_WIFI " Ready");
    lv_obj_align(wifi_status, LV_ALIGN_LEFT_MID, 10, 0);
    
    // SD card status
    lv_obj_t *sd_status = lv_label_create(status_bar);
    lv_label_set_text(sd_status, LV_SYMBOL_SD_CARD " 32GB");
    lv_obj_align(sd_status, LV_ALIGN_CENTER, -100, 0);
    
    // GPS status
    lv_obj_t *gps_status = lv_label_create(status_bar);
    lv_label_set_text(gps_status, LV_SYMBOL_GPS " No Fix");
    lv_obj_align(gps_status, LV_ALIGN_CENTER, 100, 0);
    
    // Battery/Power
    lv_obj_t *power_status = lv_label_create(status_bar);
    lv_label_set_text(power_status, LV_SYMBOL_USB " USB");
    lv_obj_align(power_status, LV_ALIGN_RIGHT_MID, -10, 0);
}

void MarauderMenu::createScanScreen() {
    screen_scan = lv_obj_create(NULL);
    
    // Title
    lv_obj_t *title = lv_label_create(screen_scan);
    lv_label_set_text(title, "WiFi Scanner");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    
    // Back button
    lv_obj_t *btn_back = lv_btn_create(screen_scan);
    lv_obj_set_size(btn_back, 100, 40);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *label_back = lv_label_create(btn_back);
    lv_label_set_text(label_back, LV_SYMBOL_LEFT " Back");
    lv_obj_center(label_back);
    lv_obj_add_event_cb(btn_back, back_event_cb, LV_EVENT_CLICKED, NULL);
    
    // Scan button
    lv_obj_t *btn_scan = lv_btn_create(screen_scan);
    lv_obj_set_size(btn_scan, 100, 40);
    lv_obj_align(btn_scan, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_t *label_scan = lv_label_create(btn_scan);
    lv_label_set_text(label_scan, "Scan " LV_SYMBOL_REFRESH);
    lv_obj_center(label_scan);
    lv_obj_add_event_cb(btn_scan, scan_event_cb, LV_EVENT_CLICKED, NULL);
    
    // Create table for scan results
    lv_obj_t *table = lv_table_create(screen_scan);
    lv_obj_set_size(table, 780, 360);
    lv_obj_align(table, LV_ALIGN_CENTER, 0, 20);
    
    // Set table columns
    lv_table_set_col_cnt(table, 5);
    lv_table_set_col_width(table, 0, 250);  // SSID
    lv_table_set_col_width(table, 1, 150);  // BSSID
    lv_table_set_col_width(table, 2, 80);   // Channel
    lv_table_set_col_width(table, 3, 80);   // RSSI
    lv_table_set_col_width(table, 4, 100);  // Security
    
    // Set headers
    lv_table_set_cell_value(table, 0, 0, "SSID");
    lv_table_set_cell_value(table, 0, 1, "BSSID");
    lv_table_set_cell_value(table, 0, 2, "Ch");
    lv_table_set_cell_value(table, 0, 3, "RSSI");
    lv_table_set_cell_value(table, 0, 4, "Security");
    
    // Status bar
    createStatusBar(screen_scan);
}

void MarauderMenu::createAttackScreen() {
    screen_attack = lv_obj_create(NULL);
    
    // Title
    lv_obj_t *title = lv_label_create(screen_attack);
    lv_label_set_text(title, "Attack Menu");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    
    // Back button
    lv_obj_t *btn_back = lv_btn_create(screen_attack);
    lv_obj_set_size(btn_back, 100, 40);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *label_back = lv_label_create(btn_back);
    lv_label_set_text(label_back, LV_SYMBOL_LEFT " Back");
    lv_obj_center(label_back);
    lv_obj_add_event_cb(btn_back, back_event_cb, LV_EVENT_CLICKED, NULL);
    
    // Attack options grid
    static lv_coord_t col_dsc[] = {190, 190, 190, 190, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {80, 80, 80, 80, LV_GRID_TEMPLATE_LAST};
    
    lv_obj_t *grid = lv_obj_create(screen_attack);
    lv_obj_set_size(grid, 780, 340);
    lv_obj_align(grid, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_grid_dsc_array(grid, col_dsc, row_dsc);
    
    const char *attack_labels[] = {
        "Deauth\nAttack",
        "Beacon\nSpam",
        "Probe\nRequest",
        "Rick\nRoll",
        "Random\nSSID",
        "AP\nClone",
        "Karma\nAttack",
        "Loud\nBluetooth",
        "BLE\nSpam",
        "SwiftPair\nSpam",
        "Samsung\nSpam",
        "Stop\nAll"
    };
    
    for (int i = 0; i < 12; i++) {
        lv_obj_t *btn = lv_btn_create(grid);
        lv_obj_set_grid_cell(btn, LV_GRID_ALIGN_STRETCH, i % 4, 1,
                             LV_GRID_ALIGN_STRETCH, i / 4, 1);
        
        if (i == 11) {  // Stop button - make it red
            lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), 0);
        }
        
        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, attack_labels[i]);
        lv_obj_center(label);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        
        lv_obj_add_event_cb(btn, attack_event_cb, LV_EVENT_CLICKED, (void*)i);
    }
    
    createStatusBar(screen_attack);
}

void MarauderMenu::createSniffScreen() {
    screen_sniff = lv_obj_create(NULL);
    
    // Implementation similar to scan screen
    // Add packet sniffer interface
}

void MarauderMenu::createSettingsScreen() {
    screen_settings = lv_obj_create(NULL);
    
    // Title
    lv_obj_t *title = lv_label_create(screen_settings);
    lv_label_set_text(title, "Settings");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    
    // Back button
    lv_obj_t *btn_back = lv_btn_create(screen_settings);
    lv_obj_set_size(btn_back, 100, 40);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *label_back = lv_label_create(btn_back);
    lv_label_set_text(label_back, LV_SYMBOL_LEFT " Back");
    lv_obj_center(label_back);
    lv_obj_add_event_cb(btn_back, back_event_cb, LV_EVENT_CLICKED, NULL);
    
    // Settings list
    lv_obj_t *list = lv_list_create(screen_settings);
    lv_obj_set_size(list, 780, 360);
    lv_obj_align(list, LV_ALIGN_CENTER, 0, 20);
    
    // Add settings items
    lv_list_add_btn(list, LV_SYMBOL_WIFI, "WiFi Settings");
    lv_list_add_btn(list, LV_SYMBOL_BLUETOOTH, "Bluetooth Settings");
    lv_list_add_btn(list, LV_SYMBOL_IMAGE, "Display Settings");
    lv_list_add_btn(list, LV_SYMBOL_SD_CARD, "Storage Settings");
    lv_list_add_btn(list, LV_SYMBOL_GPS, "GPS Settings");
    lv_list_add_btn(list, LV_SYMBOL_POWER, "Power Settings");
    lv_list_add_btn(list, LV_SYMBOL_SETTINGS, "Advanced Settings");
    lv_list_add_btn(list, LV_SYMBOL_INFO, "About");
    
    createStatusBar(screen_settings);
}

// Event callbacks
void MarauderMenu::main_menu_event_cb(lv_event_t *e) {
    int btn_id = (int)lv_event_get_user_data(e);
    
    switch(btn_id) {
        case 0: // WiFi Scan
            lv_scr_load(screen_scan);
            break;
        case 1: // BLE Scan
            // Load BLE screen
            break;
        case 2: // Attack Menu
            lv_scr_load(screen_attack);
            break;
        case 3: // Packet Sniffer
            lv_scr_load(screen_sniff);
            break;
        case 4: // Evil Portal
            // Load Evil Portal screen
            break;
        case 5: // GPS Logger
            // Load GPS screen
            break;
        case 6: // File Manager
            // Load file manager
            break;
        case 7: // Settings
            lv_scr_load(screen_settings);
            break;
    }
}

void MarauderMenu::back_event_cb(lv_event_t *e) {
    lv_scr_load(screen_main);
}

void MarauderMenu::scan_event_cb(lv_event_t *e) {
    // Trigger WiFi scan
    // This will send a message to the WiFi task
}

void MarauderMenu::attack_event_cb(lv_event_t *e) {
    int attack_id = (int)lv_event_get_user_data(e);
    
    // Handle different attacks based on ID
    switch(attack_id) {
        case 0: // Deauth
            // Start deauth attack
            break;
        case 1: // Beacon Spam
            // Start beacon spam
            break;
        // ... other attacks
        case 11: // Stop All
            // Stop all attacks
            break;
    }
}
Phase 4: Build System
File 10: .github/workflows/build.yml
yamlname: Build Firmware

on:
  push:
    branches: [ main, develop ]
    tags:
      - 'v*'
  pull_request:
    branches: [ main ]
  workflow_dispatch:

jobs:
  build:
    runs-on: ubuntu-latest
    
    steps:
    - uses: actions/checkout@v3
      with:
        submodules: recursive
    
    - name: Cache PlatformIO
      uses: actions/cache@v3
      with:
        path: |
          ~/.platformio
          .pio
        key: ${{ runner.os }}-pio-${{ hashFiles('**/platformio.ini') }}
    
    - name: Setup Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.10'
    
    - name: Install PlatformIO
      run: |
        python -m pip install --upgrade pip
        pip install platformio
        pio upgrade
    
    - name: Build Firmware
      run: |
        pio run -e waveshare_7inch
    
    - name: Generate Build Info
      run: |
        echo "Build Date: $(date)" > build_info.txt
        echo "Git Commit: ${{ github.sha }}" >> build_info.txt
        echo "Git Branch: ${{ github.ref }}" >> build_info.txt
        pio run --target size -e waveshare_7inch | tail -10 >> build_info.txt
    
    - name: Upload Firmware
      uses: actions/upload-artifact@v3
      with:
        name: firmware
        path: |
          .pio/build/waveshare_7inch/firmware.bin
          .pio/build/waveshare_7inch/partitions.bin
          .pio/build/waveshare_7inch/bootloader.bin
          build_info.txt
    
    - name: Create Release
      if: startsWith(github.ref, 'refs/tags/')
      uses: softprops/action-gh-release@v1
      with:
        files: |
          .pio/build/waveshare_7inch/firmware.bin
          build_info.txt
        generate_release_notes: true
File 11: scripts/flash.sh
bash#!/bin/bash
# Flash script for Waveshare 7-inch Marauder

PORT=${1:-/dev/ttyUSB0}
BAUD=921600

echo "ESP32 Marauder - Waveshare 7-inch Flash Tool"
echo "============================================="
echo "Port: $PORT"
echo ""

# Check if port exists
if [ ! -e "$PORT" ]; then
    echo "Error: Port $PORT not found!"
    echo "Usage: ./flash.sh [port]"
    exit 1
fi

# Check for firmware files
if [ ! -f ".pio/build/waveshare_7inch/firmware.bin" ]; then
    echo "Building firmware..."
    pio run -e waveshare_7inch
fi

echo "Flashing bootloader..."
esptool.py --chip esp32s3 --port $PORT --baud $BAUD \
    --before default_reset --after hard_reset write_flash \
    -z --flash_mode dio --flash_freq 80m --flash_size 16MB \
    0x0 .pio/build/waveshare_7inch/bootloader.bin

echo "Flashing partition table..."
esptool.py --chip esp32s3 --port $PORT --baud $BAUD \
    --before default_reset --after hard_reset write_flash \
    -z --flash_mode dio --flash_freq 80m --flash_size 16MB \
    0x8000 .pio/build/waveshare_7inch/partitions.bin

echo "Flashing firmware..."
esptool.py --chip esp32s3 --port $PORT --baud $BAUD \
    --before default_reset --after hard_reset write_flash \
    -z --flash_mode dio --flash_freq 80m --flash_size 16MB \
    0x10000 .pio/build/waveshare_7inch/firmware.bin

echo ""
echo "Flash complete! Opening serial monitor..."
pio device monitor -p $PORT -b 115200
Phase 5: Clone and Adapt Marauder Code
Commands to execute:
bash# Clone original Marauder source
cd lib
git clone https://github.com/justcallmekoko/ESP32Marauder marauder_original
cd marauder_original

# Copy needed files to src
cp esp32_marauder/*.cpp ../../src/
cp esp32_marauder/*.h ../../include/

# Files to modify for 7-inch display:
# - Display.cpp/Display.h (replace TFT_eSPI with RGB driver)
# - MenuFunctions.cpp (adapt for 800x480 screen)
# - settings.h (add WAVESHARE_7_INCH configuration)
# - pins.h (use our pin definitions)

# Clone LVGL
cd ..
git clone --branch release/v9.2 https://github.com/lvgl/lvgl

# Clone ESP32_Display_Panel
git clone https://github.com/esp-arduino-libs/ESP32_Display_Panel

# Clone GT911 driver
git clone https://github.com/worthylyx/GT911
Phase 6: Testing Protocol
File 12: test/test_main.cpp
cpp#include <unity.h>
#include "Display.h"
#include "TouchDriver.h"
#include "pins.h"

void test_display_init() {
    Display display;
    TEST_ASSERT_TRUE(display.begin());
}

void test_touch_init() {
    TouchDriver touch;
    TEST_ASSERT_TRUE(touch.begin());
}

void test_memory_allocation() {
    size_t free_heap = esp_get_free_heap_size();
    TEST_ASSERT_GREATER_THAN(100000, free_heap);
    
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    TEST_ASSERT_GREATER_THAN(1000000, psram_free);
}

void test_wifi_init() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    TEST_ASSERT_EQUAL(ESP_OK, esp_wifi_init(&cfg));
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_display_init);
    RUN_TEST(test_touch_init);
    RUN_TEST(test_memory_allocation);
    RUN_TEST(test_wifi_init);
    UNITY_END();
}

void loop() {
}
Final Instructions for Claude Code

Start by creating all the files listed above in the repository
Install required dependencies using PlatformIO
Clone the reference repositories into lib/ folder
Modify Marauder source files for 7-inch display compatibility
Implement the RGB display driver using ESP32_Display_Panel
Test each component individually before integration
Build the complete firmware using: pio run -e waveshare_7inch
Document any issues or modifications in CHANGELOG.md

Critical Success Factors

Display Driver: The RGB interface requires precise timing. Use ESP32-S3's LCD_CAM peripheral.
Memory Management: Allocate display buffers in PSRAM, keep WiFi buffers in DRAM
Task Separation: Keep WiFi on Core 0, GUI on Core 1
Touch Calibration: GT911 may need calibration stored in NVS
Power Management: 7-inch display draws ~500mA, implement brightness control

Repository Maintenance
After implementation:

Create releases using semantic versioning (v1.0.0)
Update README with screenshots and demo videos
Create GitHub Wiki for user documentation
Set up Discord or discussions for community support
Consider web flasher using ESP Web Tools

This completes the comprehensive implementation guide. Save this as IMPLEMENTATION_GUIDE.md in your repository and Claude Code can use it to build the entire project systematically.
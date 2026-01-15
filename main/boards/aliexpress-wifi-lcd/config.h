#pragma once

// --- Display Configuration (ILI9341) ---
#define LCD_DRIVER_ILI9341 1
#define LCD_WIDTH 240
#define LCD_HEIGHT 320
#define LCD_MOSI_GPIO 11  // SPI write data signal
#define LCD_MISO_GPIO 13  // SPI read data signal
#define LCD_CLK_GPIO 12   // SPI clock signal
#define LCD_CS_GPIO 10    // Chip select (active low)
#define LCD_DC_GPIO 46    // Command/Data select (high=data, low=command)
#define LCD_RST_GPIO -1   // Shares EN reset button with ESP32-S3
#define LCD_BL_GPIO 45    // Backlight control (high=on, low=off)
#define LCD_BL_ON_LEVEL 1

// Display orientation and rotation
#define LCD_ROTATION 0  // Adjust as needed: 0, 1, 2, or 3 for 0°, 90°, 180°, 270°

// --- Touch Configuration (FT6336) ---
#define TOUCH_DRIVER_FT6336 1
#define TOUCH_I2C_NUM I2C_NUM_0
#define TOUCH_I2C_SDA_GPIO 16
#define TOUCH_I2C_SCL_GPIO 15
#define TOUCH_I2C_FREQ_HZ 400000
#define TOUCH_RST_GPIO 18
#define TOUCH_INT_GPIO 17
#define TOUCH_WIDTH LCD_WIDTH
#define TOUCH_HEIGHT LCD_HEIGHT

// --- Audio Configuration (I2S) ---
// I2S configuration for speaker and microphone
#define AUDIO_I2S_NUM I2S_NUM_0
#define AUDIO_I2S_MCK_IO 4   // Master clock signal
#define AUDIO_I2S_BCK_IO 5   // Bit clock signal
#define AUDIO_I2S_WS_IO 7    // Left/right channel select (high=right, low=left)
#define AUDIO_I2S_DO_IO 6    // Bit output data signal (speaker)
#define AUDIO_I2S_DI_IO 8    // Bit input data signal (microphone)

// Audio sample rates
#define AUDIO_OUTPUT_SAMPLE_RATE 16000  // 16kHz for TTS playback
#define AUDIO_INPUT_SAMPLE_RATE 16000   // 16kHz for voice recording

// --- Audio Amplifier Enable ---
// Active low enable (low=enable, high=disable)
#define AUDIO_PA_ENABLE_GPIO 1
#define AUDIO_PA_ENABLE_LEVEL 0

// --- SD Card Configuration (Optional) ---
#define SD_CARD_ENABLED 1
#define SD_MODE_SDMMC 1  // Using SDMMC interface (4-bit mode)
#define SD_CLK_GPIO 38   // SDIO clock signal
#define SD_CMD_GPIO 40   // SDIO command signal
#define SD_D0_GPIO 39    // SDIO data line 0
#define SD_D1_GPIO 41    // SDIO data line 1
#define SD_D2_GPIO 48    // SDIO data line 2
#define SD_D3_GPIO 47    // SDIO data line 3

// --- Button Configuration ---
#define BUTTON_BOOT_GPIO 0  // Download mode key (hold to enter download mode)
// EN button is the reset button, shared with LCD reset

// --- Status LED Configuration ---
#define LED_RGB_GPIO 42     // Single-line RGB LED (WS2812 or similar)
#define LED_TYPE_RGB 1

// --- Battery Monitoring ---
#define BATTERY_ADC_GPIO 9  // Battery voltage ADC input

// --- Expansion Pins (Available for general use) ---
#define EXPAND_IO_1 2
#define EXPAND_IO_2 3
#define EXPAND_IO_3 14
#define EXPAND_IO_4 21

// --- WiFi Configuration ---
// These are typically configured at runtime, but you can set defaults
// #define DEFAULT_WIFI_SSID "YourSSID"
// #define DEFAULT_WIFI_PASSWORD "YourPassword"

// --- Power Management ---
#define ENABLE_POWER_SAVE 0  // Disable power save mode for stable audio

// --- Memory Configuration ---
// ESP32-S3 specific memory settings
#define PSRAM_ENABLED 1  // Enable if your board has PSRAM
#define PSRAM_MODE_OCTAL 0  // Set to 1 if using octal PSRAM

// --- Debug Configuration ---
// #define DEBUG_ENABLED 1
// #define DEBUG_UART_NUM UART_NUM_0
// #define DEBUG_BAUD_RATE 115200
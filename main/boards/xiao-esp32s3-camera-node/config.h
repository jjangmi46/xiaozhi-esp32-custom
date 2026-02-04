#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// This board is a camera-only node that sends images via ESP-NOW
// It does not have audio or display - those are handled by the main board

// XIAO ESP32S3 Sense Camera Pins
#define CAMERA_PIN_D0    GPIO_NUM_15
#define CAMERA_PIN_D1    GPIO_NUM_17
#define CAMERA_PIN_D2    GPIO_NUM_18
#define CAMERA_PIN_D3    GPIO_NUM_16
#define CAMERA_PIN_D4    GPIO_NUM_14
#define CAMERA_PIN_D5    GPIO_NUM_12
#define CAMERA_PIN_D6    GPIO_NUM_11
#define CAMERA_PIN_D7    GPIO_NUM_48
#define CAMERA_PIN_XCLK  GPIO_NUM_10
#define CAMERA_PIN_PCLK  GPIO_NUM_13
#define CAMERA_PIN_VSYNC GPIO_NUM_38
#define CAMERA_PIN_HREF  GPIO_NUM_47
#define CAMERA_PIN_SIOC  GPIO_NUM_39
#define CAMERA_PIN_SIOD  GPIO_NUM_40
#define CAMERA_PIN_PWDN  GPIO_NUM_NC
#define CAMERA_PIN_RESET GPIO_NUM_NC
#define XCLK_FREQ_HZ     20000000

#define BUILTIN_LED_GPIO GPIO_NUM_21
#define BOOT_BUTTON_GPIO GPIO_NUM_0

// Dummy audio settings (required by framework but not used)
#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 16000

#endif // _BOARD_CONFIG_H_

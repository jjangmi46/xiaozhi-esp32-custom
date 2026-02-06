#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// XIAO ESP32S3 Camera Node
// This board acts as a camera-only peripheral that receives UART commands
// and uploads images directly to the cloud

// Audio - minimal config (no speaker on camera node)
#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// No audio on camera node - using NoAudioCodec
// PDM Microphone pins (not used but defined for compilation)
#define AUDIO_I2S_PDM_MIC
#define AUDIO_I2S_MIC_GPIO_CLK  GPIO_NUM_42
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_41

// LED and Button
#define BUILTIN_LED_GPIO        GPIO_NUM_21
#define BOOT_BUTTON_GPIO        GPIO_NUM_0

// XIAO ESP32S3 Sense Camera Pins (OV2640/OV3660)
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

// UART Bridge (communication with Freenove master)
// Using free GPIO pins (GPIO 43/44 conflict with USB console!)
// GPIO3 = TX (to Freenove RX/GPIO21)
// GPIO4 = RX (from Freenove TX/GPIO2)
#define UART_BRIDGE_PORT      UART_NUM_2
#define UART_BRIDGE_TX_PIN    GPIO_NUM_3   // TX
#define UART_BRIDGE_RX_PIN    GPIO_NUM_4   // RX
#define UART_BRIDGE_BAUD      115200

#endif

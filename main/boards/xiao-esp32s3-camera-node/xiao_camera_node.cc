// XIAO ESP32S3 Sense Camera Node
// This board acts as a remote camera that sends images via ESP-NOW
// It pairs with boards like Freenove that have display but no camera

#include <cstring>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_heap_caps.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "wifi_board.h"
#include "wifi_station.h"
#include "codecs/no_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "led/single_led.h"
#include "esp32_camera.h"
#include "jpg/image_to_jpeg.h"

#include "esp_video_init.h"

#define TAG "XiaoCameraNode"

// ESP-NOW Protocol (must match espnow_camera.h)
#define ESPNOW_CAMERA_CMD_CAPTURE    0x01
#define ESPNOW_CAMERA_CMD_ACK        0x02
#define ESPNOW_CAMERA_CMD_DATA       0x03
#define ESPNOW_CAMERA_CMD_END        0x04
#define ESPNOW_CAMERA_CMD_ERROR      0x05
#define ESPNOW_CAMERA_MAX_DATA_LEN   230

struct EspNowCameraPacket {
    uint8_t cmd;
    uint16_t seq;
    uint16_t total_packets;
    uint32_t total_size;
    uint8_t data[ESPNOW_CAMERA_MAX_DATA_LEN];
} __attribute__((packed));

class XiaoCameraNode : public WifiBoard {
private:
    Button boot_button_;
    Esp32Camera* camera_ = nullptr;

    uint8_t peer_mac_[6] = {0};
    bool peer_registered_ = false;

    // Task handle for processing capture requests
    TaskHandle_t capture_task_ = nullptr;
    QueueHandle_t request_queue_ = nullptr;

    void InitializeCamera() {
        ESP_LOGI(TAG, "Initializing camera");

        static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                [0] = CAMERA_PIN_D0,
                [1] = CAMERA_PIN_D1,
                [2] = CAMERA_PIN_D2,
                [3] = CAMERA_PIN_D3,
                [4] = CAMERA_PIN_D4,
                [5] = CAMERA_PIN_D5,
                [6] = CAMERA_PIN_D6,
                [7] = CAMERA_PIN_D7,
            },
            .vsync_io = CAMERA_PIN_VSYNC,
            .de_io = CAMERA_PIN_HREF,
            .pclk_io = CAMERA_PIN_PCLK,
            .xclk_io = CAMERA_PIN_XCLK,
        };

        esp_video_init_sccb_config_t sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = 0,
                .scl_pin = CAMERA_PIN_SIOC,
                .sda_pin = CAMERA_PIN_SIOD,
            },
            .freq = 100000,
        };

        esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = sccb_config,
            .reset_pin = CAMERA_PIN_RESET,
            .pwdn_pin = CAMERA_PIN_PWDN,
            .dvp_pin = dvp_pin_config,
            .xclk_freq = XCLK_FREQ_HZ,
        };

        esp_video_init_config_t video_config = {
            .dvp = &dvp_config,
        };

        camera_ = new Esp32Camera(video_config);
    }

    void InitializeEspNow() {
        ESP_LOGI(TAG, "Initializing ESP-NOW");

        esp_err_t ret = esp_now_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(ret));
            return;
        }

        ret = esp_now_register_recv_cb(OnEspNowReceive);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_now_register_recv_cb failed: %s", esp_err_to_name(ret));
            return;
        }

        ESP_LOGI(TAG, "ESP-NOW initialized, waiting for capture requests...");
    }

    static void OnEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
        auto* board = static_cast<XiaoCameraNode*>(&Board::GetInstance());
        board->HandleEspNowReceive(info->src_addr, data, len);
    }

    void HandleEspNowReceive(const uint8_t* mac, const uint8_t* data, int len) {
        if (len < 1) return;

        uint8_t cmd = data[0];

        if (cmd == ESPNOW_CAMERA_CMD_CAPTURE) {
            ESP_LOGI(TAG, "Received capture request from %02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

            // Register peer if not already done
            if (!peer_registered_ || memcmp(peer_mac_, mac, 6) != 0) {
                if (peer_registered_) {
                    esp_now_del_peer(peer_mac_);
                }
                memcpy(peer_mac_, mac, 6);

                esp_now_peer_info_t peer_info = {};
                memcpy(peer_info.peer_addr, mac, 6);
                peer_info.channel = 0;
                peer_info.encrypt = false;

                esp_err_t ret = esp_now_add_peer(&peer_info);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
                    return;
                }
                peer_registered_ = true;
            }

            // Queue capture request
            uint8_t req = 1;
            xQueueSend(request_queue_, &req, 0);
        }
    }

    static void CaptureTaskFunc(void* arg) {
        auto* board = static_cast<XiaoCameraNode*>(arg);
        board->CaptureTaskLoop();
    }

    void CaptureTaskLoop() {
        uint8_t req;
        while (true) {
            if (xQueueReceive(request_queue_, &req, portMAX_DELAY) == pdTRUE) {
                ProcessCaptureRequest();
            }
        }
    }

    void ProcessCaptureRequest() {
        ESP_LOGI(TAG, "Processing capture request...");

        // Capture image
        if (!camera_ || !camera_->Capture()) {
            ESP_LOGE(TAG, "Camera capture failed");
            SendError();
            return;
        }

        // Encode to JPEG and send
        SendCapturedImage();
    }

    void SendCapturedImage() {
        // Get frame data from camera
        const uint8_t* frame_data = camera_->GetFrameData();
        size_t frame_size = camera_->GetFrameSize();
        uint16_t width = camera_->GetFrameWidth();
        uint16_t height = camera_->GetFrameHeight();
        v4l2_pix_fmt_t format = camera_->GetFrameFormat();

        if (!frame_data || frame_size == 0) {
            ESP_LOGE(TAG, "No frame data available");
            SendError();
            return;
        }

        ESP_LOGI(TAG, "Frame captured: %dx%d, %zu bytes, format=0x%lx", width, height, frame_size, format);

        // Allocate buffer for JPEG data
        size_t jpeg_capacity = 64 * 1024;
        uint8_t* jpeg_buffer = (uint8_t*)heap_caps_malloc(jpeg_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!jpeg_buffer) {
            ESP_LOGE(TAG, "Failed to allocate JPEG buffer");
            SendError();
            return;
        }

        // Encode to JPEG
        struct JpegContext {
            uint8_t* buffer;
            size_t capacity;
            size_t size;
        } ctx = { jpeg_buffer, jpeg_capacity, 0 };

        bool encode_ok = image_to_jpeg_cb(
            const_cast<uint8_t*>(frame_data), frame_size, width, height, format, 80,
            [](void* arg, size_t index, const void* data, size_t len) -> size_t {
                auto* ctx = static_cast<JpegContext*>(arg);
                if (index == 0 && data != nullptr && len > 0) {
                    if (len <= ctx->capacity) {
                        memcpy(ctx->buffer, data, len);
                        ctx->size = len;
                        return len;
                    }
                }
                return 0;
            },
            &ctx
        );

        if (!encode_ok || ctx.size == 0) {
            ESP_LOGE(TAG, "JPEG encoding failed");
            heap_caps_free(jpeg_buffer);
            SendError();
            return;
        }

        ESP_LOGI(TAG, "JPEG encoded: %zu bytes", ctx.size);

        // Calculate number of packets needed
        uint16_t total_packets = (ctx.size + ESPNOW_CAMERA_MAX_DATA_LEN - 1) / ESPNOW_CAMERA_MAX_DATA_LEN;

        // Send ACK with image info
        EspNowCameraPacket ack_packet = {};
        ack_packet.cmd = ESPNOW_CAMERA_CMD_ACK;
        ack_packet.total_packets = total_packets;
        ack_packet.total_size = ctx.size;
        ack_packet.data[0] = width & 0xFF;
        ack_packet.data[1] = (width >> 8) & 0xFF;
        ack_packet.data[2] = height & 0xFF;
        ack_packet.data[3] = (height >> 8) & 0xFF;

        esp_err_t ret = esp_now_send(peer_mac_, (uint8_t*)&ack_packet,
                                      sizeof(EspNowCameraPacket) - ESPNOW_CAMERA_MAX_DATA_LEN + 4);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send ACK: %s", esp_err_to_name(ret));
            heap_caps_free(jpeg_buffer);
            SendError();
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay for receiver to process ACK

        // Send image data in chunks
        size_t offset = 0;
        uint16_t seq = 0;

        while (offset < ctx.size) {
            size_t chunk_size = ctx.size - offset;
            if (chunk_size > ESPNOW_CAMERA_MAX_DATA_LEN) {
                chunk_size = ESPNOW_CAMERA_MAX_DATA_LEN;
            }

            EspNowCameraPacket data_packet = {};
            data_packet.cmd = ESPNOW_CAMERA_CMD_DATA;
            data_packet.seq = seq;
            data_packet.total_packets = total_packets;
            data_packet.total_size = ctx.size;
            memcpy(data_packet.data, jpeg_buffer + offset, chunk_size);

            size_t packet_size = sizeof(EspNowCameraPacket) - ESPNOW_CAMERA_MAX_DATA_LEN + chunk_size;

            ret = esp_now_send(peer_mac_, (uint8_t*)&data_packet, packet_size);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send data packet %d: %s", seq, esp_err_to_name(ret));
            }

            offset += chunk_size;
            seq++;

            // Small delay between packets to avoid overwhelming the receiver
            vTaskDelay(pdMS_TO_TICKS(2));

            if (seq % 50 == 0) {
                ESP_LOGI(TAG, "Sent %d/%d packets", seq, total_packets);
            }
        }

        heap_caps_free(jpeg_buffer);

        // Send END packet
        vTaskDelay(pdMS_TO_TICKS(10));
        SendEnd();

        ESP_LOGI(TAG, "Image transfer complete: %d packets sent", seq);
    }

    void SendError() {
        if (!peer_registered_) return;

        EspNowCameraPacket packet = {};
        packet.cmd = ESPNOW_CAMERA_CMD_ERROR;
        esp_now_send(peer_mac_, (uint8_t*)&packet, sizeof(packet.cmd));
    }

    void SendEnd() {
        if (!peer_registered_) return;

        EspNowCameraPacket packet = {};
        packet.cmd = ESPNOW_CAMERA_CMD_END;
        esp_now_send(peer_mac_, (uint8_t*)&packet, sizeof(packet.cmd));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting &&
                !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
        });
    }

public:
    XiaoCameraNode() : boot_button_(BOOT_BUTTON_GPIO) {
        ESP_LOGI(TAG, "Initializing XIAO ESP32S3 Camera Node");

        // Create request queue
        request_queue_ = xQueueCreate(4, sizeof(uint8_t));

        InitializeButtons();
        InitializeCamera();

        // Note: ESP-NOW will be initialized after WiFi starts in StartNetwork()

        ESP_LOGI(TAG, "Camera node initialized (ESP-NOW will start after WiFi)");
    }

    virtual void StartNetwork() override {
        // First start WiFi (required for ESP-NOW)
        WifiBoard::StartNetwork();

        // Now initialize ESP-NOW
        InitializeEspNow();

        // Create capture task
        xTaskCreate(CaptureTaskFunc, "CaptureTask", 8192, this, 5, &capture_task_);

        ESP_LOGI(TAG, "Camera node ready");
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        // No audio on camera node
        static NoAudioCodec audio_codec;
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        // No display on camera node
        return nullptr;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(XiaoCameraNode);

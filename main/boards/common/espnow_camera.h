#pragma once

#include <string>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_now.h>
#include <esp_wifi_types.h>

#include "camera.h"

// ESP-NOW Camera Protocol
// Request: 1 byte command
// Response: Multiple packets with image data

#define ESPNOW_CAMERA_CMD_CAPTURE    0x01
#define ESPNOW_CAMERA_CMD_ACK        0x02
#define ESPNOW_CAMERA_CMD_DATA       0x03
#define ESPNOW_CAMERA_CMD_END        0x04
#define ESPNOW_CAMERA_CMD_ERROR      0x05

#define ESPNOW_CAMERA_MAX_DATA_LEN   230  // ESP-NOW max payload is 250, leave room for header

struct EspNowCameraPacket {
    uint8_t cmd;
    uint16_t seq;
    uint16_t total_packets;
    uint32_t total_size;
    uint8_t data[ESPNOW_CAMERA_MAX_DATA_LEN];
} __attribute__((packed));

class EspNowCamera : public Camera {
private:
    uint8_t peer_mac_[6];
    bool initialized_ = false;

    // Image buffer
    uint8_t* image_data_ = nullptr;
    size_t image_size_ = 0;
    size_t image_capacity_ = 0;
    uint16_t image_width_ = 0;
    uint16_t image_height_ = 0;

    // Synchronization
    SemaphoreHandle_t capture_semaphore_ = nullptr;
    std::atomic<bool> capture_in_progress_{false};
    std::atomic<uint16_t> packets_received_{0};
    std::atomic<uint16_t> total_packets_{0};

    // Explain URL
    std::string explain_url_;
    std::string explain_token_;

    static EspNowCamera* instance_;

    static void OnDataReceived(const esp_now_recv_info_t* info, const uint8_t* data, int len);
    static void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);

    void HandleReceivedData(const uint8_t* mac, const uint8_t* data, int len);
    bool SendCaptureRequest();

public:
    // peer_mac: MAC address of XIAO camera node (6 bytes)
    EspNowCamera(const uint8_t* peer_mac);
    ~EspNowCamera();

    bool Initialize();

    virtual void SetExplainUrl(const std::string& url, const std::string& token) override;
    virtual bool Capture() override;
    virtual bool SetHMirror(bool enabled) override;
    virtual bool SetVFlip(bool enabled) override;
    virtual std::string Explain(const std::string& question) override;

    // Get raw image data for display
    const uint8_t* GetImageData() const { return image_data_; }
    size_t GetImageSize() const { return image_size_; }
    uint16_t GetImageWidth() const { return image_width_; }
    uint16_t GetImageHeight() const { return image_height_; }
};

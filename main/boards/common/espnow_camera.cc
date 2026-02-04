#include "espnow_camera.h"

#include <cstring>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <nvs_flash.h>

#include "board.h"
#include "display.h"
#include "lvgl_display.h"
#include "system_info.h"

#define TAG "EspNowCamera"

#define CAPTURE_TIMEOUT_MS  10000  // 10 second timeout for capture
#define MAX_IMAGE_SIZE      (320 * 240 * 2)  // Max image size (QVGA RGB565)

EspNowCamera* EspNowCamera::instance_ = nullptr;

EspNowCamera::EspNowCamera(const uint8_t* peer_mac) {
    memcpy(peer_mac_, peer_mac, 6);
    instance_ = this;
}

EspNowCamera::~EspNowCamera() {
    if (initialized_) {
        esp_now_unregister_recv_cb();
        esp_now_unregister_send_cb();
        esp_now_del_peer(peer_mac_);
        esp_now_deinit();
    }
    if (image_data_) {
        heap_caps_free(image_data_);
        image_data_ = nullptr;
    }
    if (capture_semaphore_) {
        vSemaphoreDelete(capture_semaphore_);
        capture_semaphore_ = nullptr;
    }
    instance_ = nullptr;
}

bool EspNowCamera::Initialize() {
    if (initialized_) {
        return true;
    }

    // Create semaphore for capture synchronization
    capture_semaphore_ = xSemaphoreCreateBinary();
    if (!capture_semaphore_) {
        ESP_LOGE(TAG, "Failed to create capture semaphore");
        return false;
    }

    // Allocate image buffer
    image_capacity_ = MAX_IMAGE_SIZE;
    image_data_ = (uint8_t*)heap_caps_malloc(image_capacity_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!image_data_) {
        ESP_LOGE(TAG, "Failed to allocate image buffer");
        return false;
    }

    // Initialize ESP-NOW
    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Register callbacks
    ret = esp_now_register_recv_cb(OnDataReceived);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_register_recv_cb failed: %s", esp_err_to_name(ret));
        esp_now_deinit();
        return false;
    }

    ret = esp_now_register_send_cb(OnDataSent);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_register_send_cb failed: %s", esp_err_to_name(ret));
        esp_now_deinit();
        return false;
    }

    // Add peer
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_mac_, 6);
    peer_info.channel = 0;  // Use current channel
    peer_info.encrypt = false;

    ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(ret));
        esp_now_deinit();
        return false;
    }

    ESP_LOGI(TAG, "ESP-NOW Camera initialized, peer MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             peer_mac_[0], peer_mac_[1], peer_mac_[2], peer_mac_[3], peer_mac_[4], peer_mac_[5]);

    initialized_ = true;
    return true;
}

void EspNowCamera::OnDataReceived(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (instance_) {
        instance_->HandleReceivedData(info->src_addr, data, len);
    }
}

void EspNowCamera::OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "ESP-NOW send failed");
    }
}

void EspNowCamera::HandleReceivedData(const uint8_t* mac, const uint8_t* data, int len) {
    // Verify it's from our peer
    if (memcmp(mac, peer_mac_, 6) != 0) {
        return;
    }

    if (len < sizeof(EspNowCameraPacket) - ESPNOW_CAMERA_MAX_DATA_LEN) {
        ESP_LOGW(TAG, "Received packet too small: %d", len);
        return;
    }

    const EspNowCameraPacket* packet = (const EspNowCameraPacket*)data;

    switch (packet->cmd) {
        case ESPNOW_CAMERA_CMD_ACK: {
            // ACK contains image info: total_packets and total_size
            total_packets_ = packet->total_packets;
            image_size_ = 0;
            packets_received_ = 0;

            // Extract width and height from data field (first 4 bytes)
            if (len >= sizeof(EspNowCameraPacket) - ESPNOW_CAMERA_MAX_DATA_LEN + 4) {
                image_width_ = packet->data[0] | (packet->data[1] << 8);
                image_height_ = packet->data[2] | (packet->data[3] << 8);
            }

            ESP_LOGI(TAG, "Capture ACK: %d packets, %lu bytes total, %dx%d",
                     total_packets_.load(), packet->total_size, image_width_, image_height_);
            break;
        }

        case ESPNOW_CAMERA_CMD_DATA: {
            if (!capture_in_progress_) {
                return;
            }

            uint16_t seq = packet->seq;
            size_t data_len = len - (sizeof(EspNowCameraPacket) - ESPNOW_CAMERA_MAX_DATA_LEN);

            // Calculate offset based on sequence number
            size_t offset = seq * ESPNOW_CAMERA_MAX_DATA_LEN;
            if (offset + data_len > image_capacity_) {
                ESP_LOGW(TAG, "Image data exceeds buffer capacity");
                return;
            }

            memcpy(image_data_ + offset, packet->data, data_len);
            image_size_ = offset + data_len;
            packets_received_++;

            if (seq % 50 == 0) {
                ESP_LOGD(TAG, "Received packet %d/%d", seq, total_packets_.load());
            }
            break;
        }

        case ESPNOW_CAMERA_CMD_END: {
            ESP_LOGI(TAG, "Capture complete: received %d/%d packets, %zu bytes",
                     packets_received_.load(), total_packets_.load(), image_size_);
            capture_in_progress_ = false;
            xSemaphoreGive(capture_semaphore_);
            break;
        }

        case ESPNOW_CAMERA_CMD_ERROR: {
            ESP_LOGE(TAG, "Camera node reported error");
            capture_in_progress_ = false;
            xSemaphoreGive(capture_semaphore_);
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", packet->cmd);
            break;
    }
}

bool EspNowCamera::SendCaptureRequest() {
    EspNowCameraPacket packet = {};
    packet.cmd = ESPNOW_CAMERA_CMD_CAPTURE;

    esp_err_t ret = esp_now_send(peer_mac_, (uint8_t*)&packet, sizeof(packet.cmd));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send capture request: %s", esp_err_to_name(ret));
        return false;
    }
    return true;
}

void EspNowCamera::SetExplainUrl(const std::string& url, const std::string& token) {
    explain_url_ = url;
    explain_token_ = token;
}

bool EspNowCamera::Capture() {
    if (!initialized_) {
        ESP_LOGE(TAG, "Camera not initialized");
        return false;
    }

    if (capture_in_progress_) {
        ESP_LOGW(TAG, "Capture already in progress");
        return false;
    }

    // Reset state
    capture_in_progress_ = true;
    packets_received_ = 0;
    total_packets_ = 0;
    image_size_ = 0;

    // Clear any pending semaphore
    xSemaphoreTake(capture_semaphore_, 0);

    // Send capture request
    if (!SendCaptureRequest()) {
        capture_in_progress_ = false;
        return false;
    }

    ESP_LOGI(TAG, "Capture request sent, waiting for response...");

    // Wait for capture to complete
    if (xSemaphoreTake(capture_semaphore_, pdMS_TO_TICKS(CAPTURE_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Capture timeout");
        capture_in_progress_ = false;
        return false;
    }

    if (image_size_ == 0) {
        ESP_LOGE(TAG, "No image data received");
        return false;
    }

    // Display preview image
    auto display = dynamic_cast<LvglDisplay*>(Board::GetInstance().GetDisplay());
    if (display != nullptr && image_width_ > 0 && image_height_ > 0) {
        uint16_t w = image_width_;
        uint16_t h = image_height_;
        size_t stride = ((w * 2) + 3) & ~3;

        // Copy image data for LVGL (it will take ownership)
        uint8_t* preview_data = (uint8_t*)heap_caps_malloc(image_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (preview_data) {
            memcpy(preview_data, image_data_, image_size_);
            auto image = std::make_unique<LvglAllocatedImage>(preview_data, image_size_, w, h, stride, LV_COLOR_FORMAT_RGB565);
            display->SetPreviewImage(std::move(image));
        }
    }

    return true;
}

bool EspNowCamera::SetHMirror(bool enabled) {
    // Not supported for remote camera
    ESP_LOGW(TAG, "SetHMirror not supported for ESP-NOW camera");
    return false;
}

bool EspNowCamera::SetVFlip(bool enabled) {
    // Not supported for remote camera
    ESP_LOGW(TAG, "SetVFlip not supported for ESP-NOW camera");
    return false;
}

std::string EspNowCamera::Explain(const std::string& question) {
    if (explain_url_.empty()) {
        throw std::runtime_error("Image explain URL is not set");
    }

    if (image_size_ == 0 || !image_data_) {
        throw std::runtime_error("No image captured");
    }

    // The image from XIAO is already JPEG encoded
    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(3);

    std::string boundary = "----ESP32_CAMERA_BOUNDARY";

    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", Board::GetInstance().GetUuid().c_str());
    if (!explain_token_.empty()) {
        http->SetHeader("Authorization", "Bearer " + explain_token_);
    }
    http->SetHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
    http->SetHeader("Transfer-Encoding", "chunked");

    if (!http->Open("POST", explain_url_)) {
        throw std::runtime_error("Failed to connect to explain URL");
    }

    // Question field
    std::string question_field;
    question_field += "--" + boundary + "\r\n";
    question_field += "Content-Disposition: form-data; name=\"question\"\r\n";
    question_field += "\r\n";
    question_field += question + "\r\n";
    http->Write(question_field.c_str(), question_field.size());

    // File field header
    std::string file_header;
    file_header += "--" + boundary + "\r\n";
    file_header += "Content-Disposition: form-data; name=\"file\"; filename=\"camera.jpg\"\r\n";
    file_header += "Content-Type: image/jpeg\r\n";
    file_header += "\r\n";
    http->Write(file_header.c_str(), file_header.size());

    // Image data
    http->Write((const char*)image_data_, image_size_);

    // Footer
    std::string footer = "\r\n--" + boundary + "--\r\n";
    http->Write(footer.c_str(), footer.size());
    http->Write("", 0);

    if (http->GetStatusCode() != 200) {
        throw std::runtime_error("Failed to upload photo");
    }

    std::string result = http->ReadAll();
    http->Close();

    ESP_LOGI(TAG, "Explain result: %s", result.c_str());
    return result;
}

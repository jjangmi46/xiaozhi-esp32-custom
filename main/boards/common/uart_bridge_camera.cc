#include "uart_bridge_camera.h"

#include <cstring>
#include <stdexcept>
#include <esp_log.h>

#include "board.h"
#include "system_info.h"

static const char* TAG = "UartBridgeCamera";

UartBridgeCamera::UartBridgeCamera(uart_port_t port, gpio_num_t tx, gpio_num_t rx, int baud_rate)
    : uart_port_(port), tx_pin_(tx), rx_pin_(rx), baud_rate_(baud_rate) {
    response_semaphore_ = xSemaphoreCreateBinary();
}

UartBridgeCamera::~UartBridgeCamera() {
    if (rx_task_handle_) {
        vTaskDelete(rx_task_handle_);
    }
    if (uart_queue_) {
        vQueueDelete(uart_queue_);
    }
    if (response_semaphore_) {
        vSemaphoreDelete(response_semaphore_);
    }
    uart_driver_delete(uart_port_);
}

bool UartBridgeCamera::Initialize() {
    ESP_LOGI(TAG, "Initializing UART bridge camera on port %d (TX:%d, RX:%d, baud:%d)",
             uart_port_, tx_pin_, rx_pin_, baud_rate_);

    uart_config_t uart_config = {
        .baud_rate = baud_rate_,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(uart_port_, UART_BRIDGE_BUF_SIZE * 2,
                                         UART_BRIDGE_BUF_SIZE * 2, 20, &uart_queue_, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return false;
    }

    ret = uart_param_config(uart_port_, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return false;
    }

    ret = uart_set_pin(uart_port_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return false;
    }

    // Start receive task
    xTaskCreate(UartRxTask, "UartBridgeRx", 4096, this, 5, &rx_task_handle_);

    ESP_LOGI(TAG, "UART bridge camera initialized");
    return true;
}

void UartBridgeCamera::UartRxTask(void* arg) {
    auto* camera = static_cast<UartBridgeCamera*>(arg);
    camera->ProcessUartData();
}

void UartBridgeCamera::ProcessUartData() {
    uart_event_t event;
    uint8_t* rx_buffer = (uint8_t*)malloc(UART_BRIDGE_BUF_SIZE);
    std::string line_buffer;

    while (true) {
        if (xQueueReceive(uart_queue_, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA: {
                    int len = uart_read_bytes(uart_port_, rx_buffer, event.size, pdMS_TO_TICKS(100));
                    if (len > 0) {
                        for (int i = 0; i < len; i++) {
                            char c = rx_buffer[i];
                            if (c == '\n') {
                                // Complete line received
                                if (!line_buffer.empty()) {
                                    // Remove trailing \r if present
                                    if (line_buffer.back() == '\r') {
                                        line_buffer.pop_back();
                                    }
                                    HandleResponse(line_buffer);
                                    line_buffer.clear();
                                }
                            } else {
                                line_buffer += c;
                            }
                        }
                    }
                    break;
                }
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO overflow");
                    uart_flush_input(uart_port_);
                    xQueueReset(uart_queue_);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART buffer full");
                    uart_flush_input(uart_port_);
                    xQueueReset(uart_queue_);
                    break;
                default:
                    break;
            }
        }
    }

    free(rx_buffer);
}

void UartBridgeCamera::HandleResponse(const std::string& line) {
    ESP_LOGD(TAG, "Received: %s", line.c_str());

    if (waiting_for_response_) {
        // Check for OK: or ERR: prefix
        if (line.substr(0, 3) == "OK:" || line.substr(0, 4) == "ERR:") {
            response_buffer_ = line;
            xSemaphoreGive(response_semaphore_);
        }
    }
}

bool UartBridgeCamera::SendCommand(const std::string& cmd) {
    std::string full_cmd = cmd + "\n";
    int written = uart_write_bytes(uart_port_, full_cmd.c_str(), full_cmd.length());
    if (written != full_cmd.length()) {
        ESP_LOGE(TAG, "Failed to send command: %s", cmd.c_str());
        return false;
    }
    ESP_LOGI(TAG, "Sent: %s", cmd.c_str());
    return true;
}

bool UartBridgeCamera::WaitForResponse(std::string& response, int timeout_ms) {
    waiting_for_response_ = true;
    response_buffer_.clear();

    bool got_response = xSemaphoreTake(response_semaphore_, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;

    waiting_for_response_ = false;

    if (got_response) {
        response = response_buffer_;
        return true;
    }

    ESP_LOGE(TAG, "Timeout waiting for response");
    return false;
}

void UartBridgeCamera::SetExplainUrl(const std::string& url, const std::string& token) {
    explain_url_ = url;
    explain_token_ = token;

    // Send vision config to XIAO with device credentials
    // Format: VISION:<url>|<token>|<device_id>|<client_id>
    std::string device_id = SystemInfo::GetMacAddress();
    std::string client_id = Board::GetInstance().GetUuid();
    std::string cmd = "VISION:" + url + "|" + token + "|" + device_id + "|" + client_id;
    SendCommand(cmd);

    ESP_LOGI(TAG, "Vision URL configured: %s (device: %s)", url.c_str(), device_id.c_str());
}

bool UartBridgeCamera::Capture() {
    // XIAO captures on demand when Explain() is called
    // Nothing to do here
    return true;
}

bool UartBridgeCamera::SetHMirror(bool enabled) {
    // Could send a command to XIAO if needed
    // For now, not implemented
    return true;
}

bool UartBridgeCamera::SetVFlip(bool enabled) {
    // Could send a command to XIAO if needed
    // For now, not implemented
    return true;
}

std::string UartBridgeCamera::Explain(const std::string& question) {
    if (explain_url_.empty()) {
        throw std::runtime_error("Vision URL not configured");
    }

    // Send SNAP command with question
    std::string cmd = "SNAP:" + question;
    if (!SendCommand(cmd)) {
        throw std::runtime_error("Failed to send capture command");
    }

    // Wait for response (may take a while for image upload)
    std::string response;
    if (!WaitForResponse(response, UART_BRIDGE_TIMEOUT_MS)) {
        throw std::runtime_error("Timeout waiting for camera response");
    }

    // Parse response
    if (response.substr(0, 3) == "OK:") {
        // Return the JSON result after "OK:"
        return response.substr(3);
    } else if (response.substr(0, 4) == "ERR:") {
        throw std::runtime_error(response.substr(4));
    } else {
        throw std::runtime_error("Invalid response from camera node");
    }
}

// XIAO ESP32S3 Camera Node
// This board acts as a camera-only peripheral that receives UART commands
// from a master board (like Freenove) and uploads images directly to the cloud

#include <cstring>
#include <exception>
#include <vector>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/uart.h>

#include "wifi_board.h"
#include "wifi_station.h"
#include "display/display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "led/single_led.h"
#include "esp32_camera.h"

#include "esp_video_init.h"

#define TAG "XiaoCameraNode"

#define UART_BUF_SIZE 1024

class XiaoCameraNode : public WifiBoard {
private:
    Button boot_button_;
    Esp32Camera* camera_ = nullptr;

    // UART for communication with master (Freenove)
    QueueHandle_t uart_queue_ = nullptr;
    TaskHandle_t uart_task_ = nullptr;

    void InitializeCamera() {
        ESP_LOGI(TAG, "Initializing camera...");

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

        if (camera_->IsReady()) {
            ESP_LOGI(TAG, "Camera initialized successfully");
        } else {
            ESP_LOGE(TAG, "Camera initialization failed - camera not ready");
        }
    }

    void InitializeUart() {
        ESP_LOGI(TAG, "Initializing UART bridge (port %d, TX:%d, RX:%d)",
                 UART_BRIDGE_PORT, UART_BRIDGE_TX_PIN, UART_BRIDGE_RX_PIN);

        uart_config_t uart_config = {
            .baud_rate = UART_BRIDGE_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        esp_err_t ret = uart_driver_install(UART_BRIDGE_PORT, UART_BUF_SIZE * 2,
                                            UART_BUF_SIZE * 2, 20, &uart_queue_, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
            return;
        }

        ret = uart_param_config(UART_BRIDGE_PORT, &uart_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
            return;
        }

        ret = uart_set_pin(UART_BRIDGE_PORT, UART_BRIDGE_TX_PIN, UART_BRIDGE_RX_PIN,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
            return;
        }

        // Start UART receive task
        xTaskCreate(UartTaskFunc, "UartTask", 8192, this, 5, &uart_task_);

        ESP_LOGI(TAG, "UART bridge initialized");
    }

    static void UartTaskFunc(void* arg) {
        auto* board = static_cast<XiaoCameraNode*>(arg);
        board->UartTaskLoop();
    }

    void UartTaskLoop() {
        uart_event_t event;
        uint8_t* rx_buffer = (uint8_t*)malloc(UART_BUF_SIZE);
        std::string line_buffer;

        ESP_LOGI(TAG, "UART task started, waiting for commands...");

        while (true) {
            if (xQueueReceive(uart_queue_, &event, portMAX_DELAY)) {
                switch (event.type) {
                    case UART_DATA: {
                        int len = uart_read_bytes(UART_BRIDGE_PORT, rx_buffer, event.size,
                                                  pdMS_TO_TICKS(100));
                        if (len > 0) {
                            for (int i = 0; i < len; i++) {
                                char c = rx_buffer[i];
                                if (c == '\n') {
                                    if (!line_buffer.empty()) {
                                        if (line_buffer.back() == '\r') {
                                            line_buffer.pop_back();
                                        }
                                        ProcessCommand(line_buffer);
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
                    case UART_BUFFER_FULL:
                        ESP_LOGW(TAG, "UART overflow, flushing");
                        uart_flush_input(UART_BRIDGE_PORT);
                        xQueueReset(uart_queue_);
                        break;
                    default:
                        break;
                }
            }
        }

        free(rx_buffer);
    }

    void ProcessCommand(const std::string& cmd) {
        ESP_LOGI(TAG, "Received command: %s", cmd.c_str());

        if (cmd.substr(0, 7) == "VISION:") {
            HandleVisionCommand(cmd.substr(7));
        } else if (cmd.substr(0, 5) == "SNAP:") {
            HandleSnapCommand(cmd.substr(5));
        } else {
            ESP_LOGW(TAG, "Unknown command: %s", cmd.c_str());
            SendResponse("ERR:Unknown command");
        }
    }

    void HandleVisionCommand(const std::string& params) {
        // Format: VISION:<url>|<token>|<device_id>|<client_id>
        // Parse by splitting on '|'
        std::vector<std::string> parts;
        size_t start = 0;
        size_t end = 0;
        while ((end = params.find('|', start)) != std::string::npos) {
            parts.push_back(params.substr(start, end - start));
            start = end + 1;
        }
        parts.push_back(params.substr(start));

        if (parts.size() < 2) {
            ESP_LOGE(TAG, "Invalid VISION format (need at least url|token)");
            SendResponse("ERR:Invalid VISION format");
            return;
        }

        std::string url = parts[0];
        std::string token = parts[1];
        std::string device_id = (parts.size() > 2) ? parts[2] : "";
        std::string client_id = (parts.size() > 3) ? parts[3] : "";

        if (camera_) {
            camera_->SetExplainUrl(url, token);
            if (!device_id.empty() && !client_id.empty()) {
                camera_->SetCredentials(device_id, client_id);
                ESP_LOGI(TAG, "Vision URL set: %s (using device: %s)", url.c_str(), device_id.c_str());
            } else {
                ESP_LOGI(TAG, "Vision URL set: %s (using local device ID)", url.c_str());
            }
            SendResponse("OK:Vision configured");
        } else {
            SendResponse("ERR:Camera not initialized");
        }
    }

    void HandleSnapCommand(const std::string& question) {
        ESP_LOGI(TAG, "Processing SNAP: %s", question.c_str());

        if (!camera_) {
            SendResponse("ERR:Camera not initialized");
            return;
        }

        // Check if camera is ready with detailed logging
        ESP_LOGI(TAG, "Camera state: streaming_on=%d, video_fd=%d, ptr=%p",
                 camera_->IsStreamingOn(), camera_->GetVideoFd(), (void*)camera_);

        if (!camera_->IsReady()) {
            ESP_LOGE(TAG, "Camera not ready! streaming_on=%d, video_fd=%d",
                     camera_->IsStreamingOn(), camera_->GetVideoFd());
            SendResponse("ERR:Camera not ready");
            return;
        }

        // Check if WiFi is connected (needed for cloud upload)
        if (!WifiStation::GetInstance().IsConnected()) {
            SendResponse("ERR:WiFi not connected");
            return;
        }

        // Capture image
        if (!camera_->Capture()) {
            ESP_LOGE(TAG, "Camera capture failed");
            SendResponse("ERR:Capture failed");
            return;
        }

        // Upload to cloud and get explanation
        try {
            std::string result = camera_->Explain(question);
            ESP_LOGI(TAG, "Explain result: %s", result.c_str());
            SendResponse("OK:" + result);
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "Explain failed: %s", e.what());
            SendResponse("ERR:" + std::string(e.what()));
        }
    }

    void SendResponse(const std::string& response) {
        std::string full_response = response + "\n";
        uart_write_bytes(UART_BRIDGE_PORT, full_response.c_str(), full_response.length());
        ESP_LOGD(TAG, "Sent response: %s", response.c_str());
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

        InitializeButtons();
        InitializeCamera();

        ESP_LOGI(TAG, "Camera node initialized (UART will start after WiFi)");
    }

    virtual void StartNetwork() override {
        // First start WiFi (required for cloud upload)
        WifiBoard::StartNetwork();

        // Now initialize UART communication
        InitializeUart();

        ESP_LOGI(TAG, "Camera node ready, waiting for commands");
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        // No audio on camera node - return nullptr to skip audio service
        return nullptr;
    }

    virtual Display* GetDisplay() override {
        // No display on camera node - use NoDisplay
        static NoDisplay no_display;
        return &no_display;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(XiaoCameraNode);

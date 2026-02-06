#pragma once

#include <string>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <driver/uart.h>
#include <driver/gpio.h>

#include "camera.h"

// UART Bridge Camera Protocol
// This camera class sends commands to a remote XIAO camera node via UART
// The XIAO node captures images and uploads them directly to the cloud

#define UART_BRIDGE_BUF_SIZE    1024
#define UART_BRIDGE_TIMEOUT_MS  30000  // 30 seconds for image upload

class UartBridgeCamera : public Camera {
private:
    uart_port_t uart_port_;
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    int baud_rate_;

    std::string explain_url_;
    std::string explain_token_;

    // Response handling
    SemaphoreHandle_t response_semaphore_;
    std::string response_buffer_;
    std::atomic<bool> waiting_for_response_{false};

    // UART receive task
    TaskHandle_t rx_task_handle_ = nullptr;
    QueueHandle_t uart_queue_ = nullptr;

    static void UartRxTask(void* arg);
    void ProcessUartData();
    void HandleResponse(const std::string& line);

    bool SendCommand(const std::string& cmd);
    bool WaitForResponse(std::string& response, int timeout_ms = UART_BRIDGE_TIMEOUT_MS);

public:
    UartBridgeCamera(uart_port_t port, gpio_num_t tx, gpio_num_t rx, int baud_rate = 115200);
    virtual ~UartBridgeCamera();

    bool Initialize();

    // Camera interface
    virtual void SetExplainUrl(const std::string& url, const std::string& token) override;
    virtual bool Capture() override;
    virtual bool SetHMirror(bool enabled) override;
    virtual bool SetVFlip(bool enabled) override;
    virtual std::string Explain(const std::string& question) override;
};

#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

// Touch support
#include "esp_lcd_touch_ft5x06.h"
#include <esp_lvgl_port.h>
#include <esp_timer.h>

#include "wifi_station.h"
#include "application.h"
#include "audio/codecs/es8311_audio_codec.h"
#include "button.h"
#include "display/lcd_display.h"
#include "led/single_led.h"
#include "system_reset.h"

#include <esp_log.h>
#include <driver/ledc.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "wifi_board.h"
#include "mcp_server.h"
#include "config.h"
#include "esp_lcd_ili9341.h"

// Touch Pins (FT6336G) - from lcdwiki.com specs
#define TOUCH_RST_PIN       GPIO_NUM_18

// Multi-tap detection timing (microseconds)
#define MULTI_TAP_WINDOW_US      400000   // 400ms window to detect multi-tap sequence

#define TAG "FreenoveBoard"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);

class FreenoveESP32S3Display : public WifiBoard {
 private:
  Button boot_button_;
  LcdDisplay *display_;
  i2c_master_bus_handle_t codec_i2c_bus_;
  esp_lcd_touch_handle_t touch_handle_ = nullptr;

  // Multi-tap detection
  esp_timer_handle_t tap_timer_ = nullptr;
  esp_timer_handle_t angry_revert_timer_ = nullptr;
  int tap_count_ = 0;
  bool first_multi_tap_ = true;

  void InitializeSpi() {
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
    buscfg.miso_io_num = DISPLAY_MIS0_PIN;
    buscfg.sclk_io_num = DISPLAY_SCK_PIN;
    buscfg.quadwp_io_num = GPIO_NUM_NC;
    buscfg.quadhd_io_num = GPIO_NUM_NC;
    buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
  }

  void InitializeLcdDisplay() {
    esp_lcd_panel_io_handle_t panel_io = nullptr;
    esp_lcd_panel_handle_t panel = nullptr;

    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = DISPLAY_CS_PIN;
    io_config.dc_gpio_num = DISPLAY_DC_PIN;
    io_config.spi_mode = DISPLAY_SPI_MODE;
    io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
    io_config.trans_queue_depth = 10;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &panel_io));
    
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = DISPLAY_RST_PIN;
    panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
    panel_config.bits_per_pixel = 16;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
    
    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
    esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
    esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    
    display_ = new SpiLcdDisplay(panel_io, panel, 
        DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, 
        DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
  }

  void InitializeI2c() {
      i2c_master_bus_config_t i2c_bus_cfg = {
          .i2c_port = AUDIO_CODEC_I2C_NUM,
          .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
          .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
          .clk_source = I2C_CLK_SRC_DEFAULT,
          .glitch_ignore_cnt = 7,
          .intr_priority = 0,
          .trans_queue_depth = 0,
          .flags = { .enable_internal_pullup = 1 },
      };
      ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
  }

  // LVGL touch event callback — fires on each tap (click) on the screen
  static void OnScreenClicked(lv_event_t* e) {
      auto* board = static_cast<FreenoveESP32S3Display*>(lv_event_get_user_data(e));
      board->tap_count_++;
      ESP_LOGI(TAG, "Tap #%d detected", board->tap_count_);

      // Reset/restart the multi-tap window timer
      esp_timer_stop(board->tap_timer_);
      esp_timer_start_once(board->tap_timer_, MULTI_TAP_WINDOW_US);
  }

  // Timer callback — fires when multi-tap window expires, processes accumulated taps
  static void OnTapTimerExpired(void* arg) {
      auto* board = static_cast<FreenoveESP32S3Display*>(arg);
      int taps = board->tap_count_;
      board->tap_count_ = 0;
      ESP_LOGI(TAG, "Processing %d tap(s)", taps);

      auto& app = Application::GetInstance();
      auto display = Board::GetInstance().GetDisplay();

      if (taps == 1) {
          // Single tap: Toggle chat state (start/stop listening)
          if (app.GetDeviceState() == kDeviceStateStarting &&
              !WifiStation::GetInstance().IsConnected()) {
              board->ResetWifiConfiguration();
          } else {
              app.ToggleChatState();
          }
      }
      else if (taps >= 4) {
          // 4+ taps: First time always tickled, then random
          bool tickled = board->first_multi_tap_ || (esp_random() % 3) != 0;
          board->first_multi_tap_ = false;

          if (tickled) {
              ESP_LOGI(TAG, "Tickled! (%d taps detected)", taps);
              display->SetEmotion("funny");
              // display->SetChatMessage("assistant", "Hahaha stop it, that tickles!");

              const char* uart_msg = "E:funny\n";
              uart_write_bytes(UART_NUM_1, uart_msg, strlen(uart_msg));
              ESP_LOGI(TAG, "Sent UART: E:funny");
          } else {
              ESP_LOGI(TAG, "Irritated! (%d taps detected)", taps);
              display->SetEmotion("angry");
              // display->SetChatMessage("assistant", "Stop tapping me so much!");

              const char* uart_msg = "E:angry\n";
              uart_write_bytes(UART_NUM_1, uart_msg, strlen(uart_msg));
              ESP_LOGI(TAG, "Sent UART: E:angry");
          }

          // Revert to neutral after 3 seconds
          esp_timer_stop(board->angry_revert_timer_);
          esp_timer_start_once(board->angry_revert_timer_, 3000000);
      }
      // 2-3 taps: ignored or add custom behavior here
  }

  // Timer callback — reverts display from angry state back to neutral
  static void OnAngryRevert(void* arg) {
      auto* board = static_cast<FreenoveESP32S3Display*>(arg);
      auto display = Board::GetInstance().GetDisplay();
      display->SetEmotion("neutral");
      display->SetChatMessage("system", "");
      ESP_LOGI(TAG, "Reverted from angry to neutral");
  }

  void InitializeTouch() {
    ESP_LOGI(TAG, "Initializing FT6336 Touch...");

    // Hardware reset the touch controller
    gpio_config_t rst_gpio_config = {
        .pin_bit_mask = (1ULL << TOUCH_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_gpio_config);
    gpio_set_level(TOUCH_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(TOUCH_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Use standard FT5x06 I2C config (same as working esp32-s3-touch-lcd-3.5 board)
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    tp_io_config.scl_speed_hz = 400000;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(codec_i2c_bus_, &tp_io_config, &tp_io_handle));

    // Configure the touch driver
    esp_lcd_touch_config_t touch_config = {
        .x_max = DISPLAY_WIDTH,
        .y_max = DISPLAY_HEIGHT,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = DISPLAY_SWAP_XY,
            .mirror_x = DISPLAY_MIRROR_X,
            .mirror_y = DISPLAY_MIRROR_Y,
        },
    };

    esp_err_t ret = esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &touch_config, &touch_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize FT5x06 driver (0x%x)", ret);
        return;
    }

    // Register touch with LVGL (same approach as working esp32-s3-touch-lcd-3.5 board)
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = lv_display_get_default(),
        .handle = touch_handle_,
    };
    lvgl_port_add_touch(&touch_cfg);

    // Create multi-tap timer
    esp_timer_create_args_t timer_args = {};
    timer_args.callback = OnTapTimerExpired;
    timer_args.arg = this;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name = "tap_timer";
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &tap_timer_));

    // Create angry-revert timer (reverts display back to neutral after irritated response)
    esp_timer_create_args_t angry_timer_args = {};
    angry_timer_args.callback = OnAngryRevert;
    angry_timer_args.arg = this;
    angry_timer_args.dispatch_method = ESP_TIMER_TASK;
    angry_timer_args.name = "angry_revert";
    ESP_ERROR_CHECK(esp_timer_create(&angry_timer_args, &angry_revert_timer_));

    // Add a full-screen clickable overlay on LVGL's top layer for tap detection
    lvgl_port_lock(0);
    lv_obj_t *touch_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(touch_overlay);
    lv_obj_set_size(touch_overlay, lv_pct(100), lv_pct(100));
    lv_obj_add_flag(touch_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(touch_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(touch_overlay, OnScreenClicked, LV_EVENT_CLICKED, this);
    lvgl_port_unlock();

    ESP_LOGI(TAG, "Touch initialization complete.");
  }

  void InitializeButtons() {
    boot_button_.OnClick([this]() {
      auto &app = Application::GetInstance();
      if (app.GetDeviceState() == kDeviceStateStarting &&
          !WifiStation::GetInstance().IsConnected()) {
        ResetWifiConfiguration();
      }
      app.ToggleChatState();
    });
  }

 public:
  FreenoveESP32S3Display(): boot_button_(BOOT_BUTTON_GPIO)
  {
    InitializeI2c();
    InitializeSpi();
    InitializeLcdDisplay();
    InitializeTouch();
    InitializeButtons();
    InitializeTools(); 
    GetBacklight()->SetBrightness(100);
  }

  void InitializeTools() {
      auto& mcp_server = McpServer::GetInstance();

      mcp_server.AddTool("self.get_weather", 
        "Get local weather.", 
        PropertyList({ Property("location", kPropertyTypeString, "City name") }), 
        [this](const PropertyList& properties) -> ReturnValue {
            std::string location = properties["location"].value<std::string>();
            GetDisplay()->SetChatMessage("assistant", ("Checking weather for " + location).c_str());
            return "Weather tool executed."; 
        });

      mcp_server.AddTool("self.music_player.play", 
        "Play music URL.", 
        PropertyList({ Property("url", kPropertyTypeString, "URL") }), 
        [this](const PropertyList& properties) -> ReturnValue {
            GetDisplay()->SetChatMessage("assistant", "Playing music...");
            GetDisplay()->SetEmotion("music");
            return "Music started.";
        });
  }
  

  virtual Led *GetLed() override { 
      static SingleLed led(BUILTIN_LED_GPIO); 
      return &led; 
  }
  
  virtual AudioCodec* GetAudioCodec() override {
    static Es8311AudioCodec audio_codec(codec_i2c_bus_, AUDIO_CODEC_I2C_NUM,
      AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK, 
      AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, 
      AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR, 
      true, true);
    return &audio_codec;
  }
  
  virtual Display *GetDisplay() override { 
      return display_; 
  }
  
  virtual Backlight *GetBacklight() override { 
      static PwmBacklight b(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT); 
      return &b; 
  }
};

DECLARE_BOARD(FreenoveESP32S3Display);
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

// Add these for Touch
#include "esp_lcd_touch_ft5x06.h"
#include "lvgl.h"

#include "wifi_station.h"
#include "application.h"
#include "audio/codecs/es8311_audio_codec.h"
#include "button.h"
#include "display/lcd_display.h"
#include "led/single_led.h"
#include "system_reset.h"

#include <esp_log.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "wifi_board.h"
#include "mcp_server.h"
#include "config.h"
#include "esp_lcd_ili9341.h"

// Touch Pins as per your hardware
#define TOUCH_INT_PIN       GPIO_NUM_17
#define TOUCH_I2C_ADDRESS   0x38

#define TAG "FreenoveBoard"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);

class FreenoveESP32S3Display : public WifiBoard {
 private:
  Button boot_button_;
  LcdDisplay *display_;
  i2c_master_bus_handle_t codec_i2c_bus_;
  esp_lcd_touch_handle_t touch_handle_ = nullptr;
  
  // Track touch state to detect a "Release" (Tap)
    // Replace last_touch_state_ with this:
  int touch_press_count_ = 0; 

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

// --- LVGL v9 TOUCH INPUT CALLBACK ---
  static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
      auto* board = (FreenoveESP32S3Display*)lv_indev_get_user_data(indev);
      if (board->touch_handle_ == nullptr) return;

      esp_lcd_touch_point_data_t point;
      uint8_t tp_num = 0;

      // 1. Force the driver to read from I2C
      esp_err_t err = esp_lcd_touch_read_data(board->touch_handle_);
      
      // 2. Get the actual data
      bool touched = (esp_lcd_touch_get_data(board->touch_handle_, &point, &tp_num, 1) == ESP_OK && tp_num > 0);

      if (touched) {
          // Log coordinates once to verify it works
          ESP_LOGD(TAG, "Touch Raw: X=%d, Y=%d", point.x, point.y);
          
          data->point.x = point.x;
          data->point.y = point.y;
          data->state = LV_INDEV_STATE_PRESSED;
          board->touch_press_count_++;
      } else {
          data->state = LV_INDEV_STATE_RELEASED;
          
          // Trigger if held for more than 1 frame (debounce) but less than 1 second
          if (board->touch_press_count_ > 1 && board->touch_press_count_ < 50) {
              ESP_LOGI(TAG, "Validated Screen Tap Detected!");
              auto &app = Application::GetInstance();
              if (app.GetDeviceState() == kDeviceStateStarting &&
                  !WifiStation::GetInstance().IsConnected()) {
                board->ResetWifiConfiguration();
              }
              app.ToggleChatState();
          }
          board->touch_press_count_ = 0;
      }
  }

  void InitializeTouch() {
    ESP_LOGI(TAG, "Initializing FT6336 Touch (Diagnostic Mode)...");

    // 1. Manually check if I2C device at 0x38 exists
    // (This helps verify the hardware is wired correctly)
    
    // 2. Configure I2C IO (Back to 400kHz for compatibility with Audio)
    esp_lcd_panel_io_i2c_config_t tp_io_config = {};
    tp_io_config.dev_addr = TOUCH_I2C_ADDRESS; // 0x38
    tp_io_config.scl_speed_hz = 400000; 
    tp_io_config.control_phase_bytes = 1;
    tp_io_config.dc_bit_offset = 0;
    tp_io_config.lcd_cmd_bits = 8;
    tp_io_config.lcd_param_bits = 8;

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(codec_i2c_bus_, &tp_io_config, &tp_io_handle));

    // 3. Configure the touch driver
    esp_lcd_touch_config_t touch_config = {};
    touch_config.x_max = DISPLAY_WIDTH;
    touch_config.y_max = DISPLAY_HEIGHT;
    touch_config.rst_gpio_num = GPIO_NUM_NC; 
    
    // We set this to NC (Not Connected) inside the driver config 
    // This forces the driver to poll via I2C instead of waiting for a hardware interrupt
    touch_config.int_gpio_num = GPIO_NUM_NC; 
    
    touch_config.levels.reset = 0;
    touch_config.levels.interrupt = 0;
    
    // Freenove 2.8" usually needs these to align with the screen
    touch_config.flags.swap_xy = DISPLAY_SWAP_XY;
    touch_config.flags.mirror_x = DISPLAY_MIRROR_X;
    touch_config.flags.mirror_y = DISPLAY_MIRROR_Y;

    esp_err_t ret = esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &touch_config, &touch_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize FT5x06 driver (0x%x)", ret);
        return;
    }

    // 4. Register with LVGL 9
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lvgl_touch_read_cb);
    lv_indev_set_user_data(indev, this);
    
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
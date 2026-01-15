#include "board.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_lcd_panel_ili9341.h"
#include "esp_lcd_touch_ft5x06.h"

static const char* TAG = "AliExpressWiFiLCD";

namespace xiaozhi {

class AliExpressWiFiLCDBoard : public Board {
public:
    AliExpressWiFiLCDBoard() = default;
    ~AliExpressWiFiLCDBoard() override = default;

    esp_err_t Initialize() override {
        ESP_LOGI(TAG, "Initializing AliExpress WiFi LCD Board");
        
        esp_err_t ret;
        
        // Initialize power amplifier control (active low)
        ret = InitializeAudioPA();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize audio PA");
            return ret;
        }

        // Initialize LCD display
        ret = InitializeLCD();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize LCD");
            return ret;
        }

        // Initialize touch controller
        ret = InitializeTouch();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize touch");
            return ret;
        }

        // Initialize I2S audio
        ret = InitializeAudio();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize audio");
            return ret;
        }

        // Initialize SD card if enabled
#ifdef SD_CARD_ENABLED
        ret = InitializeSDCard();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "SD card initialization failed or no card present");
            // Non-critical, continue
        }
#endif

        ESP_LOGI(TAG, "Board initialization complete");
        return ESP_OK;
    }

private:
    esp_err_t InitializeAudioPA() {
        ESP_LOGI(TAG, "Configuring audio PA on GPIO %d (active %s)", 
                 AUDIO_PA_ENABLE_GPIO, AUDIO_PA_ENABLE_LEVEL ? "HIGH" : "LOW");
        
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << AUDIO_PA_ENABLE_GPIO);
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        
        esp_err_t ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            return ret;
        }
        
        // Initially disable amplifier (set to inactive state)
        gpio_set_level((gpio_num_t)AUDIO_PA_ENABLE_GPIO, !AUDIO_PA_ENABLE_LEVEL);
        
        return ESP_OK;
    }

    esp_err_t InitializeLCD() {
        ESP_LOGI(TAG, "Initializing ILI9341 LCD");
        
        // Configure SPI bus for LCD
        spi_bus_config_t buscfg = {
            .mosi_io_num = LCD_MOSI_GPIO,
            .miso_io_num = LCD_MISO_GPIO,  // LCD has MISO on GPIO13
            .sclk_io_num = LCD_CLK_GPIO,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t),
        };

        esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
            return ret;
        }

        // Configure LCD panel IO
        esp_lcd_panel_io_handle_t io_handle = NULL;
        esp_lcd_panel_io_spi_config_t io_config = {
            .cs_gpio_num = LCD_CS_GPIO,
            .dc_gpio_num = LCD_DC_GPIO,
            .spi_mode = 0,
            .pclk_hz = 40 * 1000 * 1000,  // 40 MHz
            .trans_queue_depth = 10,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
        };

        ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create LCD panel IO: %s", esp_err_to_name(ret));
            return ret;
        }

        // Configure LCD panel
        esp_lcd_panel_handle_t panel_handle = NULL;
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = LCD_RST_GPIO,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
            .bits_per_pixel = 16,
        };

        ret = esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create LCD panel: %s", esp_err_to_name(ret));
            return ret;
        }

        // Reset and initialize the display
        esp_lcd_panel_reset(panel_handle);
        esp_lcd_panel_init(panel_handle);
        esp_lcd_panel_invert_color(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        
        // Set rotation
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_set_gap(panel_handle, 0, 0);

        // Turn on display
        esp_lcd_panel_disp_on_off(panel_handle, true);

        // Configure backlight
        if (LCD_BL_GPIO >= 0) {
            gpio_config_t bk_gpio_config = {
                .pin_bit_mask = (1ULL << LCD_BL_GPIO),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&bk_gpio_config);
            gpio_set_level((gpio_num_t)LCD_BL_GPIO, LCD_BL_ON_LEVEL);
        }

        ESP_LOGI(TAG, "LCD initialized successfully");
        return ESP_OK;
    }

    esp_err_t InitializeTouch() {
        ESP_LOGI(TAG, "Initializing FT6336 touch controller");
        
        // Configure I2C for touch controller
        i2c_config_t i2c_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = TOUCH_I2C_SDA_GPIO,
            .scl_io_num = TOUCH_I2C_SCL_GPIO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                .clk_speed = TOUCH_I2C_FREQ_HZ,
            },
            .clk_flags = 0,
        };

        esp_err_t ret = i2c_param_config(TOUCH_I2C_NUM, &i2c_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure I2C: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = i2c_driver_install(TOUCH_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
            return ret;
        }

        // Configure touch reset pin
        if (TOUCH_RST_GPIO >= 0) {
            gpio_config_t rst_gpio_config = {
                .pin_bit_mask = (1ULL << TOUCH_RST_GPIO),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&rst_gpio_config);
            
            // Reset touch controller
            gpio_set_level((gpio_num_t)TOUCH_RST_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level((gpio_num_t)TOUCH_RST_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        // Configure touch interrupt pin
        if (TOUCH_INT_GPIO >= 0) {
            gpio_config_t int_gpio_config = {
                .pin_bit_mask = (1ULL << TOUCH_INT_GPIO),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_NEGEDGE,
            };
            gpio_config(&int_gpio_config);
        }

        ESP_LOGI(TAG, "Touch controller initialized successfully");
        return ESP_OK;
    }

    esp_err_t InitializeAudio() {
        ESP_LOGI(TAG, "Initializing I2S audio with MCLK on GPIO4");
        
        // Configure I2S standard channel
        i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(AUDIO_I2S_NUM, I2S_ROLE_MASTER);
        chan_cfg.auto_clear = true;
        
        i2s_chan_handle_t tx_handle = NULL;
        i2s_chan_handle_t rx_handle = NULL;
        
        // Create I2S channels
        esp_err_t ret = i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create I2S channels: %s", esp_err_to_name(ret));
            return ret;
        }

        // Configure I2S standard for output (speaker)
        i2s_std_config_t std_cfg = {
            .clk_cfg = {
                .sample_rate_hz = AUDIO_OUTPUT_SAMPLE_RATE,
                .clk_src = I2S_CLK_SRC_DEFAULT,
                .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            },
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
            .gpio_cfg = {
                .mclk = (gpio_num_t)AUDIO_I2S_MCK_IO,  // GPIO4 for master clock
                .bclk = (gpio_num_t)AUDIO_I2S_BCK_IO,
                .ws = (gpio_num_t)AUDIO_I2S_WS_IO,
                .dout = (gpio_num_t)AUDIO_I2S_DO_IO,
                .din = (gpio_num_t)AUDIO_I2S_DI_IO,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false,
                },
            },
        };

        // Initialize TX channel (speaker)
        if (tx_handle) {
            ret = i2s_channel_init_std_mode(tx_handle, &std_cfg);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to init I2S TX: %s", esp_err_to_name(ret));
                return ret;
            }
            i2s_channel_enable(tx_handle);
        }

        // Initialize RX channel (microphone)
        if (rx_handle) {
            std_cfg.clk_cfg.sample_rate_hz = AUDIO_INPUT_SAMPLE_RATE;
            ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to init I2S RX: %s", esp_err_to_name(ret));
                return ret;
            }
            i2s_channel_enable(rx_handle);
        }

        // Enable audio amplifier
        gpio_set_level((gpio_num_t)AUDIO_PA_ENABLE_GPIO, AUDIO_PA_ENABLE_LEVEL);
        
        ESP_LOGI(TAG, "Audio initialized successfully");
        return ESP_OK;
    }

    esp_err_t InitializeSDCard() {
        ESP_LOGI(TAG, "Initializing SD card in 4-bit SDMMC mode");
        
        // SD card initialization for 4-bit SDMMC mode
        // Configuration for all 4 data lines (DATA0-DATA3)
        /*
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
        
        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
        slot_config.width = 4;  // 4-bit mode using D0-D3
        slot_config.clk = SD_CLK_GPIO;
        slot_config.cmd = SD_CMD_GPIO;
        slot_config.d0 = SD_D0_GPIO;
        slot_config.d1 = SD_D1_GPIO;
        slot_config.d2 = SD_D2_GPIO;
        slot_config.d3 = SD_D3_GPIO;
        
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024
        };
        
        sdmmc_card_t* card;
        esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "SD card mounted successfully");
            sdmmc_card_print_info(stdout, card);
        }
        
        return ret;
        */
        
        return ESP_OK;
    }
};



}  // namespace xiaozhi
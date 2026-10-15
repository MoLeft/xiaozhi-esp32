#include "application.h"
#include "button.h"
#include "codecs/box_audio_codec.h"
#include "config.h"
#include "display/emote_display.h"
#include "display/lcd_display.h"
#include "esp32_camera.h"
#include "i2c_device.h"
#include "mcp_server.h"
#include "motion_detector.h"
#include "press_to_talk_mcp_tool.h"
#include "settings.h"
#include "wifi_board.h"

#include <driver/i2c_master.h>
#include <driver/sdmmc_host.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_touch_ft5x06.h>
#include <esp_log.h>
#include <esp_lvgl_port.h>
#include <esp_timer.h>
#include <esp_vfs_fat.h>
#include <freertos/task.h>
#include <lvgl.h>
#include <sdmmc_cmd.h>
#include <atomic>
#include <mutex>

#define TAG "LichuangDevBoard"

class Pca9557 : public I2cDevice {
public:
    Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x01, 0x03);
        WriteReg(0x03, 0xf8);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        uint8_t data = ReadReg(0x01);
        data = (data & ~(1 << bit)) | (level << bit);
        WriteReg(0x01, data);
    }
};

class CustomAudioCodec : public BoxAudioCodec {
private:
    Pca9557* pca9557_;

public:
    CustomAudioCodec(i2c_master_bus_handle_t i2c_bus, Pca9557* pca9557)
        : BoxAudioCodec(i2c_bus, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                        AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
                        AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, GPIO_NUM_NC,
                        AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR, AUDIO_INPUT_REFERENCE,
                        28.0f,  // Physical MIC1 gain
                        2,      // Physical MIC3 is the playback reference input
                        0.0f),
          pca9557_(pca9557) {}

    virtual void EnableOutput(bool enable) override {
        BoxAudioCodec::EnableOutput(enable);
        if (enable) {
            pca9557_->SetOutputState(1, 1);
        } else {
            pca9557_->SetOutputState(1, 0);
        }
    }
};

class LichuangDevBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t pca9557_handle_;
    i2c_master_dev_handle_t imu_device_ = nullptr;
    Button boot_button_;
    Display* display_;
    Pca9557* pca9557_;
    Esp32Camera* camera_;
    PressToTalkMcpTool* press_to_talk_tool_ = nullptr;
    std::atomic<bool> wake_shake_enabled_{true};
    std::atomic<bool> wake_topple_enabled_{true};
    std::atomic<MotionPose> pose_{MotionPose::Unknown};
    std::atomic<uint32_t> pose_sample_ms_{0};
    std::mutex sd_mutex_;
    sdmmc_card_t* sd_card_ = nullptr;

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // Initialize PCA9557
        pca9557_ = new Pca9557(i2c_bus_, 0x19);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            // During startup (before connected), pressing BOOT button enters Wi-Fi config mode without reboot
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            // In press-to-talk mode the click event is handled by press down/up
            if (!press_to_talk_tool_ || !press_to_talk_tool_->IsPressToTalkEnabled()) {
                app.ToggleChatState();
            }
        });

        boot_button_.OnPressDown([this]() {
            if (press_to_talk_tool_ && press_to_talk_tool_->IsPressToTalkEnabled()) {
                Application::GetInstance().StartListening();
            }
        });
        boot_button_.OnPressUp([this]() {
            if (press_to_talk_tool_ && press_to_talk_tool_->IsPressToTalkEnabled()) {
                Application::GetInstance().StopListening();
            }
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        pca9557_->SetOutputState(0, 0);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);

#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
#else
        display_ = new SpiLcdDisplay(panel_io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
    }

    void InitializeTouch()
    {
        esp_lcd_touch_handle_t tp = nullptr;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_HEIGHT,
            .y_max = DISPLAY_WIDTH,
            .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
            .int_gpio_num = GPIO_NUM_NC, 
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 1,
                .mirror_x = 1,
                .mirror_y = 0,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = {
            .dev_addr = ESP_LCD_TOUCH_IO_I2C_FT5x06_ADDRESS,
            .control_phase_bytes = 1,
            .dc_bit_offset = 0,
            .lcd_cmd_bits = 8,
            .flags =
            {
                .disable_control_phase = 1,
            }
        };
        tp_io_config.scl_speed_hz = 400000;

        if (esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to create touch panel IO, continuing without touch");
            return;
        }
        if (esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp) != ESP_OK || tp == nullptr) {
            ESP_LOGW(TAG, "FT5x06 touch controller not found, continuing without touch");
            esp_lcd_panel_io_del(tp_io_handle);
            return;
        }

        /* Add touch input (for selected screen) */
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(), 
            .handle = tp,
        };

        if(touch_cfg.disp) {
            lvgl_port_add_touch(&touch_cfg);
        } else {
            ESP_LOGE(TAG, "Touch display is not initialized");
        }
    }

    esp_err_t WriteImuReg(uint8_t reg, uint8_t value) {
        uint8_t data[] = {reg, value};
        return i2c_master_transmit(imu_device_, data, sizeof(data), 100);
    }

    void InitializeMotionSensor() {
        i2c_device_config_t imu_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x6a,
            .scl_speed_hz = 100000,
        };
        esp_err_t err = i2c_master_bus_add_device(i2c_bus_, &imu_config, &imu_device_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "QMI8658 unavailable: %s", esp_err_to_name(err));
            return;
        }

        uint8_t reg = 0x00;
        uint8_t id = 0;
        err = i2c_master_transmit_receive(imu_device_, &reg, 1, &id, 1, 100);
        if (err != ESP_OK || id != 0x05) {
            ESP_LOGW(TAG, "QMI8658 not found (err=%s, id=0x%02x)", esp_err_to_name(err), id);
            i2c_master_bus_rm_device(imu_device_);
            imu_device_ = nullptr;
            return;
        }

        for (int attempt = 0; attempt < 3; ++attempt) {
            err = WriteImuReg(0x60, 0xb0);
            if (err != ESP_OK)
                continue;
            vTaskDelay(pdMS_TO_TICKS(30));
            err = WriteImuReg(0x02, 0x40);  // Auto-increment
            if (err == ESP_OK)
                err = WriteImuReg(0x03, 0x15);  // Accel: 4g, 250 Hz; self-test off
            if (err == ESP_OK)
                err = WriteImuReg(0x08, 0x01);  // Enable accel only
            if (err != ESP_OK)
                continue;

            uint8_t status_reg = 0x2e;
            uint8_t status = 0;
            for (int poll = 0; poll < 10; ++poll) {
                vTaskDelay(pdMS_TO_TICKS(10));
                err = i2c_master_transmit_receive(imu_device_, &status_reg, 1, &status, 1, 100);
                if (err != ESP_OK || (status & 1))
                    break;
            }
            if (err == ESP_OK && (status & 1))
                break;
            if (err == ESP_OK)
                err = ESP_ERR_TIMEOUT;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "QMI8658 did not start sampling: %s", esp_err_to_name(err));
            i2c_master_bus_rm_device(imu_device_);
            imu_device_ = nullptr;
            return;
        }

        if (xTaskCreate(MotionTask, "motion", 4096, this, 4, nullptr) != pdPASS) {
            ESP_LOGW(TAG, "Failed to start motion task");
            i2c_master_bus_rm_device(imu_device_);
            imu_device_ = nullptr;
            return;
        }
        ESP_LOGI(TAG, "QMI8658 motion controls ready");
    }

    static void MotionTask(void* arg) {
        auto* board = static_cast<LichuangDevBoard*>(arg);
        MotionDetector detector(MotionDetector::ScreenUpAxis::X, true);
        unsigned failed_reads = 0;
        while (true) {
            uint8_t status_reg = 0x2e;
            uint8_t status = 0;
            esp_err_t err =
                i2c_master_transmit_receive(board->imu_device_, &status_reg, 1, &status, 1, 100);
            if (err == ESP_OK && (status & 1) == 0) {
                vTaskDelay(pdMS_TO_TICKS(80));
                continue;
            }
            uint8_t reg = 0x35;
            uint8_t raw[6] = {};
            if (err == ESP_OK) {
                err =
                    i2c_master_transmit_receive(board->imu_device_, &reg, 1, raw, sizeof(raw), 100);
            }
            if (err == ESP_OK) {
                failed_reads = 0;
                int16_t ax = static_cast<int16_t>(raw[0] | (raw[1] << 8));
                int16_t ay = static_cast<int16_t>(raw[2] | (raw[3] << 8));
                int16_t az = static_cast<int16_t>(raw[4] | (raw[5] << 8));
                auto& app = Application::GetInstance();
                uint32_t now_ms = esp_timer_get_time() / 1000;
                auto decision = detector.Update(
                    ax, ay, az, now_ms, app.GetDeviceState() == kDeviceStateIdle,
                    board->wake_shake_enabled_.load(), board->wake_topple_enabled_.load());
                board->pose_.store(decision.pose);
                board->pose_sample_ms_.store(now_ms);
                if (decision.rotation_changed && lvgl_port_lock(100)) {
                    lv_display_t* display = lv_display_get_default();
                    if (display != nullptr) {
                        lv_display_set_rotation(display, decision.inverted ? LV_DISPLAY_ROTATION_180
                                                                           : LV_DISPLAY_ROTATION_0);
                        ESP_LOGI(TAG, "Screen %s (accel=%d,%d,%d)",
                                 decision.inverted ? "inverted" : "normal", ax, ay, az);
                    }
                    lvgl_port_unlock();
                }
                if (decision.shake) {
                    ESP_LOGI(TAG, "Shake detected, waking XiaoZhi");
                    app.WakeWordInvoke("小智刚刚被用户摇晃唤醒了");
                }
                if (decision.toppled) {
                    ESP_LOGI(TAG, "Topple detected, waking XiaoZhi");
                    app.WakeWordInvoke("小智被推倒了");
                }
            } else if (++failed_reads % 20 == 1) {
                ESP_LOGW(TAG, "QMI8658 read failed: %s", esp_err_to_name(err));
            }
            vTaskDelay(pdMS_TO_TICKS(80));
        }
    }

    void InitializeCamera() {
        // Open camera power
        pca9557_->SetOutputState(2, 0);

        camera_config_t config = {};
        config.ledc_channel = LEDC_CHANNEL_2;
        config.ledc_timer = LEDC_TIMER_2;
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = -1;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 1;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

        camera_ = new Esp32Camera(config);
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        Settings motion_settings("motion");
        wake_shake_enabled_.store(motion_settings.GetBool("shake", true));
        wake_topple_enabled_.store(motion_settings.GetBool("topple", true));

        mcp_server.AddTool(
            "self.motion.get_pose",
            "Get the board's current physical pose from its accelerometer: upright, inverted, "
            "face_up, face_down, or moving_or_tilted. Also reports current gesture wake settings.",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                const uint32_t sample_ms = pose_sample_ms_.load();
                const uint32_t age_ms = esp_timer_get_time() / 1000 - sample_ms;
                const bool available = sample_ms != 0 && age_ms <= 1000;
                const char* pose = "unknown";
                if (available) {
                    switch (pose_.load()) {
                        case MotionPose::Upright:
                            pose = "upright";
                            break;
                        case MotionPose::Inverted:
                            pose = "inverted";
                            break;
                        case MotionPose::FaceUp:
                            pose = "face_up";
                            break;
                        case MotionPose::FaceDown:
                            pose = "face_down";
                            break;
                        case MotionPose::MovingOrTilted:
                            pose = "moving_or_tilted";
                            break;
                        default:
                            break;
                    }
                }
                cJSON* result = cJSON_CreateObject();
                cJSON_AddBoolToObject(result, "available", available);
                cJSON_AddStringToObject(result, "pose", pose);
                cJSON_AddBoolToObject(result, "shake_wake_enabled", wake_shake_enabled_.load());
                cJSON_AddBoolToObject(result, "topple_wake_enabled", wake_topple_enabled_.load());
                if (available)
                    cJSON_AddNumberToObject(result, "sample_age_ms", age_ms);
                return result;
            });

        mcp_server.AddTool(
            "self.motion.set_wake_gestures",
            "Enable or disable shake and topple-to-flat wake gestures independently. "
            "Both arguments are required. To change only one gesture, first call "
            "self.motion.get_pose and preserve the other setting. Settings survive reboot; "
            "screen rotation remains automatic.",
            PropertyList({Property("shake", kPropertyTypeBoolean),
                          Property("topple", kPropertyTypeBoolean)}),
            [this](const PropertyList& properties) -> ReturnValue {
                const bool shake = properties["shake"].value<bool>();
                const bool topple = properties["topple"].value<bool>();
                {
                    Settings settings("motion", true);
                    settings.SetBool("shake", shake);
                    settings.SetBool("topple", topple);
                }
                wake_shake_enabled_.store(shake);
                wake_topple_enabled_.store(topple);
                cJSON* result = cJSON_CreateObject();
                cJSON_AddBoolToObject(result, "shake", shake);
                cJSON_AddBoolToObject(result, "topple", topple);
                return result;
            });

        mcp_server.AddTool(
            "self.storage.get_status",
            "Check the board's microSD card and report FAT filesystem total and free space. "
            "An unavailable card is reported without formatting it.",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                std::lock_guard<std::mutex> lock(sd_mutex_);
                esp_err_t err = ESP_OK;
                if (sd_card_ == nullptr) {
                    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
                    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
                    slot.width = 1;
                    slot.clk = GPIO_NUM_47;
                    slot.cmd = GPIO_NUM_48;
                    slot.d0 = GPIO_NUM_21;
                    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
                    esp_vfs_fat_mount_config_t mount = {};
                    mount.format_if_mount_failed = false;
                    mount.max_files = 2;
                    err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mount, &sd_card_);
                } else {
                    err = sdmmc_get_status(sd_card_);
                }

                uint64_t total_bytes = 0;
                uint64_t free_bytes = 0;
                if (err == ESP_OK) {
                    err = esp_vfs_fat_info("/sdcard", &total_bytes, &free_bytes);
                }
                if (err != ESP_OK && sd_card_ != nullptr) {
                    esp_vfs_fat_sdcard_unmount("/sdcard", sd_card_);
                    sd_card_ = nullptr;
                }

                cJSON* result = cJSON_CreateObject();
                cJSON_AddBoolToObject(result, "mounted", err == ESP_OK);
                if (err == ESP_OK) {
                    cJSON_AddNumberToObject(result, "total_bytes", total_bytes);
                    cJSON_AddNumberToObject(result, "free_bytes", free_bytes);
                } else {
                    cJSON_AddStringToObject(result, "error", esp_err_to_name(err));
                }
                return result;
            });

        mcp_server.AddTool("self.system.reconfigure_wifi",
            "End this conversation and enter WiFi configuration mode.\n"
            "**CAUTION** You must ask the user to confirm this action.",
            PropertyList(), [this](const PropertyList& properties) {
                EnterWifiConfigMode();
                return true;
            });

        // Allow switching between press-to-talk (长按说话) and click-to-talk (单击唤醒)
        press_to_talk_tool_ = new PressToTalkMcpTool();
        press_to_talk_tool_->Initialize();
    }

public:
    LichuangDevBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        InitializeTouch();
        InitializeButtons();
        InitializeCamera();
        InitializeTools();
        InitializeMotionSensor();

        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CustomAudioCodec audio_codec(
            i2c_bus_, 
            pca9557_);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(LichuangDevBoard);

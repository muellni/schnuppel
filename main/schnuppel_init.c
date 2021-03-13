#include "schnuppel_init.h"

#include "a2dp_stream.h"
#include "audio_mem.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_peripherals.h"
#include "i2s_stream.h"
#include "opus_decoder.h"
#include "periph_adc_button.h"
#include "periph_button.h"
#include "periph_touch.h"
#include "periph_wifi.h"
#include "snapclient_stream.h"

schnuppel_handle_t schnuppel_init()
{
    schnuppel_handle_t schnuppel = audio_calloc(1, sizeof(struct schnuppel_handle));
    schnuppel_init_board(schnuppel);
    schnuppel_init_bt(schnuppel);
    schnuppel_init_bt_stream_reader(schnuppel);
    schnuppel_init_i2s_stream(schnuppel);
    schnuppel_init_snapclient(schnuppel);
    schnuppel_init_opus(schnuppel);
    schnuppel_init_periph(schnuppel);
    schnuppel_init_event(schnuppel);
    return schnuppel;
}

void schnuppel_init_board(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Start audio codec chip");
    schnuppel->board = audio_board_init();
    audio_hal_ctrl_codec(schnuppel->board->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
}

void schnuppel_init_pipeline(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Create audio pipeline");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    schnuppel->pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(schnuppel->pipeline);
}

void schnuppel_init_bt(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Init Bluetooth");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_dev_set_device_name("Schnuppel");

    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
}

void schnuppel_init_bt_stream_reader(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Init Bluetooth stream reader");

    a2dp_stream_config_t a2dp_config = {
        .type = AUDIO_STREAM_READER,
        .user_callback = {0},
        .audio_hal = schnuppel->board->audio_hal,
    };
    schnuppel->bt_stream_reader = a2dp_stream_init(&a2dp_config);

}

void schnuppel_init_snapclient(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Create snapclient source stream");
    snapclient_stream_cfg_t snapclient_cfg = SNAPCLIENT_STREAM_CFG_DEFAULT();
	snapclient_cfg.port = CONFIG_SNAPSERVER_PORT;
	snapclient_cfg.host = CONFIG_SNAPSERVER_HOST;
	snapclient_cfg.audio_board = schnuppel->board;
    schnuppel->snapclient_stream = snapclient_stream_init(&snapclient_cfg);
}

void schnuppel_init_opus(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Create opus decoder");
    opus_decoder_cfg_t opus_cfg = DEFAULT_OPUS_DECODER_CONFIG();
    schnuppel->opus_decoder = decoder_opus_init(&opus_cfg);
}

void schnuppel_init_i2s_stream(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    schnuppel->i2s_stream_writer = i2s_stream_init(&i2s_cfg);
}

void schnuppel_init_pipeline_elements(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Register all elements to audio pipeline");
    if (schnuppel->snapclient_stream)
    {
        audio_pipeline_register(schnuppel->pipeline, schnuppel->snapclient_stream, "snapclient");
    }
    if (schnuppel->opus_decoder)
    {
        audio_pipeline_register(schnuppel->pipeline, schnuppel->opus_decoder, "opus");
    }
    if (schnuppel->i2s_stream_writer)
    {
        audio_pipeline_register(schnuppel->pipeline, schnuppel->i2s_stream_writer, "i2s");
    }
    if (schnuppel->bt_stream_reader)
    {
        audio_pipeline_register(schnuppel->pipeline, schnuppel->bt_stream_reader, "bt");
    }
}

void schnuppel_init_periph(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Initialize peripheral");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    schnuppel->periph_set = esp_periph_set_init(&periph_cfg);
    ESP_LOGI(TAG, "Initialize Touch peripheral");
    audio_board_key_init(schnuppel->periph_set);

    periph_wifi_cfg_t wifi_cfg = {
        .ssid = CONFIG_ESP_WIFI_SSID,
        .password = CONFIG_ESP_WIFI_PASSWORD,
    };

    ESP_LOGI(TAG, "Initialize Wifi peripheral");
    schnuppel->wifi_handle = periph_wifi_init(&wifi_cfg);

    ESP_LOGI(TAG, "Initialize Bluetooth peripheral");
    schnuppel->bt_periph = bt_create_periph();
}

void schnuppel_init_event(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    schnuppel->event_handle = audio_event_iface_init(&evt_cfg);
}

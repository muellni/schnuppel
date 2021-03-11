#include "schnuppel.h"

#include <sys/time.h>

#include "audio_mem.h"
#include "esp_log.h"
#include "esp_peripherals.h"
#include "i2s_stream.h"
#include "opus_decoder.h"
#include "periph_adc_button.h"
#include "periph_button.h"
#include "periph_touch.h"
#include "periph_wifi.h"
#include "snapclient_stream.h"

#if 0

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/apps/sntp.h"


#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "board.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_wifi.h"
#include "filter_resample.h"
#include "periph_touch.h"
#include "periph_button.h"
#include "periph_adc_button.h"

#define HFP_RESAMPLE_RATE 16000

static void wait_for_sntp(void)
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;

    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "europe.pool.ntp.org");
    sntp_init();

    const int retry_count = 20;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}

/* event for handler "bt_av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/* handler for bluetooth stack enabled events */
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

#endif

schnuppel_handle_t schnuppel_init()
{
    schnuppel_handle_t result;
    result = audio_calloc(1, sizeof(struct schnuppel_handle));
    schnuppel_init_board(result);
    schnuppel_init_pipeline(result);
    schnuppel_init_snapclient(result);
    schnuppel_init_opus(result);
    schnuppel_init_i2s_stream(result);
    schnuppel_init_pipeline_elements(result);
    schnuppel_init_periph(result);
    schnuppel_init_event(result);
    return result;
}

void schnuppel_start(schnuppel_handle_t schnuppel)
{
    schnuppel_start_snapclient(schnuppel);
    schnuppel_start_periph(schnuppel);
    schnuppel_start_time(schnuppel);
    schnuppel_start_pipeline(schnuppel);
}

void schnuppel_init_board(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Start audio codec chip");
    schnuppel->board = audio_board_init();
    audio_hal_ctrl_codec(schnuppel->board->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
}

void schnuppel_init_pipeline(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Create audio pipeline, add all elements to pipeline, and subscribe pipeline event");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    schnuppel->pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(schnuppel->pipeline);
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
    ESP_LOGI(TAG, "reate opus decoder");
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
    audio_pipeline_register(schnuppel->pipeline, schnuppel->snapclient_stream, "snapclient");
    //audio_pipeline_register(schnuppel->pipeline, schnuppel->bt_stream_reader, "bt");
    audio_pipeline_register(schnuppel->pipeline, schnuppel->opus_decoder, "opus");
    audio_pipeline_register(schnuppel->pipeline, schnuppel->i2s_stream_writer, "i2s");
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
}

void schnuppel_init_event(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    schnuppel->event_handle = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "Listening event from all elements of pipeline");
    audio_pipeline_set_listener(schnuppel->pipeline, schnuppel->event_handle);

    ESP_LOGI(TAG, "Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(schnuppel->periph_set), schnuppel->event_handle);
}

void schnuppel_start_snapclient(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Linking snapclient to audio pipeline");

    const char *link_tag[2] = {"snapclient", "i2s"};
    audio_pipeline_link(schnuppel->pipeline, &link_tag[0], 2);
}

void schnuppel_start_periph(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Start the Wi-Fi network");
    esp_periph_start(schnuppel->periph_set, schnuppel->wifi_handle);
    ESP_LOGI(TAG, "wait for connection");
	while (1) {
		esp_err_t result = periph_wifi_wait_for_connected(schnuppel->wifi_handle, 2000 / portTICK_PERIOD_MS);
		if (result == ESP_OK)
			break;
		ESP_LOGW(TAG, "still waiting for connection");
	}
}

void schnuppel_start_time(schnuppel_handle_t schnuppel)
{
	struct timeval now;
	if (gettimeofday(&now, NULL)) {
		ESP_LOGW(TAG, "Failed to gettimeofday");
	} else {
		ESP_LOGI(TAG, "Current timestamp is %ld.%ld", now.tv_sec, now.tv_usec);
	}
}

void schnuppel_start_pipeline(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Start audio_pipeline");
    audio_pipeline_run(schnuppel->pipeline);

	i2s_stream_set_clk(schnuppel->i2s_stream_writer, 48000 , 16, 2);
}

void schnuppel_handle_event(schnuppel_handle_t schnuppel)
{
    audio_event_iface_msg_t msg;
    char source[20];
    ESP_LOGI(TAG, "[ X ] Waiting for a new message");
    esp_err_t ret = audio_event_iface_listen(schnuppel->event_handle, &msg, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
        return;
    }
    //if (msg.source == (void *) opus_decoder)
    //	sprintf(source, "%s", "opus");
    //else
    if (msg.source == (void *) schnuppel->snapclient_stream)
        sprintf(source, "%s", "snapclient");
    else if (msg.source == (void *) schnuppel->i2s_stream_writer)
        sprintf(source, "%s", "i2s");
    else
        sprintf(source, "%s", "unknown");

    ESP_LOGI(TAG, "[ X ] Event message %d:%d from %s", msg.source_type, msg.cmd, source);
    if (msg.cmd == AEL_MSG_CMD_REPORT_STATUS)
        switch ( (int) msg.data ) {
            case AEL_STATUS_NONE:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_NONE");
                break;
            case AEL_STATUS_ERROR_OPEN:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_ERROR_OPEN");
                break;
            case AEL_STATUS_ERROR_INPUT:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_ERROR_INPUT");
                break;
            case AEL_STATUS_ERROR_PROCESS:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_ERROR_PROCESS");
                break;
            case AEL_STATUS_ERROR_OUTPUT:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_ERROR_OUTPUT");
                break;
            case AEL_STATUS_ERROR_CLOSE:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_ERROR_CLOSE");
                break;
            case AEL_STATUS_ERROR_TIMEOUT:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_ERROR_TIMEOUT");
                break;
            case AEL_STATUS_ERROR_UNKNOWN:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_ERROR_UNKNOWN");
                break;
            case AEL_STATUS_INPUT_DONE:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_INPUT_DONE");
                break;
            case AEL_STATUS_INPUT_BUFFERING:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_INPUT_BUFFERING");
                break;
            case AEL_STATUS_OUTPUT_DONE:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_OUTPUT_DONE");
                break;
            case AEL_STATUS_OUTPUT_BUFFERING:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_OUTPUT_BUFFERING");
                break;
            case AEL_STATUS_STATE_RUNNING:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_STATE_RUNNING");
                break;
            case AEL_STATUS_STATE_PAUSED:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_STATE_PAUSED");
                break;
            case AEL_STATUS_STATE_STOPPED:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_STATE_STOPPED");
                break;
            case AEL_STATUS_STATE_FINISHED:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_STATE_FINISHED");
                break;
            case AEL_STATUS_MOUNTED:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_MOUNTED");
                break;
            case AEL_STATUS_UNMOUNTED:
                ESP_LOGI(TAG, "[ X ]   status AEL_STATUS_UNMOUNTED");
                break;
        }

    if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT
        && msg.source == (void *) schnuppel->snapclient_stream
        && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
        ESP_LOGI(TAG, "[ X ] report music info ");
        audio_element_info_t music_info = {0};
        audio_element_getinfo(schnuppel->snapclient_stream, &music_info);

        ESP_LOGI(TAG, "[ * ] Receive music info from snapclient decoder, sample_rates=%d, bits=%d, ch=%d",
                    music_info.sample_rates, music_info.bits, music_info.channels);

        //audio_element_setinfo(opus_decoder, &music_info);
        audio_element_setinfo(schnuppel->i2s_stream_writer, &music_info);

        i2s_stream_set_clk(schnuppel->i2s_stream_writer, music_info.sample_rates , music_info.bits, music_info.channels);
        return;
    }

    if ((msg.source_type == PERIPH_ID_TOUCH || msg.source_type == PERIPH_ID_BUTTON || msg.source_type == PERIPH_ID_ADC_BTN)
        && (msg.cmd == PERIPH_TOUCH_TAP || msg.cmd == PERIPH_BUTTON_PRESSED || msg.cmd == PERIPH_ADC_BUTTON_PRESSED)) {

        if ((int) msg.data == get_input_play_id()) {
            ESP_LOGI(TAG, "[ * ] [Play] touch tap event");
        } else if ((int) msg.data == get_input_set_id()) {
            ESP_LOGI(TAG, "[ * ] [Set] touch tap event");
        } else if ((int) msg.data == get_input_volup_id()) {
            ESP_LOGI(TAG, "[ * ] [Vol+] touch tap event");
        } else if ((int) msg.data == get_input_voldown_id()) {
            ESP_LOGI(TAG, "[ * ] [Vol-] touch tap event");
        }
    }

    if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) schnuppel->opus_decoder
        && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
        ESP_LOGI(TAG, "[ X ] opus message ");
        audio_element_info_t music_info = {0};
        audio_element_getinfo(schnuppel->opus_decoder, &music_info);

        ESP_LOGI(TAG, "[ * ] Receive music info from opus decoder, sample_rates=%d, bits=%d, ch=%d",
                    music_info.sample_rates, music_info.bits, music_info.channels);

        audio_element_setinfo(schnuppel->i2s_stream_writer, &music_info);

        i2s_stream_set_clk(schnuppel->i2s_stream_writer, music_info.sample_rates , music_info.bits, music_info.channels);
        return;
    }

    /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
    if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) schnuppel->i2s_stream_writer
        && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
        && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
        ESP_LOGI(TAG, "i2s wants to stop!");
        schnuppel_stop(schnuppel);
    }
}

void schnuppel_stop(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Stop audio_pipeline");
    audio_pipeline_stop(schnuppel->pipeline);
    audio_pipeline_wait_for_stop(schnuppel->pipeline);
    audio_pipeline_terminate(schnuppel->pipeline);

	audio_pipeline_unregister(schnuppel->pipeline, schnuppel->snapclient_stream);
    audio_pipeline_unregister(schnuppel->pipeline, schnuppel->opus_decoder);
    audio_pipeline_unregister(schnuppel->pipeline, schnuppel->i2s_stream_writer);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(schnuppel->pipeline);

    /* Make sure audio_pipeline_remove_listener is called before destroying event_iface */
    audio_event_iface_destroy(schnuppel->event_handle);

    /* Release all resources */
	/* D: still neeeded?
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
    audio_pipeline_unregister(pipeline, opus_decoder);
	*/
    audio_pipeline_deinit(schnuppel->pipeline);
    audio_element_deinit(schnuppel->i2s_stream_writer);
    audio_element_deinit(schnuppel->opus_decoder);
    audio_element_deinit(schnuppel->snapclient_stream);
}

#if 0

void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(BT_AV_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
    default: {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
    return;
}
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
    switch (event) {
    case BT_APP_EVT_STACK_UP: {
        /* set up device name */
        char *dev_name = "Schnuppel";
        esp_bt_dev_set_device_name(dev_name);

        esp_bt_gap_register_callback(bt_app_gap_cb);

        /* initialize AVRCP controller */
        esp_avrc_ct_init();
        esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
        /* initialize AVRCP target */
        assert (esp_avrc_tg_init() == ESP_OK);
        esp_avrc_tg_register_callback(bt_app_rc_tg_cb);

        esp_avrc_rn_evt_cap_mask_t evt_set = {0};
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
        assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);

        /* initialize A2DP sink */
        esp_a2d_register_callback(&bt_app_a2d_cb);
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
        esp_a2d_sink_init();

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}
#endif
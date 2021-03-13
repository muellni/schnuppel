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
    ESP_LOGI(TAG, "Handle event: Waiting for a new message");
    esp_err_t ret = audio_event_iface_listen(schnuppel->event_handle, &msg, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Handle event: Event interface error : %d", ret);
        return;
    }

    bool handled = false;

    switch (msg.source_type) {
        case AUDIO_ELEMENT_TYPE_ELEMENT:
            handled = schnuppel_handle_event_audio_element_type(schnuppel, msg);
            break;
        case PERIPH_ID_TOUCH:
        case PERIPH_ID_BUTTON:
        case PERIPH_ID_ADC_BTN:
            if (msg.cmd == PERIPH_TOUCH_TAP || msg.cmd == PERIPH_BUTTON_PRESSED || msg.cmd == PERIPH_ADC_BUTTON_PRESSED){
                handled = schnuppel_handle_event_button_down(schnuppel, msg);
            }
            break;
    }

    if (handled) {
        return;
    }

    if (msg.source == (void *) schnuppel->i2s_stream_writer) {
        handled = schnuppel_handle_event_i2s_stream_writer(schnuppel, msg);
    } else if (msg.source == (void *) schnuppel->snapclient_stream) {
        handled = schnuppel_handle_event_snapclient_stream(schnuppel, msg);
    } else if (msg.source == (void *) schnuppel->opus_decoder) {
        handled = schnuppel_handle_event_opus_decoder(schnuppel, msg);
    } else if (msg.source == (void *) schnuppel->bt_stream_reader) {
        handled = schnuppel_handle_event_bt_stream_reader(schnuppel, msg);
    }
    else {
        ESP_LOGE(TAG, "Handle event: unknown source");
    }

    if (handled) {
        return;
    }

    if (msg.cmd == AEL_MSG_CMD_REPORT_STATUS) {
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
    }
}

bool schnuppel_handle_event_audio_element_type(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
{
    ESP_LOGI(TAG, "Handle event: Processing message AUDIO_ELEMENT_TYPE_ELEMENT");

    if (msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
        ESP_LOGI(TAG, "  report music info ");
        audio_element_info_t music_info = {0};
        audio_element_getinfo(msg.source, &music_info);

        //audio_element_setinfo(schnuppel->opus_decoder, &music_info);
        audio_element_setinfo(schnuppel->i2s_stream_writer, &music_info);

        i2s_stream_set_clk(schnuppel->i2s_stream_writer, music_info.sample_rates , music_info.bits, music_info.channels);
        return true;
    } else {
        ESP_LOGI(TAG, "  unhandled event ");
    }
    return false;
}

bool schnuppel_handle_event_button_down(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
{
    ESP_LOGI(TAG, "Handle event: Processing message AUDIO_ELEMENT_TYPE_ELEMENT");

    if ((int) msg.data == get_input_play_id()) {
        ESP_LOGI(TAG, "[ * ] [Play] touch tap event");
        return true;
    } else if ((int) msg.data == get_input_set_id()) {
        ESP_LOGI(TAG, "[ * ] [Set] touch tap event");
        return true;
    } else if ((int) msg.data == get_input_volup_id()) {
        ESP_LOGI(TAG, "[ * ] [Vol+] touch tap event");
        return true;
    } else if ((int) msg.data == get_input_voldown_id()) {
        ESP_LOGI(TAG, "[ * ] [Vol-] touch tap event");
        return true;
    }

    return false;
}

bool schnuppel_handle_event_i2s_stream_writer(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
{
    ESP_LOGI(TAG, "Handle event: Processing message from i2s_stream_writer ");

    /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
    if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
        && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
        ESP_LOGI(TAG, "i2s wants to stop!");
        schnuppel_stop(schnuppel);
        return true;
    }

    return false;
}

bool schnuppel_handle_event_snapclient_stream(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
{
    ESP_LOGI(TAG, "Handle event: Processing message from snapclient_stream ");
    return false;
}

bool schnuppel_handle_event_opus_decoder(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
{
    ESP_LOGI(TAG, "Handle event: Processing message from opus_decoder ");
    return false;
}

bool schnuppel_handle_event_bt_stream_reader(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
{
    ESP_LOGI(TAG, "Handle event: Processing message from bt_stream_reader ");
    return false;
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

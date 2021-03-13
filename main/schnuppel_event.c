#include "schnuppel_event.h"

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

void schnuppel_event_handle(schnuppel_handle_t schnuppel)
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
            handled = schnuppel_event_handle_audio_element_type(schnuppel, msg);
            break;
        case PERIPH_ID_TOUCH:
        case PERIPH_ID_BUTTON:
        case PERIPH_ID_ADC_BTN:
            if (msg.cmd == PERIPH_TOUCH_TAP || msg.cmd == PERIPH_BUTTON_PRESSED || msg.cmd == PERIPH_ADC_BUTTON_PRESSED){
                handled = schnuppel_event_handle_button_down(schnuppel, msg);
            }
            break;
    }

    if (handled) {
        return;
    }

    if (msg.source == (void *) schnuppel->i2s_stream_writer_bt || msg.source == (void *) schnuppel->i2s_stream_writer_snapclient) {
        handled = schnuppel_event_handle_i2s_stream_writer(schnuppel, msg);
    } else if (msg.source == (void *) schnuppel->snapclient_stream) {
        handled = schnuppel_event_handle_snapclient_stream(schnuppel, msg);
    } else if (msg.source == (void *) schnuppel->opus_decoder) {
        handled = schnuppel_event_handle_opus_decoder(schnuppel, msg);
    } else if (msg.source == (void *) schnuppel->bt_stream_reader || msg.source == (void *) schnuppel->bt_periph) {
        handled = schnuppel_event_handle_bt(schnuppel, msg);
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
    if (!handled) {
        ESP_LOGE(TAG, "Handle event: unhandled event");
    }
}

bool schnuppel_event_handle_audio_element_type(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
{
    ESP_LOGI(TAG, "Handle event: Processing message AUDIO_ELEMENT_TYPE_ELEMENT");

    if (msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
        audio_element_info_t music_info = {0};
        audio_element_getinfo(msg.source, &music_info);

        ESP_LOGI(TAG, "  receive music info: sample_rates=%d, bits=%d, ch=%d",
                    music_info.sample_rates, music_info.bits, music_info.channels);

        audio_element_setinfo(schnuppel->i2s_stream_writer_bt, &music_info);
        audio_element_setinfo(schnuppel->i2s_stream_writer_snapclient, &music_info);

        i2s_stream_set_clk(schnuppel->i2s_stream_writer_bt, music_info.sample_rates , music_info.bits, music_info.channels);
        i2s_stream_set_clk(schnuppel->i2s_stream_writer_snapclient, music_info.sample_rates , music_info.bits, music_info.channels);
        return true;
    } else {
        ESP_LOGI(TAG, "  unhandled event ");
    }
    return false;
}

bool schnuppel_event_handle_button_down(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
{
    ESP_LOGI(TAG, "Handle event: Processing message button down");

    switch ((int)msg.data) {
        case TOUCH_SET:
            ESP_LOGI(TAG, "  set touch set");
            return true;
        case TOUCH_PLAY:
            ESP_LOGI(TAG, "  set touch play");
            return true;
        case TOUCH_VOLUP:
            ESP_LOGI(TAG, "  set touch vol up");
            schnuppel_switch_mode(schnuppel, SCHNUPPEL_MODE_SNAPCLIENT);
            return true;
        case TOUCH_VOLDWN:
            ESP_LOGI(TAG, "  set touch vol down");
            schnuppel_switch_mode(schnuppel, SCHNUPPEL_MODE_BT);
            return true;
    }
    return false;
}

bool schnuppel_event_handle_i2s_stream_writer(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
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

bool schnuppel_event_handle_snapclient_stream(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
{
    ESP_LOGI(TAG, "Handle event: Processing message from snapclient_stream ");
    return false;
}

bool schnuppel_event_handle_opus_decoder(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
{
    ESP_LOGI(TAG, "Handle event: Processing message from opus_decoder ");
    return false;
}

bool schnuppel_event_handle_bt(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg)
{
    ESP_LOGI(TAG, "Handle event: Processing message from bt");
    if (msg.source_type == PERIPH_ID_BLUETOOTH
        && msg.source == (void *)schnuppel->bt_periph) {
        if (msg.cmd == PERIPH_BLUETOOTH_CONNECTED) {
            ESP_LOGI(TAG, "Bluetooth connected");
            return true;
        } else  if (msg.cmd == PERIPH_BLUETOOTH_DISCONNECTED) {
            ESP_LOGI(TAG, "Bluetooth disconnected");
            return true;
        }
    }
    return false;
}

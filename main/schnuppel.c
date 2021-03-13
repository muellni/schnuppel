#include "schnuppel.h"

#include <sys/time.h>

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

#include "schnuppel_init.h"

void schnuppel_start(schnuppel_handle_t schnuppel)
{
    schnuppel_start_periph(schnuppel);
    schnuppel_start_time(schnuppel);
    audio_pipeline_run(schnuppel->pipeline_bt);
    audio_pipeline_pause(schnuppel->pipeline_bt);
    audio_pipeline_run(schnuppel->pipeline_snapclient);
}

void schnuppel_switch_mode(schnuppel_handle_t schnuppel, enum SchnuppelMode mode)
{
    switch (mode) {
        case SCHNUPPEL_MODE_BT:
            {
                audio_pipeline_pause(schnuppel->pipeline_snapclient);
                audio_pipeline_resume(schnuppel->pipeline_bt);
            }
            break;
        case SCHNUPPEL_MODE_SNAPCLIENT:
            {
                audio_pipeline_pause(schnuppel->pipeline_bt);
                audio_pipeline_resume(schnuppel->pipeline_snapclient);
            }
            break;
    }
    schnuppel->mode = mode;
}

void schnuppel_start_periph(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Start all peripherals");

    ESP_LOGI(TAG, "  Bluetooth");
    esp_periph_start(schnuppel->periph_set, schnuppel->bt_periph);

    ESP_LOGI(TAG, "  Wi-Fi network");
    esp_periph_start(schnuppel->periph_set, schnuppel->wifi_handle);
    ESP_LOGI(TAG, "wait for connection");
	while (1) {
		esp_err_t result = periph_wifi_wait_for_connected(schnuppel->wifi_handle, 2000 / portTICK_PERIOD_MS);
		if (result == ESP_OK)
			break;
		ESP_LOGW(TAG, "still waiting for connection");
	}

    ESP_LOGI(TAG, "Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(schnuppel->periph_set), schnuppel->event_handle);
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
    //audio_pipeline_run(schnuppel->pipeline);

	//i2s_stream_set_clk(schnuppel->i2s_stream_writer, 48000 , 16, 2);
}


void schnuppel_stop(schnuppel_handle_t schnuppel)
{
    /*
    if (schnuppel->pipeline)
    {
        ESP_LOGW(TAG, "Stop audio_pipeline");
        //audio_pipeline_stop(schnuppel->pipeline);
        //audio_pipeline_wait_for_stop(schnuppel->pipeline);
        //audio_pipeline_terminate(schnuppel->pipeline);
        //audio_pipeline_remove_listener(schnuppel->pipeline);
        audio_pipeline_deinit(schnuppel->pipeline);
        schnuppel->pipeline = NULL;
    }
    */

    /*
    a2dp_destroy();
    if (schnuppel->bt_stream_reader) {
	    audio_pipeline_unregister(schnuppel->pipeline, schnuppel->bt_stream_reader);
        audio_element_deinit(schnuppel->bt_stream_reader);
        schnuppel->bt_stream_reader = NULL;
    }
    */

    /*
    if (schnuppel->snapclient_stream) {
	    audio_pipeline_unregister(schnuppel->pipeline, schnuppel->snapclient_stream);
        audio_element_deinit(schnuppel->snapclient_stream);
        schnuppel->snapclient_stream = NULL;
    }

    if (schnuppel->opus_decoder) {
	    audio_pipeline_unregister(schnuppel->pipeline, schnuppel->opus_decoder);
        audio_element_deinit(schnuppel->opus_decoder);
        schnuppel->opus_decoder = NULL;
    }
    */

    /*
    if (schnuppel->i2s_stream_writer) {
	    audio_pipeline_unregister(schnuppel->pipeline, schnuppel->i2s_stream_writer);
        audio_element_deinit(schnuppel->i2s_stream_writer);
        schnuppel->i2s_stream_writer = NULL;
    }
    */
    
    /*
    if (schnuppel->pipeline)
    {
        audio_pipeline_deinit(schnuppel->pipeline);
    }
    */
}

void schnuppel_uninit(schnuppel_handle_t schnuppel)
{
    ESP_LOGI(TAG, "Stop Schnuppel");

    /* Make sure audio_pipeline_remove_listener is called before destroying event_iface */
    audio_event_iface_destroy(schnuppel->event_handle);

    esp_periph_set_stop_all(schnuppel->periph_set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(schnuppel->periph_set), schnuppel->event_handle);

    esp_periph_set_destroy(schnuppel->periph_set);

    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
}

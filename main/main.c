#include <string.h>
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "schnuppel.h"
#include "schnuppel_event.h"
#include "schnuppel_init.h"

void app_main(void)
{
	// setup logging
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

	ESP_ERROR_CHECK(esp_netif_init());

    schnuppel_handle_t schnuppel = schnuppel_init();
    schnuppel_start(schnuppel);
    //schnuppel_start_snapclient(schnuppel);
    //schnuppel_start_bt(schnuppel);
    while(1) {
        schnuppel_event_handle(schnuppel);
    }
}
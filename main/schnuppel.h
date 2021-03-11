#pragma once

#include "audio_element.h"
#include "audio_pipeline.h"
#include "board.h"

static const char *TAG = "SCHNUPPEL";

struct schnuppel_handle {
    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_writer, opus_decoder, snapclient_stream, bt_stream_reader;
    audio_board_handle_t board;
    esp_periph_set_handle_t periph_set;
    esp_periph_handle_t wifi_handle;
};

typedef struct schnuppel_handle *schnuppel_handle_t;

schnuppel_handle_t schnuppel_init();

void schnuppel_start(schnuppel_handle_t schnuppel);

void schnuppel_init_board(schnuppel_handle_t schnuppel);

void schnuppel_init_pipeline(schnuppel_handle_t schnuppel);

void schnuppel_init_snapclient(schnuppel_handle_t schnuppel);

void schnuppel_init_opus(schnuppel_handle_t schnuppel);

void schnuppel_init_i2s_stream(schnuppel_handle_t schnuppel);

void schnuppel_init_pipeline_elements(schnuppel_handle_t schnuppel);

void schnuppel_init_periph(schnuppel_handle_t schnuppel);

void schnuppel_init_event(schnuppel_handle_t schnuppel);

void schnuppel_start_snapclient(schnuppel_handle_t schnuppel);

void schnuppel_start_periph(schnuppel_handle_t schnuppel);
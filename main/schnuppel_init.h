#pragma once

#include "schnuppel.h"

schnuppel_handle_t schnuppel_init();

void schnuppel_init_board(schnuppel_handle_t schnuppel);

void schnuppel_init_pipeline(schnuppel_handle_t schnuppel);

void schnuppel_init_bt(schnuppel_handle_t schnuppel);

void schnuppel_init_bt_stream_reader(schnuppel_handle_t schnuppel);

void schnuppel_init_snapclient(schnuppel_handle_t schnuppel);

void schnuppel_init_opus(schnuppel_handle_t schnuppel);

void schnuppel_init_i2s_stream(schnuppel_handle_t schnuppel);

void schnuppel_init_pipeline_elements(schnuppel_handle_t schnuppel);

void schnuppel_init_periph(schnuppel_handle_t schnuppel);

void schnuppel_init_event(schnuppel_handle_t schnuppel);

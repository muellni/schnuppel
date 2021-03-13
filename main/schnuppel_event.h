#pragma once

#include "audio_event_iface.h"
#include "schnuppel.h"

void schnuppel_event_handle(schnuppel_handle_t schnuppel);

bool schnuppel_event_handle_audio_element_type(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg);

bool schnuppel_event_handle_button_down(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg);

bool schnuppel_event_handle_i2s_stream_writer(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg);

bool schnuppel_event_handle_snapclient_stream(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg);

bool schnuppel_event_handle_opus_decoder(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg);

bool schnuppel_event_handle_bt(schnuppel_handle_t schnuppel, audio_event_iface_msg_t msg);

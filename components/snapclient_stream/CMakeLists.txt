idf_component_register(SRCS "snapclient_stream.c"
                       INCLUDE_DIRS "include"
                       REQUIRES audio_pipeline audio_sal audio_board audio_hal esp_peripherals esp-adf-libs lightsnapcast)

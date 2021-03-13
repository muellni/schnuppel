idf_component_register(
    SRCS 
        "main.c"
        "schnuppel.c"
       
       
    REQUIRES 
        bluetooth_service 
        snapclient_stream)

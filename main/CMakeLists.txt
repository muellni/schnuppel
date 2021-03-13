idf_component_register(
    SRCS 
        "main.c"
        "schnuppel.c"
        "schnuppel_event.c"      
        "schnuppel_init.c"      
       
    REQUIRES 
        bluetooth_service 
        snapclient_stream)

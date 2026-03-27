idf_component_register(
    SRCS
       
        "sht30.c"
        "veml6030.c"
        "ntc.c"
    INCLUDE_DIRS "."
    REQUIRES esp-tflite-micro
)
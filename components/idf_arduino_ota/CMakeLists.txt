idf_component_register(SRCS "idf_arduino_ota.c"
                       INCLUDE_DIRS "."
                       REQUIRES app_update freertos esp_event log)
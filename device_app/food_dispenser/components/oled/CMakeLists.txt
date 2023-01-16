idf_component_register(SRCS "oled.c"
                    INCLUDE_DIRS "include"
                    REQUIRES lwip freertos esp_wifi esp_system esp_event nvs_flash)

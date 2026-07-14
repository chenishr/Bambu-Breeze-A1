idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES json mqtt esp_wifi esp_event nvs_flash driver esp_http_server led_strip)

target_add_binary_data(${COMPONENT_LIB} "index.html" TEXT)

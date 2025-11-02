idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."

    REQUIRES
        freertos
        esp_wifi
        esp_now
        esp_netif
        esp_event
        nvs_flash
        esp_log
)

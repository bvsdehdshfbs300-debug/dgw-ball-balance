idf_component_register(SRCS "main.c" "emm42.c" "mpu6050.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_wifi nvs_flash esp_netif driver esp_timer lwip)

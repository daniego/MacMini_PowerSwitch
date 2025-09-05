idf_component_register(
  SRCS "main.c"
  INCLUDE_DIRS "."
  REQUIRES esp_netif esp_eth esp_wifi nvs_flash esp_http_server esp_timer driver mdns
)

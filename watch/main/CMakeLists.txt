idf_component_register(
  SRCS "main.c" "pan_tompkins.c" "display.c" "battery.c" "ble_ecg.c"
  REQUIRES driver esp_timer esp_adc bt nvs_flash
  INCLUDE_DIRS "."
)

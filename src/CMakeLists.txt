idf_component_register(SRCS main.cpp compat.cpp FurbleControl.cpp CameraPage.cpp
                       INCLUDE_DIRS "." "../include"
                       REQUIRES driver esp_timer esp_system nvs_flash
                                esp_driver_usb_serial_jtag bt)

# Arduino 工程里的 %d 配 int32_t (xtensa 上为 long) 与 format-truncation 属无害告警
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format -Wno-error=format-truncation)

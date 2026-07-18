file(GLOB srcs "main.c" "*/src/*.c")

file(GLOB comp_incs "*/include/*.h")

set(comp_inc_dirs "")

foreach(comp_inc ${comp_incs})
    get_filename_component(comp_inc_dir ${comp_inc} DIRECTORY)
    list(APPEND comp_inc_dirs ${comp_inc_dir})
endforeach()

set(libs spi_flash esp_wifi lwip esp_netif nvs_flash app_update mbedtls esp_driver_mcpwm
                    esp_driver_pcnt esp_driver_gpio esp_timer spiffs esp_driver_i2c esp_driver_spi driver)

idf_component_register(SRCS "${srcs}"
                    PRIV_REQUIRES ${libs}
                    INCLUDE_DIRS "${comp_inc_dirs}" "public_include")
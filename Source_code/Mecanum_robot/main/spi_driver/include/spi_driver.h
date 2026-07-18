#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_
#include "public.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "soc/io_mux_reg.h"

#define SPI_MOSI_PIN GPIO_NUM_40
#define SPI_MISO_PIN GPIO_NUM_41
#define SPI_SCLK_PIN GPIO_NUM_38

esp_err_t init_spi_driver(void);

#endif
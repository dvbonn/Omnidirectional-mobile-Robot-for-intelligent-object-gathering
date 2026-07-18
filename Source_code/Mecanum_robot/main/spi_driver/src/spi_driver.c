#include "spi_driver.h"

esp_err_t init_spi_driver(void)
{
    PIN_FUNC_SELECT(IO_MUX_GPIO41_REG, FUNC_MTDI_GPIO41);
    PIN_FUNC_SELECT(IO_MUX_GPIO40_REG, FUNC_MTDO_GPIO40);

    // //debugging: print the IO configuration of the SPI pins
    // gpio_dump_io_configuration(stdout, 1ULL << SPI_MOSI_PIN);
    // gpio_dump_io_configuration(stdout, 1ULL << SPI_MISO_PIN);


    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCLK_PIN,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = 32
    };

    return spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_DISABLED);
}
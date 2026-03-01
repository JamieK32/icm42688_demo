
#ifndef ICM_SPI_DRIVER_H__
#define ICM_SPI_DRIVER_H__

#include "ti_msp_dl_config.h"

int my_spi_read(void *user_ctx, uint8_t reg, uint8_t *data, uint32_t len);
int my_spi_write(void *user_ctx, uint8_t reg, const uint8_t *data, uint32_t len);

#endif 

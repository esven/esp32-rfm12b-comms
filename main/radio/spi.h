/*
 * SPI driver for STM32 family processors
 *
 * 2009-2010 Michal Demin
 *
 */

#ifndef RADIO_SPI_H_
#define RADIO_SPI_H_

#include "esp_attr.h"
#include <stdint.h>

void RFM_SPI_init(void);

uint16_t IRAM_ATTR SPI_Xfer(uint16_t data);

#endif /* RADIO_SPI_H_ */

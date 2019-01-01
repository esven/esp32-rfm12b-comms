/*
 * SPI driver for ESP32 family processors
 *
 * 2018 Sven Ebenfeld
 *
 */
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include <string.h>

#include "spi.h"
#include "platform.h"

spi_device_handle_t spi;

void RFM_SPI_init(void)
{
	esp_err_t ret;
//	spi_device_handle_t spi;
	spi_bus_config_t buscfg={
			.miso_io_num=PIN_NUM_MISO,
			.mosi_io_num=PIN_NUM_MOSI,
			.sclk_io_num=PIN_NUM_CLK,
			.quadwp_io_num=-1,
			.quadhd_io_num=-1,
			.max_transfer_sz=1,
			.flags= 0U | SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_NATIVE_PINS
			| SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI,
	};
	spi_device_interface_config_t devcfg={
			.clock_speed_hz=SPI_MASTER_FREQ_8M,    //Clock out at 1 MHz
			.mode=0,                               //SPI mode 0
			.spics_io_num=RFM_CS_PIN,              //CS pin
			.queue_size=1,
	};
	//Initialize non-SPI GPIOs
	gpio_set_direction(RFM_CS_PIN, GPIO_MODE_OUTPUT);
	//Initialize the SPI bus
	ret=spi_bus_initialize(VSPI_HOST, &buscfg, 0);
	ESP_ERROR_CHECK(ret);
	//Attach the RFM12B to the SPI bus
	ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
}


/* Simple Byte transmit */
uint16_t IRAM_ATTR SPI_Xfer(const uint16_t data)
{
	uint16_t retData;
	esp_err_t ret;
	spi_transaction_t t;
	 uint8_t *s = (uint8_t *) &t;
	 for (int n = sizeof(t); n; n--, s++) *s = 0;  //Zero out the transaction
	t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
	t.length=16;                     //Data is 16 bits
	t.tx_data[0] = data >> 8;
	t.tx_data[1] = data & 0xFF;
	//t.tx_buffer = &data;       //The data is the cmd itself
	ret=spi_device_polling_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);            //Should have had no issues.
	retData = (t.rx_data[0] << 8) | t.rx_data[1];
//	printf("TX: 0x%04X, RX: 0x%04X\r\n", data, retData);
	return retData;
}




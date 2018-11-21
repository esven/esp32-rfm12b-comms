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
			.max_transfer_sz=2,
			.flags= SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_NATIVE_PINS,
	};
	spi_device_interface_config_t devcfg={
			.clock_speed_hz=1*1000*1000,           //Clock out at 8 MHz
			.mode=0,                               //SPI mode 0
			.spics_io_num=RFM_CS_PIN,              //CS pin
			.queue_size=1,
	};
//	ret=spicommon_eriph_claim(VSPI_HOST);
	//Initialize the SPI bus
	ret=spi_bus_initialize(VSPI_HOST, &buscfg, 0);
	ESP_ERROR_CHECK(ret);
	//Attach the RFM12B to the SPI bus
	ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
	//Initialize non-SPI GPIOs
	gpio_set_direction(RFM_CS_PIN, GPIO_MODE_OUTPUT);
}


/* Simple Byte transmit */
uint16_t SPI_Xfer(const uint16_t data)
{
	uint16_t retData;
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));        //Zero out the transaction
	t.length=16;                     //Command is 8 bits
	t.tx_buffer = &data;       //The data is the cmd itself
	ret=spi_device_polling_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);            //Should have had no issues.
	retData = (t.rx_data[1] << 8) | t.rx_data[0];
	return retData;
}




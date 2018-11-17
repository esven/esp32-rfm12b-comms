/*
 * SPI driver for ESP32 family processors
 *
 * 2018 Sven Ebenfeld
 *
 */
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <string.h>

#include "spi.h"
#include "platform.h"

static spi_device_handle_t spi;

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
	};
	spi_device_interface_config_t devcfg={
			.clock_speed_hz=8*1000*1000,           //Clock out at 8 MHz
			.mode=0,                               //SPI mode 0
			.spics_io_num=RFM_CS_PIN,              //CS pin
	};
	//Initialize the SPI bus
	ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
	ESP_ERROR_CHECK(ret);
	//Attach the RFM12B to the SPI bus
	ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
	//Initialize non-SPI GPIOs
	gpio_set_direction(RFM_CS_PIN, GPIO_MODE_OUTPUT);
}


/* Simple Byte transmit */
uint16_t SPI_Xfer(uint16_t data)
{
	uint16_t retData;
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));        //Zero out the transaction
	t.length=16;                     //Command is 8 bits
	t.tx_data[0]= data & 0xFF;       //The data is the cmd itself
	t.tx_data[1]= data >> 8;         //The data is the cmd itself
	t.user=(void*)0;                 //D/C needs to be set to 0
	t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_RXDATA | SPI_TRANS_MODE_DIO;
	ret=spi_device_polling_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);            //Should have had no issues.
	retData = (t.rx_data[1] << 8) | t.rx_data[0];
	return retData;
}




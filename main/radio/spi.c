/*
 * SPI driver for ESP32 family processors
 *
 * 2018 Sven Ebenfeld
 *
 */
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <string.h>

#include "spi.h"
#include "platform.h"

typedef typeof(SPI1.clock) spi_clock_reg_t;

static spi_device_handle_t spi;
spi_dev_t *spi_hw;
static spi_clock_reg_t clk_reg;

void RFM_SPI_init(void)
{
	bool ret;
	uint32_t hw_flags;
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
	//Initialize non-SPI GPIOs
	gpio_set_direction(RFM_CS_PIN, GPIO_MODE_OUTPUT);
	ret=spicommon_periph_claim(VSPI_HOST);
	if (ret) {
		spicommon_bus_initialize_io(VSPI_HOST, &buscfg, 0,
				SPICOMMON_BUSFLAG_MASTER | buscfg.flags,
				&hw_flags);
		spi_hw=spicommon_hw_for_host(VSPI_HOST);
		spi_cal_clock(APB_CLK_FREQ, SPI_MASTER_FREQ_8M, 128, (uint32_t *) &clk_reg);
		spicommon_cs_initialize(VSPI_HOST, RFM_CS_PIN, 0, false);
		spi_hw->pin.master_ck_sel &= (1<<0);
		spi_hw->pin.master_cs_pol &= (1<<0);
		spi_hw->ctrl2.mosi_delay_mode = 0;
		spi_hw->ctrl2.mosi_delay_num = 0;
	}
}


/* Simple Byte transmit */
uint16_t IRAM_ATTR SPI_Xfer(const uint16_t data)
{
	uint16_t retData;
	uint8_t tmp[4];
	uint32_t dataBuffer;

	spi_hw->slave.trans_done = 0;
    //Configure clock settings
	spi_hw->clock.val = clk_reg.val;
	//Configure bit order
	spi_hw->ctrl.rd_bit_order= 0;
	spi_hw->ctrl.wr_bit_order= 0;
	//Configure polarity
	spi_hw->pin.ck_idle_edge=0;
	spi_hw->user.ck_out_edge=0;
	//Configure misc stuff
	spi_hw->user.doutdin= 1;
	spi_hw->user.sio= 0;
	//Configure CS pin and timing
	spi_hw->ctrl2.setup_time=-1;
	spi_hw->user.cs_setup= 0;
	//set hold_time to 0 will not actually append delay to CS
	//set it to 1 since we do need at least one clock of hold time in most cases
	spi_hw->ctrl2.hold_time=0;
	if (spi_hw->ctrl2.hold_time == 0)
		spi_hw->ctrl2.hold_time = 1;
	spi_hw->user.cs_hold=1;

	spi_hw->pin.cs0_dis = 0;
	spi_hw->pin.cs1_dis = 1;
	spi_hw->pin.cs2_dis = 1;
	//Reset DMA peripheral
	spi_hw->dma_conf.val |= SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
	spi_hw->dma_out_link.start=0;
	spi_hw->dma_in_link.start=0;
	spi_hw->dma_conf.val &= ~(SPI_OUT_RST|SPI_IN_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);
	spi_hw->dma_conf.out_data_burst_en=1;
	spi_hw->dma_conf.indscr_burst_en=1;
	spi_hw->dma_conf.outdscr_burst_en=1;
	//Set up QIO/DIO if needed
	spi_hw->ctrl.val &= ~(SPI_FREAD_DUAL|SPI_FREAD_QUAD|SPI_FREAD_DIO|SPI_FREAD_QIO);
	spi_hw->user.val &= ~(SPI_FWRITE_DUAL|SPI_FWRITE_QUAD|SPI_FWRITE_DIO|SPI_FWRITE_QIO);

	//SPI iface needs to be configured for a delay in some cases.
	//configure dummy bits
	spi_hw->user.usr_dummy = 0;
	spi_hw->user1.usr_dummy_cyclelen = 0;
	//if the data comes too late, delay half a SPI clock to improve reading
	spi_hw->ctrl2.miso_delay_num = 0;
	spi_hw->ctrl2.miso_delay_mode = 0;
	// Deactivate ADDR and CMD phase
	spi_hw->user1.usr_addr_bitlen = 0;
	spi_hw->user2.usr_command_bitlen = 0;
	spi_hw->user.usr_addr = 0;
	spi_hw->user.usr_command = 0;

	spi_hw->mosi_dlen.usr_mosi_dbitlen = 15;
	spi_hw->user.usr_mosi = 1;
	spi_hw->miso_dlen.usr_miso_dbitlen = 15;
	spi_hw->user.usr_miso = 1;

	tmp[0] = data >> 8;
	tmp[1] = data & 0xFF;
	memcpy(&dataBuffer, &tmp, 4);
	spi_hw->data_buf[0]=dataBuffer;

	//Kick off transfer
	spi_hw->cmd.usr=1;
	// Wait for transfer complete
	while (spi_hw->cmd.usr==1) {}
	// Read result
	dataBuffer = spi_hw->data_buf[0];
	memcpy(&tmp, &dataBuffer, 4);
	retData = (tmp[0] << 8) | tmp[1];
//	printf("TX: 0x%04X, RX: 0x%04X\r\n", data, retData);
	return retData;
}




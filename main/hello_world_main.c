/* Hello World Example

 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "radio/rfm12b.h"
#include "platform.h"

//void RCC_Configuration(void)
//{
//	SystemInit();
//	SysTick_Init();
//
//	// Enable GPIO modules
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_SYSCFG, ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);
//}

#define digitalPinToPort(pin)       (((pin)>31)?1:0)
#define digitalPinToBitMask(pin)    (1UL << (((pin)>31)?((pin)-32):(pin)))
#define digitalPinToTimer(pin)      (0)
#define analogInPinToBit(P)         (P)
#define portOutputRegister(port)    ((volatile uint32_t*)((port)?GPIO_OUT1_REG:GPIO_OUT_REG))
#define portInputRegister(port)     ((volatile uint32_t*)((port)?GPIO_IN1_REG:GPIO_IN_REG))
#define portModeRegister(port)      ((volatile uint32_t*)((port)?GPIO_ENABLE1_REG:GPIO_ENABLE_REG))

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

void GPIO_Configuration(void) {
	gpio_config_t gpioConfig;
	// GPI4 - RFM - IRQn
	gpioConfig.pin_bit_mask = GPIO_SEL_4;
	gpioConfig.mode = GPIO_MODE_INPUT;
	gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
	gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpioConfig.intr_type = GPIO_INTR_NEGEDGE;
	gpio_config(&gpioConfig);

	// GPIO17 - RFM Chip Select
	gpioConfig.pin_bit_mask = GPIO_SEL_17;
	gpioConfig.mode = GPIO_MODE_OUTPUT;
	gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
	gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpioConfig.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&gpioConfig);

	RFM_CS(Bit_SET);
}

void GPIO_RGBMatrix_Configuration(void) {
	gpio_config_t gpioConfig;

	// GPIO17 - RFM Chip Select
	gpioConfig.pin_bit_mask = GPIO_SEL_0 | GPIO_SEL_2 | GPIO_SEL_4 | GPIO_SEL_12
			| GPIO_SEL_13 | GPIO_SEL_14 | GPIO_SEL_15 | GPIO_SEL_16
			| GPIO_SEL_17 | GPIO_SEL_18 | GPIO_SEL_19 | GPIO_SEL_21
			| GPIO_SEL_22 | GPIO_SEL_23;
	gpioConfig.mode = GPIO_MODE_OUTPUT;
	gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
	gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpioConfig.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&gpioConfig);
	gpio_set_level(RGB_OE, 1);
	gpio_set_level(RGB_LAT, 0);
}

void IRAM_ATTR timer_group1_isr(void *para) {
	int timer_idx = (int) para;

	/* Retrieve the interrupt status and the counter value
	 from the timer that reported the interrupt */
	uint32_t intr_status = TIMERG1.int_st_timers.val;
	TIMERG1.hw_timer[timer_idx].update = 1;
	uint64_t timer_counter_value =
			((uint64_t) TIMERG1.hw_timer[timer_idx].cnt_high) << 32
					| TIMERG1.hw_timer[timer_idx].cnt_low;

	/* Clear the interrupt
	 and update the alarm time for the timer with without reload */
	if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
		;
		TIMERG1.int_clr_timers.t0 = 1;
	}

	/* After the alarm has been triggered
	 we need enable it again, so it is triggered the next time */
	TIMERG1.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

	/* Now just send the event data back to the main program task */
	static uint8_t color = 0;
	static uint8_t address = 0;
	static uint32_t counter = 0;
	gpio_set_level(RGB_OE, 1);
	for (int i = 0; i < 64; i++) {
		GPIO.out_w1tc = 0xEFF000;
		GPIO.out_w1tc = 0x10; // Reset Clock
		GPIO.out_w1ts = 0x0 | (((color + ( address %4 )+ i) & 0x7) << 17) | (((color + ( address %4 )+ i) & 0x7) << 21) | ((address & 0x0F) << 12); // Set R1 and R2
		GPIO.out_w1ts = 0x10; // Set Clock
	}
	counter++;
	if (counter % 150 == 0)
		color++;
	if (color > 7)
		color = 0;
	address++;
	if (address > 7)
		address = 0;
	GPIO.out_w1ts = 0x4; // Set LATCH
	gpio_set_level(RGB_OE, 0);
	GPIO.out_w1tc = 0x4; // Clear LATCH
}

void Timer_RGBMatrix_Configuration(void) {
	timer_config_t tim_config;
	tim_config.divider = TIMER_DIVIDER;
	tim_config.counter_dir = TIMER_COUNT_UP;
	tim_config.counter_en = TIMER_PAUSE;
	tim_config.alarm_en = true;
	tim_config.auto_reload = true;
	tim_config.intr_type = TIMER_INTR_LEVEL;

	timer_init(TIMER_GROUP_1, TIMER_0, &tim_config);
	/* Timer's counter will initially start from value below.
	 Also, if auto_reload is set, this value will be automatically reload on alarm */
	timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);
	/* Configure the alarm value and the interrupt on alarm. */
	timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 400 * TIMER_DIVIDER);
	timer_enable_intr(TIMER_GROUP_1, TIMER_0);
	timer_isr_register(TIMER_GROUP_1, TIMER_0, timer_group1_isr,
			(void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

	timer_start(TIMER_GROUP_1, TIMER_0);

}

void app_main() {
	uint8_t send_buffer[] = { 0xCA, 0xFE, 0xAF, 0xFE, 0xCA, 0xFF, 0xEE, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	printf("Hello world!\n");
	GPIO_RGBMatrix_Configuration();
	Timer_RGBMatrix_Configuration();

	/* Print chip information */
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
			(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	printf("silicon revision %d, ", chip_info.revision);

	printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ?
					"embedded" : "external");

	printf("Starting Loop!\n");
	while (1) {
//		gpio_set_level(RGB_OE, 1);
//		for (int i = 0; i < 24; i++) {
//			GPIO.out_w1tc = 0xFF620;
//			GPIO.out_w1tc = 0x10;
//			GPIO.out_w1ts = 0x0 | ((color & 0x7)  << 17) | ((color & 0x7)  << 14) | ((address & 0x01) << 5) | ((address & 0x0E) << 8); // Set R1 and R2
//			GPIO.out_w1ts = 0x10;
//		}
////		printf("GPIO: 0x%08x Color: 0x%04x Address: 0x%02x\r\n", GPIO.out, color, address);
//		counter++;
//		if (counter % 15000 == 0)
//			color++;
//		if (color > 7)
//			color = 1;
//		address++;
//		if (address > 7)
//			address = 0;
//		GPIO.out_w1ts = 0x4;
//		gpio_set_level(RGB_OE, 0);
//		GPIO.out_w1tc = 0x4;
//		vTaskDelay(8 / portTICK_PERIOD_MS);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		printf("Looping!\n");
	};
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	printf("Initializing RFM12b\n");
	RFM_Init();
	printf("Initializing RFM12b finished\n");

	while (1) {
		RFM_Send(2, send_buffer, sizeof(send_buffer));
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	for (int i = 10; i >= 0; i--) {
		printf("Restarting in %d seconds...\n", i);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	printf("Restarting now.\n");
	fflush(stdout);
	esp_restart();
}

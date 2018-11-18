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

void GPIO_Configuration(void)
{
	gpio_config_t gpioConfig;
	// GPI4 - RFM - IRQn
	gpioConfig.pin_bit_mask = GPIO_SEL_4;
	gpioConfig.mode         = GPIO_MODE_INPUT;
	gpioConfig.pull_up_en   = GPIO_PULLUP_DISABLE;
	gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpioConfig.intr_type    = GPIO_INTR_NEGEDGE;
	gpio_config(&gpioConfig);

	// GPIO17 - RFM Chip Select
	gpioConfig.pin_bit_mask = GPIO_SEL_17;
	gpioConfig.mode         = GPIO_MODE_OUTPUT;
	gpioConfig.pull_up_en   = GPIO_PULLUP_ENABLE;
	gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpioConfig.intr_type    = GPIO_INTR_DISABLE;
	gpio_config(&gpioConfig);

	RFM_CS(Bit_SET);
}

void app_main()
{
	uint8_t send_buffer[] = { 0xCA, 0xFE, 0xAF, 0xFE, 0xCA, 0xFF, 0xEE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    printf("Hello world!\n");
    GPIO_Configuration();

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

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

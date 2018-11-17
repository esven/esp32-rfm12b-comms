/*
 * platform.h
 *
 *  Created on: 14.11.2018
 *      Author: esven
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_

#define Bit_RESET	0
#define Bit_SET		1

//#define LED_ON Bit_RESET
//#define LED_OFF Bit_SET

//#define LED_GREEN_PIN      GPIO_Pin_5
//#define LED_GREEN_GPIO     GPIOA

#define RFM_CS_PIN         17

#define RFM_IRQ_PIN        4

#define PIN_NUM_MISO		19
#define PIN_NUM_MOSI		23
#define PIN_NUM_CLK			18

//#define RFM_RST_PIN        GPIO_Pin_9
//#define RFM_RST_GPIO       GPIOA
//
//#define RFM_INT_PIN        GPIO_Pin_3
//#define RFM_INT_GPIO       GPIOA

//#define LED_RED(x)         GPIO_WriteBit(LED_RED_GPIO, LED_RED_PIN, x)
//#define LED_GREEN(x)       GPIO_WriteBit(LED_GREEN_GPIO, LED_GREEN_PIN, x)
//#define LED_YELLOW(x)      GPIO_WriteBit(LED_YELLOW_GPIO, LED_YELLOW_PIN, x)

#define RFM_CS(x)          gpio_set_level(RFM_CS_PIN, x)
//#define RFM_RST(x)         GPIO_WriteBit(RFM_RST_GPIO, RFM_RST_PIN, x)
#define RFM_IRQ_READ()     gpio_get_level(RFM_IRQ_PIN)

#endif /* PLATFORM_H_ */
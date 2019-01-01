/*
 * platform.h
 *
 *  Created on: 14.11.2018
 *      Author: esven
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_

#define Bit_RESET			0
#define Bit_SET				1

//#define LED_ON Bit_RESET
//#define LED_OFF Bit_SET

//#define LED_GREEN_PIN      GPIO_Pin_5
//#define LED_GREEN_GPIO     GPIOA

#define RFM_CS_PIN			17
#define RFM_LED1_PIN		21
#define RFM_LED2_PIN		22
#define RFM_IRQ_PIN			4

#define PIN_NUM_MISO		19
#define PIN_NUM_MOSI		23
#define PIN_NUM_CLK			18

//#define RFM_RST_PIN		GPIO_Pin_9
//
//#define RFM_INT_PIN		GPIO_Pin_3

#define digitalPinToBitMask(pin)	(1U << (((pin)>31)?((pin)-32):(pin)))

#define RFM_CS_SET()		GPIO.out_w1ts = digitalPinToBitMask(RFM_CS_PIN);
#define RFM_CS_RST()		GPIO.out_w1tc = digitalPinToBitMask(RFM_CS_PIN);
//#define RFM_RST(x)        gpio_set_level(RFM_RST_PIN, x)
#define RFM_IRQ_READ()		gpio_get_level(RFM_IRQ_PIN)
#define RFM_LED1_ON()		gpio_set_level(RFM_LED1_PIN, Bit_RESET)
#define RFM_LED2_ON()		gpio_set_level(RFM_LED2_PIN, Bit_RESET)
#define RFM_LED1_OFF()		gpio_set_level(RFM_LED1_PIN, Bit_SET)
#define RFM_LED2_OFF()		gpio_set_level(RFM_LED2_PIN, Bit_SET)

#endif /* PLATFORM_H_ */

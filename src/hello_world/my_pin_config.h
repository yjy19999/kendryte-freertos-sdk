/**
* @par  Copyright (C): 2016-2022, Shenzhen Yahboom Tech
* @file         pin_config.c
* @author       Gengyue
* @version      V1.0
* @date         2020.05.27
* @brief        硬件引脚与软件GPIO的宏定义
* @details      
* @par History  见如下说明
*                 
* version:	由于K210使用fpioa现场可编程IO阵列，允许用户将255个内部功能映射到芯片外围的48个自由IO上
*           所以把硬件IO和软件GPIO功能抽出来单独设置，这样更容易理解。
*/
#ifndef _MY_PIN_CONFIG_H_
#define _MY_PIN_CONFIG_H_
/*****************************HEAR-FILE************************************/
#include <pin_cfg.h>
#include "main.h"
/*****************************HARDWARE-PIN*********************************/
// 硬件IO口，与原理图对应
// Two leds
#define PIN_LED_0             (0)
#define PIN_LED_1             (17)
// RGB
#define PIN_RGB_R             (6)
#define PIN_RGB_G             (7)
#define PIN_RGB_B             (8)
// Key
#define PIN_KEY               (16)
// Usb uart
#define PIN_UART_USB_RX       (4)
#define PIN_UART_USB_TX       (5)
// LCD
#define PIN_LCD_CS            (36)
#define PIN_LCD_RST           (37)
#define PIN_LCD_RS            (38)
#define PIN_LCD_WR            (39)

/*****************************FUNC-GPIO************************************/
// GPIO口的功能，绑定到软件IO口
#define FUNC_LED0             (LED0_GPIONUM + FUNC_GPIOHS0)
#define FUNC_LED1             (LED1_GPIONUM + FUNC_GPIOHS0)
#define FUNC_RGB_R            (FUNC_TIMER0_TOGGLE1)
#define FUNC_RGB_G            (FUNC_TIMER0_TOGGLE2)
#define FUNC_RGB_B            (FUNC_TIMER0_TOGGLE3)
#define FUNC_KEY              (5 + FUNC_GPIOHS0)
#define FUNC_USB_RX           FUNC_UART1_RX
#define FUNC_USB_TX           FUNC_UART1_TX
#define FUNC_LCD_CS           (FUNC_SPI0_SS3)
#define FUNC_LCD_RST          (FUNC_GPIOHS0 + LCD_RST_GPIONUM)
#define FUNC_LCD_RS           (FUNC_GPIOHS0 + LCD_RS_GPIONUM)
#define FUNC_LCD_WR           (FUNC_SPI0_SCLK)
// FUNC_UART1_TX
/*****************************struct for gpio*********************************/
// gpio struct
const fpioa_cfg_t g_fpioa_cfg =
{
    .version = PIN_CFG_VERSION,
    .functions_count = 12,
    .functions =
    {
        {PIN_LED_0, fpioa_function_t(FUNC_LED0)},
        {PIN_LED_1, fpioa_function_t(FUNC_LED1)},
        {PIN_RGB_R, fpioa_function_t(FUNC_RGB_R)},
        {PIN_RGB_G, fpioa_function_t(FUNC_RGB_G)},
        {PIN_RGB_B, fpioa_function_t(FUNC_RGB_B)},
        {PIN_UART_USB_RX, fpioa_function_t(FUNC_USB_RX)},
        {PIN_UART_USB_TX, fpioa_function_t(FUNC_USB_TX)},
        {PIN_KEY, fpioa_function_t(FUNC_KEY)},
        {PIN_LCD_CS, fpioa_function_t(FUNC_LCD_CS)},
        {PIN_LCD_RST, fpioa_function_t(FUNC_LCD_RST)},
        {PIN_LCD_RS, fpioa_function_t(FUNC_LCD_RS)},
        {PIN_LCD_WR, fpioa_function_t(FUNC_LCD_WR)}
    }
};

const pin_cfg_t g_pin_cfg =
{
    .version = PIN_CFG_VERSION,
    .set_spi0_dvp_data = 1
};

#endif /* _PIN_CONFIG_H_ */

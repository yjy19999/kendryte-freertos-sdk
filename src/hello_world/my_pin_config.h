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

/*****************************HARDWARE-PIN*********************************/
// 硬件IO口，与原理图对应
#define PIN_LED_0             (0)
#define PIN_LED_1             (17)

#define PIN_RGB_R             (6)
#define PIN_RGB_G             (7)
#define PIN_RGB_B             (8)

#define PIN_UART_USB_RX       (4)
#define PIN_UART_USB_TX       (5)
/*****************************FUNC-GPIO************************************/
// GPIO口的功能，绑定到软件IO口
#define FUNC_LED0             (0 + FUNC_GPIOHS0)
#define FUNC_LED1             (1 + FUNC_GPIOHS0)
#define FUNC_RGB_R            (2 + FUNC_GPIOHS0)
#define FUNC_RGB_G            (3 + FUNC_GPIOHS0)
#define FUNC_RGB_B            (4 + FUNC_GPIOHS0)
#define FUNC_USB_RX           FUNC_UART1_RX
#define FUNC_USB_TX           FUNC_UART1_TX
// FUNC_UART1_TX
/*****************************SOFTWARE-GPIO********************************/
// 软件GPIO口，与程序对应
#define LED0_GPIONUM          (0)
#define LED1_GPIONUM          (1)
#define RGB_R_GPIONUM         (2)
#define RGB_G_GPIONUM         (3)
#define RGB_B_GPIONUM         (4)
/*****************************struct for gpio*********************************/
// gpio struct
const fpioa_cfg_t g_fpioa_cfg =
{
    .version = PIN_CFG_VERSION,
    .functions_count = 7,
    .functions =
    {
        {PIN_LED_0, fpioa_function_t(FUNC_LED0)},
        {PIN_LED_1, fpioa_function_t(FUNC_LED1)},
        {PIN_RGB_R, fpioa_function_t(FUNC_RGB_R)},
        {PIN_RGB_G, fpioa_function_t(FUNC_RGB_G)},
        {PIN_RGB_B, fpioa_function_t(FUNC_RGB_B)},
        {PIN_UART_USB_RX, fpioa_function_t(FUNC_USB_RX)},
        {PIN_UART_USB_TX, fpioa_function_t(FUNC_USB_TX)}
    }
};
#endif /* _PIN_CONFIG_H_ */

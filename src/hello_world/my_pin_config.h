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

/*****************************FUNC-GPIO************************************/
// GPIO口的功能，绑定到软件IO口
#define FUNC_LED0             (0 + FUNC_GPIOHS0)
#define FUNC_LED1             (1 + FUNC_GPIOHS0)
/*****************************SOFTWARE-GPIO********************************/
// 软件GPIO口，与程序对应
#define LED0_GPIONUM          (0)
#define LED1_GPIONUM          (1)

/*****************************struct for gpio*********************************/
// gpio struct
const fpioa_cfg_t g_fpioa_cfg =
{
    .version = PIN_CFG_VERSION,
    .functions_count = 2,
    .functions =
    {
        {PIN_LED_0, FUNC_LED0},
        {PIN_LED_1, FUNC_LED1}
    }
};
#endif /* _PIN_CONFIG_H_ */

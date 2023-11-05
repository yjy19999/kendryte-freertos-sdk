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
// Uart radar
#define PIN_UART_RADAR1_RX    (20)
#define PIN_UART_RADAR1_TX    (21) 
#define PIN_UART_RADAR2_RX    (22)
#define PIN_UART_RADAR2_TX    (23)  
// LCD
#define PIN_LCD_CS            (36)
#define PIN_LCD_RST           (37)
#define PIN_LCD_RS            (38)
#define PIN_LCD_WR            (39)
// SD
#define PIN_TF_MISO            (26)
#define PIN_TF_CLK             (27)
#define PIN_TF_MOSI            (28)
#define PIN_TF_CS              (29)
// Camera
#define PIN_DVP_PCLK           (47)
#define PIN_DVP_XCLK           (46)
#define PIN_DVP_HSYNC          (45)
#define PIN_DVP_PWDN           (44)
#define PIN_DVP_VSYNC          (43)
#define PIN_DVP_RST            (42)
#define PIN_DVP_SCL            (41)
#define PIN_DVP_SDA            (40)

/*****************************FUNC-GPIO************************************/
// GPIO口的功能，绑定到软件IO口
#define FUNC_LED0             (LED0_GPIONUM + FUNC_GPIOHS0)
#define FUNC_LED1             (LED1_GPIONUM + FUNC_GPIOHS0)
#define FUNC_RGB_R            (FUNC_TIMER0_TOGGLE1)
#define FUNC_RGB_G            (FUNC_TIMER0_TOGGLE2)
#define FUNC_RGB_B            (FUNC_TIMER0_TOGGLE3)
#define FUNC_KEY              (5 + FUNC_GPIOHS0)
// #define FUNC_USB_RX           FUNC_UART1_RX
// #define FUNC_USB_TX           FUNC_UART1_TX
#define FUNC_USB_RX           FUNC_UARTHS_RX
#define FUNC_USB_TX           FUNC_UARTHS_TX
#define FUNC_RADAR1_RX        FUNC_UART2_RX
#define FUNC_RADAR1_TX        FUNC_UART2_TX  
#define FUNC_RADAR2_RX        FUNC_UART3_RX
#define FUNC_RADAR2_TX        FUNC_UART3_TX
#define FUNC_LCD_CS           (FUNC_SPI0_SS3)
#define FUNC_LCD_RST          (FUNC_GPIOHS0 + LCD_RST_GPIONUM)
#define FUNC_LCD_RS           (FUNC_GPIOHS0 + LCD_RS_GPIONUM)
#define FUNC_LCD_WR           (FUNC_SPI0_SCLK)
#define FUNC_TF_SPI_MISO      (FUNC_SPI1_D1)
#define FUNC_TF_SPI_CLK       (FUNC_SPI1_SCLK)
#define FUNC_TF_SPI_MOSI      (FUNC_SPI1_D0)
#define FUNC_TF_SPI_CS        (FUNC_GPIOHS0 + TF_CS_GPIONUM)
// FUNC_UART1_TX
/*****************************struct for gpio*********************************/
// gpio struct
const fpioa_cfg_t g_fpioa_cfg =
{
    .version = PIN_CFG_VERSION,
    .functions_count = 28,
    .functions =
    {
        {PIN_LED_0, fpioa_function_t(FUNC_LED0)},
        {PIN_LED_1, fpioa_function_t(FUNC_LED1)},
        {PIN_RGB_R, fpioa_function_t(FUNC_RGB_R)},
        {PIN_RGB_G, fpioa_function_t(FUNC_RGB_G)},
        {PIN_RGB_B, fpioa_function_t(FUNC_RGB_B)},
        {PIN_UART_USB_RX, fpioa_function_t(FUNC_USB_RX)},
        {PIN_UART_USB_TX, fpioa_function_t(FUNC_USB_TX)},
        {PIN_UART_RADAR1_RX, fpioa_function_t(FUNC_RADAR1_RX)},
        {PIN_UART_RADAR1_TX, fpioa_function_t(FUNC_RADAR1_TX)},
        {PIN_UART_RADAR2_RX, fpioa_function_t(FUNC_RADAR2_RX)},
        {PIN_UART_RADAR2_TX, fpioa_function_t(FUNC_RADAR2_TX)},
        {PIN_KEY, fpioa_function_t(FUNC_KEY)},
        {PIN_LCD_CS, fpioa_function_t(FUNC_LCD_CS)},
        {PIN_LCD_RST, fpioa_function_t(FUNC_LCD_RST)},
        {PIN_LCD_RS, fpioa_function_t(FUNC_LCD_RS)},
        {PIN_LCD_WR, fpioa_function_t(FUNC_LCD_WR)},
        {PIN_TF_MISO, fpioa_function_t(FUNC_TF_SPI_MISO)},
        {PIN_TF_CLK, fpioa_function_t(FUNC_TF_SPI_CLK)},
        {PIN_TF_MOSI, fpioa_function_t(FUNC_TF_SPI_MOSI)},
        {PIN_TF_CS, fpioa_function_t(FUNC_TF_SPI_CS)},
        {PIN_DVP_RST, fpioa_function_t(FUNC_CMOS_RST)},
        {PIN_DVP_PWDN, fpioa_function_t(FUNC_CMOS_PWDN)},
        {PIN_DVP_SCL, fpioa_function_t(FUNC_SCCB_SCLK)},
        {PIN_DVP_SDA, fpioa_function_t(FUNC_SCCB_SDA)},
        {PIN_DVP_XCLK, fpioa_function_t(FUNC_CMOS_XCLK)},
        {PIN_DVP_VSYNC, fpioa_function_t(FUNC_CMOS_VSYNC)},
        {PIN_DVP_HSYNC, fpioa_function_t(FUNC_CMOS_HREF)},
        {PIN_DVP_PCLK, fpioa_function_t(FUNC_CMOS_PCLK)}
    }
};

const pin_cfg_t g_pin_cfg =
{
    .version = PIN_CFG_VERSION,
    .set_spi0_dvp_data = 1
};

#endif /* _PIN_CONFIG_H_ */

#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include <devices.h>
#include <string.h>
#define PROCESSOR0_ID 0
#define PROCESSOR1_ID 1

extern handle_t gio;
extern handle_t uart1;
extern handle_t timer0;
extern handle_t timer1;
extern handle_t pwm_rgb;
extern handle_t spi_lcd;
extern handle_t spi_sd;
extern handle_t dma0;
extern handle_t sd0;
extern handle_t file_dvp;
extern handle_t spi_flash;
extern handle_t model_handle;
extern handle_t uart_radar1;
extern handle_t uart_radar2;
/*****************************SOFTWARE-GPIO********************************/
// 软件GPIO口，与程序对应
#define LED0_GPIONUM          (0)
#define LED1_GPIONUM          (1)
#define RGB_R_GPIONUM         (2)
#define RGB_G_GPIONUM         (3)
#define RGB_B_GPIONUM         (4)
#define KEY_GPIONUM           (5)
#define LCD_RST_GPIONUM       (6)
#define LCD_RS_GPIONUM        (7)
#define TF_CS_GPIONUM         (8)  
#endif
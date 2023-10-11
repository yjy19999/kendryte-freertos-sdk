/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "st7789.h"
#include "main.h"

#define WAIT_CYCLE      0U
#define SPI_HIGH_CLOCK_RATE 10000000U
#define SPI_LOW_CLOCK_RATE 10000000U

enum _instruction_length
{
    INSTRUCTION_LEN_0 = 0,
    INSTRUCTION_LEN_8 = 8,
    INSTRUCTION_LEN_16 = 16,
    INSTRUCTION_LEN_32 = 32,
} ;

enum _address_length
{
    ADDRESS_LEN_0 = 0,
    ADDRESS_LEN_8 = 8,
    ADDRESS_LEN_16 = 16,
    ADDRESS_LEN_32 = 32,
} ;

enum _frame_length
{
    FRAME_LEN_0 = 0,
    FRAME_LEN_8 = 8,
    FRAME_LEN_16 = 16,
    FRAME_LEN_32 = 32,
} ;

handle_t spi_dfs8;
handle_t spi_dfs16;
handle_t spi_dfs32;
/* 初始化RS引脚GPIO为输出模式 */
static void init_rs_gpio(void)
{
    gpio_set_drive_mode(gio, LCD_RS_GPIONUM, GPIO_DM_OUTPUT);
    gpio_set_pin_value(gio, LCD_RS_GPIONUM, GPIO_PV_HIGH);
}

/* 开始传输命令 */
static void set_start_cmd(void)
{
    gpio_set_pin_value(gio, LCD_RS_GPIONUM, GPIO_PV_LOW);
}

/* 开始传输数据 */
static void set_start_data(void)
{
    gpio_set_pin_value(gio, LCD_RS_GPIONUM, GPIO_PV_HIGH);
}

/* 初始化LCD复位引脚GPIO */
static void init_rst(void)
{
    gpio_set_drive_mode(gio, LCD_RST_GPIONUM, GPIO_DM_OUTPUT);
    gpio_set_pin_value(gio, LCD_RST_GPIONUM, GPIO_PV_HIGH);
}

/* 设置LCD-RST电平 */
static void set_rst(uint8_t val)
{
    gpio_set_pin_value(gio,LCD_RST_GPIONUM, val);
}

/* ST7789底层初始化 */
void tft_hard_init(void)
{
    init_rs_gpio();
    init_rst();
    set_rst(0);

    configASSERT(spi_lcd);
    spi_dfs8=spi_get_device(spi_lcd, SPI_MODE_0, SPI_FF_OCTAL, 1 << SPI_SLAVE_SELECT, FRAME_LEN_8);
    spi_dev_config_non_standard(spi_dfs8, INSTRUCTION_LEN_8, ADDRESS_LEN_0, WAIT_CYCLE, SPI_AITM_AS_FRAME_FORMAT);
    spi_dfs16 = spi_get_device(spi_lcd, SPI_MODE_0, SPI_FF_OCTAL, 1 << SPI_SLAVE_SELECT, FRAME_LEN_16);
    spi_dev_config_non_standard(spi_dfs16, INSTRUCTION_LEN_16, ADDRESS_LEN_0, WAIT_CYCLE, SPI_AITM_AS_FRAME_FORMAT);   
    spi_dfs32 = spi_get_device(spi_lcd, SPI_MODE_0, SPI_FF_OCTAL, 1 << SPI_SLAVE_SELECT, FRAME_LEN_32);
    spi_dev_config_non_standard(spi_dfs32, INSTRUCTION_LEN_0, ADDRESS_LEN_32, WAIT_CYCLE, SPI_AITM_AS_FRAME_FORMAT);    
    
    spi_dev_set_clock_rate(spi_dfs8, SPI_LOW_CLOCK_RATE);
    spi_dev_set_clock_rate(spi_dfs16, SPI_HIGH_CLOCK_RATE);
    spi_dev_set_clock_rate(spi_dfs32, SPI_HIGH_CLOCK_RATE);
    set_rst(1);
}

/* SPI写命令 */
void tft_write_command(uint8_t cmd)
{
    set_start_cmd();
    io_write(spi_dfs8, (const uint8_t *)(&cmd), 1);
    
}

/* SPI写数据（uint8_t类型） */
void tft_write_byte(uint8_t *data_buf, uint32_t length)
{
    set_start_data();
    io_write(spi_dfs8, (const uint8_t *)(data_buf), length);
}

/* SPI写数据（uint16_t类型） */
void tft_write_half(uint16_t *data_buf, uint32_t length)
{
    set_start_data();
    io_write(spi_dfs16, (const uint8_t *)(data_buf), length * 2);
}

/* SPI写数据（uint32_t类型） */
void tft_write_word(uint32_t *data_buf, uint32_t length, uint32_t flag)
{
    set_start_data();
    io_write(spi_dfs32, (const uint8_t *)data_buf, length * 4);
}

/* 以相同的数据填充 */
void tft_fill_data(uint32_t *data_buf, uint32_t length)
{
    set_start_data();
    spi_dev_fill(spi_dfs32, 0, *data_buf, *data_buf, length - 1);
}

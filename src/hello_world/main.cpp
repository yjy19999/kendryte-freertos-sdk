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

// CMake command:
// cmake .. -DPROJ=hello_world -G "MinGW Makefiles"
// C:\Users\yjy19\Desktop\K210\kendryte-toolchain\bin\make.exe
// C:\msys64\mingw64\bin\mingw32-make.exe

// Settings for freertos https://blog.csdn.net/kunkliu/article/details/125890955

#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
// #include <cstring>
#include <queue>
#include <deque>
#include <complex>
#include <filesystem.h>
#include <storage/sdcard.h>
#include <sleep.h>
#include "my_pin_config.h"
#include "main.h"
extern "C"
{
#include "sysctl.h"
#include "lcd.h"
#include "hal.h"
#include "ov2640.h"
// #include "ov9655.h"
#include "dvp.h"
#include "dvp_cam.h"
}

// handle for devices
handle_t gio;
handle_t uart1;
handle_t timer1;
handle_t pwm_rgb;
handle_t spi_lcd;
handle_t spi_sd;
handle_t dma0;
handle_t sd0;
handle_t file_dvp;

std::deque<uint8_t> queue_usb;
int frame_count = 0;
const int frame_length = 10;
uint8_t data_buf[frame_length];
std::complex<float> recv_data[4][frame_length / 4];
extern volatile bool g_dvp_finish_flag;
void key_isr();
void on_tick_timer1();

void init_rgb_pwm()
{
    configASSERT(pwm_rgb);
    pwm_set_frequency(pwm_rgb, 2000);
    pwm_set_active_duty_cycle_percentage(pwm_rgb, 0, 0);
    pwm_set_active_duty_cycle_percentage(pwm_rgb, 1, 0);
    pwm_set_active_duty_cycle_percentage(pwm_rgb, 2, 0);
    pwm_set_enable(pwm_rgb, 0, true);
    pwm_set_enable(pwm_rgb, 1, true);
    pwm_set_enable(pwm_rgb, 2, true);
}

void init_rgb_gpio(void)
{
    // 设置RGB灯的GPIO模式为输出
    gpio_set_drive_mode(gio, RGB_R_GPIONUM, GPIO_DM_OUTPUT);
    gpio_set_drive_mode(gio, RGB_G_GPIONUM, GPIO_DM_OUTPUT);
    gpio_set_drive_mode(gio, RGB_B_GPIONUM, GPIO_DM_OUTPUT);
    gpio_set_pin_value(gio, RGB_R_GPIONUM, GPIO_PV_HIGH);
    gpio_set_pin_value(gio, RGB_G_GPIONUM, GPIO_PV_HIGH);
    gpio_set_pin_value(gio, RGB_B_GPIONUM, GPIO_PV_HIGH);
}

void deinit_rgb_gpio(void)
{
    gpio_set_drive_mode(gio, RGB_R_GPIONUM, GPIO_DM_INPUT);
    gpio_set_drive_mode(gio, RGB_G_GPIONUM, GPIO_DM_INPUT);
    gpio_set_drive_mode(gio, RGB_B_GPIONUM, GPIO_DM_INPUT);
}

void init_led(void)
{
    gpio_set_drive_mode(gio, LED0_GPIONUM, GPIO_DM_OUTPUT);
    gpio_set_drive_mode(gio, LED1_GPIONUM, GPIO_DM_OUTPUT);
    gpio_set_pin_value(gio, LED0_GPIONUM, GPIO_PV_HIGH);
    gpio_set_pin_value(gio, LED1_GPIONUM, GPIO_PV_HIGH);
}

void deinit_led(void)
{
    gpio_set_drive_mode(gio, LED0_GPIONUM, GPIO_DM_INPUT);
    gpio_set_drive_mode(gio, LED1_GPIONUM, GPIO_DM_INPUT);
}

void init_uart(handle_t uart)
{
    uart_config(uart, 115200, 8, UART_STOP_1, UART_PARITY_NONE);
    uart_set_read_timeout(uart, 10 * 1000);
}

void init_key(void)
{
    gpio_set_drive_mode(gio, KEY_GPIONUM, GPIO_DM_INPUT_PULL_UP);
    gpio_set_pin_edge(gio, KEY_GPIONUM, GPIO_PE_BOTH);
    gpio_set_on_changed(gio, KEY_GPIONUM, (gpio_on_changed_t)key_isr, NULL);
}

void init_timer(handle_t timer, size_t interval, timer_on_tick_t on_tick_fun)
{
    timer_set_interval(timer, interval);
    timer_set_on_tick(timer, (timer_on_tick_t)on_tick_fun, NULL);
    timer_set_enable(timer, true);
}

void dma_transfer_data(int *src, int *dest, size_t data_len)
{
    size_t burst_size = 4;
    handle_t dma = dma_open_free();
    dma_transmit(dma, src, dest, true, true, sizeof(int), data_len, burst_size);
    if (dest[0] != src[0])
    {
        lcd_draw_string(16, 40, const_cast<char *>("warning dma transfer failed"), (uint16_t)RED);
    }
    dma_close(dma);
}

void init_sdcard(void)
{
    // SHOW_DEBUG(16,60,"run_here_0");
    handle_t sd0 = spi_sdcard_driver_install(spi_sd, gio, TF_CS_GPIONUM);
    io_close(spi_sd);
    // SHOW_DEBUG(16,80,"run_here_1");
    configASSERT(sd0);
    // SHOW_DEBUG(16,100,"run_here_2");
    usleep(200*1000);
    configASSERT(filesystem_mount("fs/0/", sd0) == 0);
    // int res=filesystem_mount("fs/0/", sd0);
    // SHOW_DEBUG(16,120,std::string(res));
    // SHOW_DEBUG(16,140,"run_here_3"); 
    // io_close(sd0);
}

void init_camera(void)
{
    dvp_init();
    ov2640_init();
}

void find_sdcard()
{
    // print all file name
    find_find_data_t find_data;
    handle_t find = filesystem_find_first("/fs/0/", "*", &find_data);
    configASSERT(find);
    uint16_t show_y = 40;
    do
    {
        // printf("%s\n", find_data.filename);
        lcd_draw_string(16, show_y, const_cast<char *>(find_data.filename), (uint16_t)RED);
        show_y += 10;
    } while (filesystem_find_next(find, &find_data));
    filesystem_file_close(find);

    // printf("Done\n");
}

void rgb_gpio_pool(void)
{
    static int val = 0;
    val++;
    val %= 3;
    switch (val)
    {
    case 0:
        gpio_set_pin_value(gio, RGB_R_GPIONUM, GPIO_PV_LOW);
        gpio_set_pin_value(gio, RGB_G_GPIONUM, GPIO_PV_HIGH);
        gpio_set_pin_value(gio, RGB_B_GPIONUM, GPIO_PV_HIGH);
        break;
    case 1:
        gpio_set_pin_value(gio, RGB_R_GPIONUM, GPIO_PV_HIGH);
        gpio_set_pin_value(gio, RGB_G_GPIONUM, GPIO_PV_LOW);
        gpio_set_pin_value(gio, RGB_B_GPIONUM, GPIO_PV_HIGH);
        break;
    case 2:
        gpio_set_pin_value(gio, RGB_R_GPIONUM, GPIO_PV_HIGH);
        gpio_set_pin_value(gio, RGB_G_GPIONUM, GPIO_PV_HIGH);
        gpio_set_pin_value(gio, RGB_B_GPIONUM, GPIO_PV_LOW);
    default:
        break;
    }
}

void rgb_pwm_pool(void)
{
    static double percent[3] = {0, 1, 1};
    bool flag;
    for (auto i : percent)
    {
        if (i > 1.0)
        {
            flag = true;
            i = 1.0;
        }
        else if (i < 0.0)
        {
            flag = false;
            i = 0.0;
        }
        flag ? (i -= 0.01) : (i += 0.01);
    }
    // printf("%f\t%f\t%f\n",percent[0],percent[1],percent[2]);
    for (int i = 0; i < 3; i++)
    {
        pwm_set_active_duty_cycle_percentage(pwm_rgb, i, percent[i]);
    }
}

void show_proc_id(void)
{
    int proc_id = 0;
    proc_id = static_cast<int>(uxTaskGetProcessorId());
    // printf("processor for task4 is %d\n", proc_id);
}

void key_isr()
{
    int val = 0;
    val = gpio_get_pin_value(gio, LED1_GPIONUM);
    gpio_set_pin_value(gio, LED1_GPIONUM, gpio_pin_value_t(val = !val));
}

void on_tick_timer1()
{
    ;
}

void vTask1()
{
    show_proc_id();
    while (1)
    {
        static int val0 = 0;
        gpio_set_pin_value(gio, LED0_GPIONUM, gpio_pin_value_t(val0 = !val0));
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void vTask2()
{
    show_proc_id();
    while (1)
    {
        static int val1 = 0;
        gpio_set_pin_value(gio, LED1_GPIONUM, gpio_pin_value_t(val1 = !val1));
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void vTask3()
{
    show_proc_id();
    while (1)
    {
        rgb_pwm_pool();
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void vTask4()
{
    // uint8_t recv[30]={0};
    bool recv_success = false;
    uint8_t recv = 0;
    const char *hel = {"hello uart!\n"};
    io_write(uart1, (uint8_t *)hel, strlen(hel));
    show_proc_id();

    while (1)
    {
        // send recv data back
        // if(io_read(uart1, recv, 10) < 0)
        //     printf("time out \n");
        while (io_read(uart1, &recv, 1) != 1)
            ;
        queue_usb.push_back(recv);
        // io_write(uart1, recv, 10);
        if (queue_usb.size() > frame_length + 3)
        {
            if (queue_usb.front() != 0x56)
            {
                queue_usb.pop_front();
            }
            else
            {
                if (queue_usb[1] == 0x30 && queue_usb[2] == 0x3A)
                {
                    while (queue_usb.size() < frame_length)
                    {
                        while (io_read(uart1, &recv, 1) != 1)
                            ;
                        queue_usb.push_back(recv);
                    }
                    std::copy(queue_usb.begin(), queue_usb.begin() + frame_length, data_buf);
                    for (int i = 0; i < frame_length; i++)
                    {
                        queue_usb.pop_front();
                    }
                    recv_success = true;
                }
            }
        }
        if (recv_success)
        {
            int idx_begin[4];
            int idx_end[4];
            // idx_end=frame_length*(i+1)/4-4;
            for (int i = 0; i < 4; i++)
            {
                int save_idx = 0;
                idx_begin[i] = frame_length * i / 4 + 3;
                idx_end[i] = frame_length * (i + 1) / 4 - 4;
                for (int j = idx_begin[i]; j < idx_end[i]; j += 4)
                {
                    recv_data[i][save_idx] = std::complex<double>(data_buf[j] + data_buf[j + 1] * 255, data_buf[j + 2] + data_buf[j + 3] * 255);
                    save_idx++;
                }
            }
            frame_count++;
        }
        // uint8_t front_data = 0;
        // if (usb_recv_len > 10)
        // {
        //     for (int i = 0; i < 10; i++)
        //     {
        //         front_data = queue_usb.front();
        //         io_write(uart1, &front_data, 1);
        //         queue_usb.pop();
        //     }
        // }
        // io_write(uart1, (uint8_t *)hel, strlen(hel));

        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void vTask5()
{
    while (1)
    {
        while (g_dvp_finish_flag == false)
            ;
        g_dvp_finish_flag = false;
        lcd_draw_picture(0, 0, 320, 240, gram_mux ? lcd_gram1 : lcd_gram0);
        gram_mux ^= 0x01;
        const char *sig = {"vTask5run\n"};
        io_write(uart1,(uint8_t *)sig,strlen(sig));
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

int main()
{
    // set frequency to 400Mhz
    system_set_cpu_frequency(400000000);

    gio = io_open("/dev/gpio0");
    configASSERT(gio);
    uart1 = io_open("/dev/uart1");
    configASSERT(uart1);
    timer1 = io_open("/dev/timer1");
    configASSERT(timer1);
    pwm_rgb = io_open("/dev/pwm0");
    configASSERT(pwm_rgb);
    spi_lcd = io_open("/dev/spi0");
    configASSERT(spi_lcd);
    spi_sd = io_open("/dev/spi1");
    configASSERT(spi_sd);
    file_dvp = io_open("/dev/dvp0");
    configASSERT(file_dvp);
    dma0 = dma_open_free();
    configASSERT(dma0);

    // init_rgb();
    init_led();
    usleep(10*1000);

    init_uart(uart1);
    usleep(10*1000);
    
    // init_timer(timer0,10*1e6,NULL);
    init_rgb_pwm();
    usleep(10*1000);

    sysctl_set_spi0_dvp_data(1);
    sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
    lcd_init();
    usleep(10*1000);

    lcd_clear(GREEN);

    lcd_draw_picture_half(0, 0, 320, 240, (uint16_t *)gImage_logo);

    lcd_draw_string(16, 40, const_cast<char *>("Hello World!"), (uint16_t)RED);
    // init_timer(timer1);
    usleep(100*1000);
    // init_sdcard();
    lcd_draw_string(16, 60, const_cast<char *>("LCD OK!"), (uint16_t)RED);
    // test sd card
    usleep(20*1000);
    // find_sdcard();
    usleep(100*1000);
    dvp_init();
    ov2640_init();
    lcd_draw_string(16, 80, const_cast<char *>("Camera OK!"), (uint16_t)RED);
    usleep(100*1000);
    vTaskSuspendAll();
    xTaskCreate(TaskFunction_t(vTask1), "vTask1", 512, NULL, 3, NULL);
    // xTaskCreate(TaskFunction_t(vTask2), "vTask2", 128, NULL, 2, NULL);
    xTaskCreateAtProcessor(PROCESSOR0_ID, TaskFunction_t(vTask3), "vTask3", 128, NULL, 5, NULL);
    // xTaskCreate(TaskFunction_t(vTask4), "vTask4", 128, NULL, 4, NULL);
    // xTaskCreateAtProcessor(PROCESSOR1_ID, TaskFunction_t(vTask1), "vTask1", 512, NULL, 3, NULL);
    xTaskCreateAtProcessor(PROCESSOR1_ID, TaskFunction_t(vTask5), "vTask5", 512, NULL, 5, NULL);

    if (!xTaskResumeAll())
    {
        taskYIELD();
    }

    const char *sig = {"run here!\n"};
    io_write(uart1, (uint8_t *)sig, strlen(sig));
    while (1)
        ;
}

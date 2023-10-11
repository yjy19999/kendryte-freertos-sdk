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
#include <queue>
#include <deque>
#include <complex>

#include "my_pin_config.h"
#include "main.h"
extern "C"
{
    #include "sysctl.h"
    #include "lcd.h"
}
// handle for devices
handle_t gio;
handle_t uart1;
handle_t timer1;
handle_t pwm_rgb;
handle_t spi_lcd;

std::deque<uint8_t> queue_usb;
int frame_count = 0;
const int frame_length = 10;
uint8_t data_buf[frame_length];
std::complex<float> recv_data[4][frame_length / 4];

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

void init_timer(handle_t timer,size_t interval,timer_on_tick_t on_tick_fun)
{
    timer_set_interval(timer, interval);
    timer_set_on_tick(timer, (timer_on_tick_t)on_tick_fun, NULL);
    timer_set_enable(timer, true);
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
    static double percent[3]={0,1,1};
    bool flag;
    for(auto i:percent)
    {
        if(i>1.0)
        {
            flag=true;
            i=1.0;
        }
        else if(i<0.0)
        {
            flag=false;
            i=0.0;
        }        
        flag ? (i-=0.01):(i+=0.01);     
    }
    // printf("%f\t%f\t%f\n",percent[0],percent[1],percent[2]);
    for(int i=0;i<3;i++)
    {
        pwm_set_active_duty_cycle_percentage(pwm_rgb, i, percent[i]);        
    }      
}

void show_proc_id(void)
{
    int proc_id = 0;
    proc_id = static_cast<int>(uxTaskGetProcessorId());
    printf("processor for task4 is %d\n", proc_id);
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

int main()
{
    gio = io_open("/dev/gpio0");
    configASSERT(gio);
    uart1 = io_open("/dev/uart1");
    configASSERT(uart1);
    timer1 = io_open("/dev/timer1");
    configASSERT(timer1);
    pwm_rgb = io_open("/dev/pwm0");
    configASSERT(pwm_rgb);
    spi_lcd=io_open("/dev/spi0");
    configASSERT(spi_lcd);

    // init_rgb();
    init_led();
    init_uart(uart1);
    
    // init_timer(timer0,10*1e6,NULL);
    init_rgb_pwm();

    sysctl_set_spi0_dvp_data(1);
    sysctl_set_power_mode(SYSCTL_POWER_BANK6,SYSCTL_POWER_V18);    
    lcd_init();

    
    usleep(100000);
    lcd_clear(GREEN);
    
    lcd_draw_picture_half(0, 0, 320, 240, (uint16_t*)gImage_logo);

    lcd_draw_string(16,40,const_cast<char*>("Hello World!"),(uint16_t)RED);
    // init_timer(timer1);

    vTaskSuspendAll();
    xTaskCreate(TaskFunction_t(vTask1), "vTask1", 512, NULL, 3, NULL);
    // xTaskCreate(TaskFunction_t(vTask2), "vTask2", 128, NULL, 2, NULL);
    xTaskCreateAtProcessor(PROCESSOR0_ID, TaskFunction_t(vTask3), "vTask3", 128, NULL, 5, NULL);
    // xTaskCreate(TaskFunction_t(vTask4), "vTask4", 128, NULL, 4, NULL);
    // xTaskCreateAtProcessor(PROCESSOR1_ID, TaskFunction_t(vTask1), "vTask1", 512, NULL, 3, NULL);
    if (!xTaskResumeAll())
    {
        taskYIELD();
    }

    const char *sig = {"run here!\n"};
    io_write(uart1, (uint8_t *)sig, strlen(sig));
    while (1)
        ;
}

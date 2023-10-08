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
#include <devices.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include "my_pin_config.h"
#include "main.h"
#include <string.h>
#include <queue>

// handle for devices
handle_t gio;
handle_t uart1;
std::queue<uint8_t> queue_usb;

void init_rgb(void)
{
    // 设置RGB灯的GPIO模式为输出
    gpio_set_drive_mode(gio, RGB_R_GPIONUM, GPIO_DM_OUTPUT);
    gpio_set_drive_mode(gio, RGB_G_GPIONUM, GPIO_DM_OUTPUT);
    gpio_set_drive_mode(gio, RGB_B_GPIONUM, GPIO_DM_OUTPUT);
    gpio_set_pin_value(gio, RGB_R_GPIONUM, GPIO_PV_HIGH);
    gpio_set_pin_value(gio, RGB_G_GPIONUM, GPIO_PV_HIGH);
    gpio_set_pin_value(gio, RGB_B_GPIONUM, GPIO_PV_HIGH);
}

void deinit_rgb(void)
{
    gpio_set_drive_mode(gio, RGB_R_GPIONUM, GPIO_DM_INPUT);
    gpio_set_drive_mode(gio, RGB_G_GPIONUM, GPIO_DM_INPUT);
    gpio_set_drive_mode(gio, RGB_B_GPIONUM, GPIO_DM_INPUT);
}

void rgb_blink_pool(void)
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

void init_uart(void)
{
    uart_config(uart1, 115200, 8, UART_STOP_1, UART_PARITY_NONE);
    uart_set_read_timeout(uart1, 10 * 1000);
}

void show_proc_id(void)
{
    int proc_id = 0;
    proc_id = static_cast<int>(uxTaskGetProcessorId());
    printf("processor for task4 is %d\n", proc_id);
}

void vTask1()
{
    show_proc_id();
    while (1)
    {
        static int val = 0;
        gpio_set_pin_value(gio, LED0_GPIONUM, gpio_pin_value_t(val = !val));
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void vTask2()
{
    show_proc_id();
    while (1)
    {
        static int val = 0;
        gpio_set_pin_value(gio, LED1_GPIONUM, gpio_pin_value_t(val = !val));
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void vTask3()
{
    show_proc_id();
    while (1)
    {
        rgb_blink_pool();
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void vTask4()
{
    // uint8_t recv[30]={0};
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
        queue_usb.push(recv);
        // io_write(uart1, recv, 10);
        int usb_recv_len = queue_usb.size();
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

    // init_rgb();
    // init_led();
    init_uart();

    vTaskSuspendAll();
    xTaskCreate(TaskFunction_t(vTask1), "vTask1", 512, NULL, 3, NULL);
    // xTaskCreate(TaskFunction_t(vTask2), "vTask2", 128, NULL, 2, NULL);
    // xTaskCreate(TaskFunction_t(vTask3), "vTask3", 128, NULL, 2, NULL);
    xTaskCreate(TaskFunction_t(vTask4), "vTask4", 128, NULL, 4, NULL);
    xTaskCreateAtProcessor(PROCESSOR1_ID, TaskFunction_t(vTask1), "vTask1", 512, NULL, 3, NULL);
    if (!xTaskResumeAll())
    {
        taskYIELD();
    }

    while (1)
        ;
}

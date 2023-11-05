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
#include "semphr.h"
#include <task.h>
#include <string.h>
// #include <cstring>
#include <queue>
#include <deque>
#include <complex>
#include <filesystem.h>
#include <storage/sdcard.h>
#include <sleep.h>
#include <stdio.h>
#include <algorithm>
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
    #include "image_process.h"
    #include "w25qxx.h"
    #include "incbin.h"
    #include "fft_soft.h"
    #include "region_layer.h"
}

#define FFT_N               512U
/**
 * FFT 16-bit registers lead to data overflow (-32768-32767),
 * FFT has nine layers, shift determines which layer needs shift operation to prevent overflow.
 * (for example, 0x1ff means 9 layers do shift operation; 0x03 means the first layer and the second layer do shift operation) 
 * If shifted, the result is different from the normal FFT.
 */
#define FFT_FORWARD_SHIFT   0x0U
#define FFT_BACKWARD_SHIFT  0x1ffU
#define PI                  3.14159265358979323846

#define RX_ANTENNA_SPACE 2.35e-3
#define TX_ANTENNA_SPACE 5.54e-3
#define FFT_SAMPLE 151
#define CHIRP 1
#define TX_NUM 1
#define RX_NUM 4
#define IQ_NUM 2
#define HEADER_SIZE 4
#define END_SIZE 4

const int RADAR_FRAME_LENGTH=(FFT_SAMPLE*CHIRP*2*IQ_NUM+IQ_NUM+HEADER_SIZE+END_SIZE)*TX_NUM*RX_NUM;

enum RecvState
{
    RECV_NULL,
    RECV_START,
    RECV_HEAD,
    RECV_GET_SIZE
};

typedef struct _complex_hard
{
    int16_t real;
    int16_t imag;
} complex_hard_t;


// handle for devices
handle_t gio;
handle_t uart1;
handle_t timer0;
handle_t timer1;
handle_t pwm_rgb;
handle_t spi_lcd;
handle_t spi_sd;
handle_t dma0;
handle_t sd0;
handle_t file_dvp;
handle_t spi_flash;
handle_t model_handle;

handle_t uart_radar1;
handle_t uart_radar2;

std::deque<uint8_t> queue_usb_0;
std::deque<uint8_t> queue_usb_1;
std::deque<uint8_t> *pqueue_usb;
// std::deque<uint8_t> *pqueue_usb_back;
bool recv_success = false;
bool recv_copy = false;
const int read_size=512;
uint8_t recv0[read_size+5]={0};
// uint8_t recv1[read_size+5]={0};
uint8_t* precv;
// uint8_t* precv_back;

extern camera_context_t camera_ctx;
int frame_count = 0;
// const int frame_length = 10;
// uint8_t data_buf[frame_length];
std::complex<float> recv_data[4][RADAR_FRAME_LENGTH / 4];

fft_data_t *input_data;
fft_data_t *output_data;

SemaphoreHandle_t xMutexUsb = xSemaphoreCreateMutex();
SemaphoreHandle_t xMutexCopy = xSemaphoreCreateMutex();
SemaphoreHandle_t xMutexPrint = xSemaphoreCreateMutex();

static region_layer_t face_detect_rl;
static obj_info_t face_detect_info;
#define ANCHOR_NUM 5
static float anchor[ANCHOR_NUM * 2] = {1.889,2.5245,  2.9465,3.94056, 3.99987,5.3658, 5.155437,6.92275, 6.718375,9.01025};

#define LOAD_KMODEL_FROM_FLASH 1

#if LOAD_KMODEL_FROM_FLASH
// handle_t spi3;
#define KMODEL_SIZE (380 * 1024)
uint8_t *model_data;
#else
INCBIN(model, "../src/hello_world/model.kmodel");
#endif

void key_isr();
void on_tick_timer1();

void hardware_init(void)
{
    // set frequency to 400Mhz
    system_set_cpu_frequency(400000000);

    gio = io_open("/dev/gpio0");
    configASSERT(gio);
    // uart1 = io_open("/dev/uart1");
    // configASSERT(uart1);
    uart_radar1 = io_open("/dev/uart2");
    configASSERT(uart_radar1);
    uart_radar2 = io_open("/dev/uart3");
    configASSERT(uart_radar2);
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
    spi_flash = io_open("/dev/spi3");
    configASSERT(spi_flash);
}

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

void init_uart(handle_t uart,uint32_t baud,uint32_t timeout)
{
    uart_config(uart, baud, 8, UART_STOP_1, UART_PARITY_NONE);
    uart_set_read_timeout(uart, timeout);
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
    sd0 = spi_sdcard_driver_install(spi_sd, gio, TF_CS_GPIONUM);
    // io_close(spi_sd);
    // SHOW_DEBUG(16,80,"run_here_1");
    configASSERT(sd0);
    // SHOW_DEBUG(16,100,"run_here_2");
    usleep(200*1000);
    configASSERT(filesystem_mount("/fs/0:", sd0) == 0);
    char fs_mount_ok[]="fs mount ok\n";
    // io_write(uart1,(uint8_t *)fs_mount_ok,strlen(fs_mount_ok));
    // int res=filesystem_mount("fs/0/", sd0);
    // SHOW_DEBUG(16,120,std::string(res));
    // SHOW_DEBUG(16,140,"run_here_3"); 
    // io_close(sd0);
}

void init_camera(void)
{
    dvp_init(&camera_ctx);
    ov2640_init();
}

void find_sdcard()
{
    // print all file name
    find_find_data_t find_data;
    handle_t find = filesystem_find_first("/fs//0:/model", "*", &find_data);
    // configASSERT(find);
    // uint16_t show_y = 40;
    do
    {
        // printf("%s\n", find_data.filename);
        // lcd_draw_string(16, show_y, const_cast<char *>(find_data.filename), (uint16_t)RED);
        
        // show_y += 10;
        // io_write(uart1,(uint8_t *)(find_data.filename),strlen(find_data.filename));
        usleep(100*1000);
    } while (filesystem_find_next(find, &find_data));
    filesystem_file_close(find);

    char done_str[]="Done";
    // io_write(uart1,(uint8_t *)done_str,strlen(done_str));
    // printf("Done\n");
}

void test_write_sdcard(void)
{
    handle_t file;
    do
    {
        file=filesystem_file_open("/fs/model.txt",FILE_ACCESS_READ_WRITE,FILE_MODE_OPEN_ALWAYS);
        // char try_str[]="try open file\n";
        // io_write(uart1,(uint8_t*)try_str,strlen(try_str));
        usleep(100*1000);
    } while (file==NULL_HANDLE);

    if(file==NULL_HANDLE)
    {
        char fail_str[]="open file failed\n";
        // io_write(uart1,(uint8_t*)fail_str,strlen(fail_str));
    }

    char msg[]="k233333333333333333";
    char buffer[1000];
    filesystem_file_write(file, (const uint8_t *)msg, strlen(msg)+1);
    usleep(100*1000);
    filesystem_file_flush(file);
    filesystem_file_set_position(file, 0);
    filesystem_file_read(file, (uint8_t *)buffer, strlen(msg)+1);
    // io_write(uart1,(uint8_t*)buffer,strlen(buffer));
    usleep(100*1000);
    filesystem_file_close(file);
}

handle_t kmodel_get(const char* model_path)
{

    handle_t model_file;
    handle_t model_context; 
    
    int try_time=5;
    do
    {
        model_file=filesystem_file_open(model_path,FILE_ACCESS_READ,FILE_MODE_OPEN_EXISTING);
        usleep(100*1000);
    } while (try_time--);
    
    if(model_file==NULL_HANDLE)
    {
        char fail_str[]="open file failed\n";
        // io_write(uart1,(uint8_t*)fail_str,strlen(fail_str));
    }    
    filesystem_file_flush(model_file);
    filesystem_file_set_position(model_file, 0);
    size_t model_size=filesystem_file_get_size(model_file);

    model_data = (uint8_t *)malloc(model_size + 255);
    uint8_t *model_data_align = (uint8_t *)(((uintptr_t)model_data+255)&(~255));

    filesystem_file_read(model_file, model_data_align, model_size);
    usleep(100*1000);
    model_context = kpu_model_load_from_buffer(model_data_align);
    return model_context;
}

// void kpu_run_input(void)
// {
//     kpu_run(model_context)
// }

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


void vTaskRecv()
{

    int recv_cnt=0;
    // initialize the queue

    while(true)
    {
        xSemaphoreTake(xMutexCopy, portMAX_DELAY);
        int buf_size=uart_get_buf_size(uart_radar1);
        // printf("The size of buff is %d\n",buf_size);
        while (io_read(uart_radar1, precv, buf_size) != buf_size)
        {
            ;
        }
        for(int i=0;i<read_size;i++)
        {
            pqueue_usb->push_back(precv[i]);
        }
        // std::swap(precv,precv_back);
        recv_cnt+=buf_size;
        recv_copy=true;
        xSemaphoreGive(xMutexCopy);
        vTaskDelay(3 / portTICK_RATE_MS);
    }
}


void vTaskCopy()
{
    unsigned int frame_num=0;
    RecvState recv_state=RECV_NULL;
    RecvState pre_state=recv_state;

    while(true)
    {
        xSemaphoreTake(xMutexCopy, portMAX_DELAY);
        if(recv_copy)
        {
            recv_state=RECV_START;
            recv_copy=false;
        }
        xSemaphoreGive(xMutexCopy);
        // recv_cnt++;
        // if(recv_cnt%1000==0)
        // {
        //     printf("recv_cnt is %d\n",recv_cnt);
        //     printf("queue_length is %ld\n",pqueue_usb->size());
        // }
        switch (recv_state)
        {
        case RECV_NULL:
            ;
            break;
        case RECV_START:
            while(!pqueue_usb->empty() && pqueue_usb->front()!='V')
            {
                pqueue_usb->pop_front();                
            }
            recv_state=RECV_HEAD;
            break;
        case RECV_HEAD:
            if(pqueue_usb->size()>4)
            {
                if((*pqueue_usb)[1]!='A' || (*pqueue_usb)[3]!='X')
                {
                    pqueue_usb->clear();
                    recv_state=RECV_NULL;
                }
                else
                {
                    frame_num=((*pqueue_usb)[2]);
                    recv_state=RECV_GET_SIZE;
                }
            }
            break;
        case RECV_GET_SIZE:
            if(pqueue_usb->size()>RADAR_FRAME_LENGTH)
            {
                
                printf("frame num is %u\n",frame_num);
                // io_write(uart1,write_buf,10);
                // pqueue_usb=pqueue_usb_back;
                xSemaphoreTake(xMutexUsb, portMAX_DELAY);
                // pqueue_usb_back->clear();
                // std::swap(pqueue_usb,pqueue_usb_back);
                recv_success=true;
                xSemaphoreGive(xMutexUsb);
                pqueue_usb->clear();

                recv_state=RECV_NULL;
            }
            break;
        default:
            break;
        }
        // if(pre_state!=recv_state)
        // {
        //     pre_state=recv_state;
        //     printf("current recv_state is %d",recv_state);
        // }

        // if(recv_cnt%10==0)
        // {
        //     std::copy(queue_usb.begin(),queue_usb.begin()+9,write_buf);
        //     io_write(uart1,write_buf,10);
        // }
        
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}


void vTaskProcess()
{
    while (true)
    {
        xSemaphoreTake(xMutexUsb, portMAX_DELAY);
        if (recv_success)
        {
            int idx_begin[4];
            int idx_end[4];
            // idx_end=frame_length*(i+1)/4-4;
            for (int i = 0; i < 4; i++)
            {
                int save_idx = 0;
                idx_begin[i] = RADAR_FRAME_LENGTH * i / 4 + 3;
                idx_end[i] = RADAR_FRAME_LENGTH * (i + 1) / 4 - 4;
                for (int j = idx_begin[i]; j < idx_end[i]; j += 4)
                {
                    recv_data[i][save_idx] = std::complex<float>((*pqueue_usb)[j] + (*pqueue_usb)[j + 1] * 255, (*pqueue_usb)[j + 2] + (*pqueue_usb)[j + 3] * 255);
                    save_idx++;
                }
            }
            frame_count++;
            recv_success=false;
            printf("processed %d frames",frame_count);
        }
        xSemaphoreGive(xMutexUsb);
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void vTask4()
{
    // uint8_t recv[30]={0};
    
    uint8_t recv = 0;
    const char *hel = {"hello uart!\n"};
    // io_write(uart1, (uint8_t *)hel, strlen(hel));
    show_proc_id();

    while (1)
    {
        // send recv data back
        // if(io_read(uart1, recv, 10) < 0)
        //     printf("time out \n");
        // while (io_read(uart1, &recv, 1) != 1)
            ;
        // queue_usb->push_back(recv);
        // // io_write(uart1, recv, 10);
        // if (queue_usb->size() > frame_length + 3)
        // {
        //     if (queue_usb.front() != 0x56)
        //     {
        //         queue_usb.pop_front();
        //     }
        //     else
        //     {
        //         if (queue_usb[1] == 0x30 && queue_usb[2] == 0x3A)
        //         {
        //             while (queue_usb.size() < frame_length)
        //             {
        //                 // while (io_read(uart1, &recv, 1) != 1)
        //                     ;
        //                 queue_usb.push_back(recv);
        //             }
        //             std::copy(queue_usb.begin(), queue_usb.begin() + frame_length, data_buf);
        //             for (int i = 0; i < frame_length; i++)
        //             {
        //                 queue_usb.pop_front();
        //             }
        //             recv_success = true;
        //         }
        //     }
        // }

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

void vTaskDetect()
{
    face_detect_rl.anchor_number = ANCHOR_NUM;
    face_detect_rl.anchor = anchor;
    face_detect_rl.threshold = 0.7;
    face_detect_rl.nms_value = 0.3;
    region_layer_init(&face_detect_rl, 20, 15, 30, camera_ctx.ai_image->width, camera_ctx.ai_image->height);
    
    while (true)
    {
        while (camera_ctx.dvp_finish_flag == false)
            ;
        camera_ctx.dvp_finish_flag = false;
        uint32_t *lcd_gram = camera_ctx.gram_mux ? (uint32_t *)camera_ctx.lcd_image1->addr : (uint32_t *)camera_ctx.lcd_image0->addr;
        camera_ctx.gram_mux ^= 0x01;

        float* output;
        size_t output_size;
        const char *fail_str="Cannot run kmodel.\n";

        if(kpu_run(model_handle,camera_ctx.ai_image->addr)!=0)
        {  
            // io_write(uart1, (uint8_t *)fail_str, strlen(fail_str));
        }
        else
        {
            if(0!=kpu_get_output(model_handle, 0, (uint8_t **)&output, &output_size))
            {
                // io_write(uart1, (uint8_t *)fail_str, strlen(fail_str));
            }
        }

        face_detect_rl.input = output;
        region_layer_run(&face_detect_rl, &face_detect_info);
        for (uint32_t face_cnt = 0; face_cnt < face_detect_info.obj_number; face_cnt++) {
            draw_edge(lcd_gram, &face_detect_info, face_cnt, RED);
        }        
        lcd_draw_picture(0, 0, 320, 240, lcd_gram);
        // const char *sig = {"vTask5run\n"};
        // io_write(uart1,(uint8_t *)sig,strlen(sig));
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}


int main()
{    
    // init_rgb();
    hardware_init();
    init_led();
    usleep(10*1000);

    // init_uart(uart1,115200,10*1000);
    usleep(10*1000);
    init_uart(uart_radar1,921600,10*1000);
    usleep(10*1000);
    init_uart(uart_radar2,115200,10*1000);
    usleep(10*1000);
    // init_timer(timer0,10*1e6,NULL);
    init_rgb_pwm();
    usleep(10*1000);

    sysctl_set_spi0_dvp_data(1);
    sysctl_set_power_mode(SYSCTL_POWER_BANK3, SYSCTL_POWER_V33);
    sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
    sysctl_set_power_mode(SYSCTL_POWER_BANK7, SYSCTL_POWER_V18);
    sysctl_pll_set_freq(SYSCTL_PLL0, 800000000UL);
    sysctl_pll_set_freq(SYSCTL_PLL1, 300000000UL); 
    sysctl_pll_set_freq(SYSCTL_PLL2, 45158400UL);
    lcd_init();
    usleep(10*1000);

    lcd_clear(GREEN);

    lcd_draw_picture_half(0, 0, 320, 240, (uint16_t *)gImage_logo);

    lcd_draw_string(16, 40, const_cast<char *>("Hello World!"), (uint16_t)RED);
    // init_timer(timer1);
    usleep(100*1000);
    init_sdcard();
    lcd_draw_string(16, 60, const_cast<char *>("LCD OK!"), (uint16_t)RED);
    // test sd card
    usleep(20*1000);
    // find_sdcard();
    test_write_sdcard();

    usleep(100*1000);
    dvp_init(&camera_ctx);
    ov2640_init();
    lcd_draw_string(16, 80, const_cast<char *>("Camera OK!"), (uint16_t)RED);
    usleep(100*1000);
    w25qxx_init(spi_flash);
    usleep(100*1000);
    model_handle=kmodel_get("/fs/detect.kmodel");
    lcd_draw_string(16, 100, const_cast<char *>("KPU OK!"), (uint16_t)RED);
    
    recv_copy=false;
    recv_success=false;
    
    pqueue_usb=&queue_usb_0;
    // pqueue_usb_back=&queue_usb_1;
    precv=recv0;
    // precv_back=recv1;

    if(!pqueue_usb->empty())
    {   
        pqueue_usb->clear();   
    }
    // if(!pqueue_usb_back->empty())
    // {   
    //     pqueue_usb_back->clear();   
    // }
    
    vTaskSuspendAll();
    xTaskCreate(TaskFunction_t(vTask1), "vTask1", 512, NULL, 3, NULL);
    // xTaskCreate(TaskFunction_t(vTask2), "vTask2", 128, NULL, 2, NULL);
    xTaskCreateAtProcessor(PROCESSOR0_ID, TaskFunction_t(vTask3), "vTask3", 128, NULL, 5, NULL);
    xTaskCreateAtProcessor(PROCESSOR0_ID, TaskFunction_t(vTaskRecv), "vTaskRecv", 1024, NULL, 4, NULL);
    xTaskCreateAtProcessor(PROCESSOR0_ID, TaskFunction_t(vTaskCopy), "vTaskCopy", 1024, NULL, 4, NULL);
    // xTaskCreateAtProcessor(PROCESSOR1_ID, TaskFunction_t(vTaskProcess), "vTaskProcess", 1024, NULL, 5, NULL);
    // xTaskCreateAtProcessor(PROCESSOR1_ID, TaskFunction_t(vTask1), "vTask1", 512, NULL, 3, NULL);
    // xTaskCreateAtProcessor(PROCESSOR1_ID, TaskFunction_t(vTaskDetect), "vTaskDetect", 8192, NULL, 5, NULL);

    if (!xTaskResumeAll())
    {
        taskYIELD();
    }

    printf("Run here!\n");
    while (1)
        ;

}

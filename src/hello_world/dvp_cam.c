#include <stdio.h>
#include <sys/unistd.h>
#include <main.h>
#include "dvp_cam.h"
#include "dvp.h"
#include "plic.h"
#include "sysctl.h"
#include "iomem.h"

enum _data_for
{
    DATA_FOR_AI = 0,
    DATA_FOR_DISPLAY = 1,
} ;

enum _enable
{
    DISABLE = 0,
    ENABLE = 1,
} ;

uint32_t *display_buf = NULL;
uint32_t display_buf_addr = 0;
volatile bool g_dvp_finish_flag;
volatile uint8_t gram_mux;
uint32_t *lcd_gram0;
uint32_t *lcd_gram1;

#define OVXXXX_ADDR    0x60 
#define OV9655_PID_1   0x9657   
#define OV9655_PID_2   0x9656  
#define OV2640_PID     0x2642

// For freertos
void on_irq_dvp(dvp_frame_event_t event, void* userdata)
{
    switch (event)
    {
        case VIDEO_FE_BEGIN:
            dvp_enable_frame(file_dvp);
            break;
        case VIDEO_FE_END:
            dvp_set_output_attributes(file_dvp, DATA_FOR_DISPLAY, VIDEO_FMT_RGB565, gram_mux ? lcd_gram0 : lcd_gram1);
            g_dvp_finish_flag = true;
            break;
        default:
            configASSERT(!"Invalid event.");
    }
}

void sensor_restart()
{
    dvp_set_signal(file_dvp, DVP_SIG_POWER_DOWN, 1);
    usleep(200 * 1000);
    dvp_set_signal(file_dvp, DVP_SIG_POWER_DOWN, 0);
    usleep(200 * 1000);
    dvp_set_signal(file_dvp, DVP_SIG_RESET, 0);
    usleep(200 * 1000);
    dvp_set_signal(file_dvp, DVP_SIG_RESET, 1);
    usleep(200 * 1000);
}

void dvp_init()
{
    lcd_gram0 = (uint32_t *)iomem_malloc(320*240*2);
    lcd_gram1 = (uint32_t *)iomem_malloc(320*240*2);

    sensor_restart();
    dvp_xclk_set_clock_rate(file_dvp, 24000000); /* 24MHz XCLK*/
    dvp_config(file_dvp, CAM_WIDTH_PIXEL, CAM_HIGHT_PIXEL, DISABLE);

    dvp_set_output_enable(file_dvp, DATA_FOR_AI, DISABLE);
    dvp_set_output_enable(file_dvp, DATA_FOR_DISPLAY, ENABLE);

    dvp_set_output_attributes(file_dvp, DATA_FOR_DISPLAY, VIDEO_FMT_RGB565, (void*)lcd_gram0);
    // dvp_set_output_attributes(file_dvp, DATA_FOR_DISPLAY, VIDEO_FMT_RGB565, (void*)lcd_gram1);
    // dvp_set_output_attributes(file_dvp, DATA_FOR_AI, VIDEO_FMT_RGB24_PLANAR, (void*)0x40600000);

    dvp_set_frame_event_enable(file_dvp, VIDEO_FE_END, DISABLE);
    dvp_set_frame_event_enable(file_dvp, VIDEO_FE_BEGIN, DISABLE);

    dvp_set_on_frame_event(file_dvp, on_irq_dvp, NULL);

    dvp_set_frame_event_enable(file_dvp, VIDEO_FE_END, ENABLE);
    dvp_set_frame_event_enable(file_dvp, VIDEO_FE_BEGIN, ENABLE);
}

// // For standalone
// /* dvp中断回调函数 */
// static int on_dvp_irq_cb(void *ctx)
// {
//     /* 读取DVP中断状态，如果完成则刷新显示地址的数据，并清除中断标志，否则读取摄像头数据*/
//     if (dvp_get_interrupt(DVP_STS_FRAME_FINISH))
//     {
//         dvp_set_display_addr((uint32_t)display_buf_addr);
//         dvp_clear_interrupt(DVP_STS_FRAME_FINISH);
//         g_dvp_finish_flag = 1;
//     }
//     else
//     {
//         if (g_dvp_finish_flag == 0)
//             dvp_start_convert();
//         dvp_clear_interrupt(DVP_STS_FRAME_START);
//     }
//     return 0;
// }

// /* dvp初始化 */
// void dvp_cam_init(void)
// {
//     /* DVP初始化，设置sccb的寄存器长度为8bit */
//     dvp_init(8);
//     /* 设置输入时钟为24000000*/
//     dvp_set_xclk_rate(24000000);
//     /* 使能突发传输模式 */
//     dvp_enable_burst();
//     /* 关闭AI输出模式，使能显示模式 */
//     dvp_set_output_enable(DATA_FOR_AI, 0);
//     dvp_set_output_enable(DATA_FOR_DISPLAY, 1);
//     /* 设置输出格式为RGB */
//     dvp_set_image_format(DVP_CFG_RGB_FORMAT);
//     /* 设置输出像素大小为320*240 */
//     dvp_set_image_size(CAM_WIDTH_PIXEL, CAM_HIGHT_PIXEL);

//     /* 设置DVP的显示地址参数和中断 */
//     display_buf = (uint32_t*)iomem_malloc(CAM_WIDTH_PIXEL * CAM_HIGHT_PIXEL * 2);
//     display_buf_addr = display_buf;
//     dvp_set_display_addr((uint32_t)display_buf_addr);
//     dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
//     dvp_disable_auto();
// }

// void dvp_cam_set_irq(void)
// {
//     /* DVP 中断配置：中断优先级，中断回调，使能DVP中断 */
//     printf("DVP interrupt config\r\n");
//     plic_set_priority(IRQN_DVP_INTERRUPT, 1);
//     plic_irq_register(IRQN_DVP_INTERRUPT, on_dvp_irq_cb, NULL);
//     plic_irq_enable(IRQN_DVP_INTERRUPT);

//     /* 清除DVP中断位 */
//     g_dvp_finish_flag = 0;
//     dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
//     dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 1);
// }


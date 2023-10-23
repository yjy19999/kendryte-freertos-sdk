#ifndef _DVP_CAM_H_
#define _DVP_CAM_H_

#include <stdint.h>
#include "image_process.h"

#define CAM_WIDTH_PIXEL        (320)
#define CAM_HIGHT_PIXEL        (240)
#define KPU_CHANNEL            (3)
#define DISP_CHANNEL           (2)

enum OV_type
{
    OV_error = 0,
    OV_9655 = 1,
    OV_2640 = 2,
};

typedef struct
{
    volatile uint8_t dvp_finish_flag;
    image_t *ai_image;
    image_t *lcd_image0;
    image_t *lcd_image1;
    volatile uint8_t gram_mux;
} camera_context_t;

extern uint32_t display_buf_addr;
extern camera_context_t camera_ctx;
// extern volatile bool g_dvp_finish_flag;
// extern volatile uint8_t gram_mux;
// extern uint32_t *lcd_gram0;
// extern uint32_t *lcd_gram1;

void dvp_cam_init(void);
void dvp_cam_set_irq(void);
int OVxxxx_read_id(void);
void dvp_init(camera_context_t *ctx);
#endif /* _DVP_CAM_H_ */

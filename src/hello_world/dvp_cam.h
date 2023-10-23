#ifndef _DVP_CAM_H_
#define _DVP_CAM_H_

#include <stdint.h>

#define CAM_WIDTH_PIXEL        (320)
#define CAM_HIGHT_PIXEL        (240)

enum OV_type
{
    OV_error = 0,
    OV_9655 = 1,
    OV_2640 = 2,
};

extern uint32_t display_buf_addr;
extern volatile bool g_dvp_finish_flag;
extern volatile uint8_t gram_mux;
extern uint32_t *lcd_gram0;
extern uint32_t *lcd_gram1;

void dvp_cam_init(void);
void dvp_cam_set_irq(void);
int OVxxxx_read_id(void);
void dvp_init();
#endif /* _DVP_CAM_H_ */

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
#include "main.h"
#include <stdio.h>
#include "ov2640.h"
#include "dvp.h"
#include "plic.h"
#include "dvp_cam.h"
#include <sleep.h>

#define REGLENGTH 8

#define OV2640_CHIPID_HIGH  0x0A
#define OV2640_CHIPID_LOW   0x0B

#define OV2640_ID 0x2642

const uint16_t jpeg_size_tbl[][2] = {
    { 160, 120 }, //QQVGA
    { 176, 144 }, //QCIF
    { 320, 240 }, //QVGA
    { 400, 240 }, //WQVGA
    { 352, 288 }, //CIF
};

const uint8_t ov2640_config[][2]=
{
    {0xff, 0x01},
    {0x12, 0x80},
    {0xff, 0x00},
    {0x2c, 0xff},
    {0x2e, 0xdf},
    {0xff, 0x01},
    {0x3c, 0x32},
    {0x11, 0x00},
    {0x09, 0x02},
    {0x04, 0xd8},
    {0x13, 0xe5},
    {0x14, 0x48},
    {0x2c, 0x0c},
    {0x33, 0x78},
    {0x3a, 0x33},
    {0x3b, 0xfb},
    {0x3e, 0x00},
    {0x43, 0x11},
    {0x16, 0x10},
    {0x39, 0x92},
    {0x35, 0xda},
    {0x22, 0x1a},
    {0x37, 0xc3},
    {0x23, 0x00},
    {0x34, 0xc0},
    {0x36, 0x1a},
    {0x06, 0x88},
    {0x07, 0xc0},
    {0x0d, 0x87},
    {0x0e, 0x41},
    {0x4c, 0x00},
    {0x48, 0x00},
    {0x5b, 0x00},
    {0x42, 0x03},
    {0x4a, 0x81},
    {0x21, 0x99},
    {0x24, 0x40},
    {0x25, 0x38},
    {0x26, 0x82},
    {0x5c, 0x00},
    {0x63, 0x00},
    {0x46, 0x22},
    {0x0c, 0x3c},
    {0x61, 0x70},
    {0x62, 0x80},
    {0x7c, 0x05},
    {0x20, 0x80},
    {0x28, 0x30},
    {0x6c, 0x00},
    {0x6d, 0x80},
    {0x6e, 0x00},
    {0x70, 0x02},
    {0x71, 0x94},
    {0x73, 0xc1},
    {0x3d, 0x34},
    {0x5a, 0x57},
    {0x12, 0x40},
    {0x17, 0x11},
    {0x18, 0x43},
    {0x19, 0x00},
    {0x1a, 0x4b},
    {0x32, 0x09},
    {0x37, 0xc0},
    {0x4f, 0xca},
    {0x50, 0xa8},
    {0x5a, 0x23},
    {0x6d, 0x00},
    {0x3d, 0x38},
    {0xff, 0x00},
    {0xe5, 0x7f},
    {0xf9, 0xc0},
    {0x41, 0x24},
    {0xe0, 0x14},
    {0x76, 0xff},
    {0x33, 0xa0},
    {0x42, 0x20},
    {0x43, 0x18},
    {0x4c, 0x00},
    {0x87, 0xd5},
    {0x88, 0x3f},
    {0xd7, 0x03},
    {0xd9, 0x10},
    {0xd3, 0x82},
    {0xc8, 0x08},
    {0xc9, 0x80},
    {0x7c, 0x00},
    {0x7d, 0x00},
    {0x7c, 0x03},
    {0x7d, 0x48},
    {0x7d, 0x48},
    {0x7c, 0x08},
    {0x7d, 0x20},
    {0x7d, 0x10},
    {0x7d, 0x0e},
    {0x90, 0x00},
    {0x91, 0x0e},
    {0x91, 0x1a},
    {0x91, 0x31},
    {0x91, 0x5a},
    {0x91, 0x69},
    {0x91, 0x75},
    {0x91, 0x7e},
    {0x91, 0x88},
    {0x91, 0x8f},
    {0x91, 0x96},
    {0x91, 0xa3},
    {0x91, 0xaf},
    {0x91, 0xc4},
    {0x91, 0xd7},
    {0x91, 0xe8},
    {0x91, 0x20},
    {0x92, 0x00},
    {0x93, 0x06},
    {0x93, 0xe3},
    {0x93, 0x05},
    {0x93, 0x05},
    {0x93, 0x00},
    {0x93, 0x04},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x93, 0x00},
    {0x96, 0x00},
    {0x97, 0x08},
    {0x97, 0x19},
    {0x97, 0x02},
    {0x97, 0x0c},
    {0x97, 0x24},
    {0x97, 0x30},
    {0x97, 0x28},
    {0x97, 0x26},
    {0x97, 0x02},
    {0x97, 0x98},
    {0x97, 0x80},
    {0x97, 0x00},
    {0x97, 0x00},
    {0xc3, 0xed},
    {0xa4, 0x00},
    {0xa8, 0x00},
    {0xc5, 0x11},
    {0xc6, 0x51},
    {0xbf, 0x80},
    {0xc7, 0x10},
    {0xb6, 0x66},
    {0xb8, 0xa5},
    {0xb7, 0x64},
    {0xb9, 0x7c},
    {0xb3, 0xaf},
    {0xb4, 0x97},
    {0xb5, 0xff},
    {0xb0, 0xc5},
    {0xb1, 0x94},
    {0xb2, 0x0f},
    {0xc4, 0x5c},
    {0xc0, 0x64},
    {0xc1, 0x4b},
    {0x8c, 0x00},
    {0x86, 0x3d},
    {0x50, 0x00},
    {0x51, 0xc8},
    {0x52, 0x96},
    {0x53, 0x00},
    {0x54, 0x00},
    {0x55, 0x00},
    {0x5a, 0xc8},
    {0x5b, 0x96},
    {0x5c, 0x00},
    {0xd3, 0x02},
    {0xc3, 0xed},
    {0x7f, 0x00},
    {0xda, 0x08},
    {0xe5, 0x1f},
    {0xe1, 0x67},
    {0xe0, 0x00},
    {0xdd, 0x7f},
    {0x05, 0x00},
    {0xff, 0x00},
    {0xe0, 0x04},
    {0x5a, 0x50},
    {0x5b, 0x3c},
    {0x5c, 0x00},
    {0xe0, 0x00},
    {0x00, 0x00}
};

handle_t file_sccb;
handle_t file_ov2640;

uint8_t OV2640_WR_Reg(uint16_t reg, uint8_t data)
{
    sccb_dev_write_byte(file_ov2640, reg, data);
    return 0;
}

uint8_t OV2640_RD_Reg(uint16_t reg)
{
    return sccb_dev_read_byte(file_ov2640, reg);
}

/* 初始化ov2640摄像头 */
int ov2640_init(void)
{
    file_sccb=io_open("/dev/sccb0");
    usleep(200*1000);
    configASSERT(file_sccb);
    file_ov2640=sccb_get_device(file_sccb,OV2640_ADDR,REGLENGTH);
    usleep(200*1000);
    configASSERT(file_ov2640);

    uint16_t reg;
    reg = OV2640_RD_Reg(OV2640_CHIPID_HIGH);
    reg <<= 8;
    reg |= OV2640_RD_Reg(OV2640_CHIPID_LOW);
    
    char id_str[20];
    // sprintf(id_str,"ID: 0x%04X \r\n", reg);
    // io_write(uart1,(uint8_t*)id_str,20);
    usleep(100 * 1000);
    if (reg !=OV2640_ID)
    {
        char* fault_str="ov2640 get id error.";
        // io_write(uart1,(uint8_t*)fault_str,20);
    }
    usleep(100 * 1000);
    //dvp_cam_init();
    //dvp_cam_set_irq();

    uint16_t v_manuf_id;
    uint16_t v_device_id;

    ov2640_read_id(&v_manuf_id, &v_device_id);
    char man_dev_id[50];
    sprintf(man_dev_id,"manuf_id:0x%04x,device_id:0x%04x\n", v_manuf_id, v_device_id);
    // io_write(uart1,(uint8_t*)man_dev_id,50);
    uint16_t index = 0;
    for (index = 0; ov2640_config[index][0]; index++)
        OV2640_WR_Reg(ov2640_config[index][0], ov2640_config[index][1]);
    return 0;
}

/* 读取ov2640的地址 */
int ov2640_read_id(uint16_t *manuf_id, uint16_t *device_id)
{
    OV2640_WR_Reg(0xFF,0x01);
    (*manuf_id) = (OV2640_RD_Reg(0x1C) << 8) | OV2640_RD_Reg(0x1D);
    (*device_id) = (OV2640_RD_Reg(0x0A) << 8) | OV2640_RD_Reg(0x0B);
    return 0;
}

void OV2640_JPEG_Mode(void)
{
    uint16_t i = 0;

    for (i = 0; i < (sizeof(jpeg_size_tbl) / 4); i++)
    {
        OV2640_WR_Reg(jpeg_size_tbl[i][0], jpeg_size_tbl[i][1]);
    }
}
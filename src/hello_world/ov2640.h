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
#ifndef _OV2640_H
#define _OV2640_H

#include <stdint.h>

#define OV2640_ADDR         0x60

int ov2640_init(void);
int ov2640_read_id(uint16_t *manuf_id, uint16_t *device_id);
void OV2640_JPEG_Mode(void);
#endif /* _OV2640_H */

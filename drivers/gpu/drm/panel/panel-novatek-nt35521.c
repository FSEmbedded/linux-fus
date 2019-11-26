/*
 * i.MX drm driver - Novatek MIPI-DSI panel driver
 *
 * Copyright (C) 2019 F&S Ele
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#define MAX_DATA_LEN 20

typedef struct cmd_table {
    u8      cmd_type;
    size_t  payload_len;
    char    payload[MAX_DATA_LEN];
}cmd_table;


/*
 *              29 01 00 00 00 00 06 F0 55 AA 52 08 00
				29 01 00 00 00 00 03 B1 68 21
				23 01 00 00 00 00 02 B5 C8
				29 01 00 00 00 00 02 B6 0F
				29 01 00 00 00 00 05 B8 00 00 0A 00
				23 01 00 00 00 00 02 B9 00
				23 01 00 00 00 00 02 BA 02
				29 01 00 00 00 00 03 BB 63 63
				29 01 00 00 00 00 03 BC 00 00
				
				29 01 00 00 00 00 06 BD 02 7F 0D 0B 00
				29 01 00 00 00 00 11 CC 41 36 87 54 46 65 10 12 14 10 12 14 40 08 15 05
				23 01 00 00 00 00 02 D0 00
				29 01 00 00 00 00 11 D1 00 04 08 0C 10 14 18 1C 20 24 28 2C 30 34 38 3C
				23 01 00 00 00 00 02 D3 00
				29 01 00 00 00 00 03 D6 44 44
				29 01 00 00 00 00 0D D7 00 00 00 00 00 00 00 00 00 00 00 00
				29 01 00 00 00 00 0E D8 00 00 00 00 00 00 00 00 00 00 00 00 00
				29 01 00 00 00 00 03 D9 03 06
				29 01 00 00 00 00 03 E5 00 FF
				29 01 00 00 00 00 05 E6 F3 EC E7 DF
				29 01 00 00 00 00 0B E7 F3 D9 CC CD B3 A6 99 99 99 95
				29 01 00 00 00 00 0B E8 F3 D9 CC CD B3 A6 99 99 99 95
				29 01 00 00 00 00 03 E9 00 04
				23 01 00 00 00 00 02 EA 00
				
				
				
				29 01 00 00 00 00 05 EE 87 78 00 00
				29 01 00 00 00 00 03 EF 07 FF
				29 01 00 00 00 00 06 F0 55 AA 52 08 01
				29 01 00 00 00 00 03 B0 0D 0D
				29 01 00 00 00 00 03 B1 0D 0D
				29 01 00 00 00 00 03 B3 2D 2D
				29 01 00 00 00 00 03 B4 19 19
				29 01 00 00 00 00 03 B5 06 06
				29 01 00 00 00 00 03 B6 05 05
				29 01 00 00 00 00 03 B7 05 05
				29 01 00 00 00 00 03 B8 05 05
				29 01 00 00 00 00 03 B9 44 44
				29 01 00 00 00 00 03 BA 36 36
				29 01 00 00 00 00 03 BC 50 00
				29 01 00 00 00 00 03 BD 50 00
				23 01 00 00 00 00 02 BE 39
				23 01 00 00 00 00 02 BF 39
				23 01 00 00 00 00 02 C0 0C
				23 01 00 00 00 00 02 C1 00
				
				
				29 01 00 00 00 00 03 C2 19 19
				29 01 00 00 00 00 03 C3 0A 0A
				29 01 00 00 00 00 03 C4 23 23
				29 01 00 00 00 00 04 C7 00 80 00
				29 01 00 00 00 00 07 C9 00 00 00 00 00 00
				23 01 00 00 00 00 02 CA 01
				29 01 00 00 00 00 03 CB 0B 53
				23 01 00 00 00 00 02 CC 00
				29 01 00 00 00 00 04 CD 0B 52 53
				23 01 00 00 00 00 02 CE 44
				29 01 00 00 00 00 04 CF 00 50 50
				29 01 00 00 00 00 03 D0 50 50
				29 01 00 00 00 00 03 D1 50 50
				23 01 00 00 00 00 02 D2 39
				23 01 00 00 00 00 02 D3 39
				29 01 00 00 00 00 06 F0 55 AA 52 08 02


				29 01 00 00 00 00 11 B0 00 AC 00 BA 00 D9 00 ED 01 01 01 1E 01 3A 01 62
				29 01 00 00 00 00 11 B1 01 85 01 B8 01 E4 02 27 02 5B 02 5D 02 8C 02 BE
				29 01 00 00 00 00 11 B2 02 DF 03 0C 03 2A 03 51 03 6D 03 8D 03 A4 03 BE
				29 01 00 00 00 00 05 B3 03 CC 03 CC
				29 01 00 00 00 00 11 B4 00 AC 00 BA 00 D9 00 ED 01 01 01 1E 01 3A 01 62
				29 01 00 00 00 00 11 B5 01 85 01 B8 01 E4 02 27 02 5B 02 5D 02 8C 02 BE
				29 01 00 00 00 00 11 B6 02 DF 03 0C 03 2A 03 51 03 6D 03 8D 03 A4 03 BE
				29 01 00 00 00 00 05 B7 03 CC 03 CC
				29 01 00 00 00 00 11 B8 00 AC 00 BA 00 D9 00 ED 01 01 01 1E 01 3A 01 62
				29 01 00 00 00 00 11 B9 01 85 01 B8 01 E4 02 27 02 5B 02 5D 02 8C 02 BE
				29 01 00 00 00 00 11 BA 02 DF 03 0C 03 2A 03 51 03 6D 03 8D 03 A4 03 BE
				29 01 00 00 00 00 05 BB 03 CC 03 CC
				29 01 00 00 00 00 11 BC 00 AC 00 BA 00 D9 00 ED 01 01 01 1E 01 3A 01 62
				29 01 00 00 00 00 11 BD 01 85 01 B8 01 E4 02 27 02 5B 02 5D 02 8C 02 BE
				29 01 00 00 00 00 11 BE 02 DF 03 0C 03 2A 03 51 03 6D 03 8D 03 A4 03 BE
				29 01 00 00 00 00 05 BF 03 CC 03 CC



				29 01 00 00 00 00 11 C0 00 AC 00 BA 00 D9 00 ED 01 01 01 1E 01 3A 01 62
				29 01 00 00 00 00 11 C1 01 85 01 B8 01 E4 02 27 02 5B 02 5D 02 8C 02 BE
				29 01 00 00 00 00 11 C2 02 DF 03 0C 03 2A 03 51 03 6D 03 8D 03 A4 03 BE
				29 01 00 00 00 00 05 C3 03 CC 03 CC
				29 01 00 00 00 00 11 C4 00 AC 00 BA 00 D9 00 ED 01 01 01 1E 01 3A 01 62
				29 01 00 00 00 00 11 C5 01 85 01 B8 01 E4 02 27 02 5B 02 5D 02 8C 02 BE
				29 01 00 00 00 00 11 C6 02 DF 03 0C 03 2A 03 51 03 6D 03 8D 03 A4 03 BE
				29 01 00 00 00 00 05 C7 03 CC 03 CC
				23 01 00 00 00 00 02 EE 00
				29 01 00 00 00 00 06 F0 55 AA 52 08 03
				29 01 00 00 00 00 03 B0 00 00
				29 01 00 00 00 00 03 B1 00 00
				29 01 00 00 00 00 06 B2 03 00 00 00 00
				29 01 00 00 00 00 06 B3 03 00 00 00 00
				29 01 00 00 00 00 06 B4 03 00 00 00 00
				29 01 00 00 00 00 06 B5 03 00 00 00 00
				29 01 00 00 00 00 06 B6 03 00 00 00 00
				29 01 00 00 00 00 06 B7 03 00 00 00 00
				29 01 00 00 00 00 06 B8 03 00 00 00 00
				29 01 00 00 00 00 06 B9 03 00 00 00 00
				29 01 00 00 00 00 06 BA 35 10 00 00 00
				29 01 00 00 00 00 06 BB 35 10 00 00 00
				29 01 00 00 00 00 06 BC 35 10 00 00 00
				29 01 00 00 00 00 06 BD 35 10 00 00 00
				29 01 00 00 00 00 05 C0 00 34 00 00
				29 01 00 00 00 00 05 C1 00 34 00 00
				29 01 00 00 00 00 05 C2 00 34 00 00
				29 01 00 00 00 00 05 C3 00 34 00 00
				23 01 00 00 00 00 02 C4 40
				23 01 00 00 00 00 02 C5 40
				23 01 00 00 00 00 02 C6 40
				23 01 00 00 00 00 02 C7 40
				23 01 00 00 00 00 02 EF 00

				29 01 00 00 00 00 06 F0 55 AA 52 08 05
				29 01 00 00 00 00 03 B0 1B 10
				29 01 00 00 00 00 03 B1 1B 10
				29 01 00 00 00 00 03 B2 1B 10
				29 01 00 00 00 00 03 B3 1B 10
				29 01 00 00 00 00 03 B4 1B 10
				29 01 00 00 00 00 03 B5 1B 10
				29 01 00 00 00 00 03 B6 1B 10
				29 01 00 00 00 00 03 B7 1B 10
				23 01 00 00 00 00 02 B8 00
				23 01 00 00 00 00 02 B9 00
				23 01 00 00 00 00 02 BA 00
				23 01 00 00 00 00 02 BB 00
				23 01 00 00 00 00 02 BC 00
				29 01 00 00 00 00 06 BD 03 03 03 00 01
				23 01 00 00 00 00 02 C0 03
				23 01 00 00 00 00 02 C1 05
				23 01 00 00 00 00 02 C2 03
				23 01 00 00 00 00 02 C3 05
				23 01 00 00 00 00 02 C4 80
				23 01 00 00 00 00 02 C5 A2
				23 01 00 00 00 00 02 C6 80
				23 01 00 00 00 00 02 C7 A2
				29 01 00 00 00 00 03 C8 01 20
				29 01 00 00 00 00 03 C9 00 20
				29 01 00 00 00 00 03 CA 01 00
				29 01 00 00 00 00 03 CB 00 00
				29 01 00 00 00 00 04 CC 00 00 01
				29 01 00 00 00 00 04 CD 00 00 01
				29 01 00 00 00 00 04 CE 00 00 01
				29 01 00 00 00 00 04 CF 00 00 01
				23 01 00 00 00 00 02 D0 00

				29 01 00 00 00 00 06 D1 03 00 00 07 10
				29 01 00 00 00 00 06 D2 13 00 00 07 11
				29 01 00 00 00 00 06 D3 23 00 00 07 10
				29 01 00 00 00 00 06 D4 33 00 00 07 11
				23 01 00 00 00 00 02 E5 06
				23 01 00 00 00 00 02 E6 06
				23 01 00 00 00 00 02 E7 06
				23 01 00 00 00 00 02 E8 06
				23 01 00 00 00 00 02 E9 06
				23 01 00 00 00 00 02 EA 06
				23 01 00 00 00 00 02 EB 06
				23 01 00 00 00 00 02 EC 06
				23 01 00 00 00 00 02 ED 31
				29 01 00 00 00 00 06 F0 55 AA 52 08 06
				29 01 00 00 00 00 03 B0 10 11
				29 01 00 00 00 00 03 B1 12 13
				29 01 00 00 00 00 03 B2 08 00
				29 01 00 00 00 00 03 B3 2D 2D
				29 01 00 00 00 00 03 B4 2D 34
				29 01 00 00 00 00 03 B5 34 2D
				29 01 00 00 00 00 03 B6 2D 34
				29 01 00 00 00 00 03 B7 34 34
				29 01 00 00 00 00 03 B8 02 0A
				29 01 00 00 00 00 03 B9 00 08
				29 01 00 00 00 00 03 BA 09 01
				29 01 00 00 00 00 03 BB 0B 03
				29 01 00 00 00 00 03 BC 34 34
				29 01 00 00 00 00 03 BD 34 2D
				29 01 00 00 00 00 03 BE 2D 34
				29 01 00 00 00 00 03 BF 34 2D
				29 01 00 00 00 00 03 C0 2D 2D
				29 01 00 00 00 00 03 C1 01 09
				29 01 00 00 00 00 03 C2 19 18
				29 01 00 00 00 00 03 C3 17 16
				29 01 00 00 00 00 03 C4 19 18
				29 01 00 00 00 00 03 C5 17 16
				29 01 00 00 00 00 03 C6 01 09
				29 01 00 00 00 00 03 C7 2D 2D
				29 01 00 00 00 00 03 C8 2D 34
				29 01 00 00 00 00 03 C9 34 2D
				29 01 00 00 00 00 03 CA 2D 34
				29 01 00 00 00 00 03 CB 34 34
				29 01 00 00 00 00 03 CC 0B 03
				29 01 00 00 00 00 03 CD 09 01
				29 01 00 00 00 00 03 CE 00 08
				29 01 00 00 00 00 03 CF 02 0A
				29 01 00 00 00 00 03 D0 34 34
				29 01 00 00 00 00 03 D1 34 2D
				29 01 00 00 00 00 03 D2 2D 34
				29 01 00 00 00 00 03 D3 34 2D
				29 01 00 00 00 00 03 D4 2D 2D
				29 01 00 00 00 00 03 D5 08 00
				29 01 00 00 00 00 03 D6 10 11
				29 01 00 00 00 00 03 D7 12 13
				29 01 00 00 00 00 06 D8 55 55 55 55 55
				29 01 00 00 00 00 06 D9 55 55 55 55 55
				29 01 00 00 00 00 03 E5 34 34
				29 01 00 00 00 00 03 E6 34 34
				23 01 00 00 00 00 02 E7 05
				29 01 00 00 00 00 06 F0 55 AA 52 00 00
				05 01 00 00 00 00 02 11 00
				05 01 00 00 14 00 02 29 00
				29 01 00 00 00 00 06 F0 55 AA 52 08 01
				29 01 00 00 00 00 06 F0 55 AA 52 00 00
				29 01 00 00 00 00 02 53 2C
 */


static const struct cmd_table def_cmd_table[] = {
    /* page 0 */
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00}},
    {0x29, 0x03, {0xB1, 0x68, 0x21}},
    {0x23, 0x02, {0xB5, 0xC8}},
    {0x29, 0x02, {0xB6, 0x0F}},
    {0x29, 0x05, {0xB8, 0x00, 0x00, 0x0A, 0x00}},
    {0x23, 0x02, {0xB9, 0x00}},
    {0x23, 0x02, {0xBA, 0x02}},
    {0x29, 0x03, {0xBB, 0x63, 0x63}},
    {0x29, 0x03, {0xBC, 0x00, 0x00}},
    {0x29, 0x06, {0xBD, 0x02, 0x7F, 0x0D, 0x0B, 0x00}},
    {0x29, 0x11, {0xCC, 0x41, 0x36, 0x87, 0x54, 0x46, 0x65, 0x10, 0x12, 0x14, 0x10, 0x12, 0x14, 0x40, 0x08, 0x15, 0x05}},
    {0x22, 0x02, {0xD0, 0x00}},
    {0x29, 0x11, {0xD1, 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30, 0x34, 0x38, 0x3C}},
    {0x23, 0x02, {0xD3, 0x00}},
    {0x29, 0x03, {0xD6, 0x44, 0x44}},
    {0x29, 0x0D, {0xD7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {0x29, 0x0E, {0xD8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {0x29, 0x03, {0xD9, 0x03, 0x06}},
    {0x29, 0x03, {0xE5, 0x00, 0xFF}},
    {0x29, 0x05, {0xE6, 0xF3, 0xEC, 0xE7, 0xDF}},
    {0x29, 0x0B, {0xE7, 0xF3, 0xD9, 0xCC, 0xCD, 0xB3, 0xA6, 0x99, 0x99, 0x99, 0x95}},
    {0x29, 0x0B, {0xE8, 0xF3, 0xD9, 0xCC, 0xCD, 0xB3, 0xA6, 0x99, 0x99, 0x99, 0x95}},
    {0x29, 0x03, {0xE9, 0x00, 0x04}},
    {0x23, 0x02, {0xEA, 0x00}},
    {0x29, 0x05, {0xEE, 0x87, 0x78, 0x00, 0x00}},
    {0x29, 0x03, {0xEF, 0x07, 0xFF}},
    /* page 1 */
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01}},
    {0x29, 0x03, {0xB0, 0x0D, 0x0D}},
    {0x29, 0x03, {0xB1, 0x0D, 0x0D}},
    {0x29, 0x03, {0xB3, 0x2D, 0x2D}},
    {0x29, 0x03, {0xB4, 0x19, 0x19}},
    {0x29, 0x03, {0xB5, 0x06, 0x06}},
    {0x29, 0x03, {0xB6, 0x05, 0x05}},
    {0x29, 0x03, {0xB7, 0x05, 0x05}},
    {0x29, 0x03, {0xB8, 0x05, 0x05}},
    {0x29, 0x03, {0xB9, 0x44, 0x44}},
    {0x29, 0x03, {0xBA, 0x36, 0x36}},
    {0x29, 0x03, {0xBC, 0x50, 0x00}},
    {0x29, 0x03, {0xBD, 0x50, 0x00}},
    {0x23, 0x02, {0xBE, 0x39}},
    {0x23, 0x02, {0xBF, 0x39}},
    {0x23, 0x02, {0xC0, 0x0C}},
    {0x23, 0x02, {0xC1, 0x00}},
    {0x29, 0x03, {0xC2, 0x19, 0x19}},
    {0x29, 0x03, {0xC3, 0x0A, 0x0A}},
    {0x29, 0x03, {0xC4, 0x23, 0x23}},
    {0x29, 0x04, {0xC7, 0x00, 0x80, 0x00}},
    {0x29, 0x07, {0xC9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {0x23, 0x02, {0xCA, 0x01}},
    {0x29, 0x03, {0xCB, 0xCB, 0x53}},
    {0x23, 0x02, {0xCC, 0x00}},
    {0x29, 0x04, {0xCD, 0x0B, 0x52, 0x53}},
    {0x23, 0x02, {0xCE, 0x44}},
    {0x29, 0x04, {0xCF, 0x00, 0x50, 0x50}},
    {0x29, 0x03, {0xD0, 0x50, 0x50}},
    {0x29, 0x03, {0xD1, 0x50, 0x50}},
    {0x29, 0x03, {0x02, 0xD2, 0x39}},
    {0x23, 0x02, {0xD3, 0x39}},
    /* page 2 */
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02}},
    {0x29, 0x11, {0xB0, 0x00, 0xAC, 0x00, 0xBA, 0x00, 0xD9, 0x00, 0xED, 0x01, 0x01, 0x01, 0x1E, 0x01, 0x3A, 0x01, 0x62}},
    {0x29, 0x11, {0xB1, 0x01, 0x85, 0x01, 0xB8, 0x01, 0xE4, 0x02, 0x27, 0x02, 0x5B, 0x02, 0x5D, 0x02, 0x8C, 0x02, 0xBE}},
    {0x29, 0x11, {0xB2, 0x02, 0xDF, 0x03, 0x0C, 0x03, 0x2A, 0x03, 0x51, 0x03, 0x6D, 0x03, 0x8D, 0x03, 0xA4, 0x03, 0xBE}},
    {0x29, 0x05, {0xB3, 0x03, 0xCC, 0x03, 0xCC}},
    {0x29, 0x11, {0xB4, 0x00, 0xAC, 0x00, 0xBA, 0x00, 0xD9, 0x00, 0xED, 0x01, 0x01, 0x01, 0x1E, 0x01, 0x3A, 0x01, 0x62}},
    {0x29, 0x11, {0xB5, 0x01, 0x85, 0x01, 0xB8, 0x01, 0xE4, 0x02, 0x27, 0x02, 0x5B, 0x02, 0x5D, 0x02, 0x8C, 0x02, 0xBE}},
    {0x29, 0x11, {0xB6, 0x02, 0xDF, 0x03, 0x0C, 0x03, 0x2A, 0x03, 0x51, 0x03, 0x6D, 0x03, 0x8D, 0x03, 0xA4, 0x03, 0xBE}},
    {0x29, 0x05, {0xB7, 0x03, 0xCC, 0x03, 0xCC}},
    {0x29, 0x11, {0xB8, 0x00, 0xAC, 0x00, 0xBA, 0x00, 0xD9, 0x00, 0xED, 0x01, 0x01, 0x01, 0x1E, 0x01, 0x3A, 0x01, 0x62}},
    {0x29, 0x11, {0xB9, 0x01, 0x85, 0x01, 0xB8, 0x01, 0xE4, 0x02, 0x27, 0x02, 0x5B, 0x02, 0x5D, 0x02, 0x8C, 0x02, 0xBE}},
    {0x29, 0x11, {0xBA, 0x02, 0xDF, 0x03, 0x0C, 0x03, 0x2A, 0x03, 0x51, 0x03, 0x6D, 0x03, 0x8D, 0x03, 0xA4, 0x03, 0xBE}},
    {0x29, 0x11, {0xBB, 0x03, 0xCC, 0x03, 0xCC}},
    {0x29, 0x11, {0xBC, 0x00, 0xAC, 0x00, 0xBA, 0x00, 0xD9, 0x00, 0xED, 0x01, 0x01, 0x01, 0x1E, 0x01, 0x3A, 0x01, 0x62}},
    {0x29, 0x11, {0xBD, 0x01, 0x85, 0x01, 0xB8, 0x01, 0xE4, 0x02, 0x27, 0x02, 0x5B, 0x02, 0x5D, 0x02, 0x8C, 0x02, 0xBE}},
    {0x29, 0x11, {0xBE, 0x02, 0xDF, 0x03, 0x0C, 0x03, 0x2A, 0x03, 0x51, 0x03, 0x6D, 0x03, 0x8D, 0x03, 0xA4, 0x03, 0xBE}},
    {0x29, 0x05, {0xBF, 0x03, 0xCC, 0x03, 0xCC}},
    {0x29, 0x11, {0xC0, 0x00, 0xAC, 0x00, 0xBA, 0x00, 0xD9, 0x00, 0xED, 0x01, 0x01, 0x01, 0x1E, 0x01, 0x3A, 0x01, 0x62}},
    {0x29, 0x11, {0xC1, 0x01, 0x85, 0x01, 0xB8, 0x01, 0xE4, 0x02, 0x27, 0x02, 0x5B, 0x02, 0x5D, 0x02, 0x8C, 0x02, 0xBE}},
    {0x29, 0x11, {0xC2, 0x02, 0xDF, 0x03, 0x0C, 0x03, 0x2A, 0x03, 0x51, 0x03, 0x6D, 0x03, 0x8D, 0x03, 0xA4, 0x03, 0xBE}},
    {0x29, 0x05, {0xC3, 0x03, 0xCC, 0x03, 0xCC}},
    {0x29, 0x11, {0xC4, 0x00, 0xAC, 0x00, 0xBA, 0x00, 0xD9, 0x00, 0xED, 0x01, 0x01, 0x01, 0x1E, 0x01, 0x3A, 0x01, 0x62}},
    {0x29, 0x11, {0xC5, 0x01, 0x85, 0x01, 0xB8, 0x01, 0xE4, 0x02, 0x27, 0x02, 0x5B, 0x02, 0x5D, 0x02, 0x8C, 0x02, 0xBE}},
    {0x29, 0x11, {0xC6, 0x02, 0xDF, 0x03, 0x0C, 0x03, 0x2A, 0x03, 0x51, 0x03, 0x6D, 0x03, 0x8D, 0x03, 0xA4, 0x03, 0xBE}},
    {0x29, 0x05, {0xC7, 0x03, 0xCC, 0x03, 0xCC}},
    {0x23, 0x02, {0xEE, 0x00}},
    /* page 3 */
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x03}},
    {0x29, 0x03, {0xB0, 0x00, 0x00}},
    {0x29, 0x03, {0xB1, 0x00, 0x00}},
    {0x29, 0x06, {0xB2, 0x03, 0x00, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xB3, 0x03, 0x00, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xB4, 0x03, 0x00, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xB5, 0x03, 0x00, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xB6, 0x03, 0x00, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xB7, 0x03, 0x00, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xB8, 0x03, 0x00, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xB9, 0x03, 0x00, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xBA, 0x35, 0x10, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xBB, 0x35, 0x10, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xBC, 0x35, 0x10, 0x00, 0x00, 0x00}},
    {0x29, 0x06, {0xBD, 0x35, 0x10, 0x00, 0x00, 0x00}},
    {0x29, 0x05, {0xC0, 0x00, 0x34, 0x00, 0x00}},
    {0x29, 0x05, {0xC1, 0x00, 0x34, 0x00, 0x00}},
    {0x29, 0x05, {0xC2, 0x00, 0x34, 0x00, 0x00}},
    {0x29, 0x05, {0xC3, 0x00, 0x34, 0x00, 0x00}},
    {0x23, 0x02, {0xC4, 0x40}},
    {0x23, 0x02, {0xC5, 0x40}},
    {0x23, 0x02, {0xC6, 0x40}},
    {0x23, 0x02, {0xC7, 0x40}},
    {0x23, 0x02, {0xEF, 0x00}},
    /* page 5 */
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05}},
    {0x29, 0x03, {0xB0, 0x1B, 0x10}},
    {0x29, 0x03, {0xB1, 0x1B, 0x10}},
    {0x29, 0x03, {0xB2, 0x1B, 0x10}},
    {0x29, 0x03, {0xB3, 0x1B, 0x10}},
    {0x29, 0x03, {0xB4, 0x1B, 0x10}},
    {0x29, 0x03, {0xB5, 0x1B, 0x10}},
    {0x29, 0x03, {0xB6, 0x1B, 0x10}},
    {0x29, 0x03, {0xB7, 0x1B, 0x10}},
    {0x23, 0x02, {0xB8, 0x00}},
    {0x23, 0x02, {0xB9, 0x00}},
    {0x23, 0x02, {0xBA, 0x00}},
    {0x23, 0x02, {0xBB, 0x00}},
    {0x23, 0x02, {0xBC, 0x00}},
    {0x29, 0x06, {0xBD, 0x03, 0x03, 0x03, 0x00, 0x01}},
    {0x23, 0x02, {0xC0, 0x03}},
    {0x23, 0x02, {0xC1, 0x05}},
    {0x23, 0x02, {0xC2, 0x03}},
    {0x23, 0x02, {0xC3, 0x05}},
    {0x23, 0x02, {0xC4, 0x80}},
    {0x23, 0x02, {0xC5, 0xA2}},
    {0x23, 0x02, {0xC6, 0x80}},
    {0x23, 0x02, {0xC7, 0xA2}},
    {0x29, 0x03, {0xC8, 0x01, 0x20}},
    {0x29, 0x03, {0xC9, 0x00, 0x20}},
    {0x29, 0x03, {0xCA, 0x01, 0x00}},
    {0x29, 0x03, {0xCB, 0x00, 0x00}},
    {0x29, 0x04, {0xCC, 0x00, 0x00, 0x01}},
    {0x29, 0x04, {0xCD, 0x00, 0x00, 0x01}},
    {0x29, 0x04, {0xCE, 0x00, 0x00, 0x01}},
    {0x29, 0x04, {0xCF, 0x00, 0x00, 0x01}},
    {0x23, 0x02, {0xD0, 0x00}},
    {0x29, 0x06, {0xD1, 0x03, 0x00, 0x00, 0x07, 0x10}},
    {0x29, 0x06, {0xD2, 0x13, 0x00, 0x00, 0x07, 0x11}},
    {0x29, 0x06, {0xD3, 0x23, 0x00, 0x00, 0x07, 0x10}},
    {0x29, 0x06, {0xD4, 0x33, 0x00, 0x00, 0x07, 0x11}},
    {0x23, 0x02, {0xE5, 0x06}},
    {0x23, 0x02, {0xE6, 0x06}},
    {0x23, 0x02, {0xE7, 0x06}},
    {0x23, 0x02, {0xE8, 0x06}},
    {0x23, 0x02, {0xE9, 0x06}},
    {0x23, 0x02, {0xEA, 0x06}},
    {0x23, 0x02, {0xEB, 0x06}},
    {0x23, 0x02, {0xEC, 0x06}},
    {0x23, 0x02, {0xED, 0x31}},
    /* page 6 */
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x06}},
    {0x29, 0x03, {0xB0, 0x10, 0x11}},
    {0x29, 0x03, {0xB1, 0x12, 0x13}},
    {0x29, 0x03, {0xB2, 0x08, 0x00}},
    {0x29, 0x03, {0xB3, 0x2D, 0x2D}},
    {0x29, 0x03, {0xB4, 0x2D, 0x34}},
    {0x29, 0x03, {0xB5, 0x34, 0x2D}},
    {0x29, 0x03, {0xB6, 0x2D, 0x34}},
    {0x29, 0x03, {0xB7, 0x34, 0x34}},
    {0x29, 0x03, {0xB8, 0x02, 0x0A}},
    {0x29, 0x03, {0xB9, 0x00, 0x08}},
    {0x29, 0x03, {0xBA, 0x09, 0x01}},
    {0x29, 0x03, {0xBB, 0x0B, 0x03}},
    {0x29, 0x03, {0xBC, 0x34, 0x34}},
    {0x29, 0x03, {0xBD, 0x34, 0x2D}},
    {0x29, 0x03, {0xBE, 0x2D, 0x34}},
    {0x29, 0x03, {0xBF, 0x34, 0x2D}},

    {0x29, 0x03, {0xC0, 0x2D, 0x2D}},
    {0x29, 0x03, {0xC1, 0x01, 0x09}},
    {0x29, 0x03, {0xC2, 0x19, 0x18}},
    {0x29, 0x03, {0xC3, 0x17, 0x16}},
    {0x29, 0x03, {0xC4, 0x19, 0x18}},
    {0x29, 0x03, {0xC5, 0x17, 0x16}},
    {0x29, 0x03, {0xC6, 0x01, 0x09}},
    {0x29, 0x03, {0xC7, 0x2D, 0x2D}},
    {0x29, 0x03, {0xC8, 0x2D, 0x34}},
    {0x29, 0x03, {0xC9, 0x34, 0x2D}},
    {0x29, 0x03, {0xCA, 0x2D, 0x34}},
    {0x29, 0x03, {0xCB, 0x34, 0x34}},
    {0x29, 0x03, {0xCC, 0x0B, 0x03}},
    {0x29, 0x03, {0xCD, 0x09, 0x01}},
    {0x29, 0x03, {0xCE, 0x00, 0x08}},
    {0x29, 0x03, {0xCF, 0x02, 0x0A}},

    {0x29, 0x03, {0xD0, 0x34, 0x34}},
    {0x29, 0x03, {0xD1, 0x34, 0x2D}},
    {0x29, 0x03, {0xD2, 0x2D, 0x34}},
    {0x29, 0x03, {0xD3, 0x34, 0x2D}},
    {0x29, 0x03, {0xD4, 0x2D, 0x2D}},
    {0x29, 0x03, {0xD5, 0x08, 0x00}},

    {0x29, 0x03, {0xD6, 0x10, 0x11}},
    {0x29, 0x03, {0xD7, 0x12, 0x13}},
    {0x29, 0x06, {0xD8, 0x55, 0x55, 0x55, 0x55, 0x55}},
    {0x29, 0x06, {0xD9, 0x55, 0x55, 0x55, 0x55, 0x55}},
    {0x29, 0x03, {0xE5, 0x34, 0x34}},
    {0x29, 0x03, {0xE6, 0x34, 0x34}},
    {0x23, 0x02, {0xE7, 0x05}},

    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x00, 0x00}},

    {0x05, 0x02, {0x11, 0x00}},
    {0x05, 0x02, {0x29, 0x00}}, //???? 05 01 00 00 ->14 00 02 29 00

    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01}},
    {0x29, 0x06, {0xF0, 0x55, 0xAA, 0x52, 0x00, 0x00}},

    /* Write Display Control (WRCTRLD):
     * brightness ctrl on, dimming on, backlight ctrl on */
    {0x29, 0x02, {0x53, 0x2C}},

};

#define CMD_TABLE_LEN 2
typedef u8 cmd_set_table[CMD_TABLE_LEN];

static const u32 nt_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

struct nt_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;
	struct backlight_device *backlight;

	bool prepared;
	bool enabled;

	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
};

static inline struct nt_panel *to_nt_panel(struct drm_panel *panel)
{
	return container_of(panel, struct nt_panel, base);
}

static int nt_panel_push_cmd_list(struct mipi_dsi_device *dsi)
{
	size_t i;
        const struct cmd_table *cmd;

    /* number of command table elements */
        size_t count = sizeof(def_cmd_table) / sizeof(cmd_table);
	int ret = 0;

//        printk("### %s++\n",__func__);
        /* tx data */
	for (i = 0; i < count ; i++) {
            cmd = &def_cmd_table[i];
            switch(cmd->cmd_type)
            {
                case MIPI_DSI_DCS_SHORT_WRITE:
                    ret = mipi_dsi_dcs_write(dsi, cmd->payload[0], 0, 0);
                    break;
                case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
                case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
                case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
                case MIPI_DSI_GENERIC_LONG_WRITE:
                default:
                    ret = mipi_dsi_generic_write(dsi, cmd, cmd->payload_len);
            }
            if (ret < 0)
                return ret;
	}

	return ret;
};

static int color_format_from_dsi_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return 0x55;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		return 0x66;
	case MIPI_DSI_FMT_RGB888:
		return 0x77;
	default:
		return 0x77; /* for backward compatibility */
	}
};

static int nt_panel_prepare(struct drm_panel *panel)
{
        struct nt_panel *nt = to_nt_panel(panel);


       // printk("### %s++", __func__);

	if (nt->prepared)
		return 0;

	if (nt->reset != NULL) {
		gpiod_set_value(nt->reset, 0);
		usleep_range(5000, 10000);
		gpiod_set_value(nt->reset, 1);
		usleep_range(20000, 25000);
	}

	nt->prepared = true;

        //printk("### %s--", __func__);

	return 0;
}

static int nt_panel_unprepare(struct drm_panel *panel)
{
	struct nt_panel *nt = to_nt_panel(panel);
	struct device *dev = &nt->dsi->dev;

        //printk("### %s++", __func__);

	if (!nt->prepared)
		return 0;

	if (nt->enabled) {
		DRM_DEV_ERROR(dev, "Panel still enabled!\n");
		return -EPERM;
	}

	if (nt->reset != NULL) {
		gpiod_set_value(nt->reset, 0);
		usleep_range(15000, 17000);
		gpiod_set_value(nt->reset, 1);
	}

	nt->prepared = false;

        //printk("### %s--", __func__);

	return 0;
}

static int nt_panel_enable(struct drm_panel *panel)
{
	struct nt_panel *nt = to_nt_panel(panel);
	struct mipi_dsi_device *dsi = nt->dsi;
	struct device *dev = &dsi->dev;
	int color_format = color_format_from_dsi_format(dsi->format);
	u16 brightness;
	int ret;

        //printk("### %s++", __func__);

	if (nt->enabled)
		return 0;

	if (!nt->prepared) {
		DRM_DEV_ERROR(dev, "Panel not prepared!\n");
		return -EPERM;
	}

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

         //printk("### push cmd list");
#if 1
	ret = nt_panel_push_cmd_list(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to send MCS (%d)\n", ret);
		goto fail;
	}
 #endif

	/* Software reset */
	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to do Software Reset (%d)\n", ret);
		goto fail;
	}

        msleep(120);

	/* Set pixel format */
	ret = mipi_dsi_dcs_set_pixel_format(dsi, color_format);
	DRM_DEV_DEBUG_DRIVER(dev, "Interface color format set to 0x%x\n",
				color_format);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set pixel format (%d)\n", ret);
		goto fail;
	}
	/* Set display brightness */
	brightness = nt->backlight->props.brightness;
	ret = mipi_dsi_dcs_set_display_brightness(dsi, brightness);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display brightness (%d)\n",
			      ret);
		goto fail;
	}

	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to exit sleep mode (%d)\n", ret);
		goto fail;
	}

         msleep(5);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display ON (%d)\n", ret);
		goto fail;
	}

	nt->backlight->props.power = FB_BLANK_UNBLANK;
	backlight_update_status(nt->backlight);

	nt->enabled = true;

        //printk("### %s-- ok", __func__);

	return 0;

fail:
	if (nt->reset != NULL)
		gpiod_set_value(nt->reset, 0);


        //printk("### %s-- fails", __func__);

	return ret;
}

static int nt_panel_disable(struct drm_panel *panel)
{
	struct nt_panel *nt = to_nt_panel(panel);
	struct mipi_dsi_device *dsi = nt->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (!nt->enabled)
		return 0;

       // printk("### %s++", __func__);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	usleep_range(10000, 15000);

	nt->backlight->props.power = FB_BLANK_POWERDOWN;
	backlight_update_status(nt->backlight);

	nt->enabled = false;

	return 0;
}

static int nt_panel_get_modes(struct drm_panel *panel)
{
	struct nt_panel *nt = to_nt_panel(panel);
	struct device *dev = &nt->dsi->dev;
	struct drm_connector *connector = panel->connector;
	struct drm_display_mode *mode;
	u32 *bus_flags = &connector->display_info.bus_flags;
	int ret;

        //printk("#### %s++\n",__func__);

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_DEV_ERROR(dev, "Failed to create display mode!\n");
		return 0;
	}

	drm_display_mode_from_videomode(&nt->vm, mode);
	mode->width_mm = nt->width_mm;
	mode->height_mm = nt->height_mm;
	connector->display_info.width_mm = nt->width_mm;
	connector->display_info.height_mm = nt->height_mm;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	if (nt->vm.flags & DISPLAY_FLAGS_DE_HIGH)
		*bus_flags |= DRM_BUS_FLAG_DE_HIGH;
	if (nt->vm.flags & DISPLAY_FLAGS_DE_LOW)
		*bus_flags |= DRM_BUS_FLAG_DE_LOW;
	if (nt->vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_NEGEDGE;
	if (nt->vm.flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		*bus_flags |= DRM_BUS_FLAG_PIXDATA_POSEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
			nt_bus_formats, ARRAY_SIZE(nt_bus_formats));
	if (ret)
		return ret;

	drm_mode_probed_add(panel->connector, mode);

        //printk("#### %s--\n",__func__);

	return 1;
}

static int nt_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct nt_panel *nt = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	u16 brightness;
	int ret;

	if (!nt->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "\n");

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int nt_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct nt_panel *nt = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret = 0;

	if (!nt->prepared)
		return 0;

	DRM_DEV_DEBUG_DRIVER(dev, "New brightness: %d\n", bl->props.brightness);

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops nt_bl_ops = {
	.update_status = nt_bl_update_status,
	.get_brightness = nt_bl_get_brightness,
};

static const struct drm_panel_funcs nt_panel_funcs = {
	.prepare = nt_panel_prepare,
	.unprepare = nt_panel_unprepare,
	.enable = nt_panel_enable,
	.disable = nt_panel_disable,
	.get_modes = nt_panel_get_modes,
};


/*
 * The clock might range from 66MHz (30Hz refresh rate)
 * to 132MHz (60Hz refresh rate)
 */
static const struct display_timing nt_default_timing = {
	.pixelclock = { 66000000, 132000000, 132000000 },
	.hactive = { 1080, 1080, 1080 },
	.hfront_porch = { 20, 20, 20 },
	.hsync_len = { 2, 2, 2 },
	.hback_porch = { 34, 34, 34 },
	.vactive = { 1920, 1920, 1920 },
	.vfront_porch = { 10, 10, 10 },
	.vsync_len = { 2, 2, 2 },
	.vback_porch = { 4, 4, 4 },
	.flags = DISPLAY_FLAGS_HSYNC_LOW |
		 DISPLAY_FLAGS_VSYNC_LOW |
		 DISPLAY_FLAGS_DE_LOW |
		 DISPLAY_FLAGS_PIXDATA_NEGEDGE,
};

static int nt_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	struct device_node *timings;
	struct nt_panel *panel;
	struct backlight_properties bl_props;
	int ret;
    /* use default burst mode */
	u32 video_mode = 0;

        //printk("### %s++", __func__);

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;

	dsi->format = MIPI_DSI_FMT_RGB888;
        dsi->mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO
                | MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_BURST;

	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
			break;
		case 1:
			/* non-burst mode with sync event */
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;

		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret < 0) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	/*
	 * 'display-timings' is optional, so verify if the node is present
	 * before calling of_get_videomode so we won't get console error
	 * messages
	 */
	timings = of_get_child_by_name(np, "display-timings");
	if (timings) {
		of_node_put(timings);
		ret = of_get_videomode(np, &panel->vm, 0);
	} else {
		videomode_from_timing(&nt_default_timing, &panel->vm);
	}
	if (ret < 0)
		return ret;

	of_property_read_u32(np, "panel-width-mm", &panel->width_mm);
	of_property_read_u32(np, "panel-height-mm", &panel->height_mm);

	panel->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(panel->reset))
		panel->reset = NULL;
	else
		gpiod_set_value(panel->reset, 0);


	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	panel->backlight = devm_backlight_device_register(
				dev, dev_name(dev),
				dev, dsi,
				&nt_bl_ops, &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_init(&panel->base);
	panel->base.funcs = &nt_panel_funcs;
	panel->base.dev = dev;

	ret = drm_panel_add(&panel->base);

	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&panel->base);

        //printk("### %s-- ret=%d", __func__, ret);

	return ret;
}

static int nt_panel_remove(struct mipi_dsi_device *dsi)
{
	struct nt_panel *nt = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to detach from host (%d)\n",
			ret);

	drm_panel_detach(&nt->base);

	if (nt->base.dev)
		drm_panel_remove(&nt->base);

	return 0;
}

static void nt_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct nt_panel *nt = mipi_dsi_get_drvdata(dsi);

	nt_panel_disable(&nt->base);
	nt_panel_unprepare(&nt->base);
}

static const struct of_device_id nt_of_match[] = {
	{ .compatible = "novatek,nt35521", },
	{ }
};
MODULE_DEVICE_TABLE(of, nt_of_match);

static struct mipi_dsi_driver nt_panel_driver = {
	.driver = {
		.name = "panel-novatek-nt35521",
		.of_match_table = nt_of_match,
	},
	.probe = nt_panel_probe,
	.remove = nt_panel_remove,
	.shutdown = nt_panel_shutdown,
};
module_mipi_dsi_driver(nt_panel_driver);

MODULE_AUTHOR("F&S Elektronik Systeme GmbH");
MODULE_DESCRIPTION("Novatek NT35521");
MODULE_LICENSE("GPL v2");

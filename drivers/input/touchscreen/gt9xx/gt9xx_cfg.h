/* drivers/input/touchscreen/gt9xx_cfg.h
 *
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifndef _GOODIX_GT9XX_CFG_H_
#define _GOODIX_GT9XX_CFG_H_

/* CFG for GT911 */
u8 gtp_dat_gt11[] = {
	/* <1200, 1920>*/
	#include "WGJ89006B_GT911_Config_20140625_085816_0X43.cfg"
};

u8 gtp_dat_8_9[] = {
	/* TODO:Puts your update firmware data here! */
	/* <1920, 1200> 8.9 */
	/* #include "WGJ89006B_GT9271_Config_20140625_085816_0X41.cfg" */
	/* #include "WGJ10162_GT9271_Config_20140820_182456.cfg" */
	#include "ZXHD_08003C1F1_GT911_ygkj_1200X1920_20190916.cfg"
};

u8 gtp_dat_8_9_1[] = {
	#include "GT9271_Config_20170526.cfg"
};

u8 gtp_dat_9_7[] = {
	/* <1536, 2048> 9.7 */
	#include "ZXHD_07005C1F1_GT911_ygkj_1200x1920_20190225.cfg"
};

u8 gtp_dat_10_1[] = {
	/* TODO:Puts your update firmware data here! */
	/* <1200, 1920> 10.1 */
	#include "WGJ10187_GT9271_Config_20140623_104014_0X41.cfg"
};

u8 gtp_dat_7[] = {
	/* TODO:Puts your update firmware data here! */
	/* <1024, 600> 7.0 */
	#include "ZXHD_07005C1F1_GT911_ygkj_1200x1920_20190225.cfg"
};

#endif /* _GOODIX_GT9XX_CFG_H_ */

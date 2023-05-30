/* drivers/input/sensors/access/kxtik.c
 *
 * Copyright (C) 2016 Liteon Singapore Ptd Ltd.
 * Author: Shi Zhigang
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>
#include "ltr_559als.h"

static int ps_hi_thres = 200;
static int ps_low_thres = 100;
static int near_far_value = 0;

int sensor_ps_read(struct i2c_client *client)
{
	int psval_lo, psval_hi, psdata;

	psval_lo = sensor_read_reg(client, APS_RO_PS_DATA_0);
	psval_hi = sensor_read_reg(client, APS_RO_PS_DATA_1);
	
	psdata = ((psval_hi & 7) * 256) + psval_lo;
	//DBG("[ltr559] %s psdata=%d\n",__func__,psdata);

	return psdata;
}

static int sensor_ps_enable(struct i2c_client *client)
{
	int error;
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);
	
	error = sensor_write_reg(client, sensor->ops->ctrl_reg, MODE_PS_Active);
	
	msleep(WAKEUP_DELAY);

	return error;
}

// Put PS into Standby mode
static int sensor_ps_disable(struct i2c_client *client)
{
	int error;
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);

	error = sensor_write_reg(client, sensor->ops->ctrl_reg, MODE_PS_StdBy);
	return error;
}

/****************operate according to sensor chip:start************/

static int sensor_active_ps(struct i2c_client *client, int enable, int rate)
{
	int result = 0;

	DBG("[ltr559ps] %s,enable = %d\n", __func__, enable);
	if (enable)
		result = sensor_ps_enable(client);
	else
		result = sensor_ps_disable(client);

	return result;
}

static int sensor_init_ps(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);
	int result = 0;
	char value = 0;

	DBG("[ltr559ps] %s:line=%d\n", __func__, __LINE__);
	
	sensor_write_reg(client, APS_RW_PS_N_PULSES, 0x04);
	sensor_write_reg(client, APS_RW_PS_LED, 0x7F);
	sensor_write_reg(client, APS_RW_INTERRUPT, 0x03);
	sensor_write_reg(client, APS_RW_INTERRUPT_PERSIST, 0x22);

	sensor_write_reg(client, APS_RW_PS_THRES_UP_0, ps_hi_thres & 0xFF);
	sensor_write_reg(client, APS_RW_PS_THRES_UP_1, (ps_hi_thres >> 8) & 0xff);
	sensor_write_reg(client, APS_RW_PS_THRES_LOW_0, ps_low_thres & 0xFF);
	sensor_write_reg(client, APS_RW_PS_THRES_LOW_1, (ps_low_thres >> 8) & 0xff);

	result = sensor->ops->active(client, 0, 0);
	if (result)
	{
		DBG("[ltr559ps] %s:line=%d,error\n", __func__, __LINE__);
		return result;
	}

	sensor->status_cur = SENSOR_OFF;

	return result;
}

static int ps_report_value(struct input_dev *input, int data)
{
	if (data >= ps_hi_thres)
	{
		near_far_value = 0;
	}
	else
	{		
		near_far_value = 1;
	}

	input_report_abs(input, ABS_DISTANCE, near_far_value);	
	input_sync(input);

	return near_far_value;
}

static int sensor_report_value_ps(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);
	int result = 0;
	char value = 0;
	char index = 0;

	if (sensor->pdata->irq_enable)
	{
		if (sensor->ops->int_status_reg)
		{
			value = sensor_read_reg(client, sensor->ops->int_status_reg);
		}
	}
	
	result = sensor_ps_read(client);
	index = ps_report_value(sensor->input_dev, result); // 11bit is ps data;

	DBG("[ltr559ps] %s:%s result=0x%x,index=%d\n", __func__, sensor->ops->name, result, index);

	return result;
}

struct sensor_operate proximity_ltr559_ops = {
	.name				= "ps_ltr559",
	.type				= SENSOR_TYPE_PROXIMITY,//sensor type and it should be correct
	.id_i2c				= PROXIMITY_ID_LTR559,	//i2c id number
	.read_reg			= SENSOR_UNKNOW_DATA,	//read data
	.read_len			= 2,					//data length
	.id_reg				= APS_RO_PART_ID,		//read device id from this register
	.id_data 			= 0x92,					//device id
	.precision			= 11,					//16 bits
	.ctrl_reg 			= APS_RW_PS_CONTR,		//enable or disable
	.int_status_reg 	= APS_RO_ALS_PS_STATUS,	//interrupt status register
	.range				= {0,2047},				//range	
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
	.active				= sensor_active_ps,
	.init				= sensor_init_ps,
	.report				= sensor_report_value_ps,
};

/****************operate according to sensor chip:end************/
static struct sensor_operate *proximity_get_ops(void)
{
	return &proximity_ltr559_ops;
}

static int __init proximity_ltr559_init(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int result = 0;
	int type = ops->type;		
	
	printk("++++++++++++++++++++++++++++++++++++++++++++++++ proximity_ltr559_init\n");
	result = sensor_register_slave(type, NULL, NULL, proximity_get_ops);

	return result;
}

static void __exit proximity_ltr559_exit(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int type = ops->type;

	sensor_unregister_slave(type, NULL, NULL, proximity_get_ops);
}


module_init(proximity_ltr559_init);
module_exit(proximity_ltr559_exit);

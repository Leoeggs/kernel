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

#define DEBUG_SWITCH 1

#if DEBUG_SWITCH
#define DBG_printf(fmt,arg...)           printk(KERN_WARNING "<<-SENSORS-INFO->> "fmt"\n",##arg)
#else
#define DBG_printf(fmt,arg...)
#endif

//static struct miscdevice light_dev_device;
extern int sensor_register_slave(int type, struct i2c_client *client,
			struct sensor_platform_data *slave_pdata,
			struct sensor_operate *(*get_sensor_ops)(void));


extern int sensor_unregister_slave(int type, struct i2c_client *client,
			struct sensor_platform_data *slave_pdata,
			struct sensor_operate *(*get_sensor_ops)(void));
			
int sensor_als_read(struct i2c_client *client)
{	
	int alsval_ch0_lo, alsval_ch0_hi, alsval_ch0;
	int alsval_ch1_lo, alsval_ch1_hi, alsval_ch1;
	int luxdata_int;
	int ratio;

	alsval_ch1_lo = sensor_read_reg(client, APS_RO_ALS_DATA_CH1_0);
	alsval_ch1_hi = sensor_read_reg(client, APS_RO_ALS_DATA_CH1_1);
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;

	alsval_ch0_lo = sensor_read_reg(client, APS_RO_ALS_DATA_CH0_0);
	alsval_ch0_hi = sensor_read_reg(client, APS_RO_ALS_DATA_CH0_1);
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
	
	if ((alsval_ch1 == 0) || (alsval_ch0 == 0))
	{
		luxdata_int = 0;
		goto out;
	}

	ratio = (alsval_ch1 * 100) / (alsval_ch0 + alsval_ch1);

	if (ratio < 45) {
		luxdata_int = (((17743 * alsval_ch0) + (11059 * alsval_ch1))) / 1000;
	}
	else if ((ratio < 64) && (ratio >= 45)) {
		luxdata_int = (((42785 * alsval_ch0) - (19548 * alsval_ch1))) / 1000;
	}
	else if ((ratio < 85) && (ratio >= 64)) {
		luxdata_int = (((5926 * alsval_ch0) + (1185 * alsval_ch1))) / 1000;
	}
	else {
		luxdata_int = 0;
	}

	DBG_printf("[ltr559] %s luxdata_int=%d\n",__func__,luxdata_int);
out:
	return luxdata_int;
}

static int sensor_als_enable(struct i2c_client *client)
{
	int error;
	int gainrange = 1;

	struct sensor_private_data *sensor =
                (struct sensor_private_data *) i2c_get_clientdata(client);
	if (gainrange == 1)
		error = sensor_write_reg(client, sensor->ops->ctrl_reg, MODE_ALS_ON_Range1);
	else if (gainrange == 2)
		error = sensor_write_reg(client, sensor->ops->ctrl_reg, MODE_ALS_ON_Range2);
	else if (gainrange == 4)
		error = sensor_write_reg(client, sensor->ops->ctrl_reg, MODE_ALS_ON_Range3);
	else if (gainrange == 8)
		error = sensor_write_reg(client, sensor->ops->ctrl_reg, MODE_ALS_ON_Range4);
	else if (gainrange == 48)
		error = sensor_write_reg(client, sensor->ops->ctrl_reg, MODE_ALS_ON_Range5);
	else if (gainrange == 96)
		error = sensor_write_reg(client, sensor->ops->ctrl_reg, MODE_ALS_ON_Range6);
	else
		error = -1;

	msleep(WAKEUP_DELAY);

	return error;
}

// Put ALS into Standby mode
static int sensor_als_disable(struct i2c_client *client)
{
	int error;
	struct sensor_private_data *sensor =
                (struct sensor_private_data *) i2c_get_clientdata(client);

	error = sensor_write_reg(client, sensor->ops->ctrl_reg, MODE_ALS_StdBy);
	return error;
}

/****************operate according to sensor chip:start************/

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	int result = 0;

    DBG_printf("[ltr559als] %s,enable = %d\n",__func__,enable);
	if(enable)
		result = sensor_als_enable(client);
    else
		result = sensor_als_disable(client);
	
	return result;
}
#if	0

static int light_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int light_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}



static const struct file_operations light_dev_fops = {
	.owner = THIS_MODULE,
	.open = light_dev_open,
	.release = light_dev_release,
	
};

static struct miscdevice light_dev_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltr559als_dev",
	.fops = &light_dev_fops,
};

#endif
static int sensor_init(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);
	int result = 0;
//	char value = 0;

    DBG_printf("[ltr559als] %s:line=%d\n",__func__,__LINE__);	
	
	sensor_write_reg(client, APS_RW_ALS_THRES_UP_0, 0xFF);
	sensor_write_reg(client, APS_RW_ALS_THRES_UP_1, 0xFF);
	sensor_write_reg(client, APS_RW_ALS_THRES_LOW_0, 0x00);
	sensor_write_reg(client, APS_RW_ALS_THRES_LOW_1, 0x00);

	result = sensor->ops->active(client,0,0);
	//misc_register(&light_dev_device);
	if(result)
	{
		DBG_printf("[ltr559als] %s:line=%d,error\n",__func__,__LINE__);
		return result;
	}

	sensor->status_cur = SENSOR_OFF;

	sensor->client->addr = LTR559_I2C_SLAVE_ADDR;
	return result;
}

static int light_report_value(struct input_dev *input, int data)
{
	unsigned char index = 0;

    if (data < 0)
	{
		DBG_printf("[ltr559als] light val err\n");
		data = 0; // no light
	}

	if(data <= 10){
		index = 0;goto report;
	}
	else if(data <= 160){
		index = 1;goto report;
	}
	else if(data <= 225){
		index = 2;goto report;
	}
	else if(data <= 320){
		index = 3;goto report;
	}
	else if(data <= 640){
		index = 4;goto report;
	}
	else if(data <= 1280){
		index = 5;goto report;
	}
	else if(data <= 2600){
		index = 6;goto report;
	}
	else{
		index = 7;goto report;
	}

report:
	input_report_abs(input, ABS_MISC, data);
	input_sync(input);
	return index;
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);
	int result = 0;
	char value = 0;
	char index = 0;

	if(sensor->pdata->irq_enable)
	{
		if(sensor->ops->int_status_reg)
		{
			value = sensor_read_reg(client, sensor->ops->int_status_reg);
		}
	}

	//result = sensor->ops->active(client,1,0);
	result = sensor_als_read(client);
	index = light_report_value(sensor->input_dev, result); // 16bit is ls data;

//	DBG_printf("[ltr559als] %s:%s result=0x%x,index=%d\n",__func__,sensor->ops->name, result,index);

	return result;
}

struct sensor_operate light_ltr559_ops = {
	.name				= "light_ltr553",
	.type				= SENSOR_TYPE_LIGHT,	//sensor type and it should be correct
	.id_i2c				= LIGHT_ID_LTR553ALS,		//i2c id number
	.read_reg			= SENSOR_UNKNOW_DATA,	//read data
	.read_len			= 2,					//data length
	.id_reg				= APS_RO_PART_ID,		//read device id from this register
	.id_data 			= 0x92,					//device id
	.precision			= 16,					//16 bits
	.ctrl_reg 			= APS_RW_ALS_CONTR,		//enable or disable
	.int_status_reg 	= APS_RO_ALS_PS_STATUS,	//interrupt status register
	.range				= {0,65535},			//range
	.brightness         = {10,255},             // brightness
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
	.active				= sensor_active,
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/
static int light_ltr553_probe(struct i2c_client *client,
				   const struct i2c_device_id *devid)
{

	return sensor_register_device(client, NULL, devid, &light_ltr559_ops);
}

static int light_ltr553_remove(struct i2c_client *client)
{
	return sensor_unregister_device(client, NULL, &light_ltr559_ops);
}

static const struct i2c_device_id light_ltr553_id[] = {
	{"light_ltr553", LIGHT_ID_LTR553ALS},
	{}
};

static struct i2c_driver light_ltr553_driver = {
	.probe = light_ltr553_probe,
	.remove = light_ltr553_remove,
	.shutdown = sensor_shutdown,
	.id_table = light_ltr553_id,
	.driver = {
		.name = "light_ltr553",
	#ifdef CONFIG_PM
		.pm = &sensor_pm_ops,
	#endif
	},
};

module_i2c_driver(light_ltr553_driver);

MODULE_AUTHOR("luowei <lw@rock-chips.com>");
MODULE_DESCRIPTION("itr_ltr553 driver");
MODULE_LICENSE("GPL");

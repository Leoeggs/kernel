/* drivers/input/sensors/access/kxtik.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
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
#include <linux/err.h>
#include<linux/timer.h>
#include <linux/sensor-dev.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"
#define ALS_CMD 	0x01
#define ALS_DT1		0x02
#define ALS_DT2		0X03
#define ALS_THDH1	0X04
#define ALS_THDH2	0X05
#define ALS_THDL1	0X06
#define ALS_THDL2	0X07
#define STA_TUS		0X08
#define PS_CMD		0X09
#define PS_DT		0X0A
#define PS_THDH		0X0B
#define PS_THDL		0X0C
#define SW_RESET	0X80

//ALS_CMD
#define ALS_SD_ENABLE	(0<<0)
#define ALS_SD_DISABLE	(1<<0)
#define ALS_INT_DISABLE	(0<<1)
#define ALS_INT_ENABLE	(1<<1)
#define ALS_1T_100MS	(0<<2)
#define ALS_2T_200MS	(1<<2)
#define ALS_4T_400MS	(2<<2)
#define ALS_8T_800MS	(3<<2)
#define ALS_RANGE_57671	(0<<6)
#define ALS_RANGE_28836	(1<<6)

//PS_CMD
#define PS_SD_ENABLE	(0<<0)
#define PS_SD_DISABLE	(1<<0)
#define PS_INT_DISABLE	(0<<1)
#define PS_INT_ENABLE	(1<<1)
#define PS_10T_2MS	(0<<2)
#define PS_15T_3MS	(1<<2)
#define PS_20T_4MS	(2<<2)
#define PS_25T_5MS	(3<<2)
#define PS_CUR_100MA	(0<<4)
#define PS_CUR_200MA	(1<<4)
#define PS_SLP_10MS	(0<<5)
#define PS_SLP_30MS	(1<<5)
#define PS_SLP_90MS	(2<<5)
#define PS_SLP_270MS	(3<<5)
#define TRIG_PS_OR_LS	(0<<7)
#define TRIG_PS_AND_LS	(1<<7)

/**
 * @defgroup ErrCode Errors code shown on display
 * @{
 */
#define ERR_DETECT             -1
#define ERR_DEMO_RANGE_ONE     1
#define ERR_DEMO_RANGE_MULTI   2


typedef enum {
	LONG_RANGE 		= 0, /*!< Long range mode */
	HIGH_SPEED 		= 1, /*!< High speed mode */
	HIGH_ACCURACY	= 2, /*!< High accuracy mode */
} RangingConfig_e;

typedef enum {
	RANGE_VALUE 	= 0, /*!< Range displayed in cm */
	BAR_GRAPH 		= 1, /*!< Range displayed as a bar graph : one bar per sensor */
} DemoMode_e;

#define DEBUG_SWITCH 0

#if DEBUG_SWITCH
#define debug_printf(fmt,arg...)           printk(KERN_WARNING "<<-SENSORS-INFO->> "fmt"\n",##arg)
#else
#define debug_printf(fmt,arg...)
#endif

#define warn_printf(fmt,arg...)           printk(KERN_WARNING "<<-SENSORS-INFO->> "fmt"\n",##arg)

#define HAL_Delay(u)		 udelay(u)

//STA_TUS
#define STA_PS_INT	(1<<5)
#define	STA_ALS_INT	(1<<4)

static VL53L0X_DEV gDev;
static VL53L0X_RangingMeasurementData_t RangingMeasurementData;
int LeakyFactorFix8 = (int)( 0.0 *256); //(int)( 0.6 *256);
int nDevPresent=0;
static struct miscdevice proximity_dev_device;
 static int gLeakyRange = 0;
static int ps_dowork( VL53L0X_DEV s);

void HandleError(int err){
    debug_printf("VL53L0X error %d \n",err);
  
}


/**
 * Reset all sensor then do presence detection
 *
 * All present devices are data initiated and assigned to their final I2C address
 * @return
 */
int DetectSensors(int SetDisplay) {
    int i;
    uint16_t Id;
    int status;
    int FinalAddress;
	uint8_t d88;
     VL53L0X_Dev_t *pDev;
    /* Reset all */
    nDevPresent = 0;
  

    /* detect all sensors (even on-board)*/
  	i = 0;
   
        pDev = gDev;
        pDev->I2cDevAddr = 0x29;
        pDev->Present = 0;
      
        HAL_Delay(2);
        FinalAddress=0x52+(i+1)*2;

        do {
        	/* Set I2C standard mode (400 KHz) before doing the first register access */
        
        	status = VL53L0X_WrByte(pDev, 0x88, 0x00);
		  status = VL53L0X_RdByte(pDev, 0x88, &d88);
 		 debug_printf("#0x88 =%x\n",  d88);

		
        	/* Try to read one register using default 0x52 address */
            status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
			  debug_printf("known ID %x\n", Id);
            if (status) {
                debug_printf("#status %d Read id fail\n", status);
                break;
            }
            if (Id == 0xEEAA) {
				/* Sensor is found => Change its I2C address to final one */
           //     status = VL53L0X_SetDeviceAddress(pDev,FinalAddress);
                if (status != 0) {
                    debug_printf("# VL53L0X_SetDeviceAddress fail %d\n", status);
                    break;
                }
            //    pDev->I2cDevAddr = FinalAddress /2;
                /* Check all is OK with the new I2C address and initialize the sensor */
                status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
                if (status != 0) {
					   debug_printf("# VL53L0X_SetDeviceAddress fail %d\n", status);
					break;
				}

                status = VL53L0X_DataInit(pDev);
                if( status == 0 ){
                    pDev->Present = 1;
                }
                else{
                    debug_printf("VL53L0X_DataInit status %d fail\n", status);
                    break;
                }
                debug_printf("VL53L0X %d Present and initiated to final 0x%x\n", pDev->Id, pDev->I2cDevAddr);
                nDevPresent++;
                pDev->Present = 1;
            }
            else {
                debug_printf("#%d unknown ID %x\n", i, Id);
                status = 1;
            }
        } while (0);
        /* if fail r can't use for any reason then put the  device back to reset */
        if (status) {
            debug_printf("VL53L0X_DataInit  if fail r can't use for any reason then put the  device back to reset %d fail\n", status);
        }
 
    /* Display detected sensor(s) */
  
        udelay(1000);
  
    return nDevPresent;
}



/**
 *  Setup all detected sensors for single shot mode and setup ranging configuration
 */
static void SetupSingleShot(RangingConfig_e rangingConfig){
  
    int status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

  	debug_printf("SetupSingleShot+++\n");
        if( gDev->Present){
            status=VL53L0X_StaticInit(gDev);
            if( status ){
                debug_printf("VL53L0X_StaticInit  failed\n");
            }

            status = VL53L0X_PerformRefCalibration(gDev, &VhvSettings, &PhaseCal);
			if( status ){
			   debug_printf("VL53L0X_PerformRefCalibration failed\n");
			}

			status = VL53L0X_PerformRefSpadManagement(gDev, &refSpadCount, &isApertureSpads);
			if( status ){
			   debug_printf("VL53L0X_PerformRefSpadManagement failed\n");
			}

            status = VL53L0X_SetDeviceMode(gDev, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
            if( status ){
               debug_printf("VL53L0X_SetDeviceMode failed\n");
            }

            status = VL53L0X_SetLimitCheckEnable(gDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
			}

			status = VL53L0X_SetLimitCheckEnable(gDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
			}
			/* Ranging configuration */
            switch(rangingConfig) {
            case LONG_RANGE:
            	signalLimit = (FixPoint1616_t)(0.1*65536);
            	sigmaLimit = (FixPoint1616_t)(60*65536);
            	timingBudget = 33000;
            	preRangeVcselPeriod = 18;
            	finalRangeVcselPeriod = 14;
            	break;
            case HIGH_ACCURACY:
				signalLimit = (FixPoint1616_t)(0.25*65536);
				sigmaLimit = (FixPoint1616_t)(18*65536);
				timingBudget = 200000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
            case HIGH_SPEED:
				signalLimit = (FixPoint1616_t)(0.25*65536);
				sigmaLimit = (FixPoint1616_t)(32*65536);
				timingBudget = 20000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
            default:
            	debug_printf("Not Supported");
            }

            status = VL53L0X_SetLimitCheckValue(gDev,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckValue failed\n");
			}

			status = VL53L0X_SetLimitCheckValue(gDev,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
			if( status ){
			   debug_printf("VL53L0X_SetLimitCheckValue failed\n");
			}

            status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(gDev,  timingBudget);
            if( status ){
               debug_printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
            }

            status = VL53L0X_SetVcselPulsePeriod(gDev,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
			if( status ){
			   debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
			}

            status = VL53L0X_SetVcselPulsePeriod(gDev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
			if( status ){
			   debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
			}

			status = VL53L0X_PerformRefCalibration(gDev, &VhvSettings, &PhaseCal);
			if( status ){
			   debug_printf("VL53L0X_PerformRefCalibration failed\n");
			}

            gDev->LeakyFirst=1;
        }
   
}

/* Store new ranging data into the device structure, apply leaky integrator if needed */
static void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange){
    if( pRange->RangeStatus == 0 ){
        if( pDev->LeakyFirst ){
            pDev->LeakyFirst = 0;
            pDev->LeakyRange = pRange->RangeMilliMeter;
        }
        else{
            pDev->LeakyRange = (pDev->LeakyRange*LeakyFactorFix8 + (256-LeakyFactorFix8)*pRange->RangeMilliMeter)>>8;
        }
    }
    else{
        pDev->LeakyFirst = 1;
    }
}

/**
 * Implement the ranging demo with all modes managed through the blue button (short and long press)
 * This function implements a while loop until the blue button is pressed
 * @param UseSensorsMask Mask of any sensors to use if not only one present
 * @param rangingConfig Ranging configuration to be used (same for all sensors)
 */
 
#define MAX_COUNT 4
static int RangeDemo(int UseSensorsMask, RangingConfig_e rangingConfig){
    int over=0;
    int status;
	int count = 0;
	int avlid = 0;
	int unavlid = 0;
		struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(gDev->I2cHandle);
		
  //	debug_printf("%s result=0x%x,index=%d\n",__func__,UseSensorsMask,rangingConfig);
    /* Setup all sensors in Single Shot mode */
    SetupSingleShot(rangingConfig);

    /* Start ranging until blue button is pressed */
    do{
   
     
            /* only one sensor */
        	/* Call All-In-One blocking API function */
            status = VL53L0X_PerformSingleRangingMeasurement(gDev,&RangingMeasurementData);
            if( status ==0 ){
            	/* Push data logging to UART */
            	debug_printf("%d,%d,%d,%d\n", gDev->Id,  RangingMeasurementData.RangeStatus, RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps);
            	Sensor_SetNewRange(gDev,&RangingMeasurementData);
		  count = 0;
                /* Display distance in cm */
            	if( RangingMeasurementData.RangeStatus == 0 ){
			avlid++;	
                      debug_printf( "%3d",(int)gDev->LeakyRange);
            		if(avlid>MAX_COUNT && gDev->LeakyRange<=1500 )
			{
				gLeakyRange = gDev->LeakyRange;
				sensor->ops->report(sensor->client);
				avlid = 0;	

				 debug_printf( "send:%3d",(int)gLeakyRange);

            		}
			unavlid = 0;
                }
                else{
                  	avlid = 0;
			if(unavlid>2){
				gLeakyRange = 0;
				sensor->ops->report(sensor->client);
				unavlid = 0;
			}
			unavlid++;
                }
            }
            else{

		   debug_printf( "%s status:%d",__func__,status);
                HandleError(ERR_DEMO_RANGE_ONE);
				msleep(100);
				  count++;
				  unavlid = 0;
            }
   	   msleep(10);
	 
        /* Check blue button */
        if(gDev->force_end_work ){
            over=1;
	     unavlid = 0;
            break;
        }
    }while( !over && count<10);
      return status;
}


static int ps_dowork(VL53L0X_DEV s)
{ 
 
	
   	printk(KERN_ALERT "--ps_dowork---in---\n");
 	


		while(gDev->force_end_work)
		{
			msleep(100);
			printk(KERN_ALERT "--ps_dowork---waiting---\n");
		}
	
		
	if (!s->force_end_work && !work_pending(&s->work) && !freezing(current))
	{

		queue_work(s->workqueue, &s->work);//
	
		printk(KERN_ALERT "--ps_dowork---ok---\n");
    
		return 1;	
	}
	else
	{

  
	  printk(KERN_ALERT "--ps_dowork---error---\n");
   
	  return 0;
 
	}


}

static void ps_work(struct work_struct *w)
{ 

	  RangingConfig_e RangingConfig =HIGH_SPEED;// HIGH_SPEED;//HIGH_ACCURACY //HIGH_SPEED
// 	 DemoMode_e DemoMode = RANGE_VALUE;
  int UseSensorsMask = 1<<1;
//	mdelay(20000);
	while(!gDev->force_end_work)
	{
		DetectSensors(1);
 		 RangeDemo(UseSensorsMask,RangingConfig);
		 msleep(200);
	}
	gDev->force_end_work = 0;
}
/****************operate according to sensor chip:start************/

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	
	int result = 0;

	debug_printf("%s:enable=%d\n",__func__, enable);

	//register setting according to chip datasheet
	if(!enable)
	{
	//	status = PS_SD_DISABLE;
	//	sensor->ops->ctrl_data |= status;
		gDev->force_end_work = 1;
	}
	else
	{
	//	status = ~PS_SD_DISABLE;
	//	sensor->ops->ctrl_data &= status;
		
		ps_dowork(gDev);
	}

	


//	if(enable)
//	sensor->ops->report(sensor->client);

	return result;

}

static int proximity_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int proximity_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}



static const struct file_operations proximity_dev_fops = {
	.owner = THIS_MODULE,
	.open = proximity_dev_open,
	.release = proximity_dev_release,
	
};

static struct miscdevice proximity_dev_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vl53l0x_dev",
	.fops = &proximity_dev_fops,
};



static int sensor_init(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);
		struct device_node *np = client->dev.of_node;
	  int rst_pin;
	   enum of_gpio_flags rst_flags;
	int result = 0;
 	debug_printf("%s:line=%d,addr:0x%x\n",__func__,__LINE__,client->addr);
	gDev->I2cHandle = client;
	gDev->I2cDevAddr = client->addr;
//	result = sensor->ops->active(client,1,200*1000);

	rst_pin = of_get_named_gpio_flags(np, "reset-gpio", 0, &rst_flags);
	if(rst_pin)
	{
		result = gpio_request(rst_pin,"PS_RESET");
		
		if(result <0)
		{
				warn_printf("%s:line=%d,request reset gpio error\n",__func__,__LINE__);
		}
		else
		{
			 gpio_direction_output(rst_pin,1);
			 msleep(10);
			 gpio_direction_output(rst_pin,0);
			 msleep(20);
			 gpio_direction_output(rst_pin,1);	   
			 warn_printf("%s:line=%d,reset gpio OK\n",__func__,__LINE__);
			 result = 0;
		}
	}
	else
	{
	   	warn_printf("%s:line=%d,request of_get_named_gpio_flags reset error\n",__func__,__LINE__);
	}
	
	 misc_register(&proximity_dev_device);


	if(result)
	{
		debug_printf("%s:line=%d\n",__func__,__LINE__);
		return result;
	}

	sensor->status_cur = SENSOR_OFF;


	return result;
}



static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);
	int result = 0;
	input_report_abs(sensor->input_dev, ABS_DISTANCE, gLeakyRange);
	input_sync(sensor->input_dev);

	debug_printf("%s:%s result=%d,\n",__func__,sensor->ops->name, gLeakyRange);
	result = gLeakyRange;
	return result;
}

struct sensor_operate proximity_vl53l0x_ops = {
	.name				= "ps_vl53l0x",
	.type				= SENSOR_TYPE_PROXIMITY,	//sensor type and it should be correct
	.id_i2c				= PROXIMITY_ID_VL53L0X,		//i2c id number
	.read_reg			= PS_DT,			//read data
	.read_len			= 1,				//data length
	.id_reg				= SENSOR_UNKNOW_DATA,		//read device id from this register
	.id_data 			= SENSOR_UNKNOW_DATA,		//device id
	.precision			= 8,				//8 bits
	.ctrl_reg 			= PS_CMD,			//enable or disable
	.int_status_reg 		= STA_TUS,			//intterupt status register
	.range				= {0,1500},			//range
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
	.active				= sensor_active,
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/
#if 0
//function name should not be changed
static struct sensor_operate *proximity_get_ops(void)
{
	debug_printf("%s:line=%d\n",__func__,__LINE__);
	return &proximity_vl53l0x_ops;
}

static int __init proximity_vl53l0x_init(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int result = 0;
	int type = ops->type;
	debug_printf("%s:line=%d\n",__func__,__LINE__);
  	 gDev = kzalloc(sizeof(*gDev), GFP_KERNEL);
    if (gDev == NULL)
    {
        debug_printf("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }
    
   	 memset(gDev, 0, sizeof(*gDev));
  	 gDev->force_end_work = 0;
   	 gDev->Present =1;
	    gDev->workqueue = create_singlethread_workqueue("ps_vl53l0x");
		
        if (!gDev->workqueue) 
	   {
        	//dev_warn(&s->spi_wk->dev, "cannot create workqueue\n");
        	return -EBUSY;
        }

	 INIT_WORK(&gDev->work, ps_work);
		 
	 printk("++++++++++++++++++++++++++++++++++++++++++++++++ pppproximity_vl53l0x_init\n");
	result = sensor_register_slave(type, NULL, NULL, proximity_get_ops);
	return result;
}

static void __exit proximity_vl53l0x_exit(void)
{
	struct sensor_operate *ops = proximity_get_ops();
	int type = ops->type;
debug_printf("%s:line=%d\n",__func__,__LINE__);
		gDev->force_end_work = 1;
	if (gDev->workqueue) 
	{
		flush_workqueue(gDev->workqueue);
		destroy_workqueue(gDev->workqueue);
		gDev->workqueue = NULL;
	}
	sensor_unregister_slave(type, NULL, NULL, proximity_get_ops);
}


module_init(proximity_vl53l0x_init);
module_exit(proximity_vl53l0x_exit);

#endif

/****************operate according to sensor chip:end************/
static int proximity_vl53l0x_probe(struct i2c_client *client,
				   const struct i2c_device_id *devid)
{


	debug_printf("%s:line=%d\n",__func__,__LINE__);
  	 gDev = kzalloc(sizeof(*gDev), GFP_KERNEL);
    if (gDev == NULL)
    {
        debug_printf("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }
    
   	 memset(gDev, 0, sizeof(*gDev));
  	 gDev->force_end_work = 0;
   	 gDev->Present =1;
	    gDev->workqueue = create_singlethread_workqueue("ps_vl53l0x");
		
        if (!gDev->workqueue) 
	   {
        	//dev_warn(&s->spi_wk->dev, "cannot create workqueue\n");
        	return -EBUSY;
        }

	 INIT_WORK(&gDev->work, ps_work);
	return sensor_register_device(client, NULL, devid, &proximity_vl53l0x_ops);
}

static int proximity_vl53l0x_remove(struct i2c_client *client)
{

	debug_printf("%s:line=%d\n",__func__,__LINE__);
		gDev->force_end_work = 1;
	if (gDev->workqueue) 
	{
		flush_workqueue(gDev->workqueue);
		destroy_workqueue(gDev->workqueue);
		gDev->workqueue = NULL;
	}
	return sensor_unregister_device(client, NULL, &proximity_vl53l0x_ops);
}

static const struct i2c_device_id proximity_vl53l0x_id[] = {
	{"ps_vl53l0x", PROXIMITY_ID_VL53L0X},
	{}
};

static struct i2c_driver proximity_vl53l0x_driver = {
	.probe = proximity_vl53l0x_probe,
	.remove = proximity_vl53l0x_remove,
	.shutdown = sensor_shutdown,
	.id_table = proximity_vl53l0x_id,
	.driver = {
		.name = "proximity_vl53l0x",
	#ifdef CONFIG_PM
		.pm = &sensor_pm_ops,
	#endif
	},
};

module_i2c_driver(proximity_vl53l0x_driver);

MODULE_AUTHOR("luowei <lw@rock-chips.com>");
MODULE_DESCRIPTION("ps_vl53l0x proximity driver");
MODULE_LICENSE("GPL");


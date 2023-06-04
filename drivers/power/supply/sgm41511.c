// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
//#include <mt-plat/mtk_boot.h>
//#include <mt-plat/upmu_common.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include "sgm41511.h"
//#include "charger_class.h"
//#include "mtk_charger.h"
/**********************************************************
 *
 *   [I2C Slave Setting]
 *
 *********************************************************/

#define SGM41511_REG_NUM    (0xB)

/* SGM41511 REG06 BOOST_LIM[5:4], uV */
static const unsigned int BOOST_VOLT_LIMIT[] = {
	4850000, 5000000, 5150000, 5300000		
};
 /* SGM41511 REG02 BOOST_LIM[7:7], uA */
#if (defined(__SGM41542_CHIP_ID__) || defined(__SGM41541_CHIP_ID__)|| defined(__SGM41543_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	1200000, 2000000
};
#else
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	500000, 1200000
};
#endif

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))

static const unsigned int IPRECHG_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};

static const unsigned int ITERM_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};
#endif

static enum power_supply_usb_type SGM41511_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,	
};

/*static const struct charger_properties SGM41511_chg_props = {
	.alias_name = SGM41511_NAME,
};*/

/**********************************************************
 *
 *   [Global Variable]
 *
 *********************************************************/
static struct power_supply_desc SGM41511_power_supply_desc;
//static struct SGM41511_device *s_chg_dev_otg;
static struct SGM41511_device *sgm;
/**********************************************************
 *
 *   [I2C Function For Read/Write SGM41511]
 *
 *********************************************************/
static int __SGM41511_read_byte(struct SGM41511_device *sgm, u8 reg, u8 *data)
{
    s32 ret;

    ret = i2c_smbus_read_byte_data(sgm->client, reg);
    if (ret < 0) {
        pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
        return ret;
    }

    *data = (u8) ret;

    return 0;
}

static int __SGM41511_write_byte(struct SGM41511_device *sgm, int reg, u8 val)
{
    s32 ret;

    ret = i2c_smbus_write_byte_data(sgm->client, reg, val);
    if (ret < 0) {
        pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
               val, reg, ret);
        return ret;
    }
    return 0;
}

static int SGM41511_read_reg(struct SGM41511_device *sgm, u8 reg, u8 *data)
//int SGM41511_read_reg(struct SGM41511_device *sgm, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __SGM41511_read_byte(sgm, reg, data);
	mutex_unlock(&sgm->i2c_rw_lock);

	return ret;
}

static int SGM41511_write_reg(struct SGM41511_device *sgm, u8 reg, u8 val)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __SGM41511_write_byte(sgm, reg, val);
	mutex_unlock(&sgm->i2c_rw_lock);

	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
	//lgh add
	else 
	    pr_err("Write ok: %s: reg=0x%02X, val=0x%x\n", __func__,reg, val);
	// lgh end
	return ret;
}

static int SGM41511_update_bits(struct SGM41511_device *sgm, u8 reg,
					u8 mask, u8 val)
{
	int ret;
	u8 tmp;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __SGM41511_read_byte(sgm, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= val & mask;

	ret = __SGM41511_write_byte(sgm, reg, tmp);
	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);
	//lgh add
	else 
	    pr_err("Write ok: %s: reg=0x%02X, val=0x%x\n", __func__,reg, tmp);
	// lgh end
out:
	mutex_unlock(&sgm->i2c_rw_lock);
	return ret;
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
 #if 0
 static int SGM41511_set_watchdog_timer(struct SGM41511_device *sgm, int time)
{
	int ret;
	u8 reg_val;

	if (time == 0)
		reg_val = SGM41511_WDT_TIMER_DISABLE;
	else if (time == 40)
		reg_val = SGM41511_WDT_TIMER_40S;
	else if (time == 80)
		reg_val = SGM41511_WDT_TIMER_80S;
	else
		reg_val = SGM41511_WDT_TIMER_160S;	

	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_5,
				SGM41511_WDT_TIMER_MASK, reg_val);

	return ret;
}


 static int SGM41511_get_term_curr(struct SGM41511_device *sgm)
{
	int ret;
	u8 reg_val;
	int curr;
	int offset = SGM41511_TERMCHRG_I_MIN_uA;

	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_CTRL_3, &reg_val);
	if (ret)
		return ret;

	reg_val &= SGM41511_TERMCHRG_CUR_MASK;
	curr = reg_val * SGM41511_TERMCHRG_CURRENT_STEP_uA + offset;
	return curr;
}

static int SGM41511_get_prechrg_curr(struct SGM41511_device *sgm)
{
	int ret;
	u8 reg_val;
	int curr;
	int offset = SGM41511_PRECHRG_I_MIN_uA;

	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_CTRL_3, &reg_val);
	if (ret)
		return ret;

	reg_val = (reg_val&SGM41511_PRECHRG_CUR_MASK)>>4;
	curr = reg_val * SGM41511_PRECHRG_CURRENT_STEP_uA + offset;
	return curr;
}

static int SGM41511_get_ichg_curr(struct SGM41511_device *sgm)
{
	int ret;
	u8 ichg;
    unsigned int curr;
	
	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_CTRL_2, &ichg);
	if (ret)
		return ret;	

	ichg &= SGM41511_ICHRG_I_MASK;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))	
	if (ichg <= 0x8)
		curr = ichg * 5000;
	else if (ichg <= 0xF)
		curr = 40000 + (ichg - 0x8) * 10000;
	else if (ichg <= 0x17)
		curr = 110000 + (ichg - 0xF) * 20000;
	else if (ichg <= 0x20)
		curr = 270000 + (ichg - 0x17) * 30000;
	else if (ichg <= 0x30)
		curr = 540000 + (ichg - 0x20) * 60000;
	else if (ichg <= 0x3C)
		curr = 1500000 + (ichg - 0x30) * 120000;
	else
		curr = 3000000;
#else
	curr = ichg * SGM41511_ICHRG_I_STEP_uA;
#endif	
	return curr;
}
#endif

static int SGM41511_set_term_curr(struct SGM41511_device *sgm, int uA)
{
	u8 reg_val;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))	
	
	for(reg_val = 1; reg_val < 16 && uA >= ITERM_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#else
	if (uA < SGM41511_TERMCHRG_I_MIN_uA)
		uA = SGM41511_TERMCHRG_I_MIN_uA;
	else if (uA > SGM41511_TERMCHRG_I_MAX_uA)
		uA = SGM41511_TERMCHRG_I_MAX_uA;
	
	reg_val = (uA - SGM41511_TERMCHRG_I_MIN_uA) / SGM41511_TERMCHRG_CURRENT_STEP_uA;
#endif

	return SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_3,
				  SGM41511_TERMCHRG_CUR_MASK, reg_val);
}

static int SGM41511_set_prechrg_curr(struct SGM41511_device *sgm, int uA)
{
	u8 reg_val;
	
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	for(reg_val = 1; reg_val < 16 && uA >= IPRECHG_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#else
	if (uA < SGM41511_PRECHRG_I_MIN_uA)
		uA = SGM41511_PRECHRG_I_MIN_uA;
	else if (uA > SGM41511_PRECHRG_I_MAX_uA)
		uA = SGM41511_PRECHRG_I_MAX_uA;

	reg_val = (uA - SGM41511_PRECHRG_I_MIN_uA) / SGM41511_PRECHRG_CURRENT_STEP_uA;
#endif
	reg_val = reg_val << 4;
	return SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_3,
				  SGM41511_PRECHRG_CUR_MASK, reg_val);
}

static int SGM41511_set_ichrg_curr(struct SGM41511_device *sgm, unsigned int uA)
{
	int ret;
	u8 reg_val;
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	if (uA < SGM41511_ICHRG_I_MIN_uA)
		uA = SGM41511_ICHRG_I_MIN_uA;
	else if ( uA > sgm->init_data.max_ichg)
		uA = sgm->init_data.max_ichg;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	if (uA <= 40000)
		reg_val = uA / 5000;	
	else if (uA <= 110000)
		reg_val = 0x08 + (uA -40000) / 10000;	
	else if (uA <= 270000)
		reg_val = 0x0F + (uA -110000) / 20000;	
	else if (uA <= 540000)
		reg_val = 0x17 + (uA -270000) / 30000;	
	else if (uA <= 1500000)
		reg_val = 0x20 + (uA -540000) / 60000;	
	else if (uA <= 2940000)
		reg_val = 0x30 + (uA -1500000) / 120000;
	else 
		reg_val = 0x3d;
#else

	reg_val = uA / SGM41511_ICHRG_I_STEP_uA;
#endif
	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_2,
				  SGM41511_ICHRG_I_MASK, reg_val);
	
	return ret;
}

static int SGM41511_set_chrg_volt(struct SGM41511_device *sgm, unsigned int chrg_volt)
{
	int ret;
	u8 reg_val;
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	if (chrg_volt < SGM41511_VREG_V_MIN_uV)
		chrg_volt = SGM41511_VREG_V_MIN_uV;
	else if (chrg_volt > sgm->init_data.max_vreg)
		chrg_volt = sgm->init_data.max_vreg;
	
	
	reg_val = (chrg_volt-SGM41511_VREG_V_MIN_uV) / SGM41511_VREG_V_STEP_uV;
	reg_val = reg_val<<3;
	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_4,
				  SGM41511_VREG_V_MASK, reg_val);

	return ret;
}

static int SGM41511_get_chrg_volt(struct SGM41511_device *sgm,unsigned int *volt)
{
	int ret;
	u8 vreg_val;
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_CTRL_4, &vreg_val);
	if (ret)
		return ret;	

	vreg_val = (vreg_val & SGM41511_VREG_V_MASK)>>3;

	if (15 == vreg_val)
		*volt = 4352000; //default
	else if (vreg_val < 25)	
		*volt = vreg_val*SGM41511_VREG_V_STEP_uV + SGM41511_VREG_V_MIN_uV;	

	return 0;
}

/*static int SGM41511_get_vindpm_offset_os(struct SGM41511_device *sgm)
{
	int ret;
	u8 reg_val;

	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_CTRL_f, &reg_val);
	if (ret)
		return ret;	

	reg_val = reg_val & SGM41511_VINDPM_OS_MASK;	

	return reg_val;
}

static int SGM41511_set_vindpm_offset_os(struct SGM41511_device *sgm,u8 offset_os)
{
	int ret;	
	
	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_f,
				  SGM41511_VINDPM_OS_MASK, offset_os);
	
	if (ret){
		pr_err("%s fail\n",__func__);
		return ret;
	}
	
	return ret;
}*/
static int SGM41511_set_input_volt_lim(struct SGM41511_device *sgm, unsigned int vindpm)
{
	int ret;
	unsigned int offset;
	u8 reg_val;
	u8 os_val;
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	if (vindpm < SGM41511_VINDPM_V_MIN_uV ||
	    vindpm > SGM41511_VINDPM_V_MAX_uV)
 		return -EINVAL;	
	
	if (vindpm < 5900000){
		os_val = 0;
		offset = 3900000;
	}		
	else if (vindpm >= 5900000 && vindpm < 7500000){
		os_val = 1;
		offset = 5900000; //uv
	}		
	else if (vindpm >= 7500000 && vindpm < 10500000){
		os_val = 2;
		offset = 7500000; //uv
	}		
	else{
		os_val = 3;
		offset = 10500000; //uv
	}		
	
	//SGM41511_set_vindpm_offset_os(sgm,os_val);
	reg_val = (vindpm - offset) / SGM41511_VINDPM_STEP_uV;	

	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_6,
				  SGM41511_VINDPM_V_MASK, reg_val); 

	return ret;
}
#if 0
static int SGM41511_get_input_volt_lim(struct SGM41511_device *sgm)
{
	int ret;
	int offset;
	u8 vlim;
	int temp;

	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_CTRL_6, &vlim);
	if (ret)
		return ret;
	
	temp = SGM41511_get_vindpm_offset_os(sgm);
	if (0 == temp)
		offset = 3900000; //uv
	else if (1 == temp)
		offset = 5900000;
	else if (2 == temp)
		offset = 7500000;
	else if (3 == temp)
		offset = 10500000;
	
	temp = offset + (vlim & 0x0F) * SGM41511_VINDPM_STEP_uV;
	return temp;
}
#endif

static int SGM41511_set_input_curr_lim(struct SGM41511_device *sgm, unsigned int iindpm)
{
	int ret;
	u8 reg_val;
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	if (iindpm < SGM41511_IINDPM_I_MIN_uA ||
			iindpm > SGM41511_IINDPM_I_MAX_uA)
		return -EINVAL;	

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
	reg_val = (iindpm-SGM41511_IINDPM_I_MIN_uA) / SGM41511_IINDPM_STEP_uA;
#else		
	if (iindpm >= SGM41511_IINDPM_I_MIN_uA && iindpm <= 3100000)//default
		reg_val = (iindpm-SGM41511_IINDPM_I_MIN_uA) / SGM41511_IINDPM_STEP_uA;
	else if (iindpm > 3100000 && iindpm < SGM41511_IINDPM_I_MAX_uA)
		reg_val = 0x1E;
	else
		reg_val = SGM41511_IINDPM_I_MASK;
#endif
	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_0,
				  SGM41511_IINDPM_I_MASK, reg_val);
	return ret;
}

static int SGM41511_get_input_curr_lim(struct SGM41511_device *sgm,unsigned int *ilim)
{
	int ret;	
	u8 reg_val;
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_CTRL_0, &reg_val);
	if (ret)
		return ret;	
	if (SGM41511_IINDPM_I_MASK == (reg_val & SGM41511_IINDPM_I_MASK))
		*ilim =  SGM41511_IINDPM_I_MAX_uA;
	else
		*ilim = (reg_val & SGM41511_IINDPM_I_MASK)*SGM41511_IINDPM_STEP_uA + SGM41511_IINDPM_I_MIN_uA;

	return 0;
}

static int SGM41511_get_state(struct SGM41511_device *sgm,
			     struct SGM41511_state *state)
{
	u8 chrg_stat;
	u8 fault;
	u8 chrg_param_0,chrg_param_1,chrg_param_2;
	int ret;

	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_STAT, &chrg_stat);
	if (ret){
		ret = SGM41511_read_reg(sgm, SGM41511_CHRG_STAT, &chrg_stat);
		if (ret){
			pr_err("%s read SGM41511_CHRG_STAT fail\n",__func__);
			return ret;
		}
	}
	state->chrg_type = chrg_stat & SGM41511_VBUS_STAT_MASK;
	state->chrg_stat = chrg_stat & SGM41511_CHG_STAT_MASK;
	state->online = !!(chrg_stat & SGM41511_PG_STAT);
	//state->therm_stat = !!(chrg_stat & SGM41511_THERM_STAT);
	state->vsys_stat = !!(chrg_stat & SGM41511_VSYS_STAT);
	
	pr_err("%s chrg_type =0x%x,chrg_stat =0x%x online = %d\n",__func__,state->chrg_type,state->chrg_stat,state->online);
	
	

	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_FAULT, &fault);
	
	if (ret){
		pr_err("%s read SGM41511_CHRG_FAULT fail\n",__func__);
		return ret;
	}
	//lgh add read 0x09 twice
	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_FAULT, &fault);
	if (ret){
		pr_err("%s read SGM41511_CHRG_FAULT fail\n",__func__);
		return ret;
	}
    pr_err("-----0x09=0x%x -----\n",fault);
		
    //lgh end
	state->chrg_fault = fault;	
	state->ntc_fault = fault & SGM41511_TEMP_MASK;
	state->health = state->ntc_fault;

	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_CTRL_0, &chrg_param_0);
	if (ret){
		pr_err("%s read SGM41511_CHRG_CTRL_0 fail\n",__func__);
		return ret;
	}
	state->hiz_en = !!(chrg_param_0 & SGM41511_HIZ_EN);
	
	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_CTRL_5, &chrg_param_1);
	if (ret){
		pr_err("%s read SGM41511_CHRG_CTRL_5 fail\n",__func__);
		return ret;
	}
	state->term_en = !!(chrg_param_1 & SGM41511_TERM_EN);
	
	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_CTRL_a, &chrg_param_2);
	if (ret){
		pr_err("%s read SGM41511_CHRG_CTRL_a fail\n",__func__);
		return ret;
	}
	state->vbus_gd = !!(chrg_param_2 & SGM41511_VBUS_GOOD);

	return 0;
}

static int SGM41511_set_hiz_en(struct SGM41511_device *sgm, bool hiz_en)
{
	u8 reg_val;
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	dev_notice(sgm->dev, "%s:%d", __func__, hiz_en);
	reg_val = hiz_en ? SGM41511_HIZ_EN : 0;

	return SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_0,
				  SGM41511_HIZ_EN, reg_val);
}

static int SGM41511_enable_charger(struct SGM41511_device *sgm)
{
    int ret;
    
    ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_1, SGM41511_CHRG_EN,
                     SGM41511_CHRG_EN);
	gpio_direction_output(sgm->sgm41511_charger_en,0);//lgh add -enble charger
    return ret;
}

static int SGM41511_disable_charger(struct SGM41511_device *sgm)
{
    int ret;
    
    ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_1, SGM41511_CHRG_EN,
                     0);
	gpio_direction_output(sgm->sgm41511_charger_en,1);//lgh add -disable charger				 
    return ret;
}

static int SGM41511_charging_switch(struct SGM41511_device *sgm,bool enable)
{
	int ret;
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	if (enable)
		ret = SGM41511_enable_charger(sgm);
	else
		ret = SGM41511_disable_charger(sgm);
	return ret;
}

static int SGM41511_set_recharge_volt(struct SGM41511_device *sgm, int mV)
{
	u8 reg_val;
	
	reg_val = (mV - SGM41511_VRECHRG_OFFSET_mV) / SGM41511_VRECHRG_STEP_mV;

	return SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_4,
				  SGM41511_VRECHARGE, reg_val);
}

static int SGM41511_set_wdt_rst(struct SGM41511_device *sgm, bool is_rst)
{
	u8 val;
	
	if (is_rst)
		val = SGM41511_WDT_RST_MASK;
	else
		val = 0;
	return SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_1,
				  SGM41511_WDT_RST_MASK, val);	
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int SGM41511_dump_register(struct SGM41511_device *sgm)
//int SGM41511_dump_register(struct SGM41511_device *sgm)
{

	unsigned char i = 0;
	unsigned int ret = 0;
	unsigned char SGM41511_reg[SGM41511_REG_NUM] = { 0 }; 
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
		
	for (i = 0; i < SGM41511_REG_NUM+1; i++) {
		ret = SGM41511_read_reg(sgm,i, &SGM41511_reg[i]);
	 //if (ret == 0) {
			if (ret<0){
			pr_info("[SGM41511] i2c transfor error\n");
			return 1;
		}
		pr_info("%s,[0x%x]=0x%x ",__func__, i, SGM41511_reg[i]);
	}
	
	return 0;
}


/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int SGM41511_hw_chipid_detect(struct SGM41511_device *sgm)
{
	int ret = 0;
	u8 val = 0;
	ret = SGM41511_read_reg(sgm,SGM41511_CHRG_CTRL_b,&val);
	if (ret < 0)
	{
		pr_info("[%s] read SGM41511_CHRG_CTRL_b fail\n", __func__);
		return ret;
	}		
	val = val & SGM41511_PN_MASK;
	pr_info("[%s] Reg[0x0B]=0x%x\n", __func__,val);
	
	return val;
}

static int SGM41511_reset_watch_dog_timer(struct SGM41511_device
		*sgm)
{
	int ret;
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);

	pr_info("charging_reset_watch_dog_timer\n");

	ret = SGM41511_set_wdt_rst(sgm,0x1);	/* RST watchdog */	

	return ret;
}


static int SGM41511_get_charging_status(struct SGM41511_device *sgm,
				       bool *is_done)
{
	//struct SGM41511_state state;
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	//SGM41511_get_state(sgm, &state);

	if (sgm->state.chrg_stat == SGM41511_TERM_CHRG)
		*is_done = true;
	else
		*is_done = false;

	return 0;
}

static int SGM41511_set_en_timer(struct SGM41511_device *sgm)
{
	int ret;	

	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_5,
				SGM41511_SAFETY_TIMER_EN, SGM41511_SAFETY_TIMER_EN);

	return ret;
}

static int SGM41511_set_disable_timer(struct SGM41511_device *sgm)
{
	int ret;	

	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_5,
				SGM41511_SAFETY_TIMER_EN, 0);

	return ret;
}

static int SGM41511_enable_safetytimer(struct SGM41511_device *sgm,bool en)
{
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	int ret = 0;

	if (en)
		ret = SGM41511_set_en_timer(sgm);
	else
		ret = SGM41511_set_disable_timer(sgm);
	return ret;
}

static int SGM41511_get_is_safetytimer_enable(struct SGM41511_device
		*sgm,bool *en)
{
	int ret = 0;
	u8 val = 0;
	
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	ret = SGM41511_read_reg(sgm,SGM41511_CHRG_CTRL_5,&val);
	if (ret < 0)
	{
		pr_info("[%s] read SGM41511_CHRG_CTRL_5 fail\n", __func__);
		return ret;
	}
	*en = !!(val & SGM41511_SAFETY_TIMER_EN);
	return 0;
}

#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
static int SGM41511_en_pe_current_partern(struct SGM41511_device
		*sgm,bool is_up)
{
	int ret = 0;	
	
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_d,
				SGM41511_EN_PUMPX, SGM41511_EN_PUMPX);
	if (ret < 0)
	{
		pr_info("[%s] read SGM41511_CHRG_CTRL_d fail\n", __func__);
		return ret;
	}
	if (is_up)
		ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_d,
				SGM41511_PUMPX_UP, SGM41511_PUMPX_UP);
	else
		ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_d,
				SGM41511_PUMPX_DN, SGM41511_PUMPX_DN);
	return ret;
}
#endif

static enum power_supply_property SGM41511_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	//lgh POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_PRESENT
};

static int SGM41511_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		return true;
	default:
		return false;
	}
}
static int SGM41511_charger_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	//struct SGM41511_device *sgm = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		//ret = SGM41511_set_input_curr_lim(s_chg_dev_otg, val->intval);
		ret = SGM41511_set_input_curr_lim(sgm, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		//SGM41511_charging_switch(s_chg_dev_otg,val->intval);
		SGM41511_charging_switch(sgm,val->intval);	
		break;
	/*case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		//ret = SGM41511_set_input_volt_lim(s_chg_dev_otg, val->intval);
		ret = SGM41511_set_input_volt_lim(sgm, val->intval);
		break;*/
	default:
		return -EINVAL;
	}

	return ret;
}

static int SGM41511_charger_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct SGM41511_device *sgm = power_supply_get_drvdata(psy);
	struct SGM41511_state state;
	int ret = 0;

	mutex_lock(&sgm->lock);
	//ret = SGM41511_get_state(sgm, &state);
	state = sgm->state;
	mutex_unlock(&sgm->lock);
	if (ret)
		return ret;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.chrg_type || (state.chrg_type == SGM41511_OTG_MODE))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (!state.chrg_stat)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (state.chrg_stat == SGM41511_TERM_CHRG)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (state.chrg_stat) {		
		case SGM41511_PRECHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case SGM41511_FAST_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;		
		case SGM41511_TERM_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case SGM41511_NOT_CHRGING:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = SGM41511_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = SGM41511_NAME;
		break;
	
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = state.vbus_gd;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = SGM41511_power_supply_desc.type;
		break;
	/*case POWER_SUPPLY_PROP_USB_TYPE:
		//val->intval = get_charger_type(sgm);
		
		break; lgh*/

	case POWER_SUPPLY_PROP_HEALTH:
		if (state.chrg_fault & 0xF8)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;

		switch (state.health) {
		case SGM41511_TEMP_HOT:
			val->intval = POWER_SUPPLY_HEALTH_HOT;
			break;
		case SGM41511_TEMP_WARM:
			val->intval = POWER_SUPPLY_HEALTH_WARM;
			break;
		case SGM41511_TEMP_COOL:
			val->intval = POWER_SUPPLY_HEALTH_COOL;
			break;
		case SGM41511_TEMP_COLD:
			val->intval = POWER_SUPPLY_HEALTH_COLD;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		//val->intval = state.vbus_adc;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		//val->intval = state.ibus_adc;
		break;

/*	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = SGM41511_get_input_volt_lim(sgm);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;*/

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:		
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !state.hiz_en;
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

#if 0
static bool SGM41511_state_changed(struct SGM41511_device *sgm,
				  struct SGM41511_state *new_state)
{
	struct SGM41511_state old_state;

	mutex_lock(&sgm->lock);
	old_state = sgm->state;
	mutex_unlock(&sgm->lock);

	return (old_state.chrg_type != new_state->chrg_type ||
		old_state.chrg_stat != new_state->chrg_stat     ||		
		old_state.online != new_state->online		    ||
		old_state.therm_stat != new_state->therm_stat	||		
		old_state.vsys_stat != new_state->vsys_stat 	||
		old_state.chrg_fault != new_state->chrg_fault	
		);
}
#endif

static void charger_monitor_work_func(struct work_struct *work)
{
	int ret = 0;
	struct SGM41511_device * sgm = NULL;
	struct delayed_work *charge_monitor_work = NULL;
	//static u8 last_chg_method = 0;
	struct SGM41511_state state;

	charge_monitor_work = container_of(work, struct delayed_work, work);
	if(charge_monitor_work == NULL) {
		pr_err("Cann't get charge_monitor_work\n");
		return ;
	}
	sgm = container_of(charge_monitor_work, struct SGM41511_device, charge_monitor_work);
	if(sgm == NULL) {
		pr_err("Cann't get sgm \n");
		return ;
	}

	ret = SGM41511_get_state(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);
	
	if(!sgm->state.vbus_gd) {
		dev_err(sgm->dev, "Vbus not present, disable charge\n");
		SGM41511_disable_charger(sgm);
		goto OUT;
	}
	if(!state.online)
	{
		dev_err(sgm->dev, "Vbus not online\n");		
		goto OUT;
	}	
	SGM41511_dump_register(sgm);
	pr_err("%s\n",__func__);
OUT:	
	schedule_delayed_work(&sgm->charge_monitor_work, 10*HZ);
	
}

static void charger_detect_work_func(struct work_struct *work)
{
	struct delayed_work *charge_detect_delayed_work = NULL;
	struct SGM41511_device * sgm = NULL;
	//static int charge_type_old = 0;
	int curr_in_limit = 0;	
	struct SGM41511_state state;	
	int ret;
	
	charge_detect_delayed_work = container_of(work, struct delayed_work, work);
	if(charge_detect_delayed_work == NULL) {
		pr_err("Cann't get charge_detect_delayed_work\n");
		return ;
	}
	sgm = container_of(charge_detect_delayed_work, struct SGM41511_device, charge_detect_delayed_work);
	if(sgm == NULL) {
		pr_err("Cann't get SGM41511_device\n");
		return ;
	}

	if (!sgm->charger_wakelock->active)
		__pm_stay_awake(sgm->charger_wakelock);

	ret = SGM41511_get_state(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;	
	mutex_unlock(&sgm->lock);	
	
	if(!sgm->state.vbus_gd) {
		dev_err(sgm->dev, "Vbus not present, disable charge\n");
		SGM41511_disable_charger(sgm);
		goto err;
	} 
	//lgh add 2023-4-6
	if(sgm->state.chrg_stat == 0x18){
		dev_err(sgm->dev, "Charging terminated, disable charge\n");
		SGM41511_disable_charger(sgm);
		goto err;
	}//lgh end

	if(!state.online)
	{
		dev_err(sgm->dev, "Vbus not online\n");		
		goto err;
	}	
//lgh #if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
	switch(sgm->state.chrg_type) {
		case SGM41511_USB_SDP:
			pr_err("SGM41511 charger type: SDP\n");
			curr_in_limit = 500000;
			break;

		case SGM41511_USB_CDP:
			pr_err("SGM41511 charger type: CDP\n");
			curr_in_limit = 1500000;
			break;

		case SGM41511_USB_DCP:
			pr_err("SGM41511 charger type: DCP\n");
			curr_in_limit = 2000000;
			break;

		case SGM41511_UNKNOWN:
			pr_err("SGM41511 charger type: UNKNOWN\n");
			curr_in_limit = 500000;
			break;	

		default:
			pr_err("SGM41511 charger type: default\n");
			//curr_in_limit = 500000;
			//break;
			return;
	}

	//set charge parameters
	dev_err(sgm->dev, "Update: curr_in_limit = %d\n", curr_in_limit);
	dev_err(sgm->dev, "chrg_type = 0x%x\n", sgm->state.chrg_type);//lgh
	SGM41511_set_input_curr_lim(sgm, curr_in_limit);
	
//lgh #endif
	//enable charge
	SGM41511_enable_charger(sgm);

	SGM41511_dump_register(sgm);
	dev_err(sgm->dev, "dump ok:  %s\n", __func__);//lgh 
	power_supply_changed(sgm->charger);	
	return;
err:
	//release wakelock
	power_supply_changed(sgm->charger);	
	dev_err(sgm->dev, "Relax wakelock\n");
	__pm_relax(sgm->charger_wakelock);
	return;
}

static irqreturn_t SGM41511_irq_handler_thread(int irq, void *private)
{
	struct SGM41511_device *sgm = private;

	//lock wakelock
	pr_err("%s entry\n",__func__);
    
	schedule_delayed_work(&sgm->charge_detect_delayed_work, 100);
	power_supply_changed(sgm->charger);
	
	return IRQ_HANDLED;
}
static char *SGM41511_charger_supplied_to[] = {
	"main-battery",	
};

static struct power_supply_desc SGM41511_power_supply_desc = {
	.name = "SGM41511-charger",
	//.type = POWER_SUPPLY_TYPE_USB,
	.type = POWER_SUPPLY_TYPE_USB_CDP,
	//.usb_types = SGM41511_usb_type,
	//.num_usb_types = ARRAY_SIZE(SGM41511_usb_type),
	.properties = SGM41511_power_supply_props,
	.num_properties = ARRAY_SIZE(SGM41511_power_supply_props),
	.get_property = SGM41511_charger_get_property,
	.set_property = SGM41511_charger_set_property,
	.property_is_writeable = SGM41511_property_is_writeable,
};

static int SGM41511_power_supply_init(struct SGM41511_device *sgm,
							struct device *dev)
{
	struct power_supply_config psy_cfg = { .drv_data = sgm,
						.of_node = dev->of_node, };
	//lgh add
	SGM41511_power_supply_desc.get_property;
	SGM41511_power_supply_desc.set_property;
	//lgh end

	psy_cfg.supplied_to = SGM41511_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(SGM41511_charger_supplied_to);

	sgm->charger = devm_power_supply_register(sgm->dev,
						 &SGM41511_power_supply_desc,
						 &psy_cfg);
	if (IS_ERR(sgm->charger))
		return -EINVAL;
	
	return 0;
}

static int SGM41511_hw_init(struct SGM41511_device *sgm)
{
	int ret = 0;	
	struct power_supply_battery_info bat_info = { };	

	bat_info.constant_charge_current_max_ua =
			SGM41511_ICHRG_I_DEF_uA;

	bat_info.constant_charge_voltage_max_uv =
			SGM41511_VREG_V_DEF_uV;

	bat_info.precharge_current_ua =
			SGM41511_PRECHRG_I_DEF_uA;

	bat_info.charge_term_current_ua =
			SGM41511_TERMCHRG_I_DEF_uA;

	sgm->init_data.max_ichg =
			SGM41511_ICHRG_I_MAX_uA;

	sgm->init_data.max_vreg =
			SGM41511_VREG_V_MAX_uV;

		//lgh add avoid WDG rest
	ret=SGM41511_set_wdt_rst(sgm,0x1);
	if (ret){
		pr_err("%s  fail\n",__func__);
		goto err_out;
	} 
    pr_err("%s good0\n",__func__);//lgh
	//Disable watchdog timer 00
		ret=SGM41511_write_reg(sgm,SGM41511_CHRG_CTRL_5,0x8f);
		if (ret){
		pr_err("%s  fail\n",__func__);
		goto err_out;
	} 
	//set reg01=0x10 rigister sys_min=2.6V
		ret=SGM41511_write_reg(sgm,SGM41511_CHRG_CTRL_1,0x10);
		if (ret){
		pr_err("%s  fail\n",__func__);
		goto err_out;
	} 
	//lgh end
	ret = SGM41511_set_ichrg_curr(sgm,
				bat_info.constant_charge_current_max_ua);
	if (ret){
		//lgh
		pr_err("%s fail\n",__func__);
		goto err_out;
	}
	pr_err("%s good1\n",__func__);//lgh

	
	
	ret = SGM41511_set_prechrg_curr(sgm, bat_info.precharge_current_ua);
	if (ret)
		goto err_out;
    pr_err("%s good2\n",__func__);//lgh
	ret = SGM41511_set_chrg_volt(sgm,
				bat_info.constant_charge_voltage_max_uv);
	if (ret)
		goto err_out;
	pr_err("%s good3\n",__func__);//lgh
	ret = SGM41511_set_term_curr(sgm, bat_info.charge_term_current_ua);
	if (ret)
		goto err_out;
	pr_err("%s good4\n",__func__);//lgh
	ret = SGM41511_set_input_volt_lim(sgm, sgm->init_data.vlim);
	if (ret)
		goto err_out;

	ret = SGM41511_set_input_curr_lim(sgm, sgm->init_data.ilim);
	if (ret)
		goto err_out;
	pr_err("%s good5\n",__func__);//lgh
	#if 0
	ret = SGM41511_set_vac_ovp(sgm);//14V
	if (ret)
		goto err_out;	
	#endif
	ret = SGM41511_set_recharge_volt(sgm, 200);//100~200mv
	if (ret)
		goto err_out;
	pr_err("%s good6\n",__func__);//lgh

	//lgh modify MAST INT
	ret=SGM41511_write_reg(sgm,SGM41511_CHRG_CTRL_a,0x83);
		if (ret){
		pr_err("%s  fail\n",__func__);
		goto err_out;
		}
	//lgh end
	dev_notice(sgm->dev, "ichrg_curr:%d prechrg_curr:%d chrg_vol:%d"
		" term_curr:%d input_curr_lim:%d",
		bat_info.constant_charge_current_max_ua,
		bat_info.precharge_current_ua,
		bat_info.constant_charge_voltage_max_uv,
		bat_info.charge_term_current_ua,
		sgm->init_data.ilim);

	return 0;

err_out:
	return ret;

}

static int SGM41511_parse_dt(struct SGM41511_device *sgm)
{
	int ret;	
	int irq_gpio = 0, irqn = 0;	
	int chg_en_gpio = 0;	
	
	ret = device_property_read_u32(sgm->dev,
				       "input-voltage-limit-microvolt",
				       &sgm->init_data.vlim);
	if (ret)
		sgm->init_data.vlim = SGM41511_VINDPM_DEF_uV;

	if (sgm->init_data.vlim > SGM41511_VINDPM_V_MAX_uV ||
	    sgm->init_data.vlim < SGM41511_VINDPM_V_MIN_uV)
		{//lgh
			return -EINVAL;
			
		dev_err(sgm->dev,"inpu dt failed %s,%d\n",__func__,sgm->init_data.vlim);//lgh

		}
		//lgh add
		dev_err(sgm->dev,"input dt1 ok %s,init_data.vlim=%d\n",__func__,sgm->init_data.vlim);//lgh


	ret = device_property_read_u32(sgm->dev,
				       "input-current-limit-microamp",
				       &sgm->init_data.ilim);
	if (ret)
		sgm->init_data.ilim = SGM41511_IINDPM_DEF_uA;

	if (sgm->init_data.ilim > SGM41511_IINDPM_I_MAX_uA ||
	    sgm->init_data.ilim < SGM41511_IINDPM_I_MIN_uA)
		{
			return -EINVAL;
			//lgh add
		dev_err(sgm->dev,"input dt failed %s,%d\n",__func__,sgm->init_data.ilim);//lgh

		}

		//lgh add
		dev_err(sgm->dev,"input dt2 ok %s,init_data.ilim=%d\n",__func__,sgm->init_data.ilim);//lgh
	
	irq_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio))
	{
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, irq_gpio);
		return -EINVAL;
	}
	ret = gpio_request(irq_gpio, "SGM41511 irq pin");
	if (ret<0) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, irq_gpio);
		return ret;
	}
	gpio_direction_input(irq_gpio);
	irqn = gpio_to_irq(irq_gpio);
	if (irqn < 0) {
		dev_err(sgm->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		return irqn;
	}
	sgm->client->irq = irqn;

	sgm->sgm41511_charger_en = of_get_named_gpio(sgm->dev->of_node, "sgm,chg-en-gpio", 0);
	if (!gpio_is_valid(sgm->sgm41511_charger_en))
	{
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, sgm->sgm41511_charger_en);
		return -EINVAL;
	}
	ret = gpio_request(sgm->sgm41511_charger_en, "sgm chg en pin");
	if (ret<0) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, sgm->sgm41511_charger_en);
		return ret;
	}
	gpio_direction_output(sgm->sgm41511_charger_en,0);//default enable charge
	return 0;
}
//lgh modify 2023-3-6
#if 0
static int SGM41511_enable_vbus(struct regulator_device *rdev)
{	
	int ret = 0;
	//struct SGM41511_device *sgm = charger_get_data(s_chg_dev_otg);
	
	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_1, SGM41511_OTG_EN,
                     SGM41511_OTG_EN);
	return ret;
}

static int SGM41511_disable_vbus(struct regulator_device *rdev)
{
	int ret = 0;
	//struct SGM41511_device *sgm = charger_get_data(s_chg_dev_otg);	
struct SGM41511_device *sgm = charger_get_data(s_chg_dev_otg);
	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_1, SGM41511_OTG_EN,
                     0);

	return ret;
}

static int SGM41511_is_enabled_vbus(struct regulator_device *rdev)

{
	u8 temp = 0;
	int ret = 0;
	//struct SGM41511_device *sgm = charger_get_data(s_chg_dev_otg);	

	ret = SGM41511_read_reg(sgm, SGM41511_CHRG_CTRL_1, &temp);
	return (temp&SGM41511_OTG_EN)? 1 : 0;
}
#endif
//lgh modify 2023-3-6
#if 0
static int SGM41511_enable_otg(struct SGM41511_device *sgm, bool en)
{
	int ret = 0;

	pr_info("%s en = %d\n", __func__, en);
	if (en) {
		ret = SGM41511_enable_vbus(NULL);
	} else {
		ret = SGM41511_disable_vbus(NULL);
	}
	return ret;
}


static int SGM41511_set_boost_voltage_limit(struct charger_device
		*chg_dev, u32 uV)
{	
	int ret = 0;
	char reg_val = -1;
	int i = 0;
	struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	while(i<4){
		if (uV == BOOST_VOLT_LIMIT[i]){
			reg_val = i;
			break;
		}
		i++;
	}
	if (reg_val < 0)
		return reg_val;
	reg_val = reg_val << 4;
	ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_6,
				  SGM41511_BOOSTV, reg_val);

	return ret;
}


static int SGM41511_set_boost_current_limit(struct SGM41511_device *sgm, u32 uA)
{	
	int ret = 0;
	//struct SGM41511_device *sgm = charger_get_data(chg_dev);
	
	if (uA == BOOST_CURRENT_LIMIT[0]){
		ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_2, SGM41511_BOOST_LIM,
                     0); 
	}
		
	else if (uA == BOOST_CURRENT_LIMIT[1]){
		ret = SGM41511_update_bits(sgm, SGM41511_CHRG_CTRL_2, SGM41511_BOOST_LIM,
                     BIT(7)); 
	}
	return ret;
}

static struct regulator_ops SGM41511_vbus_ops = {
	.enable = SGM41511_enable_vbus,
	.disable = SGM41511_disable_vbus,
	.is_enabled = SGM41511_is_enabled_vbus,
};

static const struct regulator_desc SGM41511_otg_rdesc = {
	.of_match = "usb-otg-vbus",
	.name = "usb-otg-vbus",
	.ops = &SGM41511_vbus_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static int SGM41511_vbus_regulator_register(struct SGM41511_device *sgm)
{
	struct regulator_config config = {};
	int ret = 0;
	/* otg regulator */
	config.dev = sgm->dev;
	config.driver_data = sgm;
	sgm->otg_rdev = devm_regulator_register(sgm->dev,
						&SGM41511_otg_rdesc, &config);
	sgm->otg_rdev->constraints->valid_ops_mask |= REGULATOR_CHANGE_STATUS;
	if (IS_ERR(sgm->otg_rdev)) {
		ret = PTR_ERR(sgm->otg_rdev);
		pr_info("%s: register otg regulator failed (%d)\n", __func__, ret);
	}
	return ret;
}

static struct charger_ops SGM41511_chg_ops = {
	.enable_hz = SGM41511_set_hiz_en,

	/* Normal charging */
	.dump_registers = SGM41511_dump_register,
	.enable = SGM41511_charging_switch,
	.get_charging_current = NULL,
	.set_charging_current = SGM41511_set_ichrg_curr,
	.get_input_current = SGM41511_get_input_curr_lim,
	.set_input_current = SGM41511_set_input_curr_lim,
	.get_constant_voltage = SGM41511_get_chrg_volt,
	.set_constant_voltage = SGM41511_set_chrg_volt,
	.kick_wdt = SGM41511_reset_watch_dog_timer,
	.set_mivr = SGM41511_set_input_volt_lim,
	.is_charging_done = SGM41511_get_charging_status,

	/* Safety timer */
	.enable_safety_timer = SGM41511_enable_safetytimer,
	.is_safety_timer_enabled = SGM41511_get_is_safetytimer_enable,


	/* Power path */
	/*.enable_powerpath = SGM41511_enable_power_path, */
	/*.is_powerpath_enabled = SGM41511_get_is_power_path_enable, */


	/* OTG */
	.enable_otg = SGM41511_enable_otg,	
	.set_boost_current_limit = SGM41511_set_boost_current_limit,
	//.event = SGM41511_do_event,
	
	/* PE+/PE+20 */
#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
	.send_ta_current_pattern = SGM41511_en_pe_current_partern,
#else
	.send_ta_current_pattern = NULL,
#endif
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
//	.set_ta20_reset = NULL,
	.enable_cable_drop_comp = NULL,
};
#endif
static int SGM41511_driver_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	u8 val = 0;
	//lgh add
	//struct power_supply_desc *supply_desc;
	//lgh end
	struct device *dev = &client->dev;
	struct SGM41511_device *sgm;

    //char *name = NULL;
	char *name;
	
	pr_info("[%s]\n", __func__);

	sgm = devm_kzalloc(dev, sizeof(*sgm), GFP_KERNEL);
	if (!sgm)
		return -ENOMEM;

	sgm->client = client;
	sgm->dev = dev;	
	
	(&sgm->lock);
	mutex_init(&sgm->i2c_rw_lock);
	
	
	i2c_set_clientdata(client, sgm);
	
	ret = SGM41511_parse_dt(sgm);
	if (ret)
		return ret;
	
	
	ret = SGM41511_hw_chipid_detect(sgm);
	if (ret != SGM41511_PN_ID){
		pr_info("[%s] device not found !!!\n", __func__);
		return ret;
	}	
	

	name = devm_kasprintf(sgm->dev, GFP_KERNEL, "%s","SGM41511 suspend wakelock");
	sgm->charger_wakelock =	wakeup_source_register(NULL, name);
	
	/* Register charger device */
	/*sgm->chg_dev = charger_device_register("primary_chg",
						&client->dev, sgm,
						&SGM41511_chg_ops,
						&SGM41511_chg_props);
	if (IS_ERR_OR_NULL(sgm->chg_dev)) {
		pr_info("%s: register charger device  failed\n", __func__);
		ret = PTR_ERR(sgm->chg_dev);
		return ret;
	}    */
	
	/* otg regulator */
	/*s_chg_dev_otg=sgm->chg_dev; */
	
   
	
	INIT_DELAYED_WORK(&sgm->charge_detect_delayed_work, charger_detect_work_func);
	INIT_DELAYED_WORK(&sgm->charge_monitor_work, charger_monitor_work_func);
	
	//lgh
	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						SGM41511_irq_handler_thread,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(&client->dev), sgm);
		if (ret)
			return ret;
		enable_irq_wake(client->irq);
	}	
	//lgh add
	
	//lgh end
	ret = SGM41511_power_supply_init(sgm, dev);
	if (ret) {
		pr_err("Failed to register power supply\n");
		return ret;
	}
   
	
	ret = SGM41511_hw_init(sgm);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		return ret;
	}
	//lgh
	#if 0

	//OTG setting
	//SGM41511_set_otg_voltage(s_chg_dev_otg, 5000000); //5V
	//SGM41511_set_otg_current(s_chg_dev_otg, 1200000); //1.2A

	//Charger setting
	//SGM41511_set_otg_voltage(s_chg_dev_otg, 5000000); //5V
	//SGM41511_set_otg_current(s_chg_dev_otg, 1200000); //1.2A

	//ret = SGM41511_vbus_regulator_register(sgm); //lgh modify 2023-3-6
	 #endif
	schedule_delayed_work(&sgm->charge_monitor_work,100);

	
	
	return 0; //lgh return ret

}

static int SGM41511_charger_remove(struct i2c_client *client)
{
    struct SGM41511_device *sgm = i2c_get_clientdata(client);

    cancel_delayed_work_sync(&sgm->charge_monitor_work);

    //regulator_unregister(sgm->otg_rdev); lgh modify 2023-3-6

    power_supply_unregister(sgm->charger); 
	
	mutex_destroy(&sgm->lock);
    mutex_destroy(&sgm->i2c_rw_lock);       

    return 0;
}

static void SGM41511_charger_shutdown(struct i2c_client *client)
{
    int ret = 0;
	
	struct SGM41511_device *sgm = i2c_get_clientdata(client);
    ret = SGM41511_disable_charger(sgm);
    if (ret) {
        pr_err("Failed to disable charger, ret = %d\n", ret);
    }
    pr_info("SGM41511_charger_shutdown\n");
}

static const struct i2c_device_id SGM41511_i2c_ids[] = {
	{ "sgm41511", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, SGM41511_i2c_ids);

static const struct of_device_id SGM41511_of_match[] = {
	{ .compatible = "sgm,sgm41511", },
	{ }
};
MODULE_DEVICE_TABLE(of, SGM41511_of_match);


static struct i2c_driver SGM41511_driver = {
	.driver = {
		.name = "SGM41511-charger",
		.of_match_table = SGM41511_of_match,		
	},
	.probe = SGM41511_driver_probe,
	.remove = SGM41511_charger_remove,
	.shutdown = SGM41511_charger_shutdown,
	.id_table = SGM41511_i2c_ids,
};
module_i2c_driver(SGM41511_driver);

MODULE_AUTHOR(" qhq <Allen_qin@sg-micro.com>");
MODULE_DESCRIPTION("SGM41511 charger driver");
MODULE_LICENSE("GPL v2");

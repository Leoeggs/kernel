/* SPDX-License-Identifier: GPL-2.0-only */
// SGM41511 Charger Driver
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.sg-micro.com

#ifndef _SGM41511_CHARGER_H
#define _SGM41511_CHARGER_H

#include <linux/i2c.h>

#define SGM41511_MANUFACTURER	"Texas Instruments"
//#define __SGM41541_CHIP_ID__
#define __SGM41511_CHIP_ID__
//#define __SGM41513_CHIP_ID__
//#define __SGM41513A_CHIP_ID__
//#define __SGM41513D_CHIP_ID__
//#define __SGM41516_CHIP_ID__
//#define __SGM41516D_CHIP_ID__
//#define __SGM41543_CHIP_ID__
//#define __SGM41543D_CHIP_ID__

#ifdef __SGM41511_CHIP_ID__
#define SGM41511_NAME		"sgm41511"
#define SGM41511_PN_ID     0x10
#endif


/*define register*/
#define SGM41511_CHRG_CTRL_0	0x00
#define SGM41511_CHRG_CTRL_1	0x01
#define SGM41511_CHRG_CTRL_2	0x02
#define SGM41511_CHRG_CTRL_3	0x03
#define SGM41511_CHRG_CTRL_4	0x04
#define SGM41511_CHRG_CTRL_5	0x05
#define SGM41511_CHRG_CTRL_6	0x06
#define SGM41511_CHRG_CTRL_7	0x07
#define SGM41511_CHRG_STAT	    0x08
#define SGM41511_CHRG_FAULT	    0x09
#define SGM41511_CHRG_CTRL_a	0x0a
#define SGM41511_CHRG_CTRL_b	0x0b
//#define SGM41511_CHRG_CTRL_c	0x0c
//#define SGM41511_CHRG_CTRL_d	0x0d
//#define SGM41511_INPUT_DET   	0x0e
//#define SGM41511_CHRG_CTRL_f	0x0f

/* charge status flags  */
#define SGM41511_CHRG_EN		BIT(4)
#define SGM41511_HIZ_EN		    BIT(7)
#define SGM41511_TERM_EN		BIT(7)
#define SGM41511_VAC_OVP_MASK	GENMASK(7, 6)
#define SGM41511_DPDM_ONGOING   BIT(7)
#define SGM41511_VBUS_GOOD      BIT(7)

#define SGM41511_BOOSTV 		GENMASK(5, 4)
#define SGM41511_BOOST_LIM 		BIT(7)
#define SGM41511_OTG_EN		    BIT(5)

/* Part ID  */
#define SGM41511_PN_MASK	    GENMASK(6, 3)

/* WDT TIMER SET  */
#define SGM41511_WDT_TIMER_MASK        GENMASK(5, 4)
#define SGM41511_WDT_TIMER_DISABLE     0
#define SGM41511_WDT_TIMER_40S         BIT(4)
#define SGM41511_WDT_TIMER_80S         BIT(5)
#define SGM41511_WDT_TIMER_160S        (BIT(4)| BIT(5))

#define SGM41511_WDT_RST_MASK          BIT(6)

/* SAFETY TIMER SET  */
#define SGM41511_SAFETY_TIMER_MASK     GENMASK(3, 3)
#define SGM41511_SAFETY_TIMER_DISABLE     0
#define SGM41511_SAFETY_TIMER_EN       BIT(3)
#define SGM41511_SAFETY_TIMER_5H         0
#define SGM41511_SAFETY_TIMER_10H      BIT(2)

/* recharge voltage  */
#define SGM41511_VRECHARGE              BIT(0)
#define SGM41511_VRECHRG_STEP_mV		100
#define SGM41511_VRECHRG_OFFSET_mV		100

/* charge status  */
#define SGM41511_VSYS_STAT		BIT(0)
#define SGM41511_THERM_STAT		BIT(1)
#define SGM41511_PG_STAT		BIT(2)
#define SGM41511_CHG_STAT_MASK	GENMASK(4, 3)
#define SGM41511_PRECHRG		BIT(3)
#define SGM41511_FAST_CHRG	    BIT(4)
#define SGM41511_TERM_CHRG	    (BIT(3)| BIT(4))

/* charge type  */
#define SGM41511_VBUS_STAT_MASK	GENMASK(7, 5)
#define SGM41511_NOT_CHRGING	0
#define SGM41511_USB_SDP		BIT(5)
#define SGM41511_USB_CDP		BIT(6)
#define SGM41511_USB_DCP		(BIT(5) | BIT(6))
#define SGM41511_UNKNOWN	    (BIT(7) | BIT(5))
#define SGM41511_NON_STANDARD	(BIT(7) | BIT(6))
#define SGM41511_OTG_MODE	    (BIT(7) | BIT(6) | BIT(5))

/* TEMP Status  */
#define SGM41511_TEMP_MASK	    GENMASK(2, 0)
#define SGM41511_TEMP_NORMAL	BIT(0)
#define SGM41511_TEMP_WARM	    BIT(1)
#define SGM41511_TEMP_COOL	    (BIT(0) | BIT(1))
#define SGM41511_TEMP_COLD	    (BIT(0) | BIT(3))
#define SGM41511_TEMP_HOT	    (BIT(2) | BIT(3))

/* precharge current  */
#define SGM41511_PRECHRG_CUR_MASK		GENMASK(7, 4)
#define SGM41511_PRECHRG_CURRENT_STEP_uA		60000
#define SGM41511_PRECHRG_I_MIN_uA		60000
#define SGM41511_PRECHRG_I_MAX_uA		780000
#define SGM41511_PRECHRG_I_DEF_uA		180000

/* termination current  */
#define SGM41511_TERMCHRG_CUR_MASK		GENMASK(3, 0)
#define SGM41511_TERMCHRG_CURRENT_STEP_uA	60000
#define SGM41511_TERMCHRG_I_MIN_uA		60000
#define SGM41511_TERMCHRG_I_MAX_uA		960000
#define SGM41511_TERMCHRG_I_DEF_uA		180000

/* charge current  */
#define SGM41511_ICHRG_I_MASK		GENMASK(5, 0)

#define SGM41511_ICHRG_I_MIN_uA			0
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__))
#define SGM41511_ICHRG_I_MAX_uA			3000000
#define SGM41511_ICHRG_I_DEF_uA			1980000
#else
#define SGM41511_ICHRG_I_STEP_uA	    60000
#define SGM41511_ICHRG_I_MAX_uA			3780000
#define SGM41511_ICHRG_I_DEF_uA			2000000
#endif
/* charge voltage  */
#define SGM41511_VREG_V_MASK		GENMASK(7, 3)
#define SGM41511_VREG_V_MAX_uV	    4624000
#define SGM41511_VREG_V_MIN_uV	    3856000
#define SGM41511_VREG_V_DEF_uV	    4208000
#define SGM41511_VREG_V_STEP_uV	    32000

/* iindpm current  */
#define SGM41511_IINDPM_I_MASK		GENMASK(4, 0)
#define SGM41511_IINDPM_I_MIN_uA	100000

#define SGM41511_IINDPM_I_MAX_uA	3800000

#define SGM41511_IINDPM_STEP_uA	    100000
#define SGM41511_IINDPM_DEF_uA	    2400000

/* vindpm voltage  */
#define SGM41511_VINDPM_V_MASK      GENMASK(3, 0)
#define SGM41511_VINDPM_V_MIN_uV    3900000
#define SGM41511_VINDPM_V_MAX_uV    12000000
#define SGM41511_VINDPM_STEP_uV     100000
#define SGM41511_VINDPM_DEF_uV	    3600000
#define SGM41511_VINDPM_OS_MASK     GENMASK(1, 0)

/* DP DM SEL  */
#define SGM41511_DP_VSEL_MASK       GENMASK(4, 3)
#define SGM41511_DM_VSEL_MASK       GENMASK(2, 1)

/* PUMPX SET  */
#define SGM41511_EN_PUMPX           BIT(7)
#define SGM41511_PUMPX_UP           BIT(6)
#define SGM41511_PUMPX_DN           BIT(5)

/* customer define jeita paramter */
#define JEITA_TEMP_ABOVE_T4_CV	0
#define JEITA_TEMP_T3_TO_T4_CV	4100000
#define JEITA_TEMP_T2_TO_T3_CV	4350000
#define JEITA_TEMP_T1_TO_T2_CV	4350000
#define JEITA_TEMP_T0_TO_T1_CV	0
#define JEITA_TEMP_BELOW_T0_CV	0

#define JEITA_TEMP_ABOVE_T4_CC_CURRENT	0
#define JEITA_TEMP_T3_TO_T4_CC_CURRENT	1000000
#define JEITA_TEMP_T2_TO_T3_CC_CURRENT	2400000
#define JEITA_TEMP_T1_TO_T2_CC_CURRENT	2000000
#define JEITA_TEMP_T0_TO_T1_CC_CURRENT	0
#define JEITA_TEMP_BELOW_T0_CC_CURRENT	0

#define TEMP_T4_THRES  50
#define TEMP_T4_THRES_MINUS_X_DEGREE 47
#define TEMP_T3_THRES  45
#define TEMP_T3_THRES_MINUS_X_DEGREE 39
#define TEMP_T2_THRES  20
#define TEMP_T2_THRES_PLUS_X_DEGREE 16
#define TEMP_T1_THRES  0
#define TEMP_T1_THRES_PLUS_X_DEGREE 6
#define TEMP_T0_THRES  0
#define TEMP_T0_THRES_PLUS_X_DEGREE  0
#define TEMP_NEG_10_THRES 0

struct SGM41511_init_data {
	u32 ichg;	/* charge current		*/
	u32 ilim;	/* input current		*/
	u32 vreg;	/* regulation voltage		*/
	u32 iterm;	/* termination current		*/
	u32 iprechg;	/* precharge current		*/
	u32 vlim;	/* minimum system voltage limit */
	u32 max_ichg;
	u32 max_vreg;
};

struct SGM41511_state {
	bool vsys_stat;
	//bool therm_stat;
	bool online;	
	u8 chrg_stat;
	//u8 vbus_status;

	bool chrg_en;
	bool hiz_en;
	bool term_en;
	bool vbus_gd;
	u8 chrg_type;
	u8 health;
	u8 chrg_fault;
	u8 ntc_fault;
};

struct SGM41511_jeita {
	int jeita_temp_above_t4_cv;
	int jeita_temp_t3_to_t4_cv;
	int jeita_temp_t2_to_t3_cv;
	int jeita_temp_t1_to_t2_cv;
	int jeita_temp_t0_to_t1_cv;
	int jeita_temp_below_t0_cv;
	int jeita_temp_above_t4_cc_current;
	int jeita_temp_t3_to_t4_cc_current;
	int jeita_temp_t2_to_t3_cc_current;
	int jeita_temp_t1_to_t2_cc_current;
	int jeita_temp_below_t0_cc_current;
	int temp_t4_thres;
	int temp_t4_thres_minus_x_degree;
	int temp_t3_thres;
	int temp_t3_thres_minus_x_degree;
	int temp_t2_thres;
	int temp_t2_thres_plus_x_degree;
	int temp_t1_thres;
	int temp_t1_thres_plus_x_degree;
	int temp_t0_thres;
	int temp_t0_thres_plus_x_degree;
	int temp_neg_10_thres;
};

struct SGM41511_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;	
	struct power_supply *usb;
	struct power_supply *ac;
	struct mutex lock;
	struct mutex i2c_rw_lock;

	struct usb_phy *usb2_phy;
	struct usb_phy *usb3_phy;
	struct notifier_block usb_nb;
	struct work_struct usb_work;
	unsigned long usb_event;
	struct regmap *regmap;

	char model_name[I2C_NAME_SIZE];
	int device_id;

	//lgh add gpio
	struct gpio_desc  *sgm41511_charger_en;
	//end 

	struct SGM41511_init_data init_data;
	struct SGM41511_state state;
	u32 watchdog_timer;
	#if 0//defined(CONFIG_MTK_GAUGE_VERSION) && (CONFIG_MTK_GAUGE_VERSION == 30)
	//struct charger_device *chg_dev;
	#endif
	//struct regulator_dev *otg_rdev; lgh modify 2023-3-6

	struct delayed_work charge_detect_delayed_work;
	struct delayed_work charge_monitor_work;
	struct notifier_block pm_nb;
	bool SGM41511_suspend_flag;

	struct wakeup_source *charger_wakelock;
	//bool enable_sw_jeita;
	//struct SGM41511_jeita data;
};

#endif /* _SGM41511_CHARGER_H */

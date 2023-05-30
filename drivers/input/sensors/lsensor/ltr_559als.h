#ifndef __LTR559_H__
#define __LTR559_H__

#define LTR559_I2C_SLAVE_ADDR	0x23//0x46 // 0x23 << 1

#define DRIVER_VERSION			"1.0"
//#define LTR_PS_CALIBRATION_FILE "/data/misc/ltr_ps_calibration.conf"
/*REG address*/

#define APS_RW_ALS_CONTR		0x80 // ALS operation mode control SW reset
#define APS_RW_PS_CONTR			0x81 // PS operation mode control
#define APS_RW_PS_LED			0x82 // PS LED setting
#define APS_RW_PS_N_PULSES		0x83 // PS number of pulses
#define APS_RW_PS_MEAS_RATE		0x84 // PS measurement rate in active mode
#define APS_RW_ALS_MEAS_RATE	0x85 // ALS measurement rate in active mode
#define APS_RO_PART_ID			0x86 // Part Number ID and Revision ID
#define APS_RO_MANUFAC_ID		0x87 // Manufacturer ID
#define APS_RO_ALS_DATA_CH1_0	0x88 // ALS measurement CH1 data, lower byte
#define APS_RO_ALS_DATA_CH1_1	0x89 // ALS measurement CH1 data, upper byte
#define APS_RO_ALS_DATA_CH0_0	0x8A // ALS measurement CH0 data, lower byte
#define APS_RO_ALS_DATA_CH0_1	0x8B // ALS measurement CH0 data, lower byte
#define APS_RO_ALS_PS_STATUS	0x8C // ALS and PS new data status
#define APS_RO_PS_DATA_0		0x8D // PS measurement data, lower byte
#define APS_RO_PS_DATA_1		0x8E //PS measurement data, upper byte
#define APS_RW_INTERRUPT		0x8F // Interrupt settings
#define APS_RW_PS_THRES_UP_0	0x90 // PS interrupt upper threshold, lower byte
#define APS_RW_PS_THRES_UP_1	0x91 // PS interrupt upper threshold, upper byte
#define APS_RW_PS_THRES_LOW_0	0x92 // PS interrupt lower threshold, lower byte
#define APS_RW_PS_THRES_LOW_1	0x93 // PS interrupt lower threshold, upper byte
#define APS_RW_PS_OFFSET_1		0x94
#define APS_RW_PS_OFFSET_0		0x95
#define APS_RW_ALS_THRES_UP_0	0x97 // ALS interrupt upper threshold, lower byte
#define APS_RW_ALS_THRES_UP_1	0x98 // ALS interrupt lower threshold, upper byte
#define APS_RW_ALS_THRES_LOW_0	0x99 // ALS interrupt upper threshold, lower byte
#define APS_RW_ALS_THRES_LOW_1	0x9A // ALS interrupt lower threshold,upper byte
#define APS_RW_INTERRUPT_PERSIST 0x9E// ALS / PS Interrupt persist setting

/* Basic Operating Modes */
// FIXME:
// We should not enable the als and ps by default

#define MODE_ALS_ON_Range1		0x01  ///for als gain x1
#define MODE_ALS_ON_Range2		0x05  ///for als  gain x2
#define MODE_ALS_ON_Range3		0x09  ///for als  gain x4
#define MODE_ALS_ON_Range4		0x0D  ///for als gain x8
#define MODE_ALS_ON_Range5		0x19  ///for als gain x48
#define MODE_ALS_ON_Range6		0x1D  ///for als gain x96
#define MODE_ALS_StdBy			0x00
#define MODE_PS_Active			(0x03)
#define MODE_PS_StdBy			(0x00)

#define ALS_SW_RESET			(1 << 1)

/* Power On response time in ms */
#define PON_DELAY				600
#define WAKEUP_DELAY			10

#define PARTID					0x92

#endif

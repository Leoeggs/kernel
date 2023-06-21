/*
 * Haoyu AIP8563 RTC driver
 *
 * Copyright (C) 2013 MundoReader S.L.
 * Author: Heiko Stuebner <heiko@sntech.de>
 *
 * based on rtc-AIP8563
 * Copyright (C) 2010 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/clk-provider.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>

#define AIP8563_CTL1		0x00
#define AIP8563_CTL1_TEST	BIT(7)
#define AIP8563_CTL1_STOP	BIT(5)
#define AIP8563_CTL1_TESTC	BIT(3)

#define AIP8563_CTL2		0x01
#define AIP8563_CTL2_TI_TP	BIT(4)
#define AIP8563_CTL2_AF		BIT(3)
#define AIP8563_CTL2_TF		BIT(2)
#define AIP8563_CTL2_AIE	BIT(1)
#define AIP8563_CTL2_TIE	BIT(0)

#define AIP8563_SEC		0x02
#define AIP8563_SEC_VL		BIT(7)
#define AIP8563_SEC_MASK	0x7f

#define AIP8563_MIN		0x03
#define AIP8563_MIN_MASK	0x7f

#define AIP8563_HOUR		0x04
#define AIP8563_HOUR_MASK	0x3f

#define AIP8563_DAY		0x05
#define AIP8563_DAY_MASK	0x3f

#define AIP8563_WEEKDAY		0x06
#define AIP8563_WEEKDAY_MASK	0x07

#define AIP8563_MONTH		0x07
#define AIP8563_MONTH_CENTURY	BIT(7)
#define AIP8563_MONTH_MASK	0x1f

#define AIP8563_YEAR		0x08

#define AIP8563_ALM_MIN		0x09
#define AIP8563_ALM_HOUR	0x0a
#define AIP8563_ALM_DAY		0x0b
#define AIP8563_ALM_WEEK	0x0c

/* Each alarm check can be disabled by setting this bit in the register */
#define AIP8563_ALM_BIT_DISABLE	BIT(7)

#define AIP8563_CLKOUT		0x0d
#define AIP8563_CLKOUT_ENABLE	BIT(7)
#define AIP8563_CLKOUT_32768	0
#define AIP8563_CLKOUT_1024	1
#define AIP8563_CLKOUT_32	2
#define AIP8563_CLKOUT_1	3
#define AIP8563_CLKOUT_MASK	3

#define AIP8563_TMR_CTL		0x0e
#define AIP8563_TMR_CTL_ENABLE	BIT(7)
#define AIP8563_TMR_CTL_4096	0
#define AIP8563_TMR_CTL_64	1
#define AIP8563_TMR_CTL_1	2
#define AIP8563_TMR_CTL_1_60	3
#define AIP8563_TMR_CTL_MASK	3

#define AIP8563_TMR_CNT		0x0f

struct aip8563 {
	struct i2c_client	*client;
	struct rtc_device	*rtc;
#ifdef CONFIG_COMMON_CLK
	struct clk_hw		clkout_hw;
#endif
};

/*
 * RTC handling
 */

static int aip8563_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 buf[7];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, AIP8563_SEC, 7, buf);

	tm->tm_sec = bcd2bin(buf[0] & AIP8563_SEC_MASK);
	tm->tm_min = bcd2bin(buf[1] & AIP8563_MIN_MASK);
	tm->tm_hour = bcd2bin(buf[2] & AIP8563_HOUR_MASK);
	tm->tm_mday = bcd2bin(buf[3] & AIP8563_DAY_MASK);
	tm->tm_wday = bcd2bin(buf[4] & AIP8563_WEEKDAY_MASK); /* 0 = Sun */
	tm->tm_mon = bcd2bin(buf[5] & AIP8563_MONTH_MASK) - 1; /* 0 = Jan */
	tm->tm_year = bcd2bin(buf[6]) + 100;

	return 0;
}

static int aip8563_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 buf[7];
	int ret;

	/* Years >= 2100 are to far in the future, 19XX is to early */
	if (tm->tm_year < 100 || tm->tm_year >= 200)
		return -EINVAL;

	buf[0] = bin2bcd(tm->tm_sec);
	buf[1] = bin2bcd(tm->tm_min);
	buf[2] = bin2bcd(tm->tm_hour);
	buf[3] = bin2bcd(tm->tm_mday);
	buf[4] = bin2bcd(tm->tm_wday);
	buf[5] = bin2bcd(tm->tm_mon + 1);

	/*
	 * While the AIP8563 has a century flag in the month register,
	 * it does not seem to carry it over a subsequent write/read.
	 * So we'll limit ourself to 100 years, starting at 2000 for now.
	 */
	buf[6] = bin2bcd(tm->tm_year - 100);

	/*
	 * CTL1 only contains TEST-mode bits apart from stop,
	 * so no need to read the value first
	 */
	ret = i2c_smbus_write_byte_data(client, AIP8563_CTL1,
						AIP8563_CTL1_STOP);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_i2c_block_data(client, AIP8563_SEC, 7, buf);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(client, AIP8563_CTL1, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int aip8563_rtc_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);
	int data;

	data = i2c_smbus_read_byte_data(client, AIP8563_CTL2);
	if (data < 0)
		return data;

	if (enabled)
		data |= AIP8563_CTL2_AIE;
	else
		data &= ~AIP8563_CTL2_AIE;

	return i2c_smbus_write_byte_data(client, AIP8563_CTL2, data);
};

static int aip8563_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rtc_time *alm_tm = &alm->time;
	u8 buf[4];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, AIP8563_ALM_MIN, 4, buf);
	if (ret < 0)
		return ret;

	/* The alarm only has a minute accuracy */
	alm_tm->tm_sec = 0;

	alm_tm->tm_min = (buf[0] & AIP8563_ALM_BIT_DISABLE) ?
					-1 :
					bcd2bin(buf[0] & AIP8563_MIN_MASK);
	alm_tm->tm_hour = (buf[1] & AIP8563_ALM_BIT_DISABLE) ?
					-1 :
					bcd2bin(buf[1] & AIP8563_HOUR_MASK);
	alm_tm->tm_mday = (buf[2] & AIP8563_ALM_BIT_DISABLE) ?
					-1 :
					bcd2bin(buf[2] & AIP8563_DAY_MASK);
	alm_tm->tm_wday = (buf[3] & AIP8563_ALM_BIT_DISABLE) ?
					-1 :
					bcd2bin(buf[3] & AIP8563_WEEKDAY_MASK);

	ret = i2c_smbus_read_byte_data(client, AIP8563_CTL2);
	if (ret < 0)
		return ret;

	if (ret & AIP8563_CTL2_AIE)
		alm->enabled = 1;

	return 0;
}

static int aip8563_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rtc_time *alm_tm = &alm->time;
	u8 buf[4];
	int ret;

	/*
	 * The alarm has no seconds so deal with it
	 */
	if (alm_tm->tm_sec) {
		alm_tm->tm_sec = 0;
		alm_tm->tm_min++;
		if (alm_tm->tm_min >= 60) {
			alm_tm->tm_min = 0;
			alm_tm->tm_hour++;
			if (alm_tm->tm_hour >= 24) {
				alm_tm->tm_hour = 0;
				alm_tm->tm_mday++;
				if (alm_tm->tm_mday > 31)
					alm_tm->tm_mday = 0;
			}
		}
	}

	ret = i2c_smbus_read_byte_data(client, AIP8563_CTL2);
	if (ret < 0)
		return ret;

	ret &= ~AIP8563_CTL2_AIE;

	ret = i2c_smbus_write_byte_data(client, AIP8563_CTL2, ret);
	if (ret < 0)
		return ret;

	buf[0] = (alm_tm->tm_min < 60 && alm_tm->tm_min >= 0) ?
			bin2bcd(alm_tm->tm_min) : AIP8563_ALM_BIT_DISABLE;

	buf[1] = (alm_tm->tm_hour < 24 && alm_tm->tm_hour >= 0) ?
			bin2bcd(alm_tm->tm_hour) : AIP8563_ALM_BIT_DISABLE;

	buf[2] = (alm_tm->tm_mday <= 31 && alm_tm->tm_mday >= 1) ?
			bin2bcd(alm_tm->tm_mday) : AIP8563_ALM_BIT_DISABLE;

	buf[3] = (alm_tm->tm_wday < 7 && alm_tm->tm_wday >= 0) ?
			bin2bcd(alm_tm->tm_wday) : AIP8563_ALM_BIT_DISABLE;

	ret = i2c_smbus_write_i2c_block_data(client, AIP8563_ALM_MIN, 4, buf);
	if (ret < 0)
		return ret;

	return aip8563_rtc_alarm_irq_enable(dev, alm->enabled);
}

static const struct rtc_class_ops aip8563_rtc_ops = {
	.read_time		= aip8563_rtc_read_time,
	.set_time		= aip8563_rtc_set_time,
	.alarm_irq_enable	= aip8563_rtc_alarm_irq_enable,
	.read_alarm		= aip8563_rtc_read_alarm,
	.set_alarm		= aip8563_rtc_set_alarm,
};

/*
 * Handling of the clkout
 */

#ifdef CONFIG_COMMON_CLK
#define clkout_hw_to_aip8563(_hw) container_of(_hw, struct aip8563, clkout_hw)

static int clkout_rates[] = {
	32768,
	1024,
	32,
	1,
};

static unsigned long aip8563_clkout_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct aip8563 *aip8563 = clkout_hw_to_aip8563(hw);
	struct i2c_client *client = aip8563->client;
	int ret = i2c_smbus_read_byte_data(client, AIP8563_CLKOUT);

	if (ret < 0)
		return 0;

	ret &= AIP8563_CLKOUT_MASK;
	return clkout_rates[ret];
}

static long aip8563_clkout_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *prate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(clkout_rates); i++)
		if (clkout_rates[i] <= rate)
			return clkout_rates[i];

	return 0;
}

static int aip8563_clkout_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct aip8563 *aip8563 = clkout_hw_to_aip8563(hw);
	struct i2c_client *client = aip8563->client;
	int ret = i2c_smbus_read_byte_data(client, AIP8563_CLKOUT);
	int i;

	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(clkout_rates); i++)
		if (clkout_rates[i] == rate) {
			ret &= ~AIP8563_CLKOUT_MASK;
			ret |= i;
			return i2c_smbus_write_byte_data(client,
							 AIP8563_CLKOUT, ret);
		}

	return -EINVAL;
}

static int aip8563_clkout_control(struct clk_hw *hw, bool enable)
{
	struct aip8563 *aip8563 = clkout_hw_to_aip8563(hw);
	struct i2c_client *client = aip8563->client;
	int ret = i2c_smbus_read_byte_data(client, AIP8563_CLKOUT);

	if (ret < 0)
		return ret;

	if (enable)
		ret |= AIP8563_CLKOUT_ENABLE;
	else
		ret &= ~AIP8563_CLKOUT_ENABLE;

	return i2c_smbus_write_byte_data(client, AIP8563_CLKOUT, ret);
}

static int aip8563_clkout_prepare(struct clk_hw *hw)
{
	return aip8563_clkout_control(hw, 1);
}

static void aip8563_clkout_unprepare(struct clk_hw *hw)
{
	aip8563_clkout_control(hw, 0);
}

static int aip8563_clkout_is_prepared(struct clk_hw *hw)
{
	struct aip8563 *aip8563 = clkout_hw_to_aip8563(hw);
	struct i2c_client *client = aip8563->client;
	int ret = i2c_smbus_read_byte_data(client, AIP8563_CLKOUT);

	if (ret < 0)
		return ret;

	return !!(ret & AIP8563_CLKOUT_ENABLE);
}

static const struct clk_ops aip8563_clkout_ops = {
	.prepare = aip8563_clkout_prepare,
	.unprepare = aip8563_clkout_unprepare,
	.is_prepared = aip8563_clkout_is_prepared,
	.recalc_rate = aip8563_clkout_recalc_rate,
	.round_rate = aip8563_clkout_round_rate,
	.set_rate = aip8563_clkout_set_rate,
};

static struct clk *aip8563_clkout_register_clk(struct aip8563 *aip8563)
{
	struct i2c_client *client = aip8563->client;
	struct device_node *node = client->dev.of_node;
	struct clk *clk;
	struct clk_init_data init = {};
	int ret;

	ret = i2c_smbus_write_byte_data(client, AIP8563_CLKOUT,
						0);
	if (ret < 0)
		return ERR_PTR(ret);

	init.name = "aip8563-clkout";
	init.ops = &aip8563_clkout_ops;
	init.flags = 0;
	init.parent_names = NULL;
	init.num_parents = 0;
	aip8563->clkout_hw.init = &init;

	/* optional override of the clockname */
	of_property_read_string(node, "clock-output-names", &init.name);

	/* register the clock */
	clk = clk_register(&client->dev, &aip8563->clkout_hw);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	return clk;
}
#endif

/*
 * The alarm interrupt is implemented as a level-low interrupt in the
 * aip8563, while the timer interrupt uses a falling edge.
 * We don't use the timer at all, so the interrupt is requested to
 * use the level-low trigger.
 */
static irqreturn_t aip8563_irq(int irq, void *dev_id)
{
	struct aip8563 *aip8563 = (struct aip8563 *)dev_id;
	struct i2c_client *client = aip8563->client;
	struct mutex *lock = &aip8563->rtc->ops_lock;
	int data, ret;

	mutex_lock(lock);

	/* Clear the alarm flag */

	data = i2c_smbus_read_byte_data(client, AIP8563_CTL2);
	if (data < 0) {
		dev_err(&client->dev, "%s: error reading i2c data %d\n",
			__func__, data);
		goto out;
	}

	data &= ~AIP8563_CTL2_AF;

	ret = i2c_smbus_write_byte_data(client, AIP8563_CTL2, data);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error writing i2c data %d\n",
			__func__, ret);
	}

out:
	mutex_unlock(lock);
	return IRQ_HANDLED;
}

static int aip8563_init_device(struct i2c_client *client)
{
	int ret;

	/* Clear stop flag if present */
	ret = i2c_smbus_write_byte_data(client, AIP8563_CTL1, 0);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte_data(client, AIP8563_CTL2);
	if (ret < 0)
		return ret;

	/* Disable alarm and timer interrupts */
	ret &= ~AIP8563_CTL2_AIE;
	ret &= ~AIP8563_CTL2_TIE;

	/* Clear any pending alarm and timer flags */
	if (ret & AIP8563_CTL2_AF)
		ret &= ~AIP8563_CTL2_AF;

	if (ret & AIP8563_CTL2_TF)
		ret &= ~AIP8563_CTL2_TF;

	ret &= ~AIP8563_CTL2_TI_TP;

	return i2c_smbus_write_byte_data(client, AIP8563_CTL2, ret);
}

#ifdef CONFIG_PM_SLEEP
static int aip8563_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	if (device_may_wakeup(dev)) {
		ret = enable_irq_wake(client->irq);
		if (ret) {
			dev_err(dev, "enable_irq_wake failed, %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int aip8563_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(client->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(aip8563_pm_ops, aip8563_suspend, aip8563_resume);

static int aip8563_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct aip8563 *aip8563;
	int ret,i;
	/*
	 * aip8563 initial time(2023_1_1_12:00:00),
	 * avoid aip8563 read time error
	 */
	struct rtc_time tm_read, tm = {
		.tm_wday = 0,
		.tm_year = 123,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 12,
		.tm_min = 0,
		.tm_sec = 0,
	};

	aip8563 = devm_kzalloc(&client->dev, sizeof(*aip8563), GFP_KERNEL);
	if (!aip8563)
		return -ENOMEM;

	aip8563->client = client;
	i2c_set_clientdata(client, aip8563);

	device_set_wakeup_capable(&client->dev, true);

	for (i = 0; i < 5; i++) {
		ret = aip8563_init_device(client);
		dev_err(&client->dev, "init device, %d\n", ret);
		if (!ret) 
			break;
		if (4==i) {
			dev_err(&client->dev, "could not init device, %d\n", ret);
			return ret;
		}
	}
	
	if (client->irq > 0) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, aip8563_irq,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						client->name, aip8563);
		if (ret < 0) {
			dev_err(&client->dev, "irq %d request failed, %d\n",
				client->irq, ret);
			return ret;
		}
	}

	/* check state of calendar information */
	ret = i2c_smbus_read_byte_data(client, AIP8563_SEC);
	if (ret < 0)
		return ret;

	dev_dbg(&client->dev, "rtc information is %s\n",
		(ret & AIP8563_SEC_VL) ? "invalid" : "valid");

	aip8563_rtc_read_time(&client->dev, &tm_read);
	if (((tm_read.tm_year < 70) | (tm_read.tm_year > 200)) |
	    (tm_read.tm_mon == -1) | (rtc_valid_tm(&tm_read) != 0))
		aip8563_rtc_set_time(&client->dev, &tm);

	aip8563->rtc = devm_rtc_device_register(&client->dev, client->name,
						&aip8563_rtc_ops, THIS_MODULE);
	if (IS_ERR(aip8563->rtc))
		return PTR_ERR(aip8563->rtc);

	/* the aip8563 alarm only supports a minute accuracy */
	aip8563->rtc->uie_unsupported = 1;

#ifdef CONFIG_COMMON_CLK
	aip8563_clkout_register_clk(aip8563);
#endif

	return 0;
}

static const struct i2c_device_id aip8563_id[] = {
	{ "aip8563", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, aip8563_id);

static const struct of_device_id aip8563_dt_idtable[] = {
	{ .compatible = "wuxi-i-core,aip8563" },
	{},
};
MODULE_DEVICE_TABLE(of, aip8563_dt_idtable);

static struct i2c_driver aip8563_driver = {
	.driver		= {
		.name	= "rtc-aip8563",
		.pm	= &aip8563_pm_ops,
		.of_match_table	= aip8563_dt_idtable,
	},
	.probe		= aip8563_probe,
	.id_table	= aip8563_id,
};

module_i2c_driver(aip8563_driver);

MODULE_AUTHOR("Heiko Stuebner <heiko@sntech.de>");
MODULE_DESCRIPTION("AIP8563 RTC driver");
MODULE_LICENSE("GPL");

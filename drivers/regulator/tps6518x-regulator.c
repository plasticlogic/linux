/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/tps6518x.h>
#include <linux/gpio.h>
#include <linux/pmic_status.h>
#include <linux/of_gpio.h>

struct tps6518x_data {
	int num_regulators;
	struct tps6518x *tps6518x;
	struct regulator_dev **rdev;
};


static int tps6518x_pass_num = { 2 };
static int tps6518x_vcom = { -2680000 };
static int tps65180_current_Enable_Register = 0;

static int tps6518x_is_power_good(struct tps6518x *tps6518x);
/*
 * to_reg_val(): Creates a register value with new data
 *
 * Creates a new register value for a particular field.  The data
 * outside of the new field is not modified.
 *
 * @cur_reg: current value in register
 * @reg_mask: mask of field bits to be modified
 * @fld_val: new value for register field.
 */
static unsigned int to_reg_val(unsigned int cur_reg, unsigned int fld_mask,
							   unsigned int fld_val)
{
	return (cur_reg & (~fld_mask)) | fld_val;
}

/*
 * Regulator operations
 */
/* Convert uV to the VCOM register bitfield setting */

static int vcom_rs_to_uV(unsigned int reg_setting)
{
	if (reg_setting <= TPS65180_VCOM_MIN_SET)
		return TPS65180_VCOM_MIN_uV;
	if (reg_setting >= TPS65180_VCOM_MAX_SET)
		return TPS65180_VCOM_MAX_uV;
	return -(reg_setting * TPS65180_VCOM_STEP_uV);
}
static int vcom2_rs_to_uV(unsigned int reg_setting)
{
	if (reg_setting <= TPS65185_VCOM_MIN_SET)
		return TPS65185_VCOM_MIN_uV;
	if (reg_setting >= TPS65185_VCOM_MAX_SET)
		return TPS65185_VCOM_MAX_uV;
	return -(reg_setting * TPS65185_VCOM_STEP_uV);
}


static int vcom_uV_to_rs(int uV)
{
	if (uV <= TPS65180_VCOM_MIN_uV)
		return TPS65180_VCOM_MIN_SET;
	if (uV >= TPS65180_VCOM_MAX_uV)
		return TPS65180_VCOM_MAX_SET;
	return (-uV) / TPS65180_VCOM_STEP_uV;
}

static int vcom2_uV_to_rs(int uV)
{
	if (uV <= TPS65185_VCOM_MIN_uV)
		return TPS65185_VCOM_MIN_SET;
	if (uV >= TPS65185_VCOM_MAX_uV)
		return TPS65185_VCOM_MAX_SET;
	return (-uV) / TPS65185_VCOM_STEP_uV;
}

static int epdc_pwr0_enable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);

	gpio_set_value(tps6518x->gpio_pmic_powerup, 1);

	return 0;

}

static int epdc_pwr0_disable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);

	gpio_set_value(tps6518x->gpio_pmic_powerup, 0);

	return 0;

}
static int tps6518x_v3p3_enable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);

	gpio_set_value(tps6518x->gpio_pmic_powerup, 1);
	return 0;
}

static int tps6518x_v3p3_disable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);

	gpio_set_value(tps6518x->gpio_pmic_powerup, 0);
	return 0;

}
static int tps6518x_v3p3_is_enabled(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);
	int gpio = gpio_get_value(tps6518x->gpio_pmic_powerup);

	if (gpio == 0)
		return 0;
	else
		return 1;
}

static int tps6518x_vcom_set_voltage(struct regulator_dev *reg,
					int minuV, int uV, unsigned *selector)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);
	unsigned int cur_reg_val; /* current register value to modify */
	unsigned int new_reg_val; /* new register value to write */
	int retval;

	/*
	 * this will not work on tps65182
	 */
	if (tps6518x->revID == 65182)
		return 0;
	
#if 0
	if (uV < 200000)
		return 0;
#endif

	switch (tps6518x->revID & 15)
	{
		case 0 : /* TPS65180 */
		case 1 : /* TPS65181 */
		case 4 : /* TPS65180-rev1 */
			tps6518x_reg_read(REG_TPS65180_VCOM_ADJUST,&cur_reg_val);
			new_reg_val = to_reg_val(cur_reg_val,
					BITFMASK(VCOM_SET),
					BITFVAL(VCOM_SET, vcom_uV_to_rs(uV)));

			retval = tps6518x_reg_write(REG_TPS65180_VCOM_ADJUST,
					new_reg_val);
			break;
		case 5 : /* TPS65185 */
		case 6 : /* TPS65186 */
			gpio_set_value(tps6518x->gpio_pmic_wakeup,1);
			retval = tps6518x_reg_write(REG_TPS65185_VCOM1,
					vcom2_uV_to_rs(uV) & 255);
			tps6518x_reg_read( REG_TPS65185_VCOM2,&cur_reg_val);
			new_reg_val = to_reg_val(cur_reg_val,
					BITFMASK(VCOM2_SET),
					BITFVAL(VCOM2_SET, vcom2_uV_to_rs(uV)/256));

			retval = tps6518x_reg_write(REG_TPS65185_VCOM2,
					new_reg_val);

			break;
		default :
		retval = -1;
	}
	return retval;
}

static int tps6518x_vcom_get_voltage(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);
	unsigned int cur_reg_val; /* current register value */
	unsigned int cur_reg2_val; /* current register value */
	unsigned int cur_fld_val; /* current bitfield value*/
	int vcomValue;

	/*
	 * this will not work on tps65182
	 */
	if (tps6518x->revID == 65182)
		return 0;
	
	switch (tps6518x->revID & 15)
	{
		case 0 : /* TPS65180 */
		case 1 : /* TPS65181 */
		case 4 : /* TPS65180-rev1 */
			tps6518x_reg_read(REG_TPS65180_VCOM_ADJUST, &cur_reg_val);
			cur_fld_val = BITFEXT(cur_reg_val, VCOM_SET);
			vcomValue = vcom_rs_to_uV(cur_fld_val);
			break;
		case 5 : /* TPS65185 */
		case 6 : /* TPS65186 */
			tps6518x_reg_read(REG_TPS65185_VCOM1,&cur_reg_val);
			tps6518x_reg_read(REG_TPS65185_VCOM2,&cur_reg2_val);
			cur_reg_val |= 256 * (1 & cur_reg2_val);
			vcomValue = vcom2_rs_to_uV(cur_reg_val);
			break;
		default:
			vcomValue = 0;
	}
	
	return vcomValue;

}

static int tps6518x_vcom_enable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);
	unsigned int cur_reg_val; /* current register value */
	int vcomEnable = 0;
	/*
	 * check for the TPS65182 device
	 */
	if (tps6518x->revID == 65182)
	{
		gpio_set_value(tps6518x->gpio_pmic_vcom_ctrl,vcomEnable);
		return 0;
	}

	/*
	 * Check to see if we need to set the VCOM voltage.
	 * Should only be done one time. And, we can
	 * only change vcom voltage if we have been enabled.
	 */
	if (!tps6518x->vcom_setup && tps6518x_is_power_good(tps6518x)) {
		tps6518x_vcom_set_voltage(reg,
			tps6518x->vcom_uV,
			tps6518x->vcom_uV,
			NULL);
		tps6518x->vcom_setup = true;
	}

	switch (tps6518x->revID & 15)
	{
		case 0 : /* TPS65180 */
		case 1 : /* TPS65181 */
		case 4 : /* TPS65180-rev1 */
			vcomEnable = 1;
			break;
		case 5 : /* TPS65185 */
		case 6 : /* TPS65186 */
			tps6518x_reg_read(REG_TPS65185_VCOM2,&cur_reg_val);
			// do not enable vcom if HiZ bit is set
			if (cur_reg_val & (1<<VCOM_HiZ_LSH))
				vcomEnable = 0;
			else
				vcomEnable = 1;
			break;
		default:
			vcomEnable = 0;
	}
	gpio_set_value(tps6518x->gpio_pmic_vcom_ctrl,vcomEnable);

	return 0;
}

static int tps6518x_vcom_disable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);

	gpio_set_value(tps6518x->gpio_pmic_vcom_ctrl,0);
	return 0;
}

static int tps6518x_vcom_is_enabled(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);

	int gpio = gpio_get_value(tps6518x->gpio_pmic_vcom_ctrl);
	if (gpio == 0)
		return 0;
	else
		return 1;
}


static int tps6518x_is_power_good(struct tps6518x *tps6518x)
{
	/*
	 * XOR of polarity (starting value) and current
	 * value yields whether power is good.
	 */
	return gpio_get_value(tps6518x->gpio_pmic_pwrgood) ^
		tps6518x->pwrgood_polarity;
}

static int tps6518x_wait_power_good(struct tps6518x *tps6518x)
{
	int i;
	for (i = 0; i < tps6518x->max_wait * 3; i++) {
		if (tps6518x_is_power_good(tps6518x))
			return 0;

		msleep(1);
	}
	return -ETIMEDOUT;
}

static int tps6518x_display_enable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);
	unsigned int cur_reg_val; /* current register value to modify */
	unsigned int fld_mask;	  /* register mask for bitfield to modify */
	unsigned int fld_val;	  /* new bitfield value to write */
	unsigned int new_reg_val; /* new register value to write */
	if (tps6518x->revID == 65182)
	{
		epdc_pwr0_enable(reg);
	}
	else
	{
		gpio_set_value(tps6518x->gpio_pmic_wakeup,1);

		/* enable display regulators */
		cur_reg_val = tps65180_current_Enable_Register & 0x3f;
		fld_mask = BITFMASK(VDDH_EN) | BITFMASK(VPOS_EN) |
			BITFMASK(VEE_EN) | BITFMASK(VNEG_EN);
		fld_val = BITFVAL(VDDH_EN, true) | BITFVAL(VPOS_EN, true) |
			BITFVAL(VEE_EN, true) | BITFVAL(VNEG_EN, true) | BITFVAL(VCOM_EN, true);
		new_reg_val = tps65180_current_Enable_Register = to_reg_val(cur_reg_val, fld_mask, fld_val);
		tps6518x_reg_write(REG_TPS65180_ENABLE, new_reg_val);

		/* turn on display regulators */
		cur_reg_val = tps65180_current_Enable_Register & 0x3f;
		fld_mask = BITFMASK(ACTIVE);
		fld_val = BITFVAL(ACTIVE, true);
		new_reg_val = tps65180_current_Enable_Register = to_reg_val(cur_reg_val, fld_mask, fld_val);
		tps6518x_reg_write(REG_TPS65180_ENABLE, new_reg_val);
	}

	return tps6518x_wait_power_good(tps6518x);
}

static int tps6518x_display_disable(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);
	unsigned int cur_reg_val; /* current register value to modify */
	unsigned int fld_mask;	  /* register mask for bitfield to modify */
	unsigned int fld_val;	  /* new bitfield value to write */
	unsigned int new_reg_val; /* new register value to write */

	if (tps6518x->revID == 65182)
	{
		epdc_pwr0_disable(reg);
	}
	else
	{
		/* turn off display regulators */
		cur_reg_val = tps65180_current_Enable_Register & 0x3f;
		fld_mask = BITFMASK(VCOM_EN) | BITFMASK(STANDBY);
		fld_val = BITFVAL(VCOM_EN, true) | BITFVAL(STANDBY, true);
		new_reg_val = tps65180_current_Enable_Register = to_reg_val(cur_reg_val, fld_mask, fld_val);
		tps6518x_reg_write(REG_TPS65180_ENABLE, new_reg_val);

	}

	msleep(tps6518x->max_wait);

	return 0;
}

static int tps6518x_display_is_enabled(struct regulator_dev *reg)
{
	struct tps6518x *tps6518x = rdev_get_drvdata(reg);

	if (tps6518x->revID == 65182)
		return gpio_get_value(tps6518x->gpio_pmic_wakeup) ? 1:0;
	else
		return tps65180_current_Enable_Register & BITFMASK(ACTIVE);
}

/*
 * Regulator operations
 */

static struct regulator_ops tps6518x_display_ops = {
	.enable = tps6518x_display_enable,
	.disable = tps6518x_display_disable,
	.is_enabled = tps6518x_display_is_enabled,
};

static struct regulator_ops tps6518x_vcom_ops = {
	.enable = tps6518x_vcom_enable,
	.disable = tps6518x_vcom_disable,
	.get_voltage = tps6518x_vcom_get_voltage,
	.set_voltage = tps6518x_vcom_set_voltage,
	.is_enabled = tps6518x_vcom_is_enabled,
};

static struct regulator_ops tps6518x_v3p3_ops = {
	.enable = tps6518x_v3p3_enable,
	.disable = tps6518x_v3p3_disable,
	.is_enabled = tps6518x_v3p3_is_enabled,
};

/*
 * Regulator descriptors
 */
static struct regulator_desc tps6518x_reg[TPS6518x_NUM_REGULATORS] = {
{
	.name = "DISPLAY",
	.id = TPS6518x_DISPLAY,
	.ops = &tps6518x_display_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VCOM",
	.id = TPS6518x_VCOM,
	.ops = &tps6518x_vcom_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "V3P3",
	.id = TPS6518x_V3P3,
	.ops = &tps6518x_v3p3_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
};

static void tps6518x_setup_timings(struct tps6518x *tps6518x)
{

	int temp0, temp1, temp2, temp3;

	/* read the current setting in the PMIC */
	if ((tps6518x->revID == TPS65180_PASS1) || (tps6518x->revID == TPS65181_PASS1) ||
	   (tps6518x->revID == TPS65180_PASS2) || (tps6518x->revID == TPS65181_PASS2)) {
	   tps6518x_reg_read(REG_TPS65180_PWRSEQ0, &temp0);
	   tps6518x_reg_read(REG_TPS65180_PWRSEQ1, &temp1);
	   tps6518x_reg_read(REG_TPS65180_PWRSEQ2, &temp2);

	   if ((temp0 != tps6518x->pwr_seq0) ||
		(temp1 != tps6518x->pwr_seq1) ||
		(temp2 != tps6518x->pwr_seq2)) {
		tps6518x_reg_write(REG_TPS65180_PWRSEQ0, tps6518x->pwr_seq0);
		tps6518x_reg_write(REG_TPS65180_PWRSEQ1, tps6518x->pwr_seq1);
		tps6518x_reg_write(REG_TPS65180_PWRSEQ2, tps6518x->pwr_seq2);
	    }
	}

	if ((tps6518x->revID == TPS65185_PASS0) ||
		 (tps6518x->revID == TPS65186_PASS0) ||
		 (tps6518x->revID == TPS65185_PASS1) ||
		 (tps6518x->revID == TPS65186_PASS1) ||
		 (tps6518x->revID == TPS65185_PASS2) ||
		 (tps6518x->revID == TPS65186_PASS2)) {
	   tps6518x_reg_read(REG_TPS65185_UPSEQ0, &temp0);
	   tps6518x_reg_read(REG_TPS65185_UPSEQ1, &temp1);
	   tps6518x_reg_read(REG_TPS65185_DWNSEQ0, &temp2);
	   tps6518x_reg_read(REG_TPS65185_DWNSEQ1, &temp3);
	   if ((temp0 != tps6518x->upseq0) ||
		(temp1 != tps6518x->upseq1) ||
		(temp2 != tps6518x->dwnseq0) ||
		(temp3 != tps6518x->dwnseq1)) {
		tps6518x_reg_write(REG_TPS65185_UPSEQ0, tps6518x->upseq0);
		tps6518x_reg_write(REG_TPS65185_UPSEQ1, tps6518x->upseq1);
		tps6518x_reg_write(REG_TPS65185_DWNSEQ0, tps6518x->dwnseq0);
		tps6518x_reg_write(REG_TPS65185_DWNSEQ1, tps6518x->dwnseq1);
	    }
	}
}

#define CHECK_PROPERTY_ERROR_KFREE(prop) \
do { \
	int ret = of_property_read_u32(tps6518x->dev->of_node, \
					#prop, &tps6518x->prop); \
	if (ret < 0) { \
		return ret;	\
	}	\
} while (0);

#ifdef CONFIG_OF
static int tps6518x_pmic_dt_parse_pdata(struct platform_device *pdev,
					struct tps6518x_platform_data *pdata)
{
	struct tps6518x *tps6518x = dev_get_drvdata(pdev->dev.parent);
	struct device_node *pmic_np, *regulators_np, *reg_np;
	struct tps6518x_regulator_data *rdata;
	int i, ret;

	pmic_np = of_node_get(tps6518x->dev->of_node);
	if (!pmic_np) {
		dev_err(&pdev->dev, "could not find pmic sub-node\n");
		return -ENODEV;
	}

	regulators_np = of_find_node_by_name(pmic_np, "regulators");
	if (!regulators_np) {
		dev_err(&pdev->dev, "could not find regulators sub-node\n");
		return -EINVAL;
	}

	pdata->num_regulators = of_get_child_count(regulators_np);
	dev_dbg(&pdev->dev, "num_regulators %d\n", pdata->num_regulators);

	rdata = devm_kzalloc(&pdev->dev, sizeof(*rdata) *
				pdata->num_regulators, GFP_KERNEL);
	if (!rdata) {
		of_node_put(regulators_np);
		dev_err(&pdev->dev, "could not allocate memory for"
			"regulator data\n");
		return -ENOMEM;
	}

	pdata->regulators = rdata;
	for_each_child_of_node(regulators_np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(tps6518x_reg); i++)
			if (!of_node_cmp(reg_np->name, tps6518x_reg[i].name))
				break;

		if (i == ARRAY_SIZE(tps6518x_reg)) {
			dev_warn(&pdev->dev, "don't know how to configure"
				"regulator %s\n", reg_np->name);
			continue;
		}

		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(&pdev->dev,
							     reg_np);
		rdata->reg_node = reg_np;
		rdata++;
	}
	of_node_put(regulators_np);

	tps6518x->max_wait = (6 + 6 + 6 + 6);

	tps6518x->gpio_pmic_wakeup = of_get_named_gpio(pmic_np,
					"gpio_pmic_wakeup", 0);
	if (!gpio_is_valid(tps6518x->gpio_pmic_wakeup)) {
		dev_err(&pdev->dev, "no epdc pmic wakeup pin available\n");
		goto err;
	}
	ret = devm_gpio_request_one(&pdev->dev, tps6518x->gpio_pmic_wakeup,
				GPIOF_OUT_INIT_LOW, "epdc-pmic-wake");
	if (ret < 0)
		goto err;

	tps6518x->gpio_pmic_vcom_ctrl = of_get_named_gpio(pmic_np,
					"gpio_pmic_vcom_ctrl", 0);
	if (!gpio_is_valid(tps6518x->gpio_pmic_vcom_ctrl)) {
		dev_err(&pdev->dev, "no epdc pmic vcom_ctrl pin available\n");
		goto err;
	}
	ret = devm_gpio_request_one(&pdev->dev, tps6518x->gpio_pmic_vcom_ctrl,
				GPIOF_OUT_INIT_LOW, "epdc-vcom");
	if (ret < 0)
		goto err;

	tps6518x->gpio_pmic_powerup = of_get_named_gpio(pmic_np,
					"gpio_pmic_powerup", 0);
	if (!gpio_is_valid(tps6518x->gpio_pmic_powerup)) {
		dev_err(&pdev->dev, "no epdc pmic powerup pin available\n");
		goto err;
	}
	ret = devm_gpio_request_one(&pdev->dev, tps6518x->gpio_pmic_powerup,
				GPIOF_OUT_INIT_LOW, "epdc-powerup");
	if (ret < 0)
		goto err;

	tps6518x->gpio_pmic_intr = of_get_named_gpio(pmic_np,
					"gpio_pmic_intr", 0);
	if (!gpio_is_valid(tps6518x->gpio_pmic_intr)) {
		dev_err(&pdev->dev, "no epdc pmic intr pin available\n");
		goto err;
	}
	ret = devm_gpio_request_one(&pdev->dev, tps6518x->gpio_pmic_intr,
				GPIOF_IN, "epdc-pmic-int");
	if (ret < 0)
		goto err;

	tps6518x->gpio_pmic_pwrgood = of_get_named_gpio(pmic_np,
					"gpio_pmic_pwrgood", 0);
	if (!gpio_is_valid(tps6518x->gpio_pmic_pwrgood)) {
		dev_err(&pdev->dev, "no epdc pmic pwrgood pin available\n");
		goto err;
	}
	ret = devm_gpio_request_one(&pdev->dev, tps6518x->gpio_pmic_pwrgood,
				GPIOF_IN, "epdc-pwrstat");
	if (ret < 0)
		goto err;

err:
	return 0;

}
#else
static int tps6518x_pmic_dt_parse_pdata(struct platform_device *pdev,
					struct tps6518x *tps6518x)
{
	return 0;
}
#endif	/* !CONFIG_OF */

/*
 * Regulator init/probing/exit functions
 */
static int tps6518x_regulator_probe(struct platform_device *pdev)
{
	struct tps6518x *tps6518x = dev_get_drvdata(pdev->dev.parent);
	struct tps6518x_platform_data *pdata = tps6518x->pdata;
	struct tps6518x_data *priv;
	struct regulator_dev **rdev;
	struct regulator_config config = { };
	int size, i, ret = 0;

    printk("tps6518x_regulator_probe starting\n");

	if (tps6518x->dev->of_node) {
		ret = tps6518x_pmic_dt_parse_pdata(pdev, pdata);
		if (ret)
			return ret;
	}
	priv = devm_kzalloc(&pdev->dev, sizeof(struct tps6518x_data),
			       GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	size = sizeof(struct regulator_dev *) * pdata->num_regulators;
	priv->rdev = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!priv->rdev)
		return -ENOMEM;

	rdev = priv->rdev;
	priv->num_regulators = pdata->num_regulators;
	platform_set_drvdata(pdev, priv);

	tps6518x->vcom_setup = false;
	tps6518x->pass_num = tps6518x_pass_num;
	tps6518x->vcom_uV = tps6518x_vcom;

	for (i = 0; i < pdata->num_regulators; i++) {
		int id = pdata->regulators[i].id;

		config.dev = tps6518x->dev;
		config.init_data = pdata->regulators[i].initdata;
		config.driver_data = tps6518x;
		config.of_node = pdata->regulators[i].reg_node;

		rdev[i] = regulator_register(&tps6518x_reg[id], &config);
		if (IS_ERR(rdev[i])) {
			ret = PTR_ERR(rdev[i]);
			dev_err(&pdev->dev, "regulator init failed for %d\n",
					id);
			rdev[i] = NULL;
			goto err;
		}
	}

	/*
	 * Set up PMIC timing values.
	 * Should only be done one time!  Timing values may only be
	 * changed a limited number of times according to spec.
	 */
	tps6518x_setup_timings(tps6518x);

    printk("tps6518x_regulator_probe success\n");
	return 0;
err:
	while (--i >= 0)
		regulator_unregister(rdev[i]);
	return ret;
}

static int tps6518x_regulator_remove(struct platform_device *pdev)
{
	struct tps6518x_data *priv = platform_get_drvdata(pdev);
	struct regulator_dev **rdev = priv->rdev;
	int i;

	for (i = 0; i < priv->num_regulators; i++)
		regulator_unregister(rdev[i]);
	return 0;
}

static const struct platform_device_id tps6518x_pmic_id[] = {
	{ "tps6518x-pmic", 0},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, tps6518x_pmic_id);

static struct platform_driver tps6518x_regulator_driver = {
	.probe = tps6518x_regulator_probe,
	.remove = tps6518x_regulator_remove,
	.id_table = tps6518x_pmic_id,
	.driver = {
		.name = "tps6518x-pmic",
	},
};

static int __init tps6518x_regulator_init(void)
{
	return platform_driver_register(&tps6518x_regulator_driver);
}
subsys_initcall_sync(tps6518x_regulator_init);

static void __exit tps6518x_regulator_exit(void)
{
	platform_driver_unregister(&tps6518x_regulator_driver);
}
module_exit(tps6518x_regulator_exit);


/*
 * Parse user specified options (`tps6518x:')
 * example:
 *   tps6518x:pass=2,vcom=-1250000
 */
static int __init tps6518x_setup(char *options)
{
	int ret;
	char *opt;
	unsigned long ulResult;
	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;
		if (!strncmp(opt, "pass=", 5)) {
			ret = strict_strtoul((const char *)(opt + 5), 0, &ulResult);
			tps6518x_pass_num = ulResult;
			if (ret < 0)
				return ret;
		}
		if (!strncmp(opt, "vcom=", 5)) {
			int offs = 5;
			if (opt[5] == '-')
				offs = 6;
			ret = strict_strtoul((const char *)(opt + offs), 0, &ulResult);
			tps6518x_vcom = (int) ulResult;
			if (ret < 0)
				return ret;
			tps6518x_vcom = -tps6518x_vcom;
		}
	}

	return 1;
}

__setup("tps6518x:", tps6518x_setup);

static int __init tps65182_setup(char *options)
{
	int ret;
	char *opt;
	unsigned long ulResult;
	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;
		if (!strncmp(opt, "pass=", 5)) {
			ret = strict_strtoul((const char *)(opt + 5), 0, &ulResult);
			tps6518x_pass_num = ulResult;
			if (ret < 0)
				return ret;
		}
		if (!strncmp(opt, "vcom=", 5)) {
			int offs = 5;
			if (opt[5] == '-')
				offs = 6;
			ret = strict_strtoul((const char *)(opt + offs), 0, &ulResult);
			tps6518x_vcom = (int) ulResult;
			if (ret < 0)
				return ret;
			tps6518x_vcom = -tps6518x_vcom;
		}
	}

	return 1;
}

__setup("tps65182:", tps65182_setup);


/* Module information */
MODULE_DESCRIPTION("TPS6518x regulator driver");
MODULE_LICENSE("GPL");

/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*!
 * @file pmic/core/tps6518x.c
 * @brief This file contains TPS6518x specific PMIC code. This implementaion
 * may differ for each PMIC chip.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/pmic_status.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tps6518x.h>
#include <asm/mach-types.h>

static int tps6518x_detect(struct i2c_client *client, struct i2c_board_info *info);
struct i2c_client *tps6518x_client;
static struct regulator *gpio_regulator;

static struct mfd_cell tps6518x_devs[] = {
	{ .name = "tps6518x-pmic", },
	{ .name = "tps6518x-sns", },
};

static const unsigned short normal_i2c[] = {EPDC_PMIC_I2C_ADDR, I2C_CLIENT_END};

int tps6518x_reg_read(int reg_num, unsigned int *reg_val)
{
	int result;

	if (tps6518x_client == NULL)
		return PMIC_ERROR;

	result = i2c_smbus_read_byte_data(tps6518x_client, reg_num);
	if (result < 0) {
		dev_err(&tps6518x_client->dev,
			"Unable to read tps6518x register via I2C\n");
		return PMIC_ERROR;
	}

	*reg_val = result;
	return PMIC_SUCCESS;
}

int tps6518x_reg_write(int reg_num, const unsigned int reg_val)
{
	int result;

	if (tps6518x_client == NULL)
		return PMIC_ERROR;

	result = i2c_smbus_write_byte_data(tps6518x_client, reg_num, reg_val);
	if (result < 0) {
	//	dev_err(&tps6518x_client->dev,
	//		"Unable to write TPS6518x register via I2C\n");
		return PMIC_ERROR;
	}

	return PMIC_SUCCESS;
}

#ifdef CONFIG_OF
static struct tps6518x_platform_data *tps6518x_i2c_parse_dt_pdata(
					struct device *dev)
{
	struct tps6518x_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for pdata\n");
		return ERR_PTR(-ENOMEM);
	}

	return pdata;
}
#else
static struct tps6518x_platform_data *tps6518x_i2c_parse_dt_pdata(
					struct device *dev)
{
	return NULL;
}
#endif	/* !CONFIG_OF */

static int tps6518x_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct tps6518x *tps6518x;
	struct tps6518x_platform_data *pdata = client->dev.platform_data;
	struct device_node *np = client->dev.of_node;
	int ret = 0;

	printk("tps6518x_probe calling\n");

	if (!np)
		return -ENODEV;

	gpio_regulator = devm_regulator_get(&client->dev, "SENSOR");
	if (!IS_ERR(gpio_regulator)) {
		ret = regulator_enable(gpio_regulator);
		if (ret) {
			dev_err(&client->dev, "gpio set voltage error\n");
			return ret;
		}
	}


	/* Create the PMIC data structure */
	tps6518x = kzalloc(sizeof(struct tps6518x), GFP_KERNEL);
	if (tps6518x == NULL) {
		kfree(client);
		return -ENOMEM;
	}

	/* Initialize the PMIC data structure */
	i2c_set_clientdata(client, tps6518x);
	tps6518x->dev = &client->dev;
	tps6518x->i2c_client = client;

	tps6518x_client = client;
	ret = tps6518x_detect(client, NULL);
	if (ret)
		goto err1;

	mfd_add_devices(tps6518x->dev, -1, tps6518x_devs,
			ARRAY_SIZE(tps6518x_devs),
			NULL, 0, NULL);

	if (tps6518x->dev->of_node) {
		pdata = tps6518x_i2c_parse_dt_pdata(tps6518x->dev);
		if (IS_ERR(pdata)) {
			ret = PTR_ERR(pdata);
			goto err2;
		}

	}
	tps6518x->pdata = pdata;

	dev_info(&client->dev, "PMIC TPS6518x for eInk display\n");

	printk("tps6518x_probe success\n");

	return ret;

err2:
	mfd_remove_devices(tps6518x->dev);
err1:
	kfree(tps6518x);

	return ret;
}


static int tps6518x_remove(struct i2c_client *i2c)
{
	struct tps6518x *tps6518x = i2c_get_clientdata(i2c);

	mfd_remove_devices(tps6518x->dev);

	return 0;
}

static int tps6518x_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int tps6518x_resume(struct i2c_client *client)
{
	return 0;
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int tps6518x_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	//struct tps6518x_platform_data *pdata = client->dev.platform_data;
	struct i2c_adapter *adapter = client->adapter;
	u8 revId;
    printk("tps6518x_detect calling\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	/* identification */
	revId = i2c_smbus_read_byte_data(client,
		  REG_TPS6518x_REVID);

	/*
	 * Known rev-ids
	 * tps165180 pass 1 = 0x50, tps65180 pass2 = 0x60, tps65181 pass1 = 0x51, tps65181 pass2 = 0x61, 
	 * tps65182, 
	 * tps65185 pass0 = 0x45, tps65186 pass0 0x46, tps65185 pass1 = 0x55, tps65186 pass1 0x56, tps65185 pass2 = 0x65, tps65186 pass2 0x66
	 */
	if (!((revId == TPS65180_PASS1) ||
		 (revId == TPS65181_PASS1) ||
		 (revId == TPS65180_PASS2) ||
		 (revId == TPS65181_PASS2) ||
		 (revId == TPS65185_PASS0) ||
		 (revId == TPS65186_PASS0) ||
		 (revId == TPS65185_PASS1) ||
		 (revId == TPS65186_PASS1) ||
		 (revId == TPS65185_PASS2) ||
		 (revId == TPS65186_PASS2)))
	{
		dev_info(&adapter->dev,
		    "Unsupported chip (Revision ID=0x%02X).\n",  revId);
		return -ENODEV;
	}

	if (info) {
		strlcpy(info->type, "tps6518x_sensor", I2C_NAME_SIZE);
	}
    printk("tps6518x_detect success\n");

	return 0;
}

static const struct i2c_device_id tps6518x_id[] = {
	{ "tps6518x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps6518x_id);

static const struct of_device_id tps6518x_dt_ids[] = {
	{
		.compatible = "ti,tps6518x",
		.data = (void *) &tps6518x_id[0],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, tps6518x_dt_ids);


static struct i2c_driver tps6518x_driver = {
	.driver = {
		   .name = "tps6518x",
		   .owner = THIS_MODULE,
		   .of_match_table = tps6518x_dt_ids,
	},
	.probe = tps6518x_probe,
	.remove = tps6518x_remove,
	.suspend = tps6518x_suspend,
	.resume = tps6518x_resume,
	.id_table = tps6518x_id,
	.detect = tps6518x_detect,
	.address_list = &normal_i2c[0],
};

static int __init tps6518x_init(void)
{
	return i2c_add_driver(&tps6518x_driver);
}

static void __exit tps6518x_exit(void)
{
	i2c_del_driver(&tps6518x_driver);
}

/*
 * Module entry points
 */
subsys_initcall(tps6518x_init);
module_exit(tps6518x_exit);

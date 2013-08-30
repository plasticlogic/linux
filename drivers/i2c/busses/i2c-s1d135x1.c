/*
 * i2c-s1d135x1.c -- Epson S1D135x1 SPI to I2C bridge driver
 *
 * Copyright (C) 2013 Plastic Logic Limited
 *
 * Authors: Guillaume Tucker <guillaume.tucker@plasticlogic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/modelffb.h>

/* Note: This driver relies on the main S1D135x1 driver to initialize the
 * controller and populate the API functions.  */

struct s1d135x1_spi s1d135x1_spi;
EXPORT_SYMBOL(s1d135x1_spi);

/* SPI register addresses to control the I2C master bus */
enum s1d135x1_i2c_reg {
	S1D135X1_I2C_REG_STAT = 0x0218,
	S1D135X1_I2C_REG_CMD = 0x021A,
	S1D135X1_I2C_REG_READ_DATA = 0x021C,
	S1D135X1_I2C_REG_WRITE_DATA = 0x021E,
};

/* I2C_REG_STAT register bits */
enum s1d135x1_i2c_stat_reg {
	S1D135X1_I2C_STAT_GO = 1 << 0,
	S1D135X1_I2C_STAT_RX_NAK = 1 << 1,
	S1D135X1_I2C_STAT_BUSY = 1 << 2,
	S1D135X1_I2C_STAT_TIP = 1 << 3,
	S1D135X1_I2C_STAT_ERROR = 1 << 6,
	S1D135X1_I2C_STAT_RESET = 1 << 15,
};

/* I2C_REG_CMD register bits */
enum s1d135x1_i2c_cmd_reg {
	S1D135X1_I2C_CMD_GO = 1 << 0,
	S1D135X1_I2C_CMD_READ = 1 << 1,
	S1D135X1_I2C_CMD_TX_NAK = 1 << 2,
	S1D135X1_I2C_CMD_GEN = 1 << 4,
	S1D135X1_I2C_CMD_START = 1 << 5,
	S1D135X1_I2C_CMD_NO_DATA = 1 << 6,
};

/* Device data */
struct s1d135x1_i2c_dev {
	struct device *dev;
	struct i2c_adapter adap;
	u16 status;
};

/* private functions */

static int s1d135x1_i2c_poll(struct s1d135x1_i2c_dev *dev,
			     struct spi_device *spi)
{
	int i;

	for (i = 50; i; --i) {
		dev->status = s1d135x1_spi.read_reg(
			spi, S1D135X1_I2C_REG_STAT);

		if (!(dev->status & S1D135X1_I2C_STAT_GO))
			break;
	}

	if (!i)
		return -ETIMEDOUT;

	return 0;
}

static int s1d135x1_i2c_read_data(struct s1d135x1_i2c_dev *dev,
				  struct spi_device *spi,
				  const struct i2c_msg *msg)
{
	static const u8 DATA_CMD =
		S1D135X1_I2C_CMD_GO | S1D135X1_I2C_CMD_READ;
	static const u8 END_CMD = 
		S1D135X1_I2C_CMD_GEN | S1D135X1_I2C_CMD_TX_NAK;
	const size_t last_byte = msg->len - 1;
	u16 data;
	int i;
	int stat;

	for (i = 0; i < msg->len; ++i) {
		u8 cmd = (i == last_byte) ? (DATA_CMD | END_CMD) : DATA_CMD;

		s1d135x1_spi.write_reg(spi, S1D135X1_I2C_REG_CMD, cmd);

		stat = s1d135x1_i2c_poll(dev, spi);
		if (stat)
			return stat;

		data = s1d135x1_spi.read_reg(spi, S1D135X1_I2C_REG_READ_DATA);
		msg->buf[i] = data & 0x00FF;
	}

	return 0;
}

static int s1d135x1_i2c_write_data(struct s1d135x1_i2c_dev *dev,
				   struct spi_device *spi,
				   const struct i2c_msg *msg)
{
	static const u8 DATA_CMD = S1D135X1_I2C_CMD_GO;
	static const u8 END_CMD = S1D135X1_I2C_CMD_GEN;
	const size_t last_byte = msg->len - 1;
	int i;
	int stat;

	for (i = 0; i < msg->len; ++i) {
		u8 cmd = (i == last_byte) ? (DATA_CMD | END_CMD) : DATA_CMD;

		s1d135x1_spi.write_reg(spi, S1D135X1_I2C_REG_WRITE_DATA,
				       msg->buf[i]);
		s1d135x1_spi.write_reg(spi, S1D135X1_I2C_REG_CMD, cmd);

		stat = s1d135x1_i2c_poll(dev, spi);
		if (stat)
			return stat;
	}

	return 0;
}

static int s1d135x1_i2c_xfer_msg(struct s1d135x1_i2c_dev *dev,
				 struct spi_device *spi,
				 const struct i2c_msg *msg)
{
	static const u8 START_CMD =
		(S1D135X1_I2C_CMD_START | S1D135X1_I2C_CMD_GEN |
		 S1D135X1_I2C_CMD_GO);
	const bool is_read = msg->flags & I2C_M_RD;
	const u8 i2c_addr = (msg->addr << 1) | (is_read ? 1 : 0);
	int stat;

	s1d135x1_spi.write_reg(spi, S1D135X1_I2C_REG_WRITE_DATA, i2c_addr);
	s1d135x1_spi.write_reg(spi, S1D135X1_I2C_REG_CMD, START_CMD);

	stat = s1d135x1_i2c_poll(dev, spi);
	if (stat)
		return stat;

	if (dev->status & S1D135X1_I2C_STAT_RX_NAK) {
		dev_err(&dev->adap.dev,
			"Device NAK on address 0x%02X\n", msg->addr);
		return -ENODEV;
	}

	if (is_read)
		stat = s1d135x1_i2c_read_data(dev, spi, msg);
	else
		stat = s1d135x1_i2c_write_data(dev, spi, msg);

	return 0;
}

/* I2C bus driver implementation */

static u32 s1d135x1_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static int s1d135x1_i2c_master_xfer(struct i2c_adapter *adap,
				    struct i2c_msg *msgs, int num)
{
	struct s1d135x1_i2c_dev *dev = i2c_get_adapdata(adap);
	struct spi_device *spi;
	int stat;
	int i;

	if (!s1d135x1_spi.init_done) {
		dev_err(&adap->dev, "Epson S1D135x1 not initialized\n");
		return -ENODEV;
	}

	spi = s1d135x1_spi.get();
	if (!spi)
		return -ENODEV;

	stat = 0;

	for (i = 0; i < num; ++i) {
		stat = s1d135x1_i2c_xfer_msg(dev, spi, &msgs[i]);
		if (stat)
			goto exit_put_spi;
	}

exit_put_spi:
	s1d135x1_spi.put(spi);

	return stat ? stat : num;
}

static const struct i2c_algorithm s1d135x1_i2c_algo = {
	.functionality = s1d135x1_i2c_functionality,
	.master_xfer = s1d135x1_i2c_master_xfer,
};

static int __devinit s1d135x1_i2c_probe(struct platform_device *pdev)
{
	struct s1d135x1_i2c_dev *dev;
	struct i2c_adapter *adap;
	int stat;

	dev = kzalloc(sizeof(struct s1d135x1_i2c_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	adap = &dev->adap;
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD; /* ToDo: check */
	adap->algo = &s1d135x1_i2c_algo;
	adap->nr = pdev->id;
	adap->dev.parent = &pdev->dev;
	strlcpy(adap->name, "s1d135x1 SPI-I2C adapter", sizeof(adap->name));
	stat = i2c_add_numbered_adapter(adap);
	if (stat) {
		dev_err(&pdev->dev, "Failed to add I2C adapter\n");
		goto exit_free_device;
	}

	dev->dev = &pdev->dev;
	i2c_set_adapdata(adap, dev);
	platform_set_drvdata(pdev, dev);

	dev_info(&dev->adap.dev, "S1D135x1 SPI-I2C bridge ready");

	return 0;

exit_free_device:
	kfree(dev);

	return stat;
}

static int s1d135x1_i2c_remove(struct platform_device *pdev)
{
	struct s1d135x1_i2c_dev *dev = platform_get_drvdata(pdev);

	i2c_del_adapter(&dev->adap);
	kfree(dev);

	return 0;
}

static struct platform_driver s1d135x1_i2c_driver = {
	.probe = s1d135x1_i2c_probe,
	.remove = s1d135x1_i2c_remove,
	.driver = {
		.name = "s1d135x1_i2c",
		.owner = THIS_MODULE,
		.pm = NULL,
	},
};

static int __init s1d135x1_i2c_init(void)
{
	s1d135x1_spi.init_done = false;

	return platform_driver_register(&s1d135x1_i2c_driver);
}
subsys_initcall(s1d135x1_i2c_init);

static void __exit s1d135x1_i2c_exit(void)
{
	platform_driver_unregister(&s1d135x1_i2c_driver);
}
module_exit(s1d135x1_i2c_exit);

MODULE_AUTHOR("Guillaume Tucker <guillaume.tucker@plasticlogic.com>");
MODULE_DESCRIPTION("Epson S1D135x1 SPI-I2C bridge driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:s1d135x1_i2c");

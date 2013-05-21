/*
 * i2c-sc18is60x.c -- NXP SC18IS600/601 SPI to I2C bridge driver
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
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c-sc18is60x.h>

/* Maximum I2C and SPI message lengths */
#define SC18IS60X_MAX_I2C_LEN 96
#define SC18IS60X_MAX_SPI_LEN (SC18IS60X_MAX_I2C_LEN + 5)

/* SPI Command data offset */
enum sc18is60x_cmd {
	SC18IS60X_CMD = 0,
};

/* SPI commands */
enum sc18is60x_spi_command {
	SC18IS60X_WRITE = 0x00,
	SC18IS60X_READ = 0x01,
	SC18IS60X_WR_RD = 0x02,
	SC18IS60X_READ_BUFFER = 0x06,
	SC18IS60X_WRITE_REG = 0x20,
	SC18IS60X_READ_REG = 0x21,
};

/* I2C write data offsets */
enum sc18is60x_i2c_write_data {
	SC18IS60X_I2C_LEN = 1,
	SC18IS60X_I2C_ADDR = 2,
	SC18IS60X_I2C_DATA = 3,
};

/* I2C read data offsets */
enum sc18is60x_i2c_read_data {
	SC18IS60X_I2C_BUFFER = 1,
};

/* I2C write and read data offset */
enum sc18is60x_i2c_wr_rd_data {
	SC18IS60X_I2C_WR_LEN = 1,
	SC18IS60X_I2C_RD_LEN = 2,
	SC18IS60X_I2C_WR_ADDR = 3,
	SC18IS60X_I2C_WR_DATA = 4,
	SC18IS60X_I2C_WR_RD_LEN = 5,
};

/* SPI register data offsets */
enum sc18is60x_reg_data {
	SC18IS60X_REG = 1,
	SC18IS60X_REG_VALUE = 2,
	SC18IS60X_REG_LEN = 3,
};

/* SPI register addresses */
enum sc18is60x_spi_register {
	SC18IS60X_REG_IO_CONFIG = 0,
	SC18IS60X_REG_IO_STATE = 1,
	SC18IS60X_REG_I2C_CLOCK = 2,
	SC18IS60X_REG_I2C_TO = 3,
	SC18IS60X_REG_I2C_STAT = 4,
	SC18IS60X_REG_I2C_ADDR = 5,
};

/* I2C status register values */
enum sc18is60x_i2c_status {
	SC18IS60X_I2C_OK = 0xF0,
	SC18IS60X_I2C_NAK_ADDR = 0xF1,
	SC18IS60X_I2C_NAK_BYTE = 0xF2,
	SC18IS60X_I2C_BUSY = 0xF3,
	SC18IS60X_I2C_TIMEOUT = 0xF8,
	SC18IS60X_I2C_DATA_ERROR = 0xF9,
};

/* Device data */
struct sc18is60x_dev {
	struct device *dev;
	struct i2c_adapter adapter;
	struct spi_transfer transfer[SC18IS60X_MAX_SPI_LEN];
	u8 data_out[SC18IS60X_MAX_SPI_LEN];
	u8 data_in[SC18IS60X_MAX_SPI_LEN];
	u8 reg_data[SC18IS60X_REG_LEN];
};

#ifdef CONFIG_I2C_SC18IS60X_SHARED_SPI
/* hack to share same SPI device with another driver */
static u8 sc18is60x_saved_spi_mode;
struct spi_device *sc18is60x_shared_spi = NULL;
EXPORT_SYMBOL(sc18is60x_shared_spi);
DEFINE_MUTEX(sc18is60x_spi_lock);
EXPORT_SYMBOL(sc18is60x_spi_lock);

static struct spi_device *sc18is60x_get_spi(void)
{
	mutex_lock(&sc18is60x_spi_lock);

	if (!sc18is60x_shared_spi)
		goto exit_err_unlock;

	sc18is60x_saved_spi_mode = sc18is60x_shared_spi->mode;
	sc18is60x_shared_spi->mode = SPI_MODE_3;

	if (spi_setup(sc18is60x_shared_spi)) {
		dev_err(&sc18is60x_shared_spi->dev,
			"Failed to set-up SPI bus in mode 3\n");
		goto exit_err_unlock;
	}

	return sc18is60x_shared_spi;

exit_err_unlock:
	mutex_unlock(&sc18is60x_spi_lock);

	return NULL;
}

static int sc18is60x_put_spi(struct spi_device *spi)
{
	int stat;

	BUG_ON(spi != sc18is60x_shared_spi);

	sc18is60x_shared_spi->mode = sc18is60x_saved_spi_mode;

	stat = spi_setup(sc18is60x_shared_spi);
	if (stat) {
		dev_err(&sc18is60x_shared_spi->dev,
			"Failed to restore original SPI mode\n");
	}

	mutex_unlock(&sc18is60x_spi_lock);

	return stat;
}
#else
# error "Not implemented yet."
/* ToDo: register as an SPI driver for normal usage */
static inline struct spi_device *sc18is60x_get_spi(void) {
	return NULL;
}
static inline int sc18is60x_put_spi(struct spi_device *spi) {
	return -EINVAL;
}
#endif /* CONFIG_I2C_SC18IS60X_SHARED_SPI */

static int sc18is60x_send_spi_message(struct sc18is60x_dev *dev,
				      struct spi_device *spi, size_t len,
				      bool is_reg)
{
	struct sc18is60x_platform_data *pdata = dev->dev->platform_data;
	struct spi_transfer *transfer = dev->transfer;
	u8 *data_out = dev->data_out;
	u8 *data_in = is_reg ? dev->reg_data : dev->data_in;
	struct spi_message m;
	size_t i;
	int stat;

	BUG_ON(!is_reg && (len > SC18IS60X_MAX_SPI_LEN));
	BUG_ON(is_reg && (len > SC18IS60X_REG_LEN));

	spi_message_init(&m);

	for (i = 0; i < len; ++i) {
		struct spi_transfer *t = &transfer[i];

		t->tx_buf = &data_out[i];
		t->rx_buf = &data_in[i];
		t->len = 1;
		t->speed_hz = 1000000;
		t->delay_usecs = 8;
		t->bits_per_word = 8;
		spi_message_add_tail(t, &m);
	}

	if (pdata->use_gpio_cs)
		gpio_set_value(pdata->gpio_cs, 0);
	stat = spi_sync(spi, &m);
	if (pdata->use_gpio_cs)
		gpio_set_value(pdata->gpio_cs, 1);

	return stat;
}

static int sc18is60x_read_spi_reg(struct sc18is60x_dev *dev,
				  struct spi_device *spi, u8 reg, u8 *value)
{
	u8 *data_out = dev->data_out;
	int stat;

	data_out[SC18IS60X_CMD] = SC18IS60X_READ_REG;
	data_out[SC18IS60X_REG] = reg;
	data_out[SC18IS60X_REG_VALUE] = 0x00;

	stat = sc18is60x_send_spi_message(dev, spi, SC18IS60X_REG_LEN, true);

	if (!stat)
		*value = dev->reg_data[SC18IS60X_REG_VALUE];

	return stat;
}

static int sc18is60x_poll_i2c(struct sc18is60x_dev *dev,
			      struct spi_device *spi)
{
	enum sc18is60x_i2c_status i2c_status;
	u8 r = 0;
	int stat;

	do {
		udelay(100);
		stat = sc18is60x_read_spi_reg(dev, spi,
					      SC18IS60X_REG_I2C_STAT, &r);
		if (stat) {
			dev_err(&spi->dev,
				"Failed to read I2C status register\n");
			return stat;
		}

		i2c_status = r;
	} while (i2c_status == SC18IS60X_I2C_BUSY);

	if (i2c_status != SC18IS60X_I2C_OK)
		dev_err(&spi->dev, "I2C error, status: %02X\n", i2c_status);

	switch (i2c_status) {
	case SC18IS60X_I2C_OK:
		stat = 0;
		break;
	case SC18IS60X_I2C_NAK_ADDR:
		stat = -ENODEV;
		break;
	case SC18IS60X_I2C_NAK_BYTE:
	case SC18IS60X_I2C_TIMEOUT:
	case SC18IS60X_I2C_DATA_ERROR:
	case SC18IS60X_I2C_BUSY:
	default:
		stat = -EIO;
		break;
	}

	return stat;
}

static int sc18is60x_send_spi_data(struct sc18is60x_dev *dev,
				   struct spi_device *spi, size_t len)
{
	int stat;

	stat = sc18is60x_send_spi_message(dev, spi, len, false);
	if (stat)
		return stat;

	return sc18is60x_poll_i2c(dev, spi);
}

static int sc18is60x_single_msg(struct i2c_adapter *adap, struct i2c_msg *msg,
				struct spi_device *spi)
{
	struct sc18is60x_dev *dev= i2c_get_adapdata(adap);
	u8 *data_out = dev->data_out;
	int stat;
	size_t spi_len;

	if (msg->flags & I2C_M_RD) {
		data_out[SC18IS60X_CMD] = SC18IS60X_READ;
		data_out[SC18IS60X_I2C_LEN] = msg->len;
		data_out[SC18IS60X_I2C_ADDR] = (msg->addr) << 1 | 0x01;
		spi_len = SC18IS60X_I2C_DATA;
	} else {
		data_out[SC18IS60X_CMD] = SC18IS60X_WRITE;
		data_out[SC18IS60X_I2C_LEN] = msg->len;
		data_out[SC18IS60X_I2C_ADDR] = (msg->addr) << 1;
		memcpy(&data_out[SC18IS60X_I2C_DATA], msg->buf, msg->len);
		spi_len = SC18IS60X_I2C_DATA + msg->len;
	}

	stat = sc18is60x_send_spi_data(dev, spi, spi_len);
	if (stat)
		return stat;

	if (msg->flags & I2C_M_RD) {
		u8 *data_in = dev->data_in;

		data_out[SC18IS60X_CMD] = SC18IS60X_READ_BUFFER;
		memset(&data_out[SC18IS60X_I2C_BUFFER ], 0, msg->len);
		spi_len = SC18IS60X_I2C_BUFFER  + msg->len;

		stat = sc18is60x_send_spi_data(dev, spi, spi_len);
		if (stat)
			return stat;

		memcpy(msg->buf, &data_in[SC18IS60X_I2C_BUFFER], msg->len);
	}

	return stat;
}

static int sc18is60x_double_msg(struct i2c_adapter *adap, struct i2c_msg *msgs,
				struct spi_device *spi)
{
	struct sc18is60x_dev *dev = i2c_get_adapdata(adap);
	struct i2c_msg *msg1 = &msgs[0];
	struct i2c_msg *msg2 = &msgs[1];
	u8 *data_out = dev->data_out;
	u8 *data_in = dev->data_in;
	size_t spi_len;
	int stat;

	if ((msg1->flags & I2C_M_RD) || !(msg2->flags & I2C_M_RD)){
		dev_err(&adap->dev,
			"Unsupported message sequence, "
			"can only write and then read\n");
		return -EINVAL;
	}

	data_out[SC18IS60X_CMD] = SC18IS60X_WR_RD;
	data_out[SC18IS60X_I2C_WR_LEN] = msg1->len;
	data_out[SC18IS60X_I2C_RD_LEN] = msg2->len;
	data_out[SC18IS60X_I2C_WR_ADDR] = msg1->addr << 1;
	memcpy(&data_out[SC18IS60X_I2C_WR_DATA], msg1->buf, msg1->len);
	data_out[SC18IS60X_I2C_WR_DATA + msg1->len] = (msg2->addr << 1) | 1;
	spi_len = SC18IS60X_I2C_WR_RD_LEN + msg1->len;

	stat = sc18is60x_send_spi_data(dev, spi, spi_len);
	if (stat)
		return stat;

	udelay(100);

	data_out[SC18IS60X_CMD] = SC18IS60X_READ_BUFFER;
	memset(&data_out[SC18IS60X_I2C_BUFFER], 0, msg2->len);
	spi_len = SC18IS60X_I2C_BUFFER + msg2->len;

	stat = sc18is60x_send_spi_data(dev, spi, spi_len);
	if (stat)
		return stat;

	memcpy(msg2->buf, &data_in[SC18IS60X_I2C_BUFFER], msg2->len);

	return 0;
}

static int sc18is60x_master_xfer(struct i2c_adapter *adap,
				 struct i2c_msg *msgs, int num)
{
	struct spi_device *spi = sc18is60x_get_spi();
	int msg_n;
	int stat;

	if (!spi) {
		dev_err(&adap->dev, "No SPI device\n");
		return -ENODEV;
	}

	for (msg_n = 0; msg_n < num; ++msg_n) {
		if (msgs[msg_n].len > SC18IS60X_MAX_I2C_LEN) {
			dev_err(&adap->dev,
				"Message is too long: %d (max is %d)\n",
				msgs[msg_n].len, SC18IS60X_MAX_I2C_LEN);
			return -EINVAL;
		}
	}

	if (num == 0) {
		stat = 0;
	} else if (num == 1)  {
		stat = sc18is60x_single_msg(adap, &msgs[0], spi);
	} else if (num == 2) {
		stat = sc18is60x_double_msg(adap, msgs, spi);
	} else {
		dev_err(&adap->dev,
			"Too many I2C messages: %d (max is 2)\n", num);
		stat = -EINVAL;
	}

	if (stat)
		dev_err(&adap->dev, "Transaction failed\n");

	stat = sc18is60x_put_spi(spi);

	return stat ? stat : num;
}

static u32 sc18is60x_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm sc18is60x_algo = {
	.functionality = sc18is60x_functionality,
	.master_xfer = sc18is60x_master_xfer,
};

static int __devinit sc18is60x_probe(struct platform_device *pdev)
{
	struct sc18is60x_platform_data *pdata = pdev->dev.platform_data;
	struct sc18is60x_dev *dev;
	struct i2c_adapter *adap;
	int stat;

	dev = kzalloc(sizeof(struct sc18is60x_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	adap = &dev->adapter;
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->algo = &sc18is60x_algo;
	adap->nr = pdev->id;
	adap->dev.parent= &pdev->dev;
	strlcpy(adap->name, "sc18is60x SPI-I2C adapter", sizeof(adap->name));
	stat = i2c_add_numbered_adapter(adap);
	if (stat) {
		dev_err(&pdev->dev, "Failed to add I2C adapter");
		goto exit_free_device;
	}

	if (pdata->use_gpio_cs) {
		dev_info(&adap->dev, "Using GPIO CS: %d\n", pdata->gpio_cs);

		if (gpio_request(pdata->gpio_cs, "I2C_SPI_CS") < 0) {
			dev_err(&pdev->dev,
				"Failed to request I2C_SPI_CS: %d\n",
				pdata->gpio_cs);
			stat = -EBUSY;
			goto exit_free_i2c_adapter;
		}

		gpio_direction_output(pdata->gpio_cs, 1);
	}

	dev->dev = &pdev->dev;
	i2c_set_adapdata(adap, dev);
	platform_set_drvdata(pdev, dev);
	dev_info(&dev->adapter.dev, "Ready.");

	return 0;

exit_free_i2c_adapter:
	i2c_del_adapter(&dev->adapter);
exit_free_device:
	kfree(dev);

	return stat;
}

static int sc18is60x_remove(struct platform_device *pdev)
{
	struct sc18is60x_dev *dev = platform_get_drvdata(pdev);
	struct sc18is60x_platform_data *pdata = dev->dev->platform_data;

	if (pdata->use_gpio_cs)
		gpio_free(pdata->gpio_cs);
	i2c_del_adapter(&dev->adapter);
	kfree(dev);

	return 0;
}

static struct platform_driver sc18is60x_driver = {
	.probe = sc18is60x_probe,
	.remove = sc18is60x_remove,
	.driver = {
		.name = "sc18is60x",
		.owner = THIS_MODULE,
		.pm = NULL,
	},
};

static int __init sc18is60x_init(void)
{
	return platform_driver_register(&sc18is60x_driver);
}
subsys_initcall(sc18is60x_init);

static void __exit sc18is60x_exit(void)
{
	platform_driver_unregister(&sc18is60x_driver);
}
module_exit(sc18is60x_exit);

MODULE_AUTHOR("Guillaume Tucker <guillaume.tucker@plasticlogic.com>");
MODULE_DESCRIPTION("sc18is60x SPI-I2C bridge driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sc18is60x");

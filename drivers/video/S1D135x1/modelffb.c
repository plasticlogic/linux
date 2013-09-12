/*
 * modelffb.c -- FB driver for EPSON modelF EPD controller
 *
 * Copyright (C) 2012 Seiko Epson Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */

#include <linux/module.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/spi/spi.h>
#include <linux/timer.h>
#include <linux/pmeter.h>
#include <linux/modelffb.h>
#ifndef CONFIG_MODELF_DEFERRED_IO
#include <linux/mm.h>
#endif

#include <mach/board-am335xevm.h>
#include <mach/irqs.h>

#include "modelffb.h"

#define CHECKREV 110

/* Set to 1 to print the time it takes to send the image data */
#define TIME_SEND_IMAGE 0

/* Set to 1 to use the MT PROM */
#define USE_MTP 0

/* Set to 1 to enable pmeter synchronous update messages */
#if 0
# define SYNC_LOG(msg, ...) pmeter_logf("epsync "msg, ##__VA_ARGS__)
static const char *modelffb_sync_status_table[MODELFFB_SYNC_N] = {
	[MODELFFB_SYNC_IDLE] = "idle",
	[MODELFFB_SYNC_PENDING] = "pending",
	[MODELFFB_SYNC_BUSY] = "busy",
};
#else
# define SYNC_LOG(msg, ...)
#endif

/* Maximum image pool size in bytes */
#define IMAGE_POOL_MAX_SZ (32 * 1024)

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
#include <linux/mutex.h>
#include "pl_hardware.h"
#include "vcom.h"
#include "temperature-set.h"

static char *modelffb_panel_type_modparam = "";
module_param_named(panel_type, modelffb_panel_type_modparam, charp, S_IRUGO);
MODULE_PARM_DESC(panel_type, "Panel type identifier");

/* ToDo: define in board file */
struct pl_hardware_config modelffb_pl_config = {
#ifdef CONFIG_MODELF_PL_ROBIN
	.i2c_bus_number = 4,
#else
#if defined(CONFIG_I2C_S1D135X1) || defined(CONFIG_I2C_S1D135X1_MODULE)
	.i2c_bus_number = 5,
#else
	.i2c_bus_number = 3,
#endif
#endif
	.dac_i2c_address = 0x39,
	.adc_i2c_address = 0x34,
#ifdef CONFIG_MODELF_PL_Z6_Z7
	.hvpmic_id = PLHW_HVPMIC_TPS65185,
#else
	.hvpmic_id = PLHW_HVPMIC_MAX17135,
#endif
};

static DEFINE_MUTEX(temperature_lock);

#endif

#define NON_DMA_MAX_BYTES (160 - 2)

static struct modelffb_par *parinfo;

#ifdef CONFIG_I2C_SC18IS60X_SHARED_SPI
/* hack to share same SPI device with sc18is60x I2C bridge driver if only 1
 * chip select is available */
extern struct mutex sc18is60x_spi_lock;
extern struct spi_device *sc18is60x_shared_spi;
#endif /* CONFIG_I2C_SC18IS60X_SHARED_SPI */

static inline int modelffb_lock(void)
{
	if (down_interruptible(&parinfo->access_sem)) {
		dev_err(parinfo->dev, "Down semaphore interrupted\n");
		return -EBUSY;
	}

#ifdef CONFIG_I2C_SC18IS60X_SHARED_SPI
	mutex_lock(&sc18is60x_spi_lock);
#endif

	return 0;
}

static inline void modelffb_unlock(void)
{
#ifdef CONFIG_I2C_SC18IS60X_SHARED_SPI
	mutex_unlock(&sc18is60x_spi_lock);
#endif

	up(&parinfo->access_sem);
}

/* ----------------------------------------------------------------------------
 * SPI interface
 *
 * NOTE! Do not use __xx functions without semaphore.
 * Disordered command/data may cause problem.
 */

#define SWAP_BYTE_16(x) ((x >> 8) | (x << 8))

static int __devinit modelffb_request_bus(void)
{
	struct spi_master *master;

	master = spi_busnum_to_master(parinfo->pdata->spi_info->bus_num);
	if (!master) {
		dev_err(parinfo->dev, "Failed to get master SPI bus %d\n",
			parinfo->pdata->spi_info->bus_num);
		return -ENODEV;
	}

	parinfo->spi = spi_new_device(master, parinfo->pdata->spi_info);
	if (!parinfo->spi) {
		dev_err(parinfo->dev, "Failed to create new SPI device\n");
		return -ENODEV;
	}

#ifdef CONFIG_I2C_SC18IS60X_SHARED_SPI
	mutex_lock(&sc18is60x_spi_lock);
	sc18is60x_shared_spi = parinfo->spi;
	mutex_unlock(&sc18is60x_spi_lock);
#endif

	return 0;
}

static void __devexit modelffb_release_bus(void)
{
#ifdef CONFIG_I2C_SC18IS60X_SHARED_SPI
	mutex_lock(&sc18is60x_spi_lock);
	sc18is60x_shared_spi = NULL;
	mutex_unlock(&sc18is60x_spi_lock);
#endif

	spi_unregister_device(parinfo->spi);
}

static inline void my_spi_write(struct spi_device *spi, const void *buffer,
				size_t bytes)
{
	int error;
	struct spi_message m;
	struct spi_transfer t;

	spi_message_init(&m);
	memset(&t, 0, sizeof t);

	t.tx_buf = buffer;
	t.len = bytes;
	t.bits_per_word = 16;
	t.speed_hz = parinfo->opt.spi_freq_hz;
	spi_message_add_tail(&t, &m);

	error = spi_sync(spi, &m);
	if (error < 0)
		dev_err(parinfo->dev, "SPI write error: %d\n", error);
}

static inline void my_spi_read(struct spi_device *spi, void *buffer,
			       size_t bytes)
{
	int error;
	struct spi_message m;
	struct spi_transfer t;

	spi_message_init(&m);
	memset(&t, 0, sizeof t);

	t.rx_buf = buffer;
	t.len = bytes;
	t.bits_per_word = 16;
	t.speed_hz = parinfo->opt.spi_freq_hz;
	spi_message_add_tail(&t, &m);

	error = spi_sync(spi, &m);
	if (error < 0)
		dev_err(parinfo->dev, "SPI read error: %d\n", error);
}

static inline void __modelffb_write_n16(struct spi_device *spi,
					const uint16_t *data, size_t n)
{
	struct spi_message m;
	struct spi_transfer t[5];
	int error;
	size_t i;

	BUG_ON(n > 5);

	spi_message_init(&m);
	memset(t, 0, (n * sizeof(struct spi_transfer)));

	for (i = 0; i < n; ++i) {
		struct spi_transfer *it = &t[i];

		it->tx_buf = &data[i];
		it->len = sizeof(uint16_t);
		it->bits_per_word = 16;
		it->speed_hz = parinfo->opt.spi_freq_hz;
		spi_message_add_tail(it, &m);
	}

	error = spi_sync(spi, &m);
	if (error < 0)
		dev_err(parinfo->dev, "SPI write error: %d\n", error);
}

static inline uint16_t __modelffb_read_reg(struct spi_device *spi,
					   uint16_t address)
{
	struct spi_message m;
	struct spi_transfer t[3];
	uint16_t readval;
	int error;

	spi_message_init(&m);
	memset(t, 0, (2 * sizeof(struct spi_transfer)));

	t[0].tx_buf = &address;
	t[0].len = sizeof(address);
	t[0].bits_per_word = 16;
	t[0].speed_hz = parinfo->opt.spi_freq_hz;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = &readval;
	t[1].len = sizeof(readval);
	t[1].bits_per_word = 16;
	t[1].speed_hz = parinfo->opt.spi_freq_hz;
	memcpy(&t[2], &t[1], sizeof(struct spi_transfer));
	spi_message_add_tail(&t[1], &m);
	spi_message_add_tail(&t[2], &m);

	error = spi_sync(spi, &m);
	if (error < 0) {
		dev_err(parinfo->dev, "SPI reg read error: %d\n", error);
		return error;
	}

	return readval;
}

static inline void __modelffb_write_command(uint16_t command)
{
#ifdef CONFIG_MODELF_SWAP_SPI_BYTE
	command = SWAP_BYTE_16(command);
#endif

	if (parinfo->pdata->gpio_hdc) {
		gpio_set_value(parinfo->pdata->gpio_hdc, 0);
		my_spi_write(parinfo->spi, &command, 2);
		gpio_set_value(parinfo->pdata->gpio_hdc, 1);
	} else {
		gpio_set_value(parinfo->pdata->gpio_cs, 0);
		my_spi_write(parinfo->spi, &command, 2);
	}
}

static inline void __modelffb_write_data(uint16_t data)
{
#ifdef CONFIG_MODELF_SWAP_SPI_BYTE
	uint16_t buffer = SWAP_BYTE_16(data);
	my_spi_write(parinfo->spi, &buffer, 2);
#else
	my_spi_write(parinfo->spi, &data, 2);
#endif
}

static inline int __modelffb_write_n_data(const uint16_t *data, size_t n)
{
	int i;
#ifdef CONFIG_MODELF_SWAP_SPI_BYTE
	uint16_t *buffer = kmalloc(n + 2, GFP_KERNEL);

	if (!buffer) {
		dev_err(parinfo->dev, "kmalloc swap buffer failed\n");
		return -ENOMEM;
	}

/* Data size over NON_DMA_MAX_BYTES will automativally be sent via DMA.
 * It is necessary to allocate coherent memory for using SPI with DMA.
 * But this area is out of sight from deferred I/O because of page fault
 * mechanism.
 * Pooling sent data to coherent area might be one solution.
 */
	for (i = 0; i < n / 2 + 1; i++) { /* + 1 for out of 16-bit alignment */
		*(buffer + i) = SWAP_BYTE_16(*(data + i));
	}

	for (i = 0; i < n / 2 + 1; i += NON_DMA_MAX_BYTES / 2) {
		my_spi_write(parinfo->spi, buffer + i,
			     n - (i * 2) < NON_DMA_MAX_BYTES ?
			     (n % NON_DMA_MAX_BYTES) : NON_DMA_MAX_BYTES);
	}

	kfree(buffer);
#else
	for (i = 0; i < n / 2 + 1; i += NON_DMA_MAX_BYTES / 2) {
		my_spi_write(parinfo->spi, data + i,
			     n - (i * 2) < NON_DMA_MAX_BYTES ?
			     (n % NON_DMA_MAX_BYTES) : NON_DMA_MAX_BYTES);
	}
#endif

	return 0;
}

static uint16_t __modelffb_read_data(void)
{
#ifdef CONFIG_MODELF_SWAP_SPI_BYTE
	uint16_t readval;

	my_spi_read(parinfo->spi, &readval, 2);
	return SWAP_BYTE_16(readval);
#else
	uint16_t readval;

	my_spi_read(parinfo->spi, &readval, 2);
	return readval;
#endif
}

/* ----------------------------------------------------------------------------
 * access functions
 */

static inline uint16_t __modelffb_read_dummy_data(void)
{
	return __modelffb_read_data();
}

static inline void __modelffb_immediate_command(uint16_t command)
{
	__modelffb_write_command(command);
}

static inline void __modelffb_command_end(void)
{
	if (!parinfo->pdata->gpio_hdc)
		gpio_set_value(parinfo->pdata->gpio_cs, 1);
}

static inline void __modelffb_immediate_simple_command(uint16_t command)
{
	__modelffb_immediate_command(command);
	__modelffb_command_end();
}

static inline uint16_t __modelffb_reg_read(uint16_t address)
{
	uint16_t readval;

	__modelffb_immediate_command(MODELF_COM_READ_REG);
	readval = __modelffb_read_reg(parinfo->spi, address);
	__modelffb_command_end();

	return readval;
}

static inline int __modelffb_check_HRDY_ready(void)
{
	if (parinfo->pdata->gpio_hrdy)
		return gpio_get_value(parinfo->pdata->gpio_hrdy);

	return (__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS)
		& MODELF_BF_INT_HRDY_STATUS);
}

static inline int __modelffb_wait_for_HRDY_ready(int ms_timeout)
{
	uint32_t start_jiffy = jiffies;
	int i;

	/* sync GPMC access and gpio peep timing */
	if (__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS) &
	    MODELF_BF_INT_HRDY_STATUS)
		goto success;

	/* poll gpio afterward */
	for (i = 1; i < 1000; i *= 2) {
		if (__modelffb_check_HRDY_ready())
			goto success;
		udelay(i);
	}

	while (1) {
		if (__modelffb_check_HRDY_ready())
			goto success;
		else if ((jiffies - start_jiffy) * (1000 / HZ) > ms_timeout) {
			break;
		}
		msleep(1);
	}

	dev_err(parinfo->dev, "HRDY timeout, "
		"MODELF_REG_SYSTEM_STATUS = 0x%04x, "
		"MODELF_REG_POWER_SAVE_MODE = 0x%04X\n",
		__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS),
		__modelffb_reg_read(MODELF_REG_POWER_SAVE_MODE));

	return -ETIMEDOUT;

success:
	return 0;
}

static int __modelffb_delay_for_HRDY_ready(int ms_timeout)
{
	uint32_t start_jiffy = jiffies;

	/* sync GPMC access and gpio peep timing */
	if (__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS) &
	    MODELF_BF_INT_HRDY_STATUS)
		goto success;

	/* poll gpio afterward */
	while (1) {
		if (__modelffb_check_HRDY_ready())
			goto success;
		else if ((jiffies - start_jiffy) * (1000 / HZ) > ms_timeout) {
			break;
		}
		udelay(1);
	}

	dev_err(parinfo->dev, "HRDY timeout, "
		"MODELF_REG_SYSTEM_STATUS = 0x%04x, "
		"MODELF_REG_POWER_SAVE_MODE = 0x%04X\n",
		__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS),
		__modelffb_reg_read(MODELF_REG_POWER_SAVE_MODE)); 

	return -ETIMEDOUT;

success:
	return 0;
}

static inline void __modelffb_command(uint16_t command)
{
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	__modelffb_immediate_command(command);
}

static inline void __modelffb_command_p1(uint16_t command, uint16_t param1)
{
	__modelffb_command(command);
	__modelffb_write_data(param1);
}

static inline void __modelffb_command_p2(uint16_t command, uint16_t param1,
					 uint16_t param2)
{
	const uint16_t data[2] = { param1, param2 };

	__modelffb_command(command);
	__modelffb_write_n16(parinfo->spi, data, 2);
}

static inline void __modelffb_command_p3(uint16_t command, uint16_t param1,
					 uint16_t param2, uint16_t param3)
{
	const uint16_t data[3] = { param1, param2, param3 };

	__modelffb_command(command);
	__modelffb_write_n16(parinfo->spi, data, 3);
}

static inline void __modelffb_command_p4(uint16_t command, uint16_t param1,
					 uint16_t param2,uint16_t param3,
					 uint16_t param4)
{
	const uint16_t data[4] = { param1, param2, param3, param4 };

	__modelffb_command(command);
	__modelffb_write_n16(parinfo->spi, data, 4);
}

static inline void __modelffb_command_p5(uint16_t command, uint16_t param1,
					 uint16_t param2, uint16_t param3,
					 uint16_t param4, uint16_t param5)
{
	const uint16_t data[5] = { param1, param2, param3, param4, param5 };

	__modelffb_command(command);
	__modelffb_write_n16(parinfo->spi, data, 5);
}

static inline void __modelffb_simple_command(uint16_t command)
{
	__modelffb_command(command);
	__modelffb_command_end();
}
static inline void __modelffb_simple_command_p1(
	uint16_t command, uint16_t param1)
{
	__modelffb_command_p1(command, param1);
	__modelffb_command_end();
}

static inline void __modelffb_simple_command_p2(
	uint16_t command, uint16_t param1, uint16_t param2)
{
	__modelffb_command_p2(command, param1, param2);
	__modelffb_command_end();
}

static inline void __modelffb_simple_command_p3(
	uint16_t command, uint16_t param1, uint16_t param2, uint16_t param3)
{
	__modelffb_command_p3(command, param1, param2, param3);
	__modelffb_command_end();
}

static inline void __modelffb_simple_command_p4(
	uint16_t command, uint16_t param1, uint16_t param2, uint16_t param3,
	uint16_t param4)
{
	__modelffb_command_p4(command, param1, param2, param3, param4);
	__modelffb_command_end();
}

static inline void __modelffb_simple_command_p5(
	uint16_t command, uint16_t param1, uint16_t param2,
	uint16_t param3, uint16_t param4, uint16_t param5)
{
	__modelffb_command_p5(command, param1, param2, param3, param4, param5);
	__modelffb_command_end();
}

static inline int __modelffb_wait_for_reg_value(
	uint16_t reg, uint16_t mask, uint16_t value, int ms_timeout)
{
	uint32_t start_jiffy = jiffies;

	while (1) {
		if ((__modelffb_reg_read(reg) & mask) == value)
			goto success;
		else if ((jiffies - start_jiffy) * (1000 / HZ) > ms_timeout) {
			break;
		}
		msleep(1);
	}

	dev_err(parinfo->dev, "Reading reg %04x timed out for 0x%04x\n",
		reg, value);

	return -ETIMEDOUT;

success:
	return 0;
}

static inline void __modelffb_reg_write(uint16_t address, uint16_t data)
{
	__modelffb_simple_command_p2(MODELF_COM_WRITE_REG, address, data);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
}

static inline void __modelffb_immediate_reg_write(uint16_t address,
						  uint16_t data)
{
	__modelffb_immediate_command(MODELF_COM_WRITE_REG);
	__modelffb_write_data(address);
	__modelffb_write_data(data);
	__modelffb_command_end();
}

static int __modelffb_data_transfer(uint16_t *data, size_t n)
{
	int retval;

	__modelffb_command_p1(MODELF_COM_WRITE_REG, MODELF_DATA_PORT);
	__modelffb_write_n_data(data, n);
	__modelffb_command_end();
	__modelffb_immediate_simple_command(MODELF_COM_END_OF_RAW_ACCESS);

	retval = __modelffb_delay_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval)
		dev_err(parinfo->dev, "Failed to send data\n");

	return retval;
}

/* =========== Synchronous update support =========== */

static inline void modelffb_sync_set_status(enum modelffb_sync_status status)
{
	SYNC_LOG("%d -> %d %s",
		 parinfo->sync_status, status,
		 modelffb_sync_status_table[status]);
	parinfo->sync_status = status;
	wake_up_interruptible(&parinfo->sync_update_wait);
}

static inline void modelffb_sync_wait(const char *status_str)
{
	static const char *status_table[MODELFFB_SYNC_N] = {
		[MODELFFB_SYNC_IDLE] = "idle",
		[MODELFFB_SYNC_PENDING] = "pending",
		[MODELFFB_SYNC_BUSY] = "busy",
	};
	enum modelffb_sync_status status;

	for (status = 0; status < MODELFFB_SYNC_N; ++status)
		if (!strcmp(status_str, status_table[status]))
			break;

	if (status == MODELFFB_SYNC_N) {
		dev_err(parinfo->dev, "Invalid sync status: %s\n", status_str);
		return;
	}

	SYNC_LOG("wait %d %s", status, modelffb_sync_status_table[status]);
	wait_event_interruptible(parinfo->sync_update_wait,
				 (parinfo->sync_status == status));
	SYNC_LOG("wait OK %d %s", status, modelffb_sync_status_table[status]);
}

static inline void modelffb_sync_wait_power(const char *power_str)
{
	bool power_state;

	if (!strcmp(power_str, "on")) {
		power_state = true;
	} else if (!strcmp(power_str, "off")) {
		power_state = false;
	} else {
		dev_err(parinfo->dev, "Invalid power state: %s\n", power_str);
		return;
	}

	SYNC_LOG("power %d %s", power_state, power_str);
	wait_event_interruptible(parinfo->sync_update_wait,
				 (pl_hardware_is_enabled(parinfo->pl_hardware)
				  == power_state));
	SYNC_LOG("power OK %d %s", power_state, power_str);
}

/* =========== model F operations =========== */

#ifdef CONFIG_MODELF_DEBUG
static void __modelffb_print_reg(uint16_t reg)
{
	dev_info(parinfo->dev, "register 0x%04x = 0x%04x\n",
		 reg, __modelffb_reg_read(reg));
}

static void modelffb_print_reg(uint16_t reg)
{
	if (modelffb_lock())
		return;
	__modelffb_print_reg(reg);
	modelffb_unlock();
}
#endif

static inline void modelffb_dump_memory(uint32_t modelf_addr, size_t size)
{
	int i;

	printk(KERN_INFO "dumping 0x%08x bytes memory from 0x%08x",
	       size, modelf_addr);

	if (modelffb_lock())
		return;

	__modelffb_simple_command_p4(MODELF_COM_RAW_READ,
		modelf_addr & 0xffff, (modelf_addr >> 16) & 0xffff,
		(size / 2) & 0xffff, ((size / 2) >> 16) & 0xffff);

	__modelffb_command_p1(MODELF_COM_READ_REG, MODELF_DATA_PORT);
	__modelffb_read_dummy_data();

	for (i = 0; i < size / 2; i++) {
		if (i % 8 == 0)
			printk(KERN_INFO "\n%08x: ", modelf_addr + i * 2);
		printk(KERN_INFO "%04x", __modelffb_read_data());
		if (i % 8 != 7)
			printk(" ");
	}
	printk(KERN_INFO "\n");
	__modelffb_command_end();
	__modelffb_simple_command(MODELF_COM_END_OF_RAW_ACCESS);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	modelffb_unlock();
}

static inline void modelffb_dump_register(uint16_t modelf_addr, size_t size)
{
	int i;

	printk(KERN_INFO "dumping %d registers from 0x%04x",
	       size, modelf_addr);

	if (modelffb_lock())
		return;

	for (i = 0; i < size; i++) {
		if (i % 8 == 0)
			printk(KERN_INFO "\nreg 0x%04x: ", modelf_addr + i * 2);
		printk(KERN_INFO "%04x",
		       __modelffb_reg_read(modelf_addr + i * 2));
		if (i % 8 != 7)
			printk(KERN_INFO " ");
	}
	printk(KERN_INFO "\n");

	modelffb_unlock();
}

static inline uint16_t modelffb_read_register(uint16_t modelf_addr)
{
	uint16_t ret_val;

	ret_val = modelffb_lock();
	if (ret_val)
		return ret_val;

	ret_val = __modelffb_reg_read(modelf_addr);
	modelffb_unlock();

	return ret_val;
}

static inline void modelffb_write_register(uint16_t modelf_addr,
					   uint16_t modelf_data)
{
	dev_info(parinfo->dev, "write register 0x%04x = 0x%04x\n",
		 modelf_addr, modelf_data);

	if (modelffb_lock())
		return;
	__modelffb_reg_write(modelf_addr, modelf_data);
	modelffb_unlock();
}

static int __modelffb_send_init_code(void)
{
	int readval;

	__modelffb_command(MODELF_COM_INIT);
	__modelffb_write_n_data((uint16_t*)parinfo->init_code,
				parinfo->init_code_size);
	__modelffb_command_end();
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	readval = __modelffb_reg_read(MODELF_REG_INIT_CODE_CHECKSUM);
	if ((readval & MODELF_BF_INIT_CODE_CHECKSUM) ==
	    MODELF_INIT_CODE_CHECKSUM_ERROR) {
		dev_err(parinfo->dev, "Init code checksum error!\n");
		return -EIO;
	}

	return 0;
}

static int __modelffb_send_waveform(void)
{
	int retval = 0;
	uint16_t readval = 0;

	__modelffb_command_p4(MODELF_COM_RAW_WRITE,
		MODELF_WAVEFORM_ADDR_LOW, MODELF_WAVEFORM_ADDR_HIGH,
		(parinfo->waveform_size / 2) & 0xffff,
		((parinfo->waveform_size / 2) >> 16) & 0xffff);

	__modelffb_data_transfer((uint16_t*)parinfo->waveform,
				 parinfo->waveform_size);

	retval = __modelffb_delay_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		dev_err(parinfo->dev, "Failed to send waveform\n");
		return retval;
	}

	readval = __modelffb_reg_read(MODELF_REG_DSPE_INT_STATUS);

	if (readval & MODELF_INT_WF_INVALID_FORMAT) {
		dev_err(parinfo->dev, "Invalid waveform format\n");
		return -EIO;
	}

	if (readval & MODELF_INT_WF_CHECKSUM_ERROR) {
		dev_err(parinfo->dev, "Waveform checksum error\n");
		return -EIO;
	}

	if (readval & MODELF_INT_WF_OVERFLOW) {
		dev_err(parinfo->dev, "Waveform overflow\n");
		return -EIO;
	}

	return 0;
}

#if USE_MTP
static int __modelffb_set_mtp_vcom_internal(int ms_timeout)
{
	int retval = 0;

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_ALL_STATE_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	__modelffb_reg_write(MODELF_REG_MTP_ADRDATA,
			     MODELF_MTP_VCOM_ADDR << 4);
	__modelffb_reg_write(MODELF_REG_MTP_CONTROL, MODELF_VCOM_READ_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_OPERATION_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	__modelffb_reg_write(MODELF_REG_MTP_ADRDATA,
			     (MODELF_MTP_VCOM_ADDR + 1) << 4);
	__modelffb_reg_write(MODELF_REG_MTP_CONTROL, MODELF_VCOM_READ_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_OPERATION_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	__modelffb_reg_write(MODELF_REG_MTP_CONTROL,
			     MODELF_MTP_READ_STOP_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_MODE_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev, "MTP vcom was set to internal register\n");
#endif
	return 0;

err:
	dev_err(parinfo->dev, "Failed to read mtp vcom value\n");

	return retval;
}

static int __modelffb_set_mtp_vcom_alu(int ms_timeout)
{
	int mtp_vcom;
	int retval = 0;

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_ALL_STATE_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	__modelffb_reg_write(MODELF_REG_MTP_ADRDATA, MODELF_MTP_VCOM_ADDR << 8);
	__modelffb_reg_write(MODELF_REG_MTP_CONTROL,
			     MODELF_MTP_READ_START_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_OPERATION_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	mtp_vcom = __modelffb_reg_read(MODELF_REG_MTP_READ_DATA) << 4;

	__modelffb_reg_write(MODELF_REG_MTP_ADRDATA,
			     (MODELF_MTP_VCOM_ADDR + 1) << 8);
	__modelffb_reg_write(MODELF_REG_MTP_CONTROL,
			     MODELF_MTP_READ_START_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_OPERATION_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	mtp_vcom |= __modelffb_reg_read(MODELF_REG_MTP_READ_DATA);

	__modelffb_reg_write(MODELF_REG_MTP_CONTROL,
			     MODELF_MTP_READ_STOP_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_MODE_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	__modelffb_reg_write(MODELF_REG_ALU_TEMPORARY_0, mtp_vcom);

#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev,
		 "MTP vcom value 0x%02x set to ALU Temporary Register 0\n",
		 mtp_vcom);
#endif

	return 0;

err:
	dev_err(parinfo->dev, "Failed to read mtp vcom value\n");

	return retval;
}

static int __modelffb_set_mtp_vcom(int ms_timeout)
{
	int retval = 0;

	retval = __modelffb_set_mtp_vcom_internal(ms_timeout);
	if (retval != 0)
		goto err;

	retval = __modelffb_set_mtp_vcom_alu(ms_timeout);
	if (retval != 0)
		goto err;

	return 0;

err:
	dev_err(parinfo->dev, "Failed to set mtp vcom value\n");

	return retval;
}
#endif /* USE_MTP */

static void __devinit __modelffb_reset(void)
{
	__modelffb_immediate_command(MODELF_COM_WRITE_REG);
	__modelffb_write_data(MODELF_REG_SOFTWARE_RESET);
	__modelffb_write_data(0);
	__modelffb_command_end();
	msleep(1);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
}

#define AM33XX_CONTROL_PADCONF_SPI0_D0_OFFSET		   0x0954
#define AM33XX_CONTROL_PADCONF_SPI0_D1_OFFSET		   0x0958

static int modelffb_chip_init(void)
{
	uint16_t readval;
	int retval = 0;

	retval = modelffb_lock();
	if (retval)
		goto err;

	__modelffb_reset();

	readval = __modelffb_reg_read(MODELF_REG_PRODUCT_CODE);
	dev_info(parinfo->dev, "Product code = 0x%04x\n", readval);
	if (readval != MODELF_PRODUCT_CODE) {
		dev_err(parinfo->dev, "Invalid product code\n");
		retval = -EIO;
		goto up_sem;
	}

	__modelffb_reg_write(MODELF_REG_CLOCK_CONFIGURATION,
		MODELF_INTERNAL_CLOCK_ENABLE);
	msleep(10);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		dev_err(parinfo->dev, "Clock enable failed\n");
		goto up_sem;
	}

	retval = __modelffb_send_init_code();
	if (retval != 0) {
		dev_err(parinfo->dev, "Failed to send init code\n");
		goto up_sem;
	}

	__modelffb_simple_command(MODELF_COM_INIT_THEN_STANDBY);
	msleep(100);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		dev_err(parinfo->dev, "Init and standby failed\n");
		goto up_sem;
	}

	if ((__modelffb_reg_read(MODELF_REG_WAVEFORM_DEC_BYPASS)
	     & MODELF_WAVEFORM_DECORDER_SELECT) &&
	    !(parinfo->status & MODELF_STATUS_KEYCODE_STORED)) {
		dev_err(parinfo->dev, "Keycode not stored\n");
		retval = -EIO;
		goto up_sem;
	}

	__modelffb_reg_write(MODELF_REG_PROTECTION_KEY_1, parinfo->keycode1);
	__modelffb_reg_write(MODELF_REG_PROTECTION_KEY_2, parinfo->keycode2);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		dev_err(parinfo->dev, "Failed to write keycode\n");
		goto up_sem;
	}

	readval = __modelffb_reg_read(MODELF_REG_UPDATE_BUFFER_CONF);
	__modelffb_reg_write(MODELF_REG_UPDATE_BUFFER_CONF,
			     (readval & ~MODELF_LUT_AUTO_SELECT));
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		dev_err(parinfo->dev, "Set update buffer config failed\n");
		goto up_sem;
	}

#if USE_MTP
	retval = __modelffb_set_mtp_vcom(MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto up_sem;
#endif

	retval = __modelffb_send_waveform();
	if (retval != 0)
		goto up_sem;

	__modelffb_reg_write(MODELF_REG_INTERRUPT_CONTROL,
			     MODELF_INT_DISPLAY_ENGINE);
	__modelffb_reg_write(MODELF_REG_DSPE_INT_ENABLE,
			     MODELF_INT_DSPE_ONE_LUT_FREE);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		dev_err(parinfo->dev, "Set interrupt control failed\n");
		goto up_sem;
	}

	__modelffb_immediate_reg_write(MODELF_REG_POWER_SAVE_MODE, 
		(__modelffb_reg_read(MODELF_REG_POWER_SAVE_MODE)
		 & ~MODELF_POWER_ACTIVE) | MODELF_POWER_ACTIVE);
	__modelffb_simple_command(MODELF_COM_RUN);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		dev_err(parinfo->dev, "Failed to enter run mode\n");
		goto up_sem;
	}
	parinfo->power_mode = MODELF_POWER_RUN;

	__modelffb_simple_command(MODELF_COM_CLEAR_GATE_DRIVER);
	__modelffb_simple_command(MODELF_COM_WAIT_TRIGGER_DONE);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		dev_err(parinfo->dev, "Failed to clear gate driver\n");
		goto up_sem;
	}

#ifdef CONFIG_MODELF_PL_ROBIN /* vertical mirror to reverse source data */
	__modelffb_simple_command_p1(MODELF_COM_ROTATE, 0x0400);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval) {
		dev_err(parinfo->dev, "Rotate command failed\n");
		goto up_sem;
	}
#endif

#if defined(CONFIG_I2C_S1D135X1) || defined(CONFIG_I2C_S1D135X1_MODULE)
	__modelffb_immediate_reg_write(MODELF_REG_I2C_CLOCK,
				       parinfo->pdata->i2c_clk_divider);
#endif

	modelffb_unlock();

	return 0;

up_sem:
	modelffb_unlock();
err:
	dev_err(parinfo->dev, "Failed to initialise controller\n");

	return retval;
}

static void __init __modelffb_bit_swap_table_init(void)
{
	int i;

	for (i = 0; i < 256; i++) {
		parinfo->bit_swap_table[i] =
			(i << 7 & (1 << 7)) |
			(i << 5 & (1 << 6)) |
			(i << 3 & (1 << 5)) |
			(i << 1 & (1 << 4)) |
			(i >> 1 & (1 << 3)) |
			(i >> 3 & (1 << 2)) |
			(i >> 5 & (1 << 1)) |
			(i >> 7 & (1 << 0));
	}
}

static inline uint8_t __swap_bit(uint8_t b)
{
	return parinfo->bit_swap_table[b];
}

static void __modelffb_pool_read_shaped_1bit_image(int x, int y, int width)
{
	int i;
	struct fb_info *info = parinfo->fbinfo;
	uint32_t start = (uint32_t)info->screen_base +
		(uint32_t)info->fix.line_length * y + x / 32;
	int offset = x % 8;

	for (i = x; i < (x + width); i += 8) {
		*(uint8_t*)(parinfo->image_pool + (i - x) / 8) =
			__swap_bit(
			(*(uint8_t*)(start + (i - x) / 8) << offset) |
			(*(uint8_t*)(start + (i - x) / 8 + 1) >> (8 - offset)));
	}
}

/* Original algorithm did not round correctly.
 * this one is based on the one here
 * http://stackoverflow.com/questions/2442576/how-does-one-convert-16-bit-rgb565-to-24-bit-rgb888
 */
static inline uint8_t __modelffb_8bit_desaturate(uint16_t color)
{
	uint32_t red, green, blue;
	uint8_t grey;
	red = (color & 0x1f);
	red = red * 527 + 23;
	green = (color >> 5) & 0x3f;
	green = green * 259 + 23;
	blue = (color >> 11) & 0x1f;
	blue = blue * 527 + 23;
	/* red, green and blue are scaled now by 64
	 * remove scale and take mean */
	grey = (uint8_t)((red + green + blue) / (3 * 64));
	
	return grey;
}

/* -- straight pixel ordering -- */

static void __modelffb_pool_read_shaped_16bit_image(
	int x, int y, int width, int lines)
{
	struct fb_info *info = parinfo->fbinfo;
	uint8_t *out = parinfo->image_pool;
	int line;

	for (line = 0; line < lines; ++line) {
		const uint16_t *in = (const uint16_t *)
			(info->screen_base +
			 (info->fix.line_length * (line + y)) +
			 (x * sizeof(uint16_t)));
		int i;

		for (i = 0; i < width; ++i)
			*out++ = __modelffb_8bit_desaturate(*in++);
	}
}

/* -- interleaved sources pixel ordering -- */

#define fixup_interleaved_area(_x, _y, _w, _h) ({	\
		_x /= 2;				\
		_y *= 2;				\
		_w = (_w + 1) / 2;			\
		_h = _h * 2;				\
	})

static void __modelffb_pool_read_shaped_16bit_interleaved_image(
	int x, int y, int width, int lines)
{
	struct fb_info *info = parinfo->fbinfo;
	const int fb_lines = lines / 2;
	int fb_line;

	for (fb_line = 0; fb_line < fb_lines; ++fb_line) {
		const uint16_t *in = (const uint16_t *)
			(info->screen_base +
			 (info->fix.line_length * (fb_line + (y / 2))) +
			 ((x * 2) * sizeof(uint16_t)));
		uint8_t *out1 = parinfo->image_pool + (fb_line * 2 * width);
		uint8_t *out2 = &out1[width];
		int i;

		for (i = 0; i < width; ++i) {
			*out2++ = __modelffb_8bit_desaturate(*in++);
			*out1++ = __modelffb_8bit_desaturate(*in++);
		}
	}
}

/* -- common implementation for straight and interleaved pixel ordering -- */

static void __modelffb_send_image_16b(int x, int y, int width, int height)
{
	void (* const copy_data)(int x, int y, int width, int height) =
		parinfo->opt.interleaved_sources ?
		__modelffb_pool_read_shaped_16bit_interleaved_image :
		__modelffb_pool_read_shaped_16bit_image;
	const int last_line = y + height;
	const int ep_x = x + parinfo->left_border;
	int line;

	for (line = y; line < last_line;) {
		const int lines = min(parinfo->image_pool_lines,
				      (last_line - line));
		const size_t length = width * lines;

		copy_data(x, line, width, lines);
		__modelffb_command_p5(
			MODELF_COM_LOAD_IMAGE_AREA, MODELF_BPP_8,
			(ep_x & 0x1ff), (line & 0x3ff), (width & 0x1ff),
			(lines & 0x3ff));
		__modelffb_data_transfer( /* 16-bit access */
			parinfo->image_pool, ((length + 1) / 2) * 2);
		line += lines;
	}
}

static int __modelffb_send_image(int x, int y, int width, int height)
{
	int retval = 0;
	const int last_line = y + height;
	struct fb_info *info = parinfo->fbinfo;
	int line;
#if TIME_SEND_IMAGE
	const uint32_t start_jiffy = jiffies;
	uint32_t delta_jiffy;
#endif

	switch (info->var.bits_per_pixel) {
	case 1:
		if (parinfo->opt.interleaved_sources) {
			dev_err(parinfo->dev,
				"Interleaved sources not supported with 1bpp "
				"pixel format\n");
			return -EINVAL;
		}

		for (line = y; line < last_line; line++) {
			__modelffb_pool_read_shaped_1bit_image(x, line, width);
			__modelffb_command_p5(
				MODELF_COM_LOAD_IMAGE_AREA, 1, (x & 0x1ff),
				(line & 0x3ff), (width & 0x1ff), (1 & 0x3ff));
			__modelffb_data_transfer(parinfo->image_pool,
				((width + 15) / 16) * 2); /* 16-bit access */
		}
		break;
	case 8:
		if (parinfo->opt.interleaved_sources) {
			dev_err(parinfo->dev,
				"Interleaved sources not supported with 8bpp "
				"pixel format\n");
			return -EINVAL;
		}

		for (line = y; line < last_line; line++) {
			__modelffb_command_p5(
				MODELF_COM_LOAD_IMAGE_AREA, MODELF_BPP_8,
				(x & 0x1ff), (line & 0x3ff), (width & 0x1ff),
				(1 & 0x3ff));
			__modelffb_data_transfer(
				(uint16_t*)((uint32_t)info->screen_base +
				(uint32_t)info->fix.line_length * line + x),
				((width + 1) / 2) * 2); /* 16-bit access */
		}
		break;
	case 16:
		if (parinfo->opt.interleaved_sources)
			fixup_interleaved_area(x, y, width, height);
		__modelffb_send_image_16b(x, y, width, height);
		break;
	default:
		break;
	}

#if TIME_SEND_IMAGE
	delta_jiffy = jiffies - start_jiffy;
	dev_info(parinfo->dev,
		 "Image (%d, %d) %dx%d (%d pixels) sent in %dms\n",
		 x, y, width, height, (width * height),
		 (delta_jiffy * 1000 / HZ));
#endif

	return retval;
}

static int modelffb_send_image(int x, int y, int width, int height)
{
	int retval = 0;

	retval = modelffb_lock();
	if (retval)
		return retval;
	retval = __modelffb_send_image(x, y, width, height);
	modelffb_unlock();
	return retval;
}

static void __modelffb_fill_frame_buffer(uint8_t color)
{
	struct fb_info *info = parinfo->fbinfo;
	uint8_t to_be_filled_color = 0;

	if (info->var.bits_per_pixel == 1) {
		to_be_filled_color = color & 1;
		to_be_filled_color =
			(to_be_filled_color << 0) |
			(to_be_filled_color << 1) |
			(to_be_filled_color << 2) |
			(to_be_filled_color << 3) |
			(to_be_filled_color << 4) |
			(to_be_filled_color << 5) |
			(to_be_filled_color << 6) |
			(to_be_filled_color << 7);
	}
	else {
		to_be_filled_color = color;
	}

	memset(info->screen_base, to_be_filled_color,
	       (info->var.xres * info->var.yres * 2));
	__modelffb_send_image(0, 0, info->var.xres, info->var.yres);
}

static int modelffb_fill_update_buffer(uint8_t color)
{
	int retval = 0;

	retval = modelffb_lock();
	if (retval)
		return retval;

	__modelffb_fill_frame_buffer(color);
	__modelffb_simple_command(MODELF_COM_INIT_UPDATE_BUFFER);
	__modelffb_simple_command(MODELF_COM_WAIT_TRIGGER_DONE);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	modelffb_unlock();
	return retval;
}

static int __modelffb_send_all_image(void)
{
	struct fb_info *info = parinfo->fbinfo;

	return __modelffb_send_image(0, 0, info->var.xres, info->var.yres);
}

static int modelffb_send_all_image(void)
{
	struct fb_info *info = parinfo->fbinfo;

	return modelffb_send_image(0, 0, info->var.xres, info->var.yres);
}

static int __modelffb_cleanup_full(int waveform_mode)
{
	int retval;

	__modelffb_simple_command_p1(MODELF_COM_UPDATE_FULL,
				     MODELF_WAVEFORM_MODE(waveform_mode));
	__modelffb_simple_command(MODELF_COM_WAIT_TRIGGER_DONE);
	__modelffb_simple_command(MODELF_COM_WAIT_FRAME_END);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	if (!retval) {
		SYNC_LOG("cleanup");
		modelffb_sync_set_status(MODELFFB_SYNC_BUSY);
	}

	return retval;
}

static int modelffb_cleanup_full(int waveform_mode)
{
	int retval;

#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev, "cleanup full in effect\n");
#endif

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	retval = pl_hardware_enable(parinfo->pl_hardware);
	if (retval) {
		dev_err(parinfo->dev, "Failed to enable PL hardware\n");
		goto err_exit;
	}
	wake_up_interruptible(&parinfo->sync_update_wait);
#endif

	retval = modelffb_lock();
	if (retval)
		goto err_power_off;

	retval = __modelffb_cleanup_full(waveform_mode);

	modelffb_unlock();

	return 0;

err_power_off:
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	pl_hardware_disable(parinfo->pl_hardware);
	wake_up_interruptible(&parinfo->sync_update_wait);
err_exit:
#endif

	return retval;
}

static int __modelffb_update_full(int waveform_mode)
{
	int retval;

	__modelffb_simple_command_p1(MODELF_COM_PARTIAL_UPDATE_FULL,
		MODELF_WAVEFORM_MODE(waveform_mode));
	__modelffb_simple_command(MODELF_COM_WAIT_TRIGGER_DONE);
	__modelffb_simple_command(MODELF_COM_WAIT_FRAME_END);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	if (!retval) {
		SYNC_LOG("update");
		modelffb_sync_set_status(MODELFFB_SYNC_BUSY);
	}

	return retval;
}

static int modelffb_update_full(int waveform_mode)
{
	int retval;

#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev, "update full in effect\n");
#endif

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	retval = pl_hardware_enable(parinfo->pl_hardware);
	if (retval) {
		dev_err(parinfo->dev, "Failed to enable PL hardware\n");
		goto err_exit;
	}
	wake_up_interruptible(&parinfo->sync_update_wait);
#endif

	retval = modelffb_lock();
	if (retval)
		goto err_power_off;

	retval = __modelffb_update_full(waveform_mode);

	modelffb_unlock();

	return 0;

err_power_off:
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	pl_hardware_disable(parinfo->pl_hardware);
	wake_up_interruptible(&parinfo->sync_update_wait);
err_exit:
#endif
	return retval;
}

static int __modelffb_find_free_lut(void)
{
	int i;

	for (i = 0; i < MODELF_MAX_LUT_NUM; i++) {
		if (parinfo->lut_in_use[i].busy == 0) {
			return i;
		}
	}

	return -ENOMEM;
}

static int __modelffb_rects_overlap(int x1, int y1, int width1, int height1,
	int x2, int y2, int width2, int height2)
{
	return ((x2 < (x1 + width1)) &&	((x2 + width2) > x1) &&
		(y2 < (y1 + height1)) && ((y2 + height2) > y1));
}

static int __modelffb_rect_no_overlap(int x, int y, int width, int height)
{
	int i;

	for (i = 0; i < MODELF_MAX_LUT_NUM; i++) {
		if (parinfo->lut_in_use[i].busy) {
			if (__modelffb_rects_overlap(
				    x, y, width, height,
				    parinfo->lut_in_use[i].x,
				    parinfo->lut_in_use[i].y,
				    parinfo->lut_in_use[i].width,
				    parinfo->lut_in_use[i].height)) {
				return 0;
			}
		}
	}

	return 1;
}

#ifdef CONFIG_MODELF_DEBUG
static void __modelffb_print_lut(void)
{
	int i;

	printk(KERN_INFO "MODELFFB: LUT info\n");

	for (i = 0; i < MODELF_MAX_LUT_NUM; i++) {
		printk(KERN_INFO "  %d: ", i);
		if (parinfo->lut_in_use[i].busy) {
			printk(KERN_INFO "(%d, %d) - (%d, %d)\n",
			       parinfo->lut_in_use[i].x,
			       parinfo->lut_in_use[i].y,
			       (parinfo->lut_in_use[i].x +
				parinfo->lut_in_use[i].width - 1),
			       (parinfo->lut_in_use[i].y +
				parinfo->lut_in_use[i].height - 1));
		} else {
			printk(KERN_INFO "\n");
		}
	}
}
#endif

static int __modelffb_request_lut(int x, int y, int width, int height)
{
	int retval = 0;
	int lut_id;

	if (!__modelffb_rect_no_overlap(x, y, width, height)) {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "Rectangle overlapped\n");
#endif
		retval = -EBUSY;
		goto end;
	}

	lut_id = __modelffb_find_free_lut();
	if (lut_id >= 0) {
		parinfo->lut_in_use[lut_id].busy = 1;
		parinfo->lut_in_use[lut_id].x = x;
		parinfo->lut_in_use[lut_id].y = y;
		parinfo->lut_in_use[lut_id].width = width;
		parinfo->lut_in_use[lut_id].height = height;

		retval = lut_id;
	} else {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "No free lut\n");
#endif
		retval = -EBUSY;
	}

end:
	return retval;
}

static int modelffb_request_lut(int x, int y, int width, int height)
{
	int retval = 0;

	retval = modelffb_lock();
	if (retval)
		return retval;
	__modelffb_request_lut(x, y, width, height);
	modelffb_unlock();
	return retval;
}

static int __modelffb_lut_n_is_busy(int n)
{
	uint16_t lut_status;

	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	lut_status = __modelffb_reg_read(MODELF_REG_LUT_STATUS);
	return (lut_status >> (n + 12)) & 0x1;
}

static int __modelffb_sync_lut(void)
{
	int i;

	for (i = 0; i < MODELF_MAX_LUT_NUM; i++) {
		parinfo->lut_in_use[i].busy = __modelffb_lut_n_is_busy(i);
	}

	return 0;
}

static int modelffb_sync_lut(void)
{
	int retval;

	retval = modelffb_lock();
	if (retval)
		return retval;
	__modelffb_sync_lut();
	modelffb_unlock();
	return 0;
}

static inline int __modelffb_lut_is_empty(void)
{
	int i;

	for (i = 0; i < MODELF_MAX_LUT_NUM; i++) {
		if (parinfo->lut_in_use[i].busy == 1) {
			return 0;
		}
	}

	return 1;
}

static inline int __modelffb_lut_queue_is_empty(void) {
	if (parinfo->lut_queue_count == 0)
		return 1;
	else
		return 0;
}

static inline int __modelffb_lut_queue_is_full(void) {
	if (parinfo->lut_queue_count >= MODELF_MAX_LUT_QUEUE_NUM)
		return 1;
	else
		return 0;
}

static int modelffb_enqueue_lut_queue(int oneshot_type, int waveform_mode,
	int x, int y, int width, int height)
{
	int retval = 0;
	int next;

	retval = modelffb_lock();
	if (retval)
		return retval;

	next = (parinfo->lut_queue_head + parinfo->lut_queue_count) %
		MODELF_MAX_LUT_QUEUE_NUM;

	if (!__modelffb_lut_queue_is_full()) {
		parinfo->lut_queue[next].oneshot_type = oneshot_type;
		parinfo->lut_queue[next].waveform_mode = waveform_mode;
		parinfo->lut_queue[next].x = x;
		parinfo->lut_queue[next].y = y;
		parinfo->lut_queue[next].width = width;
		parinfo->lut_queue[next].height = height;
		parinfo->lut_queue_count++;
		retval = parinfo->lut_queue_count;
	}
	else {
		retval = -ENOMEM;
	}

	modelffb_unlock();
	return retval;
}

static int __modelffb_dequeue_lut_queue(void)
{
	parinfo->lut_queue_head =
		(parinfo->lut_queue_head + 1) % MODELF_MAX_LUT_QUEUE_NUM;
	parinfo->lut_queue_count--;

	return parinfo->lut_queue_head;
}

static struct modelffb_oneshot_info* __modelffb_peek_lut_queue(void)
{
	return &parinfo->lut_queue[parinfo->lut_queue_head];
}

static int __modelffb_reset_lut_queue(void)
{
	parinfo->lut_queue_head = 0;
	parinfo->lut_queue_count = 0;
	return 0;
}

#ifdef CONFIG_MODELF_DEBUG
static void __modelffb_print_lut_queue(void)
{
	int i;

	dev_info(parinfo->dev, "QUEUE info\n  head = %d, count = %d\n",
		 parinfo->lut_queue_head, parinfo->lut_queue_count);

	for (i = 0; i < MODELF_MAX_LUT_QUEUE_NUM; i++) {
		dev_info(parinfo->dev, "  (%d, %d) - (%d, %d)\n",
		parinfo->lut_queue[i].x,
		parinfo->lut_queue[i].y,
		parinfo->lut_queue[i].x + parinfo->lut_queue[i].width - 1,
		parinfo->lut_queue[i].y + parinfo->lut_queue[i].height - 1);
	}
}
#endif

static void __modelffb_cleanup_area_lut(int x, int y, int width, int height,
					int waveform_mode, int lut)
{
	if (parinfo->opt.interleaved_sources)
		fixup_interleaved_area(x, y, width, height);

	x += parinfo->left_border;

	__modelffb_simple_command_p5(MODELF_COM_UPDATE_AREA,
		MODELF_WAVEFORM_MODE(waveform_mode) | MODELF_UPDATE_LUT(lut),
		x & 0x1ff, y & 0x3ff, width & 0x1ff, height & 0x3ff);
	__modelffb_simple_command(MODELF_COM_WAIT_TRIGGER_DONE);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	SYNC_LOG("cleanup_area_lut");
	modelffb_sync_set_status(MODELFFB_SYNC_BUSY);
}

static void modelffb_cleanup_area_lut(int x, int y, int width, int height,
				      int waveform_mode, int lut)
{
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	if (pl_hardware_enable(parinfo->pl_hardware)) {
		dev_err(parinfo->dev, "Failed to enable PL hardware\n");
		return;
	}
	wake_up_interruptible(&parinfo->sync_update_wait);
#endif

	if (modelffb_lock())
		goto err_power_off;

	__modelffb_cleanup_area_lut(x, y, width, height, waveform_mode, lut);

	modelffb_unlock();

	return;

err_power_off:
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	pl_hardware_disable(parinfo->pl_hardware);
	wake_up_interruptible(&parinfo->sync_update_wait);
#endif
	return;
}

static void modelffb_cleanup_area(int x, int y, int width, int height,
	int waveform_mode)
{
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	if (pl_hardware_enable(parinfo->pl_hardware)) {
		dev_err(parinfo->dev, "Failed to enable PL hardware\n");
		return;
	}
	wake_up_interruptible(&parinfo->sync_update_wait);
#endif

	if (modelffb_lock())
		goto err_power_off;

	if (parinfo->opt.interleaved_sources)
		fixup_interleaved_area(x, y, width, height);

	x += parinfo->left_border;

	__modelffb_wait_for_reg_value(MODELF_REG_LUT_STATUS,
				      MODELF_BF_LUT_IN_USE, 0,
				      MODELF_TIMEOUT_MS);
	__modelffb_simple_command_p5(MODELF_COM_UPDATE_AREA,
		MODELF_WAVEFORM_MODE(waveform_mode) | MODELF_UPDATE_LUT(0),
		x & 0x1ff, y & 0x3ff, width & 0x1ff, height & 0x3ff);
	__modelffb_simple_command(MODELF_COM_WAIT_TRIGGER_DONE);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	SYNC_LOG("cleanup_area");
	modelffb_sync_set_status(MODELFFB_SYNC_BUSY);

	modelffb_unlock();

err_power_off:
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	pl_hardware_disable(parinfo->pl_hardware);
	wake_up_interruptible(&parinfo->sync_update_wait);
#endif
	return;
}

static void __modelffb_update_area(int x, int y, int width, int height,
				 int waveform_mode, int lut)
{
	__modelffb_simple_command_p5(
		MODELF_COM_PARTIAL_UPDATE_AREA,
		MODELF_WAVEFORM_MODE(waveform_mode) | MODELF_UPDATE_LUT(lut),
		(x & 0x1ff), (y & 0x3ff), (width & 0x1ff), (height & 0x3ff));
}

static void modelffb_update_area(int x, int y, int width, int height,
				 int waveform_mode, int lut)
{
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	if (pl_hardware_enable(parinfo->pl_hardware)) {
		dev_err(parinfo->dev, "Failed to enable PL hardware\n");
		return;
	}
	wake_up_interruptible(&parinfo->sync_update_wait);
#endif

	if (modelffb_lock())
		goto err_power_off;

	__modelffb_update_area(x, y, width, height, waveform_mode, lut);

	modelffb_unlock();

err_power_off:
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	pl_hardware_disable(parinfo->pl_hardware);
	wake_up_interruptible(&parinfo->sync_update_wait);
#endif
	return;
}

static int modelffb_panel_init(void)
{
	modelffb_fill_update_buffer(0xff); /* fill white */
	return modelffb_cleanup_full(MODELF_WAVEFORM_WHITEOUT);
}

static void __modelffb_run(void)
{
	__modelffb_immediate_reg_write(
		MODELF_REG_POWER_SAVE_MODE,
		(__modelffb_reg_read(MODELF_REG_POWER_SAVE_MODE) &
		 ~MODELF_POWER_ACTIVE) | MODELF_POWER_ACTIVE);
	__modelffb_immediate_simple_command(MODELF_COM_RUN);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
}

static void modelffb_run(void)
{
	if (modelffb_lock())
		return;
	__modelffb_run();
	modelffb_unlock();

#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev, "WORKQUEUE: go into run mode\n");
#endif
}

static void __modelffb_standby(void)
{
	__modelffb_immediate_reg_write(
		MODELF_REG_POWER_SAVE_MODE,
		(__modelffb_reg_read(MODELF_REG_POWER_SAVE_MODE) &
		 ~MODELF_POWER_ACTIVE) | MODELF_POWER_ACTIVE);
	/* must not check HRDY before issuing command as this won't work on a
	 * sleep -> standby transition */
	__modelffb_immediate_simple_command(MODELF_COM_STANDBY);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
}

static void __modelffb_sleep(void)
{
	__modelffb_immediate_simple_command(MODELF_COM_SLEEP);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	__modelffb_immediate_reg_write(MODELF_REG_POWER_SAVE_MODE,
				       MODELF_POWER_PASSIVE);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
}

static void modelffb_sleep(void)
{
	if (modelffb_lock())
		return;
	__modelffb_sleep();
	modelffb_unlock();

#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev, "WORKQUEUE: go into sleep mode\n");
#endif
}

static void __modelffb_clear_temperature_judge(void)
{
	__modelffb_reg_write(MODELF_REG_DSPE_INT_STATUS,
			     MODELF_INT_WF_UPDATE_INTERRUPT);
}

static void modelffb_measure_temperature(void)
{
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	uint16_t regval;
	int temperature;
	int stat;
#endif

#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev, "Measuring temperature\n");
#endif

	if (modelffb_lock())
		return;

	/* only wait for LUTs if we're in run mode, shouldn't read synchronous
	 * registers otherwise */
	if (parinfo->power_mode == MODELF_POWER_RUN) {
		__modelffb_wait_for_reg_value(MODELF_REG_LUT_STATUS,
					      MODELF_BF_LUT_IN_USE, 0,
			MODELF_TIMEOUT_MS);
	}

	__modelffb_standby();
	__modelffb_simple_command(MODELF_COM_TEMPERATURE);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	if ((__modelffb_reg_read(MODELF_REG_DSPE_INT_STATUS)
			& MODELF_INT_WF_UPDATE_INTERRUPT) != 0) {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "Temperature zone has been changed\n");
#endif
		__modelffb_send_waveform();
		__modelffb_clear_temperature_judge();
	}
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	regval =  __modelffb_reg_read(MODELF_REG_SENSOR_TEMPERATURE);
#ifdef CONFIG_MODELF_DEBUG
	__modelffb_print_reg(MODELF_REG_SENSOR_TEMPERATURE);
#endif
	if (parinfo->power_mode == MODELF_POWER_RUN)
		__modelffb_run();
	else if (parinfo->power_mode == MODELF_POWER_SLEEP)
		__modelffb_sleep();

	modelffb_unlock();

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	mutex_lock(&temperature_lock);
	temperature = pl_hardware_constrain_temperature_range((int8_t)regval);
	stat = pl_hardware_set_temperature(parinfo->pl_hardware, temperature);
	if (stat)
		dev_err(parinfo->dev, "Failed to set the temperature\n");
	mutex_unlock(&temperature_lock);
#endif
}

/* =========== frame buffer stuffs =========== */
static void modelffb_check_panel_resolution(void)
{
	struct fb_info *info = parinfo->fbinfo;

	if (modelffb_lock())
		return;

	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	modelffb_unlock();

	info->var.xres = modelffb_read_register(MODELF_REG_LINE_DATA_LENGTH);
	info->var.yres = modelffb_read_register(MODELF_REG_FRAME_DATA_LENGTH);

	if (parinfo->opt.interleaved_sources) {
		info->var.xres *= 2;
		info->var.yres /= 2;
	}

	info->var.xres -= (parinfo->left_border + parinfo->right_border);
	info->var.xres_virtual = info->var.xres;
	info->var.yres_virtual = info->var.yres;

	dev_info(parinfo->dev, "FB resolution: %dx%d\n",
		 info->var.xres, info->var.yres);
}

static int modelffb_check_var(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	if ((var->xres != (info->fix.line_length / 2)) ||
	    (var->yres != (info->fix.smem_len / info->fix.line_length)) ||
	    (var->xres_virtual != (info->fix.line_length / 2)) ||
	    (var->yres_virtual !=
	     (info->fix.smem_len / info->fix.line_length))) {
		var->xres = info->fix.line_length / 2;
		var->yres = info->fix.smem_len / info->fix.line_length;
		var->xres_virtual = var->xres;
		var->yres_virtual = var->yres;

		return -EINVAL;
	}

	switch (var->bits_per_pixel) {
	case 1:
		var->grayscale		= 1;
		var->red.offset		= 0;
		var->red.length		= 1;
		var->red.msb_right	= 0;
		var->green.offset	= 0;
		var->green.length	= 1;
		var->green.msb_right	= 0;
		var->blue.offset	= 0;
		var->blue.length	= 1;
		var->blue.msb_right	= 0;
		var->transp.offset	= 0;
		var->transp.length	= 0;
		var->transp.msb_right	= 0;
		return 0;
	case 8:
		var->grayscale		= 1;
		var->red.offset		= 0;
		var->red.length		= 8;
		var->red.msb_right	= 0;
		var->green.offset	= 0;
		var->green.length	= 8;
		var->green.msb_right	= 0;
		var->blue.offset	= 0;
		var->blue.length	= 8;
		var->blue.msb_right	= 0;
		var->transp.offset	= 0;
		var->transp.length	= 0;
		var->transp.msb_right	= 0;
		return 0;
	case 16:
		var->grayscale		= 0;
		var->red.offset		= 0;
		var->red.length		= 5;
		var->red.msb_right	= 0;
		var->green.offset	= 5;
		var->green.length	= 6;
		var->green.msb_right	= 0;
		var->blue.offset	= 11;
		var->blue.length	= 5;
		var->blue.msb_right	= 0;
		var->transp.offset	= 0;
		var->transp.length	= 0;
		var->transp.msb_right	= 0;
		return 0;
	default:
		return -EINVAL;
	}
}

#ifndef CONFIG_MODELF_DEFERRED_IO
static int modelffb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long page, pos;

	if ((offset + size) > info->fix.smem_len)
		return -EINVAL;

	pos = (unsigned long)info->fix.smem_start + offset;

	while (size > 0) {
		page = vmalloc_to_pfn((void *)pos);

		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;

		start += PAGE_SIZE;
		pos += PAGE_SIZE;

		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	vma->vm_flags |= VM_RESERVED; /* avoid to swap out this VMA */

	return 0;
}
#endif

static struct fb_ops modelffb_ops = {
	.owner		= THIS_MODULE,
#ifndef CONFIG_MODELF_DEFERRED_IO
	.fb_read	= fb_sys_read,
	.fb_write	= fb_sys_write,
	.fb_mmap	= modelffb_mmap,
#endif
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
	.fb_check_var	= modelffb_check_var,
};

static struct fb_fix_screeninfo modelffb_fix __devinitdata = {
	.id			= "modelffb",
	.type			= FB_TYPE_PACKED_PIXELS,
	.visual			= FB_VISUAL_TRUECOLOR,
	.xpanstep		= 0,
	.ypanstep		= 0,
	.ywrapstep		= 0,
	.accel			= FB_ACCEL_NONE,
};

/* RGB565 by default */
static struct fb_var_screeninfo modelffb_var __devinitdata = {
	.bits_per_pixel		= 16,
	.grayscale		= 0,
	.red.offset		= 0,
	.red.length		= 5,
	.red.msb_right		= 0,
	.green.offset		= 5,
	.green.length		= 6,
	.green.msb_right	= 0,
	.blue.offset		= 11,
	.blue.length		= 5,
	.blue.msb_right		= 0,
	.transp.offset		= 0,
	.transp.length		= 0,
	.transp.msb_right	= 0,
};

static int __devinit modelffb_framebuffer_alloc(struct platform_device *pdev)
{
	struct fb_info *info;
	int retval;

	info = framebuffer_alloc(sizeof(struct modelffb_par), &pdev->dev);
	platform_set_drvdata(pdev, info);

	if (!info) {
		dev_err(parinfo->dev, "Failed to alloc modelffb fb_info\n");
		retval = -ENOMEM;
		goto err;
	}

	parinfo = info->par;
	if (!parinfo) {
		dev_err(parinfo->dev, "Failed to alloc modelffb_par\n");
		retval = -ENOMEM;
		goto framebuffer_release;
	}

	parinfo->pdev = pdev;
	parinfo->dev = &pdev->dev;
	parinfo->pdata = pdev->dev.platform_data;

	info->pseudo_palette = kmalloc(sizeof(u32) * 256, GFP_KERNEL);
	if (!info->pseudo_palette) {
		dev_err(parinfo->dev, "Failed to alloc pseudo_palette\n");
		retval = -ENOMEM;
		goto framebuffer_release;
	}

	info->fix = modelffb_fix;
	info->var = modelffb_var;
	info->fbops = &modelffb_ops;

	parinfo->fbinfo = info;
	parinfo->left_border = 0;
	parinfo->right_border = 0;
	parinfo->lut_queue_head = 0;
	parinfo->lut_queue_count = 0;
	memset(parinfo->lut_in_use, 0, MODELF_MAX_LUT_NUM);
	memset(parinfo->lut_queue, 0, MODELF_MAX_LUT_QUEUE_NUM);

	parinfo->opt.clear_on_init = 1;
	parinfo->opt.clear_on_exit = 0;
	parinfo->opt.interleaved_sources = 0;
	parinfo->opt.spi_freq_hz = 0;

	return 0;

framebuffer_release:
	framebuffer_release(info);
err:
	return retval;
}

static void __devexit modelffb_framebuffer_release(void)
{
	struct fb_info *info = parinfo->fbinfo;

#ifdef CONFIG_I2C_SC18IS60X_SHARED_SPI
	mutex_lock(&sc18is60x_spi_lock);
	sc18is60x_shared_spi = NULL;
	mutex_unlock(&sc18is60x_spi_lock);
#endif

	kfree(info->pseudo_palette);
	framebuffer_release(info);
}

static int modelffb_alloc_vram(void)
{
	struct fb_info *info = parinfo->fbinfo;
	size_t ep_xres;
	size_t ep_line_length;
	size_t image_pool_size;
	size_t mem_len;
	int retval = 0;

	mem_len = info->var.xres * info->var.yres * 2;
#ifndef CONFIG_MODELF_DEFERRED_IO
	mem_len = roundup(mem_len, PAGE_SIZE);
#endif
	info->fix.smem_len	= mem_len;
	info->fix.mmio_len	= mem_len;
	info->fix.line_length	= info->var.xres * 2;

	info->screen_base = vmalloc(info->fix.smem_len + 4);
	/* extra 4 bytes of the guard for 16-bit access */
	if (!info->screen_base) {
		dev_err(parinfo->dev, "Failed to alloc video memory\n");
		retval = -ENOMEM;
		goto err;
	}

	if (parinfo->opt.interleaved_sources)
		ep_xres = info->var.xres / 2;
	else
		ep_xres = info->var.xres;

	ep_line_length = ep_xres * 2; /* 16bpp */
	parinfo->image_pool_lines = IMAGE_POOL_MAX_SZ / ep_line_length;
	if (parinfo->opt.interleaved_sources)
		parinfo->image_pool_lines =
			round_down(parinfo->image_pool_lines, 2);
	image_pool_size = ep_line_length * parinfo->image_pool_lines;
	dev_info(parinfo->dev, "Image pool lines: %d, size: %zu\n",
		 parinfo->image_pool_lines, image_pool_size);
	parinfo->image_pool = kmalloc(image_pool_size, GFP_KERNEL);
	if (!info->screen_base) {
		dev_err(parinfo->dev, "Failed to allocate image pool\n");
		retval = -ENOMEM;
		goto free_screen;
	}

	info->fix.smem_start = (uint32_t)info->screen_base;
	info->fix.mmio_start = (uint32_t)info->screen_base;

	memset(info->screen_base, 0xff, info->fix.smem_len);

	return 0;

free_screen:
	vfree(info->screen_base);
err:
	return retval;
}

static void __devexit modelffb_free_vram(void)
{
	struct fb_info *info = parinfo->fbinfo;

	kfree(parinfo->image_pool);
	vfree(info->screen_base);
}

static void __modelffb_start_temperature_timer(void)
{
	if (parinfo->seconds_to_measure_temperature > 0) {
		parinfo->temperature_timer.expires = jiffies +
			(parinfo->seconds_to_measure_temperature * HZ);
		add_timer(&parinfo->temperature_timer);
	}
}

static void modelffb_temperature_work_function(struct work_struct *ws)
{
	modelffb_measure_temperature();
	__modelffb_start_temperature_timer();
}

static DECLARE_WORK(modelffb_temperature_work,
		    modelffb_temperature_work_function);

static int __modelffb_queue_temperature(void)
{
	queue_work(parinfo->workqueue, &modelffb_temperature_work);
	return 0;
}

static void __modelffb_temperature_timer_handler(unsigned long data)
{
	__modelffb_queue_temperature();
}

static int modelffb_ready_to_standby_sleep(void)
{
	int retval;

	modelffb_sync_lut();
	modelffb_lock();
	retval = __modelffb_check_HRDY_ready() && __modelffb_lut_is_empty();
	modelffb_unlock();

	return retval;
}

static void modelffb_commit_sleep(void)
{
	if (parinfo->power_mode != MODELF_POWER_SLEEP) {
		if (!modelffb_ready_to_standby_sleep()) {
			return;
		}
		del_timer_sync(&parinfo->sleep_timer);
		parinfo->power_mode = MODELF_POWER_SLEEP;

		modelffb_sleep();

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	pl_hardware_disable(parinfo->pl_hardware);
	wake_up_interruptible(&parinfo->sync_update_wait);
#endif
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "WORKQUEUE: go into standby mode\n");
#endif
	}
	else {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "WORKQUEUE: already in sleep mode\n");
#endif
	}
}

static void modelffb_sleep_work_function(struct work_struct *ws)
{
	modelffb_commit_sleep();
}

static DECLARE_WORK(modelffb_sleep_work, modelffb_sleep_work_function);

static int __modelffb_queue_sleep(void)
{
	queue_work(parinfo->workqueue, &modelffb_sleep_work);
	return 0;
}

static void __modelffb_sleep_timer_handler(unsigned long data)
{
	__modelffb_queue_sleep();
}

/* allow 2.5 x normal frame time after an update completes before HV off */
#define HV_OFF_MSECS 50

static void __modelffb_set_sleep_timer(void)
{
	if (parinfo->seconds_to_sleep > 0) {
		parinfo->sleep_timer.expires = jiffies +
			(HV_OFF_MSECS * HZ / 1000);
		add_timer(&parinfo->sleep_timer);
	}
}

static void modelffb_commit_run(void)
{
	del_timer_sync(&parinfo->sleep_timer);

	if (parinfo->power_mode != MODELF_POWER_RUN) {
		parinfo->power_mode = MODELF_POWER_RUN;
		modelffb_run();
		__modelffb_set_sleep_timer();
	}
	else {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "WORKQUEUE: already in run mode\n");
#endif
	}
}

static void modelffb_oneshot(int waveform_mode, int x, int y, int width,
			     int height)
{
	int free_lut;

	free_lut = modelffb_request_lut(x, y, width, height);

	if (free_lut) {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev,
			 "LUT is busy => enqueue this oneshot\n");
#endif
		if (modelffb_enqueue_lut_queue(
			    MODELF_ONESHOT_TYPE_ONESHOT, waveform_mode,
			    x, y, width, height)) {
			parinfo->need_flush_image = MODELF_NEED_FLUSH_IMAGE;
			dev_info(parinfo->dev,
				 "LUT queue has been overflowed, "
				 "all image will be sent at once.\n");
		}
	} else {
		modelffb_send_image(x, y, width, height);
		modelffb_update_area(x, y, width, height,
				     parinfo->waveform_mode, free_lut);
		/* HIRQ handler will call modelf_queue_sleep() later */
	}
}

static void modelffb_oneshot_cleanup(int waveform_mode, int x, int y,
				     int width, int height)
{
	int free_lut;

	free_lut = modelffb_request_lut(x, y, width, height);

	if (free_lut) {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "LUT is busy, enqueue this cleanup\n");
#endif
		if (modelffb_enqueue_lut_queue(
			    MODELF_ONESHOT_TYPE_CLENUP, waveform_mode,
			    x, y, width, height)) {
#ifdef CONFIG_MODELF_DEFERRED_IO
			if (parinfo->vram_updated == MODELF_VRAM_NEED_UPDATE)
				parinfo->need_flush_image =
					MODELF_NEED_FLUSH_IMAGE;
#endif
			parinfo->need_cleanup = MODELF_NEED_CLEANUP;
			dev_info(parinfo->dev,
				 "LUT queue has been overflowed, "
				 "cleanup all will be committed later.\n");
		}
	} else {
#ifdef CONFIG_MODELF_DEFERRED_IO
		if (parinfo->vram_updated == MODELF_VRAM_NEED_UPDATE)
#endif
			modelffb_send_image(x, y, width, height);
		modelffb_cleanup_area_lut(x, y, width, height, waveform_mode,
					  free_lut);
		/* HIRQ handler will call modelf_queue_sleep() later */
	}
}

static void modelffb_oneshot_work_function(struct work_struct *ws)
{
	struct modelffb_oneshot_work *work = (struct modelffb_oneshot_work*)ws;

	modelffb_commit_run();
#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev, "%s (%d, %d) - (%d, %d)\n",
		 (work->oneshot_type == MODELF_ONESHOT_TYPE_ONESHOT ?
		  "oneshot" : "cleanup"),
		 work->x, work->y, (work->x + work->width - 1),
		 (work->y + work->height - 1));
#endif

	if (parinfo->suspend_update == MODELF_RESUME_UPDATE) {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "Update has resumed\n");
#endif
	} else if (work->oneshot_type == MODELF_ONESHOT_TYPE_CLENUP) {
		modelffb_oneshot_cleanup(work->waveform_mode, work->x, work->y,
					 work->width, work->height);
#ifdef CONFIG_MODELF_DEFERRED_IO
	} else if (parinfo->vram_updated == MODELF_VRAM_NO_NEED_UPDATE) {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "VRAM is up to date\n");
#endif
#endif
	} else {
		modelffb_oneshot(work->waveform_mode,
			work->x, work->y, work->width, work->height);
	}

#ifdef CONFIG_MODELF_DEBUG
	__modelffb_print_lut();
	__modelffb_print_lut_queue();
#endif
	kfree((void*)work);
}

static int __modelffb_queue_oneshot(int oneshot_type, int waveform_mode,
				    int x, int y, int width, int height)
{
	int retval = 0;
	struct modelffb_oneshot_work *os_work;

	os_work = kmalloc(sizeof(struct modelffb_oneshot_work), GFP_KERNEL);
	if (!os_work) {
		dev_err(parinfo->dev, "Failed to allocate oneshot work\n");
		retval = -ENOMEM;
		goto end;
	}
	INIT_WORK((struct work_struct*)os_work,
		  modelffb_oneshot_work_function);

	os_work->oneshot_type = oneshot_type;
	os_work->waveform_mode = waveform_mode;
	os_work->x = x;
	os_work->y = y;
	os_work->width = width;
	os_work->height = height;

	queue_work(parinfo->workqueue, (struct work_struct*)os_work);

end:
	return retval;
}

static void modelffb_cleanup_work_function(struct work_struct *ws)
{
	struct modelffb_cleanup_work *work = (struct modelffb_cleanup_work*)ws;
	struct fb_info *info = parinfo->fbinfo;

#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev,
		 "WORKQUEUE: cleanup mode_%d (%d, %d) - (%d, %d)\n",
		 work->waveform_mode, work->x, work->y,
		 work->x + work->width - 1, work->y + work->height - 1);
#endif

	modelffb_commit_run();
	if ((work->x == 0) && (work->y == 0) &&
	    (work->width == info->var.xres) &&
	    (work->height == info->var.yres)) {
		modelffb_cleanup_full(work->waveform_mode);
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "cleanup all\n");
#endif
	} else {
		modelffb_cleanup_area(work->x, work->y, work->width,
				      work->height, work->waveform_mode);
	}
	/* HIRQ handler will call modelf_queue_sleep() later */

	kfree((void*)work);
}

static int __modelffb_queue_cleanup(int waveform_mode, int x, int y,
				    int width, int height)
{
	SYNC_LOG("queue_cleanup");
	modelffb_sync_set_status(MODELFFB_SYNC_PENDING);

	if (parinfo->suspend_update == MODELF_SUSPEND_UPDATE) {
		__modelffb_queue_oneshot(MODELF_ONESHOT_TYPE_CLENUP,
					 waveform_mode, x, y, width, height);
	} else {
		struct modelffb_cleanup_work *cu_work;
		cu_work = kmalloc(sizeof(struct modelffb_cleanup_work),
				  GFP_KERNEL);
		if (!cu_work) {
			dev_err(parinfo->dev,
				"Failed to allocate cleanup queue\n");
			return -ENOMEM;
		}
		INIT_WORK((struct work_struct*)cu_work,
			  modelffb_cleanup_work_function);

		cu_work->waveform_mode = waveform_mode;
		cu_work->x = x;
		cu_work->y = y;
		cu_work->width = width;
		cu_work->height = height;
		queue_work(parinfo->workqueue, (struct work_struct *)cu_work);
	}

	return 0;
}

static void modelffb_suspend_update_work_function(struct work_struct *ws)
{
	struct modelffb_suspend_update_work *work =
		(struct modelffb_suspend_update_work*)ws;

	if (work->update == MODELF_SUSPEND_UPDATE) {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "WORKQUEUE: suspend update\n");
#endif
	} else if (work->update == MODELF_RESUME_UPDATE) {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "WORKQUEUE: resume update\n");
#ifdef CONFIG_MODELF_DEFERRED_IO
		parinfo->vram_updated = MODELF_VRAM_NO_NEED_UPDATE;
#endif
#endif

		modelffb_commit_run();
		modelffb_send_all_image();
		modelffb_update_full(parinfo->waveform_mode);
		/* HIRQ handler will call modelf_queue_sleep() later */
	}
	parinfo->suspend_update = work->update;

	kfree((void*)work);
}

static int __modelffb_queue_suspend_update(int update)
{
	int retval = 0;
	struct modelffb_suspend_update_work *su_work;

	if (parinfo->suspend_update == update) {
		dev_err(parinfo->dev, "%s doubled\n",
			parinfo->suspend_update == MODELF_SUSPEND_UPDATE ?
			"suspend update" : "resume update");
		goto end;
	}

	su_work = kmalloc(sizeof(struct modelffb_suspend_update_work),
			  GFP_KERNEL);
	if (!su_work) {
		dev_err(parinfo->dev,
			"Failed to allocate suspend_update work\n");
		retval = -ENOMEM;
		goto end;
	}
	INIT_WORK((struct work_struct*)su_work,
		  modelffb_suspend_update_work_function);

	su_work->update = update;

	queue_work(parinfo->workqueue, (struct work_struct*)su_work);

end:
	return retval;
}

static void modelffb_irq_work_function(struct work_struct *ws)
{
	int free_lut;

	SYNC_LOG("IRQ");

	del_timer_sync(&parinfo->sleep_timer);
#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev, "WORKQUEUE: HIRQ\n");
#endif

	if (modelffb_lock())
		return;
	__modelffb_run();
	__modelffb_reg_write(MODELF_REG_DSPE_INT_STATUS,
			     MODELF_INT_DSPE_ONE_LUT_FREE);
	__modelffb_reg_write(MODELF_REG_INTERRUPT_STATUS,
			     MODELF_INT_DISPLAY_ENGINE);
	__modelffb_sync_lut();

	if ((parinfo->need_flush_image == MODELF_NO_NEED_FLUSH_IMAGE) &&
	    (parinfo->need_cleanup == MODELF_NO_NEED_CLEANUP)) {
		struct modelffb_oneshot_info *os_info;

		while (1) {
			if (__modelffb_lut_queue_is_empty())
				break;

			os_info = __modelffb_peek_lut_queue();
			free_lut = __modelffb_request_lut(
				os_info->x, os_info->y, os_info->width,
				os_info->height);

			if (free_lut)
				break;

			__modelffb_dequeue_lut_queue();

#ifdef CONFIG_MODELF_DEFERRED_IO
			if (parinfo->vram_updated == MODELF_VRAM_NEED_UPDATE)
				__modelffb_send_image(
					os_info->x, os_info->y,
					os_info->width, os_info->height);
#endif

			if (os_info->oneshot_type ==
			    MODELF_ONESHOT_TYPE_ONESHOT)
				__modelffb_update_area(
					os_info->x, os_info->y,
					os_info->width, os_info->height,
					os_info->waveform_mode, free_lut);
			else if (os_info->oneshot_type ==
				 MODELF_ONESHOT_TYPE_CLENUP)
				__modelffb_cleanup_area_lut(
					os_info->x, os_info->y,
					os_info->width, os_info->height,
					os_info->waveform_mode, free_lut);
		}

		modelffb_sync_set_status(MODELFFB_SYNC_IDLE);
	} else if (__modelffb_lut_is_empty()) {
		if (parinfo->need_flush_image == MODELF_NEED_FLUSH_IMAGE) {
			__modelffb_reset_lut_queue();
			parinfo->need_flush_image = MODELF_NO_NEED_FLUSH_IMAGE;
#ifdef CONFIG_MODELF_DEFERRED_IO
			parinfo->vram_updated = MODELF_VRAM_NO_NEED_UPDATE;
#endif
			__modelffb_send_all_image();
		}

		if (parinfo->need_cleanup == MODELF_NEED_CLEANUP) {
			parinfo->need_cleanup = MODELF_NO_NEED_CLEANUP;
			__modelffb_cleanup_full(parinfo->waveform_mode);
		} else if (parinfo->need_cleanup == MODELF_NO_NEED_CLEANUP)
			__modelffb_update_full(parinfo->waveform_mode);
	}

	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (parinfo->power_mode == MODELF_POWER_RUN)
		__modelffb_run();
	else if (parinfo->power_mode == MODELF_POWER_SLEEP)
		__modelffb_sleep();

	modelffb_unlock();
	__modelffb_set_sleep_timer();
	enable_irq(parinfo->pdata->hirq);
}

static DECLARE_WORK(modelffb_irq_work, modelffb_irq_work_function);

static irqreturn_t __modelffb_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(parinfo->pdata->hirq);
	queue_work(parinfo->workqueue, &modelffb_irq_work);

	return IRQ_HANDLED;
}

#ifdef CONFIG_MODELF_DEFERRED_IO
static void modelffb_deferred_io_work_function(struct work_struct *ws)
{
#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev, "WORKQUEUE: deferred\n");
#endif

	if (parinfo->suspend_update == MODELF_SUSPEND_UPDATE) {
#ifdef CONFIG_MODELF_DEBUG
		dev_info(parinfo->dev, "Update has been suspended\n");
#endif
	}
	else if (parinfo->vram_updated == MODELF_VRAM_NEED_UPDATE) {
		parinfo->vram_updated = MODELF_VRAM_NO_NEED_UPDATE;
		modelffb_commit_run();
		modelffb_send_all_image();
		modelffb_update_full(parinfo->waveform_mode);
	}
}

static DECLARE_WORK(modelffb_deferred_io_work,
		    modelffb_deferred_io_work_function);

static void __modelffb_dpy_deferred_io(struct fb_info *info,
				       struct list_head *pagelist)
{
#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev, "Deferred I/O\n");
#endif
	parinfo->vram_updated = MODELF_VRAM_NEED_UPDATE;
	queue_work(parinfo->workqueue, &modelffb_deferred_io_work);
}

static struct fb_deferred_io __modelffb_defio = {
	.delay		= HZ / MODELF_DEFERRED_IO_DELAY_DENOMINATOR,
	.deferred_io	= __modelffb_dpy_deferred_io,
};
#endif /* CONFIG_MODELF_DEFERRED_IO */

static int __devinit modelffb_register_framebuffer(void)
{
	struct fb_info *info = parinfo->fbinfo;
	int retval;

#ifdef CONFIG_MODELF_DEFERRED_IO
	info->fbdefio = &__modelffb_defio;
	fb_deferred_io_init(info);
	parinfo->vram_updated = MODELF_VRAM_NO_NEED_UPDATE;
#endif

	retval = register_framebuffer(info);
	if (retval) {
		dev_err(parinfo->dev, "Failed to register frame buffer (%d)\n",
			retval);
		goto err;
	}

	return 0;

err:
#ifdef CONFIG_MODELF_DEFERRED_IO
	fb_deferred_io_cleanup(info);
#endif
	return retval;
}

static void modelffb_unregister_framebuffer(void)
{
	struct fb_info *info = parinfo->fbinfo;

	unregister_framebuffer(info);
#ifdef CONFIG_MODELF_DEFERRED_IO
	fb_deferred_io_cleanup(info);
#endif
}

static int modelffb_regulator_init(void)
{
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	int stat;

	parinfo->pl_hardware = pl_hardware_alloc();
	if (!parinfo->pl_hardware) {
		stat = -ENOMEM;
		goto exit_now;
	}

	stat = pl_hardware_init(parinfo->pl_hardware, &modelffb_pl_config);
	if (stat) {
		dev_err(parinfo->dev,
			"Failed to initialize Plastic Logic hardware\n");
		goto exit_free_plhw;
	}

	return 0;

exit_free_plhw:
	pl_hardware_free(parinfo->pl_hardware);
exit_now:
	return stat;
#else
	return 0;
#endif
}

static void modelffb_regulator_exit(void)
{
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	if (parinfo->pl_hardware)
		pl_hardware_free(parinfo->pl_hardware);
#endif
}

static int modelffb_modelf_init(void)
{
	int stat;

	if (!(parinfo->status & MODELF_STATUS_INITCODE_STORED)) {
		dev_err(parinfo->dev, "Init code not stored\n");
		return -EINVAL;
	}

	if (!(parinfo->status & MODELF_STATUS_WAVEFORM_STORED)) {
		dev_err(parinfo->dev, "Waveform not stored\n");
		return -EINVAL;
	}

	stat = modelffb_chip_init();
	if (stat) {
		dev_err(parinfo->dev, "chip_init failed\n");
		goto exit_now;
	}

	modelffb_check_panel_resolution();

	stat = modelffb_alloc_vram();
	if (stat) {
		dev_err(parinfo->dev, "alloc_vram failed\n");
		goto exit_now;
	}

	stat = modelffb_register_framebuffer();
	if (stat) {
		dev_err(parinfo->dev, "register_framebuffer failed\n");
		goto free_vram;
	}

	stat = request_irq(parinfo->pdata->hirq, __modelffb_irq_handler,
			   (IRQF_DISABLED | IRQF_TRIGGER_HIGH), "modelf",
			   parinfo->dev);
	if (stat) {
		dev_err(parinfo->dev, "request_irq failed\n");
		goto framebuffer_unregister;
	}

	stat = modelffb_regulator_init();
	if (stat) {
		dev_err(parinfo->dev, "regulator_init failed\n");
		goto free_hirq;
	}

	if (parinfo->opt.clear_on_init) {
		stat = modelffb_panel_init();
		if (stat) {
			dev_err(parinfo->dev, "panel_init failed\n");
			goto free_regulator;
		}
	}

	modelffb_measure_temperature();
	__modelffb_start_temperature_timer();

#ifdef CONFIG_MODELF_DEBUG
	modelffb_print_reg(MODELF_REG_DSPE_INT_STATUS);
	modelffb_print_reg(MODELF_REG_SENSOR_TEMPERATURE);
#endif

	return 0;

free_regulator:
	modelffb_regulator_exit();
free_hirq:
	free_irq(parinfo->pdata->hirq, parinfo->dev);
framebuffer_unregister:
	modelffb_unregister_framebuffer();
free_vram:
	modelffb_free_vram();
exit_now:

	return stat;
}

/* =========== file accessor =========== */
static ssize_t __modelffb_store_init_code(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if (count > MODELF_MAX_INITCODE_SIZE) {
		dev_err(parinfo->dev, "MAX_INITCODE_SIZE has been reached\n");
	}
	else {
		memcpy((void*)parinfo->init_code, buf, count);
		parinfo->init_code_size += count;
	}

	parinfo->status |= MODELF_STATUS_INITCODE_STORED;
	return count;
}

static ssize_t __modelffb_store_waveform(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if (parinfo->waveform_index + count >
		parinfo->waveform + MODELF_MAX_WAVEFORM_SIZE) {
		dev_err(parinfo->dev, "MAX_WAVEFORM_SIZE has been reached\n");
	}
	else {
		memcpy((void*)parinfo->waveform_index, buf, count);
		parinfo->waveform_index += count;
		parinfo->waveform_size += count;
	}

	parinfo->status |= MODELF_STATUS_WAVEFORM_STORED;
	return count;
}

static int __parse_uint32_2(char *str, uint32_t *int1, uint32_t *int2)
{ /* str "0xff 123" => int1 255 & int2 123 */
	int retval = 0;
	char *tok, *next_tok = str;

	tok = strsep(&next_tok, " \t\n");
	if (tok != NULL) {
		if (strncmp(tok, "0x", 2) == 0)
			sscanf(tok, "0x%x", int1);
		else
			sscanf(tok, "%d", int1);
	}
	else {
		retval = -1;
		goto end;
	}

	tok = strsep(&next_tok, " \t\n");
	if (tok != NULL) {
		if (strncmp(tok, "0x", 2) == 0)
			sscanf(tok, "0x%x", int2);
		else
			sscanf(tok, "%d", int2);
	}
	else {
		retval = -2;
		goto end;
	}

end:
	return retval;
}

static int __parse_uint32_4(char *str, uint32_t *int1, uint32_t *int2,
	uint32_t *int3, uint32_t *int4)
{ /* str "0xff 123 0x0 12" => int1 255 & int2 123 & int3 0 & int4 12 */
	int retval = 0;
	char *tok, *next_tok = str;

	tok = strsep(&next_tok, " \t\n");
	if (tok != NULL) {
		if (strncmp(tok, "0x", 2) == 0)
			sscanf(tok, "0x%x", int1);
		else
			sscanf(tok, "%d", int1);
	}
	else {
		retval = -1;
		goto end;
	}

	tok = strsep(&next_tok, " \t\n");
	if (tok != NULL) {
		if (strncmp(tok, "0x", 2) == 0)
			sscanf(tok, "0x%x", int2);
		else
			sscanf(tok, "%d", int2);
	}
	else {
		retval = -2;
		goto end;
	}

	tok = strsep(&next_tok, " \t\n");
	if (tok != NULL) {
		if (strncmp(tok, "0x", 2) == 0)
			sscanf(tok, "0x%x", int3);
		else
			sscanf(tok, "%d", int3);
	}
	else {
		retval = -3;
		goto end;
	}

	tok = strsep(&next_tok, " \t\n");
	if (tok != NULL) {
		if (strncmp(tok, "0x", 2) == 0)
			sscanf(tok, "0x%x", int4);
		else
			sscanf(tok, "%d", int4);
	}
	else {
		retval = -4;
		goto end;
	}

end:
	return retval;
}

static int __modelffb_set_element(char *str)
{
	char *tok, *next_tok = str;

	tok = strsep(&next_tok, " \t\n");
	if (!tok) {
		dev_err(parinfo->dev, "No element identifier\n");
		return -EINVAL;
	}

	if (!strcmp(tok, "waveform_mode")) {
		/* echo set waveform_mode mode >
		   /sys/devices/platform/modelffb/control */
		tok = strsep(&next_tok, " \t\n");
		if (!tok) {
			dev_err(parinfo->dev,
				"No waveform mode specified\n");
			return -EINVAL;
		}

		sscanf(tok, "%d", &parinfo->waveform_mode);
		dev_info(parinfo->dev, "Waveform mode has been set as %d\n",
			 parinfo->waveform_mode);
	} else if (!strcmp(tok, "powercut_time")) {
		/* echo set powercut_time seconds >
		   /sys/devices/platform/modelffb/control */
		tok = strsep(&next_tok, " \t\n");
		if (!tok) {
			dev_err(parinfo->dev,
				"No powercut time specified\n");
			return -EINVAL;
		}

		del_timer_sync(&parinfo->sleep_timer);
		sscanf(tok, "%d", &parinfo->seconds_to_sleep);
		dev_info(parinfo->dev, "%d seconds to standby\n",
			 parinfo->seconds_to_sleep);
		__modelffb_queue_sleep();
	} else if (strcmp(tok, "measure_temp_interval") == 0) {
		/* echo set measure_temp_interval seconds >
		   /sys/devices/platform/modelffb/control */
		tok = strsep(&next_tok, " \t\n");
		if (!tok) {
			dev_err(parinfo->dev,
				"No interval time specified\n");
			return -EINVAL;
		}

		del_timer_sync(&parinfo->temperature_timer);
		sscanf(tok, "%d",
		       &parinfo->seconds_to_measure_temperature);
		dev_info(parinfo->dev, "%d seconds to measure temperature\n",
			 parinfo->seconds_to_measure_temperature);
		if (parinfo->seconds_to_measure_temperature)
			__modelffb_queue_temperature();
	} else if (!strcmp(tok, "send_waveform")) {
		/* echo set send_waveform_wait wait/nowait >
		   /sys/devices/platform/modelffb/control */
		tok = strsep(&next_tok, " \t\n");
		if (!tok) {
			dev_err(parinfo->dev,
				"No wait selection specified\n");
			return -EINVAL;
		}

		if (!strcmp(tok, "wait")) {
			dev_info(parinfo->dev, "send waveform: wait\n");
			parinfo->send_waveform_wait =
				MODELF_SEND_WAVEFORM_WAIT;
		} else if (!strcmp(tok, "nowait")) {
			dev_info(parinfo->dev, "send waveform: nowait\n");
			parinfo->send_waveform_wait =
				MODELF_SEND_WAVEFORM_NOWAIT;
		}
	} else {
		dev_err(parinfo->dev, "Invalid set argument\n");
		return -EINVAL;
	}

	return 0;
}

static int modelffb_setopt(struct fb_info *info, char *tokbuf, size_t len)
{
	char *opt;
	char *value;
	long unsigned int_value;

	opt = strsep(&tokbuf, " \t\n");

	if (!opt) {
		dev_err(parinfo->dev, "No option identifier specified\n");
		return -EINVAL;
	}

	value = strsep(&tokbuf, " \t\n");

	if (!value) {
		dev_err(parinfo->dev, "No value specified for %s\n", opt);
		return -EINVAL;
	}

	if (kstrtoul(value, 10, &int_value)) {
		dev_err(parinfo->dev, "Invalid numerical value: %s\n", value);
		return -EINVAL;
	}

	if (!strcmp(opt, "clear_on_exit") && value) {
		parinfo->opt.clear_on_exit = int_value ? 1 : 0;
	} else if (!strcmp(opt, "clear_on_init") && value) {
		parinfo->opt.clear_on_init = int_value ? 1 : 0;
	} else if (!strcmp(opt, "interleaved_sources") && value) {
		if (parinfo->status & MODELF_STATUS_INIT_DONE) {
			dev_err(parinfo->dev,
				"Controller already initialised\n");
			return -EINVAL;
		}

		parinfo->opt.interleaved_sources = int_value ? 1 : 0;
	} else if (!strcmp(opt, "spi_freq_hz")) {
		parinfo->opt.spi_freq_hz = int_value;
	} else {
		dev_err(parinfo->dev, "Invalid setopt identifier\n");
	}

	return 0;
}

static ssize_t modelffb_store_control(
	struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	char localbuf[256];
	char *tok, *next_tok;
	struct fb_info *info = parinfo->fbinfo;

	strncpy(localbuf, buf, 255);
	localbuf[255] = '\0';
	next_tok = localbuf;

	tok = strsep(&next_tok, " \t\n");

	if (!strcmp(tok, "init")) {
	/* echo init > /sys/devices/platform/modelffb/control */
		if (parinfo->status & MODELF_STATUS_INIT_DONE) {
			dev_err(parinfo->dev, "Already initialised\n");
		} else if (modelffb_modelf_init() == 0)
			parinfo->status |= MODELF_STATUS_INIT_DONE;
	} else if (!strcmp(tok, "set")) {
	/* echo set element value > /sys/devices/platform/modelffb/control */
		__modelffb_set_element(next_tok);
	} else if (!strcmp(tok, "setopt")) {
		modelffb_setopt(info, next_tok, count);
	} else if (!(parinfo->status & MODELF_STATUS_INIT_DONE)) {
		dev_err(parinfo->dev, "Not yet initialised\n");
	} else if (!strcmp(tok, "dumpmem")) {
	/* echo dumpmem address size >
	   /sys/devices/platform/modelffb/control */
		uint32_t start = 0;
		uint32_t size = 0;

		if (!__parse_uint32_2(next_tok, &start, &size)) {
			modelffb_commit_run();
			modelffb_dump_memory(start, size);
			__modelffb_queue_sleep();
		}
	} else if (!strcmp(tok, "dumpreg")) {
	/* echo dumpreg address size >
	   /sys/devices/platform/modelffb/control */
		uint32_t start = 0;
		uint32_t size = 0;

		if (__parse_uint32_2(next_tok, &start, &size) == 0) {
			modelffb_commit_run();
			modelffb_dump_register(start, size);
			__modelffb_queue_sleep();
		}
	} else if (!strcmp(tok, "writereg")) {
	/* echo writereg address data >
	   /sys/devices/platform/modelffb/control */
		uint32_t addr = 0;
		uint32_t data = 0;

		if (__parse_uint32_2(next_tok, &addr, &data) == 0) {
			modelffb_commit_run();
			modelffb_write_register(addr, data);
			__modelffb_queue_sleep();
		}
	} else if (!strcmp(tok, "cleanup")) {
	/* echo cleanup [wfmode] [x] [y] [width] [height]
		> /sys/devices/platform/modelffb/control */
		int waveform_mode = parinfo->waveform_mode;
		int x = 0;
		int y = 0;
		int width = info->var.xres;
		int height = info->var.yres;

		tok = strsep(&next_tok, " \t\n");
		if (tok) {
			if (!strcmp(tok, "whiteout"))
				waveform_mode = MODELF_WAVEFORM_WHITEOUT;
			else if (!strcmp(tok, "direct_mono"))
				waveform_mode = MODELF_WAVEFORM_DIRECT_MONO;
			else if (!strcmp(tok, "high_quality"))
				waveform_mode = MODELF_WAVEFORM_HIGH_QUALITY;
			else if (!strcmp(tok, "high_speed"))
				waveform_mode = MODELF_WAVEFORM_HIGH_SPEED;
			else
				sscanf(tok, "%d", &waveform_mode);
		}
		__parse_uint32_4(next_tok, &x, &y, &width, &height);
		__modelffb_queue_cleanup(waveform_mode, x, y, width, height);
	} else if (!strcmp(tok, "sync")) {
		tok = strsep(&next_tok, " \t\n");

		if (unlikely(!tok))
			dev_err(parinfo->dev, "No sync status provided\n");
		else
			modelffb_sync_wait(tok);
	} else if (!strcmp(tok, "power")) {
		tok = strsep(&next_tok, " \t\n");

		if (unlikely(!tok))
			dev_err(parinfo->dev, "No power status provided\n");
		else
			modelffb_sync_wait_power(tok);
	} else if (!strcmp(tok, "suspend_update")) {
	/* echo suspend_update > /sys/devices/platform/modelffb/control */
		__modelffb_queue_suspend_update(MODELF_SUSPEND_UPDATE);
	} else if (!strcmp(tok, "resume_update")) {
	/* echo resume_update > /sys/devices/platform/modelffb/control */
		__modelffb_queue_suspend_update(MODELF_RESUME_UPDATE);
	} else if (!strcmp(tok, "oneshot")) {
	/* echo oneshot [x] [y] [width] [height] >
	   /sys/devices/platform/modelffb/control */
		int x = 0;
		int y = 0;
		int width = info->var.xres;
		int height = info->var.yres;

		__parse_uint32_4(next_tok, &x, &y, &width, &height);
		__modelffb_queue_oneshot(MODELF_ONESHOT_TYPE_ONESHOT,
					 parinfo->waveform_mode,
					 x, y, width, height);
	} else {
		dev_err(parinfo->dev, "Invalid control argument\n");
	}

	return count;
}

static ssize_t __modelffb_store_keycode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%4x%4x", (unsigned int *)&parinfo->keycode1,
		(unsigned int *)&parinfo->keycode2);
	parinfo->status |= MODELF_STATUS_KEYCODE_STORED;
	return count;
}

static ssize_t modelffb_show_on_drawing(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (parinfo->power_mode == MODELF_POWER_RUN) {
		modelffb_sync_lut();
		if (__modelffb_lut_is_empty())
			strncpy(buf, "0", 255);
		else
			strncpy(buf, "1", 255);
	} else {
		strncpy(buf, "0", 255);
	}

	return 1;
}

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
static ssize_t temperature_range_show(
	struct device *dev, struct device_attribute *attr, char *buf,
	struct temperature_set* set,
	int (*format_text)(const struct temperature_entry* p_range, char *tbuf,
			   int buf_space))
{
	char *tbuf = buf;
	int written;
	const struct temperature_entry** p_range = NULL;
	int buf_space = PAGE_SIZE;

	if (!format_text)
		return -EINVAL;

	p_range = temperature_set_get_first_range(set);

	while (buf_space >= 0 && p_range != 0) {
		written = format_text(*p_range, tbuf, buf_space);

		p_range = temperature_set_get_next_range(set, p_range);

		tbuf += written;
		buf_space -= written;
	}

	return tbuf - buf;
}

static ssize_t temperature_range_store(
	struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count, struct temperature_set* set,
	void (*erase_set)(struct temperature_set* set),
	int (*read_and_store)(const char *p_range_text))

{
	int retval = -EINVAL;
	const char *p_next_range = buf;

	if (!erase_set || !read_and_store)
		return -EINVAL;

	erase_set(set);

	do {
		/* give up if invalid */
		retval = read_and_store(p_next_range);

		if (retval)
			break;

		/* locate \n at end of this range */
		p_next_range = strchr(p_next_range, '\n');

		/* then advance to the 1st char of the next range */
		if (!p_next_range || (*++p_next_range == '\0'))
			retval = strlen(buf);
	} while (p_next_range != 0 && *p_next_range != '\0');

	temperature_set_commit(set);

	return retval;
}

static ssize_t vcom_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct temperature_set *set = get_vcom_temperature_set();

	return temperature_range_show(dev, attr, buf, set,
				      temperature_set_format_integer);
}

static ssize_t vcom_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct temperature_set *set;
	ssize_t stat;

	mutex_lock(&temperature_lock);

	set = get_vcom_temperature_set();
	if (set == NULL) {
		stat = -EINVAL;
		goto exit_unlock;
	}

	stat = temperature_range_store(dev, attr, buf, count, set,
				       temperature_set_erase_ints,
				       read_and_store_vcom);
	if (stat < 0) {
		goto exit_unlock;
	} else if (stat != count) {
		/* ToDo: tidy up error handling... */
		stat = -EINVAL;
		goto exit_unlock;
	}

	stat = pl_hardware_refresh_current_vcom(parinfo->pl_hardware);
	if (stat)
		goto exit_unlock;

	stat = count;

exit_unlock:
	mutex_unlock(&temperature_lock);

	return stat;
}
#endif

static DEVICE_ATTR(init_code, S_IFREG | S_IWUSR | S_IRUSR,
	NULL, __modelffb_store_init_code);
static DEVICE_ATTR(waveform, S_IFREG | S_IWUSR | S_IRUSR,
	NULL, __modelffb_store_waveform);
static DEVICE_ATTR(control, S_IFREG | S_IWUSR | S_IRUSR,
	NULL, modelffb_store_control);
static DEVICE_ATTR(keycode, S_IFREG | S_IWUSR | S_IRUSR,
	NULL, __modelffb_store_keycode);
static DEVICE_ATTR(on_drawing, S_IFREG | S_IRUSR | S_IRGRP | S_IROTH,
	modelffb_show_on_drawing, NULL);
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
static DEVICE_ATTR(vcom, S_IRUGO | S_IWUGO, vcom_show, vcom_store);
#endif

static int modelffb_create_file(void)
{
	int retval = 0;

	retval = device_create_file(parinfo->dev, &dev_attr_init_code);
	if (retval) {
		dev_err(parinfo->dev, "Cannot create file \"init_code\"\n");
		goto end;
	}

	retval = device_create_file(parinfo->dev, &dev_attr_waveform);
	if (retval) {
		dev_err(parinfo->dev, "Cannot create file \"waveform\"\n");
		goto remove_init_code;
	}

	retval = device_create_file(parinfo->dev, &dev_attr_control);
	if (retval) {
		dev_err(parinfo->dev, "Cannot create file \"control\"\n");
		goto remove_waveform;

	}
	retval = device_create_file(parinfo->dev, &dev_attr_keycode);
	if (retval) {
		dev_err(parinfo->dev, "Cannot create file \"keycode\"\n");
		goto remove_control;
	}

	retval = device_create_file(parinfo->dev, &dev_attr_on_drawing);
	if (retval) {
		dev_err(parinfo->dev, "Cannot create file \"on_drawing\"\n");
		goto remove_keycode;
	}

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	retval = device_create_file(parinfo->dev, &dev_attr_vcom);
	if (retval) {
		dev_err(parinfo->dev, "Cannot create file \"vcom\"\n");
		goto remove_on_drawing;
	}
#endif

	return 0;

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
remove_on_drawing:
	device_remove_file(parinfo->dev, &dev_attr_on_drawing);
#endif
remove_keycode:
	device_remove_file(parinfo->dev, &dev_attr_keycode);
remove_control:
	device_remove_file(parinfo->dev, &dev_attr_control);
remove_waveform:
	device_remove_file(parinfo->dev, &dev_attr_waveform);
remove_init_code:
	device_remove_file(parinfo->dev, &dev_attr_init_code);
end:
	return retval;
}

static void modelffb_remove_file(void)
{
	device_remove_file(parinfo->dev, &dev_attr_on_drawing);
	device_remove_file(parinfo->dev, &dev_attr_keycode);
	device_remove_file(parinfo->dev, &dev_attr_control);
	device_remove_file(parinfo->dev, &dev_attr_waveform);
	device_remove_file(parinfo->dev, &dev_attr_init_code);
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	device_remove_file(parinfo->dev, &dev_attr_vcom);
#endif
}

/* =========== SPI-I2C bridge support =========== */

#if defined(CONFIG_I2C_S1D135X1) || defined(CONFIG_I2C_S1D135X1_MODULE)

static struct spi_device *modelffb_i2c_get_spi(void)
{
	if (modelffb_lock())
		return NULL;

	__modelffb_run();

	return parinfo->spi;
}

static void modelffb_i2c_put_spi(struct spi_device *spi)
{
	BUG_ON(spi != parinfo->spi);

	__modelffb_queue_sleep();
	modelffb_unlock();
}

static void modelffb_i2c_write_reg(struct spi_device *spi, u16 address,
				   u16 data)
{
	BUG_ON(spi != parinfo->spi);

	gpio_set_value(parinfo->pdata->gpio_cs, 0);
	__modelffb_immediate_reg_write(address, data);
	gpio_set_value(parinfo->pdata->gpio_cs, 1);
}

static u16 modelffb_i2c_read_reg(struct spi_device *spi, u16 address)
{
	u16 regval;

	BUG_ON(spi != parinfo->spi);

	gpio_set_value(parinfo->pdata->gpio_cs, 0);
	regval = __modelffb_reg_read(address);
	gpio_set_value(parinfo->pdata->gpio_cs, 1);

	return regval;
}

#endif /* CONFIG_I2C_S1D135X1 */

/* =========== SPI driver interface (stubs) =========== */

static int __devinit modelffb_spi_probe(struct spi_device *spi)
{
	return 0;
}

static int __devexit modelffb_spi_remove(struct spi_device *spi)
{
	return 0;
}

static void modelffb_spi_shutdown(struct spi_device *spi)
{
}

static int modelffb_spi_suspend(struct spi_device *spi, pm_message_t msg)
{
	return 0;
}

static int modelffb_spi_resume(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver modelffb_spi_driver = {
	.probe		= modelffb_spi_probe,
	.remove 	= __devexit_p(modelffb_spi_remove),
	.shutdown	= modelffb_spi_shutdown,
	.suspend	= modelffb_spi_suspend,
	.resume		= modelffb_spi_resume,
	.driver = {
		.name	= "modelffb_spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
};

/* =========== registration & unregistration =========== */

static int __devinit modelffb_setup_gpio(int gpio, const char *name)
{
	int stat;

	stat = gpio_request(gpio, name);
	if (stat) {
		dev_err(parinfo->dev, "failed to request GPIO %s %d\n",
		       name, gpio);
		return stat;
	}

	return 0;
}

static int __devinit modelffb_alloc_memory(void)
{
	int retval;

	parinfo->init_code = (uint32_t)vmalloc(MODELF_MAX_INITCODE_SIZE);
	if (!parinfo->init_code) {
		retval = -ENOMEM;
		goto exit_now;
	}

	parinfo->waveform = (uint32_t)vmalloc(MODELF_MAX_WAVEFORM_SIZE);
	if (!parinfo->waveform) {
		retval = -ENOMEM;
		goto exit_free_init_code;
	}

	parinfo->waveform_index = parinfo->waveform;
	parinfo->waveform_size = 0;
	parinfo->init_code_size = 0;

	return 0;

exit_free_init_code:
	vfree((void*)parinfo->init_code);
exit_now:
	return retval;
}

static void __devexit modelffb_free_memory(void)
{
	vfree((void*)parinfo->waveform);
	vfree((void*)parinfo->init_code);
}

static int __devinit modelffb_probe(struct platform_device *pdev)
{
	const struct modelffb_platform_data *pdata = pdev->dev.platform_data;
	int retval = 0;

#ifdef CONFIG_MODELF_DEBUG
	dev_info(parinfo->dev, "Check rev: %d\n", CHECKREV);
#endif

	retval = modelffb_framebuffer_alloc(pdev);
	if (retval) {
		dev_err(parinfo->dev, "Failed to alloc frame buffer\n");
		goto exit_now;
	}

	retval = modelffb_alloc_memory();
	if (retval) {
		dev_err(parinfo->dev, "Failed to alloc modelffb_par\n");
		goto exit_release_framebuffer;
	}
	__modelffb_bit_swap_table_init();

	retval = modelffb_create_file();
	if (retval)
		goto exit_free_memory;

	parinfo->workqueue = create_singlethread_workqueue("modelf_workqueue");
	if (!parinfo->workqueue) {
		dev_err(parinfo->dev, "Failed to create workqueue\n");
		retval = -ENOMEM;
		goto exit_free_sysfs;
	}

	if (pdata->gpio_hdc) {
		retval = modelffb_setup_gpio(pdata->gpio_hdc, "MODELF_HDC");
		if (retval)
			goto exit_destroy_workqueue;
		gpio_direction_output(pdata->gpio_hdc, 1);
		dev_info(parinfo->dev, "Using HDC on pin %d\n",
			 pdata->gpio_hdc);
	} else {
		retval = modelffb_setup_gpio(pdata->gpio_cs, "MODELF_CS");
		if (retval)
			goto exit_destroy_workqueue;
		gpio_direction_output(pdata->gpio_cs, 1);
		dev_info(parinfo->dev, "Using SPI CS on pin %d\n",
			 pdata->gpio_cs);
	}

	if (pdata->gpio_hrdy) {
		retval = modelffb_setup_gpio(pdata->gpio_hrdy, "MODELF_HRDY");
		if (retval)
			goto exit_free_gpio_hdc_cs;
		gpio_direction_input(pdata->gpio_hrdy);
		dev_info(parinfo->dev, "Using HRDY on pin %d\n",
			 pdata->gpio_hrdy);
	}

	if (pdata->gpio_pwr_en) {
		retval = modelffb_setup_gpio(pdata->gpio_pwr_en, "MODELF_PWR");
		if (retval)
			goto exit_free_gpio_hrdy;
		gpio_direction_output(pdata->gpio_pwr_en, 1);
		dev_info(parinfo->dev, "Using power enable on pin %d\n",
			 pdata->gpio_pwr_en);
	}

	if (pdata->gpio_reset) {
		retval = modelffb_setup_gpio(pdata->gpio_reset,"MODELF_RESET");
		if (retval)
			goto exit_free_gpio_pwr_en;
		gpio_direction_output(pdata->gpio_reset, 1);
		dev_info(parinfo->dev, "Using reset on pin %d\n",
			 pdata->gpio_reset);
	}

	retval = spi_register_driver(&modelffb_spi_driver);
	if (retval) {
		dev_err(parinfo->dev, "Failed to register SPI driver\n");
		goto exit_free_gpio_reset;
	}

	retval = modelffb_request_bus();
	if (retval)
		goto exit_unregister_spi_driver;

#if defined(CONFIG_I2C_S1D135X1) || defined(CONFIG_I2C_S1D135X1_MODULE)
	s1d135x1_spi.get = modelffb_i2c_get_spi;
	s1d135x1_spi.put = modelffb_i2c_put_spi;
	s1d135x1_spi.write_reg = modelffb_i2c_write_reg;
	s1d135x1_spi.read_reg = modelffb_i2c_read_reg;
	s1d135x1_spi.init_done = true;
#endif

	parinfo->status = 0;
	parinfo->waveform_mode = MODELF_WAVEFORM_HIGH_QUALITY;
	parinfo->suspend_update = MODELF_RESUME_UPDATE;
	parinfo->need_flush_image = MODELF_NO_NEED_FLUSH_IMAGE;
	parinfo->need_cleanup = MODELF_NO_NEED_CLEANUP;
	parinfo->seconds_to_sleep = 4;
	parinfo->power_mode = MODELF_POWER_STANDBY;
	parinfo->seconds_to_measure_temperature = 60;
	parinfo->send_waveform_wait = MODELF_SEND_WAVEFORM_WAIT;

	sema_init(&parinfo->access_sem, 1);
	sema_init(&parinfo->lut_in_use_sem, 1);
	sema_init(&parinfo->lut_queue_sem, 1);

	init_timer(&parinfo->sleep_timer);
	parinfo->sleep_timer.function = __modelffb_sleep_timer_handler;
	init_timer(&parinfo->temperature_timer);
	parinfo->temperature_timer.function =
		__modelffb_temperature_timer_handler;
	init_waitqueue_head(&parinfo->sync_update_wait);
	parinfo->sync_status = MODELFFB_SYNC_IDLE;

	if (!strcmp(modelffb_panel_type_modparam, "Type16")) {
		parinfo->left_border = 80;
	} else if (!strcmp(modelffb_panel_type_modparam, "Type17")) {
		parinfo->right_border = 7;
	} else if (!strcmp(modelffb_panel_type_modparam, "Type19")) {
		dev_info(parinfo->dev, "Interleaved sources enabled\n");
		parinfo->opt.interleaved_sources = 1;
	}

	if (parinfo->left_border)
		dev_info(parinfo->dev, "Left border: %d pixels\n",
			 parinfo->left_border);

	if (parinfo->right_border)
		dev_info(parinfo->dev, "Right border: %d pixels\n",
			 parinfo->right_border);

#ifdef CONFIG_MODELF_DEFERRED_IO
	dev_info(parinfo->dev, "Deferred I/O enabled\n");
#endif
	dev_info(parinfo->dev, "Ready\n");

	return 0;

exit_unregister_spi_driver:
	spi_unregister_driver(&modelffb_spi_driver);
exit_free_gpio_reset:
	if (pdata->gpio_reset)
		gpio_free(pdata->gpio_reset);
exit_free_gpio_pwr_en:
	if (pdata->gpio_pwr_en)
		gpio_free(pdata->gpio_pwr_en);
exit_free_gpio_hrdy:
	if (pdata->gpio_hrdy)
		gpio_free(pdata->gpio_hrdy);
exit_free_gpio_hdc_cs:
	if (pdata->gpio_hdc)
		gpio_free(pdata->gpio_hdc);
	else
		gpio_free(pdata->gpio_cs);
exit_destroy_workqueue:
	destroy_workqueue(parinfo->workqueue);
exit_free_sysfs:
	modelffb_remove_file();
exit_free_memory:
	modelffb_free_memory();
exit_release_framebuffer:
	modelffb_framebuffer_release();
exit_now:

	return retval;
}

static void __devexit do_clear_on_exit(void)
{
	if (parinfo->power_mode != MODELF_POWER_RUN)
		modelffb_run();

	if (modelffb_lock())
		return;

	__modelffb_fill_frame_buffer(0xff);
	__modelffb_simple_command(
		MODELF_COM_INIT_UPDATE_BUFFER);
	__modelffb_simple_command(
		MODELF_COM_WAIT_TRIGGER_DONE);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	modelffb_unlock();

	modelffb_cleanup_full(MODELF_WAVEFORM_WHITEOUT);
	modelffb_sleep();
}

static int __devexit modelffb_remove(struct platform_device *pdev)
{
#if defined(CONFIG_I2C_S1D135X1) || defined(CONFIG_I2C_S1D135X1_MODULE)
	s1d135x1_spi.init_done = false;
#endif

	del_timer_sync(&parinfo->sleep_timer);
	del_timer_sync(&parinfo->temperature_timer);

	if (parinfo->status & MODELF_STATUS_INIT_DONE) {
		disable_irq_nosync(parinfo->pdata->hirq);
		free_irq(parinfo->pdata->hirq, parinfo->dev);

		if (parinfo->opt.clear_on_exit)
			do_clear_on_exit();

		modelffb_unregister_framebuffer();
		modelffb_free_vram();
	}

	modelffb_regulator_exit();
	modelffb_release_bus();
	spi_unregister_driver(&modelffb_spi_driver);

	if (parinfo->pdata->gpio_reset)
		gpio_free(parinfo->pdata->gpio_reset);

	if (parinfo->pdata->gpio_pwr_en)
		gpio_free(parinfo->pdata->gpio_pwr_en);

	if (parinfo->pdata->gpio_hrdy)
		gpio_free(parinfo->pdata->gpio_hrdy);

	if (parinfo->pdata->gpio_hdc)
		gpio_free(parinfo->pdata->gpio_hdc);
	else
		gpio_free(parinfo->pdata->gpio_cs);

	flush_workqueue(parinfo->workqueue);
	destroy_workqueue(parinfo->workqueue);
	modelffb_remove_file();
	modelffb_free_memory();
	modelffb_framebuffer_release();

	dev_info(parinfo->dev, "modelF has been removed.\n");

	return 0;
}

static void modelffb_shutdown(struct platform_device *pdev)
{
	modelffb_remove(pdev);
}

static int modelffb_suspend(struct platform_device *pdev, pm_message_t msg)
{
	del_timer_sync(&parinfo->sleep_timer);
	modelffb_commit_sleep();

	return 0;
}

static int modelffb_resume(struct platform_device *pdev)
{
	modelffb_commit_run();

	return 0;
}

static struct platform_driver modelffb_driver = {
	.probe		= modelffb_probe,
	.remove 	= __devexit_p(modelffb_remove),
	.shutdown	= modelffb_shutdown,
	.suspend	= modelffb_suspend,
	.resume		= modelffb_resume,
	.driver		= {
		.name	= "modelffb",
		.bus	= &platform_bus_type,
		.owner	= THIS_MODULE,
	},
};

static int __init __modelffb_init(void)
{
	return platform_driver_register(&modelffb_driver);
}
module_init(__modelffb_init);

static void __exit __modelffb_exit(void)
{
	platform_driver_unregister(&modelffb_driver);
}
module_exit(__modelffb_exit);

MODULE_DESCRIPTION("fbdev driver for modelF EPD controller");
MODULE_VERSION("2.0");
MODULE_LICENSE("GPL");

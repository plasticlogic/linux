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
#include <linux/spi/spidev.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/spi/spi.h>
#include <linux/timer.h>

#include <plat/gpmc.h>
#include <mach/board-am335xevm.h>
#include <mach/irqs.h>

#include "modelffb.h"

#define CHECKREV 110

/* Set to 1 to print the time it takes to send the image data */
#define TIME_SEND_IMAGE 0

/* Set to 1 to use the MT PROM */
#define USE_MTP 0

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
#include <linux/mutex.h>
#include "pl_hardware.h"
#include "vcom.h"
#include "temperature-set.h"

/* ToDo: define in board file */
struct pl_hardware_config pl_config = {
#ifdef CONFIG_MODELF_PL_ROBIN
	.i2c_bus_number = 4,
#else
	.i2c_bus_number = 3,
#endif
	.dac_i2c_address = 0x39,
	.adc_i2c_address = 0x34,
};

static DEFINE_MUTEX(temperature_lock);

#endif

#ifdef CONFIG_MODELF_CONNECTION_ASYNC
#define MODELF_CONNECTION "asynchronous bus"
#elif CONFIG_MODELF_CONNECTION_SPI
#define MODELF_CONNECTION "SPI bus"
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
		printk(KERN_ERR "MODELFFB: down semaphore interrupted\n");
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

/* =========== asynchronous bus specific functions =========== */
#ifdef CONFIG_MODELF_CONNECTION_ASYNC
static int __devinit __modelffb_request_asyncbus(void)
{
	int retval = 0;
	unsigned long modelfaddr;

	retval = gpmc_cs_request(MODELF_CS, MODELF_IO_SIZE, &modelfaddr);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: request gpmc failed\n");
		retval = -EIO;
		goto end;
	}

	parinfo->command_addr = (uint32_t)ioremap(modelfaddr, MODELF_IO_SIZE);
	if (!parinfo->command_addr) {
		printk(KERN_ERR "MODELFFB: map I/O address failed\n");
		retval = -EIO;
		goto end;
	}
	parinfo->data_addr = parinfo->command_addr + MODELF_DATA_ADDR_OFFSET;

#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: command address = 0x%08x(0x%08x)\n",
		parinfo->command_addr, (uint32_t)modelfaddr);
	printk(KERN_INFO "MODELFFB:    data address = 0x%08x(0x%08x)\n",
		parinfo->data_addr, (uint32_t)modelfaddr + MODELF_DATA_ADDR_OFFSET);
#endif

end:
	return retval;
}

static void __devexit __modelffb_release_asyncbus(void)
{
	iounmap((void*)parinfo->command_addr);
	gpmc_cs_free(MODELF_CS);
}

static int __devinit __modelffb_request_bus(void)
{
	int retval = 0;

	retval = __modelffb_request_asyncbus();
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: setup async bus faild\n");
	}

	return retval;
}

static void __devexit __modelffb_release_bus(void)
{
	__modelffb_release_asyncbus();
}

static void __modelffb_write_command_async(uint16_t command)
{
	*(uint16_t*)parinfo->command_addr = command;
}

static void __modelffb_write_data_async(uint16_t data)
{
	*(uint16_t*)parinfo->data_addr = data;
}

static void __modelffb_write_n_data_async(uint16_t *data, size_t n)
{
	int i;

	i = 0;
	while ((i + 1) * MODELF_DATA_SIZE < n) {
		memcpy((void*)parinfo->data_addr,
			(uint16_t*)data + i * MODELF_DATA_SIZE / 2, MODELF_DATA_SIZE);
		i++;
	}

	memcpy((void*)parinfo->data_addr,
		(uint16_t*)data + i * MODELF_DATA_SIZE / 2,
		((n - i * MODELF_DATA_SIZE) / 4) * 4);

	if (n % 4 == 2) /* out of 32-bit alignment */
		*(uint16_t*)parinfo->data_addr = *((uint16_t*)data + n / 2 - 1);
}

static uint16_t __modelffb_read_data_async(void)
{
	return *(uint16_t*)parinfo->data_addr;
}
#endif /* CONFIG_MODELF_CONNECTION_ASYNC */

/* =========== SPI bus specific functions =========== */
#ifdef CONFIG_MODELF_CONNECTION_SPI

#define SWAP_BYTE_16(x) ((x >> 8) | (x << 8))

static int __devinit __modelffb_request_bus(void)
{
	return 0;
}

static void __devexit __modelffb_release_bus(void)
{
}

static void my_spi_write(struct spi_device *spi, void *buffer, size_t bytes)
{
	int error;
	struct spi_message m;
        struct spi_transfer t[1];

        spi_message_init(&m);
        memset(t, 0, sizeof(t));

        t[0].tx_buf = buffer;
        t[0].len = bytes;
        t[0].bits_per_word = 16;
        spi_message_add_tail(&t[0], &m);

        error = spi_sync(spi, &m);
        if (error < 0)
                printk(KERN_ERR "MODELFFB: SPI write error: %d\n", error);
}

static void my_spi_read(struct spi_device *spi, void *buffer, size_t bytes)
{
	int error;
	struct spi_message m;
        struct spi_transfer t[1];

        spi_message_init(&m);
        memset(t, 0, sizeof(t));

        t[0].rx_buf = buffer;
        t[0].len = bytes;
        t[0].bits_per_word = 16;
        spi_message_add_tail(&t[0], &m);

        error = spi_sync(spi, &m);
        if (error < 0)
                printk(KERN_ERR "MODELFFB: SPI read error: %d\n", error);
}

static void __modelffb_write_command_spi(uint16_t command)
{
#ifdef CONFIG_MODELF_SWAP_SPI_BYTE
	command = SWAP_BYTE_16(command);
#endif

#ifdef CONFIG_MODELF_SPI_WITHOUT_HDC
	gpio_set_value(MODELF_SPIHCS_GPIO, 0);
	my_spi_write(parinfo->spi, &command, 2);
#else
	gpio_set_value(MODELF_SPIHDC_GPIO, 0);
	my_spi_write(parinfo->spi, &command, 2);
	gpio_set_value(MODELF_SPIHDC_GPIO, 1);
#endif
}

static void __modelffb_write_data_spi(uint16_t data)
{
#ifdef CONFIG_MODELF_SWAP_SPI_BYTE
	uint16_t buffer = SWAP_BYTE_16(data);
	my_spi_write(parinfo->spi, &buffer, 2);
#else
	my_spi_write(parinfo->spi, &data, 2);
#endif
}

static void __modelffb_write_n_data_spi(uint16_t *data, size_t n)
{
	int i;
#ifdef CONFIG_MODELF_SWAP_SPI_BYTE
	uint16_t *buffer = kmalloc(n + 2, GFP_KERNEL);

	if (!buffer) {
		printk(KERN_ERR "MODELFFB: kmalloc swap buffer failed\n");
		return;
	}

/* Data size over NON_DMA_MAX_BYTES will automativally be sent via DMA.
 * It is necessary to allocate coherent memory for using SPI with DMA.
 * But this area is out of sight from deferred I/O because of page fault mechanism.
 * Pooling sent data to coherent area might be one solution.
 */
	for (i = 0; i < n / 2 + 1; i++) { /* + 1 for out of 16-bit alignment */
		*(buffer + i) = SWAP_BYTE_16(*(data + i));
	}

	for (i = 0; i < n / 2 + 1; i += NON_DMA_MAX_BYTES / 2) {
		my_spi_write(parinfo->spi, buffer + i,
			n - i * 2 < NON_DMA_MAX_BYTES ? n % NON_DMA_MAX_BYTES : NON_DMA_MAX_BYTES);
	}

	kfree(buffer);
#else
	for (i = 0; i < n / 2 + 1; i += NON_DMA_MAX_BYTES / 2) {
		my_spi_write(parinfo->spi, data + i,
			n - i * 2 < NON_DMA_MAX_BYTES ? n % NON_DMA_MAX_BYTES : NON_DMA_MAX_BYTES);
	}
#endif
}

static uint16_t __modelffb_read_data_spi(void)
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
#endif /* CONFIG_MODELF_CONNECTION_SPI */

/* =========== access functions ===========
 * NOTE! Do not use __xx functions without semaphore.
 * Disordered command/data may cause problem.
 */

static int __modelffb_write_data(uint16_t data)
{
#ifdef CONFIG_MODELF_CONNECTION_ASYNC
	__modelffb_write_data_async(data);
#elif CONFIG_MODELF_CONNECTION_SPI
	__modelffb_write_data_spi(data);
#endif
	return 0;
}

static int __modelffb_write_n_data(uint16_t *data, size_t n)
{
#ifdef CONFIG_MODELF_CONNECTION_ASYNC
	__modelffb_write_n_data_async(data, n);
#elif CONFIG_MODELF_CONNECTION_SPI
	__modelffb_write_n_data_spi(data, n);
#endif
	return 0;
}

static uint16_t __modelffb_read_data(void)
{
#ifdef CONFIG_MODELF_CONNECTION_ASYNC
	return __modelffb_read_data_async();
#elif CONFIG_MODELF_CONNECTION_SPI
	return __modelffb_read_data_spi();
#endif
}

#ifdef CONFIG_MODELF_CONNECTION_SPI
static uint16_t __modelffb_read_dummy_data(void)
{
	return __modelffb_read_data_spi();
}
#endif

static int __modelffb_immediate_command(uint16_t command)
{
#ifdef CONFIG_MODELF_CONNECTION_ASYNC
	__modelffb_write_command_async(command);
#elif CONFIG_MODELF_CONNECTION_SPI
	__modelffb_write_command_spi(command);
#endif
	return 0;
}

static int __modelffb_command_end(void)
{
#ifdef CONFIG_MODELF_SPI_WITHOUT_HDC
	gpio_set_value(MODELF_SPIHCS_GPIO, 1);
#endif
	return 0;
}

static int __modelffb_immediate_simple_command(uint16_t command)
{
	__modelffb_immediate_command(command);
	__modelffb_command_end();
	return 0;
}

static uint16_t __modelffb_reg_read(uint16_t address)
{
	uint16_t readval;

	__modelffb_immediate_command(MODELF_COM_READ_REG);
	__modelffb_write_data(address);
#ifdef CONFIG_MODELF_CONNECTION_SPI
	__modelffb_read_dummy_data();
#endif
	readval = __modelffb_read_data();
	__modelffb_command_end();

	return readval;
}

static inline int __modelffb_check_HRDY_ready(void)
{
#ifdef CONFIG_MODELF_CONNECTION_SPI
	if ((__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS)
	     & MODELF_BF_INT_HRDY_STATUS))
		return 1;
#else
	if (gpio_get_value(MODELF_HRDY_GPIO) == 1)
		return 1;
#endif
	return 0;
}

static inline int __modelffb_wait_for_HRDY_ready(int ms_timeout)
{
	uint32_t start_jiffy = jiffies;
	int i;

	/* sync GPMC access and gpio peep timing */
	if ((__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS) & MODELF_BF_INT_HRDY_STATUS) != 0)
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
	printk(KERN_ERR "MODELFFB: HRDY timeout, MODELF_REG_SYSTEM_STATUS = 0x%04x, MODELF_REG_POWER_SAVE_MODE = 0x%04X\n",
		__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS),
		__modelffb_reg_read(MODELF_REG_POWER_SAVE_MODE)); 
	return -EBUSY;

success:
	__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS);
	return 0;
}

static int __modelffb_delay_for_HRDY_ready(int ms_timeout)
{
	uint32_t start_jiffy = jiffies;

	/* sync GPMC access and gpio peep timing */
	if ((__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS) & MODELF_BF_INT_HRDY_STATUS) != 0)
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
	printk(KERN_ERR "MODELFFB: HRDY timeout, MODELF_REG_SYSTEM_STATUS = 0x%04x, MODELF_REG_POWER_SAVE_MODE = 0x%04X\n",
		__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS),
		__modelffb_reg_read(MODELF_REG_POWER_SAVE_MODE)); 
	return -EBUSY;

success:
	__modelffb_reg_read(MODELF_REG_SYSTEM_STATUS);
	return 0;
}

static int __modelffb_command(uint16_t command)
{
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	return __modelffb_immediate_command(command);
}

static inline void __modelffb_command_p1(uint16_t command, uint16_t param1)
{
	__modelffb_command(command);
	__modelffb_write_data(param1);
}

static inline void __modelffb_command_p2(uint16_t command, uint16_t param1, uint16_t param2)
{
	__modelffb_command(command);
	__modelffb_write_data(param1);
	__modelffb_write_data(param2);
}

static inline void __modelffb_command_p3(uint16_t command, uint16_t param1, uint16_t param2,
	uint16_t param3)
{
	__modelffb_command(command);
	__modelffb_write_data(param1);
	__modelffb_write_data(param2);
	__modelffb_write_data(param3);
}

static inline void __modelffb_command_p4(uint16_t command, uint16_t param1, uint16_t param2,
	uint16_t param3, uint16_t param4)
{
	__modelffb_command(command);
	__modelffb_write_data(param1);
	__modelffb_write_data(param2);
	__modelffb_write_data(param3);
	__modelffb_write_data(param4);
}

static inline void __modelffb_command_p5(uint16_t command, uint16_t param1, uint16_t param2,
	uint16_t param3, uint16_t param4, uint16_t param5)
{
	__modelffb_command(command);
	__modelffb_write_data(param1);
	__modelffb_write_data(param2);
	__modelffb_write_data(param3);
	__modelffb_write_data(param4);
	__modelffb_write_data(param5);
}

static int __modelffb_simple_command(uint16_t command)
{
	__modelffb_command(command);
	__modelffb_command_end();
	return 0;
}
static inline void __modelffb_simple_command_p1(uint16_t command, uint16_t param1)
{
	__modelffb_command_p1(command, param1);
	__modelffb_command_end();
}

static inline void __modelffb_simple_command_p2(uint16_t command, uint16_t param1, uint16_t param2)
{
	__modelffb_command_p2(command, param1, param2);
	__modelffb_command_end();
}

static inline void __modelffb_simple_command_p3(uint16_t command, uint16_t param1, uint16_t param2,
	uint16_t param3)
{
	__modelffb_command_p3(command, param1, param2, param3);
	__modelffb_command_end();
}

static inline void __modelffb_simple_command_p4(uint16_t command, uint16_t param1, uint16_t param2,
	uint16_t param3, uint16_t param4)
{
	__modelffb_command_p4(command, param1, param2, param3, param4);
	__modelffb_command_end();
}

static inline void __modelffb_simple_command_p5(uint16_t command, uint16_t param1, uint16_t param2,
	uint16_t param3, uint16_t param4, uint16_t param5)
{
	__modelffb_command_p5(command, param1, param2, param3, param4, param5);
	__modelffb_command_end();
}

static inline int __modelffb_wait_for_reg_value(uint16_t reg, uint16_t mask,
	uint16_t value, int ms_timeout)
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
	printk(KERN_ERR "MODELFFB: read reg %04x timeouted for 0x%04x\n", reg, value);
	return -EBUSY;

success:
	return 0;
}

static void __modelffb_reg_write(uint16_t address, uint16_t data)
{
	__modelffb_simple_command_p2(MODELF_COM_WRITE_REG, address, data);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
}

static void __modelffb_immediate_reg_write(uint16_t address, uint16_t data)
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
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: failed to send data\n");
	}

	return retval;
}

#ifdef CONFIG_MODELF_CONNECTION_ASYNC
static int __modelffb_data_transfer_wait(uint16_t *data, size_t n)
{
	int retval;
	int i;

	__modelffb_command_p1(MODELF_COM_WRITE_REG, MODELF_DATA_PORT);
	for (i = 0; i < ((n + 3) / 4) * 4; i += 2) {
		*(uint16_t*)parinfo->data_addr = *((uint16_t*)data + i / 2);
		/* Wait 5us for FIFO empty */
		/* The 64 clocks is 4us and 5us is used for safety margin */
		udelay(5);		
	}
	__modelffb_command_end();
	__modelffb_immediate_simple_command(MODELF_COM_END_OF_RAW_ACCESS);

	retval = __modelffb_delay_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: failed to send data\n");
	}

	return retval;
}
#endif

/* =========== Synchronous update support =========== */

static inline void modelffb_sync_set_status(enum modelffb_sync_status status)
{
	parinfo->sync_status = status;
	wake_up_interruptible(&parinfo->sync_update_wait);
}

static inline void modelffb_wait_sync(const char *status_str)
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
		printk(KERN_ERR "Invalid sync status: %s\n", status_str);
		return;
	}

	wait_event_interruptible(parinfo->sync_update_wait,
				 (parinfo->sync_status == status));
}

/* =========== model F operations =========== */

#ifdef CONFIG_MODELF_DEBUG
static void __modelffb_print_reg(uint16_t reg)
{
	printk(KERN_INFO "MODELFFB: register 0x%04x = 0x%04x\n", reg, __modelffb_reg_read(reg));
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

	printk(KERN_INFO "MODELFFB: dumping 0x%08x bytes memory from 0x%08x",
		size, modelf_addr);

	if (modelffb_lock())
		return;

	__modelffb_simple_command_p4(MODELF_COM_RAW_READ,
		modelf_addr & 0xffff, (modelf_addr >> 16) & 0xffff,
		(size / 2) & 0xffff, ((size / 2) >> 16) & 0xffff);

	__modelffb_command_p1(MODELF_COM_READ_REG, MODELF_DATA_PORT);
#ifdef CONFIG_MODELF_CONNECTION_SPI
	__modelffb_read_dummy_data();
#endif
	for (i = 0; i < size / 2; i++) {
		if (i % 8 == 0)
			printk("\n%08x: ", modelf_addr + i * 2);
		printk("%04x", __modelffb_read_data());
		if (i % 8 != 7)
			printk(" ");
	}
	printk("\n");
	__modelffb_command_end();
	__modelffb_simple_command(MODELF_COM_END_OF_RAW_ACCESS);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	modelffb_unlock();
}

static inline void modelffb_dump_register(uint16_t modelf_addr, size_t size)
{
	int i;

	printk(KERN_INFO "MODELFFB: dumping %d registers from 0x%04x",
	       size, modelf_addr);

	if (modelffb_lock())
		return;

	for (i = 0; i < size; i++) {
		if (i % 8 == 0)
			printk("\nreg 0x%04x: ", modelf_addr + i * 2);
		printk("%04x", __modelffb_reg_read(modelf_addr + i * 2));
		if (i % 8 != 7)
			printk(" ");
	}
	printk("\n");

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

static inline void modelffb_write_register(uint16_t modelf_addr, uint16_t modelf_data)
{
	printk(KERN_INFO "MODELFFB: write register 0x%04x = 0x%04x", modelf_addr, modelf_data);

	if (modelffb_lock())
		return;
	__modelffb_reg_write(modelf_addr, modelf_data);
	modelffb_unlock();
}

static int __modelffb_send_init_code(void)
{
	int readval;

	__modelffb_command(MODELF_COM_INIT);
	__modelffb_write_n_data((uint16_t*)parinfo->init_code, parinfo->init_code_size);
	__modelffb_command_end();
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	readval = __modelffb_reg_read(MODELF_REG_INIT_CODE_CHECKSUM);
	if ((readval & MODELF_BF_INIT_CODE_CHECKSUM) == MODELF_INIT_CODE_CHECKSUM_ERROR) {
		printk(KERN_ERR "MODELFFB: init code checksum error!\n");
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

#ifdef CONFIG_MODELF_CONNECTION_ASYNC
	if (parinfo->send_waveform_wait == MODELF_SEND_WAVEFORM_WAIT) {
		__modelffb_data_transfer_wait((uint16_t*)parinfo->waveform, parinfo->waveform_size);
	}
	else {
		__modelffb_data_transfer((uint16_t*)parinfo->waveform, parinfo->waveform_size);
	}
#elif CONFIG_MODELF_CONNECTION_SPI
	__modelffb_data_transfer((uint16_t*)parinfo->waveform, parinfo->waveform_size);
#endif

	retval = __modelffb_delay_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: failed to send waveform!\n");
		goto end;
	}

	readval = __modelffb_reg_read(MODELF_REG_DSPE_INT_STATUS);

	if ((readval & MODELF_INT_WF_INVALID_FORMAT) != 0) {
		printk(KERN_ERR "MODELFFB: invalid waveform format!\n");
		retval = -EIO;
	}
	if ((readval & MODELF_INT_WF_CHECKSUM_ERROR) != 0) {
		printk(KERN_ERR "MODELFFB: waveform checksum error!\n");
		retval = -EIO;
	}
	if ((readval & MODELF_INT_WF_OVERFLOW) != 0) {
		printk(KERN_ERR "MODELFFB: waveform overflow!\n");
		retval = -EIO;
	}

end:
	return retval;
}

#if USE_MTP
static int __modelffb_set_mtp_vcom_internal(int ms_timeout)
{
	int retval = 0;

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_ALL_STATE_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	__modelffb_reg_write(MODELF_REG_MTP_ADRDATA, MODELF_MTP_VCOM_ADDR << 4);
	__modelffb_reg_write(MODELF_REG_MTP_CONTROL, MODELF_VCOM_READ_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_OPERATION_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	__modelffb_reg_write(MODELF_REG_MTP_ADRDATA, (MODELF_MTP_VCOM_ADDR +1) << 4);
	__modelffb_reg_write(MODELF_REG_MTP_CONTROL, MODELF_VCOM_READ_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_OPERATION_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	__modelffb_reg_write(MODELF_REG_MTP_CONTROL, MODELF_MTP_READ_STOP_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_MODE_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: mtp vcom value has set to internal register\n");
#endif
	return 0;

err:
	printk(KERN_INFO "MODELFFB: failed to read mtp vcom value\n");
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
	__modelffb_reg_write(MODELF_REG_MTP_CONTROL, MODELF_MTP_READ_START_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_OPERATION_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	mtp_vcom = __modelffb_reg_read(MODELF_REG_MTP_READ_DATA) << 4;

	__modelffb_reg_write(MODELF_REG_MTP_ADRDATA, (MODELF_MTP_VCOM_ADDR +1) << 8);
	__modelffb_reg_write(MODELF_REG_MTP_CONTROL, MODELF_MTP_READ_START_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_OPERATION_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	mtp_vcom |= __modelffb_reg_read(MODELF_REG_MTP_READ_DATA);

	__modelffb_reg_write(MODELF_REG_MTP_CONTROL, MODELF_MTP_READ_STOP_TRIGGER);

	retval = __modelffb_wait_for_reg_value(MODELF_REG_MTP_STATUS,
		MODELF_MTP_READ_MODE_BUSY, 0, MODELF_MTP_TIMEOUT_MS);
	if (retval != 0)
		goto err;

	__modelffb_reg_write(MODELF_REG_ALU_TEMPORARY_0, mtp_vcom);

#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: mtp vcom value 0x%02x has set to ALU Temporary Register 0\n",
		mtp_vcom);
#endif
	return 0;

err:
	printk(KERN_INFO "MODELFFB: failed to read mtp vcom value\n");
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
	printk(KERN_INFO "MODELFFB: failed to set mtp vcom value\n");
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

#define AM33XX_CONTROL_PADCONF_SPI0_D0_OFFSET                   0x0954
#define AM33XX_CONTROL_PADCONF_SPI0_D1_OFFSET                   0x0958

static int modelffb_chip_init(void)
{
	uint16_t readval;
	int retval = 0;

	retval = modelffb_lock();
	if (retval)
		goto err;

	__modelffb_reset();

	readval = __modelffb_reg_read(MODELF_REG_PRODUCT_CODE);
	printk(KERN_INFO "MODELFFB: product code = 0x%04x\n", readval);
	if (readval != MODELF_PRODUCT_CODE) {
		printk(KERN_ERR "MODELFFB: invalid product code!!\n");
		retval = -EIO;
		goto up_sem;
	}

	__modelffb_reg_write(MODELF_REG_CLOCK_CONFIGURATION,
		MODELF_INTERNAL_CLOCK_ENABLE);
	msleep(10);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: clock enable failed\n");
		goto up_sem;
	}

#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_ERR "MODELFFB: calling __modelffb_send_init_code()\n");
#endif
	retval = __modelffb_send_init_code();
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: failed to send init code\n");
		goto up_sem;
	}

#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_ERR "MODELFFB: calling __modelffb_simple_command(MODELF_COM_INIT_THEN_STANDBY)\n");
#endif
	__modelffb_simple_command(MODELF_COM_INIT_THEN_STANDBY);
	msleep(100);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: init and standby failed\n");
		goto up_sem;
	}

	if (((__modelffb_reg_read(MODELF_REG_WAVEFORM_DEC_BYPASS)
			& MODELF_WAVEFORM_DECORDER_SELECT) != 0) && 
			((parinfo->status & MODELF_STATUS_KEYCODE_STORED) == 0)) {
		printk(KERN_ERR "MODELFFB: keycode not stored\n");
		retval = -EIO;
		goto up_sem;
	}

	__modelffb_reg_write(MODELF_REG_PROTECTION_KEY_1, parinfo->keycode1);
	__modelffb_reg_write(MODELF_REG_PROTECTION_KEY_2, parinfo->keycode2);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: write keycode failed\n");
		goto up_sem;
	}

	readval = __modelffb_reg_read(MODELF_REG_UPDATE_BUFFER_CONF);
	__modelffb_reg_write(MODELF_REG_UPDATE_BUFFER_CONF, readval & ~MODELF_LUT_AUTO_SELECT);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: set update buffer config failed\n");
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

	__modelffb_reg_write(MODELF_REG_INTERRUPT_CONTROL, MODELF_INT_DISPLAY_ENGINE);
	__modelffb_reg_write(MODELF_REG_DSPE_INT_ENABLE, MODELF_INT_DSPE_ONE_LUT_FREE);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: set interrupt control failed\n");
		goto up_sem;
	}

	__modelffb_immediate_reg_write(MODELF_REG_POWER_SAVE_MODE, 
		(__modelffb_reg_read(MODELF_REG_POWER_SAVE_MODE) & ~MODELF_POWER_ACTIVE)
		| MODELF_POWER_ACTIVE);
	__modelffb_simple_command(MODELF_COM_RUN);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: set into run mode faild\n");
		goto up_sem;
	}
	parinfo->power_mode = MODELF_POWER_RUN;

	__modelffb_simple_command(MODELF_COM_CLEAR_GATE_DRIVER);
	__modelffb_simple_command(MODELF_COM_WAIT_TRIGGER_DONE);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: clear gate driver failed\n");
		goto up_sem;
	}

#ifdef CONFIG_MODELF_PL_ROBIN /* vertical mirror to reverse source data */
	__modelffb_simple_command_p1(MODELF_COM_ROTATE, 0x0400);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (retval) {
		printk(KERN_ERR "MODELFFB: rotate command failed\n");
		goto up_sem;
	}
#endif

	modelffb_unlock();
#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: init done\n");
#endif
	return 0;

up_sem:
	modelffb_unlock();
err:
	printk(KERN_INFO "MODELFFB: chip_init failed\n");
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

static void __modelffb_pool_reed_shaped_1bit_image(int x, int y, int width)
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
static inline uint8_t __modelffb_8bit_desaturete(uint16_t color)
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

static void __modelffb_pool_reed_shaped_16bit_image(int x, int y, int width)
{
	int i;
	struct fb_info *info = parinfo->fbinfo;
	uint32_t start = (uint32_t)info->screen_base +
		(uint32_t)info->fix.line_length * y + x * 2;

	for (i = x; i < (x + width); i++) {
		*(uint8_t*)(parinfo->image_pool + (i - x)) =
			__modelffb_8bit_desaturete(*(uint16_t*)(start + (i - x) * 2));
	}
}

static int __modelffb_send_image(int x, int y, int width, int height)
{
	int retval = 0;
	struct fb_info *info = parinfo->fbinfo;
	int line;
#if (defined(CONFIG_MODELF_DEBUG) || TIME_SEND_IMAGE)
	uint32_t start_jiffy = jiffies;
	uint32_t delta_jiffy;
#endif
	switch (info->var.bits_per_pixel) {
	case 1:
		for (line = y; line < y + height; line++) {
			__modelffb_pool_reed_shaped_1bit_image(x, line, width);
			__modelffb_command_p5(MODELF_COM_LOAD_IMAGE_AREA,
				1, x & 0x1ff, line & 0x3ff, width & 0x1ff, 1 & 0x3ff);
			__modelffb_data_transfer((uint16_t*)parinfo->image_pool,
				((width + 15) / 16) * 2); /* 16-bit access */
		}
		break;
	case 8:
		for (line = y; line < y + height; line++) {
			__modelffb_command_p5(MODELF_COM_LOAD_IMAGE_AREA, MODELF_BPP_8,
				x & 0x1ff, line & 0x3ff, width & 0x1ff, 1 & 0x3ff);
			__modelffb_data_transfer(
				(uint16_t*)((uint32_t)info->screen_base +
				(uint32_t)info->fix.line_length * line + x),
				((width + 1) / 2) * 2); /* 16-bit access */
		}
		break;
	case 16:
		for (line = y; line < y + height; line++) {
			__modelffb_pool_reed_shaped_16bit_image(x, line, width);
			__modelffb_command_p5(MODELF_COM_LOAD_IMAGE_AREA, MODELF_BPP_8,
				x & 0x1ff, line & 0x3ff, width & 0x1ff, 1 & 0x3ff);
			__modelffb_data_transfer((uint16_t*)parinfo->image_pool,
				((width + 1) / 2) * 2); /* 16-bit access */
		}
		break;
	default:
		break;
	}

#if (defined(CONFIG_MODELF_DEBUG) || TIME_SEND_IMAGE)
	delta_jiffy = jiffies - start_jiffy;
	printk(KERN_INFO "MODELFFB: image (%d, %d) - (%d, %d) sent in %dms\n",
	       x, y, (x + width - 1), (y + height - 1),
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

	memset(info->screen_base, to_be_filled_color, (info->var.xres * info->var.yres * 2));
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

	if (!retval)
		modelffb_sync_set_status(MODELFFB_SYNC_BUSY);

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
	int retval = 0;

	__modelffb_simple_command_p1(MODELF_COM_UPDATE_FULL, MODELF_WAVEFORM_MODE(waveform_mode));
	__modelffb_simple_command(MODELF_COM_WAIT_TRIGGER_DONE);
	__modelffb_simple_command(MODELF_COM_WAIT_FRAME_END);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	if (!retval)
		modelffb_sync_set_status(MODELFFB_SYNC_BUSY);

	return retval;
}

static int modelffb_cleanup_full(int waveform_mode)
{
	int retval = 0;

#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: cleanup full in effect\n");
#endif

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
        retval = pl_hardware_enable(parinfo->pl_hardware);
        if (retval) {
                dev_err(parinfo->dev, "Failed to enable PL hardware\n");
                goto err_exit;
        }
#endif

	retval = modelffb_lock();
	if (retval)
		goto err_power_off;

	retval = __modelffb_cleanup_full(waveform_mode);

	modelffb_unlock();

err_power_off:
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
        pl_hardware_disable(parinfo->pl_hardware);
err_exit:
#endif
	retval = 0; /* ToDo: really? */
	return retval;
}

static int __modelffb_update_full(int waveform_mode)
{
	int retval = 0;

	__modelffb_simple_command_p1(MODELF_COM_PARTIAL_UPDATE_FULL,
		MODELF_WAVEFORM_MODE(waveform_mode));
	__modelffb_simple_command(MODELF_COM_WAIT_TRIGGER_DONE);
	__modelffb_simple_command(MODELF_COM_WAIT_FRAME_END);
	retval = __modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	if (!retval)
		modelffb_sync_set_status(MODELFFB_SYNC_BUSY);

	return retval;
}

static int modelffb_update_full(int waveform_mode)
{
	int retval = 0;

#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: update full in effect\n");
#endif

        /* Enable power to the EPD panel */
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
        retval = pl_hardware_enable(parinfo->pl_hardware);
        if (retval) {
                dev_err(parinfo->dev, "Failed to enable PL hardware\n");
                goto err_exit;
        }
#endif

	retval = modelffb_lock();
	if (retval)
		goto err_power_off;

	retval = __modelffb_update_full(waveform_mode);

	modelffb_unlock();

err_power_off:
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
        pl_hardware_disable(parinfo->pl_hardware);
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

static int __modelffb_rects_overwrap(int x1, int y1, int width1, int height1,
	int x2, int y2, int width2, int height2)
{
	return x2 < x1 + width1 && x2 + width2 > x1 && y2 < y1 + height1 && y2 + height2 > y1;
}

static int __modelffb_rect_no_overwrap(int x, int y, int width, int height)
{
	int i;

	for (i = 0; i < MODELF_MAX_LUT_NUM; i++) {
		if (parinfo->lut_in_use[i].busy == 1 &&
				__modelffb_rects_overwrap(x, y, width, height,
				parinfo->lut_in_use[i].x, parinfo->lut_in_use[i].y,
				parinfo->lut_in_use[i].width, parinfo->lut_in_use[i].height)) {
			return 0;
		}
	}

	return 1;
}

#ifdef CONFIG_MODELF_DEBUG
static void __modelffb_print_lut(void)
{
	int i;
	printk("MODELFFB: LUT info\n");
	for (i = 0; i < MODELF_MAX_LUT_NUM; i++) {
		printk("  %d: ", i);
		if (parinfo->lut_in_use[i].busy == 1) {
			printk("(%d, %d) - (%d, %d)\n",
			parinfo->lut_in_use[i].x,
			parinfo->lut_in_use[i].y,
			parinfo->lut_in_use[i].x + parinfo->lut_in_use[i].width - 1,
			parinfo->lut_in_use[i].y + parinfo->lut_in_use[i].height - 1);
		}
		else {
			printk("\n");
		}
	}
}
#endif

static int __modelffb_request_lut(int x, int y, int width, int height)
{
	int retval = 0;
	int lut_id;

	if (!__modelffb_rect_no_overwrap(x, y, width, height)) {
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: rectangle overwrapped\n");
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
	}
	else {
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: no free lut\n");
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

	next = (parinfo->lut_queue_head + parinfo->lut_queue_count) % MODELF_MAX_LUT_QUEUE_NUM;

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
	int retval;
	parinfo->lut_queue_head = (parinfo->lut_queue_head + 1) % MODELF_MAX_LUT_QUEUE_NUM;
	parinfo->lut_queue_count--;
	retval = parinfo->lut_queue_head;
	return retval;
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
	printk("MODELFFB: QUEUE info\n  head = %d, count = %d\n",
		parinfo->lut_queue_head, parinfo->lut_queue_count);
	for (i = 0; i < MODELF_MAX_LUT_QUEUE_NUM; i++) {
		printk("  (%d, %d) - (%d, %d)\n",
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
	__modelffb_simple_command_p5(MODELF_COM_UPDATE_AREA,
		MODELF_WAVEFORM_MODE(waveform_mode) | MODELF_UPDATE_LUT(lut),
		x & 0x1ff, y & 0x3ff, width & 0x1ff, height & 0x3ff);
	__modelffb_simple_command(MODELF_COM_WAIT_TRIGGER_DONE);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
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
#endif

	if (modelffb_lock())
		goto err_power_off;

	__modelffb_cleanup_area_lut(x, y, width, height, waveform_mode, lut);

	modelffb_unlock();

err_power_off:
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
/* this switched power off too early
        pl_hardware_disable(parinfo->pl_hardware); */
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
#endif

	if (modelffb_lock())
		goto err_power_off;

	__modelffb_wait_for_reg_value(MODELF_REG_LUT_STATUS,
				      MODELF_BF_LUT_IN_USE, 0,
				      MODELF_TIMEOUT_MS);
	__modelffb_simple_command_p5(MODELF_COM_UPDATE_AREA,
		MODELF_WAVEFORM_MODE(waveform_mode) | MODELF_UPDATE_LUT(0),
		x & 0x1ff, y & 0x3ff, width & 0x1ff, height & 0x3ff);
	__modelffb_simple_command(MODELF_COM_WAIT_TRIGGER_DONE);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	modelffb_sync_set_status(MODELFFB_SYNC_BUSY);

	modelffb_unlock();

err_power_off:
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
        pl_hardware_disable(parinfo->pl_hardware);
#endif
	return;
}

static void __modelffb_update_area(int x, int y, int width, int height,
				 int waveform_mode, int lut)
{
	__modelffb_simple_command_p5(MODELF_COM_PARTIAL_UPDATE_AREA,
		MODELF_WAVEFORM_MODE(waveform_mode) | MODELF_UPDATE_LUT(lut),
		x & 0x1ff, y & 0x3ff, width & 0x1ff, height & 0x3ff);
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
#endif

	if (modelffb_lock())
		goto err_power_off;

	__modelffb_update_area(x, y, width, height, waveform_mode, lut);

	modelffb_unlock();

err_power_off:
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
        pl_hardware_disable(parinfo->pl_hardware);
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
	__modelffb_immediate_reg_write(MODELF_REG_POWER_SAVE_MODE, 
		(__modelffb_reg_read(MODELF_REG_POWER_SAVE_MODE) & ~MODELF_POWER_ACTIVE)
		| MODELF_POWER_ACTIVE);
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
	printk(KERN_INFO "MODELFFB: WORKQUEUE: go into run mode\n");
#endif
}

static void __modelffb_standby(void)
{
	__modelffb_immediate_reg_write(MODELF_REG_POWER_SAVE_MODE, 
		(__modelffb_reg_read(MODELF_REG_POWER_SAVE_MODE) & ~MODELF_POWER_ACTIVE)
		| MODELF_POWER_ACTIVE);
	/* must not check HRDY before issuing command as this won't work on a sleep->standby transition */
	__modelffb_immediate_simple_command(MODELF_COM_STANDBY);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
}

static void __modelffb_sleep(void)
{
	__modelffb_immediate_simple_command(MODELF_COM_SLEEP);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	__modelffb_immediate_reg_write(MODELF_REG_POWER_SAVE_MODE, MODELF_POWER_PASSIVE);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
}

static void modelffb_sleep(void)
{
	if (modelffb_lock())
		return;
	__modelffb_sleep();
	modelffb_unlock();

#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: WORKQUEUE: go into sleep mode\n");
#endif
}

static void __modelffb_clear_temperature_judge(void)
{
	__modelffb_reg_write(MODELF_REG_DSPE_INT_STATUS, MODELF_INT_WF_UPDATE_INTERRUPT);
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
	printk(KERN_INFO "MODELFFB: measure temperature\n");
#endif

	if (modelffb_lock())
		return;

	/* only wait for LUTs if we're in run mode, shouldn't read synchronous
	 * registers otherwise */
	if (parinfo->power_mode == MODELF_POWER_RUN) {
		__modelffb_wait_for_reg_value(MODELF_REG_LUT_STATUS, MODELF_BF_LUT_IN_USE, 0,
			MODELF_TIMEOUT_MS);
	}

	__modelffb_standby();
	__modelffb_simple_command(MODELF_COM_TEMPERATURE);
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);

	if ((__modelffb_reg_read(MODELF_REG_DSPE_INT_STATUS)
			& MODELF_INT_WF_UPDATE_INTERRUPT) != 0) {
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: temperature zone has been changed\n");
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
	if (stat != 0)
		printk("pl_hardware_set_temperature() failed\n");
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
	info->var.xres_virtual = info->var.xres;
	info->var.yres = modelffb_read_register(MODELF_REG_FRAME_DATA_LENGTH);
	info->var.yres_virtual = info->var.yres;

#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: FB resolution is %dx%d\n",
		info->var.xres, info->var.yres);
#endif
}

static int modelffb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if (var->xres != info->fix.line_length / 2 ||
			var->yres != info->fix.smem_len / info->fix.line_length ||
			var->xres_virtual != info->fix.line_length / 2 ||
			var->yres_virtual != info->fix.smem_len / info->fix.line_length) {
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

static struct fb_ops modelffb_ops = {
	.owner		= THIS_MODULE,
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

#ifdef CONFIG_MODELF_CONNECTION_ASYNC
static int __devinit modelffb_framebuffer_alloc(struct platform_device *pdev)
{
	int retval;
	struct fb_info *info;

	info = framebuffer_alloc(sizeof(struct modelffb_par), &pdev->dev);
	platform_set_drvdata(pdev, info);
#elif CONFIG_MODELF_CONNECTION_SPI
static int __devinit modelffb_framebuffer_alloc(struct spi_device *spi)
{
	int retval;
	struct fb_info *info;

	info = framebuffer_alloc(sizeof(struct modelffb_par), &spi->dev);
	spi->bits_per_word = 16;
	spi_set_drvdata(spi, info);
#endif

	if (!info) {
		printk(KERN_ERR "MODELFFB: failed to alloc modelffb fb_info\n");
		retval = -ENOMEM;
		goto err;
	}

	parinfo = info->par;
	if (!parinfo) {
		printk(KERN_ERR "MODELFFB: failed to alloc modelffb_par\n");
		retval = -ENOMEM;
		goto framebuffer_release;
	}
#ifdef CONFIG_MODELF_CONNECTION_ASYNC
	parinfo->pdev = pdev;
	parinfo->dev = &pdev->dev;
#elif CONFIG_MODELF_CONNECTION_SPI
	parinfo->spi = spi;
	parinfo->dev = &spi->dev;
#endif

	info->pseudo_palette = kmalloc(sizeof(u32) * 256, GFP_KERNEL);
	if (!info->pseudo_palette) {
		printk(KERN_ERR "MODELFFB: failed to alloc pseudo_palette\n");
		retval = -ENOMEM;
		goto framebuffer_release;
	}

	info->fix = modelffb_fix;
	info->var = modelffb_var;
	info->fbops = &modelffb_ops;

	parinfo->fbinfo = info;
	parinfo->lut_queue_head = 0;
	parinfo->lut_queue_count = 0;
	memset(parinfo->lut_in_use, 0, MODELF_MAX_LUT_NUM);
	memset(parinfo->lut_queue, 0, MODELF_MAX_LUT_QUEUE_NUM);

#ifdef CONFIG_I2C_SC18IS60X_SHARED_SPI
	mutex_lock(&sc18is60x_spi_lock);
	sc18is60x_shared_spi = parinfo->spi;
	mutex_unlock(&sc18is60x_spi_lock);
#endif

        /* get pmic regulators */
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
        parinfo->pl_hardware = pl_hardware_alloc();
        if (!parinfo->pl_hardware) {
                retval = -ENOMEM;
                goto out_regulator;
        }

        retval = pl_hardware_init(parinfo->pl_hardware,
                                        &pl_config);
        if (retval) {
                dev_err(parinfo->dev,
                        "failed to initialize Plastic Logic hardware\n");
		retval = -ENODEV;
		goto out_regulator;
        }

        retval = pl_hardware_set_vcom(parinfo->pl_hardware, 9742);
        if (retval) {
                dev_err(parinfo->dev, "failed to set VCOM voltage\n");
                goto out_regulator;
        }
#endif

	parinfo->opt_clear_on_exit = false;

	return 0;

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
out_regulator:
        pl_hardware_free(parinfo->pl_hardware);
#endif
#ifdef CONFIG_I2C_SC18IS60X_SHARED_SPI
	mutex_lock(&sc18is60x_spi_lock);
	sc18is60x_shared_spi = NULL;
	mutex_unlock(&sc18is60x_spi_lock);
#endif

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
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
        pl_hardware_free(parinfo->pl_hardware);
#endif

	framebuffer_release(info);
}

static int modelffb_alloc_vram(void)
{
	struct fb_info *info = parinfo->fbinfo;
	int retval = 0;

	info->fix.smem_len	= info->var.xres * info->var.yres * 2;
	info->fix.mmio_len	= info->var.xres * info->var.yres * 2;
	info->fix.line_length	= info->var.xres * 2;

	info->screen_base = vmalloc(info->fix.smem_len + 4);
	/* extra 4 bytes of the guard for 16-bit access */
	if (!info->screen_base) {
		printk(KERN_ERR "MODELFFB: failed to alloc video memory\n");
		retval = -ENOMEM;
		goto err;
	}

	parinfo->image_pool = (uint32_t)kmalloc(info->fix.line_length, GFP_KERNEL);
	if (!info->screen_base) {
		printk(KERN_ERR "MODELFFB: failed to alloc image pool\n");
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
	printk(KERN_ERR "MODELFFB: modelffb_free_vram()\n");
	kfree((void*)parinfo->image_pool);
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

static DECLARE_WORK(modelffb_temperature_work, modelffb_temperature_work_function);

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
#endif
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: WORKQUEUE: go into standby mode\n");
#endif
	}
	else {
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: WORKQUEUE: already in sleep mode\n");
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
		printk(KERN_INFO "MODELFFB: WORKQUEUE: already in run mode\n");
#endif
	}
}

static void modelffb_oneshot(int waveform_mode, int x, int y, int width, int height)
{
	int free_lut;

	free_lut = modelffb_request_lut(x, y, width, height);
	if (free_lut < 0) {
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: lut is busy => enqueue this oneshot\n");
#endif
		if (modelffb_enqueue_lut_queue(MODELF_ONESHOT_TYPE_ONESHOT, waveform_mode,
				x, y, width, height) < 0) {
			parinfo->need_flush_image = MODELF_NEED_FLUSH_IMAGE;
			printk(KERN_INFO "MODELFFB: lut queue has been overflowed\n");
			printk(KERN_INFO "MODELFFB: all image will be sent at once\n");
		}
	}
	else {
		modelffb_send_image(x, y, width, height);
		modelffb_update_area(x, y, width, height, parinfo->waveform_mode, free_lut);
		/* HIRQ handler will call modelf_queue_sleep() later */
	}
}

static void modelffb_oneshot_cleanup(int waveform_mode, int x, int y, int width, int height)
{
	int free_lut;

	free_lut = modelffb_request_lut(x, y, width, height);
	if (free_lut < 0) {
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: lut is busy => enqueue this cleanup\n");
#endif
		if (modelffb_enqueue_lut_queue(MODELF_ONESHOT_TYPE_CLENUP, waveform_mode,
				x, y, width, height) < 0) {
			if (parinfo->vram_updated == MODELF_VRAM_NEED_UPDATE)
				parinfo->need_flush_image = MODELF_NEED_FLUSH_IMAGE;
			parinfo->need_cleanup = MODELF_NEED_CLEANUP;
			printk(KERN_INFO "MODELFFB: lut queue has been overflowed\n");
			printk(KERN_INFO "MODELFFB: cleanup all will be committed later\n");
		}
	}
	else {
		if (parinfo->vram_updated == MODELF_VRAM_NEED_UPDATE)
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
	printk(KERN_INFO "MODELFFB: %s (%d, %d) - (%d, %d)\n",
		work->oneshot_type == MODELF_ONESHOT_TYPE_ONESHOT ? "oneshot" : "cleanup",
		work->x, work->y, work->x + work->width - 1, work->y + work->height - 1);
#endif

	if (parinfo->suspend_update == MODELF_RESUME_UPDATE) {
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: update is resumed\n");
#endif
	}
	else if (work->oneshot_type == MODELF_ONESHOT_TYPE_CLENUP)
		modelffb_oneshot_cleanup(work->waveform_mode,
			work->x, work->y, work->width, work->height);
	else if (parinfo->vram_updated == MODELF_VRAM_NO_NEED_UPDATE) {
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: VRAM is up to date\n");
#endif
	}
	else
		modelffb_oneshot(work->waveform_mode,
			work->x, work->y, work->width, work->height);

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
		printk(KERN_ERR "MODELFFB: alloc oneshot work failed\n");
		retval = -ENOMEM;
		goto end;
	}
	INIT_WORK((struct work_struct*)os_work, modelffb_oneshot_work_function);

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
	printk(KERN_INFO "MODELFFB: WORKQUEUE: cleanup mode_%d (%d, %d) - (%d, %d)\n",
		work->waveform_mode, work->x, work->y,
		work->x + work->width - 1, work->y + work->height - 1);
#endif

	modelffb_commit_run();
	if (work->x == 0 && work->y == 0 &&
			work->width == info->var.xres && work->height == info->var.yres) {
		modelffb_cleanup_full(work->waveform_mode);
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: cleanup all\n");
#endif
	}
	else {
		modelffb_cleanup_area(
			work->x, work->y, work->width, work->height, work->waveform_mode);
	}
	/* HIRQ handler will call modelf_queue_sleep() later */

	kfree((void*)work);
}

static int __modelffb_queue_cleanup(int waveform_mode, int x, int y, int width, int height)
{
	int retval = 0;

	modelffb_sync_set_status(MODELFFB_SYNC_PENDING);

	if (parinfo->suspend_update == MODELF_SUSPEND_UPDATE) {
		__modelffb_queue_oneshot(MODELF_ONESHOT_TYPE_CLENUP, waveform_mode,
			x, y, width, height);
	}
	else {
		struct modelffb_cleanup_work *cu_work;
		cu_work = kmalloc(sizeof(struct modelffb_cleanup_work), GFP_KERNEL);
		if (!cu_work) {
			printk(KERN_ERR "MODELFFB: alloc cleanup workqueue failed\n");
			retval = -ENOMEM;
			goto end;
		}
		INIT_WORK((struct work_struct*)cu_work, modelffb_cleanup_work_function);

		cu_work->waveform_mode = waveform_mode;
		cu_work->x = x;
		cu_work->y = y;
		cu_work->width = width;
		cu_work->height = height;

		queue_work(parinfo->workqueue, (struct work_struct*)cu_work);
	}

end:
	return retval;
}

static void modelffb_suspend_update_work_function(struct work_struct *ws)
{
	struct modelffb_suspend_update_work *work =
		(struct modelffb_suspend_update_work*)ws;

	if (work->update == MODELF_SUSPEND_UPDATE) {
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: WORKQUEUE: suspend update\n");
#endif
	}
	else if (work->update == MODELF_RESUME_UPDATE) {
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: WORKQUEUE: resume update\n");
		parinfo->vram_updated = MODELF_VRAM_NO_NEED_UPDATE;
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
		printk(KERN_ERR "MODELFFB: %s doubled\n",
			parinfo->suspend_update == MODELF_SUSPEND_UPDATE ?
			"suspend update" : "resume update");
		goto end;
	}

	su_work = kmalloc(sizeof(struct modelffb_suspend_update_work), GFP_KERNEL);
	if (!su_work) {
		printk(KERN_ERR "MODELFFB: alloc suspend_update work failed\n");
		retval = -ENOMEM;
		goto end;
	}
	INIT_WORK((struct work_struct*)su_work, modelffb_suspend_update_work_function);

	su_work->update = update;

	queue_work(parinfo->workqueue, (struct work_struct*)su_work);

end:
	return retval;
}

static void modelffb_irq_work_function(struct work_struct *ws)
{
	int free_lut;

	del_timer_sync(&parinfo->sleep_timer);
#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: WORKQUEUE: HIRQ\n");
#endif

	if (modelffb_lock())
		return;
	__modelffb_run();
	__modelffb_reg_write(MODELF_REG_DSPE_INT_STATUS, MODELF_INT_DSPE_ONE_LUT_FREE);
	__modelffb_reg_write(MODELF_REG_INTERRUPT_STATUS, MODELF_INT_DISPLAY_ENGINE);
	__modelffb_sync_lut();

	if (parinfo->need_flush_image == MODELF_NO_NEED_FLUSH_IMAGE &&
			parinfo->need_cleanup == MODELF_NO_NEED_CLEANUP) {
		struct modelffb_oneshot_info *os_info;
		while (1) {
			if (__modelffb_lut_queue_is_empty())
				break;

			os_info = __modelffb_peek_lut_queue();
			free_lut = __modelffb_request_lut(os_info->x, os_info->y,
				os_info->width, os_info->height);

			if (free_lut < 0)
				break;

			__modelffb_dequeue_lut_queue();

			if (parinfo->vram_updated == MODELF_VRAM_NEED_UPDATE)
				__modelffb_send_image(os_info->x, os_info->y,
					os_info->width, os_info->height);

			if (os_info->oneshot_type == MODELF_ONESHOT_TYPE_ONESHOT)
				__modelffb_update_area(os_info->x, os_info->y,
					os_info->width, os_info->height,
					os_info->waveform_mode, free_lut);
			else if (os_info->oneshot_type == MODELF_ONESHOT_TYPE_CLENUP)
				__modelffb_cleanup_area_lut(os_info->x, os_info->y,
					os_info->width, os_info->height,
					os_info->waveform_mode, free_lut);
		}

		modelffb_sync_set_status(MODELFFB_SYNC_IDLE);
	}
	else if (__modelffb_lut_is_empty()) {
		if (parinfo->need_flush_image == MODELF_NEED_FLUSH_IMAGE) {
			__modelffb_reset_lut_queue();
			parinfo->need_flush_image = MODELF_NO_NEED_FLUSH_IMAGE;
			parinfo->vram_updated = MODELF_VRAM_NO_NEED_UPDATE;
			__modelffb_send_all_image();
		}

		if (parinfo->need_cleanup == MODELF_NEED_CLEANUP) {
			parinfo->need_cleanup = MODELF_NO_NEED_CLEANUP;
			__modelffb_cleanup_full(parinfo->waveform_mode);
		}
		else if (parinfo->need_cleanup == MODELF_NO_NEED_CLEANUP)
			__modelffb_update_full(parinfo->waveform_mode);
	}
	__modelffb_wait_for_HRDY_ready(MODELF_TIMEOUT_MS);
	if (parinfo->power_mode == MODELF_POWER_RUN)
		__modelffb_run();
	else if (parinfo->power_mode == MODELF_POWER_SLEEP)
		__modelffb_sleep();

	modelffb_unlock();

	__modelffb_set_sleep_timer();


	enable_irq(MODELF_HIRQ);
}

static DECLARE_WORK(modelffb_irq_work, modelffb_irq_work_function);

static irqreturn_t __modelffb_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(MODELF_HIRQ);
	queue_work(parinfo->workqueue, &modelffb_irq_work);

	return IRQ_HANDLED;
}

static void modelffb_deferred_io_work_function(struct work_struct *ws)
{
#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: WORKQUEUE: deferred\n");
#endif

	if (parinfo->suspend_update == MODELF_SUSPEND_UPDATE) {
#ifdef CONFIG_MODELF_DEBUG
		printk(KERN_INFO "MODELFFB: update has been suspended\n");
#endif
	}
	else if (parinfo->vram_updated == MODELF_VRAM_NEED_UPDATE) {
		parinfo->vram_updated = MODELF_VRAM_NO_NEED_UPDATE;
		modelffb_commit_run();
		modelffb_send_all_image();
		modelffb_update_full(parinfo->waveform_mode);
	}
}

static DECLARE_WORK(modelffb_deferred_io_work, modelffb_deferred_io_work_function);

static void __modelffb_dpy_deferred_io(struct fb_info *info, struct list_head *pagelist)
{
#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: deferred\n");
#endif
	parinfo->vram_updated = MODELF_VRAM_NEED_UPDATE;
	queue_work(parinfo->workqueue, &modelffb_deferred_io_work);
}

static struct fb_deferred_io __modelffb_defio = {
	.delay		= HZ / MODELF_DEFERRED_IO_DELAY_DENOMINATOR,
	.deferred_io	= __modelffb_dpy_deferred_io,
};

static int __devinit modelffb_register_framebuffer(void)
{
	struct fb_info *info = parinfo->fbinfo;
	int retval;

	info->fbdefio = &__modelffb_defio;
	fb_deferred_io_init(info);
	parinfo->vram_updated = MODELF_VRAM_NO_NEED_UPDATE;

	retval = register_framebuffer(info);
	if(retval < 0) {
		printk(KERN_ERR "MODELFFB: register frame buffer failed (%d)\n", retval);
		goto err;
	}

	return 0;

err:
	fb_deferred_io_cleanup(info);
	return retval;
}

static void modelffb_unregister_framebuffer(void)
{
	struct fb_info *info = parinfo->fbinfo;

	unregister_framebuffer(info);
	fb_deferred_io_cleanup(info);
}

static int modelffb_modelf_init(void)
{
	int stat;

	if (!(parinfo->status & MODELF_STATUS_INITCODE_STORED)) {
		printk(KERN_ERR "MODELFFB: init code not stored\n");
		return -EINVAL;
	}

	if (!(parinfo->status & MODELF_STATUS_WAVEFORM_STORED)) {
		printk(KERN_ERR "MODELFFB: waveform not stored\n");
		return -EINVAL;
	}

	stat = modelffb_chip_init();
	if (stat) {
		printk(KERN_ERR "MODELFFB: chip_init failed\n");
		goto exit_now;
	}

	modelffb_check_panel_resolution();

	stat = modelffb_alloc_vram();
	if (stat) {
		printk(KERN_ERR "MODELFFB: alloc_vram failed\n");
		goto exit_now;
	}

	stat = modelffb_register_framebuffer();
	if (stat) {
		printk(KERN_ERR "MODELFFB: register_framebuffer failed\n");
		goto free_vram;
	}

	stat = modelffb_panel_init();
	if (stat) {
		printk(KERN_ERR "MODELFFB: panel_init failed\n");
		goto framebuffer_unregister;
	}

	stat = request_irq(MODELF_HIRQ, __modelffb_irq_handler,
			   (IRQF_DISABLED | IRQF_TRIGGER_HIGH), "modelf",
			   parinfo->dev);
	if (stat) {
		printk(KERN_ERR "MODELFFB: request_irq failed\n");
		goto framebuffer_unregister;
	}

	modelffb_measure_temperature();
	__modelffb_start_temperature_timer();

#ifdef CONFIG_MODELF_DEBUG
	modelffb_print_reg(MODELF_REG_DSPE_INT_STATUS);
	modelffb_print_reg(MODELF_REG_SENSOR_TEMPERATURE);
#endif

	return 0;

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
		printk(KERN_ERR "MODELFFB: MAX_INITCODE_SIZE has been reached\n");
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
		printk(KERN_ERR "MODELFFB: MAX_WAVEFORM_SIZE has been reached\n");
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
	int retval = 0;
	char *tok, *next_tok = str;

	tok = strsep(&next_tok, " \t\n");
	if (tok != NULL) {
		if (strcmp(tok, "waveform_mode") == 0) {
		/* echo set waveform_mode mode > /sys/devices/platform/modelffb/control */
			tok = strsep(&next_tok, " \t\n");
			if (tok != NULL) {
				sscanf(tok, "%d", &parinfo->waveform_mode);
				printk(KERN_INFO "MODELFFB: waveform mode has been set as %d\n",
					parinfo->waveform_mode);
			}
			else {
				printk(KERN_ERR "MODELFFB: no waveform mode has been specified\n");
			}
		}
		else if (strcmp(tok, "powercut_time") == 0) {
		/* echo set powercut_time seconds > /sys/devices/platform/modelffb/control */
			tok = strsep(&next_tok, " \t\n");
			if (tok != NULL) {
				del_timer_sync(&parinfo->sleep_timer);
				sscanf(tok, "%d", &parinfo->seconds_to_sleep);
				printk(KERN_INFO "MODELFFB: %d seconds to stand by\n",
					parinfo->seconds_to_sleep);
				__modelffb_queue_sleep();
			}
			else {
				printk(KERN_ERR "MODELFFB: no powercut time has been specified\n");
			}
		}
		else if (strcmp(tok, "measure_temp_interval") == 0) {
		/* echo set measure_temp_interval seconds > /sys/devices/platform/modelffb/control */
			tok = strsep(&next_tok, " \t\n");
			if (tok != NULL) {
				del_timer_sync(&parinfo->temperature_timer);
				sscanf(tok, "%d", &parinfo->seconds_to_measure_temperature);
				printk(KERN_INFO "MODELFFB: %d seconds to measure temperature\n",
					parinfo->seconds_to_measure_temperature);
				if (parinfo->seconds_to_measure_temperature != 0)
					__modelffb_queue_temperature();
			}
			else {
				printk(KERN_ERR "MODELFFB: no interval time has been specified\n");
			}
		}
		else if (strcmp(tok, "send_waveform") == 0) {
		/* echo set send_waveform_wait wait/nowait > /sys/devices/platform/modelffb/control */
			tok = strsep(&next_tok, " \t\n");
			if (tok != NULL) {
				if (strcmp(tok, "wait") == 0) {
				printk(KERN_INFO "MODELFFB: send waveform: wait\n");
				parinfo->send_waveform_wait = MODELF_SEND_WAVEFORM_WAIT;
				}
				else if (strcmp(tok, "nowait") == 0) {
				printk(KERN_INFO "MODELFFB: send waveform: nowait\n");
				parinfo->send_waveform_wait = MODELF_SEND_WAVEFORM_NOWAIT;
				}
			}
			else {
				printk(KERN_ERR "MODELFFB: no wait selection has been specified\n");
			}
		}
		else {
			printk(KERN_ERR "MODELFFB: invalid set argument\n");
		}
	}
	else {
		retval = -1;
		goto end;
	}
end:
	return retval;
}

static void modelffb_setopt(struct fb_info *info, char *tokbuf, size_t len)
{
	char *opt;
	char *value;

	opt = strsep(&tokbuf, " \t\n");

	if (!opt)
		return;

	value = strsep(&tokbuf, " \t\n");

	if (!strcmp(opt, "clear_on_exit") && value) {
		long unsigned int int_value;

		if (kstrtoul(value, 10, &int_value))
			return;

		parinfo->opt_clear_on_exit = int_value ? true : false;
	}
}

static ssize_t modelffb_store_control(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	char localbuf[256];
	char *tok, *next_tok;
	struct fb_info *info = parinfo->fbinfo;

	strncpy(localbuf, buf, 255);
	localbuf[255] = '\0';
	next_tok = localbuf;

	tok = strsep(&next_tok, " \t\n");

	if (strcmp(tok, "init") == 0) {
	/* echo init > /sys/devices/platform/modelffb/control */
		if ((parinfo->status & MODELF_STATUS_INIT_DONE) ==
		    MODELF_STATUS_INIT_DONE) {
			printk(KERN_ERR "MODELFFB: init doubled\n");
		}
		else if (modelffb_modelf_init() == 0)
			parinfo->status |= MODELF_STATUS_INIT_DONE;
	}
	else if (strcmp(tok, "set") == 0) {
	/* echo set element value > /sys/devices/platform/modelffb/control */
		__modelffb_set_element(next_tok);
	}
	else if (!strcmp(tok, "setopt")) {
		modelffb_setopt(info, next_tok, count);
	}
	else if ((parinfo->status & MODELF_STATUS_INIT_DONE) == 0) {
		printk(KERN_ERR "MODELFFB: not yet init done\n");
	}
	else if (strcmp(tok, "dumpmem") == 0) {
	/* echo dumpmem address size > /sys/devices/platform/modelffb/control */
		uint32_t start = 0, size = 0;

		if (__parse_uint32_2(next_tok, &start, &size) == 0) {
			modelffb_commit_run();
			modelffb_dump_memory(start, size);
			__modelffb_queue_sleep();
		}
	}
	else if (strcmp(tok, "dumpreg") == 0) {
	/* echo dumpreg address size > /sys/devices/platform/modelffb/control */
		uint32_t start = 0, size = 0;

		if (__parse_uint32_2(next_tok, &start, &size) == 0) {
			modelffb_commit_run();
			modelffb_dump_register(start, size);
			__modelffb_queue_sleep();
		}
	}
	else if (strcmp(tok, "writereg") == 0) {
	/* echo writereg address data > /sys/devices/platform/modelffb/control */
		uint32_t addr = 0, data = 0;

		if (__parse_uint32_2(next_tok, &addr, &data) == 0) {
			modelffb_commit_run();
			modelffb_write_register(addr, data);
			__modelffb_queue_sleep();
		}
	}
	else if (strcmp(tok, "cleanup") == 0) {
	/* echo cleanup [wfmode] [x] [y] [width] [height]
		> /sys/devices/platform/modelffb/control */
		int waveform_mode = parinfo->waveform_mode, x = 0, y = 0;
		int width = info->var.xres, height = info->var.yres;

		tok = strsep(&next_tok, " \t\n");
		if (tok != NULL) {
			if (strcmp(tok, "whiteout") == 0)
				waveform_mode = MODELF_WAVEFORM_WHITEOUT;
			else if (strcmp(tok, "direct_mono") == 0)
				waveform_mode = MODELF_WAVEFORM_DIRECT_MONO;
			else if (strcmp(tok, "high_quality") == 0)
				waveform_mode = MODELF_WAVEFORM_HIGH_QUALITY;
			else if (strcmp(tok, "high_speed") == 0)
				waveform_mode = MODELF_WAVEFORM_HIGH_SPEED;
			else
				sscanf(tok, "%d", &waveform_mode);
		}
		__parse_uint32_4(next_tok, &x, &y, &width, &height);

		__modelffb_queue_cleanup(waveform_mode, x, y, width, height);
	}
	else if (!strcmp(tok, "sync")) {
		tok = strsep(&next_tok, " \t\n");

		if (unlikely(!tok))
			printk(KERN_ERR "No sync status provided\n");
		else
			modelffb_wait_sync(tok);
	}
	else if (strcmp(tok, "suspend_update") == 0) {
	/* echo suspend_update > /sys/devices/platform/modelffb/control */
		__modelffb_queue_suspend_update(MODELF_SUSPEND_UPDATE);
	}
	else if (strcmp(tok, "resume_update") == 0) {
	/* echo resume_update > /sys/devices/platform/modelffb/control */
		__modelffb_queue_suspend_update(MODELF_RESUME_UPDATE);
	}
	else if (strcmp(tok, "oneshot") == 0) {
	/* echo oneshot [x] [y] [width] [height] > /sys/devices/platform/modelffb/control */
		int x = 0, y = 0, width = info->var.xres, height = info->var.yres;

		__parse_uint32_4(next_tok, &x, &y, &width, &height);
		__modelffb_queue_oneshot(MODELF_ONESHOT_TYPE_ONESHOT, parinfo->waveform_mode,
			x, y, width, height);
	}
	else {
		printk(KERN_ERR "MODELFFB: invalid control argument\n");
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
	}
	else {
		strncpy(buf, "0", 255);
	}

	return 1;
}

#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
static ssize_t temperature_range_show(struct device *dev,
                                struct device_attribute *attr, char *buf,
                                struct temperature_set* set,
                                int (*format_text)(const struct temperature_entry* p_range, char *tbuf, int buf_space))
{
        char *tbuf = buf;
        int written;
        const struct temperature_entry** p_range = 0;
        int buf_space = PAGE_SIZE;

        if (format_text == 0)
                return -EINVAL;

        p_range = temperature_set_get_first_range(set);

        while (buf_space >= 0 && p_range != 0) {
                written = (format_text)(*p_range, tbuf, buf_space);

                p_range = temperature_set_get_next_range(set, p_range);

                tbuf += written;
                buf_space -= written;
        }

        return tbuf - buf;
}

static ssize_t temperature_range_store(struct device *dev,
                                struct device_attribute *attr, const char *buf,
                                size_t count,
                                struct temperature_set* set,
                                void (*erase_set)(struct temperature_set* set),
                                int (*read_and_store)(const char *p_range_text))

{
        int retval = -EINVAL;
        const char *p_next_range = buf;

        if (erase_set == 0 || read_and_store == 0)
                return -EINVAL;

        (erase_set)(set);

        do {
                /* give up if invalid */
                retval = (read_and_store)(p_next_range);

                if (retval)
                        break;

                /* locate \n at end of this range */
                p_next_range = strchr(p_next_range, '\n');

                /* then advance to the 1st char of the next range */
                if (p_next_range == 0 || *++p_next_range == '\0') {
                        retval = strlen(buf);
                }
        }
        while (p_next_range != 0 && *p_next_range != '\0');
        temperature_set_commit(set);

        return retval;
}

static ssize_t vcom_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct temperature_set *set = get_vcom_temperature_set();

        return temperature_range_show(dev, attr, buf, set, temperature_set_format_integer);
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
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: cannot create file \"init_code\"\n");
		goto end;
	}
	retval = device_create_file(parinfo->dev, &dev_attr_waveform);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: cannot create file \"waveform\"\n");
		goto remove_init_code;
	}
	retval = device_create_file(parinfo->dev, &dev_attr_control);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: cannot create file \"control\"\n");
		goto remove_waveform;
	}
	retval = device_create_file(parinfo->dev, &dev_attr_keycode);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: cannot create file \"keycode\"\n");
		goto remove_control;
	}
	retval = device_create_file(parinfo->dev, &dev_attr_on_drawing);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: cannot create file \"on_drawing\"\n");
		goto remove_keycode;
	}
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
	retval = device_create_file(parinfo->dev, &dev_attr_vcom);
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: cannot create file \"vcom\".\n");
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
	printk(KERN_ERR "MODELFFB: modelffb_remove_file()\n");
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

/* =========== registration & unregistration =========== */
static int __devinit modelffb_alloc_memory(void)
{
	int retval = 0;

	parinfo->init_code = (uint32_t)vmalloc(MODELF_MAX_INITCODE_SIZE);
	if (!parinfo->init_code) {
		retval = -ENOMEM;
		goto end;
	}

	parinfo->waveform = (uint32_t)vmalloc(MODELF_MAX_WAVEFORM_SIZE);
	if (!parinfo->waveform) {
		retval = -ENOMEM;
		goto free_init_code;
	}

	parinfo->waveform_index = parinfo->waveform;
	parinfo->waveform_size = 0;
	parinfo->init_code_size = 0;

	return 0;

free_init_code:
	vfree((void*)parinfo->init_code);
end:
	return retval;
}

static void __devexit modelffb_free_memory(void)
{
	vfree((void*)parinfo->waveform);
	vfree((void*)parinfo->init_code);
}

#ifdef CONFIG_MODELF_CONNECTION_ASYNC
static int __devinit modelffb_probe(struct platform_device *pdev)
#elif CONFIG_MODELF_CONNECTION_SPI
static int __devinit modelffb_probe(struct spi_device *spi)
#endif
{
	int retval = 0;

#ifdef CONFIG_MODELF_DEBUG
	printk(KERN_INFO "MODELFFB: check rev: %d\n", CHECKREV);
#endif

#ifdef CONFIG_MODELF_CONNECTION_SPI
#ifdef CONFIG_MODELF_SPI_WITHOUT_HDC
	retval = gpio_request(MODELF_SPIHCS_GPIO, "MODELF_HCS");
	if (retval) {
		printk(KERN_ERR "MODELFFB: gpio not allocated\n");
		goto err;
	}
	gpio_direction_output(MODELF_SPIHCS_GPIO, 1);
	gpio_set_value(MODELF_SPIHCS_GPIO, 1);
#else
	retval = gpio_request(MODELF_SPIHDC_GPIO, "MODELF_HDC");
	if (retval) {
		printk(KERN_ERR "MODELFFB: gpio not allocated\n");
		goto err;
	}
	gpio_direction_output(MODELF_SPIHDC_GPIO, 1);
	gpio_set_value(MODELF_SPIHDC_GPIO, 1);
#endif
#else
	if (gpio_request(MODELF_HRDY_GPIO, "MODELF_HRDY") < 0) {
		printk(KERN_ERR "MODELFFB: MODELF_HRDY (GPIO %d) is busy\n",
		       MODELF_HRDY_GPIO);
		retval = -EBUSY;
		goto err;
	}
	gpio_direction_input(MODELF_HRDY_GPIO);
#endif

#ifdef CONFIG_MODELF_CONNECTION_ASYNC
	retval = modelffb_framebuffer_alloc(pdev);
#elif CONFIG_MODELF_CONNECTION_SPI
	retval = modelffb_framebuffer_alloc(spi);

#endif
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: failed to alloc frame buffer\n");
		goto free_gpio;
	}

	retval = modelffb_alloc_memory();
	if (retval != 0) {
		printk(KERN_ERR "MODELFFB: failed to alloc modelffb_par members\n");
		goto framebuffer_release;
	}
	__modelffb_bit_swap_table_init();

	retval = __modelffb_request_bus();
	if (retval != 0) {
		goto free_memory;
	}

	retval = modelffb_create_file();
	if (retval != 0) {
		goto release_bus;
	}

	parinfo->workqueue = create_singlethread_workqueue("modelf_workqueue");
	if (!parinfo->workqueue) {
		printk(KERN_ERR "MODELFFB: create workqueue failed\n");
		goto free_gpio;
	}

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
	parinfo->temperature_timer.function = __modelffb_temperature_timer_handler;
	init_waitqueue_head(&parinfo->sync_update_wait);
	parinfo->sync_status = MODELFFB_SYNC_IDLE;

	printk(KERN_INFO "MODELFFB: modelF on %s has been probed.\n",
	       MODELF_CONNECTION);

	return 0;

free_gpio:
#ifdef CONFIG_MODELF_CONNECTION_SPI
#ifdef CONFIG_MODELF_SPI_WITHOUT_HDC
	gpio_free(MODELF_SPIHCS_GPIO);
#else
	gpio_free(MODELF_SPIHDC_GPIO);
#endif
#else
	gpio_free(MODELF_HRDY_GPIO);
#endif
#ifdef CONFIG_MODELF_CONNECTION_ASYNC
remove_file:
	modelffb_remove_file();
#endif
release_bus:
	__modelffb_release_bus();
free_memory:
	modelffb_free_memory();
framebuffer_release:
	modelffb_framebuffer_release();
err:
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

#ifdef CONFIG_MODELF_CONNECTION_ASYNC
static int __devexit modelffb_remove(struct platform_device *pdev)
#elif CONFIG_MODELF_CONNECTION_SPI
static int __devexit modelffb_remove(struct spi_device *spi)
#endif
{
	del_timer_sync(&parinfo->sleep_timer);
	del_timer_sync(&parinfo->temperature_timer);

	if ((parinfo->status & MODELF_STATUS_INIT_DONE)
	    == MODELF_STATUS_INIT_DONE) {
		disable_irq_nosync(MODELF_HIRQ);
		free_irq(MODELF_HIRQ, parinfo->dev);

		if (parinfo->opt_clear_on_exit)
			do_clear_on_exit();

		modelffb_unregister_framebuffer();
		modelffb_free_vram();
	}

#ifdef CONFIG_MODELF_CONNECTION_SPI
#ifdef CONFIG_MODELF_SPI_WITHOUT_HDC
	gpio_free(MODELF_SPIHCS_GPIO);
#else
	gpio_free(MODELF_SPIHDC_GPIO);
#endif
#endif
	flush_workqueue(parinfo->workqueue);
	destroy_workqueue(parinfo->workqueue);
#ifdef CONFIG_MODELF_CONNECTION_ASYNC
	gpio_free(MODELF_HRDY_GPIO);
#endif
	modelffb_remove_file();
	__modelffb_release_bus();
	modelffb_free_memory();
	modelffb_framebuffer_release();

	printk(KERN_INFO "MODELFFB: modelF has been removed.\n");
	return 0;
}

#ifdef CONFIG_MODELF_CONNECTION_ASYNC
static void modelffb_shutdown(struct platform_device *pdev)
{
	modelffb_remove(pdev);
}
#elif CONFIG_MODELF_CONNECTION_SPI
static void modelffb_shutdown(struct spi_device *spi)
{
	modelffb_remove(spi);
}
#endif

#ifdef CONFIG_MODELF_CONNECTION_ASYNC
static int modelffb_suspend(struct platform_device *pdev, pm_message_t msg)
#elif CONFIG_MODELF_CONNECTION_SPI
static int modelffb_suspend(struct spi_device *spi, pm_message_t msg)
#endif
{
	del_timer_sync(&parinfo->sleep_timer);
	modelffb_commit_sleep();

	return 0;
}

#ifdef CONFIG_MODELF_CONNECTION_ASYNC
static int modelffb_resume(struct platform_device *pdev)
#elif CONFIG_MODELF_CONNECTION_SPI
static int modelffb_resume(struct spi_device *spi)
#endif
{
	modelffb_commit_run();

	return 0;
}

#ifdef CONFIG_MODELF_CONNECTION_ASYNC
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
#elif CONFIG_MODELF_CONNECTION_SPI
static struct spi_driver modelffb_driver = {
	.probe		= modelffb_probe,
	.remove 	= __devexit_p(modelffb_remove),
	.shutdown	= modelffb_shutdown,
	.suspend	= modelffb_suspend,
	.resume		= modelffb_resume,
	.driver = {
		.name	= "modelffb_spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
};
#endif

static int __init __modelffb_init(void)
{
	int retval = 0;
#ifdef CONFIG_MODELF_CONNECTION_ASYNC
	retval = platform_driver_register(&modelffb_driver);
#elif CONFIG_MODELF_CONNECTION_SPI
	retval = spi_register_driver(&modelffb_driver);
#endif
	return retval;
}

static void __exit __modelffb_exit(void)
{
#ifdef CONFIG_MODELF_CONNECTION_ASYNC
	platform_driver_unregister(&modelffb_driver);
#elif CONFIG_MODELF_CONNECTION_SPI
	spi_unregister_driver(&modelffb_driver);
#endif
}

module_init(__modelffb_init);
module_exit(__modelffb_exit);

MODULE_DESCRIPTION("fbdev driver for modelF EPD controller");
MODULE_VERSION("2.0");
MODULE_LICENSE("GPL");

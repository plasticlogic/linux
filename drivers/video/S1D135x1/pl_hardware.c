/*
 * pl_hardware.c -- Plastic Logic e-paper display power control
 *
 * Copyright (C) 2012, 2013 Plastic Logic Limited
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

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include "pl_hardware.h"
#include "temperature-set.h"
#include "vcom.h"

/* Set to 1 to enable the CPLD code */
#define USE_CPLD 0

/* I2C addresses */
#define CPLD_I2C_ADDRESS 0x70
#define HVPMIC_I2C_ADDRESS 0x48

/* CPLD parameters */
#define CPLD_MIN_VERSION 0x01
#define CPLD_NB_BYTES 3

/* VCOM DAC */
/* ToDo: make platform-dependent */
#define VCOM_MAX 13000
#define VCOM_MIN 1000
#define VCOM_DEFAULT ((VCOM_MAX + VCOM_MIN) / 2)
#define VCOM_OFFSET (-19532)
#define VCOM_COEF_INT 20
#define VCOM_COEF_DEC 779
#define DAC5820_CMD_LOAD_IN_DAC_A__UP_DAC_B__OUT_AB 0x0
#define DAC5820_CMD_EXT__DATA_0 0xF

/* HVPMIC MAX17135 timings */
#define HVPMIC_NB_TIMINGS 8
#define HVPMIC_TIMING_UP_VGNEG 8
#define HVPMIC_TIMING_UP_VSNEG 2
#define HVPMIC_TIMING_UP_VSPOS 11
#define HVPMIC_TIMING_UP_VGPOS 3
#define HVPMIC_TIMING_DOWN_VGPOS 0
#define HVPMIC_TIMING_DOWN_VSPOS 0
#define HVPMIC_TIMING_DOWN_VSNEG 0
#define HVPMIC_TIMING_DOWN_VGNEG 0

#if USE_CPLD
/* CPLD definitions */

enum pl_hardware_cpld_switch {
	CPLD_HVEN,
	CPLD_COM_SW_EN,
	CPLD_COM_SW_CLOSE,
	CPLD_COM_PSU,
	CPLD_BPCOM_CLAMP,
};

struct cpld_byte_0 {
	__u8 cpld_hven:1;
	__u8 bpcom_clamp:1;
	__u8 version:6;
};

struct cpld_byte_1 {
	__u8 vcom_sw_close:1;
	__u8 vcom_sw_en:1;
	__u8 vcom_psu_en:1;
	__u8 vgpos_clamp:1;
	__u8 build_version:4;
};

struct cpld_byte_2 {
	__u8 board_id:4;
	__u8 reserved:4;
};

union pl_hardware_cpld {
	struct {
		struct cpld_byte_0 b0;
		struct cpld_byte_1 b1;
		struct cpld_byte_2 b2;
	};
	u8 data[CPLD_NB_BYTES];
};
#endif /* USE_CPLD */

/* HVPMIC definitions */

enum pl_hardware_hvpmic_register {
	HVPMIC_REG_EXT_TEMP   = 0x00,
	HVPMIC_REG_CONF       = 0x01,
	HVPMIC_REG_INT_TEMP   = 0x04,
	HVPMIC_REG_TEMP_STAT  = 0x05,
	HVPMIC_REG_PROD_REV   = 0x06,
	HVPMIC_REG_PROD_ID    = 0x07,
	HVPMIC_REG_DVR        = 0x08,
	HVPMIC_REG_ENABLE     = 0x09,
	HVPMIC_REG_FAULT      = 0x0A,
	HVPMIC_REG_PROG       = 0x0C,
	HVPMIC_REG_TIMING_1   = 0x10,
	HVPMIC_REG_TIMING_2   = 0x11,
	HVPMIC_REG_TIMING_3   = 0x12,
	HVPMIC_REG_TIMING_4   = 0x13,
	HVPMIC_REG_TIMING_5   = 0x14,
	HVPMIC_REG_TIMING_6   = 0x15,
	HVPMIC_REG_TIMING_7   = 0x16,
	HVPMIC_REG_TIMING_8   = 0x17,
};

union hvpmic_fault {
	struct {
		char fbpg:1;
		char hvinp:1;
		char hvinn:1;
		char fbng:1;
		char hvinpsc:1;
		char hvinnsc:1;
		char ot:1;
		char pok:1;
	};
	char byte;
};

struct pl_hardware_hvpmic {
	__u8 prod_id;
	__u8 prod_rev;
	__u8 timings[HVPMIC_NB_TIMINGS];
};

/* VCOM DAC definitions */

enum dac5820_pd {
	DAC5820_PD_ON         = 0x0,
	DAC5820_PD_OFF_FLOAT  = 0x1,
	DAC5820_PD_OFF_1K     = 0x2,
	DAC5820_PD_OFF_100K   = 0x3
};

struct dac5820_byte_write_cmd {
	char data_high:4;
	char cmd:4;
};

struct dac5820_byte_cmd_ext {
	char pd:2;
	char a:1;
	char b:1;
	char reserved:4;
};

struct dac5820_byte_write_data {
	char reserved:4;
	char data_low:4;
};

union dac5820_write_payload {
	struct {
		struct dac5820_byte_write_cmd cmd_byte;
		struct dac5820_byte_write_data data_byte;
	};
	char bytes[2];
};

union dac5820_ext_payload {
	struct {
		struct dac5820_byte_write_cmd cmd_byte;
		struct dac5820_byte_cmd_ext ext_byte;
	};
	char bytes[2];
};

struct pl_hardware_vcom_dac {
	__u8 dac_value;
	__u8 i2c_address;
	int vcom_mv;
};

/* Module A GPIO pins */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))
#if defined(CONFIG_MODELF_PL_Z1_3)
#define GPIO_POK		GPIO_TO_PIN(0, 15)
#define GPIO_PMIC_EN		GPIO_TO_PIN(3, 21)
#define GPIO_VCOM_SW_CLOSE	GPIO_TO_PIN(0, 14)
#elif defined(CONFIG_MODELF_PL_Z5_0)
#define GPIO_POK		GPIO_TO_PIN(0, 15)
#define GPIO_PMIC_EN		GPIO_TO_PIN(2, 25)
#define GPIO_VCOM_SW_CLOSE	GPIO_TO_PIN(0, 14)
#define GPIO_I2C_ENABLE		GPIO_TO_PIN(1 , 4)
#define GPIO_3V3_ENABLE		GPIO_TO_PIN(1,  1)
#define GPIO_VPP_SOLOMON	GPIO_TO_PIN(1, 29)
#define GPIO_SOL_EP_SELECT	GPIO_TO_PIN(2, 24)
#define GPIO_GPIO1		GPIO_TO_PIN(1,  0)
#define GPIO_GPIO2		GPIO_TO_PIN(2, 22)
#define GPIO_GPIO3		GPIO_TO_PIN(2, 23)
#define GPIO_LED1		GPIO_TO_PIN(1,  2)
#define GPIO_LED2		GPIO_TO_PIN(1,  6)
#define GPIO_LED3		GPIO_TO_PIN(1,  3)
#define GPIO_LED4		GPIO_TO_PIN(1,  7)
#endif

/* Opaque public instance structure */

struct pl_hardware {
	bool init_done;
	const struct pl_hardware_config *config;
	struct i2c_adapter *i2c;
#if USE_CPLD
	union pl_hardware_cpld cpld;
#endif
	struct pl_hardware_hvpmic hvpmic;
	struct pl_hardware_vcom_dac vcom_dac;
	int last_temperature;
	bool hv_on;
	bool is_module_a;
};

#if USE_CPLD
/* CPLD */
static int pl_hardware_cpld_init(struct pl_hardware *p);
static int pl_hardware_cpld_switch(struct pl_hardware *p,
				   enum pl_hardware_cpld_switch sw, bool on);
static int pl_hardware_cpld_read_data(struct pl_hardware *p);
static int pl_hardware_cpld_write_data(struct pl_hardware *p);
#endif

/* HVPMIC */
static int pl_hardware_hvpmic_init(struct pl_hardware *p);
static int pl_hardware_hvpmic_load_timings(struct pl_hardware *p);
static int pl_hardware_hvpmic_wait_pok(struct pl_hardware *p);

/* VCOM DAC */
static int pl_hardware_dac_init(struct pl_hardware *p);
static int pl_hardware_dac_set_power(struct pl_hardware *p,
				     bool on);
static void pl_hardware_dac_set_voltage(struct pl_hardware *p,
					int vcom_mv);
static int pl_hardware_dac_write(struct pl_hardware *p);

/* I2C helpers */
static int pl_hardware_read_i2c_reg(struct i2c_adapter *i2c, __u8 addr,
				    __u8 reg, void *data, size_t size);
static int pl_hardware_write_i2c_reg8(struct i2c_adapter *i2c, __u8 addr,
				      __u8 reg, __u8 value);
static int pl_hardware_i2c_rdwr(struct i2c_adapter *i2c, __u8 addr,
				__u16 flags, void *data, size_t size);

static int pl_hardware_do_set_temperature(struct pl_hardware *plhw,
					 int temperature);

static int pl_hardware_module_a_init(struct pl_hardware *p);

/* Module A */
static void pl_hardware_free_module_a(struct pl_hardware *p);
#ifdef CONFIG_MODELF_PL_Z1_3
static int pl_hardware_module_a_wait_pok(struct pl_hardware *p);
static int pl_hardware_gpio_switch(struct pl_hardware *p, int gpio, bool on);
#endif

#if defined(CONFIG_MODELF_PL_Z5_0)
/* Hummingbird Z5 hardware */
static int pl_hardware_z50_init(struct pl_hardware *p);
static int pl_hardware_z50_free(struct pl_hardware *p);
#endif

/* ----------------------------------------------------------------------------
 * public interface
 */

struct pl_hardware *pl_hardware_alloc(void)
{
	struct pl_hardware *p;

	p = kmalloc(sizeof(struct pl_hardware), GFP_KERNEL);

	if (p)
		p->init_done = false;

	return p;
}
EXPORT_SYMBOL(pl_hardware_alloc);

int pl_hardware_init(struct pl_hardware *p,
			      const struct pl_hardware_config *config)
{
	int stat;

	if (p->init_done)
		return -EINVAL;

	p->config = config;
	printk("PLHW I2C bus number: %d\n", p->config->i2c_bus_number);

	p->i2c = i2c_get_adapter(p->config->i2c_bus_number);
	if (!p->i2c) {
		printk("PLHW: Failed to get I2C adapter for bus %d\n",
		       p->config->i2c_bus_number);
		stat = -ENXIO;
		goto err_exit;
	}

	p->is_module_a = false;

#ifdef CONFIG_MODELF_PL_Z5_0
	stat = pl_hardware_z50_init(p);
	if (stat) {
		printk("PLHW: Failed to intialise HBZ5 hardware\n");
		goto err_free_i2c;
	}
#endif

#if USE_CPLD
	stat = pl_hardware_cpld_init(p);
#else /* ToDo: also handle CPLD free when other init steps fail... */
	stat = -1;
#endif
	if (stat) {
		printk("PLHW: No CPLD found, assume Module A\n");
		stat = pl_hardware_module_a_init(p);
		if (stat) {
			printk("PLHW: Failed to initialise Module A\n");
#ifdef CONFIG_MODELF_PL_Z5_0
			goto err_free_z50;
#else
			goto err_free_i2c;
#endif
		}
	}

	stat = pl_hardware_hvpmic_init(p);
	if (stat) {
		printk("PLHW: Failed to initialise HVPMIC\n");
#ifdef CONFIG_MODELF_PL_Z5_0
		goto err_free_z50;
#else
		goto err_free_i2c;
#endif
	}

	stat = pl_hardware_dac_init(p);
	if (stat) {
		printk("PLHW: Failed to intialise VCOM DAC\n");
#ifdef CONFIG_MODELF_PL_Z5_0
		goto err_free_z50;
#else
		goto err_free_i2c;
#endif
	}

	/* initialise to a value that temperature sensor can never indicate */
	p->last_temperature = MIN_TEMPERATURE;

	pl_hardware_dac_set_voltage(p, VCOM_DEFAULT);

	printk("PLHW: ready.\n");

	p->init_done = true;
	p->hv_on = false;

	return 0;

#ifdef CONFIG_MODELF_PL_Z5_0
err_free_z50:
	pl_hardware_z50_free();
#endif
err_free_i2c:
	i2c_put_adapter(p->i2c);
err_exit:

	return stat;
}
EXPORT_SYMBOL(pl_hardware_init);

void pl_hardware_free(struct pl_hardware *p)
{
	printk("PLHW: free\n");

	if (p->init_done) {
		i2c_put_adapter(p->i2c);
		pl_hardware_free_module_a(p);
#ifdef CONFIG_MODELF_PL_Z5_0
		pl_hardware_z50_free(p);
#endif
	}

	kfree(p);
}
EXPORT_SYMBOL(pl_hardware_free);

int pl_hardware_set_vcom(struct pl_hardware *p,
				  int vcom_mv)
{
	if ((vcom_mv < VCOM_MIN) || (vcom_mv > VCOM_MAX)) {
		printk("PLHW: VCOM voltage out of range: %dmV "
		       "(range is [%dmV, %dmV]\n",
		       vcom_mv, VCOM_MIN, VCOM_MAX);
		return -EINVAL;
	}

	pl_hardware_dac_set_voltage(p, vcom_mv);

	return 0;
}
EXPORT_SYMBOL(pl_hardware_set_vcom);

#define STEP(cmd, msg) do {				\
		const int stat = (cmd);			\
		if (stat) {				\
			printk("PLHW: "msg" failed\n");	\
			return stat;			\
		}					\
} while (0)

int pl_hardware_enable(struct pl_hardware *p)
{
	if (!p->init_done) {
		printk("PLHW: Not initialised\n");
		return -EINVAL;
	}

	if (p->hv_on)
		return 0;

	if (p->is_module_a == false) {
#if USE_CPLD
		STEP(pl_hardware_cpld_switch(p, CPLD_BPCOM_CLAMP, true),
		     "Clamp BPCOM to 0V");
		STEP(pl_hardware_cpld_switch(p, CPLD_HVEN, true), "HV ON");
#endif
		STEP(pl_hardware_hvpmic_wait_pok(p), "wait for POK");
#if USE_CPLD
		STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_CLOSE, false),
		     "COM open");
		STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_EN, true),
		     "COM enable");
		STEP(pl_hardware_cpld_switch(p, CPLD_COM_PSU, true),
		     "COM PSU on");
		STEP(pl_hardware_dac_write(p), "COM DAC value");
		STEP(pl_hardware_dac_set_power(p, true), "DAC power on");
		STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_CLOSE, true),
		     "COM close");
#endif
	} else {
#ifdef CONFIG_MODELF_PL_Z1_3
		STEP(pl_hardware_gpio_switch(p, GPIO_VCOM_SW_CLOSE, false),
		     "COM open");
		STEP(pl_hardware_gpio_switch(p, GPIO_PMIC_EN, true), "HV ON");
		STEP(pl_hardware_module_a_wait_pok(p), "wait for POK");
#endif
		STEP(pl_hardware_dac_write(p), "COM DAC value");
		STEP(pl_hardware_dac_set_power(p, true), "DAC power on");
#ifdef CONFIG_MODELF_PL_Z1_3
		STEP(pl_hardware_gpio_switch(p, GPIO_VCOM_SW_CLOSE, true),
		     "COM close");
#endif
	}

	p->hv_on = true;

	return 0;
}
EXPORT_SYMBOL(pl_hardware_enable);

int pl_hardware_disable(struct pl_hardware *p)
{
	if (!p->init_done) {
		printk("PLHW: Not initialised\n");
		return -EINVAL;
	}

	if (!p->hv_on)
		return 0;

	if (p->is_module_a == false) {
#if USE_CPLD
		STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_CLOSE, false),
		     "COM open");
		STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_EN, false),
		     "COM disable");
		STEP(pl_hardware_dac_set_power(p, false), "DAC power off");
		STEP(pl_hardware_gpio_switch(p, CPLD_COM_PSU, false),
		     "COM PSU off");
		STEP(pl_hardware_cpld_switch(p, CPLD_HVEN, false), "HV OFF");
#endif
	} else {
#ifndef CONFIG_MODELF_PL_ROBIN
		STEP(pl_hardware_gpio_switch(p, GPIO_VCOM_SW_CLOSE, false),
		     "COM open");
#endif
		STEP(pl_hardware_dac_set_power(p, false), "DAC power off");
#ifndef CONFIG_MODELF_PL_ROBIN
		STEP(pl_hardware_gpio_switch(p, GPIO_PMIC_EN, false),
		     "HV OFF");
#endif
	}

	p->hv_on = false;

	return 0;
}
EXPORT_SYMBOL(pl_hardware_disable);

#undef STEP

/* ----------------------------------------------------------------------------
 * static functions
 */

/* CPLD */

#if USE_CPLD
static int pl_hardware_cpld_init(struct pl_hardware *p)
{
	int stat;

	stat = pl_hardware_cpld_read_data(p);
	if (stat)
		return stat;

	printk("PLHW CPLD version: %d, build: %d, board id: 0x%02X\n",
	       p->cpld.b0.version, p->cpld.b1.build_version,
	       p->cpld.b2.board_id);

	if (p->cpld.b0.version < CPLD_MIN_VERSION) {
		printk("PLHW unsupported CPLD version (min: 0x%02X)\n",
		       CPLD_MIN_VERSION);
		return -ENODEV;
	}

	return 0;
}

static int pl_hardware_cpld_switch(struct pl_hardware *p,
				   enum pl_hardware_cpld_switch sw, bool on)
{
	switch (sw) {
	case CPLD_HVEN:         p->cpld.b0.cpld_hven     = on ? 1 : 0;  break;
	case CPLD_COM_SW_EN:    p->cpld.b1.vcom_sw_en    = on ? 1 : 0;  break;
	case CPLD_COM_SW_CLOSE: p->cpld.b1.vcom_sw_close = on ? 1 : 0;  break;
	case CPLD_COM_PSU:      p->cpld.b1.vcom_psu_en   = on ? 1 : 0;  break;
	case CPLD_BPCOM_CLAMP:  p->cpld.b0.bpcom_clamp   = on ? 1 : 0;  break;
	default:
		printk("PLHW: invalid switch identifier\n");
		return -EINVAL;
	}

	return pl_hardware_cpld_write_data(p);
}

static int pl_hardware_cpld_read_data(struct pl_hardware *p)
{
	int stat;

	stat = pl_hardware_i2c_rdwr(p->i2c, CPLD_I2C_ADDRESS, I2C_M_RD,
				 p->cpld.data, CPLD_NB_BYTES);
	if (stat)
		printk("PLHW: Failed to read CPLD data\n");

	return stat;
}

static int pl_hardware_cpld_write_data(struct pl_hardware *p)
{
	int stat;

	stat = pl_hardware_i2c_rdwr(p->i2c, CPLD_I2C_ADDRESS, 0,
				 p->cpld.data, CPLD_NB_BYTES);
	if (stat)
		printk("PLHW: Failed to write CPLD data\n");

	return 0;
}
#endif /* USE_CPLD */

/* HVPMIC */

int pl_hardware_hvpmic_init(struct pl_hardware *p)
{
	u8 timings[HVPMIC_NB_TIMINGS];
	int stat;

	timings[0] = HVPMIC_TIMING_UP_VGNEG;
	timings[1] = HVPMIC_TIMING_UP_VSNEG;
	timings[2] = HVPMIC_TIMING_UP_VSPOS;
	timings[3] = HVPMIC_TIMING_UP_VGPOS;
	timings[4] = HVPMIC_TIMING_DOWN_VGPOS;
	timings[5] = HVPMIC_TIMING_DOWN_VSPOS;
	timings[6] = HVPMIC_TIMING_DOWN_VSNEG;
	timings[7] = HVPMIC_TIMING_DOWN_VGNEG;

	stat = pl_hardware_read_i2c_reg(p->i2c, HVPMIC_I2C_ADDRESS,
					HVPMIC_REG_PROD_REV,
					&p->hvpmic.prod_rev, 1);
	if (stat)
		return stat;

	stat = pl_hardware_read_i2c_reg(p->i2c, HVPMIC_I2C_ADDRESS,
					HVPMIC_REG_PROD_ID,
					&p->hvpmic.prod_id, 1);
	if (stat)
		return stat;

	memcpy(p->hvpmic.timings, timings, HVPMIC_NB_TIMINGS);

	printk("PLHW: HVPMIC rev 0x%02X, id 0x%02X\n",
	       p->hvpmic.prod_rev, p->hvpmic.prod_id);
	printk("PLHW timings on: %d, %d, %d, %d, "
	       "timings off: %d, %d, %d, %d\n",
	       timings[0], timings[1], timings[2], timings[3],
	       timings[4], timings[5], timings[6], timings[7]);

	return pl_hardware_hvpmic_load_timings(p);
}

static int pl_hardware_hvpmic_load_timings(struct pl_hardware *p)
{
	__u8 reg;
	int i;

	for (i = 0, reg = HVPMIC_REG_TIMING_1;
	     i < HVPMIC_NB_TIMINGS;
	     ++i, ++reg) {
		int stat;

		stat = pl_hardware_write_i2c_reg8(p->i2c, HVPMIC_I2C_ADDRESS,
						  reg, p->hvpmic.timings[i]);
		if (stat)
			return stat;
	}

	return 0;
}

static int pl_hardware_hvpmic_wait_pok(struct pl_hardware *p)
{
	static const unsigned POLL_DELAY_MS = 5;
	unsigned timeout = 100;
	int pok = 0;
	int stat = 0;

	while (!pok) {
		union hvpmic_fault fault;

		mdelay(POLL_DELAY_MS);

		stat = pl_hardware_read_i2c_reg(
			p->i2c, HVPMIC_I2C_ADDRESS,
			HVPMIC_REG_FAULT, &fault.byte, 1);
		if (stat) {
			printk("PLHW: failed to read HVPMIC POK\n");
			break;
		}

		pok = fault.pok;

		if (timeout > POLL_DELAY_MS) {
			timeout -= POLL_DELAY_MS;
		} else {
			timeout = 0;

			if (!pok) {
				printk("PLHW: POK timeout\n");
				stat = -ETIME;
				break;
			}
		}
	}

	return stat;
}

/* VCOM DAC */

#if 0
static int pl_hardware_dac_set_power(struct pl_hardware *p,
				     bool on);
#endif

static int pl_hardware_dac_init(struct pl_hardware *p)
{
	p->vcom_dac.i2c_address = p->config->dac_i2c_address;
	p->vcom_dac.dac_value = 0;

	return 0;
}

static int pl_hardware_dac_set_power(struct pl_hardware *p,
				     bool on)
{
	union dac5820_ext_payload payload;

	payload.cmd_byte.cmd = DAC5820_CMD_EXT__DATA_0;
	payload.cmd_byte.data_high = 0;
	payload.ext_byte.reserved = 0;
	payload.ext_byte.a = 1;
	payload.ext_byte.b = 0;
	payload.ext_byte.pd = on ? DAC5820_PD_ON : DAC5820_PD_OFF_100K;

	return pl_hardware_i2c_rdwr(p->i2c, p->vcom_dac.i2c_address, 0,
				    payload.bytes, sizeof payload);
}

static void pl_hardware_dac_set_voltage(struct pl_hardware *p,
					int vcom_mv)
{
	long dac_value;
	int round;

	dac_value = (vcom_mv * VCOM_COEF_INT);
	dac_value += (vcom_mv * VCOM_COEF_DEC / 1000);
	dac_value += VCOM_OFFSET;
	round = ((dac_value % 1000) > 500) ? 1 : 0;
	dac_value /= 1000;
	dac_value += round;

	if (dac_value < 0)
		p->vcom_dac.dac_value = 0;
	else if (dac_value > 0xFF)
		p->vcom_dac.dac_value = 0xFF;
	else
		p->vcom_dac.dac_value = dac_value;

	p->vcom_dac.vcom_mv = vcom_mv;

	printk("PLHW: VCOM DAC %dmV -> %d\n",
	       p->vcom_dac.vcom_mv, p->vcom_dac.dac_value);
}

static int pl_hardware_dac_write(struct pl_hardware *p)
{
	union dac5820_write_payload payload;

	payload.cmd_byte.cmd = DAC5820_CMD_LOAD_IN_DAC_A__UP_DAC_B__OUT_AB;
	payload.cmd_byte.data_high = (p->vcom_dac.dac_value >> 4) & 0xF;
	payload.data_byte.data_low = p->vcom_dac.dac_value & 0xF;
	payload.data_byte.reserved = 0;

	return pl_hardware_i2c_rdwr(p->i2c, p->vcom_dac.i2c_address, 0,
				    payload.bytes, sizeof payload);
}

/* I2C helpers */

static int pl_hardware_read_i2c_reg(struct i2c_adapter *i2c, __u8 addr,
				    __u8 reg, void *data, size_t size)
{
	struct i2c_msg msgs[2] = {
		{
			.addr = addr,
			.flags = 0,
			.len = 1,
			.buf = &reg
		},
		{
			.addr = addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = (__u8 *) data
		}
	};

	const size_t n_msgs = ARRAY_SIZE(msgs);
	const int n_xfer = i2c_transfer(i2c, msgs, n_msgs);

	if (n_xfer != n_msgs) {
		printk("PLHW: I2C reg read error (addr 0x%02X, reg 0x%02X, "
		       "size %i)\n",
		       addr, reg, size);
		return -EIO;
	}

	return 0;
}

static int pl_hardware_write_i2c_reg8(struct i2c_adapter *i2c, __u8 addr,
				   __u8 reg, __u8 value)
{
	__u8 w_data[2] = {reg, value};

	struct i2c_msg msg = {
		.addr = addr,
		.flags = 0,
		.len = 2,
		.buf = w_data,
	};

	const int n_xfer = i2c_transfer(i2c, &msg, 1);

	if (n_xfer != 1) {
		printk("PLHW: I2C write reg failed (addr 0x%02X, reg 0x%02X, "
		       "value 0x%02X)\n", addr, reg, value);
		return -EIO;
	}

	return 0;
}

static int pl_hardware_i2c_rdwr(struct i2c_adapter *i2c, __u8 addr,
				__u16 flags, void *data, size_t size)
{
	struct i2c_msg msg = {
		.addr = addr,
		.flags = flags,
		.len = size,
		.buf = (__u8 *) data
	};

	const int n_xfer = i2c_transfer(i2c, &msg, 1);

	if (n_xfer != 1) {
		printk("PLHW: I2C error (0x%02X, flags 0x%04X, size %i)\n",
		       addr, flags, size);
		return -EIO;
	}

	return 0;
}

s32 pl_hardware_constrain_temperature_range(s32 temperature)
{
        struct temperature_set* set;
        s32 constrained_temp;

        set = get_vcom_temperature_set();
        constrained_temp = constrain_temp_to_set(set, temperature, "VCom");

        return constrained_temp;
}
EXPORT_SYMBOL(pl_hardware_constrain_temperature_range);

int pl_hardware_set_temperature(struct pl_hardware *plhw,
					 int temperature)
{
        int stat = 0;
        struct temperature_set* set;

        set = get_vcom_temperature_set();

        /* assume that temperature is already constrained to a valid value */
        if ((temperature != plhw->last_temperature)
	    || temperature_set_changed(set)) {
        	pr_info("Temperature now %d degrees C\n", temperature);

        	stat = pl_hardware_do_set_temperature(plhw, temperature);
	}

        return stat;
}
EXPORT_SYMBOL(pl_hardware_set_temperature);

int pl_hardware_refresh_current_vcom(struct pl_hardware *plhw)
{
	return pl_hardware_do_set_temperature(plhw, plhw->last_temperature);
}
EXPORT_SYMBOL(pl_hardware_refresh_current_vcom);

static int pl_hardware_do_set_temperature(struct pl_hardware *plhw,
					 int temperature)
{
        struct temperature_set* set;
        s32 vcom_mv;
	int ret;

        set = get_vcom_temperature_set();
        vcom_mv = temperature_set_lookup_int(set, temperature);

	ret = pl_hardware_set_vcom(plhw, vcom_mv);
        plhw->last_temperature = temperature;

	return ret;
}

int pl_hardware_is_module_a(const struct pl_hardware *plhw)
{
	return plhw->is_module_a;
}
EXPORT_SYMBOL(pl_hardware_is_module_a);

static int pl_hardware_module_a_init(struct pl_hardware *p)
{
#ifdef CONFIG_MODELF_PL_ROBIN
	p->is_module_a = true;

	return 0;
#else
	int retval;

	p->is_module_a = true;

	if (gpio_request(GPIO_POK, "POK") < 0) {
		printk(KERN_ERR "POK (GPIO %d) is busy.\n", GPIO_POK);
		retval = -EBUSY;
		goto error_ret;
	}
	gpio_direction_input(GPIO_POK);

	if (gpio_request(GPIO_PMIC_EN, "PMIC_EN") < 0) {
		printk(KERN_ERR "PMIC_EN (GPIO %d) is busy.\n", GPIO_PMIC_EN);
		retval = -EBUSY;
		goto free_pok;
	}
	gpio_direction_output(GPIO_PMIC_EN, 0);

	if (gpio_request(GPIO_VCOM_SW_CLOSE, "VCOM_SW_CLOSE") < 0) {
		printk(KERN_ERR "VCOM_SW_CLOSE (GPIO %d) is busy.\n",
		       GPIO_VCOM_SW_CLOSE);
		retval = -EBUSY;
		goto free_pmic_en;
	}
	gpio_direction_output(GPIO_VCOM_SW_CLOSE, 0);

	return 0;

free_pmic_en:
	gpio_free(GPIO_PMIC_EN);
free_pok:
	gpio_free(GPIO_POK);
error_ret:
	return retval;
#endif
}

static void pl_hardware_free_module_a(struct pl_hardware *p)
{
	if (p->is_module_a) {
#ifndef CONFIG_MODELF_PL_ROBIN
		gpio_free(GPIO_VCOM_SW_CLOSE);
		gpio_free(GPIO_PMIC_EN);
		gpio_free(GPIO_POK);
#endif
	}
}

#ifdef CONFIG_MODELF_PL_Z1_3
static int pl_hardware_module_a_wait_pok(struct pl_hardware *p)
{
	static const unsigned POLL_DELAY_MS = 5;
	unsigned timeout = 100;
	int pok = 0;
	int stat = 0;

	while (!pok) {
		mdelay(POLL_DELAY_MS);

		pok = gpio_get_value(GPIO_POK);

		if (timeout > POLL_DELAY_MS) {
			timeout -= POLL_DELAY_MS;
		} else {
			timeout = 0;

			if (!pok) {
				printk("PLHW: POK timeout\n");
				stat = -ETIME;
				break;
			}
		}
	}

	return stat;
}

static int pl_hardware_gpio_switch(struct pl_hardware *p, int gpio, bool on)
{
	gpio_set_value(gpio, (on ? 1 : 0));

	return 0;
}
#endif

#ifdef CONFIG_MODELF_PL_Z5_0

static const struct gpio z50_io_init[] = {
	{ GPIO_SOL_EP_SELECT,GPIOF_IN,            "SOL_EP_SELECT"   },
	{ GPIO_I2C_ENABLE,   GPIOF_OUT_INIT_HIGH, "GPIO_I2C_ENABLE" },
	{ GPIO_3V3_ENABLE,   GPIOF_OUT_INIT_HIGH, "GPIO_3V3_ENABLE" },
	{ GPIO_VPP_SOLOMON,  GPIOF_OUT_INIT_LOW,  "GPIO_VPP_SOLOMON"},
	{ GPIO_GPIO1, GPIOF_OUT_INIT_LOW,  "GPIO_GPIO1" },
	{ GPIO_GPIO2, GPIOF_OUT_INIT_HIGH, "GPIO_GPIO2" },
	{ GPIO_GPIO3, GPIOF_OUT_INIT_LOW,  "GPIO_GPIO3" },
	{ GPIO_LED1,  GPIOF_OUT_INIT_LOW,  "GPIO_LED1"  },
	{ GPIO_LED2,  GPIOF_OUT_INIT_HIGH, "GPIO_LED2"  },
	{ GPIO_LED3,  GPIOF_OUT_INIT_LOW,  "GPIO_LED3"  },
	{ GPIO_LED4,  GPIOF_OUT_INIT_HIGH, "GPIO_LED4"  },
};

static int pl_hardware_z50_init(struct pl_hardware *p)
{
	int retval;
	int i;
	const struct gpio *io = z50_io_init;

	for (i = 0; i < ARRAY_SIZE(z50_io_init); i++, io++) {

		printk(KERN_INFO "PLHW: Requesting: %s (GPIO %d).\n",
		       io->label, io->gpio);

		if (gpio_request_one(io->gpio, io->flags, io->label) < 0) {
			printk(KERN_ERR "PLHW: %s (GPIO %d) is busy.\n",
			       io->label, io->gpio);
			retval = -EBUSY;
			goto error_ret;
		}
	}

	return 0;

error_ret:
	while (i--) {
		printk(KERN_INFO "PLHW: Backing out: %s (GPIO %d).\n",
		       io->label, io->gpio);
		gpio_free((--io)->gpio);
	}

	return retval;
}

static int pl_hardware_z50_free(struct pl_hardware *p)
{
	gpio_free_array(z50_io_init, ARRAY_SIZE(z50_io_init));

	return 0;
}

#endif /* CONFIG_MODELF_PL_Z5_0 */

MODULE_AUTHOR("Guillaume Tucker <guillaume.tucker@plasticlogic.com");
MODULE_DESCRIPTION("Plastic Logic E-Paper hardware control");
MODULE_LICENSE("GPL");

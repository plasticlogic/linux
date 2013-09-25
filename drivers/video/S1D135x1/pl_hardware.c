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
#define MAX17135_I2C_ADDRESS 0x48 /* Maxim MAX17135 HVPMIC */
#define TPS65185_I2C_ADDRESS 0x68 /* TI TPS65185 HVPMIC */

/* CPLD parameters */
#define CPLD_MIN_VERSION 0x01
#define CPLD_NB_BYTES 3

/* DAC */
#define DAC5820_CMD_LOAD_IN_DAC_A__UP_DAC_B__OUT_AB 0x0
#define DAC5820_CMD_EXT__DATA_0 0xF

/* ADC */
#define ADC11607_NB_RESULTS 4
#define ADC11607_SEL_INT_REF_ON 0x1
#define ADC11607_SEL_EXT_REF 0x2
#define ADC11607_SEL_INT_REF 0x4
#define ADC11607_SEL_AIN__REF_OUT 0x2
#define ADC11607_MAX_VALUE 0x3FF
#define ADC11607_INVALID_RESULT ((ADC11607_MAX_VALUE) + 1)

/* VCOM */
#define VCOM_MAX 13000
#define VCOM_MIN 1000
#define VCOM_DEFAULT ((VCOM_MAX + VCOM_MIN) / 2)
#define VCOM_REF_ADC_CHANNEL 0
#define VCOM_FB_ADC_CHANNEL 1
#define VCOM_ADC_SCALE 10
#define VCOM_VREF_MUL 24200
#define VCOM_VREF_DIV 332
#define VCOM_VGSWING 57089 /* Note: on other systems it's 70000 */
#define VCOM_CORR_I 110

/* MAX17135 HVPMIC timings */
#define MAX17135_NB_TIMINGS 8
#define MAX17135_TIMING_UP_VGNEG 8
#define MAX17135_TIMING_UP_VSNEG 2
#define MAX17135_TIMING_UP_VSPOS 11
#define MAX17135_TIMING_UP_VGPOS 3
#define MAX17135_TIMING_DOWN_VGPOS 0
#define MAX17135_TIMING_DOWN_VSPOS 0
#define MAX17135_TIMING_DOWN_VSNEG 0
#define MAX17135_TIMING_DOWN_VGNEG 0

/* TPS65185 HVPMIC */
#define TPS65185_PWRDOWN_DELAY 4

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

/* MAX17135 definitions */

enum pl_hardware_max17135_register {
	MAX17135_REG_EXT_TEMP   = 0x00,
	MAX17135_REG_CONF       = 0x01,
	MAX17135_REG_INT_TEMP   = 0x04,
	MAX17135_REG_TEMP_STAT  = 0x05,
	MAX17135_REG_PROD_REV   = 0x06,
	MAX17135_REG_PROD_ID    = 0x07,
	MAX17135_REG_DVR        = 0x08,
	MAX17135_REG_ENABLE     = 0x09,
	MAX17135_REG_FAULT      = 0x0A,
	MAX17135_REG_PROG       = 0x0C,
	MAX17135_REG_TIMING_1   = 0x10,
	MAX17135_REG_TIMING_2   = 0x11,
	MAX17135_REG_TIMING_3   = 0x12,
	MAX17135_REG_TIMING_4   = 0x13,
	MAX17135_REG_TIMING_5   = 0x14,
	MAX17135_REG_TIMING_6   = 0x15,
	MAX17135_REG_TIMING_7   = 0x16,
	MAX17135_REG_TIMING_8   = 0x17,
};

union max17135_fault {
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

struct pl_hardware_max17135 {
	__u8 prod_id;
	__u8 prod_rev;
	__u8 timings[MAX17135_NB_TIMINGS];
};

/* TI TPS65185 HVPMIC definitions */

enum tps65185_register {
	TPS65185_REG_TMST_VALUE = 0x00,
	TPS65185_REG_ENABLE     = 0x01,
	TPS65185_REG_VADJ       = 0x02,
	TPS65185_REG_VCOM1      = 0x03,
	TPS65185_REG_VCOM2      = 0x04,
	TPS65185_REG_INT_EN1    = 0x05,
	TPS65185_REG_INT_EN2    = 0x06,
	TPS65185_REG_INT1       = 0x07,
	TPS65185_REG_INT2       = 0x08,
	TPS65185_REG_UPSEQ0     = 0x09,
	TPS65185_REG_UPSEQ1     = 0x0A,
	TPS65185_REG_DWNSEQ0    = 0x0B,
	TPS65185_REG_DWNSEQ1    = 0x0C,
	TPS65185_REG_TMST1      = 0x0D,
	TPS65185_REG_TMST2      = 0x0E,
	TPS65185_REG_PG_STAT    = 0x0F,
	TPS65185_REG_REV_ID     = 0x10,
	TPS65185_REG_MAX
};

union tps65185_version {
	struct {
		char version:4;
		char minor:2;
		char major:2;
	};
	u8 byte;
};

struct pl_hardware_tps65185 {
	union tps65185_version ver;
};

/* DAC definitions */

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

struct pl_hardware_dac {
	__u8 i2c_address;
};

/* ADC definitions */

struct adc11607_setup {
	char reserved:1;
	char reset:1;
	char bip_uni:1;
	char clk_sel:1;
	char sel:3;
	char setup_1:1;
};

struct adc11607_config {
	char se_diff:1;
	char cs:4;
	char scan:2;
	char config_0:1;
};

union adc11607_setup_config {
	struct {
		struct adc11607_setup setup;
		struct adc11607_config config;
	};
	char bytes[2];
};

struct pl_hardware_adc {
	__u8 i2c_address;
	union adc11607_setup_config cmd;
	unsigned nb_channels;
	unsigned ref_mv;
	int current_channel;
	unsigned results[ADC11607_NB_RESULTS];
};

/* VCOM definitions */

struct pl_hardware_vcom {
	int ref_mv;
	int target_mv;
	int vref_mv;
	int last_v_in;
	int swing;
	bool dac_measured;
	int dac_offset;
	int dac_dx;
	int dac_dy;
	int dac_step_mv;
};

/* Module A GPIO pins */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#if defined(CONFIG_MODELF_PL_Z1_3)
#define GPIO_POK		GPIO_TO_PIN(0, 15)
#define GPIO_PMIC_EN		GPIO_TO_PIN(3, 21)
#define GPIO_VCOM_SW_CLOSE	GPIO_TO_PIN(0, 14)
#elif defined(CONFIG_MODELF_PL_Z5)
#define GPIO_POK		GPIO_TO_PIN(0, 15)
#define GPIO_PMIC_EN		GPIO_TO_PIN(2, 25)
#define GPIO_VCOM_SW_CLOSE	GPIO_TO_PIN(0, 14)
#define GPIO_I2C_ENABLE		GPIO_TO_PIN(1 , 4)
#define GPIO_VPP_SOLOMON	GPIO_TO_PIN(1, 29)
#define GPIO_SOL_EP_SELECT	GPIO_TO_PIN(2, 24)
#elif defined(CONFIG_MODELF_PL_Z6_Z7)
#define GPIO_POK		GPIO_TO_PIN(0, 15)
#define GPIO_PMIC_EN		GPIO_TO_PIN(1, 14)
#define GPIO_VCOM_SW_CLOSE	GPIO_TO_PIN(0, 14)
#define GPIO_PMIC_FLT		GPIO_TO_PIN(0, 27)
#define GPIO_RESERVE1		GPIO_TO_PIN(0, 22)
#define GPIO_RESERVE2		GPIO_TO_PIN(1, 13)
#define GPIO_SEL1		GPIO_TO_PIN(1, 12)
#define GPIO_SEL2		GPIO_TO_PIN(0, 23)
#define GPIO_SEL3		GPIO_TO_PIN(0, 31)
#define GPIO_SEL4		GPIO_TO_PIN(0, 30)
#endif

/* Opaque public instance structure */

struct pl_hardware {
	bool init_done;
	const struct pl_hardware_config *config;
	struct i2c_adapter *i2c;
#if USE_CPLD
	union pl_hardware_cpld cpld;
#endif
	struct pl_hardware_max17135 max17135;
	struct pl_hardware_tps65185 tps65185;
	struct pl_hardware_dac dac;
	struct pl_hardware_adc adc;
	struct pl_hardware_vcom vcom;
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

/* MAX17135 HVPMIC */
static int pl_hardware_max17135_init(struct pl_hardware *p);
static int pl_hardware_max17135_load_timings(struct pl_hardware *p);
#if USE_CPLD
static int pl_hardware_max17135_wait_pok(struct pl_hardware *p);
#endif

/* TPS65185 HVPMIC */
static int pl_hardware_tps65185_init(struct pl_hardware *p);

/* DAC */
static int pl_hardware_dac_init(struct pl_hardware *p);
static int pl_hardware_dac_set_power(struct pl_hardware *p,
				     bool on);
#if !defined(CONFIG_MODELF_PL_Z6_Z7)
static int pl_hardware_dac_write(struct pl_hardware *p, __u8 value);
#endif

/* ADC */
#if !defined(CONFIG_MODELF_PL_ROBIN) && !defined(CONFIG_MODELF_PL_Z6_Z7)
static int pl_hardware_adc_init(struct pl_hardware *p, unsigned init_channel);
static void pl_hardware_adc_set_nb_channels(struct pl_hardware_adc *adc);
static int pl_hardware_adc_select_channel(struct pl_hardware *p,
					  unsigned channel);
static void pl_hardware_adc_invalidate(struct pl_hardware_adc *adc);
static int pl_hardware_adc_read_results(struct pl_hardware *p);
static int pl_hardware_adc_get_mv(struct pl_hardware *p);
static int pl_hardware_adc_read_mv(struct pl_hardware *p,
				   unsigned channel, int *value);
#endif

/* VCOM calibration */
static int pl_hardware_vcomcal_init(struct pl_hardware *p);
static int pl_hardware_vcomcal_set_vcom(struct pl_hardware *p);
#if !defined(CONFIG_MODELF_PL_ROBIN) && !defined(CONFIG_MODELF_PL_Z6_Z7)
static int pl_hardware_vcomcal_measure_dac(struct pl_hardware *p);
static int pl_hardware_vcomcal_get_dac_value(struct pl_hardware *p,
					     int vcom_mv);
static void pl_hardware_vcomcal_scale_vcom(struct pl_hardware *p);
#endif

/* I2C helpers */
static int pl_hardware_read_i2c_reg(struct i2c_adapter *i2c, __u8 addr,
				    __u8 reg, void *data, size_t size);
static int pl_hardware_write_i2c_reg8(struct i2c_adapter *i2c, __u8 addr,
				      __u8 reg, __u8 value);
static int pl_hardware_i2c_rdwr(struct i2c_adapter *i2c, __u8 addr,
				__u16 flags, void *data, size_t size);

static int pl_hardware_do_set_temperature(struct pl_hardware *plhw,
					 int temperature);

/* GPIO control */
static int pl_hardware_gpio_init(struct pl_hardware *p);
static void pl_hardware_gpio_free(struct pl_hardware *p);
#ifndef CONFIG_MODELF_PL_ROBIN
static int pl_hardware_gpio_wait_pok(struct pl_hardware *p);
static int pl_hardware_gpio_switch(struct pl_hardware *p, int gpio, bool on);
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
	static int (*hvpmic_init[])(struct pl_hardware *) = {
		[PLHW_HVPMIC_MAX17135] = &pl_hardware_max17135_init,
		[PLHW_HVPMIC_TPS65185] = &pl_hardware_tps65185_init,
	};
	int stat;

	if (p->init_done)
		return -EINVAL;

	p->config = config;
	printk("PLHW: I2C bus number: %d\n", p->config->i2c_bus_number);

	p->i2c = i2c_get_adapter(p->config->i2c_bus_number);
	if (!p->i2c) {
		printk("PLHW: Failed to get I2C adapter for bus %d\n",
		       p->config->i2c_bus_number);
		stat = -ENXIO;
		goto err_exit;
	}

	p->is_module_a = false;

#if USE_CPLD
	stat = pl_hardware_cpld_init(p);
#else /* ToDo: also handle CPLD free when other init steps fail... */
	stat = -1;
#endif
	if (stat) {
		printk("PLHW: No CPLD found, assume Module A\n");
		stat = pl_hardware_gpio_init(p);
		if (stat) {
			printk("PLHW: Failed to initialise Module A\n");
			goto err_free_i2c;
		}
	}

	stat = hvpmic_init[config->hvpmic_id](p);
	if (stat) {
		printk("PLHW: Failed to initialise HVPMIC\n");
		goto err_free_all;
	}

	if (config->hvpmic_id == PLHW_HVPMIC_MAX17135) {
		stat = pl_hardware_dac_init(p);
		if (stat) {
			printk("PLHW: Failed to intialise VCOM DAC\n");
			goto err_free_all;
		}
	}

#if !defined(CONFIG_MODELF_PL_ROBIN) && !defined(CONFIG_MODELF_PL_Z6_Z7)
	stat = pl_hardware_adc_init(p, VCOM_FB_ADC_CHANNEL);
	if (stat) {
		printk("PLHW: Failed to initialise ADC\n");
		goto err_free_all;
	}
#endif

	stat = pl_hardware_vcomcal_init(p);
	if (stat) {
		printk("PLHW: Failed to initialise VCOM calibration\n");
		goto err_free_all;
	}

	/* initialise to a value that temperature sensor can never indicate */
	p->last_temperature = MIN_TEMPERATURE;

	printk("PLHW: ready.\n");

	p->init_done = true;
	p->hv_on = false;

	return 0;

err_free_all:
	pl_hardware_gpio_free(p);
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
		pl_hardware_gpio_free(p);
#ifdef CONFIG_MODELF_PL_Z6_Z7 /* it takes time to turn the power off */
		mdelay(150);
#endif
	}

	kfree(p);
}
EXPORT_SYMBOL(pl_hardware_free);

int pl_hardware_set_vcom(struct pl_hardware *p, int vcom_mv)
{
	if ((vcom_mv < VCOM_MIN) || (vcom_mv > VCOM_MAX)) {
		printk("PLHW: VCOM voltage out of range: %dmV "
		       "(range is [%dmV, %dmV]\n",
		       vcom_mv, VCOM_MIN, VCOM_MAX);
		return -EINVAL;
	}

	p->vcom.ref_mv = vcom_mv;

#if !defined(CONFIG_MODELF_PL_ROBIN) && !defined(CONFIG_MODELF_PL_Z6_Z7)
	pl_hardware_vcomcal_scale_vcom(p);
#endif

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
		STEP(pl_hardware_max17135_wait_pok(p), "wait for POK");
		STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_CLOSE, false),
		     "COM open");
		STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_EN, true),
		     "COM enable");
		STEP(pl_hardware_cpld_switch(p, CPLD_COM_PSU, true),
		     "COM PSU on");
		STEP(pl_hardware_dac_set_power(p, true), "DAC power on");
		STEP(pl_hardware_vcomcal_set_vcom(p), "VCOM calibration");
		STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_CLOSE, true),
		     "COM close");
#endif
	} else {
#ifndef CONFIG_MODELF_PL_ROBIN
		STEP(pl_hardware_gpio_switch(p, GPIO_VCOM_SW_CLOSE, false),
		     "COM open");
		STEP(pl_hardware_gpio_switch(p, GPIO_PMIC_EN, true), "HV ON");
		STEP(pl_hardware_gpio_wait_pok(p), "wait for POK");
#endif
		if (p->config->hvpmic_id == PLHW_HVPMIC_MAX17135)
			STEP(pl_hardware_dac_set_power(p, true),
			     "DAC power on");
		STEP(pl_hardware_vcomcal_set_vcom(p), "VCOM calibration");
#ifndef CONFIG_MODELF_PL_ROBIN
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
		if (p->config->hvpmic_id == PLHW_HVPMIC_MAX17135)
			STEP(pl_hardware_dac_set_power(p, false),
			     "DAC power off");
#ifndef CONFIG_MODELF_PL_ROBIN
		STEP(pl_hardware_gpio_switch(p, GPIO_PMIC_EN, false),
		     "HV OFF");
#endif
	}

	if (p->config->hvpmic_id == PLHW_HVPMIC_TPS65185)
		/* The TPS65185 PMIC has a configurable delay before powering down.
		 * Add a delay here to give the PMIC a chance to power down
		 * before continuing */
		mdelay(TPS65185_PWRDOWN_DELAY); 
	p->hv_on = false;

	return 0;
}
EXPORT_SYMBOL(pl_hardware_disable);

bool pl_hardware_is_enabled(struct pl_hardware *p)
{
	return p->hv_on;
}
EXPORT_SYMBOL(pl_hardware_is_enabled);

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

/* MAX17135 HVPMIC */

int pl_hardware_max17135_init(struct pl_hardware *p)
{
	u8 timings[MAX17135_NB_TIMINGS];
	int stat;

	timings[0] = MAX17135_TIMING_UP_VGNEG;
	timings[1] = MAX17135_TIMING_UP_VSNEG;
	timings[2] = MAX17135_TIMING_UP_VSPOS;
	timings[3] = MAX17135_TIMING_UP_VGPOS;
	timings[4] = MAX17135_TIMING_DOWN_VGPOS;
	timings[5] = MAX17135_TIMING_DOWN_VSPOS;
	timings[6] = MAX17135_TIMING_DOWN_VSNEG;
	timings[7] = MAX17135_TIMING_DOWN_VGNEG;

	stat = pl_hardware_read_i2c_reg(p->i2c, MAX17135_I2C_ADDRESS,
					MAX17135_REG_PROD_REV,
					&p->max17135.prod_rev, 1);
	if (stat)
		return stat;

	stat = pl_hardware_read_i2c_reg(p->i2c, MAX17135_I2C_ADDRESS,
					MAX17135_REG_PROD_ID,
					&p->max17135.prod_id, 1);
	if (stat)
		return stat;

	memcpy(p->max17135.timings, timings, MAX17135_NB_TIMINGS);

	printk("PLHW: MAX17135 rev 0x%02X, id 0x%02X\n",
	       p->max17135.prod_rev, p->max17135.prod_id);
	printk("PLHW: timings on: %d, %d, %d, %d, "
	       "timings off: %d, %d, %d, %d\n",
	       timings[0], timings[1], timings[2], timings[3],
	       timings[4], timings[5], timings[6], timings[7]);

	return pl_hardware_max17135_load_timings(p);
}

static int pl_hardware_max17135_load_timings(struct pl_hardware *p)
{
	__u8 reg;
	int i;

	for (i = 0, reg = MAX17135_REG_TIMING_1;
	     i < MAX17135_NB_TIMINGS;
	     ++i, ++reg) {
		int stat;

		stat = pl_hardware_write_i2c_reg8(p->i2c, MAX17135_I2C_ADDRESS,
						  reg, p->max17135.timings[i]);
		if (stat)
			return stat;
	}

	return 0;
}

#if USE_CPLD
static int pl_hardware_max17135_wait_pok(struct pl_hardware *p)
{
	static const unsigned POLL_DELAY_MS = 5;
	unsigned timeout = 100;
	int pok = 0;
	int stat = 0;

	while (!pok) {
		union max17135_fault fault;

		mdelay(POLL_DELAY_MS);

		stat = pl_hardware_read_i2c_reg(
			p->i2c, MAX17135_I2C_ADDRESS,
			MAX17135_REG_FAULT, &fault.byte, 1);
		if (stat) {
			printk("PLHW: failed to read MAX17135 POK\n");
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
#endif

/* TPS65185 HVPMIC */

static int pl_hardware_tps65185_init(struct pl_hardware *p)
{
	struct reg_data {
		u8 reg;
		u8 data;
	};
	static const struct reg_data init_data[] = {
		{ TPS65185_REG_ENABLE,     0x00 },
		{ TPS65185_REG_VADJ,       0x03 },
		{ TPS65185_REG_VCOM1,      0x00 },
		{ TPS65185_REG_VCOM2,      0x00 },
		{ TPS65185_REG_INT_EN1,    0x00 },
		{ TPS65185_REG_INT_EN2,    0x00 },
		{ TPS65185_REG_UPSEQ0,     0x78 },
		{ TPS65185_REG_UPSEQ1,     0x00 },
		{ TPS65185_REG_DWNSEQ0,    0x00 },
		{ TPS65185_REG_DWNSEQ1,    0x00 },
		{ TPS65185_REG_TMST1,      0x00 },
		{ TPS65185_REG_TMST2,      0x78 }
	};
	struct pl_hardware_tps65185 *pdata = &p->tps65185;
	int stat;
	int i;

	mdelay(10);

	stat = pl_hardware_read_i2c_reg(p->i2c, TPS65185_I2C_ADDRESS,
					TPS65185_REG_REV_ID,
					&pdata->ver.byte, 1);
	if (stat)
		return stat;

	printk("PLHW: TPS65185 version: %d.%d.%d\n",
	       pdata->ver.major, pdata->ver.minor, pdata->ver.version);

	for (i = 0; i < ARRAY_SIZE(init_data); i++) {
		stat = pl_hardware_write_i2c_reg8(p->i2c, TPS65185_I2C_ADDRESS,
						  init_data[i].reg,
						  init_data[i].data);
		if (stat) {
			printk("PLHW: Failed to write to TPS65185 reg %02X\n",
			       init_data[i].reg);
			return stat;
		}
	}

	return 0;
}

/* DAC */

static int pl_hardware_dac_init(struct pl_hardware *p)
{
	p->dac.i2c_address = p->config->dac_i2c_address;

	return 0;
}

static int pl_hardware_dac_set_power(struct pl_hardware *p, bool on)
{
	union dac5820_ext_payload payload;

	payload.cmd_byte.cmd = DAC5820_CMD_EXT__DATA_0;
	payload.cmd_byte.data_high = 0;
	payload.ext_byte.reserved = 0;
	payload.ext_byte.a = 1;
	payload.ext_byte.b = 0;
	payload.ext_byte.pd = on ? DAC5820_PD_ON : DAC5820_PD_OFF_100K;

	return pl_hardware_i2c_rdwr(p->i2c, p->dac.i2c_address, 0,
				    payload.bytes, sizeof payload);
}

#if !defined(CONFIG_MODELF_PL_Z6_Z7)
static int pl_hardware_dac_write(struct pl_hardware *p, __u8 value)
{
	union dac5820_write_payload payload;

	payload.cmd_byte.cmd = DAC5820_CMD_LOAD_IN_DAC_A__UP_DAC_B__OUT_AB;
	payload.cmd_byte.data_high = (value >> 4) & 0xF;
	payload.data_byte.data_low = value & 0xF;
	payload.data_byte.reserved = 0;

	return pl_hardware_i2c_rdwr(p->i2c, p->dac.i2c_address, 0,
				    payload.bytes, sizeof payload);
}
#endif

/* ADC */

#if !defined(CONFIG_MODELF_PL_ROBIN) && !defined(CONFIG_MODELF_PL_Z6_Z7)
static int pl_hardware_adc_init(struct pl_hardware *p, unsigned init_channel)
{
	struct adc11607_setup * const setup = &p->adc.cmd.setup;
	struct adc11607_config * const config = &p->adc.cmd.config;

	p->adc.i2c_address = p->config->adc_i2c_address;
	p->adc.ref_mv = 2048;
	p->adc.current_channel = init_channel;

	setup->reset = 1;
	setup->bip_uni = 0;
	setup->clk_sel = 0;
	setup->setup_1 = 1;
	setup->sel = ADC11607_SEL_INT_REF | ADC11607_SEL_INT_REF_ON;

	config->se_diff = 1;
	config->cs = init_channel;
	config->scan = 0;
	config->config_0 = 0;

	pl_hardware_adc_set_nb_channels(&p->adc);

	if (init_channel >= p->adc.nb_channels) {
		printk("PLHW: Invalid ADC channel number\n");
		return -1;
	}

	pl_hardware_adc_invalidate(&p->adc);

	return pl_hardware_i2c_rdwr(p->i2c, p->adc.i2c_address, 0,
				    p->adc.cmd.bytes, sizeof p->adc.cmd);
}

static void pl_hardware_adc_set_nb_channels(struct pl_hardware_adc *adc)
{
	unsigned n;

	if (!adc->cmd.config.se_diff)
		n = 2;
	else if ((adc->cmd.setup.sel == 2) || (adc->cmd.setup.sel == 3))
		n = 3;
	else
		n = 4;

	adc->nb_channels = n;
}

static int pl_hardware_adc_select_channel(struct pl_hardware *p,
					  unsigned channel)
{
	if (channel >= p->adc.nb_channels) {
		printk("PLHW: Invalid ADC channel number\n");
		return -1;
	}

	if (channel == p->adc.current_channel)
		return 0;

	pl_hardware_adc_invalidate(&p->adc);
	p->adc.cmd.config.cs = channel;
	p->adc.current_channel = channel;

	return pl_hardware_i2c_rdwr(p->i2c, p->adc.i2c_address, 0,
				    &p->adc.cmd.config,
				    sizeof p->adc.cmd.config);
}

static void pl_hardware_adc_invalidate(struct pl_hardware_adc *adc)
{
	int i;

	for (i = 0; i < ADC11607_NB_RESULTS; ++i)
		adc->results[i] = ADC11607_INVALID_RESULT;
}

static int pl_hardware_adc_read_results(struct pl_hardware *p)
{
	char data[32];
	size_t n;
	size_t read_n;
	const char *it;
	unsigned i;
	int stat;

	n = p->adc.cmd.config.cs + 1;
	read_n = n * 2;

	stat = pl_hardware_i2c_rdwr(p->i2c, p->adc.i2c_address, I2C_M_RD,
				    data, read_n);
	if (stat) {
		printk("PLHW: Failed to read ADC results\n");
		pl_hardware_adc_invalidate(&p->adc);
		return stat;
	}

	for (it = data, i = 0; i < n; ++i, it += 2)
		p->adc.results[i] = ((it[0] & 0x03) << 8) | it[1];

	for (i = n; i < ADC11607_NB_RESULTS; ++i)
		p->adc.results[i] = ADC11607_INVALID_RESULT;

	return 0;
}

static int pl_hardware_adc_get_mv(struct pl_hardware *p)
{
	int adc_mv;

	adc_mv = p->adc.results[p->adc.current_channel] * p->adc.ref_mv;

	return DIV_ROUND_CLOSEST(adc_mv, ADC11607_MAX_VALUE);
}

static int pl_hardware_adc_read_mv(struct pl_hardware *p,
				   unsigned channel, int *value)
{
	int stat;

	stat = pl_hardware_adc_select_channel(p, channel);
	if (stat)
		return stat;

	mdelay(2);

	stat = pl_hardware_adc_read_results(p);
	if (stat)
		return stat;

	*value = pl_hardware_adc_get_mv(p);

	return 0;
}
#endif /* !CONFIG_MODELF_PL_ROBIN */

/* VCOM calibration */

static int pl_hardware_vcomcal_init(struct pl_hardware *p)
{
	p->vcom.ref_mv = VCOM_DEFAULT;
	p->vcom.dac_measured = false;

	return 0;
}

#if defined(CONFIG_MODELF_PL_Z6_Z7)
static int pl_hardware_vcomcal_set_vcom(struct pl_hardware *p)
{
	static const int VCOM_DIV_MV = 33; /* 33mV per DAC step */
	const int dac_value = p->vcom.ref_mv / VCOM_DIV_MV;
	const u8 reg1 = dac_value & 0xFF;
	const u8 reg2 = (dac_value >> 8) & 0x01;
	int stat;

	stat = pl_hardware_write_i2c_reg8(p->i2c, TPS65185_I2C_ADDRESS,
					  TPS65185_REG_VCOM1, reg1);
	if (stat)
		return stat;

	return pl_hardware_write_i2c_reg8(p->i2c, TPS65185_I2C_ADDRESS,
					  TPS65185_REG_VCOM2, reg2);
}
#elif defined(CONFIG_MODELF_PL_ROBIN)
static int pl_hardware_vcomcal_set_vcom(struct pl_hardware *p)
{
	static const int VCOM_COEF_INT = 20;
	static const int VCOM_COEF_DEC = 779;
	static const int VCOM_OFFSET = -19532;
	long dac_value;

	dac_value = (p->vcom.ref_mv * VCOM_COEF_INT);
	dac_value += (p->vcom.ref_mv * VCOM_COEF_DEC) / 1000;
	dac_value += VCOM_OFFSET;
	dac_value = DIV_ROUND_CLOSEST(dac_value, 1000);

	if (dac_value < 0)
		dac_value = 0;
	else if (dac_value > 0xFF)
		dac_value = 0xFF;
	else
		dac_value = dac_value;

	return pl_hardware_dac_write(p, (u8)dac_value);
}
#else
static int pl_hardware_vcomcal_set_vcom(struct pl_hardware *p)
{
	int v_in;
	int delta_stop;
	int delta;
	int n;
	int stat;

	if (!p->vcom.dac_measured) {
		stat = pl_hardware_vcomcal_measure_dac(p);
		if (stat)
			return stat;

		pl_hardware_vcomcal_scale_vcom(p);
	}

	delta_stop = p->vcom.dac_step_mv * 6 / 5; /* give 20% margin */
	delta = 0;
	v_in = p->vcom.last_v_in;
	n = 10;

	do {
		int dac_val;
		int v_out;

		dac_val = pl_hardware_vcomcal_get_dac_value(p, v_in);

		stat = pl_hardware_dac_write(p,  dac_val);
		if (stat)
			return stat;

		stat = pl_hardware_adc_read_mv(p, VCOM_FB_ADC_CHANNEL, &v_out);
		if (stat)
			return stat;

		v_out *= VCOM_ADC_SCALE;
		delta = p->vcom.target_mv - v_out;
		p->vcom.last_v_in = v_in;
		v_in += delta * VCOM_CORR_I / 100;
	} while ((abs(delta) > delta_stop) && --n);

	if (!n) {
		printk("PLHW: VCOMCAL Failed to converge in time\n");
		return -1;
	}

	return 0;
}

static int pl_hardware_vcomcal_measure_dac(struct pl_hardware *p)
{
	const int x1 = 85;
	const int x2 = 170;
	int y1;
	int y2;
	int vref;
	int stat;

	stat = pl_hardware_dac_write(p, x1);
	if (stat)
		return -1;

	stat = pl_hardware_adc_read_mv(p, VCOM_FB_ADC_CHANNEL, &y1);
	if (stat)
		return -1;

	stat = pl_hardware_dac_write(p, x2);
	if (stat)
		return -1;

	stat = pl_hardware_adc_read_mv(p, VCOM_FB_ADC_CHANNEL, &y2);
	if (stat)
		return -1;

	y1 *= VCOM_ADC_SCALE;
	y2 *= VCOM_ADC_SCALE;

	p->vcom.dac_dx = x2 - x1;
	p->vcom.dac_dy = y2 - y1;

	if (!p->vcom.dac_dx || !p->vcom.dac_dy) {
		printk("PLHW: Failed to measure DAC characteristics\n");
		return -1;
	}

	p->vcom.dac_offset = y1 - ((x1 * p->vcom.dac_dy) / p->vcom.dac_dx);
	p->vcom.dac_step_mv = p->vcom.dac_dy / p->vcom.dac_dx;

	printk("PLHW: DAC dx: %d, dy: %d, offset: %d, step: %d\n",
	       p->vcom.dac_dx, p->vcom.dac_dy, p->vcom.dac_offset,
	       p->vcom.dac_step_mv);

	stat = pl_hardware_adc_read_mv(p, VCOM_REF_ADC_CHANNEL, &vref);
	if (stat)
		return stat;

	p->vcom.swing = vref * VCOM_VREF_MUL;
	p->vcom.swing = DIV_ROUND_CLOSEST(p->vcom.swing, VCOM_VREF_DIV);
	p->vcom.dac_measured = true;

	return 0;
}

static int pl_hardware_vcomcal_get_dac_value(struct pl_hardware *p,
					     int vcom_mv)
{
	long dac_value;

	dac_value = (vcom_mv - p->vcom.dac_offset) * p->vcom.dac_dx;
	dac_value = DIV_ROUND_CLOSEST(dac_value, p->vcom.dac_dy);

	if (dac_value < 0)
		dac_value = 0;
	else if (dac_value > 0xFF)
		dac_value = 0xFF;

	return dac_value;
}

static void pl_hardware_vcomcal_scale_vcom(struct pl_hardware *p)
{
	if (!p->vcom.dac_measured)
		return;

	p->vcom.target_mv = p->vcom.ref_mv * p->vcom.swing / VCOM_VGSWING;
	p->vcom.last_v_in = p->vcom.target_mv;
	printk("PLHW: scaled VCOM %d -> %d (swing: %d) mV\n",
	       p->vcom.ref_mv, p->vcom.target_mv, p->vcom.swing);
}
#endif /* !CONFIG_MODELF_PL_ROBIN */

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

#ifdef CONFIG_MODELF_PL_ROBIN
static int pl_hardware_gpio_init(struct pl_hardware *p)
{
	p->is_module_a = true;

	return 0;
}
#else
static const struct gpio pl_hardware_gpio_init_data[] = {
	{ GPIO_POK,           GPIOF_IN,            "GPIO_POK"              },
	{ GPIO_PMIC_EN,       GPIOF_OUT_INIT_LOW,  "GPIO_PMIC_EN"          },
	{ GPIO_VCOM_SW_CLOSE, GPIOF_OUT_INIT_LOW,  "GPIO_VCOM_SW_CLOSE"    },
#if defined(CONFIG_MODELF_PL_Z5)
	{ GPIO_I2C_ENABLE,    GPIOF_OUT_INIT_HIGH, "GPIO_I2C_ENABLE"       },
	{ GPIO_VPP_SOLOMON,   GPIOF_OUT_INIT_LOW,  "GPIO_VPP_SOLOMON"      },
	{ GPIO_SOL_EP_SELECT, GPIOF_IN,            "GPIO_SOL_EP_SELECT"    },
#elif defined(CONFIG_MODELF_PL_Z6_Z7)
	{ GPIO_RESERVE1,      GPIOF_OUT_INIT_HIGH, "GPIO_RESERVE1"         },
	{ GPIO_RESERVE2,      GPIOF_OUT_INIT_HIGH, "GPIO_RESERVE2"         },
#endif
};

static int pl_hardware_gpio_init(struct pl_hardware *p)
{
	int stat;
	int i;

	p->is_module_a = true;

	for (i = 0; i < ARRAY_SIZE(pl_hardware_gpio_init_data); i++) {
		const struct gpio *gpio = &pl_hardware_gpio_init_data[i];

		printk(KERN_INFO "PLHW: Requesting: %s (GPIO %d).\n",
		       gpio->label, gpio->gpio);

		stat = gpio_request_one(gpio->gpio, gpio->flags, gpio->label);
		if (stat) {
			printk(KERN_ERR "PLHW: %s (GPIO %d) is busy.\n",
			       gpio->label, gpio->gpio);
			goto error_ret;
		}
	}

	return 0;

error_ret:
	while (i--) {
		const struct gpio *gpio = &pl_hardware_gpio_init_data[i];

		printk(KERN_INFO "PLHW: Backing out: %s (GPIO %d).\n",
		       gpio->label, gpio->gpio);
		gpio_free(gpio->gpio);
	}

	return stat;
}
#endif

static void pl_hardware_gpio_free(struct pl_hardware *p)
{
#ifndef CONFIG_MODELF_PL_ROBIN
	int i;

	for (i = 0; i < ARRAY_SIZE(pl_hardware_gpio_init_data); i++)
		gpio_free(pl_hardware_gpio_init_data[i].gpio);
#endif
}

#ifndef CONFIG_MODELF_PL_ROBIN
static int pl_hardware_gpio_wait_pok(struct pl_hardware *p)
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
#endif /* !CONFIG_MODELF_PL_ROBIN */

MODULE_AUTHOR("Guillaume Tucker <guillaume.tucker@plasticlogic.com>");
MODULE_DESCRIPTION("Plastic Logic E-Paper hardware control");
MODULE_LICENSE("GPL");

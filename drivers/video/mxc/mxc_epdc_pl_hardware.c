/*
 * mxc_epdc_pl_hardware.c -- Plastic Logic e-paper display power control
 *
 * Copyright (C) 2012 Plastic Logic Limited
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
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mxc_epdc_pl_hardware.h>


/* CPLD */
static int pl_hardware_cpld_init(struct mxc_epdc_pl_hardware *p);
static void pl_hardware_cpld_free(struct mxc_epdc_pl_hardware *p);
static int pl_hardware_cpld_switch(struct mxc_epdc_pl_hardware *p,
				   enum pl_hardware_cpld_switch sw, bool on);
static int pl_hardware_cpld_read_data(struct mxc_epdc_pl_hardware *p);
static int pl_hardware_cpld_write_data(struct mxc_epdc_pl_hardware *p);
static void pl_hardware_cpld_write_fast_data(struct mxc_epdc_pl_hardware *p);

static inline int pl_hardware_cpld_sel_psu(struct mxc_epdc_pl_hardware *p,
					   struct pl_hardware_psu *psu)
{
	if (p->fast_cpld.alt_i2c == psu->i2c_sel_id)
		return 0;

	return pl_hardware_cpld_switch(p, CPLD_ALT_I2C, psu->i2c_sel_id);
}

/* HVPMIC */
static int pl_hardware_hvpmic_init(struct mxc_epdc_pl_hardware *p,
				   struct pl_hardware_psu *psu);
static int pl_hardware_hvpmic_load_timings(struct mxc_epdc_pl_hardware *p,
					   struct pl_hardware_psu *psu);
static int pl_hardware_hvpmic_wait_pok(struct mxc_epdc_pl_hardware *p,
				       struct pl_hardware_psu *psu);

/* DAC */
static int pl_hardware_dac_init(struct mxc_epdc_pl_hardware *p,
				struct pl_hardware_psu *psu);
static int pl_hardware_dac_set_power(struct mxc_epdc_pl_hardware *p,
				     bool on, struct pl_hardware_psu *psu);
static int pl_hardware_dac_write(struct mxc_epdc_pl_hardware *p,
				 struct pl_hardware_psu *psu, __u8 value);

/* ADC */
static int pl_hardware_adc_init(struct mxc_epdc_pl_hardware *p,
				struct pl_hardware_psu *psu,
				unsigned init_channel);
static void pl_hardware_adc_set_nb_channels(struct pl_hardware_adc *adc);
static int pl_hardware_adc_select_channel(struct mxc_epdc_pl_hardware *p,
					  struct pl_hardware_psu *psu,
					  unsigned channel);
static void pl_hardware_adc_invalidate(struct pl_hardware_adc *adc);
static int pl_hardware_adc_read_results(struct mxc_epdc_pl_hardware *p,
					struct pl_hardware_psu *psu);
static int pl_hardware_adc_get_mv(struct pl_hardware_psu *psu);
static int pl_hardware_adc_read_mv(struct mxc_epdc_pl_hardware *p,
				   struct pl_hardware_psu *psu,
				   unsigned channel, int *value);

/* VCOM calibration */
static int pl_hardware_vcomcal_init(struct mxc_epdc_pl_hardware *p,
				    struct pl_hardware_psu *psu);
static int pl_hardware_vcomcal_set_vcom(struct mxc_epdc_pl_hardware *p,
					struct pl_hardware_psu *psu);
static int pl_hardware_vcomcal_measure_dac(struct mxc_epdc_pl_hardware *p,
					   struct pl_hardware_psu *psu);
static int pl_hardware_vcomcal_get_dac_value(struct mxc_epdc_pl_hardware *p,
					     struct pl_hardware_psu *psu,
					     int vcom_mv);

/* I2C helpers */
static int pl_hardware_read_i2c_reg(struct i2c_adapter *i2c, __u8 addr,
				    __u8 reg, void *data, size_t size);
static int pl_hardware_write_i2c_reg8(struct i2c_adapter *i2c, __u8 addr,
				      __u8 reg, __u8 value);
static int pl_hardware_i2c_rdwr(struct i2c_adapter *i2c, __u8 addr,
				__u16 flags, void *data, size_t size);

static inline int pl_hardware_psu_read_i2c_reg(
	struct mxc_epdc_pl_hardware *p, struct pl_hardware_psu *psu,
	__u8 addr, __u8 reg, void *data, size_t size)
{
	return pl_hardware_read_i2c_reg(p->i2c, addr, reg, data, size);
}

static inline int pl_hardware_psu_write_i2c_reg8(
	struct mxc_epdc_pl_hardware *p, struct pl_hardware_psu *psu,
	__u8 addr, __u8 reg, __u8 value)
{
	return pl_hardware_write_i2c_reg8(p->i2c, addr, reg, value);
}

static inline int pl_hardware_psu_i2c_rdwr(
	struct mxc_epdc_pl_hardware *p, struct pl_hardware_psu *psu,
	__u8 addr, __u16 flags, void *data, size_t size)
{
	return pl_hardware_i2c_rdwr(p->i2c, addr, flags, data, size);
}

/* ----------------------------------------------------------------------------
 * public interface
 */

struct mxc_epdc_pl_hardware *mxc_epdc_pl_hardware_alloc(void)
{
	struct mxc_epdc_pl_hardware *p;

	p = kmalloc(sizeof(struct mxc_epdc_pl_hardware), GFP_KERNEL);
	
	if (p)
		p->init_done = false;

	return p;
}
EXPORT_SYMBOL(mxc_epdc_pl_hardware_alloc);

int mxc_epdc_pl_hardware_init(struct mxc_epdc_pl_hardware *p,
			      struct mxc_epdc_plhw_pdata *pdata,
			      const struct mxc_epdc_plhw_config *conf)
{
	int stat;
	int i;
	if (p->init_done){
		return -EINVAL;
	}
	if (!conf->psu_n || (conf->psu_n > 2)) {
		printk("PLHW: Invalid number of PSUs: %d "
		       "(min is 1, max is 2)\n", conf->psu_n);
		return -EINVAL;
	}

	p->pdata = pdata;
	p->conf = conf;
	p->i2c = i2c_get_adapter(p->pdata->i2c_bus_number);
	if (!p->i2c) {
		printk("PLHW: Failed to get I2C adapter for bus %d\n",
		       p->pdata->i2c_bus_number);
		stat = -ENXIO;
		goto err_exit;
	}
	
	if(!(pdata->use_minimal_cpld)){
		stat = pl_hardware_cpld_init(p);
		if (stat) {
			printk("PLHW: Failed to initialise CPLD\n");
			goto err_free_i2c;
		}
}
	
	for (i = 0; i < conf->psu_n; ++i) {
		struct pl_hardware_psu *psu = &p->psu[i];

		psu->i2c_sel_id = i;
		stat = pl_hardware_hvpmic_init(p, psu);
		if (stat) {
			printk("PLHW: Failed to initialise HVPMIC\n");
			goto err_free_cpld;
		}
		stat = pl_hardware_dac_init(p, psu);
		if (stat) {
			printk("PLHW: Failed to intialise DAC\n");
			goto err_free_cpld;
		}
		stat = pl_hardware_adc_init(p, psu, VCOM_FB_ADC_CHANNEL);
		if (stat) {
			printk("PLHW: Failed to initialise ADC\n");
			goto err_free_cpld;
		}
		stat = pl_hardware_vcomcal_init(p, psu);
		if (stat) {
			printk("PLHW: Failed to initialise VCOM calibration\n");
			goto err_free_cpld;
		}
	}

	printk("PLHW: Ready, number of PSUs: %d\n", conf->psu_n);

	p->init_done = true;

	return 0;
err_free_cpld:
	if(!(pdata->use_minimal_cpld))
		pl_hardware_cpld_free(p);
err_free_i2c:
	i2c_put_adapter(p->i2c);
err_exit:
	printk("PLHW: mxc_epdc_pl_hardware_init: Error\n");
	return stat;
}
EXPORT_SYMBOL(mxc_epdc_pl_hardware_init);

void mxc_epdc_pl_hardware_free(struct mxc_epdc_pl_hardware *p)
{
	if (p->init_done) {
		pl_hardware_cpld_free(p);
		i2c_put_adapter(p->i2c);
	}

	kfree(p);
}
EXPORT_SYMBOL(mxc_epdc_pl_hardware_free);

int mxc_epdc_pl_hardware_set_vcom(struct mxc_epdc_pl_hardware *p,
				  const int *vcoms_mv)
{
	int i = 0;

	for (i = 0; i < p->conf->psu_n; ++i) {
		const int vcom = vcoms_mv[i];
		printk("PLHW: VCOM %i: %i\n", i, vcom);
		if ((vcom < VCOM_MIN) || (vcom > VCOM_MAX)) {
			printk("PLHW: VCOM voltage out of range: %dmV "
			       "(range is [%dmV, %dmV])\n",
			       vcom, VCOM_MIN, VCOM_MAX);
			return -EINVAL;
		}
		
		p->psu[i].vcom.reference_mv = vcom;
	}

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_pl_hardware_set_vcom);

#define STEP(cmd, msg) do {				\
		const int stat = (cmd);			\
		if (stat) {				\
			printk("PLHW: "msg" failed\n");	\
			return stat;			\
		} 						\
} while (0)

int mxc_epdc_pl_hardware_enable(struct mxc_epdc_pl_hardware *p)
{
	struct pl_hardware_psu *psu;

	if (!p->init_done)
		return -EINVAL;
	psu = &p->psu[0];
	STEP(pl_hardware_cpld_switch(p, CPLD_HVEN, true), "HV ON");
	STEP(pl_hardware_hvpmic_wait_pok(p, psu), "wait for POK");
	STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_EN, true), "COM enable");
	STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_CLOSE, false), "COM open");
	STEP(pl_hardware_cpld_switch(p, CPLD_COM_PSU, true), "COM PSU on");
	STEP(pl_hardware_dac_set_power(p, true, psu), "DAC power on");
	STEP(pl_hardware_vcomcal_set_vcom(p, psu), "VCOM calibration");
	STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_CLOSE, true),"COM close");

	if (p->conf->psu_n == 1)
		return 0;

	psu = &p->psu[1];
	STEP(pl_hardware_cpld_switch(p, CPLD_HVEN1, true), "HV1 on");
	STEP(pl_hardware_hvpmic_wait_pok(p, psu), "wait for POK");
	STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_CLOSE1,false),"COM1 open");
	STEP(pl_hardware_dac_set_power(p, true, psu), "DAC1 power on");
	STEP(pl_hardware_vcomcal_set_vcom(p, psu), "VCOM1 calibration");
	STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_CLOSE1,true),"COM1 close");
	return 0;
}
EXPORT_SYMBOL(mxc_epdc_pl_hardware_enable);

int mxc_epdc_pl_hardware_disable(struct mxc_epdc_pl_hardware *p)
{
	struct pl_hardware_psu *psu;

	if (!p->init_done)
		return -EINVAL;
	psu = &p->psu[0];
	STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_CLOSE, false),"COM open");
	STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_EN, false), "COM disable");
	STEP(pl_hardware_dac_set_power(p, false, psu), "DAC power off");
	STEP(pl_hardware_cpld_switch(p, CPLD_COM_PSU, false), "COM PSU off");
	STEP(pl_hardware_cpld_switch(p, CPLD_HVEN, false), "HV OFF");
	
	if (p->conf->psu_n == 1)
		return 0;

	psu = &p->psu[1];
	STEP(pl_hardware_cpld_switch(p, CPLD_COM_SW_CLOSE1,false),"COM1 open");
	STEP(pl_hardware_dac_set_power(p, false, psu), "DAC1 power off");
	STEP(pl_hardware_cpld_switch(p, CPLD_HVEN1, false), "HV1 OFF");
	return 0;
}
EXPORT_SYMBOL(mxc_epdc_pl_hardware_disable);

#undef STEP

/* ----------------------------------------------------------------------------
 * static functions
 */

/* CPLD */

static int pl_hardware_cpld_init(struct mxc_epdc_pl_hardware *p)
{
	static char *gpio_name[MXC_EPDC_PL_HARDWARE_GPIO_N] = {
		[MXC_EPDC_PL_HARDWARE_FAST_D0]  = "plhw_fast_d0",
		[MXC_EPDC_PL_HARDWARE_FAST_D1]  = "plhw_fast_d1",
		[MXC_EPDC_PL_HARDWARE_FAST_D2]  = "plhw_fast_d2",
		[MXC_EPDC_PL_HARDWARE_FAST_CLK] = "plhw_fast_clk",
		[MXC_EPDC_PL_HARDWARE_FAST_EN]  = "plhw_fast_en",
	};
	int stat;
	int i;
	//printk("PLHW: pl_hardware_cpld_init\n");
	stat = pl_hardware_cpld_read_data(p);
	if (stat)
		return stat;

	printk("PLHW: CPLD version: %d, build: %d, board id: 0x%02X\n",
	       p->cpld.b0.version, p->cpld.b1.build_version,
	       p->cpld.b2.board_id);

	if (p->cpld.b0.version != CPLD_REQ_VERSION) {
		printk("PLHW: unsupported CPLD version "
		       "(required: 0x%02X found: 0x%02X)\n",
		       CPLD_REQ_VERSION, p->cpld.b0.version);
		return -ENODEV;
	}

	if (p->conf->source_2bpp_conversion)
		printk("PLHW: Enabling 2bpp conversion mode\n");

	stat = pl_hardware_cpld_switch(p, CPLD_SOURCE_2BPP,
			p->conf->source_2bpp_conversion);

	if (stat)
		return stat;

	if (p->conf->interlaced_gates) 
		printk("PLHW: Enabling interlaced gates\n");

	stat = pl_hardware_cpld_switch(p, CPLD_PING_PONG, 
			p->conf->interlaced_gates);

	if (stat)
		return stat;

	if (p->conf->source_cs_logic)
		printk("PLHW: Enabling source chip select logic\n");

	stat = pl_hardware_cpld_switch(p, CPLD_SRC_CS_LOGIC,
			p->conf->source_cs_logic);

	if (stat)
		return stat;

	for (i = 0; i < MXC_EPDC_PL_HARDWARE_GPIO_N; ++i) {
		printk("PLHW: Fast GPIO init: %s %d\n",
		       gpio_name[i], p->pdata->fast_gpio[i]);

		stat = gpio_request(p->pdata->fast_gpio[i], gpio_name[i]);
		if (stat) {
			printk("PLHW: Failed to request GPIO\n");
			break;
		}

		stat = gpio_direction_output(p->pdata->fast_gpio[i], 0);
		if (stat) {
			printk("PLHW: Failed to initialise GPIO\n");
			gpio_free(p->pdata->fast_gpio[i]);
			break;
		}
	}

	if (stat) {
		while (--i >= 0)
			gpio_free(p->pdata->fast_gpio[i]);

		return stat;
	}

	stat = pl_hardware_cpld_switch(p, CPLD_ALT_I2C, 0);
	if (stat)
		return stat;

	return 0;
}

static void pl_hardware_cpld_free(struct mxc_epdc_pl_hardware *p)
{
	int i;
	//printk("PLHW: pl_hardware_cpld_free\n");
	for (i = 0; i < MXC_EPDC_PL_HARDWARE_GPIO_N; ++i)
		gpio_free(p->pdata->fast_gpio[i]);
}

static int pl_hardware_cpld_switch(struct mxc_epdc_pl_hardware *p,
				   enum pl_hardware_cpld_switch sw, bool on)
{
	int stat = 0;
	if(p->pdata->use_minimal_cpld){
		switch (sw) {

		case CPLD_HVEN:{
			//p->cpld.b0.cpld_hven     = on ? 1 : 0;  
			gpio_set_value(EPDC_HVEN, on ? 1 : 0);
			break;
		}
		case CPLD_COM_SW_EN:{
			//p->cpld.b1.vcom_sw_en    = on ? 1 : 0;  
			gpio_set_value(EPDC_VCOM_SW_CLOSE, on ? 1 : 0);
			break;
		}
		case CPLD_COM_SW_CLOSE:{
			//p->cpld.b1.vcom_sw_close = on ? 1 : 0;  
			gpio_set_value(EPDC_VCOM_SW_EN, on ? 1 : 0);
			break;
		 }
		case CPLD_COM_PSU: {
			//p->cpld.b1.vcom_psu_en   = on ? 1 : 0; 
			gpio_set_value(EPDC_VCOM_PSU_EN, on ? 1 : 0); 
			break;
		}
		default:
			printk("PLHW: Invalid switch identifier\n");
			return -EINVAL;
		}
	}else{
		switch (sw) {
		case CPLD_HVEN:         p->cpld.b0.cpld_hven     = on ? 1 : 0;  break;
		case CPLD_COM_SW_EN:    p->cpld.b1.vcom_sw_en    = on ? 1 : 0;  break;
		case CPLD_COM_SW_CLOSE: p->cpld.b1.vcom_sw_close = on ? 1 : 0;  break;
		case CPLD_COM_PSU:      p->cpld.b1.vcom_psu_en   = on ? 1 : 0;  break;
		case CPLD_SRC_CS_LOGIC: p->cpld.b0.src_cs_logic  = on ? 1 : 0;  break;
		case CPLD_HVEN1:        p->cpld.b2.cpld_hven1    = on ? 1 : 0;  break;
		case CPLD_COM_SW_CLOSE1: p->cpld.b2.vcom_sw_close1 = on ? 1 : 0; break;
		case CPLD_PING_PONG:    p->cpld.b2.ping_pong     = on ? 1 : 0;  break;
		case CPLD_SOURCE_2BPP:  p->cpld.b2.source_2bpp   = on ? 1 : 0;  break;
		case CPLD_ALT_I2C:      p->fast_cpld.alt_i2c     = on ? 1 : 0;  break;
		default:
			printk("PLHW: Invalid switch identifier\n");
			return -EINVAL;
		
		}
		if (sw == CPLD_ALT_I2C) {
			pl_hardware_cpld_write_fast_data(p);
			stat = 0;
		} else {
			stat = pl_hardware_cpld_write_data(p);
		}

	}	
	return stat;
	
}


static int pl_hardware_cpld_read_data(struct mxc_epdc_pl_hardware *p)
{
	int stat;
	//printk("PLHW: pl_hardware_cpld_read_data\n");
	stat = pl_hardware_i2c_rdwr(p->i2c, CPLD_I2C_ADDRESS, I2C_M_RD,
				    p->cpld.data, CPLD_NB_BYTES);
	if (stat)
		printk("PLHW: Failed to read CPLD data\n");

	return stat;
}

static int pl_hardware_cpld_write_data(struct mxc_epdc_pl_hardware *p)
{
	int stat;
	//printk("PLHW: pl_hardware_cpld_write_data\n");
	stat = pl_hardware_i2c_rdwr(p->i2c, CPLD_I2C_ADDRESS, 0,
				    p->cpld.data, CPLD_NB_BYTES);
	if (stat)
		printk("PLHW: Failed to write CPLD data\n");

	return 0;
}

static void pl_hardware_cpld_write_fast_data(struct mxc_epdc_pl_hardware *p)
{
	const int *gpio = p->pdata->fast_gpio;

	gpio_set_value(gpio[MXC_EPDC_PL_HARDWARE_FAST_EN], 1);

	gpio_set_value(gpio[MXC_EPDC_PL_HARDWARE_FAST_D0],
		       p->fast_cpld.alt_i2c);
	gpio_set_value(gpio[MXC_EPDC_PL_HARDWARE_FAST_D1], 0);
	gpio_set_value(gpio[MXC_EPDC_PL_HARDWARE_FAST_D2], 0);
	gpio_set_value(gpio[MXC_EPDC_PL_HARDWARE_FAST_CLK], 1);
	gpio_set_value(gpio[MXC_EPDC_PL_HARDWARE_FAST_CLK], 0);

	gpio_set_value(gpio[MXC_EPDC_PL_HARDWARE_FAST_EN], 0);

	gpio_set_value(gpio[MXC_EPDC_PL_HARDWARE_FAST_D0], 0);
	gpio_set_value(gpio[MXC_EPDC_PL_HARDWARE_FAST_D1], 0);
	gpio_set_value(gpio[MXC_EPDC_PL_HARDWARE_FAST_D2], 0);
}


/* HVPMIC */

int pl_hardware_hvpmic_init(struct mxc_epdc_pl_hardware *p,
			    struct pl_hardware_psu *psu)
{
	static const u8 timings_table
		[MXC_EPDC_PL_HARDWARE_SEQ_N][HVPMIC_NB_TIMINGS] = {
		[MXC_EPDC_PL_HARDWARE_SEQ_0] = {
		/* on:  GN  SN  SP  GP   off: GP  SP  SN  GN */
			24,  7,  2, 12,        7, 14, 12,  2
		},
		[MXC_EPDC_PL_HARDWARE_SEQ_1] = {
		/* on:  GN  SN  SP  GP   off: GP  SP  SN  GN */
			12,  7,  2, 23,        2, 14, 12,  7
		},
	};
	const u8 *timings;
	int stat;

	stat = pl_hardware_psu_read_i2c_reg(p, psu, HVPMIC_I2C_ADDRESS,
					    HVPMIC_REG_PROD_REV,
					    &psu->hvpmic.prod_rev, 1);
	if (stat)
		return stat;

	stat = pl_hardware_psu_read_i2c_reg(p, psu, HVPMIC_I2C_ADDRESS,
					    HVPMIC_REG_PROD_ID,
					    &psu->hvpmic.prod_id, 1);
	if (stat)
		return stat;

	if (p->conf->power_seq >= MXC_EPDC_PL_HARDWARE_SEQ_N) {
		return -EINVAL;
	}

	timings = timings_table[p->conf->power_seq];

	memcpy(psu->hvpmic.timings, timings, HVPMIC_NB_TIMINGS);

	printk("PLHW: HVPMIC rev 0x%02X, id 0x%02X\n",
	       psu->hvpmic.prod_rev, psu->hvpmic.prod_id);
	printk("PLHW: Timings on: %d, %d, %d, %d, "
	       "timings off: %d, %d, %d, %d\n",
	       timings[0], timings[1], timings[2], timings[3],
	       timings[4], timings[5], timings[6], timings[7]);

	return pl_hardware_hvpmic_load_timings(p, psu);
}

static int pl_hardware_hvpmic_load_timings(struct mxc_epdc_pl_hardware *p,
					   struct pl_hardware_psu *psu)
{
	__u8 reg;
	int i;
	for (i = 0, reg = HVPMIC_REG_TIMING_1;
	     i < HVPMIC_NB_TIMINGS;
	     ++i, ++reg) {
		int stat;
	
		stat = pl_hardware_psu_write_i2c_reg8(
			p, psu, HVPMIC_I2C_ADDRESS, reg,
			psu->hvpmic.timings[i]);
		if (stat)
			return stat;
	}
	
	return 0;
}

static int pl_hardware_hvpmic_wait_pok(struct mxc_epdc_pl_hardware *p,
				       struct pl_hardware_psu *psu)
{
	static const unsigned POLL_DELAY_MS = 5;
	unsigned timeout = 100;
	int pok = 0;
	int stat = 0;
	while (!pok) {
		union hvpmic_fault fault;

		mdelay(POLL_DELAY_MS);

		stat = pl_hardware_psu_read_i2c_reg(
			p, psu, HVPMIC_I2C_ADDRESS,
			HVPMIC_REG_FAULT, &fault.byte, 1);
		if (stat) {
			printk("PLHW: Failed to read HVPMIC POK\n");
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

/* DAC */

static int pl_hardware_dac_init(struct mxc_epdc_pl_hardware *p,
				struct pl_hardware_psu *psu)
{
	psu->dac.i2c_address = p->pdata->dac_i2c_address;
	return 0;
}

static int pl_hardware_dac_set_power(struct mxc_epdc_pl_hardware *p,
				     bool on, struct pl_hardware_psu *psu)
{
	union dac5820_ext_payload payload;

	payload.cmd_byte.cmd = DAC5820_CMD_EXT__DATA_0;
	payload.cmd_byte.data_high = 0;
	payload.ext_byte.reserved = 0;
	payload.ext_byte.a = 1;
	payload.ext_byte.b = 0;
	payload.ext_byte.pd = on ? DAC5820_PD_ON : DAC5820_PD_OFF_100K;

	return pl_hardware_psu_i2c_rdwr(p, psu, psu->dac.i2c_address, 0,
					payload.bytes, sizeof payload);
}

static int pl_hardware_dac_write(struct mxc_epdc_pl_hardware *p,
				 struct pl_hardware_psu *psu, __u8 value)
{
	union dac5820_write_payload payload;

	payload.cmd_byte.cmd = DAC5820_CMD_LOAD_IN_DAC_A__UP_DAC_B__OUT_AB;
	payload.cmd_byte.data_high = (value >> 4) & 0xF;
	payload.data_byte.data_low = value & 0xF;
	payload.data_byte.reserved = 0;

	return pl_hardware_psu_i2c_rdwr(p, psu, psu->dac.i2c_address, 0,
					payload.bytes, sizeof payload);
}

/* ADC */

static int pl_hardware_adc_init(struct mxc_epdc_pl_hardware *p,
				struct pl_hardware_psu *psu,
				unsigned init_channel)
{
	struct adc11607_setup * const setup = &psu->adc.cmd.setup;
	struct adc11607_config * const config = &psu->adc.cmd.config;
	psu->adc.i2c_address = p->pdata->adc_i2c_address;
	psu->adc.ref_mv = 2048;
	psu->adc.current_channel = init_channel;

	setup->reset = 1;
	setup->bip_uni = 0;
	setup->clk_sel = 0;
	setup->setup_1 = 1;
	setup->sel = ADC11607_SEL_INT_REF | ADC11607_SEL_INT_REF_ON;

	config->se_diff = 1;
	config->cs = init_channel;
	config->scan = 0;
	config->config_0 = 0;

	pl_hardware_adc_set_nb_channels(&psu->adc);

	if (init_channel >= psu->adc.nb_channels) {
		printk("PLHW: Invalid ADC channel number\n");
		return -1;
	}

	pl_hardware_adc_invalidate(&psu->adc);

	return pl_hardware_i2c_rdwr(p->i2c, psu->adc.i2c_address, 0,
				    psu->adc.cmd.bytes, sizeof psu->adc.cmd);
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

static int pl_hardware_adc_select_channel(struct mxc_epdc_pl_hardware *p,
					  struct pl_hardware_psu *psu,
					  unsigned channel)
{
	if (channel >= psu->adc.nb_channels) {
		printk("PLHW: Invalid ADC channel number\n");
		return -1;
	}

	if (channel == psu->adc.current_channel)
		return 0;

	pl_hardware_adc_invalidate(&psu->adc);
	psu->adc.cmd.config.cs = channel;
	psu->adc.current_channel = channel;

	return pl_hardware_i2c_rdwr(p->i2c, psu->adc.i2c_address, 0,
				    &psu->adc.cmd.config,
				    sizeof psu->adc.cmd.config);
}

static void pl_hardware_adc_invalidate(struct pl_hardware_adc *adc)
{
	int i;

	for (i = 0; i < ADC11607_NB_RESULTS; ++i)
		adc->results[i] = ADC11607_INVALID_RESULT;
}

static int pl_hardware_adc_read_results(struct mxc_epdc_pl_hardware *p,
					struct pl_hardware_psu *psu)
{
	char data[32];
	size_t n;
	size_t read_n;
	const char *it;
	unsigned i;
	int stat;

	n = psu->adc.cmd.config.cs + 1;
	read_n = n * 2;

	stat = pl_hardware_i2c_rdwr(p->i2c, psu->adc.i2c_address, I2C_M_RD,
				    data, read_n);
	if (stat) {
		printk("PLHW: Failed to read ADC results\n");
		pl_hardware_adc_invalidate(&psu->adc);
		return stat;
	}

	for (it = data, i = 0; i < n; ++i, it += 2)
		psu->adc.results[i] = ((it[0] & 0x03) << 8) | it[1];

	for (i = n; i < ADC11607_NB_RESULTS; ++i)
		psu->adc.results[i] = ADC11607_INVALID_RESULT;

	return 0;
}

static int pl_hardware_adc_get_mv(struct pl_hardware_psu *psu)
{
	int adc_mv;

	adc_mv = psu->adc.results[psu->adc.current_channel] * psu->adc.ref_mv;

	return DIV_ROUND_CLOSEST(adc_mv, ADC11607_MAX_VALUE);
}

static int pl_hardware_adc_read_mv(struct mxc_epdc_pl_hardware *p,
				   struct pl_hardware_psu *psu,
				   unsigned channel, int *value)
{
	int stat;

	stat = pl_hardware_adc_select_channel(p, psu, channel);
	if (stat)
		return stat;

	mdelay(2);

	stat = pl_hardware_adc_read_results(p, psu);
	if (stat)
		return stat;

	*value = pl_hardware_adc_get_mv(psu);

	return 0;
}

/* VCOM calibration */

static int pl_hardware_vcomcal_init(struct mxc_epdc_pl_hardware *p,
				    struct pl_hardware_psu *psu)
{		
	psu->vcom.reference_mv = VCOM_DEFAULT;
	psu->vcom.dac_measured = false;

	return 0;
}

static int pl_hardware_vcomcal_set_vcom(struct mxc_epdc_pl_hardware *p,
					struct pl_hardware_psu *psu)
{
	int v_in;
	int delta_stop;
	int delta;
	int n;
	int stat;
	if (!psu->vcom.dac_measured) {
		stat = pl_hardware_vcomcal_measure_dac(p, psu);
		if (stat)
			return stat;
	}

	delta_stop = psu->vcom.dac_step_mv * 6 / 5; /* give 20% margin */
	delta = 0;
	v_in = psu->vcom.last_v_in;
	n = 15;

	do {
		int dac_val;
		int v_out;

		dac_val = pl_hardware_vcomcal_get_dac_value(p, psu, v_in);

		stat = pl_hardware_dac_write(p, psu, dac_val);
		if (stat)
			return stat;

		stat = pl_hardware_adc_read_mv(p, psu, VCOM_FB_ADC_CHANNEL,
					       &v_out);
		if (stat)
			return stat;

		v_out *= VCOM_ADC_SCALE;
		delta = psu->vcom.target_mv - v_out;
		psu->vcom.last_v_in = v_in;
		v_in += delta * VCOM_CORR_I / 100;
	} while ((abs(delta) > delta_stop) && --n);

	if (!n) {
		printk("VCOMCAL Failed to converge in time\n");
		return -1;
	}

	return 0;
}

static int pl_hardware_vcomcal_measure_dac(struct mxc_epdc_pl_hardware *p,
					   struct pl_hardware_psu *psu)
{
	const int x1 = 85;
	const int x2 = 170;
	int y1;
	int y2;
	int vref;
	int swing;
	int stat;
	stat = pl_hardware_dac_write(p, psu, x1);
	if (stat)
		return -1;

	stat = pl_hardware_adc_read_mv(p, psu, VCOM_FB_ADC_CHANNEL, &y1);
	if (stat)
		return -1;

	stat = pl_hardware_dac_write(p, psu, x2);
	if (stat)
		return -1;

	stat = pl_hardware_adc_read_mv(p, psu, VCOM_FB_ADC_CHANNEL, &y2);
	if (stat)
		return -1;

	y1 *= VCOM_ADC_SCALE;
	y2 *= VCOM_ADC_SCALE;

	psu->vcom.dac_dx = x2 - x1;
	psu->vcom.dac_dy = y2 - y1;

	if (!psu->vcom.dac_dx || !psu->vcom.dac_dy) {
		printk("PLHW: Failed to measure DAC characteristics\n");
		return -1;
	}

	psu->vcom.dac_offset =
		y1 - ((x1 * psu->vcom.dac_dy) / psu->vcom.dac_dx);
	psu->vcom.dac_step_mv = psu->vcom.dac_dy / psu->vcom.dac_dx;

	printk("PLHW: DAC dx: %d, dy: %d, offset: %d, step: %d\n",
	       psu->vcom.dac_dx, psu->vcom.dac_dy, psu->vcom.dac_offset,
	       psu->vcom.dac_step_mv);

	stat = pl_hardware_adc_read_mv(p, psu, VCOM_REF_ADC_CHANNEL, &vref);
	if (stat)
		return stat;

	swing = vref * VCOM_VREF_MUL;
	swing = DIV_ROUND_CLOSEST(swing, VCOM_VREF_DIV);
	psu->vcom.target_mv = psu->vcom.reference_mv * swing / VCOM_VGSWING;

	psu->vcom.last_v_in = psu->vcom.target_mv;
	psu->vcom.dac_measured = true;

	printk("PLHW: Scaled VCOM %d -> %d (swing: %d) mV\n",
	       psu->vcom.reference_mv, psu->vcom.target_mv, swing);

	return 0;
}

static int pl_hardware_vcomcal_get_dac_value(struct mxc_epdc_pl_hardware *p,
					     struct pl_hardware_psu *psu,
					     int vcom_mv)
{
	long dac_value;

	dac_value = (vcom_mv - psu->vcom.dac_offset) * psu->vcom.dac_dx;
	dac_value = DIV_ROUND_CLOSEST(dac_value, psu->vcom.dac_dy);

	if (dac_value < 0)
		dac_value = 0;
	else if (dac_value > 0xFF)
		dac_value = 0xFF;

	return dac_value;
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
		       "size %i)\n", addr, reg, size);
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

MODULE_AUTHOR("Guillaume Tucker <guillaume.tucker@plasticlogic.com");
MODULE_DESCRIPTION("Plastic Logic E-Paper hardware control");
MODULE_LICENSE("GPL");

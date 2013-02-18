/*
 * mxc_epdc_pl_hardware.h -- Plastic Logic display power control
 *
 *      Copyright (C) 2012 Guillaume Tucker, Plastic Logic Limited
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#ifndef INCLUDE_MXC_EPDC_PL_HARDWARE_H
#define INCLUDE_MXC_EPDC_PL_HARDWARE_H 1

/* Opaque instance structure */
struct mxc_epdc_pl_hardware;

enum mxc_epdc_pl_hardware_fast_gpio {
	MXC_EPDC_PL_HARDWARE_FAST_D0 = 0,
	MXC_EPDC_PL_HARDWARE_FAST_D1,
	MXC_EPDC_PL_HARDWARE_FAST_D2,
	MXC_EPDC_PL_HARDWARE_FAST_CLK,
	MXC_EPDC_PL_HARDWARE_FAST_EN,
};
#define MXC_EPDC_PL_HARDWARE_GPIO_N 5

struct mxc_epdc_plhw_pdata {
	int i2c_bus_number;
	__u8 dac_i2c_address;
	__u8 adc_i2c_address;
	int fast_gpio[MXC_EPDC_PL_HARDWARE_GPIO_N];
};

struct mxc_epdc_plhw_config {
	int psu_n;
	int source_2bpp;
	int interlaced_gates;
};

extern struct mxc_epdc_pl_hardware *mxc_epdc_pl_hardware_alloc(void);
extern int mxc_epdc_pl_hardware_init(struct mxc_epdc_pl_hardware *plhw,
				     const struct mxc_epdc_plhw_pdata *pdata,
				     const struct mxc_epdc_plhw_config *conf);
extern void mxc_epdc_pl_hardware_free(struct mxc_epdc_pl_hardware *plhw);

extern int mxc_epdc_pl_hardware_set_vcom(struct mxc_epdc_pl_hardware *plhw,
					 const int *vcoms_mv);
extern int mxc_epdc_pl_hardware_enable(struct mxc_epdc_pl_hardware *plhw);
extern int mxc_epdc_pl_hardware_disable(struct mxc_epdc_pl_hardware *plhw);

#endif /* INCLUDE_MXC_EPDC_PL_HARDWARE_H */

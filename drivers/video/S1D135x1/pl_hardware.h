/*
 * pl_hardware.h -- Plastic Logic display power control
 *
 *      Copyright (C) 2012 Guillaume Tucker, Plastic Logic Limited
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#ifndef INCLUDE_PL_HARDWARE_H
#define INCLUDE_PL_HARDWARE_H 1

/* Opaque instance structure */
struct pl_hardware;

enum pl_hardware_hvpmic_id {
	PLHW_HVPMIC_MAX17135,
	PLHW_HVPMIC_TPS65185,
};

struct pl_hardware_config {
	int i2c_bus_number;
	__u8 dac_i2c_address;
	__u8 adc_i2c_address;
	enum pl_hardware_hvpmic_id hvpmic_id;
};

extern int pl_hardware_static_init(void);
extern void pl_hardware_static_free(void);

extern struct pl_hardware *pl_hardware_alloc(void);
extern int pl_hardware_init(struct pl_hardware *plhw,
			    const struct pl_hardware_config *config);
extern void pl_hardware_free(struct pl_hardware *plhw);

extern int pl_hardware_set_vcom(struct pl_hardware *plhw, long vcom_mv);
extern int pl_hardware_get_vcom(struct pl_hardware *plhw, long *vcom_mv);
extern int pl_hardware_enable(struct pl_hardware *plhw);
extern int pl_hardware_disable(struct pl_hardware *plhw);
extern bool pl_hardware_is_enabled(struct pl_hardware *plhw);

#endif /* INCLUDE_PL_HARDWARE_H */

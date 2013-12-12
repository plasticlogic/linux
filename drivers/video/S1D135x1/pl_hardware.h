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

#include <linux/kernel.h>

#define PLHW_INVALID_VCOM INT_MAX

/* Opaque instance structure */
struct plhw;

enum plhw_hvpmic_id {
	PLHW_HVPMIC_MAX17135,
	PLHW_HVPMIC_TPS65185,
};

struct plhw_config {
	int i2c_bus_number;
	__u8 dac_i2c_address;
	__u8 adc_i2c_address;
	enum plhw_hvpmic_id hvpmic_id;
	int init_vcom_mv;

};

extern int plhw_static_init(void);
extern void plhw_static_free(void);

extern struct plhw *plhw_alloc(void);
extern int plhw_init(struct plhw *plhw, const struct plhw_config *config);
extern void plhw_free(struct plhw *plhw);

extern int plhw_set_vcom(struct plhw *plhw, long vcom_mv);
extern int plhw_get_vcom(struct plhw *plhw, long *vcom_mv);
extern int plhw_enable(struct plhw *plhw);
extern int plhw_disable(struct plhw *plhw);
extern bool plhw_is_enabled(struct plhw *plhw);

#endif /* INCLUDE_PL_HARDWARE_H */

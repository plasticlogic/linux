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

#ifdef CONFIG_MODELF_PL_ROBIN
# include <linux/spi/spi.h>
#endif

/* Opaque instance structure */
struct pl_hardware;

struct pl_hardware_config {
	int i2c_bus_number;
	__u8 dac_i2c_address;
};

extern struct pl_hardware *pl_hardware_alloc(void);
extern int pl_hardware_init(struct pl_hardware *plhw,
				     const struct pl_hardware_config *config);
extern void pl_hardware_free(struct pl_hardware *plhw);

extern int pl_hardware_set_vcom(struct pl_hardware *plhw,
					 int vcom_mv);
extern int pl_hardware_enable(struct pl_hardware *plhw);
extern int pl_hardware_disable(struct pl_hardware *plhw);

int pl_hardware_set_temperature(struct pl_hardware *plhw,
					 int temperature);

int pl_hardware_refresh_current_vcom(struct pl_hardware *plhw);

s32 pl_hardware_constrain_temperature_range(s32 temperature);

int pl_hardware_is_module_a(const struct pl_hardware *plhw);

#ifdef CONFIG_MODELF_PL_ROBIN
extern void pl_hardware_i2c_spi_cs_hack(void);
extern void pl_hardware_i2c_spi_write_test(struct spi_device *spi, char *msg);
#endif

#endif /* INCLUDE_PL_HARDWARE_H */

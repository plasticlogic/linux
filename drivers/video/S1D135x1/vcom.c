/*
 * vcom.c -- Plastic Logic Display temperature dependent vcom parameter set.
 *
 *      Copyright (C) 2012 Plastic Logic Limited, Author J.J. Long
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>

#include "vcom.h"
#include "temperature-set.h"

static struct temperature_entry *vcom_temps_and_voltages[MAX_TEMP_RANGES];

static const struct temperature_entry default_vcom_temps_and_voltage = {
	.low_temp = MIN_TEMPERATURE,
	.high_temp = MAX_TEMPERATURE,
	.u.ival = 9700
};


static struct temperature_set vcom_set = {
	.u.entries = vcom_temps_and_voltages,
	.default_entry = &default_vcom_temps_and_voltage,
	.no_of_entries = 0
};

struct temperature_set* get_vcom_temperature_set(void)
{
	return &vcom_set;
}

static int store_vcom_temps_and_voltage(const struct temperature_entry *p_new)
{
	int status = temperature_set_store_entry(&vcom_set, p_new, ARRAY_SIZE(vcom_temps_and_voltages));
	return status;
}

static int validate_vcom(const struct temperature_entry *p_new)
{
	int status = 0;
	/*s32 millivolts = p_new->u.ival;*/

	status = validate_temperatures(p_new);

	return status;
}

int read_and_store_vcom(const char *p_range_text)
{
	int retval = -EINVAL;
	struct temperature_entry range;

	retval = temperature_set_read_integer(p_range_text, &range);

	if (retval == 0) {
		retval = validate_vcom(&range);

		if (retval == 0) {
			retval = store_vcom_temps_and_voltage(&range);
			if (retval != 0)
				printk("Failed to store Vcom, retval = %d\n", retval);
		}
	}
	return retval;
}

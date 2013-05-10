/*
 * i2c-sc18is60x.c -- NXP SC18IS600/601 SPI to I2C bridge driver
 *
 * Copyright (C) 2013 Plastic Logic Limited
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

#ifndef INCLUDE_LINUX_I2C_SC18IS60X_H
#define INCLUDE_LINUX_I2C_SC18IS60X_H 1

struct sc18is60x_platform_data {
	bool use_gpio_cs;
	int gpio_cs;
};

#endif /* INCLUDE_LINUX_I2C_SC18IS60X_H */

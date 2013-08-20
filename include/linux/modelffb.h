/*
 * include/linux/modelffb.h
 *
 * FB driver for EPSON modelF EPD controller
 *
 * Copyright (C) 2012 Seiko Epson Corporation
 * Copyright (C) 2013 Plastic Logic Limited
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

#ifndef INCLUDE_LINUX_MODELFFB_H
#define INCLUDE_LINUX_MODELFFB_H 1

struct spi_board_info;
struct pinmux_config;

struct modelffb_platform_data {
	struct spi_board_info *spi_info;
	int gpio_hrdy; /* host ready, not compulsory with SPI */
	int gpio_hdc; /* data / command switch, leave to 0 if not used */
	int gpio_cs;  /* SPI chip select, used if gpio_hdc is left to 0 */
	unsigned int hirq; /* GPIO-based host IRQ */
};

#endif /* INCLUDE_LINUX_MODELFFB_H */

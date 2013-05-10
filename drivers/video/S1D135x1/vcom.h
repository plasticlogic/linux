/*
 * vcom.h -- Plastic Logic Display temperature dependent vcom parameter set.
 *
 *      Copyright (C) 2012 Plastic Logic Limited, Author J.J. Long
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#ifndef _VCOM_H_
#define _VCOM_H_	1

struct temperature_set;

struct temperature_set* get_vcom_temperature_set(void);
int read_and_store_vcom(const char *p_range_text);
#endif


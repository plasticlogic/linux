/*
 * temperature-set.h -- Plastic Logic Display temperature dependent parameter set.
 *
 *      Copyright (C) 2012 Plastic Logic Limited, Author J.J. Long
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#ifndef _TEMPERATURE_SET_H_
#define _TEMPERATURE_SET_H_

#define MAX_TEMP_RANGES			20
#define MIN_TEMPERATURE			-128
#define MAX_TEMPERATURE			127

#define TEMPERATURE_NOT_FOUND	0x80000000

struct temperature_entry
{
	int low_temp;
	int high_temp;
	union data {
		void *p;
		const void *p_const;
		s32 ival;
	} u;
};

struct temperature_set {
	/* compiler requires cast to convert non-const to const pointers */
	union pointers {
		struct temperature_entry** entries;
		const struct temperature_entry** const_entries;
	} u;
	const struct temperature_entry* default_entry;
	size_t no_of_entries;
	bool changed;
};

int temperature_set_store_entry(struct temperature_set* set, const struct temperature_entry *p_new, size_t max_entries);
void temperature_set_commit(struct temperature_set* set);
const struct temperature_entry** temperature_set_lookup(struct temperature_set* set, int temperature);
const struct temperature_entry** temperature_set_get_first_range(struct temperature_set* set);
const struct temperature_entry** temperature_set_get_next_range(struct temperature_set* set,
								const struct temperature_entry** p_current);
int validate_temperatures(const struct temperature_entry *p_new);

int temperature_set_format_integer(const struct temperature_entry* p_range, char *tbuf, int buf_space);
int temperature_set_read_integer(const char *p_range_text, struct temperature_entry *range);
void temperature_set_erase_ints(struct temperature_set* set);
void temperature_set_erase_pointers(struct temperature_set* set);
s32 temperature_set_lookup_int(struct temperature_set* set, int temperature);
const void* temperature_set_lookup_pointer(struct temperature_set* set, int temperature);

const struct temperature_entry* temperature_set_get_lowest_range(struct temperature_set* set);
const struct temperature_entry* temperature_set_get_highest_range(struct temperature_set* set);
s32 temperature_set_get_lowest_temp(struct temperature_set* set);
s32 temperature_set_get_highest_temp(struct temperature_set* set);

s32 constrain_temp_to_set(struct temperature_set* set, s32 temperature,
                          const char *set_name);

bool temperature_set_changed(const struct temperature_set* set);
#endif


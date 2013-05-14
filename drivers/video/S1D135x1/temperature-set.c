/*
 * temperature-set.c -- Plastic Logic Display temperature dependent parameter set.
 *
 *      Copyright (C) 2012 Plastic Logic Limited, Author J.J. Long
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/sort.h>
#include "temperature-set.h"

static void temperature_set_erase(struct temperature_set* set, bool delete_entry)
{
	struct temperature_entry **p_entry;
	size_t num_left;


	if (set == 0)
		return;

	p_entry = set->u.entries;

	num_left = set->no_of_entries;

	for (; num_left != 0 && *p_entry != 0; ++p_entry, --num_left) {
		if (*p_entry != 0) {
			if (delete_entry && (*p_entry)->u.p != 0)
				kfree((*p_entry)->u.p);
			kfree(*p_entry);
		}
	}

	set->no_of_entries = 0;
	set->changed = true;
}

int temperature_set_store_entry(struct temperature_set* set, const struct temperature_entry *p_new, size_t max_entries)
{
	if (set == 0)
		return -EINVAL;

#if 0
	printk("temperature_set_store_entry() : set = %p, p_new = %p, max_entries = %d, no_of_entries = %d\n", set, p_new, max_entries, set->no_of_entries);
#endif
	if (set->no_of_entries < max_entries) {
		set->u.entries[set->no_of_entries] = kmalloc(sizeof(*p_new), GFP_KERNEL);

		memcpy(set->u.entries[set->no_of_entries], p_new, sizeof(*p_new));

		++set->no_of_entries;
	}
	else
		return -ENOMEM;

	set->changed = true;

	return 0;
}
EXPORT_SYMBOL(temperature_set_store_entry);

static int compare_low_temp(const void* p_left, const void* p_right)
{
	const struct temperature_entry* pl = *((const struct temperature_entry**) p_left);
	const struct temperature_entry* pr = *((const struct temperature_entry**) p_right);

	int delta = pl->low_temp - pr->low_temp;
	/* printk("pl = %p, pr = %p\n", pl, pr); */
	/* printk("delta = %d, pl->low_temp = %d, pr->low_temp = %d\n", delta, pl->low_temp, pr->low_temp); */

	return delta;
}

void temperature_set_commit(struct temperature_set* set)
{
#if 0
	printk("temperature_set_commit() : set = %p, set->no_of_entries = %d\n", set, set->no_of_entries);
#endif
	sort(set->u.entries, set->no_of_entries, sizeof(*set->u.entries),
		compare_low_temp, NULL);

	set->changed = true;
}
EXPORT_SYMBOL(temperature_set_commit);

static void *my_bsearch(const void *key, const void *base, size_t num, size_t size,
		int (*compare)(const void *key, const void *elt))
{
	int start = 0, end = num - 1, mid, result;
	if (num == 0)
		return NULL;

	while (start <= end) {
		mid = (start + end) / 2;
		result = compare(key, base + mid * size);
		if (result < 0)
			end = mid - 1;
		else if (result > 0)
			start = mid + 1;
		else
			return (void *)base + mid * size;
	}

	return NULL;
}

static int compare_temperature(const void* p_key, const void* p_elem)
{
	int result;
	int temperature = *(const int *) p_key;
	const struct temperature_entry* p_el = *((const struct temperature_entry**) p_elem);

	if (temperature >= p_el->low_temp && temperature <= p_el->high_temp)
		result = 0;
	else if (temperature < p_el->low_temp)
		result = -1;
	else
		result = 1;

#if 0
	 printk("temperature = %d, p_elem = %p, p_el = %p\n", temperature, p_elem, p_el);
	 printk("p_el->low_temp = %d, result = %d\n", p_el->low_temp, result);
#endif
	return result;
}

const struct temperature_entry** temperature_set_lookup(struct temperature_set* set, int temperature)
{
	const struct temperature_entry **p_entry = 0;

#if 0
	printk("temperature_set_lookup() : set->no_of_entries = %d\n", set->no_of_entries);
#endif
	/* return default entry if set is empty */
	if (set->no_of_entries == 0)
		p_entry = &set->default_entry;
	else {
		/* binary search for required entry */
		/* unfortunately, kernel doesn't have a generic bsearch() kind of function */
		p_entry = my_bsearch(&temperature, set->u.const_entries, set->no_of_entries,
				sizeof(*set->u.const_entries), compare_temperature);
	}

	/* indicate that set has not changed on 1st lookup */
	set->changed = false;

	return p_entry;
}
EXPORT_SYMBOL(temperature_set_lookup);

const struct temperature_entry** temperature_set_get_first_range(struct temperature_set* set)
{
	const struct temperature_entry** p_first = 0;

	if (set != 0) {
		if (set->u.const_entries[0] != 0 && set->no_of_entries != 0)
			p_first = set->u.const_entries;
		else
			p_first = &set->default_entry;
	}

	return p_first;
}
EXPORT_SYMBOL(temperature_set_get_first_range);

const struct temperature_entry** temperature_set_get_next_range(struct temperature_set* set,
								const struct temperature_entry** p_current)
{
	const struct temperature_entry** p_next = 0;
	off_t offset;

	/* there is no next if we are pointing at the default entry */
	if (set != 0 && p_current != &set->default_entry) {
		offset = p_current - set->u.const_entries;

		if (offset >= 0 && offset < (set->no_of_entries - 1)) {
			p_next = p_current + 1;
			if (*p_next == 0)
				p_next = 0;
		}
	}

	return p_next;
}
EXPORT_SYMBOL(temperature_set_get_next_range);

int validate_temperatures(const struct temperature_entry *p_new)
{
	int status = 0;

	if (p_new->low_temp > p_new->high_temp
		|| p_new->low_temp < MIN_TEMPERATURE || p_new->low_temp > MAX_TEMPERATURE
		|| p_new->high_temp < MIN_TEMPERATURE || p_new->high_temp > MAX_TEMPERATURE)
		status = -EINVAL;

	return status;
}
EXPORT_SYMBOL(validate_temperatures);

int temperature_set_format_integer(const struct temperature_entry* p_range, char *tbuf, int buf_space)
{
	int written;

	written = scnprintf(tbuf, buf_space, "%d,%d,%d\n",
				p_range->low_temp, p_range->high_temp, p_range->u.ival);

	return written;
}
EXPORT_SYMBOL(temperature_set_format_integer);

int temperature_set_read_integer(const char *p_range_text, struct temperature_entry *range)
{
	int num_read;
	int retval = -EINVAL;

	num_read = sscanf(p_range_text, "%d,%d,%d",
			&range->low_temp, &range->high_temp, &range->u.ival);
	/* error if range is malformed */
	if (num_read == 3)
		retval = 0;

	return retval;
}
EXPORT_SYMBOL(temperature_set_read_integer);

void temperature_set_erase_ints(struct temperature_set* set)
{
	temperature_set_erase(set, false);
}
EXPORT_SYMBOL(temperature_set_erase_ints);

void temperature_set_erase_pointers(struct temperature_set* set)
{
	temperature_set_erase(set, true);
}
EXPORT_SYMBOL(temperature_set_erase_pointers);


s32 temperature_set_lookup_int(struct temperature_set* set, int temperature)
{
	const struct temperature_entry **p_entry = 0;
	s32 val = TEMPERATURE_NOT_FOUND;

	p_entry = temperature_set_lookup(set, temperature);

	if (p_entry != 0)
		val = (*p_entry)->u.ival;

	return val;
}
EXPORT_SYMBOL(temperature_set_lookup_int);

static const struct temperature_entry* temperature_set_get_range(struct temperature_set* set, size_t index)
{
	const struct temperature_entry *p_entry = 0;

	if (set != 0) {
		/* use default entry when empty */
		if (set->no_of_entries == 0)
			p_entry = set->default_entry;
		else if (set->u.const_entries != 0)
			p_entry = set->u.const_entries[index];
	}

	return p_entry;
}

const struct temperature_entry* temperature_set_get_lowest_range(struct temperature_set* set)
{
	const struct temperature_entry *p_entry = 0;

	p_entry = temperature_set_get_range(set, 0);

	return p_entry;
}

const struct temperature_entry* temperature_set_get_highest_range(struct temperature_set* set)
{
	const struct temperature_entry *p_entry = 0;

	if (set != 0)
		p_entry = temperature_set_get_range(set, set->no_of_entries - 1);

	return p_entry;
}

s32 temperature_set_get_lowest_temp(struct temperature_set* set)
{
	const struct temperature_entry* p;
	s32 temp = TEMPERATURE_NOT_FOUND;

	p = temperature_set_get_lowest_range(set);

	if (p != 0)
		temp = p->low_temp;

	return temp;
}

s32 temperature_set_get_highest_temp(struct temperature_set* set)
{
	const struct temperature_entry* p;
	s32 temp = TEMPERATURE_NOT_FOUND;

	p = temperature_set_get_highest_range(set);

	if (p != 0)
		temp = p->high_temp;

	return temp;
}

const void* temperature_set_lookup_pointer(struct temperature_set* set, int temperature)
{
	const struct temperature_entry **p_entry = 0;
	const void *p = 0;

	p_entry = temperature_set_lookup(set, temperature);

	if (p_entry != 0)
		p = (*p_entry)->u.p;

	return p;
}

s32 constrain_temp_to_set(struct temperature_set* set, s32 temperature,
                          const char *set_name)
{
        s32 constrained_temp;
        s32 lowest_temp;
        s32 highest_temp;

        constrained_temp = temperature;

        lowest_temp = temperature_set_get_lowest_temp(set);
        highest_temp = temperature_set_get_highest_temp(set);

        /* leave temperature alone if set is empty */
        if (lowest_temp == TEMPERATURE_NOT_FOUND
                || highest_temp == TEMPERATURE_NOT_FOUND) {
                pr_info("Set %s is empty, measured temp = %d\n",
                       set_name, constrained_temp);
        }
        if (constrained_temp < lowest_temp) {
                pr_err("Measured temperature %d lower than %s minimum %d\n",
                       constrained_temp, set_name, lowest_temp);
                constrained_temp = lowest_temp;
        } else if (constrained_temp > highest_temp) {
                pr_err("Measured temperature %d greater than %s maximum %d\n",
                       constrained_temp, set_name, highest_temp);
                constrained_temp = highest_temp;
        }

        return constrained_temp;
}
EXPORT_SYMBOL(constrain_temp_to_set);


bool temperature_set_changed(const struct temperature_set* set)
{
	return set->changed;
}
EXPORT_SYMBOL(temperature_set_changed);


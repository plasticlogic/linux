/*
 * pmeter.h -- Performance Meter driver interface definition
 *
 * Copyright (c) 2011, 2012, 2013 Plastic Logic Limited
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

#ifndef _LINUX_PMETER_H_
#define _LINUX_PMETER_H_ 1

#ifdef __KERNEL__
#include <linux/time.h>
#else
#include <sys/time.h>
#endif

#define PMETER_TAG_SZ (32 - sizeof(struct timeval))

/* -- user-side read operations -- */

struct pmeter_record {
	struct timeval time;
	char tag[PMETER_TAG_SZ];
};

#ifndef __KERNEL__ /* user-side functions */

#include <fcntl.h>
#include <unistd.h>

#define pmeter_logf(msg, ...) do {					\
		char buf[PMETER_TAG_SZ];				\
		snprintf(buf, PMETER_TAG_SZ, msg, ##__VA_ARGS__);	\
		pmeter_log(buf);					\
	} while (0)

static inline int pmeter_log(const char *tag)
{
	int fd;
	size_t n;
	ssize_t wr_n;

	fd = open("/dev/pmeter", O_WRONLY);

	if (fd < 0)
		return -1;

	for (n = 0; (tag[n] != '\0') && (n < PMETER_TAG_SZ); ++n);

	wr_n = write(fd, tag, n);

	close(fd);

	return ((wr_n < 0) || (wr_n != n)) ? -1 : 0;
}

#else /* __KERNEL__ */

/* -- kernel internal usage -- */

#if defined(CONFIG_PMETER) || defined(CONFIG_PMETER_MODULE)
# define pmeter_logf(msg, ...) do {					\
		char buf[PMETER_TAG_SZ];				\
		snprintf(buf, PMETER_TAG_SZ, msg, ##__VA_ARGS__);	\
		pmeter_log(buf);					\
	} while (0)
extern void pmeter_log(const char *tag);
#else
# define pmeter_logf(msg, ...)
static inline void pmeter_log(const char *tag) {}
#endif

#endif /* __KERNEL__ */

#endif /* _LINUX_PMETER_H_ */

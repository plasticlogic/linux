/*
 * pmeter.c -- Performance Meter driver
 *
 * Copyright (c) 2011 Plastic Logic Limited
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

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/pmeter.h>
#include <asm/uaccess.h>

#define PMETER_BUFFER_LEN 1024
#define PMETER_TAG_LEN (PMETER_TAG_SZ - 1)

struct pmeter_reader {
	struct list_head list_entry;
	const struct pmeter_record *cur;
	bool overflow;
};

/* -- module parameters -- */

static uint pmeter_buffer_len_modparam = PMETER_BUFFER_LEN;
module_param_named(buffer_length, pmeter_buffer_len_modparam, uint, S_IRUGO);
MODULE_PARM_DESC(buffer_length, "Buffer length in number of entries");

/* -- static variables -- */

static struct miscdevice pmeter_mdev;
static struct list_head pmeter_readers;
static unsigned pmeter_n_readers;
static spinlock_t pmeter_lock;
static struct pmeter_record *pmeter_buffer = NULL; /* buffer of entries */
static struct pmeter_record *pmeter_cur;           /* current log entry */
static const struct pmeter_record *pmeter_last;    /* last buffer entry */
static const struct pmeter_record *pmeter_end;     /* buffer's end address */

/* -- local functions -- */

static void pmeter_update_readers_locked(const struct pmeter_record *prev_last)
{
	struct pmeter_reader *reader;

	list_for_each_entry(reader, &pmeter_readers, list_entry) {
		if (reader->cur == prev_last) {
			reader->cur = pmeter_last;

			if (!reader->overflow) {
				dev_warn(pmeter_mdev.this_device,
					 "overflow on %p\n", reader);
				reader->overflow = true;
			}
		}
	}
}

static void pmeter_do_log(const char *tag)
{
	unsigned long long time = cpu_clock(smp_processor_id());
	unsigned long time_ns = do_div(time, 1e9);

	spin_lock(&pmeter_lock);

	pmeter_cur->time.tv_sec = (unsigned long) time;
	pmeter_cur->time.tv_usec = time_ns / 1000;
	strncpy(pmeter_cur->tag, tag, PMETER_TAG_SZ);
	pmeter_cur->tag[PMETER_TAG_LEN] = '\0';

	if (++pmeter_cur == pmeter_end)
		pmeter_cur = pmeter_buffer;

	if (pmeter_cur == pmeter_last) {
		const struct pmeter_record *prev_last = pmeter_last;

		if (++pmeter_last == pmeter_end)
			pmeter_last = pmeter_buffer;

		pmeter_update_readers_locked(prev_last);
	}

	spin_unlock(&pmeter_lock);
}

/* -- public interface -- */

void pmeter_log(const char *tag)
{
	if (!pmeter_buffer)
		return;

	pmeter_do_log(tag);
}
EXPORT_SYMBOL(pmeter_log);

/* -- sysfs attributes -- */

static ssize_t pmeter_attr_count_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buffer)
{
	int count;

	count = pmeter_cur - pmeter_last;

	if (count < 0)
		count += pmeter_buffer_len_modparam;

	return scnprintf(buffer, PAGE_SIZE, "%d\n", count);
}
static DEVICE_ATTR(count, 0444, pmeter_attr_count_show, NULL);

static ssize_t pmeter_attr_readers_show(struct device *dev,
					struct device_attribute *attr,
					char *buffer)
{
	return scnprintf(buffer, PAGE_SIZE, "%d\n", pmeter_n_readers);
}
static DEVICE_ATTR(readers, 0444, pmeter_attr_readers_show, NULL);

static ssize_t pmeter_clear_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct pmeter_reader *reader;

	spin_lock(&pmeter_lock);
	list_for_each_entry(reader, &pmeter_readers, list_entry) {
		if (reader->cur != pmeter_cur)
			reader->overflow = true;
	}
	pmeter_last = pmeter_cur;
	spin_unlock(&pmeter_lock);

	return count;
}
static DEVICE_ATTR(clear, 0222, NULL, pmeter_clear_store);

static struct attribute *pmeter_attrs[] = {
	&dev_attr_count.attr,
	&dev_attr_readers.attr,
	&dev_attr_clear.attr,
	NULL
};

static const struct attribute_group pmeter_attr_group = {
	.attrs = pmeter_attrs,
};

/* -- file operations -- */

static ssize_t pmeter_read(struct file *file, char __user *user_buffer,
			   size_t size, loff_t *offset)
{
	struct pmeter_reader *reader;
	ssize_t n_read = sizeof(struct pmeter_record);

	if (size < n_read)
		return -ENOBUFS;

	reader = file->private_data;

	if (reader->overflow)
		return -EFAULT;

	spin_lock(&pmeter_lock);

	if (pmeter_cur == reader->cur) {
		n_read = 0;
		goto unlock;
	}

	if (copy_to_user(user_buffer, reader->cur, n_read)) {
		n_read = -EFAULT;
		goto unlock;
	}

	if (++reader->cur == pmeter_end)
		reader->cur = pmeter_buffer;

unlock:
	spin_unlock(&pmeter_lock);

	return n_read;
}

static ssize_t pmeter_write(struct file *file, const char __user *user_buffer,
			    size_t size, loff_t *offset)
{
	const unsigned long copy_sz = min(size, PMETER_TAG_LEN);
	char tmp_tag[PMETER_TAG_SZ];
	const char *end;
	char *it;

	if (copy_from_user(tmp_tag, user_buffer, copy_sz))
		return -EFAULT;

	for (it = tmp_tag, end = tmp_tag + copy_sz; it != end; ++it) {
		if (*it == '\n')
			break;
	}

	*it = '\0';

	pmeter_do_log(tmp_tag);

	return size;
}

static int pmeter_open(struct inode *inode, struct file *file)
{
	struct pmeter_reader *reader;

	if (!(file->f_mode & FMODE_READ))
		return 0;

	reader = kmalloc(sizeof(struct pmeter_reader), GFP_KERNEL);
	if (!reader)
		return -ENOMEM;

	dev_info(pmeter_mdev.this_device, "open: %p\n", reader);
	INIT_LIST_HEAD(&reader->list_entry);
	reader->overflow = false;
	spin_lock(&pmeter_lock);
	list_add_tail(&reader->list_entry, &pmeter_readers);
	reader->cur = pmeter_last;
	spin_unlock(&pmeter_lock);
	++pmeter_n_readers;
	file->private_data = reader;

	return 0;
}

static int pmeter_release(struct inode *inode, struct file *file)
{
	struct pmeter_reader *reader;

	if (!(file->f_mode & FMODE_READ))
		return 0;

	reader = file->private_data;
	dev_info(pmeter_mdev.this_device, "release: %p\n", reader);
	spin_lock(&pmeter_lock);
	list_del(&reader->list_entry);
	spin_unlock(&pmeter_lock);
	--pmeter_n_readers;
	kfree(reader);

	return 0;
}

static const struct file_operations pmeter_fops = {
	.owner = THIS_MODULE,
	.read = pmeter_read,
	.write = pmeter_write,
	.open = pmeter_open,
	.release = pmeter_release,
};

/* -- module functions -- */

static inline void pmeter_free_buffer(void) {
	kfree(pmeter_buffer);
	pmeter_buffer = NULL;
}

static inline void pmeter_free_attrs(void) {
	sysfs_remove_group(&pmeter_mdev.this_device->kobj, &pmeter_attr_group);
}

static inline void pmeter_free_mdev(void) {
	misc_deregister(&pmeter_mdev);
}

static int __init pmeter_init(void)
{
	size_t buffer_sz;
	int stat;

	buffer_sz = pmeter_buffer_len_modparam * sizeof(struct pmeter_record);
	pmeter_buffer = kmalloc(buffer_sz, GFP_KERNEL);
	if (!pmeter_buffer)
		return -ENOMEM;

	pmeter_last = pmeter_cur = pmeter_buffer;
	pmeter_end = &pmeter_buffer[pmeter_buffer_len_modparam];

	pmeter_mdev.minor = MISC_DYNAMIC_MINOR;
	pmeter_mdev.name = "pmeter";
	pmeter_mdev.fops = &pmeter_fops;
	stat = misc_register(&pmeter_mdev);
	if (stat) {
		pr_err("failed to register misc device\n");
		goto err_free_buffer;
	}

	stat = sysfs_create_group(&pmeter_mdev.this_device->kobj,
				  &pmeter_attr_group);
	if (stat) {
		pr_err("failed to create attributes group\n");
		goto err_free_mdev;
	}

	spin_lock_init(&pmeter_lock);
	INIT_LIST_HEAD(&pmeter_readers);
	pmeter_n_readers = 0;

	dev_info(pmeter_mdev.this_device, "Performance Meter driver ready\n");
	dev_info(pmeter_mdev.this_device, "buffer size: %zu bytes\n",
		 buffer_sz);
	dev_info(pmeter_mdev.this_device, "buffer length: %zu entries\n",
		 pmeter_buffer_len_modparam);

	return 0;

err_free_mdev:
	pmeter_free_mdev();
err_free_buffer:
	pmeter_free_buffer();

	return stat;
}

static void __exit pmeter_exit(void)
{
	dev_info(pmeter_mdev.this_device, "exit\n");
	pmeter_free_attrs();
	pmeter_free_mdev();
	pmeter_free_buffer();
}

MODULE_AUTHOR("Guillaume Tucker <guillaume.tucker@plasticlogic.com>");
MODULE_DESCRIPTION("Performance Meter");
MODULE_LICENSE("GPL");
module_init(pmeter_init);
module_exit(pmeter_exit);

/*
 * dummy_kbd.c -- Dummy keyboard driver
 *
 * Copyright (c) 2013 Plastic Logic Limited
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
#include <linux/input.h>
#include <linux/fs.h>

struct dummy_kbd {
	struct input_dev *input_dev;
	int key;
	int on;
};

static struct dummy_kbd dummy_kbd;

/* Attributes */

static ssize_t dummy_kbd_key_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct dummy_kbd *kbd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", kbd->key);
}

static ssize_t dummy_kbd_key_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct dummy_kbd *kbd = dev_get_drvdata(dev);
	int key;

	sscanf(buf, "%d\n", &key);

	if ((key < 0) || (key > KEY_MAX))
		return -EINVAL;

	kbd->key = key;

	return count;
}

static DEVICE_ATTR(key, 0666, dummy_kbd_key_show, dummy_kbd_key_store);

static ssize_t dummy_kbd_on_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct dummy_kbd *kbd = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", kbd->on);
}

static ssize_t dummy_kbd_on_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct dummy_kbd *kbd = dev_get_drvdata(dev);
	int on;

	sscanf(buf, "%d\n", &on);

	if ((on != 0) && (on != 1))
		return -EINVAL;

	kbd->on = on;

	return count;
}

static DEVICE_ATTR(on, 0666, dummy_kbd_on_show, dummy_kbd_on_store);

static ssize_t dummy_kbd_sync_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct dummy_kbd *kbd = dev_get_drvdata(dev);

	input_report_key(kbd->input_dev, kbd->key, kbd->on);
	input_sync(kbd->input_dev);

	return count;
}

static DEVICE_ATTR(sync, 0666, NULL, dummy_kbd_sync_store);

static struct attribute *dummy_kbd_attrs[] = {
	&dev_attr_key.attr,
	&dev_attr_on.attr,
	&dev_attr_sync.attr,
	NULL
};

static const struct attribute_group dummy_kbd_attr_group = {
	.attrs = dummy_kbd_attrs,
};

/* Input device */

static int dummy_kbd_open(struct input_dev *dev)
{
	dev_info(&dev->dev, "open\n");

	return 0;
}

static void dummy_kbd_close(struct input_dev *dev)
{
	dev_info(&dev->dev, "close\n");
}

/* Module interface */

static int __init dummy_kbd_init(void)
{
	struct input_dev *input_dev;
	int i;
	int stat;

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("Failed to allocate input device\n");
		return -ENOMEM;
	}

	input_dev->name = "Dummy keyboard";
	input_dev->id.bustype = BUS_VIRTUAL;
	input_dev->open = dummy_kbd_open;
	input_dev->close = dummy_kbd_close;

	__set_bit(EV_KEY, input_dev->evbit);

	for (i = 0; i < BITS_TO_LONGS(KEY_CNT); ++i)
		input_dev->keybit[i] = -1UL;

	input_set_drvdata(input_dev, &dummy_kbd);

	stat = input_register_device(input_dev);
	if (stat) {
		dev_err(&input_dev->dev, "Failed to register input device\n");
		return stat;
	}

	stat = sysfs_create_group(&input_dev->dev.kobj, &dummy_kbd_attr_group);
	if (stat) {
		dev_err(&input_dev->dev, "Failed to create sysfs group");
		return stat;
	}

	dummy_kbd.input_dev = input_dev;
	dummy_kbd.key = 0;
	dummy_kbd.on = 0;

	return 0;
}

static void __exit dummy_kbd_exit(void)
{
	sysfs_remove_group(&dummy_kbd.input_dev->dev.kobj,
			   &dummy_kbd_attr_group);
	input_unregister_device(dummy_kbd.input_dev);
}

module_init(dummy_kbd_init);
module_exit(dummy_kbd_exit);

MODULE_AUTHOR("Guillaume Tucker <guillaume.tucker@plasticlogic.com>");
MODULE_DESCRIPTION("Dummy keyboard driver");
MODULE_LICENSE("GPL");

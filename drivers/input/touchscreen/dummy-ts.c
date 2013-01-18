/*
 * dummy_ts.c -- Dummy touch screen driver
 *
 * Copyright (c) 2012, 2013 Plastic Logic Limited
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

/* ToDo: support multiple instances. */

struct dummy_ts {
	struct input_dev *input_dev;
	int x;
	int y;
	int touch;
};

static struct dummy_ts dummy_ts;

/* Parameters */

static uint dummy_ts_xres_modparam = 640;
module_param_named(xres, dummy_ts_xres_modparam, uint, S_IRUGO);
MODULE_PARM_DESC(xres, "Horizontal resolution in pixels");

static uint dummy_ts_yres_modparam = 480;
module_param_named(yres, dummy_ts_yres_modparam, uint, S_IRUGO);
MODULE_PARM_DESC(yres, "Vertical resolution in pixels");

/* Attributes */

static ssize_t dummy_ts_x_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct dummy_ts *dts = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", dts->x);
}
static ssize_t dummy_ts_x_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dummy_ts *dts = dev_get_drvdata(dev);
	int x;

	sscanf(buf, "%d\n", &x);

	if ((x < 0) || (x > dummy_ts_xres_modparam))
		return -EINVAL;

	dts->x = x;

	return count;
}
static DEVICE_ATTR(x, 0666, dummy_ts_x_show, dummy_ts_x_store);

static ssize_t dummy_ts_y_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct dummy_ts *dts = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", dts->y);
}
static ssize_t dummy_ts_y_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct dummy_ts *dts = dev_get_drvdata(dev);
	int y;

	sscanf(buf, "%d\n", &y);

	if ((y < 0) || (y > dummy_ts_yres_modparam))
		return -EINVAL;

	dts->y = y;

	return count;
}
static DEVICE_ATTR(y, 0666, dummy_ts_y_show, dummy_ts_y_store);

static ssize_t dummy_ts_touch_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct dummy_ts *dts = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", dts->touch);
}
static ssize_t dummy_ts_touch_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct dummy_ts *dts = dev_get_drvdata(dev);
	int touch;

	sscanf(buf, "%d\n", &touch);

	if ((touch != 0) && (touch != 1))
		return -EINVAL;

	dts->touch = touch;

	return count;
}
static DEVICE_ATTR(touch, 0666, dummy_ts_touch_show, dummy_ts_touch_store);

static ssize_t dummy_ts_sync_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct dummy_ts *dts = dev_get_drvdata(dev);

	input_report_abs(dts->input_dev, ABS_X, dts->x);
	input_report_abs(dts->input_dev, ABS_Y, dts->y);
	input_report_key(dts->input_dev, BTN_TOUCH, dts->touch);
	input_sync(dts->input_dev);

	return count;
}
static DEVICE_ATTR(sync, 0666, NULL, dummy_ts_sync_store);

static struct attribute *dummy_ts_attrs[] = {
	&dev_attr_x.attr,
	&dev_attr_y.attr,
	&dev_attr_touch.attr,
	&dev_attr_sync.attr,
	NULL
};
static const struct attribute_group dummy_ts_attr_group = {
	.attrs = dummy_ts_attrs,
};

/* Input device */

static int dummy_ts_open(struct input_dev *dev)
{
	dev_info(&dev->dev, "open\n");

	return 0;
}

static void dummy_ts_close(struct input_dev *dev)
{
	dev_info(&dev->dev, "close\n");
}

/* Module interface */

static int __init dummy_ts_init(void)
{
	struct input_dev *input_dev;
	int stat;

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("Failed to allocate input device\n");
		return -ENOMEM;
	}

	input_dev->name = "Dummy touch screen";
	input_dev->id.bustype = BUS_VIRTUAL;
	input_dev->dev.parent = NULL;
	input_dev->open = dummy_ts_open;
	input_dev->close = dummy_ts_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, dummy_ts_xres_modparam, 0,0);
	input_set_abs_params(input_dev, ABS_Y, 0, dummy_ts_yres_modparam, 0,0);

	input_set_drvdata(input_dev, &dummy_ts);

	stat = input_register_device(input_dev);
	if (stat) {
		dev_err(&input_dev->dev, "Failed to register input device\n");
		return stat;
	}

	dev_info(&input_dev->dev, "Resolution: %dx%d\n",
		 dummy_ts_xres_modparam, dummy_ts_yres_modparam);

	stat = sysfs_create_group(&input_dev->dev.kobj, &dummy_ts_attr_group);
	if (stat) {
		dev_err(&input_dev->dev, "Failed to create sysfs group");
		return stat;
	}

	dummy_ts.input_dev = input_dev;
	dummy_ts.x = 0;
	dummy_ts.y = 0;
	dummy_ts.touch = 0;

	return 0;
}

static void __exit dummy_ts_exit(void)
{
	sysfs_remove_group(&dummy_ts.input_dev->dev.kobj,&dummy_ts_attr_group);
	input_unregister_device(dummy_ts.input_dev);
}

module_init(dummy_ts_init);
module_exit(dummy_ts_exit);

MODULE_AUTHOR("Guillaume Tucker <guillaume.tucker@plasticlogic.com>");
MODULE_DESCRIPTION("Dummy touch screen driver");
MODULE_LICENSE("GPL");

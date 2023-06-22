/*
 * Kernel driver for the Panasonic AMG88xx-series sensors
 *
 * Copyright (C) 2018  Iiro Vuorio <iiro.vuorio@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <amg88xx.h>

/*
 * Low level access helper functions
 */
static inline int amg88xx_write8(struct i2c_client *client, u8 reg_addr,
				 u8 value)
{
	return i2c_smbus_write_byte_data(client, reg_addr, value);
}

static inline int amg88xx_read8(struct i2c_client *client, u8 reg_addr)
{
	return i2c_smbus_read_byte_data(client, reg_addr);
}

static int amg88xx_read16(struct i2c_client *client, u8 regl, u8 regh)
{
	int ret;
	u16 val;

	// First get the high bits and then the low bits
	ret = amg88xx_read8(client, regh);
	if (ret < 0)
		return ret;
	else
		val = ret << 8;

	ret = amg88xx_read8(client, regl);
	if (ret < 0)
		return ret;
	else
		val |= ret;

	return val;
}

static int amg88xx_write16(struct i2c_client *client, u8 regl, u8 regh,
			   u16 value)
{
	int ret;

	// Write the low register first and then the high register
	ret = amg88xx_write8(client, regl, (u8)value);
	if (ret < 0)
		return ret;

	return amg88xx_write8(client, regh, (u8)(value >> 8));
}

/*
 * Handler for the threaded irq
 */
static irqreturn_t irq_handler(int irq, void *dev)
{
	struct amg88xx *device = dev;

	// Signal the userspace by notifying pollers on the 'interrupt' file
	sysfs_notify(&device->client->dev.kobj, NULL, "interrupt");

	return IRQ_HANDLED;
}

/*
 * Helper functions for device access
 */
static inline int amg88xx_set_dev_mode(struct amg88xx *dev,
				       enum amg88xx_device_mode mode)
{
	return amg88xx_write8(dev->client, DEVICE_MODE_REG, mode);
}

static int amg88xx_get_dev_mode(struct amg88xx *dev, int *result)
{
	int ret;

	ret = amg88xx_read8(dev->client, DEVICE_MODE_REG);
	if (ret < 0)
		return ret;
	else
		*result = ret;

	return 0;
}

static inline int amg88xx_reset(struct amg88xx *dev,
				enum amg88xx_reset_mode mode)
{
	return amg88xx_write8(dev->client, RESET_REG, mode);
}

static int amg88xx_get_framerate(struct amg88xx *dev, unsigned *framerate)
{
	int ret;

	ret = amg88xx_read8(dev->client, FRAMERATE_REG);
	if (ret < 0)
		return ret;
	else if (ret == FPS10)
		*framerate = 10;
	else if (ret == FPS1)
		*framerate = 1;
	else
		*framerate = 0;

	return 0;
}

static inline int amg88xx_set_framerate(struct amg88xx *dev,
					enum amg88xx_fps framerate)
{
	return amg88xx_write8(dev->client, FRAMERATE_REG, framerate);
}

static int amg88xx_get_int_conf(struct amg88xx *dev, int *mode, int *enabled)
{
	int ret;

	ret = amg88xx_read8(dev->client, INTERRUPT_CTRL_REG);
	if (ret < 0) {
		return ret;
	} else {
		*mode = ret & 0x2;
		*enabled = ret & 0x1;
	}

	return 0;
}

static int amg88xx_set_int_mode(struct amg88xx *dev,
				enum amg88xx_interrupt_mode mode)
{
	int ret;
	u8 val;

	ret = amg88xx_read8(dev->client, INTERRUPT_CTRL_REG);
	if (ret < 0) {
		return ret;
	} else {
		if (mode)
			val = ret | 0x2;
		else
			val = ret & ~(0x2);
	}

	return amg88xx_write8(dev->client, INTERRUPT_CTRL_REG, val);
}

static int amg88xx_set_int_state(struct amg88xx *dev,
				 enum amg88xx_interrupt_state state)
{
	int ret;
	u8 val;

	ret = amg88xx_read8(dev->client, INTERRUPT_CTRL_REG);
	if (ret < 0) {
		return ret;
	} else {
		if (state)
			val = ret | 0x1;
		else
			val = ret & ~(0x1);
	}

	return amg88xx_write8(dev->client, INTERRUPT_CTRL_REG, val);
}

static int amg88xx_get_int_upper_limit(struct amg88xx *dev, s16 *limit)
{
	int ret;

	ret = amg88xx_read16(dev->client, UPPER_INTERRUPT_LOW_REG,
			     UPPER_INTERRUPT_HIGH_REG);
	if (ret < 0)
		return ret;
	else
		convert_to_s16(*limit, ret);

	return 0;
}

static int amg88xx_get_int_lower_limit(struct amg88xx *dev, s16 *limit)
{
	int ret;

	ret = amg88xx_read16(dev->client, LOWER_INTERRUPT_LOW_REG,
			     LOWER_INTERRUPT_HIGH_REG);
	if (ret < 0)
		return ret;
	else
		convert_to_s16(*limit, ret);

	return 0;
}

static int amg88xx_get_int_hysteresis(struct amg88xx *dev, s16 *hysteresis)
{
	int ret;

	ret = amg88xx_read16(dev->client, INTERRUPT_HYST_LOW_REG,
			     INTERRUPT_HYST_HIGH_REG);
	if (ret < 0)
		return ret;
	else
		convert_to_s16(*hysteresis, ret);

	return 0;
}

static inline int amg88xx_set_int_upper_limit(struct amg88xx *dev, s16 limit)
{
	return amg88xx_write16(dev->client, UPPER_INTERRUPT_LOW_REG,
			       UPPER_INTERRUPT_HIGH_REG, limit);
}

static inline int amg88xx_set_int_lower_limit(struct amg88xx *dev, s16 limit)
{
	return amg88xx_write16(dev->client, LOWER_INTERRUPT_LOW_REG,
			       LOWER_INTERRUPT_HIGH_REG, limit);
}

static inline int amg88xx_set_int_hysteresis(struct amg88xx *dev,
					     s16 hysteresis)
{
	return amg88xx_write16(dev->client, INTERRUPT_HYST_LOW_REG,
			       INTERRUPT_HYST_HIGH_REG, hysteresis);
}

static int amg88xx_read_thermistor(struct amg88xx *dev, s16 *result)
{
	int ret;

	ret = amg88xx_read16(dev->client, THERM_LOW_REG, THERM_HIGH_REG);
	if (ret < 0)
		return ret;

	convert_to_s16(*result, ret);

	return 0;
}

static int amg88xx_read_sensor(struct amg88xx *dev, s16 *res_array)
{
	int i;
	int ret;
	u8 reg_addr = SENSOR_FIRST_REG;

	// Loop through all the sensor registers
	for (i = 0; i < 64; i++) {
		ret = amg88xx_read16(dev->client, reg_addr, reg_addr + 1);
		if (ret < 0)
			return ret;

		convert_to_s16(res_array[i], ret);
		reg_addr += 2;
	}

	return 0;
}

static int amg88xx_read_interrupt_map(struct amg88xx *dev, u8 *res_array)
{
	int i;
	int ret;
	u8 reg_addr = PIXEL_ROW1_REG;

	// Loop trough all the interrupt map registers
	for (i = 0; i < 8; i++) {
		ret = amg88xx_read8(dev->client, reg_addr);
		if (ret < 0)
			return ret;

		res_array[i] = ret;
		reg_addr++;
	}

	return 0;
}

/*
 * sysfs entries and the functions to implement them
 */
static ssize_t show_sensor(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct amg88xx *device;
	int ret;
	int row;
	int col;
	int nwrite;
	s16 sensor_array[64];
	unsigned index = 0;

	device = dev_get_drvdata(dev);

	ret = amg88xx_read_sensor(device, sensor_array);
	if (ret < 0) {
		dev_err(dev, "Failed to read the sensor\n");
		return ret;
	}

	for (row = 0; row < 8; row++) {
		for (col = 0; col < 8; col++) {
			/*
			 * Write all the values on a row. Each value is sepparated by a comma
			 * and there is newline character after the last value
			 */
			nwrite = scnprintf(&buf[index], PAGE_SIZE - index,
					   col < 7 ? "%d, " : "%d\n",
					   sensor_array[row * 8 + col]);
			index += nwrite;
		}
	}

	return index;
}
static DEVICE_ATTR(sensor, S_IRUGO, show_sensor, NULL);

static ssize_t show_thermistor(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct amg88xx *device;
	int ret;
	s16 thermistor_value;

	device = dev_get_drvdata(dev);

	ret = amg88xx_read_thermistor(device, &thermistor_value);
	if (ret < 0) {
		dev_err(dev, "Failed to read thermistor value\n");
		return ret;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", thermistor_value);
}
static DEVICE_ATTR(thermistor, S_IRUGO, show_thermistor, NULL);

static ssize_t show_device_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct amg88xx *device;
	int ret;
	int mode;
	const char *str;

	device = dev_get_drvdata(dev);

	ret = amg88xx_get_dev_mode(device, &mode);
	if (ret < 0) {
		dev_err(dev, "Failed to read device mode\n");
		return ret;
	}

	switch (mode) {
	case NORMAL_MODE:
		str = mode_strs[0];
		break;
	case SLEEP_MODE:
		str = mode_strs[1];
		break;
	case STANDBY60_MODE:
		str = mode_strs[2];
		break;
	case STANDBY10_MODE:
		str = mode_strs[3];
		break;
	default:
		dev_err(dev, "Unkown mode from hw\n");
		return -EREMOTEIO;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", str);
}

static ssize_t store_device_mode(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct amg88xx *device;
	int ret;
	enum amg88xx_device_mode mode;

	device = dev_get_drvdata(dev);

	if (sysfs_streq("normal", buf)) {
		mode = NORMAL_MODE;
	} else if (sysfs_streq("sleep", buf)) {
		mode = SLEEP_MODE;
	} else if (sysfs_streq("standby_60", buf)) {
		mode = STANDBY60_MODE;
	} else if (sysfs_streq("standby_10", buf)) {
		mode = STANDBY10_MODE;
	} else {
		dev_err(dev, "Input is not a supported mode\n");
		return -EINVAL;
	}

	ret = amg88xx_set_dev_mode(device, mode);
	if (ret < 0) {
		dev_err(dev, "Failed to set device mode\n");
		return ret;
	}

	return count;
}
static DEVICE_ATTR(device_mode, S_IRUGO | S_IWUSR | S_IWGRP, show_device_mode,
		   store_device_mode);

static ssize_t show_interrupt_mode(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct amg88xx *device;
	int ret;
	int mode;
	int enabled;

	device = dev_get_drvdata(dev);

	ret = amg88xx_get_int_conf(device, &mode, &enabled);
	if (ret < 0) {
		dev_err(dev, "Failed to read interrupt mode\n");
		return ret;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 mode ? "absolute" : "differential");
}

static ssize_t store_interrupt_mode(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct amg88xx *device;
	int ret;
	enum amg88xx_interrupt_mode mode;

	device = dev_get_drvdata(dev);

	if (sysfs_streq("absolute", buf)) {
		mode = ABSOLUTE_VALUE_MODE;
	} else if (sysfs_streq("differential", buf)) {
		mode = DIFFERENCE_MODE; // FIXME
	} else {
		dev_err(dev, "Invalid interrupt mode\n");
		return -EINVAL;
	}

	ret = amg88xx_set_int_mode(device, mode);
	if (ret < 0) {
		dev_err(dev, "Failed to set interrupt mode\n");
		return ret;
	}

	return count;
}
static DEVICE_ATTR(interrupt_mode, S_IRUGO | S_IWUSR | S_IWGRP,
		   show_interrupt_mode, store_interrupt_mode);

static ssize_t show_interrupt_state(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct amg88xx *device;
	int ret;
	int mode;
	int enabled;

	device = dev_get_drvdata(dev);

	ret = amg88xx_get_int_conf(device, &mode, &enabled);
	if (ret < 0) {
		dev_err(dev, "Failed to read interrupt mode\n");
		return ret;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 enabled ? "enabled" : "disabled");
}

static ssize_t store_interrupt_state(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct amg88xx *device;
	int ret;
	enum amg88xx_interrupt_state state;

	device = dev_get_drvdata(dev);

	if (sysfs_streq("enabled", buf)) {
		state = INT_ENABLED;
	} else if (sysfs_streq("disabled", buf)) {
		state = INT_DISABLED;
	} else {
		dev_err(dev, "Invalid interrupt state\n");
		return -EINVAL;
	}

	ret = amg88xx_set_int_state(device, state);
	if (ret < 0) {
		dev_err(dev, "Failed to set interrupt state\n");
		return ret;
	}

	return count;
}
static DEVICE_ATTR(interrupt_state, S_IRUGO | S_IWUSR | S_IWGRP,
		   show_interrupt_state, store_interrupt_state);

static ssize_t show_interrupt_levels(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct amg88xx *device;
	s16 upper;
	s16 lower;
	s16 hysteresis;
	int ret;

	device = dev_get_drvdata(dev);

	ret = amg88xx_get_int_upper_limit(device, &upper);
	if (ret < 0)
		return ret;

	ret = amg88xx_get_int_lower_limit(device, &lower);
	if (ret < 0)
		return ret;

	ret = amg88xx_get_int_hysteresis(device, &hysteresis);
	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d,%d,%d\n", upper, lower,
			 hysteresis);
}

static ssize_t store_interrupt_levels(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct amg88xx *device;
	char *temp;
	s16 values[3];
	int ret;
	int i;
	int index = 0;

	// Allocate a temporary buffer for substring handling
	temp = kmalloc(count, GFP_KERNEL);
	if (temp == NULL)
		return -ENOMEM;

	device = dev_get_drvdata(dev);

	for (i = 0; i < 3; i++) {
		const char *substr_end;
		size_t strl;

		/*
		 * Calculate the length of the substring and copy it adding a null
		 * terminator to the end
		 */
		substr_end = strchrnul(&buf[index], ',');
		strl = substr_end - &buf[index];

		strncpy(temp, &buf[index], strl);
		temp[strl] = '\0';

		// Convert the value to s16 number and check for upper limit
		ret = kstrtos16(temp, 10, &values[i]);
		if (ret < 0) {
			dev_err(dev, "Failed to read value for %s from input\n",
				i == 0 ? "upper limit" :
					 (i == 1 ? "lower limit" :
						   "hysteresis"));
			goto exit;
		}

		if (values[i] < -2048 || values[i] > 2047) {
			dev_err(dev, "Illegal input value for %s\n",
				i == 0 ? "upper limit" :
					 (i == 1 ? "lower limit" :
						   "hysteresis"));
			ret = -EINVAL;
			goto exit;
		}

		index += strl + 1;
	}

	ret = amg88xx_set_int_upper_limit(device, values[0]);
	if (ret < 0) {
		dev_err(dev, "Failed to set the interrupt upper limit\n");
		goto exit;
	}

	ret = amg88xx_set_int_lower_limit(device, values[1]);
	if (ret < 0) {
		dev_err(dev, "Failed to set the interrupt lower limit\n");
		goto exit;
	}

	ret = amg88xx_set_int_hysteresis(device, values[2]);
	if (ret < 0) {
		dev_err(dev, "Failed to set the interrupt hysteresis\n");
		goto exit;
	}

	ret = count;

exit:
	kfree(temp);

	return ret;
}
static DEVICE_ATTR(interrupt_levels, S_IRUGO | S_IWUSR | S_IWGRP,
		   show_interrupt_levels, store_interrupt_levels);

static ssize_t show_interrupt(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct amg88xx *device;
	int ret;

	device = dev_get_drvdata(dev);

	ret = gpiod_get_value(device->int_gpio);
	if (ret < 0) {
		dev_err(dev, "Failed to read interrupt gpio value\n");
		return ret;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 !ret ? "active" : "not_active");
}
static DEVICE_ATTR(interrupt, S_IRUGO, show_interrupt, NULL);

static ssize_t show_interrupt_map(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct amg88xx *device;
	int row;
	int col;
	int nwrite;
	int ret;
	u8 int_array[8];
	int index = 0;

	device = dev_get_drvdata(dev);

	ret = amg88xx_read_interrupt_map(device, int_array);
	if (ret < 0) {
		dev_err(dev, "Failed to read the interrupt map\n");
		return ret;
	}

	for (row = 0; row < 8; row++) {
		for (col = 0; col < 8; col++) {
			nwrite = scnprintf(&buf[index], PAGE_SIZE - index,
					   col < 7 ? "%s, " : "%s\n",
					   int_array[row] & 1 << col ? "1" :
								       "0");
			index += nwrite;
		}
	}

	return index;
}
static DEVICE_ATTR(interrupt_map, S_IRUGO, show_interrupt_map, NULL);

static ssize_t store_reset(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct amg88xx *device;
	enum amg88xx_reset_mode reset_mode;
	int ret;

	device = dev_get_drvdata(dev);

	if (sysfs_streq("full", buf)) {
		reset_mode = FULL_RST;
	} else if (sysfs_streq("partial", buf)) {
		reset_mode = PARTIAL_RST;
	} else {
		dev_err(dev, "Invalid reset mode\n");
		return -EINVAL;
	}

	ret = amg88xx_reset(device, reset_mode);
	if (ret < 0) {
		dev_err(dev, "Failed to reset device\n");
		return ret;
	}

	return count;
}
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, store_reset);

static ssize_t show_framerate(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct amg88xx *device;
	int framerate;
	int ret;

	device = dev_get_drvdata(dev);

	ret = amg88xx_get_framerate(device, &framerate);
	if (ret < 0) {
		dev_err(dev, "Failed to read framerate\n");
		return ret;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", framerate);
}

static ssize_t store_framerate(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct amg88xx *device;
	int fps;
	int ret;

	device = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &fps);
	if (ret < 0) {
		dev_err(dev, "Failed to read framerate from input\n");
		return ret;
	}

	if (!(fps == 1 || fps == 10)) {
		dev_err(dev, "Invalid framerate value\n");
		return -EINVAL;
	}

	ret = amg88xx_set_framerate(device, fps == 1 ? FPS1 : FPS10);
	if (ret < 0) {
		dev_err(dev, "Failed to set framerate\n");
		return ret;
	}

	return count;
}
static DEVICE_ATTR(framerate, S_IRUGO | S_IWUSR | S_IWGRP, show_framerate,
		   store_framerate);

// TODO all the rest of the sysfs stuff
static struct attribute *amg88xx_attrs[] = {
	&dev_attr_sensor.attr,
	&dev_attr_thermistor.attr,
	&dev_attr_device_mode.attr,
	&dev_attr_interrupt_mode.attr,
	&dev_attr_interrupt_state.attr,
	&dev_attr_interrupt_levels.attr,
	&dev_attr_interrupt.attr,
	&dev_attr_interrupt_map.attr,
	&dev_attr_reset.attr,
	&dev_attr_framerate.attr,
	// TODO create sysfs entries
	NULL,
};

static const struct attribute_group amg88xx_attr_group = {
	.attrs = amg88xx_attrs,
};

static int amg88xx_pinctrl_select_normal(struct amg88xx *dev)
{
	int ret = 0;
	AMG_FUNC_ENTER();

	if (dev->pinctrl && dev->pins_active) {
		ret = pinctrl_select_state(dev->pinctrl, dev->pins_active);
		if (ret < 0) {
			AMG_ERROR("Set normal pin state error:%d", ret);
		}
	}
	AMG_FUNC_EXIT();
	return ret;
}

static int amg88xx_pinctrl_select_suspend(struct amg88xx *dev)
{
	int ret = 0;
	AMG_FUNC_ENTER();

	if (dev->pinctrl && dev->pins_suspend) {
		ret = pinctrl_select_state(dev->pinctrl, dev->pins_suspend);
		if (ret < 0) {
			AMG_ERROR("Set suspend pin state error:%d", ret);
		}
	}
	AMG_FUNC_EXIT();
	return ret;
}

static int amg88xx_pinctrl_select_release(struct amg88xx *dev)
{
	int ret = 0;
	AMG_FUNC_ENTER();

	if (dev->pinctrl && dev->pins_release) {
		ret = pinctrl_select_state(dev->pinctrl, dev->pins_release);
		if (ret < 0) {
			AMG_ERROR("Set release pin state error:%d", ret);
		}
	}
	AMG_FUNC_EXIT();
	return ret;
}

static int amg88xx_pinctrl_init(struct amg88xx *dev)
{
	int ret = 0;
	AMG_FUNC_ENTER();

	dev->pinctrl = devm_pinctrl_get(dev->dev);
	if (IS_ERR_OR_NULL(dev->pinctrl)) {
		AMG_ERROR("Failed to get pinctrl, please check dts");
		ret = PTR_ERR(dev->pinctrl);
		goto err_pinctrl_get;
	}

	dev->pins_active =
		pinctrl_lookup_state(dev->pinctrl, "amg88xx_sens_active");
	if (IS_ERR_OR_NULL(dev->pins_active)) {
		AMG_ERROR("Pin state[active] not found");
		ret = PTR_ERR(dev->pins_active);
		goto err_pinctrl_lookup;
	}

	dev->pins_suspend =
		pinctrl_lookup_state(dev->pinctrl, "amg88xx_sens_suspend");
	if (IS_ERR_OR_NULL(dev->pins_suspend)) {
		AMG_ERROR("Pin state[suspend] not found");
		ret = PTR_ERR(dev->pins_suspend);
		goto err_pinctrl_lookup;
	}

	dev->pins_release =
		pinctrl_lookup_state(dev->pinctrl, "amg88xx_sens_release");
	if (IS_ERR_OR_NULL(dev->pins_release)) {
		AMG_ERROR("Pin state[release] not found");
		ret = PTR_ERR(dev->pins_release);
	}

	AMG_FUNC_EXIT();
	return 0;
err_pinctrl_lookup:
	if (dev->pinctrl) {
		devm_pinctrl_put(dev->pinctrl);
	}
err_pinctrl_get:
	dev->pinctrl = NULL;
	dev->pins_release = NULL;
	dev->pins_suspend = NULL;
	dev->pins_active = NULL;
	AMG_FUNC_EXIT();
	return ret;

	AMG_FUNC_EXIT();
	return 0;
}

static int amg88xx_power_source_ctrl(struct amg88xx *dev, int enable)
{
	int ret = 0;

	AMG_FUNC_ENTER();
	if (enable) {
		if (dev->power_disabled) {
			AMG_DEBUG("regulator enable !");

			if (!IS_ERR_OR_NULL(dev->vcc_i2c)) {
				ret = regulator_enable(dev->vcc_i2c);
				if (ret) {
					AMG_ERROR(
						"enable vcc_i2c regulator failed,ret=%d",
						ret);
				}
			}
			dev->power_disabled = false;
		}
	} else {
		if (!dev->power_disabled) {
			AMG_DEBUG("regulator disable !");
			if (!IS_ERR_OR_NULL(dev->vcc_i2c)) {
				ret = regulator_disable(dev->vcc_i2c);
				if (ret) {
					AMG_ERROR(
						"disable vcc_i2c regulator failed,ret=%d",
						ret);
				}
			}
			dev->power_disabled = true;
		}
	}

	AMG_FUNC_EXIT();
	return ret;
}

static int amg88xx_power_source_exit(struct amg88xx *dev)
{
	amg88xx_pinctrl_select_release(dev);

	amg88xx_power_source_ctrl(dev, DISABLE);

	if (!IS_ERR_OR_NULL(dev->vcc_i2c)) {
		if (regulator_count_voltages(dev->vcc_i2c) > 0)
			regulator_set_voltage(dev->vcc_i2c, 0,
					      AMG_I2C_VTG_MAX_UV);
		regulator_put(dev->vcc_i2c);
	}

	return 0;
}

static int amg88xx_power_source_init(struct amg88xx *dev)
{
	int ret = 0;
	AMG_FUNC_ENTER();
	dev->vcc_i2c = regulator_get(dev->dev, "vcc_i2c");
	if (!IS_ERR_OR_NULL(dev->vcc_i2c)) {
		AMG_ERROR("%s%d ", __func__, __LINE__);
		if (regulator_count_voltages(dev->vcc_i2c) > 0) {
			ret = regulator_set_voltage(dev->vcc_i2c,
						    AMG_I2C_VTG_MIN_UV,
						    AMG_I2C_VTG_MAX_UV);
			if (ret) {
				AMG_ERROR(
					"vcc_i2c regulator set_vtg failed,ret=%d",
					ret);
				regulator_put(dev->vcc_i2c);
			}
		}
	}

	amg88xx_pinctrl_init(dev);
	amg88xx_pinctrl_select_normal(dev);

	dev->power_disabled = true;
	ret = amg88xx_power_source_ctrl(dev, ENABLE);
	if (ret) {
		AMG_ERROR("fail to enable power(regulator)");
	}
	AMG_ERROR("%s%d ", __func__, __LINE__);
	AMG_FUNC_EXIT();
	return ret;
}

static int amg88xx_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct amg88xx *device = NULL;
	AMG_FUNC_ENTER();

	device = (struct amg88xx *)kzalloc(sizeof(*device), GFP_KERNEL);
	if (device == NULL) {
		AMG_ERROR("device is NULL");
		return -ENOMEM;
	} else {
		device->client = client;
	}

	device->dev = &client->dev;

	i2c_set_clientdata(client, device);

	ret = amg88xx_power_source_init(device);
	if (ret) {
		AMG_ERROR("power init fail");
		goto err_init;
	}

	device->int_gpio = devm_gpiod_get(&client->dev, "interrupt", GPIOD_IN);
	if (IS_ERR(device->int_gpio)) {
		AMG_ERROR("Failed to get a gpio line for interrupt");
		dev_err(&client->dev,
			"Failed to get a gpio line for interrupt\n");
		goto err_init;
	}

	ret = devm_request_threaded_irq(
		&client->dev, client->irq, NULL, irq_handler,
		IRQF_SHARED | IRQF_ONESHOT | IRQF_TRIGGER_FALLING, client->name,
		device);
	if (ret < 0) {
		AMG_ERROR("Failed to request a threaded irq");
		dev_err(&client->dev, "Failed to request a threaded irq\n");
		goto err_init;
	}

	dev_set_drvdata(&client->dev, device);

	ret = amg88xx_reset(device, PARTIAL_RST);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to reset device\n");
		goto err_init;
	}

	ret = sysfs_create_group(&client->dev.kobj, &amg88xx_attr_group);
	if (ret) {
		AMG_ERROR("[EX]: sysfs_create_group() failed!!");
		sysfs_remove_group(&client->dev.kobj, &amg88xx_attr_group);
		goto err_init;
	}
	AMG_INFO("INIT SUCCESS");

	AMG_FUNC_EXIT();
	return 0;
err_init:
	amg88xx_power_source_exit(device);
	AMG_FUNC_EXIT();
	return ret;
}

static int amg88xx_remove_entry(struct amg88xx *dev)
{
	amg88xx_set_dev_mode(dev, SLEEP_MODE);

	// remove sysfs
	sysfs_remove_group(&dev->client->dev.kobj, &amg88xx_attr_group);

	amg88xx_pinctrl_select_suspend(dev);
	amg88xx_power_source_exit(dev);
	return 0;
}

static int amg88xx_remove(struct i2c_client *client)
{
	return amg88xx_remove_entry(i2c_get_clientdata(client));
}

static const struct i2c_device_id amg88xx_id_table[] = {
	{ DRIVER_NAME, 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, amg88xx_id_table);

static const struct of_device_id amg88xx_of_match[] = {
	{
		.compatible = "panasonic,amg88xx",
	},
	{},
};
MODULE_DEVICE_TABLE(of, amg88xx_of_match);

static struct i2c_driver amg88xx_driver = {
	.probe = amg88xx_probe,
	.remove = amg88xx_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(amg88xx_of_match),
	},
	.id_table = amg88xx_id_table,
};

static int __init amg88xx_init(void)
{
	int ret = 0;
	AMG_FUNC_ENTER();
	ret = i2c_add_driver(&amg88xx_driver);
	if (ret != 0) {
		AMG_ERROR("amg88xx driver init failed");
	}
	AMG_FUNC_EXIT();
	return ret;
}

static void __exit amg88xx_exit(void)
{
	AMG_FUNC_ENTER();
	i2c_del_driver(&amg88xx_driver);
	AMG_FUNC_EXIT();
}

module_init(amg88xx_init);
module_exit(amg88xx_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("Iiro Vuorio <iiro.vuorio@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("A kernel driver for the Panasonic AMG88xx-series sensors");

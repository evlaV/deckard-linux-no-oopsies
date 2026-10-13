/*
 * Power Accumulator Driver for Maxim MAX34417
 *
 * Author: Erman Komurcu <erman.komurcu@maximintegrated.com>
 * Copyright (C) Maxim Integrated
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hwmon-sysfs.h>
#include <linux/regulator/consumer.h>

#define MAX34417_REG_UPDATE                0x00
#define MAX34417_REG_CONTROL               0x01
#define MAX34417_REG_ACC_COUNT             0x02
#define MAX34417_REG_POWER_ACC_BASE        0x03
#define MAX34417_REG_VOLTAGE_BASE          0x07
#define MAX34417_REG_DEV_ID                0x0F
#define MAX34417_DEV_ID                    0x38
#define MAX34417_DEV_ID_MASK               0x38
#define MAX34417_REG_BULK_POWER_READOUT    0x10
#define MAX34417_REG_BULK_VOLTAGE_READOUT  0x11

#define MAX34417_BULK_POWER_READOUT_LEN 	28
#define MAX34417_BULK_VOLTAGE_READOUT_LEN 	8

#define MAX34417_CHANNEL_COUNT			4

#define MAX34417_PWR_CORRECTION_SCALE		24
#define MAX34417_PWR_AVG_FULL_SCALE_BITS	30

#define MAX34417_VOLTAGE_CORRECTION_SCALE	24
#define MAX34417_VOLTAGE_FULL_SCALE_BITS	14

/*
 * The Perr/Verr correction trims a few LSB of voltage error off the power
 * average. A power-collapsed rail latches ~0 V while the accumulator still
 * holds power from when it was up. Below this voltage the trim would
 * dominate the reading and the unsigned math underflows, so skip it.
 */
#define MAX34417_PERR_VERR_MIN_MV		100

#define MAX34417_OVERFLOW_BITMASK		BIT(0)
#define MAX34417_SLOW_BITMASK			BIT(1)
#define MAX34417_PARK_0_MASK			BIT(2)
#define MAX34417_PARK_1_MASK			BIT(3)
#define MAX34417_PARK_EN_MASK			BIT(4)
#define MAX34417_SMM_BITMASK			BIT(5)
#define MAX34417_CAM_BITMASK			BIT(6)
#define MAX34417_MODE_BITMASK			BIT(7)

#define MAX34417_PARK_VALUE_MASK		0x0C
#define MAX34417_PARK_VALUE_POS			2

#define MAX34417_UPDATE_FLAG_FORCE		BIT(0)
#define MAX34417_UPDATE_FLAG_BROADCAST		BIT(1)

/* Addresses scanned */
static const unsigned short max34417_i2c_addrs[] = { 0x10, 0x12, 0x14,
		0x16, 0x18, 0x1A, 0x1C, 0x1E, I2C_CLIENT_END };

enum chip_id {
	ID_MAX34417
};

struct max34417_input {
	const char *label;
	int shunt_resistor;
	int correction;
	//TODO:
};

struct max34417_data {
	struct i2c_client *client;
	struct i2c_client *broadcaster;
	struct mutex update_lock;	/* Mutex for sync */
	u8 rsense;			/* Sense Resistor value in Ohms */
	u8 config;			/* Sensor configuration */
	u8 perr_verr;		/* Perr_Verr correction enable/disable */
	u32 correction;		/* Power correction multiplier */
	u8 accumulator_update_mode;
	struct regulator *vdd;
	struct regulator *vio;
	struct max34417_input inputs[MAX34417_CHANNEL_COUNT];
};

static int max34417_accumulator_read(struct device *dev, unsigned int *acc_count);
static int max34417_accumulator_update(struct device *dev, int channel, int flags);

static long max34417_power_calc(u32 correction, unsigned long long raw_power, unsigned int acc_count)
{
	unsigned long long scaled_avg;
	scaled_avg = div_u64(raw_power, acc_count) * 1000000 * correction;
	return scaled_avg >> MAX34417_PWR_AVG_FULL_SCALE_BITS;
}

static int max34417_calc_correction(u32 rsense, u32 * correction)
{
	switch (rsense) {
	case 100:
		*correction = 1 * MAX34417_PWR_CORRECTION_SCALE;
		break;
	case 50:
		*correction = 2 * MAX34417_PWR_CORRECTION_SCALE;
		break;
	case 40:
		*correction = 2.5 * MAX34417_PWR_CORRECTION_SCALE;
		break;
	case 25:
		*correction = 4 * MAX34417_PWR_CORRECTION_SCALE;
		break;
	case 20:
		*correction = 5 * MAX34417_PWR_CORRECTION_SCALE;
		break;
	case 15:
		*correction = 6.667f * MAX34417_PWR_CORRECTION_SCALE;
		break;
	case 10:
		*correction = 10 * MAX34417_PWR_CORRECTION_SCALE;
		break;
	case 5:
		*correction = 20 * MAX34417_PWR_CORRECTION_SCALE;
		break;
	case 4:
		*correction = 25 * MAX34417_PWR_CORRECTION_SCALE;
		break;
	case 2:
		*correction = 50 * MAX34417_PWR_CORRECTION_SCALE;
		break;
	case 1:
		*correction = 100 * MAX34417_PWR_CORRECTION_SCALE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const char * const measurement_modes[] = {
	"smm", "cam", "normal"
};

static const char * const park_modes[] = {
	"disabled", "enabled"
};

static const char * const perr_verr_modes[] = {
	"disabled", "enabled"
};

static const char * const park_values[] = {
	"1", "2", "3", "4"
};

static const char * const accumulator_update_modes[] = {
	"manual", "auto"
};

static ssize_t measurement_mode_show(struct device *dev,
				     struct device_attribute *devattr,
				     char *buf)
{
	const struct max34417_data *data = dev_get_drvdata(dev);

	if (data->config & MAX34417_SMM_BITMASK)
		return sprintf(buf, "smm\n");
	else if (data->config & MAX34417_CAM_BITMASK)
		return sprintf(buf, "cam\n");

	return sprintf(buf, "normal\n");
}

static ssize_t park_mode_show(struct device *dev,
			      struct device_attribute *devattr,
			      char *buf)
{
	const struct max34417_data *data = dev_get_drvdata(dev);

	if (data->config & MAX34417_PARK_EN_MASK)
		return sprintf(buf, "enabled\n");

	return sprintf(buf, "disabled\n");
}

static ssize_t park_value_show(struct device *dev,
			       struct device_attribute *devattr,
			       char *buf)
{
	const struct max34417_data *data = dev_get_drvdata(dev);

	int channel = (data->config & MAX34417_PARK_VALUE_MASK) >> MAX34417_PARK_VALUE_POS;

	return sprintf(buf, "%d\n", channel + 1);
}

static ssize_t perr_verr_show(struct device *dev,
			      struct device_attribute *devattr,
			      char *buf)
{
	const struct max34417_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", perr_verr_modes[data->perr_verr]);
}

static ssize_t measurement_mode_store(struct device *dev,
				      struct device_attribute *devattr,
				      const char *buf, size_t count)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	int mode, err;
	u8 new_cfg = data->config;

	if (mutex_lock_interruptible(&data->update_lock))
		return -ERESTARTSYS;

	mode = sysfs_match_string(measurement_modes, buf);
	if (mode < 0) {
		err = mode;
		goto error;
	}

	new_cfg &= ~(MAX34417_SMM_BITMASK | MAX34417_CAM_BITMASK);

	if (mode == 0)
		new_cfg |= MAX34417_SMM_BITMASK;
	else if (mode == 1)
		new_cfg |= MAX34417_CAM_BITMASK;

	err = i2c_smbus_write_byte_data(data->client,
					MAX34417_REG_CONTROL, new_cfg);
	if (err)
		goto error;

	data->config = new_cfg;

	err = i2c_smbus_write_byte(data->client, MAX34417_REG_UPDATE);
	if (err)
		goto error;

	mutex_unlock(&data->update_lock);
	return count;
error:
	mutex_unlock(&data->update_lock);
	return err;
}

static ssize_t park_mode_store(struct device *dev,
			       struct device_attribute *devattr,
			       const char *buf, size_t count)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	int mode, err;
	u8 new_cfg = data->config;

	if (mutex_lock_interruptible(&data->update_lock))
		return -ERESTARTSYS;

	mode = sysfs_match_string(park_modes, buf);
	if (mode < 0) {
		err = mode;
		goto error;
	}

	new_cfg &= ~(MAX34417_PARK_EN_MASK);

	if (mode != 0)
		new_cfg |= MAX34417_PARK_EN_MASK;

	err = i2c_smbus_write_byte_data(data->client,
					MAX34417_REG_CONTROL, new_cfg);
	if (err)
		goto error;

	data->config = new_cfg;

	err = i2c_smbus_write_byte(data->client, MAX34417_REG_UPDATE);
	if (err)
		goto error;

	mutex_unlock(&data->update_lock);
	return count;
error:
	mutex_unlock(&data->update_lock);
	return err;
}

static ssize_t perr_verr_store(struct device *dev,
			       struct device_attribute *devattr,
			       const char *buf, size_t count)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	int mode, err;

	if (mutex_lock_interruptible(&data->update_lock))
		return -ERESTARTSYS;

	mode = sysfs_match_string(perr_verr_modes, buf);
	if (mode < 0) {
		err = mode;
		goto error;
	}

	data->perr_verr = (u8)mode;

	mutex_unlock(&data->update_lock);
	return count;
error:
	mutex_unlock(&data->update_lock);
	return err;
}

static ssize_t park_value_store(struct device *dev,
				struct device_attribute *devattr,
				const char *buf, size_t count)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	int mode, err;
	u8 new_cfg = data->config;

	if (mutex_lock_interruptible(&data->update_lock))
		return -ERESTARTSYS;

	mode = sysfs_match_string(park_values, buf);
	if (mode <= 0 || mode > 4) {
		err = mode;
		goto error;
	}

	new_cfg &= ~(MAX34417_PARK_VALUE_MASK);
	new_cfg |= (mode) << MAX34417_PARK_VALUE_POS;

	err = i2c_smbus_write_byte_data(data->client,
					MAX34417_REG_CONTROL, new_cfg);
	if (err)
		goto error;

	data->config = new_cfg;

	err = i2c_smbus_write_byte(data->client, MAX34417_REG_UPDATE);
	if (err)
		goto error;

	mutex_unlock(&data->update_lock);
	return count;
error:
	mutex_unlock(&data->update_lock);
	return err;
}

static ssize_t accumulator_update_mode_show(struct device *dev,
			       struct device_attribute *devattr,
			       char *buf)
{
	const struct max34417_data *data = dev_get_drvdata(dev);

	if(data->accumulator_update_mode == 1)
		return sprintf(buf, "auto\n");

	return sprintf(buf, "manual\n");
}

static ssize_t accumulator_update_mode_store(struct device *dev,
			       struct device_attribute *devattr,
			       const char *buf, size_t count)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	int mode, err;

	if (mutex_lock_interruptible(&data->update_lock))
		return -ERESTARTSYS;

	mode = sysfs_match_string(accumulator_update_modes, buf);
	if (mode < 0) {
		err = mode;
		goto error;
	}

	data->accumulator_update_mode = mode; //0=manual, 1=auto

	mutex_unlock(&data->update_lock);
	return count;
error:
	mutex_unlock(&data->update_lock);
	return err;
}

static ssize_t accumulator_update_show(struct device *dev,
			       struct device_attribute *devattr,
			       char *buf)
{
	unsigned int accumulator = 0;
	int err = 0;

	err = max34417_accumulator_read(dev,&accumulator);

	if(err) accumulator = 0;

	return sprintf(buf, "%d\n", accumulator);
}

static ssize_t accumulator_update_store(struct device *dev,
			       struct device_attribute *devattr,
			       const char *buf, size_t count)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	int flags = MAX34417_UPDATE_FLAG_FORCE;
	int err;

	if (mutex_lock_interruptible(&data->update_lock))
		return -ERESTARTSYS;

	//do accumulator update
	err = max34417_accumulator_update(dev, 0, flags);

	mutex_unlock(&data->update_lock);

	return count;
}

static ssize_t accumulator_broadcast_update_store(struct device *dev,
			       struct device_attribute *devattr,
			       const char *buf, size_t count)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	int flags;
	int err;

	if (mutex_lock_interruptible(&data->update_lock))
		return -ERESTARTSYS;

	//do accumulator update
	flags = MAX34417_UPDATE_FLAG_FORCE | MAX34417_UPDATE_FLAG_BROADCAST;

	err = max34417_accumulator_update(dev, 0, flags);

	mutex_unlock(&data->update_lock);

	return count;
}

static ssize_t accumulator_broadcast_update_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	int addr = 0;

	if (data->broadcaster)
		addr = data->broadcaster->addr;

	return sprintf(buf, "0x%x\n", data->broadcaster->addr);
}

static SENSOR_DEVICE_ATTR(measurement_mode, 0644,
	measurement_mode_show, measurement_mode_store, 0);

static SENSOR_DEVICE_ATTR(park_mode, 0644,
		park_mode_show, park_mode_store, 0);

static SENSOR_DEVICE_ATTR(perr_verr_mode, 0644,
		perr_verr_show, perr_verr_store, 0);

static SENSOR_DEVICE_ATTR(park_value, 0644,
		park_value_show, park_value_store, 0);

static SENSOR_DEVICE_ATTR(accumulator_update, 0644,
		accumulator_update_show, accumulator_update_store, 0);

static SENSOR_DEVICE_ATTR(accumulator_broadcast_update, 0644,
		accumulator_broadcast_update_show,
		accumulator_broadcast_update_store, 0);

static SENSOR_DEVICE_ATTR(accumulator_update_mode, 0644,
		accumulator_update_mode_show, accumulator_update_mode_store, 0);

static const char * const slow_modes[] = {
	"disabled", "enabled",
};

static ssize_t slow_mode_show(struct device *dev,
			      struct device_attribute *devattr,
			      char *buf)
{
	const struct max34417_data *data = dev_get_drvdata(dev);

	if (data->config & MAX34417_SLOW_BITMASK)
		return sprintf(buf, "enabled\n");

	return sprintf(buf, "disabled\n");
}

static ssize_t slow_mode_store(struct device *dev,
			       struct device_attribute *devattr,
			       const char *buf, size_t count)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	int mode, err;
	u8 new_cfg = data->config;

	if (mutex_lock_interruptible(&data->update_lock))
		return -ERESTARTSYS;

	mode = sysfs_match_string(slow_modes, buf);
	if (mode < 0) {
		err = mode;
		goto error;
	}

	new_cfg &= ~(MAX34417_SLOW_BITMASK);

	if (mode == 1)
		new_cfg |= MAX34417_SLOW_BITMASK;

	err = i2c_smbus_write_byte_data(data->client,
					MAX34417_REG_CONTROL, new_cfg);
	if (err)
		goto error;

	data->config = new_cfg;

	err = i2c_smbus_write_byte(data->client, MAX34417_REG_UPDATE);
	if (err)
		goto error;

	mutex_unlock(&data->update_lock);
	return count;
error:
	mutex_unlock(&data->update_lock);
	return err;
}

static SENSOR_DEVICE_ATTR(slow_mode, 0644,
	slow_mode_show, slow_mode_store, 0);

static int max34417_read_string(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, const char **str)
{
	struct max34417_data *data = dev_get_drvdata(dev);

	*str = data->inputs[channel].label;

	return 0;
}

static int max34417_channel_label_get(const void *drvdata, int channel, const char **str)
{
	const struct max34417_data *data = drvdata;

	*str = data->inputs[channel].label;

	return 0;
}

static int max34417_accumulator_read(struct device *dev, unsigned int *acc_count)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	u8 buf[I2C_SMBUS_BLOCK_MAX] = { 0 };
	int err = 0;
	int regval;

	if(!data || !acc_count)
		goto error;

	regval = i2c_smbus_read_i2c_block_data(data->client,
						MAX34417_REG_ACC_COUNT,
						4,
						buf);

	if (regval < 0) {
		dev_err(dev, "Could not read accumulator data\n");
		err = -EIO;
		goto error;
	}

	*acc_count = (unsigned int)buf[3];
	*acc_count |= (unsigned int)buf[2] << 8UL;
	*acc_count |= (unsigned int)buf[1] << 16UL;

	if (*acc_count == 0) {
		dev_err(dev, "Accumulator count is 0\n");
		err = -EINVAL;
		goto error;
	}

	return 0;

error:
	return err;
}


static int max34417_accumulator_update(struct device *dev, int channel, int flags)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	struct i2c_client *client;
	int update_needed = 0;
	int normal_mode = 0;
	int smm_mode = 0;
	int cam_mode = 0;
	int err = 0;

	if(!data) {
		err = -EINVAL;
		goto error;
	}

	if (data->config & MAX34417_SMM_BITMASK) 
		smm_mode = 1;
	else if (data->config & MAX34417_CAM_BITMASK)
		cam_mode = 1;

	normal_mode = !smm_mode && !cam_mode;

	//0=manual mode, 1=auto mode
	if ( data->accumulator_update_mode == 1 )
	{
		if( channel == 0 )
		{
			// only send update command for channel 0
			// channel 0 must be read first, and then all other channels
			// this allows us to save and align all channel measurements
			update_needed = 1;
		}
	}

	if (flags & MAX34417_UPDATE_FLAG_BROADCAST && data->broadcaster)
		client = data->broadcaster;
	else
		client = data->client;

	if( update_needed || (flags & MAX34417_UPDATE_FLAG_FORCE))
	{
		err = i2c_smbus_write_byte(client, MAX34417_REG_UPDATE);

		if (err) goto error;

		// Wait for accumulator update
		usleep_range(1000, 2000);
	}

error:
	return err;
}

static int max34417_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	struct max34417_data *data = dev_get_drvdata(dev);
	int err, regval;
	unsigned int acc_count;
	unsigned int voltage, voltage_raw, voltage_adj, voltage_err;
	unsigned long long power, current_fs;
	u8 cfg;

	u8 buf[I2C_SMBUS_BLOCK_MAX] = { 0 };

	if (channel >= MAX34417_CHANNEL_COUNT)
		return -EINVAL;

	if (mutex_lock_interruptible(&data->update_lock))
		return -ERESTARTSYS;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			err = max34417_accumulator_update(dev, channel, 0);
			if (err) goto error;

			regval = i2c_smbus_read_i2c_block_data(data->client,
							   MAX34417_REG_VOLTAGE_BASE + channel,
							   3,
							   buf);

			if (regval < 0) {
				dev_err(dev, "Could not read voltage data\n");
				err = -EIO;
				goto error;
			}

			voltage = ((unsigned int)buf[1] << 8);
			voltage |= ((unsigned int)buf[2]);
			voltage >>= 2;

			voltage *= MAX34417_VOLTAGE_CORRECTION_SCALE * 1000;
			voltage >>= MAX34417_VOLTAGE_FULL_SCALE_BITS;
			*val = voltage;

			mutex_unlock(&data->update_lock);
			return 0;
		default:
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
	case hwmon_power:
		switch (attr) {
		case hwmon_power_input:
		case hwmon_power_average:
			if ((attr == hwmon_power_input &&
			     data->config & MAX34417_CAM_BITMASK) ||
					(attr == hwmon_power_average &&
					data->config & MAX34417_SMM_BITMASK)) {
				/* There is no data available on
				 * single measurement mode
				 * while CAM is enabled
				 */
				err = -ENODATA;
				goto error;
			}

			err = max34417_accumulator_update(dev, channel, 0);
			if (err) goto error;

			err = max34417_accumulator_read(dev, &acc_count);
			if (err) goto error;

			memset(buf, 0, I2C_SMBUS_BLOCK_MAX);

			regval = i2c_smbus_read_i2c_block_data(data->client,
							   MAX34417_REG_POWER_ACC_BASE + channel,
							   8,
							   buf);

			if (regval < 0) {
				dev_err(dev, "Could not read power accumulator value\n");
				err = -EIO;
				goto error;
			}

			power = (unsigned long long)buf[7];
			power |= ((unsigned long long)buf[6] << 8UL);
			power |= ((unsigned long long)buf[5] << 16UL);
			power |= ((unsigned long long)buf[4] << 24UL);
			power |= ((unsigned long long)buf[3] << 32UL);
			power |= ((unsigned long long)buf[2] << 40UL);
			power |= ((unsigned long long)buf[1] << 48UL);

			if (data->perr_verr && acc_count >= 1000) {
				regval = i2c_smbus_read_i2c_block_data(data->client,
							   MAX34417_REG_VOLTAGE_BASE + channel,
							   3, buf);

				if (regval < 0) {
					dev_err(dev, "Could not read voltage data\n");
					err = -EIO;
					goto error;
				}

				voltage_raw = ((unsigned int)buf[1] << 8);
				voltage_raw |= ((unsigned int)buf[2]);

				voltage = (voltage_raw >> 2) *
						MAX34417_VOLTAGE_CORRECTION_SCALE * 1000;
				voltage >>= MAX34417_VOLTAGE_FULL_SCALE_BITS;

				if (voltage >= MAX34417_PERR_VERR_MIN_MV &&
				    voltage <= 2000) {
					power  = div_u64(power, acc_count);
					current_fs = div_u64(power, (voltage_raw / 4));

					if (voltage < 1000)
						voltage_err = 3;
					else if (voltage < 1500)
						voltage_err = 2;
					else
						voltage_err = 1;

					voltage_adj = (voltage_raw / 4) - voltage_err;

					power = (current_fs * voltage_adj) * acc_count;
				}
			}

			struct max34417_input *input = &data->inputs[channel];
			*val = max34417_power_calc(input->correction, power, acc_count);

			if (data->config & MAX34417_CAM_BITMASK) {
				cfg = i2c_smbus_read_byte_data(data->client,
							       MAX34417_REG_CONTROL);
				if (cfg & MAX34417_OVERFLOW_BITMASK) {
					dev_err(dev, "Overflow on accumulator detected\n");
					cfg &= ~(MAX34417_OVERFLOW_BITMASK);
					err = i2c_smbus_write_byte_data(data->client,
									MAX34417_REG_CONTROL,
									cfg);
					if (err)
						goto error;
					err = i2c_smbus_write_byte(data->client,
								   MAX34417_REG_UPDATE);
					if (err)
						goto error;
				}
			}
			mutex_unlock(&data->update_lock);
			return 0;
		default:
			err = -EINVAL;
			goto error;
		}
	default:
		err = -EINVAL;
		goto error;
	}

error:
	mutex_unlock(&data->update_lock);
	return err;
}

static int max34417_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	return -EOPNOTSUPP;
}

static umode_t max34417_is_visible(const void *drvdata,
				   enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	const struct max34417_data *data = drvdata;
	const struct max34417_input *input = NULL;

	if (channel >= MAX34417_CHANNEL_COUNT)
		return -EINVAL;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_label:
			if (channel - 1 < MAX34417_CHANNEL_COUNT)
				input = &data->inputs[channel];
			return (input && input->label) ? 0444 : 0;
		case hwmon_in_input:
			return 0444;
		default:
			return 0;
		}
	case hwmon_power:
		switch (attr) {
		case hwmon_power_label:
			if (channel - 1 < MAX34417_CHANNEL_COUNT)
				input = &data->inputs[channel];
			return (input && input->label) ? 0444 : 0;
		case hwmon_power_input:
		case hwmon_power_average:
			return 0444;
		default:
			return 0;
		}
		break;
	default:
		break;
	}

	return 0;
}

static const u32 max34417_in_config[] = {
	HWMON_I_INPUT | HWMON_I_LABEL,
	HWMON_I_INPUT | HWMON_I_LABEL,
	HWMON_I_INPUT | HWMON_I_LABEL,
	HWMON_I_INPUT | HWMON_I_LABEL,
	0
};

static const struct hwmon_channel_info max34417_in = {
	.type = hwmon_in,
	.config = max34417_in_config,
};

static const u32 max34417_power_config[] = {
	HWMON_P_INPUT | HWMON_P_AVERAGE | HWMON_P_LABEL,
	HWMON_P_INPUT | HWMON_P_AVERAGE | HWMON_P_LABEL,
	HWMON_P_INPUT | HWMON_P_AVERAGE | HWMON_P_LABEL,
	HWMON_P_INPUT | HWMON_P_AVERAGE | HWMON_P_LABEL,
	0
};

static const struct hwmon_channel_info max34417_power = {
	.type = hwmon_power,
	.config = max34417_power_config,
};

static const struct hwmon_channel_info *max34417_info[] = {
	&max34417_in,
	&max34417_power,
	NULL
};

static const struct hwmon_ops max34417_hwmon_ops = {
	.is_visible = max34417_is_visible,
	.channel_label_get = max34417_channel_label_get,
	.read = max34417_read,
	.read_string = max34417_read_string,
	.write = max34417_write,
};

static const struct hwmon_chip_info max34417_chip_info = {
	.ops = &max34417_hwmon_ops,
	.info = max34417_info,
};

static struct attribute *max34417_hwmon_attributes[] = {
	&sensor_dev_attr_measurement_mode.dev_attr.attr,
	&sensor_dev_attr_park_mode.dev_attr.attr,
	&sensor_dev_attr_park_value.dev_attr.attr,
	&sensor_dev_attr_perr_verr_mode.dev_attr.attr,
	&sensor_dev_attr_slow_mode.dev_attr.attr,
	&sensor_dev_attr_accumulator_update.dev_attr.attr,
	&sensor_dev_attr_accumulator_broadcast_update.dev_attr.attr,
	&sensor_dev_attr_accumulator_update_mode.dev_attr.attr,
	NULL
};

static const struct attribute_group max34417_hwmon_group = {
	.attrs = max34417_hwmon_attributes,
};
__ATTRIBUTE_GROUPS(max34417_hwmon);

static void max34417_remove(void *data)
{
	const struct max34417_data *max31730 = data;
	const struct i2c_client *client = max31730->client;

	i2c_smbus_write_byte_data(client, MAX34417_REG_CONTROL, 0);
}

static int max34417_probe(struct i2c_client *client)
{
	 struct device_node *broadcaster_node;
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct max34417_data *data;
	struct max34417_input *input;
	char old_name[I2C_NAME_SIZE];
	struct device_node *np;
	u32 r_sense;
	uint regval;
	int err;

	data = devm_kzalloc(dev, sizeof(struct max34417_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	np = of_node_get(client->dev.of_node);

	if (!np) {
		dev_err(dev,
			"Could not find max34417 entry in device tree\n");
		return -ENODATA;
	}

	err = of_property_read_u32(np, "maxim,sense-resistor", &r_sense);
	if (err < 0) {
		dev_err(dev,
			"Could not find maxim,sense-resistor in device tree\n");
		return err;
	}

	data->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(data->vdd)) {
		dev_err(dev, "Failed to get vdd\n");
		return PTR_ERR(data->vdd);
	}

	data->vio = devm_regulator_get(dev, "vio");
	if (IS_ERR(data->vio)) {
		dev_err(dev, "Failed to get vio\n");
		return PTR_ERR(data->vio);
	};

	for_each_child_of_node_scoped(np, child) {
		u32 val;
		u32 shunt_resistor;
		int ret = of_property_read_u32(child, "reg", &val);
		if (ret) {
			dev_err(dev, "missing reg property of %pOFn\n", child);
			return ret;
		} else if (val > MAX34417_CHANNEL_COUNT) {
			dev_err(dev, "invalid reg %d of %pOFn\n", val, child);
			return -EINVAL;
		}

		input = &data->inputs[val];

		/* Save the connected input label if available */
		of_property_read_string(child, "label", &input->label);
		
		/* Get shunt resistor value if it exists for the input*/
		ret = of_property_read_u32(child, "shunt-resistor", &shunt_resistor);
		input->shunt_resistor = ret ? r_sense : shunt_resistor;

		u32 correction = 1;
		if (max34417_calc_correction(input->shunt_resistor, &correction))
		{
			//failed get proper shunt resistor correction
		}
		else
		{
			input->correction = correction;
		}
	}

	broadcaster_node = of_parse_phandle(client->dev.of_node,
					    "broadcaster-parent", 0);
	if (broadcaster_node) {
		data->broadcaster = of_find_i2c_device_by_node(broadcaster_node);
		of_node_put(broadcaster_node);
	}

	dev_set_drvdata(dev, data);

	data->client = client;
	data->perr_verr = 1; /* enabled by default */
	data->accumulator_update_mode = 1; //auto mode
	u32 correction = 1; //default value
	if (max34417_calc_correction(r_sense, &correction)) {
		dev_err(dev, "Could not calculate power correction for rs=%d\n",r_sense);
		return -EINVAL;
	}
	//this is the default r_sense value -- can be overriden by the shunt_resistor value for each channel
	data->rsense = r_sense;

	mutex_init(&data->update_lock);

	/* Enable the power */
	err = regulator_enable(data->vdd);
	if (err) {
		dev_err(dev, "failed to enable vdd regulator: %d\n", err);
		return err;
	}
	err = regulator_enable(data->vio);
	if (err) {
		dev_err(dev, "failed to enable vdd regulator: %d\n", err);
		return err;
	}

	// allow power up to complete
	usleep_range(4000,5000);

	regval = i2c_smbus_read_byte_data(client, MAX34417_REG_DEV_ID);

	/* Initializing in single shot measurement mode */
	err = i2c_smbus_write_byte_data(client, MAX34417_REG_CONTROL, 0xA0);
	if (err) {
		dev_err(dev, "i2c write failed\n");
		return err;
	}

	data->config = 0xA0;

	err = i2c_smbus_write_byte(client, MAX34417_REG_UPDATE);
	if (err) {
		dev_err(dev, "i2c write failed\n");
		return err;
	}

	err = devm_add_action_or_reset(dev, max34417_remove, data);
	if (err) {
		dev_err(dev, "devm add remove failed\n");
		return err;
	}

	strscpy(old_name, client->name, sizeof(old_name));
	snprintf(client->name, sizeof(client->name), "%s_%d_%x", old_name, i2c_adapter_id(client->adapter), client->addr);

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 data,
							 &max34417_chip_info,
							 max34417_hwmon_groups);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int max34417_detect(struct i2c_client *client,
			   struct i2c_board_info *info)
{
	int regval;

	regval = i2c_smbus_read_byte_data(client, MAX34417_REG_DEV_ID);
	if ((regval & MAX34417_DEV_ID_MASK) != MAX34417_DEV_ID)
		return -ENODEV;

	strncpy(info->type, "max34417", I2C_NAME_SIZE);

	return 0;
}

static const struct of_device_id __maybe_unused max34417_of_match[] = {
	{ .compatible = "maxim,max34417", },
	{ },
};
MODULE_DEVICE_TABLE(of, max34417_of_match);

static const struct i2c_device_id max34417_id_table[] = {
	{ "max34417", .driver_data = ID_MAX34417 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max34417_id_table);

static struct i2c_driver max34417_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "max34417",
		.of_match_table = of_match_ptr(max34417_of_match),
	},
	.probe			= max34417_probe,
	.detect			= max34417_detect,
	.id_table		= max34417_id_table,
	.address_list	= max34417_i2c_addrs,
};

module_i2c_driver(max34417_driver);

MODULE_AUTHOR("Erman Komurcu <Erman.Komurcu@maximintegrated.com>");
MODULE_DESCRIPTION("MAX34417 Power Accumulator Driver");
MODULE_LICENSE("GPL v2");

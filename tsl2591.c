// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 Joe Sandom <jgsandom@hotmail.co.uk>
 * 
 * Datasheet Available at: https://ams.com/tsl25911
 *
 * @brief Device driver for the TAOS TSL2591. This is a very-high sensitivity
 * light-to-digital converter that transforms light intensity into a digital
 * signal.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/debugfs.h>
#include <linux/delay.h>

#define als_time_secs_to_ms(x) ((x + 1) * 100)
#define als_time_ms_to_secs(x) ((x / 100) - 1)

/* TSL2591 Register Set */
#define TSL2591_ENABLE      0x00
#define TSL2591_CONTROL     0x01
#define TSL2591_AILTL       0x04
#define TSL2591_AILTH       0x05
#define TSL2591_AIHTL       0x06
#define TSL2591_AIHTH       0x07
#define TSL2591_NP_AILTL    0x08
#define TSL2591_NP_AILTH    0x09
#define TSL2591_NP_AIHTL    0x0A
#define TSL2591_NP_AIHTH    0x0B
#define TSL2591_PERSIST     0x0C
#define TSL2591_PACKAGE_ID  0x11
#define TSL2591_DEVICE_ID   0x12
#define TSL2591_STATUS      0x13
#define TSL2591_C0_DATAL    0x14
#define TSL2591_C0_DATAH    0x15
#define TSL2591_C1_DATAL    0x16
#define TSL2591_C1_DATAH    0x17

/* TSL2591 Command Register Masks */
#define TSL2591_CMD_NOP             0xA0
#define TSL2591_CMD_SF_INTSET       0xD4
#define TSL2591_CMD_SF_CALS_I       0xD5
#define TSL2591_CMD_SF_CALS_NPI     0xD7
#define TSL2591_CMD_SF_CNP_ALSI     0xDA

/* TSL2591 Enable Register Masks */
#define TSL2591_PWR_ON              0x01
#define TSL2591_PWR_OFF             0x00
#define TSL2591_ENABLE_ALS          0x02
#define TSL2591_ENABLE_ALS_INT      0x10
#define TSL2591_ENABLE_SLEEP_INT    0x40
#define TSL2591_ENABLE_NP_INT       0x80

/* TSL2591 Control Register Masks */
#define TSL2591_CTRL_ALS_INTEGRATION_100MS  0x00
#define TSL2591_CTRL_ALS_INTEGRATION_200MS  0x01
#define TSL2591_CTRL_ALS_INTEGRATION_300MS  0x02
#define TSL2591_CTRL_ALS_INTEGRATION_400MS  0x03
#define TSL2591_CTRL_ALS_INTEGRATION_500MS  0x04
#define TSL2591_CTRL_ALS_INTEGRATION_600MS  0x05
#define TSL2591_CTRL_ALS_LOW_GAIN           0x00
#define TSL2591_CTRL_ALS_MED_GAIN           0x10
#define TSL2591_CTRL_ALS_HIGH_GAIN          0x20
#define TSL2591_CTRL_ALS_MAX_GAIN           0x30
#define TSL2591_CTRL_SYS_RESET              0x80

/* TSL2591 Persist Register Masks */
#define TSL2591_PRST_ALS_INT_CYCLE_0        0x00
#define TSL2591_PRST_ALS_INT_CYCLE_ANY      0x01
#define TSL2591_PRST_ALS_INT_CYCLE_2        0x02
#define TSL2591_PRST_ALS_INT_CYCLE_3        0x03
#define TSL2591_PRST_ALS_INT_CYCLE_5        0x04
#define TSL2591_PRST_ALS_INT_CYCLE_10       0x05
#define TSL2591_PRST_ALS_INT_CYCLE_15       0x06
#define TSL2591_PRST_ALS_INT_CYCLE_20       0x07
#define TSL2591_PRST_ALS_INT_CYCLE_25       0x08
#define TSL2591_PRST_ALS_INT_CYCLE_30       0x09
#define TSL2591_PRST_ALS_INT_CYCLE_35       0x0A
#define TSL2591_PRST_ALS_INT_CYCLE_40       0x0B
#define TSL2591_PRST_ALS_INT_CYCLE_45       0x0C
#define TSL2591_PRST_ALS_INT_CYCLE_50       0x0D
#define TSL2591_PRST_ALS_INT_CYCLE_55       0x0E
#define TSL2591_PRST_ALS_INT_CYCLE_60       0x0F

/* TSL2591 PID Register Mask */
#define TSL2591_PACKAGE_ID_MASK 0x30

/* TSL2591 ID Register Mask */
#define TSL2591_DEVICE_ID_MASK  0xFF

/* TSL2591 Status Register Masks */
#define TSL2591_STS_ALS_VALID   0x00
#define TSL2591_STS_ALS_INT     0x10
#define TSL2591_STS_NPERS_INT   0x20

/* TSL2591 Constant Values */
#define TSL2591_PACKAGE_ID_VAL  0x00
#define TSL2591_DEVICE_ID_VAL   0x50

/* Power off suspend delay time MS */
#define TSL2591_POWER_OFF_DELAY_MS	2000

/* TSL2591 Default Values */
#define DEFAULT_ALS_INTEGRATION_TIME    TSL2591_CTRL_ALS_INTEGRATION_300MS
#define MAX_ALS_INTEGRATION_TIME_MS     600
#define DEFAULT_ALS_GAIN                TSL2591_CTRL_ALS_MED_GAIN
#define NUMBER_OF_DATA_CHANNELS         4

/* Literals */
#define TSL2591_CTRL_ALS_LOW_GAIN_LIT   "low"
#define TSL2591_CTRL_ALS_MED_GAIN_LIT   "med"
#define TSL2591_CTRL_ALS_HIGH_GAIN_LIT  "high"
#define TSL2591_CTRL_ALS_MAX_GAIN_LIT   "max"

static const u32 tsl2591_integration_opts[] = {
	TSL2591_CTRL_ALS_INTEGRATION_100MS,
	TSL2591_CTRL_ALS_INTEGRATION_200MS,
	TSL2591_CTRL_ALS_INTEGRATION_300MS,
	TSL2591_CTRL_ALS_INTEGRATION_400MS,
	TSL2591_CTRL_ALS_INTEGRATION_500MS,
	TSL2591_CTRL_ALS_INTEGRATION_600MS,
};

static const u32 tsl2591_gain_opts[] = {
	TSL2591_CTRL_ALS_LOW_GAIN,
	TSL2591_CTRL_ALS_MED_GAIN,
	TSL2591_CTRL_ALS_HIGH_GAIN,
	TSL2591_CTRL_ALS_MAX_GAIN,
};

static const u8 tsl2591_data_channels[] = {
	TSL2591_C0_DATAL,
	TSL2591_C0_DATAH,
	TSL2591_C1_DATAL,
	TSL2591_C1_DATAH,
};

struct tsl2591_als_readings {
	u16 als_ch0;
	u16 als_ch1;
};

struct tsl2591_settings {
	u8 als_int_time;
	u8 als_gain;
};

struct tsl2591_chip {
	struct mutex als_mutex;
	struct i2c_client *client;
	struct tsl2591_settings als_settings;
	struct tsl2591_als_readings als_readings;
};

static char *tsl2591_gain_to_str(const u8 als_gain)
{
	char *gain_str;

	switch (als_gain) {
	case TSL2591_CTRL_ALS_LOW_GAIN:
		gain_str = TSL2591_CTRL_ALS_LOW_GAIN_LIT;
		break;
	case TSL2591_CTRL_ALS_MED_GAIN:
		gain_str = TSL2591_CTRL_ALS_MED_GAIN_LIT;
		break;
	case TSL2591_CTRL_ALS_HIGH_GAIN:
		gain_str = TSL2591_CTRL_ALS_HIGH_GAIN_LIT;
		break;
	case TSL2591_CTRL_ALS_MAX_GAIN:
		gain_str = TSL2591_CTRL_ALS_MAX_GAIN_LIT;
		break;
	default:
		gain_str = "error";
		break;
	}

	return gain_str;
}

static int tsl2591_gain_from_str(const char *als_gain_str)
{
	if (strstr(als_gain_str, TSL2591_CTRL_ALS_LOW_GAIN_LIT) != NULL)
		return TSL2591_CTRL_ALS_LOW_GAIN;
	else if (strstr(als_gain_str, TSL2591_CTRL_ALS_MED_GAIN_LIT) != NULL)
		return TSL2591_CTRL_ALS_MED_GAIN;
	else if (strstr(als_gain_str, TSL2591_CTRL_ALS_HIGH_GAIN_LIT) != NULL)
		return TSL2591_CTRL_ALS_HIGH_GAIN;
	else if (strstr(als_gain_str, TSL2591_CTRL_ALS_MAX_GAIN_LIT) != NULL)
		return TSL2591_CTRL_ALS_MAX_GAIN;
	else
		return -EINVAL;
}

static int tsl2591_wait_adc_complete(struct tsl2591_chip *chip)
{
	struct tsl2591_settings settings = chip->als_settings;
	int delay = als_time_secs_to_ms(settings.als_int_time);

	if (!delay) {
		delay = MAX_ALS_INTEGRATION_TIME_MS;
		dev_warn(&chip->client->dev,
			"Failed to get int time, setting default delay: %d\n", delay);
	}

	msleep(delay);

	return 0;
}

static int tsl2591_get_lux_data(struct iio_dev *indio_dev)
{
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	struct i2c_client *client = chip->client;
	int i;
	int ret;
	u8 channel_data[NUMBER_OF_DATA_CHANNELS];

	tsl2591_wait_adc_complete(chip);

	for (i = 0; i < NUMBER_OF_DATA_CHANNELS; ++i) {
		int reg = TSL2591_CMD_NOP | tsl2591_data_channels[i];

		ret = i2c_smbus_read_byte_data(client, TSL2591_CMD_NOP | reg);

		if (ret < 0) {
			dev_err(&client->dev, "%s: failed to read register %x\n", __func__, reg);
			return -EINVAL;
		}
		channel_data[i] = ret;
	}

	chip->als_readings.als_ch0 = 
		le16_to_cpup((const __le16 *)&channel_data[0]);
	chip->als_readings.als_ch1 = 
		le16_to_cpup((const __le16 *)&channel_data[2]);

	return 0;
}

static int tsl2591_als_calibrate(struct tsl2591_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct tsl2591_settings als_settings = chip->als_settings;
	int ret;

	u8 configuration;

	dev_info(&client->dev, "Setting configuration - als_int_time: %x\n",
				als_settings.als_int_time);
	dev_info(&client->dev, "Setting configuration - als_gain: %x\n",
				als_settings.als_gain);

	configuration = als_settings.als_int_time | als_settings.als_gain;

	ret = i2c_smbus_write_byte_data(client,
		TSL2591_CMD_NOP | TSL2591_CONTROL, configuration);
	if (ret < 0)
		dev_err(&client->dev,
			"%s: failed to set configuration to %x\n", __func__,
			configuration);

	return ret;
}

static int tsl2591_set_power_state(struct tsl2591_chip *chip, u8 state)
{
	struct i2c_client *client = chip->client;
	int ret;

	ret = i2c_smbus_write_byte_data(client, 
		TSL2591_CMD_NOP | TSL2591_ENABLE, state);
	if (ret < 0)
		dev_err(&client->dev,
			"%s: failed to set the power state to %x\n", __func__,
			state);

	return ret;
}

static int tsl2591_set_pm_runtime_busy(struct tsl2591_chip *chip, bool busy)
{
	int ret;

	if (busy) {
		ret = pm_runtime_get_sync(&chip->client->dev);
		if (ret < 0)
			pm_runtime_put_noidle(&chip->client->dev);
	} else {
		pm_runtime_mark_last_busy(&chip->client->dev);
		ret = pm_runtime_put_autosuspend(&chip->client->dev);
	}

	return ret;
}

static ssize_t in_illuminance_integration_time_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	int ret;

	int als_int_time = als_time_secs_to_ms(chip->als_settings.als_int_time);

	mutex_lock(&chip->als_mutex);
	ret = sprintf(buf, "%d\n", als_int_time);
	mutex_unlock(&chip->als_mutex);

	return ret;
}

static ssize_t in_illuminance_integration_time_store(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	struct i2c_client *client = chip->client;

	u8 int_time;
	int value;
	int i;

	if (kstrtoint(buf, 0, &value) || !value)
		return -EINVAL;

	mutex_lock(&chip->als_mutex);

	int_time = als_time_ms_to_secs(value);

	for (i = 0; i < ARRAY_SIZE(tsl2591_integration_opts); ++i) {
		if (tsl2591_integration_opts[i] == int_time) {
			chip->als_settings.als_int_time = int_time;
			break;
		}
		if (i == (ARRAY_SIZE(tsl2591_integration_opts) - 1)) {
			goto calibrate_error;
		}
	}

	if (tsl2591_als_calibrate(chip)) {
		goto calibrate_error;
	}

	mutex_unlock(&chip->als_mutex);

	return len;

calibrate_error:
	dev_err(&client->dev, "Failed to calibrate sensor\n");
	mutex_unlock(&chip->als_mutex);
	return -EINVAL;
}

static ssize_t in_illuminance_gain_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	int ret;

	char *gain = tsl2591_gain_to_str(chip->als_settings.als_gain);

	mutex_lock(&chip->als_mutex);
	ret = sprintf(buf, "%s\n", gain);
	mutex_unlock(&chip->als_mutex);

	return ret;
}

static ssize_t in_illuminance_gain_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	struct i2c_client *client = chip->client;

	int gain;

	if (!buf)
		return -EINVAL;

	mutex_lock(&chip->als_mutex);

	gain = tsl2591_gain_from_str(buf);
	if (gain == -EINVAL)
		goto calibrate_error;

	chip->als_settings.als_gain = gain;

	if (tsl2591_als_calibrate(chip))
		goto calibrate_error;

	mutex_unlock(&chip->als_mutex);

	return len;

calibrate_error:
	dev_err(&client->dev, "Failed to calibrate sensor\n");
	mutex_unlock(&chip->als_mutex);
	return -EINVAL;
}

static IIO_CONST_ATTR(in_illuminance_integration_time_available_ms,
				"100 200 300 400 500 600");
static IIO_CONST_ATTR(in_illuminance_gain_available,
				"low med high max");
static IIO_DEVICE_ATTR_RW(in_illuminance_integration_time, 0);
static IIO_DEVICE_ATTR_RW(in_illuminance_gain, 0);

static struct attribute *sysfs_attrs_ctrl[] = {
	&iio_const_attr_in_illuminance_integration_time_available_ms.dev_attr.attr,
	&iio_const_attr_in_illuminance_gain_available.dev_attr.attr,
	&iio_dev_attr_in_illuminance_integration_time.dev_attr.attr,
	&iio_dev_attr_in_illuminance_gain.dev_attr.attr,
	NULL
};

static const struct attribute_group tsl2591_attribute_group = {
	.attrs = sysfs_attrs_ctrl,
};

static const struct iio_chan_spec tsl2591_channels[] = {
	{
		.type = IIO_LIGHT,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_LIGHT,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_BOTH,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static int tsl2591_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	struct i2c_client *client = chip->client;
	int ret, pm_ret;

	dev_dbg(&client->dev, "Reading from sensor");

	ret = tsl2591_set_pm_runtime_busy(chip, true);

	if (ret < 0)
		return ret;

	mutex_lock(&chip->als_mutex);

	ret = -EINVAL;
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_LIGHT) {
			ret = tsl2591_get_lux_data(indio_dev);
			if (ret < 0)
				break;

			if (chan->channel2 == IIO_MOD_LIGHT_BOTH)
				*val = chip->als_readings.als_ch0;
			else if (chan->channel2 == IIO_MOD_LIGHT_IR)
				*val = chip->als_readings.als_ch1;
			else
				*val = 0;

			ret = IIO_VAL_INT;
		}
			break;
	}

	mutex_unlock(&chip->als_mutex);

	pm_ret = tsl2591_set_pm_runtime_busy(chip, false);
	if (pm_ret < 0)
		return pm_ret;

	return ret;
}

static int tsl2591_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	int ret;

	ret = tsl2591_set_pm_runtime_busy(chip, true);
	if (ret < 0)
		return ret;

	mutex_lock(&chip->als_mutex);

	printk("Writing value to device.\n");

	mutex_unlock(&chip->als_mutex);

	ret = tsl2591_set_pm_runtime_busy(chip, false);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct iio_info tsl2591_info = {
	.attrs = &tsl2591_attribute_group,
	.read_raw = tsl2591_read_raw,
	.write_raw = tsl2591_write_raw,
};

static int __maybe_unused tsl2591_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	int ret;

	mutex_lock(&chip->als_mutex);

	dev_dbg(dev, "PM Suspending\n");
	ret = tsl2591_set_power_state(chip, TSL2591_PWR_OFF);

	mutex_unlock(&chip->als_mutex);

	return ret;
}

static int __maybe_unused tsl2591_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	int ret;

	mutex_lock(&chip->als_mutex);

	dev_dbg(dev, "PM Resuming\n");
	ret = tsl2591_set_power_state(chip, TSL2591_PWR_ON |
				TSL2591_ENABLE_ALS |
				TSL2591_ENABLE_ALS_INT);

	mutex_unlock(&chip->als_mutex);

	return ret;
}

static const struct dev_pm_ops tsl2591_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(tsl2591_suspend, tsl2591_resume, NULL)
};

#ifdef CONFIG_OF
static void tsl2591_compatible_int_time(struct tsl2591_chip *chip, 
					u32 als_integration_time)
{
	struct device *dev = &chip->client->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(tsl2591_integration_opts); ++i) {
		if (tsl2591_integration_opts[i] == als_integration_time) {
			chip->als_settings.als_int_time = als_integration_time;
			break;
		}
		if (i == (ARRAY_SIZE(tsl2591_integration_opts) - 1)) {
			dev_warn(dev, "incompatible als-integration-time. 	Setting default: %d\n", DEFAULT_ALS_INTEGRATION_TIME);
			chip->als_settings.als_int_time = DEFAULT_ALS_INTEGRATION_TIME;
		}
	}
}

static void tsl2591_compatible_gain(struct tsl2591_chip *chip, u32 als_gain)
{
	struct device *dev = &chip->client->dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(tsl2591_gain_opts); ++i) {
		if (tsl2591_gain_opts[i] == als_gain) {
			chip->als_settings.als_gain = als_gain;
			break;
		}
		if (i == (ARRAY_SIZE(tsl2591_gain_opts) - 1)) {
			dev_warn(dev, "incompatible als-gain. Setting default: %d\n", DEFAULT_ALS_GAIN);
			chip->als_settings.als_gain = DEFAULT_ALS_GAIN;
		}
	}
}

static int tsl2591_probe_of(struct tsl2591_chip *chip)
{
	struct device *dev = &chip->client->dev;
	struct device_node *np = dev->of_node;
	int ret;

	u32 als_integration_time;
	u32 als_gain;

	if (!np)
		return -ENODEV;

	dev_dbg(dev, "Probing device tree\n");

	ret = of_property_read_u32(np, "als-integration-time",
		&als_integration_time);

	if (ret) {
		dev_warn(dev, "als-integration-time not defined. Setting 		default: %d\n", DEFAULT_ALS_INTEGRATION_TIME);
		chip->als_settings.als_int_time = DEFAULT_ALS_INTEGRATION_TIME;
	} else {
		tsl2591_compatible_int_time(chip, als_integration_time);
		dev_info(dev, "als-integration-time = %d\n",
			chip->als_settings.als_int_time);
	}

	ret = of_property_read_u32(np, "als-gain", &als_gain);

	if (ret) {
		dev_warn(dev, "als-gain not defined. Setting default: %d\n",
			DEFAULT_ALS_GAIN);
		chip->als_settings.als_int_time = DEFAULT_ALS_GAIN;
	} else {
		tsl2591_compatible_gain(chip, als_gain);
		dev_info(dev, "als-gain = %d\n", chip->als_settings.als_gain);
	}

	return 0;
}
#else
static int tsl2591_default_config(struct tsl2591_chip *chip)
{
	struct device *dev = &chip->client->dev;

	dev_dbg(dev, "Loading default configuration\n");

	chip->als_settings.als_int_time = DEFAULT_ALS_INTEGRATION_TIME;
	chip->als_settings.als_int_time = DEFAULT_ALS_GAIN;

	dev_dbg(dev, "als-integration-time = %d\n", chip->als_settings,			als_int_time);
	dev_dbg(dev, "als-gain = %d\n", chip->als_settings.als_gain);

	return 0;
}
#endif

static int tsl2591_probe(struct i2c_client *client)
{
	int ret = 0;
	struct tsl2591_chip *chip;
	struct iio_dev *indio_dev;

	dev_info(&client->dev, "Start probing device.\n");

	if (!i2c_check_functionality(client->adapter, 
		I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "%s: i2c smbus byte data functionality is 	not supported\n", __func__);
		return -EOPNOTSUPP;
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	chip = iio_priv(indio_dev);
	chip->client = client;
	i2c_set_clientdata(client, indio_dev);

#ifdef CONFIG_OF
	if (tsl2591_probe_of(chip)) {
		dev_err(&client->dev, "No platform data\n");
		return -ENODEV;
	}
#else
	if (tsl2591_default_config(chip)) {
		dev_err(&client->dev, "Failed to load default config\n");
		return -EINVAL;
	}
#endif

	mutex_init(&chip->als_mutex);

	ret = i2c_smbus_read_byte_data(client, 
		TSL2591_CMD_NOP | TSL2591_DEVICE_ID);

	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to read the device ID 		register\n", __func__);
		return ret;
	}

	if ((ret & TSL2591_DEVICE_ID_MASK) != TSL2591_DEVICE_ID_VAL) {
		dev_err(&client->dev, "%s: received an unknown device ID %x\n",
			__func__, ret);
		return -EINVAL;
	}

	indio_dev->info = &tsl2591_info;
	indio_dev->channels = tsl2591_channels;
	indio_dev->num_channels = ARRAY_SIZE(tsl2591_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = chip->client->name;

	pm_runtime_enable(&client->dev);
	pm_runtime_set_autosuspend_delay(&client->dev,
					 TSL2591_POWER_OFF_DELAY_MS);
	pm_runtime_use_autosuspend(&client->dev);

	ret = devm_iio_device_register(&client->dev, indio_dev);

	if (ret) {
		dev_err(&client->dev, "%s: iio registration failed\n",
			__func__);
		return ret;
	}

	if (tsl2591_als_calibrate(chip)) {
		dev_err(&client->dev, "Failed to calibrate sensor\n");
		return -EINVAL;
	}

	dev_info(&client->dev, "Probe complete - tsl2591 found.\n");

	return 0;
}

static int tsl2591_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct tsl2591_chip *chip = iio_priv(indio_dev);

	dev_info(&client->dev, "Removing device.\n");

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_put_noidle(&client->dev);

	return tsl2591_set_power_state(chip, TSL2591_PWR_OFF);
}

static const struct of_device_id tsl2591_of_match[] = {
	{ .compatible = "amstaos,tsl2591", },
	{},
};
MODULE_DEVICE_TABLE(of, tsl2591_of_match);

static struct i2c_driver tsl2591_driver = {
	.driver = {
		.name = "tsl2591",
		.pm = &tsl2591_pm_ops,
		.of_match_table = of_match_ptr(tsl2591_of_match),
	},
	.probe_new = tsl2591_probe,
	.remove = tsl2591_remove,
};
module_i2c_driver(tsl2591_driver);

MODULE_AUTHOR("Joe Sandom <jgsandom@hotmail.co.uk>");
MODULE_DESCRIPTION("TAOS tsl2591 ambient light sensor driver");
MODULE_LICENSE("GPL");

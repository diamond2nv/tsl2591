// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 Joe Sandom <joe.sandom@outlook.com>
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
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

#define als_time_secs_to_ms(x) (((x) + 1) * 100)
#define als_time_ms_to_secs(x) (((x) / 100) - 1)

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
#define TSL2591_CMD_SF_INTSET       0xE4
#define TSL2591_CMD_SF_CALS_I       0xE5
#define TSL2591_CMD_SF_CALS_NPI     0xE7
#define TSL2591_CMD_SF_CNP_ALSI     0xEA

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
#define TSL2591_STS_ALS_VALID   0x01
#define TSL2591_STS_ALS_INT     0x10
#define TSL2591_STS_NPERS_INT   0x20
#define TSL2591_STS_VAL_HIGH    0x01

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
#define DEFAULT_ALS_PERSIST             TSL2591_PRST_ALS_INT_CYCLE_ANY
#define DEFAULT_ALS_LOWER_THRESHOLD	100
#define DEFAULT_ALS_UPPER_THRESHOLD	1500

#define ALS_MIN_VALUE	0
#define ALS_MAX_VALUE	65535
#define ALS_STS_VALID_COUNT 10

/* Literals */
#define TSL2591_CTRL_ALS_LOW_GAIN_LIT   "low"
#define TSL2591_CTRL_ALS_MED_GAIN_LIT   "med"
#define TSL2591_CTRL_ALS_HIGH_GAIN_LIT  "high"
#define TSL2591_CTRL_ALS_MAX_GAIN_LIT   "max"

/* LUX Calculations */
/* AGAIN values from Adafruits TSL2591 Arduino library */
/* https://github.com/adafruit/Adafruit_TSL2591_Library */
#define TSL2591_CTRL_ALS_LOW_AGAIN   1
#define TSL2591_CTRL_ALS_MED_AGAIN   25
#define TSL2591_CTRL_ALS_HIGH_AGAIN  428
#define TSL2591_CTRL_ALS_MAX_AGAIN   9876
#define TSL2591_LUX_COEFFICIENT      408

static const u8 tsl2591_data_channels[] = {
	TSL2591_C0_DATAL,
	TSL2591_C0_DATAH,
	TSL2591_C1_DATAL,
	TSL2591_C1_DATAH,
};

struct tsl2591_als_readings {
	u16 als_ch0;
	u16 als_ch1;
	u16 als_visible;
	u16 als_lux_int;
	u16 als_lux_decimal;
};

struct tsl2591_settings {
	u8 als_int_time;
	u8 als_gain;
	u8 als_persist;
	u16 als_lower_threshold;
	u16 als_upper_threshold;
};

struct tsl2591_chip {
	/* Ambient light sensor mutex */
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

static int tsl2591_gain_to_again(const u8 als_gain)
{
	int a_gain;

	switch (als_gain) {
	case TSL2591_CTRL_ALS_LOW_GAIN:
		a_gain = TSL2591_CTRL_ALS_LOW_AGAIN;
		break;
	case TSL2591_CTRL_ALS_MED_GAIN:
		a_gain = TSL2591_CTRL_ALS_MED_AGAIN;
		break;
	case TSL2591_CTRL_ALS_HIGH_GAIN:
		a_gain = TSL2591_CTRL_ALS_HIGH_AGAIN;
		break;
	case TSL2591_CTRL_ALS_MAX_GAIN:
		a_gain = TSL2591_CTRL_ALS_MAX_AGAIN;
		break;
	default:
		a_gain = -EINVAL;
		break;
	}

	return a_gain;
}

static int tsl2591_gain_from_str(const char *als_gain_str)
{
	if (strstr(als_gain_str, TSL2591_CTRL_ALS_LOW_GAIN_LIT))
		return TSL2591_CTRL_ALS_LOW_GAIN;
	else if (strstr(als_gain_str, TSL2591_CTRL_ALS_MED_GAIN_LIT))
		return TSL2591_CTRL_ALS_MED_GAIN;
	else if (strstr(als_gain_str, TSL2591_CTRL_ALS_HIGH_GAIN_LIT))
		return TSL2591_CTRL_ALS_HIGH_GAIN;
	else if (strstr(als_gain_str, TSL2591_CTRL_ALS_MAX_GAIN_LIT))
		return TSL2591_CTRL_ALS_MAX_GAIN;
	else
		return -EINVAL;
}

static int tsl2591_compatible_int_time(struct tsl2591_chip *chip,
				      u32 als_integration_time)
{
	switch (als_integration_time) {
	case TSL2591_CTRL_ALS_INTEGRATION_100MS:
	case TSL2591_CTRL_ALS_INTEGRATION_200MS:
	case TSL2591_CTRL_ALS_INTEGRATION_300MS:
	case TSL2591_CTRL_ALS_INTEGRATION_400MS:
	case TSL2591_CTRL_ALS_INTEGRATION_500MS:
	case TSL2591_CTRL_ALS_INTEGRATION_600MS:
		chip->als_settings.als_int_time = als_integration_time;
		return 0;
	default:
		break;
	}

	return -EINVAL;
}

static int tsl2591_compatible_gain(struct tsl2591_chip *chip, u32 als_gain)
{
	switch (als_gain) {
	case TSL2591_CTRL_ALS_LOW_GAIN:
	case TSL2591_CTRL_ALS_MED_GAIN:
	case TSL2591_CTRL_ALS_HIGH_GAIN:
	case TSL2591_CTRL_ALS_MAX_GAIN:
		chip->als_settings.als_gain = als_gain;
		return 0;
	default:
		break;
	}

	return -EINVAL;
}

static int tsl2591_compatible_als_persist(struct tsl2591_chip *chip,
					 u32 als_persist)
{
	switch (als_persist) {
	case TSL2591_PRST_ALS_INT_CYCLE_0:
	case TSL2591_PRST_ALS_INT_CYCLE_ANY:
	case TSL2591_PRST_ALS_INT_CYCLE_2:
	case TSL2591_PRST_ALS_INT_CYCLE_3:
	case TSL2591_PRST_ALS_INT_CYCLE_5:
	case TSL2591_PRST_ALS_INT_CYCLE_10:
	case TSL2591_PRST_ALS_INT_CYCLE_15:
	case TSL2591_PRST_ALS_INT_CYCLE_20:
	case TSL2591_PRST_ALS_INT_CYCLE_25:
	case TSL2591_PRST_ALS_INT_CYCLE_30:
	case TSL2591_PRST_ALS_INT_CYCLE_35:
	case TSL2591_PRST_ALS_INT_CYCLE_40:
	case TSL2591_PRST_ALS_INT_CYCLE_45:
	case TSL2591_PRST_ALS_INT_CYCLE_50:
	case TSL2591_PRST_ALS_INT_CYCLE_55:
	case TSL2591_PRST_ALS_INT_CYCLE_60:
		chip->als_settings.als_persist = als_persist;
		return 0;
	default:
		break;
	}

	return -EINVAL;
}

static int tsl2591_wait_adc_complete(struct tsl2591_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct tsl2591_settings settings = chip->als_settings;
	int delay = als_time_secs_to_ms(settings.als_int_time);
	int ret;
	int i;

	if (!delay) {
		delay = MAX_ALS_INTEGRATION_TIME_MS;
		dev_warn(&chip->client->dev,
			"Failed to get int time, setting default delay: %d\n",
			delay);
	}

	/*
	 * Sleep for als integration time to allow enough time
	 * for an ADC read cycle to complete. Check status after
	 * delay for als valid
	 */
	msleep(delay);

	/* Check for status als valid flag for up to 100ms */
	for (i = 0; i < ALS_STS_VALID_COUNT; ++i) {
		ret = i2c_smbus_read_byte_data(client, TSL2591_CMD_NOP |
			TSL2591_STATUS);

		if (ret < 0) {
			dev_err(&client->dev, "%s:failed to read register\n", __func__);
			return -EINVAL;
		}

		if ((ret & TSL2591_STS_ALS_VALID) == TSL2591_STS_VAL_HIGH)
			break;

		if (i == (ALS_STS_VALID_COUNT - 1))
			return -ENODATA;

		usleep_range(9000, 10000);
	}

	return 0;
}

/**
 * tsl2591_get_lux_data() - Reads raw channel data and calculates lux
 *
 * Formula for lux calculation;
 * Derived from Adafruit's TSL2591 library
 * Link: https://github.com/adafruit/Adafruit_TSL2591_Library
 * Counts Per Lux (CPL) = (ATIME_ms * AGAIN) / LUX DF
 * lux = ((C0DATA - C1DATA) * (1 - (C1DATA / C0DATA))) / CPL
 *
 * Scale values to get more representative value of lux i.e.
 * lux = ((C0DATA - C1DATA) * (1000 - ((C1DATA * 1000) / C0DATA))) / CPL
 *
 * Channel 0 = IR + Visible
 * Channel 1 = IR only
 * Visible = Channel 0 - Channel 1
 *
 */
static int tsl2591_get_lux_data(struct iio_dev *indio_dev)
{
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	struct tsl2591_settings *settings = &chip->als_settings;
	struct i2c_client *client = chip->client;
	int i;
	int ret;
	u8 channel_data[NUMBER_OF_DATA_CHANNELS];

	int counts_per_lux;
	int lux;

	ret = tsl2591_wait_adc_complete(chip);
	if (ret < 0) {
		dev_warn(&client->dev, "No data available. Err: %d\n", ret);
		return -ENODATA;
	}

	for (i = 0; i < NUMBER_OF_DATA_CHANNELS; ++i) {
		int reg = TSL2591_CMD_NOP | tsl2591_data_channels[i];

		ret = i2c_smbus_read_byte_data(client, TSL2591_CMD_NOP | reg);

		if (ret < 0) {
			dev_err(&client->dev,
				"%s: failed to read register %#04x\n",
				__func__, reg);
			return -EINVAL;
		}
		channel_data[i] = ret;
	}

	chip->als_readings.als_ch0 =
		le16_to_cpup((const __le16 *)&channel_data[0]);
	chip->als_readings.als_ch1 =
		le16_to_cpup((const __le16 *)&channel_data[2]);

	dev_dbg(&client->dev, "both: %d\n", chip->als_readings.als_ch0);
	dev_dbg(&client->dev, "ir: %d\n", chip->als_readings.als_ch1);

	/* Visible light is channel 0 - channel 1 i.e. both - IR = visible */
	chip->als_readings.als_visible =
		chip->als_readings.als_ch0 - chip->als_readings.als_ch1;

	/* Calculate counts per lux value */
	counts_per_lux = (als_time_secs_to_ms(settings->als_int_time) *
		tsl2591_gain_to_again(settings->als_gain)) /
		TSL2591_LUX_COEFFICIENT;

	dev_dbg(&client->dev, "Counts Per Lux (CPL): %d\n", counts_per_lux);

	/* Calculate lux value */
	lux = ((chip->als_readings.als_ch0 - chip->als_readings.als_ch1) *
	(1000 - ((chip->als_readings.als_ch1 * 1000) /
	chip->als_readings.als_ch0))) / counts_per_lux;

	/* Divide by 1000 to get real lux value before scaling */
	chip->als_readings.als_lux_int = lux / 1000;

	/* Get the decimal part of lux reading */
	chip->als_readings.als_lux_decimal =
		(lux - (chip->als_readings.als_lux_int * 1000));

	dev_dbg(&client->dev, "Raw lux calculation: %d\n", lux);

	dev_dbg(&client->dev, "Lux Value: %d.%d\n",
		chip->als_readings.als_lux_int,
		chip->als_readings.als_lux_decimal);

	return ret;
}

static int tsl2591_als_calibrate(struct tsl2591_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct tsl2591_settings als_settings = chip->als_settings;
	int ret;

	u8 configuration;

	dev_dbg(&client->dev, "Setting configuration - als_int_time: %#04x\n",
				als_settings.als_int_time);
	dev_dbg(&client->dev, "Setting configuration - als_gain: %#04x\n",
				als_settings.als_gain);

	configuration = als_settings.als_int_time | als_settings.als_gain;

	ret = i2c_smbus_write_byte_data(client,
		TSL2591_CMD_NOP | TSL2591_CONTROL, configuration);
	if (ret < 0)
		dev_err(&client->dev,
			"%s: failed to set configuration to %#04x\n", __func__,
			configuration);

	return ret;
}

static int tsl2591_clear_als_irq(struct tsl2591_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret;

	ret = i2c_smbus_write_byte(client, TSL2591_CMD_SF_CALS_NPI);
	if (ret < 0)
		dev_err(&client->dev, "failed to clear als irq\n");

	return ret;
}

static int tsl2591_als_thresholds(struct tsl2591_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct tsl2591_settings als_settings = chip->als_settings;
	int ret;

	u8 als_lower_l;
	u8 als_lower_h;
	u8 als_upper_l;
	u8 als_upper_h;

	if (als_settings.als_lower_threshold >= als_settings.als_upper_threshold) {
		dev_warn(&client->dev, "lower threshold higher than upper\n");
		return -EINVAL;
	}

	if (als_settings.als_upper_threshold > ALS_MAX_VALUE) {
		dev_warn(&client->dev, "upper threshold higher than max\n");
		return -EINVAL;
	}

	als_lower_l = (als_settings.als_lower_threshold & 0x00FF);
	als_lower_h = ((als_settings.als_lower_threshold >> 8) & 0x00FF);
	als_upper_l = (als_settings.als_upper_threshold & 0x00FF);
	als_upper_h = ((als_settings.als_upper_threshold >> 8) & 0x00FF);

	dev_dbg(&client->dev, "Setting configuration - als lower l: %#04x\n",
				als_lower_l);
	dev_dbg(&client->dev, "Setting configuration - als lower h: %#04x\n",
				als_lower_h);
	dev_dbg(&client->dev, "Setting configuration - als upper l: %#04x\n",
				als_upper_l);
	dev_dbg(&client->dev, "Setting configuration - als upper h: %#04x\n",
				als_upper_h);

	ret = i2c_smbus_write_byte_data(client, TSL2591_CMD_NOP |
		TSL2591_PERSIST, als_settings.als_persist);

	if (ret < 0)
		dev_err(&client->dev,
			"%s: failed to set als persist any\n", __func__);

	ret = i2c_smbus_write_byte_data(client, TSL2591_CMD_NOP |
		TSL2591_AILTL, als_lower_l);

	if (ret < 0)
		dev_err(&client->dev,
			"%s: failed to set als lower threshold\n", __func__);

	ret = i2c_smbus_write_byte_data(client, TSL2591_CMD_NOP |
		TSL2591_AILTH, als_lower_h);

	if (ret < 0)
		dev_err(&client->dev,
			"%s: failed to set als lower threshold\n", __func__);

	ret = i2c_smbus_write_byte_data(client, TSL2591_CMD_NOP |
		TSL2591_AIHTL, als_upper_l);

	if (ret < 0)
		dev_err(&client->dev,
			"%s: failed to set als upper threshold\n", __func__);

	ret = i2c_smbus_write_byte_data(client, TSL2591_CMD_NOP |
		TSL2591_AIHTH, als_upper_h);

	if (ret < 0)
		dev_err(&client->dev,
			"%s: failed to set als upper threshold\n", __func__);

	return ret;
}

static int tsl2591_set_power_state(struct tsl2591_chip *chip, u8 state)
{
	struct i2c_client *client = chip->client;
	int ret;

	ret = i2c_smbus_write_byte_data(client, TSL2591_CMD_NOP |
					TSL2591_ENABLE, state);
	if (ret < 0)
		dev_err(&client->dev,
			"%s: failed to set the power state to %#04x\n", __func__,
			state);

	return ret;
}

static int tsl2591_set_pm_runtime_busy(struct tsl2591_chip *chip, bool busy)
{
	struct i2c_client *client = chip->client;
	int ret;

	if (!client->irq) {
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

	dev_dbg(&client->dev, "irq enabled, skipping pm runtime\n");

	return 0;
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

	u32 int_time;
	int value;

	if (kstrtoint(buf, 0, &value) || !value)
		return -EINVAL;

	mutex_lock(&chip->als_mutex);

	int_time = als_time_ms_to_secs(value);

	if (tsl2591_compatible_int_time(chip, int_time))
		goto calibrate_error;

	if (tsl2591_als_calibrate(chip))
		goto calibrate_error;

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

static ssize_t in_illuminance_lower_threshold_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	int ret;

	mutex_lock(&chip->als_mutex);
	ret = sprintf(buf, "%d\n", chip->als_settings.als_lower_threshold);
	mutex_unlock(&chip->als_mutex);

	return ret;
}

static ssize_t in_illuminance_lower_threshold_store(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	struct i2c_client *client = chip->client;

	int value;

	if (kstrtoint(buf, 0, &value) || !value)
		return -EINVAL;

	mutex_lock(&chip->als_mutex);

	if (value >= ALS_MAX_VALUE ||
		value >= chip->als_settings.als_upper_threshold)
		goto calibrate_error;
	else
		chip->als_settings.als_lower_threshold = value;

	if (tsl2591_als_thresholds(chip))
		goto calibrate_error;

	mutex_unlock(&chip->als_mutex);

	return len;

calibrate_error:
	dev_err(&client->dev, "Failed to calibrate sensor\n");
	mutex_unlock(&chip->als_mutex);
	return -EINVAL;
}

static ssize_t in_illuminance_upper_threshold_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	int ret;

	mutex_lock(&chip->als_mutex);
	ret = sprintf(buf, "%d\n", chip->als_settings.als_upper_threshold);
	mutex_unlock(&chip->als_mutex);

	return ret;
}

static ssize_t in_illuminance_upper_threshold_store(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	struct i2c_client *client = chip->client;

	int value;

	if (kstrtoint(buf, 0, &value) || !value)
		return -EINVAL;

	mutex_lock(&chip->als_mutex);

	if (value > ALS_MAX_VALUE ||
		value <= chip->als_settings.als_lower_threshold)
		goto calibrate_error;
	else
		chip->als_settings.als_upper_threshold = value;

	if (tsl2591_als_thresholds(chip))
		goto calibrate_error;

	mutex_unlock(&chip->als_mutex);

	return len;

calibrate_error:
	dev_err(&client->dev, "Failed to calibrate sensor\n");
	mutex_unlock(&chip->als_mutex);
	return -EINVAL;
}

static ssize_t in_illuminance_persist_filter_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	int ret;

	mutex_lock(&chip->als_mutex);
	ret = sprintf(buf, "%d\n", chip->als_settings.als_persist);
	mutex_unlock(&chip->als_mutex);

	return ret;
}

static ssize_t in_illuminance_persist_filter_store(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	struct i2c_client *client = chip->client;

	int value;

	if (kstrtoint(buf, 0, &value) || !value)
		return -EINVAL;

	mutex_lock(&chip->als_mutex);

	if (tsl2591_compatible_als_persist(chip, value))
		goto calibrate_error;

	if (tsl2591_als_thresholds(chip))
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
static IIO_CONST_ATTR(in_illuminance_persist_filter_available,
				"0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15");
static IIO_DEVICE_ATTR_RW(in_illuminance_integration_time, 0);
static IIO_DEVICE_ATTR_RW(in_illuminance_gain, 0);
static IIO_DEVICE_ATTR_RW(in_illuminance_lower_threshold, 0);
static IIO_DEVICE_ATTR_RW(in_illuminance_upper_threshold, 0);
static IIO_DEVICE_ATTR_RW(in_illuminance_persist_filter, 0);

static struct attribute *sysfs_attrs_ctrl[] = {
	&iio_const_attr_in_illuminance_integration_time_available_ms.dev_attr.attr,
	&iio_const_attr_in_illuminance_gain_available.dev_attr.attr,
	&iio_const_attr_in_illuminance_persist_filter_available.dev_attr.attr,
	&iio_dev_attr_in_illuminance_integration_time.dev_attr.attr,
	&iio_dev_attr_in_illuminance_gain.dev_attr.attr,
	&iio_dev_attr_in_illuminance_lower_threshold.dev_attr.attr,
	&iio_dev_attr_in_illuminance_upper_threshold.dev_attr.attr,
	&iio_dev_attr_in_illuminance_persist_filter.dev_attr.attr,
	NULL
};

static const struct attribute_group tsl2591_attribute_group = {
	.attrs = sysfs_attrs_ctrl,
};

static const struct iio_event_spec tsl2591_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec tsl2591_channels[] = {
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_BOTH,
		.event_spec = tsl2591_events,
		.num_event_specs = ARRAY_SIZE(tsl2591_events),
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_INTENSITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
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
		if (chan->type == IIO_INTENSITY) {
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
	case IIO_CHAN_INFO_PROCESSED:
		if (chan->type == IIO_LIGHT) {
			ret = tsl2591_get_lux_data(indio_dev);
			if (ret < 0)
				break;
			*val = chip->als_readings.als_lux_int;
			*val2 = (chip->als_readings.als_lux_decimal * 1000);
			ret = IIO_VAL_INT_PLUS_MICRO;
		} else if (chan->type == IIO_INTENSITY) {
			ret = tsl2591_get_lux_data(indio_dev);
			if (ret < 0)
				break;

			*val = chip->als_readings.als_visible;
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

static const struct iio_info tsl2591_info = {
	.attrs = &tsl2591_attribute_group,
	.read_raw = tsl2591_read_raw,
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

static irqreturn_t tsl2591_irq_handler(int irq, void *private)
{
	struct iio_dev *dev_info = private;
	struct tsl2591_chip *chip = iio_priv(dev_info);

	s64 timestamp = iio_get_time_ns(dev_info);

	iio_push_event(dev_info,
			IIO_UNMOD_EVENT_CODE(IIO_LIGHT, 0,
			IIO_EV_TYPE_THRESH,
			IIO_EV_DIR_EITHER),
			timestamp);

	tsl2591_clear_als_irq(chip);

	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static int tsl2591_probe_of(struct tsl2591_chip *chip)
{
	struct device *dev = &chip->client->dev;
	struct device_node *np = dev->of_node;
	int ret;

	u32 als_integration_time;
	u32 als_gain;
	u32 als_persist;
	u32 als_lower_threshold;
	u32 als_upper_threshold;

	if (!np)
		return -ENODEV;

	dev_dbg(dev, "Probing device tree\n");

	ret = of_property_read_u32(np, "als-integration-time",
		&als_integration_time);

	if (ret) {
		dev_warn(dev,
			 "Setting default als-integration-time: %d\n",
			 DEFAULT_ALS_INTEGRATION_TIME);
		chip->als_settings.als_int_time = DEFAULT_ALS_INTEGRATION_TIME;
	} else {
		if (tsl2591_compatible_int_time(chip, als_integration_time)) {
			dev_warn(dev, "Setting default als-integration-time\n");
			chip->als_settings.als_int_time =
				DEFAULT_ALS_INTEGRATION_TIME;
		}
		dev_info(dev, "als-integration-time = %#04x\n",
			chip->als_settings.als_int_time);
	}

	ret = of_property_read_u32(np, "als-gain", &als_gain);

	if (ret) {
		dev_warn(dev, "Setting default als-gain: %#04x\n",
			DEFAULT_ALS_GAIN);
		chip->als_settings.als_int_time = DEFAULT_ALS_GAIN;
	} else {
		if (tsl2591_compatible_gain(chip, als_gain)) {
			dev_warn(dev, "Setting default als-gain\n");
			chip->als_settings.als_gain = DEFAULT_ALS_GAIN;
		}
		dev_info(dev, "als-gain = %#04x\n",
			chip->als_settings.als_gain);
	}

	ret = of_property_read_u32(np, "als-persist", &als_persist);

	if (ret) {
		dev_warn(dev,
			"Setting default als-persist: %#04x\n",
			DEFAULT_ALS_PERSIST);
		chip->als_settings.als_persist = DEFAULT_ALS_PERSIST;
	} else {
		if (tsl2591_compatible_als_persist(chip, als_persist)) {
			dev_warn(dev, "Setting default als-persist\n");
			chip->als_settings.als_persist = DEFAULT_ALS_PERSIST;
		}
		dev_info(dev, "als-persist = %#04x\n",
			chip->als_settings.als_persist);
	}

	ret = of_property_read_u32(np, "als-lower-threshold",
		&als_lower_threshold);

	if (ret) {
		dev_warn(dev,
			"Setting default als-lower-threshold: %d\n",
			DEFAULT_ALS_LOWER_THRESHOLD);
		chip->als_settings.als_lower_threshold =
			DEFAULT_ALS_LOWER_THRESHOLD;
	} else {
		if (als_lower_threshold >= ALS_MAX_VALUE) {
			dev_warn(dev, "Setting default als-lower-threshold\n");
			chip->als_settings.als_lower_threshold =
				DEFAULT_ALS_LOWER_THRESHOLD;
		} else {
			chip->als_settings.als_lower_threshold =
				als_lower_threshold;
		}
		dev_info(dev, "als-lower-threshold = %d\n",
			chip->als_settings.als_lower_threshold);
	}

	ret = of_property_read_u32(np, "als-upper-threshold",
		&als_upper_threshold);

	if (ret) {
		dev_warn(dev,
			"Setting default als-upper-threshold: %d\n",
			DEFAULT_ALS_UPPER_THRESHOLD);
		chip->als_settings.als_upper_threshold =
			DEFAULT_ALS_UPPER_THRESHOLD;
	} else {
		if (als_upper_threshold > ALS_MAX_VALUE) {
			dev_warn(dev, "Setting default als-upper-threshold\n");
			chip->als_settings.als_upper_threshold =
				DEFAULT_ALS_UPPER_THRESHOLD;
		} else {
			chip->als_settings.als_upper_threshold =
				als_upper_threshold;
		}
		dev_info(dev, "als-upper-threshold = %d\n",
			chip->als_settings.als_upper_threshold);
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
	chip->als_settings.als_persist = DEFAULT_ALS_PERSIST;
	chip->als_settings.als_lower_threshold = DEFAULT_ALS_LOWER_THRESHOLD;
	chip->als_settings.als_upper_threshold = DEFAULT_ALS_UPPER_THRESHOLD;

	dev_dbg(dev, "als-integration-time = %d\n",
		chip->als_settings.als_int_time);
	dev_dbg(dev, "als-gain = %d\n", chip->als_settings.als_gain);
	dev_dbg(dev, "als-persist = %d\n", chip->als_settings.als_persist);
	dev_dbg(dev, "als-lower-threshold = %d\n",
		chip->als_settings.als_lower_threshold);
	dev_dbg(dev, "als-upper-threshold = %d\n",
		chip->als_settings.als_upper_threshold);

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
		dev_err(&client->dev,
			"%s: i2c smbus byte data functionality is not supported\n", __func__);
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

	if (client->irq) {
		dev_dbg(&client->dev, "Registering interrupt\n");
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, tsl2591_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"tsl2591_irq", indio_dev);
		if (ret) {
			dev_err(&client->dev, "irq request error %d\n", -ret);
			return -EINVAL;
		}
	}

	mutex_init(&chip->als_mutex);

	ret = i2c_smbus_read_byte_data(client,
		TSL2591_CMD_NOP | TSL2591_DEVICE_ID);

	if (ret < 0) {
		dev_err(&client->dev,
			"%s: failed to read the device ID register\n",
			__func__);
		return ret;
	}

	if ((ret & TSL2591_DEVICE_ID_MASK) != TSL2591_DEVICE_ID_VAL) {
		dev_err(&client->dev, "%s: received an unknown device ID %#04x\n",
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

	/*
	 * Power device on from probe to allow als cycles to
	 * generate interrupts when intensity is out of bounds.
	 * When interrupts are not enabled, device only needs to be
	 * powered on when reading
	 */
	if (client->irq) {
		tsl2591_resume(&client->dev);
		pm_runtime_set_active(&client->dev);
	}

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
	if (tsl2591_clear_als_irq(chip)) {
		dev_err(&client->dev, "Failed to clear irq\n");
		return -EINVAL;
	}
	if (tsl2591_als_thresholds(chip)) {
		dev_err(&client->dev, "Failed to set als thresholds\n");
		return -EINVAL;
	}

	dev_info(&client->dev, "Probe complete - tsl2591 found.\n");

	return 0;
}

static int tsl2591_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct tsl2591_chip *chip = iio_priv(indio_dev);

	dev_dbg(&client->dev, "Removing device.\n");

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

MODULE_AUTHOR("Joe Sandom <joe.sandom@outlook.com>");
MODULE_DESCRIPTION("TAOS tsl2591 ambient light sensor driver");
MODULE_LICENSE("GPL");

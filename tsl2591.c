// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * @file    tsl2591.c
 * @author  Joe Sandom
 * @date    21st October 2020
 *
 * @brief Device driver for the TAOS TSL2591. This is a very-high sensitivity
 * light-to-digital converter that transforms light intensity into a digital signal.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

/* TSL2591 Register Set */
#define TSL2591_ENABLE      0x00
#define TSL2591_CONFIG      0x01
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

struct tsl2591_chip {
	struct mutex als_mutex;
	struct i2c_client *client;
};

static int tsl2591_set_power_state(struct tsl2591_chip *chip, u8 state)
{
	int ret;

	ret = i2c_smbus_write_byte_data(chip->client,
					TSL2591_CMD_NOP | TSL2591_ENABLE, state);
	if (ret < 0)
		dev_err(&chip->client->dev,
			"%s: failed to set the power state to %d\n", __func__,
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

static struct attribute *sysfs_attrs_ctrl[] = {
	NULL
};

static const struct attribute_group tsl2591_attribute_group = {
	.attrs = sysfs_attrs_ctrl,
};

/* Multiple channels can be specified - e.g. infrared light vs visible light */
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
	int ret;

	ret = tsl2591_set_pm_runtime_busy(chip, true);
	if (ret < 0)
		return ret;

	printk("Reading value from device.\n");

	ret = tsl2591_set_pm_runtime_busy(chip, false);
	if (ret < 0)
		return ret;

	return 0;
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

	printk("Writing value to device.\n");

	ret = tsl2591_set_pm_runtime_busy(chip, false);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct iio_info tsl2591_info = {
	//.attrs = &tsl2591_attribute_group, /* General purpose device attributes */
	.read_raw = tsl2591_read_raw, /* Requesting a value from the device */
	.write_raw = tsl2591_write_raw, /* Writing a value to the device */
};

static int tsl2591_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret = 0;
	struct tsl2591_chip *chip;
	struct iio_dev *indio_dev;

	dev_info(&client->dev, "Start probing device.\n");

	/* This checks whether the i2c adapter supports smbus byte write and read functionality */
	/* If this functionality is not present, no point continuing because this i2c client driver */
	/* will not work with the i2c adapter it's attached to */
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "%s: i2c smbus byte data functionality is not supported\n",
		__func__);
		return -EOPNOTSUPP;
	}

	/* Allocate an iio device using the tsl2591 structure */
	/* devm means memory is managed and free'd automatically */
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	/* Assign allocated memory iio device to chip */
	/* Assign parsed i2c client to internal i2c client in tsl2591 structure */
	chip = iio_priv(indio_dev);
	chip->client = client;
	i2c_set_clientdata(client, indio_dev);

	mutex_init(&chip->als_mutex);

	/* Read the device ID number */
	ret = i2c_smbus_read_byte_data(client,
				       TSL2591_CMD_NOP | TSL2591_DEVICE_ID);

	if (ret < 0) {
		dev_err(&client->dev,
			"%s: failed to read the device ID register\n", __func__);
		return ret;
	}

	if ((ret & TSL2591_DEVICE_ID_MASK) != TSL2591_DEVICE_ID_VAL) {
		dev_err(&client->dev, "%s: received an unknown device ID %x\n",
			__func__, ret);
		return -EINVAL;
	}

	/* Initialise iio device info */
	indio_dev->info = &tsl2591_info; /* callbacks and constant info from driver */
	indio_dev->channels = tsl2591_channels; /* channel spec table */
	indio_dev->num_channels = ARRAY_SIZE(tsl2591_channels); /* num channels */
	indio_dev->modes = INDIO_DIRECT_MODE; /* operating modes supported by device */
	indio_dev->name = chip->client->name; /* name of device */

	pm_runtime_enable(&client->dev);
	pm_runtime_set_autosuspend_delay(&client->dev,
					 TSL2591_POWER_OFF_DELAY_MS);
	pm_runtime_use_autosuspend(&client->dev);

	/* Register the device on the iio framework */
	ret = devm_iio_device_register(indio_dev->dev.parent, indio_dev);
	if (ret) {
		dev_err(&client->dev, "%s: iio registration failed\n",
			__func__);
		return ret;
	}

	/* Parse dt node - get default chip config */

	dev_info(&client->dev, "Probe complete - Light sensor found.\n");

	return 0;
}

static int tsl2591_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct tsl2591_chip *chip = iio_priv(indio_dev);

	dev_info(&client->dev, "Removing device.\n");

	iio_device_unregister(indio_dev);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_put_noidle(&client->dev);

	return tsl2591_set_power_state(chip, TSL2591_PWR_OFF);
}

static int __maybe_unused tsl2591_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct tsl2591_chip *chip = iio_priv(indio_dev);
	int ret;

	mutex_lock(&chip->als_mutex);

	printk("PM Suspending tsl2591.\n");
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

	printk("PM Resuming tsl2591.\n");
	ret = tsl2591_set_power_state(chip, TSL2591_PWR_ON);

	mutex_unlock(&chip->als_mutex);

	return ret;
}

static const struct dev_pm_ops tsl2591_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(tsl2591_suspend, tsl2591_resume, NULL)
};

static const struct i2c_device_id tsl2591_id[] = {
	{ "tsl2591", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tsl2591_id);

static const struct of_device_id tsl2591_of_match[] = {
	{ .compatible = "amstaos,tsl2591", },
	{},
};
MODULE_DEVICE_TABLE(of, tsl2591_of_match);

/* Driver definition */
static struct i2c_driver tsl2591_driver = {
	.driver = {
		.name = "tsl2591",
		.pm = &tsl2591_pm_ops,
		.of_match_table = tsl2591_of_match,
	},
	.id_table = tsl2591_id,
	.probe = tsl2591_probe,
	.remove = tsl2591_remove,
};
module_i2c_driver(tsl2591_driver);

MODULE_AUTHOR("Joe Sandom <jgsandom@hotmail.co.uk>");
MODULE_DESCRIPTION("TAOS tsl2591 ambient light sensor driver");
MODULE_LICENSE("GPL");

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
#include <linux/pm_runtime.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

struct tsl2591_chip {
	struct i2c_client *client;
};

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
   printk("Reading value from device.\n");

   return 0;
}

static int tsl2591_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
   printk("Writing value to device.\n");

   return 0;
}

static const struct iio_info tsl2591_info = {
	.attrs = &tsl2591_attribute_group, /* General purpose device attributes */
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
	if (!indio_dev) {
		return -ENOMEM;
   }

   /* Assign allocated memory iio device to chip */
   /* Assign parsed i2c client to internal i2c client in tsl2591 structure */
   chip = iio_priv(indio_dev);
	chip->client = client;
	i2c_set_clientdata(client, indio_dev);

   /* All chip handshaking would go here */
   /*---------------------------------------*/

   /* Initialise iio device info */
   indio_dev->info = &tsl2591_info; /* callbacks and constant info from driver */
	indio_dev->channels = tsl2591_channels; /* channel spec table */
	indio_dev->num_channels = ARRAY_SIZE(tsl2591_channels); /* num channels */
	indio_dev->modes = INDIO_DIRECT_MODE; /* operating modes supported by device */
	indio_dev->name = chip->client->name; /* name of device */

   /* Add power management handling here */
   /* --------------------------------------*/

   if (!indio_dev) {
		return -ENOMEM;
   }
   /* Register the device on the iio framework */
   ret = devm_iio_device_register(indio_dev->dev.parent, indio_dev);
	if (ret) {
		dev_err(&client->dev, "%s: iio registration failed\n",
			__func__);
		return ret;
	}

   dev_info(&client->dev, "Probe complete - Light sensor found.\n");

	return 0;
}

static int tsl2591_remove(struct i2c_client *client)
{
   struct iio_dev *indio_dev = i2c_get_clientdata(client);

   dev_info(&client->dev, "Removing device.\n");

	iio_device_unregister(indio_dev);

   return 0;
}

static int __maybe_unused tsl2591_suspend(struct device *dev)
{
	return 0;
}

/* REMOVE: __maybe_unused - gcc does not flag warning if this function is not used */
static int __maybe_unused tsl2591_resume(struct device *dev)
{
	return 0;
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

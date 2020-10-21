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

static int tsl2591_probe(struct i2c_client *clientp,
			 const struct i2c_device_id *idp)
{
   dev_info(&clientp->dev, "Probing device.\n");
	return 0;
}

static int tsl2591_remove(struct i2c_client *client)
{
   dev_info(&client->dev, "Removing device.\n");
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

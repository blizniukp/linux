/*
 *  max17048_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/power/max17048_battery.h>
#include <linux/slab.h>

#define MAX17048_VCELL 0x02
#define MAX17048_VCELL_MSB 0x02
#define MAX17048_VCELL_LSB 0x03
#define MAX17048_SOC 0x04
#define MAX17048_SOC_MSB 0x04
#define MAX17048_SOC_LSB 0x05
#define MAX17048_MODE_MSB 0x06
#define MAX17048_MODE_LSB 0x07
#define MAX17048_VER_MSB 0x08
#define MAX17048_VER_LSB 0x09
#define MAX17048_RCOMP_MSB 0x0C
#define MAX17048_RCOMP_LSB 0x0D
#define MAX17048_CMD_MSB 0xFE
#define MAX17048_CMD_LSB 0xFF

#define MAX17048_TABLE		0x40
#define MAX17048_OCV		0x0E
#define MAX17048_UNLOCK		0x3E
#define MAX17048_UNLOCK_VALUE	0x4a57
#define MAX17048_HIBRT		0x0A
#define MAX17048_CONFIG		0x0C
#define MAX17048_VLRT		0x14
#define MAX17048_VRESET		0x18
#define MAX17048_STATUS		0x1A
#define MAX17048_RCOMPSEG1	0x80
#define MAX17048_RCOMPSEG2	0x90

#define MAX17048_DELAY 1000
#define MAX17048_BATTERY_FULL 100
#define MAX17048_BATTERY_LOW 15

struct max17048_chip {
    struct i2c_client* client;
    struct delayed_work work;
    struct power_supply* battery;
	struct max17048_battery_model *model_data;
	
    /* State Of Connect */
    int online;
    /* battery voltage */
    int vcell;
    /* battery capacity */
    int soc;
    /* State Of Charge */
    int status;
	/* battery health */
	int health;
	/* battery capacity */
	int capacity_level;
};

static int max17048_get_property(struct power_supply* psy,
    enum power_supply_property psp,
    union power_supply_propval* val)
{
    struct max17048_chip* chip = power_supply_get_drvdata(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = chip->status;
        break;
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = chip->online;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = chip->vcell;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = chip->soc;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

static int max17048_write_w_reg(struct i2c_client* client, int reg, u16 value)
{
    int ret;

    ret = i2c_smbus_write_word_data(client, reg, value);

    if (ret < 0)
        dev_err(&client->dev, "%s: err %d\n", __func__, ret);

    return ret;
}

static int max17048_read_word(struct i2c_client *client, int reg)
{
	int ret;
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in reading register"
					"0x%02x err %d\n", __func__, reg, ret);
		return ret;
	} else {
		ret = (int)swab16((uint16_t)(ret & 0x0000ffff));
		return ret;
	}
}

static void max17048_reset(struct i2c_client* client)
{
    max17048_write_w_reg(client, MAX17048_CMD_MSB, 0x5400);
}

static void max17048_get_vcell(struct i2c_client* client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int vcell;
	vcell = max17048_read_word(client, MAX17048_VCELL);
	if (vcell < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, vcell);
	else
		chip->vcell = (uint16_t)vcell;
	}

static void max17048_get_soc(struct i2c_client* client)
{
	struct max17048_chip *chip = i2c_get_clientdata(client);
	int soc;
	soc = max17048_read_word(client, MAX17048_SOC);
	if (soc < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, soc);
	else
		chip->soc = (uint16_t)soc >> 9;
	if (chip->soc > MAX17048_BATTERY_FULL) {
		chip->soc = MAX17048_BATTERY_FULL;
		chip->status = POWER_SUPPLY_STATUS_FULL;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (chip->soc < MAX17048_BATTERY_LOW) {
		chip->health = POWER_SUPPLY_HEALTH_DEAD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else {
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}
}

static void max17048_get_version(struct i2c_client* client)
{
	uint16_t version = swab16(i2c_smbus_read_word_data(client, MAX17048_VER_MSB));
	dev_info(&client->dev, "MAX17048 Fuel-Gauge Ver 0x%x\n", version);
}

static void max17048_get_status(struct i2c_client* client)
{
    struct max17048_chip* chip = i2c_get_clientdata(client);

    chip->status = POWER_SUPPLY_STATUS_CHARGING;

    if (chip->soc > MAX17048_BATTERY_FULL)
        chip->status = POWER_SUPPLY_STATUS_FULL;
}

static void max17048_work(struct work_struct* work)
{
    struct max17048_chip* chip;

    chip = container_of(work, struct max17048_chip, work.work);

    max17048_get_vcell(chip->client);
    max17048_get_soc(chip->client);
    max17048_get_status(chip->client);

    queue_delayed_work(system_power_efficient_wq, &chip->work,
        MAX17048_DELAY);
}

static enum power_supply_property max17048_battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
};

static const struct power_supply_desc max17048_battery_desc = {
    .name = "battery",
    .type = POWER_SUPPLY_TYPE_BATTERY,
    .get_property = max17048_get_property,
    .properties = max17048_battery_props,
    .num_properties = ARRAY_SIZE(max17048_battery_props),
};

static int max17048_probe(struct i2c_client* client,
    const struct i2c_device_id* id)
{
    struct i2c_adapter* adapter = to_i2c_adapter(client->dev.parent);
    struct power_supply_config psy_cfg = {};
    struct max17048_chip* chip;
		
    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
        return -EIO;

    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;

    chip->client = client;

    i2c_set_clientdata(client, chip);
    psy_cfg.drv_data = chip;

    max17048_get_version(client);
	
    chip->battery = power_supply_register(&client->dev,
        &max17048_battery_desc, &psy_cfg);
    if (IS_ERR(chip->battery)) {
        dev_err(&client->dev, "failed: power supply register\n");
        return PTR_ERR(chip->battery);
    }

    max17048_reset(client);

    INIT_DEFERRABLE_WORK(&chip->work, max17048_work);
    queue_delayed_work(system_power_efficient_wq, &chip->work,
        MAX17048_DELAY);

    return 0;
}

static int max17048_remove(struct i2c_client* client)
{
    struct max17048_chip* chip = i2c_get_clientdata(client);

    power_supply_unregister(chip->battery);
    cancel_delayed_work(&chip->work);
    return 0;
}

#ifdef CONFIG_PM_SLEEP

static int max17048_suspend(struct device* dev)
{
    struct i2c_client* client = to_i2c_client(dev);
    struct max17048_chip* chip = i2c_get_clientdata(client);

    cancel_delayed_work(&chip->work);
    return 0;
}

static int max17048_resume(struct device* dev)
{
    struct i2c_client* client = to_i2c_client(dev);
    struct max17048_chip* chip = i2c_get_clientdata(client);

    queue_delayed_work(system_power_efficient_wq, &chip->work,
        MAX17048_DELAY);
    return 0;
}

static SIMPLE_DEV_PM_OPS(max17048_pm_ops, max17048_suspend, max17048_resume);
#define MAX17048_PM_OPS (&max17048_pm_ops)

#else

#define MAX17048_PM_OPS NULL

#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id max17048_id[] = {
    { "max17048" },
    {}
};
MODULE_DEVICE_TABLE(i2c, max17048_id);

static struct i2c_driver max17048_i2c_driver = {
    .driver = {
        .name = "max17048",
        .pm = MAX17048_PM_OPS,
    },
    .probe = max17048_probe,
    .remove = max17048_remove,
    .id_table = max17048_id,
};
module_i2c_driver(max17048_i2c_driver);

MODULE_AUTHOR("Pawel Blizniuk <pawel.blizniuk@keratronik.pl>");
MODULE_DESCRIPTION("MAX17048 Fuel Gauge");
MODULE_LICENSE("GPL");
/*
 * Copyright (c) 2024, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>

#include "bmi260_config_data.h"

struct bmi260_data {
	struct device *dev;
	struct i2c_client *client;
	uint8_t chip_id;
};

#define BMI260_REG_CHIP_ID 	0x00
#define BMI260_REG_INIT_CTRL	0x59
#define BMI260_REG_INIT_DATA	0x5e
#define BMI260_REG_PWR_CONF	0x7c
#define BMI260_REG_CMD 		0x7e

#define BMI260_CHIP_ID 		0x24
#define BMI260_CMD_RESET	0xb6

#define I2C_MAX_RETRIES 10

static int bmi260_i2c_write(struct i2c_client *client,
	uint8_t reg, uint8_t *data, size_t len)
{
	int i = 0;
	uint8_t buf[len + 1];
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = len + 1,
			.buf = buf,
		},
	};

	buf[0] = reg;
	memcpy(buf + 1, data, len);

	while (i < I2C_MAX_RETRIES) {
		if (i2c_transfer(client->adapter, msg, 1) != 1) {
			i++;
			msleep(2);
		} else {
			break;
		}
	}

	if (i == I2C_MAX_RETRIES) {
		pr_err("failed to write reg %02x\n", reg);
		return -EIO;
	}

	return 0;
}

static int bmi260_i2c_read(struct i2c_client *client,
	uint8_t reg, uint8_t *data, size_t len)
{
	int i = 0;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	while (i < I2C_MAX_RETRIES) {
		if (i2c_transfer(client->adapter, msg, 2) != 2) {
			i++;
			msleep(2);
		} else {
			break;
		}
	}

	return 0;
}

static int bmi260_read_id(struct bmi260_data *bd)
{
	int ret = 0;
	uint8_t buf;

	// POR sequence - 4.4, pg25
	// soft reset
	buf = BMI260_CMD_RESET;
	ret = bmi260_i2c_write(bd->client, BMI260_REG_CMD, &buf, 1);
	msleep(20);
	// disable power-save mode
	buf = 0;
	ret |= bmi260_i2c_write(bd->client, BMI260_REG_PWR_CONF, &buf, 1);
	// start init
	buf = 0;
	ret |= bmi260_i2c_write(bd->client, BMI260_REG_INIT_CTRL, &buf, 1);
	// write config data
	// may need to use regmap's bulk write if this doesnt work
	ret |= bmi260_i2c_write(bd->client, BMI260_REG_INIT_DATA,
		bmi270_config_file, ARRAY_SIZE(bmi270_config_file));
	// end init
	buf = 1;
	ret |= bmi260_i2c_write(bd->client, BMI260_REG_INIT_CTRL, &buf, 1);
	// settle
	msleep(20);
	// read chip id
	buf = 0;
	ret |= bmi260_i2c_read(bd->client, BMI260_REG_CHIP_ID, &buf, 1);
	if (buf != BMI260_CHIP_ID) {
		pr_err("invalid device id: %02x\n", buf);
		ret = -EINVAL;
	}
	bd->chip_id = buf;

	return ret;
}

static ssize_t bmi260_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bmi260_data *bd = dev_get_drvdata(dev);

	if (!bd)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%x\n", bd->chip_id);
}

static struct device_attribute attrs[] = {
	__ATTR(chip_id, S_IRUGO, bmi260_id_show, NULL),
};

static int bmi260_probe(struct i2c_client *client)
{
	int ret;
	struct bmi260_data *bd;
	uint8_t attr_count;

	bd = devm_kzalloc(&client->dev, sizeof(struct bmi260_data), GFP_KERNEL);
	if (!bd)
		return -ENOMEM;

	bd->client = client;
	bd->dev = &client->dev;
	i2c_set_clientdata(client, bd);

	ret = bmi260_read_id(bd);
	if (ret) {
		pr_err("failed to read chip id: %d\n", ret);
		return ret;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		ret = sysfs_create_file(&client->dev.kobj,
			&attrs[attr_count].attr);
		if (ret < 0) {
			pr_err("failed to create sysfs entries\n");
			goto err_sysfs;
		}
	}

	return 0;

err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&client->dev.kobj,
			&attrs[attr_count].attr);
	}
	return ret;
}

static void bmi260_remove(struct i2c_client *client)
{
	uint8_t attr_count;

	for (attr_count = ARRAY_SIZE(attrs); attr_count > 0;) {
		sysfs_remove_file(&client->dev.kobj,
			&attrs[--attr_count].attr);
	}
}

static struct i2c_device_id bmi260_id_table[] = {
	{ "bmi260", 0, },
	{ },
};
MODULE_DEVICE_TABLE(i2c, bmi260_id_table);

static const struct of_device_id bmi260_dt_ids[] = {
	{ .compatible = "bosch,bmi260" },
	{ },
};

static struct i2c_driver bmi260_driver = {
	.driver = {
		.name = "bmi260-iio",
		.of_match_table = of_match_ptr(bmi260_dt_ids),
	},
	.probe = bmi260_probe,
	.remove = bmi260_remove,
	.id_table = bmi260_id_table,
};

static int __init bmi260_init(void)
{
	if (i2c_add_driver(&bmi260_driver))
		return -ENODEV;

	return 0;
}

static void __exit bmi260_exit(void)
{
	i2c_del_driver(&bmi260_driver);
}

module_init(bmi260_init);
module_exit(bmi260_exit);

MODULE_AUTHOR("Alex Deddo <adeddo27@gmail.com>");
MODULE_DESCRIPTION("Bosch BMI260 IIO driver");
MODULE_LICENSE("GPL");

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
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/kfifo.h>

#include "bmi260_config_data.h"

struct bmi260_data {
	struct device *dev;
	struct i2c_client *client;
	uint8_t chip_id;
	int irq_gpio;
	int irq;
	int mode;
};

enum {
	LOW_POWER = 0,
	NORMAL,
	PERFORMANCE,
	DISABLED,
	MODE_MAX
};

#define FIFO_SIZE 256
static DECLARE_KFIFO(bmi260_fifo, u8, FIFO_SIZE);

#define BMI260_REG_CHIP_ID 	0x00
#define BMI260_REG_OIS_DATA	0x0c
#define BMI260_REG_ACC_CONF	0x40
#define BMI260_REG_GYRO_CONF	0x42
#define BMI260_REG_INIT_CTRL	0x59
#define BMI260_REG_INIT_DATA	0x5e
#define BMI260_REG_PWR_CONF	0x7c
#define BMI260_REG_PWR_CTRL	0x7d
#define BMI260_REG_CMD 		0x7e

#define BMI260_CHIP_ID 		0x24
#define BMI260_CMD_RESET	0xb6

#define I2C_MAX_RETRIES 10

// hindsight should have used a write_byte method, never writing more than 1 byte so far
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

static ssize_t bmi260_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct bmi260_data *bd = dev_get_drvdata(dev);

	if (!bd)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d\n", bd->mode);
}

static ssize_t bmi260_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int r, mode;
	struct bmi260_data *bd = dev_get_drvdata(dev);
	uint8_t reg;

	if (!bd)
		return -ENODEV;

	r = kstrtoint(buf, 10, &mode);
	if (r || mode < 0 || mode >= MODE_MAX)
		return -EINVAL;

	switch (mode) {
	case DISABLED:
		reg = 0;
		r = bmi260_i2c_write(bd->client, BMI260_REG_PWR_CTRL, &reg, 1);
		break;
	case LOW_POWER:
		// enable accel. disable aux, gyro, temp
		reg = 0x4;
		r = bmi260_i2c_write(bd->client, BMI260_REG_PWR_CTRL, &reg, 1);

		// disable acc_filter_perf, set acc_bwp to 2 repititions, set acc_odr to 50hz
		reg = 0x17;
		r |= bmi260_i2c_write(bd->client, BMI260_REG_ACC_CONF, &reg, 1);

		// enable fifo read in low power, enable advanced power mode
		reg = 0x3;
		r |= bmi260_i2c_write(bd->client, BMI260_REG_PWR_CONF, &reg, 1);
		break;
	case NORMAL:
		// enable accel, gyro, temp. disable aux
		reg = 0x4;
		r = bmi260_i2c_write(bd->client, BMI260_REG_PWR_CTRL, &reg, 1);

		// enable acc_filter_perf bit, set acc_bwp to normal mode, acc_odr to 100hz
		reg = 0xa8;
		r |= bmi260_i2c_write(bd->client, BMI260_REG_ACC_CONF, &reg, 1);

		// enable gyro_filter_perf bit, set gyr_bmp to normal mode, gyr_odr to 200hz
		reg = 0xa9;
		r |= bmi260_i2c_write(bd->client, BMI260_REG_GYRO_CONF, &reg, 1);

		// enable fifo read in low power, enable advanced power mode
		reg = 0x2;
		r |= bmi260_i2c_write(bd->client, BMI260_REG_PWR_CONF, &reg, 1);
	case PERFORMANCE:
		// TODO
	default:
		break;
	}
	if (r)
		return r;

	bd->mode = mode;

	return count;
}

static struct device_attribute attrs[] = {
	__ATTR(chip_id, S_IRUGO, bmi260_id_show, NULL),
	__ATTR(mode, S_IWUSR | S_IRUGO, bmi260_mode_show, bmi260_mode_store),
};

static irqreturn_t bmi260_isr(int irq, void *data)
{
	uint8_t buf[12] = { 0, };
	struct bmi260_data *bd = (struct bmi260_data *)data;

	switch (bd->mode) {
	case LOW_POWER:
		if (!bmi260_i2c_read(bd->client, BMI260_REG_OIS_DATA, buf, 6))
			kfifo_in(&bmi260_fifo, buf, 6);
		break;
	case NORMAL:
	case PERFORMANCE:
		if (bmi260_i2c_read(bd->client, BMI260_REG_OIS_DATA, buf, 12))
			kfifo_in(&bmi260_fifo, buf, 12);
		break;
	default:
		break;
	}

	return IRQ_HANDLED;
}

static int bmi260_probe(struct i2c_client *client)
{
	int ret;
	struct bmi260_data *bd;
	uint8_t attr_count;
	unsigned long irq_flags = 0;
	struct gpio_desc *irq_gpio;

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

	bd->mode = DISABLED;

	irq_gpio = devm_gpiod_get(&client->dev, "irq-gpio", GPIOD_IN);
	if (IS_ERR(irq_gpio)) {
		pr_err("failed to get gpio\n");
		return PTR_ERR(irq_gpio);
	}

	irq_flags = IRQF_TRIGGER_FALLING;
	bd->irq_gpio = desc_to_gpio(irq_gpio);

	ret = devm_gpio_request(&client->dev, bd->irq_gpio, "irq-gpio");
	if (ret) {
		pr_err("failed to request gpio: %d\n", ret);
		return ret;
	}

	bd->irq = gpio_to_irq(bd->irq_gpio);

	kfifo_reset(&bmi260_fifo);

	ret = devm_request_threaded_irq(&client->dev, bd->irq, NULL,
		bmi260_isr, irq_flags, "bm260_irq", bd);
	if (ret) {
		pr_err("failed to request irq: %d\n", ret);
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

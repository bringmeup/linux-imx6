/*
 * vcnl4000.c - Support for Vishay VCNL4000/4010/4020/4200 combined ambient
 * light and proximity sensor
 *
 * Copyright 2012 Peter Meerwald <pmeerw@pmeerw.net>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for:
 *   VCNL4000/10/20 (7-bit I2C slave address 0x13)
 *   VCNL4200 (7-bit I2C slave address 0x51)
 *
 * TODO:
 *   allow to adjust IR current
 *   proximity threshold and event handling
 *   periodic ALS/proximity measurement (VCNL4010/20)
 *   interrupts (VCNL4010/20/200)
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define VCNL4000_DRV_NAME "vcnl4000"
#define VCNL4000_PROD_ID	0x01
#define VCNL4010_PROD_ID	0x02 /* for VCNL4020, VCNL4010 */
#define VCNL4200_PROD_ID	0x58

#define VCNL4000_COMMAND	0x80 /* Command register */
#define VCNL4000_PROD_REV	0x81 /* Product ID and Revision ID */
#define VCNL4000_LED_CURRENT	0x83 /* IR LED current for proximity mode */
#define VCNL4000_AL_PARAM	0x84 /* Ambient light parameter register */
#define VCNL4000_AL_RESULT_HI	0x85 /* Ambient light result register, MSB */
#define VCNL4000_AL_RESULT_LO	0x86 /* Ambient light result register, LSB */
#define VCNL4000_PS_RESULT_HI	0x87 /* Proximity result register, MSB */
#define VCNL4000_PS_RESULT_LO	0x88 /* Proximity result register, LSB */
#define VCNL4000_PS_MEAS_FREQ	0x89 /* Proximity test signal frequency */
#define VCNL4000_PS_MOD_ADJ	0x8a /* Proximity modulator timing adjustment */

#define VCNL4200_AL_CONF	0x00 /* Ambient light configuration */
#define VCNL4200_PS_CONF1	0x03 /* Proximity configuration */
#define VCNL4200_PS_CONF2	0x04 /* Proximity configuration */
#define VCNL4200_PS_DATA	0x08 /* Proximity data */
#define VCNL4200_AL_DATA	0x09 /* Ambient light data */
#define VCNL4200_DEV_ID		0x0e /* Device ID, slave address and version */

/* Bit masks for COMMAND register */
#define VCNL4000_AL_RDY		BIT(6) /* ALS data ready? */
#define VCNL4000_PS_RDY		BIT(5) /* proximity data ready? */
#define VCNL4000_AL_OD		BIT(4) /* start on-demand ALS measurement */
#define VCNL4000_PS_OD		BIT(3) /* start on-demand proximity measurement */

enum vcnl4000_device_ids {
	VCNL4000,
	VCNL4200,
};

struct vcnl4000_data {
	struct i2c_client *client;
	enum vcnl4000_device_ids id;
	const char *prod;
	int rev;
	int al_scale;
	const struct vcnl4000_chip_spec *chip_spec;
	struct mutex lock;
};

struct vcnl4000_chip_spec {
	int (*init)(struct vcnl4000_data *data);
	int (*measure_light)(struct vcnl4000_data *data, int *val);
	int (*measure_proximity)(struct vcnl4000_data *data, int *val);
};

static const struct i2c_device_id vcnl4000_id[] = {
	{ "vcnl4000", VCNL4000 },
	{ "vcnl4200", VCNL4200 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vcnl4000_id);

static int vcnl4000_init(struct vcnl4000_data *data)
{
	int ret, prod_id;

	ret = i2c_smbus_read_byte_data(data->client, VCNL4000_PROD_REV);
	if (ret < 0)
		return ret;

	prod_id = ret >> 4;
	if (prod_id != VCNL4010_PROD_ID && prod_id != VCNL4000_PROD_ID)
		return -ENODEV;

	data->prod = prod_id == VCNL4010_PROD_ID ? "VCNL4010/4020" : "VCNL4000";
	data->rev = ret & 0xf;

	data->al_scale = 250000;

	return 0;
};

static int vcnl4200_init(struct vcnl4000_data *data)
{
	int ret;

	ret = i2c_smbus_read_word_data(data->client, VCNL4200_DEV_ID);
	if (ret < 0)
		return ret;

	if ((ret & 0xff) != VCNL4200_PROD_ID)
		return -ENODEV;

	data->prod = "VCNL4200";
	data->rev = ret >> 8 & 0xf;

	/* Set defaults and enable both sensors */
	ret = i2c_smbus_write_byte_data(data->client, VCNL4200_AL_CONF, 0x00);
	if (ret < 0)
		return -EIO;
	/* PS_ IT=9T, PS_Duty=1/640 */
	ret = i2c_smbus_write_word_data(data->client, VCNL4200_PS_CONF1, 0x008a);
	if (ret < 0)
		return -EIO;
	/* PS_MPS=8 multi-pulses, I_LED=50mA */
	ret = i2c_smbus_write_word_data(data->client, VCNL4200_PS_CONF2, 0x0060);
	if (ret < 0)
		return -EIO;

	data->al_scale = 24000;

	return 0;
};

static int vcnl4000_measure(struct vcnl4000_data *data, u8 req_mask,
				u8 rdy_mask, u8 data_reg, int *val)
{
	int tries = 20;
	__be16 buf;
	int ret;

	mutex_lock(&data->lock);

	ret = i2c_smbus_write_byte_data(data->client, VCNL4000_COMMAND,
					req_mask);
	if (ret < 0)
		goto fail;

	/* wait for data to become ready */
	while (tries--) {
		ret = i2c_smbus_read_byte_data(data->client, VCNL4000_COMMAND);
		if (ret < 0)
			goto fail;
		if (ret & rdy_mask)
			break;
		msleep(20); /* measurement takes up to 100 ms */
	}

	if (tries < 0) {
		dev_err(&data->client->dev,
			"vcnl4000_measure() failed, data not ready\n");
		ret = -EIO;
		goto fail;
	}

	ret = i2c_smbus_read_i2c_block_data(data->client,
		data_reg, sizeof(buf), (u8 *) &buf);
	if (ret < 0)
		goto fail;

	mutex_unlock(&data->lock);
	*val = be16_to_cpu(buf);

	return 0;

fail:
	mutex_unlock(&data->lock);
	return ret;
}

static int vcnl4200_measure(struct vcnl4000_data *data, u8 reg, int *val)
{
	int ret;

	ret = i2c_smbus_read_word_data(data->client, reg);
	if (ret < 0)
		return ret;

	*val = ret;

	return 0;
}

static int vcnl4000_measure_light(struct vcnl4000_data *data, int *val)
{
	return vcnl4000_measure(data,
			VCNL4000_AL_OD, VCNL4000_AL_RDY,
			VCNL4000_AL_RESULT_HI, val);
}

static int vcnl4200_measure_light(struct vcnl4000_data *data, int *val)
{
	return vcnl4200_measure(data, VCNL4200_AL_DATA, val);
}

static int vcnl4000_measure_proximity(struct vcnl4000_data *data, int *val)
{
	return vcnl4000_measure(data,
			VCNL4000_PS_OD, VCNL4000_PS_RDY,
			VCNL4000_PS_RESULT_HI, val);
}

static int vcnl4200_measure_proximity(struct vcnl4000_data *data, int *val)
{
	return vcnl4200_measure(data, VCNL4200_PS_DATA, val);
}

static const struct vcnl4000_chip_spec vcnl4000_chip_spec_cfg[] = {
	[VCNL4000] = {
		.init = vcnl4000_init,
		.measure_light = vcnl4000_measure_light,
		.measure_proximity = vcnl4000_measure_proximity,
	},
	[VCNL4200] = {
		.init = vcnl4200_init,
		.measure_light = vcnl4200_measure_light,
		.measure_proximity = vcnl4200_measure_proximity,
	},
};

static const struct iio_chan_spec vcnl4000_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE),
	}, {
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}
};

static int vcnl4000_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	int ret;
	struct vcnl4000_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = data->chip_spec->measure_light(data, val);
			if (ret < 0)
				return ret;
			return IIO_VAL_INT;
		case IIO_PROXIMITY:
			ret = data->chip_spec->measure_proximity(data, val);
			if (ret < 0)
				return ret;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		if (chan->type != IIO_LIGHT)
			return -EINVAL;

		*val = 0;
		*val2 = data->al_scale;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_info vcnl4000_info = {
	.read_raw = vcnl4000_read_raw,
	.driver_module = THIS_MODULE,
};

static int vcnl4000_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct vcnl4000_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	data->id = id->driver_data;
	data->chip_spec = &vcnl4000_chip_spec_cfg[data->id];
	mutex_init(&data->lock);

	ret = data->chip_spec->init(data);
	if (ret < 0)
		return ret;

	dev_dbg(&client->dev, "%s Ambient light/proximity sensor, Rev: %02x\n",
		data->prod, data->rev);

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &vcnl4000_info;
	indio_dev->channels = vcnl4000_channels;
	indio_dev->num_channels = ARRAY_SIZE(vcnl4000_channels);
	indio_dev->name = VCNL4000_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static struct i2c_driver vcnl4000_driver = {
	.driver = {
		.name   = VCNL4000_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	.probe  = vcnl4000_probe,
	.id_table = vcnl4000_id,
};

module_i2c_driver(vcnl4000_driver);

MODULE_AUTHOR("Peter Meerwald <pmeerw@pmeerw.net>");
MODULE_DESCRIPTION("Vishay VCNL4000 proximity/ambient light sensor driver");
MODULE_LICENSE("GPL");

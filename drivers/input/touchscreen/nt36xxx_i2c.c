// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * NT36XXX I2C Touchscreen Driver
 *
 * Copyright (C) 2020 - 2021 Goodix, Inc.
 * Copyright (C) 2023 Linaro Ltd.
 * Copyright (C) 2023-2024 George Chan <gchan9527@gmail.com>
 *
 * Based on goodix_ts_berlin driver.
 */
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>

#include "nt36xxx.h"

#define I2C_READ_PREFIX_LEN	1
#define I2C_WRITE_PREFIX_LEN	1

#define DEBUG 0

/*
 * there are two kinds of i2c read/write:
 * 	(a)i2c_read()/i2c_write()/i2c_write_then_read(),
 * 	(b)and the i2c_sync itself.
 *
 * we have to choose one and stick together, cross-use otherwise caused problem.
 * the addressing mode is | 0xff 0xXX 0xYY | 0xZ1 ... data1...| 0xZ2 ...data2... | ...
 * 	0xXX is bit[23..16]
 * 	0xYY is bit[15..7]
 * above describe a 'page select' ops
 * 	0xZ1 is bit[7..0], addr for read ops
 *	0xZ2 is bit[7..0] | 0x80, addr for write ops
 * there is no restriction on the read write order.
*/

static int _nt36xxx_i2c_write(void *context, const void *data, size_t count)
{
        struct device *dev = context;
        struct i2c_client *i2c = to_i2c_client(dev);
        int ret;

        ret = i2c_master_send(i2c, data, count);
        if (ret == count)
                return 0;
        else if (ret < 0)
                return ret;
        else
                return -EIO;
}

static int nt36xxx_i2c_write(void *dev, const void *data,
                                   size_t len)
{
	int32_t ret;

	void *data1 = kmemdup(data, len, GFP_KERNEL|GFP_DMA);
	if (!data1)
		return -ENOMEM;

	u8 addr[4] = { 0xff, *(u32 *)data >> 15, *(u32 *)data >> 7,  (*(u32 *)data & 0x7f) | 0x80};
	memcpy(data1, addr, 4);

	dev_dbg(dev, "%s len=0x%lx", __func__, len);

	_nt36xxx_i2c_write(dev, data1, 3);
	ret = _nt36xxx_i2c_write(dev, data1 + 3, len - 3);
	if (ret)
		dev_err(dev, "transfer err %d\n ", ret);
	else if (DEBUG) {

		print_hex_dump(KERN_INFO, __func__, DUMP_PREFIX_OFFSET,
			16, 1, data, 3, true);

		print_hex_dump(KERN_INFO, __func__, DUMP_PREFIX_OFFSET,
			16, 1, data + 3, (len - 3) > 0x20 ? 0x20 : len - 3 , true);
	}

	kfree(data1);
	return ret;
}

static int _nt36xxx_i2c_read(void *context,
                           const void *reg, size_t reg_size,
                           void *val, size_t val_size)
{
        struct device *dev = context;
        struct i2c_client *i2c = to_i2c_client(dev);
        struct i2c_msg xfer[2];
        int ret;

        xfer[0].addr = i2c->addr;
        xfer[0].flags = 0;
        xfer[0].len = reg_size;
        xfer[0].buf = (void *)reg;

        xfer[1].addr = i2c->addr;
        xfer[1].flags = I2C_M_RD;
        xfer[1].len = val_size;
        xfer[1].buf = val;

        ret = i2c_transfer(i2c->adapter, xfer, 2);
        if (ret == 2)
                return 0;
        else if (ret < 0)
                return ret;
        else
                return -EIO;
}

static int nt36xxx_i2c_read(void *dev, const void *reg_buf,
                                  size_t reg_size, void *val_buf,
                                  size_t val_size)
{
	int ret;
	u8 addr[4] = { 0xff, *(u32 *)reg_buf >> 15, *(u32 *)reg_buf >> 7,  *(u32 *)reg_buf & 0x7f };

	ret = _nt36xxx_i2c_write(dev, addr, 3);
	if (ret) {
		dev_err(dev, "transfer0 err %s %d ret=%d", __func__, __LINE__, ret);
		return ret;
	}

	ret = _nt36xxx_i2c_read(dev, &addr[3] , 1, val_buf, val_size);
	if (ret) {
		dev_err(dev, "transfer1 err %s %d ret=%d", __func__, __LINE__, ret);
		return ret;
	}

	if (DEBUG) {
		print_hex_dump(KERN_INFO, __func__, DUMP_PREFIX_OFFSET,
			16, 1, addr, 3, true);

		print_hex_dump(KERN_INFO, __func__, DUMP_PREFIX_OFFSET,
			16, 1, addr, (val_size) > 0x20 ? 0x20 : val_size % 0x20 , true);

		print_hex_dump(KERN_INFO, __func__, DUMP_PREFIX_OFFSET,
			16, 1, val_buf, (val_size > 0x20) ? 0x20 : val_size % 0x20 , true);
	}

	return ret;
}

const struct regmap_config nt36xxx_regmap_config_32bit = {
	.name = "nt36xxx_hw",
	.reg_bits = 32,
	.val_bits = 8,
	.read = nt36xxx_i2c_read,
	.write = nt36xxx_i2c_write,

	.zero_flag_mask = true, /* this is needed to make sure addr is not write_masked */
	.cache_type = REGCACHE_NONE,
};

static const struct input_id nt36xxx_i2c_input_id = {
	.bustype = BUS_I2C,
};

static int nt36xxx_i2c_probe(struct i2c_client *i2c)
{
	struct regmap_config *regmap_config;
	struct regmap *regmap;

	regmap_config = devm_kmemdup(&i2c->dev, &nt36xxx_regmap_config_32bit,
				     sizeof(*regmap_config), GFP_KERNEL);
	if (!regmap_config) {
		dev_err(&i2c->dev, "memdup regmap_config fail\n");
		return -ENOMEM;
	}

	regmap = devm_regmap_init(&i2c->dev, NULL, i2c, regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return nt36xxx_probe(&i2c->dev, i2c->irq,
				   &nt36xxx_i2c_input_id, regmap);
}

static const struct nt36xxx_chip_data nt36xxx_default_config = {
	.config = &nt36xxx_regmap_config_32bit,
	.mmap = nt36676f_memory_maps, /* by luck that magic addr are same */
	.trim_table = nt36xxx_i2c_trim_id_table,
	.max_x = 1080,
	.max_y = 2400,
	.abs_x_max = 1080,
	.abs_y_max = 2400,
	.id = &nt36xxx_i2c_input_id,
	.mapid = 0,
	.fw_name = "novatek_ts_tianma_fw.bin",
	.ic_fw_needed = BIT(NT36675_IC),
};

static const struct nt36xxx_chip_data nt36xxx_probe_default_config = {
	.config = &nt36xxx_regmap_config_32bit,
	.mmap = nt36676f_memory_maps, /* by luck that magic addr are same */
	.trim_table = nt36xxx_i2c_trim_id_table,
	.max_x = 1080,
	.max_y = 2400,
	.abs_x_max = 1080,
	.abs_y_max = 2400,
	.id = &nt36xxx_i2c_input_id,
	.mapid = 100,
};

static const struct i2c_device_id nt36xxx_i2c_ids[] = {
	{ "nt36675-i2c", 0 },
	{ "nt36xxx-i2c", 1 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, nt36xxx_i2c_ids);

static const struct of_device_id nt36xxx_i2c_of_match[] = {
	{
		.compatible = "novatek,nt36xxx-i2c",
		.data = &nt36xxx_default_config,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, nt36xxx_i2c_of_match);

static struct i2c_driver nt36xxx_i2c_driver = {
	.driver = {
		.name = "nt36675-i2c",
		.of_match_table = nt36xxx_i2c_of_match,
		.pm = pm_sleep_ptr(&nt36xxx_pm_ops),
	},
	.probe = nt36xxx_i2c_probe,
	.id_table = nt36xxx_i2c_ids,
};
module_i2c_driver(nt36xxx_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NT36XXX I2C Touchscreen driver");
MODULE_AUTHOR("Neil Armstrong <neil.armstrong@linaro.org>");
MODULE_AUTHOR("George Chan <gchan9527@gmail.com>");

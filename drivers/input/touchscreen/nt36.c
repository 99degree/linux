// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Novatek NT36xxx series touchscreens
 *
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 * Copyright (C) 2020 AngeloGioacchino Del Regno <kholk11@gmail.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/spi/spi.h>
#include <linux/pm_runtime.h>
#include <linux/jiffies.h>
#include <linux/firmware.h>


#define debug 1
#define debug_info pr_info
//#define debug_pr pr_info("%s %d", __func__, __LINE__);
#define debug_pr pr_info("%s %d", __func__, __LINE__);
#define dbg_print_hex_dump print_hex_dump

#define MAX_SPI_FREQ_HZ 5000000

/* FW Param address */
#define NT36XXX_FW_ADDR		0x01

/* since this is 0xffff so is enough to cover address needed except PAGE_CRC */
/* after trial and error 63 is too much, so set as 62 */
#define NT36XXX_TRANSFER_LEN	(32*1024)
/* Number of bytes for chip identification */
#define NT36XXX_ID_LEN_MAX	6

/* Touch info */
#define TOUCH_DEFAULT_MAX_WIDTH  1080
#define TOUCH_DEFAULT_MAX_HEIGHT 2246
#define TOUCH_MAX_FINGER_NUM	 10
#define TOUCH_MAX_PRESSURE	 1000

/* Point data length */
#define POINT_DATA_LEN		65

/* Global pages */
#define NT36XXX_PAGE_CHIP_INFO	0x0001f64e
#define NT36XXX_PAGE_CRC	0x0003f135

/* Misc */
#define NT36XXX_NUM_SUPPLIES	 2
#define NT36XXX_MAX_RETRIES	 5
#define NT36XXX_MAX_FW_RST_RETRY 50

/* define special mapping relationship */

/*
 * the old fw expect spi/i2c compatible so addr bit 8 is reserved.
 * and original bit 8 and upward are shift to bit 9 resp.
 * also the most significant byte is set to 0xff for 32 bit addressing.
 */
//#define MMAP_CUSTOM_0XFF(a) ((((a >> 7) & 0xff) << 8) | (((a >> 15 ) & 0xff) << 16) | (0xff000000) | (a & 0xff))
#define MMAP_CUSTOM_0XFF(a) a

static inline int dbg_regmap_read(struct regmap *map, unsigned int reg, unsigned int *val) {
	debug_info("%s %d, reg 0x%x", __func__, __LINE__, reg);
	return regmap_read(map, reg, val);
}

static inline int dbg_regmap_noinc_read(struct regmap *map, unsigned int reg, unsigned char *val, size_t val_len) {
        debug_info("%s %d, reg 0x%x", __func__, __LINE__, reg);
        return regmap_noinc_read(map, reg, val, val_len);
}

static inline int dbg_regmap_write(struct regmap *map, unsigned int reg, unsigned int val) {
	debug_info("%s %d, reg 0x%x", __func__, __LINE__, reg);
	return regmap_write(map, reg, val);
}

/* Main mmap to spi addr */
enum {
        MMAP_BASELINE_ADDR,
        MMAP_BASELINE_BTN_ADDR,
        MMAP_BLD_CRC_EN_ADDR,
        MMAP_BLD_DES_ADDR,
        MMAP_BLD_ILM_DLM_CRC_ADDR,
        MMAP_BLD_LENGTH_ADDR,
        MMAP_BOOT_RDY_ADDR,
        MMAP_DIFF_BTN_PIPE0_ADDR,
        MMAP_DIFF_BTN_PIPE1_ADDR,
        MMAP_DIFF_PIPE0_ADDR,
        MMAP_DIFF_PIPE1_ADDR,
        MMAP_DLM_DES_ADDR,
        MMAP_DLM_LENGTH_ADDR,
        MMAP_DMA_CRC_EN_ADDR,
        MMAP_DMA_CRC_FLAG_ADDR,
	MMAP_ENG_RST_ADDR,
        MMAP_EVENT_BUF_ADDR,
        MMAP_G_DLM_CHECKSUM_ADDR,
        MMAP_G_ILM_CHECKSUM_ADDR,
        MMAP_ILM_DES_ADDR,
        MMAP_ILM_LENGTH_ADDR,
        MMAP_POR_CD_ADDR,
        MMAP_RAW_BTN_PIPE0_ADDR,
        MMAP_RAW_BTN_PIPE1_ADDR,
        MMAP_RAW_PIPE0_ADDR,
        MMAP_RAW_PIPE1_ADDR,
        MMAP_READ_FLASH_CHECKSUM_ADDR,
        MMAP_RW_FLASH_DATA_ADDR,
        MMAP_R_DLM_CHECKSUM_ADDR,
        MMAP_R_ILM_CHECKSUM_ADDR,
        MMAP_SPI_RD_FAST_ADDR,
        MMAP_SWRST_N8_ADDR,

	/* below are magic numbers in source code */
	MMAP_MAGIC_NUMBER_0X1F64E_ADDR,

	MMAP_TOP_ADDR,
	MMAP_MAX_ADDR = MMAP_TOP_ADDR,
} nvt_ts_mem_map;

struct nvt_fw_parse_data {
	uint8_t partition;
	uint8_t ilm_dlm_num;
};

struct nvt_ts_bin_map {
	char name[12];
	uint32_t bin_addr;
	uint32_t sram_addr;
	uint32_t size;
	uint32_t crc;
};

struct nvt_ts_hw_info {
	uint8_t carrier_system;
	uint8_t hw_crc;
};

struct nt36xxx_abs_object {
	u16 x;
	u16 y;
	u16 z;
	u8 tm;
};

struct nt36xxx_fw_info {
	u8 fw_ver;
	u8 x_num;
	u8 y_num;
	u8 max_buttons;
	u16 abs_x_max;
	u16 abs_y_max;
	u16 nvt_pid;
};

struct nt36xxx_chip_data;

struct nt36xxx_ts {
	struct regmap *regmap;
	//struct regmap *fw_regmap;

	struct input_dev *input;
	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset_gpio;
        struct gpio_desc *irq_gpio;
	int irq;
	struct device *dev;

	struct mutex lock;

	struct touchscreen_properties prop;
	struct nt36xxx_fw_info fw_info;
	struct nt36xxx_abs_object abs_obj;

	struct workqueue_struct *nvt_fw_dl_wq;
	struct delayed_work nvt_fwu_work;

	/* this is a duplicate with nt36xxx_chip_data and since the address might
	 * change in boot/init/download stages so make it a copy of initial map and
	 * update accordingly */
	u32 *mmap;
	u32 mmap_data[MMAP_MAX_ADDR];

	struct nvt_fw_parse_data fw_data;
	struct nvt_ts_bin_map *bin_map;

	uint8_t carrier_system;
	uint8_t hw_crc;

	//uint8_t *fwbuf;
	const struct firmware *fw_entry;
	const struct nt36xxx_chip_data *data;
};

struct nt36xxx_chip_data {
	const u32 *mmap;
	const struct regmap_config *config;

	/* from struct input_id, used to set this struct */
	__u16 bustype;
	const char* fw_name;
};

enum nt36xxx_chips {
	NT36525_IC = 0,
	NT36672A_IC,
	NT36676F_IC,
	NT36772_IC,
	NT36675_IC,
	NT36870_IC,
	NTMAX_IC,
};

struct nt36xxx_trim_table {
	u8 id[NT36XXX_ID_LEN_MAX];
	u8 mask[NT36XXX_ID_LEN_MAX];
	enum nt36xxx_chips mapid;
	uint8_t carrier_system;
	uint8_t hw_crc;
};

enum nt36xxx_swrst_n8_cmds {
	NT36XXX_SW_RESET_SPI = 0x7e,
};

//used under NT36XXX_EVT_HOST_CMD
enum nt36xxx_cmds {
	NT36XXX_CMD_ENTER_SLEEP = 0x11,
	NT36XXX_CMD_ENTER_WKUP_GESTURE = 0x13,
	NT36XXX_CMD_UNLOCK = 0x35,
	NT36XXX_CMD_SW_RESET_SPI = 0x55,
	NT36XXX_CMD_BOOTLOADER_RESET = 0x69,
	NT36XXX_CMD_SW_RESET = 0xa5,
	NT36XXX_CMD_SW_RESET_IDLE_SPI = 0xaa,
	NT36XXX_CMD_SET_PAGE = 0xff,
};

/**
 * enum nt36xxx_fw_state - Firmware state
 * @NT36XXX_STATE_INIT: IC Reset
 * @NT36XXX_STATE_REK: ReK baseline
 * @NT36XXX_STATE_REK_FINISH: Baseline is ready
 * @NT36XXX_STATE_NORMAL_RUN: Firmware is running
 */
enum nt36xxx_fw_state {
	NT36XXX_STATE_INIT = 0xa0,
	NT36XXX_STATE_REK = 0xa1,
	NT36XXX_STATE_REK_FINISH = 0xa2,
	NT36XXX_STATE_NORMAL_RUN = 0xa3,
	NT36XXX_STATE_MAX = 0xaf
};

enum nt36xxx_events {
	NT36XXX_EVT_REPORT = 0x00,
	NT36XXX_EVT_CRC = 0x35,
	NT36XXX_EVT_CHIPID = 0x4e,  //suspect related to address 0x1f64e, ie 0x1f600 + 4e
	NT36XXX_EVT_HOST_CMD = 0x50,
	NT36XXX_EVT_HS_OR_SUBCMD = 0x51,   /* Handshake or subcommand byte */
	NT36XXX_EVT_RESET_COMPLETE = 0x60,
	NT36XXX_EVT_FWINFO = 0x78,
	NT36XXX_EVT_PROJECTID = 0x9a,
};

static const struct nt36xxx_trim_table trim_id_table[] = {
	{
	 .id = { 0x0A, 0xFF, 0xFF, 0x72, 0x66, 0x03 },
	 .mask = { 1, 0, 0, 1, 1, 1 },
	 .mapid = NT36672A_IC,
	},
	{
	 .id = { 0x55, 0x00, 0xFF, 0x00, 0x00, 0x00 },
	 .mask = { 1, 1, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0x55, 0x72, 0xFF, 0x00, 0x00, 0x00 },
	 .mask = { 1, 1, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xAA, 0x00, 0xFF, 0x00, 0x00, 0x00 },
	 .mask = { 1, 1, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xAA, 0x72, 0xFF, 0x00, 0x00, 0x00 },
	 .mask = { 1, 1, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x72, 0x67, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x70, 0x66, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x70, 0x67, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x72, 0x66, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x25, 0x65, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x70, 0x68, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36772_IC,
	},
	{
	 .id = { 0xFF, 0xFF, 0xFF, 0x76, 0x66, 0x03 },
	 .mask = { 0, 0, 0, 1, 1, 1 },
	 .mapid = NT36676F_IC,
	},
        {
	 .id = { 0xFF, 0xFF, 0xFF, 0x75, 0x66, 0x03},
	 .mask = { 0, 0, 0, 1, 1, 1 },
         .mapid = NT36675_IC,
	 .hw_crc = 2,
	 .carrier_system = 0,
        },
	{ },
};
//buf[0]=0x00FF, buf[1]=0x0000, buf[2]=0x0000, buf[3]=0x0001, buf[4]=0x0075, buf[5]=0x0066, buf[6]=0x0000
//[ 4803.844771] 1400 ok13
//struct nt36xxx_mem_map {
//	u32 paging;
//	u32 addr;
//};

static const u32 nt36675_memory_maps[] = {
	[MMAP_EVENT_BUF_ADDR] = MMAP_CUSTOM_0XFF(0x22D00),
	[MMAP_RAW_PIPE0_ADDR] = MMAP_CUSTOM_0XFF(0x24000),
	[MMAP_RAW_PIPE1_ADDR] = MMAP_CUSTOM_0XFF(0x24000),
	[MMAP_BASELINE_ADDR] = MMAP_CUSTOM_0XFF(0x21B90),
	[MMAP_DIFF_PIPE0_ADDR] = MMAP_CUSTOM_0XFF(0x20C60),
	[MMAP_DIFF_PIPE1_ADDR] = MMAP_CUSTOM_0XFF(0x24C60),
	[MMAP_READ_FLASH_CHECKSUM_ADDR] = MMAP_CUSTOM_0XFF(0x24000),
	[MMAP_RW_FLASH_DATA_ADDR] = MMAP_CUSTOM_0XFF(0x24002),
	[MMAP_BOOT_RDY_ADDR] = MMAP_CUSTOM_0XFF(0x3F10D),
	[MMAP_BLD_LENGTH_ADDR] = MMAP_CUSTOM_0XFF(0x3F138),
	[MMAP_ILM_LENGTH_ADDR] = MMAP_CUSTOM_0XFF(0x3F118),
	[MMAP_DLM_LENGTH_ADDR] = MMAP_CUSTOM_0XFF(0x3F130),
	[MMAP_BLD_DES_ADDR] = MMAP_CUSTOM_0XFF(0x3F114),
	[MMAP_ILM_DES_ADDR] = MMAP_CUSTOM_0XFF(0x3F128),
	[MMAP_DLM_DES_ADDR] = MMAP_CUSTOM_0XFF(0x3F12C),
	[MMAP_G_ILM_CHECKSUM_ADDR] = MMAP_CUSTOM_0XFF(0x3F100),
	[MMAP_G_DLM_CHECKSUM_ADDR] = MMAP_CUSTOM_0XFF(0x3F104),
	[MMAP_R_ILM_CHECKSUM_ADDR] = MMAP_CUSTOM_0XFF(0x3F120),
	[MMAP_R_DLM_CHECKSUM_ADDR] = MMAP_CUSTOM_0XFF(0x3F124),
	[MMAP_BLD_CRC_EN_ADDR] = MMAP_CUSTOM_0XFF(0x3F30E),
	[MMAP_DMA_CRC_EN_ADDR] = MMAP_CUSTOM_0XFF(0x3F136),
	[MMAP_BLD_ILM_DLM_CRC_ADDR] = MMAP_CUSTOM_0XFF(0x3F133),
	[MMAP_DMA_CRC_FLAG_ADDR] = MMAP_CUSTOM_0XFF(0x3F134),

	/* below are specified by dts), so it might change by project based */
	[MMAP_SPI_RD_FAST_ADDR] = MMAP_CUSTOM_0XFF(0x03F310),
	[MMAP_SWRST_N8_ADDR] = MMAP_CUSTOM_0XFF(0x03F0FE),

	[MMAP_ENG_RST_ADDR] = MMAP_CUSTOM_0XFF(0x7FFF80),
	[MMAP_MAGIC_NUMBER_0X1F64E_ADDR] = MMAP_CUSTOM_0XFF(0x1F64E),

	[MMAP_TOP_ADDR] = MMAP_CUSTOM_0XFF(0xffffff),
};

/**
 * nt36xxx_set_page - Set page number for read/write
 * @ts: Main driver structure
 *
 * Return: Always zero for success, negative number for error
 */
static int nt36xxx_set_page(struct nt36xxx_ts *ts, u32 pageaddr)
{
	u32 data = cpu_to_be32(pageaddr) >> 7;
	int ret;

	ret = regmap_noinc_write(ts->regmap, NT36XXX_CMD_SET_PAGE,
				 &data, sizeof(data));
	if (ret)
		return ret;

	usleep_range(100, 200);
	return ret;
}

static int nt36xxx_eng_reset_idle(struct nt36xxx_ts *ts)
{
        int ret;
	//u8 buf[1] = { 0x5a };
debug_info("%s %d", __func__, __LINE__);
	if(!ts) {
                pr_err("%s %s empty", __func__, "nt36xxx_ts");
                return -EINVAL;
        }

	if(!ts->mmap) {
		pr_err("%s %s empty", __func__, "ts->mmap");
		return -EINVAL;
	}

        if(ts->mmap[MMAP_ENG_RST_ADDR] == 0) {
                pr_err("%s %s empty", __func__, "MMAP_ENG_RST_ADDR");
                return -EINVAL;
        }

	/* HACK to output somethinf without read */
        ret = dbg_regmap_write(ts->regmap, ts->mmap[MMAP_ENG_RST_ADDR],
                           0x5a);

//        ret = dbg_regmap_write(ts->regmap, 0x7ffafb, //ts->mmap[MMAP_ENG_RST_ADDR],
//                           0x5a);

        if (ret)
                return ret;

        /* Wait until the MCU resets the fw state */
        usleep_range(15000, 16000);
        return ret;
}

/**
 * nt36xxx_sw_reset_idle - Warm restart the firmware
 * @ts: Main driver structure
 *
 * This function restarts the running firmware without rebooting to
 * the bootloader (warm restart)
 *
 * Return: Always zero for success, negative number for error
 */
static int nt36xxx_sw_reset_idle(struct nt36xxx_ts *ts)
{
	int ret;
debug_info("%s %d", __func__, __LINE__);
	if (ts->mmap[MMAP_SWRST_N8_ADDR])
		nt36xxx_set_page(ts, ts->mmap[MMAP_SWRST_N8_ADDR]);

	ret = dbg_regmap_write(ts->regmap, ts->mmap[MMAP_SWRST_N8_ADDR],
			   NT36XXX_CMD_SW_RESET);

	debug_info("%s %d", __func__, __LINE__);

	if (ret)
		return ret;

	/* Wait until the MCU resets the fw state */
	usleep_range(15000, 16000);
	return ret;
}

/**
 * nt36xxx_bootloader_reset - Reset MCU to bootloader
 * @ts: Main driver structure
 *
 * Return: Always zero for success, negative number for error
 */
static int nt36xxx_bootloader_reset(struct nt36xxx_ts *ts)
{
	int ret = 0;
debug_info("%s %d", __func__, __LINE__);
	//in spi version, need to set page to SWRST_N8_ADDR
	if (ts->mmap[MMAP_SWRST_N8_ADDR]) {
		ret = dbg_regmap_write(ts->regmap, ts->mmap[MMAP_SWRST_N8_ADDR],
			   NT36XXX_CMD_BOOTLOADER_RESET);
	        if (ret)
        	        return ret;
	} else {
		pr_info("plz make sure MMAP_SWRST_N8_ADDR is set!");
		return -EINVAL;
	}

	/* MCU has to reboot from bootloader: this is the typical boot time */
	msleep(35);

	if (ts->mmap[MMAP_SPI_RD_FAST_ADDR]) {
                ret = dbg_regmap_write(ts->regmap, ts->mmap[MMAP_SPI_RD_FAST_ADDR], 0);
                if (ret)
                        return ret;
	}

debug_info("%s %d exit", __func__, __LINE__);
	return ret;
}

/**
 * nt36xxx_check_reset_state - Check the boot state during reset
 * @ts: Main driver structure
 * @fw_state: Enumeration containing firmware states
 *
 * Return: Always zero for success, negative number for error
 */
static int nt36xxx_check_reset_state(struct nt36xxx_ts *ts,
				     enum nt36xxx_fw_state fw_state)
{
	u8 buf[3] = { 0 };
	int ret = 0, retry = NT36XXX_MAX_FW_RST_RETRY;
debug_info("%s %d", __func__, __LINE__);
	do {
		ret = dbg_regmap_noinc_read(ts->regmap, ts->mmap[MMAP_EVENT_BUF_ADDR] | NT36XXX_EVT_RESET_COMPLETE,
					buf, sizeof(buf));
		if (likely(ret == 0) &&
		    (buf[0] >= fw_state) &&
		    (buf[0] <= NT36XXX_STATE_MAX)) {
			ret = 0;
			break;
		}
		usleep_range(10000, 11000);
	} while (--retry);

	if (!retry) {
		dev_err(ts->dev, "Firmware reset failed.\n");
		ret = -EBUSY;
	}

debug_info("%s %d", __func__, __LINE__);
	return ret;
}

#if 0
/**
 * nt36xxx_read_pid - Read Novatek Project ID
 * @ts: Main driver structure
 *
 * Return: Always zero for success, negative number for error
 */
static int nt36xxx_read_pid(struct nt36xxx_ts *ts)
{
	__be16 pid;
	int ret;

	ret = nt36xxx_set_page(ts, ts->mmap[evtbuf_addr);
	if (ret)
		return ret;

	ret = regmap_noinc_read(ts->regmap, NT36XXX_EVT_PROJECTID,
				&pid, sizeof(pid));
	if (ret < 0)
		return ret;

	ts->fw_info.nvt_pid = be16_to_cpu(pid);
debug_info("%s %d", __func__, __LINE__);
	return 0;
}

/**
 * __nt36xxx_get_fw_info - Get working params from firmware
 * @ts: Main driver structure
 *
 * Return: Always zero for success, negative number for error
 */
static int __nt36xxx_get_fw_info(struct nt36xxx_ts *ts)
{
	struct nt36xxx_fw_info *fwi = &ts->fw_info;
	u8 buf[11] = { 0 };
	int ret = 0;

	ret = nt36xxx_set_page(ts, ts->mmap[evtbuf_addr);
	if (ret)
		return ret;

	ret = regmap_noinc_read(ts->regmap, NT36XXX_EVT_FWINFO,
				buf, sizeof(buf));
	if (ret)
		return ret;

	fwi->fw_ver = buf[0];
	fwi->x_num = buf[2];
	fwi->y_num = buf[3];
	fwi->abs_x_max = get_unaligned_be16(&buf[4]);
	fwi->abs_y_max = get_unaligned_be16(&buf[6]);
	fwi->max_buttons = buf[10];

	/* Check fw info integrity and clear x_num, y_num if broken */
	if ((buf[0] + buf[1]) != 0xFF) {
		dev_err(ts->dev,
			"FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n",
			buf[0], buf[1]);
		fwi->fw_ver = 0;
		fwi->x_num = 18;
		fwi->y_num = 32;
		fwi->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		fwi->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		fwi->max_buttons = 0;
		return -EINVAL;
	}

	/* Get Novatek ProjectID */
	return nt36xxx_read_pid(ts);
}

static int nt36xxx_get_fw_info(struct nt36xxx_ts *ts)
{
	struct nt36xxx_fw_info *fwi = &ts->fw_info;
	int i, ret = 0;

	for (i = 0; i < NT36XXX_MAX_RETRIES; i++) {
		ret = __nt36xxx_get_fw_info(ts);
		if (ret == 0)
			break;
	}

	dev_dbg(ts->dev,
		"FW Info: PID=0x%x, ver=0x%x res=%ux%u max=%ux%u buttons=%u",
		fwi->nvt_pid, fwi->fw_ver, fwi->x_num, fwi->y_num,
		fwi->abs_x_max, fwi->abs_y_max, fwi->max_buttons);

	return ret;
}
#endif

/**
 * nt36xxx_report - Report touch events
 * @ts: Main driver structure
 *
 * Return: Always zero for success, negative number for error
 */
static void nt36xxx_report(struct nt36xxx_ts *ts)
{
	struct nt36xxx_abs_object *obj = &ts->abs_obj;
	struct input_dev *input = ts->input;
	u8 input_id = 0;
	u8 point[POINT_DATA_LEN + 1] = { 0 };
	unsigned int ppos = 0;
	int i, ret, finger_cnt = 0;

debug_info("%s %d", __func__, __LINE__);
	mutex_lock(&ts->lock);

	ret = regmap_raw_read(ts->regmap, ts->mmap[MMAP_EVENT_BUF_ADDR], // NT36XXX_EVT_REPORT,
				point, sizeof(point));
	if (ret < 0) {
		dev_err(ts->dev,
			"Cannot read touch point data: %d\n", ret);
		goto xfer_error;
	}

debug_info("%s %d", __func__, __LINE__);

	for (i = 0; i < TOUCH_MAX_FINGER_NUM; i++) {
		ppos = 6 * i;
		input_id = point[ppos + 0] >> 3;
		if ((input_id == 0) || (input_id > TOUCH_MAX_FINGER_NUM))
			continue;

		if (((point[ppos] & 0x07) == 0x01) ||
		    ((point[ppos] & 0x07) == 0x02)) {
			obj->x = (point[ppos + 1] << 4) +
				 (point[ppos + 3] >> 4);
			obj->y = (point[ppos + 2] << 4) +
				 (point[ppos + 3] & 0xf);
			if ((obj->x > ts->prop.max_x) ||
			    (obj->y > ts->prop.max_y))
				continue;

			obj->tm = point[ppos + 4];
			if (obj->tm == 0)
				obj->tm = 1;

			obj->z = point[ppos + 5];
			if (i < 2) {
				obj->z += point[i + 63] << 8;
				if (obj->z > TOUCH_MAX_PRESSURE)
					obj->z = TOUCH_MAX_PRESSURE;
			}

			if (obj->z == 0)
				obj->z = 1;

			input_mt_slot(input, input_id - 1);
			input_mt_report_slot_state(input,
						   MT_TOOL_FINGER, true);
			touchscreen_report_pos(input, &ts->prop, obj->x,
					       obj->y, true);

			input_report_abs(input, ABS_MT_TOUCH_MAJOR, obj->tm);
			input_report_abs(input, ABS_MT_PRESSURE, obj->z);

			finger_cnt++;
		}
	}
	input_mt_sync_frame(input);
	input_sync(input);

xfer_error:
	enable_irq(ts->irq);

	mutex_unlock(&ts->lock);
}

static irqreturn_t nt36xxx_irq_handler(int irq, void *dev_id)
{
	struct nt36xxx_ts *ts = dev_id;

        mutex_lock(&ts->lock);

	disable_irq_nosync(ts->irq);
	//nt36xxx_report(ts);

	debug_info("%d", __LINE__);
	enable_irq(ts->irq);

	mutex_unlock(&ts->lock);

	return IRQ_HANDLED;
}


/**
 * nt36xxx_chip_version_init - Detect Novatek NT36xxx family IC
 * @ts: Main driver structure
 *
 * This function reads the ChipID from the IC and sets the right
 * memory map for the detected chip.
 *
 * Return: Always zero for success, negative number for error
 */
//nvt_ts_check_chip_ver_trim
static int nt36xxx_chip_version_init(struct nt36xxx_ts *ts)
{
	u8 buf[7] = { 0 };
	int retry = NT36XXX_MAX_RETRIES;
	int sz = sizeof(trim_id_table) / sizeof(struct nt36xxx_trim_table);
	int i, list, mapid, ret;

	ret = nt36xxx_bootloader_reset(ts);
	if (ret) {
		dev_err(ts->dev, "Can't reset the nvt IC\n");
		return ret;
	}

debug_pr

	do {
		//ret = regmap_write(ts->regmap, ts->mmap[MMAP_MAGIC_NUMBER_0X1F64E_ADDR], 0);
                ret = dbg_regmap_noinc_read(ts->regmap, ts->mmap[MMAP_MAGIC_NUMBER_0X1F64E_ADDR], buf, 7);
                if (ret)
                        continue;

debug_pr
		//ret = dbg_regmap_read(ts->regmap, ts->mmap[MMAP_MAGIC_NUMBER_0X1F64E_ADDR], (unsigned int *)buf);

		dev_info(ts->dev, "%s %d, buf[0]=0x%02X, buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n", 
			__func__, __LINE__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

debug_pr

		/* Compare read chip id with trim list */
		for (list = 0; list < sz; list++) {
debug_pr
			/* Compare each not masked byte */
			for (i = 0; i < NT36XXX_ID_LEN_MAX; i++) {
				if (trim_id_table[list].mask[i] &&
				    buf[i] != trim_id_table[list].id[i])
					break;
			}
debug_pr
			/* found and match with mask */
			if (i == NT36XXX_ID_LEN_MAX) {
debug_pr
				mapid = trim_id_table[list].mapid;
				ret = 0;
				ts->carrier_system = trim_id_table[list].carrier_system;
				ts->hw_crc = trim_id_table[list].hw_crc;

				/* copy the const mmap into drvdata */
				memcpy(ts->mmap_data, ts->mmap, sizeof(ts->mmap_data));
				ts->mmap = ts->mmap_data;

//debug_info("%s %d", __func__, __LINE__, MMAP_MAX_ADDR);

			        dbg_print_hex_dump(KERN_INFO, "nt36xxx mmap addr map: ", DUMP_PREFIX_OFFSET,
		                    16, 4, ts->mmap, sizeof(ts->mmap_data), true);

				dev_info(ts->dev, "This is NVT touch IC, %06x, crc %s, mapid %d loop %d", *(int*)&buf[4], ts->hw_crc > 0?"present":"absent", mapid, list);
				return 0;
			}

			ret = -ENOENT;
		}

		usleep_range(10000, 11000);
	} while (--retry);

	return ret;
}

static int32_t nvt_bin_header_parser(int hw_crc, const u8 *fwdata, size_t fwsize, struct nvt_ts_bin_map **bin_map_ptr, uint8_t *partition_ptr, uint8_t ilm_dlm_num)
{
	uint8_t list = 0;
	uint32_t pos = 0x00;
	uint32_t end = 0x00;
	uint8_t info_sec_num = 0;
	uint8_t ovly_sec_num = 0;
	uint8_t ovly_info = 0;
	uint8_t partition;
	struct nvt_ts_bin_map *bin_map;

	/* Find the header size */
	end = fwdata[0] + (fwdata[1] << 8) + (fwdata[2] << 16) + (fwdata[3] << 24);
	pos = 0x30;	// info section start at 0x30 offset
	while (pos < end) {
		info_sec_num ++;
		pos += 0x10;	/* each header info is 16 bytes */
	}

	/*
	 * Find the DLM OVLY section
	 * [0:3] Overlay Section Number
	 * [4]   Overlay Info
	 */
	ovly_info = (fwdata[0x28] & 0x10) >> 4;
	ovly_sec_num = (ovly_info) ? (fwdata[0x28] & 0x0F) : 0;

	/*
	 * calculate all partition number
	 * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
	 */
	*partition_ptr = partition = ilm_dlm_num + ovly_sec_num + info_sec_num;
	pr_info("ovly_info = %d, ilm_dlm_num = %d, ovly_sec_num = %d, info_sec_num = %d, partition = %d\n",
			ovly_info, ilm_dlm_num, ovly_sec_num, info_sec_num, partition);

	/* allocated memory for header info */
	*bin_map_ptr = bin_map = (struct nvt_ts_bin_map *)kzalloc((partition + 1) * sizeof(struct nvt_ts_bin_map), GFP_KERNEL);
	if(bin_map == NULL) {
		pr_err("kzalloc for bin_map failed!\n");
		return -ENOMEM;
	}


	for (list = 0; list < partition; list++) {
		/*
		 * [1] parsing ILM & DLM header info
		 * bin_addr : sram_addr : size (12-bytes)
		 * crc located at 0x18 & 0x1C
		 */
		if (list < ilm_dlm_num) {
			memcpy(&bin_map[list].bin_addr, &(fwdata[0 + list*12]), 4);
			memcpy(&bin_map[list].sram_addr, &(fwdata[4 + list*12]), 4);
			memcpy(&bin_map[list].size, &(fwdata[8 + list*12]), 4);
			memcpy(&bin_map[list].crc, &(fwdata[0x18 + list*4]), 4);

                        if (hw_crc) {
                                debug_info("%s %d", __func__, __LINE__);
			} else { //ts->hw_crc

				pr_err("%s %d sw-crc not support", __func__, __LINE__);
#if 0
				if ((bin_map[list].bin_addr + bin_map[list].size) < fwsize)
					bin_map[list].crc = CheckSum(&fwdata[bin_map[list].bin_addr], bin_map[list].size);
				else {
					pr_err("access range (0x%08X to 0x%08X) is larger than bin size!\n",
							bin_map[list].bin_addr, bin_map[list].bin_addr + bin_map[list].size);
					return -EINVAL;
				}
#endif
				return -EINVAL;
			} //ts->hw_crc
			if (list == 0)
				sprintf(bin_map[list].name, "ILM");
			else if (list == 1)
				sprintf(bin_map[list].name, "DLM");
		}

		/*
		 * [2] parsing others header info
		 * sram_addr : size : bin_addr : crc (16-bytes)
		 */
		if ((list >= ilm_dlm_num) && (list < (ilm_dlm_num + info_sec_num))) {

			/* others partition located at 0x30 offset */
			pos = 0x30 + (0x10 * (list - ilm_dlm_num));

			memcpy(&bin_map[list].sram_addr, &(fwdata[pos]), 4);
			memcpy(&bin_map[list].size, &(fwdata[pos+4]), 4);
			memcpy(&bin_map[list].bin_addr, &(fwdata[pos+8]), 4);
                        memcpy(&bin_map[list].crc, &(fwdata[pos+12]), 4);

			if (hw_crc)
				pr_info("ok, hw_crc presents!");
			else { //ts->hw_crc
#if 0
				if ((bin_map[list].bin_addr + bin_map[list].size) < fwsize)
					bin_map[list].crc = CheckSum(&fwdata[bin_map[list].bin_addr], bin_map[list].size);
				else {
					pr_err("access range (0x%08X to 0x%08X) is larger than bin size!\n",
							bin_map[list].bin_addr, bin_map[list].bin_addr + bin_map[list].size);
					return -EINVAL;
				}
#endif
				pr_err("%s %d sw_crc not support", __func__, __LINE__);
				return -EINVAL;
			} //ts->hw_crc
			/* detect header end to protect parser function */
			if ((bin_map[list].bin_addr == 0) && (bin_map[list].size != 0)) {
				sprintf(bin_map[list].name, "Header");
			} else {
				sprintf(bin_map[list].name, "Info-%d", (list - ilm_dlm_num));
			}
		}

		/*
		 * [3] parsing overlay section header info
		 * sram_addr : size : bin_addr : crc (16-bytes)
		 */
		if (list >= (ilm_dlm_num + info_sec_num)) {
			/* overlay info located at DLM (list = 1) start addr */
			pos = bin_map[1].bin_addr + (0x10 * (list- ilm_dlm_num - info_sec_num));

			memcpy(&bin_map[list].sram_addr, &(fwdata[pos]), 4);
			memcpy(&bin_map[list].size, &(fwdata[pos+4]), 4);
			memcpy(&bin_map[list].bin_addr, &(fwdata[pos+8]), 4);
			memcpy(&bin_map[list].crc, &(fwdata[pos+12]), 4);

			if (hw_crc)
				debug_info("ok have hw_crc");
			else { //ts->hw_crc
#if 0
				if ((bin_map[list].bin_addr + bin_map[list].size) < fwsize)
					bin_map[list].crc = CheckSum(&fwdata[bin_map[list].bin_addr], bin_map[list].size);
				else {
					pr_err("access range (0x%08X to 0x%08X) is larger than bin size!\n",
							bin_map[list].bin_addr, bin_map[list].bin_addr + bin_map[list].size);
					return -EINVAL;
				}
#endif
                                pr_err("%s %d sw_crc not support", __func__, __LINE__);
                                return -EINVAL;
			} //ts->hw_crc
			sprintf(bin_map[list].name, "Overlay-%d", (list- ilm_dlm_num - info_sec_num));
		}

if (bin_map[list].name[0] > 0) {
//dbg_print_hex_dump(KERN_INFO, "nt36xxx fwdata: ", DUMP_PREFIX_OFFSET,
//            16, 4, fwdata, 0x20, true);

debug_info("%s %d", __func__, __LINE__);
debug_info("%s list=[%d] bin_addr=[0x%x] sram_addr=[0x%x] size=[0x%x] crc=[%x]",
        __func__, list, bin_map[list].bin_addr, bin_map[list].sram_addr, bin_map[list].size, bin_map[list].crc);
}

		/* BIN size error detect */
		if ((bin_map[list].bin_addr + bin_map[list].size) > fwsize) {
			pr_err("access range (0x%08X to 0x%08X) is larger than bin size!\n",
					bin_map[list].bin_addr, bin_map[list].bin_addr + bin_map[list].size);
			return -EINVAL;
		}
	}

	return 0;
}

static int32_t nt36xxx_download_firmware_hw_crc(struct nt36xxx_ts *ts) {
	uint32_t list = 0;
	uint32_t bin_addr, sram_addr, size;
//	int32_t ret = 0;
	struct nvt_ts_bin_map *bin_map = ts->bin_map;

debug_info("%s %d", __func__, __LINE__);

        nt36xxx_bootloader_reset(ts);

debug_info("%s %d", __func__, __LINE__);

	for (list = 0; list < ts->fw_data.partition; list++) {
		int j;

debug_info("%s %d list %d", __func__, __LINE__, list);
		/* initialize variable */
		sram_addr = bin_map[list].sram_addr;
		size = bin_map[list].size;
		bin_addr = bin_map[list].bin_addr;

		/* ignore reserved partition (Reserved Partition size is zero) */
		if (!size)
			continue;
		else
			size = size + 1;

		if (size / NT36XXX_TRANSFER_LEN)
			pr_info("%s %d paged write [%s] 0x%x, window 0x%x", __func__, __LINE__, bin_map[list].name, size, NT36XXX_TRANSFER_LEN);

		for (j = 0; j < size; j += NT36XXX_TRANSFER_LEN) {
			int window_size = (size / NT36XXX_TRANSFER_LEN) ? NT36XXX_TRANSFER_LEN : (size % NT36XXX_TRANSFER_LEN);
			regmap_noinc_write(ts->regmap, sram_addr + j, &ts->fw_entry[bin_addr + j], window_size);
		}

	}

debug_info("%s %d", __func__, __LINE__);

	return 0;
}

static void _nt36xxx_boot_download_firmware(struct nt36xxx_ts *ts);
static void nt36xxx_boot_download_firmware(struct work_struct *work) {
	struct nt36xxx_ts *ts = container_of(work, struct nt36xxx_ts, nvt_fwu_work.work);
	_nt36xxx_boot_download_firmware(ts);
}

static void _nt36xxx_boot_download_firmware(struct nt36xxx_ts *ts) {
	int i, ret;
	size_t fw_need_write_size = 0;
	u8 val[16 * 4] = {0}; /* holding 16 parts crc by 4 bytes */

	pr_info("%s %d", __func__, __LINE__);

//	pm_runtime_get_sync(ts->dev);

//	pm_runtime_set_active(ts->dev);

	 pm_runtime_disable(ts->dev);

	//return;
	//TODO: workaround something.
	ts->hw_crc = 2;

	if (ts->fw_entry)
		goto program_upload;

	ret = request_firmware(&ts->fw_entry, ts->data->fw_name, ts->dev);
	if (ret)
		return;

	for (i = (ts->fw_entry->size / 4096); i > 0; i--) {
		if (strncmp(&ts->fw_entry->data[i * 4096 - 3], "NVT", 3) == 0) {
			fw_need_write_size = i * 4096;
			break;
		}

                if (strncmp(&ts->fw_entry->data[i * 4096 - 3], "MOD", 3) == 0) {
                        fw_need_write_size = i * 4096;
                        break;
                }
	}

	if (fw_need_write_size == 0)
		goto release_fw;

	if (*(ts->fw_entry->data + (fw_need_write_size - 4096)) + *(ts->fw_entry->data + ((fw_need_write_size - 4096) + 1)) != 0xFF) {
                pr_err("bin file FW_VER + FW_VER_BAR should be 0xFF!");
                pr_err("FW_VER=0x%02X, FW_VER_BAR=0x%02X\n", 
					*(ts->fw_entry->data+(fw_need_write_size - 4096)),
					*(ts->fw_entry->data+(fw_need_write_size - 4096 + 1)));
		goto release_fw;
	}

	ts->fw_data.ilm_dlm_num = 2;

	ret = nvt_bin_header_parser(ts->hw_crc, ts->fw_entry->data, ts->fw_entry->size, &ts->bin_map, &ts->fw_data.partition, ts->fw_data.ilm_dlm_num);
	if (ret)
		goto release_fw;

program_upload:
//	ts->fwbuf = (uint8_t *)kzalloc((NT36XXX_TRANSFER_LEN+1), GFP_KERNEL);
//	if (!ts->fwbuf)
//		goto release_bin_map;

	if (ts->hw_crc) {
		ret = nt36xxx_download_firmware_hw_crc(ts);

		if (ret) {
			dev_err(ts->dev, "nt36xxx_download_firmware_hw_crc fail!");
	                goto release_fw_buf;
		}

	} else {
		dev_err(ts->dev, "non-hw_crc model is not support yet!");
		goto release_fw_buf;
	}

	/* set ilm & dlm reg bank */
	//nvt_set_bld_hw_crc();

debug_pr

	for (i = 0; i < ts->fw_data.partition; i++) {
		if (0 == strncmp(ts->bin_map[i].name, "IL", 2) /*["ILM"]*/) {
			regmap_noinc_write(ts->regmap, ts->mmap[MMAP_ILM_DES_ADDR], &ts->bin_map[i].bin_addr, sizeof(ts->bin_map[i].bin_addr));
			regmap_noinc_write(ts->regmap, ts->mmap[MMAP_ILM_LENGTH_ADDR], &ts->bin_map[i].size, sizeof(ts->bin_map[i].size));
			regmap_noinc_write(ts->regmap, ts->mmap[MMAP_G_ILM_CHECKSUM_ADDR], &ts->bin_map[i].crc, sizeof(ts->bin_map[i].crc));
		}
		if (0 == strncmp(ts->bin_map[i].name, "DL", 2) /*["DLM"]*/) {
                        regmap_noinc_write(ts->regmap, ts->mmap[MMAP_DLM_DES_ADDR], &ts->bin_map[i].bin_addr, sizeof(ts->bin_map[i].bin_addr));
                        regmap_noinc_write(ts->regmap, ts->mmap[MMAP_DLM_LENGTH_ADDR], &ts->bin_map[i].size, sizeof(ts->bin_map[i].size));
                        regmap_noinc_write(ts->regmap, ts->mmap[MMAP_G_DLM_CHECKSUM_ADDR], &ts->bin_map[i].crc, sizeof(ts->bin_map[i].crc));
		}
	}

debug_pr
	/* nvt_bld_crc_enable() */
	/* crc enable */

	// val[0] = MMAP_BLD_CRC_EN_ADDR
	// val[1] = 0xff;
	regmap_noinc_read(ts->regmap, ts->mmap[MMAP_BLD_CRC_EN_ADDR], val, 2);
debug_pr
	regmap_noinc_write(ts->regmap, ts->mmap[MMAP_BLD_CRC_EN_ADDR], val, 2);


debug_pr
	/* nvt_fw_crc_enable() */
	val[0] = NT36XXX_EVT_RESET_COMPLETE;
	val[1] = 0;
	/* clear fw reset status & enable fw crc check */
	regmap_noinc_write(ts->regmap, ts->mmap[MMAP_EVENT_BUF_ADDR], val, 2);

debug_pr

	val[0] = NT36XXX_EVT_HOST_CMD;
	val[1] = 0xae;
	/* enable fw crc */
	regmap_noinc_write(ts->regmap, ts->mmap[MMAP_EVENT_BUF_ADDR], val, 2);

	/* nvt_boot_ready() */
debug_pr

	/* Set Boot Ready Bit */
	regmap_write(ts->regmap, ts->mmap[MMAP_BOOT_RDY_ADDR], 1);

	/* old logic 5ms, retention to 10ms */
	usleep_range(10000, 11000);

debug_pr

	/* nvt_check_fw_reset_state() */
	ret = nt36xxx_check_reset_state(ts, NT36XXX_STATE_INIT);
	if (ret)
		goto release_fw_buf;

debug_pr
	/* nvt_get_fw_info() */

	/* -----------------*/
	/* nvt_check_fw_checksum() */

	WARN_ON(sizeof(val) < ts->fw_data.partition * 4);

	ret = regmap_noinc_read(ts->regmap, ts->mmap[MMAP_R_ILM_CHECKSUM_ADDR], val, ts->fw_data.partition * 4);
	if (ret)
		goto release_fw_buf;

	for (i = 0; i < ts->fw_data.partition; i++) {
		/* ignore reserved partition (Reserved Partition size is zero) */
		if(!ts->bin_map[i].size)
			continue;

		if (!memcmp(&ts->bin_map[i].crc, &val[1+i*4], 4)) {
			dev_err(ts->dev, "[%d] BIN_checksum=0x%08X, FW_checksum=0x%08X\n",
					i, ts->bin_map[i].crc, *(uint32_t *)&val[1+i*4]);
			ret = -EIO;
			goto release_fw_buf;
		}
	}

        ret = nt36xxx_check_reset_state(ts, NT36XXX_STATE_REK);
        if (ret < 0)
                goto release_fw_buf;

//	pm_runtime_set_suspended(ts->dev);
//	pm_runtime_put_sync(ts->dev);
	return;

release_fw_buf:
//	kfree(ts->fwbuf);
//	ts->fwbuf = NULL;
	kfree(ts->bin_map);
	ts->bin_map = NULL;
release_fw:
	release_firmware(ts->fw_entry);

//	pm_runtime_set_suspended(ts->dev);
//	pm_runtime_put(ts->dev);
	return;
}

static int nt36xxx_regmap_reg_read(void *context, unsigned int reg, unsigned int *val) {
	WARN(1,"%s", __func__);
	return 0;
}

static int nt36xxx_regmap_reg_write(void *context, unsigned int reg, unsigned int val) {
        WARN(1,"%s", __func__);
        return 0;
}

//bulk
static int nt36xxx_regmap_bulk_read(void *context, const void *reg, size_t reg_size,
                    void *val, size_t val_size) {
        struct device *dev = context;
        struct spi_device *spi = to_spi_device(dev);

//	WARN(1,"%s", __func__);

        print_hex_dump(KERN_INFO, "nt36xxx_regmap_bulk_read reg: ", DUMP_PREFIX_OFFSET,
                    16, 1, reg, reg_size % 0x20, true);

//        print_hex_dump(KERN_INFO, "nt36xxx_regmap_bulk_read val: ", DUMP_PREFIX_OFFSET,
//                    16, 1, val, val_size % 0x20, true);

        return spi_write_then_read(spi, reg, reg_size, val, val_size);
}

static int nt36xxx_regmap_bulk_write(void *context, const void *data, size_t count) {
        struct device *dev = context;
        struct spi_device *spi = to_spi_device(dev);
	int ret;

        print_hex_dump(KERN_INFO, "nt36xxx_regmap_bulk_write: ", DUMP_PREFIX_OFFSET,
                    16, 1, data, count % 0x20, true);

        ret = spi_write(spi, data, count);
	if (ret)
		dev_info(dev, "%s 0x%x", __func__, ret);

	return ret;
}

bool nt36xxx_volatile_reg(struct device *dev, unsigned int reg) {
        dev_info(dev, "%s 0x%x %d", __func__, reg, -1);
	return true;
}

bool nt36xxx_writeable_reg(struct device *dev, unsigned int reg) {
	uint8_t buf[4] = {0};

        dev_info(dev, "%s 0x%x %d", __func__, reg, -1);

	buf[0] = 0xFF;
	buf[1] = (reg >> 15) & 0xFF;
	buf[2] = (reg >> 7) & 0xFF;

	return !nt36xxx_regmap_bulk_write(dev, buf, 3);
}

bool nt36xxx_readable_reg(struct device *dev, unsigned int reg) {
        uint8_t buf[4] = {0};

        dev_info(dev, "%s 0x%x %d", __func__, reg, -1);

        buf[0] = 0xFF;
        buf[1] = (reg >> 15) & 0xFF;
        buf[2] = (reg >> 7) & 0xFF;

	return !nt36xxx_regmap_bulk_write(dev, buf, 3);
};

struct regmap_range_cfg nt36xxx_range_cfg[] = {
};

const struct regmap_config nt36xxx_regmap_hw_config = {
	.name = "nt36xxx_hw",
        .reg_bits = 8,
        .val_bits = 8,
        .max_register = 0xffffffff,
	.cache_type = REGCACHE_NONE,
//	.reg_base = 0xff800000,

	.reg_write = nt36xxx_regmap_reg_write,
	.reg_read = nt36xxx_regmap_reg_read,
	.write = nt36xxx_regmap_bulk_write,
	.read = nt36xxx_regmap_bulk_read,
	.volatile_reg = nt36xxx_volatile_reg,

	.writeable_reg = nt36xxx_writeable_reg,
	.readable_reg = nt36xxx_readable_reg,


	.write_flag_mask = 0x80,
//	.ranges = nt36xxx_range_cfg,
//	.num_ranges = ARRAY_SIZE(nt36xxx_range_cfg),
};

static void nt36xxx_disable_regulators(void *data)
{
	struct nt36xxx_ts *ts = data;

	regulator_bulk_disable(NT36XXX_NUM_SUPPLIES, ts->supplies);
}

static int nt36xxx_sub_probe(struct nt36xxx_ts *ts);
static int nt36xxx_ts_probe(struct platform_device *pdev)
{
	const struct nt36xxx_chip_data *data = of_device_get_match_data(&pdev->dev);
        struct nt36xxx_ts *ts;

debug_info("%d 00 ok", __LINE__);

        ts = devm_kzalloc(&pdev->dev, sizeof(*ts), GFP_KERNEL);
        if (!ts)
                return -ENOMEM;

        ts->regmap = devm_regmap_init(pdev->dev.parent, NULL, ts, data->config);
        if (IS_ERR(ts->regmap)) {
                //ret = PTR_ERR(ts->regmap);
                return -EINVAL;
        }

        platform_set_drvdata(pdev, ts);

        ts->dev = &pdev->dev;
        memcpy(ts->mmap_data, data->mmap, sizeof(ts->mmap_data));
	ts->mmap = ts->mmap_data;
        ts->data = data;

	return nt36xxx_sub_probe(ts);
}

static int nt36xxx_spi_probe(struct spi_device *spi)
{
        struct nt36xxx_ts *ts;
	//struct nt36xxx_chip_data *data = of_device_get_match_data(spi->dev);

debug_info("%d 33 ok", __LINE__);

	ts = devm_kzalloc(&spi->dev, sizeof(*ts), GFP_KERNEL);
        if (!ts)
                return -ENOMEM;

	spi_set_drvdata(spi, ts);

	ts->dev = &spi->dev;
	ts->irq = spi->irq;
	//ts->data = data;
	//ts->mmap = data->mmap;

#if 0
        /* don't exceed max specified SPI CLK frequency */
        if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
                dev_err(&spi->dev, "SPI CLK %d Hz?\n", spi->max_speed_hz);
                return -EINVAL;
        }
#endif
debug_info("%d 44 ok", __LINE__);

        ts->regmap = devm_regmap_init_spi(spi, &nt36xxx_regmap_hw_config);
        if (IS_ERR(ts->regmap))
                return PTR_ERR(ts->regmap);

debug_info("%d 55 ok", __LINE__);

        return nt36xxx_sub_probe(ts);
}

static int nt36xxx_sub_probe(struct nt36xxx_ts *ts) {
	struct device *dev = ts->dev;
	const struct nt36xxx_chip_data *chip_data;
        int ret;

//	dev_err(dev, "%s\n", __func__);
debug_info("%d 66 ok", __LINE__);
#if 0
	/*
		ATM the irq is specified as optional
	*/
	if (irq <= 0) {
		dev_err(dev, "No irq specified\n");
		return -EINVAL;
	}
#endif

	if(!ts)
		return -EINVAL;

debug_info("%d 66a ok", __LINE__);

	if(!dev)
		return -EINVAL;

debug_info("%d 66b ok", __LINE__);

	chip_data = of_device_get_match_data(ts->dev);

debug_info("%d 66c ok", __LINE__);

	if(!chip_data)
		return -EINVAL;


	ts->data = chip_data;
	memcpy(ts->mmap_data, chip_data->mmap, sizeof(ts->mmap_data));
	ts->mmap = ts->mmap_data;

debug_info("%d 66d ok", __LINE__);

	ts->supplies = devm_kcalloc(dev,
				    NT36XXX_NUM_SUPPLIES,
				    sizeof(*ts->supplies),
				    GFP_KERNEL);
	if (!ts->supplies)
		return -ENOMEM;

debug_info("%d 77 ok", __LINE__);

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

debug_info("%d ok", __LINE__);
	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						 GPIOD_OUT_LOW);
debug_info("%d ok1", __LINE__);
	if (IS_ERR(ts->reset_gpio))
		return PTR_ERR(ts->reset_gpio);
debug_info("%d ok2", __LINE__);
	gpiod_set_consumer_name(ts->reset_gpio, "nt36xxx reset");

debug_info("%d ok3", __LINE__);

        ts->irq_gpio = devm_gpiod_get_optional(dev, "irq",
                                                 GPIOD_IN);
debug_info("%d ok4", __LINE__);
        if (IS_ERR(ts->irq_gpio))
                return PTR_ERR(ts->irq_gpio);
debug_info("%d ok5", __LINE__);
        gpiod_set_consumer_name(ts->irq_gpio, "nt36xxx irq");
debug_info("%d ok6", __LINE__);
	/* These supplies are optional */
	ts->supplies[0].supply = "vdd";
	ts->supplies[1].supply = "vio";
	ret = devm_regulator_bulk_get(dev,
				      NT36XXX_NUM_SUPPLIES,
				      ts->supplies);
debug_info("%d ok7", __LINE__);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Cannot get supplies: %d\n", ret);
debug_info("%d ok8", __LINE__);
	ret = regulator_bulk_enable(NT36XXX_NUM_SUPPLIES, ts->supplies);
	if (ret)
		return ret;
debug_info("%d ok9", __LINE__);
	usleep_range(10000, 11000);

	ret = devm_add_action_or_reset(dev,
				       nt36xxx_disable_regulators, ts);

debug_info("%d ok10", __LINE__);
	if (ret)
		return ret;

	mutex_init(&ts->lock);
debug_info("%d ok11", __LINE__);

        //ret = nt36xxx_engineering_reset(ts);
	ret = nt36xxx_eng_reset_idle(ts);
        if (ret) {
                dev_err(dev, "Failed to check chip version\n");
                return ret;
        }

debug_info("%d ok12", __LINE__);
	/* Set memory maps for the specific chip version */
	ret = nt36xxx_chip_version_init(ts);
	if (ret) {
		dev_err(dev, "Failed to check chip version\n");
		return ret;
	}


#ifdef debug
        _nt36xxx_boot_download_firmware(ts);
        return -EINVAL;
#endif


debug_info("%d ok13", __LINE__);
	ts->input->phys = devm_kasprintf(dev, GFP_KERNEL,
				     "%s/input0", dev_name(dev));
	if (!ts->input->phys)
		return -ENOMEM;

	ts->input->name = "Novatek NT36XXX Touchscreen";
	ts->input->id.bustype = ts->data->bustype;
	ts->input->dev.parent = dev;

	__set_bit(EV_KEY, ts->input->evbit);
	__set_bit(EV_ABS, ts->input->evbit);
	input_set_capability(ts->input, EV_KEY, BTN_TOUCH);

	ret = input_mt_init_slots(ts->input, TOUCH_MAX_FINGER_NUM,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret) {
		dev_err(dev, "Cannot init MT slots (%d)\n", ret);
		return ret;
	}

	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0,
			     TOUCH_MAX_PRESSURE, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0,
			     ts->fw_info.abs_x_max - 1, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0,
			     ts->fw_info.abs_y_max - 1, 0, 0);

	/* Override the firmware defaults, if needed */
	touchscreen_parse_properties(ts->input, true, &ts->prop);

	input_set_drvdata(ts->input, ts);

	ret = input_register_device(ts->input);
	if (ret) {
		dev_err(dev, "Failed to register input device: %d\n",
			ret);
		return ret;
	}

	ts->irq = gpiod_to_irq(ts->irq_gpio);

	ret = devm_request_threaded_irq(dev, ts->irq, NULL,
					nt36xxx_irq_handler, IRQF_ONESHOT,
					dev_name(dev), ts);
	if (ret) {
		dev_err(dev, "request irq failed: %d\n", ret);
		return ret;
	}

	ts->nvt_fw_dl_wq = alloc_ordered_workqueue("nt36xxx_fw_dl_wq", WQ_MEM_RECLAIM);

	INIT_DELAYED_WORK(&ts->nvt_fwu_work, nt36xxx_boot_download_firmware);

	queue_delayed_work(ts->nvt_fw_dl_wq, &ts->nvt_fwu_work, msecs_to_jiffies(50));

	dev_info(dev, "ok probe done!");
	return 0;
}

static int __maybe_unused nt36xxx_suspend(struct device *dev)
{
	struct nt36xxx_ts *ts = dev_get_drvdata(dev);
	int ret;
pr_info_ratelimited("%s %d", __func__, __LINE__);
	disable_irq(ts->irq);

	cancel_delayed_work(&ts->nvt_fwu_work);
	flush_workqueue(ts->nvt_fw_dl_wq);

	//destroy_workqueue();

        if (ts->mmap[MMAP_EVENT_BUF_ADDR]) {
                ret = dbg_regmap_write(ts->regmap, ts->mmap[MMAP_EVENT_BUF_ADDR], NT36XXX_CMD_ENTER_SLEEP);
                if (ret)
                        return ret;
        }

	if (ret) {
		dev_err(ts->dev, "Cannot enter suspend!!\n");
		return ret;
	}

	gpiod_set_value(ts->reset_gpio, 1);

	return 0;
}

static int __maybe_unused nt36xxx_resume(struct device *dev)
{
	struct nt36xxx_ts *ts = dev_get_drvdata(dev);
	int ret;
pr_info_ratelimited("%s %d", __func__, __LINE__);
	mutex_lock(&ts->lock);

	gpiod_set_value(ts->reset_gpio, 0);

	/* Reboot the MCU (also recalibrates the TS) */
	ret = nt36xxx_bootloader_reset(ts);
	if (ret < 0)
		goto end;

	queue_delayed_work(ts->nvt_fw_dl_wq, &ts->nvt_fwu_work, msecs_to_jiffies(500));

	//should do at fw_download()
	//ret = nt36xxx_check_reset_state(ts, NT36XXX_STATE_REK);
	//if (ret < 0)
	//	goto end;


	// TODO: need to wake the orkqueue to dl fw
	enable_irq(ts->irq);
end:
	mutex_unlock(&ts->lock);
	return ret;
}

static SIMPLE_DEV_PM_OPS(nt36xxx_ts_pm,
                         nt36xxx_suspend, nt36xxx_resume);

//static const struct nt36xxx_ts_data nvt_ts_spi = {
//	.bus_type = BUS_SPI,
//};

const struct nt36xxx_chip_data nt36675 = {
        .config = &nt36xxx_regmap_hw_config,
	.mmap = nt36675_memory_maps,
	.bustype = BUS_SPI,
	.fw_name = "novatek_ts_tianma_fw.bin",
};

static const struct of_device_id nt36xxx_ts_of_match[] = {
        { .compatible = "novatek,nt36525-spi", .data = &nt36675, },
	{ .compatible = "novatek,nt36675-spi", .data = &nt36675, },
	{ .compatible = "novatek,NVT-ts-spi", .data = &nt36675, },
        { .compatible = "novatek,nt36525", .data = &nt36675, },
        { .compatible = "novatek,nt36675", .data = &nt36675, },
        { }
};

#if 0
MODULE_DEVICE_TABLE(of, nt36xxx_spi_of_match);

static struct platform_driver nt36xxx_ts_driver = {
        .driver = {
                .name	= "nvt-ts-regmap-unified",
		.pm     = pm_sleep_ptr(&nt36xxx_ts_pm),
		.of_match_table = nt36xxx_ts_of_match,
        },
        .probe	= nt36xxx_ts_probe,
        //.remove	= nt36xxx_ts_remove,
};

module_platform_driver(nt36xxx_ts_driver);
#endif


static const struct spi_device_id nt36xxx_spi_ts_id[] = {
        { "nt36675-spi", 0 },
        { }
};
MODULE_DEVICE_TABLE(spi, nt36xxx_spi_ts_id);
MODULE_DEVICE_TABLE(of, nt36xxx_ts_of_match);

static struct spi_driver nt36xxx_spi_ts_driver = {
        .driver = {
                .name   = "nt36675-spi",
                .pm     = &nt36xxx_ts_pm,
                .of_match_table = nt36xxx_ts_of_match,
        },
        .id_table       = nt36xxx_spi_ts_id,
        .probe          = nt36xxx_spi_probe,
};
module_spi_driver(nt36xxx_spi_ts_driver);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Novatek NT36XXX Touchscreen Driver");
MODULE_AUTHOR("AngeloGioacchino Del Regno <kholk11@gmail.com>");

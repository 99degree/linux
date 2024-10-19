/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 * Copyright (C) 2020 AngeloGioacchino Del Regno <kholk11@gmail.com>
 * Copyright (C) 2023-2024 George Chan <gchan9527@gmail.com>
 */

#ifndef NT36XXX_H
#define NT36XXX_H

#define NT36XXX_INPUT_DEVICE_NAME	"Novatek NT36XXX Touch Sensor"
#define MAX_SPI_FREQ_HZ 5000000

/* FW Param address */
#define NT36XXX_FW_ADDR		0x01

#define NT36XXX_TRANSFER_LEN	(63*1024)

#define NT36XXX_READ_PREFIX_LEN     1
#define NT36XXX_WRITE_PREFIX_LEN    1

/* due to extra framework layer, the transfer trunk is as small as
 * 128 otherwize dma error happened, all routed to spi_sync()
 */

/* Number of bytes for chip identification */
#define NT36XXX_ID_LEN_MAX	6
#define NT36XXX_ID_LIST_MAX	32

/* Touch info */
#define NT36XXX_TOUCH_DEFAULT_MAX_WIDTH  1080
#define NT36XXX_TOUCH_DEFAULT_MAX_HEIGHT 2246
#define NT36XXX_TOUCH_MAX_FINGER_NUM	 10
#define NT36XXX_TOUCH_MAX_PRESSURE	 1000

/* Point data length */
#define NT36XXX_POINT_DATA_LEN		65

/* Misc */
#define NT36XXX_NUM_SUPPLIES	 2
#define NT36XXX_MAX_RETRIES	 5
#define NT36XXX_MAX_FW_RST_RETRY 50

/* mapid */
enum nt36xxx_chips {
	NT_AUTO_DET_IC,
	NT36525_IC = 0x1,
	NT36672A_IC,
	NT36676F_IC,
	NT36772_IC,
	NT36675_IC,

	NT51900_IC,
	NT51920_IC,
	NT51923_IC,
	NT51926_IC,

	NT_NIL_IC,
	/* below are not supported atm */
	NT36526_IC,
	NT36870_IC,

	NTMAX_IC,
};

enum nt36xxx_cmds {
	NT36XXX_CMD_ENTER_SLEEP = 0x11,
	NT36XXX_CMD_BOOTLOADER_RESET = 0x69,
	NT36XXX_CMD_SOFTWARE_RESET = 0xaa,
};

enum nt36xxx_events {
	NT36XXX_EVT_REPORT = 0x00,
	NT36XXX_EVT_CRC	= 0x35,
	NT36XXX_EVT_HOST_CMD = 0x50,
	NT36XXX_EVT_HS_OR_SUBCMD = 0x51, /* Handshake or subcommand byte */
	NT36XXX_EVT_RESET_COMPLETE = 0x60,
	NT36XXX_EVT_FWINFO = 0x78,
	NT36XXX_EVT_READ_PID = 0x80,
	NT36XXX_EVT_PROJECTID = 0x9a, /* Excess 0x80 write bit, messed trouble, ignored */
};

enum nt36xxx_fw_state {
	NT36XXX_STATE_INIT = 0xa0,              /* IC Reset */
	NT36XXX_STATE_REK = 0xa1,               /* ReK baseline */
	NT36XXX_STATE_REK_FINISH = 0xa2,        /* Baseline is ready */
	NT36XXX_STATE_NORMAL_RUN = 0xa3,        /* Firmware is running */
	NT36XXX_STATE_MAX = 0xaf
};

struct nt36xxx_ts;
struct nt36xxx_trim_data;

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
	uint32_t loaded;
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

struct nt36xxx_chip_data {
	const u32 *mmap;
	const struct regmap_config *config;
	const struct nt36xxx_trim_data *trim_data;

	const char *fw_name;
	unsigned int max_x;
	unsigned int max_y;
	unsigned int abs_x_max;
	unsigned int abs_y_max;
	unsigned int max_button;
	const struct input_id *id;

	unsigned int mapid;

	/* defined as BIT(NT36675_IC) | BIT(NT36672C_IC)... */
	unsigned int ic_fw_needed;
};

struct nt36xxx_trim_table {
	u8 id[NT36XXX_ID_LEN_MAX];
	u8 mask[NT36XXX_ID_LEN_MAX];
	enum nt36xxx_chips mapid;
	uint8_t carrier_system;
	uint8_t hw_crc;
};

int nt36xxx_probe(struct device *dev, int irq, const struct input_id *id,
			struct regmap *regmap);

int nt36xxx_of_compatible(struct device *dev);

extern const struct dev_pm_ops nt36xxx_pm_ops;

extern const u32 nt36675_memory_maps[];
extern const u32 nt36672a_memory_maps[];
extern const u32 nt36772_memory_maps[];
extern const u32 nt36676f_memory_maps[];
extern const u32 nt36525_memory_maps[];
extern const u32 nt51900_memory_maps[];

extern const struct nt36xxx_trim_table nt36xxx_spi_trim_id_table[];
extern const struct nt36xxx_trim_table nt36xxx_i2c_trim_id_table[];
extern const struct nt36xxx_trim_table nt51xxx_trim_id_table[];

extern const struct nt36xxx_trim_data nt36xxx_spi_trim_data;
extern const struct nt36xxx_trim_data nt36xxx_i2c_trim_data;
extern const struct nt36xxx_trim_data nt51xxx_i2c_trim_data;
#endif

// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 */

#ifndef NT36XXX_H
#define NT36XXX_H

#include <linux/i2c.h>

#include "nt36xxx_mem_map.h"

#define NT36XXX_INPUT_DEVICE_NAME	"Novatek NT36XXX Touch Sensor"
/* This chip has different address when in bootloader :( */
#define NT36XXX_BLDR_ADDR 0x01

/* Input device info */
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"

/* Touch info */
#define TOUCH_DEFAULT_MAX_WIDTH 1080
#define TOUCH_DEFAULT_MAX_HEIGHT 2246
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_FORCE_NUM 1000

/* Point data length */
#define POINT_DATA_LEN 65

struct nvt_i2c {
	struct i2c_client *client;
	struct input_dev *input;

	struct work_struct ts_work;
	struct workqueue_struct *ts_workq;

	bool dev_pm_suspend;
	struct completion dev_pm_suspend_completion;

	struct regulator_bulk_data *supplies;

	int8_t phys[32];
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	int reset_gpio;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	uint8_t carrier_system;
	uint16_t nvt_pid;
};

enum nt36xxx_cmds {
	NT36XXX_CMD_ENTER_SLEEP = 0x11,
	NT36XXX_CMD_UNLOCK = 0x35,
	NT36XXX_CMD_BOOTLOADER_RESET = 0x69,
	NT36XXX_CMD_SW_RESET = 0xA5,
};

enum nt36xxx_fw_state {
	NT36XXX_STATE_INIT = 0xA0,	/* IC reset */
	NT36XXX_STATE_REK,		/* ReK baseline */
	NT36XXX_STATE_REK_FINISH,	/* Baseline is ready */
	NT36XXX_STATE_NORMAL_RUN,	/* Normal run */
	NT36XXX_STATE_MAX = 0xAF
};

enum nt36xxx_i2c_events {
	NT36XXX_EVT_HOST_CMD = 0x50,
	NT36XXX_EVT_HS_OR_SUBCMD = 0x51,   /* Handshake or subcommand byte */
	NT36XXX_EVT_RESET_COMPLETE = 0x60,
	NT36XXX_EVT_FWINFO = 0x78,
	NT36XXX_EVT_PROJECTID = 0x9A,
};

#endif

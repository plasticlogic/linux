/*
 * linux/drivers/video/modelffb.h
 *
 * FB driver for EPSON modelF EPD controller
 *
 * Copyright (C) 2012 Seiko Epson Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */

#if !defined (_MODELFFB_H)
#define _MODELFFB_H

#include <linux/fb.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/spi/spi.h>

#define MODELF_IO_SIZE SZ_16M
#define MODELF_DATA_ADDR_OFFSET 0x400 /* a_11 => gpmc_a10 */
#define MODELF_DATA_SIZE 0x200
#define MODELF_MAX_INITCODE_SIZE 4096
#define MODELF_MAX_WAVEFORM_SIZE (160 * 1024)

#define MODELF_DEFERRED_IO_DELAY_DENOMINATOR 30
#define MODELF_TIMEOUT_MS 5000
#define MODELF_MTP_TIMEOUT_MS 500
#define MODELF_MAX_LUT_NUM 4
#define MODELF_MAX_LUT_QUEUE_NUM (MODELF_MAX_LUT_NUM * 2)

struct modelffb_oneshot_info {
	int waveform_mode;
	int oneshot_type;
	int busy;
	int x;
	int y;
	int width;
	int height;
};

enum modelffb_sync_status {
	MODELFFB_SYNC_IDLE = 0,
	MODELFFB_SYNC_PENDING,
	MODELFFB_SYNC_BUSY,
	MODELFFB_SYNC_N,
};

struct modelffb_par {
	struct fb_info *fbinfo;
	unsigned left_border;
	unsigned right_border;
	uint32_t command_addr;
	uint32_t data_addr;
	uint32_t init_code;
	int init_code_size;
	uint32_t waveform;
	uint32_t waveform_index;
	int waveform_size;
	int waveform_mode;
	uint32_t status;
	uint16_t keycode1;
	uint16_t keycode2;
#ifdef CONFIG_MODELF_DEFERRED_IO
	int vram_updated;
#endif
	int suspend_update;
	struct workqueue_struct *workqueue;
	struct semaphore access_sem;
	struct modelffb_oneshot_info lut_in_use[MODELF_MAX_LUT_NUM];
	struct modelffb_oneshot_info lut_queue[MODELF_MAX_LUT_QUEUE_NUM];
	int lut_queue_head;
	int lut_queue_count;
	struct semaphore lut_in_use_sem;
	struct semaphore lut_queue_sem;
	int need_flush_image;
	int need_cleanup;
	int power_mode;
	void *image_pool;
	int image_pool_lines;
	wait_queue_head_t sync_update_wait;
	enum modelffb_sync_status sync_status;
	struct platform_device *pdev;
	struct spi_device *spi;
	struct device *dev;
	const struct modelffb_platform_data *pdata;
	int seconds_to_sleep;
	int seconds_to_measure_temperature;
	struct timer_list sleep_timer;
	struct timer_list temperature_timer;
	uint8_t bit_swap_table[256];
	int send_waveform_wait;
#if (defined(CONFIG_MODELF_PL_HARDWARE) \
     || defined(CONFIG_MODELF_PL_HARDWARE_MODULE))
        struct pl_hardware *pl_hardware;
#endif
	struct {
		u32 clear_on_init:1;
		u32 clear_on_exit:1;
		u32 interleaved_sources:1;
		unsigned spi_freq_hz;
	} opt;
};

struct modelffb_cleanup_work {
	struct work_struct work;
	int waveform_mode;
	int x;
	int y;
	int width;
	int height;
};

struct modelffb_suspend_update_work {
	struct work_struct work;
	int update;
};

struct modelffb_oneshot_work {
	struct work_struct work;
	int oneshot_type;
	int waveform_mode;
	int x;
	int y;
	int width;
	int height;
};

/* model F command list */
#define MODELF_COM_INIT			0x00
#define MODELF_COM_RUN			0x02
#define MODELF_COM_STANDBY		0x04
#define MODELF_COM_SLEEP		0x05
#define MODELF_COM_INIT_THEN_STANDBY	0x06
#define MODELF_COM_ROTATE		0x0B
#define MODELF_COM_READ_REG		0x10
#define MODELF_COM_WRITE_REG		0x11
#define MODELF_COM_TEMPERATURE		0x12
#define MODELF_COM_VOLTAGE		0x13
#define MODELF_COM_RAW_READ		0x1C
#define MODELF_COM_RAW_WRITE		0x1D
#define MODELF_COM_END_OF_RAW_ACCESS	0x1E
#define MODELF_COM_LOAD_IMAGE_FULL	0x20
#define MODELF_COM_LOAD_IMAGE_AREA	0x22
#define MODELF_COM_END_OF_LOAD_IMAGE	0x23
#define MODELF_COM_IMAGE_ADDRESS	0x25
#define MODELF_COM_WAIT_TRIGGER_DONE	0x28
#define MODELF_COM_WAIT_FRAME_END	0x29
#define MODELF_COM_WAIT_LUT_FREE	0x2A
#define MODELF_COM_WAIT_MASKED_LUT_FREE	0x2B
#define MODELF_COM_INIT_UPDATE_BUFFER	0x32
#define MODELF_COM_UPDATE_FULL		0x33
#define MODELF_COM_UPDATE_AREA		0x34
#define MODELF_COM_PARTIAL_UPDATE_FULL	0x35
#define MODELF_COM_PARTIAL_UPDATE_AREA	0x36
#define MODELF_COM_CLEAR_GATE_DRIVER	0x37
#define MODELF_COM_KICKBACK_MESUREMENT	0x3D
#define MODELF_COM_VCOM_MESUREMENT_1	0x3E
#define MODELF_COM_VCOM_MESUREMENT_2	0x3D

/* model F register list */
#define MODELF_REG_REVISION_CODE	0x0000
#define MODELF_REG_PRODUCT_CODE		0x0002
#define MODELF_REG_POWER_SAVE_MODE	0x0006
#define MODELF_REG_SOFTWARE_RESET	0x0008
#define MODELF_REG_SYSTEM_STATUS	0x000A
#define MODELF_REG_CLOCK_CONFIGURATION	0x0010
#define MODELF_REG_TIMER_S_DRV_CONF	0x001C
#define MODELF_REG_I2C_CLOCK		0x001A
#define MODELF_REG_ALU_TEMPORARY_0	0x0080
#define MODELF_REG_MEMORY_ACCESS_CONF	0x0140
#define MODELF_REG_INTERRUPT_STATUS	0x0240
#define MODELF_REG_INTERRUPT_MASK	0x0242
#define MODELF_REG_INTERRUPT_CONTROL	0x0244
#define MODELF_REG_INIT_CODE_CHECKSUM	0x02A8
#define MODELF_REG_UPDATE_BUFFER_CONF	0x0330
#define MODELF_REG_FILL_COLOR		0x0332
#define MODELF_REG_DSPE_CONTROL		0x0334
#define MODELF_REG_LUT_STATUS		0x0336
#define MODELF_REG_DSPE_INT_STATUS	0x033A
#define MODELF_REG_DSPE_INT_ENABLE	0x033E
#define MODELF_REG_FRAME_DATA_LENGTH	0x0400
#define MODELF_REG_LINE_DATA_LENGTH	0x0406
#define MODELF_REG_WAVEFORM_DEC_BYPASS	0x0420
#define MODELF_REG_PROTECTION_KEY_1	0x042C
#define MODELF_REG_PROTECTION_KEY_2	0x042E
#define MODELF_REG_MTP_STATUS	0x0500
#define MODELF_REG_MTP_CONTROL	0x0502
#define MODELF_REG_MTP_ADRDATA	0x0504
#define MODELF_REG_MTP_READ_DATA	0x0506
#define MODELF_REG_SENSOR_TEMPERATURE	0x0576
#define MODELF_REG_FAKE_TEMPERATURE	0x057E

#define MODELF_DATA_PORT		0x0154

/* model F register bit field list */
#define MODELF_BF_WAVEFORM_INIT_BUSY	(1 << 7)
#define MODELF_BF_INT_HRDY_STATUS	(1 << 13)
#define MODELF_BF_DSPE_OPERATION_FILL	(0x1 << 1)
#define MODELF_BF_DSPE_OPERATE		(0x1 << 0)
#define MODELF_WAVEFORM_MODE(x)		((x << 8) & 0x0f00)
#define MODELF_UPDATE_LUT(x)		((x << 4) & 0x0030)
#define MODELF_LUT_AUTO_SELECT		(1 << 7)
#define MODELF_INT_DISPLAY_ENGINE	(1 << 1)
#define MODELF_INT_DSPE_ONE_LUT_FREE	(1 << 3)
#define MODELF_INT_WF_INVALID_FORMAT	(1 << 11)
#define MODELF_INT_WF_CHECKSUM_ERROR	(1 << 12)
#define MODELF_INT_WF_OVERFLOW		(1 << 13)
#define MODELF_INT_WF_UPDATE_INTERRUPT	(1 << 14)
#define MODELF_BF_INIT_CODE_CHECKSUM	(1 << 15)
#define MODELF_BF_MEMORY_ACCESS_TYPE	(1 << 0)
#define MODELF_BF_LUT_IN_USE		(0xf << 12)
#define MODELF_MTP_ALL_STATE_BUSY		(0xffff)
#define MODELF_MTP_READ_OPERATION_BUSY		(1 << 8)
#define MODELF_MTP_READ_MODE_BUSY		(1 << 12)
#define MODELF_VCOM_READ_TRIGGER		(1 << 3)
#define MODELF_MTP_READ_STOP_TRIGGER		(1 << 1)
#define MODELF_MTP_READ_START_TRIGGER		(1 << 0)

/* model F register bit field value list */
#define MODELF_INTERNAL_CLOCK_ENABLE	(1 << 7)
#define MODELF_INTERNAL_CLOCK_DISABLE	(0 << 7)
#define MODELF_BF_INTERNAL_CLOCK_DIV_2	(1 << 0)
#define MODELF_WAVEFORM_INIT_BUSY	(1 << 7)
#define MODELF_WAVEFORM_INIT_DONE	(0 << 7)
#define MODELF_WAVEFORM_AUTO_JUDGE	(1 << 2)
#define MODELF_WAVEFORM_DECORDER_SELECT	(1 << 1)
#define MODELF_INT_HRDY_STATUS_LOW	(0 << 13)
#define MODELF_INT_HRDY_STATUS_HIGH	(1 << 13)
#define MODELF_BPP_1			(0 << 4)
#define MODELF_BPP_2			(1 << 4)
#define MODELF_BPP_4			(2 << 4)
#define MODELF_BPP_8			(3 << 4)
#define MODELF_POWER_ACTIVE		(1 << 8)
#define MODELF_POWER_PASSIVE		(0 << 8)
#define MODELF_INIT_CODE_CHECKSUM_OK	(1 << 15)
#define MODELF_INIT_CODE_CHECKSUM_ERROR	(0 << 15)

/* model F const value list */
#define MODELF_PRODUCT_CODE		0x0053
#define MODELF_WAVEFORM_ADDR_LOW	0x0000
#define MODELF_WAVEFORM_ADDR_HIGH	0x0008
#define MODELF_WAVEFORM_WHITEOUT	0
#define MODELF_WAVEFORM_DIRECT_MONO	1
#define MODELF_WAVEFORM_HIGH_QUALITY	2
#define MODELF_WAVEFORM_HIGH_SPEED	3
#define MODELF_MTP_VCOM_ADDR	0x0000

/* model F status */
#define MODELF_STATUS_INIT_DONE		(1 << 0)
#define MODELF_STATUS_KEYCODE_STORED	(1 << 1)
#define MODELF_STATUS_INITCODE_STORED	(1 << 2)
#define MODELF_STATUS_WAVEFORM_STORED	(1 << 3)

#define MODELF_VRAM_NO_NEED_UPDATE	0
#define MODELF_VRAM_NEED_UPDATE		1

#define MODELF_NO_NEED_FLUSH_IMAGE	0
#define MODELF_NEED_FLUSH_IMAGE		1

#define MODELF_NO_NEED_CLEANUP		0
#define MODELF_NEED_CLEANUP		1

#define MODELF_RESUME_UPDATE		0
#define MODELF_SUSPEND_UPDATE		1

#define MODELF_POWER_RUN		0
#define MODELF_POWER_STANDBY		1
#define MODELF_POWER_SLEEP		2

#define MODELF_ONESHOT_TYPE_ONESHOT	0
#define MODELF_ONESHOT_TYPE_CLENUP	1

#define MODELF_SEND_WAVEFORM_NOWAIT	0
#define MODELF_SEND_WAVEFORM_WAIT	1

#endif

/*
 * Copyright (C) 2010-2014 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
/*
 * Based on STMP378X LCDIF
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

#include <linux/busfreq-imx.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/cpufreq.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/dmaengine.h>
#include <linux/pxp_dma.h>
#include <linux/pm_runtime.h>
#include <linux/mxcfb.h>
#include <linux/mxcfb_epdc.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/driver.h>
#include <linux/fsl_devices.h>
#include <linux/bitops.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/mtd/mtd.h>

#include <linux/string.h>

#ifdef CONFIG_LZSS
#include <linux/lzss.h>
#endif
/*
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/buffer_head.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
*/
#include <asm/cacheflush.h>

#include "epdc_regs.h"

#ifndef CONFIG_FB_MXC_EPDC_MAX_UPDATES
#define CONFIG_FB_MXC_EPDC_MAX_UPDATES 10
#endif

#define NO_DEAD_ZONES 1

/* Maximum size of the DMA memory allocated by this driver */
#define MXCFB_MAX_DMA_ALLOC ((unsigned long)88e6)
struct dead_zones;
struct fb_videomode;
struct imx_epdc_fb_mode;
struct mxc_epdc_fb_data;

/* module parameters */
#define USE_ALL_PARAMETERS 0
#define MXC_FB_INVALID_VCOM INT_MAX
#define MXC_FB_MAX_VCOM_N 2
static int mxc_epdc_fb_vcom_modparam[MXC_FB_MAX_VCOM_N] = {
	[0 ... (MXC_FB_MAX_VCOM_N - 1)] = MXC_FB_INVALID_VCOM };
static size_t mxc_epdc_fb_vcom_n_modparam = 0;
module_param_array_named(vcom, mxc_epdc_fb_vcom_modparam, int,
			 &mxc_epdc_fb_vcom_n_modparam, S_IRUGO);
MODULE_PARM_DESC(vcom, "VCOM voltages");

static char *mxc_epdc_fb_panel_type_modparam = "";
module_param_named(panel_type, mxc_epdc_fb_panel_type_modparam, charp,
		   S_IRUGO);
MODULE_PARM_DESC(panel_type, "Panel type identifier");

static int mxc_epdc_fb_use_cfa_modparam = 0;
module_param_named(use_cfa, mxc_epdc_fb_use_cfa_modparam, int,
		S_IRUGO);
//*		
static int mxc_epdc_fb_init_clear_modparam = 1;
module_param_named(init_clear, mxc_epdc_fb_init_clear_modparam, int,
		   S_IRUGO);
MODULE_PARM_DESC(mxc_epdc_fb_init_clear_modparam, "Clear Display on Init");
//*/

static int mxc_epdc_fb_use_nvm_modparam = 1;
module_param_named(use_nvm, mxc_epdc_fb_use_nvm_modparam, int,
		S_IRUGO);
int get_dead_zones(struct dead_zones* dz, int x);
int get_dead_zone_area(struct mxcfb_rect* area, struct dead_zones* dz);
void scramble_array(struct mxc_epdc_fb_data *fb_data, void* source_buf, void* target_buf, struct mxcfb_rect * update_rect);
/*		
static char *mxc_epdc_fb_panel_size_modparam = NULL;
module_param_named(panel_size, mxc_epdc_fb_panel_size_modparam, charp,
		   S_IRUGO);
MODULE_PARM_DESC(panel_size, "Panel size identifier");
*/
static char* mxc_epdc_fb_waveform_modparam = "";
module_param_named(waveform, mxc_epdc_fb_waveform_modparam, charp, 
		   S_IRUGO);
MODULE_PARM_DESC(waveform, "Waveform identifier");

static int mxc_epdc_fb_scrambling_modparam = 0;
module_param_named(use_scrambling, mxc_epdc_fb_scrambling_modparam, int,
		   S_IRUGO);
MODULE_PARM_DESC(use_scrambling, "Process all Content with Scrambling algorithm");

static int mxc_epdc_fb_dithering_modparam = 0;
module_param_named(use_dithering, mxc_epdc_fb_dithering_modparam, int,
		   S_IRUGO);
MODULE_PARM_DESC(use_dithering, "Process all Content with Dithering algorithm and Mono waveform");
#if USE_ALL_PARAMETERS
static char *vmode_name_modparam = "D107_T3.1";
static int vmode_refresh_modparam = 50;
static int vmode_xres_modparam = 1280;
static int vmode_yres_modparam = 960;
static long vmode_pixclock_modparam = 50000000;
static int vmode_left_margin_modparam = 166; /* line_begin */
static int vmode_right_margin_modparam = 56; /* line_end */
static int vmode_upper_margin_modparam = 4; /* frame_begin */
static int vmode_lower_margin_modparam = 4; /* frame_end */
static int vmode_hsync_len_modparam = 8;
static int vmode_vsync_len_modparam = 1;
static int vmode_sync_modparam = 0;
static int vmode_vmode_modparam = FB_VMODE_NONINTERLACED;
static int vmode_flag_modparam = 0;
static int ptype_sddo_shift_right_modparam = 1;
static int ptype_sddo_invert_modparam = 1;
static int ptype_vscan_holdoff_modparam = 1;
static int ptype_sdoed_width_modparam = 6;
static int ptype_sdoed_delay_modparam = 0;
static int ptype_sdoez_width_modparam = 0;
static int ptype_sdoez_delay_modparam = 0;
static int ptype_gdclk_hp_offs_modparam = 470;
static int ptype_gdsp_offs_modparam = 4;
static int ptype_gdsp_frame_sync_modparam = 0;
static int ptype_gdsp_active_high_modparam = 0;
static int ptype_gdoe_offs_modparam = 4;
static int ptype_gdoe_delayed_gclk_modparam = 0;
static int ptype_gdoe_active_high_modparam = 1;
static int ptype_gdclk_offs_modparam = 0;
static int ptype_num_ce_modparam = 1;
static int ptype_sddo_16_bits_modparam = 0;
static int ptype_sddo_flip_bits_modparam = 1;
static int ptype_tft_4bpp_modparam = 0;
static int ptype_dual_scan_modparam = 0;
static int ptype_scan_dir_0_up_modparam = 0;
static int ptype_scan_dir_1_up_modparam = 0;
static int ptype_flip_top_modparam = 0;
static int ptype_sdclk_hold_modparam = 0;
//static int ptype_left_border_modparam = 0;
//static int ptype_right_border_modparam = 0;

module_param_named(vmode_name, vmode_name_modparam, charp, S_IRUGO); 						//"PL_STD047",
module_param_named(vmode_refresh, vmode_refresh_modparam, int, S_IRUGO); 					//50,
module_param_named(vmode_xres, vmode_xres_modparam, int, S_IRUGO); 							//800,
module_param_named(vmode_yres, vmode_yres_modparam, int, S_IRUGO); 							//450,
module_param_named(vmode_pixclock, vmode_pixclock_modparam, long, S_IRUGO); 					//14424000,
module_param_named(vmode_left_margin, vmode_left_margin_modparam, int, S_IRUGO); 			//27, / * line_begin * /
module_param_named(vmode_right_margin, vmode_right_margin_modparam, int, S_IRUGO);	 		//88, / * line_end * /
module_param_named(vmode_upper_margin, vmode_upper_margin_modparam, int, S_IRUGO); 			//4, / * frame_begin * /
module_param_named(vmode_lower_margin, vmode_lower_margin_modparam, int, S_IRUGO); 			//4, / * frame_end * /
module_param_named(vmode_hsync_len, vmode_hsync_len_modparam, int, S_IRUGO); 				//11,
module_param_named(vmode_vsync_len, vmode_vsync_len_modparam, int, S_IRUGO); 				//1,
module_param_named(vmode_sync, vmode_sync_modparam, int, S_IRUGO); 							//0,
module_param_named(vmode_vmode, vmode_vmode_modparam, int, S_IRUGO); 						//FB_VMODE_NONINTERLACED,
module_param_named(vmode_flag, vmode_flag_modparam, int, S_IRUGO); 							//0,
module_param_named(ptype_sddo_shift_right, ptype_sddo_shift_right_modparam, int, S_IRUGO);		//1,
module_param_named(ptype_sddo_invert, ptype_sddo_invert_modparam, int, S_IRUGO); 	
module_param_named(ptype_vscan_holdoff, ptype_vscan_holdoff_modparam, int, S_IRUGO);		//1,
module_param_named(ptype_sdoed_width, ptype_sdoed_width_modparam, int, S_IRUGO); 			//10,
module_param_named(ptype_sdoed_delay, ptype_sdoed_delay_modparam, int, S_IRUGO); 			//0,
module_param_named(ptype_sdoez_width, ptype_sdoez_width_modparam, int, S_IRUGO); 			//10,
module_param_named(ptype_sdoez_delay, ptype_sdoez_delay_modparam, int, S_IRUGO); 			//20,
module_param_named(ptype_gdclk_hp_offs, ptype_gdclk_hp_offs_modparam, int, S_IRUGO); 		//740,
module_param_named(ptype_gdsp_offs, ptype_gdsp_offs_modparam, int, S_IRUGO); 				//2,
module_param_named(ptype_gdsp_frame_sync, ptype_gdsp_frame_sync_modparam, int, S_IRUGO); 	//false,
module_param_named(ptype_gdsp_active_high, ptype_gdsp_active_high_modparam, int, S_IRUGO); //false,
module_param_named(ptype_gdoe_offs, ptype_gdoe_offs_modparam, int, S_IRUGO); 				//24,
module_param_named(ptype_gdoe_delayed_gclk, ptype_gdoe_delayed_gclk_modparam, int, S_IRUGO); //true,
module_param_named(ptype_gdoe_active_high, ptype_gdoe_active_high_modparam, int, S_IRUGO); //true,
module_param_named(ptype_gdclk_offs, ptype_gdclk_offs_modparam, int, S_IRUGO); 				//4,
module_param_named(ptype_num_ce, ptype_num_ce_modparam, int, S_IRUGO); 						//1,
module_param_named(ptype_sddo_16_bits, ptype_sddo_16_bits_modparam, int, S_IRUGO); 			//0,
module_param_named(ptype_sddo_flip_bits, ptype_sddo_flip_bits_modparam, int, S_IRUGO); 	//false,
module_param_named(ptype_tft_4bpp, ptype_tft_4bpp_modparam, int, S_IRUGO); 					//false,
module_param_named(ptype_dual_scan, ptype_dual_scan_modparam, int, S_IRUGO); 					//false,
module_param_named(ptype_scan_dir_0_up, ptype_scan_dir_0_up_modparam, int, S_IRUGO); 		//false,
module_param_named(ptype_scan_dir_1_up, ptype_scan_dir_1_up_modparam, int, S_IRUGO); 		//false,
module_param_named(ptype_flip_top, ptype_flip_top_modparam, int, S_IRUGO); 				//false,
module_param_named(ptype_sdclk_hold, ptype_sdclk_hold_modparam, int, S_IRUGO); 			//false,
//module_param_named(ptype_left_border, ptype_left_border_modparam, int, S_IRUGO); 			//0,
//module_param_named(ptype_right_border, ptype_right_border_modparam, int, S_IRUGO); 			//0,

static int use_pedantic_conf_modparam = 0;
module_param_named(use_pedantic_conf, use_pedantic_conf_modparam, int, S_IRUGO);
#endif
/*
 * Enable this define to have a default panel
 * loaded during driver initialization
 */
#define DEFAULT_PANEL_HW_INIT

#define NUM_SCREENS_MIN	20

#define EPDC_V1_NUM_LUTS	16
#define EPDC_V1_MAX_NUM_UPDATES 20
#define EPDC_V2_NUM_LUTS	64
#define EPDC_V2_MAX_NUM_UPDATES 64
#define EPDC_MAX_NUM_BUFFERS	2
#define INVALID_LUT		(-1)
#define DRY_RUN_NO_LUT		100

/* Maximum update buffer image width due to v2.0 and v2.1 errata ERR005313. */
#define EPDC_V2_MAX_UPDATE_WIDTH	2047
#define EPDC_V2_ROTATION_ALIGNMENT	8

#define DEFAULT_TEMP_INDEX	0
#define DEFAULT_TEMP		20 /* room temp in deg Celsius */

#define INIT_UPDATE_MARKER	0x12345678
#define PAN_UPDATE_MARKER	0x12345679

#define POWER_STATE_OFF	0
#define POWER_STATE_ON	1

#define MERGE_OK	0
#define MERGE_FAIL	1
#define MERGE_BLOCK	2

#define NVM_PANELTYPE_VALID 0x01
#define NVM_VCOM_VALID 0x02
#define NVM_WF_NAME_VALID 0x04
#define NVM_WAVEFORM_VALID 0x08
#define NVM_WFLEN_VALID 0x10


static unsigned long default_bpp = 16;
DEFINE_MUTEX(hard_lock);

int epdc_max_update_width = EPDC_V2_MAX_UPDATE_WIDTH;

struct nvm_content{
	char paneltype[15];
	char wf_name[31];
	int vcom[2];
	int wflen[2];
	int wf_num;
	uint8_t waveform[0x3AFFF];
};

struct dead_zones{
	int w;
	int l;
};

struct hbz_gpio{
	int gpio_vcom_sw1;
	int gpio_vcom_sw2;
	int gpio_vsource_low1;
	int gpio_vsource_high1;
	int gpio_vsource_low2;
	int gpio_vsource_high2;
};

struct update_marker_data {
	struct list_head aw_list;
	struct list_head full_list;
	struct list_head upd_list;
	u32 update_marker;
	struct completion update_completion;
	int lut_num;
	bool collision_test;
	bool waiting;
};

struct update_desc_list {
	struct list_head list;
	struct mxcfb_update_data upd_data;/* Update parameters */
	u32 epdc_offs;		/* Added to buffer ptr to resolve alignment */
	u32 epdc_stride;	/* Depends on rotation & whether we skip PxP */
	struct list_head upd_marker_list; /* List of markers for this update */
	u32 update_order;	/* Numeric ordering value for update */
};

/* This structure represents a list node containing both
 * a memory region allocated as an output buffer for the PxP
 * update processing task, and the update description (mode, region, etc.) */
struct update_data_list {
	struct list_head list;
	dma_addr_t phys_addr;	/* Pointer to phys address of processed Y buf */
	void *virt_addr;
	struct update_desc_list *update_desc;
	int lut_num;		/* Assigned before update is processed into working buffer */
	u64 collision_mask;	/* Set when update creates collision */
				/* Mask of the LUTs the update collides with */
};

struct mxc_epdc_fb_data {
	struct fb_info info;
	struct fb_var_screeninfo epdc_fb_var; /* Internal copy of screeninfo
						so we can sync changes to it */
	u32 pseudo_palette[16];
	char fw_str[256];
	struct list_head list;
	const struct imx_epdc_fb_mode *cur_mode;
	struct imx_epdc_fb_platform_data *pdata;
	int blank;
	u32 max_pix_size;
	ssize_t map_size;
	dma_addr_t phys_start;
	u32 fb_offset;
	int default_bpp;
	int native_width;
	int native_height;
	int epdc_xres;
	int num_screens;
	int use_cfa;
	int epdc_irq;
	struct device *dev;
	int power_state;
	int wait_for_powerdown;
	struct completion powerdown_compl;
	struct clk *epdc_clk_axi;
	struct clk *epdc_clk_pix;
#if 0
	struct regulator *display_regulator;
	struct regulator *vcom_regulator;
	struct regulator *v3p3_regulator;
#else
	struct regulator* display_regulator[2];
	struct regulator* vcom_regulator[2];
	struct regulator* v3p3_regulator[2];	
	struct dead_zones* dead_zones;
	int dead_pixel_in_line;
	struct nvm_content nvm;
	int vcom[2];
#endif
//#endif
	bool fw_default_load;
	int rev;
	int buf_pix_fmt;
	uint8_t prev_image[16777216];
	/* FB elements related to EPDC updates */
	int num_luts;
	int max_num_updates;
	bool in_init;
	bool hw_ready;
	bool hw_initializing;
	struct completion init_completion;
	bool waiting_for_idle;
	u32 auto_mode;
	u32 upd_scheme;
	struct list_head upd_pending_list;
	struct list_head upd_buf_queue;
	struct list_head upd_buf_free_list;
	struct list_head upd_buf_collision_list;
	struct update_data_list *cur_update;
	struct mutex queue_mutex;
	int trt_entries;
	int temp_index;
	u8 *temp_range_bounds;
	struct mxcfb_waveform_modes wv_modes;
	bool wv_modes_update;
	u32 *waveform_buffer_virt;
	u32 waveform_buffer_phys;
	u32 waveform_buffer_size;
	u32 *working_buffer_virt;
	u32 working_buffer_phys;
	u32 working_buffer_size;
	u32 max_updates;
	dma_addr_t *phys_addr_updbuf;
	void **virt_addr_updbuf;
	u32 upd_buffer_num;
	u32 max_num_buffers;
	dma_addr_t phys_addr_copybuf;	/* Phys address of copied update data */
	void *virt_addr_copybuf;	/* Used for PxP SW workaround */
	u32 order_cnt;
	struct list_head full_marker_list;
	struct list_head awaiting_marker_list;
	u32 *lut_update_order;		/* Array size = number of luts */
	u64 epdc_colliding_luts;
	u64 luts_complete_wb;
	struct completion updates_done;
	struct delayed_work epdc_done_work;
	struct workqueue_struct *epdc_submit_workqueue;
	struct work_struct epdc_submit_work;
	struct workqueue_struct *epdc_intr_workqueue;
	struct work_struct epdc_intr_work;
	bool waiting_for_wb;
	bool waiting_for_lut;
	bool waiting_for_lut15;
	struct completion update_res_free;
	struct completion lut15_free;
	struct completion eof_event;
	int eof_sync_period;
	struct mutex power_mutex;
	bool powering_down;
	bool updates_active;
	int pwrdown_delay;
	unsigned long tce_prevent;
	bool restrict_width; /* work around rev >=2.0 width and
				stride restriction  */
	struct hbz_gpio* hbz_gpio;
	/* FB elements related to PxP DMA */
	struct completion pxp_tx_cmpl;
	struct pxp_channel *pxp_chan;
	struct pxp_config_data pxp_conf;
	struct dma_async_tx_descriptor *txd;
	dma_cookie_t cookie;
	struct scatterlist sg[2];
	int no_powerdown;
	struct mutex pxp_mutex; /* protects access to PxP */
};

struct waveform_data_header {
	unsigned int wi0;//0
	unsigned int wi1;//1
	unsigned int wi2;//2
	unsigned int wi3;//3
	unsigned int wi4;//4
	unsigned int wi5;//5
	unsigned int wi6;//6
	unsigned int xwia:24;
	unsigned int cs1:8;//7
	unsigned int wmta:24;
	unsigned int fvsn:8;//8
	unsigned int luts:8;
	unsigned int mc:8;
	unsigned int trc:8;
	unsigned int reserved0_0:8;//9
	unsigned int eb:8;
	unsigned int sb:8;
	unsigned int reserved0_1:8;
	unsigned int reserved0_2:8;//10
	unsigned int reserved0_3:8;
	unsigned int reserved0_4:8;
	unsigned int reserved0_5:8;
	unsigned int cs2:8;
};

struct mxcfb_waveform_data_file {
	struct waveform_data_header wdh;
	u32 *data;	/* Temperature Range Table + Waveform Data */
};

static struct fb_videomode s079_t1_3_mode = {
	.name = "S079_T1.3",
	.refresh = 50,
	.xres = 1920,
	.yres = 768,
	.pixclock = 27900000,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode s079_t1_4_mode = { //2x8
	.name = "S079_T1.4",
	.refresh = 50,
	.xres = 3968,
	.yres = 768,
	.pixclock = 89263420,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode s079_t1_2_mode = { //2x3
	.name = "S079_T1.2",
	.refresh = 50,
	.xres = 1408,
	.yres = 768,
	.pixclock = 17293997,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode s079_t1_1_mode = { //1x1
	.name = "S079_T1.1",
	.refresh = 50,
	.xres = 384,
	.yres = 384,
	.pixclock = 7232532,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
}; 

static struct fb_videomode s115_t1_5_mode = { //2x2
	.name = "S115_T1.5",
	.refresh = 50,
	.xres = 2048,
	.yres = 384,
	.pixclock = 13829346,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};    

static struct fb_videomode s115_t1_4_mode = { //2x3
	.name = "S115_T1.4",
	.refresh = 50,
	.xres = 3072,
	.yres = 384,
	.pixclock = 17286683, //20744019,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode s115_t1_6_mode = { //1x4
	.name = "S115_T1.6",
	.refresh = 50,
	.xres = 4096,
	.yres = 192,
	.pixclock = 13658692,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode s115_t1_3_mode = { //2x4
	.name = "S115_T1.3",
	.refresh = 50,
	.xres = 4096,
	.yres = 384,
	.pixclock = 27658692,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode s115_t1_2_mode = { //2x1
	.name = "S115_T1.2",
	.refresh = 50,
	.xres = 1024,
	.yres = 384,
	.pixclock = 6914673,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
     
static struct fb_videomode s115_t1_1_mode = { // 1x1
	.name = "S115_T1.1",
	.refresh = 50,
	.xres = 1024,
	.yres = 192,
	.pixclock = 6915630,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
   
static struct fb_videomode s047_t2_1_mode = {
	.name = "S047_T2.1",
	.refresh = 50,
	.xres = 800,
	.yres = 450,
	.pixclock = 12e6,
	.left_margin = 166, 
	.right_margin = 56, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode s047_t2_1_85hz_mode = {
	.name = "S047_T2.1_85Hz",
	.refresh = 50,
	.xres = 800,
	.yres = 450,
	.pixclock = 20e6,
	.left_margin = 166, 
	.right_margin = 56, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode d107c_t3_1_mode = {
	.name = "D107C_T3.1",
	.refresh = 50,
	.xres = 1280,
	.yres = 960,
	.pixclock = 40e6, //25e6,
	.left_margin = 166,//216,//166, 
	.right_margin = 2,//56, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 1,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode d107_t3_1_mode = {
	.name = "D107_T3.1",
	.refresh = 50,
	.xres = 1280,
	.yres = 960,
	.pixclock = 40e6, //25e6,
	.left_margin = 166,//216,//166, 
	.right_margin = 2,//56, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 1,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode d107_t3_0_mode = {
	.name = "D107_T3.0",
	.refresh = 50,
	.xres = 2560,
	.yres = 960,
	.pixclock = 25e6, //25e6,
	.left_margin = 166,//216,//166, 
	.right_margin = 2,//56, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode d107_t3_1_85hz_mode = {
	.name = "D107_T3.1_85Hz",
	.refresh = 50,
	.xres = 1280,
	.yres = 960,
	.pixclock = 85e6, //25e6,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode t154_t3_1_mode = {
	.name = "T154_T3.1",
	.refresh = 50,
	.xres = 1280,
	.yres = 1920,
	.pixclock = 50e6, //25e6,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode q154_t3_1_mode = {
	.name = "Q154_T3.1",
	.refresh = 50,
	.xres = 1280,
	.yres = 1920,
	.pixclock = 50e6, //25e6,
	.left_margin = 166, 
	.right_margin = 2, 
	.upper_margin = 4, 
	.lower_margin = 4, 
	.hsync_len = 8,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode e60_v110_mode = {
	.name = "E60_V110",
	.refresh = 50,
	.xres = 800,
	.yres = 600,
	.pixclock = 18604700,
	.left_margin = 8,
	.right_margin = 178,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode e60_v220_mode = {
	.name = "E60_V220",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 30000000,
	.left_margin = 8,
	.right_margin = 164,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode e060scm_mode = {
	.name = "E060SCM",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 26666667,
	.left_margin = 8,
	.right_margin = 100,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct fb_videomode e97_v110_mode = {
	.name = "E97_V110",
	.refresh = 50,
	.xres = 1200,
	.yres = 825,
	.pixclock = 32000000,
	.left_margin = 12,
	.right_margin = 128,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct imx_epdc_fb_mode panel_modes[] = {
	{
		&e60_v110_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		428,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e60_v220_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		465,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		9,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e060scm_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		419,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		5,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e97_v110_mode,
		8,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		632,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		3,      /* num_ce */
	},
	/* S115 11,5" shelf label display with 1380x96px */
		{// 1x1
		.vmode = &s115_t1_1_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 350,
		.gdsp_offs = 800,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = false,
		.scan_dir_0_up = true,
		.scan_dir_1_up = true,
		.flip_top = false,
		.sdclk_hold = false,

	},
     { //2x1
		.vmode = &s115_t1_2_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 350,
		.gdsp_offs = 800, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = true,
		.scan_dir_0_up = true,
		.scan_dir_1_up = true,
		.flip_top = false,
		.sdclk_hold = false,

	},
	{ //2x3
		.vmode = &s115_t1_4_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 880,
		.gdsp_offs = 2000, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = false,
		.tft_4bpp = false,//
		.dual_scan = true,
		.scan_dir_0_up = true,
		.scan_dir_1_up = true,
		.flip_top = false,
		.sdclk_hold = false,

	},
		{ //1x4
		.vmode = &s115_t1_6_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 880,
		.gdsp_offs = 2400, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = false,
		.tft_4bpp = false,//
		.dual_scan = false,
		.scan_dir_0_up = true,
		.scan_dir_1_up = true,
		.flip_top = false,
		.sdclk_hold = false,
	},
	{ //2x2
		.vmode = &s115_t1_5_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 880,
		.gdsp_offs = 1400, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = false,
		.tft_4bpp = false,//
		.dual_scan = true,
		.scan_dir_0_up = true,
		.scan_dir_1_up = true,
		.flip_top = false,
		.sdclk_hold = false,

	},
		{ //2x4
		.vmode = &s115_t1_3_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 880,
		.gdsp_offs = 2400, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = false,
		.tft_4bpp = false,//
		.dual_scan = true,
		.scan_dir_0_up = true,
		.scan_dir_1_up = true,
		.flip_top = false,
		.sdclk_hold = false,

	},
	/* S079 7,92" Display 768x192px */
	{//2x4
		.vmode = &s079_t1_3_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 650,
		.gdsp_offs = 1500, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = true,
		.scan_dir_0_up = false,
		.scan_dir_1_up = false,
		.flip_top = false,
		.sdclk_hold = false,

	},
	{//2x8
		.vmode = &s079_t1_4_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 1000,
		.gdsp_offs = 2200, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = true,
		.scan_dir_0_up = false,
		.scan_dir_1_up = false,
		.flip_top = false,
		.sdclk_hold = false,

	},
	{//2x1
		.vmode = &s079_t1_2_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 500,
		.gdsp_offs = 1266, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = true,
		.scan_dir_0_up = false,
		.scan_dir_1_up = false,
		.flip_top = false,
		.sdclk_hold = false,

	},
	{ //1x1
		.vmode = &s079_t1_1_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 250,
		.gdsp_offs = 500, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = false,
		.scan_dir_0_up = false,
		.scan_dir_1_up = false,
		.flip_top = false,
		.sdclk_hold = false,

	},
	{
		.vmode = &s047_t2_1_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 440,
		.gdsp_offs = 1266, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = false,
		.scan_dir_0_up = false,
		.scan_dir_1_up = false,
		.flip_top = false,
		.sdclk_hold = false,

	},
	{
		.vmode = &s047_t2_1_85hz_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 440,
		.gdsp_offs = 1266, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = false,
		.scan_dir_0_up = false,
		.scan_dir_1_up = false,
		.flip_top = false,
		.sdclk_hold = false,

	},
	{
		.vmode = &d107_t3_1_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 450,
		.gdsp_offs = 1266, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,//
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = false,
		.scan_dir_0_up = false,
		.scan_dir_1_up = false,
		.flip_top = false,
		.sdclk_hold = false,

	},
	
	{
		.vmode = &d107c_t3_1_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 450,
		.gdsp_offs = 1266, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,//
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = false,
		.scan_dir_0_up = false,
		.scan_dir_1_up = false,
		.flip_top = false,
		.sdclk_hold = false,

	},
	{
		.vmode = &d107_t3_0_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 790,
		.gdsp_offs = 1266, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,//
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = false,
		.scan_dir_0_up = false,
		.scan_dir_1_up = false,
		.flip_top = false,
		.sdclk_hold = false,
},
	{
		.vmode = &d107_t3_1_85hz_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 470,
		.gdsp_offs = 1266, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.sddo_invert = false,
		.sddo_shift_right = true,
		.tft_4bpp = false,
		.dual_scan = false,
		.scan_dir_0_up = false,
		.scan_dir_1_up = false,
		.flip_top = false,
		.sdclk_hold = false,

	},
	{
		.vmode = &t154_t3_1_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 450,
		.gdsp_offs = 1266, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.sddo_invert = false,
		.sddo_shift_right = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.tft_4bpp = false,
		.dual_scan = true,
		.scan_dir_0_up = false,
		.scan_dir_1_up = false,
		.flip_top = false,
		.sdclk_hold = false,

	},
	{
		.vmode = &q154_t3_1_mode,
		.vscan_holdoff = 1,
		.sdoed_width = 6,
		.sdoed_delay = 0,
		.sdoez_width = 0,
		.sdoez_delay = 0,
		.gdclk_hp_offs = 450,
		.gdsp_offs = 1266, //4 ,
		.gdsp_frame_sync = false,
		.gdsp_active_high = false,
		.gdoe_offs = 1, //4,
		.gdoe_delayed_gclk = false,
		.gdoe_active_high = true,
		.gdclk_offs = 0,
		.num_ce = 1,
		.sddo_shift_right = true,
		.sddo_invert = false,
	#if 0
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.tft_4bpp = false,
		.dual_scan = true,
		.scan_dir_0_up = true,
		.scan_dir_1_up = false,
		.flip_top = true,
		.sdclk_hold = false,
	#else
		.sddo_16_bits = 0,
		.sddo_flip_bits = true,
		.tft_4bpp = false,
		.dual_scan = true,
		.scan_dir_0_up = true,
		.scan_dir_1_up = false,
		.flip_top = true,
		.sdclk_hold = false,
	#endif
	},//*/


};

static struct imx_epdc_fb_platform_data epdc_data = {
	.epdc_mode = panel_modes,
	.num_modes = ARRAY_SIZE(panel_modes),
};

void __iomem *epdc_base;

struct mxc_epdc_fb_data *g_fb_data;

/* forward declaration */

static void v3p3_powerdown(struct mxc_epdc_fb_data *fb_data);
static void v3p3_powerup(struct mxc_epdc_fb_data *fb_data);
static int mxc_epdc_fb_get_temp_index(struct mxc_epdc_fb_data *fb_data,
						int temp);
static void mxc_epdc_fb_flush_updates(struct mxc_epdc_fb_data *fb_data);
static int mxc_epdc_fb_blank(int blank, struct fb_info *info);
static int mxc_epdc_fb_init_hw(struct fb_info *info);
static int pxp_process_update(struct mxc_epdc_fb_data *fb_data,
			      u32 src_width, u32 src_height,
			      struct mxcfb_rect *update_region);
static int pxp_complete_update(struct mxc_epdc_fb_data *fb_data, u32 *hist_stat);
static int copy_update(struct mxc_epdc_fb_data *fb_data,
			      void *src_buf_virt, void *dst_buf_virt,
			      struct mxcfb_rect *src_update_region);
static int regal_process_update(struct mxc_epdc_fb_data *fb_data,
			      void *src_buf_virt,
			      struct mxcfb_rect *src_update_region);
static int cfa_process_update(struct mxc_epdc_fb_data *fb_data,
			      u32 src_width, u32 src_height,
			      void *src_buf_virt, void *dst_buf_virt,
			      struct mxcfb_rect *src_update_region);

static void draw_mode0(struct mxc_epdc_fb_data *fb_data);
static bool is_free_list_full(struct mxc_epdc_fb_data *fb_data);

static void do_dithering_processing_Y1_v1_0(
		unsigned char *update_region_ptr,
		struct mxcfb_rect *update_region,
		unsigned long update_region_stride,
		int *err_dist);
static void do_dithering_processing_Y4_v1_0(
		unsigned char *update_region_ptr,
		struct mxcfb_rect *update_region,
		unsigned long update_region_stride,
		int *err_dist);
static void do_ordered_dithering_processing(
		unsigned char *update_region_ptr,
		struct mxcfb_rect *update_region,
		unsigned long update_region_stride,
		int *err_dist);

static void dump_fb_mode(struct mxc_epdc_fb_data *fb_data)
{
	printk("***** DUMP FB MODE *****\n");
	printk("struct fb_videomode *vmode: %p\n",fb_data->cur_mode->vmode);
	printk("struct fb_videomode *vmode.name: %s\n",fb_data->cur_mode->vmode->name);
	printk("struct fb_videomode *vmode.refresh: %i\n",fb_data->cur_mode->vmode->refresh);
	printk("struct fb_videomode *vmode.xres: %i\n",fb_data->cur_mode->vmode->xres);
	printk("struct fb_videomode *vmode.yres: %i\n",fb_data->cur_mode->vmode->yres);
	printk("struct fb_videomode *vmode.pixclock: %i\n",fb_data->cur_mode->vmode->pixclock);
	printk("struct fb_videomode *vmode.left_margin: %i\n",fb_data->cur_mode->vmode->left_margin);
	printk("struct fb_videomode *vmode.right_margin: %i\n",fb_data->cur_mode->vmode->right_margin);
	printk("struct fb_videomode *vmode.upper_margin: %i\n",fb_data->cur_mode->vmode->upper_margin);
	printk("struct fb_videomode *vmode.lower_margin: %i\n",fb_data->cur_mode->vmode->lower_margin);
	printk("struct fb_videomode *vmode.hsync_len: %i\n",fb_data->cur_mode->vmode->hsync_len);
	printk("struct fb_videomode *vmode.vsync_len: %i\n",fb_data->cur_mode->vmode->vsync_len);
	printk("struct fb_videomode *vmode.sync: %i\n",fb_data->cur_mode->vmode->sync);
	printk("struct fb_videomode *vmode.vmode: %i\n",fb_data->cur_mode->vmode->vmode);
	printk("struct fb_videomode *vmode.flag: %i\n",fb_data->cur_mode->vmode->flag);
    printk("int vscan_holdoff: %i\n",fb_data->cur_mode->vscan_holdoff);
    printk("int sdoed_width: %i\n",fb_data->cur_mode->sdoed_width);
    printk("int sdoed_delay: %i\n",fb_data->cur_mode->sdoed_delay);
    printk("int sdoez_width: %i\n",fb_data->cur_mode->sdoez_width);
    printk("int sdoez_delay: %i\n",fb_data->cur_mode->sdoez_delay);
    printk("int gdclk_hp_offs: %i\n",fb_data->cur_mode->gdclk_hp_offs);
    printk("int gdsp_offs: %i\n",fb_data->cur_mode->gdsp_offs);
    printk("bool gdsp_frame_sync: %i\n",fb_data->cur_mode->gdsp_frame_sync);
    printk("bool gdsp_active_high: %i\n",fb_data->cur_mode->gdsp_active_high);
    printk("int gdoe_offs: %i\n",fb_data->cur_mode->gdoe_offs);
    printk("bool gdoe_delayed_gclk: %i\n",fb_data->cur_mode->gdoe_delayed_gclk);
    printk("bool gdoe_active_high: %i\n",fb_data->cur_mode->gdoe_active_high);
    printk("int gdclk_offs: %i\n",fb_data->cur_mode->gdclk_offs);
    printk("int num_ce: %i\n",fb_data->cur_mode->num_ce);
    printk("bool sddo_16_bits: %i\n",fb_data->cur_mode->sddo_16_bits);
    printk("bool sddo_flip_bits: %i\n",fb_data->cur_mode->sddo_flip_bits);
    printk("bool tft_4bpp: %i\n",fb_data->cur_mode->tft_4bpp);
    printk("bool dual_scan: %i\n",fb_data->cur_mode->dual_scan);
    printk("bool scan_dir_0_up: %i\n",fb_data->cur_mode->scan_dir_0_up);
    printk("bool scan_dir_1_up: %i\n",fb_data->cur_mode->scan_dir_1_up);
    printk("bool flip_top: %i\n",fb_data->cur_mode->flip_top);
    printk("bool sdclk_hold: %i\n",fb_data->cur_mode->sdclk_hold);
    //printk("int left_border: %i\n",fb_data->cur_mode->left_border);
    //printk("int right_border: %i\n",fb_data->cur_mode->right_border);
}
	
#ifdef DEBUG
static void dump_pxp_config(struct mxc_epdc_fb_data *fb_data,
			    struct pxp_config_data *pxp_conf)
{

	dev_info(fb_data->dev, "S0 fmt 0x%x",
		pxp_conf->s0_param.pixel_fmt);
	dev_info(fb_data->dev, "S0 width 0x%x",
		pxp_conf->s0_param.width);
	dev_info(fb_data->dev, "S0 height 0x%x",
		pxp_conf->s0_param.height);
	dev_info(fb_data->dev, "S0 ckey 0x%x",
		pxp_conf->s0_param.color_key);
	dev_info(fb_data->dev, "S0 ckey en 0x%x",
		pxp_conf->s0_param.color_key_enable);

	dev_info(fb_data->dev, "OL0 combine en 0x%x",
		pxp_conf->ol_param[0].combine_enable);
	dev_info(fb_data->dev, "OL0 fmt 0x%x",
		pxp_conf->ol_param[0].pixel_fmt);
	dev_info(fb_data->dev, "OL0 width 0x%x",
		pxp_conf->ol_param[0].width);
	dev_info(fb_data->dev, "OL0 height 0x%x",
		pxp_conf->ol_param[0].height);
	dev_info(fb_data->dev, "OL0 ckey 0x%x",
		pxp_conf->ol_param[0].color_key);
	dev_info(fb_data->dev, "OL0 ckey en 0x%x",
		pxp_conf->ol_param[0].color_key_enable);
	dev_info(fb_data->dev, "OL0 alpha 0x%x",
		pxp_conf->ol_param[0].global_alpha);
	dev_info(fb_data->dev, "OL0 alpha en 0x%x",
		pxp_conf->ol_param[0].global_alpha_enable);
	dev_info(fb_data->dev, "OL0 local alpha en 0x%x",
		pxp_conf->ol_param[0].local_alpha_enable);

	dev_info(fb_data->dev, "Out fmt 0x%x",
		pxp_conf->out_param.pixel_fmt);
	dev_info(fb_data->dev, "Out width 0x%x",
		pxp_conf->out_param.width);
	dev_info(fb_data->dev, "Out height 0x%x",
		pxp_conf->out_param.height);

	dev_info(fb_data->dev,
		"drect left 0x%x right 0x%x width 0x%x height 0x%x",
		pxp_conf->proc_data.drect.left, pxp_conf->proc_data.drect.top,
		pxp_conf->proc_data.drect.width,
		pxp_conf->proc_data.drect.height);
	dev_info(fb_data->dev,
		"srect left 0x%x right 0x%x width 0x%x height 0x%x",
		pxp_conf->proc_data.srect.left, pxp_conf->proc_data.srect.top,
		pxp_conf->proc_data.srect.width,
		pxp_conf->proc_data.srect.height);
	dev_info(fb_data->dev, "Scaling en 0x%x", pxp_conf->proc_data.scaling);
	dev_info(fb_data->dev, "HFlip en 0x%x", pxp_conf->proc_data.hflip);
	dev_info(fb_data->dev, "VFlip en 0x%x", pxp_conf->proc_data.vflip);
	dev_info(fb_data->dev, "Rotation 0x%x", pxp_conf->proc_data.rotate);
	dev_info(fb_data->dev, "BG Color 0x%x", pxp_conf->proc_data.bgcolor);
}

static void dump_epdc_reg(void)
{

	printk(/*KERN_DEBUG*/ "\n\n");
	printk(/*KERN_DEBUG*/ "EPDC_CTRL 0x%x\n", __raw_readl(EPDC_CTRL));
	printk(/*KERN_DEBUG*/ "EPDC_WVADDR 0x%x\n", __raw_readl(EPDC_WVADDR));
	printk(/*KERN_DEBUG*/ "EPDC_WB_ADDR 0x%x\n", __raw_readl(EPDC_WB_ADDR));
	printk(/*KERN_DEBUG*/ "EPDC_RES 0x%x\n", __raw_readl(EPDC_RES));
	printk(/*KERN_DEBUG*/ "EPDC_FORMAT 0x%x\n", __raw_readl(EPDC_FORMAT));
	printk(/*KERN_DEBUG*/ "EPDC_FIFOCTRL 0x%x\n", __raw_readl(EPDC_FIFOCTRL));
	printk(/*KERN_DEBUG*/ "EPDC_UPD_ADDR 0x%x\n", __raw_readl(EPDC_UPD_ADDR));
	printk(/*KERN_DEBUG*/ "EPDC_UPD_STRIDE 0x%x\n", __raw_readl(EPDC_UPD_STRIDE));
	printk(/*KERN_DEBUG*/ "EPDC_UPD_FIXED 0x%x\n", __raw_readl(EPDC_UPD_FIXED));
	printk(/*KERN_DEBUG*/ "EPDC_UPD_CORD 0x%x\n", __raw_readl(EPDC_UPD_CORD));
	printk(/*KERN_DEBUG*/ "EPDC_UPD_SIZE 0x%x\n", __raw_readl(EPDC_UPD_SIZE));
	printk(/*KERN_DEBUG*/ "EPDC_UPD_CTRL 0x%x\n", __raw_readl(EPDC_UPD_CTRL));
	printk(/*KERN_DEBUG*/ "EPDC_TEMP 0x%x\n", __raw_readl(EPDC_TEMP));
	printk(/*KERN_DEBUG*/ "EPDC_AUTOWV_LUT 0x%x\n", __raw_readl(EPDC_AUTOWV_LUT));
	printk(/*KERN_DEBUG*/ "EPDC_TCE_CTRL 0x%x\n", __raw_readl(EPDC_TCE_CTRL));
	printk(/*KERN_DEBUG*/ "EPDC_TCE_SDCFG 0x%x\n", __raw_readl(EPDC_TCE_SDCFG));
	printk(/*KERN_DEBUG*/ "EPDC_TCE_GDCFG 0x%x\n", __raw_readl(EPDC_TCE_GDCFG));
	printk(/*KERN_DEBUG*/ "EPDC_TCE_HSCAN1 0x%x\n", __raw_readl(EPDC_TCE_HSCAN1));
	printk(/*KERN_DEBUG*/ "EPDC_TCE_HSCAN2 0x%x\n", __raw_readl(EPDC_TCE_HSCAN2));
	printk(/*KERN_DEBUG*/ "EPDC_TCE_VSCAN 0x%x\n", __raw_readl(EPDC_TCE_VSCAN));
	printk(/*KERN_DEBUG*/ "EPDC_TCE_OE 0x%x\n", __raw_readl(EPDC_TCE_OE));
	printk(/*KERN_DEBUG*/ "EPDC_TCE_POLARITY 0x%x\n", __raw_readl(EPDC_TCE_POLARITY));
	printk(/*KERN_DEBUG*/ "EPDC_TCE_TIMING1 0x%x\n", __raw_readl(EPDC_TCE_TIMING1));
	printk(/*KERN_DEBUG*/ "EPDC_TCE_TIMING2 0x%x\n", __raw_readl(EPDC_TCE_TIMING2));
	printk(/*KERN_DEBUG*/ "EPDC_TCE_TIMING3 0x%x\n", __raw_readl(EPDC_TCE_TIMING3));
	printk(/*KERN_DEBUG*/ "EPDC_PIGEON_CTRL0 0x%x\n", __raw_readl(EPDC_PIGEON_CTRL0));
	printk(/*KERN_DEBUG*/ "EPDC_PIGEON_CTRL1 0x%x\n", __raw_readl(EPDC_PIGEON_CTRL1));
	printk(/*KERN_DEBUG*/ "EPDC_IRQ_MASK1 0x%x\n", __raw_readl(EPDC_IRQ_MASK1));
	printk(/*KERN_DEBUG*/ "EPDC_IRQ_MASK2 0x%x\n", __raw_readl(EPDC_IRQ_MASK2));
	printk(/*KERN_DEBUG*/ "EPDC_IRQ1 0x%x\n", __raw_readl(EPDC_IRQ1));
	printk(/*KERN_DEBUG*/ "EPDC_IRQ2 0x%x\n", __raw_readl(EPDC_IRQ2));
	printk(/*KERN_DEBUG*/ "EPDC_IRQ_MASK 0x%x\n", __raw_readl(EPDC_IRQ_MASK));
	printk(/*KERN_DEBUG*/ "EPDC_IRQ 0x%x\n", __raw_readl(EPDC_IRQ));
	printk(/*KERN_DEBUG*/ "EPDC_STATUS_LUTS 0x%x\n", __raw_readl(EPDC_STATUS_LUTS));
	printk(/*KERN_DEBUG*/ "EPDC_STATUS_LUTS2 0x%x\n", __raw_readl(EPDC_STATUS_LUTS2));
	printk(/*KERN_DEBUG*/ "EPDC_STATUS_NEXTLUT 0x%x\n", __raw_readl(EPDC_STATUS_NEXTLUT));
	printk(/*KERN_DEBUG*/ "EPDC_STATUS_COL1 0x%x\n", __raw_readl(EPDC_STATUS_COL));
	printk(/*KERN_DEBUG*/ "EPDC_STATUS_COL2 0x%x\n", __raw_readl(EPDC_STATUS_COL2));
	printk(/*KERN_DEBUG*/ "EPDC_STATUS 0x%x\n", __raw_readl(EPDC_STATUS));
	printk(/*KERN_DEBUG*/ "EPDC_UPD_COL_CORD 0x%x\n", __raw_readl(EPDC_UPD_COL_CORD));
	printk(/*KERN_DEBUG*/ "EPDC_UPD_COL_SIZE 0x%x\n", __raw_readl(EPDC_UPD_COL_SIZE));
	printk(/*KERN_DEBUG*/ "EPDC_DEBUG 0x%x\n", __raw_readl(EPDC_DEBUG));
	printk(/*KERN_DEBUG*/ "EPDC_DEBUG_LUT 0x%x\n", __raw_readl(EPDC_DEBUG_LUT));
	printk(/*KERN_DEBUG*/ "EPDC_HIST1_PARAM 0x%x\n", __raw_readl(EPDC_HIST1_PARAM));
	printk(/*KERN_DEBUG*/ "EPDC_HIST2_PARAM 0x%x\n", __raw_readl(EPDC_HIST2_PARAM));
	printk(/*KERN_DEBUG*/ "EPDC_HIST4_PARAM 0x%x\n", __raw_readl(EPDC_HIST4_PARAM));
	printk(/*KERN_DEBUG*/ "EPDC_HIST8_PARAM0 0x%x\n", __raw_readl(EPDC_HIST8_PARAM0));
	printk(/*KERN_DEBUG*/ "EPDC_HIST8_PARAM1 0x%x\n", __raw_readl(EPDC_HIST8_PARAM1));
	printk(/*KERN_DEBUG*/ "EPDC_HIST16_PARAM0 0x%x\n", __raw_readl(EPDC_HIST16_PARAM0));
	printk(/*KERN_DEBUG*/ "EPDC_HIST16_PARAM1 0x%x\n", __raw_readl(EPDC_HIST16_PARAM1));
	printk(/*KERN_DEBUG*/ "EPDC_HIST16_PARAM2 0x%x\n", __raw_readl(EPDC_HIST16_PARAM2));
	printk(/*KERN_DEBUG*/ "EPDC_HIST16_PARAM3 0x%x\n", __raw_readl(EPDC_HIST16_PARAM3));
	printk(/*KERN_DEBUG*/ "EPDC_GPIO 0x%x\n", __raw_readl(EPDC_GPIO));
	printk(/*KERN_DEBUG*/ "EPDC_VERSION 0x%x\n", __raw_readl(EPDC_VERSION));
	printk(/*KERN_DEBUG*/ "\n\n");
}

static void dump_update_data(struct device *dev,
			     struct update_data_list *upd_data_list)
{

	dev_info(dev,
		"X = %d, Y = %d, Width = %d, Height = %d, WaveMode = %d, "
		"LUT = %d, Coll Mask = 0x%llx, order = %d\n",
		upd_data_list->update_desc->upd_data.update_region.left,
		upd_data_list->update_desc->upd_data.update_region.top,
		upd_data_list->update_desc->upd_data.update_region.width,
		upd_data_list->update_desc->upd_data.update_region.height,
		upd_data_list->update_desc->upd_data.waveform_mode,
		upd_data_list->lut_num,
		upd_data_list->collision_mask,
		upd_data_list->update_desc->update_order);
}

static void dump_collision_list(struct mxc_epdc_fb_data *fb_data)
{

	struct update_data_list *plist;

	dev_info(fb_data->dev, "Collision List:\n");
	if (list_empty(&fb_data->upd_buf_collision_list))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_collision_list, list) {
		dev_info(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
		dump_update_data(fb_data->dev, plist);
	}
}

static void dump_free_list(struct mxc_epdc_fb_data *fb_data)
{

	struct update_data_list *plist;

	dev_info(fb_data->dev, "Free List:\n");
	if (list_empty(&fb_data->upd_buf_free_list))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_free_list, list)
		dev_info(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
}

static void dump_queue(struct mxc_epdc_fb_data *fb_data)
{

	struct update_data_list *plist;

	dev_info(fb_data->dev, "Queue:\n");
	if (list_empty(&fb_data->upd_buf_queue))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_buf_queue, list) {
		dev_info(fb_data->dev, "Virt Addr = 0x%x, Phys Addr = 0x%x ",
			(u32)plist->virt_addr, plist->phys_addr);
		dump_update_data(fb_data->dev, plist);
	}
}

static void dump_desc_data(struct device *dev,
			     struct update_desc_list *upd_desc_list)
{

	dev_info(dev,
		"X = %d, Y = %d, Width = %d, Height = %d, WaveMode = %d, "
		"order = %d\n",
		upd_desc_list->upd_data.update_region.left,
		upd_desc_list->upd_data.update_region.top,
		upd_desc_list->upd_data.update_region.width,
		upd_desc_list->upd_data.update_region.height,
		upd_desc_list->upd_data.waveform_mode,
		upd_desc_list->update_order);
}

static void dump_pending_list(struct mxc_epdc_fb_data *fb_data)
{

	struct update_desc_list *plist;

	dev_info(fb_data->dev, "Queue:\n");
	if (list_empty(&fb_data->upd_pending_list))
		dev_info(fb_data->dev, "Empty");
	list_for_each_entry(plist, &fb_data->upd_pending_list, list)
		dump_desc_data(fb_data->dev, plist);
}

static void dump_all_updates(struct mxc_epdc_fb_data *fb_data)
{

	dump_free_list(fb_data);
	dump_queue(fb_data);
	dump_collision_list(fb_data);
	dev_info(fb_data->dev, "Current update being processed:\n");
	if (fb_data->cur_update == NULL)
		dev_info(fb_data->dev, "No current update\n");
	else
		dump_update_data(fb_data->dev, fb_data->cur_update);
}

#else
static inline void dump_pxp_config(struct mxc_epdc_fb_data *fb_data,
				   struct pxp_config_data *pxp_conf) {}
static inline void dump_epdc_reg(void) {}
static inline void dump_update_data(struct device *dev,
			     struct update_data_list *upd_data_list) {}
static inline void dump_collision_list(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_free_list(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_queue(struct mxc_epdc_fb_data *fb_data) {}
static inline void dump_all_updates(struct mxc_epdc_fb_data *fb_data) {}

#endif

struct file *file_open(const char *path, int flags, int rights) 
{
    struct file *filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if (IS_ERR(filp)) {
        err = PTR_ERR(filp);
        printk(KERN_ERR "Cannot open file %s: %d\n", path, err);
        return NULL;
    }
    return filp;
}

void file_close(struct file *file) 
{
    filp_close(file, NULL);
}

int file_write(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size) 
{
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_write(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

int file_sync(struct file *file) 
{
    vfs_fsync(file, 0);
    return 0;
}

/********************************************************
 * Start Low-Level EPDC Functions
 ********************************************************/
int get_mtd_channel(const char name[64]){
	int i = 0, channel = -1;

	while(name[i]){
		if(name[i] <= '2' && name[i] >= '0'){
			channel = name[i] - '0';
		}
		i++;
	}
	//printk("MTD %s has channel %i\n" ,name, channel);
	return channel;
 }

int check_vcom_valid(int vcom_mv){
	if(vcom_mv > -2000 && vcom_mv < 15000)
		return 1;
	return 0;	
}

 
int check_mtd_flash(struct mxc_epdc_fb_data *fb_data){
	struct nvm_content* nvm = &fb_data->nvm;
	int result = 0;
	struct mtd_info* mtd_info;
	int num, i = 0, wf_read = 0;
	char vcom_str[15] = {0,};
	char wflen_str[15] = {0,};
	int vcom = 0;
	int wflen = 0;
	int partno_valid = 0, wf_name_valid = 0,vcom_valid = 0, wflen_valid = 0;
	size_t len;
	
    for(num = 0; num < 16; num++) {
		int channel, ret;
        mtd_info = get_mtd_device(NULL, num);
        
        
        
        if(IS_ERR(mtd_info)) {
            //printk("No device for num %d\n", num);
            continue;
        }
        if(mtd_info->type == MTD_ABSENT) {
            put_mtd_device(mtd_info);
            continue;
        }
        
        v3p3_powerdown(fb_data);
		channel = get_mtd_channel(mtd_info->name);
        
		ret = regulator_enable(fb_data->v3p3_regulator[channel]);
		if (IS_ERR((void *)ret)) {
			dev_err(fb_data->dev, "Unable to enable V3P3_1 regulator."
				"err = 0x%x\n", ret);
			return;
		}

        if( !strncmp(mtd_info->name, "part", 4) || 
			!strncmp(mtd_info->name, "vcom", 4) || 
			!strncmp(mtd_info->name, "wfle", 4) || 
			!strncmp(mtd_info->name, "wfna", 4) || 
			!strncmp(mtd_info->name, "wave", 4)) {
            
            dev_dbg(fb_data->dev, "MTD name: %s\n", mtd_info->name);
            dev_dbg(fb_data->dev, "MTD type: %i\n", mtd_info->type);
			dev_dbg(fb_data->dev, "MTD total size : %lu bytes\n", mtd_info->size);
			dev_dbg(fb_data->dev, "MTD erase size : %u bytes\n", mtd_info->erasesize);

			if(mtd_info->size > 0 && mtd_info->erasesize > 0){
				if(!strncmp(mtd_info->name, "wfna", 4)){
					if(!wf_name_valid){
						mtd_read(mtd_info, 0x0, sizeof(nvm->wf_name), &len, nvm->wf_name);
						wf_name_valid = sizeof(nvm->wf_name);
						
						for(i=0;i<sizeof(nvm->wf_name);i++){
							if((nvm->wf_name[i] < '.' || nvm->wf_name[i] > 'z') && nvm->wf_name[i] != 0){
								wf_name_valid--;
							}
						}
						
						if(wf_name_valid != sizeof(nvm->wf_name) || nvm->wf_name[0] == 0){
							dev_err(fb_data->dev, "Invalid Waveform Name %s\n", nvm->wf_name);
							wf_name_valid = 0;
						}else{
							dev_info(fb_data->dev, "We read Waveform Name: %s\n", nvm->wf_name);
							wf_name_valid = NVM_WF_NAME_VALID;
						}
					}
				}
				if(!strncmp(mtd_info->name, "wfle", 4)){
					mtd_read(mtd_info, 0x0, sizeof(wflen), &len, wflen_str);
					wflen = wflen_str[3] + (wflen_str[2] << 8) + (wflen_str[1] << 16) + (wflen_str[0] << 24); 
					
						if(!strncmp(mtd_info->name, "wflength0", 9)){
						nvm->wflen[0] = wflen;
					}else if(!strncmp(mtd_info->name, "wflength1", 9)){
						nvm->wflen[1] = wflen;
					}
					if(wflen > 0){
						dev_info(fb_data->dev, "We read Waveform Length: %i\n", wflen);
						wflen_valid = NVM_WFLEN_VALID;
					}

				}
				if(!strncmp(mtd_info->name, "part", 4)){
					//if(!partno_valid){
						mtd_read(mtd_info, 0x0, sizeof(nvm->paneltype), &len, nvm->paneltype);
						partno_valid = sizeof(nvm->paneltype);
						
						for(i=0;i<sizeof(nvm->paneltype);i++){
							if((nvm->paneltype[i] < '.' || nvm->paneltype[i] > 'z') && nvm->paneltype[i] != 0){
								partno_valid--;
							}
						}
						if((partno_valid != sizeof(nvm->paneltype)) || nvm->paneltype[0] == 0){
							dev_err(fb_data->dev, "Invalid Panel Type %s\n", nvm->paneltype);
							partno_valid = 0;
						}else{
							partno_valid = NVM_PANELTYPE_VALID;
							dev_info(fb_data->dev, "We read Panel: %s\n", nvm->paneltype);
						}
					//}
				}
				if(!strncmp(mtd_info->name, "vcom", 4)){
					
					mtd_read(mtd_info, 0x0, sizeof(vcom), &len, vcom_str);
					vcom = vcom_str[3] + (vcom_str[2] << 8) + (vcom_str[1] << 16) + (vcom_str[0] << 24); 
					
					if(vcom < 15000 && vcom > 3000){
						if(!strncmp(mtd_info->name, "vcom0", 5)){
							nvm->vcom[0] = vcom;
							dev_info(fb_data->dev, "We read VCOM0: %i\n", vcom);
						}else if(!strncmp(mtd_info->name, "vcom1", 5)){
							nvm->vcom[1] = vcom;
							dev_info(fb_data->dev, "We read VCOM1: %i\n", vcom);
						}
						
						vcom_valid = NVM_VCOM_VALID;
					}
				}
				
				if(!strncmp(mtd_info->name, "wave", 4)){
					dev_dbg(fb_data->dev, "We read %s (%i)\n", mtd_info->name, wf_read);
					if(!wf_read){
						int chunkSize = 16;
						uint32_t byte_offset = 0;
						int bytes_to_transfer = 0x3AFFF;
						uint8_t *data;
						int data_read = 0;
						/*
						struct file* waveform;
						char* wf_path;
						*/
						while(bytes_to_transfer > 0){
							// transfer chunkSize or bytes to transfer
							size_t transferChunkSize = (bytes_to_transfer >= chunkSize) ? chunkSize : bytes_to_transfer;
							data = &(nvm->waveform[byte_offset]);
							mtd_read(mtd_info, byte_offset, transferChunkSize, &len, data);
							for(i=0; i<len; i++){
								if((data[i] != 0) && (data[i] != 0xFF)){
									wf_read=NVM_WAVEFORM_VALID; // do not overwrite a waveform we already read
								}
							}
							byte_offset += transferChunkSize;
							bytes_to_transfer -= transferChunkSize;
							data_read += len;
						}
						if(!strncmp(mtd_info->name, "waveform0", 9)){
							nvm->wf_num = 0;
						}else if(!strncmp(mtd_info->name, "waveform1", 9)){
							nvm->wf_num = 1;
						}
						dev_info(fb_data->dev, "We read %i Bytes WF data (%i)\n", data_read, nvm->wf_num);
					}
				}
				//*/
			}
			//*/
			put_mtd_device(mtd_info);
            continue;
       }
       //put_mtd_device(mtd_info);
        //return 0;
    }
    mtd_info = NULL;
    v3p3_powerdown(fb_data);
    result = partno_valid + wf_name_valid  + vcom_valid + wf_read + wflen_valid;
    dev_dbg(fb_data->dev, "%s: Result: %x\n", __func__, result);
    return result;
} 
 
int mxc_epdc_fb_set_vcom_voltage(struct regulator* reg, int millivolts){

	int microvolts = 0,reg_voltage = -1;
	int timeout = 5;
	if(millivolts < -10000 || millivolts > 15000){
		printk( "Recalculate VCOM voltage: Unsupported VCOM Value %i\n", millivolts);
		millivolts = 5000;
	}
	if(millivolts < 0)
		microvolts = millivolts * 1000;
	else
		// We have a plasticlogic display which needs positive vcom voltage
		microvolts =  (((-36411 * millivolts) / 100 ) + 210485);
	
	reg_voltage = regulator_get_voltage(reg);
	//printk("Current VCOM: %i uV, Desired VCOM: %i\n", reg_voltage, microvolts);
	while(((microvolts - reg_voltage < -10000) || (microvolts - reg_voltage > 10000)) && timeout){
		if(!regulator_is_enabled(reg)){
			return -1;
		}
		//printk("Recalculate VCOM voltage: %i mV -> %i uV\n", millivolts, microvolts);
		regulator_set_voltage(reg, microvolts, microvolts);
		//printk("Set VCOM to: %i uV\n", microvolts);
		if(timeout > 0) timeout--;
	};

	return 0;
}

static inline void epdc_lut_complete_intr(int rev, u32 lut_num, bool enable)
{

	if (rev < 20) {
		if (enable)
			__raw_writel(1 << lut_num, EPDC_IRQ_MASK_SET);
		else
			__raw_writel(1 << lut_num, EPDC_IRQ_MASK_CLEAR);
	} else {
		if (enable) {
			if (lut_num < 32)
				__raw_writel(1 << lut_num, EPDC_IRQ_MASK1_SET);
			else
				__raw_writel(1 << (lut_num - 32),
					EPDC_IRQ_MASK2_SET);
		} else {
			if (lut_num < 32)
				__raw_writel(1 << lut_num,
					EPDC_IRQ_MASK1_CLEAR);
			else
				__raw_writel(1 << (lut_num - 32),
					EPDC_IRQ_MASK2_CLEAR);
		}
	}
}

static inline void epdc_working_buf_intr(bool enable)
{

	if (enable)
		__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ, EPDC_IRQ_MASK_SET);
	else
		__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ, EPDC_IRQ_MASK_CLEAR);
}

static inline void epdc_clear_working_buf_irq(void)
{

	__raw_writel(EPDC_IRQ_WB_CMPLT_IRQ | EPDC_IRQ_LUT_COL_IRQ,
		     EPDC_IRQ_CLEAR);
}

static inline void epdc_eof_intr(bool enable)
{

	if (enable)
		__raw_writel(EPDC_IRQ_FRAME_END_IRQ, EPDC_IRQ_MASK_SET);
	else
		__raw_writel(EPDC_IRQ_FRAME_END_IRQ, EPDC_IRQ_MASK_CLEAR);
}

static inline void epdc_clear_eof_irq(void)
{

	__raw_writel(EPDC_IRQ_FRAME_END_IRQ, EPDC_IRQ_CLEAR);
}

static inline bool epdc_signal_eof(void)
{

	return (__raw_readl(EPDC_IRQ_MASK) & __raw_readl(EPDC_IRQ)
		& EPDC_IRQ_FRAME_END_IRQ) ? true : false;
}

static inline void epdc_set_temp(u32 temp)
{
	__raw_writel(temp, EPDC_TEMP);
}

static inline void epdc_set_screen_res(u32 width, u32 height)
{

	u32 val = (height << EPDC_RES_VERTICAL_OFFSET) | width;
	__raw_writel(val, EPDC_RES);
}

static inline void epdc_set_update_addr(u32 addr)
{

	__raw_writel(addr, EPDC_UPD_ADDR);
}

static inline void epdc_set_update_coord(u32 x, u32 y)
{

	u32 val = (y << EPDC_UPD_CORD_YCORD_OFFSET) | x;
	__raw_writel(val, EPDC_UPD_CORD);
}

static inline void epdc_set_update_dimensions(u32 width, u32 height)
{

	u32 val = (height << EPDC_UPD_SIZE_HEIGHT_OFFSET) | width;
	__raw_writel(val, EPDC_UPD_SIZE);
}

static void epdc_set_update_waveform(struct mxcfb_waveform_modes *wv_modes)
{

	u32 val;
	/* Configure the auto-waveform look-up table based on waveform modes */

	/* Entry 1 = DU, 2 = GC4, 3 = GC8, etc. */
	val = (wv_modes->mode_du << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(0 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
	val = (wv_modes->mode_du << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(1 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
	val = (wv_modes->mode_gc4 << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(2 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
	val = (wv_modes->mode_gc8 << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(3 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
	val = (wv_modes->mode_gc16 << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(4 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
	val = (wv_modes->mode_gc32 << EPDC_AUTOWV_LUT_DATA_OFFSET) |
		(5 << EPDC_AUTOWV_LUT_ADDR_OFFSET);
	__raw_writel(val, EPDC_AUTOWV_LUT);
}

static void epdc_set_update_stride(u32 stride)
{

	__raw_writel(stride, EPDC_UPD_STRIDE);
}

static void epdc_submit_update(u32 lut_num, u32 waveform_mode, u32 update_mode,
			       bool use_dry_run, bool use_test_mode, u32 np_val)
{

	u32 reg_val = 0;
	if (use_test_mode) {
		reg_val |=
		    ((np_val << EPDC_UPD_FIXED_FIXNP_OFFSET) &
		     EPDC_UPD_FIXED_FIXNP_MASK) | EPDC_UPD_FIXED_FIXNP_EN;

		__raw_writel(reg_val, EPDC_UPD_FIXED);

		reg_val = EPDC_UPD_CTRL_USE_FIXED;
	} else {
		__raw_writel(reg_val, EPDC_UPD_FIXED);
	}

	if (waveform_mode == WAVEFORM_MODE_AUTO)
		reg_val |= EPDC_UPD_CTRL_AUTOWV;
	else
		reg_val |= ((waveform_mode <<
				EPDC_UPD_CTRL_WAVEFORM_MODE_OFFSET) &
				EPDC_UPD_CTRL_WAVEFORM_MODE_MASK);

	reg_val |= (use_dry_run ? EPDC_UPD_CTRL_DRY_RUN : 0) |
	    ((lut_num << EPDC_UPD_CTRL_LUT_SEL_OFFSET) &
	     EPDC_UPD_CTRL_LUT_SEL_MASK) |
	    update_mode;

	__raw_writel(reg_val, EPDC_UPD_CTRL);
}

static inline bool epdc_is_lut_complete(int rev, u32 lut_num)
{

	u32 val;
	bool is_compl;
	if (rev < 20) {
		val = __raw_readl(EPDC_IRQ);
		is_compl = val & (1 << lut_num) ? true : false;
	} else if (lut_num < 32) {
		val = __raw_readl(EPDC_IRQ1);
		is_compl = val & (1 << lut_num) ? true : false;
	} else {
		val = __raw_readl(EPDC_IRQ2);
		is_compl = val & (1 << (lut_num - 32)) ? true : false;
	}

	return is_compl;
}

static inline void epdc_clear_lut_complete_irq(int rev, u32 lut_num)
{

	if (rev < 20)
		__raw_writel(1 << lut_num, EPDC_IRQ_CLEAR);
	else if (lut_num < 32)
		__raw_writel(1 << lut_num, EPDC_IRQ1_CLEAR);
	else
		__raw_writel(1 << (lut_num - 32), EPDC_IRQ2_CLEAR);
}

static inline bool epdc_is_lut_active(u32 lut_num)
{

	u32 val;
	bool is_active;

	if (lut_num < 32) {
		val = __raw_readl(EPDC_STATUS_LUTS);
		is_active = val & (1 << lut_num) ? true : false;
	} else {
		val = __raw_readl(EPDC_STATUS_LUTS2);
		is_active = val & (1 << (lut_num - 32)) ? true : false;
	}

	return is_active;
}

static inline bool epdc_any_luts_active(int rev)
{

	bool any_active;

	if (rev < 20)
		any_active = __raw_readl(EPDC_STATUS_LUTS) ? true : false;
	else
		any_active = (__raw_readl(EPDC_STATUS_LUTS) |
			__raw_readl(EPDC_STATUS_LUTS2))	? true : false;

	return any_active;
}

static inline bool epdc_any_luts_available(void)
{

	bool luts_available =
	    (__raw_readl(EPDC_STATUS_NEXTLUT) &
	     EPDC_STATUS_NEXTLUT_NEXT_LUT_VALID) ? true : false;
	return luts_available;
}

static inline int epdc_get_next_lut(void)
{

	u32 val =
	    __raw_readl(EPDC_STATUS_NEXTLUT) &
	    EPDC_STATUS_NEXTLUT_NEXT_LUT_MASK;
	return val;
}

static int epdc_choose_next_lut(int rev, int *next_lut)
{

	u64 luts_status, unprocessed_luts, used_luts;
	/* Available LUTs are reduced to 16 in 5-bit waveform mode */
	bool format_p5n = ((__raw_readl(EPDC_FORMAT) &
	EPDC_FORMAT_BUF_PIXEL_FORMAT_MASK) ==
	EPDC_FORMAT_BUF_PIXEL_FORMAT_P5N);

	luts_status = __raw_readl(EPDC_STATUS_LUTS);
	if ((rev < 20) || format_p5n)
		luts_status &= 0xFFFF;
	else
		luts_status |= ((u64)__raw_readl(EPDC_STATUS_LUTS2) << 32);

	if (rev < 20) {
		unprocessed_luts = __raw_readl(EPDC_IRQ) & 0xFFFF;
	} else {
		unprocessed_luts = __raw_readl(EPDC_IRQ1) |
			((u64)__raw_readl(EPDC_IRQ2) << 32);
		if (format_p5n)
			unprocessed_luts &= 0xFFFF;
	}

	/*
	 * Note on unprocessed_luts: There is a race condition
	 * where a LUT completes, but has not been processed by
	 * IRQ handler workqueue, and then a new update request
	 * attempts to use that LUT.  We prevent that here by
	 * ensuring that the LUT we choose doesn't have its IRQ
	 * bit set (indicating it has completed but not yet been
	 * processed).
	 */
	used_luts = luts_status | unprocessed_luts;

	/*
	 * Selecting a LUT to minimize incidence of TCE Underrun Error
	 * --------------------------------------------------------
	 * We want to find the lowest order LUT that is of greater
	 * order than all other active LUTs.  If highest order LUT
	 * is active, then we want to choose the lowest order
	 * available LUT.
	 *
	 * NOTE: For EPDC version 2.0 and later, TCE Underrun error
	 *       bug is fixed, so it doesn't matter which LUT is used.
	 */

	if ((rev < 20) || format_p5n) {
		*next_lut = fls64(used_luts);
		if (*next_lut > 15)
			*next_lut = ffz(used_luts);
	} else {
		if ((u32)used_luts != ~0UL)
			*next_lut = ffz((u32)used_luts);
		else if ((u32)(used_luts >> 32) != ~0UL)
			*next_lut = ffz((u32)(used_luts >> 32)) + 32;
		else
			*next_lut = INVALID_LUT;
	}

	if (used_luts & 0x8000)
		return 1;
	else
		return 0;
}

static inline bool epdc_is_working_buffer_busy(void)
{

	u32 val = __raw_readl(EPDC_STATUS);
	bool is_busy = (val & EPDC_STATUS_WB_BUSY) ? true : false;

	return is_busy;
}

static inline bool epdc_is_working_buffer_complete(void)
{

	u32 val = __raw_readl(EPDC_IRQ);
	bool is_compl = (val & EPDC_IRQ_WB_CMPLT_IRQ) ? true : false;

	return is_compl;
}

static inline bool epdc_is_lut_cancelled(void)
{

	u32 val = __raw_readl(EPDC_STATUS);
	bool is_void = (val & EPDC_STATUS_UPD_VOID) ? true : false;

	return is_void;
}

static inline bool epdc_is_collision(void)
{

	u32 val = __raw_readl(EPDC_IRQ);
	return (val & EPDC_IRQ_LUT_COL_IRQ) ? true : false;
}

static inline u64 epdc_get_colliding_luts(int rev)
{

	u32 val = __raw_readl(EPDC_STATUS_COL);
	if (rev >= 20)
		val |= (u64)__raw_readl(EPDC_STATUS_COL2) << 32;
	return val;
}

static void epdc_set_horizontal_timing(u32 horiz_start, u32 horiz_end,
				       u32 hsync_width, u32 hsync_line_length)
{

	u32 reg_val =
	    ((hsync_width << EPDC_TCE_HSCAN1_LINE_SYNC_WIDTH_OFFSET) &
	     EPDC_TCE_HSCAN1_LINE_SYNC_WIDTH_MASK)
	    | ((hsync_line_length << EPDC_TCE_HSCAN1_LINE_SYNC_OFFSET) &
	       EPDC_TCE_HSCAN1_LINE_SYNC_MASK);
	__raw_writel(reg_val, EPDC_TCE_HSCAN1);

	reg_val =
	    ((horiz_start << EPDC_TCE_HSCAN2_LINE_BEGIN_OFFSET) &
	     EPDC_TCE_HSCAN2_LINE_BEGIN_MASK)
	    | ((horiz_end << EPDC_TCE_HSCAN2_LINE_END_OFFSET) &
	       EPDC_TCE_HSCAN2_LINE_END_MASK);
	__raw_writel(reg_val, EPDC_TCE_HSCAN2);
}

static void epdc_set_vertical_timing(u32 vert_start, u32 vert_end,
				     u32 vsync_width)
{

	u32 reg_val =
	    ((vert_start << EPDC_TCE_VSCAN_FRAME_BEGIN_OFFSET) &
	     EPDC_TCE_VSCAN_FRAME_BEGIN_MASK)
	    | ((vert_end << EPDC_TCE_VSCAN_FRAME_END_OFFSET) &
	       EPDC_TCE_VSCAN_FRAME_END_MASK)
	    | ((vsync_width << EPDC_TCE_VSCAN_FRAME_SYNC_OFFSET) &
	       EPDC_TCE_VSCAN_FRAME_SYNC_MASK);
	__raw_writel(reg_val, EPDC_TCE_VSCAN);
}

static void epdc_init_settings(struct mxc_epdc_fb_data *fb_data)
{

	const struct imx_epdc_fb_mode *epdc_mode = fb_data->cur_mode;
	struct fb_var_screeninfo *screeninfo = &fb_data->epdc_fb_var;
	u32 reg_val;
	int num_ce;
	int i;
	dev_dbg(fb_data->dev, "Enable clocks to access EPDC regs\n");
	/* Enable clocks to access EPDC regs */
	clk_prepare_enable(fb_data->epdc_clk_axi);
	clk_prepare_enable(fb_data->epdc_clk_pix);
	dev_dbg(fb_data->dev, "Reset\n");
	/* Reset */
	__raw_writel(EPDC_CTRL_SFTRST, EPDC_CTRL_SET);
	while (!(__raw_readl(EPDC_CTRL) & EPDC_CTRL_CLKGATE))
		;
	__raw_writel(EPDC_CTRL_SFTRST, EPDC_CTRL_CLEAR);
	dev_dbg(fb_data->dev, "Enable clock gating (clear to enable)\n");
	/* Enable clock gating (clear to enable) */
	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_CLEAR);
	while (__raw_readl(EPDC_CTRL) & (EPDC_CTRL_SFTRST | EPDC_CTRL_CLKGATE))
		;
	dev_dbg(fb_data->dev, "EPDC_CTRL\n");
	/* EPDC_CTRL */
	reg_val = __raw_readl(EPDC_CTRL);
	reg_val &= ~EPDC_CTRL_UPD_DATA_SWIZZLE_MASK;
	reg_val |= EPDC_CTRL_UPD_DATA_SWIZZLE_NO_SWAP;
	reg_val &= ~EPDC_CTRL_LUT_DATA_SWIZZLE_MASK;
	reg_val |= EPDC_CTRL_LUT_DATA_SWIZZLE_NO_SWAP;
	__raw_writel(reg_val, EPDC_CTRL_SET);
	dev_dbg(fb_data->dev, "EPDC_FORMAT - 4bit Buf pixel format\n");
	/* EPDC_FORMAT - 4bit Buf pixel format */
	dev_dbg(fb_data->dev, "%s %i\n",__func__, fb_data->buf_pix_fmt); 
	reg_val = fb_data->buf_pix_fmt
	    | ((0x0 << EPDC_FORMAT_DEFAULT_TFT_PIXEL_OFFSET) &
	       EPDC_FORMAT_DEFAULT_TFT_PIXEL_MASK);
	if (epdc_mode->tft_4bpp)
		reg_val |= EPDC_FORMAT_TFT_PIXEL_FORMAT_4BIT;
	else
		reg_val |= EPDC_FORMAT_TFT_PIXEL_FORMAT_2BIT;
	__raw_writel(reg_val, EPDC_FORMAT);
	dev_dbg(fb_data->dev, "EPDC_FIFOCTRL (disabled)\n");
	/* EPDC_FIFOCTRL (disabled) */
	reg_val =
	    ((100 << EPDC_FIFOCTRL_FIFO_INIT_LEVEL_OFFSET) &
	     EPDC_FIFOCTRL_FIFO_INIT_LEVEL_MASK)
	    | ((200 << EPDC_FIFOCTRL_FIFO_H_LEVEL_OFFSET) &
	       EPDC_FIFOCTRL_FIFO_H_LEVEL_MASK)
	    | ((100 << EPDC_FIFOCTRL_FIFO_L_LEVEL_OFFSET) &
	       EPDC_FIFOCTRL_FIFO_L_LEVEL_MASK);
	__raw_writel(reg_val, EPDC_FIFOCTRL);
	dev_dbg(fb_data->dev, "EPDC_TEMP - Use default temp to get index\n");
	/* EPDC_TEMP - Use default temp to get index */
	epdc_set_temp(mxc_epdc_fb_get_temp_index(fb_data, DEFAULT_TEMP));
	dev_dbg(fb_data->dev, "EPDC_RES\n");
	/* EPDC_RES */
	epdc_set_screen_res(fb_data->epdc_xres, epdc_mode->vmode->yres);
	dev_dbg(fb_data->dev, "Initialize all auto-wavefrom look-up values to 2 - GC16\n");
	/* EPDC_AUTOWV_LUT */
	/* Initialize all auto-wavefrom look-up values to 2 - GC16 */
	for (i = 0; i < 8; i++)
		__raw_writel((2 << EPDC_AUTOWV_LUT_DATA_OFFSET) |
			(i << EPDC_AUTOWV_LUT_ADDR_OFFSET), EPDC_AUTOWV_LUT);

	/*
	 * EPDC_TCE_CTRL
	 * VSCAN_HOLDOFF = 4
	 * VCOM_MODE = MANUAL
	 * VCOM_VAL = 0
	 * DDR_MODE = DISABLED
	 * LVDS_MODE_CE = DISABLED
	 * LVDS_MODE = DISABLED
	 * PIXELS_PER_SDCLK = 4
	 */
	reg_val =
	    ((epdc_mode->vscan_holdoff << EPDC_TCE_CTRL_VSCAN_HOLDOFF_OFFSET) &
	     EPDC_TCE_CTRL_VSCAN_HOLDOFF_MASK)
	    | EPDC_TCE_CTRL_PIXELS_PER_SDCLK_4;
	if (epdc_mode->sddo_16_bits)
		reg_val |= EPDC_TCE_CTRL_SDDO_WIDTH_16BIT;
	if (epdc_mode->dual_scan)
		reg_val |= EPDC_TCE_CTRL_DUAL_SCAN_ENABLE;
	if (epdc_mode->scan_dir_0_up)
		reg_val |= EPDC_TCE_CTRL_SCAN_DIR_0_UP;
	if (epdc_mode->scan_dir_1_up)
		reg_val |= EPDC_TCE_CTRL_SCAN_DIR_1_UP;
	__raw_writel(reg_val, EPDC_TCE_CTRL);
	dev_dbg(fb_data->dev, "EPDC_TCE_HSCAN\n");
	/* EPDC_TCE_HSCAN */
	epdc_set_horizontal_timing(screeninfo->left_margin,
				   screeninfo->right_margin,
				   screeninfo->hsync_len,
				   screeninfo->hsync_len);
	dev_dbg(fb_data->dev, "EPDC_TCE_VSCAN\n");
	/* EPDC_TCE_VSCAN */
	epdc_set_vertical_timing(screeninfo->upper_margin,
				 screeninfo->lower_margin,
				 screeninfo->vsync_len);
	dev_dbg(fb_data->dev, "EPDC_TCE_OE\n");
	/* EPDC_TCE_OE */
	reg_val =
	    ((epdc_mode->sdoed_width << EPDC_TCE_OE_SDOED_WIDTH_OFFSET) &
	     EPDC_TCE_OE_SDOED_WIDTH_MASK)
	    | ((epdc_mode->sdoed_delay << EPDC_TCE_OE_SDOED_DLY_OFFSET) &
	       EPDC_TCE_OE_SDOED_DLY_MASK)
	    | ((epdc_mode->sdoez_width << EPDC_TCE_OE_SDOEZ_WIDTH_OFFSET) &
	       EPDC_TCE_OE_SDOEZ_WIDTH_MASK)
	    | ((epdc_mode->sdoez_delay << EPDC_TCE_OE_SDOEZ_DLY_OFFSET) &
	       EPDC_TCE_OE_SDOEZ_DLY_MASK);
	__raw_writel(reg_val, EPDC_TCE_OE);
	dev_dbg(fb_data->dev, "EPDC_TCE_TIMING1\n");
	/* EPDC_TCE_TIMING1 */
	__raw_writel(0x0, EPDC_TCE_TIMING1);
	dev_dbg(fb_data->dev, "EPDC_TCE_TIMING2\n");
	/* EPDC_TCE_TIMING2 */
	reg_val =
	    ((epdc_mode->gdclk_hp_offs << EPDC_TCE_TIMING2_GDCLK_HP_OFFSET) &
	     EPDC_TCE_TIMING2_GDCLK_HP_MASK)
	    | ((epdc_mode->gdsp_offs << EPDC_TCE_TIMING2_GDSP_OFFSET_OFFSET) &
	       EPDC_TCE_TIMING2_GDSP_OFFSET_MASK);
	__raw_writel(reg_val, EPDC_TCE_TIMING2);
	dev_dbg(fb_data->dev, "EPDC_TCE_TIMING3\n");
	/* EPDC_TCE_TIMING3 */
	reg_val =
	    ((epdc_mode->gdoe_offs << EPDC_TCE_TIMING3_GDOE_OFFSET_OFFSET) &
	     EPDC_TCE_TIMING3_GDOE_OFFSET_MASK)
	    | ((epdc_mode->gdclk_offs << EPDC_TCE_TIMING3_GDCLK_OFFSET_OFFSET) &
	       EPDC_TCE_TIMING3_GDCLK_OFFSET_MASK);
	__raw_writel(reg_val, EPDC_TCE_TIMING3);

	/*
	 * EPDC_TCE_SDCFG
	 * SDCLK_HOLD = epdc_mode->sdclk_hold
	 * SDSHR = epdc_mode->sddo_shift_right
	 * NUM_CE = epdc_mode->num_ce
	 * SDDO_REFORMAT = epdc_mode->sddo_flip_bits
	 * SDDO_INVERT = epdc_mode->sddo_invert
	 * PIXELS_PER_CE = display horizontal resolution
	 */
	num_ce = epdc_mode->num_ce;
	if (num_ce == 0)
		num_ce = 1;
	reg_val = 0;	
	if(epdc_mode->sddo_shift_right){
		reg_val |= EPDC_TCE_SDCFG_SDSHR;
	}
	if(epdc_mode->sddo_invert){
		reg_val |= EPDC_TCE_SDCFG_SDDO_INVERT_ENABLE;
	}
	reg_val |= ((num_ce << EPDC_TCE_SDCFG_NUM_CE_OFFSET) &
	       EPDC_TCE_SDCFG_NUM_CE_MASK)
	    | ((fb_data->epdc_xres/num_ce << EPDC_TCE_SDCFG_PIXELS_PER_CE_OFFSET) &
	       EPDC_TCE_SDCFG_PIXELS_PER_CE_MASK);
	if (epdc_mode->sddo_flip_bits)
		reg_val |= EPDC_TCE_SDCFG_SDDO_REFORMAT_FLIP_PIXELS;
	if (epdc_mode->sdclk_hold)
		reg_val |= EPDC_TCE_SDCFG_SDCLK_HOLD;
	__raw_writel(reg_val, EPDC_TCE_SDCFG);

	/*
	 * EPDC_TCE_GDCFG
	 * GDRL = 1
	 */

	reg_val = EPDC_TCE_SDCFG_GDRL;
	if (epdc_mode->gdsp_frame_sync)
		reg_val |= EPDC_TCE_SDCFG_GDSP_MODE_FRAME_SYNC;
	if (epdc_mode->gdoe_delayed_gclk)
		reg_val |= EPDC_TCE_SDCFG_GDOE_MODE_DELAYED_GDCLK;
	__raw_writel(reg_val, EPDC_TCE_GDCFG);

	/*
	 * EPDC_TCE_POLARITY
	 * SDCE_POL = ACTIVE LOW
	 * SDLE_POL = ACTIVE HIGH
	 * SDOE_POL = ACTIVE HIGH
	 */
	reg_val = EPDC_TCE_POLARITY_SDLE_POL_ACTIVE_HIGH
	    | EPDC_TCE_POLARITY_SDOE_POL_ACTIVE_HIGH;
	if (epdc_mode->gdoe_active_high)
		reg_val |= EPDC_TCE_POLARITY_GDOE_POL_ACTIVE_HIGH;
	if (epdc_mode->gdsp_active_high)
		reg_val |= EPDC_TCE_POLARITY_GDSP_POL_ACTIVE_HIGH;
	__raw_writel(reg_val, EPDC_TCE_POLARITY);

	/* EPDC_IRQ_MASK */
	__raw_writel(EPDC_IRQ_TCE_UNDERRUN_IRQ, EPDC_IRQ_MASK);

	/*
	 * EPDC_GPIO
	 * PWRCOM = ?
	 * PWRCTRL = ?
	 * BDR = ?
	 */
	reg_val = ((0 << EPDC_GPIO_PWRCTRL_OFFSET) & EPDC_GPIO_PWRCTRL_MASK)
	    | ((0 << EPDC_GPIO_BDR_OFFSET) & EPDC_GPIO_BDR_MASK);
	__raw_writel(reg_val, EPDC_GPIO);

	__raw_writel(fb_data->waveform_buffer_phys, EPDC_WVADDR);
	__raw_writel(fb_data->working_buffer_phys, EPDC_WB_ADDR);
	__raw_writel(fb_data->working_buffer_phys, EPDC_WB_ADDR_TCE);
	dev_dbg(fb_data->dev, "Disable clock\n");
	/* Disable clock */
	clk_disable_unprepare(fb_data->epdc_clk_axi);
	clk_disable_unprepare(fb_data->epdc_clk_pix);
}


static void v3p3_powerdown(struct mxc_epdc_fb_data *fb_data){
#if 0	
	return;
#else	
	//dev_info(fb_data->dev, "/* turn off the V3p3 */\n");
	//msleep(10);
#if 0
	int ret = regulator_enable(fb_data->v3p3_regulator[0]);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable V3P3_1 regulator."
			"err = 0x%x\n", ret);
		//mutex_unlock(&fb_data->power_mutex);
		return;
	}

	ret = regulator_enable(fb_data->v3p3_regulator[1]);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable V3P3_2 regulator."
			"err = 0x%x\n", ret);
		//mutex_unlock(&fb_data->power_mutex);
		return;
	}

#else
	if (regulator_is_enabled(fb_data->v3p3_regulator[0]))
		regulator_disable(fb_data->v3p3_regulator[0]);
	if (regulator_is_enabled(fb_data->v3p3_regulator[1]))
		regulator_disable(fb_data->v3p3_regulator[1]);
#endif
	//msleep(10);	
#endif
}

static void v3p3_powerup(struct mxc_epdc_fb_data *fb_data){
	//dev_info(fb_data->dev, "/* turn on the V3p3 */\n");
#if 1
	//msleep(10);
	int ret = regulator_enable(fb_data->v3p3_regulator[0]);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable V3P3_1 regulator."
			"err = 0x%x\n", ret);
		mutex_unlock(&fb_data->power_mutex);
		return;
	}

	ret = regulator_enable(fb_data->v3p3_regulator[1]);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable V3P3_2 regulator."
			"err = 0x%x\n", ret);
		mutex_unlock(&fb_data->power_mutex);
		return;
	}
#else
	//msleep(10);
	if (regulator_is_enabled(fb_data->v3p3_regulator[0]))
		regulator_disable(fb_data->v3p3_regulator[0]);
	if (regulator_is_enabled(fb_data->v3p3_regulator[1]))
		regulator_disable(fb_data->v3p3_regulator[1]);
#endif
	//msleep(10);

}

static void epdc_powerup(struct mxc_epdc_fb_data *fb_data)
{
	int ret = 0;
	mutex_lock(&fb_data->power_mutex);
	
	/*
	 * If power down request is pending, clear
	 * powering_down to cancel the request.
	 */
	if (fb_data->powering_down)
		fb_data->powering_down = false;

	if (fb_data->power_state == POWER_STATE_ON) {
		mutex_unlock(&fb_data->power_mutex);	
		dev_dbg(fb_data->dev, "epdc_powerup:return: power was on - nothing to do");
		return;
	}

	dev_dbg(fb_data->dev, "EPDC Powerup\n");

	fb_data->updates_active = true;
	
	v3p3_powerup(fb_data);

	pm_runtime_get_sync(fb_data->dev);

	/* Enable clocks to EPDC */
	clk_prepare_enable(fb_data->epdc_clk_axi);
	clk_prepare_enable(fb_data->epdc_clk_pix);

	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_CLEAR);

	/* Enable power to the EPD panel */
	ret = regulator_enable(fb_data->display_regulator[0]);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable DISPLAY1 regulator."
			"err = 0x%x\n", ret);
		goto err_exit;
	}
	ret = regulator_enable(fb_data->vcom_regulator[0]);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable VCOM1 regulator."
			"err = 0x%x\n", ret);
		mutex_unlock(&fb_data->power_mutex);
		goto err_exit;
	}
	
	if(mxc_epdc_fb_set_vcom_voltage(fb_data->vcom_regulator[0], fb_data->vcom[0])){
		dev_err(fb_data->dev, "Unable to set VCOM1 Voltage to %i mV\n", fb_data->vcom[0]);
		goto err_exit;
	}		
	
	if(fb_data->cur_mode->dual_scan){
		ret = regulator_enable(fb_data->display_regulator[1]);
		if (IS_ERR((void *)ret)) {
			dev_err(fb_data->dev, "Unable to enable DISPLAY2 regulator."
				"err = 0x%x\n", ret);
			goto err_exit;
		}
		ret = regulator_enable(fb_data->vcom_regulator[1]);
		if (IS_ERR((void *)ret)) {
			dev_err(fb_data->dev, "Unable to enable VCOM2 regulator."
				"err = 0x%x\n", ret);
			mutex_unlock(&fb_data->power_mutex);
			goto err_exit;
		}
		if(mxc_epdc_fb_set_vcom_voltage(fb_data->vcom_regulator[1], fb_data->vcom[1])){
			dev_err(fb_data->dev, "Unable to set VCOM2 Voltage to %i mV\n", fb_data->vcom[1]);
			goto err_exit;
		}
	}

	fb_data->power_state = POWER_STATE_ON;

err_exit:
	mutex_unlock(&fb_data->power_mutex);
}

static void epdc_powerdown(struct mxc_epdc_fb_data *fb_data)
{

	int ret;
	
	if(fb_data->no_powerdown){
		return;
	}
	
	mutex_lock(&fb_data->power_mutex);
	/* If powering_down has been cleared, a powerup
	 * request is pre-empting this powerdown request.
	 */
	if (!fb_data->powering_down
		|| (fb_data->power_state == POWER_STATE_OFF)) {
		mutex_unlock(&fb_data->power_mutex);
		dev_dbg(fb_data->dev, "epdc_powerdown:return: power not on - nothing to do");
		return;
	}

	/* Disable power to the EPD panel */
	ret = regulator_disable(fb_data->vcom_regulator[0]);
	ret |= regulator_disable(fb_data->display_regulator[0]);
	if(fb_data->cur_mode->dual_scan){
		ret |= regulator_disable(fb_data->vcom_regulator[1]);
		ret |= regulator_disable(fb_data->display_regulator[1]);
	}
	
	dev_dbg(fb_data->dev, "EPDC Powerdown %i\n", ret);

	/* Disable clocks to EPDC */
	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_SET);
	clk_disable_unprepare(fb_data->epdc_clk_pix);
	clk_disable_unprepare(fb_data->epdc_clk_axi);

	pm_runtime_put_sync_suspend(fb_data->dev);

	v3p3_powerdown(fb_data);

	fb_data->power_state = POWER_STATE_OFF;
	fb_data->powering_down = false;

	if (fb_data->wait_for_powerdown) {
		fb_data->wait_for_powerdown = false;
		complete(&fb_data->powerdown_compl);
	}

	mutex_unlock(&fb_data->power_mutex);
}

static void epdc_init_sequence(struct mxc_epdc_fb_data *fb_data)
{

	/* Initialize EPDC, passing pointer to EPDC registers */
	epdc_init_settings(fb_data);
	if(mxc_epdc_fb_init_clear_modparam){		
		fb_data->in_init = true;
		epdc_powerup(fb_data);
		draw_mode0(fb_data);
		/* Force power down event */
		fb_data->powering_down = true;
		epdc_powerdown(fb_data);
	}
	fb_data->updates_active = false;
}

static int mxc_epdc_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{

	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset < info->fix.smem_len) {
		/* mapping framebuffer memory */
		len = info->fix.smem_len - offset;
		vma->vm_pgoff = (info->fix.smem_start + offset) >> PAGE_SHIFT;
	} else{
		printk(KERN_ERR "mmap failed\n");
		return -EINVAL;
	}
	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len){
		printk(KERN_ERR "mmap align failed\n");
		return -EINVAL;
	}
	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(info->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

static inline u_int _chan_to_field(u_int chan, struct fb_bitfield *bf)
{

	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int mxc_epdc_fb_setcolreg(u_int regno, u_int red, u_int green,
				 u_int blue, u_int transp, struct fb_info *info)
{

	unsigned int val;
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val = _chan_to_field(red, &info->var.red);
			val |= _chan_to_field(green, &info->var.green);
			val |= _chan_to_field(blue, &info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

static int mxc_epdc_fb_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{

	int count, index, r;
	u16 *red, *green, *blue, *transp;
	u16 trans = 0xffff;
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	int i;
	dev_dbg(fb_data->dev, "setcmap\n");

	if (info->fix.visual == FB_VISUAL_STATIC_PSEUDOCOLOR) {
		/* Only support an 8-bit, 256 entry lookup */
		if (cmap->len != 256)
			return 1;

		mxc_epdc_fb_flush_updates(fb_data);

		mutex_lock(&fb_data->pxp_mutex);
		/*
		 * Store colormap in pxp_conf structure for later transmit
		 * to PxP during update process to convert gray pixels.
		 *
		 * Since red=blue=green for pseudocolor visuals, we can
		 * just use red values.
		 */
		for (i = 0; i < 256; i++)
			fb_data->pxp_conf.proc_data.lut_map[i] = cmap->red[i] & 0xFF;

		fb_data->pxp_conf.proc_data.lut_map_updated = true;

		mutex_unlock(&fb_data->pxp_mutex);
	} else {
		red     = cmap->red;
		green   = cmap->green;
		blue    = cmap->blue;
		transp  = cmap->transp;
		index   = cmap->start;

		for (count = 0; count < cmap->len; count++) {
			if (transp)
				trans = *transp++;
			r = mxc_epdc_fb_setcolreg(index++, *red++, *green++, *blue++,
						trans, info);
			if (r != 0)
				return r;
		}
	}

	return 0;
}

static void adjust_coordinates(u32 xres, u32 yres, u32 rotation, int flipped,
	struct mxcfb_rect *update_region, struct mxcfb_rect *adj_update_region)
{

	const u32 flipped_left = 
		(flipped && (update_region->top < (yres / 2)))
		? xres - update_region->left - update_region->width
		: update_region->left; 
	u32 temp;
	/* If adj_update_region == NULL, pass result back in update_region */
	/* If adj_update_region == valid, use it to pass back result */
	if (adj_update_region)
		switch (rotation) {
		case FB_ROTATE_UR:
			adj_update_region->top = update_region->top;
			adj_update_region->left = flipped_left;
			adj_update_region->width = update_region->width;
			adj_update_region->height = update_region->height;
			break;
		case FB_ROTATE_CW:
			adj_update_region->top = flipped_left;
			adj_update_region->left = yres -
				(update_region->top + update_region->height);
			adj_update_region->width = update_region->height;
			adj_update_region->height = update_region->width;
			break;
		case FB_ROTATE_UD:
			adj_update_region->width = update_region->width;
			adj_update_region->height = update_region->height;
			adj_update_region->top = yres -
				(update_region->top + update_region->height);
			adj_update_region->left = xres -
				(flipped_left + update_region->width);
			break;
		case FB_ROTATE_CCW:
			adj_update_region->left = update_region->top;
			adj_update_region->top = xres -
				(flipped_left + update_region->width);
			adj_update_region->width = update_region->height;
			adj_update_region->height = update_region->width;
			break;
		}
	else
		switch (rotation) {
		case FB_ROTATE_UR:
			update_region->left = flipped_left;
			break;
		case FB_ROTATE_CW:
			temp = update_region->top;
			update_region->top = flipped_left;
			update_region->left = yres -
				(temp + update_region->height);
			temp = update_region->width;
			update_region->width = update_region->height;
			update_region->height = temp;
			break;
		case FB_ROTATE_UD:
			update_region->top = yres -
				(update_region->top + update_region->height);
			update_region->left = xres -
				(flipped_left + update_region->width);
			break;
		case FB_ROTATE_CCW:
			temp = flipped_left;
			update_region->left = update_region->top;
			update_region->top = xres -
				(temp + update_region->width);
			temp = update_region->width;
			update_region->width = update_region->height;
			update_region->height = temp;
			break;
		}
}

/*
 * Set fixed framebuffer parameters based on variable settings.
 *
 * @param       info     framebuffer information pointer
 */
static int mxc_epdc_fb_set_fix(struct fb_info *info)
{

	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	if (var->grayscale)
		fix->visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	else
		fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ypanstep = 1;

	return 0;
}

/*
 * This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the
 * change in par. For this driver it doesn't do much.
 *
 */
static int mxc_epdc_fb_set_par(struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	struct pxp_config_data *pxp_conf = &fb_data->pxp_conf;
	struct pxp_proc_data *proc_data = &pxp_conf->proc_data;
	struct fb_var_screeninfo *screeninfo = &fb_data->info.var;
	struct imx_epdc_fb_mode *epdc_modes = fb_data->pdata->epdc_mode;
	int i;
	int cfa_scale = 1;

	int ret;
	__u32 xoffset_old, yoffset_old;

	/*
	 * Can't change the FB parameters until current updates have completed.
	 * This function returns when all active updates are done.
	 */
	mxc_epdc_fb_flush_updates(fb_data);

	mutex_lock(&fb_data->queue_mutex);
	/*
	 * Set all screeninfo except for xoffset/yoffset
	 * Subsequent call to pan_display will handle those.
	 */
	xoffset_old = fb_data->epdc_fb_var.xoffset;
	yoffset_old = fb_data->epdc_fb_var.yoffset;
	fb_data->epdc_fb_var = *screeninfo;
	fb_data->epdc_fb_var.xoffset = xoffset_old;
	fb_data->epdc_fb_var.yoffset = yoffset_old;
	mutex_unlock(&fb_data->queue_mutex);

	mutex_lock(&fb_data->pxp_mutex);

	/*
	 * Update PxP config data (used to process FB regions for updates)
	 * based on FB info and processing tasks required
	 */

	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = proc_data->srect.width = screeninfo->xres;
	proc_data->drect.height = proc_data->srect.height = screeninfo->yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = screeninfo->rotate;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;

	/*
	 * configure S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	if (screeninfo->grayscale)
		pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_GREY;
	else {
		switch (screeninfo->bits_per_pixel) {
		case 16:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		case 24:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB24;
			break;
		case 32:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB32;
			break;
		default:
			pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
			break;
		}
	}
	pxp_conf->s0_param.width = screeninfo->xres_virtual;
	pxp_conf->s0_param.height = screeninfo->yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = fb_data->epdc_xres;
	pxp_conf->out_param.height = screeninfo->yres;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GREY;

	mutex_unlock(&fb_data->pxp_mutex);

	if (!fb_data->hw_ready && fb_data->hw_initializing)
		wait_for_completion_timeout(&fb_data->init_completion, HZ *10);

	/*
	 * If HW not yet initialized, check to see if we are being sent
	 * an initialization request.
	 */
	if (!fb_data->hw_ready) {
		struct fb_videomode mode;
		u32 xres_temp;

		fb_var_to_videomode(&mode, screeninfo);

		/* When comparing requested fb mode,
		   we need to use unrotated dimensions */
		if ((screeninfo->rotate == FB_ROTATE_CW) ||
			(screeninfo->rotate == FB_ROTATE_CCW)) {
			xres_temp = mode.xres;
			mode.xres = mode.yres;
			mode.yres = xres_temp;
		}

		if (fb_data->use_cfa) {
			cfa_scale = 2;
		}
		
		mode.xres *= cfa_scale ;
		mode.yres *= cfa_scale;
		/*
		* If requested video mode does not match current video
		* mode, search for a matching panel.
		*/
		if (fb_data->cur_mode &&
			!fb_mode_is_equal(fb_data->cur_mode->vmode,
			&mode)) {
			bool found_match = false;

			/* Match videomode against epdc modes */
			for (i = 0; i < fb_data->pdata->num_modes; i++) {
				if (!fb_mode_is_equal(epdc_modes[i].vmode,
					&mode))
					continue;
				fb_data->cur_mode = &epdc_modes[i];
				found_match = true;
				break;
			}

			if (!found_match) {
				dev_err(fb_data->dev,
					"Failed to match requested "
					"video mode\n");
				return -EINVAL;
			}
		}

		/* Found a match - Grab timing params */
		screeninfo->left_margin = mode.left_margin;
		screeninfo->right_margin = mode.right_margin;
		screeninfo->upper_margin = mode.upper_margin;
		screeninfo->lower_margin = mode.lower_margin;
		screeninfo->hsync_len = mode.hsync_len;
		screeninfo->vsync_len = mode.vsync_len;

		fb_data->hw_initializing = true;

		/* Initialize EPDC settings and init panel */
		ret =
		    mxc_epdc_fb_init_hw((struct fb_info *)fb_data);
		if (ret) {
			dev_err(fb_data->dev,
				"Failed to load panel waveform data\n");
			return ret;
		}
	}

	/*
	 * EOF sync delay (in us) should be equal to the vscan holdoff time
	 * VSCAN_HOLDOFF time = (VSCAN_HOLDOFF value + 1) * Vertical lines
	 * Add 25us for additional margin
	 */
	fb_data->eof_sync_period = (fb_data->cur_mode->vscan_holdoff + 1) *
		1000000/(fb_data->cur_mode->vmode->refresh *
		(fb_data->cur_mode->vmode->upper_margin +
		fb_data->cur_mode->vmode->yres +
		fb_data->cur_mode->vmode->lower_margin +
		fb_data->cur_mode->vmode->vsync_len)) + 25;

	mxc_epdc_fb_set_fix(info);

	return 0;
}

static int mxc_epdc_fb_check_var(struct fb_var_screeninfo *var,
				 struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	int cfa_scale = fb_data->use_cfa ? 2 : 1;
	
	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	if (((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
	    (var->bits_per_pixel != 16) && (var->bits_per_pixel != 8)) ||
	    fb_data->use_cfa)
		var->bits_per_pixel = default_bpp;
	printk(KERN_INFO "BPP: %i\n" ,var->bits_per_pixel);
	switch (var->bits_per_pixel) {
	case 8:
		if (var->grayscale != 0) {
			/*
			 * For 8-bit grayscale, R, G, and B offset are equal.
			 *
			 */
			var->red.length = 8;
			var->red.offset = 0;
			var->red.msb_right = 0;

			var->green.length = 8;
			var->green.offset = 0;
			var->green.msb_right = 0;

			var->blue.length = 8;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
		} else {
			var->red.length = 3;
			var->red.offset = 5;
			var->red.msb_right = 0;

			var->green.length = 3;
			var->green.offset = 2;
			var->green.msb_right = 0;

			var->blue.length = 2;
			var->blue.offset = 0;
			var->blue.msb_right = 0;

			var->transp.length = 0;
			var->transp.offset = 0;
			var->transp.msb_right = 0;
		}
		break;
	case 16:
		var->red.length = 5;
		var->red.offset = 11;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;
		break;
	}

	switch (var->rotate) {
	case FB_ROTATE_UR:
	case FB_ROTATE_UD:
		var->xres = fb_data->native_width / cfa_scale;
		var->yres = fb_data->native_height / cfa_scale;
		break;
	case FB_ROTATE_CW:
	case FB_ROTATE_CCW:
		var->xres = fb_data->native_height / cfa_scale;
		var->yres = fb_data->native_width / cfa_scale;
		break;
	default:
		/* Invalid rotation value */
		var->rotate = 0;
		dev_err(fb_data->dev, "Invalid rotation request\n");
		return -EINVAL;
	}

	var->xres_virtual = ALIGN(var->xres, 32);
	var->yres_virtual = ALIGN(var->yres, 128) * fb_data->num_screens;

	var->height = -1;
	var->width = -1;

	return 0;
}

void mxc_epdc_fb_set_waveform_modes(struct mxcfb_waveform_modes *modes,
	struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	mutex_lock(&fb_data->queue_mutex);

	memcpy(&fb_data->wv_modes, modes, sizeof(struct mxcfb_waveform_modes));

	/* Set flag to ensure that new waveform modes
	 * are programmed into EPDC before next update */
	fb_data->wv_modes_update = true;

	mutex_unlock(&fb_data->queue_mutex);
}
EXPORT_SYMBOL(mxc_epdc_fb_set_waveform_modes);

static int mxc_epdc_fb_get_temp_index(struct mxc_epdc_fb_data *fb_data, int temp)
{

	int i;
	int index = -1;

	if (fb_data->trt_entries == 0) {
		dev_info(fb_data->dev,
			"No TRT exists...using default temp index %i\n", DEFAULT_TEMP_INDEX);
		return DEFAULT_TEMP_INDEX;
	}
#if 0
	/* Search temperature ranges for a match */
	for (i = 0; i < fb_data->trt_entries /*- 1*/; i++) {
		dev_dbg(fb_data->dev, "Check temperature range %i: from %i to %i", 
			fb_data->trt_entries, 
			fb_data->temp_range_bounds[i], 
			fb_data->temp_range_bounds[i+1]);
		if ((temp >= fb_data->temp_range_bounds[i])
			&& (temp < fb_data->temp_range_bounds[i+1])) {
			index = i;
			break;
		}
	}
#else
	for (i = fb_data->trt_entries-1; i >= 0 /*- 1*/; i--) {
		dev_dbg(fb_data->dev, "Check temperature range %i: from %i to %i", 
			i, 
			fb_data->temp_range_bounds[i], 
			fb_data->temp_range_bounds[i+1]);
		if ((temp >= fb_data->temp_range_bounds[i])) {
			index = i;
			break;
		}
	}
	
#endif
	if (index < 0) {
		dev_info(fb_data->dev,
			"No TRT index match for %i...using default temp index %i\n", temp, DEFAULT_TEMP_INDEX);
		return DEFAULT_TEMP_INDEX;
	}

	dev_info(fb_data->dev, "Using temperature index %d (%i)\n", index, temp);

	return index;
}

int mxc_epdc_fb_set_temperature(int temperature, struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	/* Store temp index. Used later when configuring updates. */
	mutex_lock(&fb_data->queue_mutex);
	fb_data->temp_index = mxc_epdc_fb_get_temp_index(fb_data, temperature);
	mutex_unlock(&fb_data->queue_mutex);

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_temperature);

int mxc_epdc_fb_set_auto_update(u32 auto_mode, struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	dev_dbg(fb_data->dev, "Setting auto update mode to %d\n", auto_mode);

	if ((auto_mode == AUTO_UPDATE_MODE_AUTOMATIC_MODE)
		|| (auto_mode == AUTO_UPDATE_MODE_REGION_MODE))
		fb_data->auto_mode = auto_mode;
	else {
		dev_err(fb_data->dev, "Invalid auto update mode parameter.\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_auto_update);

int mxc_epdc_fb_set_upd_scheme(u32 upd_scheme, struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	dev_info(fb_data->dev, "Setting optimization level to %d\n", upd_scheme);
	
	/*
	 * Can't change the scheme until current updates have completed.
	 * This function returns when all active updates are done.
	 */
	mxc_epdc_fb_flush_updates(fb_data);

	if ((upd_scheme == UPDATE_SCHEME_SNAPSHOT)
		|| (upd_scheme == UPDATE_SCHEME_QUEUE)
		|| (upd_scheme == UPDATE_SCHEME_QUEUE_AND_MERGE))
		fb_data->upd_scheme = upd_scheme;
	else {
		dev_err(fb_data->dev, "Invalid update scheme specified.\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_upd_scheme);

typedef void(reverse_t)(void *dst, const void *src, size_t n);

static inline void reverse8(uint8_t *dst, const uint8_t *src, size_t n)
{

	while (n--)
		*dst++ = *src--;
}

static inline void reverse16(uint16_t *dst, const uint16_t *src, size_t n)
{

	while (n--)
		*dst++ = *src--;
}

static inline void reverse32(uint32_t *dst, const uint32_t *src, size_t n)
{

	while (n--)
		*dst++ = *src--;
}

static void copy_before_process(struct mxc_epdc_fb_data *fb_data,
	struct update_data_list *upd_data_list)
{

	const struct imx_epdc_fb_mode *epdc_mode = fb_data->cur_mode;
	struct mxcfb_update_data *upd_data =
		&upd_data_list->update_desc->upd_data;
	int i;
	unsigned char *temp_buf_ptr = fb_data->virt_addr_copybuf;
	unsigned char *src_ptr;
	struct mxcfb_rect *src_upd_region;
	int temp_buf_stride;
	int src_stride;
	int bpp = fb_data->epdc_fb_var.bits_per_pixel;
	int left_offs, right_offs;
	int x_trailing_bytes, y_trailing_bytes;
	int alt_buf_offset;
	int flip_ymax;
	size_t line_offset;
	reverse_t *reverse;
	
	/* Set source buf pointer based on input source, panning, etc. */
	if (upd_data->flags & EPDC_FLAG_USE_ALT_BUFFER) {
		src_upd_region = &upd_data->alt_buffer_data.alt_update_region;
		src_stride =
			upd_data->alt_buffer_data.width * bpp/8;
		alt_buf_offset = upd_data->alt_buffer_data.phys_addr -
			fb_data->info.fix.smem_start;
		src_ptr = fb_data->info.screen_base + alt_buf_offset
			+ src_upd_region->top * src_stride;
	} else {
		src_upd_region = &upd_data->update_region;
		src_stride = fb_data->epdc_fb_var.xres_virtual * bpp/8;
		src_ptr = fb_data->info.screen_base + fb_data->fb_offset
			+ src_upd_region->top * src_stride;
	}
	
	temp_buf_stride = ALIGN(src_upd_region->width, 8) * bpp/8;
	left_offs = src_upd_region->left * bpp/8;
	right_offs = src_upd_region->width * bpp/8;
	x_trailing_bytes = (ALIGN(src_upd_region->width, 8)
		- src_upd_region->width) * bpp/8;

	if (bpp == 16)
		reverse = (reverse_t *)reverse16;
	else if (bpp == 32)
		reverse = (reverse_t *)reverse32;
	else
		reverse = (reverse_t *)reverse8;

	flip_ymax = (fb_data->epdc_fb_var.yres / 2) - src_upd_region->top;
	flip_ymax = max(0, flip_ymax);
	line_offset = (src_upd_region->width - 1) * (bpp / 8);
	
	for (i = 0; i < src_upd_region->height; i++) {
		/* Copy the full line */
		if (!epdc_mode->flip_top || (i >= flip_ymax)) {
			memcpy(temp_buf_ptr, src_ptr + left_offs,
			       src_upd_region->width * bpp/8);
		} else {
			/* This relies on the scan direction to be inverted in
			 * order to complete the 180 degrees rotation */
			const u8 *src = src_ptr + left_offs + line_offset;

			reverse(temp_buf_ptr, src, src_upd_region->width);
		}

		/* Clear any unwanted pixels at the end of each line */
		if (src_upd_region->width & 0x7) {
			memset(temp_buf_ptr + right_offs, 0x0,
				x_trailing_bytes);
		}

		temp_buf_ptr += temp_buf_stride;
		src_ptr += src_stride;
	}

	/* Clear any unwanted pixels at the bottom of the end of each line */
	if (src_upd_region->height & 0x7) {
		y_trailing_bytes = (ALIGN(src_upd_region->height, 8)
			- src_upd_region->height) *
			ALIGN(src_upd_region->width, 8) * bpp/8;
		memset(temp_buf_ptr, 0x0, y_trailing_bytes);
	}
	
}

static int epdc_process_update(struct update_data_list *upd_data_list,
				   struct mxc_epdc_fb_data *fb_data)
{
	struct mxcfb_rect *src_upd_region; /* Region of src buffer for update */
	struct mxcfb_rect pxp_upd_region;
	u32 src_width, src_height;
	u32 offset_from_4, bytes_per_pixel;
	u32 post_rotation_xcoord, post_rotation_ycoord, width_pxp_blocks;
	u32 pxp_input_offs, pxp_output_offs, pxp_output_shift;
	u32 hist_stat = 0;
	int width_unaligned, height_unaligned;
	bool input_unaligned = false;
	bool line_overflow = false;
	int pix_per_line_added;
	bool use_temp_buf = false;
	struct mxcfb_rect temp_buf_upd_region;
	struct update_desc_list *upd_desc_list = upd_data_list->update_desc;

	int ret, wfid;

	wfid = upd_desc_list->upd_data.waveform_mode;


	/*
	 * Gotta do a whole bunch of buffer ptr manipulation to
	 * work around HW restrictions for PxP & EPDC
	 * Note: Applies to pre-2.0 versions of EPDC/PxP
	 */
	
	/*
	 * Are we using FB or an alternate (overlay)
	 * buffer for source of update?
	 */
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER) {
		src_width = upd_desc_list->upd_data.alt_buffer_data.width;
		src_height = upd_desc_list->upd_data.alt_buffer_data.height;
		src_upd_region = &upd_desc_list->upd_data.alt_buffer_data.alt_update_region;
	} else {
		src_width = fb_data->epdc_fb_var.xres_virtual;
		src_height = fb_data->epdc_fb_var.yres;
		src_upd_region = &upd_desc_list->upd_data.update_region;
	}

	bytes_per_pixel = fb_data->epdc_fb_var.bits_per_pixel/8;
	
	if(mxc_epdc_fb_scrambling_modparam){
 
		void *src_buf_virt, *dst_buf_virt;
		
		src_buf_virt = fb_data->info.screen_base + fb_data->fb_offset;
		dst_buf_virt = upd_data_list->virt_addr;
		scramble_array(
			fb_data,
			src_buf_virt, 
			dst_buf_virt, 
			src_upd_region
			);
			
		if(wfid == 5)
			regal_process_update(fb_data, dst_buf_virt, src_upd_region);	
	
		switch (fb_data->epdc_fb_var.rotate) {
		case FB_ROTATE_UR:
		case FB_ROTATE_UD:
			upd_desc_list->epdc_stride = (src_upd_region->width/2);
			break;
		case FB_ROTATE_CW:
		case FB_ROTATE_CCW:
			upd_desc_list->epdc_stride = (src_upd_region->height*2);
			break;
		}

		if ((fb_data->power_state == POWER_STATE_OFF)
				|| fb_data->powering_down) {
			epdc_powerup(fb_data);
		}
		
		return 0;	
	}
	//*/
	if (fb_data->use_cfa) {
		void *src_buf_virt, *dst_buf_virt;

		if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER) {
			src_buf_virt = fb_data->info.screen_base +
				(upd_desc_list->upd_data.alt_buffer_data.phys_addr -
				 fb_data->phys_start);
		} else {
			src_buf_virt = fb_data->info.screen_base + fb_data->fb_offset;
		}

		dst_buf_virt = upd_data_list->virt_addr;
		
		/* Use software conversion to CFA pixel format (2x2 RGBW8888) instead */
		cfa_process_update(fb_data, src_width, src_height,
				src_buf_virt, dst_buf_virt, src_upd_region);
		
		memcpy(fb_data->prev_image, dst_buf_virt, src_upd_region->height * src_upd_region->width * sizeof(uint16_t));
		
		if(wfid == 5)
			regal_process_update(fb_data, dst_buf_virt, src_upd_region);

		switch (fb_data->epdc_fb_var.rotate) {
		case FB_ROTATE_UR:
		case FB_ROTATE_UD:
			upd_desc_list->epdc_stride = src_upd_region->width;
			break;
		case FB_ROTATE_CW:
		case FB_ROTATE_CCW:
			upd_desc_list->epdc_stride = src_upd_region->height;
			break;
		}

		if ((fb_data->power_state == POWER_STATE_OFF)
				|| fb_data->powering_down) {
			epdc_powerup(fb_data);
		}
		
		return 0;
	}
	//*
		
	if(wfid == 5){
		void *src_buf_virt, *dst_buf_virt;
		printk("5");
		if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER) {
			src_buf_virt = fb_data->info.screen_base +
				(upd_desc_list->upd_data.alt_buffer_data.phys_addr -
				 fb_data->phys_start);
		} else {
			src_buf_virt = fb_data->info.screen_base + fb_data->fb_offset;
		}

		dst_buf_virt = upd_data_list->virt_addr;
		
		copy_update(fb_data, src_buf_virt, dst_buf_virt, src_upd_region);
		/* Use software conversion to CFA pixel format (2x2 RGBW8888) instead */
		
		memcpy(fb_data->prev_image, dst_buf_virt, src_upd_region->height * src_upd_region->width * sizeof(uint16_t));
		
		regal_process_update(fb_data, dst_buf_virt, src_upd_region);

		switch (fb_data->epdc_fb_var.rotate) {
		case FB_ROTATE_UR:
		case FB_ROTATE_UD:
			upd_desc_list->epdc_stride = src_upd_region->width;
			break;
		case FB_ROTATE_CW:
		case FB_ROTATE_CCW:
			upd_desc_list->epdc_stride = src_upd_region->height;
			break;
		}

		if ((fb_data->power_state == POWER_STATE_OFF)
				|| fb_data->powering_down) {
			epdc_powerup(fb_data);
		}
		
		return 0;
	}
		
	//*/
	/*
	 * SW workaround for PxP limitation (for pre-v2.0 HW)
	 *
	 * There are 3 cases where we cannot process the update data
	 * directly from the input buffer:
	 *
	 * 1) PxP must process 8x8 pixel blocks, and all pixels in each block
	 * are considered for auto-waveform mode selection. If the
	 * update region is not 8x8 aligned, additional unwanted pixels
	 * will be considered in auto-waveform mode selection.
	 *
	 * 2) PxP input must be 32-bit aligned, so any update
	 * address not 32-bit aligned must be shifted to meet the
	 * 32-bit alignment.  The PxP will thus end up processing pixels
	 * outside of the update region to satisfy this alignment restriction,
	 * which can affect auto-waveform mode selection.
	 *
	 * 3) If input fails 32-bit alignment, and the resulting expansion
	 * of the processed region would add at least 8 pixels more per
	 * line than the original update line width, the EPDC would
	 * cause screen artifacts by incorrectly handling the 8+ pixels
	 * at the end of each line.
	 *
	 * Workaround is to copy from source buffer into a temporary
	 * buffer, which we pad with zeros to match the 8x8 alignment
	 * requirement. This temp buffer becomes the input to the PxP.
	 */
	width_unaligned = src_upd_region->width & 0x7;
	height_unaligned = src_upd_region->height & 0x7;

	offset_from_4 = src_upd_region->left & 0x3;
	input_unaligned = ((offset_from_4 * bytes_per_pixel % 4) != 0) ?
				true : false;

	pix_per_line_added = (offset_from_4 * bytes_per_pixel % 4)
					/ bytes_per_pixel;
	if ((((fb_data->epdc_fb_var.rotate == FB_ROTATE_UR) ||
		fb_data->epdc_fb_var.rotate == FB_ROTATE_UD)) &&
		(ALIGN(src_upd_region->width, 8) <
			ALIGN(src_upd_region->width + pix_per_line_added, 8)))
		line_overflow = true;

	/* Grab pxp_mutex here so that we protect access
	 * to copybuf in addition to the PxP structures */
	mutex_lock(&fb_data->pxp_mutex);
	
	if (((((width_unaligned || height_unaligned || input_unaligned) &&
		(upd_desc_list->upd_data.waveform_mode == WAVEFORM_MODE_AUTO))
		|| line_overflow) && (fb_data->rev < 20)) ||
		fb_data->restrict_width || fb_data->cur_mode->flip_top) {
		dev_dbg(fb_data->dev, "Copying update before processing.\n");

		/* Update to reflect what the new source buffer will be */
		src_width = ALIGN(src_upd_region->width, 8);
		src_height = ALIGN(src_upd_region->height, 8);

		copy_before_process(fb_data, upd_data_list);

		/*
		 * src_upd_region should now describe
		 * the new update buffer attributes.
		 */
		temp_buf_upd_region.left = 0;
		temp_buf_upd_region.top = 0;
		temp_buf_upd_region.width = src_upd_region->width;
		temp_buf_upd_region.height = src_upd_region->height;
		src_upd_region = &temp_buf_upd_region;

		use_temp_buf = true;
	}

	/*
	 * For pre-2.0 HW, input address must be 32-bit aligned
	 * Compute buffer offset to account for this PxP limitation
	 */
	offset_from_4 = src_upd_region->left & 0x3;
	input_unaligned = ((offset_from_4 * bytes_per_pixel % 4) != 0) ?
				true : false;
	if ((fb_data->rev < 20) && input_unaligned) {
		/* Leave a gap between PxP input addr and update region pixels */
		pxp_input_offs =
			(src_upd_region->top * src_width + src_upd_region->left)
			* bytes_per_pixel & 0xFFFFFFFC;
		/* Update region left changes to reflect relative position to input ptr */
		pxp_upd_region.left = (offset_from_4 * bytes_per_pixel % 4)
					/ bytes_per_pixel;
	} else {
		pxp_input_offs =
			(src_upd_region->top * src_width + src_upd_region->left)
			* bytes_per_pixel;
		pxp_upd_region.left = 0;
	}

	pxp_upd_region.top = 0;

	/*
	 * For version 2.0 and later of EPDC & PxP, if no rotation, we don't
	 * need to align width & height (rotation always requires 8-pixel
	 * width & height alignment, per PxP limitations)
	 */
	if ((fb_data->epdc_fb_var.rotate == 0) && (fb_data->rev >= 20)) {
		pxp_upd_region.width = src_upd_region->width;
		pxp_upd_region.height = src_upd_region->height;
	} else {
		/* Update region dimensions to meet 8x8 pixel requirement */
		pxp_upd_region.width = ALIGN(src_upd_region->width + pxp_upd_region.left, 8);
		pxp_upd_region.height = ALIGN(src_upd_region->height, 8);
	}

	switch (fb_data->epdc_fb_var.rotate) {
	case FB_ROTATE_UR:
	default:
		post_rotation_xcoord = pxp_upd_region.left;
		post_rotation_ycoord = pxp_upd_region.top;
		width_pxp_blocks = pxp_upd_region.width;
		break;
	case FB_ROTATE_CW:
		width_pxp_blocks = pxp_upd_region.height;
		post_rotation_xcoord = width_pxp_blocks - src_upd_region->height;
		post_rotation_ycoord = pxp_upd_region.left;
		break;
	case FB_ROTATE_UD:
		width_pxp_blocks = pxp_upd_region.width;
		post_rotation_xcoord = width_pxp_blocks - src_upd_region->width - pxp_upd_region.left;
		post_rotation_ycoord = pxp_upd_region.height - src_upd_region->height - pxp_upd_region.top;
		break;
	case FB_ROTATE_CCW:
		width_pxp_blocks = pxp_upd_region.height;
		post_rotation_xcoord = pxp_upd_region.top;
		post_rotation_ycoord = pxp_upd_region.width - src_upd_region->width - pxp_upd_region.left;
		break;
	}

	/* Update region start coord to force PxP to process full 8x8 regions */
	pxp_upd_region.top &= ~0x7;
	pxp_upd_region.left &= ~0x7;

	if (fb_data->rev < 20) {
		pxp_output_shift = ALIGN(post_rotation_xcoord, 8)
			- post_rotation_xcoord;

		pxp_output_offs = post_rotation_ycoord * width_pxp_blocks
			+ pxp_output_shift;

		upd_desc_list->epdc_offs = ALIGN(pxp_output_offs, 8);
	} else {
		pxp_output_shift = 0;
		pxp_output_offs = post_rotation_ycoord * width_pxp_blocks
			+ post_rotation_xcoord;

		upd_desc_list->epdc_offs = pxp_output_offs;
	}

	upd_desc_list->epdc_stride = width_pxp_blocks;

	/* Source address either comes from alternate buffer
	   provided in update data, or from the framebuffer. */
	if (use_temp_buf)
		sg_dma_address(&fb_data->sg[0]) =
			fb_data->phys_addr_copybuf;
	else if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER)
		sg_dma_address(&fb_data->sg[0]) =
			upd_desc_list->upd_data.alt_buffer_data.phys_addr
				+ pxp_input_offs;
	else {
		sg_dma_address(&fb_data->sg[0]) =
			fb_data->info.fix.smem_start + fb_data->fb_offset
			+ pxp_input_offs;
		sg_set_page(&fb_data->sg[0],
			virt_to_page(fb_data->info.screen_base),
			fb_data->info.fix.smem_len,
			offset_in_page(fb_data->info.screen_base));
	}

	/* Update sg[1] to point to output of PxP proc task */
	sg_dma_address(&fb_data->sg[1]) = upd_data_list->phys_addr
						+ pxp_output_shift;
	sg_set_page(&fb_data->sg[1], virt_to_page(upd_data_list->virt_addr),
		    fb_data->max_pix_size,
		    offset_in_page(upd_data_list->virt_addr));

	/*
	 * Set PxP LUT transform type based on update flags.
	 */
	fb_data->pxp_conf.proc_data.lut_transform = 0;
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_ENABLE_INVERSION)
		fb_data->pxp_conf.proc_data.lut_transform |= PXP_LUT_INVERT;
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_FORCE_MONOCHROME)
		fb_data->pxp_conf.proc_data.lut_transform |=
			PXP_LUT_BLACK_WHITE;
	if (upd_desc_list->upd_data.flags & EPDC_FLAG_USE_CMAP)
		fb_data->pxp_conf.proc_data.lut_transform |=
			PXP_LUT_USE_CMAP;

	/*
	 * Toggle inversion processing if 8-bit
	 * inverted is the current pixel format.
	 */
	if (fb_data->epdc_fb_var.grayscale == GRAYSCALE_8BIT_INVERTED)
		fb_data->pxp_conf.proc_data.lut_transform ^= PXP_LUT_INVERT;

	/* This is a blocking call, so upon return PxP tx should be done */
	ret = pxp_process_update(fb_data, src_width, src_height,
		&pxp_upd_region);
	if (ret) {
		dev_err(fb_data->dev, "Unable to submit PxP update task.\n");
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}

	/* If needed, enable EPDC HW while ePxP is processing */
	if ((fb_data->power_state == POWER_STATE_OFF)
		|| fb_data->powering_down) {
		epdc_powerup(fb_data);
	}

	/* This is a blocking call, so upon return PxP tx should be done */
	ret = pxp_complete_update(fb_data, &hist_stat);
	if (ret) {
		dev_err(fb_data->dev, "Unable to complete PxP update task.\n");
		mutex_unlock(&fb_data->pxp_mutex);
		return ret;
	}

	mutex_unlock(&fb_data->pxp_mutex);

	/* Update waveform mode from PxP histogram results */
	if ((fb_data->rev <= 20) &&
		(upd_desc_list->upd_data.waveform_mode == WAVEFORM_MODE_AUTO)) {
		if (hist_stat & 0x1)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_du;
		else if (hist_stat & 0x2)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc4;
		else if (hist_stat & 0x4)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc8;
		else if (hist_stat & 0x8)
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc16;
		else
			upd_desc_list->upd_data.waveform_mode =
				fb_data->wv_modes.mode_gc32;

		dev_dbg(fb_data->dev, "hist_stat = 0x%x, new waveform = 0x%x\n",
			hist_stat, upd_desc_list->upd_data.waveform_mode);
	}

	return 0;
}

static int epdc_submit_merge(struct update_desc_list *upd_desc_list,
				struct update_desc_list *update_to_merge,
				struct mxc_epdc_fb_data *fb_data)
{

	struct mxcfb_update_data *a, *b;
	struct mxcfb_rect *arect, *brect;
	struct mxcfb_rect combine;
	const int mid_y = fb_data->epdc_fb_var.yres / 2;
	bool use_flags = false;

	a = &upd_desc_list->upd_data;
	b = &update_to_merge->upd_data;
	arect = &upd_desc_list->upd_data.update_region;
	brect = &update_to_merge->upd_data.update_region;

	/* Do not merge a dry-run collision test update */
	if ((a->flags & EPDC_FLAG_TEST_COLLISION) ||
		(b->flags & EPDC_FLAG_TEST_COLLISION))
		return MERGE_BLOCK;

	/* If display 1 flip option is enabled, don't merge update areas across
	 * the mid-height as it needs to be kept in 2 separate rectangles since
	 * top part is rotated.
	 */
	if (fb_data->cur_mode->flip_top &&
	    (((arect->top < mid_y) && (brect->top >= mid_y))
	     || ((brect->top < mid_y) && (arect->top >= mid_y))))
		return MERGE_FAIL;

	/*
	 * Updates with different flags must be executed sequentially.
	 * Halt the merge process to ensure this.
	 */
	if (a->flags != b->flags) {
		/*
		 * Special exception: if update regions are identical,
		 * we may be able to merge them.
		 */
		if ((arect->left != brect->left) ||
			(arect->top != brect->top) ||
			(arect->width != brect->width) ||
			(arect->height != brect->height))
			return MERGE_BLOCK;

		use_flags = true;
	}

	if ((a->flags == b->flags) && (a->flags & EPDC_FLAG_USE_ALT_BUFFER)
		&& (a->alt_buffer_data.phys_addr != b->alt_buffer_data.phys_addr))
		return MERGE_FAIL;

	if (a->update_mode != b->update_mode)
		a->update_mode = UPDATE_MODE_FULL;

	if (a->waveform_mode != b->waveform_mode)
		a->waveform_mode = WAVEFORM_MODE_AUTO;

	if (arect->left > (brect->left + brect->width) ||
		brect->left > (arect->left + arect->width) ||
		arect->top > (brect->top + brect->height) ||
		brect->top > (arect->top + arect->height))
		return MERGE_FAIL;

	combine.left = arect->left < brect->left ? arect->left : brect->left;
	combine.top = arect->top < brect->top ? arect->top : brect->top;
	combine.width = (arect->left + arect->width) >
			(brect->left + brect->width) ?
			(arect->left + arect->width - combine.left) :
			(brect->left + brect->width - combine.left);
	combine.height = (arect->top + arect->height) >
			(brect->top + brect->height) ?
			(arect->top + arect->height - combine.top) :
			(brect->top + brect->height - combine.top);

	/* Don't merge if combined width exceeds max width */
	if (fb_data->restrict_width) {
		u32 max_width = epdc_max_update_width * (mxc_epdc_fb_scrambling_modparam!=0?2:1);
		u32 combined_width = combine.width;
		if (fb_data->epdc_fb_var.rotate != FB_ROTATE_UR)
			max_width -= EPDC_V2_ROTATION_ALIGNMENT;
		if ((fb_data->epdc_fb_var.rotate == FB_ROTATE_CW) ||
			(fb_data->epdc_fb_var.rotate == FB_ROTATE_CCW))
			combined_width = combine.height;
		if (combined_width > max_width)
			return MERGE_FAIL;
	}

	*arect = combine;

	/* Use flags of the later update */
	if (use_flags)
		a->flags = b->flags;

	/* Merge markers */
	list_splice_tail(&update_to_merge->upd_marker_list,
		&upd_desc_list->upd_marker_list);

	/* Merged update should take on the earliest order */
	upd_desc_list->update_order =
		(upd_desc_list->update_order > update_to_merge->update_order) ?
		upd_desc_list->update_order : update_to_merge->update_order;

	return MERGE_OK;
}

static void epdc_submit_work_func(struct work_struct *work)
{
	
	int temp_index;
	struct update_data_list *next_update, *temp_update;
	struct update_desc_list *next_desc, *temp_desc;
	struct update_marker_data *next_marker, *temp_marker;
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data, epdc_submit_work);
	struct update_data_list *upd_data_list = NULL;
	struct mxcfb_rect adj_update_region, *upd_region;
	bool end_merge = false;
	bool is_transform;
	u32 update_addr;
	int *err_dist;
	int ret;
	int cfa_scale;

	/* Protect access to buffer queues and to update HW */
	mutex_lock(&fb_data->queue_mutex);

	/*
	 * Are any of our collision updates able to go now?
	 * Go through all updates in the collision list and check to see
	 * if the collision mask has been fully cleared
	 */

	list_for_each_entry_safe(next_update, temp_update,
				&fb_data->upd_buf_collision_list, list) {

		if (next_update->collision_mask != 0)
			continue;

		dev_dbg(fb_data->dev, "A collision update is ready to go!\n");

		/* Force waveform mode to auto for resubmitted collisions */
		next_update->update_desc->upd_data.waveform_mode =
			WAVEFORM_MODE_AUTO;

		/*
		 * We have a collision cleared, so select it for resubmission.
		 * If an update is already selected, attempt to merge.
		 */
		if (!upd_data_list) {
			upd_data_list = next_update;
			list_del_init(&next_update->list);

			if (fb_data->upd_scheme == UPDATE_SCHEME_QUEUE)
				/* If not merging, we have our update */
				break;
		} else {

			switch (epdc_submit_merge(upd_data_list->update_desc,
						next_update->update_desc,
						fb_data)) {
			case MERGE_OK:
				dev_dbg(fb_data->dev,
					"Update merged [collision]\n");
				list_del_init(&next_update->update_desc->list);
				kfree(next_update->update_desc);
				next_update->update_desc = NULL;
				list_del_init(&next_update->list);
				/* Add to free buffer list */
				list_add_tail(&next_update->list,
					 &fb_data->upd_buf_free_list);
				break;
			case MERGE_FAIL:
				dev_dbg(fb_data->dev,
					"Update not merged [collision]\n");
				break;
			case MERGE_BLOCK:
				dev_dbg(fb_data->dev,
					"Merge blocked [collision]\n");
				end_merge = true;
				break;
			}

			if (end_merge) {
				end_merge = false;
				break;
			}
		}
	}

	/*
	 * Skip pending update list only if we found a collision
	 * update and we are not merging
	 */
	if (!((fb_data->upd_scheme == UPDATE_SCHEME_QUEUE) &&
		upd_data_list)) {

		/*
		 * If we didn't find a collision update ready to go, we
		 * need to get a free buffer and match it to a pending update.
		 */

		/*
		 * Can't proceed if there are no free buffers (and we don't
		 * already have a collision update selected)
		*/
		if (!upd_data_list &&
			list_empty(&fb_data->upd_buf_free_list)) {
			mutex_unlock(&fb_data->queue_mutex);
			
			return;
		}

		list_for_each_entry_safe(next_desc, temp_desc,
				&fb_data->upd_pending_list, list) {

			dev_dbg(fb_data->dev, "Found a pending update!\n");

			if (!upd_data_list) {
				if (list_empty(&fb_data->upd_buf_free_list))
					break;
				upd_data_list =
					list_entry(fb_data->upd_buf_free_list.next,
						struct update_data_list, list);
				list_del_init(&upd_data_list->list);
				upd_data_list->update_desc = next_desc;
				list_del_init(&next_desc->list);
				if (fb_data->upd_scheme == UPDATE_SCHEME_QUEUE)
					/* If not merging, we have an update */
					break;
			} else {
				switch (epdc_submit_merge(upd_data_list->update_desc,
						next_desc, fb_data)) {
				case MERGE_OK:
					dev_dbg(fb_data->dev,
						"Update merged [queue]\n");
					list_del_init(&next_desc->list);
					kfree(next_desc);
					break;
				case MERGE_FAIL:
					dev_dbg(fb_data->dev,
						"Update not merged [queue]\n");
					break;
				case MERGE_BLOCK:
					dev_dbg(fb_data->dev,
						"Merge blocked [collision]\n");
					end_merge = true;
					break;
				}

				if (end_merge)
					break;
			}
		}
	}

	/* Is update list empty? */
	if (!upd_data_list) {
		mutex_unlock(&fb_data->queue_mutex);
		
		return;
	}

	/*
	 * If no processing required, skip update processing
	 * No processing means:
	 *   - FB unrotated
	 *   - FB pixel format = 8-bit grayscale
	 *   - No look-up transformations (inversion, posterization, etc.)
	 *
	 * Note: A bug with EPDC stride prevents us from skipping
	 * PxP in versions 2.0 and earlier of EPDC.
	 */
	is_transform = upd_data_list->update_desc->upd_data.flags &
		(EPDC_FLAG_ENABLE_INVERSION | EPDC_FLAG_USE_DITHERING_Y1 |
		EPDC_FLAG_USE_DITHERING_Y4 | EPDC_FLAG_FORCE_MONOCHROME |
		(mxc_epdc_fb_scrambling_modparam==1) | EPDC_FLAG_USE_CMAP) ? true : false;

	if ((fb_data->epdc_fb_var.rotate == FB_ROTATE_UR) &&
		(fb_data->epdc_fb_var.grayscale == GRAYSCALE_8BIT) &&
		!is_transform && (fb_data->rev > 20) &&
		!fb_data->restrict_width) {

		/* If needed, enable EPDC HW while ePxP is processing */
		if ((fb_data->power_state == POWER_STATE_OFF)
			|| fb_data->powering_down)
			epdc_powerup(fb_data);

		/*
		 * Set update buffer pointer to the start of
		 * the update region in the frame buffer.
		 */
		upd_region = &upd_data_list->update_desc->upd_data.update_region;

		update_addr = fb_data->info.fix.smem_start +
			((upd_region->top * fb_data->info.var.xres_virtual) +
			upd_region->left) * fb_data->info.var.bits_per_pixel/8;
		upd_data_list->update_desc->epdc_stride =
					fb_data->info.var.xres_virtual *
					fb_data->info.var.bits_per_pixel/8;
	} else {
		/* Select from PxP output buffers */
		upd_data_list->phys_addr =
			fb_data->phys_addr_updbuf[fb_data->upd_buffer_num];
		upd_data_list->virt_addr =
			fb_data->virt_addr_updbuf[fb_data->upd_buffer_num];
		fb_data->upd_buffer_num++;
		if (fb_data->upd_buffer_num > fb_data->max_num_buffers-1)
			fb_data->upd_buffer_num = 0;

		/* Release buffer queues */
		mutex_unlock(&fb_data->queue_mutex);

		/* Perform PXP processing - EPDC power will also be enabled */
		if (epdc_process_update(upd_data_list, fb_data)) {
			dev_dbg(fb_data->dev, "PXP processing error.\n");
			/* Protect access to buffer queues and to update HW */
			mutex_lock(&fb_data->queue_mutex);
			list_del_init(&upd_data_list->update_desc->list);
			kfree(upd_data_list->update_desc);
			upd_data_list->update_desc = NULL;
			/* Add to free buffer list */
			list_add_tail(&upd_data_list->list,
				&fb_data->upd_buf_free_list);
			/* Release buffer queues */
			mutex_unlock(&fb_data->queue_mutex);

			return;
		}

		/* Protect access to buffer queues and to update HW */
		mutex_lock(&fb_data->queue_mutex);

		update_addr = upd_data_list->phys_addr
				+ upd_data_list->update_desc->epdc_offs;
	}

	cfa_scale = fb_data->use_cfa ? 2 : 1;
	/* Get rotation-adjusted coordinates */
	adjust_coordinates(fb_data->epdc_fb_var.xres,
		fb_data->epdc_fb_var.yres, fb_data->epdc_fb_var.rotate,
		fb_data->cur_mode->flip_top, &upd_data_list->update_desc->upd_data.update_region,
		&adj_update_region);

	adj_update_region.left *= cfa_scale;
	adj_update_region.top *= cfa_scale;
	adj_update_region.width *= cfa_scale ;
	adj_update_region.height *= cfa_scale;
	//*
	if(mxc_epdc_fb_scrambling_modparam){
		#if 1
		/* S115: 	w=1380 	-> w=690
		 * 			h=96	-> h=192
		 * 			l1=0	-> l1=167
		 * 			l2=1380	-> l2=1024+167 (1380/2 + 2dz) + dz
		 * 			l3=2760	-> l3=1024=1024=167 ((1380/2 + 2dz) * 2) + 167
		 * 			l4=4140	-> l4=1024+1024+1024+167 (1380/2 + 2dz) * 3) + 167
		 * 			ln=x	-> ln=x/2/690 * (1+2dz)+dz
		 */
		 
		/* S079: 	w=768 	-> w=384 => max_update_width
		 * 			h=192	-> h=384
		 * 			l1=0	-> l1=0
		 * 			l2=768	-> l2=384+128
		 * 			l3=1536	-> l3=384+128+384+128
		 * 			l4=2304	-> l4=384+128+384+128+384+128
		 * 			ln=x	-> ln=x/2/384*(384+dz)
		 * 
		 */  
		//int l = adj_update_region.left;
		adj_update_region.left = adj_update_region.left/2 + get_dead_zones(fb_data->dead_zones, adj_update_region.left/2);
		
		adj_update_region.top *= 2;
		adj_update_region.width /= 2;
		adj_update_region.height *= 2;
		#else
		adj_update_region.left /= 2;
		adj_update_region.top *= 2;
		adj_update_region.width /= 2;
		#if NO_DEAD_ZONES
		dev_info(fb_data->dev, "adj_width: %i\n", adj_update_region.width);
		#else
		dev_info(fb_data->dev, "adj_width0: %i\n", adj_update_region.width);
		adj_update_region.width += adj_update_region.width?(get_dead_zone_area(&adj_update_region, fb_data->dead_zones)):0;
		dev_info(fb_data->dev, "adj_width1: %i\n", adj_update_region.width);
		#endif
		adj_update_region.height *= 2;
		#endif
	}
	//*/

	/*
	 * Is the working buffer idle?
	 * If the working buffer is busy, we must wait for the resource
	 * to become free. The IST will signal this event.
	 */
	if (fb_data->cur_update != NULL) {
		dev_dbg(fb_data->dev, "working buf busy!\n");

		/* Initialize event signalling an update resource is free */
		init_completion(&fb_data->update_res_free);

		fb_data->waiting_for_wb = true;

		/* Leave spinlock while waiting for WB to complete */
		mutex_unlock(&fb_data->queue_mutex);
		wait_for_completion(&fb_data->update_res_free);
		mutex_lock(&fb_data->queue_mutex);
	}

	/*
	 * Dithering Processing (Version 1.0 - for i.MX508 and i.MX6SL)
	 */
	if (upd_data_list->update_desc->upd_data.flags &
	    EPDC_FLAG_USE_DITHERING_Y1) {

		err_dist = kzalloc((fb_data->info.var.xres_virtual + 3) * 3
				* sizeof(int), GFP_KERNEL);

		/* Dithering Y8 -> Y1 */
		do_dithering_processing_Y1_v1_0(
				(uint8_t *)(upd_data_list->virt_addr +
				upd_data_list->update_desc->epdc_offs),
				&adj_update_region,
				(fb_data->rev < 20) ?
				ALIGN(adj_update_region.width, 8) :
				adj_update_region.width,
				err_dist);

		kfree(err_dist);
	} else if (upd_data_list->update_desc->upd_data.flags &
		EPDC_FLAG_USE_DITHERING_Y4) {

		err_dist = kzalloc((fb_data->info.var.xres_virtual + 3) * 3
				* sizeof(int), GFP_KERNEL);

		/* Dithering Y8 -> Y1 */
		do_dithering_processing_Y4_v1_0((uint8_t *)(upd_data_list->virt_addr +
				upd_data_list->update_desc->epdc_offs),
				&adj_update_region,
				(fb_data->rev < 20) ?
				ALIGN(adj_update_region.width, 8) :
				adj_update_region.width,
				err_dist);
				

		kfree(err_dist);
	}else if (upd_data_list->update_desc->upd_data.flags &
		EPDC_FLAG_USE_ORDERED_DITHERING) {

		err_dist = kzalloc((fb_data->info.var.xres_virtual + 3) * 3
				* sizeof(int), GFP_KERNEL);

		/* Dithering Y8 -> Y1 */
		do_ordered_dithering_processing((uint8_t *)(upd_data_list->virt_addr +
				upd_data_list->update_desc->epdc_offs),
				&adj_update_region,
				(fb_data->rev < 20) ?
				ALIGN(adj_update_region.width, 8) :
				adj_update_region.width,
				err_dist);
				

		kfree(err_dist);
	}
	/*
	 * If there are no LUTs available,
	 * then we must wait for the resource to become free.
	 * The IST will signal this event.
	 */
	if (!epdc_any_luts_available()) {
		dev_dbg(fb_data->dev, "no luts available!\n");

		/* Initialize event signalling an update resource is free */
		init_completion(&fb_data->update_res_free);

		fb_data->waiting_for_lut = true;

		/* Leave spinlock while waiting for LUT to free up */
		mutex_unlock(&fb_data->queue_mutex);
		wait_for_completion(&fb_data->update_res_free);
		mutex_lock(&fb_data->queue_mutex);
	}

	ret = epdc_choose_next_lut(fb_data->rev, &upd_data_list->lut_num);
	/*
	 * If LUT15 is in use (for pre-EPDC v2.0 hardware):
	 *   - Wait for LUT15 to complete is if TCE underrun prevent is enabled
	 *   - If we go ahead with update, sync update submission with EOF
	 */
	if (ret && fb_data->tce_prevent && (fb_data->rev < 20)) {
		dev_dbg(fb_data->dev, "Waiting for LUT15\n");

		/* Initialize event signalling that lut15 is free */
		init_completion(&fb_data->lut15_free);

		fb_data->waiting_for_lut15 = true;

		/* Leave spinlock while waiting for LUT to free up */
		mutex_unlock(&fb_data->queue_mutex);
		wait_for_completion(&fb_data->lut15_free);
		mutex_lock(&fb_data->queue_mutex);

		epdc_choose_next_lut(fb_data->rev, &upd_data_list->lut_num);
	} else if (ret && (fb_data->rev < 20)) {
		/* Synchronize update submission time to reduce
		   chances of TCE underrun */
		init_completion(&fb_data->eof_event);

		epdc_eof_intr(true);

		/* Leave spinlock while waiting for EOF event */
		mutex_unlock(&fb_data->queue_mutex);
		ret = wait_for_completion_timeout(&fb_data->eof_event,
			msecs_to_jiffies(1000));
		if (!ret) {
			dev_err(fb_data->dev, "Missed EOF event!\n");
			epdc_eof_intr(false);
		}
		udelay(fb_data->eof_sync_period);
		mutex_lock(&fb_data->queue_mutex);

	}

	/* LUTs are available, so we get one here */
	fb_data->cur_update = upd_data_list;

	/* Reset mask for LUTS that have completed during WB processing */
	fb_data->luts_complete_wb = 0;

	/* If we are just testing for collision, we don't assign a LUT,
	 * so we don't need to update LUT-related resources. */
	if (!(upd_data_list->update_desc->upd_data.flags
		& EPDC_FLAG_TEST_COLLISION)) {
		/* Associate LUT with update marker */
		list_for_each_entry_safe(next_marker, temp_marker,
			&upd_data_list->update_desc->upd_marker_list, upd_list)
			next_marker->lut_num = fb_data->cur_update->lut_num;

		/* Mark LUT with order */
		fb_data->lut_update_order[upd_data_list->lut_num] =
			upd_data_list->update_desc->update_order;

		epdc_lut_complete_intr(fb_data->rev, upd_data_list->lut_num,
					true);
	}

	/* Enable Collision and WB complete IRQs */
	epdc_working_buf_intr(true);

	/* Program EPDC update to process buffer */
	if (upd_data_list->update_desc->upd_data.temp != TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			upd_data_list->update_desc->upd_data.temp);
		epdc_set_temp(temp_index);
	} else
		epdc_set_temp(fb_data->temp_index);
	epdc_set_update_addr(update_addr);
	epdc_set_update_coord(adj_update_region.left, adj_update_region.top);
	epdc_set_update_dimensions(adj_update_region.width,
				   adj_update_region.height);
	if (fb_data->rev > 20){
		epdc_set_update_stride(upd_data_list->update_desc->epdc_stride * cfa_scale);
	}
	if (fb_data->wv_modes_update &&
		(upd_data_list->update_desc->upd_data.waveform_mode
			== WAVEFORM_MODE_AUTO)) {
		epdc_set_update_waveform(&fb_data->wv_modes);
		fb_data->wv_modes_update = false;
	}
	
	epdc_submit_update(upd_data_list->lut_num,
			   upd_data_list->update_desc->upd_data.waveform_mode,
			   upd_data_list->update_desc->upd_data.update_mode,
			   (upd_data_list->update_desc->upd_data.flags
				& EPDC_FLAG_TEST_COLLISION) ? true : false,
			   false, 0);

	/* Release buffer queues */
	mutex_unlock(&fb_data->queue_mutex);
	
}

static int mxc_epdc_fb_send_single_update(struct mxcfb_update_data *upd_data,
				   struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	struct update_data_list *upd_data_list = NULL;
	struct mxcfb_rect *screen_upd_region; /* Region on screen to update */
	struct mxcfb_rect adj_update_region;
	int temp_index;
	int ret;
	struct update_desc_list *upd_desc;
	struct update_marker_data *marker_data, *next_marker, *temp_marker;
	int cfa_scale;

	/* Has EPDC HW been initialized? */
	if (!fb_data->hw_ready) {
		/* Throw message if we are not mid-initialization */
		if (!fb_data->hw_initializing)
			dev_err(fb_data->dev, "Display HW not properly"
				"initialized. Aborting update.\n");
		return -EPERM;
	}

	/* Check validity of update params */
	if ((upd_data->update_mode != UPDATE_MODE_PARTIAL) &&
		(upd_data->update_mode != UPDATE_MODE_FULL)) {
		dev_err(fb_data->dev,
			"Update mode 0x%x is invalid.  Aborting update.\n",
			upd_data->update_mode);
		return -EINVAL;
	}
	if ((upd_data->waveform_mode > 255) &&
		(upd_data->waveform_mode != WAVEFORM_MODE_AUTO)) {
		dev_err(fb_data->dev,
			"Update waveform mode 0x%x is invalid."
			"  Aborting update.\n",
			upd_data->waveform_mode);
		return -EINVAL;
	}
	
	mutex_lock(&fb_data->queue_mutex);
	if ((upd_data->update_region.left + upd_data->update_region.width > fb_data->epdc_fb_var.xres) ||
		(upd_data->update_region.top + upd_data->update_region.height > fb_data->epdc_fb_var.yres)) {
		mutex_unlock(&fb_data->queue_mutex);
		dev_err(fb_data->dev,
			"Update region is outside bounds of framebuffer."
			"Aborting update.\n");
		return -EINVAL;
	}
	mutex_unlock(&fb_data->queue_mutex);

	if (upd_data->flags & EPDC_FLAG_USE_ALT_BUFFER) {
		if ((upd_data->update_region.width !=
			upd_data->alt_buffer_data.alt_update_region.width) ||
			(upd_data->update_region.height !=
			upd_data->alt_buffer_data.alt_update_region.height)) {
			dev_err(fb_data->dev,
				"Alternate update region dimensions must "
				"match screen update region dimensions.\n");
			return -EINVAL;
		}
		/* Validate physical address parameter */
		if ((upd_data->alt_buffer_data.phys_addr <
			fb_data->info.fix.smem_start) ||
			(upd_data->alt_buffer_data.phys_addr >
			fb_data->info.fix.smem_start + fb_data->map_size)) {
			dev_err(fb_data->dev,
				"Invalid physical address for alternate "
				"buffer.  Aborting update...\n");
			return -EINVAL;
		}
	}
	dev_dbg(fb_data->dev,"%X\n", upd_data->flags);
	if(upd_data->flags & EPDC_USE_ALT_VSOURCE_L){
		dev_dbg(fb_data->dev,"EPDC_USE_ALT_VSOURCE_L = 1\n");
		gpio_set_value(fb_data->hbz_gpio->gpio_vsource_low1, 1);
		gpio_set_value(fb_data->hbz_gpio->gpio_vsource_low2, 1);
	}else {
		dev_dbg(fb_data->dev,"EPDC_USE_ALT_VSOURCE_L = 0\n");
		gpio_set_value(fb_data->hbz_gpio->gpio_vsource_low1, 0);
		gpio_set_value(fb_data->hbz_gpio->gpio_vsource_low2, 0);
	}
	if(upd_data->flags & EPDC_USE_ALT_VSOURCE_H){
		dev_dbg(fb_data->dev,"EPDC_USE_ALT_VSOURCE_H = 1\n");
		gpio_set_value(fb_data->hbz_gpio->gpio_vsource_high1, 1);
		gpio_set_value(fb_data->hbz_gpio->gpio_vsource_high2, 1);
	}else{
		dev_dbg(fb_data->dev,"EPDC_USE_ALT_VSOURCE_H = 0\n");
		gpio_set_value(fb_data->hbz_gpio->gpio_vsource_high1, 0);
		gpio_set_value(fb_data->hbz_gpio->gpio_vsource_high2, 0);
	}
	
	if(upd_data->flags & EPDC_FLAG_KEEP_POWER_ON){
		fb_data->no_powerdown = 1;
	}else{
		fb_data->no_powerdown = 0;
	}

	mutex_lock(&fb_data->queue_mutex);

	/*
	 * If we are waiting to go into suspend, or the FB is blanked,
	 * we do not accept new updates
	 */
	if ((fb_data->waiting_for_idle) ||
		(fb_data->blank != FB_BLANK_UNBLANK)) {
		dev_dbg(fb_data->dev, "EPDC not active."
			"Update request abort.\n");
		mutex_unlock(&fb_data->queue_mutex);
		return -EPERM;
	}

	if (fb_data->upd_scheme == UPDATE_SCHEME_SNAPSHOT) {
		int count = 0;
		struct update_data_list *plist;

		/*
		 * If next update is a FULL mode update, then we must
		 * ensure that all pending & active updates are complete
		 * before submitting the update.  Otherwise, the FULL
		 * mode update may cause an endless collision loop with
		 * other updates.  Block here until updates are flushed.
		 */
		if (upd_data->update_mode == UPDATE_MODE_FULL) {
			mutex_unlock(&fb_data->queue_mutex);
			mxc_epdc_fb_flush_updates(fb_data);
			mutex_lock(&fb_data->queue_mutex);
		}

		/* Count buffers in free buffer list */
		list_for_each_entry(plist, &fb_data->upd_buf_free_list, list)
			count++;

		/* Use count to determine if we have enough
		 * free buffers to handle this update request */
		if (count + fb_data->max_num_buffers
			<= fb_data->max_num_updates) {
			dev_err(fb_data->dev,
				"No free intermediate buffers available.\n");
			mutex_unlock(&fb_data->queue_mutex);
			return -ENOMEM;
		}

		/* Grab first available buffer and delete from the free list */
		upd_data_list =
		    list_entry(fb_data->upd_buf_free_list.next,
			       struct update_data_list, list);

		list_del_init(&upd_data_list->list);
	}

	/*
	 * Create new update data structure, fill it with new update
	 * data and add it to the list of pending updates
	 */
	upd_desc = kzalloc(sizeof(struct update_desc_list), GFP_KERNEL);
	if (!upd_desc) {
		dev_err(fb_data->dev,
			"Insufficient system memory for update! Aborting.\n");
		if (fb_data->upd_scheme == UPDATE_SCHEME_SNAPSHOT) {
			list_add(&upd_data_list->list,
				&fb_data->upd_buf_free_list);
		}
		mutex_unlock(&fb_data->queue_mutex);
		return -EPERM;
	}
	/* Initialize per-update marker list */
	INIT_LIST_HEAD(&upd_desc->upd_marker_list);
	upd_desc->upd_data = *upd_data;
	upd_desc->update_order = fb_data->order_cnt++;
	list_add_tail(&upd_desc->list, &fb_data->upd_pending_list);

	/* If marker specified, associate it with a completion */
	if (upd_data->update_marker != 0) {
		/* Allocate new update marker and set it up */
		marker_data = kzalloc(sizeof(struct update_marker_data),
				GFP_KERNEL);
		if (!marker_data) {
			dev_err(fb_data->dev, "No memory for marker!\n");
			mutex_unlock(&fb_data->queue_mutex);
			return -ENOMEM;
		}
		list_add_tail(&marker_data->upd_list,
			&upd_desc->upd_marker_list);
		marker_data->update_marker = upd_data->update_marker;
		if (upd_desc->upd_data.flags & EPDC_FLAG_TEST_COLLISION)
			marker_data->lut_num = DRY_RUN_NO_LUT;
		else
			marker_data->lut_num = INVALID_LUT;
		init_completion(&marker_data->update_completion);
		/* Add marker to master marker list */
		list_add_tail(&marker_data->full_list,
			&fb_data->full_marker_list);
	}

	if (fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT) {
		/* Queued update scheme processing */

		mutex_unlock(&fb_data->queue_mutex);

		/* Signal workqueue to handle new update */
		queue_work(fb_data->epdc_submit_workqueue,
			&fb_data->epdc_submit_work);
		
		return 0;
	}

	/* Snapshot update scheme processing */

	/* Set descriptor for current update, delete from pending list */
	upd_data_list->update_desc = upd_desc;
	list_del_init(&upd_desc->list);

	mutex_unlock(&fb_data->queue_mutex);

	/*
	 * Hold on to original screen update region, which we
	 * will ultimately use when telling EPDC where to update on panel
	 */
	screen_upd_region = &upd_desc->upd_data.update_region;

	/* Select from PxP output buffers */
	upd_data_list->phys_addr =
		fb_data->phys_addr_updbuf[fb_data->upd_buffer_num];
	upd_data_list->virt_addr =
		fb_data->virt_addr_updbuf[fb_data->upd_buffer_num];
	fb_data->upd_buffer_num++;
	if (fb_data->upd_buffer_num > fb_data->max_num_buffers-1)
		fb_data->upd_buffer_num = 0;
	
	ret = epdc_process_update(upd_data_list, fb_data);
	if (ret) {
		mutex_unlock(&fb_data->pxp_mutex);
		
		return ret;
	}

	/* Pass selected waveform mode back to user */
	upd_data->waveform_mode = upd_desc->upd_data.waveform_mode;

	cfa_scale = fb_data->use_cfa ? 2 : 1;
	/* Get rotation-adjusted coordinates */
	adjust_coordinates(fb_data->epdc_fb_var.xres,
		fb_data->epdc_fb_var.yres, fb_data->epdc_fb_var.rotate,
		fb_data->cur_mode->flip_top,
		&upd_desc->upd_data.update_region, &adj_update_region);

	adj_update_region.left *= cfa_scale;
	adj_update_region.top *= cfa_scale;
	adj_update_region.width *= cfa_scale;
	adj_update_region.height *= cfa_scale;

	//adj_update_region.left += fb_data->cur_mode->left_border;

	/* Grab lock for queue manipulation and update submission */
	mutex_lock(&fb_data->queue_mutex);

	/*
	 * Is the working buffer idle?
	 * If either the working buffer is busy, or there are no LUTs available,
	 * then we return and let the ISR handle the update later
	 */
	if ((fb_data->cur_update != NULL) || !epdc_any_luts_available()) {
		/* Add processed Y buffer to update list */
		list_add_tail(&upd_data_list->list, &fb_data->upd_buf_queue);

		/* Return and allow the update to be submitted by the ISR. */
		mutex_unlock(&fb_data->queue_mutex);
		return 0;
	}

	/* LUTs are available, so we get one here */
	ret = epdc_choose_next_lut(fb_data->rev, &upd_data_list->lut_num);
	if (ret && fb_data->tce_prevent && (fb_data->rev < 20)) {
		dev_dbg(fb_data->dev, "Must wait for LUT15\n");
		/* Add processed Y buffer to update list */
		list_add_tail(&upd_data_list->list, &fb_data->upd_buf_queue);

		/* Return and allow the update to be submitted by the ISR. */
		mutex_unlock(&fb_data->queue_mutex);
		
		return 0;
	}

	if (!(upd_data_list->update_desc->upd_data.flags
		& EPDC_FLAG_TEST_COLLISION)) {

		/* Save current update */
		fb_data->cur_update = upd_data_list;

		/* Reset mask for LUTS that have completed during WB processing */
		fb_data->luts_complete_wb = 0;

		/* Associate LUT with update marker */
		list_for_each_entry_safe(next_marker, temp_marker,
			&upd_data_list->update_desc->upd_marker_list, upd_list)
			next_marker->lut_num = upd_data_list->lut_num;

		/* Mark LUT as containing new update */
		fb_data->lut_update_order[upd_data_list->lut_num] =
			upd_desc->update_order;

		epdc_lut_complete_intr(fb_data->rev, upd_data_list->lut_num,
					true);
	}

	/* Clear status and Enable LUT complete and WB complete IRQs */
	epdc_working_buf_intr(true);

	/* Program EPDC update to process buffer */
	epdc_set_update_addr(upd_data_list->phys_addr + upd_desc->epdc_offs);
	epdc_set_update_coord(adj_update_region.left, adj_update_region.top);
	epdc_set_update_dimensions(adj_update_region.width,
		adj_update_region.height);
	if (fb_data->rev > 20)
		epdc_set_update_stride(upd_desc->epdc_stride * cfa_scale);
	if (upd_desc->upd_data.temp != TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			upd_desc->upd_data.temp);
		epdc_set_temp(temp_index);
	} else
		epdc_set_temp(fb_data->temp_index);
	if (fb_data->wv_modes_update &&
		(upd_desc->upd_data.waveform_mode == WAVEFORM_MODE_AUTO)) {
		epdc_set_update_waveform(&fb_data->wv_modes);
		fb_data->wv_modes_update = false;
	}

	//dump_all_updates(fb_data);
	epdc_submit_update(upd_data_list->lut_num,
			   upd_desc->upd_data.waveform_mode,
			   upd_desc->upd_data.update_mode,
			   (upd_desc->upd_data.flags
				& EPDC_FLAG_TEST_COLLISION) ? true : false,
			   false, 0);

	mutex_unlock(&fb_data->queue_mutex);	
	
	return 0;
}

static int mxc_epdc_fb_do_send_update(struct mxcfb_update_data *upd_data,
				   struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	if (!fb_data->restrict_width) {
		/* No width restriction, send entire update region */
		return mxc_epdc_fb_send_single_update(upd_data, info);
	} else {
		int ret;
		__u32 width, left;
		__u32 marker;
		__u32 *region_width, *region_left;
		u32 max_upd_width = epdc_max_update_width * (mxc_epdc_fb_scrambling_modparam!=0?2:1);
		
		/* Further restrict max width due to pxp rotation
		  * alignment requirement
		  */
		if (fb_data->epdc_fb_var.rotate != FB_ROTATE_UR)
			max_upd_width -= EPDC_V2_ROTATION_ALIGNMENT;

		/* Select split of width or height based on rotation */
		if ((fb_data->epdc_fb_var.rotate == FB_ROTATE_UR) ||
			(fb_data->epdc_fb_var.rotate == FB_ROTATE_UD)) {
			region_width = &upd_data->update_region.width;
			region_left = &upd_data->update_region.left;
		} else {
			region_width = &upd_data->update_region.height;
			region_left = &upd_data->update_region.top;
		}
		
		if (*region_width <= max_upd_width){
			
			return mxc_epdc_fb_send_single_update(upd_data,	info);
		}
		width = *region_width;
		left = *region_left;
		marker = upd_data->update_marker;
		upd_data->update_marker = 0;

		do {
			*region_width = max_upd_width;
			*region_left = left;
			ret = mxc_epdc_fb_send_single_update(upd_data, info);
			if (ret)
				return ret;
			width -= max_upd_width;
			left += max_upd_width;

		} while (width > max_upd_width);

		*region_width = width;
		*region_left = left;
		upd_data->update_marker = marker;

		return mxc_epdc_fb_send_single_update(upd_data, info);
	}
}

int mxc_epdc_fb_send_update(struct mxcfb_update_data *upd_data,
				   struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info : g_fb_data;
	const int mid_y = fb_data->epdc_fb_var.yres / 2;
	struct mxcfb_update_data upd_flip_data;
	/* If update crosses the mid-height and first display is flipped for a
	 * dual tiled configuration, split into 2 update requests as half of
	 * the pixels will be rotated.
	 *
	 * Note: No need to split if update region is centered horizontally,
	 * which is what the last test is for.
	 */
	
	if (fb_data->cur_mode->flip_top
	    && (upd_data->update_region.top < mid_y)
	    && ((upd_data->update_region.top + upd_data->update_region.height)
		> mid_y)
	    && (((upd_data->update_region.left * 2)
		 + upd_data->update_region.width)
		!= fb_data->epdc_fb_var.xres)) {
		const int height_diff =
			upd_data->update_region.top +
			upd_data->update_region.height - mid_y;
		int stat;

		memcpy(&upd_flip_data, upd_data, sizeof upd_flip_data);
		upd_flip_data.update_region.height -= height_diff;
		stat = mxc_epdc_fb_do_send_update(&upd_flip_data, info);
		if (stat)
			return stat;

		memcpy(&upd_flip_data, upd_data, sizeof upd_flip_data);
		upd_flip_data.update_region.top = mid_y;
		upd_flip_data.update_region.height = height_diff;

		return mxc_epdc_fb_do_send_update(&upd_flip_data, info);
	}
	
	return mxc_epdc_fb_do_send_update(upd_data, info);
}
EXPORT_SYMBOL(mxc_epdc_fb_send_update);

int mxc_epdc_fb_wait_update_complete(struct mxcfb_update_marker_data *marker_data,
						struct fb_info *info, bool async)
{

	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	struct update_marker_data *next_marker;
	struct update_marker_data *found_marker;
	struct update_marker_data *temp;
	bool marker_found = false;
	int ret = 0;

	/* 0 is an invalid update_marker value */
	if (marker_data->update_marker == 0){
		dev_err(fb_data->dev, "0 is an invalid update_marker value\n");
		return -EINVAL;
	}
	/*
	 * Find completion associated with update_marker requested.
	 * Note: If update completed already, marker will have been
	 * cleared, it won't be found, and function will just return.
	 */

	/* Grab queue lock to protect access to marker list */
	mutex_lock(&fb_data->queue_mutex);

	list_for_each_entry_safe(next_marker, temp,
		&fb_data->full_marker_list, full_list) {
		if (next_marker->update_marker == marker_data->update_marker) {
			dev_dbg(fb_data->dev, "Waiting for marker %d\n",
				marker_data->update_marker);
			next_marker->waiting = true;
			marker_found = true;
			break;
		}
	}

	if (!marker_found) {
		/* Search in the completed markers queue, we need to cleanup marker */
		list_for_each_entry_safe(next_marker, temp,
			&fb_data->awaiting_marker_list, aw_list) {
			if (next_marker->update_marker == marker_data->update_marker) {
				dev_dbg(fb_data->dev, "Completed marker found %d\n",
						marker_data->update_marker);
				marker_found = true;
				list_del_init(&next_marker->aw_list);
				break;
			}
		}
	}

	mutex_unlock(&fb_data->queue_mutex);

	/*
	 * If marker not found, it has either been signalled already
	 * or the update request failed.  In either case, just return.
	 */
	if (!marker_found)
		return ret;

	found_marker = next_marker;

	if (async) {
		ret = try_wait_for_completion(&next_marker->update_completion);
		if (!ret)
			return -EBUSY;
	} else {
		ret = wait_for_completion_timeout(&next_marker->update_completion,
				msecs_to_jiffies(5000));
		if (!ret) {
			dev_err(fb_data->dev,
					"Timed out waiting for update completion\n");
			return -ETIMEDOUT;
		}
	}

	marker_data->collision_test = next_marker->collision_test;

	/* Delete marker from awaiting list if it was added to it after list searching
	 * but before {try,wait}_for_completion() call
	 */
	mutex_lock(&fb_data->queue_mutex);
	list_for_each_entry_safe(next_marker, temp,
		&fb_data->awaiting_marker_list, aw_list) {
		if (next_marker == found_marker) {
			list_del_init(&next_marker->aw_list);
			break;
		}
	}
	mutex_unlock(&fb_data->queue_mutex);

	/* Free update marker object */
	kfree(found_marker);

	return ret;
}
EXPORT_SYMBOL(mxc_epdc_fb_wait_update_complete);

int mxc_epdc_fb_set_pwrdown_delay(u32 pwrdown_delay,
					    struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	fb_data->pwrdown_delay = pwrdown_delay;

	return 0;
}
EXPORT_SYMBOL(mxc_epdc_fb_set_pwrdown_delay);

int mxc_epdc_get_pwrdown_delay(struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;

	return fb_data->pwrdown_delay;
}
EXPORT_SYMBOL(mxc_epdc_get_pwrdown_delay);

static int mxc_epdc_fb_ioctl(struct fb_info *info, unsigned int cmd,
			     unsigned long arg)
{

	void __user *argp = (void __user *)arg;
	int ret = -ENOTTY;
	struct mxc_epdc_fb_data *fb_data = info ?
				(struct mxc_epdc_fb_data *)info:g_fb_data;
	dev_dbg(fb_data->dev, "%s: cmd: 0x%x\n", __func__, cmd);
	
	switch (cmd) {
	case MXCFB_SET_WAVEFORM_MODES:
		{
			struct mxcfb_waveform_modes modes;
			dev_dbg(fb_data->dev, "%s: MXCFB_SET_WAVEFORM_MODES\n", __func__);
			if (!copy_from_user(&modes, argp, sizeof(modes))) {
				mxc_epdc_fb_set_waveform_modes(&modes, info);
				ret = 0;
			}
			break;
		}
	case MXCFB_SET_TEMPERATURE:
		{
			int temperature;
			dev_dbg(fb_data->dev, "%s: MXCFB_SET_TEMPERATURE\n", __func__);
			if (!get_user(temperature, (int32_t __user *) arg))
				ret = mxc_epdc_fb_set_temperature(temperature,
					info);
			break;
		}
	case MXCFB_SET_AUTO_UPDATE_MODE:
		{
			u32 auto_mode = 0;
			dev_info(fb_data->dev, "%s: MXCFB_SET_AUTO_UPDATE_MODE\n", __func__);
			if (!get_user(auto_mode, (__u32 __user *) arg))
				ret = mxc_epdc_fb_set_auto_update(auto_mode,
					info);
			break;
		}
	case MXCFB_SET_UPDATE_SCHEME:
		{
			u32 upd_scheme = 0;
			dev_dbg(fb_data->dev, "%s: MXCFB_SET_UPDATE_SCHEME\n", __func__);
			if (!get_user(upd_scheme, (__u32 __user *) arg))
				ret = mxc_epdc_fb_set_upd_scheme(upd_scheme,
					info);
			break;
		}
	case 0x4040462e:
	case MXCFB_SEND_UPDATE:
		{
			struct mxcfb_update_data upd_data;
			dev_dbg(fb_data->dev, "%s: MXCFB_SEND_UPDATE\n", __func__);
			if (mutex_lock_interruptible(&hard_lock) < 0)
				return -ERESTARTSYS;

			flush_cache_all();
			outer_flush_all();

			if (!copy_from_user(&upd_data, argp,
				sizeof(upd_data))) {
				ret = mxc_epdc_fb_send_update(&upd_data, info);
				if (ret == 0 && copy_to_user(argp, &upd_data,
					sizeof(upd_data)))
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}
			
			mutex_unlock(&hard_lock);

			break;
		}

	case MXCFB_WAIT_FOR_UPDATE_COMPLETE:
		{
			struct mxcfb_update_marker_data upd_marker_data;
			dev_dbg(fb_data->dev, "%s: MXCFB_WAIT_FOR_UPDATE_COMPLETE\n", __func__);
			if (!copy_from_user(&upd_marker_data, argp,
				sizeof(upd_marker_data))) {
				ret = mxc_epdc_fb_wait_update_complete(
					&upd_marker_data, info, false);
				if (copy_to_user(argp, &upd_marker_data,
					sizeof(upd_marker_data)))
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}

			break;
		}

	case MXCFB_CHECK_FOR_UPDATE_COMPLETE:
		{
			struct mxcfb_update_marker_data upd_marker_data;
			dev_dbg(fb_data->dev, "%s: MXCFB_CHECK_FOR_UPDATE_COMPLETE\n", __func__);
			if (!copy_from_user(&upd_marker_data, argp,
				sizeof(upd_marker_data))) {
				ret = mxc_epdc_fb_wait_update_complete(
					&upd_marker_data, info, true);
				if (copy_to_user(argp, &upd_marker_data,
					sizeof(upd_marker_data)))
					ret = -EFAULT;
			} else {
				ret = -EFAULT;
			}

			break;
		}

	case MXCFB_SET_PWRDOWN_DELAY:
		{
			int delay = 0;
			dev_dbg(fb_data->dev, "%s: MXCFB_SET_PWRDOWN_DELAY\n", __func__);
			if (!get_user(delay, (__u32 __user *) arg))
				ret =
				    mxc_epdc_fb_set_pwrdown_delay(delay, info);
			break;
		}

	case MXCFB_GET_PWRDOWN_DELAY:
		{
			int pwrdown_delay = mxc_epdc_get_pwrdown_delay(info);
			dev_dbg(fb_data->dev, "%s: MXCFB_GET_PWRDOWN_DELAY\n", __func__);
			if (put_user(pwrdown_delay,
				(int __user *)argp))
				ret = -EFAULT;
			ret = 0;
			break;
		}

	case MXCFB_GET_WORK_BUFFER:
		{
			/* copy the epdc working buffer to the user space */
			struct mxc_epdc_fb_data *fb_data = info ?
				(struct mxc_epdc_fb_data *)info:g_fb_data;
			dev_dbg(fb_data->dev, "%s: MXCFB_GET_WORK_BUFFER\n", __func__);
			flush_cache_all();
			outer_flush_all();
			if (copy_to_user((void __user *)arg,
				(const void *) fb_data->working_buffer_virt,
				fb_data->working_buffer_size))
				ret = -EFAULT;
			else
				ret = 0;
			flush_cache_all();
			outer_flush_all();
			break;
		}

	case MXCFB_DISABLE_EPDC_ACCESS:
		{
			struct mxc_epdc_fb_data *fb_data = info ?
				(struct mxc_epdc_fb_data *)info:g_fb_data;
			dev_dbg(fb_data->dev, "%s: MXCFB_DISABLE_EPDC_ACCESS\n", __func__);
			mxc_epdc_fb_flush_updates(fb_data);
			/* disable handling any user update request */
			mutex_lock(&hard_lock);
			ret = 0;
			break;
		}

	case MXCFB_ENABLE_EPDC_ACCESS:
		{
			/* enable user update handling again */
			dev_dbg(fb_data->dev, "%s: MXCFB_ENABLE_EPDC_ACCESS\n", __func__);
			mutex_unlock(&hard_lock);
			ret = 0;
			break;
		}
	case MXCFB_POWERUP:{
		struct mxc_epdc_fb_data *fb_data = info ?
				(struct mxc_epdc_fb_data *)info:g_fb_data;
		dev_dbg(fb_data->dev, "%s: MXCFB_POWERUP\n", __func__);
		epdc_powerup(fb_data);
		ret = 0;
		break;
	}
	case MXCFB_POWERDOWN:{
		struct mxc_epdc_fb_data *fb_data = info ?
				(struct mxc_epdc_fb_data *)info:g_fb_data;
		dev_dbg(fb_data->dev, "%s: MXCFB_POWERDOWN\n", __func__);
		fb_data->powering_down = true;
		epdc_powerdown(fb_data);
		ret = 0;
		break;		
		
	}
	case MXCFB_DISPLAY_ENABLE:{

		struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
		int ret, channel;

		if (get_user(channel, (__u32 __user *) arg))
			ret = -EFAULT;

		dev_dbg(fb_data->dev, "%s: MXCFB_DISPLAY_ENABLE %i %i %i\n", __func__, channel,regulator_is_enabled(fb_data->v3p3_regulator[0]),regulator_is_enabled(fb_data->v3p3_regulator[1]));
		switch (channel){
			case 1:
				ret = regulator_enable(fb_data->v3p3_regulator[0]);
				break;
			case 2:
				ret = regulator_enable(fb_data->v3p3_regulator[1]);
				break;
			case 3:
				ret = regulator_enable(fb_data->v3p3_regulator[0]);
				if(ret)
					return ret;
				ret = regulator_enable(fb_data->v3p3_regulator[1]);
				break;
			default: 
				return -EINVAL;
		}
		break;
	}
	case MXCFB_DISPLAY_DISABLE:{
		struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

		int ret, channel;
		
		if (get_user(channel, (__u32 __user *) arg))
			ret = -EFAULT;

		dev_dbg(fb_data->dev, "%s: MXCFB_DISPLAY_DISABLE %i %i %i\n", __func__, channel, regulator_is_enabled(fb_data->v3p3_regulator[0]), regulator_is_enabled(fb_data->v3p3_regulator[1]));
		
		switch (channel){
			case -1:
				if(regulator_is_enabled(fb_data->v3p3_regulator[0])){
					ret = regulator_disable(fb_data->v3p3_regulator[0]);
				}
				break;
			case -2:
				if(regulator_is_enabled(fb_data->v3p3_regulator[1])){
					ret = regulator_disable(fb_data->v3p3_regulator[1]);
				}
				break;
			case -3:
				if(regulator_is_enabled(fb_data->v3p3_regulator[0])){
					ret = regulator_disable(fb_data->v3p3_regulator[0]);
				}
				if(ret)
					return ret;
				if(regulator_is_enabled(fb_data->v3p3_regulator[1])){
					ret = regulator_disable(fb_data->v3p3_regulator[1]);
				}
				break;
			default: 
				return -EINVAL;
		}
		break;
	}

	default:
		break;
	}
	dev_dbg(fb_data->dev, "%s: ret: %i\n", __func__, ret);
	return ret;
}

static void mxc_epdc_fb_update_pages(struct mxc_epdc_fb_data *fb_data,
				     u16 y1, u16 y2)
{

	struct mxcfb_update_data update;

	/* Do partial screen update, Update full horizontal lines */
	update.update_region.left = 0;
	update.update_region.width = fb_data->epdc_fb_var.xres;
	update.update_region.top = y1;
	update.update_region.height = y2 - y1;
	update.waveform_mode = WAVEFORM_MODE_AUTO;
	update.update_mode = UPDATE_MODE_PARTIAL;
	update.update_marker = 0;
	update.temp = TEMP_USE_AMBIENT;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, &fb_data->info);
}

/* this is called back from the deferred io workqueue */
static void mxc_epdc_fb_deferred_io(struct fb_info *info,
				    struct list_head *pagelist)
{

	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	struct page *page;
	unsigned long beg, end;
	int y1, y2, miny, maxy;

	if (fb_data->auto_mode != AUTO_UPDATE_MODE_AUTOMATIC_MODE)
		return;

	flush_cache_all();
	outer_flush_all();

	miny = INT_MAX;
	maxy = 0;
	list_for_each_entry(page, pagelist, lru) {
		beg = page->index << PAGE_SHIFT;
		end = beg + PAGE_SIZE - 1;
		y1 = beg / info->fix.line_length;
		y2 = end / info->fix.line_length;
		if (y2 >= fb_data->epdc_fb_var.yres)
			y2 = fb_data->epdc_fb_var.yres - 1;
		if (miny > y1)
			miny = y1;
		if (maxy < y2)
			maxy = y2;
	}

	mxc_epdc_fb_update_pages(fb_data, miny, maxy);
}

void mxc_epdc_fb_flush_updates(struct mxc_epdc_fb_data *fb_data)
{

	int ret;

	if (fb_data->in_init)
		return;

	/* Grab queue lock to prevent any new updates from being submitted */
	mutex_lock(&fb_data->queue_mutex);

	/*
	 * 3 places to check for updates that are active or pending:
	 *   1) Updates in the pending list
	 *   2) Update buffers in use (e.g., PxP processing)
	 *   3) Active updates to panel - We can key off of EPDC
	 *      power state to know if we have active updates.
	 */
	if (!list_empty(&fb_data->upd_pending_list) ||
		!is_free_list_full(fb_data) ||
		(fb_data->updates_active == true)) {
		/* Initialize event signalling updates are done */
		init_completion(&fb_data->updates_done);
		fb_data->waiting_for_idle = true;

		mutex_unlock(&fb_data->queue_mutex);
		/* Wait for any currently active updates to complete */
		ret = wait_for_completion_timeout(&fb_data->updates_done,
						msecs_to_jiffies(8000));
		if (!ret)
			dev_err(fb_data->dev,
				"Flush updates timeout! ret = 0x%x\n", ret);

		mutex_lock(&fb_data->queue_mutex);
		fb_data->waiting_for_idle = false;
	}

	mutex_unlock(&fb_data->queue_mutex);
}

static int mxc_epdc_fb_blank(int blank, struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	int ret;

	dev_dbg(fb_data->dev, "blank = %d\n", blank);

	if (fb_data->blank == blank)
		return 0;

	fb_data->blank = blank;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
		mxc_epdc_fb_flush_updates(fb_data);
		/* Wait for powerdown */
		mutex_lock(&fb_data->power_mutex);
		if ((fb_data->power_state == POWER_STATE_ON) &&
			(fb_data->pwrdown_delay == FB_POWERDOWN_DISABLE)) {

			/* Powerdown disabled, so we disable EPDC manually */
			int count = 0;
			int sleep_ms = 10;

			mutex_unlock(&fb_data->power_mutex);

			/* If any active updates, wait for them to complete */
			while (fb_data->updates_active) {
				/* Timeout after 1 sec */
				if ((count * sleep_ms) > 1000)
					break;
				msleep(sleep_ms);
				count++;
			}

			fb_data->powering_down = true;
			epdc_powerdown(fb_data);
		} else if (fb_data->power_state != POWER_STATE_OFF) {
			fb_data->wait_for_powerdown = true;
			init_completion(&fb_data->powerdown_compl);
			mutex_unlock(&fb_data->power_mutex);
			ret = wait_for_completion_timeout(&fb_data->powerdown_compl,
				msecs_to_jiffies(5000));
			if (!ret) {
				dev_err(fb_data->dev,
					"No powerdown received!\n");
				return -ETIMEDOUT;
			}
		} else
			mutex_unlock(&fb_data->power_mutex);
		break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		mxc_epdc_fb_flush_updates(fb_data);
		break;
	}
	return 0;
}

static int mxc_epdc_fb_pan_display(struct fb_var_screeninfo *var,
				   struct fb_info *info)
{

	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	u_int y_bottom;

	dev_dbg(info->device, "%s: var->yoffset %d, info->var.yoffset %d\n",
		 __func__, var->yoffset, info->var.yoffset);
	/* check if var is valid; also, xpan is not supported */
	if (!var || (var->xoffset != info->var.xoffset) ||
	    (var->yoffset + var->yres > var->yres_virtual)) {
		dev_err(info->device, "x panning not supported\n");
		return -EINVAL;
	}

	if ((fb_data->epdc_fb_var.xoffset == var->xoffset) &&
		(fb_data->epdc_fb_var.yoffset == var->yoffset))
		return 0;	/* No change, do nothing */

	y_bottom = var->yoffset;

	if (!(var->vmode & FB_VMODE_YWRAP))
		y_bottom += var->yres;

	if (y_bottom > info->var.yres_virtual){
		dev_err(info->device, "y_bottom > info->var.yres_virtual\n");
		return -EINVAL;
	}
	
	mutex_lock(&fb_data->queue_mutex);

	fb_data->fb_offset = (var->yoffset * var->xres_virtual + var->xoffset)
		* (var->bits_per_pixel) / 8;

	fb_data->epdc_fb_var.xoffset = var->xoffset;
	fb_data->epdc_fb_var.yoffset = var->yoffset;

	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;

	mutex_unlock(&fb_data->queue_mutex);

	return 0;
}

static struct fb_ops mxc_epdc_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mxc_epdc_fb_check_var,
	.fb_set_par = mxc_epdc_fb_set_par,
	.fb_setcmap = mxc_epdc_fb_setcmap,
	.fb_setcolreg = mxc_epdc_fb_setcolreg,
	.fb_pan_display = mxc_epdc_fb_pan_display,
	.fb_ioctl = mxc_epdc_fb_ioctl,
	.fb_mmap = mxc_epdc_fb_mmap,
	.fb_blank = mxc_epdc_fb_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static struct fb_deferred_io mxc_epdc_fb_defio = {
	.delay = HZ / 4,
	.deferred_io = mxc_epdc_fb_deferred_io,
};

static void epdc_done_work_func(struct work_struct *work)
{
	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data,
			epdc_done_work.work);
	epdc_powerdown(fb_data);
}

static bool is_free_list_full(struct mxc_epdc_fb_data *fb_data)
{
	int count = 0;
	struct update_data_list *plist;

	/* Count buffers in free buffer list */
	list_for_each_entry(plist, &fb_data->upd_buf_free_list, list)
		count++;

	/* Check to see if all buffers are in this list */
	if (count == fb_data->max_num_updates)
		return true;
	else
		return false;
}

static irqreturn_t mxc_epdc_irq_handler(int irq, void *dev_id)
{

	struct mxc_epdc_fb_data *fb_data = dev_id;
	u32 ints_fired, luts1_ints_fired, luts2_ints_fired;

	/*
	 * If we just completed one-time panel init, bypass
	 * queue handling, clear interrupt and return
	 */
	if (fb_data->in_init) {
		if (epdc_is_working_buffer_complete()) {
			epdc_working_buf_intr(false);
			epdc_clear_working_buf_irq();
			dev_dbg(fb_data->dev, "Cleared WB for init update\n");
		}

		if (epdc_is_lut_complete(fb_data->rev, 0)) {
			epdc_lut_complete_intr(fb_data->rev, 0, false);
			epdc_clear_lut_complete_irq(fb_data->rev, 0);
			fb_data->in_init = false;
			dev_dbg(fb_data->dev, "Cleared LUT complete for init update\n");
		}

		return IRQ_HANDLED;
	}

	ints_fired = __raw_readl(EPDC_IRQ_MASK) & __raw_readl(EPDC_IRQ);
	if (fb_data->rev < 20) {
		luts1_ints_fired = 0;
		luts2_ints_fired = 0;
	} else {
		luts1_ints_fired = __raw_readl(EPDC_IRQ_MASK1) & __raw_readl(EPDC_IRQ1);
		luts2_ints_fired = __raw_readl(EPDC_IRQ_MASK2) & __raw_readl(EPDC_IRQ2);
	}

	if (!(ints_fired || luts1_ints_fired || luts2_ints_fired))
		return IRQ_HANDLED;

	if (__raw_readl(EPDC_IRQ) & EPDC_IRQ_TCE_UNDERRUN_IRQ) {
		dev_err(fb_data->dev,
			"TCE underrun! Will continue to update panel\n");
		/* Clear TCE underrun IRQ */
		__raw_writel(EPDC_IRQ_TCE_UNDERRUN_IRQ, EPDC_IRQ_CLEAR);
	}

	/* Check if we are waiting on EOF to sync a new update submission */
	if (epdc_signal_eof()) {
		epdc_eof_intr(false);
		epdc_clear_eof_irq();
		complete(&fb_data->eof_event);
	}

	/*
	 * Workaround for EPDC v2.0/v2.1 errata: Must read collision status
	 * before clearing IRQ, or else collision status for bits 16:63
	 * will be automatically cleared.  So we read it here, and there is
	 * no conflict with using it in epdc_intr_work_func since the
	 * working buffer processing flow is strictly sequential (i.e.,
	 * only one WB processing done at a time, so the data grabbed
	 * here should be up-to-date and accurate when the WB processing
	 * completes.  Also, note that there is no impact to other versions
	 * of EPDC by reading LUT status here.
	 */
	if (fb_data->cur_update != NULL)
		fb_data->epdc_colliding_luts = epdc_get_colliding_luts(fb_data->rev);

	/* Clear the interrupt mask for any interrupts signalled */
	__raw_writel(ints_fired, EPDC_IRQ_MASK_CLEAR);
	__raw_writel(luts1_ints_fired, EPDC_IRQ_MASK1_CLEAR);
	__raw_writel(luts2_ints_fired, EPDC_IRQ_MASK2_CLEAR);

	dev_dbg(fb_data->dev, "EPDC interrupts fired = 0x%x, "
		"LUTS1 fired = 0x%x, LUTS2 fired = 0x%x\n",
		ints_fired, luts1_ints_fired, luts2_ints_fired);

	queue_work(fb_data->epdc_intr_workqueue,
		&fb_data->epdc_intr_work);

	return IRQ_HANDLED;
}

static void epdc_intr_work_func(struct work_struct *work)
{

	struct mxc_epdc_fb_data *fb_data =
		container_of(work, struct mxc_epdc_fb_data, epdc_intr_work);
	struct update_data_list *collision_update;
	struct mxcfb_rect *next_upd_region;
	struct update_marker_data *next_marker;
	struct update_marker_data *temp;
	int temp_index;
	u64 temp_mask;
	u32 lut;
	bool ignore_collision = false;
	int i;
	bool wb_lut_done = false;
	bool free_update = true;
	int next_lut, epdc_next_lut_15;
	u32 epdc_luts_active, epdc_wb_busy, epdc_luts_avail, epdc_lut_cancelled;
	u32 epdc_collision;
	u64 epdc_irq_stat;
	bool epdc_waiting_on_wb;
	u32 coll_coord, coll_size;
	struct mxcfb_rect coll_region;
	int cfa_scale = fb_data->use_cfa ? 2 : 1;
	
	mutex_lock(&fb_data->queue_mutex);

	/* Capture EPDC status one time to limit exposure to race conditions */
	epdc_luts_active = epdc_any_luts_active(fb_data->rev);
	epdc_wb_busy = epdc_is_working_buffer_busy();
	epdc_lut_cancelled = epdc_is_lut_cancelled();
	epdc_luts_avail = epdc_any_luts_available();
	epdc_collision = epdc_is_collision();
	if (fb_data->rev < 20)
		epdc_irq_stat = __raw_readl(EPDC_IRQ);
	else
		epdc_irq_stat = (u64)__raw_readl(EPDC_IRQ1) |
			((u64)__raw_readl(EPDC_IRQ2) << 32);
	epdc_waiting_on_wb = (fb_data->cur_update != NULL) ? true : false;

	/* Free any LUTs that have completed */
	for (i = 0; i < fb_data->num_luts; i++) {
		if ((epdc_irq_stat & (1ULL << i)) == 0)
			continue;

		dev_dbg(fb_data->dev, "LUT %d completed\n", i);

		/* Disable IRQ for completed LUT */
		epdc_lut_complete_intr(fb_data->rev, i, false);

		/*
		 * Go through all updates in the collision list and
		 * unmask any updates that were colliding with
		 * the completed LUT.
		 */
		list_for_each_entry(collision_update,
				    &fb_data->upd_buf_collision_list, list) {
			collision_update->collision_mask =
			    collision_update->collision_mask & ~(1 << i);
		}

		epdc_clear_lut_complete_irq(fb_data->rev, i);

		fb_data->luts_complete_wb |= 1ULL << i;

		fb_data->lut_update_order[i] = 0;

		/* Signal completion if submit workqueue needs a LUT */
		if (fb_data->waiting_for_lut) {
			complete(&fb_data->update_res_free);
			fb_data->waiting_for_lut = false;
		}

		/* Signal completion if LUT15 free and is needed */
		if (fb_data->waiting_for_lut15 && (i == 15)) {
			complete(&fb_data->lut15_free);
			fb_data->waiting_for_lut15 = false;
		}

		/* Detect race condition where WB and its LUT complete
		   (i.e. full update completes) in one swoop */
		if (epdc_waiting_on_wb &&
			(i == fb_data->cur_update->lut_num))
			wb_lut_done = true;

		/* Signal completion if anyone waiting on this LUT */
		if (!wb_lut_done)
			list_for_each_entry_safe(next_marker, temp,
				&fb_data->full_marker_list,
				full_list) {
				if (next_marker->lut_num != i)
					continue;

				/* Found marker to signal - remove from list */
				list_del_init(&next_marker->full_list);

				/* Signal completion of update */
				dev_dbg(fb_data->dev, "Signaling marker %d\n",
					next_marker->update_marker);
				if (next_marker->waiting) {
					list_add_tail(&next_marker->aw_list, &fb_data->awaiting_marker_list);
					complete(&next_marker->update_completion);
				} else {
					kfree(next_marker);
				}
			}
	}

	/* Check to see if all updates have completed */
	if (list_empty(&fb_data->upd_pending_list) &&
		is_free_list_full(fb_data) &&
		!epdc_waiting_on_wb &&
		!epdc_luts_active) {

		fb_data->updates_active = false;

		if (fb_data->pwrdown_delay != FB_POWERDOWN_DISABLE) {
			/*
			 * Set variable to prevent overlapping
			 * enable/disable requests
			 */
			fb_data->powering_down = true;

			/* Schedule task to disable EPDC HW until next update */
			schedule_delayed_work(&fb_data->epdc_done_work,
				msecs_to_jiffies(fb_data->pwrdown_delay));

			/* Reset counter to reduce chance of overflow */
			fb_data->order_cnt = 0;
		}

		if (fb_data->waiting_for_idle)
			complete(&fb_data->updates_done);
	}

	/* Is Working Buffer busy? */
	if (epdc_wb_busy) {
		/* Can't submit another update until WB is done */
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}

	/*
	 * Were we waiting on working buffer?
	 * If so, update queues and check for collisions
	 */
	if (epdc_waiting_on_wb) {
		dev_dbg(fb_data->dev, "\nWorking buffer completed\n");

		/* Signal completion if submit workqueue was waiting on WB */
		if (fb_data->waiting_for_wb) {
			complete(&fb_data->update_res_free);
			fb_data->waiting_for_wb = false;
		}

		if (fb_data->cur_update->update_desc->upd_data.flags
			& EPDC_FLAG_TEST_COLLISION) {
			/* This was a dry run to test for collision */

			/* Signal marker */
			list_for_each_entry_safe(next_marker, temp,
				&fb_data->full_marker_list,
				full_list) {
				if (next_marker->lut_num != DRY_RUN_NO_LUT)
					continue;

				if (epdc_collision)
					next_marker->collision_test = true;
				else
					next_marker->collision_test = false;

				dev_dbg(fb_data->dev,
					"In IRQ, collision_test = %d\n",
					next_marker->collision_test);

				/* Found marker to signal - remove from list */
				list_del_init(&next_marker->full_list);
				list_add_tail(&next_marker->aw_list, &fb_data->awaiting_marker_list);

				/* Signal completion of update */
				dev_dbg(fb_data->dev, "Signaling marker "
					"for dry-run - %d\n",
					next_marker->update_marker);
				complete(&next_marker->update_completion);
			}
		} else if (epdc_lut_cancelled && !epdc_collision) {
			/*
			* Note: The update may be cancelled (void) if all
			* pixels collided. In that case we handle it as a
			* collision, not a cancel.
			*/

			/* Clear LUT status (might be set if no AUTOWV used) */

			/*
			 * Disable and clear IRQ for the LUT used.
			 * Even though LUT is cancelled in HW, the LUT
			 * complete bit may be set if AUTOWV not used.
			 */
			epdc_lut_complete_intr(fb_data->rev,
					fb_data->cur_update->lut_num, false);
			epdc_clear_lut_complete_irq(fb_data->rev,
					fb_data->cur_update->lut_num);

			fb_data->lut_update_order[fb_data->cur_update->lut_num] = 0;

			/* Signal completion if submit workqueue needs a LUT */
			if (fb_data->waiting_for_lut) {
				complete(&fb_data->update_res_free);
				fb_data->waiting_for_lut = false;
			}

			list_for_each_entry_safe(next_marker, temp,
				&fb_data->cur_update->update_desc->upd_marker_list,
				upd_list) {

				/* Del from per-update & full list */
				list_del_init(&next_marker->upd_list);
				list_del_init(&next_marker->full_list);

				/* Signal completion of update */
				dev_dbg(fb_data->dev,
					"Signaling marker (cancelled) %d\n",
					next_marker->update_marker);
				if (next_marker->waiting) {
					list_add_tail(&next_marker->aw_list, &fb_data->awaiting_marker_list);
					complete(&next_marker->update_completion);
				} else {
					kfree(next_marker);
				}
			}
		} else if (epdc_collision) {
			/* Real update (no dry-run), collision occurred */

			/* Check list of colliding LUTs, and add to our collision mask */
			fb_data->cur_update->collision_mask =
			    fb_data->epdc_colliding_luts;

			/* Clear collisions that completed since WB began */
			fb_data->cur_update->collision_mask &=
				~fb_data->luts_complete_wb;

			dev_dbg(fb_data->dev, "Collision mask = 0x%llx\n",
			       fb_data->epdc_colliding_luts);

			/* For EPDC 2.0 and later, minimum collision bounds
			   are provided by HW.  Recompute new bounds here. */
			if ((fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT)
				&& (fb_data->rev >= 20)) {
				u32 xres, yres, rotate;
				struct mxcfb_rect *cur_upd_rect =
					&fb_data->cur_update->update_desc->upd_data.update_region;
				struct mxcfb_rect orig_rect;

				/* Get collision region coords from EPDC */
				coll_coord = __raw_readl(EPDC_UPD_COL_CORD);
				coll_size = __raw_readl(EPDC_UPD_COL_SIZE);
				coll_region.left =
					(coll_coord & EPDC_UPD_COL_CORD_XCORD_MASK)
						>> EPDC_UPD_COL_CORD_XCORD_OFFSET;
				coll_region.top =
					(coll_coord & EPDC_UPD_COL_CORD_YCORD_MASK)
						>> EPDC_UPD_COL_CORD_YCORD_OFFSET;
				coll_region.width =
					(coll_size & EPDC_UPD_COL_SIZE_WIDTH_MASK)
						>> EPDC_UPD_COL_SIZE_WIDTH_OFFSET;
				coll_region.height =
					(coll_size & EPDC_UPD_COL_SIZE_HEIGHT_MASK)
						>> EPDC_UPD_COL_SIZE_HEIGHT_OFFSET;
				dev_dbg(fb_data->dev, "Coll region: l = %d->%d, "
					"t = %d, w = %d, h = %d\n",
					coll_region.left, coll_region.left/** / - fb_data->cur_mode->left_border/ **/,
					coll_region.top,
					coll_region.width, coll_region.height);
				//coll_region.left -= fb_data->cur_mode->left_border;

				if (fb_data->use_cfa) {
					coll_region.left /= 2;
					coll_region.top /= 2;
					coll_region.width = (coll_region.width + 1) / 2;
					coll_region.height = (coll_region.height + 1) / 2;
				}

				/* Convert coords back to orig orientation */
				switch (fb_data->epdc_fb_var.rotate) {
				case FB_ROTATE_CW:
					xres = fb_data->epdc_fb_var.yres;
					yres = fb_data->epdc_fb_var.xres;
					rotate = FB_ROTATE_CCW;
					break;
				case FB_ROTATE_UD:
					xres = fb_data->epdc_fb_var.xres;
					yres = fb_data->epdc_fb_var.yres;
					rotate = FB_ROTATE_UD;
					break;
				case FB_ROTATE_CCW:
					xres = fb_data->epdc_fb_var.yres;
					yres = fb_data->epdc_fb_var.xres;
					rotate = FB_ROTATE_CW;
					break;
				default:
					xres = fb_data->epdc_fb_var.xres;
					yres = fb_data->epdc_fb_var.yres;
					rotate = FB_ROTATE_UR;
					break;
				}
				memcpy(&orig_rect, cur_upd_rect, sizeof(orig_rect));
				adjust_coordinates(xres, yres, rotate, fb_data->cur_mode->flip_top,
						&coll_region, cur_upd_rect);

				dev_dbg(fb_data->dev, "Adj coll region: l = %d, "
					"t = %d, w = %d, h = %d\n",
					cur_upd_rect->left, cur_upd_rect->top,
					cur_upd_rect->width,
					cur_upd_rect->height);

				/* Correct coordinates for alt source buffer if needed */
				if (fb_data->cur_update->update_desc->upd_data.flags & EPDC_FLAG_USE_ALT_BUFFER) {
					struct mxcfb_rect *alt_upd_rect =
						&fb_data->cur_update->update_desc->upd_data.alt_buffer_data.alt_update_region;

					/* TODO: check if signed operations are needed */
					alt_upd_rect->left += (int)cur_upd_rect->left - (int)orig_rect.left;
					alt_upd_rect->top += (int)cur_upd_rect->top - (int)orig_rect.top;
					alt_upd_rect->width = cur_upd_rect->width;
					alt_upd_rect->height = cur_upd_rect->height;
				}
			}

			/*
			 * If we collide with newer updates, then
			 * we don't need to re-submit the update. The
			 * idea is that the newer updates should take
			 * precedence anyways, so we don't want to
			 * overwrite them.
			 */
			for (temp_mask = fb_data->cur_update->collision_mask, lut = 0;
				temp_mask != 0;
				lut++, temp_mask = temp_mask >> 1) {
				if (!(temp_mask & 0x1))
					continue;

				if (fb_data->lut_update_order[lut] >=
					fb_data->cur_update->update_desc->update_order) {
					dev_dbg(fb_data->dev,
						"Ignoring collision with "
						"newer update.\n");
					ignore_collision = true;
					break;
				}
			}

			if (!ignore_collision) {
				free_update = false;
				/*
				 * If update has markers, clear the LUTs to
				 * avoid signalling that they have completed.
				 */
				list_for_each_entry_safe(next_marker, temp,
					&fb_data->cur_update->update_desc->upd_marker_list,
					upd_list)
					next_marker->lut_num = INVALID_LUT;

				/* Move to collision list */
				list_add_tail(&fb_data->cur_update->list,
					 &fb_data->upd_buf_collision_list);
			}
		}

		/* Do we need to free the current update descriptor? */
		if (free_update) {
			/* Handle condition where WB & LUT are both complete */
			if (wb_lut_done)
				list_for_each_entry_safe(next_marker, temp,
					&fb_data->cur_update->update_desc->upd_marker_list,
					upd_list) {

					/* Del from per-update & full list */
					list_del_init(&next_marker->upd_list);
					list_del_init(&next_marker->full_list);

					/* Signal completion of update */
					dev_dbg(fb_data->dev,
						"Signaling marker (wb) %d\n",
						next_marker->update_marker);
					if (next_marker->waiting) {
						list_add_tail(&next_marker->aw_list, &fb_data->awaiting_marker_list);
						complete(&next_marker->update_completion);
					} else {
						kfree(next_marker);
					}
				}

			/* Free marker list and update descriptor */
			kfree(fb_data->cur_update->update_desc);

			/* Add to free buffer list */
			list_add_tail(&fb_data->cur_update->list,
				 &fb_data->upd_buf_free_list);

			/* Check to see if all updates have completed */
			if (list_empty(&fb_data->upd_pending_list) &&
				is_free_list_full(fb_data) &&
				!epdc_luts_active) {

				fb_data->updates_active = false;

				if (fb_data->pwrdown_delay !=
						FB_POWERDOWN_DISABLE) {
					/*
					 * Set variable to prevent overlapping
					 * enable/disable requests
					 */
					fb_data->powering_down = true;

					/* Schedule EPDC disable */
					schedule_delayed_work(&fb_data->epdc_done_work,
						msecs_to_jiffies(fb_data->pwrdown_delay));

					/* Reset counter to reduce chance of overflow */
					fb_data->order_cnt = 0;
				}

				if (fb_data->waiting_for_idle)
					complete(&fb_data->updates_done);
			}
		}

		/* Clear current update */
		fb_data->cur_update = NULL;

		/* Clear IRQ for working buffer */
		epdc_working_buf_intr(false);
		epdc_clear_working_buf_irq();
	}

	if (fb_data->upd_scheme != UPDATE_SCHEME_SNAPSHOT) {
		/* Queued update scheme processing */

		/* Schedule task to submit collision and pending update */
		if (!fb_data->powering_down)
			queue_work(fb_data->epdc_submit_workqueue,
				&fb_data->epdc_submit_work);

		/* Release buffer queues */
		mutex_unlock(&fb_data->queue_mutex);

		return;
	}

	/* Snapshot update scheme processing */

	/* Check to see if any LUTs are free */
	if (!epdc_luts_avail) {
		dev_dbg(fb_data->dev, "No luts available.\n");
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}

	epdc_next_lut_15 = epdc_choose_next_lut(fb_data->rev, &next_lut);
	/* Check to see if there is a valid LUT to use */
	if (epdc_next_lut_15 && fb_data->tce_prevent && (fb_data->rev < 20)) {
		dev_dbg(fb_data->dev, "Must wait for LUT15\n");
		mutex_unlock(&fb_data->queue_mutex);
		return;
	}

	/*
	 * Are any of our collision updates able to go now?
	 * Go through all updates in the collision list and check to see
	 * if the collision mask has been fully cleared
	 */
	list_for_each_entry(collision_update,
			    &fb_data->upd_buf_collision_list, list) {

		if (collision_update->collision_mask != 0)
			continue;

		dev_dbg(fb_data->dev, "A collision update is ready to go!\n");
		/*
		 * We have a collision cleared, so select it
		 * and we will retry the update
		 */
		fb_data->cur_update = collision_update;
		list_del_init(&fb_data->cur_update->list);
		break;
	}

	/*
	 * If we didn't find a collision update ready to go,
	 * we try to grab one from the update queue
	 */
	if (fb_data->cur_update == NULL) {
		/* Is update list empty? */
		if (list_empty(&fb_data->upd_buf_queue)) {
			dev_dbg(fb_data->dev, "No pending updates.\n");

			/* No updates pending, so we are done */
			mutex_unlock(&fb_data->queue_mutex);
			return;
		} else {
			dev_dbg(fb_data->dev, "Found a pending update!\n");

			/* Process next item in update list */
			fb_data->cur_update =
			    list_entry(fb_data->upd_buf_queue.next,
				       struct update_data_list, list);
			list_del_init(&fb_data->cur_update->list);
		}
	}

	/* Use LUT selected above */
	fb_data->cur_update->lut_num = next_lut;

	/* Associate LUT with update markers */
	list_for_each_entry_safe(next_marker, temp,
		&fb_data->cur_update->update_desc->upd_marker_list, upd_list)
		next_marker->lut_num = fb_data->cur_update->lut_num;

	/* Mark LUT as containing new update */
	fb_data->lut_update_order[fb_data->cur_update->lut_num] =
		fb_data->cur_update->update_desc->update_order;

	/* Enable Collision and WB complete IRQs */
	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(fb_data->rev, fb_data->cur_update->lut_num, true);

	/* Program EPDC update to process buffer */
	next_upd_region =
		&fb_data->cur_update->update_desc->upd_data.update_region;
	if (fb_data->cur_update->update_desc->upd_data.temp
		!= TEMP_USE_AMBIENT) {
		temp_index = mxc_epdc_fb_get_temp_index(fb_data,
			fb_data->cur_update->update_desc->upd_data.temp);
		epdc_set_temp(temp_index);
	} else
		epdc_set_temp(fb_data->temp_index);
	epdc_set_update_addr(fb_data->cur_update->phys_addr +
				fb_data->cur_update->update_desc->epdc_offs);

	epdc_set_update_coord(
			next_upd_region->left * cfa_scale, 
			next_upd_region->top * cfa_scale);
	epdc_set_update_dimensions(
			next_upd_region->width * cfa_scale,
			next_upd_region->height * cfa_scale);
	if (fb_data->rev > 20)
		epdc_set_update_stride(fb_data->cur_update->update_desc->epdc_stride * cfa_scale);
	if (fb_data->wv_modes_update &&
		(fb_data->cur_update->update_desc->upd_data.waveform_mode
			== WAVEFORM_MODE_AUTO)) {
		epdc_set_update_waveform(&fb_data->wv_modes);
		fb_data->wv_modes_update = false;
	}

	epdc_submit_update(fb_data->cur_update->lut_num,
			   fb_data->cur_update->update_desc->upd_data.waveform_mode,
			   fb_data->cur_update->update_desc->upd_data.update_mode,
			   false, false, 0);

	/* Release buffer queues */
	mutex_unlock(&fb_data->queue_mutex);

	return;
}

static void draw_mode0(struct mxc_epdc_fb_data *fb_data)
{

	u32 *upd_buf_ptr;
	int i;
	u32 xres, yres;

	upd_buf_ptr = (u32 *)fb_data->info.screen_base;

	epdc_working_buf_intr(true);
	epdc_lut_complete_intr(fb_data->rev, 0, true);

	xres = fb_data->native_width;
	yres = fb_data->native_height;

	/* Program EPDC update to process buffer */
	epdc_set_update_addr(fb_data->phys_start);
	epdc_set_update_coord(0, 0);
	epdc_set_update_dimensions(xres, yres);
	if (fb_data->rev > 20) 
		epdc_set_update_stride(0);

	epdc_submit_update(0, fb_data->wv_modes.mode_init, UPDATE_MODE_FULL,
		false, true, 0xFF);

	dev_dbg(fb_data->dev, "Mode0 update - Waiting for LUT to complete...\n");

	/* Will timeout after ~4-5 seconds */

	for (i = 0; i < 40; i++) {
		if (!epdc_is_lut_active(0)) {
			dev_dbg(fb_data->dev, "Mode0 init complete\n");
			return;
		}
		msleep(100);
	}

	dev_err(fb_data->dev, "Mode0 init failed!\n");

	return;
}


static void mxc_epdc_fb_fw_handler(const struct firmware *fw,
						     void *context)
{
	struct mxc_epdc_fb_data *fb_data = context;
	int ret;
	struct mxcfb_waveform_data_file *wv_file;
	int wv_data_offs;
	int i;
	/*
	struct mxcfb_update_data update;
	struct mxcfb_update_marker_data upd_marker_data;
	* */
	struct fb_var_screeninfo *screeninfo = &fb_data->epdc_fb_var;
	u32 xres, yres;
	struct clk *epdc_parent;
	unsigned long rounded_parent_rate, epdc_pix_rate,
			rounded_pix_clk, target_pix_clk;
	
	if (fw == NULL) {
		/* If default FW file load failed, we give up */
		if (fb_data->fw_default_load){
			dev_err(fb_data->dev,
				"FW file load failed, so we give up\n");
			return;
		}
		/* Try to load default waveform */
		dev_info(fb_data->dev,
			"Can't find firmware. Trying fallback fw\n");
		fb_data->fw_default_load = true;
		ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			"imx/epdc.fw", fb_data->dev, GFP_KERNEL, fb_data,
			mxc_epdc_fb_fw_handler);
		if (ret)
			dev_err(fb_data->dev,
				"Failed request_firmware_nowait err %d\n", ret);

		return;
	}
	
	wv_file = (struct mxcfb_waveform_data_file *)fw->data;
	
	/* Get size and allocate temperature range table */
	fb_data->trt_entries = wv_file->wdh.trc + 1;
	fb_data->temp_range_bounds = kzalloc(fb_data->trt_entries, GFP_KERNEL);
	
	for (i = 0; i < fb_data->trt_entries; i++)
		dev_dbg(fb_data->dev, "trt entry #%d = 0x%x\n", i, *((u8 *)&wv_file->data + i));

	/* Copy TRT data */
	memcpy(fb_data->temp_range_bounds, &wv_file->data, fb_data->trt_entries);

	/* Set default temperature index using TRT and room temp */
	fb_data->temp_index = mxc_epdc_fb_get_temp_index(fb_data, DEFAULT_TEMP);

	/* Get offset and size for waveform data */
	wv_data_offs = sizeof(wv_file->wdh) + fb_data->trt_entries + 1;
	fb_data->waveform_buffer_size = fw->size - wv_data_offs;

	/* Allocate memory for waveform data */
	fb_data->waveform_buffer_virt = dma_alloc_coherent(fb_data->dev,
						fb_data->waveform_buffer_size,
						&fb_data->waveform_buffer_phys,
						GFP_DMA | GFP_KERNEL);
	if (fb_data->waveform_buffer_virt == NULL) {
		dev_err(fb_data->dev, "Can't allocate mem for waveform!\n");
		return;
	}

	memcpy(fb_data->waveform_buffer_virt, (u8 *)(fw->data) + wv_data_offs,
		fb_data->waveform_buffer_size);
	/******************************************************************/
	/**//* Read field to determine if 4-bit or 5-bit mode */		/**/
	/**/if ((wv_file->wdh.luts & 0xC) == 0x4){						/**/
	/**/	dev_dbg(fb_data->dev,"%s %iBit", __func__, EPDC_FORMAT_BUF_PIXEL_FORMAT_P5N);			/**/
	/**/	fb_data->buf_pix_fmt = EPDC_FORMAT_BUF_PIXEL_FORMAT_P5N;/**/
	/**/}else {														/**/
	/**/	dev_dbg(fb_data->dev,"%s %iBIT", __func__, EPDC_FORMAT_BUF_PIXEL_FORMAT_P4N);			/**/
	/**/	fb_data->buf_pix_fmt = EPDC_FORMAT_BUF_PIXEL_FORMAT_P4N;/**/
	/**/} 															/**/
	/******************************************************************/

	release_firmware(fw);

	/* Enable clocks to access EPDC regs */
	clk_prepare_enable(fb_data->epdc_clk_axi);

	target_pix_clk = fb_data->cur_mode->vmode->pixclock;
	rounded_pix_clk = clk_round_rate(fb_data->epdc_clk_pix, target_pix_clk);

	if (((rounded_pix_clk >= target_pix_clk + target_pix_clk/100) ||
		(rounded_pix_clk <= target_pix_clk - target_pix_clk/100))) {
		/* Can't get close enough without changing parent clk */
		epdc_parent = clk_get_parent(fb_data->epdc_clk_pix);
		rounded_parent_rate = clk_round_rate(epdc_parent, target_pix_clk);

		epdc_pix_rate = target_pix_clk;
		while (epdc_pix_rate < rounded_parent_rate)
			epdc_pix_rate *= 2;
		clk_set_rate(epdc_parent, epdc_pix_rate);

		rounded_pix_clk = clk_round_rate(fb_data->epdc_clk_pix, target_pix_clk);
		if (((rounded_pix_clk >= target_pix_clk + target_pix_clk/100) ||
			(rounded_pix_clk <= target_pix_clk - target_pix_clk/100)))
			/* Still can't get a good clock, provide warning */
			dev_err(fb_data->dev, "Unable to get an accurate EPDC pix clk"
				"desired = %lu, actual = %lu\n", target_pix_clk,
				rounded_pix_clk);
	}

	clk_set_rate(fb_data->epdc_clk_pix, rounded_pix_clk);
	/* Enable pix clk for EPDC */
	clk_prepare_enable(fb_data->epdc_clk_pix);

	epdc_init_sequence(fb_data);

	/* Disable clocks */
	clk_disable_unprepare(fb_data->epdc_clk_axi);
	clk_disable_unprepare(fb_data->epdc_clk_pix);

	fb_data->hw_ready = true;
	fb_data->hw_initializing = false;
	complete(&fb_data->init_completion);

	/* Use unrotated (native) width/height */
	if ((screeninfo->rotate == FB_ROTATE_CW) ||
		(screeninfo->rotate == FB_ROTATE_CCW)) {
		xres = screeninfo->yres;
		yres = screeninfo->xres;
	} else {
		xres = screeninfo->xres;
		yres = screeninfo->yres;
	}
#if 0
	update.update_region.left = 0;
	update.update_region.width = xres;
	update.update_region.top = 0;
	update.update_region.height = yres;
	update.update_mode = UPDATE_MODE_FULL;
	update.waveform_mode = WAVEFORM_MODE_GLD16;
	update.update_marker = INIT_UPDATE_MARKER;
	update.temp = TEMP_USE_AMBIENT;
	update.flags = 0;

	upd_marker_data.update_marker = update.update_marker;

	mxc_epdc_fb_send_update(&update, &fb_data->info);

	/* Block on initial update */
	ret = mxc_epdc_fb_wait_update_complete(&upd_marker_data,
		&fb_data->info, false);
	if (ret < 0)
		dev_err(fb_data->dev,
			"Wait for initial update complete failed."
			" Error = 0x%x", ret);
#endif			
}

static int mxc_epdc_fb_init_hw(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;
	int ret;
	/*
	 * Create fw search string based on ID string in selected videomode.
	 * Format is "imx/epdc_[panel string].fw"
	 */
	/*
	if (fb_data->cur_mode) {
		strcat(fb_data->fw_str, "imx/epdc_");
		strcat(fb_data->fw_str, fb_data->cur_mode->vmode->name);
		strcat(fb_data->fw_str, ".fw");
	}
	*/
	if (fb_data->cur_mode) {
		if(!strncmp(mxc_epdc_fb_waveform_modparam,"/",1)){
			strcpy(fb_data->fw_str, mxc_epdc_fb_waveform_modparam);
		}else if(!strcmp(mxc_epdc_fb_waveform_modparam,"")){
			strcpy(fb_data->fw_str, "imx/epdc_");
			strcat(fb_data->fw_str, fb_data->cur_mode->vmode->name);
			strcat(fb_data->fw_str, ".fw");
		}else {
			strcpy(fb_data->fw_str, "imx/");
			strcat(fb_data->fw_str, mxc_epdc_fb_waveform_modparam);
		}
	}
	dev_info(fb_data->dev,
				"Using FW: %s\n",fb_data->fw_str);
	fb_data->fw_default_load = false;

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				fb_data->fw_str, fb_data->dev, GFP_KERNEL,
				fb_data, mxc_epdc_fb_fw_handler);
	if (ret)
		dev_err(fb_data->dev,
			"Failed request_firmware_nowait err %d\n", ret);
#ifdef DEBUG
	dump_epdc_reg();
#endif
	
	return ret;
}

static ssize_t store_update(struct device *device,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{

	struct mxcfb_update_data update;
	struct fb_info *info = dev_get_drvdata(device);
	struct mxc_epdc_fb_data *fb_data = (struct mxc_epdc_fb_data *)info;

	if (strncmp(buf, "direct", 6) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_du;
	else if (strncmp(buf, "gc16", 4) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc16;
	else if (strncmp(buf, "gc4", 3) == 0)
		update.waveform_mode = fb_data->wv_modes.mode_gc4;

	/* Now, request full screen update */
	update.update_region.left = 0;
	update.update_region.width = fb_data->epdc_fb_var.xres;
	update.update_region.top = 0;
	update.update_region.height = fb_data->epdc_fb_var.yres;
	update.update_mode = UPDATE_MODE_FULL;
	update.temp = TEMP_USE_AMBIENT;
	update.update_marker = 0;
	update.flags = 0;

	mxc_epdc_fb_send_update(&update, info);

	return count;
}



static struct device_attribute fb_attrs[] = {
	__ATTR(update, S_IRUGO|S_IWUSR, NULL, store_update),
};

static struct imx_epdc_fb_mode *mxc_epdc_find_mode(
	struct mxc_epdc_fb_data *fb_data, const char *mode_str,
	const char *type_str)
{

	const size_t mode_str_len = strlen(mode_str);
	int i;
	for (i = 0; i < fb_data->pdata->num_modes; i++) {
		struct imx_epdc_fb_mode *mode =
			&fb_data->pdata->epdc_mode[i];
		char *mode_type_str;

		if (strncmp(mode->vmode->name, mode_str, mode_str_len))
			continue;

		if (!type_str)
			return (struct imx_epdc_fb_mode *) mode;

		if (mode->vmode->name[mode_str_len] != '.')
			continue;

		mode_type_str = &mode->vmode->name[mode_str_len + 1];

		if (strcmp(mode_type_str, type_str))
			continue;

		return (struct imx_epdc_fb_mode *) mode;
	}

	return NULL;
}

static const struct of_device_id imx_epdc_dt_ids[] = {
	{ .compatible = "fsl,imx6dl-epdc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_epdc_dt_ids);

int mxc_epdc_fb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pinctrl *pinctrl;
	struct mxc_epdc_fb_data *fb_data;
	struct resource *res;
	struct fb_info *info;
	char *options, *opt;
	char *panel_str = NULL;
	char name[] = "mxcepdcfb";
	struct fb_videomode *vmode;
	struct imx_epdc_fb_mode *found_mode = NULL;
	int xres_virt, yres_virt, buf_size;
	int cfa_scale = 1;
	int nvm_result = 0;
	int xres_virt_rot, yres_virt_rot, pix_size_rot;
	struct fb_var_screeninfo *var_info;
	struct fb_fix_screeninfo *fix_info;
	struct pxp_config_data *pxp_conf;
	struct pxp_proc_data *proc_data;
	struct scatterlist *sg;
	struct update_data_list *upd_list;
	struct update_data_list *plist, *temp_list;
	int i;
	unsigned long x_mem_size = 0;
	u32 val;
	int irq;
	
	struct device_node *gpio_np = NULL;
	struct hbz_gpio* hbz_gpio = NULL;
	hbz_gpio = kzalloc(sizeof(struct hbz_gpio) , GFP_KERNEL);
	fb_data = (struct mxc_epdc_fb_data *)framebuffer_alloc(
			sizeof(struct mxc_epdc_fb_data), &pdev->dev);
	if (fb_data == NULL) {
		ret = -ENOMEM;
		goto out;
	}
		
	/* Get platform data and check validity */
	fb_data->pdata = &epdc_data;
	if ((fb_data->pdata == NULL) || (fb_data->pdata->num_modes < 1)
		|| (fb_data->pdata->epdc_mode == NULL)
		|| (fb_data->pdata->epdc_mode->vmode == NULL)) {
		dev_err(fb_data->dev,"Get platform data and check validity: Invalid Platform Data!");
		ret = -EINVAL;
		goto out_fbdata;
	}

	if (fb_get_options(name, &options)) {
		ret = -ENODEV;
		goto out_fbdata;
	}

	fb_data->tce_prevent = 0;

	if (options)
		while ((opt = strsep(&options, ",")) != NULL) {
			if (!*opt)
				continue;

			if (!strncmp(opt, "bpp=", 4))
				fb_data->default_bpp =
					simple_strtoul(opt + 4, NULL, 0);
			else if (!strncmp(opt, "x_mem=", 6))
				x_mem_size = memparse(opt + 6, NULL);
			else if (!strncmp(opt, "tce_prevent", 11))
				fb_data->tce_prevent = 1;
			else
				panel_str = opt;
		}

	fb_data->dev = &pdev->dev;
	
	fb_data->no_powerdown = 0;
	
	if(!panel_str)
		dev_err(fb_data->dev,"No panel specified!");
		
	if (!fb_data->default_bpp)
		fb_data->default_bpp = 16;

	fb_data->v3p3_regulator[0] = devm_regulator_get(&pdev->dev, "V3P3_1");
	if (IS_ERR(fb_data->v3p3_regulator[0])) {
		dev_err(&pdev->dev, "Unable to get V3P3_1 regulator."
			"err = 0x%x\n", (int)fb_data->vcom_regulator[0]);
		ret = -ENODEV;
		//goto out_dma_work_buf;
	}

	fb_data->v3p3_regulator[1] = devm_regulator_get(&pdev->dev, "V3P3_2");
	if (IS_ERR(fb_data->v3p3_regulator[1])) {
		dev_err(&pdev->dev, "Unable to get V3P3 regulator."
					"err = %i\n", (int)fb_data->vcom_regulator[1]);
		ret = -ENODEV;
		//goto out_dma_work_buf;
	}
//*
	if (regulator_is_enabled(fb_data->v3p3_regulator[0]))
		regulator_disable(fb_data->v3p3_regulator[0]);

	if (regulator_is_enabled(fb_data->v3p3_regulator[1]))
		regulator_disable(fb_data->v3p3_regulator[1]);
//*/	

	/* Set default (first defined mode) before searching for a match */
	fb_data->cur_mode = &fb_data->pdata->epdc_mode[0];
	/* Check Display SPI Flash */
	if(mxc_epdc_fb_use_nvm_modparam)
		nvm_result = check_mtd_flash(fb_data);

	if(!(mxc_epdc_fb_use_nvm_modparam && nvm_result)){
	/* No NVM flash found */
	/*-----------------------------------------------------------------------------------------------------------------*/
		
		#if USE_ALL_PARAMETERS
		if(use_pedantic_conf_modparam){
			char name[] = "PedanticDisplaySettings";
			found_mode->vmode->name = (struct fb_videomode*) kmalloc(sizeof(name),GFP_KERNEL);
			strcpy(found_mode->vmode->name, name);
			found_mode = (struct imx_epdc_fb_mode*) kmalloc(sizeof(struct imx_epdc_fb_mode),GFP_KERNEL);
			vmode = (struct fb_videomode*) kmalloc(sizeof(struct fb_videomode),GFP_KERNEL);
			found_mode->vmode = vmode;
			found_mode->sddo_invert = ptype_sddo_invert_modparam;
			found_mode->sddo_shift_right = ptype_sddo_shift_right_modparam;
			found_mode->vscan_holdoff = ptype_vscan_holdoff_modparam;//1,
			found_mode->sdoed_width = ptype_sdoed_width_modparam; // 10,
			found_mode->sdoed_delay = ptype_sdoed_delay_modparam; // 0,
			found_mode->sdoez_width = ptype_sdoez_width_modparam; // 10,
			found_mode->sdoez_delay = ptype_sdoez_delay_modparam; // 20,
			found_mode->gdclk_hp_offs = ptype_gdclk_hp_offs_modparam; // 740,
			found_mode->gdsp_offs = ptype_gdsp_offs_modparam; // 2,
			found_mode->gdsp_frame_sync = ptype_gdsp_frame_sync_modparam; // false,
			found_mode->gdsp_active_high = ptype_gdsp_active_high_modparam; // false,
			found_mode->gdoe_offs = ptype_gdoe_offs_modparam; // 24,
			found_mode->gdoe_delayed_gclk = ptype_gdoe_delayed_gclk_modparam; // true,
			found_mode->gdoe_active_high = ptype_gdoe_active_high_modparam; // true,
			found_mode->gdclk_offs = ptype_gdclk_offs_modparam; // 4,
			found_mode->num_ce = ptype_num_ce_modparam; // 1,
			found_mode->sddo_16_bits = ptype_sddo_16_bits_modparam; // 0,
			found_mode->sddo_flip_bits = ptype_sddo_flip_bits_modparam; // false,
			found_mode->tft_4bpp = ptype_tft_4bpp_modparam; // false,
			found_mode->dual_scan = ptype_dual_scan_modparam; // false,
			found_mode->scan_dir_0_up = ptype_scan_dir_0_up_modparam; // false,
			found_mode->scan_dir_1_up = ptype_scan_dir_1_up_modparam; // false,
			found_mode->flip_top = ptype_flip_top_modparam; // false,
			found_mode->sdclk_hold = ptype_sdclk_hold_modparam; // false,
			//found_mode->left_border = ptype_left_border_modparam; // 0,
			//found_mode->right_border = ptype_right_border_modparam; // 0
			found_mode->vmode->refresh = vmode_refresh_modparam;//50,
			found_mode->vmode->xres = vmode_xres_modparam;//800,
			found_mode->vmode->yres = vmode_yres_modparam;//450,
			found_mode->vmode->pixclock = vmode_pixclock_modparam;//14424000,
			found_mode->vmode->left_margin = vmode_left_margin_modparam;//27, //* line_begin 
			found_mode->vmode->right_margin = vmode_right_margin_modparam;//88, //* line_end 
			found_mode->vmode->upper_margin = vmode_upper_margin_modparam;//4, //* frame_begin 
			found_mode->vmode->lower_margin = vmode_lower_margin_modparam;//4, //* frame_end 
			found_mode->vmode->hsync_len = vmode_hsync_len_modparam;//11,
			found_mode->vmode->vsync_len = vmode_vsync_len_modparam;//1,
			found_mode->vmode->sync = vmode_sync_modparam;//0,
			found_mode->vmode->vmode = vmode_vmode_modparam;//FB_VMODE_NONINTERLACED,
			found_mode->vmode->flag = vmode_flag_modparam;//0,	
		}else
		#endif
		{
		found_mode = mxc_epdc_find_mode(fb_data, 
						mxc_epdc_fb_panel_type_modparam, NULL);
		}
		
		fb_data->vcom[0] = mxc_epdc_fb_vcom_modparam[0];
		if(mxc_epdc_fb_vcom_n_modparam == 2){
			fb_data->vcom[1] = mxc_epdc_fb_vcom_modparam[1];
		}
	}else{
		dev_info(fb_data->dev,"Read Panel data from NVM\n");
			
		if(nvm_result & NVM_PANELTYPE_VALID && !strcmp(mxc_epdc_fb_panel_type_modparam, "")){
			// partno read
			if(!strcmp(fb_data->nvm.paneltype, "70022x_MG036")){
				strcpy(fb_data->nvm.paneltype, "D107_T3.1");
			}
			if((!strcmp(fb_data->nvm.paneltype, "S115_T1.1")) || (!strcmp(fb_data->nvm.paneltype, "S079_T1.1"))){
				mxc_epdc_fb_scrambling_modparam = 1;
			}
			if(!strcmp(fb_data->nvm.paneltype, "D107C_T3.1")){
				mxc_epdc_fb_use_cfa_modparam = 1;
			}
			if(fb_data->nvm.vcom[1] < 15000 && fb_data->nvm.vcom[1] > 2000){
				//Seems we have two displays attached
				if(!strcmp(fb_data->nvm.paneltype, "D107C_T3.1"))
					strcpy(fb_data->nvm.paneltype, "T154_T3.1");
				if(!strcmp(fb_data->nvm.paneltype, "D107_T3.1"))
					strcpy(fb_data->nvm.paneltype, "Q154_T3.1");
				if(!strcmp(fb_data->nvm.paneltype, "S115_T1.1"))
					strcpy(fb_data->nvm.paneltype, "S115_T1.3");
				if(!strcmp(fb_data->nvm.paneltype, "S079_T1.1"))
					strcpy(fb_data->nvm.paneltype, "S079_T1.3");
			}	
			dev_dbg(fb_data->dev,"Read fb_data->nvm.paneltype: %s\n",  fb_data->nvm.paneltype);
			found_mode = mxc_epdc_find_mode(fb_data, fb_data->nvm.paneltype, NULL);
		}else{
			found_mode = mxc_epdc_find_mode(fb_data, 
						mxc_epdc_fb_panel_type_modparam, NULL);
		}
		//dev_err(fb_data->dev,"%i, %i, i\n",nvm_result & NVM_VCOM_VALID, mxc_epdc_fb_vcom_modparam[0], mxc_epdc_fb_vcom_modparam[1]);
		if(nvm_result & NVM_VCOM_VALID && !check_vcom_valid(mxc_epdc_fb_vcom_modparam[0])){
			fb_data->vcom[0] = fb_data->nvm.vcom[0];
			dev_info(fb_data->dev,"VCOM[0] = %i\n", fb_data->vcom[0]);
			if(fb_data->nvm.vcom[1] < 15000 && fb_data->nvm.vcom[1] > 2000){
				fb_data->vcom[1] = fb_data->nvm.vcom[1];
				dev_info(fb_data->dev,"VCOM[1] = %i\n", fb_data->vcom[1]);
			}
		}else{
			if(check_vcom_valid(mxc_epdc_fb_vcom_modparam[0])){
				fb_data->vcom[0] = mxc_epdc_fb_vcom_modparam[0];
			}else{
				fb_data->vcom[0] = 5000;
				dev_err(fb_data->dev,"VCOM0 (%imV) invalid. Setting to 5000mV!\n", mxc_epdc_fb_vcom_modparam[0]);
			}
			if(mxc_epdc_fb_vcom_n_modparam == 2){
				if(check_vcom_valid(mxc_epdc_fb_vcom_modparam[1])){
					fb_data->vcom[1] = mxc_epdc_fb_vcom_modparam[1];
				}else{
					fb_data->vcom[1] = 5000;
					dev_err(fb_data->dev,"VCOM1 (%imV) invalid. Setting to 5000mV!\n", mxc_epdc_fb_vcom_modparam[0]);
				}
			}
		}
		if(nvm_result & NVM_WF_NAME_VALID && !strcmp(mxc_epdc_fb_waveform_modparam, "")){
			char wf_path[64];
			struct file* wf_file;
			strcpy(wf_path, "/lib/firmware/imx/");
			strcat(wf_path, fb_data->nvm.wf_name);
			strcat(wf_path, ".fw");
			dev_info(fb_data->dev,"Read WF name: %s\n", wf_path);

			if((nvm_result & NVM_WFLEN_VALID) && (nvm_result & NVM_WAVEFORM_VALID)){
#ifdef CONFIG_LZSS				
				char* decompressed_waveform = kzalloc(0x500000, GFP_KERNEL);

				int size;
				struct lzss lzss;
				lzss_init(&lzss, 7, 4);
				lzss_alloc_buffer(&lzss);
				lzss_decode_buffer(&lzss, fb_data->nvm.waveform, 
							fb_data->nvm.wflen[fb_data->nvm.wf_num], 
							decompressed_waveform, &size);
				wf_file = file_open(wf_path, O_WRONLY|O_CREAT, 0644);
				if(!wf_file){
					dev_err(fb_data->dev,"Cannot export waveform file %s.\n", wf_path);
				}else{
					int len = file_write(wf_file, 0, decompressed_waveform, size);
					if(len < 0){
						dev_err(fb_data->dev,"Cannot write to waveform file\n");
					}else{
						file_sync(wf_file);
						file_close(wf_file);
						strcpy(wf_path, fb_data->nvm.wf_name);
						strcat(wf_path, ".fw");
						mxc_epdc_fb_waveform_modparam = kzalloc(15, GFP_KERNEL);
						strcpy(mxc_epdc_fb_waveform_modparam,wf_path);
					}
				}
				lzss_free_buffer(&lzss);
#else
				dev_err(fb_data->dev, "No LZSS Algo found! Cannot decompress the NVM waveform!\n");
				dev_err(fb_data->dev, "Hope we find the Waveform on the FS!\n");
#endif
				dev_dbg(fb_data->dev,"Read WF successfully: %s\n", wf_path);
			}
			//*/
			strcpy(wf_path, fb_data->nvm.wf_name);
			strcat(wf_path, ".fw");
			mxc_epdc_fb_waveform_modparam = kzalloc(15, GFP_KERNEL);
			strcpy(mxc_epdc_fb_waveform_modparam,wf_path);
		}
	
	}
	if (found_mode){
		fb_data->cur_mode = (const struct imx_epdc_fb_mode*) found_mode;
		dev_info(fb_data->dev,"Selected Panel: %s\n", found_mode->vmode->name);
	}else{
		fb_data->cur_mode = &fb_data->pdata->epdc_mode[0];
		dev_err(fb_data->dev,"Failed to read Panel Identifier. Selected Panel: %s\n", fb_data->cur_mode->vmode->name);
	}
	//dump_fb_mode(fb_data);
	fb_data->dead_pixel_in_line = 0;
//*
	
	if(!strncmp("S115_T1", fb_data->cur_mode->vmode->name, 7)){
		epdc_max_update_width = 690;
		if(mxc_epdc_fb_scrambling_modparam){
			#if NO_DEAD_ZONES
			struct dead_zones* dz;
			int i, sum_dead_area, xres, number_horizontal_displays, remain, array_length;
			sum_dead_area = 0;
			xres = found_mode->vmode->xres;
			number_horizontal_displays = xres / 1024;
			remain = xres%1024;
			array_length = number_horizontal_displays;
			dev_dbg(fb_data->dev,"number_horizontal_displays: %i, remain: %i, array_length: %i\n",number_horizontal_displays, remain,array_length);
			dz = kzalloc(((2*array_length + 3) * sizeof(struct dead_zones)), GFP_KERNEL);
			for(i=0; i<array_length; i++){
				dz[2*i].l=i*1024;
				dz[2*i].w=167;
				dev_dbg(fb_data->dev,"dead zone: %i %i\n", dz[2*i].l, dz[2*i].l+dz[2*i].w);
				sum_dead_area += dz[2*i].w;
				dz[2*i+1].l=i*1024 + 857;
				dz[2*i+1].w=167;
				dev_dbg(fb_data->dev,"dead zone: %i %i\n", dz[2*i+1].l, dz[2*i+1].l+dz[2*i].w);
				
				sum_dead_area += dz[2*i+1].w;
			}
			if(remain>167){
				dz[2*i].l=i*1024;
				dz[2*i].w=167;
				dev_dbg(fb_data->dev,"dead zone: %i %i\n", dz[2*i].l, dz[2*i].l+dz[2*i].w);
				
				sum_dead_area += dz[2*i].w;
			}
			if(remain>857){
				dz[2*i+1].l=i*1024+857;
				dz[2*i+1].w=remain-857;
				dev_dbg(fb_data->dev,"dead zone: %i %i\n", dz[2*i].l, dz[2*i].l+dz[2*i].w);
				
				sum_dead_area += dz[2*i].w;
			}
			i++;
			dz[2*i].l = -1;
			dz[2*i].w = -1;
			dev_dbg(fb_data->dev,"dead zone: %i %i\n", dz[2*i].l, dz[2*i].l+dz[2*i].w);
			fb_data->dead_pixel_in_line = sum_dead_area;
			fb_data->dead_zones = dz;
			#endif
		}
		
	}
	if(!strncmp("S079_T1",  fb_data->cur_mode->vmode->name, 7)){
		epdc_max_update_width = 384;
		if(mxc_epdc_fb_scrambling_modparam){
			#if NO_DEAD_ZONES
			struct dead_zones* dz;
			int i, sum_dead_area, xres, number_horizontal_displays, remain, array_length;
			sum_dead_area = 0;
			xres = found_mode->vmode->xres;
			number_horizontal_displays = xres / 512;
			remain = xres%512;
			array_length = number_horizontal_displays;
			dz = kzalloc(((array_length + 2) * sizeof(struct dead_zones)), GFP_KERNEL);
			for(i=0; i<array_length; i++){
				dz[i].l=i*512 + 384;
				dz[i].w=128;
				dev_dbg(fb_data->dev,"dead zone: %i %i\n", dz[i].l, dz[i].l+dz[i].w);
				sum_dead_area += dz[i].w;
			}
			if(remain>384){
				dz[i].l=i*512 + 384;
				dz[i].w=remain - 384;
				dev_dbg(fb_data->dev,"dead zone: %i %i\n", dz[i].l, dz[i].l+dz[i].w);
				sum_dead_area += dz[i].w;
			}
			i++;
			dz[i].l = -1;
			dz[i].w = -1;
			dev_dbg(fb_data->dev,"dead zone: %i %i\n", dz[i].l, dz[i].l+dz[i].w);
			fb_data->dead_pixel_in_line = sum_dead_area;
			fb_data->dead_zones = dz;
			#endif
		}
	}
	
//*/ 
	/*
	 * Color Filter Array used (one color pixel is 2x2 square of b/w pixels
	 * with color filter
	 * Divide userspace-visible screen resolution by 2 at both dimensions
	 */
	fb_data->use_cfa = mxc_epdc_fb_use_cfa_modparam;
	if (fb_data->use_cfa){
		cfa_scale = 2;
	}	

	vmode = fb_data->cur_mode->vmode;

	platform_set_drvdata(pdev, fb_data);
	info = &fb_data->info;
	/* Allocate color map for the FB */
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret)
		goto out_fbdata;

	dev_info(&pdev->dev, "physical resolution %dx%d%s, bpp %d\n",
		vmode->xres / cfa_scale , vmode->yres / cfa_scale,
		fb_data->use_cfa ? "(CFA)" : "",  fb_data->default_bpp);

	/*
	 * GPU alignment restrictions dictate framebuffer parameters:
	 * - 32-byte alignment for buffer width
	 * - 128-byte alignment for buffer height
	 * => 4K buffer alignment for buffer start
	 */
	xres_virt = ALIGN(vmode->xres, 32);
	yres_virt = ALIGN(vmode->yres, 128);
	fb_data->max_pix_size = PAGE_ALIGN(xres_virt * yres_virt);
	dev_dbg(&pdev->dev, "fb_data->max_pix_size: %i", fb_data->max_pix_size);
	/*
	 * Have to check to see if aligned buffer size when rotated
	 * is bigger than when not rotated, and use the max
	 */
	xres_virt_rot = ALIGN(vmode->yres, 32);
	yres_virt_rot = ALIGN(vmode->xres, 128);
	pix_size_rot = PAGE_ALIGN(xres_virt_rot * yres_virt_rot);
	fb_data->max_pix_size = (fb_data->max_pix_size > pix_size_rot) ?
				fb_data->max_pix_size : pix_size_rot;

	buf_size = fb_data->max_pix_size * fb_data->default_bpp/8 / cfa_scale;
	/* Compute the number of screens needed based on X memory requested */
	if (x_mem_size > 0) {
		fb_data->num_screens = DIV_ROUND_UP(x_mem_size, buf_size);
		if (fb_data->num_screens < NUM_SCREENS_MIN)
			fb_data->num_screens = NUM_SCREENS_MIN;
		else if (buf_size * fb_data->num_screens > SZ_16M)
			fb_data->num_screens = SZ_16M / buf_size;
	} else
		fb_data->num_screens = NUM_SCREENS_MIN;

	fb_data->map_size = buf_size * fb_data->num_screens;
	fb_data->working_buffer_size = vmode->yres * vmode->xres * 2;

	fb_data->max_updates = (MXCFB_MAX_DMA_ALLOC - fb_data->map_size
				- fb_data->working_buffer_size);

	if (fb_data->max_pix_size)
		fb_data->max_updates /= (3 * fb_data->max_pix_size);

	if (fb_data->max_updates > CONFIG_FB_MXC_EPDC_MAX_UPDATES)
		fb_data->max_updates = CONFIG_FB_MXC_EPDC_MAX_UPDATES;

	dev_info(&pdev->dev, "Maximum number of concurrent updates: %d\n",
		 fb_data->max_updates);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENODEV;
		goto out_cmap;
	}

	epdc_base = devm_request_and_ioremap(&pdev->dev, res);
	if (epdc_base == NULL) {
		ret = -ENOMEM;
		goto out_cmap;
	}

	/* Allocate FB memory */
	info->screen_base = dma_alloc_writecombine(&pdev->dev,
						  fb_data->map_size,
						  &fb_data->phys_start,
						  GFP_DMA | GFP_KERNEL);

	if (info->screen_base == NULL) {
		ret = -ENOMEM;
		goto out_cmap;
	}
	dev_dbg(&pdev->dev, "allocated at %p:0x%x\n", info->screen_base,
		fb_data->phys_start);
	
	var_info = &info->var;
	var_info->activate = FB_ACTIVATE_TEST;
	var_info->bits_per_pixel = fb_data->default_bpp;
	var_info->xres = vmode->xres / cfa_scale * 
		((mxc_epdc_fb_scrambling_modparam==1)?2:1) - fb_data->dead_pixel_in_line * ((mxc_epdc_fb_scrambling_modparam==1)?2:1);
	
	var_info->yres = vmode->yres / cfa_scale /
		((mxc_epdc_fb_scrambling_modparam==1)?2:1);
	dev_dbg(&pdev->dev, "var_info->yres: %i / %i / %i\n",vmode->yres , cfa_scale , ((mxc_epdc_fb_scrambling_modparam==1)?2:1));
	var_info->xres_virtual = xres_virt / cfa_scale * 
		((mxc_epdc_fb_scrambling_modparam==1)?2:1);
	/* Additional screens allow for panning  and buffer flipping */
	var_info->yres_virtual = yres_virt / cfa_scale * fb_data->num_screens / ((mxc_epdc_fb_scrambling_modparam==1)?2:1);
	dev_dbg(&pdev->dev, "var_info->xres: %i, var_info->yres: %i, var_info->xres_virtual: %i, var_info->yres_virtual: %i \n", var_info->xres, var_info->yres, var_info->xres_virtual, var_info->yres_virtual);
	var_info->pixclock = vmode->pixclock;
	var_info->left_margin = vmode->left_margin;
	var_info->right_margin = vmode->right_margin;
	var_info->upper_margin = vmode->upper_margin;
	var_info->lower_margin = vmode->lower_margin;
	var_info->hsync_len = vmode->hsync_len;
	var_info->vsync_len = vmode->vsync_len;
	var_info->vmode = FB_VMODE_NONINTERLACED;
	dev_info(&pdev->dev, "virtual resolution: %ix%i\n" , var_info->xres, var_info->yres);
	switch (fb_data->default_bpp) {
	case 32:
	case 24:
		var_info->red.offset = 16;
		var_info->red.length = 8;
		var_info->green.offset = 8;
		var_info->green.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.length = 8;
		break;

	case 16:
		var_info->red.offset = 11;
		var_info->red.length = 5;
		var_info->green.offset = 5;
		var_info->green.length = 6;
		var_info->blue.offset = 0;
		var_info->blue.length = 5;
		break;

	case 8:
		/*
		 * For 8-bit grayscale, R, G, and B offset are equal.
		 *
		 */
		var_info->grayscale = GRAYSCALE_8BIT;

		var_info->red.length = 8;
		var_info->red.offset = 0;
		var_info->red.msb_right = 0;
		var_info->green.length = 8;
		var_info->green.offset = 0;
		var_info->green.msb_right = 0;
		var_info->blue.length = 8;
		var_info->blue.offset = 0;
		var_info->blue.msb_right = 0;
		break;

	default:
		dev_err(&pdev->dev, "unsupported bitwidth %d\n",
			fb_data->default_bpp);
		ret = -EINVAL;
		goto out_dma_fb;
	}

	fix_info = &info->fix;

	strcpy(fix_info->id, "mxc_epdc_fb");
	fix_info->type = FB_TYPE_PACKED_PIXELS;
	fix_info->visual = FB_VISUAL_TRUECOLOR;
	fix_info->xpanstep = 0;
	fix_info->ypanstep = 0;
	fix_info->ywrapstep = 0;
	fix_info->accel = FB_ACCEL_NONE;
	fix_info->smem_start = fb_data->phys_start;
	fix_info->smem_len = fb_data->map_size;
	fix_info->ypanstep = 0;

	fb_data->native_width = vmode->xres;
	fb_data->native_height = vmode->yres;
	fb_data->epdc_xres = (vmode->xres /*+ fb_data->cur_mode->left_border +
		fb_data->cur_mode->right_border*/);

	info->fbops = &mxc_epdc_fb_ops;
	info->var.activate = FB_ACTIVATE_NOW;
	info->pseudo_palette = fb_data->pseudo_palette;
	info->screen_size = info->fix.smem_len;
	info->flags = FBINFO_FLAG_DEFAULT;

	mxc_epdc_fb_set_fix(info);

	fb_data->auto_mode = AUTO_UPDATE_MODE_REGION_MODE;
	fb_data->upd_scheme = UPDATE_SCHEME_QUEUE_AND_MERGE;
	dev_dbg(fb_data->dev, "Initialize our internal copy of the screeninfo\n");
	/* Initialize our internal copy of the screeninfo */
	fb_data->epdc_fb_var = *var_info;
	fb_data->fb_offset = 0;
	fb_data->eof_sync_period = 0;

	fb_data->epdc_clk_axi = clk_get(fb_data->dev, "epdc_axi");
	if (IS_ERR(fb_data->epdc_clk_axi)) {
		dev_err(&pdev->dev, "Unable to get EPDC AXI clk."
			"err = %d\n", (int)fb_data->epdc_clk_axi);
		ret = -ENODEV;
		goto out_dma_fb;
	}
	fb_data->epdc_clk_pix = clk_get(fb_data->dev, "epdc_pix");
	if (IS_ERR(fb_data->epdc_clk_pix)) {
		dev_err(&pdev->dev, "Unable to get EPDC pix clk."
			"err = %d\n", (int)fb_data->epdc_clk_pix);
		ret = -ENODEV;
		goto out_dma_fb;
	}

	clk_prepare_enable(fb_data->epdc_clk_axi);
	val = __raw_readl(EPDC_VERSION);
	clk_disable_unprepare(fb_data->epdc_clk_axi);
	fb_data->rev = ((val & EPDC_VERSION_MAJOR_MASK) >>
				EPDC_VERSION_MAJOR_OFFSET) * 10
			+ ((val & EPDC_VERSION_MINOR_MASK) >>
				EPDC_VERSION_MINOR_OFFSET);
	dev_dbg(&pdev->dev, "EPDC version = %d\n", fb_data->rev);

	if (fb_data->rev < 20) {
		fb_data->num_luts = EPDC_V1_NUM_LUTS;
		fb_data->max_num_updates = EPDC_V1_MAX_NUM_UPDATES;
	} else {
		fb_data->num_luts = EPDC_V2_NUM_LUTS;
		fb_data->max_num_updates = EPDC_V2_MAX_NUM_UPDATES;
		if (vmode->xres > epdc_max_update_width * (mxc_epdc_fb_scrambling_modparam!=0?2:1))
			fb_data->restrict_width = true;
	}
	fb_data->max_num_buffers = EPDC_MAX_NUM_BUFFERS;

	/*
	 * Initialize lists for pending updates,
	 * active update requests, update collisions,
	 * and freely available updates.
	 */
	INIT_LIST_HEAD(&fb_data->upd_pending_list);
	INIT_LIST_HEAD(&fb_data->upd_buf_queue);
	INIT_LIST_HEAD(&fb_data->upd_buf_free_list);
	INIT_LIST_HEAD(&fb_data->upd_buf_collision_list);
	/* Allocate update buffers and add them to the list */
	for (i = 0; i < fb_data->max_num_updates; i++) {
		upd_list = kzalloc(sizeof(*upd_list), GFP_KERNEL);
		if (upd_list == NULL) {
			ret = -ENOMEM;
			goto out_upd_lists;
		}

		/* Add newly allocated buffer to free list */
		list_add(&upd_list->list, &fb_data->upd_buf_free_list);
	}

	fb_data->virt_addr_updbuf =
		kzalloc(sizeof(void *) * fb_data->max_num_buffers, GFP_KERNEL);
	fb_data->phys_addr_updbuf =
		kzalloc(sizeof(dma_addr_t) * fb_data->max_num_buffers,
			GFP_KERNEL);
	for (i = 0; i < fb_data->max_num_buffers; i++) {
		/*
		 * Allocate memory for PxP output buffer.
		 * Each update buffer is 1 byte per pixel, and can
		 * be as big as the full-screen frame buffer
		 */
		fb_data->virt_addr_updbuf[i] =
			dma_alloc_coherent(&pdev->dev, fb_data->max_pix_size,
				&fb_data->phys_addr_updbuf[i],
				GFP_DMA | GFP_KERNEL);

		if (fb_data->virt_addr_updbuf[i] == NULL) {
			ret = -ENOMEM;
			goto out_upd_buffers;
		}

		dev_dbg(&pdev->dev, "allocated %d bytes @ 0x%08X\n",
			fb_data->max_pix_size, fb_data->phys_addr_updbuf[i]);
	}

	/* Counter indicating which update buffer should be used next. */
	fb_data->upd_buffer_num = 0;

	/*
	 * Allocate memory for PxP SW workaround buffer
	 * These buffers are used to hold copy of the update region,
	 * before sending it to PxP for processing.
	 */
	fb_data->virt_addr_copybuf =
	    dma_alloc_coherent(fb_data->info.device, fb_data->max_pix_size*2,
			       &fb_data->phys_addr_copybuf,
			       GFP_DMA | GFP_KERNEL);
	if (fb_data->virt_addr_copybuf == NULL) {
		ret = -ENOMEM;
		goto out_upd_buffers;
	}

	/* Allocate memory for EPDC working buffer */
	fb_data->working_buffer_virt =
	    dma_alloc_coherent(&pdev->dev, fb_data->working_buffer_size,
			       &fb_data->working_buffer_phys,
			       GFP_DMA | GFP_KERNEL);
	if (fb_data->working_buffer_virt == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for working buf!\n");
		ret = -ENOMEM;
		goto out_copybuffer;
	}
	/* Initialize EPDC pins */
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "can't get/select pinctrl\n");
		ret = PTR_ERR(pinctrl);
		goto out_copybuffer;
	}

	fb_data->in_init = false;

	fb_data->hw_ready = false;
	fb_data->hw_initializing = false;

	/*
	 * Set default waveform mode values.
	 * Should be overwritten via ioctl.
	 */
	fb_data->wv_modes.mode_init = 0;
	fb_data->wv_modes.mode_du = 1;
	fb_data->wv_modes.mode_gc4 = 3;
	fb_data->wv_modes.mode_gc8 = 2;
	fb_data->wv_modes.mode_gc16 = 2;
	fb_data->wv_modes.mode_gc32 = 2;
	fb_data->wv_modes_update = true;

	/* Initialize marker list */
	INIT_LIST_HEAD(&fb_data->full_marker_list);
	INIT_LIST_HEAD(&fb_data->awaiting_marker_list);

	/* Initialize all LUTs to inactive */
	fb_data->lut_update_order =
		kzalloc(fb_data->num_luts * sizeof(u32 *), GFP_KERNEL);
	for (i = 0; i < fb_data->num_luts; i++)
		fb_data->lut_update_order[i] = 0;

	INIT_DELAYED_WORK(&fb_data->epdc_done_work, epdc_done_work_func);
	fb_data->epdc_submit_workqueue = alloc_workqueue("EPDC Submit",
					WQ_MEM_RECLAIM | WQ_HIGHPRI |
					WQ_CPU_INTENSIVE | WQ_UNBOUND, 1);
	INIT_WORK(&fb_data->epdc_submit_work, epdc_submit_work_func);
	fb_data->epdc_intr_workqueue = alloc_workqueue("EPDC Interrupt",
					WQ_MEM_RECLAIM | WQ_HIGHPRI |
					WQ_CPU_INTENSIVE | WQ_UNBOUND, 1);
	INIT_WORK(&fb_data->epdc_intr_work, epdc_intr_work_func);

	/* Retrieve EPDC IRQ num */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "cannot get IRQ resource\n");
		ret = -ENODEV;
		goto out_dma_work_buf;
	}
	fb_data->epdc_irq = irq;

	/* Register IRQ handler */
	ret = devm_request_irq(&pdev->dev, fb_data->epdc_irq,
				mxc_epdc_irq_handler, 0, "epdc", fb_data);
	if (ret) {
		dev_err(&pdev->dev, "request_irq (%d) failed with error %d\n",
			fb_data->epdc_irq, ret);
		ret = -ENODEV;
		goto out_dma_work_buf;
	}

	info->fbdefio = &mxc_epdc_fb_defio;
#ifdef CONFIG_FB_MXC_EINK_AUTO_UPDATE_MODE
	fb_deferred_io_init(info);
#endif
	fb_data->display_regulator[0] = devm_regulator_get(&pdev->dev, "DISPLAY1");
	if (IS_ERR(fb_data->display_regulator[0])) {
		dev_err(&pdev->dev, "Unable to get display PMIC1 regulator."
			"err = 0x%x\n", (int)fb_data->display_regulator[0]);
		ret = -ENODEV;
		goto out_dma_work_buf;
	}
	fb_data->vcom_regulator[0] = devm_regulator_get(&pdev->dev, "VCOM1");
	if (IS_ERR(fb_data->vcom_regulator)) {
		dev_err(&pdev->dev, "Unable to get VCOM1 regulator."
			"err = 0x%x\n", (int)fb_data->vcom_regulator[0]);
		ret = -ENODEV;
		goto out_dma_work_buf;
	}
	
	
	//if(fb_data->cur_mode->dual_scan){
		dev_dbg(&pdev->dev, "Initialize second PMIC for dual scan "
			"configuration\n");
			fb_data->display_regulator[1] = devm_regulator_get(&pdev->dev, "DISPLAY2");
		if (IS_ERR(fb_data->display_regulator[1])) {
			dev_err(&pdev->dev, "Unable to get display DISPLAY2 regulator."
				"err = %i\n", (int)fb_data->display_regulator[1]);
			ret = -ENODEV;
			//goto out_dma_work_buf;
		}
		fb_data->vcom_regulator[1] = devm_regulator_get(&pdev->dev, "VCOM2");
		if (IS_ERR(fb_data->vcom_regulator)) {
			dev_err(&pdev->dev, "Unable to get VCOM2 regulator."
				"err = %i\n", (int)fb_data->vcom_regulator[1]);
			ret = -ENODEV;
			//goto out_dma_work_buf;
		}

		
	//}
//#endif
#if 1

	gpio_np = of_find_node_by_name(gpio_np, "epdc");
	if (!gpio_np) {
		dev_err(&pdev->dev, "could not find gpio sub-node\n");
		return -ENODEV;
	}
   
	hbz_gpio->gpio_vcom_sw1 = of_get_named_gpio(gpio_np,
					"VCOM_SW1_gpio", 0);
    if (!gpio_is_valid(hbz_gpio->gpio_vcom_sw1)) {
		dev_err(&pdev->dev, "no VCOM_SW1_gpio pin available\n");
		goto out_dma_work_buf;
	}
    
	ret = devm_gpio_request_one(&pdev->dev, hbz_gpio->gpio_vcom_sw1,
				GPIOF_OUT_INIT_LOW, "VCOM_SW1");
        
	if (ret < 0){
		dev_err(&pdev->dev, "Cannot claim GPIO %d (err: %i)\n", hbz_gpio->gpio_vcom_sw1, ret);
		hbz_gpio->gpio_vcom_sw1 = 0;
		goto out_dma_work_buf;
	}else{
		gpio_set_value(hbz_gpio->gpio_vcom_sw1, 1);
	}
  
	hbz_gpio->gpio_vcom_sw2 = of_get_named_gpio(gpio_np,
					"VCOM_SW2_gpio", 0);
	if (!gpio_is_valid(hbz_gpio->gpio_vcom_sw2)) {
		dev_err(&pdev->dev, "no VCOM_SW2_gpio pin available\n");
		goto out_dma_work_buf;
	}
	ret = devm_gpio_request_one(&pdev->dev, hbz_gpio->gpio_vcom_sw2,
				GPIOF_OUT_INIT_LOW, "VCOM_SW2");
	if (ret < 0){
		dev_err(&pdev->dev, "Cannot claim GPIO %d (err: %i)\n", hbz_gpio->gpio_vcom_sw2, ret);
		hbz_gpio->gpio_vcom_sw2 = 0;
		goto out_dma_work_buf;
	}else{
		gpio_set_value(hbz_gpio->gpio_vcom_sw2, 1);
	}

	hbz_gpio->gpio_vsource_low1 = of_get_named_gpio(gpio_np,
					"VS_LOW_SW1_gpio", 0);
	if (!gpio_is_valid(hbz_gpio->gpio_vsource_low1)) {
		dev_err(&pdev->dev, "no VS_LOW_SW1_gpio pin available\n");
		goto out_dma_work_buf;
	}
	ret = devm_gpio_request_one(&pdev->dev, hbz_gpio->gpio_vsource_low1,
				GPIOF_OUT_INIT_LOW, "VS_LOW_SW1");
	if (ret < 0){
		dev_err(&pdev->dev, "Cannot claim GPIO %d (err: %i)\n", hbz_gpio->gpio_vsource_low1, ret);
		hbz_gpio->gpio_vsource_low1 = 0;
		goto out_dma_work_buf;
	}else{
		gpio_set_value(hbz_gpio->gpio_vsource_low1, 0);
	}
	
	hbz_gpio->gpio_vsource_low2 = of_get_named_gpio(gpio_np,
					"VS_LOW_SW2_gpio", 0);
	if (!gpio_is_valid(hbz_gpio->gpio_vsource_low2)) {
		dev_err(&pdev->dev, "no VS_LOW_SW2_gpio pin available\n");
		goto out_dma_work_buf;
	}
	ret = devm_gpio_request_one(&pdev->dev, hbz_gpio->gpio_vsource_low2,
				GPIOF_OUT_INIT_LOW, "VS_LOW_SW2");
	if (ret < 0){
		dev_err(&pdev->dev, "Cannot claim GPIO %d (err: %i)\n", hbz_gpio->gpio_vsource_low2, ret);
		hbz_gpio->gpio_vsource_low2 = 0;
		goto out_dma_work_buf;
	}else{
		gpio_set_value(hbz_gpio->gpio_vsource_low2, 0);
	}
    	
	hbz_gpio->gpio_vsource_high1 = of_get_named_gpio(gpio_np,
					"VS_HIGH_SW1_gpio", 0);
	if (!gpio_is_valid(hbz_gpio->gpio_vsource_high1)) {
		dev_err(&pdev->dev, "no VS_HIGH_SW1_gpio pin available\n");
		goto out_dma_work_buf;
	}
	
	ret = devm_gpio_request_one(&pdev->dev, hbz_gpio->gpio_vsource_high1,
				GPIOF_OUT_INIT_HIGH, "VS_HIGH_SW1");
	if (ret < 0){
		dev_err(&pdev->dev, "Cannot claim GPIO %d (err: %i)\n", hbz_gpio->gpio_vsource_high1, ret);
		hbz_gpio->gpio_vsource_high1 = 0;
		goto out_dma_work_buf;
	}else{
		gpio_set_value(hbz_gpio->gpio_vsource_high1, 0);
	}
		
	hbz_gpio->gpio_vsource_high2 = of_get_named_gpio(gpio_np,
					"VS_HIGH_SW2_gpio", 0);
	if (!gpio_is_valid(hbz_gpio->gpio_vsource_high2)) {
		dev_err(&pdev->dev, "no VS_HIGH_SW2_gpio pin available\n");
		hbz_gpio->gpio_vsource_high1 = 0;
		goto out_dma_work_buf;
	}
	ret = devm_gpio_request_one(&pdev->dev, hbz_gpio->gpio_vsource_high2,
				GPIOF_OUT_INIT_HIGH, "VS_HIGH_SW2");
	if (ret < 0){
		dev_err(&pdev->dev, "Cannot claim GPIO %d (err: %i)\n", hbz_gpio->gpio_vsource_high2, ret);
		hbz_gpio->gpio_vsource_high2 = 0;
		goto out_dma_work_buf;
	}else{
		gpio_set_value(hbz_gpio->gpio_vsource_high2, 0);
	}
	
//*/
	fb_data->hbz_gpio = hbz_gpio;
#endif	
	if (device_create_file(info->dev, &fb_attrs[0]))
		dev_err(&pdev->dev, "Unable to create file from fb_attrs\n");

	fb_data->cur_update = NULL;

	mutex_init(&fb_data->queue_mutex);
	mutex_init(&fb_data->pxp_mutex);
	mutex_init(&fb_data->power_mutex);

	/*
	 * Fill out PxP config data structure based on FB info and
	 * processing tasks required
	 */
	pxp_conf = &fb_data->pxp_conf;
	proc_data = &pxp_conf->proc_data;

	/* Initialize non-channel-specific PxP parameters */
	proc_data->drect.left = proc_data->srect.left = 0;
	proc_data->drect.top = proc_data->srect.top = 0;
	proc_data->drect.width = proc_data->srect.width = fb_data->info.var.xres;
	proc_data->drect.height = proc_data->srect.height = fb_data->info.var.yres;
	proc_data->scaling = 0;
	proc_data->hflip = 0;
	proc_data->vflip = 0;
	proc_data->rotate = 0;
	proc_data->bgcolor = 0;
	proc_data->overlay_state = 0;
	proc_data->lut_transform = PXP_LUT_NONE;
	proc_data->lut_map = NULL;

	/*
	 * We initially configure PxP for RGB->YUV conversion,
	 * and only write out Y component of the result.
	 */

	/*
	 * Initialize S0 channel parameters
	 * Parameters should match FB format/width/height
	 */
	pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_RGB565;
	pxp_conf->s0_param.width = fb_data->info.var.xres_virtual;
	pxp_conf->s0_param.height = fb_data->info.var.yres;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;

	/*
	 * Initialize OL0 channel parameters
	 * No overlay will be used for PxP operation
	 */
	for (i = 0; i < 8; i++) {
		pxp_conf->ol_param[i].combine_enable = false;
		pxp_conf->ol_param[i].width = 0;
		pxp_conf->ol_param[i].height = 0;
		pxp_conf->ol_param[i].pixel_fmt = PXP_PIX_FMT_RGB565;
		pxp_conf->ol_param[i].color_key_enable = false;
		pxp_conf->ol_param[i].color_key = -1;
		pxp_conf->ol_param[i].global_alpha_enable = false;
		pxp_conf->ol_param[i].global_alpha = 0;
		pxp_conf->ol_param[i].local_alpha_enable = false;
	}

	/*
	 * Initialize Output channel parameters
	 * Output is Y-only greyscale
	 * Output width/height will vary based on update region size
	 */
	pxp_conf->out_param.width = fb_data->epdc_xres;
	pxp_conf->out_param.height = fb_data->info.var.yres;
	pxp_conf->out_param.stride = pxp_conf->out_param.width;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_GREY;


	/* Initialize color map for conversion of 8-bit gray pixels */
	fb_data->pxp_conf.proc_data.lut_map = kmalloc(256, GFP_KERNEL);
	if (fb_data->pxp_conf.proc_data.lut_map == NULL) {
		dev_err(&pdev->dev, "Can't allocate mem for lut map!\n");
		ret = -ENOMEM;
		goto out_dma_work_buf;
	}
	for (i = 0; i < 256; i++)
		fb_data->pxp_conf.proc_data.lut_map[i] = i;

	fb_data->pxp_conf.proc_data.lut_map_updated = true;

	/*
	 * Ensure this is set to NULL here...we will initialize pxp_chan
	 * later in our thread.
	 */
	fb_data->pxp_chan = NULL;

	/* Initialize Scatter-gather list containing 2 buffer addresses. */
	sg = fb_data->sg;
	sg_init_table(sg, 2);

	/*
	 * For use in PxP transfers:
	 * sg[0] holds the FB buffer pointer
	 * sg[1] holds the Output buffer pointer (configured before TX request)
	 */
	sg_dma_address(&sg[0]) = info->fix.smem_start;
	sg_set_page(&sg[0], virt_to_page(info->screen_base),
		    info->fix.smem_len, offset_in_page(info->screen_base));
	fb_data->order_cnt = 0;
	fb_data->waiting_for_wb = false;
	fb_data->waiting_for_lut = false;
	fb_data->waiting_for_lut15 = false;
	fb_data->waiting_for_idle = false;
	fb_data->blank = FB_BLANK_UNBLANK;
	fb_data->power_state = POWER_STATE_OFF;
	fb_data->powering_down = false;
	fb_data->wait_for_powerdown = false;
	fb_data->updates_active = false;
	fb_data->pwrdown_delay = 0;

	/* Register FB */
	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&pdev->dev,
			"register_framebuffer failed with error %d\n", ret);
		goto out_lutmap;
	}

	g_fb_data = fb_data;

	pm_runtime_enable(fb_data->dev);

#ifdef DEFAULT_PANEL_HW_INIT
	fb_data->hw_initializing = true;
	init_completion(&fb_data->init_completion);

	ret = mxc_epdc_fb_init_hw((struct fb_info *)fb_data);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize HW!\n");
	}

#endif

	goto out;

out_lutmap:
	kfree(fb_data->pxp_conf.proc_data.lut_map);
out_dma_work_buf:
	dma_free_writecombine(&pdev->dev, fb_data->working_buffer_size,
		fb_data->working_buffer_virt, fb_data->working_buffer_phys);
out_copybuffer:
	dma_free_writecombine(&pdev->dev, fb_data->max_pix_size*2,
			      fb_data->virt_addr_copybuf,
			      fb_data->phys_addr_copybuf);
out_upd_buffers:
	for (i = 0; i < fb_data->max_num_buffers; i++)
		if (fb_data->virt_addr_updbuf[i] != NULL)
			dma_free_coherent(&pdev->dev, fb_data->max_pix_size,
					fb_data->virt_addr_updbuf[i],
					fb_data->phys_addr_updbuf[i]);
	if (fb_data->virt_addr_updbuf != NULL)
		kfree(fb_data->virt_addr_updbuf);
	if (fb_data->phys_addr_updbuf != NULL)
		kfree(fb_data->phys_addr_updbuf);
out_upd_lists:
	list_for_each_entry_safe(plist, temp_list, &fb_data->upd_buf_free_list,
			list) {
		list_del(&plist->list);
		kfree(plist);
	}
out_dma_fb:
	dma_free_writecombine(&pdev->dev, fb_data->map_size, info->screen_base,
			      fb_data->phys_start);

out_cmap:
	fb_dealloc_cmap(&info->cmap);
out_fbdata:
	kfree(fb_data);
out:

	return ret;
}

static int mxc_epdc_fb_remove(struct platform_device *pdev)
{

	struct update_data_list *plist, *temp_list;
	struct mxc_epdc_fb_data *fb_data = platform_get_drvdata(pdev);
	int i;
/*
	if(fb_data->nvm){
		put_mtd_device(fb_data->nvm);
	}
*/
	mxc_epdc_fb_blank(FB_BLANK_POWERDOWN, &fb_data->info);

	flush_workqueue(fb_data->epdc_submit_workqueue);
	destroy_workqueue(fb_data->epdc_submit_workqueue);

	unregister_framebuffer(&fb_data->info);

	for (i = 0; i < fb_data->max_num_buffers; i++)
		if (fb_data->virt_addr_updbuf[i] != NULL)
			dma_free_coherent(&pdev->dev, fb_data->max_pix_size,
					fb_data->virt_addr_updbuf[i],
					fb_data->phys_addr_updbuf[i]);
	if (fb_data->virt_addr_updbuf != NULL)
		kfree(fb_data->virt_addr_updbuf);
	if (fb_data->phys_addr_updbuf != NULL)
		kfree(fb_data->phys_addr_updbuf);

	dma_free_writecombine(&pdev->dev, fb_data->working_buffer_size,
				fb_data->working_buffer_virt,
				fb_data->working_buffer_phys);
	if (fb_data->waveform_buffer_virt != NULL)
		dma_free_writecombine(&pdev->dev, fb_data->waveform_buffer_size,
				fb_data->waveform_buffer_virt,
				fb_data->waveform_buffer_phys);
	if (fb_data->virt_addr_copybuf != NULL)
		dma_free_writecombine(&pdev->dev, fb_data->max_pix_size*2,
				fb_data->virt_addr_copybuf,
				fb_data->phys_addr_copybuf);
	list_for_each_entry_safe(plist, temp_list, &fb_data->upd_buf_free_list,
			list) {
		list_del(&plist->list);
		kfree(plist);
	}
#ifdef CONFIG_FB_MXC_EINK_AUTO_UPDATE_MODE
	fb_deferred_io_cleanup(&fb_data->info);
#endif

	dma_free_writecombine(&pdev->dev, fb_data->map_size, fb_data->info.screen_base,
			      fb_data->phys_start);

	/* Release PxP-related resources */
	if (fb_data->pxp_chan != NULL)
		dma_release_channel(&fb_data->pxp_chan->dma_chan);

	fb_dealloc_cmap(&fb_data->info.cmap);

	framebuffer_release(&fb_data->info);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mxc_epdc_fb_suspend(struct device *dev)
{

	struct mxc_epdc_fb_data *data = dev_get_drvdata(dev);
	int ret;

	data->pwrdown_delay = FB_POWERDOWN_DISABLE;
	ret = mxc_epdc_fb_blank(FB_BLANK_POWERDOWN, &data->info);
	if (ret)
		goto out;

out:
	return ret;
}

static int mxc_epdc_fb_resume(struct device *dev)
{

	struct mxc_epdc_fb_data *data = dev_get_drvdata(dev);

	mxc_epdc_fb_blank(FB_BLANK_UNBLANK, &data->info);
	epdc_init_settings(data);
	data->updates_active = false;

	return 0;
}
#else
#define mxc_epdc_fb_suspend	NULL
#define mxc_epdc_fb_resume	NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int mxc_epdc_fb_runtime_suspend(struct device *dev)
{

	release_bus_freq(BUS_FREQ_HIGH);
	dev_dbg(dev, "epdc busfreq high release.\n");

	return 0;
}

static int mxc_epdc_fb_runtime_resume(struct device *dev)
{

	request_bus_freq(BUS_FREQ_HIGH);
	dev_dbg(dev, "epdc busfreq high request.\n");

	return 0;
}
#else
#define mxc_epdc_fb_runtime_suspend	NULL
#define mxc_epdc_fb_runtime_resume	NULL
#endif

static const struct dev_pm_ops mxc_epdc_fb_pm_ops = {
	SET_RUNTIME_PM_OPS(mxc_epdc_fb_runtime_suspend,
				mxc_epdc_fb_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(mxc_epdc_fb_suspend, mxc_epdc_fb_resume)
};

static void mxc_epdc_fb_shutdown(struct platform_device *pdev)
{
	struct mxc_epdc_fb_data *fb_data = platform_get_drvdata(pdev);

	/* Disable power to the EPD panel */
	if (regulator_is_enabled(fb_data->vcom_regulator[0]))
		regulator_disable(fb_data->vcom_regulator[0]);
	if (regulator_is_enabled(fb_data->display_regulator[0]))
		regulator_disable(fb_data->display_regulator[0]);
	if(fb_data->cur_mode->dual_scan){
		if (regulator_is_enabled(fb_data->vcom_regulator[1]))
		regulator_disable(fb_data->vcom_regulator[1]);
		if (regulator_is_enabled(fb_data->display_regulator[1]))
			regulator_disable(fb_data->display_regulator[1]);
	}

	/* Disable clocks to EPDC */
	clk_prepare_enable(fb_data->epdc_clk_axi);
	clk_prepare_enable(fb_data->epdc_clk_pix);
	__raw_writel(EPDC_CTRL_CLKGATE, EPDC_CTRL_SET);
	clk_disable_unprepare(fb_data->epdc_clk_pix);
	clk_disable_unprepare(fb_data->epdc_clk_axi);

	/* turn off the V3p3 */
#if 0
	int ret = regulator_enable(fb_data->v3p3_regulator[0]);
	if (IS_ERR((void *)ret)) {
		dev_err(fb_data->dev, "Unable to enable V3P3_1 regulator."
			"err = 0x%x\n", ret);
		mutex_unlock(&fb_data->power_mutex);
		return;
	}
	if(fb_data->cur_mode->dual_scan){
		ret = regulator_enable(fb_data->v3p3_regulator[1]);
		if (IS_ERR((void *)ret)) {
			dev_err(fb_data->dev, "Unable to enable V3P3_2 regulator."
				"err = 0x%x\n", ret);
			mutex_unlock(&fb_data->power_mutex);
			return;
		}
	}
#else
	if (regulator_is_enabled(fb_data->v3p3_regulator[0]))
		regulator_disable(fb_data->v3p3_regulator[0]);
	if (regulator_is_enabled(fb_data->v3p3_regulator[1]))
		regulator_disable(fb_data->v3p3_regulator[1]);
#endif
}

static struct platform_driver mxc_epdc_fb_driver = {
	.probe = mxc_epdc_fb_probe,
	.remove = mxc_epdc_fb_remove,
	.shutdown = mxc_epdc_fb_shutdown,
	.driver = {
		   .name = "imx_epdc_fb",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(imx_epdc_dt_ids),
		   .pm = &mxc_epdc_fb_pm_ops,
		   },
};

/* Callback function triggered after PxP receives an EOF interrupt */
static void pxp_dma_done(void *arg)
{	
	struct pxp_tx_desc *tx_desc = to_tx_desc(arg);
	struct dma_chan *chan = tx_desc->txd.chan;
	struct pxp_channel *pxp_chan = to_pxp_channel(chan);
	struct mxc_epdc_fb_data *fb_data = pxp_chan->client;
	/* This call will signal wait_for_completion_timeout() in send_buffer_to_pxp */
	complete(&fb_data->pxp_tx_cmpl);
}

static bool chan_filter(struct dma_chan *chan, void *arg)
{

	if (imx_dma_is_pxp(chan))
		return true;
	else
		return false;
}

/* Function to request PXP DMA channel */
static int pxp_chan_init(struct mxc_epdc_fb_data *fb_data)
{
	dma_cap_mask_t mask;
	struct dma_chan *chan; 
	/*
	 * Request a free channel
	 */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_PRIVATE, mask);
	chan = dma_request_channel(mask, chan_filter, NULL);
	if (!chan) {
		dev_err(fb_data->dev, "Unsuccessfully received channel!!!!\n");
		return -EBUSY;
	}

	fb_data->pxp_chan = to_pxp_channel(chan);
	fb_data->pxp_chan->client = fb_data;

	init_completion(&fb_data->pxp_tx_cmpl);
	
	return 0;
}

static int copy_update(struct mxc_epdc_fb_data *fb_data,
			      void *src_buf_virt, void *dst_buf_virt,
			      struct mxcfb_rect *src_update_region)
{
	uint16_t * src = (uint16_t *)src_buf_virt + src_update_region->top * src_update_region->width + src_update_region->left;
	memcpy(dst_buf_virt, src, src_update_region->width * src_update_region->height * sizeof(uint16_t));
	return 0;
}

static uint16_t truncate_pixel(uint16_t pixel, uint16_t mask){
	return pixel & mask;
}

static int regal_process_update(struct mxc_epdc_fb_data *fb_data,
			      void *src_buf_virt, struct mxcfb_rect *src_update_region)
{
	uint16_t *src, *srcp, *prev, *prevp;
	//uint16_t *dst_line_start, *dst;
	int16_t *top_neighbor,*bot_neighbor,*left_neighbor,*right_neighbor;
	int i,j;
	uint16_t mask, regal_offset, do_regal;
	int dst_stride = 0;
#ifdef DEBUG
	ktime_t t1, t2;
#endif
	
#ifdef DEBUG
	t1 = ktime_get();
#endif

	if(fb_data->buf_pix_fmt == EPDC_FORMAT_BUF_PIXEL_FORMAT_P5N){
		dev_err(fb_data->dev, "%s: 5bit WF",__func__);
		mask = 0xF000;
		regal_offset = 0x0800;
	}else{
		dev_err(fb_data->dev, "%s: 4bit WF",__func__);
		mask = 0xE000;
		regal_offset = 0x1000;
	}
	
	src = (uint16_t *)src_buf_virt + src_update_region->top * src_update_region->width + src_update_region->left;
	prev = (uint16_t *)fb_data->prev_image + src_update_region->top * src_update_region->width + src_update_region->left;

	for (i = 0; i < src_update_region->height; i++) {
		srcp = src;
		prevp = prev;
		
		for (j = 0; j < src_update_region->width; j++) {
			uint16_t val = *srcp;
			
			val = truncate_pixel(val , mask);
			
			/*if(i==j){
				dev_err(fb_data->dev,"(%i,%i) %x, %x-%x-%x-%x\n",i,j,val, *(srcp -1),*(srcp + 1),*(srcp - dst_stride),*(srcp + dst_stride));
			}*/
			//*
			if(*srcp != ((*prevp) & mask)){
				do_regal = 0;
				srcp++;
				prevp++;
				continue;
			}
			
			//*/
			if(j > 0){
				left_neighbor = srcp -1;
				do_regal += (truncate_pixel(*left_neighbor,mask) != val);
				left_neighbor = prevp -1;
				do_regal += (truncate_pixel(*left_neighbor,mask) != val);
			}
			
			if(j < src_update_region->width){
				right_neighbor = srcp + 1;
				do_regal += (truncate_pixel(*right_neighbor,mask) != val);
				right_neighbor = prevp + 1;
				do_regal += (truncate_pixel(*right_neighbor,mask) != val);
			}
			
			if(i>0){
				top_neighbor = srcp - dst_stride;
				do_regal +=  (truncate_pixel(*top_neighbor,mask) != val);
				top_neighbor = prevp - dst_stride;
				do_regal +=  (truncate_pixel(*top_neighbor,mask) != val);
			}
			
			if(i<src_update_region->height){
				bot_neighbor = srcp + dst_stride;
				do_regal +=  (truncate_pixel(*bot_neighbor,mask) != val);
				bot_neighbor = prevp+ dst_stride;
				do_regal +=  (truncate_pixel(*bot_neighbor,mask) != val);
			}

			if(do_regal){
				//printk("%i",do_regal);
				//if(*srcp > 0xE000){
				//	printk("%i",do_regal);
				//	dev_err(fb_data->dev,"Regal: (%i,%i) %x -> %x, %x-%x-%x-%x\n",i,j,val,val+regal_offset, *(srcp -1),*(srcp + 1),*(srcp - dst_stride),*(srcp + dst_stride));
				//}
				val += regal_offset;
			}
			/**srcp = val;*/
			do_regal = 0;
			srcp++;
			prevp++;

		}
		src += src_update_region->width;
		prev += src_update_region->width;
	}

#ifdef DEBUG
	t2 = ktime_get();
	dev_dbg(fb_data->dev, "CFA translation time = %lld us\n", ktime_to_us(ktime_sub(t2, t1)));
#endif

	return 0;
}


static int cfa_process_update(struct mxc_epdc_fb_data *fb_data,
			      u32 src_width, u32 src_height,
			      void *src_buf_virt, void *dst_buf_virt,
			      struct mxcfb_rect *src_update_region)
{
	uint16_t *src, *srcp;
	uint16_t *dst_line_start, *dst1, *dst2;
	int i,j;
	//int flip_filter = 0;
	int dst_line_step = 0, dst_step = 0, dst_stride = 0;
#ifdef DEBUG
	ktime_t t1, t2;
#endif
	dev_dbg(fb_data->dev, "Starting CFA translation\n");
	dev_dbg(fb_data->dev, "Source buf @0x%p, destination buf @0x%p\n", src_buf_virt, dst_buf_virt);
	dev_dbg(fb_data->dev, "Source buffer %ux%u pixels, update region is (%u,%u)+%ux%u (userspace size)\n",
			src_width, src_height, src_update_region->left, src_update_region->top,
			src_update_region->width, src_update_region->height);

#ifdef DEBUG
	t1 = ktime_get();
#endif

	src = (uint16_t *)src_buf_virt + src_update_region->top * src_width + src_update_region->left;
	dst_line_start = dst_buf_virt;

	switch (fb_data->epdc_fb_var.rotate) {
	case FB_ROTATE_UR:
		dst_stride = src_update_region->width;
		dst_line_start += 0;
		dst_line_step = dst_stride * 2;
		dst_step = 1;
		break;
	case FB_ROTATE_CW:
		dst_stride = src_update_region->height;
		dst_line_start += src_update_region->height - 1;
		dst_line_step = -1;
		dst_step = dst_stride * 2;;
		break;
	case FB_ROTATE_UD:
		dst_stride = src_update_region->width;
		dst_line_start += (src_update_region->width - 1) + dst_stride * 2 * (src_update_region->height - 1);
		dst_line_step = -dst_stride * 2;
		dst_step = -1;
		break;
	case FB_ROTATE_CCW:
		dst_stride = src_update_region->height;
		dst_line_start += (src_update_region->width - 1) * dst_stride * 2;
		dst_line_step = 1;
		dst_step = -dst_stride * 2;;
		break;
	}
	dev_dbg(fb_data->dev, "rotate: %d, dst_stride = %d, dst_line_start = 0x%p, dst_line_step = %d, dst_step = %d",
			fb_data->epdc_fb_var.rotate, dst_stride, dst_line_start, dst_line_step, dst_step);

	for (i = 0; i < src_update_region->height; i++) {
		srcp = src;

		dst1 = dst_line_start;
		dst2 = dst_line_start + dst_stride;

		for (j = 0; j < src_update_region->width; j++) {
			uint16_t val;
			uint8_t  r, g, b;
			unsigned int w;
			uint8_t r5, g6, b5;

			val = *srcp;
			r5 = (val >> 11) & 0x1F;
			r = (r5 << 3) | (r5 >> 2);

			g6 = (val >> 5) & 0x3F;
			g = (g6 << 2) | (g6 >> 4);

			b5 = val & 0x1F;
			b = (b5 << 3) | (b5 >> 2);

			w = ((r * 299) + (g * 587) + (b * 114)) / 1000;

			*dst2 = g | (r << 8);
			*dst1 = b | ((w & 0xFF) << 8);
		
			srcp++;
			dst1 += dst_step;
			dst2 += dst_step;
		}
		src += src_width;

		dst_line_start += dst_line_step;
	}

#ifdef DEBUG
	t2 = ktime_get();
	dev_dbg(fb_data->dev, "CFA translation time = %lld us\n", ktime_to_us(ktime_sub(t2, t1)));
#endif

	return 0;
}


/*
 * Function to call PxP DMA driver and send our latest FB update region
 * through the PxP and out to an intermediate buffer.
 * Note: This is a blocking call, so upon return the PxP tx should be complete.
 */
static int pxp_process_update(struct mxc_epdc_fb_data *fb_data,
			      u32 src_width, u32 src_height,
			      struct mxcfb_rect *update_region)
{
	dma_cookie_t cookie;
	struct scatterlist *sg = fb_data->sg;
	struct dma_chan *dma_chan;
	struct pxp_tx_desc *desc;
	struct dma_async_tx_descriptor *txd;
	struct pxp_config_data *pxp_conf = &fb_data->pxp_conf;
	struct pxp_proc_data *proc_data = &fb_data->pxp_conf.proc_data;
	int i, ret;
	int length;
	dev_dbg(fb_data->dev, "Starting PxP Send Buffer\n");
	dump_pxp_config(fb_data, pxp_conf);
	/* First, check to see that we have acquired a PxP Channel object */
	if (fb_data->pxp_chan == NULL) {
		/*
		 * PxP Channel has not yet been created and initialized,
		 * so let's go ahead and try
		 */
		ret = pxp_chan_init(fb_data);
		if (ret) {
			/*
			 * PxP channel init failed, and we can't use the
			 * PxP until the PxP DMA driver has loaded, so we abort
			 */
			dev_err(fb_data->dev, "PxP chan init failed\n");
			return -ENODEV;
		}
	}

	/*
	 * Init completion, so that we
	 * can be properly informed of the completion
	 * of the PxP task when it is done.
	 */
	init_completion(&fb_data->pxp_tx_cmpl);

	dma_chan = &fb_data->pxp_chan->dma_chan;

	txd = dma_chan->device->device_prep_slave_sg(dma_chan, sg, 2,
						     DMA_TO_DEVICE,
						     DMA_PREP_INTERRUPT,
						     NULL);
	if (!txd) {
		dev_err(fb_data->info.device,
			"Error preparing a DMA transaction descriptor.\n");
		return -EIO;
	}

	txd->callback_param = txd;
	txd->callback = pxp_dma_done;

	/*
	 * Configure PxP for processing of new update region
	 * The rest of our config params were set up in
	 * probe() and should not need to be changed.
	 */
	pxp_conf->s0_param.width = src_width;
	pxp_conf->s0_param.height = src_height;
	proc_data->srect.top = update_region->top;
	proc_data->srect.left = update_region->left;
	proc_data->srect.width = update_region->width;
	proc_data->srect.height = update_region->height;

	/*
	 * Because only YUV/YCbCr image can be scaled, configure
	 * drect equivalent to srect, as such do not perform scaling.
	 */
	proc_data->drect.top = 0;
	proc_data->drect.left = 0;

	/* PXP expects rotation in terms of degrees */
	proc_data->rotate = fb_data->epdc_fb_var.rotate * 90;
	if (proc_data->rotate > 270)
		proc_data->rotate = 0;

	/* Just as V4L2 PXP, we should pass the rotated values to PXP */
	if ((proc_data->rotate == 90) || (proc_data->rotate == 270)) {
		proc_data->drect.width = proc_data->srect.height;
		proc_data->drect.height = proc_data->srect.width;
		pxp_conf->out_param.width = update_region->height;
		pxp_conf->out_param.height = update_region->width;
		pxp_conf->out_param.stride = update_region->height;
	} else {
		proc_data->drect.width = proc_data->srect.width;
		proc_data->drect.height = proc_data->srect.height;
		pxp_conf->out_param.width = update_region->width;
		pxp_conf->out_param.height = update_region->height;
		pxp_conf->out_param.stride = update_region->width;
	}

	/* For EPDC v2.0, we need output to be 64-bit
	 * aligned since EPDC stride does not work. */
	if (fb_data->rev <= 20)
		pxp_conf->out_param.stride = ALIGN(pxp_conf->out_param.stride, 8);


	desc = to_tx_desc(txd);
	length = desc->len;
	for (i = 0; i < length; i++) {
		if (i == 0) {/* S0 */
			memcpy(&desc->proc_data, proc_data, sizeof(struct pxp_proc_data));
			pxp_conf->s0_param.paddr = sg_dma_address(&sg[0]);
			memcpy(&desc->layer_param.s0_param, &pxp_conf->s0_param,
				sizeof(struct pxp_layer_param));
		} else if (i == 1) {
			pxp_conf->out_param.paddr = sg_dma_address(&sg[1]);
			memcpy(&desc->layer_param.out_param, &pxp_conf->out_param,
				sizeof(struct pxp_layer_param));
		}
		/* TODO: OverLay */

		desc = desc->next;
	}

	/* Submitting our TX starts the PxP processing task */
	cookie = txd->tx_submit(txd);
	if (cookie < 0) {
		dev_err(fb_data->info.device, "Error sending FB through PxP\n");
		return -EIO;
	}

	fb_data->txd = txd;

	/* trigger ePxP */
	dma_async_issue_pending(dma_chan);

	return 0;
}

static int pxp_complete_update(struct mxc_epdc_fb_data *fb_data, u32 *hist_stat)
{
	int ret;
	/*
	 * Wait for completion event, which will be set
	 * through our TX callback function.
	 */
	ret = wait_for_completion_timeout(&fb_data->pxp_tx_cmpl, HZ / 10);
	if (ret <= 0) {
		dev_info(fb_data->info.device,
			 "PxP operation failed due to %s\n",
			 ret < 0 ? "user interrupt" : "timeout");
		dma_release_channel(&fb_data->pxp_chan->dma_chan);
		fb_data->pxp_chan = NULL;
		return ret ? : -ETIMEDOUT;
	}

	if ((fb_data->pxp_conf.proc_data.lut_transform & EPDC_FLAG_USE_CMAP) &&
		fb_data->pxp_conf.proc_data.lut_map_updated)
		fb_data->pxp_conf.proc_data.lut_map_updated = false;

	*hist_stat = to_tx_desc(fb_data->txd)->hist_status;
	dma_release_channel(&fb_data->pxp_chan->dma_chan);
	fb_data->pxp_chan = NULL;

	dev_dbg(fb_data->dev, "TX completed\n");

	return 0;
}
//*
int get_dead_zones(struct dead_zones* dz, int x){
	int i;
	int offset=0;
	for(i=0; dz[i].w!= -1; i++){
		if((offset+x)<dz[i].l){
			return offset;
		}
		offset += dz[i].w;
	}
	return offset;
}

int get_dead_zone_area(struct mxcfb_rect* area, struct dead_zones* dz){

	return get_dead_zones(dz, area->left + area->width);
}

//*/
void scramble_array(struct mxc_epdc_fb_data *fb_data, void* source_buf, void* target_buf, struct mxcfb_rect * update_rect)
{
	uint32_t i, x, y, new_width;
	uint8_t* tar;
	uint16_t *src;
	int src_width;
	const int tmp_buff_width = update_rect->width * update_rect->height;
	
	src_width = fb_data->cur_mode->vmode->xres * 2;
	tar = (uint8_t*) target_buf;
	src = (uint16_t*) source_buf;
	new_width = (update_rect->width / 2);
	
	for(i=0;i<tmp_buff_width; i++){
		int new_index, src_idx;
		y = 2*(i/(update_rect->width)) - 1 + (i%2);
		x = (i/2) % (update_rect->width / 2);
		src_idx = src_width * (update_rect->top + (i / update_rect->width)) 
						+ update_rect->left + (i % update_rect->width);
		new_index = y * new_width + x;
		tar[new_index]  = (src[src_idx] >> 8);
	}

	flush_cache_all();
	outer_flush_all();
}

/*
 * Different dithering algorithm can be used. We chose
 * to implement Bill Atkinson's algorithm as an example
 * Thanks Bill Atkinson for his dithering algorithm.
 */

/*
 * Dithering algorithm implementation - Y8->Y1 version 1.0 for i.MX
 */
static void do_dithering_processing_Y1_v1_0(
		unsigned char *update_region_ptr,
		struct mxcfb_rect *update_region,
		unsigned long update_region_stride,
		int *err_dist)
{
	/* create a temp error distribution array */
	int bwPix;
	int y;
	int col;
	int *err_dist_l0, *err_dist_l1, *err_dist_l2, distrib_error;
	int width_3 = update_region->width + 3;
	char *y8buf;
	int x_offset = 0;

	/* prime a few elements the error distribution array */
	for (y = 0; y < update_region->height; y++) {
		/* Dithering the Y8 in sbuf to BW suitable for A2 waveform */
		err_dist_l0 = err_dist + (width_3) * (y % 3);
		err_dist_l1 = err_dist + (width_3) * ((y + 1) % 3);
		err_dist_l2 = err_dist + (width_3) * ((y + 2) % 3);

		y8buf = update_region_ptr + x_offset;

		/* scan the line and convert the Y8 to BW */
		for (col = 1; col <= update_region->width; col++) {
			bwPix = *(err_dist_l0 + col) + *y8buf;

			if (bwPix >= 100) {//128
				*y8buf++ = 0xff;
				distrib_error = (bwPix - 255) >> 3;
			} else {
				*y8buf++ = 0;
				distrib_error = bwPix >> 3;
			}

			/* modify the error distribution buffer */
			*(err_dist_l0 + col + 2) += distrib_error;
			*(err_dist_l1 + col + 1) += distrib_error;
			*(err_dist_l0 + col + 1) += distrib_error;
			*(err_dist_l1 + col - 1) += distrib_error;
			*(err_dist_l1 + col) += distrib_error;
			*(err_dist_l2 + col) = distrib_error;
		}
		x_offset += update_region_stride;
	}

	flush_cache_all();
	outer_flush_all();
}

/*
 * Dithering algorithm implementation - Y8->Y4 version 1.0 for i.MX
 */

static void do_dithering_processing_Y4_v1_0(
		unsigned char *update_region_ptr,
		struct mxcfb_rect *update_region,
		unsigned long update_region_stride,
		int *err_dist)
{

	/* create a temp error distribution array */
	int gcPix;
	int y;
	int col;
	int *err_dist_l0, *err_dist_l1, *err_dist_l2, distrib_error;
	int width_3 = update_region->width + 3;
	char *y8buf;
	int x_offset = 0;

	/* prime a few elements the error distribution array */
	for (y = 0; y < update_region->height; y++) {
		/* Dithering the Y8 in sbuf to Y4 */
		err_dist_l0 = err_dist + (width_3) * (y % 3);
		err_dist_l1 = err_dist + (width_3) * ((y + 1) % 3);
		err_dist_l2 = err_dist + (width_3) * ((y + 2) % 3);

		y8buf = update_region_ptr + x_offset;

		/* scan the line and convert the Y8 to Y4 */
		for (col = 1; col <= update_region->width; col++) {
			gcPix = *(err_dist_l0 + col) + *y8buf;

			if (gcPix > 255)
				gcPix = 255;
			else if (gcPix < 0)
				gcPix = 0;

			distrib_error = (*y8buf - (gcPix & 0xf0)) >> 3;

			*y8buf++ = gcPix & 0xf0;

			/* modify the error distribution buffer */
			*(err_dist_l0 + col + 2) += distrib_error;
			*(err_dist_l1 + col + 1) += distrib_error;
			*(err_dist_l0 + col + 1) += distrib_error;
			*(err_dist_l1 + col - 1) += distrib_error;
			*(err_dist_l1 + col) += distrib_error;
			*(err_dist_l2 + col) = distrib_error;
		}
		x_offset += update_region_stride;
	}

	flush_cache_all();
	outer_flush_all();
}

// Ordered dithering aka Bayer matrix dithering

/* Bayer-method ordered dither.  The array line[] contains the intensity
** values for the line being processed.  As you can see, the ordered
** dither is much simpler than the error dispersion dither.  It is also
** many times faster, but it is not as accurate and produces cross-hatch
** patterns on the output.
*/


static void do_ordered_dithering_processing(
		unsigned char *update_region_ptr,
		struct mxcfb_rect *update_region,
		unsigned long update_region_stride,
		int *err_dist)
{

	int pattern[8][8] = {
		{ 0, 32,  8, 40,  2, 34, 10, 42},   /* 8x8 Bayer ordered dithering  */
		{48, 16, 56, 24, 50, 18, 58, 26},   /* pattern.  Each input pixel   */
		{12, 44,  4, 36, 14, 46,  6, 38},   /* is scaled to the 0..63 range */
		{60, 28, 52, 20, 62, 30, 54, 22},   /* before looking in this table */
		{ 3, 35, 11, 43,  1, 33,  9, 41},   /* to determine the action.     */
		{51, 19, 59, 27, 49, 17, 57, 25},
		{15, 47,  7, 39, 13, 45,  5, 37},
		{63, 31, 55, 23, 61, 29, 53, 21} };

	/* create a temp error distribution array */
	int bwPix;
	int y;
	int col;
	char *y8buf;
	int x_offset = 0;

	/* prime a few elements the error distribution array */
	for (y = 0; y < update_region->height; y++) {
		/* Dithering the Y8 in sbuf to BW suitable for A2 waveform */


		y8buf = update_region_ptr + x_offset;

		/* scan the line and convert the Y8 to BW */
		for (col = 1; col <= update_region->width; col++) {
			bwPix = *y8buf >> 2;

			if (bwPix > pattern[col & 7][y & 7]) {//128
				*y8buf++ = 0xff;

			} else {
				*y8buf++ = 0;
			}

		}
		x_offset += update_region_stride;
	}

	flush_cache_all();
	outer_flush_all();
}

module_platform_driver(mxc_epdc_fb_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC EPDC framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("fb");

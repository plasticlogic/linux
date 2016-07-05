/*
 * mxc_epdc_pl_hardware.h -- Plastic Logic display power control
 *
 *      Copyright (C) 2012 Guillaume Tucker, Plastic Logic Limited
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#ifndef INCLUDE_MXC_EPDC_PL_HARDWARE_H
#define INCLUDE_MXC_EPDC_PL_HARDWARE_H 1


#define IMX_GPIO_NR(bank, nr)		(((bank) - 1) * 32 + (nr))
/* Pin definitions for Type11 preinitialisation bitbash */
#define EPDC_GDSP IMX_GPIO_NR(2, 2)
#define EPDC_GDCLK IMX_GPIO_NR(1, 31)
/* Pin definitions for VCOM for new CPLD image*/
#define EPDC_HVEN IMX_GPIO_NR(2, 7) 			//EPDC_PWRCTRL0
#define EPDC_VCOM_SW_CLOSE IMX_GPIO_NR(2, 11) 	//EPDC_PWRCOM
#define EPDC_VCOM_SW_EN IMX_GPIO_NR(2, 3) 		//EPDC_VCOM0
#define EPDC_VCOM_PSU_EN IMX_GPIO_NR(2, 4) 		//EPDC_VCOM1

#if 1
#define PL_HARDWARE_USE_FAST_INTERFACE 1
#define PL_HARDWARE_FAST_D0     IMX_GPIO_NR(2, 11)
#define PL_HARDWARE_FAST_D1     IMX_GPIO_NR(2, 3) 
#define PL_HARDWARE_FAST_D2     IMX_GPIO_NR(2, 4) 
#define PL_HARDWARE_FAST_CLK    IMX_GPIO_NR(2, 13)
#define PL_HARDWARE_FAST_EN     IMX_GPIO_NR(2, 7) 
#else
#define PL_HARDWARE_USE_FAST_INTERFACE 0
#endif
/* I2C addresses */
#define CPLD_I2C_ADDRESS 0x70
#define HVPMIC_I2C_ADDRESS 0x48

/* CPLD parameters */
#define CPLD_REQ_VERSION 0x03
#define CPLD_NB_BYTES 3

/* DAC */
#define DAC5820_CMD_LOAD_IN_DAC_A__UP_DAC_B__OUT_AB 0x0
#define DAC5820_CMD_EXT__DATA_0 0xF

/* ADC */
#define ADC11607_NB_RESULTS 4
#define ADC11607_SEL_INT_REF_ON 0x1
#define ADC11607_SEL_EXT_REF 0x2
#define ADC11607_SEL_INT_REF 0x4
#define ADC11607_SEL_AIN__REF_OUT 0x2
#define ADC11607_MAX_VALUE 0x3FF
#define ADC11607_INVALID_RESULT ((ADC11607_MAX_VALUE) + 1)

/* VCOM */
#define VCOM_MAX 13000
#define VCOM_MIN 1000
#define VCOM_DEFAULT ((VCOM_MAX + VCOM_MIN) / 2)
#define VCOM_REF_ADC_CHANNEL 0
#define VCOM_FB_ADC_CHANNEL 1
#define VCOM_ADC_SCALE 10
#define VCOM_VREF_MUL 24200
#define VCOM_VREF_DIV 332
#define VCOM_VGSWING 70000
#define VCOM_CORR_I 110

/* HVPMIC MAX17135 timings */
#define HVPMIC_NB_TIMINGS 8

/* CPLD definitions */
enum pl_hardware_cpld_switch {
	CPLD_HVEN,
	CPLD_COM_SW_EN,
	CPLD_COM_SW_CLOSE,
	CPLD_COM_PSU,
	CPLD_SRC_CS_LOGIC,
	CPLD_HVEN1,
	CPLD_COM_SW_CLOSE1,
	CPLD_PING_PONG,
	CPLD_SOURCE_2BPP,
	CPLD_ALT_I2C,
};

struct cpld_byte_0 {
	__u8 cpld_hven:1;
	__u8 src_cs_logic:1;
	__u8 version:6;
};

struct cpld_byte_1 {
	__u8 vcom_sw_close:1;
	__u8 vcom_sw_en:1;
	__u8 vcom_psu_en:1;
	__u8 vgpos_clamp:1;
	__u8 build_version:4;
};

struct cpld_byte_2 {
	__u8 board_id:4;
	__u8 cpld_hven1:1;
	__u8 vcom_sw_close1:1;
	__u8 ping_pong:1;
	__u8 source_2bpp:1;
};

union pl_hardware_cpld {
	struct {
		struct cpld_byte_0 b0;
		struct cpld_byte_1 b1;
		struct cpld_byte_2 b2;
	};
	u8 data[CPLD_NB_BYTES];
};

struct pl_hardware_cpld_fast_data {
	__u8 alt_i2c:1;
};

/* HVPMIC definitions */

enum pl_hardware_hvpmic_register {
	HVPMIC_REG_EXT_TEMP   = 0x00,
	HVPMIC_REG_CONF       = 0x01,
	HVPMIC_REG_INT_TEMP   = 0x04,
	HVPMIC_REG_TEMP_STAT  = 0x05,
	HVPMIC_REG_PROD_REV   = 0x06,
	HVPMIC_REG_PROD_ID    = 0x07,
	HVPMIC_REG_DVR        = 0x08,
	HVPMIC_REG_ENABLE     = 0x09,
	HVPMIC_REG_FAULT      = 0x0A,
	HVPMIC_REG_PROG       = 0x0C,
	HVPMIC_REG_TIMING_1   = 0x10,
	HVPMIC_REG_TIMING_2   = 0x11,
	HVPMIC_REG_TIMING_3   = 0x12,
	HVPMIC_REG_TIMING_4   = 0x13,
	HVPMIC_REG_TIMING_5   = 0x14,
	HVPMIC_REG_TIMING_6   = 0x15,
	HVPMIC_REG_TIMING_7   = 0x16,
	HVPMIC_REG_TIMING_8   = 0x17,
};

union hvpmic_fault {
	struct {
		char fbpg:1;
		char hvinp:1;
		char hvinn:1;
		char fbng:1;
		char hvinpsc:1;
		char hvinnsc:1;
		char ot:1;
		char pok:1;
	};
	char byte;
};

struct pl_hardware_hvpmic {
	__u8 prod_id;
	__u8 prod_rev;
	__u8 timings[HVPMIC_NB_TIMINGS];
};

/* DAC definitions */

enum dac5820_pd {
	DAC5820_PD_ON         = 0x0,
	DAC5820_PD_OFF_FLOAT  = 0x1,
	DAC5820_PD_OFF_1K     = 0x2,
	DAC5820_PD_OFF_100K   = 0x3
};

struct dac5820_byte_write_cmd {
	char data_high:4;
	char cmd:4;
};

struct dac5820_byte_cmd_ext {
	char pd:2;
	char a:1;
	char b:1;
	char reserved:4;
};

struct dac5820_byte_write_data {
	char reserved:4;
	char data_low:4;
};

union dac5820_write_payload {
	struct {
		struct dac5820_byte_write_cmd cmd_byte;
		struct dac5820_byte_write_data data_byte;
	};
	char bytes[2];
};

union dac5820_ext_payload {
	struct {
		struct dac5820_byte_write_cmd cmd_byte;
		struct dac5820_byte_cmd_ext ext_byte;
	};
	char bytes[2];
};

struct pl_hardware_dac {
	__u8 i2c_address;
};

/* ADC definitions */

struct adc11607_setup {
	char reserved:1;
	char reset:1;
	char bip_uni:1;
	char clk_sel:1;
	char sel:3;
	char setup_1:1;
};

struct adc11607_config {
	char se_diff:1;
	char cs:4;
	char scan:2;
	char config_0:1;
};

union adc11607_setup_config {
	struct {
		struct adc11607_setup setup;
		struct adc11607_config config;
	};
	char bytes[2];
};

struct pl_hardware_adc {
	__u8 i2c_address;
	union adc11607_setup_config cmd;
	unsigned nb_channels;
	unsigned ref_mv;
	int current_channel;
	unsigned results[ADC11607_NB_RESULTS];
};

/* VCOM definitions */

struct pl_hardware_vcom {
	int reference_mv;
	int target_mv;
	int vref_mv;
	int last_v_in;
	bool dac_measured;
	int dac_offset;
	int dac_dx;
	int dac_dy;
	int dac_step_mv;
};

/* PSU structure */
struct pl_hardware_psu {
	int i2c_sel_id;
	struct pl_hardware_hvpmic hvpmic;
	struct pl_hardware_dac dac;
	struct pl_hardware_adc adc;
	struct pl_hardware_vcom vcom;
};

/* Opaque public instance structure */
struct mxc_epdc_pl_hardware {
	bool init_done;
	struct mxc_epdc_plhw_pdata *pdata;
	const struct mxc_epdc_plhw_config *conf;
	struct i2c_adapter *i2c;
	union pl_hardware_cpld cpld;
	struct pl_hardware_cpld_fast_data fast_cpld;
	struct pl_hardware_psu psu[2];
};


/* Opaque instance structure */
struct mxc_epdc_pl_hardware;

enum mxc_epdc_pl_hardware_fast_gpio {
	MXC_EPDC_PL_HARDWARE_FAST_D0 = 0,
	MXC_EPDC_PL_HARDWARE_FAST_D1,
	MXC_EPDC_PL_HARDWARE_FAST_D2,
	MXC_EPDC_PL_HARDWARE_FAST_CLK,
	MXC_EPDC_PL_HARDWARE_FAST_EN,
};
#define MXC_EPDC_PL_HARDWARE_GPIO_N 5

struct mxc_epdc_plhw_pdata {
	int i2c_bus_number;
	__u8 dac_i2c_address;
	__u8 adc_i2c_address;
	int use_minimal_cpld;
	int fast_gpio[MXC_EPDC_PL_HARDWARE_GPIO_N];
};

enum mxc_epdc_plhw_power_seq {
	MXC_EPDC_PL_HARDWARE_SEQ_0, /* rails: SN, GP, GN, SP */
	MXC_EPDC_PL_HARDWARE_SEQ_1, /* rails: SN, GN, GP, SP */
	MXC_EPDC_PL_HARDWARE_SEQ_N
};

struct mxc_epdc_plhw_config {
	int psu_n;
	bool source_2bpp_conversion; /* convert bpp data for 4bp display */
	bool interlaced_gates;       /* gate lines are interlaced odd/even */
	bool source_cs_logic;        /* generate CS signals (no cascading) */
	enum mxc_epdc_plhw_power_seq power_seq; /* power sequence identifier */
};

extern struct mxc_epdc_pl_hardware *mxc_epdc_pl_hardware_alloc(void);
extern int mxc_epdc_pl_hardware_init(struct mxc_epdc_pl_hardware *plhw,
				     struct mxc_epdc_plhw_pdata *pdata,
				     const struct mxc_epdc_plhw_config *conf);
extern void mxc_epdc_pl_hardware_free(struct mxc_epdc_pl_hardware *plhw);

extern int mxc_epdc_pl_hardware_set_vcom(struct mxc_epdc_pl_hardware *plhw,
					 const int *vcoms_mv);
extern int mxc_epdc_pl_hardware_enable(struct mxc_epdc_pl_hardware *plhw);
extern int mxc_epdc_pl_hardware_disable(struct mxc_epdc_pl_hardware *plhw);

#endif /* INCLUDE_MXC_EPDC_PL_HARDWARE_H */

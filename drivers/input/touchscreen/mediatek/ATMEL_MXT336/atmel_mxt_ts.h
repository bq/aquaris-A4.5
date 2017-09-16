/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>

#include "ts_key.h"

typedef unsigned short U16;
typedef unsigned char U8;

struct mxt_config_info {
	u8 self_chgtime_max;
	u8 self_chgtime_min;
};

enum {
	T15_T97_KEY = 0,
	T19_KEY,
	T24_KEY,
	T61_KEY,
	T81_KEY,
	T92_KEY,
	T93_KEY,
	T99_KEY,
	T115_KEY,
	T116_KEY,
	NUM_KEY_TYPE
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	unsigned long irqflags;
/*
	u8 t19_num_keys;
	const unsigned int *t19_keymap;
	int t15_num_keys;
	const unsigned int *t15_keymap;
*/
	const u8 *num_keys;  //len is NUM_KEY_TYPE
	const unsigned int (*keymap)[MAX_KEYS_SUPPORTED_IN_DRIVER];

	unsigned long gpio_reset;
	const char *cfg_name;
#if defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
	unsigned int max_y_t;  	//Max value of Touch AA, asix more than this value will process by VirtualKeyHit
						//Note Resolution of TP AA and LCD AA must be same value. For e.g: LCD(1920 * 1080), TP max set as(1919 + virtual area,1079), max_y_t = 1919
	struct mxt_virtual_key_space vkey_space_ratio;
#endif

	const struct mxt_config_info *config_array;
};

#endif /* __LINUX_ATMEL_MXT_TS_H */






//frank modify move the defination of plugin.h here

enum {
	MXT_RESERVED_T0 = 0,
	MXT_RESERVED_T1,
	MXT_DEBUG_DELTAS_T2,
	MXT_DEBUG_REFERENCES_T3,
	MXT_DEBUG_SIGNALS_T4,
	MXT_GEN_MESSAGEPROCESSOR_T5,
	MXT_GEN_COMMANDPROCESSOR_T6,
	MXT_GEN_POWERCONFIG_T7,
	MXT_GEN_ACQUISITIONCONFIG_T8,
	MXT_TOUCH_MULTITOUCHSCREEN_T9,
	MXT_TOUCH_SINGLETOUCHSCREEN_T10,
	MXT_TOUCH_XSLIDER_T11,
	MXT_TOUCH_YSLIDER_T12,
	MXT_TOUCH_XWHEEL_T13,
	MXT_TOUCH_YWHEEL_T14,
	MXT_TOUCH_KEYARRAY_T15,
	MXT_PROCG_SIGNALFILTER_T16,
	MXT_PROCI_LINEARIZATIONTABLE_T17,
	MXT_SPT_COMCONFIG_T18,
	MXT_SPT_GPIOPWM_T19,
	MXT_PROCI_GRIPFACESUPPRESSION_T20,
	MXT_RESERVED_T21,
	MXT_PROCG_NOISESUPPRESSION_T22,
	MXT_TOUCH_PROXIMITY_T23,
	MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24,
	MXT_SPT_SELFTEST_T25,
	MXT_DEBUG_CTERANGE_T26,
	MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27,
	MXT_SPT_CTECONFIG_T28,
	MXT_SPT_GPI_T29,
	MXT_SPT_GATE_T30,
	MXT_TOUCH_KEYSET_T31,
	MXT_TOUCH_XSLIDERSET_T32,
	MXT_RESERVED_T33,
	MXT_GEN_MESSAGEBLOCK_T34,
	MXT_SPT_GENERICDATA_T35,
	MXT_RESERVED_T36,
	MXT_DEBUG_DIAGNOSTIC_T37,
	MXT_SPT_USERDATA_T38,
	MXT_SPARE_T39,
	MXT_PROCI_GRIPSUPPRESSION_T40,
	MXT_SPARE_T41,
	MXT_PROCI_TOUCHSUPPRESSION_T42,
	MXT_SPT_DIGITIZER_T43,
	MXT_SPARE_T44,
	MXT_SPARE_T45,
	MXT_SPT_CTECONFIG_T46,
	MXT_PROCI_STYLUS_T47,
	MXT_PROCG_NOISESUPPRESSION_T48,
	MXT_SPARE_T49,
	MXT_SPARE_T50,
	MXT_SPARE_T51,
	MXT_TOUCH_PROXIMITY_KEY_T52,
	MXT_GEN_DATASOURCE_T53,
	MXT_SPARE_T54,
	MXT_ADAPTIVE_T55,
	MXT_PROCI_SHIELDLESS_T56,
	MXT_PROCI_EXTRATOUCHSCREENDATA_T57,
	MXT_SPARE_T58,
	MXT_SPARE_T59,
	MXT_SPARE_T60,
	MXT_SPT_TIMER_T61,
	MXT_PROCG_NOISESUPPRESSION_T62,
	MXT_PROCI_ACTIVESTYLUS_T63,
	MXT_SPARE_T64,
	MXT_PROCI_LENSBENDING_T65,
	MXT_SPT_GOLDENREFERENCES_T66,
	MXT_SPARE_T67,
	MXT_SPARE_T68,
	MXT_PROCI_PALMGESTUREPROCESSOR_T69,
	MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70,
	MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71,
	MXT_PROCG_NOISESUPPRESSION_T72,
	MXT_PROCI_GLOVEDETECTION_T78 = 78,
	MXT_PROCI_RETRANSMISSIONCOMPENSATION_T80 = 80,
	MXT_PROCI_UNLOCKGESTURE_T81,
	MXT_PROCI_GESTURE_T92 = 92,
	MXT_PROCI_TOUCHSEQUENCELOGGER_T93,
	MXT_TOUCH_SPT_PTC_TUNINGPARAMS_T96 = 96,
	MXT_TOUCH_PTC_KEYS_T97,
	MXT_PROCI_KEYGESTUREPROCESSOR_T99 = 99,
	MXT_TOUCH_MULTITOUCHSCREEN_T100,
	MXT_SPT_TOUCHSCREENHOVER_T101,
	MXT_SPT_SELFCAPHOVERCTECONFIG_T102,
	MXT_PROCI_AUXTOUCHCONFIG_T104 = 104,
	MXT_PROCG_NOISESUPSELFCAP_T108 = 108,
	MXT_SPT_SELFCAPGLOBALCONFIG_T109,
	MXT_SPT_SELFCAPTUNINGPARAMS_T110,
	MXT_SPT_SELFCAPCONFIG_T111,
	MXT_SPT_SELFCAPMEASURECONFIG_T113 = 113,
	MXT_PROCI_SYMBOLGESTURE_T115 = 115,
	MXT_SPT_SYMBOLGESTURECONFIG_T116,
	MXT_RESERVED_T255 = 255,
};

/* Define for T6 command */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* Define for T6 debug mode command */
#define MXT_T6_DEBUG_PAGEUP	0x1
#define MXT_T6_DEBUG_PAGEDOWN	0x2
#define MXT_T6_DEBUG_DELTA	0x10
#define MXT_T6_DEBUG_REF	0x11
#define MXT_T6_DEBUG_DELTA_PTC	0x14
#define MXT_T6_DEBUG_REF_PTC		0x15
#define MXT_T6_DEBUG_SE	0x33
#define MXT_T6_DEBUG_GESTRUE	0x34
#define MXT_T6_DEBUG_PID	0x81
#define MXT_T6_DEBUG_DELTA_SC	0xF7
//#define MXT_T6_DEBUG_SIGNAL_SC	0xFE

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	(1 << 7)
#define MXT_T6_STATUS_OFL	(1 << 6)
#define MXT_T6_STATUS_SIGERR	(1 << 5)
#define MXT_T6_STATUS_CAL	(1 << 4)
#define MXT_T6_STATUS_CFGERR	(1 << 3)
#define MXT_T6_STATUS_COMSERR	(1 << 2)

struct diagnostic_info {
	u8 cmd;
	s8 page;
	u8 index;
	u8 num;  //data length (byte)
};

struct diagnostic_block{
	struct diagnostic_info diag;
	int step;
	int max_step;
	int curr;
	int ext;

	u8 * buf;
	int interval;
	int count;
};

/* MXT_GEN_POWER_T7 field */

struct t7_config {
	u8 idle;
	u8 active;
} __packed;


/* MXT_GEN_ACQUIRE_T8 field */
enum t8_status{
	T8_HALT,
	T8_NORMAL,
	T8_NOISE,
	T8_VERY_NOISE,
	T8_MIDDLE,
	T8_WEAK_PALM,
};

struct t8_config {
	u8 atchcalst;
	u8 atchcalsthr;
	u8 atchfrccalthr;
	s8 atchfrccalratio;
	u8 tchautocal;
}__packed;

#define MXT_T8_CHRGTIME	0
#define MXT_T8_TCHDRIFT	2
#define MXT_T8_DRIFTST	3
#define MXT_T8_TCHAUTOCAL	4
#define MXT_T8_SYNC	5
#define MXT_T8_ATCHCALST	6
#define MXT_T8_ATCHCALSTHR	7
#define MXT_T8_ATCHFRCCALTHR		8
#define MXT_T8_ATCHFRCCALRATIO	9

enum t8_cfg_set_option{
	MXT_T8_MASK_ATCHCALST = 0,
	MXT_T8_MASK_ATCHCALSTHR,
	MXT_T8_MASK_ATCHFRCCALTHR,
	MXT_T8_MASK_ATCHFRCCALRATIO,
	MXT_T8_MASK_TCHAUTOCAL,
};


/* MXT_TOUCH_MULTI_T9 field */
enum t9_t100_status{
	T9_T100_NORMAL,
	T9_T100_THLD_NOISE,
	T9_T100_THLD_VERY_NOISE,
	T9_T100_NORMAL_STEP1,
	T9_T100_THLD_NOISE_STEP1,
	T9_T100_THLD_VERY_NOISE_STEP1,
	T9_T100_SINGLE_TOUCH,
	T9_T100_CONFIG_NUM,
};

enum t9_t100_cfg_set_option{
	MXT_T9_T100_MASK_TCHHR = 0,
	MXT_T9_T100_MASK_TCHHYST,
	MXT_T9_T100_MASK_MRGTHR,
	MXT_T9_T100_MASK_MRGHYST,
	MXT_T9_T100_MASK_N_TOUCH,
};

#define T9_T100_CTRL_ENABLE	(1<<0)
#define T9_T100_CTRL_REPEN	(1<<1)

struct t9_t100_config {
	u8 threshold;
	u8 hysterisis;

	u8 internal_threshold;
	u8 internal_hysterisis;
	
	u8 merge_threshold;
	u8 merge_hysterisis;
	u8 num_touch;
	u8 x0;
	u8 y0;
	u8 xsize;
	u8 ysize;
	u8 dualx_threshold;
}__packed;

#define MXT_T9_CTRL			0
#define MXT_T9_XORIGN		1
#define MXT_T9_YORIGN		2
#define MXT_T9_XSIZE		3
#define MXT_T9_YSIZE		4
#define MXT_T9_TCHHR		7
#define MXT_T9_ORIENT		9
#define MXT_T9_NUMTOUCH		14
#define MXT_T9_MRGHYST		15
#define MXT_T9_MRGTHR		16
#define MXT_T9_RANGE		18
#define MXT_T9_TCHHYST		31
#define MXT_T9_DUALX_THLD	42

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		(1 << 0)
#define MXT_T9_SUPPRESS		(1 << 1)
#define MXT_T9_AMP		(1 << 2)
#define MXT_T9_VECTOR		(1 << 3)
#define MXT_T9_MOVE		(1 << 4)
#define MXT_T9_RELEASE		(1 << 5)
#define MXT_T9_PRESS		(1 << 6)
#define MXT_T9_DETECT		(1 << 7)
/*
struct t9_range {
	u16 x;
	u16 y;
} __packed;
*/
/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH	(1 << 0)

/* T15 Key array */
struct t15_config {
	u8 threshold;
	u8 hysterisis;
	u8 x0;
	u8 y0;
	u8 xsize;
	u8 ysize;
}__packed;

#define MXT_T15_XORIGN		1
#define MXT_T15_YORIGN		2
#define MXT_T15_XSIZE		3
#define MXT_T15_YSIZE		4
#define MXT_T15_TCHHR		7

/* Define for MXT_SPT_USERDATA_T38 */

struct t38_config {
	u8 data[8];
}__packed;

//code:
//  C: T9 THLD / Length: 2
#define MXT_T38_MAGIC_WORD	0x92
#define MXT_T38_OFFICIAL_RESERVED   0
#define MXT_T38_CONFIG_VERSION_0   1
#define MXT_T38_CONFIG_VERSION_1   2
#define MXT_T38_BLOCK_LOW_LIMIT_LEVEL		3
#define MXT_T38_BLOCK_HIGH_LIMIT_LEVEL		4
#define MXT_T38_T9_T100_THLD_NORMAL_STEP1	5
#define MXT_T38_T9_T100_THLD_NOISE		6
#define MXT_T38_MGWD				7

/* Define for MXT_PROCI_GRIPSUPPRESSION_T40 */
#define MXT_GRIP_CTRL			0
#define MXT_GRIP_XLOCRIP		1
#define MXT_GRIP_XHICRIP		2
#define MXT_GRIP_YLOCRIP		3
#define MXT_GRIP_YHICRIP		4

/* T40 reg array */
struct t40_config {
	u8 ctrl;
	u8 xlow;
	u8 xhigh;
	u8 ylow;
	u8 yhigh;
}__packed;

/* T42 reg array */
#define MXT_T42_CTRL		0

enum t42_status{
	T42_NORMAL,
	T42_DISABLE,
	T42_CONFIG_NUM,
};

struct t42_config {
	u8 ctrl;
}__packed;

enum t42_cfg_set_option{
	MXT_T42_MASK_CTRL = 0,
};

/* T55 reg array */
#define MXT_T55_CTRL			0
#define MXT_T55_TARGETTHR		1
#define MXT_T55_THRADJLIM		2
#define MXT_T55_RESETSTEPTIME		3
#define MXT_T55_FORCECHGDIST		4
#define MXT_T55_FORCECHGTIME		5

#define MXT_T55_CTRL_EN		(1<<0)

enum t55_status{
	T55_NORMAL,
	T55_DISABLE,
	T55_CONFIG_NUM,
};

struct t55_config {
	u8 ctrl;
	/*
	u8 tthld;
	u8 tlimit;
	u8 rtime;
	u8 adjdistance;
	u8 adjtime;
	*/
}__packed;

/* T61 reg array */
struct t61_config {
	u8 ctrl;
	u8 cmd;
	u8 mode;
	u16 period;
}__packed;

/* T65 reg array */
#define MXT_T65_CTRL		0
#define MXT_T65_GRADTHR		1
#define MXT_T65_LPFILTER	10

enum t65_status{
	T65_NORMAL,
	T65_ZERO_GRADTHR,
	T65_CONFIG_NUM,
};

struct t65_config {
	u8 ctrl;
	u8 grad_thr;
	u8 rsv[8];
	u8 lpfilter;
}__packed;

enum t65_cfg_set_option{
	MXT_T65_MASK_CTRL = 0,
	MXT_T65_MASK_GRADTHR,
	MXT_T65_MASK_LPFILTER,
};

struct t68_config_head {
	u8 ctrl;
	u8 rsv[2];
	u16 type;
	u8 len;
}__packed;

struct t68_config_tail {
	u8 cmd;
	u8 rsv[2];
}__packed;

/* T71 reg array */
#define MXT_T71_RESET_MAGIC_WORD	0x16
#define MXT_T71_RESET_TAG		0

/* T80 reg array */
#define MXT_T80_CTRL		0
#define MXT_T80_COMP_GAIN	1

enum t80_status{
	T80_NORMAL,
	T80_LOW_GAIN,
	T80_CONFIG_NUM,
};

struct t80_config {
	u8 ctrl;
	u8 comp_gain;
	u8 target_delta;
	u8 compthr;
	u8 atchthr;
	/*
	u8 moistcfg;
	u8 moistdto;*/
}__packed;

enum t80_cfg_set_option{
	MXT_T80_MASK_CTRL = 0,
	MXT_T80_MASK_COMP_GAIN,
};

struct t92_config {
	u8 rsv[12];
	u8 rptcode;
}__packed;

#define PTC_KEY_GROUPS 4
struct t96_config {
	s16 params[PTC_KEY_GROUPS];
}__packed;

#define MXT_T61_CTRL	0
#define MXT_T61_CMD	1
#define MXT_T61_MODE	2
#define MXT_T61_PERIOD	3

#define MXT_T61_CTRL_EN		(1<<0)
#define MXT_T61_CTRL_RPTEN	(1<<1)

#define MXT_T61_RUNNING		(1<<0)
#define MXT_T61_FORCERPT	(1<<4)
#define MXT_T61_STOP		(1<<5)
#define MXT_T61_START		(1<<6)
#define MXT_T61_ELAPSED		(1<<7)

#define MXT_T100_CTRL	0
#define MXT_T100_NUMTOUCH	6
#define MXT_T100_XORIGN		8
#define MXT_T100_XSIZE		9
#define MXT_T100_YORIGN		19
#define MXT_T100_YSIZE		20
#define MXT_T100_TCHHR		30
#define MXT_T100_TCHHYST	31
#define MXT_T100_INTTHR		32
#define MXT_T100_MRGTHR		35
#define MXT_T100_DXTHRSF	38
#define MXT_T100_MRGHYST	37
#define MXT_T100_INTTHRHYST	53

#define MXT_T100_DETECT		(1 << 7)

enum{
	MSG_T9_T100_STATUS = 0,
	MSG_T9_T100_AREA,
	MSG_T9_T100_AMP,
	MSG_T9_T100_VEC,
};

#define MAX_TRACE_POINTS 10
#define MAX_AMPLITUDE_VALUE 255

struct sc_config{
	u8 gain;
	u8 chthr;
	u8 chhyst;
	u8 intthr;
	u8 inthyst;
}__packed;

struct t104_config {
	struct sc_config x;
	struct sc_config y;
}__packed;

struct t113_config {
	u8 ctrl;
	u8 gainx;
	u8 gainy;
}__packed;


struct reg_config {
	u16 reg;
	u16 instance;
	u16 offset;
	u16 reg_len;
	u8 buf[16];
	u16 len;

	unsigned long mask;

	u8 sleep;

};

enum{
	MX = 0,
	MX_POS,//aa abusolute position in the matrix
	MX_AA,//aa size(relative)
	MX_T,//touch aa position at the aa area
	MX_K,//k aa position in the aa area
	MX_SUM,
};

struct point{
	int x;
	int y;
};

struct rect{
	int x0;
	int y0;
	int x1;
	int y1;
};

enum{
	SC_X = 0,
	SC_Y,
	SC_NUM,
};

struct range_t {
	int   start;
	int   end;
};

struct mxt_config {
	struct device *dev;//linux driver device
	unsigned int max_x;
	unsigned int max_y;
	//actual size of the touch node

	struct rect m[MX_SUM];
	struct range_t r[SC_NUM];
	struct t7_config t7;
	struct t8_config t8;
	struct t9_t100_config t9_t100;
	struct t15_config t15;
	struct t38_config t38;
	struct t40_config t40;
	struct t42_config t42;
	struct t55_config t55;
#define T61_MAX_INSTANCE_NUM 2//here assume support 2 timer
	struct t61_config t61[T61_MAX_INSTANCE_NUM];	
	struct t65_config t65;
	struct t80_config t80;
	struct t92_config t92;
	struct t96_config t96;
	struct t104_config t104;
	struct t113_config t113;
}__packed;

#define PL_STATUS_FLAG_NOISE						(1<<0)
#define PL_STATUS_FLAG_VERY_NOISE				(1<<1)
#define PL_STATUS_FLAG_NOISE_CHANGE				(1<<2)
#define PL_STATUS_FLAG_DUALX						(1<<3)
#define PL_STATUS_FLAG_PROXIMITY_REMOVED		(1<<4)

#define PL_STATUS_FLAG_SUSPEND					(1<<5)
#define PL_STATUS_FLAG_RESUME					(1<<6)
#define PL_STATUS_FLAG_POWERUP					(1<<7)

#define PL_STATUS_FLAG_RESETING					(1<<8)
#define PL_STATUS_FLAG_RESET_END					(1<<9)
#define PL_STATUS_FLAG_CAL_END					(1<<10)//Cal workaround end
#define PL_STATUS_FLAG_PHONE_ON					(1<<11)
#define PL_STATUS_FLAG_REG_INIT					(1<<12)

#define PL_FUNCTION_FLAG_BINDING				(1<<12)
#define PL_FUNCTION_FLAG_OPTION					(1<<13)
#define PL_FUNCTION_FLAG_GLOVE					(1<<14)
#define PL_FUNCTION_FLAG_STYLUS					(1<<15)
#define PL_FUNCTION_FLAG_WAKEUP_GESTURE		(1<<16)
#define PL_FUNCTION_FLAG_WAKEUP					(PL_FUNCTION_FLAG_WAKEUP_GESTURE)

//High mask
#define PL_STATUS_FLAG_HIGH_MASK_SHIFT		20

#define PL_STATUS_FLAG_PLUG_CAL					(1<<20)
#define PL_STATUS_FLAG_PLUG_MSC					(1<<21)
#define PL_STATUS_FLAG_PLUG_PI					(1<<22)
#define PL_STATUS_FLAG_PLUG_CLIP					(1<<23)
#define PL_STATUS_FLAG_PLUG_WDG					(1<<24)
#define PL_STATUS_FLAG_PAUSE						(1<<27)

#define PL_STATUS_FLAG_NEED_RESET				(1<<28)
#define PL_STATUS_FLAG_NOSUSPEND				(1<<29)
#define PL_STATUS_FLAG_STOP						(1<<30)
#define PL_STATUS_FLAG_FORCE_STOP				(1<<31)

#define PL_STATUS_FLAG_NOISE_MASK			(PL_STATUS_FLAG_NOISE|PL_STATUS_FLAG_VERY_NOISE)
#define PL_STATUS_FLAG_PAUSE_MASK			(0x0ff00000)
#define PL_FUNCTION_FLAG_MASK				(0x000ff000)
#define PL_STATUS_FLAG_LOW_MASK				(0)
#define PL_STATUS_FLAG_MASK				(-1)

#define MAKEWORD(a, b)  ((unsigned short)(((unsigned char)(a)) \
	| ((unsigned short)((unsigned char)(b))) << 8))


// frank add
/* The meaning of the array, if mask != 0, only the mask bit will be modified; if mask = 0, all the value will be written */

static struct reg_config mxt_dwakeup_cfg[] = {
	//dummy
	{MXT_SPT_USERDATA_T38, 0, 0, .buf = {0}, 0, 0x1, .mask = 0x0},
	{MXT_SPT_USERDATA_T38, 0, 0, .buf = {0}, 0, 0x1, .mask = 0x0},
	{MXT_SPT_USERDATA_T38, 0, 0, .buf = {0}, 0, 0x1, .mask = 0x0},
	{MXT_SPT_USERDATA_T38, 0, 0, .buf = {0}, 0, 0x1, .mask = 0x0},
	{MXT_SPT_USERDATA_T38, 0, 0, .buf = {0}, 0, 0x1, .mask = 0x0},

	{.reg = MXT_SPT_CTECONFIG_T46,
		.offset = 2,.buf = {8,20}, .len = 2, .mask = 0},

/*
	{.reg = MXT_PROCI_GESTURE_T92,
		.offset = 0,.buf = {0x3}, .len = 1, .mask = 0x0,.flag = BIT_MASK(DWK_GESTURE)},

	{.reg = MXT_PROCI_TOUCHSEQUENCELOGGER_T93,
		.offset = 0,.buf = {0x0f}, .len = 1, .mask = 0x0,.flag = BIT_MASK(DWK_DCLICK)},
*/

//	{.reg = MXT_PROCI_STYLUS_T47,
//		.offset = 0,.buf = {0}, .len = 1, .mask = 0x1,.flag = BIT_MASK(P_COMMON)},

	{ .reg = MXT_PROCI_RETRANSMISSIONCOMPENSATION_T80,
		.offset = 0,.buf = {0}, .len = 1, .mask = 0x1},

	{.reg = MXT_PROCG_NOISESUPPRESSION_T72,
		.offset = 0,.buf = {0}, .len = 1, .mask = 0x1},

	{.reg = MXT_PROCI_GLOVEDETECTION_T78,
		.offset = 0,.buf = {0}, .len = 1, .mask = 0x1},

	{.reg = MXT_TOUCH_MULTITOUCHSCREEN_T100,
		.offset = 0,.buf = {0}, .len = 1, .mask = 0x2},

	{.reg = MXT_TOUCH_MULTITOUCHSCREEN_T100,
		.offset = 39,.buf = {0x0,0x0}, .len = 2, .mask = 0x2},

	{.reg = MXT_TOUCH_MULTITOUCHSCREEN_T100,
		.offset = 1,.buf = {0x0}, .len = 1, .mask = 0xe0},

	{.reg = MXT_PROCG_NOISESUPSELFCAP_T108,
		.offset = 0,.buf = {0}, .len = 1, .mask = 0x1},

	{.reg = MXT_GEN_POWERCONFIG_T7,
		.offset = 0,.buf = {0x3C,0x0A,0x4,0x40}, .len = 4, .mask = 0},

	{.reg = MXT_GEN_ACQUISITIONCONFIG_T8,
		.offset = 11,.buf = {0x1,0x1}, .len = 2, .mask = 0x2},

	{.reg = MXT_PROCI_AUXTOUCHCONFIG_T104,
		.offset = 0,.buf = {0}, .len = 1, .mask = 0x01},

//	{.reg = MXT_PROCI_TOUCHSUPPRESSION_T42,
//	    .offset = 3,.buf = {0xFF,0xFF}, .len = 2, .mask = 0},
	
	{.reg = MXT_PROCI_TOUCHSUPPRESSION_T42,
    	.offset = 0,.buf = {0}, .len = 1, .mask = 0x01},
	
	{.reg = MXT_SPT_TIMER_T61,.instance = 0,
		.offset = 0,.buf = {0X01}, .len = 1, .mask = 0x01},
		
	{.reg = MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70,.instance = 0,
		.offset = 0,.buf = {0X01}, .len = 1, .mask = 0x01},
		
	{.reg = MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70,.instance = 1,
		.offset = 0,.buf = {0X01}, .len = 1, .mask = 0x01},
		
	{.reg = MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70,.instance = 2,
		.offset = 0,.buf = {0X01}, .len = 1, .mask = 0x01},

	{.reg = MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70,.instance = 3,
		.offset = 0,.buf = {0X01}, .len = 1, .mask = 0x01},
	

	{.reg = MXT_PROCI_TOUCHSEQUENCELOGGER_T93,
		.offset = 0,.buf = {0x0F}, .len = 1, .mask = 0},

//	{.reg = MXT_PROCI_AUXTOUCHCONFIG_T104,
//		.offset = 7,.buf = {0x28}, .len = 1, .mask = 0},

//	{.reg = MXT_PROCI_TOUCHSUPPRESSION_T42,
//		.offset = 1,.buf = {0X0,0x32,0x19,0x80,0x0,0x0,0x0,0x0,0x0,0xFF}, .len = 10, .mask = 0},
	{.reg = MXT_SPT_USERDATA_T38,
			.offset = 0,.buf = {0}, .len = 0, .mask = 0x1, .sleep = 100},

	//dummy
	{MXT_SPT_USERDATA_T38, 0, 0, .buf = {0}, 0, 0x1, .mask = 0x0},
	{MXT_SPT_USERDATA_T38, 0, 0, .buf = {0}, 0, 0x1, .mask = 0x0},
	{MXT_SPT_USERDATA_T38, 0, 0, .buf = {0}, 0, 0x1, .mask = 0x0},
	{MXT_SPT_USERDATA_T38, 0, 0, .buf = {0}, 0, 0x1, .mask = 0x0},
	{MXT_SPT_USERDATA_T38, 0, 0, .buf = {0}, 0, 0x1, .mask = 0x0},
};



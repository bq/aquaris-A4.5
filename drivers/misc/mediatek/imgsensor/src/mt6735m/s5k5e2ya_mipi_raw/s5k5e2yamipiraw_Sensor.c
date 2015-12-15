/* *****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5k5e2yamipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
    
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k5e2yamipiraw_Sensor.h"

/****************************Modify following Strings for debug****************************/
#define PFX "s5k5e2ya_camera_sensor"
#define LOG_1 LOG_INF("s5k5e2ya,MIPI 2LANE\n")
#define LOG_2 LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define MIPI_SETTLEDELAY_AUTO     0
#define MIPI_SETTLEDELAY_MANNUAL  1


#define S5K5E2YA_USE_AWB_OTP
//#define S5K5E2YA_USE_LSC_OTP
#define OTP_SUPPORT_TRULY

#define TRULY_ID           0x02
#define VALID_OTP          0x00

BYTE T_module_id = 0;
BYTE T_flag_otp = 0;

#ifdef S5K5E2YA_USE_AWB_OTP
void S5K5E2YA_MIPI_update_wb_register_from_otp(void);
#endif


//#define CAPTURE_24FPS


static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = S5K5E2YA_SENSOR_ID,
	.checksum_value = 0xa48ebf5d,
	.pre = {
		.pclk = 179200000,				//record different mode's pclk
		.linelength = 2950,				//record different mode's linelength
		.framelength = 2000,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1280,		//record different mode's width of grabwindow
		.grabwindow_height = 960,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 23,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 179200000,
		.linelength = 2950,
		.framelength = 2025,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2560,
		.grabwindow_height = 1920,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 89616000,
		.linelength = 2950,
		.framelength = 2025,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2560,
		.grabwindow_height = 1920,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 150,	
	},
	.normal_video = {
		.pclk = 179200000,
		.linelength = 2950,
		.framelength = 2000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2560,
		.grabwindow_height = 1440,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 179200000,
		.linelength = 2950,
		.framelength = 506,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 179200000,
		.linelength = 2950,
		.framelength = 2000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.margin = 4,
	.min_shutter = 1,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 1,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 3, 
	.pre_delay_frame = 3, 
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,
	
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANNUAL,//MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x20, 0xff},
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 2576, 1936,	  8,	8, 2560, 1920, 1280,  960, 0000, 0000, 1280,  960,	  0,	0, 1280,  960}, // Preview 
 { 2576, 1936,	  8,	8, 2560, 1920, 2560, 1920, 0000, 0000, 2560, 1920,	  0,	0, 2560, 1920}, // capture 
 { 2576, 1936,	  8,  248, 2560, 1440, 2560, 1440, 0000, 0000, 2560, 1440,	  0,	0, 2560, 1440}, // video 
 { 2576, 1936,	  8,    8, 2560, 1440,  640,  480, 0000, 0000,  640,  480,	  0,	0,  640,  480}, //hight speed video 
 { 2576, 1936,	  8,  248, 2560, 1440, 1280,  720, 0000, 0000, 1280,  720,	  0,	0, 1280,  720}};// slim video 


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	//printk("*******zqiang read addr= %x %x*******\n",pu_send_cmd[0],pu_send_cmd[1]);
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}


static void set_dummy()
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);

	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);
  
}	/*	set_dummy  */

static kal_uint32 return_sensor_id()
{
	return ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
	
}


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	   
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	}

	// Update Shutter
	//write_cmos_sensor(0x0104, 0x01);   //group hold
	write_cmos_sensor(0x0202, shutter >> 8);
	write_cmos_sensor(0x0203, shutter & 0xFF);	
	//write_cmos_sensor(0x0104, 0x00);   //group hold
	
	LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);
	
}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	write_shutter(shutter);
}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	return gain>>1;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X	*/
	/* [4:9] = M meams M X		 */
	/* Total gain = M + N /16 X   */

	//
	
 
	reg_gain = gain>>1;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	//write_cmos_sensor(0x0104, 0x01);   //group hold
	write_cmos_sensor(0x0204, reg_gain >> 8);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);  
	//write_cmos_sensor(0x0104, 0x00);
	
	return gain;
}	/*	set_gain  */



//defined but not used
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	if (imgsensor.ihdr_en) {
		
		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)		
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;
			
			
				// Extend frame length first
				write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
				write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);	 
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		
		write_cmos_sensor(0x3508, (se << 4) & 0xFF); 
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F); 

		set_gain(gain);
	}

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x0101,0x00);
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0101,0x01);
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0101,0x02);		
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0101,0x03);
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}

#ifdef OTP_SUPPORT_TRULY

#define USHORT             unsigned short
#define BYTE               unsigned char

//#define INVALID_OTP        0x01

#define GAIN_DEFAULT       0x0100
#define GAIN_GREEN1_ADDR   0x020E
#define GAIN_BLUE_ADDR     0x0212
#define GAIN_RED_ADDR      0x0210
#define GAIN_GREEN2_ADDR   0x0214

USHORT truly_golden_r=0;
USHORT truly_golden_gr=0;
USHORT truly_golden_gb=0;
USHORT truly_golden_b=0;

USHORT truly_current_r=0;
USHORT truly_current_gr=0;
USHORT truly_current_gb=0;
USHORT truly_current_b=0;

kal_uint32 truly_r_ratio;
kal_uint32 truly_b_ratio;

inline BYTE S5K5E2YA_byteread_cmos_sensor(kal_uint32 addr)
{
	BYTE get_byte=0;
	char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
	return get_byte;
}

static void S5K5E2YA_wordwrite_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static void S5K5E2YA_bytewrite_cmos_sensor(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[3] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}


//kal_uint32	truly_golden_r = 0, truly_golden_gr = 0, truly_golden_gb = 0, truly_golden_b = 0;
//kal_uint32	truly_current_r = 0, truly_current_gr = 0, truly_current_gb = 0, truly_current_b = 0;
/*************************************************************************************************
* Function    :  truly_start_read_otp
* Description :  before read otp , set the reading block setting  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  0, reading block setting err
                 1, reading block setting ok 
**************************************************************************************************/
bool truly_start_read_otp(BYTE zone)
{
	S5K5E2YA_bytewrite_cmos_sensor(0x0100, 0x00);//stream off
  S5K5E2YA_bytewrite_cmos_sensor(0x0A00, 0x04);//make initial state
	S5K5E2YA_bytewrite_cmos_sensor(0x0A02, zone);   //Select the page to write by writing to 0xD0000A02 0x00~0x0F
	S5K5E2YA_bytewrite_cmos_sensor(0x0A00, 0x01);   //Enter read mode by writing 01h to 0xD0000A00
	
    mdelay(2);//wait time > 47us

	return 1;
}

/*************************************************************************************************
* Function    :  truly_stop_read_otp
* Description :  after read otp , stop and reset otp block setting  
**************************************************************************************************/
void truly_stop_read_otp()
{
  S5K5E2YA_bytewrite_cmos_sensor(0x0A00, 0x04);//make initial state
	S5K5E2YA_bytewrite_cmos_sensor(0x0A00, 0x00);//Disable NVM controller
//	S5K5E2YA_bytewrite_cmos_sensor(0x0100, 0x01);//stream on
	
}


/*************************************************************************************************
* Function    :  truly_stop_read_otp
* Description :  get otp AWB_WRITTEN_FLAG  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE], if 0x00 , this type has valid or empty otp data, otherwise, invalid otp data
**************************************************************************************************/
BYTE truly_get_otp_AWB_flag(BYTE zone)
{
    BYTE flag_AWB = 0x01;
    truly_start_read_otp(zone);
    flag_AWB = S5K5E2YA_byteread_cmos_sensor(0x0A42);
    truly_stop_read_otp();

    printk("S5K5E2 truly++: flag_AWB := 0x%02x \n", flag_AWB );
	return flag_AWB;
}

/*************************************************************************************************
* Function    :  truly_get_otp_LSC_flag
* Description :  get otp LSC_WRITTEN_FLAG  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE], if 0x00 , this type has valid or empty otp data, otherwise, invalid otp data
**************************************************************************************************/
BYTE truly_get_otp_LSC_flag(BYTE zone)
{
    BYTE flag_LSC = 0x01;
    truly_start_read_otp(zone);
    flag_LSC = S5K5E2YA_byteread_cmos_sensor(0x0A43);
    truly_stop_read_otp();

    printk("S5K5E2 truly++: flag_LSC := 0x%02x \n", flag_LSC );
	return flag_LSC;
}

/*************************************************************************************************
* Function    :  truly_get_otp_T_module_id
* Description :  get otp MID value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : module ID data , TRULY ID is 0x0001            
**************************************************************************************************/
BYTE truly_get_otp_T_module_id(BYTE zone)
{
	BYTE T_module_id = 0;

	truly_start_read_otp(zone);

	T_module_id = S5K5E2YA_byteread_cmos_sensor(0x0A04);

	truly_stop_read_otp();

	printk("S5K5E2 truly++: Module ID = 0x%02x.\n",T_module_id);

	return T_module_id;
}


/*************************************************************************************************
* Function    :  truly_get_otp_date
* Description :  get otp date value    
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f    
**************************************************************************************************/
bool truly_get_otp_date(BYTE zone) 
{
	BYTE year  = 0;
	BYTE month = 0;
	BYTE day   = 0;

	truly_start_read_otp(zone);

	year  = S5K5E2YA_byteread_cmos_sensor(0x0A05);
	month = S5K5E2YA_byteread_cmos_sensor(0x0A06);
	day   = S5K5E2YA_byteread_cmos_sensor(0x0A07);

	truly_stop_read_otp();

    printk("S5K5E2 truly++: date=%02d.%02d.%02d \n", year,month,day);

	return 1;
}



/*************************************************************************************************
* Function    :  truly_get_otp_lens_id
* Description :  get otp LENS_ID value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : LENS ID data             
**************************************************************************************************/
BYTE truly_get_otp_lens_id(BYTE zone)
{
	BYTE lens_id = 0;

	truly_start_read_otp(zone);
	
	lens_id = S5K5E2YA_byteread_cmos_sensor(0x0A08);
	truly_stop_read_otp();

	printk("S5K5E2 truly++: Lens ID = 0x%02x.\n",lens_id);

	return lens_id;
}


/*************************************************************************************************
* Function    :  truly_get_otp_vcm_id
* Description :  get otp VCM_ID value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : VCM ID data             
**************************************************************************************************/
BYTE truly_get_otp_vcm_id(BYTE zone)
{
	BYTE vcm_id = 0;

	truly_start_read_otp(zone);
	
	vcm_id = S5K5E2YA_byteread_cmos_sensor(0x0A09);

	truly_stop_read_otp();

	printk("S5K5E2 truly++: VCM ID = 0x%02x.\n",vcm_id);

	return vcm_id;	
}


/*************************************************************************************************
* Function    :  truly_get_otp_driverIC_id
* Description :  get otp driverIC id value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : driver ID data             
**************************************************************************************************/
BYTE truly_get_otp_driverIC_id(BYTE zone)
{
	BYTE driverIC_id = 0;

	truly_start_read_otp(zone);
	
	driverIC_id = S5K5E2YA_byteread_cmos_sensor(0x0A0A);

	truly_stop_read_otp();

	printk("S5K5E2 truly++: Driver ID = 0x%02x.\n",driverIC_id);

	return driverIC_id;
}

/*************************************************************************************************
* Function    :  truly_get_light_id
* Description :  get otp environment light temperature value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                        other value : driver ID data     
			            BIT0:D65(6500K) EN
						BIT1:D50(5100K) EN
						BIT2:CWF(4000K) EN
						BIT3:A Light(2800K) EN
**************************************************************************************************/
BYTE truly_get_light_id(BYTE zone)
{
	BYTE light_id = 0;

	truly_start_read_otp(zone);	

	light_id = S5K5E2YA_byteread_cmos_sensor(0x0A0B);

	truly_stop_read_otp();

	printk("S5K5E2 truly++: Light ID: 0x%02x.\n",light_id);

	return light_id;
}


/*************************************************************************************************
* Function    :  truly_otp_lenc_update
* Description :  Update lens correction 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [bool] 0 : OTP data fail 
                        1 : otp_lenc update success            
**************************************************************************************************/
bool truly_otp_lenc_update(USHORT zone)
{
    BYTE  flag_LSC = 0x01;
    flag_LSC = truly_get_otp_LSC_flag(zone);
    printk("S5K5E2 truly++: flag_LSC =0x%x \n",flag_LSC);

    //
    if(flag_LSC == 0)
    {
        //LSC correct
        S5K5E2YA_bytewrite_cmos_sensor(0x0B00,0x01);
        S5K5E2YA_bytewrite_cmos_sensor(0x3400,0x00);
    }
    //Load LSC SRAM data
    else
    {
        printk("S5K5E2 truly++: flag_LSC =0x%x,LSC Load SRAM Data!!!!! \n",flag_LSC);
        return 0;
    }
    return 1;
}



/*************************************************************************************************
* Function    :  truly_wb_gain_set
* Description :  Set WB ratio to register gain setting  512x
* Parameters  :  [int] truly_r_ratio : R ratio data compared with golden module R
                       truly_b_ratio : B ratio data compared with golden module B
* Return      :  [bool] 0 : set wb fail 
                        1 : WB set success            
**************************************************************************************************/

bool truly_wb_gain_set()
{
    USHORT R_GAIN;
    USHORT B_GAIN;
    USHORT Gr_GAIN;
    USHORT Gb_GAIN;
    USHORT G_GAIN;

    if(!truly_r_ratio || !truly_b_ratio)
    {
        printk("S5K5E2 truly++: OTP WB ratio Data Err!\n");
        return 0;
    }

    if(truly_r_ratio >= 512 )
    {
        if(truly_b_ratio>=512) 
        {
            R_GAIN = (USHORT)(GAIN_DEFAULT * truly_r_ratio / 512);						
            G_GAIN = GAIN_DEFAULT;
            B_GAIN = (USHORT)(GAIN_DEFAULT * truly_b_ratio / 512);
        }
        else
        {
            R_GAIN =  (USHORT)(GAIN_DEFAULT*truly_r_ratio / truly_b_ratio );
            G_GAIN = (USHORT)(GAIN_DEFAULT*512 / truly_b_ratio );
            B_GAIN = GAIN_DEFAULT;    
        }
    }
    else                      
    {		
        if(truly_b_ratio >= 512)
        {
            R_GAIN = GAIN_DEFAULT;
            G_GAIN = (USHORT)(GAIN_DEFAULT*512 /truly_r_ratio);		
            B_GAIN =  (USHORT)(GAIN_DEFAULT*truly_b_ratio / truly_r_ratio );
        } 
        else 
        {
            Gr_GAIN = (USHORT)(GAIN_DEFAULT*512/ truly_r_ratio );						
            Gb_GAIN = (USHORT)(GAIN_DEFAULT*512/truly_b_ratio );						
            if(Gr_GAIN >= Gb_GAIN)						
            {						
                R_GAIN = GAIN_DEFAULT;						
                G_GAIN = (USHORT)(GAIN_DEFAULT *512/ truly_r_ratio );						
                B_GAIN =  (USHORT)(GAIN_DEFAULT*truly_b_ratio / truly_r_ratio );						
            } 
            else
            {						
                R_GAIN =  (USHORT)(GAIN_DEFAULT*truly_r_ratio  / truly_b_ratio);						
                G_GAIN = (USHORT)(GAIN_DEFAULT*512 / truly_b_ratio );						
                B_GAIN = GAIN_DEFAULT;
            }
        }        
    }

    S5K5E2YA_wordwrite_cmos_sensor(GAIN_RED_ADDR, R_GAIN);		
    S5K5E2YA_wordwrite_cmos_sensor(GAIN_BLUE_ADDR, B_GAIN);     
    S5K5E2YA_wordwrite_cmos_sensor(GAIN_GREEN1_ADDR, G_GAIN); //Green 1 default gain 1x		
    S5K5E2YA_wordwrite_cmos_sensor(GAIN_GREEN2_ADDR, G_GAIN); //Green 2 default gain 1x
    printk("S5K5E2 truly++: OTP WB Update Finished! \n");
    
	return 1;
}



/*************************************************************************************************
* Function    :  truly_get_otp_wb
* Description :  Get WB data    
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f      
**************************************************************************************************/
bool truly_get_otp_wb(BYTE zone)
{
	BYTE temph = 0;
	BYTE templ = 0;
	truly_golden_r = 0, truly_golden_gr = 0, truly_golden_gb = 0, truly_golden_b = 0;
	truly_current_r = 0, truly_current_gr = 0, truly_current_gb = 0, truly_current_b = 0;

    truly_start_read_otp(zone);

	temph = S5K5E2YA_byteread_cmos_sensor(0x0A19);  
	templ = S5K5E2YA_byteread_cmos_sensor(0x0A18);   
	truly_golden_r  = (USHORT)templ + ((USHORT)temph << 8);

	temph = S5K5E2YA_byteread_cmos_sensor(0x0A1B);  
	templ = S5K5E2YA_byteread_cmos_sensor(0x0A1A);   
	truly_golden_gr  = (USHORT)templ + ((USHORT)temph << 8);

	temph = S5K5E2YA_byteread_cmos_sensor(0x0A1D);  
	templ = S5K5E2YA_byteread_cmos_sensor(0x0A1C);   
	truly_golden_gb  = (USHORT)templ + ((USHORT)temph << 8);

	temph = S5K5E2YA_byteread_cmos_sensor(0x0A1F);  
	templ = S5K5E2YA_byteread_cmos_sensor(0x0A1E);   
	truly_golden_b  = (USHORT)templ + ((USHORT)temph << 8);

	temph = S5K5E2YA_byteread_cmos_sensor(0x0A11);  
	templ = S5K5E2YA_byteread_cmos_sensor(0x0A10);   
	truly_current_r  = (USHORT)templ + ((USHORT)temph << 8);

	temph = S5K5E2YA_byteread_cmos_sensor(0x0A13);  
	templ = S5K5E2YA_byteread_cmos_sensor(0x0A12);   
	truly_current_gr  = (USHORT)templ + ((USHORT)temph << 8);

	temph = S5K5E2YA_byteread_cmos_sensor(0x0A15);  
	templ = S5K5E2YA_byteread_cmos_sensor(0x0A14);   
	truly_current_gb  = (USHORT)templ + ((USHORT)temph << 8);

	temph = S5K5E2YA_byteread_cmos_sensor(0x0A17);  
	templ = S5K5E2YA_byteread_cmos_sensor(0x0A16);   
	truly_current_b  = (USHORT)templ + ((USHORT)temph << 8);

	truly_stop_read_otp();
	
    printk("S5K5E2 truly++: truly_golden_r=%d,truly_golden_gr=%d,truly_golden_gb=%d,truly_golden_b=%d \n",truly_golden_r,truly_golden_gr,truly_golden_gb,truly_golden_b);
    printk("S5K5E2 truly++: truly_current_r=%d,truly_current_gr=%d,truly_current_gb=%d,truly_current_b=%d \n",truly_current_r,truly_current_gr,truly_current_gb,truly_current_b);    
	return 1;
}


/*************************************************************************************************
* Function    :  truly_otp_wb_update
* Description :  Update WB correction 
* Return      :  [bool] 0 : OTP data fail 
                        1 : otp_WB update success            
**************************************************************************************************/
bool truly_otp_wb_update(BYTE zone)
{
	USHORT golden_g, current_g;


	if(!truly_get_otp_wb(zone))  // get wb data from otp
		return 0;

	golden_g = (truly_golden_gr + truly_golden_gb) / 2;
	current_g = (truly_current_gr + truly_current_gb) / 2;

	if(!golden_g || !current_g || !truly_golden_r || !truly_golden_b || !truly_current_r || !truly_current_b)
	{
		printk("S5K5E2 truly++: WB update Err !\n");
		return 0;
	}

	truly_r_ratio = 512 * truly_golden_r * current_g /( golden_g * truly_current_r );
	truly_b_ratio = 512 * truly_golden_b * current_g /( golden_g * truly_current_b );
  printk("S5K5E2 truly++: truly_r_ratio=%d, truly_b_ratio=%d \n",truly_r_ratio, truly_b_ratio);
	truly_wb_gain_set();

	printk("S5K5E2 truly++: WB update finished! \n");

	return 1;
}

/*************************************************************************************************
* Function    :  truly_otp_update()
* Description :  update otp data from otp , it otp data is valid, 
                 it include get ID and WB update function  
* Return      :  [bool] 0 : update fail
                        1 : update success
**************************************************************************************************/
bool truly_otp_update()
{
	BYTE zone = 0x02;//start from Page 2
	BYTE FLG = 0x00;
	BYTE MID = 0x00,LENS_ID= 0x00,VCM_ID= 0x00;
	int i;
	
	printk("****enter truly_otp_update****\n");
	for(i=0;i<3;i++)
	{
		T_module_id = truly_get_otp_T_module_id(zone);
		T_flag_otp = truly_get_otp_AWB_flag(zone);
		if(T_flag_otp == VALID_OTP &&  T_module_id==TRULY_ID)
			break;
		else
	  	zone ++;
	}
	if(i==3)
	{
		printk("S5K5E2 truly++: No OTP Data or OTP data is invalid!!\n");
		return 0;
	}
 //   truly_get_otp_date(zone);
 //   LENS_ID=	truly_get_otp_lens_id(zone);
 //   VCM_ID=	truly_get_otp_vcm_id(zone);
 //   truly_get_otp_driverIC_id(zone);
 //   truly_get_light_id(zone);
 
	truly_otp_wb_update(zone);
	
	if(!truly_otp_lenc_update(zone))
	{
		printk("Update LSC Err\n");
		return 0;
	}
	else
		printk("Load LSC OTP Finished\n");
		
//	Sleep(30);

  return 1;
	
}
#endif

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/

static void sensor_init(void)
{
	LOG_INF("E\n");

   // +++++++++++++++++++++++++++//                                                               
	// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off
	
	// Clock Setting
	write_cmos_sensor(0x3000,0x04);       //   ct_ld_start                                    
	write_cmos_sensor(0x3002,0x03);       //   ct_sl_start                                    
	write_cmos_sensor(0x3003,0x04);       //   ct_sl_margin                                   
	write_cmos_sensor(0x3004,0x02);       //   ct_rx_start                                    
	write_cmos_sensor(0x3005,0x00);       //   ct_rx_margin (MSB)                             
	write_cmos_sensor(0x3006,0x10);       //   ct_rx_margin (LSB)                             
	write_cmos_sensor(0x3007,0x03);       //   ct_tx_start                                    
	write_cmos_sensor(0x3008,0x55);       //   ct_tx_width                                    
	write_cmos_sensor(0x3039,0x00);       //   cintc1_margin (10 --> 00)                      
	write_cmos_sensor(0x303A,0x00);       //   cintc2_margin (10 --> 00)                      
	write_cmos_sensor(0x303B,0x00);       //   offs_sh                                        
	write_cmos_sensor(0x3009,0x05);       //   ct_srx_margin                                  
	write_cmos_sensor(0x300A,0x55);       //   ct_stx_width                                   
	write_cmos_sensor(0x300B,0x38);       //   ct_dstx_width                                  
	write_cmos_sensor(0x300C,0x10);       //   ct_stx2dstx                                    
	write_cmos_sensor(0x3012,0x05);       //   ct_cds_start                                   
	write_cmos_sensor(0x3013,0x00);       //   ct_s1s_start                                   
	write_cmos_sensor(0x3014,0x22);       //   ct_s1s_end                                     
	write_cmos_sensor(0x300E,0x79);       //   ct_s3_width                                    
	write_cmos_sensor(0x3010,0x68);       //   ct_s4_width                                    
	write_cmos_sensor(0x3019,0x03);       //   ct_s4d_start                                   
	write_cmos_sensor(0x301A,0x00);       //   ct_pbr_start                                   
	write_cmos_sensor(0x301B,0x06);       //   ct_pbr_width                                   
	write_cmos_sensor(0x301C,0x00);       //   ct_pbs_start                                   
	write_cmos_sensor(0x301D,0x22);       //   ct_pbs_width                                   
	write_cmos_sensor(0x301E,0x00);       //   ct_pbr_ob_start                                
	write_cmos_sensor(0x301F,0x10);       //   ct_pbr_ob_width                                
	write_cmos_sensor(0x3020,0x00);       //   ct_pbs_ob_start                                
	write_cmos_sensor(0x3021,0x00);       //   ct_pbs_ob_width                                
	write_cmos_sensor(0x3022,0x0A);       //   ct_cds_lim_start                               
	write_cmos_sensor(0x3023,0x1E);       //   ct_crs_start                                   
	write_cmos_sensor(0x3024,0x00);       //   ct_lp_hblk_cds_start (MSB)                     
	write_cmos_sensor(0x3025,0x00);       //   ct_lp_hblk_cds_start (LSB)                     
	write_cmos_sensor(0x3026,0x00);       //   ct_lp_hblk_cds_end (MSB)                       
	write_cmos_sensor(0x3027,0x00);       //   ct_lp_hblk_cds_end (LSB)                       
	write_cmos_sensor(0x3028,0x1A);       //   ct_rmp_off_start                               
	write_cmos_sensor(0x3015,0x00);       //   ct_rmp_rst_start (MSB)                         
	write_cmos_sensor(0x3016,0x84);       //   ct_rmp_rst_start (LSB)                         
	write_cmos_sensor(0x3017,0x00);       //   ct_rmp_sig_start (MSB)                         
	write_cmos_sensor(0x3018,0xA0);       //   ct_rmp_sig_start (LSB)                         
	write_cmos_sensor(0x302B,0x10);       //   ct_cnt_margin                                  
	write_cmos_sensor(0x302C,0x0A);       //   ct_rmp_per                                     
	write_cmos_sensor(0x302D,0x06);       //   ct_cnt_ms_margin1                              
	write_cmos_sensor(0x302E,0x05);       //   ct_cnt_ms_margin2                              
	write_cmos_sensor(0x302F,0x0E);       //   rst_mx                                         
	write_cmos_sensor(0x3030,0x2F);       //   sig_mx                                         
	write_cmos_sensor(0x3031,0x08);       //   ct_latch_start                                 
	write_cmos_sensor(0x3032,0x05);       //   ct_latch_width                                 
	write_cmos_sensor(0x3033,0x09);       //   ct_hold_start                                  
	write_cmos_sensor(0x3034,0x05);       //   ct_hold_width                                  
	write_cmos_sensor(0x3035,0x00);       //   ct_lp_hblk_dbs_start (MSB)                     
	write_cmos_sensor(0x3036,0x00);       //   ct_lp_hblk_dbs_start (LSB)                     
	write_cmos_sensor(0x3037,0x00);       //   ct_lp_hblk_dbs_end (MSB)                       
	write_cmos_sensor(0x3038,0x00);       //   ct_lp_hblk_dbs_end (LSB)                       
	write_cmos_sensor(0x3088,0x06);       //   ct_lat_lsb_offset_start1                       
	write_cmos_sensor(0x308A,0x08);       //   ct_lat_lsb_offset_end1                         
	write_cmos_sensor(0x308C,0x05);       //   ct_lat_lsb_offset_start2                       
	write_cmos_sensor(0x308E,0x07);       //   ct_lat_lsb_offset_end2                         
	write_cmos_sensor(0x3090,0x06);       //   ct_conv_en_offset_start1                       
	write_cmos_sensor(0x3092,0x08);       //   ct_conv_en_offset_end1                         
	write_cmos_sensor(0x3094,0x05);       //   ct_conv_en_offset_start2                       
	write_cmos_sensor(0x3096,0x21);       //   ct_conv_en_offset_end2                         
	write_cmos_sensor(0x3099,0x0E);       //  cds_option ([3]:crs switch disable, s3,s4 streng
	write_cmos_sensor(0x3070,0x10);       //  comp1_bias (default:77)                         
	write_cmos_sensor(0x3085,0x11);       //  comp1_bias (gain1~4)                            
	write_cmos_sensor(0x3086,0x01);       //  comp1_bias (gain4~8) modified 813               
                         
	write_cmos_sensor(0x3064,0x00);       //  Multiple sampling(gainx8,x16)                   
	write_cmos_sensor(0x3062,0x08);       //  off_rst                                         
	write_cmos_sensor(0x3061,0x11);       //  dbr_tune_rd (default :08)                       
	write_cmos_sensor(0x307B,0x20);       //  dbr_tune_rgsl (default :08)                     
	write_cmos_sensor(0x3068,0x00);       //  RMP BP bias sampling                            
	write_cmos_sensor(0x3074,0x00);       //  Pixel bias sampling [2]:Default L               
	write_cmos_sensor(0x307D,0x00);       //  VREF sampling [1]                               
	write_cmos_sensor(0x3045,0x01);       //  ct_opt_l1_start                                 
	write_cmos_sensor(0x3046,0x05);       //  ct_opt_l1_width                                 
	write_cmos_sensor(0x3047,0x78);                                               
	write_cmos_sensor(0x307F,0xB1);       // RDV_OPTION[5:4], RG default high                 
	write_cmos_sensor(0x3098,0x01);       // CDS_OPTION[16] SPLA-II enable                    
	write_cmos_sensor(0x305C,0xF6);       // lob_extension[6]                                 
	write_cmos_sensor(0x306B,0x10);                                                 
	write_cmos_sensor(0x3063,0x27);       //  ADC_SAT 490mV --> 610mV                         
	//write_cmos_sensor(0x3400,0x01);       //  GAS bypass                                      
	write_cmos_sensor(0x3235,0x49);       //  L/F-ADLC on                                     
	write_cmos_sensor(0x3233,0x00);       //  D-pedestal L/F ADLC off (1FC0h)                 
	write_cmos_sensor(0x3234,0x00);                                                 
	write_cmos_sensor(0x3300,0x0C);       // BPC bypass                                       
                             
	write_cmos_sensor(0x3203,0x45);       // ADC_OFFSET_EVEN                                  
	write_cmos_sensor(0x3205,0x4D);       // ADC_OFFSET_ODD                                   
	write_cmos_sensor(0x320B,0x40);       // ADC_DEFAULT                                      
	write_cmos_sensor(0x320C,0x06);       // ADC_MAX                                          
	write_cmos_sensor(0x320D,0xC0);  

	write_cmos_sensor(0x3929,0x07); 
	write_cmos_sensor(0x0B00,0x01);	
	//set mipi non-continue mode
	
	#ifdef OTP_SUPPORT_TRULY
	if(!truly_otp_update())
   	LOG_INF("TRULY OTP LOAD Err or NO TRULY MODULE\n");
   else
   	LOG_INF("TRULY OTP LOAD SUCCESS\n");
	#endif

	#ifdef S5K5E2YA_USE_AWB_OTP
	if(T_module_id != TRULY_ID)
	{
	S5K5E2YA_MIPI_update_wb_register_from_otp();
	write_cmos_sensor(0x0B00,0x01);
	write_cmos_sensor(0x3400,0x00);	
  }
	#endif
 	
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 

	mdelay(50);
	
	
}	/*	sensor_init  */




static void preview_setting(void)
{
	// +++++++++++++++++++++++++++//                                                               
	// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off
	
	// Clock Setting
	write_cmos_sensor(0x0305,0x06); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xE0); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0x80); 
	write_cmos_sensor(0x3C1C,0x58); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x07); // frame_length_lines : def. 990d (--> 3C8 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xD0);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0x08); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x07); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0x87); 
	write_cmos_sensor(0x034C,0x05); // x_output size : def. 1288d
	write_cmos_sensor(0x034D,0x00); 
	write_cmos_sensor(0x034E,0x03); // y_output size : def. 968d
	write_cmos_sensor(0x034F,0xC0); 
	
	
	//Digital Binning
	write_cmos_sensor(0x0900,0x01);	//2x2 Binning
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0387,0x03);
	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);
	
	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x00);
	write_cmos_sensor(0x340C,0x00);
	write_cmos_sensor(0x340D,0x00);
	write_cmos_sensor(0x340E,0x00);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	//write_cmos_sensor(0x3400,0x01);
	
	#ifdef S5K5E2YA_USE_AWB_OTP
	write_cmos_sensor(0x0B00,0x01);
	write_cmos_sensor(0x3400,0x00);	
	#endif
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 
	
	 
}	/*	preview_setting  */
	

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	if (currefps == 150) { //15fps for PIP
	// +++++++++++++++++++++++++++//                                                               
	// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off

	
	// Clock Setting
	write_cmos_sensor(0x0305,0x06); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xDA); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x01); //PLLS 
	
	//S30CCC0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x01); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0xB4); 
	write_cmos_sensor(0x3C1C,0x54); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x07); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xE9);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0x08); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x07); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0x87); 
	write_cmos_sensor(0x034C,0x0A); // x_output size : def. 2560d 
	write_cmos_sensor(0x034D,0x00); 
	write_cmos_sensor(0x034E,0x07); // y_output size : def. 1920d
	write_cmos_sensor(0x034F,0x80); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x00);	//0x0 Binning
	write_cmos_sensor(0x0901,0x20);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0387,0x01);
	
	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);
	
	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x00);
	write_cmos_sensor(0x340C,0x00);
	write_cmos_sensor(0x340D,0x00);
	write_cmos_sensor(0x340E,0x00);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	//write_cmos_sensor(0x3400,0x01);
	
	#ifdef S5K5E2YA_USE_AWB_OTP
	write_cmos_sensor(0x0B00,0x01);
	write_cmos_sensor(0x3400,0x00);	
	#endif
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 
	
	
	}
	else{// Reset for operation     30fps for normal capture                                                                    
	write_cmos_sensor(0x0100,0x00); //stream off
	mdelay(40);
	
	// Clock Setting
	write_cmos_sensor(0x0305,0x06); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xE0); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	//S3CC0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0x80); 
	write_cmos_sensor(0x3C1C,0x58); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x07); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xE9);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0x08); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x07); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0x87); 
	write_cmos_sensor(0x034C,0x0A); // x_output size : def. 2560d 
	write_cmos_sensor(0x034D,0x00); 
	write_cmos_sensor(0x034E,0x07); // y_output size : def. 1920d
	write_cmos_sensor(0x034F,0x80); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x00);	//0x0 Binning
	write_cmos_sensor(0x0901,0x20);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0387,0x01);
	
	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);

	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x00);
	write_cmos_sensor(0x340C,0x00);
	write_cmos_sensor(0x340D,0x00);
	write_cmos_sensor(0x340E,0x00);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	//write_cmos_sensor(0x3400,0x01);

	// streaming ON
	#ifdef S5K5E2YA_USE_AWB_OTP
	write_cmos_sensor(0x0B00,0x01);
	write_cmos_sensor(0x3400,0x00);	
	#endif
	write_cmos_sensor(0x0100,0x01); 
	
	
	}
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	
	//5.1.3 Video 2592x1944 30fps 24M MCLK 2lane 864Mbps/lane
	// +++++++++++++++++++++++++++//                                                               
// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off

	
	// Clock Setting
	write_cmos_sensor(0x0305,0x06); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xE0); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	//S30C0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0x80); 
	write_cmos_sensor(0x3C1C,0x58); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode   
	
	// Size Setting
	write_cmos_sensor(0x0340,0x07); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xD0);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);

	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0xF8); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x06); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0x97); 
	write_cmos_sensor(0x034C,0x0A); // x_output size : def. 2560d 
	write_cmos_sensor(0x034D,0x00); 
	write_cmos_sensor(0x034E,0x05); // y_output size : def. 1920d
	write_cmos_sensor(0x034F,0xA0); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x00);	//0x0 Binning
	write_cmos_sensor(0x0901,0x20);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0387,0x01);

	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);

	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x27);
	write_cmos_sensor(0x340C,0x01);
	write_cmos_sensor(0x340D,0xA3);
	write_cmos_sensor(0x340E,0x9E);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	//write_cmos_sensor(0x3400,0x01);
	
	#ifdef S5K5E2YA_USE_AWB_OTP
	write_cmos_sensor(0x0B00,0x01);
	write_cmos_sensor(0x3400,0x00);	
	#endif
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 

	
		
}
static void hs_video_setting()
{
	LOG_INF("E\n");         
	//VGA 120fps

	write_cmos_sensor(0x0100,0x00); //stream off
	write_cmos_sensor(0x0136,0x18);
	write_cmos_sensor(0x0137,0x00);
	// Clock Setting
	write_cmos_sensor(0x0305,0x06); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xE0); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	//S30CCC0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0x80); 
	write_cmos_sensor(0x3C1C,0x58); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x01); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xFA);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0x08); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def.   
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x07); // y_addr_end : def.
	write_cmos_sensor(0x034B,0x87); 
	write_cmos_sensor(0x034C,0x02); // x_output size : def. 640d 
	write_cmos_sensor(0x034D,0x80); 
	write_cmos_sensor(0x034E,0x01); // y_output size : def. 480d
	write_cmos_sensor(0x034F,0xE0); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x01); //0x0 Binning
	write_cmos_sensor(0x0901,0x44);
	write_cmos_sensor(0x0383,0x03);
	write_cmos_sensor(0x0387,0x07);
	
	
	//Integration time	
	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x20);
	write_cmos_sensor(0x0202,0x01);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);
	
	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x00);
	write_cmos_sensor(0x340C,0x00);
	write_cmos_sensor(0x340D,0x00);
	write_cmos_sensor(0x340E,0x00);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	//write_cmos_sensor(0x3400,0x00);
	
	#ifdef S5K5E2YA_USE_AWB_OTP
	write_cmos_sensor(0x0B00,0x01);
	write_cmos_sensor(0x3400,0x00);	
	#endif
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 



	//full size 60fps
/*
	// +++++++++++++++++++++++++++//                                                               
	// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off
	
	// Clock Setting
	write_cmos_sensor(0x0305,0x05); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xCA); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	//S30CCC0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0xC9); 
	write_cmos_sensor(0x3C1C,0x59); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x04); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0x4F);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x01); // x_addr_start
	write_cmos_sensor(0x0345,0x40); 
	write_cmos_sensor(0x0346,0x01); // y_addr_start
	write_cmos_sensor(0x0347,0xA4); 
	write_cmos_sensor(0x0348,0x08); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0xBF); 
	write_cmos_sensor(0x034A,0x05); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0xDB); 
	write_cmos_sensor(0x034C,0x07); // x_output size : def. 2560d 
	write_cmos_sensor(0x034D,0x80); 
	write_cmos_sensor(0x034E,0x04); // y_output size : def. 1920d
	write_cmos_sensor(0x034F,0x38); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x00);	//0x0 Binning
	write_cmos_sensor(0x0901,0x20);
	write_cmos_sensor(0x0387,0x01);
	
	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);
	
	write_cmos_sensor(0x3407,0x4C);
	write_cmos_sensor(0x3408,0x01);
	write_cmos_sensor(0x3409,0xF3);
	write_cmos_sensor(0x340A,0x24);
	write_cmos_sensor(0x340B,0x1A);
	write_cmos_sensor(0x340C,0x03);
	write_cmos_sensor(0x340D,0x6E);
	write_cmos_sensor(0x340E,0xE8);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	write_cmos_sensor(0x3400,0x01);
	
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 
*/

	
}

static void slim_video_setting()
{
	LOG_INF("E\n");
	// +++++++++++++++++++++++++++//                                                               
// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off
	
	// Clock Setting
	write_cmos_sensor(0x0305,0x06); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xE0); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	//S30CCC0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0x80); 
	write_cmos_sensor(0x3C1C,0x58); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x07); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xD0);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0xF8); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x06); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0x97); 
	write_cmos_sensor(0x034C,0x05); // x_output size : def. 2560d 
	write_cmos_sensor(0x034D,0x00); 
	write_cmos_sensor(0x034E,0x02); // y_output size : def. 1920d
	write_cmos_sensor(0x034F,0xD0); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x01);	//2x2 Binning
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0387,0x03);
	
	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);
	
	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x00);
	write_cmos_sensor(0x340C,0x02);
	write_cmos_sensor(0x340D,0x00);
	write_cmos_sensor(0x340E,0x00);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	//write_cmos_sensor(0x3400,0x01);
	
	#ifdef S5K5E2YA_USE_AWB_OTP
	write_cmos_sensor(0x0B00,0x01);
	write_cmos_sensor(0x3400,0x00);	
	#endif
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 

}


/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
//LC--zbl--20150611--for camera dev_info start
#undef   SLT_DEVINFO_CMM
#define  SLT_DEVINFO_CMM
#ifdef   SLT_DEVINFO_CMM
#include <linux/dev_info.h>
#endif
//LC--zbl--20150611--for camera dev_info end
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
//LC--zbl--20150611--for camera dev_info start
	#ifdef SLT_DEVINFO_CMM
	struct devinfo_struct *dev;
	dev=(struct devinfo_struct*)kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
		dev->device_type = "CCM-S";
		dev->device_module = "Xinli";
		dev->device_vendor ="unknow";  
		dev->device_ic	 = "s5k5e2ya";
		dev->device_version = "unknow";
		dev->device_info = "500W";
	#endif
//LC--zbl--20150611--for camera dev_info end
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	
			#ifdef SLT_DEVINFO_CMM
				dev->device_used	=	DEVINFO_USED;
				devinfo_check_add_device(dev);
			#endif
  
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
//LC--zbl--20150611--for camera dev_info start
		#ifdef SLT_DEVINFO_CMM
			dev->device_used	=	DEVINFO_UNUSED;
			devinfo_check_add_device(dev);
		#endif
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0; 
	LOG_1;
	LOG_2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}		 
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	
	
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length;
  
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            }
			//set_dummy();
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK

				
		write_cmos_sensor(0x0200, 0x0D);
		write_cmos_sensor(0x0201, 0x78);
		write_cmos_sensor(0x0202, 0x04);
		write_cmos_sensor(0x0203, 0xE2);
		write_cmos_sensor(0x0204, 0x00);
		write_cmos_sensor(0x0205, 0x20);
		
		write_cmos_sensor(0x020E, 0x01);
		write_cmos_sensor(0x020F, 0x00);
		write_cmos_sensor(0x0210, 0x01);
		write_cmos_sensor(0x0211, 0x00);
		write_cmos_sensor(0x0212, 0x01);
		write_cmos_sensor(0x0213, 0x00);
		write_cmos_sensor(0x0214, 0x01);
		write_cmos_sensor(0x0215, 0x00);
		
		write_cmos_sensor(0x3400, 0x01);
		write_cmos_sensor(0x0601, 0x02);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0601, 0x00);
	}	 
	write_cmos_sensor(0x3200, 0x00);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	LOG_INF("feature_id = %d", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			break;
		case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			if((sensor_reg_data->RegData>>8)>0)
			   write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			else
				write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", *feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR:
			//LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_16);
			LOG_INF("Warning! Not Support IHDR Feature");
			spin_lock(&imgsensor_drv_lock);
			//imgsensor.ihdr_en = (BOOL)*feature_data_16;
            imgsensor.ihdr_en = KAL_FALSE;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
            break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
		default:
			break;
	}
  
	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5K5E2YA_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5693_MIPI_RAW_SensorInit	*/


#ifdef S5K5E2YA_USE_AWB_OTP

#define RG_TYPICAL 0x349
#define BG_TYPICAL 0x2B9


kal_uint32 tRG_Ratio_typical = RG_TYPICAL;
kal_uint32 tBG_Ratio_typical = BG_TYPICAL;
void S5K5E2YA_MIPI_LSC_from_otp(void);

void S5K5E2YA_MIPI_read_otp_wb(struct S5K5E2YA_MIPI_otp_struct *otp)
{
	kal_uint32 R_avg_H, R_avg_L, G_avg_H,G_avg_L,B_avg_H,B_avg_L;
	kal_uint32 R_gold_H, R_gold_L, G_gold_H,G_gold_L,B_gold_H,B_gold_L;
	kal_uint32 ret,ret1;
	kal_uint16 PageCount;
	for(PageCount = 3; PageCount>=2; PageCount--)
	{
		LOG_INF("[S5K5E2YA] [OTP] PageCount=%d\n", PageCount);	
		write_cmos_sensor(0x0100,0x00);
		mdelay(10);
		write_cmos_sensor(0x0A00,0x04);
		write_cmos_sensor(0x0A02, PageCount); //page set
		write_cmos_sensor(0x0A00, 0x01); //otp enable read
		mdelay(100);
	        ret = read_cmos_sensor(0x0A23);
	        ret1 = read_cmos_sensor(0x0A43);
	       LOG_INF("[S5K5E2YA] [OTP] ret=%d,ret1=%d\n", ret,ret1);	
		if(ret==0x1)
		{
			LOG_INF("[S5K5E2YA] [OTP] group1 is done");
			otp->module_integrator_id = read_cmos_sensor(0x0A04);
			otp->year = read_cmos_sensor(0x0A05);
			otp->month= read_cmos_sensor(0x0A06);
			otp->day= read_cmos_sensor(0x0A07);
			otp->R_avg_H=read_cmos_sensor(0x0A0B);
			otp->R_avg_L=read_cmos_sensor(0x0A0C);
			otp->B_avg_H=read_cmos_sensor(0x0A0D);
			otp->B_avg_L=read_cmos_sensor(0x0A0E);
			otp->G_avg_H=read_cmos_sensor(0x0A0F);
			otp->G_avg_L=read_cmos_sensor(0x0A10);
			otp->R_gold_H=read_cmos_sensor(0x0A12);
			otp->R_gold_L=read_cmos_sensor(0x0A13);
			otp->B_gold_H=read_cmos_sensor(0x0A14);
			otp->B_gold_L=read_cmos_sensor(0x0A15);	
			otp->G_gold_H=read_cmos_sensor(0x0A16);
			otp->G_gold_L=read_cmos_sensor(0x0A17);	
			LOG_INF("[S5K5E2YA] [OTP] module id is %d\n",otp->module_integrator_id);
			LOG_INF("[S5K5E2YA] [OTP] date is %d-%d-%d\n",otp->year,otp->month,otp->day);
			LOG_INF("[S5K5E2YA] [OTP] R avg H is %d R avg L is %d\n",otp->R_avg_H,otp->R_avg_L);
			LOG_INF("[S5K5E2YA] [OTP] B avg H is %d B avg L is %d\n",otp->B_avg_H,otp->B_avg_L);
			LOG_INF("[S5K5E2YA] [OTP] G avg H is %d G avg L is %d\n",otp->G_avg_H,otp->G_avg_L);
	         }
		else if (ret==0xff&&ret1==0x1)
		{
			LOG_INF("[S5K5E2YA] [OTP] group2 is done");
			otp->module_integrator_id = read_cmos_sensor(0x0A24);
			otp->year = read_cmos_sensor(0x0A25);
			otp->month= read_cmos_sensor(0x0A26);
			otp->day= read_cmos_sensor(0x0A27);
			otp->R_avg_H=read_cmos_sensor(0x0A2B);
			otp->R_avg_L=read_cmos_sensor(0x0A2C);
			otp->B_avg_H=read_cmos_sensor(0x0A2D);
			otp->B_avg_L=read_cmos_sensor(0x0A2E);
			otp->G_avg_H=read_cmos_sensor(0x0A2F);
			otp->G_avg_L=read_cmos_sensor(0x0A30);
			otp->R_gold_H=read_cmos_sensor(0x0A32);
			otp->R_gold_L=read_cmos_sensor(0x0A33);
			otp->B_gold_H=read_cmos_sensor(0x0A34);
			otp->B_gold_L=read_cmos_sensor(0x0A35);	
			otp->G_gold_H=read_cmos_sensor(0x0A36);
			otp->G_gold_L=read_cmos_sensor(0x0A37);	
			LOG_INF("[S5K5E2YA] [OTP] module id is %d\n",otp->module_integrator_id);
			LOG_INF("[S5K5E2YA] [OTP] date is %d-%d-%d\n",otp->year,otp->month,otp->day);
			LOG_INF("[S5K5E2YA] [OTP] R avg H is %d R avg L is %d\n",otp->R_avg_H,otp->R_avg_L);
			LOG_INF("[S5K5E2YA] [OTP] B avg H is %d B avg L is %d\n",otp->B_avg_H,otp->B_avg_L);
			LOG_INF("[S5K5E2YA] [OTP] G avg H is %d G avg L is %d\n",otp->G_avg_H,otp->G_avg_L);
		}
		else
		{
			LOG_INF("[S5K5E2YA] [OTP]no otp data");
		}
		write_cmos_sensor(0x0A00,0x04);
		write_cmos_sensor(0x0A00, 0x00); //otp disable read
		mdelay(10);
		write_cmos_sensor(0x0100,0x01);
	}	
}

void S5K5E2YA_MIPI_algorithm_otp_wb1(struct S5K5E2YA_MIPI_otp_struct *otp)
{
	kal_uint32 R_avg, B_avg, G_avg;
	kal_uint32 R_gold, B_gold, G_gold;
	kal_uint32 R_gain, B_gain, G_gain;
	//kal_uint32 G_gain_R, G_gain_B;
	kal_uint32 gain_default=0x0100;
	R_avg = (((otp->R_avg_H)<<8)&0xff00)+(otp->R_avg_L&0xFF);
	B_avg = (((otp->B_avg_H)<<8)&0xff00)+(otp->B_avg_L&0xFF);
	G_avg = (((otp->G_avg_H)<<8)&0xff00)+(otp->G_avg_L&0xFF);
	R_gold= (((otp->R_gold_H)<<8)&0xff00)+(otp->R_gold_L&0xFF);
	B_gold= (((otp->B_gold_H)<<8)&0xff00)+(otp->B_gold_L&0xFF);
	G_gold= (((otp->G_gold_H)<<8)&0xff00)+(otp->G_gold_L&0xFF);

	LOG_INF("[S5K5E2YA] [OTP] R_avg=%d\n",R_avg);	
	LOG_INF("[S5K5E2YA] [OTP] B_avg=%d\n",B_avg);	
	LOG_INF("[S5K5E2YA] [OTP] G_avg=%d\n",G_avg);
	LOG_INF("[S5K5E2YA] [OTP] R_gold=%d\n",R_gold);	
	LOG_INF("[S5K5E2YA] [OTP] B_gold=%d\n",B_gold);	
	LOG_INF("[S5K5E2YA] [OTP] G_gold=%d\n",G_gold);
	kal_uint32 Golden_RG = (R_gold*1000)/G_gold;
	kal_uint32 Golden_BG = (B_gold*1000)/G_gold;

	kal_uint32 Current_RG = (R_avg*1000)/G_avg;
	kal_uint32 Current_BG = (B_avg*1000)/G_avg;
	
	kal_uint32 R_ration=(Golden_RG*1000)/Current_RG;
	kal_uint32 B_ration=(Golden_BG*1000)/Current_BG;
	LOG_INF("[S5K5E2YA] [OTP] Golden_RG=0x%x\n",Golden_RG);
	LOG_INF("[S5K5E2YA] [OTP] Golden_BG=0x%x\n",Golden_BG);
	LOG_INF("[S5K5E2YA] [OTP] Current_RG=0x%x\n",Current_RG);
	LOG_INF("[S5K5E2YA] [OTP] Current_BG=0x%x\n",Current_BG);
	LOG_INF("[S5K5E2YA] [OTP] R_ration=0x%x\n",R_ration);
	LOG_INF("[S5K5E2YA] [OTP] B_ration=0x%x\n",B_ration);
	if (R_ration>=1000)
	{
		if (B_ration>=1000)
		{
			G_gain=gain_default ;
			R_gain=gain_default*R_ration/1000;
			B_gain=gain_default*B_ration/1000;
		}
		else
		{
			B_gain=gain_default;
			G_gain=gain_default*1000/B_ration;
			R_gain=gain_default*R_ration/B_ration;
		}
	}
	else
	{
		if (B_ration>=1000)
		{
			R_gain=gain_default;
			G_gain=gain_default*1000/R_ration;
			B_gain=gain_default*B_ration/R_ration;
		}
		else if( (R_ration >= B_ration) && (B_ration < 1000))
		{
			B_gain=gain_default;
			G_gain=gain_default*1000/B_ration;
			R_gain=gain_default*R_ration/B_ration;
		}
		else
		{
			R_gain=gain_default;
			G_gain=gain_default*1000/R_ration;
			B_gain=gain_default*B_ration/R_ration;
		}
	}

	otp->R_Gain = R_gain;
	otp->B_Gain = B_gain;
	otp->G_Gain = G_gain;

	LOG_INF("[S5K5E2YA] [OTP] R_gain=0x%x\n",otp->R_Gain);	
	LOG_INF("[S5K5E2YA] [OTP] B_gain=0x%x\n",otp->B_Gain);	
	LOG_INF("[S5K5E2YA] [OTP] G_gain=0x%x\n",otp->G_Gain);
}



void S5K5E2YA_MIPI_write_otp_wb(struct S5K5E2YA_MIPI_otp_struct *otp)
{
	kal_uint16 R_GainH, B_GainH, G_GainH;
	kal_uint16 R_GainL, B_GainL, G_GainL;
	kal_uint32 temp;

	temp = otp->R_Gain;
	R_GainH = (temp & 0xff00)>>8;
	temp = otp->R_Gain;
	R_GainL = (temp & 0x00ff);

	temp = otp->B_Gain;
	B_GainH = (temp & 0xff00)>>8;
	temp = otp->B_Gain;
	B_GainL = (temp & 0x00ff);

	temp = otp->G_Gain;
	G_GainH = (temp & 0xff00)>>8;
	temp = otp->G_Gain;
	G_GainL = (temp & 0x00ff);

	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] R_GainH=0x%x\n", R_GainH);	
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] R_GainL=0x%x\n", R_GainL);	
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] B_GainH=0x%x\n", B_GainH);
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] B_GainL=0x%x\n", B_GainL);	
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] G_GainH=0x%x\n", G_GainH);	
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] G_GainL=0x%x\n", G_GainL);

	write_cmos_sensor(0x020e, G_GainH);
	write_cmos_sensor(0x020f, G_GainL);
	write_cmos_sensor(0x0210, R_GainH);
	write_cmos_sensor(0x0211, R_GainL);
	write_cmos_sensor(0x0212, B_GainH);
	write_cmos_sensor(0x0213, B_GainL);
	write_cmos_sensor(0x0214, G_GainH);
	write_cmos_sensor(0x0215, G_GainL);

	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] [0x020e,0x%x]\n", read_cmos_sensor(0x020e));	
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] [0x020f,0x%x]\n", read_cmos_sensor(0x020f));	
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] [0x0210,0x%x]\n", read_cmos_sensor(0x0210));
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] [0x0211,0x%x]\n", read_cmos_sensor(0x0211));	
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] [0x0212,0x%x]\n", read_cmos_sensor(0x0212));	
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] [0x0213,0x%x]\n", read_cmos_sensor(0x0213));
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] [0x0214,0x%x]\n", read_cmos_sensor(0x0214));	
	LOG_INF("[S5K5E2YA] [S5K5E2YA_MIPI_read_otp_wb] [0x0215,0x%x]\n", read_cmos_sensor(0x0215));
}
void S5K5E2YA_MIPI_LSC_from_otp(void)
{
	//kal_uint16 PageCount,i;
	//kal_uint32 temp[64]={0};
	ushort value1=0;
	ushort value2=0;
	ushort value3=0;
	ushort value4=0;
	/*for(PageCount = 9; PageCount<=15; PageCount++)
	{
	LOG_INF("[S5K5E2YA] [OTP] LSC PageCount=%d\n", PageCount); 

	write_cmos_sensor(0x0100,0x00);
	mdelay(10);
	write_cmos_sensor(0x0A00,0x04);
	write_cmos_sensor(0x0A02, PageCount); //page set
	write_cmos_sensor(0x0A00, 0x01); //otp enable read
	mdelay(100);
	for(i=0;i<64;i++)
	    {
			temp[i]=read_cmos_sensor(0x0A04+i);
			LOG_INF("[S5K5E2YA] [OTP] LSC temp[%d]=%d\n", i,temp[i]); 
	     }
	}*/

	value1=read_cmos_sensor(0x0202);
	value2=read_cmos_sensor(0x0203);
	value3=read_cmos_sensor(0x0204);
	value4=read_cmos_sensor(0x0205);
	LOG_INF("[S5K5E2YA] [OTP] LSC value1=%d,value2=%d,value3=%d,value4=%d\n", value1,value2,value3,value4);
	// Reset for operation																		   
		write_cmos_sensor(0x0100, 0x00); //stream off
		write_cmos_sensor(0x0103, 0x01);
		mdelay(10);
		write_cmos_sensor(0x340f, 0x81);
		// Clock Setting
		write_cmos_sensor(0x0305, 0x05); //PLLP (def:5)
		write_cmos_sensor(0x0306, 0x00);
		write_cmos_sensor(0x0307, 0xB3); //PLLM (def:CCh 204d --> B3h 179d)
		write_cmos_sensor(0x3C1F, 0x00); //PLLS 
		write_cmos_sensor(0x0820, 0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
		write_cmos_sensor(0x0821, 0x5B); 
		write_cmos_sensor(0x3C1C, 0x58); //dbr_div
		// Size Setting
		write_cmos_sensor(0x0340, 0x07);// frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
		write_cmos_sensor(0x0341, 0xA7);
		write_cmos_sensor(0x0342, 0x0B);// line_length_pck : def. 2900d 
		write_cmos_sensor(0x0343, 0x54);
		write_cmos_sensor(0x0344, 0x00);// x_addr_start
		write_cmos_sensor(0x0345, 0x00);
		write_cmos_sensor(0x0346, 0x00);// y_addr_start
		write_cmos_sensor(0x0347, 0x00);
		write_cmos_sensor(0x0348, 0x0A);// x_addr_end : def. 2575d  
		write_cmos_sensor(0x0349, 0x0F);
		write_cmos_sensor(0x034A, 0x07);// y_addr_end : def. 1936d
		write_cmos_sensor(0x034B, 0x8F);
		write_cmos_sensor(0x034C, 0x0A);// x_output size : def. 2576d 
		write_cmos_sensor(0x034D, 0x10);
		write_cmos_sensor(0x034E, 0x07);// y_output size : def. 1936d
		write_cmos_sensor(0x034F, 0x90);
		//Digital Binning(default)
		write_cmos_sensor(0x0900, 0x00);//0x0 Binning
		write_cmos_sensor(0x0901, 0x20);
		write_cmos_sensor(0x0387, 0x01);
		//Integration time	
		write_cmos_sensor(0x0202, value1);  // coarse integration
		write_cmos_sensor(0x0203, value2);
		
		write_cmos_sensor(0x0200, 0x0A);  // fine integration (AA8h --> AC4h)
		write_cmos_sensor(0x0201, 0x1A);
		//Analog Timing Tuning (0923)
		write_cmos_sensor(0X3000, 0x04);	 // ct_ld_start
		write_cmos_sensor(0X3002, 0x03);	 // ct_sl_start
		write_cmos_sensor(0X3003, 0x04);	 // ct_sl_margin
		write_cmos_sensor(0X3004, 0x02);	 // ct_rx_start
		write_cmos_sensor(0X3005, 0x00);	 // ct_rx_margin (MSB) 
		write_cmos_sensor(0X3006, 0x10);	 // ct_rx_margin (LSB)
		write_cmos_sensor(0X3007, 0x03);	 // ct_tx_start
		write_cmos_sensor(0X3008, 0x55);	 // ct_tx_width
		write_cmos_sensor(0X3039, 0x00);	 // cintc1_margin
		write_cmos_sensor(0X303A, 0x00);	 // cintc2_margin
		write_cmos_sensor(0X303B, 0x00);	 // offs_sh
		write_cmos_sensor(0X3009, 0x05);	 // ct_srx_margin
		write_cmos_sensor(0X300A, 0x55);	 // ct_stx_width
		write_cmos_sensor(0X300B, 0x38);	 // ct_dstx_width
		write_cmos_sensor(0X300C, 0x10);	 // ct_stx2dstx
		write_cmos_sensor(0X3012, 0x05);	 // ct_cds_start
		write_cmos_sensor(0X3013, 0x00);	 // ct_s1s_start
		write_cmos_sensor(0X3014, 0x22);	 // ct_s1s_end
		write_cmos_sensor(0X300E, 0x79);	 // ct_s3_width
		write_cmos_sensor(0X3010, 0x68);	 // ct_s4_width
		write_cmos_sensor(0X3019, 0x03);	 // ct_s4d_start
		write_cmos_sensor(0X301A, 0x00);	 // ct_pbr_start
		write_cmos_sensor(0X301B, 0x06);	 // ct_pbr_width
		write_cmos_sensor(0X301C, 0x00);	 // ct_pbs_start
		write_cmos_sensor(0X301D, 0x22);	 // ct_pbs_width
		write_cmos_sensor(0X301E, 0x00);	 // ct_pbr_ob_start
		write_cmos_sensor(0X301F, 0x10);	 // ct_pbr_ob_width
		write_cmos_sensor(0X3020, 0x00);	 // ct_pbs_ob_start
		write_cmos_sensor(0X3021, 0x00);	 // ct_pbs_ob_width
		write_cmos_sensor(0X3022, 0x0A);	 // ct_cds_lim_start
		write_cmos_sensor(0X3023, 0x1E);	 // ct_crs_start
		write_cmos_sensor(0X3024, 0x00);	 // ct_lp_hblk_cds_start (MSB)
		write_cmos_sensor(0X3025, 0x00);	 // ct_lp_hblk_cds_start (LSB)
		write_cmos_sensor(0X3026, 0x00);	 // ct_lp_hblk_cds_end (MSB)
		write_cmos_sensor(0X3027, 0x00);	 // ct_lp_hblk_cds_end (LSB)
		write_cmos_sensor(0X3028, 0x1A);	 // ct_rmp_off_start
		write_cmos_sensor(0X3015, 0x00);	 // ct_rmp_rst_start (MSB)
		write_cmos_sensor(0X3016, 0x84);	 // ct_rmp_rst_start (LSB)
		write_cmos_sensor(0X3017, 0x00);	 // ct_rmp_sig_start (MSB)
		write_cmos_sensor(0X3018, 0xA0);	 // ct_rmp_sig_start (LSB)
		write_cmos_sensor(0X302B, 0x10);	 // ct_cnt_margin
		write_cmos_sensor(0X302C, 0x0A);	 // ct_rmp_per
		write_cmos_sensor(0X302D, 0x06);	 // ct_cnt_ms_margin1
		write_cmos_sensor(0X302E, 0x05);	 // ct_cnt_ms_margin2
		write_cmos_sensor(0X302F, 0x0E);	 // rst_mx
		write_cmos_sensor(0X3030, 0x2F);	 // sig_mx
		write_cmos_sensor(0X3031, 0x08);	 // ct_latch_start
		write_cmos_sensor(0X3032, 0x05);	 // ct_latch_width
		write_cmos_sensor(0X3033, 0x09);	 // ct_hold_start
		write_cmos_sensor(0X3034, 0x05);	 // ct_hold_width
		write_cmos_sensor(0X3035, 0x00);	 // ct_lp_hblk_dbs_start (MSB)
		write_cmos_sensor(0X3036, 0x00);	 // ct_lp_hblk_dbs_start (LSB)
		write_cmos_sensor(0X3037, 0x00);	 // ct_lp_hblk_dbs_end (MSB)
		write_cmos_sensor(0X3038, 0x00);	 // ct_lp_hblk_dbs_end (LSB)
		write_cmos_sensor(0X3088, 0x06);	 // ct_lat_lsb_offset_start1
		write_cmos_sensor(0X308A, 0x08);	 // ct_lat_lsb_offset_end1
		write_cmos_sensor(0X308C, 0x05);	 // ct_lat_lsb_offset_start2
		write_cmos_sensor(0X308E, 0x07);	 // ct_lat_lsb_offset_end2
		write_cmos_sensor(0X3090, 0x06);	 // ct_conv_en_offset_start1
		write_cmos_sensor(0X3092, 0x08);	 // ct_conv_en_offset_end1
		write_cmos_sensor(0X3094, 0x05);	 // ct_conv_en_offset_start2
		write_cmos_sensor(0X3096, 0x21);	 // ct_conv_en_offset_end2
		//CDS
		write_cmos_sensor(0x3099, 0x0E);  // cds_option ([3]:crs switch disable, s3,s4 strengthx16)
		write_cmos_sensor(0x3070, 0x10);  // comp1_bias (default:77)
		write_cmos_sensor(0x3085, 0x11);  // comp1_bias (gain1~4)
		write_cmos_sensor(0x3086, 0x01);  // comp1_bias (gain4~8) 
		//RMP
		write_cmos_sensor(0x3064, 0x00);// Multiple sampling(gainx8,x16)
		write_cmos_sensor(0x3062, 0x08);// off_rst
		//DBR
		write_cmos_sensor(0x3061, 0x11);// dbr_tune_rd (default :08, 0E 3.02V) 3.1V
		write_cmos_sensor(0x307B, 0x20);// dbr_tune_rgsl (default :08)
		//Bias sampling
		write_cmos_sensor(0x3068, 0x00);// RMP BP bias sampling [0]: Disable
		write_cmos_sensor(0x3074, 0x00);// Pixel bias sampling [2]:Default L
		write_cmos_sensor(0x307D, 0x00);// VREF sampling [0] : Disable
		write_cmos_sensor(0x3045, 0x01);// ct_opt_l1_start
		write_cmos_sensor(0x3046, 0x05);// ct_opt_l1_width
		write_cmos_sensor(0x3047, 0x78);
		//Smart PLA
		write_cmos_sensor(0x307F, 0xB1); //RDV_OPTION[5:4], RG default high
		write_cmos_sensor(0x3098, 0x01); //CDS_OPTION[16] SPLA-II enable
		write_cmos_sensor(0x305C, 0xF6); //lob_extension[6]
		write_cmos_sensor(0x306B, 0x10); // [3]bls_stx_en disable
		write_cmos_sensor(0x3063, 0x27); // ADC_SAT 490mV(19h) --> 610mV(2Fh) --> 570mV
		write_cmos_sensor(0x320C, 0x07); // ADC_MAX (def:076Ch --> 0700h)
		write_cmos_sensor(0x320D, 0x00);																		
		write_cmos_sensor(0x0b00, 0x01);
		write_cmos_sensor(0x3235, 0x49); // L/F-ADLC on
		write_cmos_sensor(0x3233, 0x00); // D-pedestal L/F ADLC off (1FC0h)
		write_cmos_sensor(0x3234, 0x00);
		write_cmos_sensor(0x3300, 0x0D); //BPC bypass
	//	write_cmos_sensor(0x0204, 0x00); //Analog gain x1
	//	write_cmos_sensor(0x0205, 0x20);
	
		write_cmos_sensor(0x0204, value3);  // coarse integration
		write_cmos_sensor(0x0205, value4);
		
		write_cmos_sensor(0x3400, 0x00);
		mdelay(10);
		write_cmos_sensor(0x3b4c, 0x00); 
		write_cmos_sensor(0x3b4c, 0x01); 
		// streaming ON
		write_cmos_sensor(0x0100, 0x01); 
	//write_cmos_sensor(0x0A00,0x04);
	//write_cmos_sensor(0x0A00, 0x00); //otp disable read
	//mdelay(10);
	//write_cmos_sensor(0x0100,0x01);

}
void S5K5E2YA_MIPI_update_wb_register_from_otp(void)
{
	struct S5K5E2YA_MIPI_otp_struct current_otp;
	S5K5E2YA_MIPI_read_otp_wb(&current_otp);
	S5K5E2YA_MIPI_algorithm_otp_wb1(&current_otp);
	S5K5E2YA_MIPI_write_otp_wb(&current_otp);
	//S5K5E2YA_MIPI_LSC_from_otp();
}
#endif

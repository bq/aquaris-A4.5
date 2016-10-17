/*
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include "cust_alsps.h"
#include "ltr559.h"
#include "alsps.h"



/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define LTR559_DEV_NAME   "ltr559"

/*----------------------------------------------------------------------------*/
//#define LCSH_DEBUG_LTR
#define APS_TAG                  "[ltr559] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)

#if defined(LCSH_DEBUG_LTR)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)     
#else
#define APS_ERR(fmt, args...)	//pr_err(APS_TAG fmt, ##args)
#define APS_LOG(fmt, args...)	//pr_err(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)	pr_err(APS_TAG fmt, ##args)
#endif

/*----------------------------------------------------------------------------*/


static struct ltr559_priv *double_tap_data = NULL; 

#define LCSH_DEBUG_HZ

#if defined(LCSH_DEBUG_HZ)
#define TPD_DEVICE   "swf559"
#define LCSH_DEBUG(a, arg...) printk(TPD_DEVICE "line=%d,func=%s: .\n" a, __LINE__,__func__,##arg)
#else
#define LCSH_DEBUG(arg...)
#endif

#undef   LC_DEVINFO_ALSPS
#define  LC_DEVINFO_ALSPS
#ifdef   LC_DEVINFO_ALSPS
#include <linux/dev_info.h>
static void alsps_devinfo_init(void);
#endif


static struct i2c_client *ltr559_i2c_client = NULL;

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr559_i2c_id[] = {{LTR559_DEV_NAME,0},{}};
static unsigned long long int_top_time;
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
struct platform_device *alspsPltFmDev;


/*----------------------------------------------------------------------------*/
static int ltr559_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int ltr559_i2c_remove(struct i2c_client *client);
static int ltr559_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int ltr559_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ltr559_i2c_resume(struct i2c_client *client);
static int ltr559_ps_enable(struct i2c_client *client, int enable);
static int ltr559_just_for_init(void);
static int ltr559_just_for_reset(void);


#define GN_MTK_BSP_PS_DYNAMIC_CALI

static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int als_times = 0;
static int als_value[3] = {0};
static int final_lux_val;

static int ps_trigger_high = 800;			
static int ps_trigger_low = 760;				
static int ps_high_trigger_delta = 141;//150;	
static int ps_low_trigger_delta = 29;  //50;	
static int dynamic_calibrate = 1947;
static int als_first_enable_flag = 0;
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int ltr559_als_read(struct i2c_client *client, u16* data);
static int ltr559_ps_read(struct i2c_client *client, int* data);


/*----------------------------------------------------------------------------*/


typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct ltr559_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/

struct ltr559_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;
    struct mutex lock;
	/*i2c address group*/
    struct ltr559_i2c_addr  addr;

     /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;
    atomic_t    als_suspend;

	
	atomic_t  init_done;
	struct device_node *irq_node;
	int		irq;

    /*data*/
    u16         als;
    int          ps;  //u16 
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/

//lc zgy 20151118  add for proximity sensor is covered when double tap
	bool		ps_enable;	 /*record current ps status*/

    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;


static struct ltr559_priv *ltr559_obj = NULL;
//static struct platform_driver ltr559_alsps_driver;


static struct i2c_client *ltr559_i2c_client;
//static int intr_flag = 1; /* hw default away after enable. */

static DEFINE_MUTEX(ltr559_mutex);
static DEFINE_MUTEX(ltrinterrupt_mutex);


static int ltr559_local_init(void);
static int ltr559_remove(void);
static int ltr559_init_flag =  -1;
static struct alsps_init_info ltr559_init_info = {
		.name = "ltr559",
		.init = ltr559_local_init,
		.uninit = ltr559_remove,

};


#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver ltr559_i2c_driver = {	
	.probe      = ltr559_i2c_probe,
	.remove     = ltr559_i2c_remove,
	.detect     = ltr559_i2c_detect,
	.suspend    = ltr559_i2c_suspend,
	.resume     = ltr559_i2c_resume,
	.id_table   = ltr559_i2c_id,
	.driver = {
		.name           = LTR559_DEV_NAME,
		#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
		#endif
	},
};



/* 
 * #########
 * ## I2C ##
 * #########
 */

// I2C Read
static int ltr559_i2c_read_reg(u8 regnum)
{
    u8 buffer[1],reg_value[1];
	int res = 0;

	mutex_lock(&ltr559_mutex);
	
	buffer[0]= regnum;
	res = i2c_master_send(ltr559_obj->client, buffer, 0x1);
	if(res <= 0)	{
	   
	   APS_ERR("read reg send res = %d\n",res);
		return res;
	}
	res = i2c_master_recv(ltr559_obj->client, reg_value, 0x1);
	if(res <= 0)
	{
		APS_ERR("read reg recv res = %d\n",res);
		return res;
	}

	mutex_unlock(&ltr559_mutex);

	return reg_value[0];
}

// I2C Write
static int ltr559_i2c_write_reg(u8 regnum, u8 value)
{
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = regnum;   
	databuf[1] = value;
	res = i2c_master_send(ltr559_obj->client, databuf, 0x2);

	if (res < 0)
		{
			APS_ERR("wirte reg send res = %d\n",res);
		   	return res;
		}
		
	else
		return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr559_show_reg_bug(void)
{
	int i,len=0;
	int reg[]={0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,/*0x8c,*/
		0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x94,0x95,0x97,0x98,0x99,0x9a,0x9e};
	for(i=0;i<26;i++)
	{
		APS_DBG("ltr559 added by fully for reg:0x%04X value: 0x%04X\n", reg[i],ltr559_i2c_read_reg(reg[i]));
	}
	
	return len;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr559_dynamic_calibrate(void)	
{																				
	int ret=0;
	int i=0;
	int data;
	int data_total=0;
	ssize_t len = 0;
	int noise = 0;
	int count = 5;
	int max = 0;

	if(!ltr559_obj)
	{	
		APS_ERR("ltr559_obj is null!!\n");
		//len = sprintf(buf, "ltr559_obj is null\n");
		return -1;
	}

	msleep(15);

	for (i = 0; i < count; i++) {
		// wait for ps value be stable
		msleep(15);

		ret = ltr559_ps_read(ltr559_obj->client, &data);
		printk("[hmm] enable_ps data=%d\n",data);
		if (ret < 0) {
			i--;
			continue;
		}
/*
		if(data & 0x8000){
			noise = dynamic_calibrate;
			break;
		}	
*/		
		data_total+=data;

		if (max++ > 100) {
			return len;
		}
	}

	//if(noise == dynamic_calibrate){
		//;
	//}else{	
		noise=data_total/count;
	//}

	if(noise < dynamic_calibrate + 100){
		dynamic_calibrate = noise;
	}

	if(noise > dynamic_calibrate + 200){
		noise = dynamic_calibrate + 100;
	}
	
	dynamic_calibrate = noise;
	printk("[hmm] enable_ps noise=%d\n",noise);
	if(noise < 50){
		ps_trigger_high = noise + 90 + ps_high_trigger_delta;
		ps_trigger_low = noise + 70 + ps_low_trigger_delta;
	}else if(noise < 100){
		ps_trigger_high = noise + 100 + ps_high_trigger_delta;
		ps_trigger_low = noise + 80 + ps_low_trigger_delta;
	}else if(noise < 200){
		ps_trigger_high = noise + 100 + ps_high_trigger_delta;
		ps_trigger_low = noise + 80 + ps_low_trigger_delta;
	}else if(noise < 300){
		ps_trigger_high = noise + 120 + ps_high_trigger_delta;
		ps_trigger_low = noise + 100 + ps_low_trigger_delta;
	}else if(noise < 400){
		ps_trigger_high = noise + 140 + ps_high_trigger_delta;
		ps_trigger_low = noise + 120 + ps_low_trigger_delta;
	}else if(noise < 500){
		ps_trigger_high = noise + 160 + ps_high_trigger_delta;
		ps_trigger_low = noise + 140 + ps_low_trigger_delta;
	}else if(noise < 600){
		ps_trigger_high = noise + 170 + ps_high_trigger_delta;
		ps_trigger_low = noise + 150 + ps_low_trigger_delta;
	}else{
		ps_trigger_high = 800 + ps_high_trigger_delta;
		ps_trigger_low = 780 + ps_low_trigger_delta;
	}

	ret = ltr559_i2c_write_reg(0x90, ps_trigger_high & 0XFF);
	ret = ltr559_i2c_write_reg(0x91, (ps_trigger_high>>8) & 0X07);
	ret = ltr559_i2c_write_reg(0x92, ps_trigger_low & 0xff);
	ret = ltr559_i2c_write_reg(0x93, (ps_trigger_low>>8) & 0x07);	

	return 0;
}
#endif


/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	res = ltr559_als_read(ltr559_obj->client, &ltr559_obj->als);
    return snprintf(buf, PAGE_SIZE, "0x%04X\n", res);    
	
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_ps(struct device_driver *ddri, char *buf)
{
	int  res;
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	res = ltr559_ps_read(ltr559_obj->client, &ltr559_obj->ps);
    return snprintf(buf, PAGE_SIZE, "0x%04X\n", ltr559_obj->ps);     
}
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(ltr559_obj->hw)
	{
	
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			ltr559_obj->hw->i2c_num, ltr559_obj->hw->power_id, ltr559_obj->hw->power_vol);
		
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}


	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr559_obj->als_suspend), atomic_read(&ltr559_obj->ps_suspend));

	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_status(struct device_driver *ddri, const char *buf, size_t count)
{
	int status1,ret;
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &status1))
	{ 
	    //ret=ltr559_ps_enable(ps_gainrange);
		ret=ltr559_ps_enable(ltr559_obj->client, status1);
		APS_DBG("iret= %d, ps_gainrange = %d\n", ret, ps_gainrange);
	}
	else
	{
		APS_DBG("invalid content: %s, length = %zu\n", buf, count);
	}
	return count;    
}


/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_reg(struct device_driver *ddri, char *buf)
{
	int i,len=0;
	int reg[]={0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,
		0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x94,0x95,0x97,0x98,0x99,0x9a,0x9e};
	for(i=0;i<27;i++)
		{
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%04X value: 0x%04X\n", reg[i],ltr559_i2c_read_reg(reg[i]));	

	    }
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_reg(struct device_driver *ddri, const char *buf, size_t count)
{
	int ret,value;
	u8 reg;
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(2 == sscanf(buf, "%hhx %x ", &reg, &value))
	{ 
		APS_DBG("before write reg: %x, reg_value = %x  write value=%x\n", reg,ltr559_i2c_read_reg(reg),value);
	    ret=ltr559_i2c_write_reg(reg,value);
		APS_DBG("after write reg: %x, reg_value = %x\n", reg,ltr559_i2c_read_reg(reg));
	}
	else
	{
		APS_DBG("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;    
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     0664, ltr559_show_als,   NULL);
static DRIVER_ATTR(ps,      0664, ltr559_show_ps,    NULL);
static DRIVER_ATTR(status,  0664, ltr559_show_status,  ltr559_store_status);
static DRIVER_ATTR(reg,     0664, ltr559_show_reg,   ltr559_store_reg);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *ltr559_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_status,
    &driver_attr_reg,
};
/*----------------------------------------------------------------------------*/
static int ltr559_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(ltr559_attr_list)/sizeof(ltr559_attr_list[0]));

	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, ltr559_attr_list[idx]);
		if(err)
		{            
			APS_ERR("driver_create_file (%s) = %d\n", ltr559_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int ltr559_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(ltr559_attr_list)/sizeof(ltr559_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, ltr559_attr_list[idx]);
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/





/* 
 * ###############
 * ## PS CONFIG ##
 * ###############

 */

static int ltr559_ps_set_thres(void)
{
	int res;
	u8 databuf[2];

	struct i2c_client *client = ltr559_obj->client;
	struct ltr559_priv *obj = ltr559_obj;	

	APS_FUN();

	//BUILD_BUG_ON_ZERO(0>1);


	APS_DBG("ps_cali.valid: %d\n", ps_cali.valid);
	if(1 == ps_cali.valid)
	{
		databuf[0] = LTR559_PS_THRES_LOW_0; 
		databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_LOW_1; 
		databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_0;	
		databuf[1] = (u8)(ps_cali.close & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_1;	
		databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
	}
	else
	{
		databuf[0] = LTR559_PS_THRES_LOW_0; 
		databuf[1] = (u8)(ps_trigger_low & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_LOW_1; 
		databuf[1] = (u8)((ps_trigger_low>> 8) & 0x00FF);
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_0;	
		databuf[1] = (u8)(ps_trigger_high & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_1;	
		databuf[1] = (u8)((ps_trigger_high >> 8) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
	
	}
	APS_DBG("ps low: %d high: %d\n", atomic_read(&obj->ps_thd_val_low), atomic_read(&obj->ps_thd_val_high));

	res = 0;
	return res;
	
	EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	return res;

}


//static int ltr559_ps_enable(int gainrange)
static int ltr559_ps_enable(struct i2c_client *client, int enable)
{
	//struct ltr559_priv *obj = ltr559_obj;
	u8 regdata;	
	int err;
	
	//int setgain;
    	APS_LOG("ltr559_ps_enable() ...start!\n");

	ltr559_just_for_init();

	regdata = ltr559_i2c_read_reg(LTR559_PS_CONTR);
	if(regdata<0)
	{
		APS_ERR("PS: read ps enable regdata: %d \n", regdata );
		return regdata;
	}

	if (enable == 1) {
		APS_LOG("PS: enable ps only \n");
		regdata = 0x2b;//0x23; 
	} else {
		APS_LOG("PS: disable ps only \n");
		//regdata &= 0xfc;
		regdata = 0x0;
	}
 
	err = ltr559_i2c_write_reg(LTR559_PS_CONTR, regdata);
	if(err<0)
	{
		APS_ERR("PS: enable ps err: %d en: %d \n", err, enable);
		return err;
	}
	mdelay(WAKEUP_DELAY);
//lc zgy 20151118 add for proximity sensor is covered when double tap
	double_tap_data->ps_enable = enable;

#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
	regdata = ltr559_i2c_read_reg(LTR559_PS_CONTR);
	if(regdata == 0x2b)
	{
		ltr559_i2c_write_reg(0x90,  0xFF);
		ltr559_i2c_write_reg(0x91, 0x07);
		ltr559_i2c_write_reg(0x92, 0x0);
		ltr559_i2c_write_reg(0x93, 0x0);	

		ltr559_dynamic_calibrate();

	}
#endif

	return 0;
	
}


static int ltr559_ps_read(struct i2c_client *client, int *data) //u16 
{
	int psval_lo, psval_hi, psdata;

	psval_lo = ltr559_i2c_read_reg(LTR559_PS_DATA_0);
	//APS_DBG("ps_rawdata_psval_lo = %d\n", psval_lo);
	if (psval_lo < 0){
	    
	    APS_DBG("psval_lo error\n");
		psdata = psval_lo;
		goto out;
	}
	psval_hi = ltr559_i2c_read_reg(LTR559_PS_DATA_1);
    //APS_DBG("ps_rawdata_psval_hi = %d\n", psval_hi);

	if (psval_hi < 0){
	    APS_DBG("psval_hi error\n");
		psdata = psval_hi;
		goto out;
	}
	
	psdata = ((psval_hi & 7)* 256) + psval_lo;
    //psdata = ((psval_hi&0x7)<<8) + psval_lo;


	*data = psdata;
    APS_DBG("ps_rawdata = %d,int *data=%d .\n", psdata,*data);
	return 0;
    
	out:
	final_prox_val = psdata;
	
	return psdata;
}

/* 
 * ################
 * ## ALS CONFIG ##
 * ################
 */

static int ltr559_als_enable(struct i2c_client *client, int enable)
{
	//struct ltr559_priv *obj = i2c_get_clientdata(client);	
	int err=0;	
	u8 regdata=0;
	int i;	
	//if (enable == obj->als_enable)		
	//	return 0;

	regdata = ltr559_i2c_read_reg(LTR559_ALS_CONTR);

	if (enable == 1) {
		APS_LOG("ALS(1): enable als only \n");
		regdata = 0x0D;
		
		als_first_enable_flag = 1;
		als_times = 0;
		for(i=0;i<3;i++){
			als_value[i] = 0;
		}
		
	} else {
		APS_LOG("ALS(1): disable als only \n");
		//regdata &= 0xfe;
		regdata = 0x0;
		als_first_enable_flag = 0;
	}

	err = ltr559_i2c_write_reg(LTR559_ALS_CONTR, regdata);
	if(err<0)
	{
		APS_ERR("ALS: enable als err: %d en: %d \n", err, enable);
		return err;
	}

	//obj->als_enable = enable;
	
	mdelay(WAKEUP_DELAY);

	return 0;
	
	#if 0
	APS_LOG("gainrange = %d\n",gainrange);
	switch (gainrange)
	{
		case ALS_RANGE_64K:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range1);
			break;

		case ALS_RANGE_32K:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range2);
			break;

		case ALS_RANGE_16K:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range3);
			break;
			
		case ALS_RANGE_8K:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range4);
			break;
			
		case ALS_RANGE_1300:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range5);
			break;

		case ALS_RANGE_600:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range6);
			break;
			
		default:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range1);			
			APS_ERR("proxmy sensor gainrange %d!\n", gainrange);
			break;
	}

	mdelay(WAKEUP_DELAY);

	/* =============== 
	 * ** IMPORTANT **
	 * ===============
	 * Other settings like timing and threshold to be set here, if required.
 	 * Not set and kept as device default for now.
 	 */
 	if(error<0)
 	    APS_LOG("ltr559_als_enable ...ERROR\n");
 	else
        APS_LOG("ltr559_als_enable ...OK\n");
	
	return error;
	#endif
        
}


// Put ALS into Standby mode
#if 0
static int ltr559_als_disable(void)
{
	int error;
	error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_StdBy); 
	if(error<0)
 	    APS_LOG("ltr559_als_disable ...ERROR\n");
 	else
        APS_LOG("ltr559_als_disable ...OK\n");
	return error;
}
#endif

static int ltr559_als_read(struct i2c_client *client, u16* data)
{
	int alsval_ch0_lo, alsval_ch0_hi, alsval_ch0;
	int alsval_ch1_lo, alsval_ch1_hi, alsval_ch1;
	int  luxdata_int;
	int ratio;
	int als_valid_status;
	u8 pulse_test;

#define ALS_BQ_CALIBRATION      115
#define ALS_BQ_CALIBRATION_DIV  100

	LCSH_DEBUG("by fully.\n");
	
	ltr559_just_for_reset();


als_data_try:
	alsval_ch0_lo = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_0);
	alsval_ch0_hi = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_1);
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
	APS_DBG("alsval_ch0_lo = %d,alsval_ch0_hi=%d,alsval_ch0=%d\n",alsval_ch0_lo,alsval_ch0_hi,alsval_ch0);
	alsval_ch1_lo = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_0);
	alsval_ch1_hi = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_1);
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
	APS_DBG("alsval_ch1_lo = %d,alsval_ch1_hi=%d,alsval_ch1=%d\n",alsval_ch1_lo,alsval_ch1_hi,alsval_ch1);

    	if((alsval_ch1==0)||(alsval_ch0==0))
    	{
       	 ratio = 100;
    	}else{
		ratio = (alsval_ch1*100) /(alsval_ch0+alsval_ch1);
        }
		
	APS_DBG("ratio = %d  gainrange = %d\n",ratio,als_gainrange);
	if (ratio < 45){
		luxdata_int = (((17743 * alsval_ch0)+(11059 * alsval_ch1))/als_gainrange)/1000;
	}
	else if ((ratio < 64) && (ratio >= 45)){
		luxdata_int = (((42785 * alsval_ch0)-(19548 * alsval_ch1))/als_gainrange)/1000;
	}
	else if ((ratio < 85) && (ratio >= 64)) {
		luxdata_int = (((5926 * alsval_ch0)+(1185 * alsval_ch1))/als_gainrange)/1000;
	}
	else {
		if(als_first_enable_flag == 1){
			APS_DBG("als first enable for the data stable. \n");
			als_first_enable_flag = 0;
			goto als_data_try;
		}else{
			APS_DBG("als first data is already stable. \n");
			luxdata_int = 0;
		}

	}

#if 1	
	//------------------------------------
	APS_DBG("before deal with als_value_lux = %d\n", luxdata_int);
	if(als_times >= 2){
		 als_value[0] = als_value[1];
		 als_value[1] = als_value[2];
		 als_value[2] = luxdata_int;

		luxdata_int = ((als_value[0] + als_value[1] + als_value[2]) / 3);

		APS_DBG("ok three data ltr559 by fully.\n");		
	}else if(als_times == 1){
		als_value[1] = als_value[2];
		als_value[2] = luxdata_int;
		als_times++;
		
		luxdata_int = ((als_value[1] + als_value[2]) / 2);	
		APS_DBG("second data ltr559 by fully.\n");
	}else if(als_times == 0){
		als_value[2] = luxdata_int;
		als_times++;
		if((luxdata_int==0) && (final_lux_val > 0)){
			luxdata_int = final_lux_val;
		}	
		final_lux_val = luxdata_int;
		APS_DBG("first data ltr559 by fully.\n");
	}
#endif
        luxdata_int = (luxdata_int * ALS_BQ_CALIBRATION) / ALS_BQ_CALIBRATION_DIV;	

	APS_DBG("als_value_lux = %d\n", luxdata_int);

	*data = luxdata_int;
	final_lux_val = luxdata_int;
	ltr559_show_reg_bug();
	return 0;

	
//err:
//	final_lux_val = luxdata_int;
//	APS_DBG("err als_value_lux = 0x%x\n", luxdata_int);
//	return luxdata_int;
}



/*----------------------------------------------------------------------------*/
int ltr559_get_addr(struct alsps_hw *hw, struct ltr559_i2c_addr *addr)
{
	/***
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	***/
	return 0;
}


/*-----------------------------------------------------------------------------*/
void ltr559_eint_func(void)
{
	struct ltr559_priv *obj = ltr559_obj;
	
	APS_FUN();

	if(!obj)
	{
		return;
	}
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
	//schedule_delayed_work(&obj->eint_work);
}

#if defined(CONFIG_OF)
static irqreturn_t ltr559_eint_handler(int irq, void *desc)
{
	ltr559_eint_func();
	disable_irq_nosync(ltr559_obj->irq);

	return IRQ_HANDLED;
}
#endif


/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int ltr559_setup_eint(struct i2c_client *client)
{
	#if 0
	struct ltr559_priv *obj = (struct ltr559_priv *)i2c_get_clientdata(client);        

	ltr559_obj = obj;
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, ltr559_eint_func, 0);
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
	#endif

	int ret;
	struct pinctrl *pinctrl;
	//struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;

	u32 ints[2] = {0, 0};

	LCSH_DEBUG();

	alspsPltFmDev = get_alsps_platformdev();
	/* gpio setting */
	pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}
	//pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	//if (IS_ERR(pins_default)) {
	//	ret = PTR_ERR(pins_default);
	//	APS_ERR("Cannot find alsps pinctrl default!\n");

	//}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");

	}
/* eint request */
	if (ltr559_obj->irq_node) {
		of_property_read_u32_array(ltr559_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		LCSH_DEBUG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		ltr559_obj->irq = irq_of_parse_and_map(ltr559_obj->irq_node, 0);
		LCSH_DEBUG("ltr559_obj->irq = %d\n", ltr559_obj->irq);
		if (!ltr559_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		//APS_ERR("irq to gpio = %d \n",irq_to_gpio(ltr559_obj->irq));
		if (request_irq(ltr559_obj->irq, ltr559_eint_handler, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		enable_irq(ltr559_obj->irq);
	} else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}
	
    return 0;
}


/*----------------------------------------------------------------------------*/
static void ltr559_power(struct alsps_hw *hw, unsigned int on) 
{

}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int ltr559_check_and_clear_intr(struct i2c_client *client) 
{
//***

	int res,intp,intl;
	u8 buffer[2];	
	u8 temp;
		//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/	
		//	  return 0;
	
		APS_FUN();
		buffer[0] = LTR559_ALS_PS_STATUS;
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}

		if(buffer[0] == 0){
			LCSH_DEBUG("by fully.\n");
	
			ltr559_just_for_reset();
		}

		
		temp = buffer[0];
		res = 1;
		intp = 0;
		intl = 0;
		if(0 != (buffer[0] & 0x02))
		{
			res = 0;
			intp = 1;
		}
		if(0 != (buffer[0] & 0x08))
		{
			res = 0;
			intl = 1;		
		}
	
		if(0 == res)
		{
			if((1 == intp) && (0 == intl))
			{
				APS_LOG("PS interrupt\n");
				buffer[1] = buffer[0] & 0xfD;
				
			}
			else if((0 == intp) && (1 == intl))
			{
				APS_LOG("ALS interrupt\n");
				buffer[1] = buffer[0] & 0xf7;
			}
			else
			{
				APS_LOG("Check ALS/PS interrup error\n");
				buffer[1] = buffer[0] & 0xf5;
			}
			//buffer[0] = LTR559_ALS_PS_STATUS	;
			//res = i2c_master_send(client, buffer, 0x2);
			//if(res <= 0)
			//{
			//	goto EXIT_ERR;
			//}
			//else
			//{
			//	res = 0;
			//}
		}
	
		return res;
	
	EXIT_ERR:
		APS_ERR("ltr559_check_and_clear_intr fail\n");
		return -1;

}
/*----------------------------------------------------------------------------*/


static int ltr559_check_intr(struct i2c_client *client) 
{
	

	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;
	APS_FUN();

	buffer[0] = LTR559_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	APS_LOG("status = %x\n", buffer[0]);

	if(buffer[0] == 0){
		LCSH_DEBUG("by fully.\n");
		ltr559_just_for_reset();
	}
	
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x02))
	{
	
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x08))
	{
		res = 0;
		intl = 1;		
	}

		if(0 == res)
		{
			if((1 == intp) && (0 == intl))
			{
				APS_LOG("PS interrupt\n");
				buffer[1] = buffer[0] & 0xfD;
				
			}
			else if((0 == intp) && (1 == intl))
			{
				APS_LOG("ALS interrupt\n");
				buffer[1] = buffer[0] & 0xf7;
			}
			else
			{
				APS_LOG("Check ALS/PS interrup error\n");
				buffer[1] = buffer[0] & 0xf5;
			}
			//buffer[0] = LTR559_ALS_PS_STATUS	;
			//res = i2c_master_send(client, buffer, 0x2);
			//if(res <= 0)
			//{
			//	goto EXIT_ERR;
			//}
			//else
			//{
			//	res = 0;
			//}
		}

	return res;

EXIT_ERR:
	APS_ERR("ltr559_check_intr fail\n");
	return 1;
}

static int ltr559_clear_intr(struct i2c_client *client) 
{
	int res;
	u8 buffer[2];

	APS_FUN();
	
	buffer[0] = LTR559_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	APS_DBG("buffer[0] = %d \n",buffer[0]);

	if(buffer[0] == 0){
		LCSH_DEBUG("by fully.\n");
		ltr559_just_for_reset();
	}

#if 0	
	buffer[1] = buffer[0] & 0x01;
	buffer[0] = LTR559_ALS_PS_STATUS	;

	res = i2c_master_send(client, buffer, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}
#endif

	return res;

EXIT_ERR:
	APS_ERR("ltr559_check_intr fail\n");
	return 1;
}


static int ltr559_just_for_reset(void)
{
	struct i2c_client *client = ltr559_obj->client;

	struct ltr559_priv *obj = ltr559_obj;   

	u8 pulse_test;
	u8 interrupt_test;
	u8 ps_enable_data;
	u8 als_enable_data;


	/*added by fully for reset20160713*/
	pulse_test = ltr559_i2c_read_reg(LTR559_PS_N_PULSES);
	interrupt_test = ltr559_i2c_read_reg(LTR559_INTERRUPT);
	ps_enable_data = ltr559_i2c_read_reg(LTR559_PS_CONTR);
	als_enable_data =  ltr559_i2c_read_reg(LTR559_ALS_CONTR);

	if((pulse_test == 1) ||(interrupt_test == 0) ){
		mdelay(200);
		ltr559_just_for_init();
		LCSH_DEBUG("added by fully11\n");

		interrupt_test = ltr559_i2c_read_reg(LTR559_INTERRUPT);
		if(interrupt_test != 0x01){
			ltr559_i2c_write_reg(LTR559_ALS_CONTR, 0x0);
			mdelay(10);
			ltr559_i2c_write_reg(LTR559_INTERRUPT, 0x01);
			ltr559_i2c_write_reg(LTR559_ALS_CONTR, als_enable_data);
			LCSH_DEBUG("added by fully11\n");
		}
		
		LCSH_DEBUG("ltr559 added by fully 20160713 for test33333333333333333\n");
		
		if(test_bit(CMC_BIT_PS,  &ltr559_obj->enable)){
			if(ps_enable_data != 0x2b){
				ltr559_i2c_write_reg(LTR559_PS_CONTR, 0x2b);
				LCSH_DEBUG("ltr559 added by fully 20160713 for test44444444444444444\n");
			}
		}
	}
	/*added by fully for reset20160713*/

	return 0;

}



static int ltr559_just_for_init(void)
{
	int res;
	int init_ps_gain;
	int init_als_gain;
	u8 databuf[2];	

	struct i2c_client *client = ltr559_obj->client;

	struct ltr559_priv *obj = ltr559_obj;   
	
	mdelay(10);

	LCSH_DEBUG("added by fully test00000000000 \n");

	res = ltr559_i2c_write_reg(LTR559_PS_LED, 0x7F); 
	if(res<0)
	{
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	} 
	
	res = ltr559_i2c_write_reg(LTR559_PS_N_PULSES, 6); 
	if(res<0)
	{
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	} 

	res = ltr559_i2c_write_reg(LTR559_PS_MEAS_RATE, 0x08); 
	if(res<0)
	{
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	} 
	
	res = ltr559_i2c_write_reg(LTR559_ALS_MEAS_RATE, 0X01); 
	if(res<0)
	{
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	} 

	/*for interrup work mode support */
	if(0 == obj->hw->polling_mode_ps)
	{	
		APS_LOG("eint enable");
		
		databuf[0] = LTR559_INTERRUPT;	
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}

		databuf[0] = LTR559_INTERRUPT_PERSIST;	
		databuf[1] = 0x40;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}

	}

	ltr559_ps_set_thres();

	res = 0;

	return res;

	EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}


static int ltr559_devinit(void)
{
	int res;
	int init_ps_gain;
	int init_als_gain;
	u8 databuf[2];	

	struct i2c_client *client = ltr559_obj->client;

	struct ltr559_priv *obj = ltr559_obj;   
	
	mdelay(PON_DELAY);

	init_ps_gain = MODE_PS_Gain16;
	//init_ps_gain = 0;

#if 0
	APS_LOG("LTR559_PS setgain = %d!\n",init_ps_gain);
	res = ltr559_i2c_write_reg(LTR559_PS_CONTR, init_ps_gain); 
	if(res<0)
	{
	    APS_LOG("ltr559 set ps gain error\n");
	    return res;
	}	
#endif	
	mdelay(WAKEUP_DELAY);

	res = ltr559_i2c_write_reg(LTR559_PS_LED, 0x7F); 
	if(res<0)
	{
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	} 
	
	res = ltr559_i2c_write_reg(LTR559_PS_N_PULSES, 6); 
	if(res<0)
	{
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	} 

	res = ltr559_i2c_write_reg(LTR559_PS_MEAS_RATE, 0x08); 
	if(res<0)
	{
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	} 
	
	res = ltr559_i2c_write_reg(LTR559_ALS_MEAS_RATE, 0X01); 
	if(res<0)
	{
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	} 

	als_gainrange = ALS_RANGE_8K;

#if 0
	// Enable ALS to Full Range at startup
	als_gainrange = ALS_RANGE_8K;

	init_als_gain = als_gainrange;

	switch (init_als_gain)
	{
		case ALS_RANGE_64K:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range1);
			break;

		case ALS_RANGE_32K:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range2);
			break;

		case ALS_RANGE_16K:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range3);
			break;
			
		case ALS_RANGE_8K:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range4);
			break;
			
		case ALS_RANGE_1300:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range5);
			break;

		case ALS_RANGE_600:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range6);
			break;
			
		default:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range1);			
			APS_ERR("proxmy sensor gainrange %d!\n", init_als_gain);
			break;
	}

	res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, init_als_gain);
	if(res<0)
	{
	    APS_LOG("ltr559 set als gain error1\n");
	    return res;
	}

#endif

	/*for interrup work mode support */
	if(0 == obj->hw->polling_mode_ps)
	{	
		APS_LOG("eint enable");
		//ltr559_ps_set_thres();
		
		databuf[0] = LTR559_INTERRUPT;	
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}

		databuf[0] = LTR559_INTERRUPT_PERSIST;	
		databuf[1] = 0x40;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}

	}

	if((res = ltr559_setup_eint(client))!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	
	if((res = ltr559_check_and_clear_intr(client)))
	{
		APS_ERR("check/clear intr: %d\n", res);
		//    return res;
	}

	res = 0;

	EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}
/*----------------------------------------------------------------------------*/


static int ltr559_get_als_value(struct ltr559_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	APS_DBG("als  = %d\n",als); 
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
static int ltr559_get_ps_value(struct ltr559_priv *obj, int ps)//u16
{
	int val, invalid = 0;

	static int val_temp = 1;
APS_DBG("func=%s,ps=%d,ps_trigger_high=%d,ps_trigger_low=%d .\n",__func__,ps,ps_trigger_high,ps_trigger_low);
	if(ps > ps_trigger_high) 
	//if((ps > atomic_read(&obj->ps_thd_val_high)))
	{
		val = 0;  /*close*/
		val_temp = 0;
		intr_flag_value = 1;
	}
			//else if((ps < atomic_read(&obj->ps_thd_val_low))&&(temp_ps[0]  < atomic_read(&obj->ps_thd_val_low)))
	else if(ps <ps_trigger_low) 
	//else if((ps < atomic_read(&obj->ps_thd_val_low)))
	{
		val = 1;  /*far away*/
		val_temp = 1;
		intr_flag_value = 0;
	}
	else
		val = val_temp;	
			
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 50000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}

/*----------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int als_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int als_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("ltr559_obj als enable value = %d\n", en);


	mutex_lock(&ltr559_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &ltr559_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &ltr559_obj->enable);
	mutex_unlock(&ltr559_mutex);
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}
	res = ltr559_als_enable(ltr559_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

static int als_get_data(int *value, int *status)
{
	int err = 0;

	struct ltr559_priv *obj = NULL;


	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}
	obj = ltr559_obj;
	err = ltr559_als_read(obj->client, &obj->als);
	if (err)
		err = -1;
	else {
		*value = obj->als;//ltr559_get_als_value(obj, obj->als); 
        //APS_ERR("als: %d\n", obj->als);
		//*value = obj->als;
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int ps_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */

static int ps_enable_nodata(int en)
{
	int res = 0;


	APS_LOG("ltr559_obj als enable value = %d\n", en);


	mutex_lock(&ltr559_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &ltr559_obj->enable);

	else
		clear_bit(CMC_BIT_PS, &ltr559_obj->enable);

	mutex_unlock(&ltr559_mutex);
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}
	res = ltr559_ps_enable(ltr559_obj->client, en);

	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int *value, int *status)
{
	int err = 0;



	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}

	err = ltr559_ps_read(ltr559_obj->client, &ltr559_obj->ps);
	if (err)
		err = -1;
	else {
		*value = ltr559_get_ps_value(ltr559_obj, ltr559_obj->ps);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}


/*----------------------------------------------------------------------------*/
/*for interrup work mode support */
static void ltr559_eint_work(struct work_struct *work)
{
	struct ltr559_priv *obj = (struct ltr559_priv *)container_of(work, struct ltr559_priv, eint_work);
	int err;
	//hwm_sensor_data sensor_data;
//	u8 buffer[1];
//	u8 reg_value[1];
	u8 databuf[2];
	int res = 0;
	int value = 1;
	int temp_noise = 0;	
	LCSH_DEBUG("***************eint work**********.\n");

	mutex_lock(&ltrinterrupt_mutex);
	
	err = ltr559_check_intr(obj->client);
	if(err < 0)
	{
		APS_ERR("ltr559_eint_work check intrs: %d\n", err);
	}
	else
	{
		//get raw data
		ltr559_ps_read(obj->client, &obj->ps);
    		if(obj->ps < 0)
    		{
    			err = -1;
    			return;
    		}

#if 0
		/*added by fully for overflow.*/
		if(obj->ps & 0x8000){
			value= 1;
			ps_report_interrupt_data(value);
			enable_irq(ltr559_obj->irq);
			return;
		}	
		/*the end added by fully for overflow.*/	
#endif		
		
		APS_DBG("ltr559_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		value = ltr559_get_ps_value(obj, obj->ps);
		//sensor_data.values[0] = ltr559_get_ps_value(obj, obj->ps);
		//sensor_data.value_divide = 1;
		//sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			
/*singal interrupt function add*/
		//intr_flag_value = value;
		APS_DBG("rawdata intr_flag_value=%d\n",intr_flag_value);
#if 1
		if(intr_flag_value){
				APS_DBG(" interrupt value ps will < ps_threshold_low");

				databuf[0] = LTR559_PS_THRES_LOW_0;	
				databuf[1] = (u8)(ps_trigger_low & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_LOW_1;	
				databuf[1] = (u8)((ps_trigger_low & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_0;	
				databuf[1] = (u8)(0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_1; 
				databuf[1] = (u8)((0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
		else{	
				APS_DBG(" interrupt value ps will > ps_threshold_high");

#if 0  //GN_MTK_BSP_PS_DYNAMIC_CALI		
				if(obj->ps  > 20 && obj->ps < (dynamic_calibrate + 100) &&  obj->ps > (dynamic_calibrate - 50)){
					if(obj->ps < 50){
						ps_trigger_high = obj->ps + 90 + ps_high_trigger_delta;
						ps_trigger_low = obj->ps + 70 + ps_low_trigger_delta;
					}else if(obj->ps < 100){
						ps_trigger_high = obj->ps + 100 + ps_high_trigger_delta;
						ps_trigger_low = obj->ps + 80 + ps_low_trigger_delta;
					}else if(obj->ps < 200){
						ps_trigger_high = obj->ps + 100 + ps_high_trigger_delta;
						ps_trigger_low = obj->ps + 80 + ps_low_trigger_delta;
					}else if(obj->ps < 300){
						ps_trigger_high = obj->ps + 120 + ps_high_trigger_delta;
						ps_trigger_low = obj->ps + 100 + ps_low_trigger_delta;
					}else if(obj->ps < 400){
						ps_trigger_high = obj->ps + 140 + ps_high_trigger_delta;
						ps_trigger_low = obj->ps + 120 +ps_low_trigger_delta;
					}else if(obj->ps < 500){
						ps_trigger_high = obj->ps + 160 + ps_high_trigger_delta;
						ps_trigger_low = obj->ps + 140 + ps_low_trigger_delta;
					}else if(obj->ps < 600){
						ps_trigger_high = obj->ps + 170 + ps_high_trigger_delta;
						ps_trigger_low = obj->ps + 150 + ps_low_trigger_delta;
					}else{
						ps_trigger_high = 800 + ps_high_trigger_delta;
						ps_trigger_low = 780 + ps_low_trigger_delta;
					}
					dynamic_calibrate = obj->ps;

				}	
				if(obj->ps  > 50){
					temp_noise = obj->ps - 50;
				}else{
					temp_noise = 0;
				}
#endif			
				
				databuf[0] = LTR559_PS_THRES_LOW_0;	
				databuf[1] = (u8)(temp_noise & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_LOW_1;	
				databuf[1] = (u8)((temp_noise & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_0;	
				databuf[1] = (u8)(ps_trigger_high & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_1; 
				databuf[1] = (u8)((ps_trigger_high & 0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
#endif

		mutex_unlock(&ltrinterrupt_mutex);

		ps_report_interrupt_data(value);
	}
	ltr559_clear_intr(obj->client);

	ltr559_show_reg_bug();
	//mt65xx_eint_unmask(CUST_EINT_ALS_NUM);      
	enable_irq(ltr559_obj->irq);

}



/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int ltr559_open(struct inode *inode, struct file *file)
{
	file->private_data = ltr559_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int ltr559_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/


static long ltr559_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)       
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct ltr559_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	APS_DBG("cmd= %d\n", cmd); 
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			//if(enable)
			//{
			    err = ltr559_ps_enable(obj->client, enable);
				if(err < 0)
				{
					APS_ERR("enable ps fail: %d en: %d\n", err, enable); 
					goto err_out;
				}
			if(enable){
				set_bit(CMC_BIT_PS, &obj->enable);
			}else{
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			APS_DBG("ALSPS_GET_PS_DATA\n"); 
		    err = ltr559_ps_read(obj->client, &obj->ps);
			if(err < 0)
			{
				goto err_out;
			}
			
			dat = ltr559_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_THRESHOLD_VALUE:
			//dat = atomic_read(&obj->ps_thd_val_high);
			dat = ps_trigger_high;
			if(dat < 0)
				dat = -1;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			err = ltr559_ps_read(obj->client, &obj->ps);
			if(err < 0)
			{
				goto err_out;
			}
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			//if(enable)
			//{
			    err = ltr559_als_enable(obj->client, enable);
				if(err < 0)
				{
					APS_ERR("enable als fail: %d en: %d\n", err, enable); 
					goto err_out;
				}
			if(enable){
				set_bit(CMC_BIT_ALS, &obj->enable);
			}else{
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
		    err = ltr559_als_read(obj->client, &obj->als);
			if(err < 0)
			{
				goto err_out;
			}

			dat = obj->als;//ltr559_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			err = ltr559_als_read(obj->client, &obj->als);
			if(err < 0)
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}

/*----------------------------------------------------------------------------*/
static struct file_operations ltr559_fops = {
	//.owner = THIS_MODULE,
	.open = ltr559_open,
	.release = ltr559_release,
	.unlocked_ioctl = ltr559_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ltr559_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ltr559_fops,
};

static int ltr559_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct ltr559_priv *obj = i2c_get_clientdata(client);    
	int err;
	APS_FUN();    

	ltr559_show_reg_bug();

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		err = ltr559_als_enable(obj->client, 0);
		if(err < 0)
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

	}

	LCSH_DEBUG("by fully.\n");
	
	ltr559_just_for_reset();
	
	return 0;

}
/*----------------------------------------------------------------------------*/
static int ltr559_i2c_resume(struct i2c_client *client)
{
	struct ltr559_priv *obj = i2c_get_clientdata(client);        
	int err;
	APS_FUN();
	u8 pulse_test;

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	ltr559_show_reg_bug();

	ltr559_power(obj->hw, 1);
/*	err = ltr559_devinit();
	if(err < 0)
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}*/
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr559_als_enable(obj->client, 1);
	    if (err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}

	LCSH_DEBUG("by fully.\n");
	
	ltr559_just_for_reset();
	
#if 0
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		err = ltr559_ps_enable(obj->client, 1);
		APS_LOG("swfps_resume ps_enable_err=%d,ps_suspend=%d .\n",err,atomic_read(&obj->ps_suspend));
	    if (err < 0)
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
#endif
	return 0;
}
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void ltr559_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct ltr559_priv *obj = container_of(h, struct ltr559_priv, early_drv);   
	int err;
	APS_FUN();    
	u8 pulse_test;

	ltr559_show_reg_bug();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->als_suspend, 1); 
	err = ltr559_als_enable(obj->client, 0);
	if(err < 0)
	{
		APS_ERR("disable als fail: %d\n", err); 
	}

	LCSH_DEBUG("by fully.\n");
	
	ltr559_just_for_reset();
	
}

static void ltr559_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct ltr559_priv *obj = container_of(h, struct ltr559_priv, early_drv);         
	int err;
	APS_FUN();
	u8 pulse_test;

	ltr559_show_reg_bug();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr559_als_enable(obj->client, 1);
		if(err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}

	LCSH_DEBUG("by fully.\n");
	
	ltr559_just_for_reset();

	
}
#endif
#if 0
int ltr559_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr559_priv *obj = (struct ltr559_priv *)self;
	
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
				    err = ltr559_ps_enable(obj->client, 1);
					if(err < 0)
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
				    err = ltr559_ps_disable(obj->client, 0);
					if(err < 0)
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				APS_ERR("get sensor ps data !\n");
				sensor_data = (hwm_sensor_data *)buff_out;
				obj->ps = ltr559_ps_read(obj->client, &obj->ps);
    			if(obj->ps < 0)
    			{
    				err = -1;
    				break;
    			}
				sensor_data->values[0] = ltr559_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int ltr559_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr559_priv *obj = (struct ltr559_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
				    err = ltr559_als_enable(obj->client, 1);
					if(err < 0)
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
				    err = ltr559_als_disable(obj->client, 0);
					if(err < 0)
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				APS_ERR("get sensor als data !\n");
				sensor_data = (hwm_sensor_data *)buff_out;
				obj->als = ltr559_als_read(obj->client, &obj->als);
                #if defined(MTK_AAL_SUPPORT)
				sensor_data->values[0] = obj->als;
				#else
				sensor_data->values[0] = ltr559_get_als_value(obj, obj->als);
				#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif

/*----------------------------------------------------------------------------*/
static int ltr559_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, LTR559_DEV_NAME);
	return 0;
}

#ifdef   LC_DEVINFO_ALSPS
static void alsps_devinfo_init(void)
{
	static struct devinfo_struct *devinfo_tp = NULL;
	devinfo_tp = kzalloc(sizeof(struct devinfo_struct), GFP_KERNEL);    
	if(NULL != devinfo_tp)
	{
		devinfo_tp->device_type = "ALSPS";
		devinfo_tp->device_module = "wecorp";
		devinfo_tp->device_vendor = "unknow"; 
		devinfo_tp->device_ic = "ltr559 ";
		devinfo_tp->device_version = "unknow";
		devinfo_tp->device_info = "unknow";
		devinfo_tp->device_used = DEVINFO_USED;
		devinfo_check_add_device(devinfo_tp);
	}else{
 		printk("swf77 failed to create tp deviinfo **\n");
	}
}
#endif


//lc zgy 20151118 add for proximity sensor is covered when double tap
static int ltr559_read_ps_value_for_double_tap(void);
int ltr559_get_ps_value_for_double_tap(void)
{
	int tp_double_tap = 1;
	int err=0;
   // printk("swft93 line=%d,enable =%d\n",__LINE__,double_tap_data->ps_enable);
  //  mutex_lock(&ltr559_for_double_tap);
	if (!double_tap_data->ps_enable) //screen suspend and ps is not open
        { 
        printk("swft93 gonging_enabel line=%d  ps_enable 111111111111111.\n",__LINE__);
		disable_irq_nosync(ltr559_obj->irq);		
		//ltr559_ps_enable(double_tap_data->client,1);
		err = ltr559_i2c_write_reg(LTR559_PS_CONTR, 0x2b);
		if(err<0)
		{
			printk("swft93 PS: enable ps err: %d \n", err);
			return err;
		}
		
		//ltr559_show_reg_bug();
		msleep(10);
			
		tp_double_tap = ltr559_read_ps_value_for_double_tap();
            //    printk("swft93 line=%d, tp_double_tap=%d (1=near)\n",__LINE__,tp_double_tap);
		//ltr559_ps_enable(double_tap_data->client,0);
		err = ltr559_i2c_write_reg(LTR559_PS_CONTR, 0x0);
		if(err<0)
		{
			printk("swft93 PS: enable ps err: %d \n", err);
			return err;
		}
		enable_irq(ltr559_obj->irq);	
	//	ltr559_show_reg_bug();
    printk("swft93 gonging_disable line=%d  ps_disable 0000000000000.\n",__LINE__);
	} else {                       //screen resume and ps is open or ps is open
		tp_double_tap = ltr559_read_ps_value_for_double_tap();
   // printk("swft93 line=%d, tp_double_tap=%d (screen resume)\n", __LINE__, tp_double_tap);
	}
	printk("swft93 read_ps_return_value line=%d  tp_double_tap =%d (1=near) .\n",__LINE__,tp_double_tap );
	if(1 == tp_double_tap)
		return 1;  //near
	else
		return 0;
}
EXPORT_SYMBOL(ltr559_get_ps_value_for_double_tap);

static int ltr559_read_ps_value_for_double_tap(void)
{
	int psdata = 8;
        int err = 0;

        if(err = ltr559_ps_read(double_tap_data->client, &double_tap_data->ps))
	{
	  printk("swft93 can not get data err:%d\n",err);
	}

        psdata = ltr559_get_ps_value(double_tap_data, double_tap_data->ps);
      //  printk("swft93 line=%d,enable=%d,psdata=%d(0=near)\n",__LINE__,double_tap_data->ps_enable,psdata);

	if (psdata < 0) {
		printk("%s read ps value fail\n", __func__);
		return -EINVAL;
	}

	if(0 == psdata)
		return 1; //near
	else
		return 0; //far
}

//lc zgy 20151118 add for proximity sensor is covered when double tap end

/*----------------------------------------------------------------------------*/
static int ltr559_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltr559_priv *obj;
	//struct hwmsen_object obj_ps, obj_als;
	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
	struct ps_control_path ps_ctl = {0};
	struct ps_data_path ps_data = {0};
	int err = 0;

	APS_LOG("ltr559_i2c_probe\n");

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ltr559_obj = obj;

	obj->hw = hw;
	//ltr559_get_addr(obj->hw, &obj->addr);

	INIT_WORK(&obj->eint_work, ltr559_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	//atomic_set(&obj->als_cmd_val, 0xDF);
	//atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);

	
	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");

	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	LCSH_DEBUG("ltr559_devinit() start...!\n");
	ltr559_i2c_client = client;
	err = ltr559_devinit();
	if(err)
	{
		goto exit_init_failed;
	}
	APS_LOG("ltr559_devinit() ...OK!\n");

	//printk("@@@@@@ manufacturer value:%x\n",ltr559_i2c_read_reg(0x87));

	err  = misc_register(&ltr559_device);
	if(err)
	{
		APS_ERR("ltr559_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	
	/* Register sysfs attribute */
	err = ltr559_create_attr(&(ltr559_init_info.platform_diver_addr->driver));
	if(err)
	{
		printk(KERN_ERR "create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;


	err = als_register_control_path(&als_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	err = ps_register_control_path(&ps_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	err = batch_register_support_info(ID_LIGHT, als_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register light batch support err = %d\n", err);

	err = batch_register_support_info(ID_PROXIMITY, ps_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register proximity batch support err = %d\n", err);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level	= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend	= ltr559_early_suspend,
	obj->early_drv.resume	= ltr559_late_resume,	 
	register_early_suspend(&obj->early_drv);
#endif


#ifdef   LC_DEVINFO_ALSPS
alsps_devinfo_init();
#endif
    double_tap_data = obj;  //lc zgy 20151118 add for proximity sensor is covered when double tap
	ltr559_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;
	
exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
		misc_deregister(&ltr559_device);
exit_init_failed:
		kfree(obj);
exit:
	ltr559_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	ltr559_init_flag =  -1;
	return err;

#if 0
#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = ltr559_early_suspend,
	obj->early_drv.resume   = ltr559_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&ltr559_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	exit_kfree:
	kfree(obj);
	exit:
	ltr559_i2c_client = NULL;           
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
#endif
}

/*----------------------------------------------------------------------------*/

static int ltr559_i2c_remove(struct i2c_client *client)
{
	int err;
	LCSH_DEBUG();
	err = ltr559_delete_attr(&ltr559_i2c_driver.driver);
	if(err)
	{
		APS_ERR("ltr559_delete_attr fail: %d\n", err);
	} 

	err = misc_deregister(&ltr559_device);
	if(err)
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	ltr559_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
#if 0
static int ltr559_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	ltr559_power(hw, 1);
	//ltr559_force[0] = hw->i2c_num;
	//ltr559_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",ltr559_force[0],ltr559_force[1]);
	if(i2c_add_driver(&ltr559_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static int ltr559_remove(void)
{
	//struct alsps_hw *hw = get_cust_alsps_hw();
	LCSH_DEBUG(); 
	ltr559_power(hw, 0);    
	i2c_del_driver(&ltr559_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
#if 0
static struct platform_driver ltr559_alsps_driver = {
	.probe      = ltr559_probe,
	.remove     = ltr559_remove,    
	.driver     = {
		.name  = "als_ps",
		//.owner = THIS_MODULE,
	}
};
#endif

//#ifdef CONFIG_OF
//static const struct of_device_id alsps_of_match[] = {
//	{ .compatible = "mediatek,als_ps", },
//	{},
//};
//#endif

//static struct platform_driver ltr559_alsps_driver =
//{
//	.probe      = ltr559_probe,
//	.remove     = ltr559_remove,    
//	.driver     = 
//	{
//		.name = "als_ps",
//       #ifdef CONFIG_OF
//		.of_match_table = alsps_of_match,
//		#endif
//	}
//};

static int  ltr559_local_init(void)
{
	/* printk("fwq loccal init+++\n"); */

	ltr559_power(hw, 1);
	if (i2c_add_driver(&ltr559_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}
	if (-1 == ltr559_init_flag)
		return -1;

	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init ltr559_init(void)
{
	const char *name = "mediatek,ltr559";

	LCSH_DEBUG();

	//i2c_register_board_info(0, &i2c_ltr559, 1);
	//if(platform_driver_register(&ltr559_alsps_driver))
	//{
	//	APS_ERR("failed to register driver");
	//	return -ENODEV;
	//}
	
	hw =   get_alsps_dts_func(name, hw);
	if (!hw)
		APS_ERR("get dts info fail\n");
	alsps_driver_add(&ltr559_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr559_exit(void)
{
	LCSH_DEBUG();
	//platform_driver_unregister(&ltr559_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(ltr559_init);
module_exit(ltr559_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("XX Xx");
MODULE_DESCRIPTION("LTR-559ALS Driver");
MODULE_LICENSE("GPL");


/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

#ifndef SLT_DRV_W9552_LED_FLASH
#define FLASH_BD7710
#endif
#if defined(FLASH_BD7710)

#define PK_ERR printk

struct pinctrl *flash_prnctrl;
struct pinctrl_state *flash_pin_out0 = NULL;
struct pinctrl_state *flash_pin_out1 = NULL;

int flash_light_gpio_init(struct platform_device *dev)
{
	int ret = 0;
	PK_ERR("[flash_light_gpio_init] start\n");
	flash_prnctrl = devm_pinctrl_get(&dev->dev);
	if (IS_ERR(flash_prnctrl)) {
		ret = PTR_ERR(flash_prnctrl);
		dev_err(&dev->dev, "zbl Cannot find flash_prnctrl!\n");
		return ret;
	}
	flash_pin_out0 = pinctrl_lookup_state(flash_prnctrl,"gpio43_out0");
	if (IS_ERR(flash_pin_out0)) {
		ret = PTR_ERR(flash_pin_out0);
		dev_err(&dev->dev, "zbl Cannot find gpio43_out0!\n");
		return ret;
	}
	flash_pin_out1 = pinctrl_lookup_state(flash_prnctrl,"gpio43_out1");
	if (IS_ERR(flash_pin_out1)) {
		ret = PTR_ERR(flash_pin_out1);
		dev_err(&dev->dev, "zbl Cannot find gpio43_out1!\n");
		return ret;
	}
	return 0;
}
#endif

/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

/*
#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)
*/
#define PK_DBG_FUNC printk
//#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);


#define STROBE_DEVICE_ID 0x60 //0xC6
#define FLASHLIGHT_DEVNAME            "mt6735m-Flashlight"
struct flash_chip_data {
	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;

	struct mutex lock;

	int mode;
	int torch_level;
};

static struct flash_chip_data chipconf;

static struct work_struct workTimeOut;


//static int g_bLtVersion;

/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

#if defined(FLASH_BD7710)
static struct i2c_client *BD7710_i2c_client = NULL;



struct BD7710_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct BD7710_chip_data {
	struct i2c_client *client;

	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;
	struct led_classdev cdev_indicator;

	struct BD7710_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

/* i2c access*/

static int BD7710_read_reg(struct i2c_client *client, u8 addr)//, u8 data)
{
    u8 beg = addr;
	int err;
	struct BD7710_chip_data *chip = i2c_get_clientdata(client);
	
	//mutex_lock(&bma050_i2c_mutex);
	mutex_lock(&chip->lock);

	if (!client)
	{
	   // mutex_unlock(&bma050_i2c_mutex);
	mutex_unlock(&chip->lock);
		return -EINVAL;
	}
		

		client->addr &= I2C_MASK_FLAG;
		client->addr |= I2C_WR_FLAG;
		client->addr |= I2C_RS_FLAG;
                client->timing = 100;//zbl add     

	    	err = i2c_master_send(client, &addr, 0x101);


		client->addr &= ~I2C_WR_FLAG;

		mutex_unlock(&chip->lock);

	printk("BD7710 Get i2c addr:0x%x,data=0x%x\n",beg,addr);
	return err;

}


static int BD7710_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	struct BD7710_chip_data *chip = i2c_get_clientdata(client);

        client->timing = 100; 
	mutex_lock(&chip->lock);
        printk("BD7710 begin to write reg 0x%2x = 0x%2x!\n",reg,val);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_ERR("BD7710 failed writting at 0x%02x\n", reg);
	return ret;
}

static int BD7710_chip_init(struct BD7710_chip_data *chip)
{
	int ret =0;
	struct i2c_client *client = chip->client;
	PK_DBG("BD7710_chip_init start--->.\n");

	pinctrl_select_state(flash_prnctrl,flash_pin_out1);

	//Can only read by cust_i2c_read_byte for combined mode
        printk("BD7710 start to read\n");
	ret = BD7710_read_reg(client, 0xFF);//,&ret);

        printk("BD7710 end of  read\n");
	msleep(10);

	pinctrl_select_state(flash_prnctrl,flash_pin_out0);

	if(ret >= 0)
	{
		ret = 0;
	}
	else
	{
		ret = -1;
	}
	PK_DBG("BD7710_chip_init end ret=%d.\n", ret);

	return ret;
}

static int BD7710_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct BD7710_chip_data *chip;
	struct BD7710_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	printk("BD7710_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "BD7710 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct BD7710_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		PK_ERR("BD7710 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct BD7710_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(BD7710_chip_init(chip)<0)
		goto err_chip_init;

	BD7710_i2c_client = client;
	printk("BD7710 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	printk("BD7710 probe is failed \n");
	return -ENODEV;
}

static int BD7710_remove(struct i2c_client *client)
{
	struct BD7710_chip_data *chip = i2c_get_clientdata(client);

    if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}

static const struct of_device_id bd7710_of_match[] = {
	{.compatible = "mediatek,bd7710"},
	{},
};

static const struct i2c_device_id BD7710_id[] = {
	{"bd7710", 0},
	{}
};

static struct i2c_driver BD7710_i2c_driver = {
	.driver = {
		.name  = "bd7710",	
		.owner = THIS_MODULE,
		.of_match_table = bd7710_of_match,
	},
	.probe	= BD7710_probe,
	.remove   = BD7710_remove,//__devexit_p()
	.id_table = BD7710_id,
};

/*
struct BD7710_platform_data BD7710_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_BD7710={ I2C_BOARD_INFO("bd7710", 0xA6>>1), \
							.platform_data = &BD7710_pdata,};
*/

//static int __init BD7710_init(void)
static int BD7710_init(void)
{
	printk("BD7710_init\n");
//	i2c_register_board_info(2, &i2c_BD7710, 1);

	return i2c_add_driver(&BD7710_i2c_driver);
}

//static void __exit BD7710_exit(void)
static void BD7710_exit(void)
{
	i2c_del_driver(&BD7710_i2c_driver);
}

/*
module_init(BD7710_init);
module_exit(BD7710_exit);

MODULE_DESCRIPTION("Flash Lighting driver for BD7710");
MODULE_AUTHOR("zhangjiano <zhangjiano@lenovo.com>");
MODULE_LICENSE("GPL v2");
*/
#define TORCH_BRIGHTNESS 1
#define FLASH_BRIGHTNESS 5

#define TORCH_DUTY_THR   2
#define MAX_DUTY_THR     11

int FL_Enable(void)
{
	struct flash_chip_data *chip = &chipconf;
	int brightness = 0;

	PK_DBG("FL_enable g_duty=%d\n",g_duty);
	if( NULL == BD7710_i2c_client)
		return 0;

	pinctrl_select_state(flash_prnctrl,flash_pin_out1);
	mdelay(10);

	if(g_duty < TORCH_DUTY_THR) /*0~7,use ic torch torch mode*/
	{
		brightness = (g_duty<<3) | g_duty; 
		PK_DBG("FL_enable torch brightness=%2x\n",brightness);

		BD7710_write_reg(BD7710_i2c_client, 0x10,0x1A); 	/*enable torch mode*/
		BD7710_write_reg(BD7710_i2c_client, 0xA0, brightness);	/*75ma torch output_en'*/

		udelay(50);

		chip->torch_level = brightness;
		chip->mode = 1;
	}
	else /*8~23,use ic torch torch mode*/
	{
		brightness = ((g_duty-TORCH_DUTY_THR)<<4) | (g_duty - TORCH_DUTY_THR);
		PK_DBG("FL_enable flash brightness=%2x\n",brightness);

		BD7710_write_reg(BD7710_i2c_client, 0x10,0x1B); //75ma torch output_en'
		BD7710_write_reg(BD7710_i2c_client, 0xB0,brightness); //75ma torch output_en'
		udelay(50);

		chip->torch_level = 0;
		chip->mode = 2;
	}
    return 0;
}

int FL_Disable(void)
{
	struct flash_chip_data *chip = &chipconf;
	PK_DBG("FL_disable g_duty=%d\n",g_duty);
	if( NULL == BD7710_i2c_client)
		return 0;

//	BD7710_write_reg(BD7710_i2c_client, 0x02,0x88); //75ma torch output_en'
//	BD7710_write_reg(BD7710_i2c_client, 0x01, 0xC0); //750ma flash output_en
//	BD7710_write_reg(BD7710_i2c_client, 0x01, 0xC0); //750ma flash output_en
	BD7710_write_reg(BD7710_i2c_client, 0x10,0x00); //75ma torch output_en'
	udelay(50);

	mdelay(1);

	chip->torch_level = 0;
	chip->mode = 0;

	pinctrl_select_state(flash_prnctrl,flash_pin_out0);  

    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	g_duty=duty;

	PK_DBG("FL_dim_duty\n");
    return 0;
}


int FL_Init(void)
{
	PK_DBG("FL_init\n");
	if(BD7710_i2c_client == NULL)
    {
    	return 0;
    }
//	BD7710_write_reg(BD7710_i2c_client, 0x10, 0xC0); //750ma flash output_en
	INIT_WORK(&workTimeOut, work_timeOutFunc);

    return 0;
}


int FL_Uninit(void)
{
	PK_DBG("FL_uninit\n");
	FL_Disable();
    return 0;
}

#endif
/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	//PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			//PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		//PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		//PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		//PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


/***************                   *******************/
//#ifdef CONFIG_LENOVO
static void chip_torch_brightness_set(struct led_classdev *cdev,
				  enum led_brightness brightness)
{
	//int i, cc;
	//struct flash_chip_data *chip = &chipconf;
	//u8 tmp4,tmp5;
	PK_ERR("[flashchip] torch brightness = %d\n",brightness);
	#if defined(FLASH_BD7710)
	if(brightness == 0)
	{
		BD7710_write_reg(BD7710_i2c_client, 0x10, 0x00); //75ma torch output_en'
		udelay(50);

		return;
	}
	else
	{
		//BD7710_write_reg(BD7710_i2c_client, 0x20, 0x88);
		BD7710_write_reg(BD7710_i2c_client, 0x10, 0x1A); //75ma torch output_en'
		BD7710_write_reg(BD7710_i2c_client, 0xA0, 0x22); //75ma torch output_en'
		udelay(50);
	}
	#elif defined(FLASH_RT9387)
	if(brightness == 0)
	{
		mdelay(4);
		chip->torch_level = 0;
		chip->mode = 0;
		PK_ERR("[flashchip] level = 0\n");

		return;
	}
	else
	{
		mdelay(4);
		chip->torch_level = 0;
		chip->mode = 0;
	}
	#endif
}


static void chip_flash_brightness_set(struct led_classdev *cdev,
				  enum led_brightness brightness)
{
	//struct flash_chip_data *chip = &chipconf;
	PK_ERR("[flashchip] flash brightness = %d\n",brightness);
	#if defined(FLASH_BD7710)
	if(brightness == 0)
	{
		//BD7710_write_reg(BD7710_i2c_client, 0x02,0x88); //75ma torch output_en'
		BD7710_write_reg(BD7710_i2c_client, 0x10, 0x00); //750ma flash output_en
		udelay(50);
		PK_ERR("[flashchip] flash level = 0\n");
	}
	else
	{
		BD7710_write_reg(BD7710_i2c_client, 0x10, 0x1B); //750ma flash output_en
		BD7710_write_reg(BD7710_i2c_client, 0xB0,0xFF); //75ma torch output_en'
		udelay(50);
		PK_ERR("[flashchip] flash level = 1\n");
	}
	#elif defined(FLASH_RT9387)
	if(brightness == 0)
	{
		mdelay(4);
		chip->torch_level = 0;
		chip->mode = 0;
		PK_ERR("[flashchip] flash level = 0\n");
	}
	else
	{
		//mdelay(4);
		chip->torch_level = 0;
		chip->mode = 2;
		PK_ERR("[flashchip] flash level = 1\n");
	}
	#endif
	return;
}

static int flashchip_probe(struct platform_device *dev)
{
	char ret;
	struct flash_chip_data *chip;

	printk("[flashchip_probe] start\n");
	chip = &chipconf;
	chip->mode = 0;
	chip->torch_level = 0;
	mutex_init(&chip->lock);

	//flash
	chip->cdev_flash.name="flash";
	chip->cdev_flash.max_brightness = 1;
	chip->cdev_flash.brightness_set = chip_flash_brightness_set;
	if(led_classdev_register((struct device *)&dev->dev,&chip->cdev_flash)<0)
		goto err_create_flash_file;
	//torch
	chip->cdev_torch.name="torch";
	chip->cdev_torch.max_brightness = 16;
	chip->cdev_torch.brightness_set = chip_torch_brightness_set;
	if(led_classdev_register((struct device *)&dev->dev,&chip->cdev_torch)<0)
		goto err_create_torch_file;

#if defined(FLASH_BD7710)
	if(flash_light_gpio_init(dev)<0){
		printk("flash init gpio fail");
		goto err_create_flash_file;
	}
	ret = BD7710_init();
    	printk("[flashchip_probe] Done,ret is %d\n",ret);
	return ret;
#endif
    	printk("[flashchip_probe] Done\n");
	return 0;

err_create_torch_file:
	led_classdev_unregister(&chip->cdev_flash);
err_create_flash_file:
	printk(KERN_ERR "[flashchip_probe] is failed !\n");
	return -ENODEV;



}

static int flashchip_remove(struct platform_device *dev)
{
	struct flash_chip_data *chip = &chipconf;
    PK_DBG("[flashchip_remove] start\n");

	led_classdev_unregister(&chip->cdev_torch);
	led_classdev_unregister(&chip->cdev_flash);
	#if defined(FLASH_BD7710)
	BD7710_exit();
	#endif
    PK_DBG("[flashchip_remove] Done\n");
    return 0;
}

static struct of_device_id mainflash_of_match[] = {
        { .compatible = "mediatek,mt6735m-Flashlight", },
        {},
};

static struct platform_driver flashchip_platform_driver =
{
    .probe      = flashchip_probe,
    .remove     = flashchip_remove,
    .driver     = {
        	.name = FLASHLIGHT_DEVNAME,
		.owner	= THIS_MODULE,
		.of_match_table = mainflash_of_match,
    },
};



//static struct platform_device flashchip_platform_device = {
//    .name = FLASHLIGHT_DEVNAME,
//    .id = 0,
//    .dev = {
//    .platform_data = &chip,
//    }
//};

static int __init flashchip_init(void)
{
    int ret = 0;
    printk("[flashchip_init] start\n");

	//ret = platform_device_register (&flashchip_platform_device);
	//if (ret) {
        //PK_ERR("[flashchip_init] platform_device_register fail\n");
        //return ret;
	//}

    	ret = platform_driver_register(&flashchip_platform_driver);
	if(ret){
		PK_ERR("[flashchip_init] platform_driver_register fail\n");
		return ret;
	}

	printk("[flashchip_init] done!\n");
    return ret;
}

static void __exit flashchip_exit(void)
{
    printk("[flashchip_exit] start\n");
    platform_driver_unregister(&flashchip_platform_driver);
    printk("[flashchip_exit] done!\n");
}

/*****************************************************************************/
module_init(flashchip_init);
module_exit(flashchip_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhangjiano@lenovo.com>");
MODULE_DESCRIPTION("Factory mode flash control Driver");
//#endif



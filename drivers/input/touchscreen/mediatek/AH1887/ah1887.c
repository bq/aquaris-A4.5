

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/seq_file.h>

//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>


#include <asm/io.h>
#include <cust_eint.h>

#include <linux/input.h>

#include <linux/proc_fs.h>
#define HALL_PROC_FILE "hall_status"

#define HALL_NEED_ENABLE_SWITCH

#ifdef HALL_NEED_ENABLE_SWITCH
#define HALL_ENABLE_PROC_FILE "hall_enable"
#endif
static u16 nEnableKey = 1;
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define AH1887_HALL_STATUS_OPEN 112
#define AH1887_HALL_STATUS_CLOSE 113

//#define LCSH_DEBUG_HZ
#if defined(LCSH_DEBUG_HZ)
#define LCSH_DEBUG(a, arg...) printk(TPD_DEVICE ": " a, ##arg)
#else
#define LCSH_DEBUG(arg...)
#endif
#define TPD_DEVICE   "swf-hall"

/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern unsigned int mt_eint_get_mask(unsigned int eint_num); //swf add,if open debug to use this function,pls modify in irq-mt-eic.c
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);



static atomic_t at_suspend;
static atomic_t at_resume;
static atomic_t	 at_close ;	// 0 close Low;  1 leave High;
static atomic_t	 at_n_close ;	// 0 close Low;  1 leave High;
static struct mutex mtx_eint_status;

static struct work_struct  eint1_work;
static struct work_struct  eint2_work;
static struct input_dev *hall_kpd = NULL;
static struct early_suspend    early_drv;
//static struct proc_dir_entry *g_hall_proc = NULL;
#ifdef HALL_NEED_ENABLE_SWITCH
static struct proc_dir_entry *g_hall_enable_proc = NULL;
#endif
/*----------------------------------------------------------------------------*/



static void ah1887_eint1_work(struct work_struct *work);
static void ah1887_eint2_work(struct work_struct *work);

/*----------------------------------------------------------------------------*/
static void ah1887_power(unsigned int on) 
{
	if(on)
	{
		//hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "hall");
	}
	else
	{
		//hwPowerDown(MT65XX_POWER_LDO_VGP4, "hall")
	}
}

static int hall_proc_read_show (struct seq_file* m, void* data)
{
	char temp[30] = {0};
	int cnt= 0;
	int status;

	mutex_lock(&mtx_eint_status);
	status = atomic_read(&at_close);
	if(status == 1)
	{
		cnt = sprintf(temp, "on\n");
	}
	else
	{
		cnt = sprintf(temp, "off\n");
	}
	mutex_unlock(&mtx_eint_status);

	//sprintf(temp, "module:%s version:%s\n", CFG_Module_buff, CFG_VER_buff);
	seq_printf(m, "%s\n", temp);
	return 0;
}

static hall_proc_open (struct inode* inode, struct file* file) 
{
    return single_open(file, hall_proc_read_show, inode->i_private);
}

static const struct file_operations g_hall_proc = 
{
    .open = hall_proc_open,
    .read = seq_read,
};
static int hall_proc_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
//static int hall_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{

	int cnt= 0;
	unsigned char status;
	mutex_lock(&mtx_eint_status);
	status = atomic_read(&at_close);
	if(status == 1)
	{
		cnt = sprintf(buffer, "on\n");
	}
	else
	{
		cnt = sprintf(buffer, "off\n");
	}
	mutex_unlock(&mtx_eint_status);
	//*eof = 1;
	*ppos += cnt; 	
	return cnt;
}

#ifdef HALL_NEED_ENABLE_SWITCH
static int hall_enable_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{

	int cnt= 0;
	unsigned char status;
	cnt = sprintf(page, "%d\n", nEnableKey);

	*eof = 1;
	return cnt;
}

static int hall_enable_proc_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	unsigned char writebuf[10];
	
	if (copy_from_user(&writebuf, buff, 10)) {
		
		printk("%s:copy from user error\n", __func__);
		return -EFAULT;
	}

	LCSH_DEBUG("hall_enable_proc_write len:%d", buflen);
	LCSH_DEBUG("hall_enable_proc_write str:%s", writebuf);

	if((writebuf[0] - '0') == 0)
	{
		nEnableKey = 0;
	}
	else
	{
		nEnableKey = 1;
	}
	return buflen;
}

#endif

static const struct file_operations cust_hall_fops = { // add by zhaofei - 2014-09-23-13-19
    .read =hall_proc_read,
};

static const struct file_operations cust_hall_enable_fops = { 
    .write = hall_enable_proc_write,
    .read =hall_enable_proc_read,
};
int ah1887_create_proc()
{

#if 0 // fix 3.10// add by zhaofei - 2014-09-23-13-14
	g_hall_proc = create_proc_entry(HALL_PROC_FILE, 0444, NULL);
	if (g_hall_proc == NULL)
	{
		printk("[HALL] create_proc_entry failed\n");
	} 
	else
	{
		g_hall_proc->read_proc = hall_proc_read;
//		g_hall_proc->write_proc = hall_proc_write;	
		printk("[HALL] create_proc_entry success\n");
	}

#ifdef HALL_NEED_ENABLE_SWITCH
	g_hall_enable_proc = create_proc_entry(HALL_ENABLE_PROC_FILE, 0444, NULL);
	if (g_hall_enable_proc == NULL)
	{
		printk("[HALL] create_proc_entry failed\n");
	} 
	else
	{
		g_hall_enable_proc->read_proc = hall_enable_proc_read;
		g_hall_enable_proc->write_proc = hall_enable_proc_write;	
		printk("[HALL] create_proc_entry success\n");
	}
#endif


#else
    if(proc_create(HALL_PROC_FILE, 0444, NULL, &g_hall_proc)== NULL)
	{
        printk("create_proc_entry  failed");
		return -1;
    }	
    
	if(proc_create(HALL_ENABLE_PROC_FILE, 0440, NULL, &cust_hall_enable_fops)== NULL)
	{
        printk("create_proc_entry  failed");
		return -1;
    }	
#endif


	return 0;	
}

//---start add litao
#define CUST_EINT_MHALL_NUM		CUST_EINT_HALL_1_NUM
#define	GPIO_MHALL_EINT_PIN		GPIO_HALL_1_PIN 										
#define	GPIO_MHALL_EINT_PIN_M_EINT	GPIO_HALL_1_PIN_M_EINT
#define	GPIO_MHALL_EINT_PIN_M_GPIO	GPIO_HALL_1_PIN_M_GPIO

#define CUST_EINT_MHALL_1_NUM	CUST_EINT_HALL_2_NUM
#define	GPIO_MHALL1_EINT_PIN		GPIO_HALL_2_PIN 										
#define	GPIO_MHALL1_EINT_PIN_M_EINT	GPIO_HALL_2_PIN_M_EINT
#define	GPIO_MHALL1_EINT_PIN_M_GPIO	GPIO_HALL_2_PIN_M_GPIO
///---end

void ah1887_eint1_func(void)
{
	LCSH_DEBUG("line=%d,fuc=%s.\n",__LINE__,__func__);
	ah1887_eint1_work(&eint1_work);
}

void ah1887_eint2_func(void)
{
	LCSH_DEBUG("line=%d,fuc=%s.\n",__LINE__,__func__);
//	schedule_work(&eint_work);
	ah1887_eint2_work(&eint2_work);
}

#define CUST_EINT_MHALL_SENSITIVE 1
#define CUST_EINT_POLARITY_LOW              0
#define CUST_EINT_POLARITY_HIGH             1

// add by zhaofei - 2014-09-29-13-46
///del litao/#define GPIO_MHALL1_EINT_PIN         (GPIO117 | 0x80000000)


int ah1887_setup_eint(void)
{
	printk("[hall]: %s  line = %d\n", __func__,__LINE__);
	unsigned char eint1_bit = 0,eint2_bit =0;
	mutex_lock(&mtx_eint_status);
//EINT1

//del litao/	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_GPIO);	///add litao
	mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, FALSE);
	eint1_bit = mt_get_gpio_in(GPIO_MHALL_EINT_PIN);	
	printk("hall zhaofei eint1 bit is %d\n",eint1_bit);
	if(eint1_bit == 0)
	{
	LCSH_DEBUG("[hall]: %s  line = %d\n", __func__,__LINE__);
		atomic_set(&at_close, 1); // low close
		mt_eint_registration(CUST_EINT_MHALL_NUM, CUST_EINT_POLARITY_HIGH, ah1887_eint1_func, 1);//1:is_auto_umask;0-->hand_umask 
	}
	else
	{
	LCSH_DEBUG("[hall]: %s  line = %d\n", __func__,__LINE__);
		atomic_set(&at_close, 0); // high open
		mt_eint_registration(CUST_EINT_MHALL_NUM, CUST_EINTF_TRIGGER_LOW, ah1887_eint1_func, 1);//1:is_auto_umask;0-->hand_umask 
	}
	mt_eint_mask(CUST_EINT_MHALL_NUM); 

//EINT2
	mt_set_gpio_dir(GPIO_MHALL1_EINT_PIN, GPIO_DIR_IN);
//del litao/	mt_set_gpio_mode(GPIO_MHALL1_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
	mt_set_gpio_mode(GPIO_MHALL1_EINT_PIN, GPIO_MHALL1_EINT_PIN_M_GPIO);   //add litao
	mt_set_gpio_pull_enable(GPIO_MHALL1_EINT_PIN, FALSE);
	eint2_bit = mt_get_gpio_in(GPIO_MHALL1_EINT_PIN);	
	printk("hall zhaofei eint2 bit is %d\n",eint2_bit);
	if(eint2_bit == 0)
	{
	LCSH_DEBUG("[hall]: %s  line = %d\n", __func__,__LINE__);
		atomic_set(&at_n_close, 1); // low close
		mt_eint_registration(CUST_EINT_MHALL_1_NUM, CUST_EINT_POLARITY_HIGH, ah1887_eint2_func, 1);
	}
	else
	{
	LCSH_DEBUG("[hall]: %s  line = %d\n", __func__,__LINE__);
		atomic_set(&at_n_close, 0); // high open
		mt_eint_registration(CUST_EINT_MHALL_1_NUM, CUST_EINTF_TRIGGER_LOW, ah1887_eint2_func, 1);
	}
	
	mt_eint_mask(CUST_EINT_MHALL_1_NUM); 
	mutex_unlock(&mtx_eint_status); 
	return 0;
}

//#if 0  ////lc --litao open 20150522 for midtest
static void ah1887_report_key_mittest( int keycode)
{
	LCSH_DEBUG("line=%d,fuc=%s.\n",__LINE__,__func__);
	input_report_key(hall_kpd, keycode, 1);
	input_report_key(hall_kpd, keycode, 0);	    	
	input_sync(hall_kpd);	
}
//#else 
static void ah1887_report_key()
{
	LCSH_DEBUG("line=%d,fuc=%s.\n",__LINE__,__func__);
	input_report_key(hall_kpd, KEY_POWER, 1);
	input_report_key(hall_kpd, KEY_POWER, 0);	    	
	input_sync(hall_kpd);	
}

//#endif
///del litao temp/ extern int tpd_enable_high_Sensitivity(int enable);
static void ah1887_eint1_work(struct work_struct *work)
{
	unsigned char eint_bit;
	unsigned char last_status;
	last_status = atomic_read(&at_close);

	if(hall_kpd != NULL)   //normal:get_mask=1 ;midtest:get_mask=1
	{
	        //LCSH_DEBUG("line=%d,last_status=%d,get_mask=%d.\n",__LINE__,last_status,mt_eint_get_mask(CUST_EINT_MHALL_NUM));
		if( 0 == last_status){
			if(atomic_read(&at_suspend) ==1){
                         // LCSH_DEBUG("line=%d,eint1=0,supend=1,get_mask=%d.\n",__LINE__,mt_eint_get_mask(CUST_EINT_MHALL_NUM));
			   //mt_eint_unmask(CUST_EINT_MHALL_NUM);
		        }
			else{
			  // mt_eint_mask(CUST_EINT_MHALL_NUM);  
			   ah1887_report_key();
                        }
			   ah1887_report_key_mittest(KEY_HALL_DOWN) ;	//lc --litao add 20150522 for midtest
		}
		else{
			if(atomic_read(&at_resume) ==1){
			  //LCSH_DEBUG("line=%d,eint1=1,resume=1,get_mask=%d.\n",__LINE__,mt_eint_get_mask(CUST_EINT_MHALL_NUM));
                           //mt_eint_unmask(CUST_EINT_MHALL_NUM); 
                        }	
			else{
			  //mt_eint_mask(CUST_EINT_MHALL_NUM);  
			  ah1887_report_key();
			}
			ah1887_report_key_mittest(KEY_HALL_UP) ;  //lc --litao add 20150522 for midtest
	       }
	}

	eint_bit = mt_get_gpio_in(GPIO_MHALL_EINT_PIN);	
	// low close
	if(eint_bit == last_status)
	{
		LCSH_DEBUG("dismiss status;\n");
	}
	if(eint_bit == 0){
		atomic_set(&at_close, 1); // 1 leave High
		mt_eint_set_polarity(CUST_EINT_MHALL_NUM, CUST_EINT_POLARITY_HIGH);
	        atomic_set(&at_suspend, 1); // swf add 20150917
	        atomic_set(&at_resume, 0);
	       LCSH_DEBUG("line=%d,tpd_enable_high_sensitivity = 1.\n",__LINE__);
		//report key or not--->normal:get_mask=1; mid_test,get_mask=1,but can go to sleep if hand mask
        }
	else{
		atomic_set(&at_close, 0); // low close
		mt_eint_set_polarity(CUST_EINT_MHALL_NUM, CUST_EINT_POLARITY_LOW);
	        atomic_set(&at_suspend, 0); // swf add 20150917
	        atomic_set(&at_resume, 1);
                LCSH_DEBUG("line=%d,tpd_enable_high_sensitivity = 0.\n",__LINE__);
		//report key or not--->normal:get_mask=1; mid_test,get_mask=1;but can not go to sleep if hand mask,so can not wake up
	}
        mt_eint_unmask(CUST_EINT_MHALL_NUM);
       // LCSH_DEBUG("line=%d,at_close=%d,get_mask=%d.\n", __LINE__,atomic_read(&at_close),mt_eint_get_mask(CUST_EINT_MHALL_NUM));
}

static void ah1887_eint2_work(struct work_struct *work)  
{
	unsigned char eint_bit;
	unsigned char last_status;
	last_status = atomic_read(&at_close);

	if(hall_kpd != NULL) 
	{
         //       LCSH_DEBUG("line=%d,last_status=%d,get_mask=%d.\n",__LINE__,last_status,mt_eint_get_mask(CUST_EINT_MHALL_1_NUM));
		if( 0 == last_status){
			if(atomic_read(&at_suspend) ==1){
                         // LCSH_DEBUG("line=%d,eint2=0,supend=1,get_mask=%d.\n",__LINE__,mt_eint_get_mask(CUST_EINT_MHALL_1_NUM));
			  //mt_eint_unmask(CUST_EINT_MHALL_1_NUM); 
		        }
			else{
			  //mt_eint_mask(CUST_EINT_MHALL_1_NUM);  
			   ah1887_report_key();
                        }
			   ah1887_report_key_mittest(KEY_HALL_DOWN) ;	//lc --litao add 20150522 for midtest
		}
		else{
			if(atomic_read(&at_resume) ==1){
                           //LCSH_DEBUG("line=%d,eint2=1,resume=1,get_mask=%d.\n",__LINE__,mt_eint_get_mask(CUST_EINT_MHALL_1_NUM));
                           //mt_eint_unmask(CUST_EINT_MHALL_1_NUM);
                        }	
			else{
			//  mt_eint_mask(CUST_EINT_MHALL_1_NUM);  
			    ah1887_report_key();
			}
			ah1887_report_key_mittest(KEY_HALL_UP) ;  //lc --litao add 20150522 for midtest
	       }
	}

	eint_bit = mt_get_gpio_in(GPIO_MHALL1_EINT_PIN);
	// low close
	if(eint_bit == last_status)
	{
		LCSH_DEBUG("dismiss status;\n");
	}
	if(eint_bit == 0){
	       atomic_set(&at_close, 1); // 1 leave High
	       mt_eint_set_polarity(CUST_EINT_MHALL_1_NUM, CUST_EINT_POLARITY_HIGH);
	       atomic_set(&at_suspend, 1); 
	       atomic_set(&at_resume, 0);
	       LCSH_DEBUG("line=%d,ah1887 tpd_enable_high_sensitivity = 1.\n",__LINE__);
	}
	else{
		atomic_set(&at_close, 0); // low close
		mt_eint_set_polarity(CUST_EINT_MHALL_1_NUM, CUST_EINT_POLARITY_LOW);
	        atomic_set(&at_suspend, 0);
	        atomic_set(&at_resume, 1);
                LCSH_DEBUG("line=%d,ah1887 tpd_enable_high_sensitivity = 0.\n",__LINE__);
	}
        mt_eint_unmask(CUST_EINT_MHALL_1_NUM);
       // LCSH_DEBUG("line=%d,at_close=%d,get_mask=%d.\n", __LINE__,atomic_read(&at_close),mt_eint_get_mask(CUST_EINT_MHALL_1_NUM));
}

int ah1887_setup_input(void)
{
	int ret;
	printk("[hall]: %s  line = %d\n", __func__,__LINE__);
	hall_kpd = input_allocate_device();
	hall_kpd->name = "HALL";
	hall_kpd->id.bustype = BUS_HOST;
	__set_bit(EV_KEY, hall_kpd->evbit);

	set_bit( KEY_POWER,  hall_kpd->keybit );// add by zhaofei - 2014-09-26-09-36
	set_bit( KEY_HALL_DOWN,  hall_kpd->keybit );
	set_bit( KEY_HALL_UP,  hall_kpd->keybit );
//	set_bit( KEY_HALL_N_OPEN,  hall_kpd->keybit );
//	set_bit( KEY_HALL_N_CLOSE,  hall_kpd->keybit );
//	set_bit( KEY_HALL_S_OPEN,  hall_kpd->keybit );
//	set_bit( KEY_HALL_S_CLOSE,  hall_kpd->keybit );

	ret = input_register_device(hall_kpd);
	if(ret)
	{
		printk("[hall]: %s register inputdevice failed\n", __func__);
	}

   
	return 0;
}

static void ah1887_early_suspend(struct early_suspend *h) 
{
  atomic_set(&at_suspend, 1);
  atomic_set(&at_resume, 0);
#if 0 //open it if choose hand umask
  if (atomic_read(&at_close)){
     mt_eint_unmask(CUST_EINT_MHALL_NUM);  
     mt_eint_unmask(CUST_EINT_MHALL_1_NUM); 
     LCSH_DEBUG("early_suspend=%d,resume=%d,at_close=%d.\n",atomic_read(&at_suspend),atomic_read(&at_resume),atomic_read(&at_close));
     LCSH_DEBUG("line=%d,mask1=%d,mask2=%d.\n",__LINE__,mt_eint_get_mask(CUST_EINT_MHALL_NUM),mt_eint_get_mask(CUST_EINT_MHALL_1_NUM));
  }
#endif
  return;
}

static void ah1887_late_resume(struct early_suspend *h)
{
  atomic_set(&at_suspend, 0);
  atomic_set(&at_resume, 1);
#if 0 //open it if choose hand umask
  if (!atomic_read(&at_close)){
     mt_eint_unmask(CUST_EINT_MHALL_NUM);
     mt_eint_unmask(CUST_EINT_MHALL_1_NUM);  
     LCSH_DEBUG("late_resume=%d,suspend=%d,!at_close=%d.\n",atomic_read(&at_resume),atomic_read(&at_suspend),!atomic_read(&at_close));
     LCSH_DEBUG("line=%d,mask1=%d,mask2=%d.\n",__LINE__,mt_eint_get_mask(CUST_EINT_MHALL_NUM),mt_eint_get_mask(CUST_EINT_MHALL_1_NUM)); 
  }
#endif	
  return;
}

int ah1887_setup_earlySuspend(void)
{
	printk("[hall]: %s  line = %d\n", __func__,__LINE__);
	#if defined(CONFIG_HAS_EARLYSUSPEND)
	//early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	early_drv.level    = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1,
	early_drv.suspend  = ah1887_early_suspend,
	early_drv.resume   = ah1887_late_resume,    
	register_early_suspend(&early_drv);
	#endif

	return 0;
}


static int ah1887_probe(struct platform_device *pdev) 
{
	printk("[hall]: %s  line = %d\n", __func__,__LINE__);
	INIT_WORK(&eint1_work, ah1887_eint1_work);

	INIT_WORK(&eint2_work, ah1887_eint2_work);

	atomic_set(&at_suspend, 0);
	
	mutex_init(&mtx_eint_status);
	ah1887_setup_eint();

	ah1887_setup_input();
	
	ah1887_setup_earlySuspend();

	mt_eint_unmask(CUST_EINT_MHALL_NUM); 
	mt_eint_unmask(CUST_EINT_MHALL_1_NUM); 
	
	ah1887_create_proc();// add by zhaofei - 2014-09-18-23-03
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int ah1887_remove(struct platform_device *pdev)
{
	printk("[hall]: %s  line = %d\n", __func__,__LINE__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver ah1887_driver = {
	.probe      = ah1887_probe,
	.remove     = ah1887_remove,    
	.driver     = {
		.name  = "ah1887",
//		.owner = THIS_MODULE,
	}
};
/*----------------------------------------------------------------------------*/


static struct platform_device ah1887_device = {
	.name = "ah1887"
};

static int __init ah1887_init(void)
{
	int retval;
	printk("[hall]: %s \n", __func__);
	
	 retval = platform_device_register(&ah1887_device);
     if (retval != 0){
	 	printk("[hall]: %s failed to register ah1887 device\n", __func__);
          return retval;
    }
	if(platform_driver_register(&ah1887_driver))
	{
		printk("[hall] failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ah1887_exit(void)
{
	platform_driver_unregister(&ah1887_driver);
}
/*----------------------------------------------------------------------------*/
module_init(ah1887_init);
module_exit(ah1887_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("longcheer");
MODULE_DESCRIPTION("BU52031NVX driver");
MODULE_LICENSE("GPL");

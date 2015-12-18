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
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/seq_file.h>
#include <asm/io.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
//#include <linux/earlysuspend.h>  //swf delete
//#include <mach/mt_devs.h>
//#include <mach/mt_typedefs.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>
//#include <cust_eint.h>
#define HALL_PROC_FILE "hall_status"
#define HALL_NEED_ENABLE_SWITCH
#ifdef HALL_NEED_ENABLE_SWITCH
	#define HALL_ENABLE_PROC_FILE "hall_enable"
#endif
static u16 nEnableKey = 1;

/********************swf add for printk start*************************/
//#define LCSH_DEBUG_HZ

#define TPD_DEVICE   "hall ah1887"
#if defined(LCSH_DEBUG_HZ)
#define LCSH_DEBUG(a, arg...)    printk(TPD_DEVICE " line=%d,func=%s: " a, __LINE__,__func__,##arg)
#define LCSH_DEBUG_USER(a, arg...)    printk(TPD_DEVICE " line=%d,func=%s: " a, __LINE__,__func__,##arg)
#else
#define LCSH_DEBUG_USER(a, arg...)      printk(TPD_DEVICE "line=%d,func=%s: " a, __LINE__,__func__,##arg)
#define LCSH_DEBUG(arg...)
#endif

#include <linux/module.h>
#include <linux/init.h>
//#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
//#include <debug.h>

#include <linux/notifier.h>
#include <linux/fb.h>

//#include <mt-plat/mt_gpio.h>
//#include <mt-plat/mt_gpio_core.h>

static int ah1887_gpio_number1=0;
static int ah1887_gpio_number2=0;

static unsigned int hall_irq_eint1,hall_irq_eint2;

static struct notifier_block hall_fb_notif;

#undef   LC_DEVINFO_HALL
#define  LC_DEVINFO_HALL
#ifdef   LC_DEVINFO_HALL
#include <linux/dev_info.h>
static void hall_devinfo_init(void);
#endif

/********************swf add for printk end************************/


/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define AH1887_HALL_STATUS_OPEN 112
#define AH1887_HALL_STATUS_CLOSE 113


#if 0//swf delete
//#define CUST_EINT_MHALL_NUM		hall_irq_eint1//CUST_EINT_HALL_1_NUM   //swf modify
#define	GPIO_MHALL_EINT_PIN		1//GPIO_HALL_1_PIN 										
#define	GPIO_MHALL_EINT_PIN_M_EINT	1//GPIO_HALL_1_PIN_M_EINT
#define	GPIO_MHALL_EINT_PIN_M_GPIO	1//GPIO_HALL_1_PIN_M_GPIO

#define CUST_EINT_MHALL_1_NUM		2//CUST_EINT_HALL_2_NUM //swf modify
#define	GPIO_MHALL1_EINT_PIN		2//GPIO_HALL_2_PIN 										
#define	GPIO_MHALL1_EINT_PIN_M_EINT	3//GPIO_HALL_2_PIN_M_EINT
#define	GPIO_MHALL1_EINT_PIN_M_GPIO	4//GPIO_HALL_2_PIN_M_GPIO
#endif


/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern int mt_eint_get_mask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern int mt_get_gpio_in(unsigned long pin);


static atomic_t at_suspend;
static atomic_t at_resume;
static atomic_t	 at_close_one ;	// 0 close Low;  1 leave High;
static atomic_t	 at_close_two ;	// 0 close Low;  1 leave High;
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
	status = atomic_read(&at_close_one);
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
	status = atomic_read(&at_close_one);
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

static int hall_enable_proc_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	unsigned char writebuf[10];
	
	if (copy_from_user(&writebuf, buff, 10)) {
		
		printk("%s:copy from user error\n", __func__);
		return -EFAULT;
	}

	printk("hall_enable_proc_write len:%d", buflen);
	printk("hall_enable_proc_write str:%s", writebuf);

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

void ah1887_create_proc(void)
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

void ah1887_eint1_func(void)
{
	//mt_eint_mask(ah1887_gpio_number1);	//mt_eint_mask(CUST_EINT_MHALL_NUM);  
	LCSH_DEBUG("hall_irq_eint1=%d,get_mask=%d.\n",hall_irq_eint1,mt_eint_get_mask(ah1887_gpio_number1));
	LCSH_DEBUG_USER("hall_irq_eint1=%d.\n",hall_irq_eint1);
	schedule_work(&eint1_work);
	disable_irq_nosync(hall_irq_eint1);
	//ah1887_eint1_work(&eint1_work);
}

void ah1887_eint2_func(void)
{
	//mt_eint_mask(ah1887_gpio_number2);	//mt_eint_mask(CUST_EINT_MHALL_NUM);  
	LCSH_DEBUG("hall_irq_eint2=%d,get_mask=%d.\n",hall_irq_eint2,mt_eint_get_mask(ah1887_gpio_number2));
	LCSH_DEBUG_USER("hall_irq_eint2=%d.\n",hall_irq_eint2);
	schedule_work(&eint2_work);
	disable_irq_nosync(hall_irq_eint2);
	//ah1887_eint2_work(&eint2_work);
}


#define CUST_EINT_MHALL_SENSITIVE 1
#define CUST_EINT_POLARITY_LOW              0
#define CUST_EINT_POLARITY_HIGH             1

// add by zhaofei - 2014-09-29-13-46
///del litao/#define GPIO_MHALL1_EINT_PIN         (GPIO117 | 0x80000000)


void ah1887_setup_eint1(void)  //walker
{

	mutex_lock(&mtx_eint_status);
#if 1 //swf add for setup_eint
	int ret;
	//unsigned char eint1_bit = 0;
	unsigned char eint1_bit_one = 9,eint2_bit_two =9;
	u32 ints[2] = {0, 0};
	struct device_node *irq_node; 
	unsigned int hall_irq;
#endif

	irq_node = of_find_compatible_node(NULL, NULL, "mediatek,ah1887_eint1");
	if(!irq_node)
        LCSH_DEBUG_USER("irq_node of_find_compatible_node is error eeeeeeeeee\n");
	if (irq_node) {
		of_property_read_u32_array(irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "ah1887");
		gpio_set_debounce(ints[0], ints[1]);
		hall_irq = irq_of_parse_and_map(irq_node, 0);
		if (!hall_irq) {
			LCSH_DEBUG_USER("irq_of_parse_and_map fail fail fail fail!!\n");
			return -1;
		}

		if(123 == ints[0]){
		        ah1887_gpio_number1 = ints[0];
			hall_irq_eint1 = hall_irq;   //eint number
                        gpio_direction_input(ah1887_gpio_number1);//(123);	 //eint gpio
			eint1_bit_one=gpio_get_value(ah1887_gpio_number1);//(123);

LCSH_DEBUG("ints[0]=%d,ints[1]=%d,hall_irq_eint1=%d,eint1_bit_one=%d,get_mask=%d.*!!\n",ints[0],ints[1],hall_irq_eint1,eint1_bit_one,mt_eint_get_mask(ah1887_gpio_number1)); 
LCSH_DEBUG_USER("ints[0]=%d,ints[1]=%d,hall_irq_eint1=%d,eint1_bit_one=%d.\n",ints[0],ints[1],hall_irq_eint1,eint1_bit_one); 
			if (eint1_bit_one == 0){
				LCSH_DEBUG_USER("00000000000000000000000.\n");
				atomic_set(&at_close_one, 1); // near  CUST_EINT_POLARITY_HIGH IRQF_TRIGGER_LOW
				ret = request_irq(hall_irq_eint1, ah1887_eint1_func, CUST_EINT_POLARITY_HIGH, "ah1887_eint1", NULL);
				if(ret > 0){
					LCSH_DEBUG_USER("request_irq  error  error error ret=%d .\n",ret);
					return -1;
				}
			}else{
				LCSH_DEBUG_USER("11111111111111111111111.\n");
				atomic_set(&at_close_one, 0); // near  CUST_EINT_POLARITY_HIGH IRQF_TRIGGER_LOW CUST_EINT_POLARITY_LOW
				ret = request_irq(hall_irq_eint1, ah1887_eint1_func, IRQF_TRIGGER_LOW, "ah1887_eint1", NULL);
				if(ret > 0){
					LCSH_DEBUG_USER("request_irq  error  error error ret=%d .\n",ret);
					return -1;
				}
			}
       			 mt_eint_mask(ah1887_gpio_number1); 
			 LCSH_DEBUG("eint1_bit_one=%d,at_close_one=%d,eint_get_mask=%d .\n",eint1_bit_one,at_close_one,mt_eint_get_mask(ah1887_gpio_number1));
			//enable_irq(hall_irq_eint1);	
		}else {
		LCSH_DEBUG_USER(" null irq node!!null null\n");
		return -1;
	       }
	}  //if (irq_node)
    mutex_unlock(&mtx_eint_status); 		
    return 0;
}


void ah1887_setup_eint2(void)  //walker
{
	mutex_lock(&mtx_eint_status);
#if 1 //swf add for setup_eint
	int ret;
	//unsigned char eint1_bit = 0;
	unsigned char eint1_bit_one = 9,eint2_bit_two =9;
	u32 ints[2] = {0, 0};
	struct device_node *irq_node; 
	unsigned int hall_irq;
#endif

	irq_node = of_find_compatible_node(NULL, NULL, "mediatek,ah1887_eint2");
	if(!irq_node)
        LCSH_DEBUG_USER("irq_node of_find_compatible_node is error eeeeeeeeee\n");
	hall_irq = irq_of_parse_and_map(irq_node, 0);
	if (irq_node) {
		of_property_read_u32_array(irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "ah1887");
		gpio_set_debounce(ints[0], ints[1]);
		hall_irq = irq_of_parse_and_map(irq_node, 0);
		if (!hall_irq) {
			LCSH_DEBUG_USER("irq_of_parse_and_map fail fail fail fail!!\n");
			return -1;
		}
		if(122 == ints[0]){
 			ah1887_gpio_number2 = ints[0];
			hall_irq_eint2 = hall_irq;
                        gpio_direction_input(ah1887_gpio_number2);//(122);	
			eint2_bit_two=gpio_get_value(ah1887_gpio_number2);//(122);
LCSH_DEBUG("ints[0]=%d,ints[1]=%d,hall_irq_eint1=%d,eint1_bit_one=%d,get_mask=%d.*!!\n",ints[0],ints[1],hall_irq_eint2,eint2_bit_two,mt_eint_get_mask(ah1887_gpio_number2)); 
LCSH_DEBUG_USER("ints[0]=%d,ints[1]=%d,hall_irq_eint2=%d,eint1_bit_two=%d.\n",ints[0],ints[1],hall_irq_eint2,eint2_bit_two); 

			if (eint2_bit_two == 0){
				LCSH_DEBUG_USER("22222222222222222222222222.\n");
				atomic_set(&at_close_two, 1); // near  CUST_EINT_POLARITY_HIGH IRQF_TRIGGER_LOW
				ret = request_irq(hall_irq_eint2, ah1887_eint2_func, CUST_EINT_POLARITY_HIGH, "ah1887_eint2", NULL);
				if(ret > 0){
					LCSH_DEBUG_USER("request_irq  error  error error ret=%d .\n",ret);
					return -1;
				}
			}else{
				LCSH_DEBUG_USER("333333333333333333333.\n");
				atomic_set(&at_close_two, 0); // near  CUST_EINT_POLARITY_HIGH IRQF_TRIGGER_LOW CUST_EINT_POLARITY_LOW
				ret = request_irq(hall_irq_eint2, ah1887_eint2_func, IRQF_TRIGGER_LOW, "ah1887_eint2", NULL);
				if(ret > 0){
					LCSH_DEBUG_USER("request_irq  error  error error ret=%d .\n",ret);
					return -1;
				}
			}
      			 mt_eint_mask(ah1887_gpio_number2); 
			 LCSH_DEBUG("eint1_bit_one=%d,at_close_two=%d,eint_get_mask=%d .\n",eint2_bit_two,at_close_two,mt_eint_get_mask(ah1887_gpio_number2));
			//enable_irq(hall_irq_eint2);
		}else {
		LCSH_DEBUG_USER(" null irq node!!null null\n");
		return -1;
	       }
	} //if (irq_node)
    mutex_unlock(&mtx_eint_status); 		
    return 0;
}

//#if 0  ////lc --litao open 20150522 for midtest
static void ah1887_report_key_mittest( int keycode)
{
	printk("[hall] ah1887_report_key mitest;\n");
		input_report_key(hall_kpd, keycode, 1);
		input_report_key(hall_kpd, keycode, 0);	    	
		input_sync(hall_kpd);	
}
//#else 
static void ah1887_report_key(void)
{
	printk("[hall] ah1887_report_key;\n");
		input_report_key(hall_kpd, KEY_POWER, 1);
		input_report_key(hall_kpd, KEY_POWER, 0);	    	
		input_sync(hall_kpd);	
}
//#endif
///del litao temp/ extern int tpd_enable_high_Sensitivity(int enable);
static void ah1887_eint1_work(struct work_struct *work)  //walker
{
	unsigned char eint_bit=6;
	unsigned char last_status;
	//mutex_lock(&mtx_eint_status);
	last_status = atomic_read(&at_close_one);
        gpio_direction_input(ah1887_gpio_number1);//(123);
	eint_bit = gpio_get_value(ah1887_gpio_number1);//(123);
        LCSH_DEBUG("last_status=%d,eint_bit=%d,get_mask=%d.\n",last_status,eint_bit,mt_eint_get_mask(ah1887_gpio_number1));

	if(hall_kpd != NULL)
	{
		if( 0 == last_status)
			////ah1887_report_key(KEY_HALL_S_CLOSE);
			{
				if(atomic_read(&at_suspend) ==1){
				LCSH_DEBUG("eint1=0,supend=1,get_mask=%d.\n",mt_eint_get_mask(ah1887_gpio_number1));
				LCSH_DEBUG_USER("eint1=0,supend=1 .\n");
				}else{
				LCSH_DEBUG("ah1887_report_key 000000000.\n");
				ah1887_report_key();
				}
				ah1887_report_key_mittest(KEY_HALL_DOWN); //for factory mode
		}
		else{
				////ah1887_report_key(KEY_HALL_S_OPEN);
				if(atomic_read(&at_resume) ==1){
				LCSH_DEBUG("eint1=1,supend=0,get_mask=%d.\n",mt_eint_get_mask(ah1887_gpio_number1));
				LCSH_DEBUG_USER("eint1=1,supend=0.\n");
				}else{
				LCSH_DEBUG("ah1887_report_key 11111111.\n");
				ah1887_report_key();
				}
				ah1887_report_key_mittest(KEY_HALL_UP);//for factory mode
		}
	}
	
	if(eint_bit == last_status){
		LCSH_DEBUG_USER(" dismiss status;\n");
	}
	
	if(eint_bit == 0){
		atomic_set(&at_close_one, 1); // 1 leave High
		mt_eint_set_polarity(ah1887_gpio_number1,CUST_EINT_POLARITY_HIGH); //IRQF_TRIGGER_LOW
		atomic_set(&at_suspend, 1);
		atomic_set(&at_resume, 0);
                LCSH_DEBUG_USER("at_suspend=%d,at_resume=%d,tpd_enable_high_sensitivity = 1.\n",at_suspend,at_resume);
	}
	else{
		atomic_set(&at_close_one, 0); // low close
		mt_eint_set_polarity(ah1887_gpio_number1, CUST_EINT_POLARITY_LOW);
		atomic_set(&at_suspend, 0);
		atomic_set(&at_resume, 1);
               LCSH_DEBUG_USER("at_suspend=%d,at_resume=%d,tpd_enable_high_sensitivity = 0.\n",at_suspend,at_resume);
	}
	 enable_irq(hall_irq_eint1);
	//mt_eint_unmask(ah1887_gpio_number1);
        LCSH_DEBUG("at_close_one=%d,get_mask=%d.\n",at_close_one,mt_eint_get_mask(ah1887_gpio_number1));
}


static void ah1887_eint2_work(struct work_struct *work)
{
	unsigned char eint_bit=6;
	unsigned char last_status;
	//mutex_lock(&mtx_eint_status);
	last_status = atomic_read(&at_close_two);
        gpio_direction_input(ah1887_gpio_number2);//(122);
	eint_bit = gpio_get_value(ah1887_gpio_number2);//(122);
	//eint_bit = mt_get_gpio_in(ah1887_gpio_number1);//GPIO_MHALL_EINT_PIN);  // double check for set eint
        LCSH_DEBUG("last_status=%d,eint_bit=%d,get_mask=%d.\n",last_status,eint_bit,mt_eint_get_mask(ah1887_gpio_number2));

	if(hall_kpd != NULL)
	{
		if( 0 == last_status)
			////ah1887_report_key(KEY_HALL_S_CLOSE);
			{
				if(atomic_read(&at_suspend) ==1){
				LCSH_DEBUG("eint1=0,supend=1,get_mask=%d.\n",mt_eint_get_mask(ah1887_gpio_number2));
				LCSH_DEBUG_USER("eint1=0,supend=1.\n");
				}else{
				LCSH_DEBUG("ah1887_report_key 000000000.\n");
				ah1887_report_key();
				}
				ah1887_report_key_mittest(KEY_HALL_DOWN); //for factory mode
		}
		else{
				////ah1887_report_key(KEY_HALL_S_OPEN);
				if(atomic_read(&at_resume) ==1){
				LCSH_DEBUG("eint1=1,supend=0,get_mask=%d.\n",mt_eint_get_mask(ah1887_gpio_number2));
				LCSH_DEBUG_USER("eint1=1,supend=0.\n");
				}else{
				LCSH_DEBUG("ah1887_report_key 11111111.\n");
				ah1887_report_key();
				}
				ah1887_report_key_mittest(KEY_HALL_UP); //for factory mode
		}
	}
	
	if(eint_bit == last_status){
		LCSH_DEBUG_USER(" dismiss status;\n");
	}
	
	if(eint_bit == 0){
		atomic_set(&at_close_two, 1); // 1 leave High
		mt_eint_set_polarity(ah1887_gpio_number2,CUST_EINT_POLARITY_HIGH); //IRQF_TRIGGER_LOW
		atomic_set(&at_suspend, 1);
		atomic_set(&at_resume, 0);
               LCSH_DEBUG_USER("at_suspend=%d,at_resume=%d,tpd_enable_high_sensitivity = 1.\n",at_suspend,at_resume);
	}
	else{
		atomic_set(&at_close_two, 0); // low close
		mt_eint_set_polarity(ah1887_gpio_number2, CUST_EINT_POLARITY_LOW);
	        atomic_set(&at_suspend, 0);
		atomic_set(&at_resume, 1);
               LCSH_DEBUG_USER("at_suspend=%d,at_resume=%d,tpd_enable_high_sensitivity = 0.\n",at_suspend,at_resume);
	}
	//disable_irq(hall_irq_eint1);
	//mt_eint_unmask(ah1887_gpio_number2);
	 enable_irq(hall_irq_eint2);
        LCSH_DEBUG("at_close_one=%d,get_mask=%d.\n",at_close_two,mt_eint_get_mask(ah1887_gpio_number2));
}


void ah1887_setup_input(void)
{
	int ret;
	LCSH_DEBUG_USER("\n" );
	hall_kpd = input_allocate_device();
	hall_kpd->name = "HALL";
	hall_kpd->id.bustype = BUS_HOST;
	__set_bit(EV_KEY, hall_kpd->evbit);

	set_bit( KEY_POWER,  hall_kpd->keybit );// add by zhaofei - 2014-09-26-09-36
	set_bit( KEY_HALL_DOWN,  hall_kpd->keybit );
	set_bit( KEY_HALL_UP,  hall_kpd->keybit );

	ret = input_register_device(hall_kpd);
	if(ret){
		LCSH_DEBUG_USER(" register inputdevice failed\n");
	}
	return 0;
}

#if 0
static void ah1887_pm_suspend(struct device *h) 
{
	printk("[hall]: %s \n", __func__);
	atomic_set(&at_suspend, 1);
	atomic_set(&at_resume, 0);
	return;
}

static void ah1887_pm_resume(struct device *h)
{
	printk("[hall]: %s \n", __func__);
	atomic_set(&at_suspend, 0);
	atomic_set(&at_resume, 1);
	return;
}

static void ah1887_suspend(struct platform_device *dev, pm_message_t state) 
{
	printk("[hall]: %s \n", __func__);
	atomic_set(&at_suspend, 1);
	atomic_set(&at_resume, 0);
	return;
}

static void ah1887_resume(struct platform_device *dev)
{
	printk("[hall]: %s \n", __func__);
	atomic_set(&at_suspend, 0);
	atomic_set(&at_resume, 1);
	return;
}
#endif

#if 0
void ah1887_setup_earlySuspend(void)
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
#endif

static int hall_fb_notifier_callback(struct notifier_block *self,unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
         
        LCSH_DEBUG_USER(" event = %d\n", event);
	if (evdata && evdata->data && event == FB_EVENT_BLANK ) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK) {
				LCSH_DEBUG_USER("event =16,going to resume.");
				atomic_set(&at_suspend, 0);
				atomic_set(&at_resume, 1);
			}else if (*blank == FB_BLANK_POWERDOWN) {
				LCSH_DEBUG_USER("event =9,going to suspend .\n");
				atomic_set(&at_suspend, 1);
				atomic_set(&at_resume, 0);
			}
	}
	return 0;
}

#ifdef   LC_DEVINFO_HALL
static void hall_devinfo_init(void)
{
	static struct devinfo_struct *devinfo_hall = NULL;
	devinfo_hall = kzalloc(sizeof(struct devinfo_struct), GFP_KERNEL);    
	if(NULL != devinfo_hall)
	{
		devinfo_hall->device_type = "HALL";
		devinfo_hall->device_module = "lampek";
		devinfo_hall->device_vendor = "unknow"; 
		devinfo_hall->device_ic = "ah1887 ";
		devinfo_hall->device_version = "unknow";
		devinfo_hall->device_info = "unknow";
		devinfo_hall->device_used = DEVINFO_USED;
		devinfo_check_add_device(devinfo_hall);
	}else{
 		LCSH_DEBUG_USER("swf77 failed to create tp deviinfo **\n");
	}
}
#endif

static int ah1887_probe(struct platform_device *pdev)  //swfprobe
{
	int error;
	
	LCSH_DEBUG_USER(".\n");
	INIT_WORK(&eint1_work, ah1887_eint1_work);
	INIT_WORK(&eint2_work, ah1887_eint2_work);   //should be opened after swf

	atomic_set(&at_suspend, 0);
	mutex_init(&mtx_eint_status);

	ah1887_setup_eint1();
	ah1887_setup_eint2();
	ah1887_setup_input();
	
	//ah1887_setup_earlySuspend();
	//disable_irq(hall_irq_eint1);
	mt_eint_unmask(ah1887_gpio_number1);//(CUST_EINT_MHALL_NUM); 
	mt_eint_unmask(ah1887_gpio_number2);//(CUST_EINT_MHALL_1_NUM);  //should be opened after swf


	hall_fb_notif.notifier_call = hall_fb_notifier_callback;
	error = fb_register_client(&hall_fb_notif);
	if (error) {
		LCSH_DEBUG_USER("Unable to register fb_notifier: %d\n",error);
	}

#ifdef   LC_DEVINFO_HALL
	hall_devinfo_init();
#endif
	//ah1887_create_proc();// add by zhaofei - 2014-09-18-23-03   //swf delete
	LCSH_DEBUG("get_mask=%d,ok ok ok ok ok ok",mt_eint_get_mask(ah1887_gpio_number1));
	LCSH_DEBUG("get_mask=%d,ok ok ok ok ok ok",mt_eint_get_mask(ah1887_gpio_number2));
	LCSH_DEBUG_USER("ok ok ok ok ok ok.\n");		
	return 0;
}
/*----------------------------------------------------------------------------*/
static int ah1887_remove(struct platform_device *pdev)
{
	printk("[hall]: %s  line = %d\n", __func__,__LINE__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct of_device_id ah1887_of_match[] = {
        { .compatible = "mediatek,ah1887_eint1",/*"mediatek,ah1887_eint2"*/},  //ah1887_eint2
        {},
};


#if 0
const struct dev_pm_ops ah1887_pm_ops = {
	.suspend = ah1887_pm_suspend,
	.resume = ah1887_pm_resume,
};
#endif

static struct platform_driver ah1887_driver = {
	.probe      = ah1887_probe,
	.remove     = ah1887_remove,    
	.driver     = {
#ifdef CONFIG_PM
		//.pm = &ah1887_pm_ops,
#endif
		.name  = "ah1887",
//		.owner = THIS_MODULE,
		.of_match_table = ah1887_of_match,
	},
	
	//.suspend = ah1887_suspend,
	//.resume = ah1887_resume,
};
/*----------------------------------------------------------------------------*/


static struct platform_device ah1887_device = {
	.name = "ah1887"
};

static int __init ah1887_init(void)
{
	int retval;
	//pr_debug("[hall]: line=%d,func=%s .\n",__LINE__, __func__);
	printk("[hall]: line=%d,func=%s .\n", __LINE__,__func__);
	
	 //retval = platform_device_register(&ah1887_device);
     //if (retval != 0){
//	 	printk("[hall]: %s failed to register ah1887 device\n", __func__);
  //        return retval;
    //}
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

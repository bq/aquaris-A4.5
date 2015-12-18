#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <asm/errno.h> 
#include <linux/cdev.h>
#include <linux/platform_device.h>

//#include <mach/mt_gpio.h>


#define KTD_I2C_NAME			"ktd2026"
#define KTD_I2C_ADDR			0x30

struct i2c_client *ktd20xx_client;

//static struct i2c_board_info __initdata i2c_ktd20xx= { I2C_BOARD_INFO(KTD_I2C_NAME, KTD_I2C_ADDR)};
//static int major;
//static struct class *ktd20xx_cls;
//struct class *breathled_class;
//struct device *breathled_dev;
//struct device *ktd22xx_dev;
typedef struct{
    u8 breadtime;
    u8 raisetime;
}BREAD_SPEED;

//low medium high spead value
BREAD_SPEED bread_value[]={
        {0x08,0x77},
        {0x05,0x44},
        {0x03,0x22}
};

//int breath_leds = 0;//1--breath_led,255--turn on 0--turn off
//int led_on_off = 255;
//unsigned long value = 0;
static char conf = 0;
static char value = 0;

void ktd22xx_lowbattery_breath_leds_red(u8 val){
	/*
	 * RED flash time period: 2.5s, rise/fall 1s, sleep 0.5s 
	 * reg5 = 0xaa, reg1 = 0x12
	 */
	conf |= 0x02; 
	printk("[LED] red blink 0x04 value is 0x%2x.\n",conf);
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x00);// mode set---IC work when both SCL and SDA goes high and normal speed
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	//i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0x4f);//set current is 10mA
    i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0x3f);//changed by BQ 20150616
	i2c_smbus_write_byte_data(ktd20xx_client, 0x05, bread_value[val].raisetime);//rase time
	i2c_smbus_write_byte_data(ktd20xx_client, 0x01, bread_value[val].breadtime);//dry flash period
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, conf);//allocate led1 to timer1 0x02
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)
}

void ktd22xx_lowbattery_breath_leds_green(u8 val){
	/*
	 * Green flash time period: 2.5s, rise/fall 1s, sleep 0.5s 
	 * reg5 = 0xaa, reg1 = 0x12
	 */
	conf |= 0x08;
	printk("[LED] green blink 0x04 value is 0x%2x.\n",conf);
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x00);// mode set---IC work when both SCL and SDA goes high and normal speed
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	//i2c_smbus_write_byte_data(ktd20xx_client, 0x07, 0x4f);//set current is 10mA
    i2c_smbus_write_byte_data(ktd20xx_client, 0x07, 0x3f);//changed by BQ 20150616
	i2c_smbus_write_byte_data(ktd20xx_client, 0x05, bread_value[val].raisetime);//rase time
	i2c_smbus_write_byte_data(ktd20xx_client, 0x01, bread_value[val].breadtime);//dry flash period
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, conf);//allocate led1 to timer1 0x08
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)
}

void ktd22xx_lowbattery_breath_leds_blue(u8 val){
	/*
	 * Blue flash time period: 2.5s, rise/fall 1s, sleep 0.5s 
	 * reg5 = 0xaa, reg1 = 0x12
	 */
	conf |= 0x20;
	printk("[LED] blue blink 0x04 value is 0x%2x.\n",conf);
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x00);// mode set---IC work when both SCL and SDA goes high and normal speed
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	//i2c_smbus_write_byte_data(ktd20xx_client, 0x08, 0x4f);//set current is 10mA
    i2c_smbus_write_byte_data(ktd20xx_client, 0x08, 0x3f);//changed by BQ 20150616
	i2c_smbus_write_byte_data(ktd20xx_client, 0x05, bread_value[val].raisetime);//rase time
	i2c_smbus_write_byte_data(ktd20xx_client, 0x01, bread_value[val].breadtime);//dry flash period
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, conf);//allocate led1 to timer1 0x20
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)
}

void ktd2xx_led_on_red(void){
        value |= 0x01;
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x00);//mode set---IC work when both SCL and SDA goes high
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	//i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0x4f);//set current is 10mA
    i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0x3f);//changed by BQ 20150616
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, value);//turn on led
}

void ktd2xx_led_on_green(void){
        value |= 0x04;
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x00);//mode set---IC work when both SCL and SDA goes high
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	//i2c_smbus_write_byte_data(ktd20xx_client, 0x07, 0x4f);//set current is 10mA
    i2c_smbus_write_byte_data(ktd20xx_client, 0x07, 0x3f);//changed by BQ 20150616
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, value);//turn on led
}

void ktd2xx_led_on_blue(void){
        value |= 0x10;
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x00);//mode set---IC work when both SCL and SDA goes high
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	//i2c_smbus_write_byte_data(ktd20xx_client, 0x08, 0x4f);//set current is 10mA
    i2c_smbus_write_byte_data(ktd20xx_client, 0x08, 0x3f);//changed by BQ 20150616
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, value);//turn on led
}

void ktd2xx_led_off_red(void){
        value &= 0xfc;
	conf &= 0xfc;
	i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0x00);//set current is 0mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, value);//turn off led

        if((value == 0x00)&&(conf == 0x00))
        {
           //conf = 0;
           i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x08);//Device OFF-Either SCL goes low or SDA stops toggling
           printk("[LED] set SCL low and SDA stops toggling\n");
        }

}

void ktd2xx_led_off_green(void){
        value &= 0xf3;
	conf &= 0xf3;
	i2c_smbus_write_byte_data(ktd20xx_client, 0x07, 0x00);//set current is 0mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, value);//turn off led

        if((value == 0x00)&&(conf == 0x00))
        {
           //conf = 0;
           i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x08);//Device OFF-Either SCL goes low or SDA stops toggling
           printk("[LED] set SCL low and SDA stops toggling\n");
        }

}

void ktd2xx_led_off_blue(void){
        value &= 0xcf;
	conf &= 0xcf;
	i2c_smbus_write_byte_data(ktd20xx_client, 0x08, 0x00);//set current is 0mA

        if((value == 0x00)&&(conf == 0x00))
        {
           //conf = 0;
           i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x08);//Device OFF-Either SCL goes low or SDA stops toggling
           printk("[LED] set SCL low and SDA stops toggling\n");
        }

}

void ktd2xx_led_off(void){
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x08);//Device OFF-Either SCL goes low or SDA stops toggling
}

#if 0 //zbl rm
void ktd22xx_breath_leds_time(int blink){
	//set breath led flash time from blink
	int period, flashtime;
	period = (blink >> 8) & 0xff;
	flashtime = blink & 0xff;
	printk("ktd20xx led write blink = %x, period = %x, flashtime = %x\n", blink, period, flashtime);

	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);// initialization LED off
	i2c_smbus_write_byte_data(ktd20xx_client, 0x00, 0x20);// mode set---IC work when both SCL and SDA goes high
	i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0xbf);//set current is 24mA
	i2c_smbus_write_byte_data(ktd20xx_client, 0x05, period);//rase time
	i2c_smbus_write_byte_data(ktd20xx_client, 0x01, flashtime);//dry flash period
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x00);//reset internal counter
	i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x02);//allocate led1 to timer1
	i2c_smbus_write_byte_data(ktd20xx_client, 0x02, 0x56);//led flashing(curerent ramp-up and down countinuously)

}

//EXPORT_SYMBOL(ktd22xx_breath_leds);
EXPORT_SYMBOL(ktd2xx_led_on);
EXPORT_SYMBOL(ktd2xx_led_off);
EXPORT_SYMBOL(breath_leds);
EXPORT_SYMBOL(led_on_off);

static ssize_t Breathled_switch_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
   return sprintf ( buf, "%d\n", breath_leds);
}
static ssize_t Breathled_switch_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count )
{
    if (NULL == buf)
        return -EINVAL;

    if (0 == count)
        return 0;

	 if(buf[0] == '0'){
		breath_leds = 0;
		led_on_off = 0;
		ktd2xx_led_off();
	 }
	 else if( buf[0] == '1'){
		breath_leds = 1;
		led_on_off = 1;
		ktd22xx_lowbattery_breath_leds_red();
	 }
	 else if( buf[0] == '2'){
		breath_leds = 2;
		led_on_off = 2;
		ktd22xx_lowbattery_breath_leds_green();
	 }
	 else if( buf[0] == '3'){
		breath_leds = 3;
		led_on_off = 3;
		ktd22xx_lowbattery_breath_leds_blue();
	 }
	 else if( buf[0] == '4'){
		breath_leds = 4;
		led_on_off = 4;
		ktd2xx_led_on_red();
	 }
	 else if( buf[0] == '5'){
		breath_leds = 5;
		led_on_off = 5;
		ktd2xx_led_on_green();
	 }
	 else if( buf[0] == '6'){
		breath_leds = 6;
		led_on_off = 6;
		ktd2xx_led_on_blue();
	 }
     else{
		breath_leds = 255;
		led_on_off = 7;
		ktd2xx_led_off();

	 }
	
	return count;

}

static ssize_t Breathled_switch_show2 ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
   return sprintf ( buf, "value :%ld\n", value);
}
static ssize_t Breathled_switch_store2 ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count )
{
	//unsigned long value = 0;
	int ret;
	
    if (NULL == buf)
        return -EINVAL;
	 
	ret = strict_strtoul(buf, 0, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:strict_strtoul failed, ret=%d\n", __func__, ret);	
		return ret;	
	}	
	//sprintf ( buf, "%d\n", sn3191_breath_led);
	ktd22xx_breath_leds_time(value);
	breath_leds = 1;
	
	return count;

}

static DEVICE_ATTR(led, 0666, Breathled_switch_show, Breathled_switch_store);
static DEVICE_ATTR(breath_led, 0666, Breathled_switch_show2, Breathled_switch_store2);
#endif

 #if 0
static void breathled_attr_create(void){
	breathled_class = class_create(THIS_MODULE, "breath_led");
	    if (IS_ERR(breathled_class))
	        pr_err("Failed to create class(gt_dclick_class)!\n");
	    breathled_dev = device_create(breathled_class,
	                                     NULL, 0, NULL, "breathled");
	    if (IS_ERR(breathled_dev))
	        pr_err("Failed to create device(breathled_dev)!\n");

	       // update
	    if (device_create_file(breathled_dev, &dev_attr_breath_led) < 0)
	        pr_err("Failed to create device file(%s)!\n", dev_attr_breath_led.attr.name);
}
#endif

#if 0 //zbl rm
static ssize_t ktd20xx_read(struct file *file, char __user *buf, size_t size, loff_t * offset)
{
	return 0;
}

static ssize_t ktd20xx_write(struct file *file, const char __user *buf, size_t size, loff_t *offset)
{

	static unsigned char rec_data[2];
	memset(rec_data, 0, 2);  //\C7\E5\C1\E3

	
	if (copy_from_user(rec_data, buf, 1))  
	{  
		return -EFAULT;  
	}  
	
	if(buf[0] == '0'){
	   breath_leds = 0;
	   led_on_off = 0;
	   ktd2xx_led_off();
	}
	else if( buf[0] == '1'){
	   breath_leds = 1;
	   led_on_off = 1;
	   ktd22xx_lowbattery_breath_leds();
	}
	else if( buf[0] == '2'){
	   breath_leds = 2;
	   led_on_off = 2;
	   ktd22xx_events_breath_leds();
	}

	else{
	   breath_leds = 255;
	   led_on_off = 3;
	   ktd2xx_led_on();
	}

	
    return 1;
}


static struct file_operations ktd20xx_fops = {
	.owner = THIS_MODULE,
	.read  = ktd20xx_read,
	.write = ktd20xx_write,
};
#endif

int leds_set_switch(char *which_led,int led_level,int blink_flag,int blink_value){
    u8 switch_flag = 0;
    u8 bread_kind = 0;

    if((strcmp(which_led, "red") == 0)){
	   switch_flag = 1;
    }else if((strcmp(which_led, "blue") == 0)){
	   switch_flag = 2;
    }else if((strcmp(which_led, "green") == 0)){
	   switch_flag = 3;
    }

    if(blink_value == 0){
           bread_kind = 3;//just turn on led,not bread
    }else if(blink_value <= 750){
           bread_kind = 2;//high speed
    }else if(blink_value <= 1250){
           bread_kind = 1;//medium speed
    }else{
           bread_kind = 0;//low speed
    }
    printk("[%s]: switch_flag is %d.bread_kind is %d.\n", __func__,switch_flag,bread_kind);

	if(0 == led_level){
          /* led off */
	     switch(switch_flag){
	     	    case 1:
			    ktd2xx_led_off_red();
			    break;
		    case 2:
			    ktd2xx_led_off_blue();
			    break;
		    case 3:
		     	    ktd2xx_led_off_green();
			    break;
		    default:
			    printk("[LED] close LED error \n");
			    break;
              }
          //ktd2xx_led_off();
	  return 0;
	}else if((0 == blink_flag)||(3 == bread_kind)){
         /* led on */
	     switch(switch_flag){
	     	    case 1:
			    ktd2xx_led_on_red();
			    break;
		    case 2:
			    ktd2xx_led_on_blue();
			    break;
		    case 3:
		     	    ktd2xx_led_on_green();
			    break;
		    default:
			    printk("[LED] open LED error \n");
			    break;
	      }
	  return 0;
	}else if(1 == blink_flag){
          /* led blink */
	      switch(switch_flag){
	    	    case 1:
			    ktd22xx_lowbattery_breath_leds_red(bread_kind);
			    break;
		    case 2:
			    ktd22xx_lowbattery_breath_leds_blue(bread_kind);
			    break;
		    case 3:
			    ktd22xx_lowbattery_breath_leds_green(bread_kind);
			    break;
		    default:
			    printk("[LED] open LED error \n");
			    break;
	      }
	  return 0;
	}
	
	return 0;
}

EXPORT_SYMBOL(leds_set_switch);
EXPORT_SYMBOL(ktd2xx_led_off);

static int ktd20xx_probe(struct i2c_client *client, const struct i2c_device_id *id){

	int ret;
	int err = 0;

	printk("[%s]: Enter!\n", __func__);
//	mt_set_gpio_mode(GPIO_VCHG_LED_CONTROL, GPIO_VCHG_LED_CONTROL_M_GPIO);		
//	mt_set_gpio_dir(GPIO_VCHG_LED_CONTROL, GPIO_DIR_OUT);	
//	mt_set_gpio_out(GPIO_VCHG_LED_CONTROL, GPIO_OUT_ONE);//zbl add
	
	
	
	ktd20xx_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!ktd20xx_client) {
		dev_err(&client->dev,
				"%s: memory allocation failed.", __func__);
		err = -ENOMEM;
		goto exit1;
	}

	ktd20xx_client = client;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
				"%s: check_functionality failed.", __func__);
		err = -ENODEV;
		goto exit0;
	}
	/* create i2c struct, when receve and transmit some byte */
	//printk("ktd2026 address is %x \n",ktd20xx_client->addr);

	//major = register_chrdev(0, "ktd20xx", &ktd20xx_fops);

	//ktd2xx_led_off(); //turn off led when first start ktd
	ret = i2c_smbus_write_byte_data(ktd20xx_client, 0x06, 0x00);//set current is 0.125mA
	ret = i2c_smbus_write_byte_data(ktd20xx_client, 0x04, 0x00);//turn off leds	
	if(ret < 0){
		printk("can't find ktd2026 led control ic!");
	}
	#if 0 //zbl rm
	else{
		/* create device node sys/class/ktd20xx/ktd2206/led
		 * led: 0:breath led 1:open led  2:close led
		 */	
		ktd20xx_cls = class_create(THIS_MODULE, "ktd20xx");
		ktd22xx_dev = device_create(ktd20xx_cls, NULL, MKDEV(major, 0), NULL, "ktd2026"); /* /dev/ktd20xx */
		if (device_create_file(ktd22xx_dev, &dev_attr_led) < 0)
		    pr_err("Failed to create device file(%s)!\n", dev_attr_led.attr.name);
		if (device_create_file(ktd22xx_dev, &dev_attr_breath_led) < 0)
		    pr_err("Failed to create device file(%s)!\n", dev_attr_led.attr.name);
	}
	#endif

exit0:
exit1:

	return err;
}

static int ktd20xx_remove(struct i2c_client *client){

	printk("[%s]: Enter!\n", __func__);
	
	unregister_chrdev(0, "ktd20xx");
	//class_destroy(ktd20xx_cls);
		
	return 0;
}
	

static const struct i2c_device_id ktd2xx_id[] = {
	{KTD_I2C_NAME, 0},
	{ }
};


static struct of_device_id ktd20xx_match_table[] = {
        { .compatible = "mediatek,ktd2026",},
        { },
};


static struct i2c_driver ktd20xx_driver = {
	.driver = {
		.name	= KTD_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ktd20xx_match_table,
	},
	.probe = ktd20xx_probe,
	.remove = ktd20xx_remove,
	.id_table = ktd2xx_id,
};

static struct pinctrl *pinctrl;
static struct pinctrl_state  *vchg_output1;

static int ktd20xx_rgb_probe(struct platform_device *pdev)
{
	int err = 0;
	pinctrl = devm_pinctrl_get(&pdev->dev);	
	if (IS_ERR(pinctrl)) {
                err = PTR_ERR(pinctrl);
                dev_err(&pdev->dev, "fwq Cannot find rgb pinctrl!\n");
                return err;
        }

	vchg_output1 = pinctrl_lookup_state(pinctrl, "state_vchg_output1");
        if (IS_ERR(vchg_output1)) {
                err = PTR_ERR(vchg_output1);
                dev_err(&pdev->dev, "fwq Cannot find rgb pinctrl state_vchg_output1!\n");
        }
	
	pinctrl_select_state(pinctrl, vchg_output1);

	err = i2c_add_driver(&ktd20xx_driver);
	if (err) {
		printk(KERN_WARNING "ktd20xx driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk( "Successfully added driver %s\n",
		          ktd20xx_driver.driver.name);
	}
	
	return err;	
}

static int ktd20xx_rgb_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id ktd20xx_rgb_of_match[] = {
        { .compatible = "mediatek,mt6735-RGBled", },
        {},
};

static struct platform_driver ktd20xx_rgb_driver = {
        .remove = ktd20xx_rgb_remove,
        .shutdown = NULL,
        .probe = ktd20xx_rgb_probe,
        .driver = {
                        .name = "mt6735-RGBled",
                        //.pm = &tpd_pm_ops,
                        .owner = THIS_MODULE,
                        .of_match_table = ktd20xx_rgb_of_match,
        },
};

static int __init ktd20xx_init(void)
{
//	int err;
	printk("%s\n",__func__);
#if 0
//	i2c_register_board_info(2, &i2c_ktd20xx, 1);
	err = i2c_add_driver(&ktd20xx_driver);
	if (err) {
		printk(KERN_WARNING "ktd20xx driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk( "Successfully added driver %s\n",
		          ktd20xx_driver.driver.name);
	}
#endif
        if (platform_driver_register(&ktd20xx_rgb_driver) != 0) {
		printk("%s error!\n",__func__);
                return -1;
        }

        return 0;
}

static void __exit ktd20xx_exit(void)
{
	printk("%s\n",__func__);
	//i2c_del_driver(&ktd20xx_driver);
}

module_init(ktd20xx_init);
module_exit(ktd20xx_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Berlin.Zhu");

#include <linux/kernel.h>	/* printk() */
#include <linux/module.h>
#include <linux/types.h>	/* size_t */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/proc_fs.h>  /*proc*/
#include <linux/genhd.h>    //special
#include <linux/cdev.h>
#include <asm/uaccess.h>   /*set_fs get_fs mm_segment_t*/

#include <linux/mmc/sd_misc.h>
#include "../mmc-host/mt6735/mt_sd.h"

//#define DEVINFO_DEBUG_EMCP 1
#define DEVINFO_DEBUG_EMCP 0

extern void msdc_check_init_done(void);
extern struct msdc_host *mtk_msdc_host[];

//////////////////////shaohui add for eMCP info//////////////////
#include  <linux/dev_info.h>

#include "../sltdevinfo/devinfo_emi.h"
unsigned int rom_size=0;
static int devinfo_register_emcp(struct msdc_host *host)
{	
	int i=0;
	char* type;
	char* module;
	char* vendor;
	char *ic;
	char *version;
	char *info=kmalloc(64,GFP_KERNEL);	//RAM SIZE
	char *info1=kmalloc(64,GFP_KERNEL);	//RAM SIZE
	unsigned long ram_size;
#ifdef DEVINFO_DEBUG_EMCP
    printk("[DEVINFO_EMCP][%s]: register emcp info.[%d]\n", __func__,num_of_emi_records);
    printk("[DEVINFO_EMCP][%s]: register emcp info.[%x]\n", __func__,host->mmc->card->type);
    printk("[DEVINFO_EMCP]: emcp device info.[%x][%x][%x][%x]\n",host->mmc->card->raw_cid[0],host->mmc->card->raw_cid[1],host->mmc->card->raw_cid[2],host->mmc->card->raw_cid[3]);
#endif

	for(i=0;i<num_of_emi_records;i++)
	{
#ifdef DEVINFO_DEBUG_EMCP
    printk("[DEVINFO_EMCP]: emcp list info.[%x][%x][%x][%x][%x][%x][%x][%x]\n",emi_settings[i].ID[0],emi_settings[i].ID[1],emi_settings[i].ID[2],emi_settings[i].ID[3],emi_settings[i].ID[4],emi_settings[i].ID[5],emi_settings[i].ID[6],emi_settings[i].ID[7]);
#endif
		//DEIVE TYPE
		//switch (host->mmc->card->type)
		//Note: In fact,we cannot get right device type here from host->mmc->card->type,So we have to define a knowned value
		switch (emi_settings[i].type)
			{
				case 0x0000:
					type=	DEVINFO_NULL;
					break;
				case 0x0001:
					type=	"RAM DDR1";
					break;
				case 0x0002:
					type=	"RAM DDR2";
					break;
				case 0x0003:
					type=	"RAM DDR3";
					break;
				case 0x0101:
					type=	"MCP(NAND+DDR1)";
					break;
				case 0x0102:
					type=	"MCP(NAND+DDR2)";
					break;
				case 0x0103:
					type=	"MCP(NAND+DDR3)";
					break;
				case 0x0201:
					type=	"MCP(eMMC+DDR1)";
					break;
				case 0x0202:
					type=	"MCP(eMMC+DDR2)";
					break;
				case 0x0203:
					type=	"MCP(eMMC+DDR3)";
					break;
				default:
					type=	DEVINFO_NULL;
					break;
			}
		//type = "RAM DDR2      "";
		
		//device module
		module=	emi_settings[i].DEVINFO_MCP;
		#ifdef DEVINFO_DEBUG_EMCP
    	printk("[DEVINFO_EMCP]: device type:%s!\n",type);
    	printk("[DEVINFO_EMCP]: device module:%s!\n",module);
		#endif

		//device vendor,eMMC PART
		//switch(emi_settings[i].ID[0])
		//For dist ddr,it is NULL in emi_setting.ID[n],we`d better get it from what we have got
		//switch((host->mmc->card->raw_cid[0]&0xFF000000)>>24)	
		switch(emi_settings[i].ID[0])
		{
			case 0x15:
				vendor=	"Samsung   ";
				break;
			case 0x90:
				vendor=	"Hynix     ";
				break;
			case 0x45:
				vendor=	"Sandisk   ";
				break;
			case 0x70:
				vendor=	"Kingston  ";
				break;
			defalut:
				vendor=	DEVINFO_NULL;
			break;
		}
	
		#ifdef DEVINFO_DEBUG_EMCP
    	printk("[DEVINFO_EMCP]: device vendor:%s!\n",vendor);
		#endif
		//device ic
		//same as module
		ic = module;

		#ifdef DEVINFO_DEBUG_EMCP
    	printk("[DEVINFO_EMCP]: device ic:%s!\n",ic);
		#endif
		//device version
		//NULL

		//device RAM size ,we can not get ROM size here
		ram_size=(unsigned long)((	emi_settings[i].DRAM_RANK_SIZE[0]/1024	+ emi_settings[i].DRAM_RANK_SIZE[1]/1024	+ emi_settings[i].DRAM_RANK_SIZE[2]/1024	+ emi_settings[i].DRAM_RANK_SIZE[3]/1024)/(1024)); 
		#ifdef DEVINFO_DEBUG_EMCP
    	printk("[DEVINFO_EMCP]: Device info:<%x> <%x><%x><%x><%d><%ld>\n",emi_settings[i].DRAM_RANK_SIZE[0],emi_settings[i].DRAM_RANK_SIZE[1],emi_settings[i].DRAM_RANK_SIZE[2],emi_settings[i].DRAM_RANK_SIZE[3],rom_size,ram_size);
		#endif

		if(emi_settings[i].type < 0x0100)//Add for dis DDR type
		{
			sprintf(info,"ram:%ldMB",ram_size);
			DEVINFO_DECLARE(type,module,"unknown",ic,version,info,DEVINFO_USED);	//we can not judge  used or not
			sprintf(info1,"rom:%dMB",rom_size);
			DEVINFO_DECLARE("eMMC","unknown",vendor,"unknown",version,info1,DEVINFO_USED);	//we can not judge  used or not
		}
		else{
		//Check if used on this board
		if((emi_settings[i].ID[0]==(host->mmc->card->raw_cid[0]&0xFF000000)>>24) && (emi_settings[i].ID[1]==(host->mmc->card->raw_cid[0]&0x00FF0000)>>16) && (emi_settings[i].ID[2]==(host->mmc->card->raw_cid[0]&0x0000FF00)>>8) && (emi_settings[i].ID[3]==(host->mmc->card->raw_cid[0]&0x000000FF)>>0)
				//shaohui add the code to enhance ability of detecting more devices,for same seriers products
			&&	(emi_settings[i].ID[4]==(host->mmc->card->raw_cid[1]&0xFF000000)>>24) && (emi_settings[i].ID[5]==(host->mmc->card->raw_cid[1]&0x00FF0000)>>16) && (emi_settings[i].ID[6]==(host->mmc->card->raw_cid[1]&0x0000FF00)>>8)
				)
		{
		//sprintf(info,"ram:%dMB",ram_size);
			switch(ram_size)
			{
				case 2048:
					sprintf(info,"ram:2048MB+rom:%dMB",rom_size);
					break;
				case 1536:
					sprintf(info,"ram:1536MB+rom:%dMB",rom_size);
					break;
				case 1024:
					sprintf(info,"ram:1024MB+rom:%dMB",rom_size);
					break;
				case 768:
					sprintf(info,"ram:768 MB+rom:%dMB",rom_size);
					break;
				case 512:
					sprintf(info,"ram:512 MB+rom:%dMB",rom_size);
					break;
				default:
					sprintf(info,"ram:512 MB+rom:%dMB",rom_size);
					break;
			}


			#ifdef DEVINFO_DEBUG_EMCP
    		printk("[DEVINFO_EMCP]: Get right device!\n");
    		printk("[DEVINFO_EMCP]: Device info:%s \n",info);
			#endif
 		//void devinfo_declare(char* type,char* module,char* vendor,char* ic,char* version,char* info,int used)
		DEVINFO_DECLARE(type,module,vendor,ic,version,info,DEVINFO_USED );	//used device regist
		}else{
			switch(ram_size)
			{
				case 2048:
					info1="ram:2048MB+rom:null";
					break;
				case 1536:
					info1="ram:1536MB+rom:null";
					break;
				case 1024:
					info1="ram:1024MB+rom:null";
					break;
				case 768:
					info1="ram:768 MB+rom:null";
					break;
				case 512:
					info1="ram:512 MB+rom:null";
					break;
				default:
					info1="ram:512 MB+rom:null";
					break;
			}
			#ifdef DEVINFO_DEBUG_EMCP
    		printk("[DEVINFO_EMCP]: Get wrong device!\n");
    		printk("[DEVINFO_EMCP]: Device info1:%s \n",info1);
			#endif
	//sprintf(info,"ram:%dMB",ram_size);
		DEVINFO_DECLARE(type,module,vendor,ic,version,info1,DEVINFO_UNUSED );	//unused device regist
		}
		setRamInfo(ram_size);
		}
	}	   

	return 0;
}


static u64 msdc_get_user_capacity(struct msdc_host *host)
{
    u64 device_capacity = 0;
    u32 device_legacy_capacity = 0;
    struct mmc_host* mmc = NULL;
    BUG_ON(!host);
    BUG_ON(!host->mmc);
    BUG_ON(!host->mmc->card);
    mmc = host->mmc;
    if(mmc_card_mmc(mmc->card)){
        if(mmc->card->csd.read_blkbits){
            device_legacy_capacity = mmc->card->csd.capacity * (2 << (mmc->card->csd.read_blkbits - 1));
        } else{
            device_legacy_capacity = mmc->card->csd.capacity;
        }
        device_capacity = (u64)(mmc->card->ext_csd.sectors)* 512 > device_legacy_capacity ? (u64)(mmc->card->ext_csd.sectors)* 512 : device_legacy_capacity;
    }
    else if(mmc_card_sd(mmc->card)){
        device_capacity = (u64)(mmc->card->csd.capacity) << (mmc->card->csd.read_blkbits);
    }
    return device_capacity;
}

///////////////////////////shaohui end/////////////////////////////
//static struct task_struct *devinfo_flash_thread_handle = NULL;
static void devinfo_flash_localinit()
{

    struct msdc_host *host_ctl;
	int num=0;
	
	//    devinfo_flash_thread_handle = kthread_create(devinfo_flash_regist, (void *) NULL, "devinfo_flash_thread");
//        wake_up_process(devinfo_flash_thread_handle);
	printk("[DEVINFO_EMCP]start to %s\n",__func__);

	//msleep(5000);		//wait for MSDC init done ,Do not use in init,this will block kernel
	msdc_check_init_done();

   
    host_ctl = mtk_msdc_host[0];  //define 0 to on board emcp

            rom_size = msdc_get_user_capacity(mtk_msdc_host[0])/1024/1024;
	devinfo_register_emcp(host_ctl);
	
#ifdef DEVINFO_DEBUG_EMCP
	printk("[DEVINFO_EMCP]OK now to get emmc info\n");
    printk("csd:0x%x,0x%x,0x%x,0x%x\n",host_ctl->mmc->card->raw_cid[0],
            host_ctl->mmc->card->raw_cid[1],
            host_ctl->mmc->card->raw_cid[2],
            host_ctl->mmc->card->raw_cid[3]);
#endif
}

static int __init devinfo_flash_init(void)
{

	devinfo_flash_localinit();
    return 0;
}

static void __exit devinfo_flash_exit(void)
{

}

module_init(devinfo_flash_init);
module_exit(devinfo_flash_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SLT DEVINFO flash memory info Management Driver");
MODULE_AUTHOR("shaohui <shaohui@longcheer.net>");

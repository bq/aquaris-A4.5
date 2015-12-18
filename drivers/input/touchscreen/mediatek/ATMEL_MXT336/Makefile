include $(srctree)/drivers/misc/mediatek/Makefile

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ATMEL_MXT336/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

#obj-y := atmel_mxt_ts.o plugin_misc.o

#frank modify
#obj-y := atmel_mxt_ts.o plugin_misc.o plugin_proci.o
obj-y := atmel_mxt_ts.o 

#obj-y := plugin_misc.o
#obj-y := atmel_mxt_ts_mtk_interface.o

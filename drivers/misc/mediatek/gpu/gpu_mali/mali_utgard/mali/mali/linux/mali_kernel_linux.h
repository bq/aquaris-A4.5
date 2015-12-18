/*
 * (c) ARM Limited 2008-2015
 */


#ifndef __MALI_KERNEL_LINUX_H__
#define __MALI_KERNEL_LINUX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/cdev.h>     /* character device definitions */
#include <linux/idr.h>
#include <linux/rbtree.h>
#include "mali_kernel_license.h"
#include "mali_osk_types.h"

extern struct platform_device *mali_platform_device;

#ifdef __cplusplus
}
#endif

#endif /* __MALI_KERNEL_LINUX_H__ */



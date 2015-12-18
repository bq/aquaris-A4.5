/*
 * (c) ARM Limited 2010-2015
 */


#include <linux/module.h>
#include <linux/mali/mali_utgard.h>
#include "mali_pm.h"

void mali_dev_pause(void)
{
	/*
	 * Deactive all groups to prevent hardware being touched
	 * during the period of mali device pausing
	 */
	mali_pm_os_suspend(MALI_FALSE);
}

EXPORT_SYMBOL(mali_dev_pause);

void mali_dev_resume(void)
{
	mali_pm_os_resume();
}

EXPORT_SYMBOL(mali_dev_resume);



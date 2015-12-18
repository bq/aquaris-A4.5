/*
 * (c) ARM Limited 2010, 2012-2015
 */


#include <linux/module.h>
#include "mali_executor.h"

int mali_perf_set_num_pp_cores(unsigned int num_cores)
{
	return mali_executor_set_perf_level(num_cores, MALI_FALSE);
}

EXPORT_SYMBOL(mali_perf_set_num_pp_cores);



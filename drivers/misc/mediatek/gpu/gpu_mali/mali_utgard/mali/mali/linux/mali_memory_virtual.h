/*
 * (c) ARM Limited 2013-2015
 */

#ifndef __MALI_GPU_VMEM_H__
#define __MALI_GPU_VMEM_H__

#include "mali_osk.h"
#include "mali_session.h"
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include "mali_memory_types.h"
#include "mali_memory_os_alloc.h"
#include "mali_memory_manager.h"



int mali_vma_offset_add(struct mali_allocation_manager *mgr,
			struct mali_vma_node *node);

void mali_vma_offset_remove(struct mali_allocation_manager *mgr,
			    struct mali_vma_node *node);

struct mali_vma_node *mali_vma_offset_search(struct mali_allocation_manager *mgr,
		unsigned long start,    unsigned long pages);

#endif



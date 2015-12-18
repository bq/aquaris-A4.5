/*
 * (c) ARM Limited 2013-2015
 */


#ifndef __MALI_MEMORY_UTIL_H__
#define __MALI_MEMORY_UTIL_H__

u32 mali_allocation_unref(struct mali_mem_allocation **alloc);

void mali_allocation_ref(struct mali_mem_allocation *alloc);

void mali_free_session_allocations(struct mali_session_data *session);

#endif



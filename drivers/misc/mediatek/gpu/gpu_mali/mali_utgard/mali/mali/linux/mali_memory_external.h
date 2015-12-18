/*
 * (c) ARM Limited 2011-2015
 */


#ifndef __MALI_MEMORY_EXTERNAL_H__
#define __MALI_MEMORY_EXTERNAL_H__

#ifdef __cplusplus
extern "C" {
#endif

_mali_osk_errcode_t mali_mem_bind_ext_buf(mali_mem_allocation *alloc,
		mali_mem_backend *mem_backend,
		u32 phys_addr,
		u32 flag);
void mali_mem_unbind_ext_buf(mali_mem_backend *mem_backend);

#ifdef __cplusplus
}
#endif

#endif



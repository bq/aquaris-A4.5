/*
 * (c) ARM Limited 2008-2010, 2012, 2014-2015
 */


#ifndef __ARCH_CONFIG_H__
#define __ARCH_CONFIG_H__

/* Use OS memory. */
#define ARCH_UMP_BACKEND_DEFAULT          1

/* OS memory won't need a base address. */
#define ARCH_UMP_MEMORY_ADDRESS_DEFAULT   0x00000000

/* 512 MB maximum limit for UMP allocations. */
#define ARCH_UMP_MEMORY_SIZE_DEFAULT 512UL * 1024UL * 1024UL


#endif /* __ARCH_CONFIG_H__ */



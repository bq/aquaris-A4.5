/*
 * (c) ARM Limited 2010-2012, 2014-2015
 */


#ifndef __MALI_CONTROL_TIMER_H__
#define __MALI_CONTROL_TIMER_H__

#include "mali_osk.h"

_mali_osk_errcode_t mali_control_timer_init(void);

void mali_control_timer_term(void);

mali_bool mali_control_timer_resume(u64 time_now);

void mali_control_timer_suspend(mali_bool suspend);
void mali_control_timer_pause(void);

void mali_control_timer_add(u32 timeout);

#endif /* __MALI_CONTROL_TIMER_H__ */




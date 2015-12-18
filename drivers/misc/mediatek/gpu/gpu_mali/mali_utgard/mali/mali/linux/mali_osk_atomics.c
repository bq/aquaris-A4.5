/*
 * (c) ARM Limited 2008-2010, 2013-2015
 */


#include "mali_osk.h"
#include <asm/atomic.h>
#include "mali_kernel_common.h"

void _mali_osk_atomic_dec(_mali_osk_atomic_t *atom)
{
	atomic_dec((atomic_t *)&atom->u.val);
}

u32 _mali_osk_atomic_dec_return(_mali_osk_atomic_t *atom)
{
	return atomic_dec_return((atomic_t *)&atom->u.val);
}

void _mali_osk_atomic_inc(_mali_osk_atomic_t *atom)
{
	atomic_inc((atomic_t *)&atom->u.val);
}

u32 _mali_osk_atomic_inc_return(_mali_osk_atomic_t *atom)
{
	return atomic_inc_return((atomic_t *)&atom->u.val);
}

void _mali_osk_atomic_init(_mali_osk_atomic_t *atom, u32 val)
{
	MALI_DEBUG_ASSERT_POINTER(atom);
	atomic_set((atomic_t *)&atom->u.val, val);
}

u32 _mali_osk_atomic_read(_mali_osk_atomic_t *atom)
{
	return atomic_read((atomic_t *)&atom->u.val);
}

void _mali_osk_atomic_term(_mali_osk_atomic_t *atom)
{
	MALI_IGNORE(atom);
}

u32 _mali_osk_atomic_xchg(_mali_osk_atomic_t *atom, u32 val)
{
	return atomic_xchg((atomic_t *)&atom->u.val, val);
}



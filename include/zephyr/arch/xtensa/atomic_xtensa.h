/**
 * Copyright (c) 2021 Intel Corporation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_ATOMIC_XTENSA_H_
#define ZEPHYR_INCLUDE_ATOMIC_XTENSA_H_

/* Included from <sys/atomic.h> */

/* Recent GCC versions actually do have working atomics support on
 * Xtensa (and so should work with CONFIG_ATOMIC_OPERATIONS_BUILTIN),
 * but existing versions of Xtensa's XCC do not.  So we define an
 * inline implementation here that is more or less identical
 */

static ALWAYS_INLINE atomic_val_t atomic_get(const atomic_t *target)
{
	atomic_val_t ret;

	/* Actual Xtensa hardware seems to have only in-order
	 * pipelines, but the architecture does define a barrier load,
	 * so use it.  There is a matching s32ri instruction, but
	 * nothing in the Zephyr API requires a barrier store (all the
	 * atomic write ops have exchange semantics.
	 */
	__asm__ volatile("l32ai %0, %1, 0"
			 : "=r"(ret) : "r"(target) : "memory");
	return ret;
}

static ALWAYS_INLINE
atomic_val_t xtensa_cas(atomic_t *addr, atomic_val_t oldval,
			atomic_val_t newval)
{
#if XCHAL_HAVE_S32C1I && XCHAL_HW_MIN_VERSION_MAJOR >= 2200
    __asm__ __volatile__ (
        "   wsr.scompare1 %2 \n"
        "   s32c1i %0, %3, 0 \n"
            : "=a"(newval) : "0" (newval), "a" (oldval), "a" (addr)
            : "memory");
#else
    __asm__ __volatile__ (
        "   l32i  %0, %3, 0 \n"         // %0 == value to test, return val
        "   bne   %2, %0, 9f \n"        // test
        "   s32i  %1, %3, 0 \n"         // write the new value
        "9: \n"
            : "=a"(newval) : "0" (newval), "a" (oldval), "a" (addr)
            : "memory");
#endif
    return newval;
}

static ALWAYS_INLINE
bool atomic_cas(atomic_t *target, atomic_val_t oldval, atomic_val_t newval)
{
	return oldval == xtensa_cas(target, oldval, newval);
}

static ALWAYS_INLINE
bool atomic_ptr_cas(atomic_ptr_t *target, void *oldval, void *newval)
{
	return (atomic_val_t) oldval
		== xtensa_cas((atomic_t *) target, (atomic_val_t) oldval,
			      (atomic_val_t) newval);
}

/* Generates an atomic exchange sequence that swaps the value at
 * address "target", whose old value is read to be "cur", with the
 * specified expression.  Evaluates to the old value which was
 * atomically replaced.
 */

#define Z__GEN_ATOMXCHG(expr) ({				\
	atomic_val_t res, cur;				\
	do {						\
		cur = *target;				\
		res = xtensa_cas(target, cur, (expr));	\
	} while (res != cur);				\
	res; })

static ALWAYS_INLINE
atomic_val_t atomic_set(atomic_t *target, atomic_val_t value)
{
	return Z__GEN_ATOMXCHG(value);
}

static ALWAYS_INLINE
atomic_val_t atomic_add(atomic_t *target, atomic_val_t value)
{
	return Z__GEN_ATOMXCHG(cur + value);
}

static ALWAYS_INLINE
atomic_val_t atomic_sub(atomic_t *target, atomic_val_t value)
{
	return Z__GEN_ATOMXCHG(cur - value);
}

static ALWAYS_INLINE
atomic_val_t atomic_inc(atomic_t *target)
{
	return Z__GEN_ATOMXCHG(cur + 1);
}

static ALWAYS_INLINE
atomic_val_t atomic_dec(atomic_t *target)
{
	return Z__GEN_ATOMXCHG(cur - 1);
}

static ALWAYS_INLINE atomic_val_t atomic_or(atomic_t *target,
					    atomic_val_t value)
{
	return Z__GEN_ATOMXCHG(cur | value);
}

static ALWAYS_INLINE atomic_val_t atomic_xor(atomic_t *target,
					     atomic_val_t value)
{
	return Z__GEN_ATOMXCHG(cur ^ value);
}

static ALWAYS_INLINE atomic_val_t atomic_and(atomic_t *target,
					     atomic_val_t value)
{
	return Z__GEN_ATOMXCHG(cur & value);
}

static ALWAYS_INLINE atomic_val_t atomic_nand(atomic_t *target,
					      atomic_val_t value)
{
	return Z__GEN_ATOMXCHG(~(cur & value));
}

static ALWAYS_INLINE void *atomic_ptr_get(const atomic_ptr_t *target)
{
	return (void *) atomic_get((atomic_t *)target);
}

static ALWAYS_INLINE void *atomic_ptr_set(atomic_ptr_t *target, void *value)
{
	return (void *) atomic_set((atomic_t *) target, (atomic_val_t) value);
}

static ALWAYS_INLINE atomic_val_t atomic_clear(atomic_t *target)
{
	return atomic_set(target, 0);
}

static ALWAYS_INLINE void *atomic_ptr_clear(atomic_ptr_t *target)
{
	return (void *) atomic_set((atomic_t *) target, 0);
}

#endif /* ZEPHYR_INCLUDE_ATOMIC_XTENSA_H_ */

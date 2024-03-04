/*
 * THIS FILE WAS AUTOMATICALLY GENERATED.  DO NOT EDIT.
 *
 * Functions here are designed to produce efficient code to
 * search an Xtensa bitmask of interrupts, inspecting only those bits
 * declared to be associated with a given interrupt level.  Each
 * dispatcher will handle exactly one flagged interrupt, in numerical
 * order (low bits first) and will return a mask of that bit that can
 * then be cleared by the calling code.  Unrecognized bits for the
 * level will invoke an error handler.
 */

#include <xtensa/config/core-isa.h>
#include <zephyr/sw_isr_table.h>


static inline int _xtensa_handle_one_int0(unsigned int mask)
{
    return 0;
}

static inline int _xtensa_handle_one_int1(unsigned int mask)
{
    int irq;
    int i = 0;

    mask &= XCHAL_INTLEVEL1_MASK;
    for (i = 0; i <= 31; i++)
        if (mask & BIT(i)) {
            mask = BIT(i);
            irq = i;
            goto handle_irq;
        }
    return 0;
    handle_irq:
    _sw_isr_table[irq].isr(_sw_isr_table[irq].arg);
    return mask;
}

static inline int _xtensa_handle_one_int2(unsigned int mask)
{
    int irq;
    int i = 0;

    mask &= XCHAL_INTLEVEL2_MASK;
    for (i = 0; i <= 31; i++)
        if (mask & BIT(i)) {
            mask = BIT(i);
            irq = i;
            goto handle_irq;
        }
    return 0;
    handle_irq:
    _sw_isr_table[irq].isr(_sw_isr_table[irq].arg);
    return mask;
}

static inline int _xtensa_handle_one_int3(unsigned int mask)
{
    int irq;
    int i = 0;

    mask &= XCHAL_INTLEVEL3_MASK;
    for (i = 0; i <= 31; i++)
        if (mask & BIT(i)) {
            mask = BIT(i);
            irq = i;
            goto handle_irq;
        }
    return 0;
    handle_irq:
    _sw_isr_table[irq].isr(_sw_isr_table[irq].arg);
    return mask;
}

static inline int _xtensa_handle_one_int4(unsigned int mask)
{
    int irq;
    int i = 0;

    mask &= XCHAL_INTLEVEL4_MASK;
    for (i = 0; i <= 31; i++)
        if (mask & BIT(i)) {
            mask = BIT(i);
            irq = i;
            goto handle_irq;
        }
    return 0;
    handle_irq:
    _sw_isr_table[irq].isr(_sw_isr_table[irq].arg);
    return mask;
}

static inline int _xtensa_handle_one_int5(unsigned int mask)
{
    int irq;
    int i = 0;

    mask &= XCHAL_INTLEVEL5_MASK;
    for (i = 0; i <= 31; i++)
        if (mask & BIT(i)) {
            mask = BIT(i);
            irq = i;
            goto handle_irq;
        }
    return 0;
    handle_irq:
    _sw_isr_table[irq].isr(_sw_isr_table[irq].arg);
    return mask;
}

static inline int _xtensa_handle_one_int6(unsigned int mask)
{
    int irq;
    int i = 0;

    mask &= XCHAL_INTLEVEL6_MASK;
    for (i = 0; i <= 31; i++)
        if (mask & BIT(i)) {
            mask = BIT(i);
            irq = i;
            goto handle_irq;
        }
    return 0;
    handle_irq:
    _sw_isr_table[irq].isr(_sw_isr_table[irq].arg);
    return mask;
}

static inline int _xtensa_handle_one_int7(unsigned int mask)
{
    int irq;
    int i = 0;

    mask &= XCHAL_INTLEVEL7_MASK;
    for (i = 0; i <= 31; i++)
        if (mask & BIT(i)) {
            mask = BIT(i);
            irq = i;
            goto handle_irq;
        }
    return 0;
    handle_irq:
    _sw_isr_table[irq].isr(_sw_isr_table[irq].arg);
    return mask;
}

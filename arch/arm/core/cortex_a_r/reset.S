/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 * Copyright (c) 2019 Stephanos Ioannidis <root@stephanos.io>
 * Copyright 2024 Arm Limited and/or its affiliates <open-source-office@arm.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Reset handler
 *
 * Reset handler that prepares the system for running C code.
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/arch/cpu.h>
#include <offsets_short.h>
#include <cortex_a_r/tcm.h>
#include "vector_table.h"
#include "boot.h"
#include "macro_priv.inc"

_ASM_FILE_PROLOGUE

GTEXT(z_arm_reset)
GDATA(z_interrupt_stacks)
GDATA(z_arm_svc_stack)
GDATA(z_arm_sys_stack)
GDATA(z_arm_fiq_stack)
GDATA(z_arm_abort_stack)
GDATA(z_arm_undef_stack)
#if defined(CONFIG_SOC_RESET_HOOK)
GTEXT(soc_reset_hook)
#endif

/**
 *
 * @brief Reset vector
 *
 * Ran when the system comes out of reset. The processor is in Supervisor mode
 * and interrupts are disabled. The processor architectural registers are in
 * an indeterminate state.
 *
 * When these steps are completed, jump to z_prep_c(), which will finish
 * setting up the system for running C code.
 *
 */
#if defined(CONFIG_AARCH32_ARMV8_R)
/*
 * ARMv8-R AArch32 cores use RVBAR (Reset Vector Base Address Register) to
 * determine the CPU start address on reset.  RVBAR only stores bits [31:5];
 * bits [4:0] are RES0.  This means the reset
 * vector address is implicitly rounded down to a 32-byte boundary.
 *
 * If __start is not 32-byte aligned, any firmware or boot-loader that
 * programs RVBAR from the ELF entry point will silently truncate the
 * address, causing the CPU to begin executing at the wrong location.
 */
	.section .text._reset_section, "ax"
	.align 5
z_arm_reset:
__start:
#else
SECTION_SUBSEC_FUNC(TEXT, _reset_section, z_arm_reset)
SECTION_SUBSEC_FUNC(TEXT, _reset_section, __start)
#endif
#if defined(CONFIG_AARCH32_ARMV8_R)
    /* Check if we are starting in HYP mode */
    mrs r0, cpsr
    and r0, r0, #MODE_MASK
    cmp r0, #MODE_HYP
    bne EL1_Reset_Handler

    /*
     * The HSCTLR register provides top-level control of system operation in Hyp mode.
     * Since the OS is not running in Hyp mode, and considering the Armv8-R AArch32
     * architecture profile, there's no need to modify HSCTLR configuration unless
     * Fast Interrupts need to be enabled.
     */

    /* Init HACTLR: Enable EL1 access to all IMP DEF registers */
    ldr r0, =HACTLR_INIT
    mcr p15, 4, r0, c1, c0, 1

   /* Go to SVC mode */
    mrs r0, cpsr
    bic r0, #MODE_MASK
    orr r0, #MODE_SVC
    msr spsr_cxsf, r0

    ldr r0, =EL1_Reset_Handler
    msr elr_hyp, r0
    dsb
    isb
    eret

EL1_Reset_Handler:

#endif

    /*
     * ---------------------------------------------------------------
     * Early banked-SP safe initialisation (ALL cores, runs immediately
     * after EL1_Reset_Handler regardless of primary/secondary split).
     *
     * When a secondary core (Core1) is kicked from TF-A's mailbox it
     * arrives here in secure SVC mode.  TF-A may leave SP_UND, SP_ABT,
     * SP_SYS, SP_IRQ and SP_FIQ with stale values from a previous boot
     * cycle (preserved in PSRAM across warm resets).  Any exception that
     * fires before the per-CPU stacks are loaded from boot_params will
     * use those stale pointers, corrupt DRAM and double-fault.
     *
     * Fix: point all exception-mode SPs to the cpu-0 stack tops as safe
     * temporaries the instant we enter the kernel, before any instruction
     * that could raise an exception (DRAM load, cache op, VFP, etc.).
     * Primary core overrides them unconditionally in _primary_core;
     * secondary core overrides them again in _secondary_core once
     * boot_params are ready.
     *
     * Use cpu-1 stack tops here (second slot = base + 2*SIZE) so that
     * secondary core does not collide with cpu-0 which also uses
     * base + 1*SIZE.  Primary core always overwrites these in
     * _primary_core with the correct cpu-0 values before any C code runs.
     *
     * FPU access is also enabled here so that the UND handler
     * (CONFIG_FPU_SHARING) can execute vmrs/vmsr without re-triggering
     * UND during this early window.
     * ---------------------------------------------------------------
     */
    mov r0, #0

    msr CPSR_c, #(MODE_FIQ | I_BIT | F_BIT)
    ldr sp, =(z_arm_fiq_stack + 2 * CONFIG_ARMV7_FIQ_STACK_SIZE)
    mov lr, r0
    msr spsr_cxsf, r0

    msr CPSR_c, #(MODE_IRQ | I_BIT | F_BIT)
    ldr sp, =(z_interrupt_stacks + 2 * CONFIG_ISR_STACK_SIZE)
    mov lr, r0
    msr spsr_cxsf, r0

    msr CPSR_c, #(MODE_ABT | I_BIT | F_BIT)
    ldr sp, =(z_arm_abort_stack + 2 * CONFIG_ARMV7_EXCEPTION_STACK_SIZE)
    mov lr, r0
    msr spsr_cxsf, r0

    msr CPSR_c, #(MODE_UND | I_BIT | F_BIT)
    ldr sp, =(z_arm_undef_stack + 2 * CONFIG_ARMV7_EXCEPTION_STACK_SIZE)
    mov lr, r0
    msr spsr_cxsf, r0

    msr CPSR_c, #(MODE_SYS | I_BIT | F_BIT)
    ldr sp, =(z_arm_sys_stack + 2 * CONFIG_ARMV7_SYS_STACK_SIZE)
    mov lr, r0

    /* Return to SVC mode (same as TF-A delivered us) with I+F masked */
    msr CPSR_c, #(MODE_SVC | I_BIT | F_BIT)
    mov lr, r0
    msr spsr_cxsf, r0

#if defined(CONFIG_FPU) && defined(CONFIG_CPU_HAS_VFP)
    mrc p15, 0, r0, c1, c0, 2
    orr r0, r0, #(CPACR_CP10(CPACR_FA) | CPACR_CP11(CPACR_FA))
    mcr p15, 0, r0, c1, c0, 2
    isb
    mov r0, #FPEXC_EN
    vmsr fpexc, r0
#endif

#if defined(CONFIG_DCLS)
    /*
     * Initialise CPU registers to a defined state if the processor is
     * configured as Dual-redundant Core Lock-step (DCLS). This is required
     * for state convergence of the two parallel executing cores.
     */

    /* Common and SVC mode registers */
    mov r0,  #0
    mov r1,  #0
    mov r2,  #0
    mov r3,  #0
    mov r4,  #0
    mov r5,  #0
    mov r6,  #0
    mov r7,  #0
    mov r8,  #0
    mov r9,  #0
    mov r10, #0
    mov r11, #0
    mov r12, #0
    mov r13, #0         /* r13_svc */
    mov r14, #0         /* r14_svc */
    mrs r0,  cpsr
    msr spsr_cxsf, r0   /* spsr_svc */

    /* FIQ mode registers */
    cps #MODE_FIQ
    mov r8,  #0         /* r8_fiq */
    mov r9,  #0         /* r9_fiq */
    mov r10, #0         /* r10_fiq */
    mov r11, #0         /* r11_fiq */
    mov r12, #0         /* r12_fiq */
    mov r13, #0         /* r13_fiq */
    mov r14, #0         /* r14_fiq */
    mrs r0,  cpsr
    msr spsr_cxsf, r0   /* spsr_fiq */

    /* IRQ mode registers */
    cps #MODE_IRQ
    mov r13, #0         /* r13_irq */
    mov r14, #0         /* r14_irq */
    mrs r0,  cpsr
    msr spsr_cxsf, r0   /* spsr_irq */

    /* ABT mode registers */
    cps #MODE_ABT
    mov r13, #0         /* r13_abt */
    mov r14, #0         /* r14_abt */
    mrs r0,  cpsr
    msr spsr_cxsf, r0   /* spsr_abt */

    /* UND mode registers */
    cps #MODE_UND
    mov r13, #0         /* r13_und */
    mov r14, #0         /* r14_und */
    mrs r0,  cpsr
    msr spsr_cxsf, r0   /* spsr_und */

    /* SYS mode registers */
    cps #MODE_SYS
    mov r13, #0         /* r13_sys */
    mov r14, #0         /* r14_sys */

#if defined(CONFIG_FPU) && defined(CONFIG_CPU_HAS_VFP)
    /*
     * Initialise FPU registers to a defined state.
     */

    /* Allow VFP coprocessor access */
    mrc p15, 0, r0, c1, c0, 2
    orr r0, r0, #(CPACR_CP10(CPACR_FA) | CPACR_CP11(CPACR_FA))
    mcr p15, 0, r0, c1, c0, 2

    /* Enable VFP */
    mov  r0, #FPEXC_EN
    vmsr fpexc, r0

    /* Initialise VFP registers */
    fmdrr d0,  r1, r1
    fmdrr d1,  r1, r1
    fmdrr d2,  r1, r1
    fmdrr d3,  r1, r1
    fmdrr d4,  r1, r1
    fmdrr d5,  r1, r1
    fmdrr d6,  r1, r1
    fmdrr d7,  r1, r1
    fmdrr d8,  r1, r1
    fmdrr d9,  r1, r1
    fmdrr d10, r1, r1
    fmdrr d11, r1, r1
    fmdrr d12, r1, r1
    fmdrr d13, r1, r1
    fmdrr d14, r1, r1
    fmdrr d15, r1, r1
#if defined(CONFIG_VFP_FEATURE_REGS_S64_D32)
    fmdrr d16, r1, r1
    fmdrr d17, r1, r1
    fmdrr d18, r1, r1
    fmdrr d19, r1, r1
    fmdrr d20, r1, r1
    fmdrr d21, r1, r1
    fmdrr d22, r1, r1
    fmdrr d23, r1, r1
    fmdrr d24, r1, r1
    fmdrr d25, r1, r1
    fmdrr d26, r1, r1
    fmdrr d27, r1, r1
    fmdrr d28, r1, r1
    fmdrr d29, r1, r1
    fmdrr d30, r1, r1
    fmdrr d31, r1, r1
#endif /* CONFIG_VFP_FEATURE_REGS_S64_D32 */

    vmsr fpscr, r1
    vmsr fpexc, r1
#endif /* CONFIG_FPU && CONFIG_CPU_HAS_VFP */

#endif /* CONFIG_DCLS */

#if defined(CONFIG_CPU_CORTEX_R52_CACHE_SEGREGATION)
    ldr r0, =IMP_CSCTLR(CONFIG_CPU_CORTEX_R52_ICACHE_FLASH_WAY,
                        CONFIG_CPU_CORTEX_R52_DCACHE_FLASH_WAY)
    mcr p15, 1, r0, c9, c1, 0
#endif

    /*
     * Invalidate I-cache for all cores (primary and secondary) before the
     * voting and literal-pool reads that follow.
     * After a warm reset SCTLR.I==1 (I-cache enabled, inherited from the
     * previous run).  All banked SP/SPSR have been set to safe values above,
     * so any prefetch/cache exception here is handled correctly.
     * ICIALLU is a NOP when SCTLR.I==0, so it is always safe to execute.
     */
    mov r0, #0
    mcr p15, 0, r0, c7, c5, 0    /* ICIALLU: invalidate entire I-cache */
    dsb
    isb

    ldr r0, =arm_cpu_boot_params

#if CONFIG_MP_MAX_NUM_CPUS > 1
    /*
     * Determine primary/secondary CPU by MPID.
     *
     * On RTL8730E the CPU topology is fixed: Core0 (MPID==0) is always
     * primary; Core1 (MPID==1) is always secondary, started via PSCI
     * cpu_on by Core0.  The voting algorithm used in the generic code
     * reads boot_params.mpid as a "claimed" sentinel, but after a warm
     * reset PSRAM retains the previous run's Core1 MPID, causing Core0
     * to conclude that another core already claimed primary and go
     * secondary — leaving no primary and deadlocking the system.
     *
     * Fix: replace the voting algorithm with a direct MPID check.
     * MPIDR[23:0] == 0 → primary core (Core0).
     * MPIDR[23:0] != 0 → secondary core (Core1 via PSCI cpu_on).
     *
     * For the PSCI cpu_on path Core0 has already set boot_params.mpid
     * to Core1's MPID before releasing Core1, so the spin-loop in
     * _secondary_core_spin still works correctly — we only skip the
     * unreliable voting part.
     */
    get_cpu_logic_id r1, r2, r3, r4 // r1: MPID, r2: logic id

    cmp r1, #0                              /* MPID == 0 → primary */
    bne _secondary_core                     /* MPID != 0 → secondary */

    /* Primary core (Core0): jump directly to _primary_core. */
    b _primary_core

    /* Invalidate the I-cache and D-cache before spinning on boot_params.
     * The primary core may have written code and data (including boot_params)
     * after secondary reset; stale I-cache lines would cause us to execute
     * wrong instructions (e.g. ldr literal-pool yields garbage function
     * pointer), and stale D-cache lines would make us read the wrong stack
     * pointers from boot_params even after the primary core flushed its cache.
     */
    mcr p15, 0, r0, c7, c5, 0    /* ICIALLU: invalidate entire I-cache */
    dsb
    isb

    /* loop until our turn comes:
     * Invalidate the boot_params D-cache line on every iteration so we always
     * see the value written by the primary core.
     *
     * NOTE: _secondary_core is also the target of the fast-path branch above
     * (b _secondary_core when voting detects another core already won).
     * All secondary-specific initialisation therefore lives HERE, inside the
     * label, so every secondary code path receives it regardless of which
     * voting branch was taken.
     */
_secondary_core:
    /*
     * Secondary core arrives here from TF-A in SVC mode with only SP_SVC
     * initialised by TF-A.  All other banked SPs (FIQ, IRQ, ABT, UND, SYS)
     * are uninitialised reset values.  If any exception fires during the
     * boot_params spin-loop below (e.g. FPU trap), the handler will use
     * those garbage SPs, corrupt memory, and double-fault.
     *
     * Use cpu-1 stack tops (second slot of each per-CPU array) as safe
     * temporaries.  This avoids colliding with cpu-0 which uses the same
     * stack arrays but at [cpu0_top = base + SIZE].  cpu-1 top is at
     * [base + 2*SIZE].  They will be replaced by the exact boot_params
     * values at label 2: below.
     *
     * Keep I+F masked so real IRQ/FIQ cannot nest here.
     */
    mov r0, #0

    msr CPSR_c, #(MODE_FIQ | I_BIT | F_BIT)
    ldr sp, =(z_arm_fiq_stack + 2 * CONFIG_ARMV7_FIQ_STACK_SIZE)
    mov lr, r0
    msr spsr_cxsf, r0

    msr CPSR_c, #(MODE_IRQ | I_BIT | F_BIT)
    ldr sp, =(z_interrupt_stacks + 2 * CONFIG_ISR_STACK_SIZE)
    mov lr, r0
    msr spsr_cxsf, r0

    msr CPSR_c, #(MODE_ABT | I_BIT | F_BIT)
    ldr sp, =(z_arm_abort_stack + 2 * CONFIG_ARMV7_EXCEPTION_STACK_SIZE)
    mov lr, r0
    msr spsr_cxsf, r0

    msr CPSR_c, #(MODE_UND | I_BIT | F_BIT)
    ldr sp, =(z_arm_undef_stack + 2 * CONFIG_ARMV7_EXCEPTION_STACK_SIZE)
    mov lr, r0
    msr spsr_cxsf, r0

    msr CPSR_c, #(MODE_SYS | I_BIT | F_BIT)
    ldr sp, =(z_arm_sys_stack + 2 * CONFIG_ARMV7_SYS_STACK_SIZE)
    mov lr, r0

    /* Return to SVC mode with I+F masked */
    msr CPSR_c, #(MODE_SVC | I_BIT | F_BIT)
    mov lr, r0
    msr spsr_cxsf, r0

#if defined(CONFIG_FPU) && defined(CONFIG_CPU_HAS_VFP)
    /*
     * Enable FPU (CP10/CP11) access in CPACR before the spin-loop so that
     * any VFP instruction inside an exception handler (CONFIG_FPU_SHARING
     * makes the UND handler execute vmrs/vmsr/vstmia) does NOT re-raise
     * Undefined Instruction → double-fault.
     */
    mrc p15, 0, r0, c1, c0, 2
    orr r0, r0, #(CPACR_CP10(CPACR_FA) | CPACR_CP11(CPACR_FA))
    mcr p15, 0, r0, c1, c0, 2
    isb
    mov r0, #FPEXC_EN
    vmsr fpexc, r0
    /* restore r0 = arm_cpu_boot_params (clobbered above) */
    ldr r0, =arm_cpu_boot_params
#endif

    /*
     * Pre-initialise TPIDRURO to point at _kernel.cpus[1] so that
     * get_cpu() inside any exception handler that fires during the
     * spin-loop below returns a valid (non-NULL) cpu pointer instead
     * of 0.  A NULL cpu pointer causes get_cpu to compute address 0
     * for the nested-count field, which is a null-pointer dereference
     * → Data Abort → exception cascade / double-fault.
     *
     * _kernel.cpus is at _kernel+___kernel_t_cpus_OFFSET (= 0).
     * cpu-1 slot is at _kernel + sizeof(struct _cpu) * 1.
     * We use the symbol _kernel and a compile-time constant offset
     * (___cpu_t_arch_OFFSET + sizeof is not directly available here,
     * so we derive it from the linker symbol _kernel and the known
     * per-CPU struct stride, which equals CONFIG_MP_MAX_NUM_CPUS-1
     * relative offset between kernel+0 and the first cpu).
     * Simpler: use the extern symbol directly via a per-cpu array.
     *
     * Architecture TPIDRURO encoding: bits [31:2] = &_cpu, bits[1:0]=0.
     */
    ldr r3, =_kernel                 /* r3 = &_kernel = &_kernel.cpus[0] */
    /* stride = (_kernel size 0x68) / CONFIG_MP_MAX_NUM_CPUS; for 2 CPUs = 0x34 */
    add r3, r3, #0x34                /* r3 = &_kernel.cpus[1] */
    mcr p15, 0, r3, c13, c0, 3      /* TPIDRURO = &_kernel.cpus[1] */

_secondary_core_spin:
    ldr r2, =arm_cpu_boot_params
    mcr p15, 0, r2, c7, c6, 1   /* DCIMVAC: invalidate boot_params cache line */
    add r2, r2, #64
    mcr p15, 0, r2, c7, c6, 1   /* invalidate second line if struct spans two */
    dsb
    isb
    ldr r2, [r0, #BOOT_PARAM_MPID_OFFSET]
    cmp r1, r2
    bne _secondary_core_spin
    /* boot_params is fresh in memory (just invalidated); load stack pointers */
    ldr r4, =arch_secondary_cpu_init
    ldr r5, [r0, #BOOT_PARAM_FIQ_SP_OFFSET]
    ldr r6, [r0, #BOOT_PARAM_IRQ_SP_OFFSET]
    ldr r7, [r0, #BOOT_PARAM_ABT_SP_OFFSET]
    ldr r8, [r0, #BOOT_PARAM_UDF_SP_OFFSET]
    ldr r9, [r0, #BOOT_PARAM_SVC_SP_OFFSET]
    ldr r10, [r0, #BOOT_PARAM_SYS_SP_OFFSET]
    b 2f

_primary_core:
#endif

    ldr r4, =z_prep_c
    ldr r5, =(z_arm_fiq_stack + CONFIG_ARMV7_FIQ_STACK_SIZE)
    ldr r6, =(z_interrupt_stacks + CONFIG_ISR_STACK_SIZE)
    ldr r7, =(z_arm_abort_stack + CONFIG_ARMV7_EXCEPTION_STACK_SIZE)
    ldr r8, =(z_arm_undef_stack + CONFIG_ARMV7_EXCEPTION_STACK_SIZE)
    ldr r9, =(z_arm_svc_stack + CONFIG_ARMV7_SVC_STACK_SIZE)
    ldr r10, =(z_arm_sys_stack + CONFIG_ARMV7_SYS_STACK_SIZE)

2:
    /*
     * Configure stack.
     */

    /* FIQ mode stack */
    msr CPSR_c, #(MODE_FIQ | I_BIT | F_BIT)
    mov sp, r5

    /* IRQ mode stack */
    msr CPSR_c, #(MODE_IRQ | I_BIT | F_BIT)
    mov sp, r6

    /* ABT mode stack */
    msr CPSR_c, #(MODE_ABT | I_BIT | F_BIT)
    mov sp, r7

    /* UND mode stack */
    msr CPSR_c, #(MODE_UND | I_BIT | F_BIT)
    mov sp, r8

    /* SVC mode stack */
    msr CPSR_c, #(MODE_SVC | I_BIT | F_BIT)
    mov sp, r9

    /* SYS mode stack */
    msr CPSR_c, #(MODE_SYS | I_BIT | F_BIT)
    mov sp, r10

#if defined(CONFIG_SOC_RESET_HOOK)
    /* Execute platform-specific initialisation if applicable */
    bl soc_reset_hook
#endif

#if defined(CONFIG_WDOG_INIT)
    /* board-specific watchdog initialization is necessary */
    bl z_arm_watchdog_init
#endif

#if defined(CONFIG_DISABLE_TCM_ECC)
    bl z_arm_tcm_disable_ecc
#endif

    bl z_arm_relocate_vector_table

    bx r4

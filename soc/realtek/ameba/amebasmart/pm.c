/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * AmebaSmart (RTL8730E) AP-core (CA32) power management for Zephyr.
 *
 * For clock-gating (SUSPEND_TO_IDLE) the core hands the sleep parameters to the
 * LP (KM0) core over IPC and gates its own clock; the LP core re-enables the
 * clock on a configured wake event.
 *
 * For power-gating (SUSPEND_TO_RAM) the CA32 hands off to the LP core over IPC
 * and enters SOCPS_SleepPG_LIB (lib_pmc.a): it saves the CP15/banked context
 * and GIC state, issues a PSCI CPU_SUSPEND SMC into TF-A which power-gates the
 * core, and on a wake event warm-boots back (restoring CP15/GIC) to where it
 * slept.  This file is the Zephyr glue around that: PM hooks, sleep_param
 * setup, wake-source enable, arch-timer re-base and SMP coordination.
 */

#include <ameba_soc.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/sys/util.h>
#ifdef CONFIG_SMP
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/pm_cpu_ops.h>
#include <zephyr/platform/hooks.h>
#endif

LOG_MODULE_REGISTER(soc_pm, LOG_LEVEL_DBG);

/* --- Power-gating (SUSPEND_TO_RAM) ------------------------------------- */

/* sleep parameters (ameba_pmc.c) + PG sleep entry (lib_pmc.a): saves CP15/GIC,
 * IPCs the LP core, issues the PSCI suspend and warm-boots back on wake.
 */
extern SLEEP_ParamDef sleep_param;
extern void SOCPS_SleepPG_LIB(void);

/* arm_arch_timer.c: re-base the system-tick accounting after power-gating has
 * reset the CA32 generic-timer counter, announcing the elapsed sleep ticks.
 */
extern void sys_clock_arm_arch_timer_pm_resync(uint32_t elapsed_ticks);

#ifdef CONFIG_SMP
/* Secondary-core register save/restore across cluster PG (lib_pmc.a): a
 * setjmp/longjmp-style backup of r4-r11+lr + CP15 + banked regs into cpu1_ctx.
 */
extern void SOCPS_Backup_CPU1(void);
extern void SOCPS_Restore_CPU1(void);
extern void __start(void); /* secondary cold-boot entry (arch reset vector) */

/* Secondary-core (CPU1) power-gating state machine. The CPU1_RUNNING /
 * CPU1_HOTPLUG / CPU1_WAKE_FROM_PG enumerators come from the HAL
 * (ameba_pmu.h) and match the FreeRTOS SDK values (0/1/2).
 */
volatile int amebasmart_cpu1_state = CPU1_RUNNING;

/*
 * SMP cluster power-gating (CPU1 hotplug + full-cluster PG).
 *
 *   0 = SMP SUSPEND_TO_RAM falls back to clock-gating (CPU1 parks in WFE).
 *   1 = real cluster PG: CPU1 powers itself off (soc_cpu1_pg) and CPU0
 *       cluster-power-gates, restarting CPU1 on warm boot. Verified working
 *       over many (40+) consecutive cluster-PG cycles.
 */
#define AMEBA_SMP_CLUSTER_PG 1
#endif /* CONFIG_SMP */

#ifdef CONFIG_SMP
/* --- Secondary core (CPU1) coordination for SMP sleep ------------------- *
 *
 * Zephyr calls pm_state_set() on every idle CPU, so under SMP both CA32 cores
 * reach it.  Only CPU0 drives the SoC sleep; CPU1 parks itself in WFE (clock
 * still fed) and CPU0 releases it with SEV on wake — the clock-gating (CG)
 * model, which needs no core power-off (mirrors the FreeRTOS ARM_CA32 SMP
 * idle hook, vPortSMPSuppressTicksAndSleep, CG path).
 */
static volatile uint8_t cpu1_parked;
static volatile uint8_t cpu1_release;

static bool soc_cpu1_in_wfe(void)
{
	CA32_TypeDef *ca32 = CA32_BASE;

	return (cpu1_parked != 0U) && ((ca32->CA32_C0_CPU_STATUS & CA32_STANDBYWFE_CORE1) != 0U);
}

static void soc_cpu1_park(void)
{
	unsigned int key = arch_irq_lock();

	cpu1_release = 0;
	cpu1_parked = 1;
	__asm__ volatile("dsb");

	do {
		__asm__ volatile("wfe");
	} while (cpu1_release == 0U);

	cpu1_parked = 0;
	__asm__ volatile("dsb");

	arch_irq_unlock(key);
}

static void soc_cpu1_release(void)
{
	cpu1_release = 1;
	__asm__ volatile("dsb");
	__asm__ volatile("sev");
}

/* MPID of the secondary core (devicetree cpu@1 reg = 1). */
#define CPU1_MPID 1

/*
 * Secondary-core power-gating (CPU hotplug), mirroring the FreeRTOS SDK
 * vSMPSleepProcessing.  Zephyr AArch32 has no native CPU hotplug, so we use a
 * setjmp/longjmp trick: SOCPS_Backup_CPU1() records a resume return point, then
 * the core powers itself off (PSCI CPU_OFF).  After the cluster warm-boots,
 * CPU0 re-starts this core (PSCI CPU_ON -> __start); its cold-boot path re-inits
 * GIC/MMU/SGI and the soc_per_core_init_hook() calls back to
 * amebasmart_cpu1_pg_resume_if_needed(), which longjmps (SOCPS_Restore_CPU1)
 * back to the return point below with the original stack/thread context intact.
 */
static void soc_cpu1_pg(void)
{
	/* setjmp: save callee-saved + CP15/banked, record the resume point. */
	SOCPS_Backup_CPU1();

	if (amebasmart_cpu1_state == CPU1_RUNNING) {
		amebasmart_cpu1_state = CPU1_HOTPLUG;

		/* Clean+invalidate the whole D-cache: cluster caches are lost. */
		DCache_CleanInvalidate(0xffffffff, 0xffffffff);

		/* Power this core off; CPU0 then cluster-PGs. Must not return. */
		pm_cpu_off();
		while (1) {
			__asm__ volatile("wfi");
		}
	}

	/* Reached only via the longjmp resume path (state == WAKE_FROM_PG). */
	amebasmart_cpu1_state = CPU1_RUNNING;
}

/*
 * Called from soc_per_core_init_hook() on the secondary cold-boot path.  If the
 * core is coming back from cluster PG, longjmp back into soc_cpu1_pg() (does not
 * return); otherwise this is a normal secondary bring-up and we fall through.
 */
bool amebasmart_cpu1_pg_resume_if_needed(void)
{
	if (amebasmart_cpu1_state != CPU1_WAKE_FROM_PG) {
		return false;
	}

	SOCPS_Restore_CPU1();

	CODE_UNREACHABLE; /* SOCPS_Restore_CPU1 longjmps, never returns */
	return true;
}
#endif /* CONFIG_SMP */

/* --- Clock-gating (SUSPEND_TO_IDLE) ------------------------------------ */

static void soc_sleep_cg(void)
{
#ifdef CONFIG_SMP
	/* Do not gate the clock until CPU1 is parked in WFE; if it has not
	 * parked yet, abandon this attempt — the idle loop retries next tick.
	 */
	if (!soc_cpu1_in_wfe()) {
		return;
	}
#endif

	SOCPS_SleepCG();

#ifdef CONFIG_SMP
	soc_cpu1_release();
#endif
}

static void soc_sleep_pg(void)
{
#ifdef CONFIG_SMP
	/* Cluster PG powers off BOTH cores, so it is only safe once CPU1 has
	 * powered itself off (soc_cpu1_pg -> CPU1_HOTPLUG). If CPU1 is still
	 * running (it may have picked CG this cycle, or not yet reached idle),
	 * do a CG sleep instead; the idle loop retries PG once CPU1 is also
	 * idle-deep and hotplugged off.
	 */
	if (amebasmart_cpu1_state != CPU1_HOTPLUG) {
		soc_sleep_cg();
		return;
	}
#endif

	sleep_param.sleep_type = SLEEP_PG;
	/* PMU_SLEEP_FOREVER: the LP core must NOT arm an AON wake timer — the AP
	 * sleeps until a configured wake event (rtc/counter/gpio) fires. sleep_time=0
	 * makes the LP wake the AP almost immediately (ap_wakeup_timer_init(0)),
	 * producing an enter/exit spin instead of a real sleep.
	 */
	sleep_param.sleep_time = PMU_SLEEP_FOREVER;
	sleep_param.dlps_enable = DISABLE;
	DCache_CleanInvalidate((u32)&sleep_param, sizeof(SLEEP_ParamDef));

	RTK_LOGS(NOTAG, RTK_LOG_INFO, "APPG\n");

	/* Save CP15/GIC context, IPC the LP core, issue the PSCI suspend;
	 * execution resumes here (warm boot, context/GIC restored) after the wake
	 * event powers the CA32 back on.
	 */
	SOCPS_SleepPG_LIB();

	RTK_LOGS(NOTAG, RTK_LOG_INFO, "APPW\n");

	/*
	 * Power-gating resets the CA32 ARM generic timer (the Zephyr system tick
	 * counter) back to ~0.  Re-base the arch-timer driver's cycle/tick
	 * accounting onto the reset counter so the tickless kernel keeps getting
	 * ticks and its clock neither hangs nor jumps (the driver's last_cycle
	 * would otherwise underflow against the reset counter).
	 *
	 * elapsed_ticks is 0: the powered-down duration is NOT added to kernel
	 * time.  Every CA32-side time source is unusable across PG — the arch
	 * timer and TIM0/SYSTIMER/debug timer are all in AP/LS domains that gate
	 * during PG, and the RTC calendar does not free-run in this path.  The
	 * only component that knows the true PG duration is KM0 (the PM master);
	 * accounting for slept time would require it to report the duration back
	 * over IPC on wake.  Kernel time therefore stays monotonic and stable
	 * across PG but does not advance during the gated interval (matching the
	 * FreeRTOS CA32 port, whose PG tick compensation is likewise a TODO).
	 */
	sys_clock_arm_arch_timer_pm_resync(0);

#ifdef CONFIG_SMP
	/* Cluster is back and CPU0 has restored its own state. Re-start CPU1
	 * via PSCI CPU_ON to the arch reset entry; its cold-boot path restores
	 * the saved context (amebasmart_cpu1_pg_resume_if_needed) and resumes
	 * CPU1's idle thread. Mark WAKE_FROM_PG first so the restore hook fires.
	 */
	amebasmart_cpu1_state = CPU1_WAKE_FROM_PG;
	DCache_CleanInvalidate(0xffffffff, 0xffffffff);

	/* RTL8730E's PSCI CPU_ON only releases Core1 from its ATF WFE hold-loop;
	 * it does NOT re-apply the physical power switches.  Core1 was powered
	 * off by pm_cpu_off(), so redo the power-on sequence (as arch_cpu_start
	 * does at initial boot) before CPU_ON, otherwise CPU_ON waits forever for
	 * a WFE-standby bit that never comes.
	 */
	soc_cpu_power_on(CPU1_MPID);
	pm_cpu_on(CPU1_MPID, (uintptr_t)&__start);

	/* Wait until CPU1 has fully warm-booted and restored its context (it
	 * sets itself back to CPU1_RUNNING at the tail of soc_cpu1_pg). The
	 * normal boot path (arch_cpu_start) likewise blocks until the secondary
	 * is up; without this CPU0 could race into the next PG while CPU1 is
	 * still re-initialising shared GIC/MMU state.
	 */
	while (amebasmart_cpu1_state != CPU1_RUNNING) {
		__asm__ volatile("dsb");
	}

	/* Flag the AP restart to the LP (KM0) core so it re-arms the PG
	 * handshake for the next cycle (matches FreeRTOS SOCPS_SleepPG). Without
	 * this the second cluster PG is never woken.  Bit is not in the Zephyr
	 * HAL header (sysreg_lsys.h): LSYS_BIT_AP_RST_WAIT_DRAM = (1 << 1).
	 */
	HAL_WRITE8(SYSTEM_CTRL_BASE_LP, REG_LSYS_AP_STATUS_SW,
		   HAL_READ8(SYSTEM_CTRL_BASE_LP, REG_LSYS_AP_STATUS_SW) | (1U << 1));
#endif
}

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

#ifdef CONFIG_SMP
	/* Secondary core never drives the SoC sleep. For CG it parks in WFE
	 * (CPU0 releases it on wake); for PG it powers itself off (hotplug) so
	 * CPU0 can cluster-PG, and is re-started on warm boot.
	 */
	if (arch_curr_cpu()->id != 0U) {
		if (AMEBA_SMP_CLUSTER_PG && state == PM_STATE_SUSPEND_TO_RAM) {
			soc_cpu1_pg();
		} else {
			/* CG, or PG-fallback-to-CG: park in WFE (see soc_sleep_pg,
			 * which then also does CG since CPU1 stays RUNNING).
			 */
			soc_cpu1_park();
		}
		return;
	}
#endif

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE: /* Clock gating */
		soc_sleep_cg();
		break;
	case PM_STATE_SUSPEND_TO_RAM: /* Power gating */
		soc_sleep_pg();
		break;
	default:
		LOG_ERR("Unsupported power state %u", state);
		k_cpu_idle();
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);

	/* Re-enable interrupts so the wake-source ISR can run. */
	irq_unlock(0);
}

/*
 * Enable each device flagged as a wake source in the devicetree
 * (wakeup-source; + wakeup-source-id = <WAKE_SRC_xxx>;).
 */
#define HANDLE_WAKEUP_SOURCE_NODE(node_id)                                                         \
	IF_ENABLED(DT_PROP_OR(node_id, wakeup_source, 0), (                                        \
		IF_ENABLED(DT_NODE_HAS_PROP(node_id, wakeup_source_id), (                          \
			do {                                                                       \
				const uint32_t wake_src = DT_PROP(node_id, wakeup_source_id);      \
				LOG_DBG("PM: Enabling wakeup source for %s (ID: %x)",              \
					DT_NODE_FULL_NAME(node_id), wake_src);                     \
				SOCPS_SetAPWakeEvent(wake_src, ENABLE);                            \
			} while (0);                                                               \
		))                                                                                 \
	))

#ifdef CONFIG_SMP
/*
 * Route all SPIs to CPU0.  GIC init targets SPIs at every online CPU, but the
 * secondary core is parked in WFE during SoC sleep and cannot service a wake
 * interrupt — if the wake SPI (rtc/counter/gpio) is delivered to the parked
 * CPU1 its ISR never runs.  Pinning SPIs to CPU0 keeps every wake ISR on the
 * core that drives and exits the sleep.
 */
static void soc_pin_spis_to_cpu0(void)
{
	for (unsigned int irq = 32; irq < CONFIG_NUM_IRQS; irq += 4) {
		sys_write32(0x01010101U, GICD_ITARGETSRn + irq);
	}
}
#endif

static int ameba_universal_wakeup_init(void)
{
#ifdef CONFIG_SMP
	soc_pin_spis_to_cpu0();
#endif
	DT_FOREACH_STATUS_OKAY_NODE(HANDLE_WAKEUP_SOURCE_NODE);
	LOG_DBG("PM: All wakeup sources initialized.");
	return 0;
}

SYS_INIT(ameba_universal_wakeup_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);

/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ameba_soc.h>

#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/drivers/timer/system_timer_lpm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/arch/common/pm_s2ram.h>
#include <zephyr/arch/arm/cortex_m/scb.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/timer/system_timer.h>
#if defined(CONFIG_FPU) && !defined(CONFIG_FPU_SHARING)
#include <zephyr/arch/arm/cortex_m/fpu.h>
#endif
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/sys_io.h>

#include "ameba_lpm_timer.h"

/*
 * Parts of a sleep only the secure world can carry out, reached through the TF-M
 * platform service; ameba_pm_tz_ioctl.h describes each request. AmebaG2 also has
 * to hand a few IPs to the non-secure zone while the AP is down, the register
 * deciding that being secure-only.
 */
#if defined(CONFIG_BUILD_WITH_TFM) && defined(CONFIG_TFM_PARTITION_PLATFORM)
#if defined(CONFIG_SOC_SERIES_AMEBAG2)
#define AMEBA_PM_TZ_PPC_HANDOVER 1
#endif
#define AMEBA_PM_TZ_SUSPEND 1

#include <tfm_platform_api.h>

#include <ameba_pm_tz_ioctl.h>
#endif

LOG_MODULE_REGISTER(soc_pm, LOG_LEVEL_DBG);

#ifdef CONFIG_PM_POLICY_CUSTOM
const struct pm_state_info *pm_policy_next_state(uint8_t cpu, int32_t ticks)
{
	uint8_t num_cpu_states;
	const struct pm_state_info *cpu_states;
	const struct pm_state_info *out_state = NULL;

#ifdef CONFIG_PM_NEED_ALL_DEVICES_IDLE
	if (pm_device_is_any_busy()) {
		return NULL;
	}
#endif

	/* Check ameba internal lock holds by wifi or bt */
	if (!pmu_ready_to_sleep()) {
		return NULL;
	}

	num_cpu_states = pm_state_cpu_get_all(cpu, &cpu_states);

	for (uint32_t i = 0; i < num_cpu_states; i++) {
		const struct pm_state_info *state = &cpu_states[i];
		uint32_t min_residency_ticks;

		min_residency_ticks =
			k_us_to_ticks_ceil32(state->min_residency_us + state->exit_latency_us);

		if (ticks < min_residency_ticks) {
			/* If current state has higher residency then use the previous state; */
			break;
		}

		/* check if state is available. */
		if (!pm_policy_state_is_available(state->state, state->substate_id)) {
			continue;
		}

		out_state = state;
	}

	return out_state;
}
#endif

/*
 * Power-gate (suspend-to-RAM) entry. lib_pmc.a calls this weak hook just before
 * sleeping; overriding it routes the sleep through Zephyr's own s2ram so the
 * resume restores Zephyr's registers/stack instead of the SDK context. Needed
 * for both Zephyr-side bootloaders: MCUboot and TF-M BL2 (//ns).
 *
 * With the ameba loader (neither bootloader) the wake entry stays the SDK's own
 * SOCPS_WakeFromPG_KM4(), which restores the core state itself, so none of this
 * -- including the SCB snapshot taken in pm_state_set() -- is built.
 */
#if defined(CONFIG_BOOTLOADER_MCUBOOT) || defined(CONFIG_TFM_BL2)
#define AMEBA_PM_ZEPHYR_S2RAM_WAKE 1
#endif

#if defined(AMEBA_PM_ZEPHYR_S2RAM_WAKE)
SRAM_ONLY_TEXT_SECTION
int pm_sleep_wfe(void)
{
	__WFE();
	__WFE();
	return -EBUSY;
}

/*
 * Power-gating resets the core, so everything outside Zephyr's s2ram context
 * (which holds only the general-purpose and stack/mask registers) has to be put
 * back by hand on resume. lib_pmc.a restores VTOR, SysTick, the NVIC and the MPU
 * around the sleep hook below, and the SDK's own wake entry
 * SOCPS_WakeFromPG_KM4() would have taken care of the rest -- but that entry is
 * replaced by z_arm_reset() here, and arch_pm_s2ram_resume() returns straight to
 * the sleep call site, long before soc_early_init_hook() would run. So do its
 * job here: re-enable the caches (their contents and their enables are both
 * gone) and restore the System Control Block, which carries the fault
 * configuration, the system-handler priorities and the coprocessor access
 * control.
 *
 * Only the power-gate path needs any of this. Clock-gating goes through
 * SOCPS_SleepCG(), which never calls vPortSystemPowerOff() and so never reaches
 * the hook below: it leaves the caches alone and keeps the core powered, so
 * there is nothing to restore. Save and restore are therefore paired on the
 * power-gate path alone and need no state check.
 */
static struct scb_context pm_scb_context;
#if defined(CONFIG_FPU) && !defined(CONFIG_FPU_SHARING)
static struct fpu_ctx_full pm_fpu_context;
#endif

SRAM_ONLY_TEXT_SECTION
static void pm_restore_core_state_after_pg(void)
{
	/*
	 * Enable the caches before restoring the SCB: their RAM content is undefined
	 * after the power-down, so they have to be invalidated as they are turned
	 * on, which writing CCR's enable bits back on its own would not do.
	 */
	sys_cache_instr_enable();
	sys_cache_data_enable();
#if defined(CONFIG_FPU) && !defined(CONFIG_FPU_SHARING)
	z_arm_restore_fp_context(&pm_fpu_context);
#endif
	z_arm_restore_scb_context(&pm_scb_context);
}

SRAM_ONLY_TEXT_SECTION
void pm_sleep_ram_for_wfe(struct CPU_BackUp_TypeDef *bk)
{
	ARG_UNUSED(bk);
#if defined(CONFIG_SOC_SERIES_AMEBADPLUS)
	/* On AmebaDplus, lib_pmc.a overwrites Img2EntryFun0.RamWakeupFun with
	 * SOCPS_WakeFromPG_KM4 before sleep. Restore to z_arm_reset so the
	 * MCUboot wake path calls arch_pm_s2ram_resume() correctly.
	 */
	extern RAM_START_FUNCTION Img2EntryFun0;
	extern void z_arm_reset(void);
	Img2EntryFun0.RamWakeupFun = z_arm_reset;
	Img2EntryFun0.VectorNS = (uint32_t)NewVectorTable;
#endif
#if defined(CONFIG_FPU) && !defined(CONFIG_FPU_SHARING)
	z_arm_save_fp_context(&pm_fpu_context);
#endif

	arch_pm_s2ram_suspend(pm_sleep_wfe);

	/*
	 * Has to complete before lib_pmc.a re-enables SysTick and the NVIC, which it
	 * does as soon as this returns: the system-handler priorities are still 0
	 * here, and BASEPRI cannot mask priority 0, so deferring this to
	 * pm_state_exit_post_ops() would leave a window in which a tick can preempt
	 * while the kernel believes interrupts are locked.
	 */
	pm_restore_core_state_after_pg();
}
#endif

#if defined(AMEBA_PM_TZ_PPC_HANDOVER)
/*
 * Mirror of the SOCPS_PeriPermissionEntry() and SOCPS_BitPermissionEntry() calls
 * the hal's vPortSystemPowerOff() and SOCPS_SleepCG() make around a sleep. Both
 * registers are secure-only, so this goes through the TF-M platform service, which
 * only accepts the IPs listed in ameba_pm_tz_ioctl.h.
 *
 * Issued here rather than from lib_pmc.a's own calls -- intercepted in
 * amebag2/pmc_veneer_tfm.c -- because it hands us the sleep hook with interrupts
 * locked and the caches on their way out, where the TF-M non-secure interface
 * cannot take its mutex. Release before the AP goes down and reclaim after it is
 * back is all the sequence requires.
 */
static void pm_tz_permission(uint32_t request, uint32_t ip_mask, bool release)
{
	struct ameba_pmc_tz_ppc_request req = {
		.ip_mask = ip_mask,
		.release = release ? 1U : 0U,
	};
	psa_invec in_vec = {
		.base = &req,
		.len = sizeof(req),
	};
	enum tfm_platform_err_t err;

	err = tfm_platform_ioctl(request, &in_vec, NULL);
	if (err != TFM_PLATFORM_ERR_SUCCESS) {
		LOG_ERR("%s %s of %08x failed: %d",
			request == AMEBA_PMC_TZ_IOCTL_BPC_PERMISSION ? "BPC" : "PPC",
			release ? "release" : "reclaim", ip_mask, (int)err);
	}
}

/*
 * Which bit permissions a state needs handed over; 0 for the states that need no
 * hand-over at all, deep-sleep among them as its resume path is not ported.
 */
static uint32_t pm_tz_bpc_mask(enum pm_state state)
{
	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		return AMEBA_PMC_TZ_BPC_CLOCK_GATE;
	case PM_STATE_SUSPEND_TO_RAM:
		return AMEBA_PMC_TZ_BPC_POWER_GATE;
	default:
		return 0U;
	}
}

static void pm_tz_handover(enum pm_state state, bool release)
{
	uint32_t bpc_mask = pm_tz_bpc_mask(state);

	if (bpc_mask == 0U) {
		return;
	}

	pm_tz_permission(AMEBA_PMC_TZ_IOCTL_BPC_PERMISSION, bpc_mask, release);
	pm_tz_permission(AMEBA_PMC_TZ_IOCTL_PPC_PERMISSION,
			 release ? AMEBA_PMC_TZ_PPC_SLEEP_RELEASE : AMEBA_PMC_TZ_PPC_WAKE_RECLAIM,
			 release);
}
#endif /* AMEBA_PM_TZ_PPC_HANDOVER */

#if defined(AMEBA_PM_TZ_SUSPEND)
static void pm_tz_suspend(void)
{
	enum tfm_platform_err_t err;

	err = tfm_platform_ioctl(AMEBA_PM_TZ_IOCTL_SUSPEND, NULL, NULL);
	if (err != TFM_PLATFORM_ERR_SUCCESS) {
		LOG_ERR("secure suspend failed: %d", (int)err);
	}
}
#endif /* AMEBA_PM_TZ_SUSPEND */

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	LOG_INF("call %s: %d", __func__, state);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE: /* Clock gating */
		pmu_set_sleep_type(SLEEP_CG);
		break;
	case PM_STATE_SUSPEND_TO_RAM: /* Power gating */
		pmu_set_sleep_type(SLEEP_PG);
		/*
		 * Snapshot the System Control Block while it is still the one the
		 * kernel set up: lib_pmc.a disables the D-cache further down this call
		 * chain, so taking the snapshot inside the sleep hook would capture --
		 * and later restore -- a CCR with the cache already turned off.
		 */
#if defined(AMEBA_PM_ZEPHYR_S2RAM_WAKE)
		z_arm_save_scb_context(&pm_scb_context);
#endif
		break;
	case PM_STATE_SOFT_OFF: /* Deepsleep */
		pmu_release_deepwakelock(PMU_OS);
		break;
	default:
		LOG_ERR("Unsupported power state %u", state);
		k_cpu_idle();
		return;
	}

#if defined(AMEBA_PM_TZ_PPC_HANDOVER)
	pm_tz_handover(state, true);
#endif

#if defined(AMEBA_PM_TZ_SUSPEND)
	/*
	 * Last secure call before the AP goes down, so that what the secure world
	 * records and flushes is not overtaken by writes of its own. Only the
	 * power-gate needs it; clock-gating keeps the core and its caches.
	 */
	if (state == PM_STATE_SUSPEND_TO_RAM) {
		pm_tz_suspend();
	}
#endif

	/*
	 * This is what actually sleeps: pmu_pre_sleep_processing() ends in
	 * SOCPS_SleepPG()/SOCPS_SleepCG(). Its out-parameter is a system-timer
	 * stamp for the hal's own tick compensation, which nothing reads here --
	 * z_sys_clock_lpm_exit() samples the timer itself -- so it goes to a local.
	 */
	{
		uint32_t hal_tick_before_sleep;

		pmu_pre_sleep_processing(&hal_tick_before_sleep);
	}
}

/*
 * Both sleep states stop SysTick, so the kernel loses the whole sleep from its
 * notion of time unless something hands the elapsed time back. Doing that by
 * calling sys_clock_announce() from here would only repair uptime: the driver also
 * keeps a cycle_count that k_cycle_get_32() reads, which is static to it and has to
 * stay in step with its announced_cycles. Reporting through the system timer's
 * low-power companion hooks lets the driver repair both.
 *
 * Only the measurement is ours: the always-on ameba system timer (TIM0), sampled in
 * common/ameba_lpm_timer.c, which the CA32 path shares. The other half of
 * z_sys_clock_lpm_enter()'s contract -- guarantee a wake within max_lpm_time_us --
 * is met by publishing that deadline for the PMC wake timer, below.
 */

BUILD_ASSERT(IS_ENABLED(CONFIG_SYSTEM_TIMER_LPM_COMPANION_HOOKS),
	     "the ameba PM path reports slept time through the system timer's low-power "
	     "companion hooks; without them neither kernel time base survives a sleep");

/*
 * Upper bound on what is handed to the PMC wake timer, set by the millisecond-to-tick
 * conversion rather than by the counter. pmu_set_wakeup_timer() scales by 32.7695 as
 * (ms << 5) + ((ms * 197) >> 8), avoiding a division; the ms * 197 in there overflows
 * 32 bits from 6.06 hours on, well before the counter itself wraps at 36.4 hours. So
 * bound it at four hours, comfortably inside that, and treat anything longer as "no
 * deadline" -- which is what the kernel means by a timeout that far out anyway.
 */
#define AMEBA_PMC_WAKE_MAX_MS (4U * 60U * 60U * 1000U)

void z_sys_clock_lpm_enter(uint64_t max_lpm_time_us)
{
	uint64_t deadline_ms = max_lpm_time_us / USEC_PER_MSEC;

	ameba_lpm_timer_mark_entry();

	/*
	 * Give the PMC the deadline the kernel is idling to. pmu_get_sleep_time()
	 * reads this range on the way into the sleep and lib_pmc.a arms the PMC wake
	 * timer from it, so the sleep ends on time whether or not the application has
	 * a wake source of its own armed.
	 *
	 * min > max selects that function's one-shot case: it returns max and resets
	 * the range to PMU_SLEEP_FOREVER, so a later sleep that arrives without a
	 * deadline (the kernel skips this hook entirely for K_TICKS_FOREVER) is not
	 * woken by a stale one. Clamped to at least 1 ms because a max of 0 is the
	 * encoding for "no deadline" and would ask for a sleep that never ends.
	 */
	if (deadline_ms == 0U) {
		deadline_ms = 1U;
	}

	if (deadline_ms <= AMEBA_PMC_WAKE_MAX_MS) {
		pmu_set_sleep_time_range(PMU_SLEEP_FOREVER, (uint32_t)deadline_ms);
	} else {
		pmu_set_sleep_time_range(PMU_SLEEP_FOREVER, PMU_SLEEP_FOREVER);
	}
}

uint64_t z_sys_clock_lpm_exit(void)
{
	return ameba_lpm_timer_elapsed_us();
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	pmu_acquire_deepwakelock(PMU_OS);

	LOG_INF("[%s] AP wake: %d", __func__, state);
	irq_unlock(0);

#if defined(AMEBA_PM_TZ_PPC_HANDOVER)
	/*
	 * After irq_unlock() on purpose: this is a secure call, and the TF-M non-secure
	 * interface is not meant to be entered with interrupts locked.
	 */
	pm_tz_handover(state, false);
#endif
}

void pm_s2ram_mark_set(void)
{
	/* Mark is set in NP image */
}

/* This function is only invoked in mcuboot image because system resume from mcuboot */
bool pm_s2ram_mark_check_and_clear(void)
{
	if (sys_read32(SYSTEM_CTRL_BASE + REG_LSYS_BOOT_CFG) & LSYS_BIT_BOOT_WAKE_FROM_PS_HS) {
		/* DO NOT clear the flag here because some post-wake code rely on it in lib_pmc.a*/
		return true;
	}
	return false;
}

/*
 * Zephyr owns the AP wake-source arming whenever it is the image booted by a
 * Zephyr-side bootloader: MCUboot, or TF-M BL2 for the //ns split S/NS build.
 * (With the ameba-loader the KM0 image arms them instead.)
 */
#if defined(CONFIG_BOOTLOADER_MCUBOOT) || defined(CONFIG_TFM_BL2)
#define HANDLE_WAKEUP_SOURCE_NODE(node_id)                                                         \
	IF_ENABLED(DT_PROP_OR(node_id, wakeup_source, 0), (         \
		IF_ENABLED(DT_NODE_HAS_PROP(node_id, wakeup_source_id), ( \
			do {                                                    \
				const uint32_t wake_src =                             \
					DT_PROP(node_id, wakeup_source_id);                 \
				LOG_DBG("PM: Enabling wakeup source for %s (ID: %x)", \
						DT_NODE_FULL_NAME(node_id), wake_src);            \
				SOCPS_SetAPWakeEvent(wake_src, ENABLE);               \
			} while (0);                                            \
		))                                                        \
	))

static int ameba_universal_wakeup_init(void)
{
	DT_FOREACH_STATUS_OKAY_NODE(HANDLE_WAKEUP_SOURCE_NODE);
	LOG_DBG("PM: All wakeup sources initialized.");
	return 0;
}

SYS_INIT(ameba_universal_wakeup_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);
#endif

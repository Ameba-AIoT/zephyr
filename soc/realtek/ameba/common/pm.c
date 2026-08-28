/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ameba_soc.h>

#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
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

/*
 * AmebaG2 only: the power-gate sleep needs a few IPs handed to the non-secure zone
 * while the AP is down, and the register that decides this is secure-only. With the
 * ameba secure world that goes through a PMC_ENTRY veneer; with TF-M it goes through
 * the platform service. See ameba_pmc_tz_ioctl.h.
 */
#if defined(CONFIG_SOC_SERIES_AMEBAG2) && defined(CONFIG_BUILD_WITH_TFM) &&                        \
	defined(CONFIG_TFM_PARTITION_PLATFORM)
#define AMEBA_PM_TZ_PPC_HANDOVER 1
#include <tfm_platform_api.h>

#include <ameba_pmc_tz_ioctl.h>
#endif

LOG_MODULE_REGISTER(soc_pm, LOG_LEVEL_DBG);

static uint32_t tick_before_sleep;

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
 * Mirror of the SOCPS_PeriPermissionEntry() calls that FreeRTOS' AP-side
 * vPortSystemPowerOff() makes around a power-gated sleep. The register is
 * secure-only, so it goes through the TF-M platform service; the secure side only
 * accepts the IPs listed in ameba_pmc_tz_ioctl.h.
 *
 * The release has to be in place before the AP powers down and can only be taken
 * back once it is running again, which is why the two halves sit in pm_state_set()
 * and pm_state_exit_post_ops() rather than around the WFE itself: the secure call
 * needs a context where the TF-M non-secure interface can take its mutex, and by
 * the time lib_pmc.a hands us the sleep hook interrupts are locked and the caches
 * are on their way out.
 */
static void pm_tz_ppc_permission(uint32_t ip_mask, bool release)
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

	err = tfm_platform_ioctl(AMEBA_PMC_TZ_IOCTL_PPC_PERMISSION, &in_vec, NULL);
	if (err != TFM_PLATFORM_ERR_SUCCESS) {
		LOG_ERR("PPC %s of %08x failed: %d", release ? "release" : "reclaim", ip_mask,
			(int)err);
	}
}
#endif /* AMEBA_PM_TZ_PPC_HANDOVER */

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
#if defined(AMEBA_PM_TZ_PPC_HANDOVER)
		pm_tz_ppc_permission(AMEBA_PMC_TZ_PPC_SLEEP_RELEASE, true);
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

	pmu_pre_sleep_processing(&tick_before_sleep);
}

/*
 * Both sleep states stop SysTick -- clock-gating takes the core clock away and
 * power-gating takes the core with it -- so the kernel loses the whole sleep from
 * its notion of time, and every k_timeout_t outlives its deadline by however long
 * the system slept. FreeRTOS gets the time back through
 * configPOST_SLEEP_PROCESSING (pmu_post_sleep_processing() -> vTaskCompTick());
 * do the equivalent here.
 *
 * The reference is the ameba system timer (TIM0), free-running at 32768 Hz and
 * kept powered across both sleep states, which pmu_pre_sleep_processing() sampled
 * into tick_before_sleep just before sleeping.
 *
 * Elapsed system-timer ticks are accumulated rather than converted one sleep at a
 * time: at 32768 Hz against a 10 kHz kernel each sleep leaves a sub-tick
 * remainder, and dropping it would lose up to 3 ticks per sleep, which a workload
 * that sleeps continuously turns straight back into a drifting clock.
 *
 * The accumulator is the reason the sample has to be checked before it goes in
 * rather than sanity-checked afterwards: it only ever moves forward, as does
 * sys_clock_announce(), so a single bad sample cannot be taken back and skews
 * uptime for the rest of the boot.
 *
 * SYSTIMER_GetPassTick() is deliberately not used for that reason. The ROM
 * implementation treats current < start as a counter wrap and returns
 * 0xFFFFFFFF - (start - current), which is right for a free-running counter and
 * catastrophic for one that got reset: it announces ~2^32 ticks, i.e. 2^32/32768
 * = 36:24:32 of sleep, after which every k_timeout_t is already expired. That is
 * not hypothetical -- TIM0 is also TIMER0, which the dts exposes as a Zephyr
 * counter device, and an application driving it as one resets and stops it. The
 * dts nodes carry a warning and the sample was moved off timer0, but a
 * non-monotonic reading has to be survivable here regardless. A real wrap takes
 * a single sleep of 36 hours and is not worth distinguishing.
 */
#define AMEBA_SYSTIMER_HZ 32768U

static void pm_announce_time_lost_while_asleep(void)
{
	static uint64_t elapsed_systimer_ticks;
	static uint64_t announced_kernel_ticks;
	uint64_t kernel_ticks;
	uint32_t now = SYSTIMER_TickGet();

	if (now < tick_before_sleep) {
		LOG_WRN("system timer went backwards (%u -> %u), sleep time lost",
			tick_before_sleep, now);
		return;
	}

	elapsed_systimer_ticks += now - tick_before_sleep;
	kernel_ticks = (elapsed_systimer_ticks * CONFIG_SYS_CLOCK_TICKS_PER_SEC) /
		       AMEBA_SYSTIMER_HZ;

	if (kernel_ticks > announced_kernel_ticks) {
		uint32_t ticks = (uint32_t)(kernel_ticks - announced_kernel_ticks);

		announced_kernel_ticks = kernel_ticks;
		sys_clock_announce(ticks);
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	pmu_acquire_deepwakelock(PMU_OS);

	pm_announce_time_lost_while_asleep();

	LOG_INF("[%s] AP wake: %d", __func__, state);
	irq_unlock(0);

#if defined(AMEBA_PM_TZ_PPC_HANDOVER)
	/*
	 * After irq_unlock() on purpose: this is a secure call, and the TF-M non-secure
	 * interface is not meant to be entered with interrupts locked.
	 */
	if (state == PM_STATE_SUSPEND_TO_RAM) {
		pm_tz_ppc_permission(AMEBA_PMC_TZ_PPC_WAKE_RECLAIM, false);
	}
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

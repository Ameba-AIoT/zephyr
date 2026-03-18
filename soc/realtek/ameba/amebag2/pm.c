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
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

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

	printf("state: %d\n", out_state->state);
	return out_state;
}
#endif

#ifdef CONFIG_BOOTLOADER_MCUBOOT
SRAM_ONLY_TEXT_SECTION
int pm_sleep_wfe(void)
{
	__WFE();
	__WFE();
	return -EBUSY;
}

SRAM_ONLY_TEXT_SECTION
void pm_sleep_ram_for_wfe(struct CPU_BackUp_TypeDef *bk)
{
	ARG_UNUSED(bk);

	arch_pm_s2ram_suspend(pm_sleep_wfe);
}
#endif

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

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);

	pmu_acquire_deepwakelock(PMU_OS);

	LOG_INF("[%s] AP wake: %d", __func__, state);
	irq_unlock(0);
}

void pm_s2ram_mark_set(void)
{
	/* Mark is set in NP image */
}

/* This function is only invoked in mcuboot image because system resume from mcuboot */
bool pm_s2ram_mark_check_and_clear(void)
{
	if (HAL_READ32(SYSTEM_CTRL_BASE, REG_LSYS_BOOT_CFG) & LSYS_BIT_BOOT_WAKE_FROM_PS_HS) {
		/* DO NOT clear the flag here because some post-wake code rely on it in lib_pmc.a*/
		return true;
	}
	return false;
}

#ifdef CONFIG_BOOTLOADER_MCUBOOT
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

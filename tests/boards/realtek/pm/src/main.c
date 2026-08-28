/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Suspend/resume tests for the Ameba SoCs.
 *
 * These are regression guards for the SoC power-management port, not a
 * demonstration of it. Suspend-to-RAM on this hardware depends on the flash and
 * SRAM layout, on which devicetree nodes are enabled, on the secure image and
 * bootloader restoring state the resume path does not re-initialise, and on the
 * timers that survive the power-down -- all of which are maintained by other
 * people for other reasons (TF-M, MCUboot/OTA, drivers). Every check below
 * corresponds to a defect that was actually hit during the port, so that
 * re-introducing one fails here instead of silently taking the AP down.
 *
 * Two kinds of check:
 *
 *   - Build-time (BUILD_ASSERT below): the devicetree and Kconfig premises the
 *     PM code relies on. These fail the build with an explanation rather than
 *     hanging a board, and they are the ones most likely to catch a change made
 *     elsewhere in the tree.
 *   - Run-time: one cycle of the requested state per case, then
 *     pm_check_resume_invariants() over everything the resume path has to bring
 *     back. The kernel-clock bounds are part of this: k_uptime_get() is fed by
 *     the SoC's own sleep-time recovery, so it catches that going wrong in
 *     either direction -- under-reporting (the clock stalling across the sleep)
 *     and over-reporting (the ROM's SYSTIMER_GetPassTick() taking a reset
 *     counter for a wrapped one and crediting ~2^32 ticks).
 *
 * Arming a wake source is not optional on this hardware. The residency the
 * kernel asks for never reaches the ameba PMC (sleep_param.sleep_time comes
 * from pmu_get_sleep_time() on the Cortex-M SoCs and is PMU_SLEEP_FOREVER on
 * AmebaSmart), so a power-gated AP with no wake source armed simply never comes
 * back.
 *
 * The blocking is done on a semaphore the wake callback gives, not on a kernel
 * timeout. A timeout is only satisfied if the kernel clock recovers the time
 * spent powered down, and that is a property under test here -- on AmebaSmart it
 * deliberately does not hold (see PM_CREDITS_GATED_TIME). Where it does not,
 * waiting on a timeout would leave the thread short of its deadline after the
 * wake, so it would idle again with the one-shot alarm already consumed and the
 * AP would power down for good.
 *
 * Note that the per-test durations ztest prints are not usable here: they come
 * from the cycle counter, which the power-down resets (SysTick-derived on the
 * Cortex-M SoCs, the ARM generic timer on AmebaSmart), so they are meaningless
 * -- ~0 on the Cortex-M targets and tens of seconds on AmebaSmart regardless of
 * how long the sleep was. The uptime stamps in the log are the ones to read.
 */

#include <ameba_soc.h>

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_SOC_SERIES_AMEBASMART)
#include <zephyr/arch/arm/cortex_a_r/cpu.h>
#include <zephyr/arch/arm/cortex_a_r/lib_helpers.h>
#else
#include <cmsis_core.h>
#endif

/* ------------------------------------------------- build-time invariants */

BUILD_ASSERT(IS_ENABLED(CONFIG_PM), "these tests only mean anything with CONFIG_PM");

/*
 * The idle path only suspends a CPU that declares idle states. On AmebaSmart
 * under SMP that includes the secondary: without cpu-power-states its pm_policy
 * returns NULL, it never parks or hotplugs off, and CPU0 hot-spins waiting for
 * it instead of cluster-power-gating.
 */
BUILD_ASSERT(DT_NODE_HAS_PROP(DT_PATH(cpus, cpu_0), cpu_power_states),
	     "cpu0 declares no cpu-power-states, so the idle path never suspends");
#if defined(CONFIG_SMP)
BUILD_ASSERT(DT_NODE_HAS_PROP(DT_PATH(cpus, cpu_1), cpu_power_states),
	     "cpu1 declares no cpu-power-states: it never parks or hotplugs off and CPU0 cannot "
	     "cluster-power-gate");
#endif

/*
 * A wake source is only armed if it carries BOTH wakeup-source (the board
 * overlay's job) and wakeup-source-id (the SoC devicetree's job) --
 * ameba_universal_wakeup_init() skips a node that is missing either, without
 * complaining, and the AP then never returns from the power-down.
 */
#define ASSERT_ARMABLE_WAKE_SOURCE(node_id, name)                                                  \
	BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(node_id), name " is disabled, so it cannot wake the " \
							    "AP");                                 \
	BUILD_ASSERT(DT_PROP_OR(node_id, wakeup_source, 0),                                        \
		     name " is not flagged wakeup-source, so ameba_universal_wakeup_init() skips " \
			  "it");                                                                   \
	BUILD_ASSERT(DT_NODE_HAS_PROP(node_id, wakeup_source_id),                                  \
		     name " carries no wakeup-source-id, so there is no PMC wake event to enable")

ASSERT_ARMABLE_WAKE_SOURCE(DT_ALIAS(pm_wake_counter), "the pm-wake-counter");
ASSERT_ARMABLE_WAKE_SOURCE(DT_NODELABEL(rtc), "the rtc");

#if !defined(CONFIG_SOC_SERIES_AMEBASMART)
/*
 * On the Cortex-M SoCs the wake path is Zephyr's own: ameba_universal_wakeup_init()
 * is compiled out unless an MCUboot or TF-M BL2 chain is in use, because on the
 * plain ameba-loader targets the KM0 image arms the wake sources instead. Built
 * for one of those, every case here would power the AP down for good.
 */
BUILD_ASSERT(IS_ENABLED(CONFIG_BOOTLOADER_MCUBOOT) || IS_ENABLED(CONFIG_TFM_BL2),
	     "nothing arms a wake source on this target: soc/realtek/ameba/common/pm.c only "
	     "compiles ameba_universal_wakeup_init() for the MCUboot / TF-M BL2 chains");

/*
 * TIMER0 is TIM0 / TIMER0_REG_BASE, the always-on reference that
 * soc_early_init_hook() starts (SYSTIMER_Init) and that the PM code reads to
 * recover the time spent asleep. Enabling the node hands the same hardware to
 * the counter driver, whose init reprograms it without re-enabling it and whose
 * counter_start() resets it mid-run -- which stops the kernel clock across
 * suspend and then makes the ROM's wrap branch credit 2^32 ticks (uptime jumps
 * by 36:24:32).
 */
BUILD_ASSERT(!DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(timer0)),
	     "TIMER0 is the ameba system timebase the PM tick recovery reads, not a spare "
	     "counter: enabling the node stops it");
BUILD_ASSERT(!DT_SAME_NODE(DT_ALIAS(pm_wake_counter), DT_NODELABEL(timer0)),
	     "pm-wake-counter must not be TIMER0 (see above); use one of the wake-capable "
	     "basic timers instead");

/*
 * Which SRAM window the image runs out of is the layout invariant that took the
 * longest to find: sram0 is the 4 KB-bootloader window, correct for //mcuboot
 * and the plain ameba-loader, while a TF-M build must use sram0_ns, which starts
 * above BL2's 32 KB resident window and the secure image's data. Pointing an NS
 * build at sram0 puts it underneath the secure image.
 */
#if defined(CONFIG_BUILD_WITH_TFM)
BUILD_ASSERT(DT_SAME_NODE(DT_CHOSEN(zephyr_sram), DT_NODELABEL(sram0_ns)),
	     "a TF-M non-secure build must run out of sram0_ns, not the bootloader-sized sram0");
#else
BUILD_ASSERT(DT_SAME_NODE(DT_CHOSEN(zephyr_sram), DT_NODELABEL(sram0)),
	     "a non-TF-M build must run out of sram0: sram0_ns is carved out for the TF-M split "
	     "and wastes the memory below it");
#endif
#endif /* !CONFIG_SOC_SERIES_AMEBASMART */

/* ------------------------------------------------------------- parameters */

/* How long the wake source is armed for. */
#define WAKE_MS 1000

/*
 * Bounds on the wall time one sleep cycle may be credited with. The lower bound
 * is below WAKE_MS because the ameba RTC alarm only has a one-second
 * granularity; the upper bound is WAKE_MS plus what the wake path itself may
 * add, which differs by an order of magnitude between //mcuboot and //ns (where
 * the secure image re-initialises on every wake).
 */
#define CREDIT_MIN_MS 800
#define CREDIT_MAX_MS (WAKE_MS + 1000)

/*
 * Failure guard only, not a deadline: it bounds the wait if a wake source never
 * fires at all. Note that on a power-gated AP the kernel clock is not running
 * either, so this can only expire once the AP is back -- a wake source that
 * never fires still hangs the run rather than failing it.
 */
#define WAKE_TIMEOUT_MS (WAKE_MS + 4000)

#if defined(CONFIG_SOC_SERIES_AMEBASMART)
/*
 * AmebaSmart differs from the Cortex-M SoCs in two ways the assertions have to
 * allow for:
 *
 * - The kernel clock does not advance across power-gating. Nothing on the CA32
 *   side survives PG -- the ARM generic timer, TIM0/SYSTIMER and the debug timer
 *   all gate with the AP domain, and the RTC calendar does not free-run on this
 *   path -- so only KM0 knows how long the gate lasted, and
 *   soc/realtek/ameba/amebasmart/pm.c re-bases the arch timer with zero elapsed
 *   ticks on purpose. Only the upper bound is checked there.
 * - Under SMP, CPU0 must not sleep until CPU1 has parked in WFE (CG) or powered
 *   itself off (PG); until then it abandons the attempt and the idle loop
 *   retries, so one sleep can be announced as several enter/exit pairs.
 */
#define PM_CREDITS_GATED_TIME    0
#define PM_ONE_ATTEMPT_PER_SLEEP 0
#else
#define PM_CREDITS_GATED_TIME    1
#define PM_ONE_ATTEMPT_PER_SLEEP 1
#endif

/* The ameba system timer (TIM0) runs off the 32.768 kHz always-on domain. */
#define AMEBA_SYSTIMER_HZ 32768U

/* ------------------------------------------------------------- canaries */

/*
 * RAM that the resume path must bring back untouched. .noinit is the
 * interesting section: a suspend-to-RAM resume comes back with RAM intact, so
 * anything that overwrites it -- a secure image or bootloader whose data now
 * overlaps the non-secure image, a resume path running on the wrong stack -- is
 * a layout regression. The boot counter is here for the coarser version of the
 * same failure: if the wake path faults or the watchdog fires, the image
 * restarts and this reads 2 or more.
 */
#define BOOT_MAGIC 0x504d4254 /* 'PMBT' */
static uint32_t boot_magic __noinit;
static uint32_t boot_count __noinit;

#define RAM_CANARY_WORDS 256
static uint32_t ram_canary[RAM_CANARY_WORDS] __noinit;

static uint32_t ram_canary_word(size_t i)
{
	return 0xa5a50000U + (uint32_t)i;
}

/*
 * Read-only data, which on the //ns and //mcuboot targets is executed and read
 * in place out of flash: checking it after a resume also exercises the flash
 * controller and the RSIP XIP window still being configured. On AmebaSmart the
 * image runs from DRAM, where the same check covers the DRAM/PSRAM re-init.
 */
#define ROM_CANARY_WORD(i, _) (0x5a5a0000U + (i))
static const uint32_t rom_canary[] = {LISTIFY(16, ROM_CANARY_WORD, (,))};

/* --------------------------------------------------- CPU state baselines */

#if !defined(CONFIG_SOC_SERIES_AMEBASMART)
#if defined(CONFIG_CPU_CORTEX_M_HAS_VTOR)
#if defined(CONFIG_SRAM_VECTOR_TABLE)
#define EXPECTED_VTOR ((uint32_t)((uintptr_t)_sram_vector_start))
#else
#define EXPECTED_VTOR ((uint32_t)((uintptr_t)_vector_start))
#endif
#endif /* CONFIG_CPU_CORTEX_M_HAS_VTOR */

#if defined(CPACR_PRESENT)
static uint32_t cpacr_at_boot;
#endif
#endif /* !CONFIG_SOC_SERIES_AMEBASMART */

static void cpu_state_baseline(void)
{
#if !defined(CONFIG_SOC_SERIES_AMEBASMART) && defined(CPACR_PRESENT)
	cpacr_at_boot = SCB->CPACR;
#endif
}

/*
 * Everything the resume path has to have brought back before the kernel may run
 * again. On the Cortex-M SoCs all of this rides on the SCB snapshot the PM code
 * takes before the power-down (soc/realtek/ameba/common/pm.c): the wake enters
 * through the ROM's RamWakeupFun, not Reset_Handler, so nothing that
 * Reset_Handler would normally set up is restored by hardware.
 */
static void pm_check_resume_invariants(void)
{
#if defined(CONFIG_SOC_SERIES_AMEBASMART)
	uint32_t sctlr = read_sctlr();

	zassert_true((sctlr & SCTLR_C_BIT) != 0U,
		     "the D-cache is off after the resume (SCTLR=%08x)", sctlr);
	zassert_true((sctlr & SCTLR_I_BIT) != 0U,
		     "the I-cache is off after the resume (SCTLR=%08x)", sctlr);
#else
	/*
	 * Caches: the first resume used to come back with both off and leave
	 * them off, which stretched the next power-down's handshake past the
	 * budget the NP allows and made it bounce the AP awake instead of gating
	 * it -- a second-cycle symptom of a first-cycle defect.
	 */
	zassert_true((SCB->CCR & SCB_CCR_IC_Msk) != 0U,
		     "the I-cache is off after the resume (CCR=%08x)", (unsigned int)SCB->CCR);
	zassert_true((SCB->CCR & SCB_CCR_DC_Msk) != 0U,
		     "the D-cache is off after the resume (CCR=%08x)", (unsigned int)SCB->CCR);

#if defined(CONFIG_CPU_CORTEX_M_HAS_VTOR)
	/*
	 * The vector table: a resume that comes back with VTOR at its reset
	 * value takes the next interrupt through whatever the ROM left at
	 * address 0, which is how the AP ends up executing ROM addresses
	 * instead of faulting somewhere legible.
	 */
	zassert_equal(SCB->VTOR & SCB_VTOR_TBLOFF_Msk, EXPECTED_VTOR & SCB_VTOR_TBLOFF_Msk,
		      "VTOR is %08x after the resume, expected the image's vector table at %08x",
		      (unsigned int)SCB->VTOR, (unsigned int)EXPECTED_VTOR);
#endif

#if defined(CPACR_PRESENT)
	/* Coprocessor access, or the first floating-point instruction after a
	 * resume takes a usage fault.
	 */
	zassert_equal(SCB->CPACR, cpacr_at_boot, "CPACR is %08x after the resume, was %08x at boot",
		      (unsigned int)SCB->CPACR, (unsigned int)cpacr_at_boot);
#endif
#endif /* CONFIG_SOC_SERIES_AMEBASMART */

	for (size_t i = 0; i < ARRAY_SIZE(rom_canary); i++) {
		zassert_equal(rom_canary[i], ROM_CANARY_WORD(i, _),
			      "read-only data reads %08x at word %u, expected %08x -- the flash "
			      "or XIP window is not configured as it was before the sleep",
			      (unsigned int)rom_canary[i], (unsigned int)i,
			      (unsigned int)ROM_CANARY_WORD(i, _));
	}

	for (size_t i = 0; i < ARRAY_SIZE(ram_canary); i++) {
		zassert_equal(ram_canary[i], ram_canary_word(i),
			      ".noinit word %u reads %08x, expected %08x -- something wrote over "
			      "non-secure RAM across the sleep",
			      (unsigned int)i, (unsigned int)ram_canary[i],
			      (unsigned int)ram_canary_word(i));
	}

	zassert_equal(boot_count, 1,
		      "the image restarted %u times; the resume path did not come back through "
		      "s2ram",
		      boot_count);
}

/* -------------------------------------------------------- state counting */

static atomic_t entry_count[PM_STATE_COUNT];
static atomic_t exit_count[PM_STATE_COUNT];

/* Given by whichever wake source ended the sleep. */
static K_SEM_DEFINE(wake_sem, 0, 1);

/*
 * Under SMP every idle CPU runs the notifiers, but only CPU0 drives the SoC
 * sleep (the secondary parks or hotplugs itself off), so only its transitions
 * describe what the SoC actually did. arch_curr_cpu() reads as CPU 0 on the
 * uniprocessor builds.
 */
static bool pm_on_sleep_driving_cpu(void)
{
	return arch_curr_cpu()->id == 0U;
}

static void notify_pm_state_entry(enum pm_state state)
{
	if (pm_on_sleep_driving_cpu()) {
		atomic_inc(&entry_count[state]);
	}
}

static void notify_pm_state_exit(enum pm_state state)
{
	if (pm_on_sleep_driving_cpu()) {
		atomic_inc(&exit_count[state]);
	}
}

static struct pm_notifier notifier = {
	.state_entry = notify_pm_state_entry,
	.state_exit = notify_pm_state_exit,
};

static void pm_counters_reset(void)
{
	for (int i = 0; i < PM_STATE_COUNT; i++) {
		atomic_clear(&entry_count[i]);
		atomic_clear(&exit_count[i]);
	}
}

/* Leave only `keep` selectable, so the idle path cannot pick a neighbouring
 * state and make the result ambiguous.
 */
static void pm_states_restrict_to(enum pm_state keep)
{
	for (int i = 0; i < PM_STATE_COUNT; i++) {
		if (i != keep) {
			pm_policy_state_lock_get(i, PM_ALL_SUBSTATES);
		}
	}
}

static void pm_states_unrestrict(enum pm_state keep)
{
	for (int i = 0; i < PM_STATE_COUNT; i++) {
		if (i != keep) {
			pm_policy_state_lock_put(i, PM_ALL_SUBSTATES);
		}
	}
}

/*
 * Sleep through one cycle of `state` and check what came back. The wake source
 * is expected to have been armed by the caller.
 */
static void pm_expect_sleep_cycle(enum pm_state state)
{
	int entries;
	int exits;
	int64_t elapsed;
	int64_t start;
#if PM_CREDITS_GATED_TIME
	uint32_t systimer_start;
	uint32_t systimer_ms;
#endif

	k_sem_reset(&wake_sem);
	pm_counters_reset();
	pm_states_restrict_to(state);

	start = k_uptime_get();
#if PM_CREDITS_GATED_TIME
	systimer_start = SYSTIMER_TickGet();
#endif
	zassert_ok(k_sem_take(&wake_sem, K_MSEC(WAKE_TIMEOUT_MS)),
		   "the armed wake source never fired");
#if PM_CREDITS_GATED_TIME
	systimer_ms = (uint32_t)(((uint64_t)(SYSTIMER_TickGet() - systimer_start) * MSEC_PER_SEC) /
				 AMEBA_SYSTIMER_HZ);
#endif
	elapsed = k_uptime_get() - start;

	pm_states_unrestrict(state);

	entries = (int)atomic_get(&entry_count[state]);
	exits = (int)atomic_get(&exit_count[state]);

	zassert_true(entries >= 1, "state %d was never entered", state);
	zassert_equal(exits, entries, "state %d entered %d times but left %d", state, entries,
		      exits);
	if (PM_ONE_ATTEMPT_PER_SLEEP) {
		zassert_equal(entries, 1, "entered state %d %d times, expected 1", state, entries);
	}

#if PM_CREDITS_GATED_TIME
	/*
	 * The reference the tick recovery is built on: TIM0 has to keep counting
	 * across the power-down, and has to not be reset behind the PM code's
	 * back. Checked before the kernel clock, because the kernel clock is
	 * derived from it -- if this one is wrong, the next check only says so
	 * indirectly.
	 */
	zassert_between_inclusive(systimer_ms, CREDIT_MIN_MS, CREDIT_MAX_MS,
				  "the ameba system timer advanced %u ms across a sleep the wake "
				  "source ended after %d ms: it is not the continuous reference "
				  "the tick recovery assumes",
				  systimer_ms, WAKE_MS);

	/* Under-reporting: the kernel clock stalled across the sleep. */
	zassert_true(elapsed >= CREDIT_MIN_MS,
		     "kernel clock advanced only %d ms across a sleep the wake source ended "
		     "after %d ms -- the SoC's sleep-time recovery is under-reporting",
		     (int)elapsed, WAKE_MS);
#endif

	/* Over-reporting: it credited more than the wake source could have taken. */
	zassert_true(elapsed <= CREDIT_MAX_MS,
		     "kernel clock advanced %d ms across a sleep the wake source ended "
		     "after %d ms -- the SoC's sleep-time recovery is over-reporting",
		     (int)elapsed, WAKE_MS);

	pm_check_resume_invariants();
}

/* ---------------------------------------------------------------- wake sources */

static const struct device *const counter_dev = DEVICE_DT_GET(DT_ALIAS(pm_wake_counter));
static const struct device *const rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rtc));

static void counter_wake_cb(const struct device *dev, uint8_t chan, uint32_t ticks, void *user)
{
	ARG_UNUSED(chan);
	ARG_UNUSED(ticks);
	ARG_UNUSED(user);

	counter_cancel_channel_alarm(dev, 0);
	k_sem_give(&wake_sem);
}

static void counter_wake_arm(void)
{
	struct counter_alarm_cfg cfg = {
		.callback = counter_wake_cb,
		.ticks = counter_us_to_ticks(counter_dev, WAKE_MS * 1000U),
		.user_data = NULL,
		/*
		 * Relative, not COUNTER_ALARM_CFG_ABSOLUTE: the ameba LP
		 * basic-timer driver only reprograms the period and rejects
		 * absolute alarms with -ENOTSUP, which would leave the alarm
		 * unarmed and the AP asleep for good.
		 */
		.flags = 0,
	};

	zassert_true(device_is_ready(counter_dev), "%s not ready", counter_dev->name);
	zassert_ok(counter_set_channel_alarm(counter_dev, 0, &cfg));
	zassert_ok(counter_start(counter_dev));
}

static void rtc_wake_cb(const struct device *dev, uint16_t id, void *user)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);
	ARG_UNUSED(user);

	k_sem_give(&wake_sem);
}

static void rtc_wake_arm(void)
{
	/* Sat Jan 01 2022 00:00:00, plus the alarm one second later. The ameba
	 * RTC alarm has a one-second granularity, so WAKE_MS is rounded there.
	 */
	struct rtc_time now = {
		.tm_sec = 0,
		.tm_min = 0,
		.tm_hour = 0,
		.tm_mday = 1,
		.tm_mon = 0,
		.tm_year = 122,
		.tm_wday = 6,
		.tm_yday = 1,
		.tm_isdst = -1,
		.tm_nsec = 0,
	};
	struct rtc_time alarm = now;
	const uint16_t mask =
		RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR;

	alarm.tm_sec = WAKE_MS / MSEC_PER_SEC;

	zassert_true(device_is_ready(rtc_dev), "%s not ready", rtc_dev->name);
	zassert_ok(rtc_set_time(rtc_dev, &now));
	/*
	 * Callback before the alarm time: rtc_alarm_set_time() is what enables the
	 * alarm interrupt (RTC_AlarmCmd(ENABLE) in the ameba driver), while
	 * rtc_alarm_set_callback() only stores the pointer. The other order leaves a
	 * window with the alarm live and no callback registered; the ameba ISR is
	 * NULL-safe there and only records the alarm as pending, so the callback
	 * would be dropped rather than the ISR misbehaving -- but there is no reason
	 * to leave the window open.
	 */
	zassert_ok(rtc_alarm_set_callback(rtc_dev, 0, rtc_wake_cb, NULL));
	zassert_ok(rtc_alarm_set_time(rtc_dev, 0, mask, &alarm));
}

/* ---------------------------------------------------------------------- tests */

/*
 * TODO: suspend-to-idle (clock-gate) is not covered yet. Forcing it by locking
 * suspend-to-RAM out of the policy leaves the AP in SOCPS_SleepCG() with no way
 * back -- the armed wake source does not bring it out and the watchdog resets
 * the chip about a second later. Every path exercised so far, here and in
 * applications/mcuboot_pm_system_manage, has gone through suspend-to-RAM, so
 * this looks untested rather than broken by something recent.
 */

#if PM_CREDITS_GATED_TIME
/*
 * The always-on reference on its own, with no power-down involved: TIM0 must be
 * running and counting at 32.768 kHz by the time the application starts. Runs
 * first and takes no measurable time, so a broken timebase is reported here
 * rather than as a confusing sleep-time failure -- and it catches the case where
 * something stopped TIM0 (an overlay enabling &timer0, a boot flow that dropped
 * SYSTIMER_Init) even where the kernel clock happens to look fine.
 */
ZTEST(realtek_pm, test_ameba_system_timebase_runs)
{
	const uint32_t wait_us = 200000U;
	uint32_t start = SYSTIMER_TickGet();
	uint32_t elapsed_ms;

	k_busy_wait(wait_us);
	elapsed_ms = (uint32_t)(((uint64_t)(SYSTIMER_TickGet() - start) * MSEC_PER_SEC) /
				AMEBA_SYSTIMER_HZ);

	/* Wide bounds on purpose: this is here to catch a stopped or reset
	 * timer, not to measure the 32 kHz oscillator's accuracy.
	 */
	zassert_between_inclusive(elapsed_ms, (wait_us / 2000U), (wait_us / 500U),
				  "the ameba system timer advanced %u ms while the CPU busy-waited "
				  "%u ms: it is stopped, reset, or not the 32.768 kHz reference "
				  "the PM code assumes",
				  elapsed_ms, wait_us / 1000U);
}
#endif

ZTEST(realtek_pm, test_suspend_to_ram_counter)
{
	counter_wake_arm();
	pm_expect_sleep_cycle(PM_STATE_SUSPEND_TO_RAM);
}

ZTEST(realtek_pm, test_suspend_to_ram_rtc)
{
	rtc_wake_arm();
	pm_expect_sleep_cycle(PM_STATE_SUSPEND_TO_RAM);
}

/*
 * Repeated power-gating: on the Cortex-M SoCs the first resume used to leave
 * both caches off for good, which stretched the next sleep's gating handshake
 * past the budget the NP allows and made it bounce the AP awake instead of
 * gating it; on AmebaSmart the secondary core has to be restarted and
 * context-restored on every warm boot. Both show up from the second cycle on,
 * never the first.
 */
ZTEST(realtek_pm, test_suspend_to_ram_repeated)
{
	for (int i = 0; i < 3; i++) {
		counter_wake_arm();
		pm_expect_sleep_cycle(PM_STATE_SUSPEND_TO_RAM);
	}
}

static void *realtek_pm_setup(void)
{
	if (boot_magic != BOOT_MAGIC) {
		boot_magic = BOOT_MAGIC;
		boot_count = 1;

		for (size_t i = 0; i < ARRAY_SIZE(ram_canary); i++) {
			ram_canary[i] = ram_canary_word(i);
		}
	} else {
		boot_count++;
	}

	cpu_state_baseline();
	pm_notifier_register(&notifier);

	/* Deep-sleep does not resume yet; keep it out of every case. */
	pm_policy_state_lock_get(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);

	return NULL;
}

ZTEST_SUITE(realtek_pm, NULL, realtek_pm_setup, NULL, NULL, NULL);

/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_a2c

#include <zephyr/drivers/can/can_sja1000.h>

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <soc.h>

LOG_MODULE_REGISTER(can_ameba_a2c, CONFIG_CAN_LOG_LEVEL);

#define A2C_TODO 0

/*
 * Newer AMEBA-series MCUs like xxxxxxxxxxxxxxxx  have some slightly different registers
 * compared to the original AMEBA, which is fully compatible with the SJA1000 controller.
 *
 * The names with A2C_ prefixes from realtek reference manuals are used for these incompatible
 * registers.
 */
#ifndef CONFIG_SOC_SERIES_AMEBA

/* A2C_BUS_TIMING_0_REG is incompatible with CAN_SJA1000_BTR0 */
#define A2C_BUS_TIMING_0_REG          (6U)
#define A2C_BAUD_PRESC_MASK           GENMASK(12, 0)
#define A2C_SYNC_JUMP_WIDTH_MASK      GENMASK(15, 14)
#define A2C_BAUD_PRESC_PREP(brp)      FIELD_PREP(A2C_BAUD_PRESC_MASK, brp)
#define A2C_SYNC_JUMP_WIDTH_PREP(sjw) FIELD_PREP(A2C_SYNC_JUMP_WIDTH_MASK, sjw)

/*
 * A2C_BUS_TIMING_1_REG is compatible with CAN_SJA1000_BTR1, but needed here for the custom
 * set_timing() function.
 */
#define A2C_BUS_TIMING_1_REG     (7U)
#define A2C_TIME_SEG1_MASK       GENMASK(3, 0)
#define A2C_TIME_SEG2_MASK       GENMASK(6, 4)
#define A2C_TIME_SAMP            BIT(7)
#define A2C_TIME_SEG1_PREP(seg1) FIELD_PREP(A2C_TIME_SEG1_MASK, seg1)
#define A2C_TIME_SEG2_PREP(seg2) FIELD_PREP(A2C_TIME_SEG2_MASK, seg2)

/* A2C_CLOCK_DIVIDER_REG is incompatible with CAN_SJA1000_CDR */
#define A2C_CLOCK_DIVIDER_REG (31U)
#define A2C_CD_MASK           GENMASK(7, 0)
#define A2C_CLOCK_OFF         BIT(8)

/*
 * Further incompatible registers currently not used by the driver:
 * - A2C_STATUS_REG has new bit 8: A2C_MISS_ST
 * - A2C_INT_RAW_REG has new bit 8: A2C_BUS_STATE_INT_ST
 * - A2C_INT_ENA_REG has new bit 8: A2C_BUS_STATE_INT_ENA
 */
#else

/* Redefinitions of the SJA1000 CDR bits to simplify driver config */
#define A2C_CD_MASK   NULL
#define A2C_CLOCK_OFF BIT(3)

#endif /* !CONFIG_SOC_SERIES_AMEBA */

struct can_ameba_a2c_config {
	mm_reg_t base;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	int irq_source;
	int irq_priority;
	int irq_flags;
};

static uint8_t can_ameba_a2c_read_reg(const struct device *dev, uint8_t reg)
{
	const struct can_sja1000_config *sja1000_config = dev->config;
	const struct can_ameba_a2c_config *a2c_config = sja1000_config->custom;
	mm_reg_t addr = a2c_config->base + reg * sizeof(uint32_t);

	return sys_read32(addr) & 0xFF;
}

static void can_ameba_a2c_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct can_sja1000_config *sja1000_config = dev->config;
	const struct can_ameba_a2c_config *a2c_config = sja1000_config->custom;
	mm_reg_t addr = a2c_config->base + reg * sizeof(uint32_t);

	sys_write32(val & 0xFF, addr);
}

#ifndef CONFIG_SOC_SERIES_AMEBA

/*
 * Required for newer AMEBA-series MCUs which violate the original SJA1000 8-bit register size.
 */
static void can_ameba_a2c_write_reg32(const struct device *dev, uint8_t reg, uint32_t val)
{
	const struct can_sja1000_config *sja1000_config = dev->config;
	const struct can_ameba_a2c_config *a2c_config = sja1000_config->custom;
	mm_reg_t addr = a2c_config->base + reg * sizeof(uint32_t);

	sys_write32(val, addr);
}

/*
 * Custom implementation instead of can_sja1000_set_timing required because A2C_BUS_TIMING_0_REG
 * is incompatible with CAN_SJA1000_BTR0.
 */
static int can_ameba_a2c_set_timing(const struct device *dev, const struct can_timing *timing)
{
	struct can_sja1000_data *data = dev->data;
	uint8_t btr0;
	uint8_t btr1;

	if (data->common.started) {
		return -EBUSY;
	}

	k_mutex_lock(&data->mod_lock, K_FOREVER);

	btr0 = A2C_BAUD_PRESC_PREP(timing->prescaler - 1) |
	       A2C_SYNC_JUMP_WIDTH_PREP(timing->sjw - 1);
	btr1 = A2C_TIME_SEG1_PREP(timing->phase_seg1 - 1) |
	       A2C_TIME_SEG2_PREP(timing->phase_seg2 - 1);

	if ((data->common.mode & CAN_MODE_3_SAMPLES) != 0) {
		btr1 |= A2C_TIME_SAMP;
	}

	can_ameba_a2c_write_reg32(dev, A2C_BUS_TIMING_0_REG, btr0);
	can_ameba_a2c_write_reg32(dev, A2C_BUS_TIMING_1_REG, btr1);

	k_mutex_unlock(&data->mod_lock);

	return 0;
}

#endif /* !CONFIG_SOC_SERIES_AMEBA */

static int can_ameba_a2c_get_core_clock(const struct device *dev, uint32_t *rate)
{
	ARG_UNUSED(dev);

	return 0;
}

#if A2C_TODO
static void can_ameba_a2c_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;

	can_sja1000_isr(dev);
}
#endif

static int can_ameba_a2c_init(const struct device *dev)
{

	const struct can_sja1000_config *sja1000_config = dev->config;
	const struct can_ameba_a2c_config *a2c_config = sja1000_config->custom;
	int err;

	if (!device_is_ready(a2c_config->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	err = pinctrl_apply_state(a2c_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		LOG_ERR("failed to configure A2C pins (err %d)", err);
		return err;
	}

	err = clock_control_on(a2c_config->clock_dev, a2c_config->clock_subsys);
	if (err != 0) {
		LOG_ERR("failed to enable CAN clock (err %d)", err);
		return err;
	}

	err = can_sja1000_init(dev);
	if (err != 0) {
		LOG_ERR("failed to initialize controller (err %d)", err);
		return err;
	}

	return err;
}

DEVICE_API(can, can_ameba_a2c_driver_api) = {
	.get_capabilities = can_sja1000_get_capabilities,
	.start = can_sja1000_start,
	.stop = can_sja1000_stop,
	.set_mode = can_sja1000_set_mode,
#ifdef CONFIG_SOC_SERIES_AMEBA
	.set_timing = can_sja1000_set_timing,
#else
	.set_timing = can_ameba_a2c_set_timing,
#endif /* CONFIG_SOC_SERIES_AMEBA */
	.send = can_sja1000_send,
	.add_rx_filter = can_sja1000_add_rx_filter,
	.remove_rx_filter = can_sja1000_remove_rx_filter,
	.get_state = can_sja1000_get_state,
	.set_state_change_callback = can_sja1000_set_state_change_callback,
	.get_core_clock = can_ameba_a2c_get_core_clock,
	.get_max_filters = can_sja1000_get_max_filters,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	.recover = can_sja1000_recover,
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */
	.timing_min = CAN_SJA1000_TIMING_MIN_INITIALIZER,
};

#ifdef CONFIG_SOC_SERIES_AMEBA
#define A2C_CLKOUT_DIVIDER_MAX (14)
#define A2C_CDR32_INIT(inst)
#else
#define A2C_CLKOUT_DIVIDER_MAX (490)
#define A2C_CDR32_INIT(inst)   .cdr32 = CAN_AMEBA_A2C_DT_CDR_INST_GET(inst)
#endif /* CONFIG_SOC_SERIES_AMEBA */

#define CAN_AMEBA_A2C_DT_CDR_INST_GET(inst)                                                        \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, clkout_divider),                                   \
		    COND_CODE_1(DT_INST_PROP(inst, clkout_divider) == 1, (A2C_CD_MASK),           \
				((DT_INST_PROP(inst, clkout_divider)) / 2 - 1)),                   \
		    (A2C_CLOCK_OFF))

#define CAN_AMEBA_A2C_INIT(inst)                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static const struct can_ameba_a2c_config can_ameba_a2c_config_##inst = {                   \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
	};                                                                                         \
	static const struct can_sja1000_config can_sja1000_config_##inst =                         \
		CAN_SJA1000_DT_CONFIG_INST_GET(                                                    \
			inst, &can_ameba_a2c_config_##inst, can_ameba_a2c_read_reg,                \
			can_ameba_a2c_write_reg, CAN_SJA1000_OCR_OCMODE_BIPHASE,                   \
			COND_CODE_0(IS_ENABLED(CONFIG_SOC_SERIES_AMEBA), (0),      \
					(CAN_AMEBA_A2C_DT_CDR_INST_GET(inst))), 25000); \
                                                                                                   \
	static struct can_sja1000_data can_sja1000_data_##inst =                                   \
		CAN_SJA1000_DATA_INITIALIZER(NULL);                                                \
                                                                                                   \
	CAN_DEVICE_DT_INST_DEFINE(inst, can_ameba_a2c_init, NULL, &can_sja1000_data_##inst,        \
				  &can_sja1000_config_##inst, POST_KERNEL,                         \
				  CONFIG_CAN_INIT_PRIORITY, &can_ameba_a2c_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CAN_AMEBA_A2C_INIT)

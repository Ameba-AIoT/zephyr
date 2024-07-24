/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_pwm

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm_ameba, CONFIG_PWM_LOG_LEVEL);

#ifdef CONFIG_PWM_CAPTURE
#define SKIP_IRQ_NUM 4U
#define PWM_INTR_CAP0  BIT(0)
#define PWM_INTR_CAP1  BIT(1)
#define PWM_INTR_CAP2  BIT(2)
#define PWM_CHANNEL_NUM   8U
#define CAPTURE_CHANNEL_IDX 6U
#else
#define PWM_CHANNEL_NUM 0U
#endif /* CONFIG_PWM_CAPTURE */

struct pwm_ameba_data {
	struct k_sem cmd_sem;
};

#ifdef CONFIG_PWM_CAPTURE
struct capture_data {
	uint32_t value;
};

struct pwm_ameba_capture_config {
	uint8_t capture_signal;
	pwm_capture_callback_handler_t callback;
	void *user_data;
	uint32_t period;
	uint32_t pulse;
	uint32_t overflows;
	uint8_t skip_irq;
	bool capture_period;
	bool capture_pulse;
	bool continuous;
	struct capture_data capture_data[SKIP_IRQ_NUM];
};
#endif /* CONFIG_PWM_CAPTURE */

struct pwm_ameba_channel_config {
	uint8_t idx;
	uint8_t timer_id;
	uint8_t operator_id;
	uint8_t generator_id;
	uint32_t freq;
	uint32_t duty;
	uint8_t prescale;
	bool inverted;
#ifdef CONFIG_PWM_CAPTURE
	struct pwm_ameba_capture_config capture;
#endif /* CONFIG_PWM_CAPTURE */
};

struct pwm_ameba_config {
	RTIM_TypeDef *dev;
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	uint8_t prescale;
	uint8_t prescale_timer0;
	uint8_t prescale_timer1;
	uint8_t prescale_timer2;
	struct pwm_ameba_channel_config channel_config[PWM_CHANNEL_NUM];
#ifdef CONFIG_PWM_CAPTURE
	void (*irq_config_func)(const struct device *dev);
#endif /* CONFIG_PWM_CAPTURE */
};

static void pwm_ameba_duty_set(const struct device *dev,
							   struct pwm_ameba_channel_config *channel)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);
}

static int pwm_ameba_configure_pinctrl(const struct device *dev)
{
	int ret;
	struct pwm_ameba_config *config = (struct pwm_ameba_config *)dev->config;

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("PWM pinctrl setup failed (%d)", ret);
		return ret;
	}
	return 0;
}

static int pwm_ameba_timer_set(const struct device *dev,
							   struct pwm_ameba_channel_config *channel)
{
	struct pwm_ameba_data *data = (struct pwm_ameba_data * const)(dev)->data;

	__ASSERT_NO_MSG(channel->freq > 0);

	ARG_UNUSED(data);

	return 0;
}

static int pwm_ameba_get_cycles_per_sec(const struct device *dev, uint32_t channel_idx,
										uint64_t *cycles)
{
	struct pwm_ameba_config *config = (struct pwm_ameba_config *)dev->config;
	struct pwm_ameba_channel_config *channel = &config->channel_config[channel_idx];

	if (!channel) {
		LOG_ERR("Error getting channel %d", channel_idx);
		return -EINVAL;
	}

#ifdef CONFIG_PWM_CAPTURE
	if (channel->idx >= CAPTURE_CHANNEL_IDX) {
		*cycles = (uint64_t)APB_CLK_FREQ;
		return 0;
	}
#endif /* CONFIG_PWM_CAPTURE */

	*cycles = (uint64_t)0 / (config->prescale + 1) / (channel->prescale + 1);

	return 0;
}

static int pwm_ameba_set_cycles(const struct device *dev, uint32_t channel_idx,
								uint32_t period_cycles, uint32_t pulse_cycles, pwm_flags_t flags)
{
	int ret = 0;
	uint64_t clk_freq;
	struct pwm_ameba_config *config = (struct pwm_ameba_config *)dev->config;
	struct pwm_ameba_data *data = (struct pwm_ameba_data * const)(dev)->data;
	struct pwm_ameba_channel_config *channel = &config->channel_config[channel_idx];

	if (!channel) {
		LOG_ERR("Error getting channel %d", channel_idx);
		return -EINVAL;
	}

	/* Update PWM frequency according to period_cycles */
	pwm_ameba_get_cycles_per_sec(dev, channel_idx, &clk_freq);

	channel->freq = (uint32_t)(clk_freq / period_cycles);
	if (!channel->freq) {
		return -EINVAL;
	}

	k_sem_take(&data->cmd_sem, K_FOREVER);

	ret = pwm_ameba_timer_set(dev, channel);
	if (ret < 0) {
		k_sem_give(&data->cmd_sem);
		return ret;
	}

	double duty_cycle = (double)pulse_cycles * 100 / (double)period_cycles;

	channel->duty = (uint32_t)duty_cycle;

	channel->inverted = (flags & PWM_POLARITY_INVERTED);

	pwm_ameba_duty_set(dev, channel);

	ret = pwm_ameba_configure_pinctrl(dev);
	if (ret < 0) {
		k_sem_give(&data->cmd_sem);
		return ret;
	}

	k_sem_give(&data->cmd_sem);

	return ret;
}

#ifdef CONFIG_PWM_CAPTURE
static int pwm_ameba_configure_capture(const struct device *dev, uint32_t channel_idx,
									   pwm_flags_t flags, pwm_capture_callback_handler_t cb,
									   void *user_data)
{
	struct pwm_ameba_config *config = (struct pwm_ameba_config *)dev->config;
	struct pwm_ameba_data *data = (struct pwm_ameba_data * const)(dev)->data;
	struct pwm_ameba_channel_config *channel = &config->channel_config[channel_idx];
	struct pwm_ameba_capture_config *capture = &channel->capture;

	if (!channel) {
		LOG_ERR("Error getting channel %d", channel_idx);
		return -EINVAL;
	}

	if ((channel->idx < CAPTURE_CHANNEL_IDX) || (channel->idx > CAPTURE_CHANNEL_IDX + 2)) {
		LOG_ERR("PWM capture only supported on channels 6, 7 and 8");
		return -EINVAL;
	}

	if (data->hal.dev->cap_chn_cfg[capture->capture_signal].capn_en) {
		LOG_ERR("PWM Capture already in progress");
		return -EBUSY;
	}

	if (!(flags & PWM_CAPTURE_TYPE_MASK)) {
		LOG_ERR("No PWM capture type specified");
		return -EINVAL;
	}

	channel->inverted = (flags & PWM_POLARITY_INVERTED);
	capture->capture_signal = channel->idx - CAPTURE_CHANNEL_IDX;
	capture->callback = cb;
	capture->user_data = user_data;
	capture->capture_period = (flags & PWM_CAPTURE_TYPE_PERIOD);
	capture->capture_pulse = (flags & PWM_CAPTURE_TYPE_PULSE);
	capture->continuous = (flags & PWM_CAPTURE_MODE_CONTINUOUS);

	return 0;
}

static int pwm_ameba_disable_capture(const struct device *dev, uint32_t channel_idx)
{
	struct pwm_ameba_config *config = (struct pwm_ameba_config *)dev->config;
	struct pwm_ameba_data *data = (struct pwm_ameba_data * const)(dev)->data;
	struct pwm_ameba_channel_config *channel = &config->channel_config[channel_idx];
	struct pwm_ameba_capture_config *capture = &channel->capture;

	if (!channel) {
		LOG_ERR("Error getting channel %d", channel_idx);
		return -EINVAL;
	}

	if ((channel->idx < CAPTURE_CHANNEL_IDX) || (channel->idx > CAPTURE_CHANNEL_IDX + 2)) {
		LOG_ERR("PWM capture only supported on channels 6, 7 and 8");
		return -EINVAL;
	}

	return 0;
}

static int pwm_ameba_enable_capture(const struct device *dev, uint32_t channel_idx)
{
	struct pwm_ameba_config *config = (struct pwm_ameba_config *)dev->config;
	struct pwm_ameba_data *data = (struct pwm_ameba_data * const)(dev)->data;
	struct pwm_ameba_channel_config *channel = &config->channel_config[channel_idx];
	struct pwm_ameba_capture_config *capture = &channel->capture;

	if (!channel) {
		LOG_ERR("Error getting channel %d", channel_idx);
		return -EINVAL;
	}

	if (!capture->callback) {
		LOG_ERR("Capture not configured");
		return -EINVAL;
	}

	if ((channel->idx < CAPTURE_CHANNEL_IDX) || (channel->idx > CAPTURE_CHANNEL_IDX + 2)) {
		LOG_ERR("PWM capture only supported on channels 6, 7 and 8");
		return -EINVAL;
	}

	if (data->hal.dev->cap_chn_cfg[capture->capture_signal].capn_en) {
		LOG_ERR("PWM Capture already in progress");
		return -EBUSY;
	}

	capture->skip_irq = 0;

	return 0;
}
#endif /* CONFIG_PWM_CAPTURE */

int pwm_ameba_init(const struct device *dev)
{
	int ret;
	struct pwm_ameba_config *config = (struct pwm_ameba_config *)dev->config;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Enable peripheral */
	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("Could not initialize clock (%d)", ret);
		return ret;
	}

#ifdef CONFIG_PWM_CAPTURE
	config->irq_config_func(dev);
#endif /* CONFIG_PWM_CAPTURE */
	return 0;
}

#ifdef CONFIG_PWM_CAPTURE
static void pwm_ameba_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
}
#endif /* CONFIG_PWM_CAPTURE */

static const struct pwm_driver_api pwm_ameba_api = {
	.set_cycles = pwm_ameba_set_cycles,
	.get_cycles_per_sec = pwm_ameba_get_cycles_per_sec,
#ifdef CONFIG_PWM_CAPTURE
	.configure_capture = pwm_ameba_configure_capture,
	.enable_capture = pwm_ameba_enable_capture,
	.disable_capture = pwm_ameba_disable_capture,
#endif /* CONFIG_PWM_CAPTURE */
};

#ifdef CONFIG_PWM_CAPTURE
#define IRQ_CONFIG_FUNC(n)                                                      \
	static void pwm_ameba_irq_config_func_##n(const struct device *dev)         \
	{																			\
        IRQ_CONNECT(DT_INST_IRQN(n),        									\
                    DT_INST_IRQ(n, priority),       							\
                    pwm_ameba_isr,                   							\
                    DEVICE_DT_INST_GET(n),              						\
                    0);                         								\
        irq_enable(DT_INST_IRQN(n));        									\
	}
#define CAPTURE_INIT(n) .irq_config_func = pwm_ameba_irq_config_func_##n
#else
#define IRQ_CONFIG_FUNC(n)
#define CAPTURE_INIT(n)
#endif /* CONFIG_PWM_CAPTURE */

#define AMEBA_PWM_INIT(n)                                                                \
	PINCTRL_DT_INST_DEFINE(n);                                                           \
	IRQ_CONFIG_FUNC(n);                                                                  \
	static struct pwm_ameba_data pwm_ameba_data_##n = {                                  \
		.cmd_sem = Z_SEM_INITIALIZER(pwm_ameba_data_##n.cmd_sem, 1, 1),               	 \
	};                                                                                   \
                                                                                 	     \
	static const struct pwm_ameba_config pwm_ameba_config_##n = {                        \
		.dev = (RTIM_TypeDef *)DT_INST_REG_ADDR(n),                     		  		 \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                     \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                              \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, idx),      	     \
		.prescale = DT_INST_PROP(n, prescale),                                           \
		CAPTURE_INIT(n)};                                                                \
                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &pwm_ameba_init, NULL, &pwm_ameba_data_##n,               	 \
			      &pwm_ameba_config_##n, POST_KERNEL,                              		 \
			      CONFIG_PWM_INIT_PRIORITY, &pwm_ameba_api);

DT_INST_FOREACH_STATUS_OKAY(AMEBA_PWM_INIT)

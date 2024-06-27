/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_counter

#include <string.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/spinlock.h>
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ameba_counter, CONFIG_COUNTER_LOG_LEVEL);

#if 0
static void counter_ameba_isr(void *arg);
#endif

typedef bool (*timer_isr_t)(void *);

struct timer_isr_func_t {
	void *args;
	struct intr_handle_data_t *timer_isr_handle;
};

struct counter_ameba_config {
	struct counter_config_info counter_info;
	int irq_source;
};

struct counter_ameba_data {
	struct counter_alarm_cfg alarm_cfg;
	uint32_t ticks;
	struct timer_isr_func_t timer_isr_fun;
};

static struct k_spinlock lock;

static int counter_ameba_init(const struct device *dev)
{
	const struct counter_ameba_config *cfg = dev->config;
	struct counter_ameba_data *data = dev->data;
	ARG_UNUSED(data);
	ARG_UNUSED(cfg);
	k_spinlock_key_t key = k_spin_lock(&lock);



	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ameba_start(const struct device *dev)
{
	struct counter_ameba_data *data = dev->data;
	ARG_UNUSED(data);
	k_spinlock_key_t key = k_spin_lock(&lock);

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ameba_stop(const struct device *dev)
{
	struct counter_ameba_data *data = dev->data;
	ARG_UNUSED(data);
	k_spinlock_key_t key = k_spin_lock(&lock);

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ameba_get_value(const struct device *dev, uint32_t *ticks)
{
	struct counter_ameba_data *data = dev->data;
	ARG_UNUSED(data);
	k_spinlock_key_t key = k_spin_lock(&lock);

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ameba_set_alarm(const struct device *dev, uint8_t chan_id,
								   const struct counter_alarm_cfg *alarm_cfg)
{
	ARG_UNUSED(chan_id);
	struct counter_ameba_data *data = dev->data;
	uint32_t now;
	ARG_UNUSED(data);
	counter_ameba_get_value(dev, &now);

	k_spinlock_key_t key = k_spin_lock(&lock);



	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ameba_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	ARG_UNUSED(chan_id);
	struct counter_ameba_data *data = dev->data;
	ARG_UNUSED(data);
	k_spinlock_key_t key = k_spin_lock(&lock);

	k_spin_unlock(&lock, key);

	return 0;
}

static int counter_ameba_set_top_value(const struct device *dev,
									   const struct counter_top_cfg *cfg)
{
	const struct counter_ameba_config *config = dev->config;

	if (cfg->ticks != config->counter_info.max_top_value) {
		return -ENOTSUP;
	} else {
		return 0;
	}
}

static uint32_t counter_ameba_get_pending_int(const struct device *dev)
{
	struct counter_ameba_data *data = dev->data;
	ARG_UNUSED(data);

	return 0;
}

static uint32_t counter_ameba_get_top_value(const struct device *dev)
{
	const struct counter_ameba_config *config = dev->config;

	return config->counter_info.max_top_value;
}

static const struct counter_driver_api counter_api = {
	.start = counter_ameba_start,
	.stop = counter_ameba_stop,
	.get_value = counter_ameba_get_value,
	.set_alarm = counter_ameba_set_alarm,
	.cancel_alarm = counter_ameba_cancel_alarm,
	.set_top_value = counter_ameba_set_top_value,
	.get_pending_int = counter_ameba_get_pending_int,
	.get_top_value = counter_ameba_get_top_value,
};

#if 0
static void counter_ameba_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct counter_ameba_data *data = dev->data;
	ARG_UNUSED(data);
}
#endif

#define AMEBA_COUNTER_INIT(idx)			 \
										 \
	static struct counter_ameba_data counter_data_##idx;			 \
										 \
	static const struct counter_ameba_config counter_config_##idx = {	 \
		.counter_info = {						 \
			.max_top_value = UINT32_MAX,				 \
			.flags = COUNTER_CONFIG_INFO_COUNT_UP,			 \
			.channels = 1						 \
		},								 \
		.irq_source = DT_INST_IRQN(idx),				 \
	};									 \
										 \
										 \
	DEVICE_DT_INST_DEFINE(idx,						 \
			      counter_ameba_init,				 \
			      NULL,						 \
			      &counter_data_##idx,				 \
			      &counter_config_##idx,				 \
			      PRE_KERNEL_1,					 \
			      CONFIG_COUNTER_INIT_PRIORITY,			 \
			      &counter_api);

DT_INST_FOREACH_STATUS_OKAY(AMEBA_COUNTER_INIT);

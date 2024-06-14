/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_gpio

#include "ameba_soc.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <errno.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_ameba, CONFIG_GPIO_LOG_LEVEL);

#define GPIO_PINNAME(PORT, PIN)			(((PORT) << 5) | ((PIN) & 0x1F))

#define GPIO_DEBOUNCE_EN 0
/* DivideCount: debounce clock division with 32KHz.range: 0x0 - 0x7F.
 *              debounce time = (DivideCount +1) * 2 * 32μs.
 */
#define DEBOUNCE_DIV_CNT 0x00

struct gpio_ameba_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* port base address */
	uint32_t base;
	/* IO port */
	int port_num;
};

struct gpio_ameba_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* port ISR callback routine address */
	sys_slist_t callbacks;
};

static int gpio_ameba_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_ameba_config *cfg = dev->config;

	*value = GPIO_PortRead(cfg->port_num, cfg->common.port_pin_mask);

	return 0;
}

static int gpio_ameba_port_set_masked_raw(const struct device *dev,
		uint32_t mask, uint32_t value)
{
	const struct gpio_ameba_config *cfg = dev->config;

	GPIO_PortDirection(cfg->port_num, mask, GPIO_Mode_OUT);
	GPIO_PortWrite(cfg->port_num, mask, value);

	return 0;
}

static int gpio_ameba_port_set_bits_raw(const struct device *dev,
										uint32_t mask)
{
	const struct gpio_ameba_config *cfg = dev->config;

	GPIO_PortWrite(cfg->port_num, mask, cfg->common.port_pin_mask);

	return 0;
}

static int gpio_ameba_port_clear_bits_raw(const struct device *dev,
		uint32_t mask)
{
	const struct gpio_ameba_config *cfg = dev->config;

	GPIO_PortWrite(cfg->port_num, mask, 0);

	return 0;
}

static int gpio_ameba_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	const struct gpio_ameba_config *cfg = dev->config;
	u32 gpio_pin;
	u32 value;
	uint8_t i;

	for (i = 0; i < 32; i++) {
		if (mask & 0x1) {
			gpio_pin = GPIO_PINNAME(cfg->port_num, i);
			value = GPIO_ReadDataBit(gpio_pin);
			GPIO_WriteBit(gpio_pin, (~value) & 0x1);
		}
		mask >>= 1;
		if (mask == 0) {
			break;
		}
	}

	return 0;
}

static int gpio_ameba_configure(const struct device *dev,
								gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_ameba_config *cfg = dev->config;

	GPIO_InitTypeDef GPIO_InitStruct_Temp;
	u32 gpio_pin;

	if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0)) {
		return -ENOTSUP;
	}

	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == 0) {
		return -ENOTSUP;
	}

	gpio_pin = GPIO_PINNAME(cfg->port_num, pin);
	GPIO_InitStruct_Temp.GPIO_Pin = gpio_pin;

	if (flags & GPIO_INPUT) {
		GPIO_InitStruct_Temp.GPIO_Mode = GPIO_Mode_IN;
	} else {
		GPIO_InitStruct_Temp.GPIO_Mode = GPIO_Mode_OUT;
	}

	if (flags & GPIO_PULL_UP) {
		GPIO_InitStruct_Temp.GPIO_PuPd = GPIO_PuPd_UP;
	} else if (flags & GPIO_PULL_DOWN) {
		GPIO_InitStruct_Temp.GPIO_PuPd = GPIO_PuPd_DOWN;
	} else {
		GPIO_InitStruct_Temp.GPIO_PuPd = GPIO_PuPd_NOPULL;
	}

	GPIO_Init(&GPIO_InitStruct_Temp);

	if (flags & GPIO_OUTPUT) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			gpio_ameba_port_set_bits_raw(dev, BIT(pin));
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			gpio_ameba_port_clear_bits_raw(dev, BIT(pin));
		}
	}

	return 0;
}

static int gpio_ameba_pin_interrupt_configure(const struct device *dev,
		gpio_pin_t pin, enum gpio_int_mode mode,
		enum gpio_int_trig trig)
{
	const struct gpio_ameba_config *cfg = dev->config;
	u32 gpio_pin;

	gpio_pin = GPIO_PINNAME(cfg->port_num, pin);
	GPIO_InitTypeDef GPIO_InitStruct_Temp;
	GPIO_InitStruct_Temp.GPIO_Pin = gpio_pin;

	GPIO_INTConfig(gpio_pin, DISABLE);

	LOG_DBG("Config GPIO int:%d-%d, mode:%x, flag:0x%x\n", cfg->port_num, pin, mode, trig);
	GPIO_InitStruct_Temp.GPIO_Pin = gpio_pin;
	GPIO_InitStruct_Temp.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct_Temp.GPIO_Mode = GPIO_Mode_INT;

	if (mode != GPIO_INT_MODE_DISABLED) {
		if (mode & GPIO_INT_MODE_EDGE) {
			switch (trig) {
			case GPIO_INT_TRIG_LOW:
				GPIO_InitStruct_Temp.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
				GPIO_InitStruct_Temp.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
				break;
			case GPIO_INT_TRIG_HIGH:
				GPIO_InitStruct_Temp.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
				GPIO_InitStruct_Temp.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_HIGH;
				break;
			case GPIO_INT_TRIG_BOTH:
				GPIO_InitStruct_Temp.GPIO_ITTrigger = GPIO_INT_Trigger_BOTHEDGE;
				break;
			}
		} else {
			GPIO_InitStruct_Temp.GPIO_ITTrigger = GPIO_INT_Trigger_LEVEL;
			switch (trig) {
			case GPIO_INT_TRIG_LOW:
				GPIO_InitStruct_Temp.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
				break;
			case GPIO_INT_TRIG_HIGH:
				GPIO_InitStruct_Temp.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_HIGH;
				break;
			default:
				LOG_ERR("GPIO level interrupt doesn't support both high and low");
				break;
			}
		}
	}
	if (GPIO_DEBOUNCE_EN) {
		GPIO_InitStruct_Temp.GPIO_ITDebounce = 1;
		GPIO_DebounceClock(cfg->port_num, DEBOUNCE_DIV_CNT);
	}
	GPIO_Init(&GPIO_InitStruct_Temp);
	GPIO_INTConfig(gpio_pin, ENABLE);

	return 0;
}

static int gpio_ameba_manage_callback(const struct device *dev,
									  struct gpio_callback *callback,
									  bool set)
{
	struct gpio_ameba_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static uint32_t gpio_ameba_get_pending_int(const struct device *dev)
{
	uint32_t irq_status;
	const struct gpio_ameba_config *cfg = dev->config;
	GPIO_TypeDef *gpio_base = (GPIO_TypeDef *)cfg->base;

	irq_status = gpio_base->GPIO_INT_STATUS;

	return irq_status;
}

static void gpio_ameba_isr(const struct device *dev)
{
	uint32_t int_status;
	struct gpio_ameba_data *data = dev->data;
	const struct gpio_ameba_config *cfg = dev->config;
	GPIO_TypeDef *gpio_base = (GPIO_TypeDef *)cfg->base;

	/* Get the int status  */
	int_status = gpio_base->GPIO_INT_STATUS;
	/* Clear pending edge interrupt */
	gpio_base->GPIO_INT_EOI = int_status;
	/* Call the registered callbacks */
	gpio_fire_callbacks(&data->callbacks, dev, int_status);
}

static const struct gpio_driver_api gpio_ameba_driver_api = {
	.pin_configure = gpio_ameba_configure,
	.port_get_raw = gpio_ameba_port_get_raw,
	.port_set_masked_raw = gpio_ameba_port_set_masked_raw,
	.port_set_bits_raw = gpio_ameba_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ameba_port_clear_bits_raw,
	.port_toggle_bits = gpio_ameba_port_toggle_bits,
	.pin_interrupt_configure = gpio_ameba_pin_interrupt_configure,
	.manage_callback = gpio_ameba_manage_callback,
	.get_pending_int = gpio_ameba_get_pending_int,
};

#define GPIO_AMEBA_INIT(n)						\
	static int gpio_ameba_port##n##_init(const struct device *dev) \
	{								\
		const struct gpio_ameba_config *cfg = dev->config;\
		LOG_INF("GPIO%d INIT, IRQ %d\n", cfg->port_num, DT_INST_IRQN(n));\
										\
		IRQ_CONNECT(DT_INST_IRQN(n),	\
			DT_INST_IRQ(n, priority),    \
			gpio_ameba_isr, \
			DEVICE_DT_INST_GET(n),\
			 0);		\
		irq_enable(DT_INST_IRQN(n));				\
												\
		return 0;						\
	}		\
	static struct gpio_ameba_data gpio_ameba_port##n##_data;	\
									\
	static const struct gpio_ameba_config gpio_ameba_port##n##_config = {\
		.common = {						\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),\
		},							\
		.base = DT_INST_REG_ADDR(n),			\
		.port_num = n,	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n,	\
				  gpio_ameba_port##n##_init, 	\
			      NULL,					\
			      &gpio_ameba_port##n##_data,		\
			      &gpio_ameba_port##n##_config,		\
			      POST_KERNEL,				\
			      CONFIG_GPIO_INIT_PRIORITY,		\
			      &gpio_ameba_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_AMEBA_INIT)


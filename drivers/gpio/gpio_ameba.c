/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_gpio

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_ameba, CONFIG_GPIO_LOG_LEVEL);

#define GPIO_PINNAME(PORT, PIN) (((PORT) << 5) | ((PIN) & 0x1F))

struct gpio_ameba_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* port base address */
	uint32_t base;
	/* IO port */
	int port;
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

	*value = GPIO_PortRead(cfg->port, cfg->common.port_pin_mask);

	return 0;
}

static int gpio_ameba_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value)
{
	const struct gpio_ameba_config *cfg = dev->config;

	/* GPIO_PortDirection(cfg->port, mask, GPIO_Mode_OUT); */

	GPIO_PortWrite(cfg->port, mask, value);

	return 0;
}

static int gpio_ameba_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_ameba_config *cfg = dev->config;

	GPIO_PortWrite(cfg->port, mask, cfg->common.port_pin_mask);

	return 0;
}

static int gpio_ameba_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_ameba_config *cfg = dev->config;

	GPIO_PortWrite(cfg->port, mask, 0);

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
			gpio_pin = GPIO_PINNAME(cfg->port, i);
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

static int gpio_ameba_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_ameba_config *cfg = dev->config;

	GPIO_InitTypeDef gpio_initstruct;
	u32 gpio_pin;

	if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0)) {
		return -ENOTSUP;
	}

	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == 0) {
		return -ENOTSUP;
	}

	gpio_pin = GPIO_PINNAME(cfg->port, pin);
	gpio_initstruct.GPIO_Pin = gpio_pin;

	if (flags & GPIO_INPUT) {
		gpio_initstruct.GPIO_Mode = GPIO_Mode_IN;
	} else {
		gpio_initstruct.GPIO_Mode = GPIO_Mode_OUT;
	}

	if (flags & GPIO_PULL_UP) {
		gpio_initstruct.GPIO_PuPd = GPIO_PuPd_UP;
	} else if (flags & GPIO_PULL_DOWN) {
		gpio_initstruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	} else {
		gpio_initstruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	}

	GPIO_Init(&gpio_initstruct);

	if (flags & GPIO_OUTPUT) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			gpio_ameba_port_set_bits_raw(dev, BIT(pin));
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			gpio_ameba_port_clear_bits_raw(dev, BIT(pin));
		}
	}

	return 0;
}

#ifdef CONFIG_GPIO_GET_CONFIG
static int gpio_ameba_get_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t *out_flags)
{
	gpio_flags_t flag = 0;

	const struct gpio_ameba_config *cfg = dev->config;
	int port = cfg->port;

	uint32_t pin_name = GPIO_PINNAME(port, pin);

	uint32_t dir = GPIO_DirectionGet(port, BIT(pin));
	uint32_t out_val = GPIO_ReadDataBit(pin_name);

	if (dir == GPIO_Mode_OUT) {
		flag |= GPIO_OUTPUT;

		if (out_val == GPIO_PIN_HIGH) {
			flag |= GPIO_OUTPUT_HIGH;
		} else {
			flag |= GPIO_OUTPUT_LOW;
		}

	} else {
		flag |= GPIO_INPUT;
		uint32_t pull_type = PAD_PullCtrlGet(pin_name);

		if (pull_type == GPIO_PuPd_UP) {
			flag |= GPIO_PULL_UP;
		} else if (pull_type == GPIO_PuPd_DOWN) {
			flag |= GPIO_PULL_DOWN;
		}
		/* flag |= GPIO_NO_PULL; */
	}

	*out_flags = flag;

	return 0;
}

#endif

#ifdef CONFIG_GPIO_GET_DIRECTION
int gpio_ameba_get_direction(const struct device *dev, gpio_port_pins_t map,
			     gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
	const struct gpio_ameba_config *cfg = dev->config;
	int port = cfg->port;
	gpio_port_pins_t ip = 0;
	gpio_port_pins_t op = 0;

	map &= cfg->common.port_pin_mask;

	for (uint8_t pin = 0; pin < 32; pin++) {
		/* check pin whether exists in pin_mask */
		if (map & BIT(pin)) {
			uint32_t pin_mask = BIT(pin);

			/* get pin direction */
			uint32_t dir = GPIO_DirectionGet(port, pin_mask);

			/* update dir for inputs and outputs */
			if (dir == GPIO_Mode_OUT) {
				op |= pin_mask;
			} else {
				ip |= pin_mask;
			}
		}
	}

	*inputs = ip;
	*outputs = op;

	return 0;
}
#endif

static int gpio_ameba_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					      enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_ameba_config *cfg = dev->config;
	u32 gpio_pin;

	gpio_pin = GPIO_PINNAME(cfg->port, pin);
	GPIO_InitTypeDef gpio_initstruct;

	gpio_initstruct.GPIO_Pin = gpio_pin;

	GPIO_INTConfig(gpio_pin, DISABLE);

	LOG_DBG("Config GPIO int:%d-%d, mode:%x, flag:0x%x\n", cfg->port, pin, mode, trig);

	gpio_initstruct.GPIO_Pin = gpio_pin;
	gpio_initstruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_initstruct.GPIO_Mode = GPIO_Mode_INT;
	gpio_initstruct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_DISABLE;

	if (mode != GPIO_INT_MODE_DISABLED) {
		if (mode == GPIO_INT_MODE_EDGE) {
			switch (trig) {
			case GPIO_INT_TRIG_LOW:
				gpio_initstruct.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
				gpio_initstruct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
				gpio_initstruct.GPIO_PuPd = GPIO_PuPd_UP;
				break;
			case GPIO_INT_TRIG_HIGH:
				gpio_initstruct.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
				gpio_initstruct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_HIGH;
				gpio_initstruct.GPIO_PuPd = GPIO_PuPd_DOWN;
				break;
			case GPIO_INT_TRIG_BOTH:
				gpio_initstruct.GPIO_ITTrigger = GPIO_INT_Trigger_BOTHEDGE;
				break;
			default:
				LOG_ERR("GPIO Edge interrupt type invalid \r\n");
				return -ENOTSUP;
			}
		} else {
			gpio_initstruct.GPIO_ITTrigger = GPIO_INT_Trigger_LEVEL;
			switch (trig) {
			case GPIO_INT_TRIG_LOW:
				gpio_initstruct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
				gpio_initstruct.GPIO_PuPd = GPIO_PuPd_UP;
				break;
			case GPIO_INT_TRIG_HIGH:
				gpio_initstruct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_HIGH;
				gpio_initstruct.GPIO_PuPd = GPIO_PuPd_DOWN;
				break;
			default:
				LOG_ERR("GPIO level interrupt doesn't support both high and low");
				return -ENOTSUP;
			}
		}

#if defined(CONFIG_GPIO_DEBOUNCE_EN)
		gpio_initstruct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
		/* GPIO_DebounceClock(cfg->port, DEBOUNCE_DIV_CNT); */
		GPIO_Init(&gpio_initstruct);
		DelayUs(64);
		GPIO_INTConfig(gpio_pin, ENABLE);
#else
		GPIO_Init(&gpio_initstruct);
		GPIO_INTConfig(gpio_pin, ENABLE);
#endif
	} else {
		GPIO_Direction(gpio_pin, GPIO_Mode_IN);
		PAD_PullCtrl(gpio_pin, gpio_initstruct.GPIO_PuPd);

		GPIO_INTMode(gpio_pin, DISABLE, 0, 0, 0);
	}

	return 0;
}

static int gpio_ameba_manage_callback(const struct device *dev, struct gpio_callback *callback,
				      bool set)
{
	struct gpio_ameba_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static uint32_t gpio_ameba_get_pending_int(const struct device *dev)
{
	uint32_t irq_status;
	const struct gpio_ameba_config *cfg = dev->config;
	uint32_t port = cfg->port;

	irq_status = GPIO_INTStatusGet(port);

	return irq_status;
}

static void gpio_ameba_isr(const struct device *dev)
{
	uint32_t int_status;
	struct gpio_ameba_data *data = dev->data;
	const struct gpio_ameba_config *cfg = dev->config;
	uint32_t port = cfg->port;

	/* Get the int status  */
	int_status = GPIO_INTStatusGet(port);

	/* Clear pending edge interrupt */
	GPIO_INTStatusClearEdge(port);

	/* Call the registered callbacks */
	gpio_fire_callbacks(&data->callbacks, dev, int_status);
}

static const struct gpio_driver_api gpio_ameba_driver_api = {
	.pin_configure = gpio_ameba_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
	.pin_get_config = gpio_ameba_get_config,
#endif
	.port_get_raw = gpio_ameba_port_get_raw,
	.port_set_masked_raw = gpio_ameba_port_set_masked_raw,
	.port_set_bits_raw = gpio_ameba_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ameba_port_clear_bits_raw,
	.port_toggle_bits = gpio_ameba_port_toggle_bits,
	.pin_interrupt_configure = gpio_ameba_pin_interrupt_configure,
	.manage_callback = gpio_ameba_manage_callback,
	.get_pending_int = gpio_ameba_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_ameba_get_direction,
#endif
};

#define GPIO_AMEBA_INIT(n)                                                                         \
	static int gpio_ameba_port##n##_init(const struct device *dev)                             \
	{                                                                                          \
		const struct gpio_ameba_config *cfg = dev->config;                                 \
		LOG_INF("GPIO%d INIT, IRQ %d\n", cfg->port, DT_INST_IRQN(n));                      \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), gpio_ameba_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
                                                                                                   \
		return 0;                                                                          \
	}                                                                                          \
	static struct gpio_ameba_data gpio_ameba_port##n##_data;                                   \
                                                                                                   \
	static const struct gpio_ameba_config gpio_ameba_port##n##_config = {                      \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),               \
			},                                                                         \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.port = n,                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, gpio_ameba_port##n##_init, NULL, &gpio_ameba_port##n##_data,      \
			      &gpio_ameba_port##n##_config, POST_KERNEL,                           \
			      CONFIG_GPIO_INIT_PRIORITY, &gpio_ameba_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_AMEBA_INIT)

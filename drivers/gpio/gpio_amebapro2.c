/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_amebapro2_gpio

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include "hal.h"

LOG_MODULE_REGISTER(gpio_ameba, CONFIG_GPIO_LOG_LEVEL);

#define GPIO_PINNAME(PORT, PIN) (((PORT) << 5) | ((PIN) & 0x1F))
#define pin_use                 10
static hal_gpio_irq_adapter_t gpio_irq_adp[pin_use];
static uint8_t gpio_irq_adp_pin[pin_use];

struct gpio_ameba_config {
	hal_gpio_adapter_t gpio_adp;
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* port base address */
	uint32_t base;
	/* IO port */
	int port;
	int max_pin_num;
};

struct gpio_ameba_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* port ISR callback routine address */
	sys_slist_t callbacks;
};

static int gpio_ameba_port_get_raw(const struct device *dev, uint32_t *value)
{
	struct gpio_ameba_config *cfg = (struct gpio_ameba_config *)dev->config;
	*value = 0;

	for (int i = 0; i < cfg->max_pin_num; i++) {
		hal_gpio_reinit(&cfg->gpio_adp, GPIO_PINNAME(cfg->port, i));
		cfg->gpio_adp.debounce_idx = 0xFF;

		*value |= (hal_gpio_read(&cfg->gpio_adp) << i);
	}

	return 0;
}

static int gpio_ameba_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value)
{
	struct gpio_ameba_config *cfg = (struct gpio_ameba_config *)dev->config;

	for (int i = 0; i < cfg->max_pin_num; i++) {
		if (mask & BIT0) {
			hal_gpio_reinit(&cfg->gpio_adp, GPIO_PINNAME(cfg->port, i));
			hal_gpio_set_dir(&cfg->gpio_adp, GPIO_OUT);
			if (value & BIT0) {
				hal_gpio_write(&cfg->gpio_adp, 1);
			} else {
				hal_gpio_write(&cfg->gpio_adp, 0);
			}
		}
		mask >>= 1;
		value >>= 1;
		if (mask == 0) {
			break;
		}
	}
	return 0;
}

static int gpio_ameba_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	struct gpio_ameba_config *cfg = (struct gpio_ameba_config *)dev->config;

	for (int i = 0; i < cfg->max_pin_num; i++) {
		if (mask & BIT0) {
			hal_gpio_reinit(&cfg->gpio_adp, GPIO_PINNAME(cfg->port, i));
			hal_gpio_set_dir(&cfg->gpio_adp, GPIO_OUT);
			hal_gpio_write(&cfg->gpio_adp, 1);
		}
		mask >>= 1;
		if (mask == 0) {
			break;
		}
	}

	return 0;
}

static int gpio_ameba_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	struct gpio_ameba_config *cfg = (struct gpio_ameba_config *)dev->config;

	for (int i = 0; i < cfg->max_pin_num; i++) {
		if (mask & BIT0) {
			hal_gpio_reinit(&cfg->gpio_adp, GPIO_PINNAME(cfg->port, i));
			hal_gpio_set_dir(&cfg->gpio_adp, GPIO_OUT);
			hal_gpio_write(&cfg->gpio_adp, 0);
		}
		mask >>= 1;
		if (mask == 0) {
			break;
		}
	}

	return 0;
}

static int gpio_ameba_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	struct gpio_ameba_config *cfg = (struct gpio_ameba_config *)dev->config;
	u32 value;

	for (int i = 0; i < cfg->max_pin_num; i++) {
		if (mask & BIT0) {
			hal_gpio_reinit(&cfg->gpio_adp, GPIO_PINNAME(cfg->port, i));
			cfg->gpio_adp.debounce_idx = 0xFF;
			value = hal_gpio_read(&cfg->gpio_adp);
			hal_gpio_write(&cfg->gpio_adp, (~value) & BIT0);
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
	struct gpio_ameba_config *cfg = (struct gpio_ameba_config *)dev->config;
	uint32_t pinname = GPIO_PINNAME(cfg->port, pin);

	hal_gpio_init(&cfg->gpio_adp, pinname);

	if ((flags & GPIO_OUTPUT) != 0) {
		hal_gpio_set_dir(&cfg->gpio_adp, GPIO_OUT);
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			hal_gpio_write(&cfg->gpio_adp, 1);
		} else {
			hal_gpio_write(&cfg->gpio_adp, 0);
		}
	} else {
		hal_gpio_set_dir(&cfg->gpio_adp, GPIO_IN);
	}

	return 0;
}

static int gpio_ameba_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					      enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	struct gpio_ameba_config *cfg = (struct gpio_ameba_config *)dev->config;
	int adp_pin_num = 0;

	LOG_DBG("Config GPIO int:%x-%x, mode:%x, flag:0x%x\n", cfg->port, pin, mode, trig);

	uint32_t pinname = GPIO_PINNAME(cfg->port, pin);

	for (int i = 0; i < pin_use; i++) {
		if ((gpio_irq_adp_pin[i] == pinname) | (gpio_irq_adp_pin[i] == 0xFF)) {
			adp_pin_num = i;
			break;
		}
		if (i == (pin_use - 1)) {
			LOG_ERR("Over pin_use, max %d\n", pin_use);
			return -1;
		}
	}
	gpio_irq_adp_pin[adp_pin_num] = pinname;
	hal_gpio_irq_init(&gpio_irq_adp[adp_pin_num], pinname, NULL, pinname & 0x1F);

	/* recover IRQ handler to zephyr ISR wrapper after hal_uart_init */
	__NVIC_SetVector((IRQn_Type)AON_IRQn, (uint32_t)_isr_wrapper);
	__NVIC_SetVector((IRQn_Type)PonGPIO_IRQn, (uint32_t)_isr_wrapper);
	__NVIC_SetVector((IRQn_Type)GPIO_IRQn, (uint32_t)_isr_wrapper);

	if (mode & GPIO_INT_EDGE) {
		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			hal_gpio_irq_set_trig_type(&gpio_irq_adp[adp_pin_num],
						   GPIO_IntType_EdgeFalling);
			break;
		case GPIO_INT_TRIG_HIGH:
			hal_gpio_irq_set_trig_type(&gpio_irq_adp[adp_pin_num],
						   GPIO_IntType_EdgeRising);
			break;
		case GPIO_INT_TRIG_BOTH:
			hal_gpio_irq_set_trig_type(&gpio_irq_adp[adp_pin_num],
						   GPIO_IntType_EdgeDual);
			break;
		default:
			LOG_ERR("GPIO Edge interrupt type invalid \r\n");
			return -1;
		}
	} else {
		switch (trig) {
		case GPIO_INT_TRIG_LOW:
			hal_gpio_irq_set_trig_type(&gpio_irq_adp[adp_pin_num],
						   GPIO_IntType_LevelLow);
			break;
		case GPIO_INT_TRIG_HIGH:
			hal_gpio_irq_set_trig_type(&gpio_irq_adp[adp_pin_num],
						   GPIO_IntType_LevelHigh);
			break;
		default:
			LOG_ERR("GPIO level interrupt doesn't support both high and low");
			return -1;
		}
	}

	if (mode != GPIO_INT_MODE_DISABLED) {
		hal_gpio_irq_enable(&gpio_irq_adp[adp_pin_num]);
	} else {
		hal_gpio_irq_disable(&gpio_irq_adp[adp_pin_num]);
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
	/*not support*/
	return -1;
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

static int highest_bit_position(int num)
{
	int position = 0;

	while (num) {
		num >>= 1;
		position++;
	}

	return position - 1;
}

static void aon_gpio_ameba_isr(const struct device *dev)
{
	uint32_t int_sts;
	uint32_t pins = 0;
	uint32_t init_num = 0;
	struct gpio_ameba_data *data = dev->data;

	/* Get the int status  */
	__NVIC_ClearPendingIRQ(AON_IRQn);
	int_sts = AON_GPIO->GPIO_INT_STS;
	AON_GPIO->GPIO_INT_CLR = int_sts;

	for (int i = 0; i < pin_use; i++) {
		if (PIN_NAME_2_PORT(gpio_irq_adp_pin[i]) == PORT_A) {
			if (pins == highest_bit_position(int_sts)) {
				init_num = i;
				break;
			}
			pins += 1;
		}
	}

	/* Call the registered callbacks */
	gpio_fire_callbacks(&data->callbacks, dev, BIT(PIN_NAME_2_PIN(gpio_irq_adp_pin[init_num])));
}

static void pon_gpio_ameba_isr(const struct device *dev)
{
	uint32_t int_sts;
	uint32_t pins = 0;
	uint32_t init_num = 0;
	struct gpio_ameba_data *data = dev->data;

	/* Get the int status  */
	__NVIC_ClearPendingIRQ(PonGPIO_IRQn);
	int_sts = PON_GPIO->GPIO_INT_STS;
	PON_GPIO->GPIO_INT_CLR = int_sts;

	for (int i = 0; i < pin_use; i++) {
		if (PIN_NAME_2_PORT(gpio_irq_adp_pin[i]) == PORT_F) {
			if (pins == highest_bit_position(int_sts)) {
				init_num = i;
				break;
			}
			pins += 1;
		}
	}

	/* Call the registered callbacks */
	gpio_fire_callbacks(&data->callbacks, dev, BIT(PIN_NAME_2_PIN(gpio_irq_adp_pin[init_num])));
}

static void gpio_ameba_isr(const struct device *dev)
{
	uint32_t int_sts;
	uint32_t pins = 0;
	uint32_t init_num = 0;
	struct gpio_ameba_data *data = dev->data;

	/* Get the int status  */
	__NVIC_ClearPendingIRQ(GPIO_IRQn);
	int_sts = SYSON_GPIO->GPIO_INT_STS;
	SYSON_GPIO->GPIO_INT_CLR = int_sts;

	for (int i = 0; i < pin_use; i++) {
		if ((PIN_NAME_2_PORT(gpio_irq_adp_pin[i]) != PORT_A) &
		    (PIN_NAME_2_PORT(gpio_irq_adp_pin[i]) != PORT_F)) {
			if (pins == highest_bit_position(int_sts)) {
				init_num = i;
				break;
			}
			pins += 1;
		}
	}

	/* Call the registered callbacks */
	gpio_fire_callbacks(&data->callbacks, dev, BIT(PIN_NAME_2_PIN(gpio_irq_adp_pin[init_num])));
}

hal_aon_gpio_comm_adapter_t mbd_aon_gpio_comm_adp;
hal_pon_gpio_comm_adapter_t mbd_pon_gpio_comm_adp;
hal_gpio_comm_adapter_t mbd_gpio_comm_adp;

static int gpio_ameba_port_aon_init(const struct device *dev)
{
	const struct gpio_ameba_config *cfg = (struct gpio_ameba_config *)dev->config;
	uint8_t port_idx = cfg->port;

	if (port_idx >= PORT_MAX_NUM) {
		DBG_GPIO_ERR("Invalid GPIO port(%u)\n", port_idx);
		return HAL_ERR_PARA;
	}

	memset(gpio_irq_adp_pin, 0xFF, sizeof(gpio_irq_adp_pin));

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), aon_gpio_ameba_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	hal_aon_gpio_comm_init(&mbd_aon_gpio_comm_adp);

	return 0;
}

static int gpio_ameba_port_pon_init(const struct device *dev)
{
	const struct gpio_ameba_config *cfg = (struct gpio_ameba_config *)dev->config;
	uint8_t port_idx = cfg->port;

	if (port_idx >= PORT_MAX_NUM) {
		DBG_GPIO_ERR("Invalid GPIO port(%u)\n", port_idx);
		return HAL_ERR_PARA;
	}

	memset(gpio_irq_adp_pin, 0xFF, sizeof(gpio_irq_adp_pin));

	IRQ_CONNECT(DT_INST_IRQN(1), DT_INST_IRQ(1, priority), pon_gpio_ameba_isr,
		    DEVICE_DT_INST_GET(1), 0);
	irq_enable(DT_INST_IRQN(1));

	hal_pon_gpio_comm_init(&mbd_pon_gpio_comm_adp);

	return 0;
}

static int gpio_ameba_port_init(const struct device *dev)
{
	const struct gpio_ameba_config *cfg = (struct gpio_ameba_config *)dev->config;
	uint8_t port_idx = cfg->port;

	if (port_idx >= PORT_MAX_NUM) {
		DBG_GPIO_ERR("Invalid GPIO port(%u)\n", port_idx);
		return HAL_ERR_PARA;
	}

	memset(gpio_irq_adp_pin, 0xFF, sizeof(gpio_irq_adp_pin));

	IRQ_CONNECT(DT_INST_IRQN(2), DT_INST_IRQ(2, priority), gpio_ameba_isr,
		    DEVICE_DT_INST_GET(2), 0);
	irq_enable(DT_INST_IRQN(2));

	hal_gpio_comm_init(&mbd_gpio_comm_adp);

	return 0;
}

static struct gpio_ameba_data gpio_ameba_port_aon_data;
static struct gpio_ameba_data gpio_ameba_port_pon_data;
static struct gpio_ameba_data gpio_ameba_port_data;

static const struct gpio_ameba_config gpio_ameba_port_aon_config = {
	.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0)},
	.base = DT_INST_REG_ADDR(0),
	.port = 0,
	.max_pin_num = 6,
};
static const struct gpio_ameba_config gpio_ameba_port_pon_config = {
	.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0)},
	.base = DT_INST_REG_ADDR(0),
	.port = 5,
	.max_pin_num = 18,
};
static const struct gpio_ameba_config gpio_ameba_port_config = {
	.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0)},
	.base = DT_INST_REG_ADDR(0),
	.port = 4,
	.max_pin_num = 7,
};

DEVICE_DT_INST_DEFINE(0, gpio_ameba_port_aon_init, NULL, &gpio_ameba_port_aon_data,
		      &gpio_ameba_port_aon_config, POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,
		      &gpio_ameba_driver_api);
DEVICE_DT_INST_DEFINE(1, gpio_ameba_port_pon_init, NULL, &gpio_ameba_port_pon_data,
		      &gpio_ameba_port_pon_config, POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,
		      &gpio_ameba_driver_api);
DEVICE_DT_INST_DEFINE(2, gpio_ameba_port_init, NULL, &gpio_ameba_port_data, &gpio_ameba_port_config,
		      POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY, &gpio_ameba_driver_api);

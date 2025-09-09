/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include <soc.h> before hal include file to avoid redefining unlikely() macro */
#include <soc.h>
#include <hal_pinmux.h>
#include <hal_gpio.h>

#include <zephyr/drivers/pinctrl.h>

#define AMEBA_GET_PORT_NUM(pin_mux)   (((pin_mux) >> 18) & 0x0F)
#define AMEBA_GET_PIN_NUM(pin_mux)    (((pin_mux) >> 13) & 0x1F)
#define AMEBA_GET_PIMNUX_ID(pin_mux)  (((((pin_mux) & 0x1E00) >> 9) << 28) | ((pin_mux) & 0x1FF))

#define AMEBA_GPIO_PINNAME(PORT, PIN) (((PORT) << 5) | ((PIN) & 0x1F))

static int ameba_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t port_idx, pin_idx;
	uint8_t gpio_pin;
	uint32_t function_id;

	port_idx = AMEBA_GET_PORT_NUM(pin->pinmux);
	pin_idx = AMEBA_GET_PIN_NUM(pin->pinmux);
	function_id = AMEBA_GET_PIMNUX_ID(pin->pinmux);
	gpio_pin = AMEBA_GPIO_PINNAME(port_idx, pin_idx);

	hal_pinmux_register(gpio_pin, function_id);

	if (pin->pull_up) {
		hal_gpio_pull_ctrl(gpio_pin, Pin_PullUp);
	} else if (pin->pull_down) {
		hal_gpio_pull_ctrl(gpio_pin, Pin_PullDown);
	} else {
		hal_gpio_pull_ctrl(gpio_pin, Pin_PullNone);
	}

	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	int ret = 0;

	ARG_UNUSED(reg);

	for (int i = 0; i < pin_cnt; i++) {
		ret = ameba_configure_pin(&pins[i]);

		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

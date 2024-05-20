/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/realtek-rtl8721f-pinctrl.h>
#include "ameba_soc.h"

#define GPIO_PINNAME(PORT, PIN)			(((PORT) << 5) | ((PIN) & 0x1F))

static int realtek_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t port_idx, pin_idx;
	uint8_t gpio_pin;
	uint32_t function_id;

	port_idx = GET_PORT_NUM(pin->pinmux);
	pin_idx = GET_PIN_NUM(pin->pinmux);
	function_id = GET_PIMNUX_ID(pin->pinmux);
	gpio_pin = GPIO_PINNAME(port_idx, pin_idx);

	Pinmux_Config(gpio_pin, function_id);

	if (pin->pull_up) {
		PAD_PullCtrl(gpio_pin, GPIO_PuPd_UP);
		PAD_SleepPullCtrl(gpio_pin, GPIO_PuPd_UP);
	} else if (pin->pull_down) {
		PAD_PullCtrl(gpio_pin, GPIO_PuPd_DOWN);
		PAD_SleepPullCtrl(gpio_pin, GPIO_PuPd_DOWN);
	} else {
		PAD_PullCtrl(gpio_pin, GPIO_PuPd_NOPULL);
		PAD_SleepPullCtrl(gpio_pin, GPIO_PuPd_NOPULL);
	}


	if (pin->slew_rate) {
		PAD_SlewRateCtrl(gpio_pin, ENABLE);
	}
	if (pin->drive_strength) {
		PAD_DrvStrength(gpio_pin, ENABLE);
	}
	if (pin->digital_input) {
		PAD_InputCtrl(gpio_pin, ENABLE);
	}
	if (pin->schmitt_disable) {
		PAD_SchmitCtrl(gpio_pin, ENABLE);
	}
	if (pin->swd_off) {
		Pinmux_Swdoff();
	}

	return 0;

}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	int ret = 0;

	ARG_UNUSED(reg);

	for (int i = 0; i < pin_cnt; i++) {
		ret = realtek_configure_pin(&pins[i]);

		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

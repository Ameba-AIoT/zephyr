/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_REALTEK_AMEBA_AMEBAPRO2_PINCTRL_SOC_H_
#define ZEPHYR_SOC_REALTEK_AMEBA_AMEBAPRO2_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pinctrl_soc_pin {
	uint32_t pinmux: 22;
	/* bit[21:18] port
	 * bit[17:13] pin
	 * bit[12:9] + bit[8:0] function ID
	 */
	uint32_t pull_down: 1;
	uint32_t pull_up: 1;
};

typedef struct pinctrl_soc_pin pinctrl_soc_pin_t;

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{                                                                                          \
		.pinmux = DT_PROP_BY_IDX(node_id, prop, idx),                                      \
		.pull_down = DT_PROP(node_id, bias_pull_down),                                     \
		.pull_up = DT_PROP(node_id, bias_pull_up),                                         \
	},

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux,           \
				Z_PINCTRL_STATE_PIN_INIT)}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_REALTEK_AMEBA_AMEBAPRO2_PINCTRL_SOC_H_ */

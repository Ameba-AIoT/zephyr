/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_AMEBA_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_AMEBA_CLOCK_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/clock_control.h>

#if defined(CONFIG_SOC_SERIES_AMEBADPLUS)
#include <zephyr/dt-bindings/clock/amebadplus_clock.h>
#else
#error: please choose the right chip type.
#endif


/** Common clock control device node for all ameba chips */
#define AMEBA_CLOCK_CONTROL_NODE       DT_NODELABEL(rcc)
#define AMEBA_CLOCK_CONTROL_DEV        DEVICE_DT_GET(AMEBA_CLOCK_CONTROL_NODE)

/* clock configure params */
struct ameba_clock_config {
	uint32_t param_value;    /* values */
	int8_t is_mux;     /* div or mux */
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_AMEBA_CLOCK_CONTROL_H_ */

/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

static const struct device *gpio_aon_dev = DEVICE_DT_GET(DT_NODELABEL(gpio_aon));
static const struct device *gpio_pon_dev = DEVICE_DT_GET(DT_NODELABEL(gpio_pon));
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio));

static void gpio_aon_ameba_irq_callback(const struct device *port, struct gpio_callback *cb,
					gpio_port_pins_t pins)
{
	printf("gpio_aon_ameba_isr\n\r");
}

static void gpio_pon_ameba_irq_callback(const struct device *port, struct gpio_callback *cb,
					gpio_port_pins_t pins)
{
	printf("gpio_pon_ameba_isr\n\r");
}

static void gpio_ameba_irq_callback(const struct device *port, struct gpio_callback *cb,
				    gpio_port_pins_t pins)
{
	printf("gpio_ameba_isr\n\r");
}

static void test_gpio(void)
{
	static struct gpio_callback gpio_cb;
	static struct gpio_callback gpio_cb2;
	static struct gpio_callback gpio_cb3;

	gpio_pin_interrupt_configure(gpio_aon_dev, 2,
				     GPIO_INT_ENABLE | GPIO_INT_EDGE | GPIO_INT_HIGH_1);
	gpio_init_callback(&gpio_cb, gpio_aon_ameba_irq_callback, BIT(2));
	gpio_add_callback(gpio_aon_dev, &gpio_cb);

	gpio_pin_configure(gpio_aon_dev, 3, GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);

	printf("gpio_aon_get 0x%x \r\n", gpio_pin_get(gpio_aon_dev, 2));
	gpio_pin_set(gpio_aon_dev, 3, 1);
	k_sleep(K_SECONDS(3));
	gpio_pin_set(gpio_aon_dev, 3, 0);

	gpio_pin_interrupt_configure(gpio_pon_dev, 0,
				     GPIO_INT_ENABLE | GPIO_INT_EDGE | GPIO_INT_HIGH_1);
	gpio_init_callback(&gpio_cb2, gpio_pon_ameba_irq_callback, BIT(0));
	gpio_add_callback(gpio_pon_dev, &gpio_cb2);

	gpio_pin_configure(gpio_pon_dev, 8, GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);

	printf("gpio_pon_get 0x%x \r\n", gpio_pin_get(gpio_pon_dev, 0));
	gpio_pin_set(gpio_pon_dev, 8, 1);
	k_sleep(K_SECONDS(3));
	gpio_pin_set(gpio_pon_dev, 8, 0);

	gpio_pin_interrupt_configure(gpio_dev, 5,
				     GPIO_INT_ENABLE | GPIO_INT_EDGE | GPIO_INT_HIGH_1);
	gpio_init_callback(&gpio_cb3, gpio_ameba_irq_callback, BIT(5));
	gpio_add_callback(gpio_dev, &gpio_cb3);

	gpio_pin_configure(gpio_dev, 6, GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);

	printf("gpio_get 0x%x \r\n", gpio_pin_get(gpio_dev, 5));
	gpio_pin_set(gpio_dev, 6, 1);
	k_sleep(K_SECONDS(3));
	gpio_pin_set(gpio_dev, 6, 0);
}

int main(void)
{
	printf("gpio example! %s\n\r", CONFIG_BOARD_TARGET);

	test_gpio();

	return 0;
}

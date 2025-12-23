/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8201fr

#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/drivers/mdio.h>
#include <string.h>
#include <zephyr/sys/util_macro.h>
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios) || DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
#include <zephyr/drivers/gpio.h>
#endif

#define LOG_MODULE_NAME phy_rt_rtl8201fr
#define LOG_LEVEL       CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include "phy_mii.h"

#define REALTEK_OUI_MSB (0x1CU)

#define FEPHY_REG_PAGE_0 0x0
#define FEPHY_REG_PAGE_1 0x1
#define FEPHY_REG_PAGE_2 0x2
#define FEPHY_REG_PAGE_3 0x3
#define FEPHY_REG_PAGE_4 0x4
#define FEPHY_REG_PAGE_5 0x5
#define FEPHY_REG_PAGE_6 0x6
#define FEPHY_REG_PAGE_7 0x7
#define FEPHY_REG_PAGE_8 0x8
#define FEPHY_REG_PAGE_9 0x9
#define FEPHY_REG_PAGE_A 0xA
#define FEPHY_REG_PAGE_B 0xB

/*page0: Interrupt indicators and SNR display reg*/
#define PHY_RT_RTL8201FR_INSR_REG  (0x1EU)
/*page0: page select reg*/
#define PHY_RT_RTL8201FR_PAGSR_REG (0x1FU)

/*page7: Interrupt and LEDs function reg*/
#define PHY_RT_RTL8201FR_INT_LED_SET_REG             (0x13)
#define PHY_RT_RTL8201FR_INER_LINKSTATUS_CHANGE_MASK BIT(13)
#define PHY_RT_RTL8201FR_INER_DUPLEX_CHANGE_MASK     BIT(12)
#define PHY_RT_RTL8201FR_INER_NWAY_ERROR_MASK        BIT(11)

/*page7: Wake-on-LAN and Interrupt function*/
#define PHY_RT_RTL8201FR_WOL_INTR_SET_REG (0x17)
#define PHY_RT_RTL8201FR_WOL_INTR_MASK    BIT(10)

#define PHY_RT_RTL8201FR_RESET_HOLD_TIME_MS 10

struct rt_rtl8201fr_config {
	uint8_t addr;
	const struct device *mdio_dev;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
	const struct gpio_dt_spec reset_gpio;
#endif
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	const struct gpio_dt_spec interrupt_gpio;
#endif
};

struct rt_rtl8201fr_data {
	const struct device *dev;
	struct phy_link_state state;
	phy_callback_t cb;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	struct gpio_callback gpio_callback;
#endif
	void *cb_data;
	struct k_mutex mutex;
	struct k_work_delayable phy_monitor_work;
};

static int phy_rt_rtl8201fr_read(const struct device *dev, uint16_t reg_addr, uint32_t *data)
{
	const struct rt_rtl8201fr_config *config = dev->config;
	int ret;

	/* Make sure excessive bits 16-31 are reset */
	*data = 0U;

	/* Read the PHY register */
	ret = mdio_read(config->mdio_dev, config->addr, reg_addr, (uint16_t *)data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_rt_rtl8201fr_write(const struct device *dev, uint16_t reg_addr, uint32_t data)
{
	const struct rt_rtl8201fr_config *config = dev->config;
	int ret;

	ret = mdio_write(config->mdio_dev, config->addr, reg_addr, (uint16_t)data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_rt_rtl8201fr_reset(const struct device *dev)
{
	const struct rt_rtl8201fr_config *config = dev->config;
	uint32_t reg_val;
	int ret;

	/*select page 0*/
	ret = phy_rt_rtl8201fr_write(dev, PHY_RT_RTL8201FR_PAGSR_REG, FEPHY_REG_PAGE_0);

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
	if (config->reset_gpio.port) {
		/* Start reset */
		ret = gpio_pin_set_dt(&config->reset_gpio, 0);
		if (ret) {
			return ret;
		}

		/* Hold reset for the minimum time specified by datasheet */
		k_busy_wait(USEC_PER_MSEC * PHY_RT_RTL8201FR_RESET_HOLD_TIME_MS);

		/* Reset over */
		ret = gpio_pin_set_dt(&config->reset_gpio, 1);
		if (ret) {
			return ret;
		}

		/* Wait another 30 ms (circuits settling time) before accessing registers */
		k_busy_wait(USEC_PER_MSEC * 30);

		goto finalize_reset;
	}
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios) */

	/* Reset PHY using register */
	ret = phy_rt_rtl8201fr_write(dev, MII_BMCR, MII_BMCR_RESET);
	if (ret) {
		LOG_ERR("Error writing phy (%d) basic control register", config->addr);
		return ret;
	}

	/* Wait for the minimum reset time specified by datasheet */
	k_busy_wait(USEC_PER_MSEC * PHY_RT_RTL8201FR_RESET_HOLD_TIME_MS);

	/* Wait for the reset to be cleared */
	do {
		ret = phy_rt_rtl8201fr_read(dev, MII_BMCR, &reg_val);
		if (ret) {
			LOG_ERR("Error reading phy (%d) basic control register", config->addr);
			return ret;
		}
	} while (reg_val & MII_BMCR_RESET);

	goto finalize_reset;

finalize_reset:
	/* Wait until correct data can be read from registers */
	do {
		ret = phy_rt_rtl8201fr_read(dev, MII_PHYID1R, &reg_val);
		if (ret) {
			LOG_ERR("Error reading phy (%d) identifier register 1", config->addr);
			return ret;
		}
	} while (reg_val != REALTEK_OUI_MSB);

	return 0;
}

static int phy_rt_rtl8201fr_restart_autonegotiation(const struct device *dev)
{
	const struct rt_rtl8201fr_config *config = dev->config;
	uint32_t bmcr = 0;
	int ret;

	/* Read control register to write back with autonegotiation bit */
	ret = phy_rt_rtl8201fr_read(dev, MII_BMCR, &bmcr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) basic control register", config->addr);
		return ret;
	}

	/* (re)start autonegotiation */
	LOG_DBG("PHY (%d) is entering autonegotiation sequence", config->addr);
	bmcr |= MII_BMCR_AUTONEG_ENABLE | MII_BMCR_AUTONEG_RESTART;

	ret = phy_rt_rtl8201fr_write(dev, MII_BMCR, bmcr);
	if (ret) {
		LOG_ERR("Error writing phy (%d) basic control register", config->addr);
		return ret;
	}

	return 0;
}

static int phy_rt_rtl8201fr_get_link(const struct device *dev, struct phy_link_state *state)
{
	const struct rt_rtl8201fr_config *config = dev->config;
	struct rt_rtl8201fr_data *data = dev->data;
	int ret;
	uint32_t physr = 0;
	uint32_t duplex = 0;
	struct phy_link_state old_state = data->state;
	struct phy_link_state new_state = {};

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

	/* Read PHY specific status register */
	ret = phy_rt_rtl8201fr_read(dev, MII_BMSR, &physr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) specific status register", config->addr);
		(void)k_mutex_unlock(&data->mutex);
		return ret;
	}

	/* Unlock mutex */
	(void)k_mutex_unlock(&data->mutex);

	new_state.is_up = physr & MII_BMSR_LINK_STATUS;

	if (!new_state.is_up) {
		goto result;
	}

	ret = phy_rt_rtl8201fr_read(dev, MII_BMCR, &physr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) specific status register", config->addr);
		(void)k_mutex_unlock(&data->mutex);
		return ret;
	}
	duplex = (physr & MII_BMCR_DUPLEX_MODE);

	if (physr & MII_BMCR_SPEED_LSB) {
		if (duplex) {
			new_state.speed = LINK_FULL_100BASE;
		} else {
			new_state.speed = LINK_HALF_100BASE;
		}
	} else {
		if (duplex) {
			new_state.speed = LINK_FULL_10BASE;
		} else {
			new_state.speed = LINK_HALF_10BASE;
		}
	}
	/*interrupt gpio status*/
	/*select page 7*/
	ret = phy_rt_rtl8201fr_write(dev, PHY_RT_RTL8201FR_PAGSR_REG, FEPHY_REG_PAGE_7);

	ret = phy_rt_rtl8201fr_read(dev, 0x13, &physr);

result:
	if (memcmp(&old_state, &new_state, sizeof(struct phy_link_state)) != 0) {
		LOG_INF("PHY %d is %s", config->addr, new_state.is_up ? "up" : "down");
		if (new_state.is_up) {
			LOG_INF("PHY (%d) Link speed %s Mb, %s duplex", config->addr,
				PHY_LINK_IS_SPEED_100M(new_state.speed) ? "100" : "10",
				PHY_LINK_IS_FULL_DUPLEX(new_state.speed) ? "full" : "half");
		}
	}

	memcpy(state, &new_state, sizeof(struct phy_link_state));

	return ret;
}

static int phy_rt_rtl8201fr_cfg_link(const struct device *dev, enum phy_link_speed speeds,
				     enum phy_cfg_link_flag flags)
{
	const struct rt_rtl8201fr_config *config = dev->config;
	struct rt_rtl8201fr_data *data = dev->data;
	int ret;

	if (flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) {
		LOG_ERR("Disabling auto-negotiation is not supported by this driver");
		return -ENOTSUP;
	}

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

	/* We are going to reconfigure the phy, don't need to monitor until done */
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (!config->interrupt_gpio.port) {
		k_work_cancel_delayable(&data->phy_monitor_work);
	}
#else
	k_work_cancel_delayable(&data->phy_monitor_work);
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

	ret = phy_mii_set_anar_reg(dev, speeds);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Error setting ANAR register for phy (%d)", config->addr);
		goto done;
	}

	ret = phy_mii_set_c1kt_reg(dev, speeds);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Error setting C1KT register for phy (%d)", config->addr);
		goto done;
	}

	/* (Re)start autonegotiation */
	ret = phy_rt_rtl8201fr_restart_autonegotiation(dev);
	if (ret) {
		LOG_ERR("Error restarting autonegotiation");
		goto done;
	}
done:
	/* Unlock mutex */
	(void)k_mutex_unlock(&data->mutex);

	/* Start monitoring */
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (!config->interrupt_gpio.port) {
		k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
	}
#else
	k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

	return ret;
}

static int phy_rt_rtl8201fr_link_cb_set(const struct device *dev, phy_callback_t cb,
					void *user_data)
{
	struct rt_rtl8201fr_data *data = dev->data;

	data->cb = cb;
	data->cb_data = user_data;

	phy_rt_rtl8201fr_get_link(dev, &data->state);

	data->cb(dev, &data->state, data->cb_data);

	return 0;
}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
static int phy_rt_rtl8201fr_clear_interrupt(struct rt_rtl8201fr_data *data)
{
	const struct device *dev = data->dev;
	const struct rt_rtl8201fr_config *config = dev->config;
	uint32_t reg_val;
	int ret;

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

	/* Read/clear PHY interrupt status register */
	ret = phy_rt_rtl8201fr_read(dev, PHY_RT_RTL8201FR_INSR_REG, &reg_val);
	if (ret) {
		LOG_ERR("Error reading phy (%d) interrupt status register", config->addr);
	}

	/* Unlock mutex */
	(void)k_mutex_unlock(&data->mutex);

	return ret;
}

static void phy_rt_rtl8201fr_interrupt_handler(const struct device *port, struct gpio_callback *cb,
					       gpio_port_pins_t pins)
{
	struct rt_rtl8201fr_data *data = CONTAINER_OF(cb, struct rt_rtl8201fr_data, gpio_callback);
	int ret;

	ret = k_work_reschedule(&data->phy_monitor_work, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to schedule phy_monitor_work from ISR");
	}
}
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

static void phy_rt_rtl8201fr_monitor_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct rt_rtl8201fr_data *data =
		CONTAINER_OF(dwork, struct rt_rtl8201fr_data, phy_monitor_work);
	const struct device *dev = data->dev;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	const struct rt_rtl8201fr_config *config = dev->config;
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */
	struct phy_link_state state = {};
	int ret;

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (config->interrupt_gpio.port) {
		ret = phy_rt_rtl8201fr_clear_interrupt(data);
		if (ret) {
			return;
		}
	}
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

	ret = phy_rt_rtl8201fr_get_link(dev, &state);

	if (ret == 0 && memcmp(&state, &data->state, sizeof(struct phy_link_state)) != 0) {
		memcpy(&data->state, &state, sizeof(struct phy_link_state));
		if (data->cb) {
			data->cb(dev, &data->state, data->cb_data);
		}
	}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (!config->interrupt_gpio.port) {
		k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
	}
#else
	k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */
}

static int phy_rt_rtl8201fr_init(const struct device *dev)
{
	const struct rt_rtl8201fr_config *config = dev->config;
	struct rt_rtl8201fr_data *data = dev->data;
	int ret;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	uint32_t reg_val;
#endif
	data->dev = dev;

	ret = k_mutex_init(&data->mutex);
	if (ret) {
		return ret;
	}

	mdio_bus_enable(config->mdio_dev);

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
	/* Configure reset pin */
	if (config->reset_gpio.port) {
		ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret) {
			return ret;
		}
	}
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios) */

	/* Reset PHY */
	ret = phy_rt_rtl8201fr_reset(dev);
	if (ret) {
		LOG_ERR("Failed to reset phy (%d)", config->addr);
		return ret;
	}

	/* Restore to default page 0 */
	ret = phy_rt_rtl8201fr_write(dev, PHY_RT_RTL8201FR_PAGSR_REG, FEPHY_REG_PAGE_0);
	if (ret) {
		LOG_ERR("Error writing phy (%d) page select register", config->addr);
		return ret;
	}

	k_work_init_delayable(&data->phy_monitor_work, phy_rt_rtl8201fr_monitor_work_handler);

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	if (!config->interrupt_gpio.port) {
		phy_rt_rtl8201fr_monitor_work_handler(&data->phy_monitor_work.work);
		goto skip_int_gpio;
	}

	/* Set INTB/PMEB pin to interrupt mode */
	ret = phy_rt_rtl8201fr_write(dev, PHY_RT_RTL8201FR_PAGSR_REG, FEPHY_REG_PAGE_7);
	if (ret) {
		LOG_ERR("Error writing phy (%d) page select register", config->addr);
		return ret;
	}

	ret = phy_rt_rtl8201fr_read(dev, PHY_RT_RTL8201FR_WOL_INTR_SET_REG, &reg_val);
	if (!ret) {
		reg_val &= ~PHY_RT_RTL8201FR_WOL_INTR_MASK;
		ret = phy_rt_rtl8201fr_write(dev, PHY_RT_RTL8201FR_WOL_INTR_SET_REG, reg_val);
		if (ret) {
			LOG_ERR("Error writing phy (%d) interrupt pin setting register",
				config->addr);
			return ret;
		}
	} else {
		LOG_ERR("Error reading phy (%d) interrupt pin setting register", config->addr);
		return ret;
	}
	/* Restore to default page 0 */
	ret = phy_rt_rtl8201fr_write(dev, PHY_RT_RTL8201FR_PAGSR_REG, FEPHY_REG_PAGE_0);
	if (ret) {
		LOG_ERR("Error writing phy (%d) page select register", config->addr);
		return ret;
	}

	/* Clear interrupt */
	ret = phy_rt_rtl8201fr_clear_interrupt(data);
	if (ret) {
		return ret;
	}

	/* Configure interrupt pin */
	ret = gpio_pin_configure_dt(&config->interrupt_gpio, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	gpio_init_callback(&data->gpio_callback, phy_rt_rtl8201fr_interrupt_handler,
			   BIT(config->interrupt_gpio.pin));
	ret = gpio_add_callback_dt(&config->interrupt_gpio, &data->gpio_callback);
	if (ret) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		return ret;
	}

	/* Enable PHY interrupt. */
	ret = phy_rt_rtl8201fr_write(dev, PHY_RT_RTL8201FR_PAGSR_REG, FEPHY_REG_PAGE_7);
	if (ret) {
		LOG_ERR("Error writing phy (%d) page select register", config->addr);
		return ret;
	}
	ret = phy_rt_rtl8201fr_read(dev, PHY_RT_RTL8201FR_INT_LED_SET_REG, &reg_val);
	if (ret) {
		LOG_ERR("Error reading phy (%d) interrupt enable register", config->addr);
		return ret;
	}
	/* Enable link status change/duplex change/auto-negotiation error interrupt*/
	reg_val |= PHY_RT_RTL8201FR_INER_LINKSTATUS_CHANGE_MASK |
		   PHY_RT_RTL8201FR_INER_DUPLEX_CHANGE_MASK | PHY_RT_RTL8201FR_INER_NWAY_ERROR_MASK;
	ret = phy_rt_rtl8201fr_write(dev, PHY_RT_RTL8201FR_INT_LED_SET_REG, reg_val);
	if (ret) {
		LOG_ERR("Error writing phy (%d) interrupt enable register", config->addr);
		return ret;
	}
	/* Restore to default page 0 */
	ret = phy_rt_rtl8201fr_write(dev, PHY_RT_RTL8201FR_PAGSR_REG, FEPHY_REG_PAGE_0);
	if (ret) {
		LOG_ERR("Error writing phy (%d) page select register", config->addr);
		return ret;
	}
skip_int_gpio:
#else
	phy_rt_rtl8201fr_monitor_work_handler(&data->phy_monitor_work.work);
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

	return 0;
}

static DEVICE_API(ethphy, rt_rtl8201fr_phy_api) = {
	.get_link = phy_rt_rtl8201fr_get_link,
	.cfg_link = phy_rt_rtl8201fr_cfg_link,
	.link_cb_set = phy_rt_rtl8201fr_link_cb_set,
	.read = phy_rt_rtl8201fr_read,
	.write = phy_rt_rtl8201fr_write,
};

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
#define RESET_GPIO(n) .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),
#else
#define RESET_GPIO(n)
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios) */

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
#define INTERRUPT_GPIO(n) .interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),
#else
#define INTERRUPT_GPIO(n)
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) */

#define REALTEK_RTL8201FR_INIT(n)                                                                  \
	static const struct rt_rtl8201fr_config rt_rtl8201fr_##n##_config = {                      \
		.addr = DT_INST_REG_ADDR(n),                                                       \
		.mdio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                      \
		RESET_GPIO(n) INTERRUPT_GPIO(n)};                                                  \
                                                                                                   \
	static struct rt_rtl8201fr_data rt_rtl8201fr_##n##_data;                                   \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &phy_rt_rtl8201fr_init, NULL, &rt_rtl8201fr_##n##_data,           \
			      &rt_rtl8201fr_##n##_config, POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,   \
			      &rt_rtl8201fr_phy_api);

DT_INST_FOREACH_STATUS_OKAY(REALTEK_RTL8201FR_INIT)

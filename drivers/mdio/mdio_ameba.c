/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/mdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_ameba, CONFIG_MDIO_LOG_LEVEL);

#define DT_DRV_COMPAT realtek_ameba_mdio

#define ADIN1100_REG_VALUE_MASK GENMASK(15, 0)

#define MDIO_AMEBA_TODO 0

struct mdio_ameba_data {
	struct k_sem sem;
	/*ETH_HandleTypeDef heth;*/
};

struct mdio_ameba_config {
	const struct pinctrl_dev_config *pincfg;
};

static int mdio_ameba_read(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t *data)
{
#if MDIO_AMEBA_TODO
	struct mdio_ameba_data *const dev_data = dev->data;
	ETH_HandleTypeDef *heth = &dev_data->heth;
	uint32_t read;
	int ret;

	k_sem_take(&dev_data->sem, K_FOREVER);

	ret = HAL_ETH_ReadPHYRegister(heth, prtad, regad, &read);

	k_sem_give(&dev_data->sem);

	if (ret != HAL_OK) {
		return -EIO;
	}

	*data = read & ADIN1100_REG_VALUE_MASK;

	return ret;
#endif
	return 0;
}

static int mdio_ameba_write(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t data)
{
#if MDIO_AMEBA_TODO
	struct mdio_ameba_data *const dev_data = dev->data;
	ETH_HandleTypeDef *heth = &dev_data->heth;
	int ret;

	k_sem_take(&dev_data->sem, K_FOREVER);

	ret = HAL_ETH_WritePHYRegister(heth, prtad, regad, data);

	k_sem_give(&dev_data->sem);

	if (ret != HAL_OK) {
		return -EIO;
	}

	return ret;
#endif
	return 0;
}

static void mdio_ameba_bus_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void mdio_ameba_bus_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static int mdio_ameba_init(const struct device *dev)
{
	struct mdio_ameba_data *const dev_data = dev->data;
	const struct mdio_ameba_config *const config = dev->config;
	int ret;

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	k_sem_init(&dev_data->sem, 1, 1);

	return 0;
}

static DEVICE_API(mdio, mdio_ameba_api) = {
	.read = mdio_ameba_read,
	.write = mdio_ameba_write,
	.bus_enable = mdio_ameba_bus_enable,
	.bus_disable = mdio_ameba_bus_disable,
};

#define MDIO_AMEBA_DEVICE(inst)                                                                    \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static struct mdio_ameba_data mdio_ameba_data_##inst = {                                   \
		/*.heth = {.Instance = (ETH_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(inst))},*/        \
	};                                                                                         \
	static struct mdio_ameba_config mdio_ameba_config_##inst = {                               \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &mdio_ameba_init, NULL, &mdio_ameba_data_##inst,               \
			      &mdio_ameba_config_##inst, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,   \
			      &mdio_ameba_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_AMEBA_DEVICE)

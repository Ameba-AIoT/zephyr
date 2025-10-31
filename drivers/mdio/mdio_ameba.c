/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>
#include <ameba_soc.h>

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/mdio.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_ameba, CONFIG_MDIO_LOG_LEVEL);

#define DT_DRV_COMPAT realtek_ameba_mdio

#define ADIN1100_REG_VALUE_MASK GENMASK(15, 0)

#define MDIO_AMEBA_TODO 0

struct mdio_ameba_data {
	struct k_sem sem;
	ETHERNET_TypeDef *heth;
};

struct mdio_ameba_config {
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
};

static void mido_ameba_clock_config(void)
{
	uint32_t temp = 0;

	RCC_PeriphClockCmd(APBPeriph_GMAC, APBPeriph_GMAC_CLOCK, ENABLE);
	Pinmux_Config(_PA_12, PINMUX_FUNCTION_EXT_CLK_OUT);

	/*phy uses external clk*/
	temp = sys_read32(PINMUX_REG_BASE + REG_PINMUX_SUB_CTRL);
	temp |= PAD_BIT_DBG_CLK_FORCE;
	temp &= ~PAD_MASK_DBG_CLK0_SEL;
	temp |= PAD_DBG_CLK0_SEL(0xB);
	sys_write32(temp, PINMUX_REG_BASE + REG_PINMUX_SUB_CTRL);

	extern void eth_ameba_set_clock(void);
	eth_ameba_set_clock();
}

static int mdio_ameba_polling(ETHERNET_TypeDef *heth)
{
	int ret = -1;
	uint32_t start = k_uptime_get_32();

	while (k_uptime_get_32() - start <= 70 * USEC_PER_MSEC) {
		if (!(heth->ETH_MIIAR & BIT_MDIO_BUSY)) {
			ret = 0;
			break;
		}
	}
	return ret;
}

static int mdio_ameba_read(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t *data)
{

	struct mdio_ameba_data *const dev_data = dev->data;
	ETHERNET_TypeDef *heth = dev_data->heth;
	int ret;

	k_sem_take(&dev_data->sem, K_FOREVER);

	heth->ETH_MIIAR = (PHYADDRESS(prtad) | REGADDR4_0(regad));

	ret = mdio_ameba_polling(heth);

	if (ret != 0) {
		LOG_ERR("mdio bus is busy\n");
		ret = -ETIMEDOUT;
		goto exit;
	}

	*data = (uint16_t)GET_DATA15_0(heth->ETH_MIIAR);

exit:
	k_sem_give(&dev_data->sem);
	return ret;
}

static int mdio_ameba_write(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t data)
{
	struct mdio_ameba_data *const dev_data = dev->data;
	ETHERNET_TypeDef *heth = dev_data->heth;
	int ret;

	k_sem_take(&dev_data->sem, K_FOREVER);

	ret = mdio_ameba_polling(heth);

	if (ret != 0) {
		LOG_ERR("mdio bus is busy\n");
		ret = -ETIMEDOUT;
		goto exit;
	}

	heth->ETH_MIIAR = (BIT_FLAG | PHYADDRESS(prtad) | REGADDR4_0(regad) | data);

exit:
	k_sem_give(&dev_data->sem);
	return ret;
}

static int mdio_ameba_init(const struct device *dev)
{
	struct mdio_ameba_data *const dev_data = dev->data;
	const struct mdio_ameba_config *const config = dev->config;
	int ret;

	mido_ameba_clock_config();

	/*config pin*/
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
};

#define MDIO_AMEBA_DEVICE(inst)                                                                    \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static const struct mdio_ameba_config mdio_ameba_config_##inst = {                         \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
	};                                                                                         \
	static struct mdio_ameba_data mdio_ameba_data_##inst = {                                   \
		.heth = (ETHERNET_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(inst))};                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &mdio_ameba_init, NULL, &mdio_ameba_data_##inst,               \
			      &mdio_ameba_config_##inst, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,   \
			      &mdio_ameba_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_AMEBA_DEVICE)

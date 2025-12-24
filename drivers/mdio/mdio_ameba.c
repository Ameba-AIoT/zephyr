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

#if defined(CONFIG_ETH_AMEBA_MAC_TO_PHY_50M)
#define ETH_CLK_DBG_SEL_VAL 0xB
#elif defined(CONFIG_ETH_AMEBA_MAC_TO_PHY_25M)
#define ETH_CLK_DBG_SEL_VAL 0xC
#endif

struct mdio_ameba_data {
	struct k_sem sem;
	ETHERNET_TypeDef *heth;
};

struct mdio_ameba_config {
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
};

static void mdio_ameba_phy_clock_dir_config(void)
{
#if defined(CONFIG_ETH_AMEBA_MAC_TO_PHY_50M) || defined(CONFIG_ETH_AMEBA_MAC_TO_PHY_25M)
	/*phy uses external clk*/
	uint32_t temp = sys_read32(PINMUX_REG_BASE + REG_PINMUX_SUB_CTRL);
	temp |= PAD_BIT_DBG_CLK_FORCE;
	temp &= ~PAD_MASK_DBG_CLK0_SEL;
	temp |= PAD_DBG_CLK0_SEL(ETH_CLK_DBG_SEL_VAL);
	sys_write32(temp, PINMUX_REG_BASE + REG_PINMUX_SUB_CTRL);

	Pinmux_Config(_PA_12, PINMUX_FUNCTION_EXT_CLK_OUT);
#endif
}

void mdio_ameba_clock_source_config(void)
{
	uint32_t clk_source;
	uint32_t pll_clk;
	uint32_t div_value;

	clk_source = RCC_PeriphClockSourceGet(GMAC);

	switch (clk_source) {
	case CKSL_GMAC_EXT50M:
		return;

	case CKSL_GMAC_USB_PLL:
		RCC_PeriphClockDividerFENSet(USB_PLL_GMAC, ENABLE);
		/*Brought about by SOC clock structure or other configurations, DD is known*/
		RCC_PeriphClockDividerFENSet(SYS_PLL_GMAC, ENABLE);
		pll_clk = USB_PLL_ClkGet();
		break;

	case CKSL_GMAC_SYS_PLL:
		RCC_PeriphClockDividerFENSet(SYS_PLL_GMAC, ENABLE);

		RCC_PeriphClockDividerFENSet(USB_PLL_GMAC, ENABLE);
		pll_clk = SYS_PLL_ClkGet();
		break;

	default:
		LOG_ERR("Error: Unknown GMAC CLK Source 0x%x\n", clk_source);
		return;
	}

	if (pll_clk == 0) {
		LOG_ERR("Error: PLL Clock is 0\n");
		return;
	}
	div_value = pll_clk / 50000000;

	if (clk_source == CKSL_GMAC_USB_PLL) {
		RCC_PeriphClockDividerSet(USB_PLL_GMAC, div_value);
	} else if (clk_source == CKSL_GMAC_SYS_PLL) {
		RCC_PeriphClockDividerSet(SYS_PLL_GMAC, div_value);
	}
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
	const struct mdio_ameba_config *const dev_cfg = dev->config;
	struct mdio_ameba_data *const dev_data = dev->data;
	int ret;

	mdio_ameba_clock_source_config();

	const struct device *clock_dev = dev_cfg->clock_dev;

	clock_control_subsys_t clock_subsys = (clock_control_subsys_t)dev_cfg->clock_subsys;

	/* clock is shared, so do not bail out if already enabled */
	ret = clock_control_on(clock_dev, clock_subsys);
	if (ret < 0) {
		LOG_ERR("Cannot enable mac clock\n");
		return ret;
	}

	mdio_ameba_phy_clock_dir_config();

	/*config MDC/MDIO pin*/
	ret = pinctrl_apply_state(dev_cfg->pincfg, PINCTRL_STATE_DEFAULT);
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
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(inst))),                  \
		.clock_subsys =                                                                    \
			(clock_control_subsys_t *)DT_CLOCKS_CELL(DT_INST_PARENT(inst), idx),       \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
	};                                                                                         \
	static struct mdio_ameba_data mdio_ameba_data_##inst = {                                   \
		.heth = (ETHERNET_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(inst))};                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &mdio_ameba_init, NULL, &mdio_ameba_data_##inst,               \
			      &mdio_ameba_config_##inst, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,   \
			      &mdio_ameba_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_AMEBA_DEVICE)

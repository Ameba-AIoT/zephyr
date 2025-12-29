/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_mac

#include <ameba_soc.h>

#include <ethernet/eth_stats.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/random/random.h>
#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include "eth.h"

#define ETH_TODO 0

/* defined in soc kconfig */
#define CONFIG_ETH_RX_DES_NUM   4
#define CONFIG_ETH_TX_DES_NUM   4
/* Defines the size (unit: Bytes) of each Tx descriptor*/
#define CONFIG_ETH_TX_DESC_SIZE 20
/* Defines the size (unit: Bytes) of each Rx descriptor*/
#define CONFIG_ETH_RX_DESC_SIZE 16

LOG_MODULE_REGISTER(eth_ameba, CONFIG_ETHERNET_LOG_LEVEL);

#define MAC_RESET_TIMEOUT_MS 100

#define ETH_AMEBA_CAPS                                                                             \
	(ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE | ETHERNET_HW_VLAN |                         \
	 ETHERNET_HW_TX_CHKSUM_OFFLOAD | ETHERNET_HW_VLAN_TAG_STRIP | ETHERNET_PRIORITY_QUEUES |   \
	 ETHERNET_PROMISC_MODE)

uint8_t tx_descriptors[CONFIG_ETH_TX_DES_NUM][sizeof(ETH_TxDescTypeDef)] __nocache __aligned(32);

uint8_t rx_descriptors[CONFIG_ETH_RX_DES_NUM][sizeof(ETH_RxDescTypeDef)] __nocache __aligned(32);
struct eth_ameba_dma_data {

	ETH_TxDescTypeDef *eth_ameba_tx_descriptors;
	ETH_RxDescTypeDef *eth_ameba_rx_descriptors;

	__aligned(32) uint8_t rx_buf[CONFIG_ETH_RX_DES_NUM][NET_ETH_MAX_FRAME_SIZE];
	__aligned(32) uint8_t tx_buf[CONFIG_ETH_TX_DES_NUM][NET_ETH_MAX_FRAME_SIZE];
};

struct eth_ameba_dev_config {
	ETHERNET_TypeDef *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pcfg;
	void (*config_irq)(void);
};

struct eth_ameba_dev_data {
	struct net_if *iface;
	ETH_InitTypeDef *eth_ameba_init;
	uint8_t mac_addr[6];

	/*emac_hal_context_t hal;*/
	struct eth_ameba_dma_data *dma;
	uint8_t txb[NET_ETH_MAX_FRAME_SIZE];
	uint8_t rxb[NET_ETH_MAX_FRAME_SIZE];

	struct k_sem rx_sem;
	struct k_mutex tx_mutex;
	K_KERNEL_STACK_MEMBER(rx_thread_stack, CONFIG_ETH_AMEBA_RX_THREAD_STACK_SIZE);
	struct k_thread rx_thread;
};

static const struct device *eth_ameba_phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(0, phy_handle));
static const uint8_t eth_ameba_phy_addr = DT_REG_ADDR(DT_INST_PHANDLE(0, phy_handle));

static enum ethernet_hw_caps eth_ameba_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETH_AMEBA_CAPS;
}

static int eth_ameba_set_config(const struct device *dev, enum ethernet_config_type type,
				const struct ethernet_config *config)
{

	struct eth_ameba_dev_data *const dev_data = dev->data;
	int ret = -ENOTSUP;
	LOG_DBG("set config type = %x\n", type);
	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		LOG_DBG("Setting MAC address\n");
		memcpy(dev_data->mac_addr, config->mac_address.addr, NET_ETH_ADDR_LEN);
		Ethernet_SetMacAddr(dev_data->mac_addr);
		net_if_set_link_addr(dev_data->iface, dev_data->mac_addr,
				     sizeof(dev_data->mac_addr), NET_LINK_ETHERNET);
		ret = 0;
		break;
	case ETHERNET_CONFIG_TYPE_RX_CHECKSUM_SUPPORT:
		ret = 0;
		break;
	case ETHERNET_CONFIG_TYPE_TX_CHECKSUM_SUPPORT:
		ret = 0;
		break;
	case ETHERNET_CONFIG_TYPE_FILTER:
		return 0;
	default:
		break;
	}

	return ret;
}

static int eth_ameba_send(const struct device *dev, struct net_pkt *pkt)
{
	struct eth_ameba_dev_data *dev_data = dev->data;
	size_t len = net_pkt_get_len(pkt);
	u8 *rmii_buf = NULL;

	k_mutex_lock(&dev_data->tx_mutex, K_FOREVER);
	if (net_pkt_read(pkt, dev_data->txb, len)) {
		k_mutex_unlock(&dev_data->tx_mutex);
		return -EIO;
	}

	rmii_buf = Ethernet_GetTXPktInfo(dev_data->eth_ameba_init);
	LOG_DBG("rmii_buf = %x, tx temp =%x\n", (u32)rmii_buf, (u32)dev_data->txb);

	if (rmii_buf == NULL) {
		LOG_ERR("TX buffer not available!\n");
		k_mutex_unlock(&dev_data->tx_mutex);
		return -EAGAIN;
	}

	/* check length*/
	if (len > dev_data->eth_ameba_init->ETH_TxBufSize) {
		LOG_ERR("TX data length exceeds TX buffer size!\n");
		k_mutex_unlock(&dev_data->tx_mutex);
		return -EINVAL;
	}

	/* copy data to buffer */
	memcpy(rmii_buf, dev_data->txb, len);

	Ethernet_UpdateTXDESCAndSend(dev_data->eth_ameba_init, len);

	k_mutex_unlock(&dev_data->tx_mutex);

	return 0;
}

static struct net_pkt *eth_ameba_rx(struct eth_ameba_dev_data *const dev_data,
				    uint32_t *frames_remaining)
{
	ARG_UNUSED(frames_remaining);
	struct net_pkt *pkt = NULL;
	uint32_t receive_len = 0;
	uint32_t cache_line_mask = sys_cache_data_line_size_get() - 1;
	uint8_t *buf = Ethernet_GetRXPktInfo(dev_data->eth_ameba_init, &receive_len);

	memcpy(dev_data->rxb, buf, receive_len);
	/*Write data to mem*/
	sys_cache_data_flush_range((void *)((u32)dev_data->rxb & cache_line_mask),
				   (u32)receive_len);

	if (receive_len == 0) {
		/* Nothing to receive */
		return NULL;
	}

	pkt = net_pkt_rx_alloc_with_buffer(dev_data->iface, receive_len, AF_UNSPEC, 0, K_MSEC(100));
	if (pkt == NULL) {
		eth_stats_update_errors_rx(dev_data->iface);
		LOG_ERR("Could not allocate rx buffer");
		return NULL;
	}

	if (net_pkt_write(pkt, dev_data->rxb, receive_len) != 0) {
		LOG_ERR("Unable to write frame into the pkt");
		eth_stats_update_errors_rx(dev_data->iface);
		net_pkt_unref(pkt);
		return NULL;
	}

	return pkt;
}
FUNC_NORETURN static void eth_ameba_rx_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct eth_ameba_dev_data *const dev_data = dev->data;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		k_sem_take(&dev_data->rx_sem, K_FOREVER);

		uint32_t frames_remaining = 0;

		do {
			struct net_pkt *pkt = eth_ameba_rx(dev_data, &frames_remaining);
			if (pkt == NULL) {
				break;
			}

			if (net_recv_data(dev_data->iface, pkt) < 0) {
				/* Upper layers are not ready to receive packets */
				net_pkt_unref(pkt);
			} else {
				Ethernet_UpdateRXDESC(dev_data->eth_ameba_init);
			}
		} while (frames_remaining > 0);
	}
}

static void eth_ameba_isr(void *arg)
{
	const struct device *dev = arg;
	struct eth_ameba_dev_data *const dev_data = dev->data;

	uint32_t intr_status = Ethernet_GetINT();

	LOG_DBG("int_status = %x\n", intr_status);
	if ((intr_status & BIT_ISR_ROK) && (intr_status & BIT_IMR_ROK)) {
		Ethernet_ClearINT(BIT_ISR_ROK);
		k_sem_give(&dev_data->rx_sem);
	}

	if ((intr_status & BIT_ISR_RER_OVF) && (intr_status & BIT_IMR_RER_OVF)) {
		Ethernet_ClearINT(BIT_ISR_RER_OVF);
	}

	if ((intr_status & BIT_ISR_TOK_TI) && (intr_status & BIT_IMR_TOK_TI)) {
		Ethernet_ClearINT(BIT_ISR_TOK_TI);
	}

	if ((intr_status & BIT_ISR_LINKCHG) && (intr_status & BIT_IMR_LINKCHG)) {
		Ethernet_ClearINT(BIT_ISR_LINKCHG);
	}
}

#if ETH_TODO
#if DT_INST_NODE_HAS_PROP(0, ref_clk_output_gpios)
static int emac_config_apll_clock(void)
{
	uint32_t expt_freq = MHZ(50);
	uint32_t real_freq = 0;
	esp_err_t ret = periph_rtc_apll_freq_set(expt_freq, &real_freq);

	if (ret == RTK_ERR_INVALID_ARG) {
		LOG_ERR("Set APLL clock coefficients failed");
		return -EIO;
	}

	if (ret == RTK_ERR_INVALID_STATE) {
		LOG_INF("APLL is occupied already, it is working at %d Hz", real_freq);
	}

	/* If the difference of real APLL frequency
	 * is not within 50 ppm, i.e. 2500 Hz,
	 * the APLL is unavailable
	 */
	if (abs((int)real_freq - (int)expt_freq) > 2500) {
		LOG_ERR("The APLL is working at an unusable frequency");
		return -EIO;
	}

	return 0;
}
#endif /* DT_INST_NODE_HAS_PROP(0, ref_clk_output_gpios) */
#endif /* ETH_TODO */

static void phy_link_state_changed(const struct device *phy_dev, struct phy_link_state *state,
				   void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	const struct eth_ameba_dev_config *dev_cfg = dev->config;
	struct eth_ameba_dev_data *const dev_data = dev->data;
	ETHERNET_TypeDef *eth_ameba_dev = dev_cfg->base;
	ARG_UNUSED(phy_dev);

	/* Check if auto-negotiation is complete */
	if (!GET_NWCOMPLETE(eth_ameba_dev->ETH_MSR)) {
		LOG_ERR("Auto-Negotiation Fail\n");
		return;
	}
	/* Turn on/off ca */
	if (state->is_up) {
		LOG_DBG("Phy Link is up\n");
		net_eth_carrier_on(dev_data->iface);
	} else {
		LOG_DBG("Phy Link is down\n");
		net_eth_carrier_off(dev_data->iface);
	}
}

int eth_ameba_initialize(const struct device *dev)
{
	const struct eth_ameba_dev_config *dev_cfg = dev->config;
	struct eth_ameba_dev_data *const dev_data = dev->data;
	ETHERNET_TypeDef *eth_ameba_dev = dev_cfg->base;
	uint32_t rand_num;
	uint16_t tmp;
	uint8_t mac[6] = {0};
	int res;

	k_mutex_init(&dev_data->tx_mutex);
	k_sem_init(&dev_data->rx_sem, 0, 1);
	const struct device *clock_dev = dev_cfg->clock_dev;

	clock_control_subsys_t clock_subsys = (clock_control_subsys_t)dev_cfg->clock_subsys;

	/* clock is shared, so do not bail out if already enabled */
	res = clock_control_on(clock_dev, clock_subsys);
	if (res < 0 && res != -EALREADY) {
		goto err;
	}

	/* configure pinmux */
	res = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (res < 0) {
		LOG_ERR("Could not configure ethernet pins");
		return res;
	}

	/* Initialize struct*/
	Ethernet_StructInit(dev_data->eth_ameba_init);

	/* Register mac event callback*/
	dev_data->eth_ameba_init->callback = NULL;

	Ethernet_AutoPolling(DISABLE);
	/*50ms*/
	k_busy_wait(50000);

	/* Reset MAC */
	eth_ameba_dev->ETH_CR |= BIT_RST;
	do {
	} while (eth_ameba_dev->ETH_CR & BIT_RST);

	/* reset phy */
	LOG_DBG("phy addr = %x\n", eth_ameba_phy_addr);

	PHY_SoftWareReset(eth_ameba_phy_addr);

	k_busy_wait(200000);

	PHY_SetRefclkDir(eth_ameba_phy_addr, dev_data->eth_ameba_init->ETH_RefClkDirec);

	Ethernet_ReadPhyReg(eth_ameba_phy_addr, FEPHY_REG_ADDR_0, &tmp);
	LOG_DBG("page0 reg0=0x%x\n", tmp);

	Ethernet_ReadPhyReg(eth_ameba_phy_addr, FEPHY_REG_ADDR_1, &tmp);
	LOG_DBG("page0 reg1=0x%x\n", tmp);

	Ethernet_ReadPhyReg(eth_ameba_phy_addr, FEPHY_REG_ADDR_2, &tmp);
	LOG_DBG("page0 reg2=0x%x\n", tmp);

	Ethernet_ReadPhyReg(eth_ameba_phy_addr, FEPHY_REG_ADDR_3, &tmp);
	LOG_DBG("page0 reg3=0x%x\n", tmp);

	/* descriptor */
	memset(dev_data->dma->eth_ameba_rx_descriptors, 0,
	       CONFIG_ETH_RX_DES_NUM * sizeof(ETH_RxDescTypeDef));
	memset(dev_data->dma->eth_ameba_rx_descriptors, 0,
	       CONFIG_ETH_TX_DES_NUM * sizeof(ETH_TxDescTypeDef));
	memset(dev_data->dma->tx_buf, 0, CONFIG_ETH_RX_DES_NUM * NET_ETH_MAX_FRAME_SIZE);
	memset(dev_data->dma->rx_buf, 0, CONFIG_ETH_TX_DES_NUM * NET_ETH_MAX_FRAME_SIZE);

	PHY_RestartAutoNego(eth_ameba_phy_addr);

	/* Tx settings */
	eth_ameba_dev->ETH_MSR |= BIT_REG_RMII2MII_EN;
	Ethernet_SetRefclkDirec(dev_data->eth_ameba_init->ETH_RefClkDirec);

	eth_ameba_dev->ETH_MSR |= BIT_TXFCE;
	eth_ameba_dev->ETH_MSR |= BIT_RXFCE;
	eth_ameba_dev->ETH_MSR |= REFCLK_PHASE(dev_data->eth_ameba_init->ETH_RefClkPhase);

	eth_ameba_dev->ETH_TCR &= ~MASK_IFG2_0;
	eth_ameba_dev->ETH_TCR |= IFG2_0(eth_ifg_time_3);

	eth_ameba_dev->ETH_TCR &= ~MASK_LBK;
	eth_ameba_dev->ETH_TCR |= LBK(eth_normal_op);

	/*set mac addr randomly*/
	rand_num = sys_rand32_get();

	mac[0] = (rand_num & 0xFE) | 0x02;
	for (int i = 1; i < 6; ++i) {
		mac[i] = sys_rand32_get() & 0xFF;
	}
	memcpy(dev_data->mac_addr, mac, NET_ETH_ADDR_LEN);

	Ethernet_SetMacAddr(dev_data->mac_addr);
	/* Accept error/runt/broadcast/multicast packets, etc.*/
	eth_ameba_dev->ETH_RCR = BIT_AAP | BIT_APM | BIT_AM | BIT_AB | BIT_AR | BIT_AER;

	/* RMII->ETH_CR |= BIT_RXJUMBO; Support jumbo packet*/
	eth_ameba_dev->ETH_ETHRNTRXCPU_DES_NUM1 =
		RX_PSE_DES_THRES_OFF_1_7_0(dev_data->eth_ameba_init->ETH_RxDescNum - 1) |
		RX_PSE_DES_THRES_ON_1_7_0(0x0) |
		ETHRNTRXCPU_DES_NUM_1_7_0(dev_data->eth_ameba_init->ETH_RxDescNum);

	/* I/O command-descriptor format: format = 011, support 1GB addressing */
	eth_ameba_dev->ETH_IO_CMD1 = DSC_FORMAT_EXTRA(0x3) | BIT_EN_4GB;
	/* short desc. format = 1, Tx & Rx FIFO threshold = 256 bytes */
	eth_ameba_dev->ETH_ETHER_IO_CMD = RXFTH(dev_data->eth_ameba_init->ETH_RxThreshold) |
					  TSH(dev_data->eth_ameba_init->ETH_TxThreshold) |
					  BIT_SHORTDESFORMAT;
	/* Set Rx/Tx buffer size */
	dev_data->eth_ameba_init->ETH_RxBufSize = NET_ETH_MAX_FRAME_SIZE;
	dev_data->eth_ameba_init->ETH_TxBufSize = NET_ETH_MAX_FRAME_SIZE;

	dev_data->eth_ameba_init->ETH_TxDescNum = CONFIG_ETH_TX_DES_NUM;
	dev_data->eth_ameba_init->ETH_RxDescNum = CONFIG_ETH_RX_DES_NUM;
	dev_data->eth_ameba_init->ETH_TxDesc =
		(ETH_TxDescTypeDef *)dev_data->dma->eth_ameba_tx_descriptors;
	dev_data->eth_ameba_init->ETH_RxDesc =
		(ETH_RxDescTypeDef *)dev_data->dma->eth_ameba_rx_descriptors;
	for (int i = 0; i < CONFIG_ETH_RX_DES_NUM; i++) {
		if (i == (CONFIG_ETH_RX_DES_NUM - 1)) {
			dev_data->eth_ameba_init->ETH_RxDesc[i].dw1 =
				BIT31 | BIT30 | NET_ETH_MAX_FRAME_SIZE;
		} else {
			dev_data->eth_ameba_init->ETH_RxDesc[i].dw1 =
				BIT31 | NET_ETH_MAX_FRAME_SIZE;
		}
		dev_data->eth_ameba_init->ETH_RxDesc[i].addr = (u32)dev_data->dma->rx_buf[i];
		dev_data->eth_ameba_init->ETH_RxDesc[i].dw2 = 0;
		dev_data->eth_ameba_init->ETH_RxDesc[i].dw3 = 0;
	}
	for (int i = 0; i < CONFIG_ETH_TX_DES_NUM; i++) {
		dev_data->eth_ameba_init->ETH_TxDesc[i].dw1 =
			FEMAC_TX_DSC_BIT_IPCS | FEMAC_TX_DSC_BIT_L4CS;
		dev_data->eth_ameba_init->ETH_TxDesc[i].addr = (u32)dev_data->dma->tx_buf[i];
		dev_data->eth_ameba_init->ETH_TxDesc[i].dw2 = 0;
		dev_data->eth_ameba_init->ETH_TxDesc[i].dw3 = 0;
		dev_data->eth_ameba_init->ETH_TxDesc[i].dw4 = 0;
	}

	eth_ameba_dev->ETH_TXFDP1 = (u32)dev_data->dma->eth_ameba_tx_descriptors;
	eth_ameba_dev->ETH_RX_FDP1 = (u32)dev_data->dma->eth_ameba_rx_descriptors;
	LOG_DBG("TXFDP1 = %x, tx desc = %x\n", eth_ameba_dev->ETH_TXFDP1,
		(u32)dev_data->dma->eth_ameba_tx_descriptors);

	eth_ameba_dev->ETH_RX_RINGSIZE1 |= RXRINGSIZE_1_LOW(CONFIG_ETH_RX_DES_NUM - 1);
	/* enable Tx & Rx */
	eth_ameba_dev->ETH_ETHER_IO_CMD |= BIT_TE | BIT_RE;

	/* isr & imr */
	eth_ameba_dev->ETH_ISR_AND_IMR = dev_data->eth_ameba_init->ETH_IntMaskAndStatus;

	/* enable auto-polling */
	Ethernet_AutoPolling(ENABLE);

	/*Enable irq*/
	dev_cfg->config_irq();

	k_tid_t tid = k_thread_create(&dev_data->rx_thread, dev_data->rx_thread_stack,
				      K_KERNEL_STACK_SIZEOF(dev_data->rx_thread_stack),
				      eth_ameba_rx_thread, (void *)dev, NULL, NULL,
				      CONFIG_ETH_AMEBA_RX_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);
	if (IS_ENABLED(CONFIG_THREAD_NAME)) {
		k_thread_name_set(tid, "ameba_eth");
	}

	/*Enable Rx ring*/
	eth_ameba_dev->ETH_IO_CMD1 |= BIT_RXRING1;

#if ETH_TODO
	/* Configure phy for Media-Independent Interface (MII) or
	 * Reduced Media-Independent Interface (RMII) mode
	 */
	const char *phy_connection_type = DT_INST_PROP_OR(0, phy_connection_type, "rmii");

	if (strcmp(phy_connection_type, "rmii") == 0) {
		emac_hal_iomux_init_rmii();
#if DT_INST_NODE_HAS_PROP(0, ref_clk_output_gpios)
		BUILD_ASSERT(DT_INST_GPIO_PIN(0, ref_clk_output_gpios) == 16 ||
				     DT_INST_GPIO_PIN(0, ref_clk_output_gpios) == 17,
			     "Only GPIOxx are allowed as a GPIO REF_CLK source!");
		int ref_clk_gpio = DT_INST_GPIO_PIN(0, ref_clk_output_gpios);

		emac_hal_iomux_rmii_clk_output(ref_clk_gpio);
		emac_ll_clock_enable_rmii_output(dev_data->hal.ext_regs);
		periph_rtc_apll_acquire();
		res = emac_config_apll_clock();
		if (res != 0) {
			goto err;
		}
		rtc_clk_apll_enable(true);
#else
		emac_hal_iomux_rmii_clk_input();
		emac_ll_clock_enable_rmii_input(dev_data->hal.ext_regs);
#endif
	} else if (strcmp(phy_connection_type, "mii") == 0) {
		emac_hal_iomux_init_mii();
		emac_ll_clock_enable_mii(dev_data->hal.ext_regs);
	} else {
		res = -EINVAL;
		goto err;
	}

	/* Reset mac registers and wait until ready */
	emac_ll_reset(dev_data->hal.dma_regs);
	bool reset_success = false;

	for (uint32_t t_ms = 0; t_ms < MAC_RESET_TIMEOUT_MS; t_ms += 10) {
		/* Busy wait rather than sleep in case kernel is not yet initialized */
		k_busy_wait(10 * 1000);
		if (emac_ll_is_reset_done(dev_data->hal.dma_regs)) {
			reset_success = true;
			break;
		}
	}
	if (!reset_success) {
		res = -ETIMEDOUT;
		goto err;
	}
#endif
err:
	return res;
}

static const struct device *eth_ameba_phy_get(const struct device *dev)
{
	ARG_UNUSED(dev);
	return eth_ameba_phy_dev;
}

static void eth_ameba_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_ameba_dev_data *dev_data = dev->data;

	dev_data->iface = iface;

	net_if_set_link_addr(iface, dev_data->mac_addr, sizeof(dev_data->mac_addr),
			     NET_LINK_ETHERNET);

	ethernet_init(iface);

	/* Do not start the interface until PHY link is up */
	net_if_carrier_off(iface);

	if (device_is_ready(eth_ameba_phy_dev)) {
		phy_link_callback_set(eth_ameba_phy_dev, phy_link_state_changed, (void *)dev);
	} else {
		LOG_ERR("PHY device not ready");
	}
}

static const struct ethernet_api eth_ameba_api = {
	.iface_api.init = eth_ameba_iface_init,
	.get_capabilities = eth_ameba_caps,
	.set_config = eth_ameba_set_config,
	.get_phy = eth_ameba_phy_get,
	.send = eth_ameba_send,
};

#define ETH_AMEBA_ETH_IRQ_INST(inst)                                                               \
	static void config_eth_ameba_eth_##inst##_irq(void)                                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ(inst, irq), DT_INST_IRQ(inst, priority), eth_ameba_isr,    \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ(inst, irq));                                                \
	}

#define ETH_AMEBA_ETH_INIT(inst)                                                                   \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	ETH_AMEBA_ETH_IRQ_INST(inst)                                                               \
                                                                                                   \
	static const struct eth_ameba_dev_config eth_ameba_cfg_##inst = {                          \
		.base = (ETHERNET_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(inst)),                     \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(inst))),                  \
		.clock_subsys =                                                                    \
			(clock_control_subsys_t *)DT_CLOCKS_CELL(DT_INST_PARENT(inst), idx),       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.config_irq = config_eth_ameba_eth_##inst##_irq,                                   \
	};                                                                                         \
                                                                                                   \
	static struct eth_ameba_dma_data eth_ameba_dma_data_##inst = {                             \
		.eth_ameba_tx_descriptors = (ETH_TxDescTypeDef *)tx_descriptors,                   \
		.eth_ameba_rx_descriptors = (ETH_RxDescTypeDef *)rx_descriptors,                   \
	};                                                                                         \
	static ETH_InitTypeDef eth_init_struct_##inst = {0};                                       \
                                                                                                   \
	static struct eth_ameba_dev_data eth_ameba_dev_##inst = {                                  \
		.eth_ameba_init = &eth_init_struct_##inst,                                         \
		.dma = &eth_ameba_dma_data_##inst,                                                 \
	};                                                                                         \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(inst, eth_ameba_initialize, NULL, &eth_ameba_dev_##inst,     \
				      &eth_ameba_cfg_##inst, CONFIG_ETH_INIT_PRIORITY,             \
				      &eth_ameba_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_AMEBA_ETH_INIT)

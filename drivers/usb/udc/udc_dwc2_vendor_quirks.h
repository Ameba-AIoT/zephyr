/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC2_VENDOR_QUIRKS_H
#define ZEPHYR_DRIVERS_USB_UDC_DWC2_VENDOR_QUIRKS_H

#include "udc_dwc2.h"

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/usb/udc.h>

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32f4_fsotg)

#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <usb_dwc2_hw.h>

struct usb_dw_stm32_clk {
	const struct device *const dev;
	const struct stm32_pclken *const pclken;
	size_t pclken_len;
};

#define DT_DRV_COMPAT snps_dwc2

static inline int stm32f4_fsotg_enable_clk(const struct usb_dw_stm32_clk *const clk)
{
	int ret;

	if (!device_is_ready(clk->dev)) {
		return -ENODEV;
	}

	if (clk->pclken_len > 1) {
		uint32_t clk_rate;

		ret = clock_control_configure(clk->dev,
					      (void *)&clk->pclken[1],
					      NULL);
		if (ret) {
			return ret;
		}

		ret = clock_control_get_rate(clk->dev,
					     (void *)&clk->pclken[1],
					     &clk_rate);
		if (ret) {
			return ret;
		}

		if (clk_rate != MHZ(48)) {
			return -ENOTSUP;
		}
	}

	return clock_control_on(clk->dev, (void *)&clk->pclken[0]);
}

static inline int stm32f4_fsotg_enable_phy(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	mem_addr_t ggpio_reg = (mem_addr_t)&config->base->ggpio;

	sys_set_bits(ggpio_reg, USB_DWC2_GGPIO_STM32_PWRDWN | USB_DWC2_GGPIO_STM32_VBDEN);

	return 0;
}

static inline int stm32f4_fsotg_disable_phy(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	mem_addr_t ggpio_reg = (mem_addr_t)&config->base->ggpio;

	sys_clear_bits(ggpio_reg, USB_DWC2_GGPIO_STM32_PWRDWN | USB_DWC2_GGPIO_STM32_VBDEN);

	return 0;
}

#define QUIRK_STM32F4_FSOTG_DEFINE(n)						\
	static const struct stm32_pclken pclken_##n[] = STM32_DT_INST_CLOCKS(n);\
										\
	static const struct usb_dw_stm32_clk stm32f4_clk_##n = {		\
		.dev = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),			\
		.pclken = pclken_##n,						\
		.pclken_len = DT_INST_NUM_CLOCKS(n),				\
	};									\
										\
	static int stm32f4_fsotg_enable_clk_##n(const struct device *dev)	\
	{									\
		return stm32f4_fsotg_enable_clk(&stm32f4_clk_##n);		\
	}									\
										\
	struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {			\
		.pre_enable = stm32f4_fsotg_enable_clk_##n,			\
		.post_enable = stm32f4_fsotg_enable_phy,			\
		.disable = stm32f4_fsotg_disable_phy,				\
		.irq_clear = NULL,						\
	};


DT_INST_FOREACH_STATUS_OKAY(QUIRK_STM32F4_FSOTG_DEFINE)

#undef DT_DRV_COMPAT

#endif /*DT_HAS_COMPAT_STATUS_OKAY(st_stm32f4_fsotg) */

#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_usbhs)

#define DT_DRV_COMPAT snps_dwc2

#include <nrfs_backend_ipc_service.h>
#include <nrfs_usb.h>

#define USBHS_DT_WRAPPER_REG_ADDR(n) UINT_TO_POINTER(DT_INST_REG_ADDR_BY_NAME(n, wrapper))

/*
 * On USBHS, we cannot access the DWC2 register until VBUS is detected and
 * valid. If the user tries to force usbd_enable() and the corresponding
 * udc_enable() without a "VBUS ready" notification, the event wait will block
 * until a valid VBUS signal is detected or until the
 * CONFIG_UDC_DWC2_USBHS_VBUS_READY_TIMEOUT timeout expires.
 */
static K_EVENT_DEFINE(usbhs_events);
#define USBHS_VBUS_READY	BIT(0)

static void usbhs_vbus_handler(nrfs_usb_evt_t const *p_evt, void *const context)
{
	const struct device *dev = context;

	switch (p_evt->type) {
	case NRFS_USB_EVT_VBUS_STATUS_CHANGE:
		LOG_DBG("USBHS new status, pll_ok = %d vreg_ok = %d vbus_detected = %d",
			p_evt->usbhspll_ok, p_evt->vregusb_ok, p_evt->vbus_detected);

		if (p_evt->usbhspll_ok && p_evt->vregusb_ok && p_evt->vbus_detected) {
			k_event_post(&usbhs_events, USBHS_VBUS_READY);
			udc_submit_event(dev, UDC_EVT_VBUS_READY, 0);
		} else {
			k_event_set_masked(&usbhs_events, 0, USBHS_VBUS_READY);
			udc_submit_event(dev, UDC_EVT_VBUS_REMOVED, 0);
		}

		break;
	case NRFS_USB_EVT_REJECT:
		LOG_ERR("Request rejected");
		break;
	default:
		LOG_ERR("Unknown event type 0x%x", p_evt->type);
		break;
	}
}

static inline int usbhs_enable_nrfs_service(const struct device *dev)
{
	nrfs_err_t nrfs_err;
	int err;

	err = nrfs_backend_wait_for_connection(K_MSEC(1000));
	if (err) {
		LOG_INF("NRFS backend connection timeout");
		return err;
	}

	nrfs_err = nrfs_usb_init(usbhs_vbus_handler);
	if (nrfs_err != NRFS_SUCCESS) {
		LOG_ERR("Failed to init NRFS VBUS handler: %d", nrfs_err);
		return -EIO;
	}

	nrfs_err = nrfs_usb_enable_request((void *)dev);
	if (nrfs_err != NRFS_SUCCESS) {
		LOG_ERR("Failed to enable NRFS VBUS service: %d", nrfs_err);
		return -EIO;
	}

	return 0;
}

static inline int usbhs_enable_core(const struct device *dev)
{
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);
	k_timeout_t timeout = K_FOREVER;

	#if CONFIG_NRFS_HAS_VBUS_DETECTOR_SERVICE
	if (CONFIG_UDC_DWC2_USBHS_VBUS_READY_TIMEOUT) {
		timeout = K_MSEC(CONFIG_UDC_DWC2_USBHS_VBUS_READY_TIMEOUT);
	}
	#endif

	if (!k_event_wait(&usbhs_events, USBHS_VBUS_READY, false, K_NO_WAIT)) {
		LOG_WRN("VBUS is not ready, block udc_enable()");
		if (!k_event_wait(&usbhs_events, USBHS_VBUS_READY, false, timeout)) {
			return -ETIMEDOUT;
		}
	}

	wrapper->ENABLE = USBHS_ENABLE_PHY_Msk | USBHS_ENABLE_CORE_Msk;
	wrapper->TASKS_START = 1UL;

	/* Wait for clock to start to avoid hang on too early register read */
	k_busy_wait(1);

	/* Enable interrupts */
	wrapper->INTENSET = 1UL;

	return 0;
}

static inline int usbhs_disable_core(const struct device *dev)
{
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);

	/* Disable interrupts */
	wrapper->INTENCLR = 1UL;

	wrapper->ENABLE = 0UL;
	wrapper->TASKS_START = 1UL;

	return 0;
}

static inline int usbhs_disable_nrfs_service(const struct device *dev)
{
	nrfs_err_t nrfs_err;

	nrfs_err = nrfs_usb_disable_request((void *)dev);
	if (nrfs_err != NRFS_SUCCESS) {
		LOG_ERR("Failed to disable NRFS VBUS service: %d", nrfs_err);
		return -EIO;
	}

	nrfs_usb_uninit();

	return 0;
}

static inline int usbhs_irq_clear(const struct device *dev)
{
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);

	wrapper->EVENTS_CORE = 0UL;

	return 0;
}

static inline int usbhs_init_caps(const struct device *dev)
{
	struct udc_data *data = dev->data;

	data->caps.can_detect_vbus = true;
	data->caps.hs = true;

	return 0;
}

static inline int usbhs_is_phy_clk_off(const struct device *dev)
{
	return !k_event_test(&usbhs_events, USBHS_VBUS_READY);
}

static inline int usbhs_post_hibernation_entry(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = config->base;
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);

	sys_set_bits((mem_addr_t)&base->pcgcctl, USB_DWC2_PCGCCTL_GATEHCLK);

	sys_write32(0x87, (mem_addr_t)wrapper + 0xC80);
	sys_write32(0x87, (mem_addr_t)wrapper + 0xC84);
	sys_write32(1, (mem_addr_t)wrapper + 0x004);

	return 0;
}

static inline int usbhs_pre_hibernation_exit(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = config->base;
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);

	sys_clear_bits((mem_addr_t)&base->pcgcctl, USB_DWC2_PCGCCTL_GATEHCLK);

	wrapper->TASKS_START = 1;
	sys_write32(0, (mem_addr_t)wrapper + 0xC80);
	sys_write32(0, (mem_addr_t)wrapper + 0xC84);

	return 0;
}

#define QUIRK_NRF_USBHS_DEFINE(n)						\
	struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {			\
		.init = usbhs_enable_nrfs_service,				\
		.pre_enable = usbhs_enable_core,				\
		.disable = usbhs_disable_core,					\
		.shutdown = usbhs_disable_nrfs_service,				\
		.irq_clear = usbhs_irq_clear,					\
		.caps = usbhs_init_caps,					\
		.is_phy_clk_off = usbhs_is_phy_clk_off,				\
		.post_hibernation_entry = usbhs_post_hibernation_entry,		\
		.pre_hibernation_exit = usbhs_pre_hibernation_exit,		\
	};

DT_INST_FOREACH_STATUS_OKAY(QUIRK_NRF_USBHS_DEFINE)

#undef DT_DRV_COMPAT

#endif /*DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_usbhs) */

#if DT_HAS_COMPAT_STATUS_OKAY(realtek_ameba_udc)

#include <zephyr/sys/sys_io.h>
#include <usb_dwc2_hw.h>
#include <zephyr/drivers/clock_control.h>

#define DT_DRV_COMPAT snps_dwc2

#define USB_ADDON_REG_CTRL_OFFSET				0x04
#define USB_ADDON_REG_CTRL_BIT_DIS_SUSPEND		BIT(1)
#define USB_ADDON_REG_CTRL_BIT_UPLL_CKRDY		BIT(5)
#define USB_ADDON_REG_CTRL_BIT_USB_OTG_RST		BIT(8)

#define PLL_UPLL_CTRL0_OFFSET					0x50
#define PLL_BIT_USB2_DIGOTGPADEN     BIT(28)
#define PLL_BIT_USB2_DIGPADEN        BIT(26)
#define PLL_BIT_USB_DPHY_EN          BIT(20)
#define PLL_BIT_PWC_UAHV_ALIVE       BIT(18)

#define UDC_DWC2_DT_INST_CORE_REG_ADDR(n)						\
	COND_CODE_1(DT_NUM_REGS(DT_DRV_INST(n)), (DT_INST_REG_ADDR(n)),		\
		    (DT_INST_REG_ADDR_BY_NAME(n, core)))

#define UDC_DWC2_DT_INST_USB_ADDON_REG_ADDR(n)						\
	COND_CODE_1(DT_NUM_REGS(DT_DRV_INST(n)), (DT_INST_REG_ADDR(n)),		\
		    (DT_INST_REG_ADDR_BY_NAME(n, usb_addon)))

#define UDC_DWC2_DT_INST_PLL_REG_ADDR(n)						\
	COND_CODE_1(DT_NUM_REGS(DT_DRV_INST(n)), (DT_INST_REG_ADDR(n)),		\
		    (DT_INST_REG_ADDR_BY_NAME(n, pll)))

struct realtek_usb_clk {
	const struct device *const dev;
	const clock_control_subsys_t clock_subsys;
};

static inline int usb_hw_init(const struct realtek_usb_clk *const clk)
{
	mem_addr_t usb_addon = (mem_addr_t)UDC_DWC2_DT_INST_USB_ADDON_REG_ADDR(0);
	mem_addr_t pll = (mem_addr_t)UDC_DWC2_DT_INST_PLL_REG_ADDR(0);
	uint32_t reg;

	if (!device_is_ready(clk->dev)) {
		return -ENODEV;
	}

	clock_control_on(clk->dev, (clock_control_subsys_t)clk->clock_subsys);

	reg = sys_read32((mem_addr_t)pll + PLL_UPLL_CTRL0_OFFSET);
	reg &= ~(PLL_BIT_USB2_DIGPADEN | PLL_BIT_USB2_DIGOTGPADEN);
	reg |= PLL_BIT_PWC_UAHV_ALIVE | PLL_BIT_USB_DPHY_EN;
	sys_write32(reg, (mem_addr_t)pll + PLL_UPLL_CTRL0_OFFSET);
	k_busy_wait(34);

	reg = sys_read32((mem_addr_t)usb_addon + USB_ADDON_REG_CTRL_OFFSET);
	reg |= USB_ADDON_REG_CTRL_BIT_USB_OTG_RST;
	sys_write32(reg, (mem_addr_t)usb_addon + USB_ADDON_REG_CTRL_OFFSET);

	return 0;
}

static inline int usb_hw_deinit(const struct realtek_usb_clk *const clk)
{
	mem_addr_t usb_addon = (mem_addr_t)UDC_DWC2_DT_INST_USB_ADDON_REG_ADDR(0);
	mem_addr_t pll = (mem_addr_t)UDC_DWC2_DT_INST_PLL_REG_ADDR(0);
	uint32_t reg;

	if (!device_is_ready(clk->dev)) {
		return -ENODEV;
	}

	reg = sys_read32((mem_addr_t)usb_addon + USB_ADDON_REG_CTRL_OFFSET);
	reg &= ~USB_ADDON_REG_CTRL_BIT_USB_OTG_RST;
	sys_write32(reg, (mem_addr_t)usb_addon + USB_ADDON_REG_CTRL_OFFSET);

	reg = sys_read32((mem_addr_t)pll + PLL_UPLL_CTRL0_OFFSET);
	reg &=  ~(PLL_BIT_PWC_UAHV_ALIVE | PLL_BIT_USB_DPHY_EN);
	reg |= PLL_BIT_USB2_DIGPADEN | PLL_BIT_USB2_DIGOTGPADEN;
	sys_write32(reg, (mem_addr_t)pll + PLL_UPLL_CTRL0_OFFSET);

	clock_control_off(clk->dev, (clock_control_subsys_t)clk->clock_subsys);

	return 0;
}

static inline int usbd_init_caps(const struct device *dev)
{
	struct udc_data *data = dev->data;

	data->caps.hs = true;
	data->caps.rwup = true;
	data->caps.out_ack = false;
	data->caps.mps0 = UDC_MPS0_64;

	return 0;
}

#define QUIRK_REALTEK_USB_OTG_DEFINE(n)						\
	static const struct realtek_usb_clk realtek_usb_clk_##n = {		\
		.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, idx),   \
	};									\
										\
	static int usbd_pre_enable(const struct device *dev)	\
	{									\
		return usb_hw_init(&realtek_usb_clk_##n);		\
	}									\
	static int usbd_disable(const struct device *dev)	\
	{									\
		return usb_hw_deinit(&realtek_usb_clk_##n);		\
	}									\
							\
	struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {	\
		.pre_enable = usbd_pre_enable,		\
		.disable = usbd_disable,			\
		.irq_clear = NULL,					\
		.caps = usbd_init_caps,				\
	};

DT_INST_FOREACH_STATUS_OKAY(QUIRK_REALTEK_USB_OTG_DEFINE)

#undef DT_DRV_COMPAT

#endif /*DT_HAS_COMPAT_STATUS_OKAY(amebad_otg) */

/* Add next vendor quirks definition above this line */

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC2_VENDOR_QUIRKS_H */

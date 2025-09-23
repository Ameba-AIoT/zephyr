/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_ili9806.h"
#include "display_ili9xxx.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(display_ili9806, CONFIG_DISPLAY_LOG_LEVEL);

int ili9806_regs_init(const struct device *dev)
{
	const struct ili9xxx_config *config = dev->config;
	const struct ili9806_regs *regs = config->regs;
	int r;

	/* Extended Command Set */
	LOG_HEXDUMP_DBG(regs->extccmdset, ILI9806_EXTCCMDSET_LEN, "EXTCCMDSET");
	r = ili9xxx_transmit(dev, ILI9806_EXTCCMDSET, regs->extccmdset, ILI9806_EXTCCMDSET_LEN);
	if (r < 0) {
		return r;
	}

	/* GIP Parameters */
	LOG_HEXDUMP_DBG(regs->gip1, ILI9806_GIP1_LEN, "GIP1");
	r = ili9xxx_transmit(dev, ILI9806_GIP1, regs->gip1, ILI9806_GIP1_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->gip2, ILI9806_GIP2_LEN, "GIP2");
	r = ili9xxx_transmit(dev, ILI9806_GIP2, regs->gip2, ILI9806_GIP2_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->gip3, ILI9806_GIP3_LEN, "GIP3");
	r = ili9xxx_transmit(dev, ILI9806_GIP3, regs->gip3, ILI9806_GIP3_LEN);
	if (r < 0) {
		return r;
	}

	/* Power & Voltage Control */
	LOG_HEXDUMP_DBG(regs->vcomcontrol, ILI9806_VCOMCONTROL_LEN, "VCOMCONTROL");
	r = ili9xxx_transmit(dev, ILI9806_VCOMCONTROL, regs->vcomcontrol, ILI9806_VCOMCONTROL_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->envolreg, ILI9806_ENVOLREG_LEN, "ENVOLREG");
	r = ili9xxx_transmit(dev, ILI9806_ENVOLREG, regs->envolreg, ILI9806_ENVOLREG_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->pwctrl1, ILI9806_PWCTRL1_LEN, "PWCTRL1");
	r = ili9xxx_transmit(dev, ILI9806_PWCTRL1, regs->pwctrl1, ILI9806_PWCTRL1_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->avdd, ILI9806_AVDD_LEN, "AVDD");
	r = ili9xxx_transmit(dev, ILI9806_AVDD, regs->avdd, ILI9806_AVDD_LEN);
	if (r < 0) {
		return r;
	}

	/* Engineer Mode */
	LOG_HEXDUMP_DBG(regs->engineerset, ILI9806_ENGINEERSET_LEN, "ENGINEERSET");
	r = ili9xxx_transmit(dev, ILI9806_ENGINEERSET, regs->engineerset, ILI9806_ENGINEERSET_LEN);
	if (r < 0) {
		return r;
	}

	/* Digital Voltage */
	LOG_HEXDUMP_DBG(regs->dvdd, ILI9806_DVDD_LEN, "DVDD");
	r = ili9xxx_transmit(dev, ILI9806_DVDD, regs->dvdd, ILI9806_DVDD_LEN);
	if (r < 0) {
		return r;
	}

	/* Display Settings */
	LOG_HEXDUMP_DBG(regs->invesiontype, ILI9806_INVESION_LEN, "INVESIONTYPE");
	r = ili9xxx_transmit(dev, ILI9806_INVESION, regs->invesiontype, ILI9806_INVESION_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->resolutionctrl, ILI9806_RESCTRL_LEN, "RESOLUTIONCTRL");
	r = ili9xxx_transmit(dev, ILI9806_RESCTRL, regs->resolutionctrl, ILI9806_RESCTRL_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->frameratectrl, ILI9806_FRMRTCTRL_LEN, "FRAMERATECTRL");
	r = ili9xxx_transmit(dev, ILI9806_FRMRTCTRL, regs->frameratectrl, ILI9806_FRMRTCTRL_LEN);
	if (r < 0) {
		return r;
	}

	/* Timing Control */
	LOG_HEXDUMP_DBG(regs->creqpcsdt, ILI9806_CREQPCSDT_LEN, "CREQPCSDT");
	r = ili9xxx_transmit(dev, ILI9806_CREQPCSDT, regs->creqpcsdt, ILI9806_CREQPCSDT_LEN);
	if (r < 0) {
		return r;
	}

	/* Power Control 2 */
	LOG_HEXDUMP_DBG(regs->pwctrl2, ILI9806_PWCTRL2_LEN, "PWCTRL2");
	r = ili9xxx_transmit(dev, ILI9806_PWCTRL2, regs->pwctrl2, ILI9806_PWCTRL2_LEN);
	if (r < 0) {
		return r;
	}

	/* Gamma Settings */
	LOG_HEXDUMP_DBG(regs->pgamctrl, ILI9806_PGAMCTRL_LEN, "PGAMCTRL");
	r = ili9xxx_transmit(dev, ILI9806_PGAMCTRL, regs->pgamctrl, ILI9806_PGAMCTRL_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->ngamctrl, ILI9806_NGAMCTRL_LEN, "NGAMCTRL");
	r = ili9xxx_transmit(dev, ILI9806_NGAMCTRL, regs->ngamctrl, ILI9806_NGAMCTRL_LEN);
	if (r < 0) {
		return r;
	}

	/* Addressing */
	LOG_HEXDUMP_DBG(regs->caddrset, ILI9806_CADDRSET_LEN, "CADDRSET");
	r = ili9xxx_transmit(dev, ILI9806_CADDRSET, regs->caddrset, ILI9806_CADDRSET_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->paddrset, ILI9806_PADDRSET_LEN, "PADDRSET");
	r = ili9xxx_transmit(dev, ILI9806_PADDRSET, regs->paddrset, ILI9806_PADDRSET_LEN);
	if (r < 0) {
		return r;
	}

	/* Pixel Interface */
	LOG_HEXDUMP_DBG(regs->pixformat, ILI9806_PIXFORMAT_LEN, "PIXFORMAT");
	r = ili9xxx_transmit(dev, ILI9806_PIXFORMAT, regs->pixformat, ILI9806_PIXFORMAT_LEN);
	if (r < 0) {
		return r;
	}

	/* Memory Access */
	LOG_HEXDUMP_DBG(regs->memaccessctrl, ILI9806_MEMACECTRL_LEN, "MEMACCESSCTRL");
	r = ili9xxx_transmit(dev, ILI9806_MEMACECTRL, regs->memaccessctrl, ILI9806_MEMACECTRL_LEN);
	if (r < 0) {
		return r;
	}

	/* Display Control */
#ifdef MIPI_DBI_AMEBA_LCDC_TODO
	/* CMN by ili9xxx.c */
	LOG_HEXDUMP_DBG(regs->slpout, ILI9806_SLPOUT_LEN, "SLPOUT");
	r = ili9xxx_transmit(dev, ILI9806_SLPOUT, regs->slpout, ILI9806_SLPOUT_LEN);
	if (r < 0) {
		return r;
	}

	/* may called by application */
	LOG_HEXDUMP_DBG(regs->displaypon, ILI9806_DISPLAYPON_LEN, "DISPLAYPON");
	r = ili9xxx_transmit(dev, ILI9806_DISPLAYPON, regs->displaypon, ILI9806_DISPLAYPON_LEN);
	if (r < 0) {
		return r;
	}

	/* Memory Write */
	/* may called by application */
	LOG_HEXDUMP_DBG(regs->memwrite, ILI9806_MEMWRITE_LEN, "MEMWRITE");
	r = ili9xxx_transmit(dev, ILI9806_MEMWRITE, regs->memwrite, ILI9806_MEMWRITE_LEN);
	if (r < 0) {
		return r;
	}
#endif

	return 0;
}

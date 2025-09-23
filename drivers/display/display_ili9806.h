/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9806_H_
#define ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9806_H_

#include <zephyr/device.h>

/* Commands/registers. */
#define ILI9806_EXTCCMDSET  0xFF
#define ILI9806_GIP1        0xBC
#define ILI9806_GIP2        0xBD
#define ILI9806_GIP3        0xBE
#define ILI9806_VCOMCONTROL 0xC7
#define ILI9806_ENVOLREG    0xED
#define ILI9806_PWCTRL1     0xC0
#define ILI9806_AVDD        0xFC
#define ILI9806_ENGINEERSET 0xDF
#define ILI9806_DVDD        0xF3
#define ILI9806_INVESION    0xB4
#define ILI9806_RESCTRL     0xF7
#define ILI9806_FRMRTCTRL   0xB1
#define ILI9806_CREQPCSDT   0xF2
#define ILI9806_PWCTRL2     0xC1
#define ILI9806_PGAMCTRL    0xE0
#define ILI9806_NGAMCTRL    0xE1
/* CMN */
#define ILI9806_CADDRSET    0x2A
#define ILI9806_PADDRSET    0x2B

/* recover */
#define ILI9806_PIXFORMAT  0x3A
#define ILI9806_MEMACECTRL 0x36

/* CMN */
#define ILI9806_SLPOUT     0x11 /* DELAY 120ms */
#define ILI9806_DISPLAYPON 0x29 /* DELAY 20ms */
#define ILI9806_MEMWRITE   0x2C

/* command length */
#define ILI9806_EXTCCMDSET_LEN  3U
#define ILI9806_GIP1_LEN        21U
#define ILI9806_GIP2_LEN        8U
#define ILI9806_GIP3_LEN        9U
#define ILI9806_VCOMCONTROL_LEN 1U
#define ILI9806_ENVOLREG_LEN    2U
#define ILI9806_PWCTRL1_LEN     3U
#define ILI9806_AVDD_LEN        1U
#define ILI9806_ENGINEERSET_LEN 6U
#define ILI9806_DVDD_LEN        1U
#define ILI9806_INVESION_LEN    3U
#define ILI9806_RESCTRL_LEN     1U
#define ILI9806_FRMRTCTRL_LEN   3U
#define ILI9806_CREQPCSDT_LEN   4U
#define ILI9806_PWCTRL2_LEN     4U
#define ILI9806_PGAMCTRL_LEN    16U
#define ILI9806_NGAMCTRL_LEN    16U
/* CMN */
#define ILI9806_CADDRSET_LEN    4U
#define ILI9806_PADDRSET_LEN    4U

/* recover */
#define ILI9806_PIXFORMAT_LEN  1U
#define ILI9806_MEMACECTRL_LEN 1U

/* CMN */
#define ILI9806_SLPOUT_LEN     0U
#define ILI9806_DISPLAYPON_LEN 0U
#define ILI9806_MEMWRITE_LEN   0U

/* X resolution (pixels). */
#define ILI9806_X_RES 480U
/** Y resolution (pixels). */
#define ILI9806_Y_RES 800U

/* ILI9806 registers to be initialized. */
struct ili9806_regs {
	uint8_t extccmdset[ILI9806_EXTCCMDSET_LEN];
	uint8_t gip1[ILI9806_GIP1_LEN];
	uint8_t gip2[ILI9806_GIP2_LEN];
	uint8_t gip3[ILI9806_GIP3_LEN];
	uint8_t vcomcontrol[ILI9806_VCOMCONTROL_LEN];
	uint8_t envolreg[ILI9806_ENVOLREG_LEN];
	uint8_t pwctrl1[ILI9806_PWCTRL1_LEN];
	uint8_t avdd[ILI9806_AVDD_LEN];
	uint8_t engineerset[ILI9806_ENGINEERSET_LEN];
	uint8_t dvdd[ILI9806_DVDD_LEN];
	uint8_t invesiontype[ILI9806_INVESION_LEN];
	uint8_t resolutionctrl[ILI9806_RESCTRL_LEN];
	uint8_t frameratectrl[ILI9806_FRMRTCTRL_LEN];
	uint8_t creqpcsdt[ILI9806_CREQPCSDT_LEN];
	uint8_t pwctrl2[ILI9806_PWCTRL2_LEN];
	uint8_t pgamctrl[ILI9806_PGAMCTRL_LEN];
	uint8_t ngamctrl[ILI9806_NGAMCTRL_LEN];
	uint8_t caddrset[ILI9806_CADDRSET_LEN]; /* CMN */
	uint8_t paddrset[ILI9806_PADDRSET_LEN]; /* CMN */
	uint8_t pixformat[ILI9806_PIXFORMAT_LEN];
	uint8_t memaccessctrl[ILI9806_MEMACECTRL_LEN];
#ifdef MIPI_DBI_AMEBA_LCDC_TODO
	uint8_t slpout[ILI9806_SLPOUT_LEN];         /* CMN */
	uint8_t displaypon[ILI9806_DISPLAYPON_LEN]; /* CMN */
	uint8_t memwrite[ILI9806_MEMWRITE_LEN];     /* CMN */

#endif
};

/* Initializer macro for ILI9806 registers. */
#define ILI9806_REGS_INIT(n)                                                                       \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), extccmdset) ==                        \
			     ILI9806_EXTCCMDSET_LEN,                                               \
		     "ili9806: Error length extended command set (EXTCCMDSET) register");          \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), gip1) == ILI9806_GIP1_LEN,            \
		     "ili9806: Error length GIP1 parameters");                                     \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), gip2) == ILI9806_GIP2_LEN,            \
		     "ili9806: Error length GIP2 parameters");                                     \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), gip3) == ILI9806_GIP3_LEN,            \
		     "ili9806: Error length GIP3 parameters");                                     \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), vcomcontrol) ==                       \
			     ILI9806_VCOMCONTROL_LEN,                                              \
		     "ili9806: Error length VCOM control (VCOMCONTROL) register");                 \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), envolreg) == ILI9806_ENVOLREG_LEN,    \
		     "ili9806: Error length enable voltage regulator (ENVOLREG) register");        \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), pwctrl1) == ILI9806_PWCTRL1_LEN,      \
		     "ili9806: Error length power control 1 (PWCTRL1) register");                  \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), avdd) == ILI9806_AVDD_LEN,            \
		     "ili9806: Error length AVDD setting (AVDD) register");                        \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), engineerset) ==                       \
			     ILI9806_ENGINEERSET_LEN,                                              \
		     "ili9806: Error length engineer mode set (ENGINEERSET) register");            \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), dvdd) == ILI9806_DVDD_LEN,            \
		     "ili9806: Error length DVDD setting (DVDD) register");                        \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), invesiontype) ==                      \
			     ILI9806_INVESION_LEN,                                                 \
		     "ili9806: Error length inversion type (INVESIONTYPE) register");              \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), resolutionctrl) ==                    \
			     ILI9806_RESCTRL_LEN,                                                  \
		     "ili9806: Error length resolution control (RESOLUTIONCTRL) register");        \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), frameratectrl) ==                     \
			     ILI9806_FRMRTCTRL_LEN,                                                \
		     "ili9806: Error length frame rate control (FRAMERATECTRL) register");         \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), creqpcsdt) == ILI9806_CREQPCSDT_LEN,  \
		     "ili9806: Error length C-REQ/PC-SDT timing (CREQPCSDT) register");            \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), pwctrl2) == ILI9806_PWCTRL2_LEN,      \
		     "ili9806: Error length power control 2 (PWCTRL2) register");                  \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), pgamctrl) == ILI9806_PGAMCTRL_LEN,    \
		     "ili9806: Error length positive gamma (PGAMCTRL) register");                  \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), ngamctrl) == ILI9806_NGAMCTRL_LEN,    \
		     "ili9806: Error length negative gamma (NGAMCTRL) register");                  \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), caddrset) == ILI9806_CADDRSET_LEN,    \
		     "ili9806: Error length column address set (CADDRSET) register");              \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), paddrset) == ILI9806_PADDRSET_LEN,    \
		     "ili9806: Error length page address set (PADDRSET) register");                \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), pixformat) == ILI9806_PIXFORMAT_LEN,  \
		     "ili9806: Error length pixel format (PIXFORMAT) register");                   \
	BUILD_ASSERT(DT_PROP_LEN(DT_INST(n, ilitek_ili9806), memaccessctrl) ==                     \
			     ILI9806_MEMACECTRL_LEN,                                               \
		     "ili9806: Error length memory access control (MEMACCESSCTRL) register");      \
	static const struct ili9806_regs ili9806_regs_##n = {                                      \
		.extccmdset = DT_PROP(DT_INST(n, ilitek_ili9806), extccmdset),                     \
		.gip1 = DT_PROP(DT_INST(n, ilitek_ili9806), gip1),                                 \
		.gip2 = DT_PROP(DT_INST(n, ilitek_ili9806), gip2),                                 \
		.gip3 = DT_PROP(DT_INST(n, ilitek_ili9806), gip3),                                 \
		.vcomcontrol = DT_PROP(DT_INST(n, ilitek_ili9806), vcomcontrol),                   \
		.envolreg = DT_PROP(DT_INST(n, ilitek_ili9806), envolreg),                         \
		.pwctrl1 = DT_PROP(DT_INST(n, ilitek_ili9806), pwctrl1),                           \
		.avdd = DT_PROP(DT_INST(n, ilitek_ili9806), avdd),                                 \
		.engineerset = DT_PROP(DT_INST(n, ilitek_ili9806), engineerset),                   \
		.dvdd = DT_PROP(DT_INST(n, ilitek_ili9806), dvdd),                                 \
		.invesiontype = DT_PROP(DT_INST(n, ilitek_ili9806), invesiontype),                 \
		.resolutionctrl = DT_PROP(DT_INST(n, ilitek_ili9806), resolutionctrl),             \
		.frameratectrl = DT_PROP(DT_INST(n, ilitek_ili9806), frameratectrl),               \
		.creqpcsdt = DT_PROP(DT_INST(n, ilitek_ili9806), creqpcsdt),                       \
		.pwctrl2 = DT_PROP(DT_INST(n, ilitek_ili9806), pwctrl2),                           \
		.pgamctrl = DT_PROP(DT_INST(n, ilitek_ili9806), pgamctrl),                         \
		.ngamctrl = DT_PROP(DT_INST(n, ilitek_ili9806), ngamctrl),                         \
		.caddrset = DT_PROP(DT_INST(n, ilitek_ili9806), caddrset),                         \
		.paddrset = DT_PROP(DT_INST(n, ilitek_ili9806), paddrset),                         \
		.pixformat = DT_PROP(DT_INST(n, ilitek_ili9806), pixformat),                       \
		.memaccessctrl = DT_PROP(DT_INST(n, ilitek_ili9806), memaccessctrl),               \
	};

/**
 * @brief Initialize ILI9806 registers with DT values.
 *
 * @param dev ILI9806 device instance
 * @return 0 on success, errno otherwise.
 */
int ili9806_regs_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9806_H_ */

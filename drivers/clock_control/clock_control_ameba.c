/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba Clock Control
 */

#include <ameba_soc.h>
#include <zephyr/drivers/clock_control/ameba_clock_control.h>
#include <zephyr/drivers/clock_control.h>

#define DT_DRV_COMPAT                          realtek_ameba_rcc

#define AMEBA_RCC_PARENT_GATE_INVALID          0xFFFFFFFF
#define AMEBA_RCC_DIV_REG_INVALID              0xFFFFFFFF
#define AMEBA_RCC_MUX_API_NULL                 NULL

#define AMEBA_RCC_GET_CLK_GRP_IDX(reg)         ((reg >> 30) & 0x03)
#define AMEBA_RCC_GET_CLK_IDX(reg)             (reg & (~(BIT(31) | BIT(30))))

typedef void(*clk_src_func)(uint32_t src, void *param);

/* Ameba clock control mux structure */
struct ameba_mux_params {
	uint32_t reg;     /* mux reg */
	uint32_t mask;    /* mux reg */
	uint32_t shift;   /* mux reg */
};

/* Ameba clock control div structure */
struct ameba_div_params {
	uint32_t reg;   /* div reg */
	uint32_t mask;  /* div mask */
	uint32_t shift; /* div bit offset */
};

static void ameba_rcc_cksl_func(u32 src, void *params);
static void ameba_rcc_lsys_cksl_uart0(u32 src, void *params);
static void ameba_rcc_lsys_cksl_uart1(u32 src, void *params);
static void ameba_rcc_lsys_cksl_uart2(u32 src, void *params);
static void ameba_rcc_soc_cksl_spic(u32 src, void *params);
static void ameba_rcc_soc_cksl_psram(u32 src, void *params);
static void ameba_rcc_soc_cksl_sport0(u32 src, void *params);
static void ameba_rcc_soc_cksl_sport1(u32 src, void *params);
static void ameba_rcc_lsys_cksl_ctc(u32 src, void *params);
static void ameba_rcc_lsys_cksl_gpio(u32 src, void *params);
static void ameba_rcc_lsys_cksl_adc(u32 src, void *params);
static void ameba_rcc_lsys_cksl_loguart(u32 src, void *params);

/* Ameba clock control structure */
struct rtk_clk {
	/* clk config*/
	uint32_t cke;   /* clock enable */
	uint32_t fen;   /* function enable */
	/* parents gating */
	uint32_t parent;

	/* mux callback api */
	clk_src_func mux_api;
	struct ameba_mux_params mux;

	/* div uart0 for example  */
	struct ameba_div_params div;
};

/* clock index,enable,fen,parent,muxapi,mux(reg,mask,shift),div(reg,mask,shift) */
#define RTK_CLK_BASIC(_struct_name, _cke_bit, _fen_bit)            \
	static struct rtk_clk _struct_name = {                         \
		.cke            = _cke_bit,                                \
		.fen            = _fen_bit,                                \
	}

#define RTK_CLK_MUX(_struct_name, _cke_bit, _fen_bit,_parent_idx, _mux_cb,        \
					_mux_reg, _mux_mask, _mux_shift)               \
	static struct rtk_clk _struct_name = {                         \
		.cke            = _cke_bit,                                \
		.fen            = _fen_bit,                                \
		.parent         = _parent_idx, 	                           \
		.mux_api        = _mux_cb,                                 \
		.mux = {                                                   \
			.reg        = _mux_reg,                                \
			.mask        = _mux_mask,                              \
			.shift        = _mux_shift,                            \
		},                                                         \
	}

#define RTK_CLK_DIV(_struct_name, _cke_bit, _fen_bit,_div_reg, _div_mask, _div_shift)    \
	static struct rtk_clk _struct_name = {                         \
		.cke            = _cke_bit,                                \
		.fen            = _fen_bit,                                \
		.parent         = AMEBA_RCC_PARENT_GATE_INVALID,           \
		.div = {                                                   \
			.reg        = _div_reg,                                \
			.mask        = _div_mask,                              \
			.shift        = _div_shift,                            \
		},                                                         \
	}

#define RTK_CLK_DEFINE(_struct_name, _cke_bit, _fen_bit, _parent_idx, _mux_cb,           \
					_mux_reg, _mux_mask, _mux_shift, _div_reg, _div_mask, _div_shift)    \
	static struct rtk_clk _struct_name = {                         \
		.cke            = _cke_bit,                                \
		.fen            = _fen_bit,                                \
		.parent         = _parent_idx, 	                           \
		.mux_api        = _mux_cb,                                 \
		.mux = {                                                   \
			.reg        = _mux_reg,                                \
			.mask        = _mux_mask,                              \
			.shift        = _mux_shift,                            \
		},                                                         \
		.div = {                                                   \
			.reg        = _div_reg,                                \
			.mask        = _div_mask,                              \
			.shift        = _div_shift,                            \
		},                                                         \
	}

/* AON Domain Clocks */
/* Clock enable by default
RTK_CLK_BASIC(aon_aon_clk,     APBPeriph_AON_CLOCK,      APBPeriph_NULL);
RTK_CLK_BASIC(aon_wodft_clk,   APBPeriph_CLOCK_NULL,     APBPeriph_NULL);
RTK_CLK_BASIC(aon_aux131k_clk, APBPeriph_CLOCK_NULL,     APBPeriph_NULL);*/
RTK_CLK_BASIC(aon_atim_clk,    APBPeriph_ATIM_CLOCK,     APBPeriph_ATIM);
RTK_CLK_BASIC(aon_sdm_clk,     APBPeriph_SDM_CLOCK,      APBPeriph_SDM);
RTK_CLK_BASIC(aon_rtc_clk,     APBPeriph_RTC_CLOCK,      APBPeriph_RTC);
RTK_CLK_BASIC(aon_iwdg_clk,    APBPeriph_CLOCK_NULL,     APBPeriph_IWDG);
RTK_CLK_MUX(aon_otpc_clk,      APBPeriph_OTPC_CLOCK,     APBPeriph_OTPC,
			AMEBA_RCC_PARENT_GATE_INVALID,
			ameba_rcc_cksl_func, REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_OTPC, 14);

/* SYSON Domain Clock */
/* Clock enable by default
RTK_CLK_BASIC(lsys_sdm_xclk,      APBPeriph_CLOCK_NULL,         APBPeriph_NULL);
RTK_CLK_BASIC(lsys_aux40m_clk,    APBPeriph_CLOCK_NULL,         APBPeriph_NULL);*/
RTK_CLK_BASIC(lsys_pwm0_clk,      APBPeriph_PWM0_CLOCK,         APBPeriph_PWM0);
RTK_CLK_BASIC(lsys_pwm1_clk,      APBPeriph_PWM1_CLOCK,         APBPeriph_PWM1);
RTK_CLK_DIV(lsys_htim0_clk,       APBPeriph_HTIM0_CLOCK,        APBPeriph_HTIM0,
			REG_LSYS_CKD_GRP1,          LSYS_MASK_CKD_XTAL_HTIM,         0);
RTK_CLK_DIV(lsys_htim1_clk,       APBPeriph_HTIM1_CLOCK,        APBPeriph_HTIM1,
			REG_LSYS_CKD_GRP1,          LSYS_MASK_CKD_XTAL_HTIM,         0);
RTK_CLK_BASIC(lsys_ledc_clk,      APBPeriph_LEDC_CLOCK,         APBPeriph_LEDC);
/* uart tclk gate wille be enable while uart rclk gate enable
RTK_CLK_BASIC(lsys_uart0_tclk,  APBPeriph_CLOCK_NULL,         APBPeriph_NULL);
RTK_CLK_BASIC(lsys_uart1_tclk,    APBPeriph_CLOCK_NULL,         APBPeriph_NULL);
RTK_CLK_BASIC(lsys_uart2_tclk,    APBPeriph_CLOCK_NULL,         APBPeriph_NULL);*/
RTK_CLK_DEFINE(lsys_uart0_rclk,     APBPeriph_UART0_CLOCK,    APBPeriph_UART0,
			   AMEBA_RCC_PARENT_GATE_INVALID,
			   ameba_rcc_lsys_cksl_uart0, 0, 0, 0,
			   REG_LSYS_CKD_GRP1,          LSYS_MASK_CKD_UART0,         20);
RTK_CLK_DEFINE(lsys_uart1_rclk,     APBPeriph_UART1_CLOCK,    APBPeriph_UART1,
			   AMEBA_RCC_PARENT_GATE_INVALID,
			   ameba_rcc_lsys_cksl_uart1, 0, 0, 0,
			   REG_LSYS_CKD_GRP1,          LSYS_MASK_CKD_UART1,         24);
RTK_CLK_DEFINE(lsys_uart2_rclk,     APBPeriph_UART2_CLOCK,    APBPeriph_UART2,
			   AMEBA_RCC_PARENT_GATE_INVALID,
			   ameba_rcc_lsys_cksl_uart2, 0, 0, 0,
			   REG_LSYS_CKD_GRP1,          LSYS_MASK_CKD_UART2,         28);
RTK_CLK_DEFINE(lsys_loguart_clk,    APBPeriph_LOGUART_CLOCK,  APBPeriph_LOGUART,
			   AMEBA_RCC_PARENT_GATE_INVALID,
			   ameba_rcc_lsys_cksl_loguart, 0, 0, 0,
			   REG_LSYS_CKD_GRP1,          LSYS_MASK_CKD_LOGUART,       16);
RTK_CLK_DEFINE(lsys_dtim_clk,       APBPeriph_DTIM_CLOCK,     APBPeriph_DTIM,
			   AMEBA_RCC_PARENT_GATE_INVALID,
			   ameba_rcc_lsys_cksl_adc,    REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_DTIM, 11,
			   REG_LSYS_CKD_GRP1,           0x3F,               6);
RTK_CLK_DEFINE(lsys_adc_clk,        APBPeriph_ADC_CLOCK,      APBPeriph_ADC,
			   AMEBA_RCC_PARENT_GATE_INVALID,
			   ameba_rcc_lsys_cksl_adc,    REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_ADC, 9,
			   REG_LSYS_CKD_GRP1,           0x0F,               12);
RTK_CLK_MUX(lsys_gpio_clk,          APBPeriph_GPIO_CLOCK,     APBPeriph_GPIO,
			AMEBA_RCC_PARENT_GATE_INVALID,
			ameba_rcc_lsys_cksl_gpio, REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_GPIO, 8);

RTK_CLK_BASIC(lsys_ltim0_clk,       APBPeriph_LTIM0_CLOCK,      APBPeriph_LTIM0);
RTK_CLK_BASIC(lsys_ltim1_clk,       APBPeriph_LTIM1_CLOCK,      APBPeriph_LTIM1);
RTK_CLK_BASIC(lsys_ltim2_clk,       APBPeriph_LTIM2_CLOCK,      APBPeriph_LTIM2);
RTK_CLK_BASIC(lsys_ltim3_clk,       APBPeriph_LTIM3_CLOCK,      APBPeriph_LTIM3);
RTK_CLK_BASIC(lsys_ltim4_clk,       APBPeriph_LTIM4_CLOCK,      APBPeriph_LTIM4);
RTK_CLK_BASIC(lsys_ltim5_clk,       APBPeriph_LTIM5_CLOCK,      APBPeriph_LTIM5);
RTK_CLK_BASIC(lsys_ltim6_clk,       APBPeriph_LTIM6_CLOCK,      APBPeriph_LTIM6);
RTK_CLK_BASIC(lsys_ltim7_clk,       APBPeriph_LTIM7_CLOCK,      APBPeriph_LTIM7);

RTK_CLK_MUX(lsys_ptim0_clk,         APBPeriph_PTIM0_CLOCK,      APBPeriph_PTIM0,
			AMEBA_RCC_PARENT_GATE_INVALID,
			ameba_rcc_cksl_func, REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_PTIM, 7);
RTK_CLK_MUX(lsys_ptim1_clk,         APBPeriph_PTIM1_CLOCK,      APBPeriph_PTIM1,
			AMEBA_RCC_PARENT_GATE_INVALID,
			ameba_rcc_cksl_func, REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_PTIM, 7);
/* RTK_CLK_BASIC(lsys_aux32k_clk,   APBPeriph_CLOCK_NULL,       APBPeriph_NULL); */
RTK_CLK_BASIC(lsys_wlon_clk,        APBPeriph_WLON_CLOCK,       APBPeriph_WLON);

RTK_CLK_MUX(lsys_lpon_clk,          APBPeriph_LPON_CLOCK,       APBPeriph_NULL,
			AMEBA_RCC_PARENT_GATE_INVALID,
			AMEBA_RCC_MUX_API_NULL,    AMEBA_RCC_DIV_REG_INVALID,           0,               0);
/*
RTK_CLK_MUX(lsys_rtc_bclk,         APBPeriph_CLOCK_NULL,        APBPeriph_NULL,    AMEBA_LPON_CLK,
		AMEBA_RCC_MUX_API_NULL,    AMEBA_RCC_DIV_REG_INVALID,           0,               0);
RTK_CLK_MUX(lsys_sdm_bclk,         APBPeriph_CLOCK_NULL,        APBPeriph_NULL,    AMEBA_LPON_CLK,
		AMEBA_RCC_MUX_API_NULL,    AMEBA_RCC_DIV_REG_INVALID,           0,               0);
RTK_CLK_MUX(lsys_wdg_bclk,         APBPeriph_CLOCK_NULL,        APBPeriph_NULL,    AMEBA_LPON_CLK,
		AMEBA_RCC_MUX_API_NULL,    AMEBA_RCC_DIV_REG_INVALID,           0,               0);*/
RTK_CLK_MUX(lsys_aipc_clk,         APBPeriph_AIPC_CLOCK,        APBPeriph_NULL,
			AMEBA_RCC_PARENT_GATE_INVALID,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);
RTK_CLK_MUX(lsys_kscan_clk,        APBPeriph_KSCAN_CLOCK,       APBPeriph_KSCAN,
			AMEBA_RCC_PARENT_GATE_INVALID,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);
RTK_CLK_MUX(lsys_sic_clk,          APBPeriph_SIC_CLOCK,         APBPeriph_SIC,
			AMEBA_RCC_PARENT_GATE_INVALID,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);

/* SOC Domain Clock */
RTK_CLK_DEFINE(soc_hp_clk,         APBPeriph_HP_CLOCK,          APBPeriph_NULL,
			   AMEBA_RCC_PARENT_GATE_INVALID,
			   ameba_rcc_cksl_func,    REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_HP, 2,
			   REG_LSYS_CKD_GRP0,           0x07,               0);
RTK_CLK_DIV(soc_sram_clk,          APBPeriph_SRAM_CLOCK,        APBPeriph_NULL,
			REG_LSYS_CKD_GRP0,          LSYS_MASK_CKD_SRAM,         16);
RTK_CLK_DEFINE(soc_peri_hclk,      APBPeriph_HPERI_CLOCK,       APBPeriph_NULL,
			   AMEBA_RCC_PARENT_GATE_INVALID,
			   ameba_rcc_cksl_func,    REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_HPERI, 3,
			   REG_LSYS_CKD_GRP0,           0x07,               8);

RTK_CLK_MUX(soc_dmac_clk,        APBPeriph_DMAC_CLOCK,         APBPeriph_DMAC,    AMEBA_PERI_HCLK,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);
RTK_CLK_MUX(soc_lx_clk,          APBPeriph_LX_CLOCK,           APBPeriph_LX,      AMEBA_PERI_HCLK,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);
RTK_CLK_MUX(soc_sdio_clk,        APBPeriph_SDIO_CLOCK,         APBPeriph_SDIO,    AMEBA_PERI_HCLK,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);
RTK_CLK_MUX(soc_spi0_bclk,       APBPeriph_SPI0_CLOCK,         APBPeriph_SPI0,    AMEBA_PERI_HCLK,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);
RTK_CLK_MUX(soc_spi1_bclk,       APBPeriph_SPI1_CLOCK,         APBPeriph_SPI1,    AMEBA_PERI_HCLK,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);
RTK_CLK_MUX(soc_wmac_bclk,       APBPeriph_WMAC_CLOCK,         APBPeriph_NULL,    AMEBA_PERI_HCLK,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);
RTK_CLK_MUX(soc_sport0_bclk,     APBPeriph_CLOCK_NULL,         APBPeriph_NULL,
			AMEBA_RCC_PARENT_GATE_INVALID,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);
RTK_CLK_MUX(soc_sport1_bclk,     APBPeriph_CLOCK_NULL,         APBPeriph_NULL,
			AMEBA_RCC_PARENT_GATE_INVALID,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);
RTK_CLK_MUX(soc_usb_bclk,        APBPeriph_USB_CLOCK,          APBPeriph_USB,     AMEBA_PERI_HCLK,
			AMEBA_RCC_MUX_API_NULL, 0, 0, 0);

RTK_CLK_MUX(soc_spic_clk,        APBPeriph_FLASH_CLOCK,        APBPeriph_FLASH,
			AMEBA_RCC_PARENT_GATE_INVALID,
			ameba_rcc_soc_cksl_spic, REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_SPIC, 4);
/* nogated clk not control by SW, spic_ps clk used to calibrate
RTK_CLK_MUX(soc_spic_ng_clk,     APBPeriph_FLASH_CLOCK,        APBPeriph_NULL,
		AMEBA_RCC_PARENT_GATE_INVALID,
		ameba_rcc_soc_cksl_spic, REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_SPIC, 4);
RTK_CLK_BASIC(soc_spic_ps_clk, APBPeriph_CLOCK_NULL,         APBPeriph_NULL,
		AMEBA_RCC_PARENT_GATE_INVALID);*/
RTK_CLK_MUX(soc_psram_clk,       APBPeriph_PSRAM_CLOCK,        APBPeriph_PSRAM,
			AMEBA_RCC_PARENT_GATE_INVALID,
			ameba_rcc_soc_cksl_psram,    REG_LSYS_CKSL_GRP0,           LSYS_BIT_CKSL_PSRAM,               6);
/*nogated clk not control by SW, spram_ps clk used to calibrate
RTK_CLK_MUX(soc_psram_ng_clk,    APBPeriph_PSRAM_CLOCK,        APBPeriph_NULL,
		AMEBA_RCC_PARENT_GATE_INVALID,
		ameba_rcc_soc_cksl_psram,    REG_LSYS_CKSL_GRP0,           LSYS_BIT_CKSL_PSRAM,               6);*/
/*RTK_CLK_BASIC(soc_spram_ps_clk,        APBPeriph_CLOCK_NULL,         APBPeriph_NULL);*/

RTK_CLK_MUX(soc_sport0_clk,       APBPeriph_SPORT0_CLOCK,       APBPeriph_SPORT0,
			AMEBA_RCC_PARENT_GATE_INVALID,
			ameba_rcc_soc_cksl_sport0,    REG_LSYS_CKSL_GRP0, LSYS_MASK_CKSL_SPORT, 18);
RTK_CLK_MUX(soc_sport1_clk,       APBPeriph_SPORT1_CLOCK,       APBPeriph_SPORT1,
			AMEBA_RCC_PARENT_GATE_INVALID,
			ameba_rcc_soc_cksl_sport1,    REG_LSYS_CKSL_GRP0, LSYS_MASK_CKSL_SPORT, 18);
RTK_CLK_BASIC(soc_ac_clk,         APBPeriph_AC_CLOCK,         APBPeriph_AC);
RTK_CLK_DIV(soc_qspi_clk,         APBPeriph_QSPI_CLOCK,       APBPeriph_QSPI,
			REG_LSYS_CKD_GRP0,          LSYS_MASK_CKD_QSPI,         28);
RTK_CLK_DIV(soc_utmifs_clk,       APBPeriph_CLOCK_NULL,       APBPeriph_NULL,
			REG_LSYS_CKD_GRP0,          LSYS_MASK_CKD_UTMIFS,         20);
RTK_CLK_DEFINE(soc_irda_clk,      APBPeriph_IRDA_CLOCK,       APBPeriph_IRDA,
			   AMEBA_RCC_PARENT_GATE_INVALID,
			   ameba_rcc_cksl_func,    REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_IRDA, 13,
			   REG_LSYS_CKD_GRP0,           LSYS_MASK_CKD_IRDA,               12);
RTK_CLK_DEFINE(soc_lp_clk,        APBPeriph_LP_CLOCK,         APBPeriph_NULL,
			   AMEBA_RCC_PARENT_GATE_INVALID,
			   ameba_rcc_cksl_func,    REG_LSYS_CKSL_GRP0, LSYS_MASK_CKSL_LP, 0,
			   REG_LSYS_CKD_GRP0,           LSYS_MASK_CKD_LP,               4);

/* SW do not ctrl the nogated clk
RTK_CLK_BASIC(soc_peri_ng_clk,     APBPeriph_LPERI_CLOCK,         APBPeriph_NULL);*/
RTK_CLK_BASIC(soc_peri_lclk,       APBPeriph_LPERI_CLOCK,         APBPeriph_NULL);
RTK_CLK_BASIC(soc_dtim_bclk,       APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_ptim0_bclk,      APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_ptim1_bclk,      APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_adc_bclk,        APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_gpio_bclk,       APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_ltim_bclk,       APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_uart_bclk,       APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_loguart_bclk,    APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_i2c0_bclk,       APBPeriph_I2C0_CLOCK,          APBPeriph_I2C0);
RTK_CLK_BASIC(soc_i2c1_bclk,       APBPeriph_I2C1_CLOCK,          APBPeriph_I2C1);
RTK_CLK_BASIC(soc_ledc_bclk,       APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_ac_bclk,         APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_trng_bclk,       APBPeriph_TRNG_CLOCK,          APBPeriph_TRNG);
RTK_CLK_BASIC(soc_ipc_bclk,        APBPeriph_IPC_CLOCK,           APBPeriph_IPC);
RTK_CLK_BASIC(soc_htim0_bclk,      APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_htim1_bclk,      APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_pwm0_bclk,       APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_pwm1_bclk,       APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_kscan_bclk,      APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_irda_bclk,       APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_otpc_bclk,       APBPeriph_CLOCK_NULL,          APBPeriph_NULL);
RTK_CLK_BASIC(soc_sic_bclk,        APBPeriph_CLOCK_NULL,          APBPeriph_NULL);

/* BT Domain Clock */
/* No register defined
RTK_CLK_DIV(bt_40m_clk,        APBPeriph_CLOCK_NULL,         APBPeriph_NULL);
RTK_CLK_DIV(bt_32k_clk,        APBPeriph_CLOCK_NULL,         APBPeriph_NULL);
RTK_CLK_DIV(bt_ana_clk,        APBPeriph_CLOCK_NULL,         APBPeriph_NULL);
*/

/* misc clocks */
RTK_CLK_BASIC(bton_clk,        APBPeriph_BTON_CLOCK,         APBPeriph_BTON);
RTK_CLK_BASIC(wdg_clk,         APBPeriph_WDG_CLOCK,          APBPeriph_NULL);
RTK_CLK_MUX(ctc_clk,           APBPeriph_CTC_CLOCK,          APBPeriph_NULL,
			AMEBA_RCC_PARENT_GATE_INVALID,
			ameba_rcc_lsys_cksl_ctc, REG_LSYS_CKSL_GRP0, LSYS_BIT_CKSL_CTC, 10);
RTK_CLK_BASIC(km0_clk,         APBPeriph_KM0_CLOCK,          APBPeriph_KM0);
RTK_CLK_BASIC(km4_clk,         APBPeriph_KM4_CLOCK,          APBPeriph_KM4);
RTK_CLK_BASIC(aes_clk,         APBPeriph_AES_CLOCK,          APBPeriph_AES);
RTK_CLK_BASIC(sha_clk,         APBPeriph_SHA_CLOCK,          APBPeriph_SHA);
RTK_CLK_BASIC(bor_clk,         APBPeriph_CLOCK_NULL,         APBPeriph_BOR);

/* Clock Domain End */

/* Rcc clock array */
static const struct rtk_clk *ameba_rcc_clk_array[1 + AMEBA_CLK_MAX] = {
	/* AON Domain */
	[AMEBA_AON_CLK]        =  NULL,
	[AMEBA_AON_WODFT_CLK]  =  NULL,
	[AMEBA_IWDG_CLK]       =  &aon_iwdg_clk,
	[AMEBA_ATIM_CLK]       =  &aon_atim_clk,
	[AMEBA_AUX131K_CLK]    =  NULL,
	[AMEBA_SDM_CLK]        =  &aon_sdm_clk,
	[AMEBA_RTC_CLK]        =  &aon_rtc_clk,
	[AMEBA_OTPC_CLK]       =  &aon_otpc_clk,
	/* SYSON Domain */
	[AMEBA_SDM_XCLK]       =  NULL,
	[AMEBA_AUX40M_CLK]     =  NULL,
	[AMEBA_PWM0_CLK]       =  &lsys_pwm0_clk,
	[AMEBA_PWM1_CLK]       =  &lsys_pwm1_clk,
	[AMEBA_HTIM0_CLK]      =  &lsys_htim0_clk,
	[AMEBA_HTIM1_CLK]      =  &lsys_htim1_clk,
	[AMEBA_LEDC_CLK]       =  &lsys_ledc_clk,
	[AMEBA_UART0_TCLK]     =  NULL,
	[AMEBA_UART1_TCLK]     =  NULL,
	[AMEBA_UART2_TCLK]     =  NULL,
	[AMEBA_UART0_RCLK]     =  &lsys_uart0_rclk,
	[AMEBA_UART1_RCLK]     =  &lsys_uart1_rclk,
	[AMEBA_UART2_RCLK]     =  &lsys_uart2_rclk,
	[AMEBA_LOGUART_CLK]    =  &lsys_loguart_clk,

	[AMEBA_DTIM_CLK]       =  &lsys_dtim_clk,
	[AMEBA_ADC_CLK]        =  &lsys_adc_clk,
	[AMEBA_GPIO_CLK]       =  &lsys_gpio_clk,
	[AMEBA_LTIM0_CLK]      =  &lsys_ltim0_clk,
	[AMEBA_LTIM1_CLK]      =  &lsys_ltim1_clk,
	[AMEBA_LTIM2_CLK]      =  &lsys_ltim2_clk,
	[AMEBA_LTIM3_CLK]      =  &lsys_ltim3_clk,
	[AMEBA_LTIM4_CLK]      =  &lsys_ltim4_clk,
	[AMEBA_LTIM5_CLK]      =  &lsys_ltim5_clk,
	[AMEBA_LTIM6_CLK]      =  &lsys_ltim6_clk,
	[AMEBA_LTIM7_CLK]      =  &lsys_ltim7_clk,
	[AMEBA_PTIM0_CLK]      =  &lsys_ptim0_clk,
	[AMEBA_PTIM1_CLK]      =  &lsys_ptim1_clk,
	[AMEBA_AUX32K_CLK]     =  NULL,
	[AMEBA_WL_SCLK]        =  &lsys_wlon_clk,
	[AMEBA_LPON_CLK]       =  &lsys_lpon_clk,
	[AMEBA_RTC_BCLK]       =  NULL,
	[AMEBA_SDM_BCLK]       =  NULL,
	[AMEBA_WDG_BCLK]       =  NULL,
	[AMEBA_AIPC_CLK]       =  &lsys_aipc_clk,
	[AMEBA_KSCAN_CLK]      =  &lsys_kscan_clk,
	[AMEBA_SIC_CLK]        =  &lsys_sic_clk,

	/* SOC Domain */
	[AMEBA_HP_CLK]     =  &soc_hp_clk,
	[AMEBA_SRAM_CLK]   =  &soc_sram_clk,
	[AMEBA_PERI_HCLK]  =  &soc_peri_hclk,
	[AMEBA_DMAC_BCLK]  =  &soc_dmac_clk,
	[AMEBA_LX_BCLK]    =  &soc_lx_clk,
	[AMEBA_SDIO_BCLK]  =  &soc_sdio_clk,
	[AMEBA_SPI0_BCLK]  =  &soc_spi0_bclk,
	[AMEBA_SPI1_BCLK]  =  &soc_spi1_bclk,
	[AMEBA_WMAC_BCLK]      =  &soc_wmac_bclk,
	[AMEBA_SPORT0_BCLK]    =  &soc_sport0_bclk,
	[AMEBA_SPORT1_BCLK]    =  &soc_sport1_bclk,
	[AMEBA_USB_BCLK]       =  &soc_usb_bclk,
	[AMEBA_SPIC_CLK]       =  &soc_spic_clk,
	[AMEBA_SPIC_CLK_NOGATED]   =  NULL,
	[AMEBA_SPIC_PS_CLK]        =  NULL,
	[AMEBA_PSRAM_CLK]          =  &soc_psram_clk,
	[AMEBA_PSRAM_CLK_NOGATED]  =  NULL,
	[AMEBA_SPRAM_PS_CLK]       =  NULL,
	[AMEBA_SPORT0_CLK]         =  &soc_sport0_clk,
	[AMEBA_SPORT1_CLK]         =  &soc_sport1_clk,
	[AMEBA_AC_CLK]             =  &soc_ac_clk,
	[AMEBA_QSPI_CLK]           =  &soc_qspi_clk,
	[AMEBA_UTMIFS_CLK]         =  &soc_utmifs_clk,
	[AMEBA_IRDA_CLK]           =  &soc_irda_clk,
	[AMEBA_LP_CLK]             =  &soc_lp_clk,
	[AMEBA_PERI_LCLK_NOGATED]  =  NULL,
	[AMEBA_PERI_LCLK]      =  &soc_peri_lclk,
	[AMEBA_DTIM_BCLK]      =  &soc_dtim_bclk,
	[AMEBA_PTIM0_BCLK]     =  &soc_ptim0_bclk,
	[AMEBA_PTIM1_BCLK]     =  &soc_ptim1_bclk,
	[AMEBA_ADC_BCLK]       =  &soc_adc_bclk,
	[AMEBA_GPIO_BCLK]      =  &soc_gpio_bclk,
	[AMEBA_LTIM_BCLK]      =  &soc_ltim_bclk,
	[AMEBA_UART_BCLK]      =  &soc_uart_bclk,
	[AMEBA_LOGUART_BCLK]   =  &soc_loguart_bclk,
	[AMEBA_I2C0_BCLK]      =  &soc_i2c0_bclk,
	[AMEBA_I2C1_BCLK]      =  &soc_i2c1_bclk,
	[AMEBA_LEDC_BCLK]      =  &soc_ledc_bclk,
	[AMEBA_AC_BCLK]        =  &soc_ac_bclk,
	[AMEBA_TRNG_BCLK]      =  &soc_trng_bclk,
	[AMEBA_IPC_BCLK]       =  &soc_ipc_bclk,
	[AMEBA_HTIM0_BCLK]     =  &soc_htim0_bclk,
	[AMEBA_HTIM1_BCLK]     =  &soc_htim1_bclk,
	[AMEBA_PWM0_BCLK]      =  &soc_pwm0_bclk,
	[AMEBA_PWM1_BCLK]      =  &soc_pwm1_bclk,
	[AMEBA_KSCAN_BCLK]     =  &soc_kscan_bclk,
	[AMEBA_IRDA_BCLK]      =  &soc_irda_bclk,
	[AMEBA_OTPC_BCLK]      =  &soc_otpc_bclk,
	[AMEBA_SIC_BCLK]       =  &soc_sic_bclk,

	/* BT Domain */
	[AMEBA_CLK_BT_40M]     =  NULL,
	[AMEBA_CLK_BT_32K]     =  NULL,
	[AMEBA_CLK_BT_ANA]     =  NULL,

	[AMEBA_CLK_BTON]       =  &bton_clk,
	[AMEBA_CLK_WDG]        =  &wdg_clk,
	[AMEBA_CLK_CTC]        =  &ctc_clk,
	[AMEBA_CLK_KM0]        =  &km0_clk,
	[AMEBA_CLK_KM4]        =  &km4_clk,
	[AMEBA_CLK_AES]        =  &aes_clk,
	[AMEBA_CLK_SHA]        =  &sha_clk,
	[AMEBA_CLK_BOR]        =  &bor_clk,

	/* Clock Info End */
	[AMEBA_CLK_MAX]        =  NULL,
};


/* static local APIs */
/**
 * @brief Ameba clock control clock select APIs
 *
 *
 * @param src clock select idx
 * @param params clock mux
 *
 */
static void ameba_rcc_cksl_func(u32 src, void *_mux)
{
	u32 reg_val = 0;
	struct ameba_mux_params *mux = (struct ameba_mux_params *)_mux;

	reg_val = HAL_READ32(SYSTEM_CTRL_BASE, mux->reg);
	reg_val &= ~(mux->mask);
	reg_val |= (src << mux->shift) & mux->mask;
	HAL_WRITE32(SYSTEM_CTRL_BASE, mux->reg, reg_val);
}

static void ameba_rcc_lsys_cksl_uart0(u32 src, void *params)
{
	ARG_UNUSED(params);
	RCC_PeriphClockSource_UART_ROM(UART0_DEV, src);
}

static void ameba_rcc_lsys_cksl_uart1(u32 src, void *params)
{
	ARG_UNUSED(params);
	RCC_PeriphClockSource_UART_ROM(UART1_DEV, src);
}

static void ameba_rcc_lsys_cksl_uart2(u32 src, void *params)
{
	ARG_UNUSED(params);
	RCC_PeriphClockSource_UART_ROM(UART2_DEV, src);
}

static void ameba_rcc_lsys_cksl_loguart(u32 src, void *params)
{
	ARG_UNUSED(params);
	RCC_PeriphClockSource_LOGUART_ROM(src);
}

static void ameba_rcc_lsys_cksl_adc(u32 src, void *params)
{
	ARG_UNUSED(params);
	RCC_PeriphClockSource_ADC(src);
}

static void ameba_rcc_lsys_cksl_ctc(u32 src, void *params)
{
	ARG_UNUSED(params);
	RCC_PeriphClockSource_CTC(src);
}

static void ameba_rcc_lsys_cksl_gpio(u32 src, void *params)
{
	ARG_UNUSED(params);
	RCC_PeriphClockSource_GPIO(src);
}

static void ameba_rcc_soc_cksl_spic(u32 src, void *params)
{
	ARG_UNUSED(params);
	RCC_PeriphClockSource_SPIC(src);
}

static void ameba_rcc_soc_cksl_psram(u32 src, void *params)
{
	ARG_UNUSED(params);
	RCC_PeriphClockSource_PSRAM(src);
}

static void ameba_rcc_soc_cksl_sport0(u32 src, void *params)
{
	ARG_UNUSED(params);
	RCC_PeriphClockSource_SPORT(AUDIO_SPORT0_DEV, src);
}

static void ameba_rcc_soc_cksl_sport1(u32 src, void *params)
{
	ARG_UNUSED(params);
	RCC_PeriphClockSource_SPORT(AUDIO_SPORT1_DEV, src);
}

/* clock control driver api functions */

/**
 * @brief Enable a clock controlled by the device
 *
 * On success, the clock is enabled and ready when this function
 * returns.
 *
 * @param dev Device structure whose driver controls the clock.
 * @param sub_system clock idx
 *
 * @return 0 on success, negative errno on failure.
 */
static int ameba_clock_on(const struct device *dev,
						  clock_control_subsys_t sub_system)
{
	uint32_t *offset = (uint32_t *)(sub_system);
	struct rtk_clk *phandle = NULL;
	uint32_t clk_idx;

	ARG_UNUSED(dev);
	if (offset == NULL) {
		return -EFAULT;
	}

	if (*offset >= AMEBA_CLK_MAX) {
		return -EFAULT;
	}

	clk_idx = *offset;

	/* Find the parents,and loop to enable all gating */
	do {
		phandle = (struct rtk_clk *)ameba_rcc_clk_array[clk_idx];
		if (phandle == NULL) {
			return -EFAULT;
		}

		RCC_PeriphClockCmd(phandle->fen, phandle->cke, ENABLE);

		clk_idx = phandle->parent;
	} while (clk_idx != AMEBA_RCC_PARENT_GATE_INVALID);

	return 0;
}

/**
 * @brief Disable a clock controlled by the device
 *
 * @param dev Device structure whose driver controls the clock.
 * @param sub_system clock idx.
 *
 * @return 0 on success, negative errno on failure.
 */
static int ameba_clock_off(const struct device *dev,
						   clock_control_subsys_t sub_system)
{
	uint32_t *offset = (uint32_t *)(sub_system);
	struct rtk_clk *phandle;

	ARG_UNUSED(dev);
	if (offset == NULL) {
		return -EFAULT;
	}

	if (*offset >= AMEBA_CLK_MAX) {
		return -EFAULT;
	}

	phandle = (struct rtk_clk *)ameba_rcc_clk_array[*offset];
	if (phandle == NULL) {
		return -EFAULT;
	}

	RCC_PeriphClockCmd(phandle->fen, phandle->cke, DISABLE);

	return 0;
}

/**
 * @brief Get clock status.
 *
 * @param dev Device.
 * @param sub_system clock idx.
 *
 * @return Status, if the clock is enabled, return on(2), else off(1).
 */
static enum clock_control_status ameba_clock_get_status(const struct device *dev,
		clock_control_subsys_t sub_system)
{
	uint32_t *offset = (uint32_t *)(sub_system);
	struct rtk_clk *phandle = NULL;
	uint32_t apb_peri_clk = 0U;
	uint32_t clk_reg_idx = 0U;
	uint32_t temp_val = 0U;
	uint32_t reg = 0U;

	ARG_UNUSED(dev);
	if (offset == NULL) {
		return -EFAULT;
	}

	if (*offset >= AMEBA_CLK_MAX) {
		return -EFAULT;
	}

	phandle = (struct rtk_clk *)ameba_rcc_clk_array[*offset];

	clk_reg_idx = AMEBA_RCC_GET_CLK_GRP_IDX(phandle->cke);
	apb_peri_clk = AMEBA_RCC_GET_CLK_IDX(phandle->cke);

	switch (clk_reg_idx) {
	case 0x0:
		reg = REG_LSYS_CKE_GRP0;
		break;
	case 0x1:
		reg = REG_LSYS_CKE_GRP1;
		break;
	case 0x3:
		reg = REG_AON_CLK;
		break;
	}

	if (phandle->cke != APBPeriph_CLOCK_NULL) {
		temp_val = HAL_READ32(SYSTEM_CTRL_BASE, reg);
		if (temp_val &= apb_peri_clk) {
			return CLOCK_CONTROL_STATUS_ON;
		}
	}

	return CLOCK_CONTROL_STATUS_OFF;
}


/**
 * @brief Configure a source clock
 *
 * @param dev Device structure whose driver controls the clock
 * @param sub_system clock idx.
 * @param data providing additional input for clock configuration
 *  (1) mux: choose the clock source
 *  (2) div: configure the clock divided value
 *
 * @retval 0 On success
 * @retval -errno Other negative errno on failure.
 */
static int ameba_clock_configure(const struct device *dev,
								 clock_control_subsys_t sub_system,
								 void *data)
{
	struct ameba_clock_config *cfg = (struct ameba_clock_config *)data;
	uint32_t *offset = (uint32_t *)sub_system;
	struct ameba_div_params *pdiv = NULL;
	clk_src_func choose_clk_src = NULL;
	struct rtk_clk *phandle = NULL;
	uint32_t reg_temp = 0U;

	ARG_UNUSED(dev);
	if ((offset == NULL) || (cfg == NULL)) {
		return -EFAULT ;
	}

	if (*offset >= AMEBA_CLK_MAX) {
		return -EFAULT;
	}

	phandle = (struct rtk_clk *)ameba_rcc_clk_array[*offset];

	if (cfg->is_mux) { /* MUX */
		choose_clk_src = phandle->mux_api;
		if (AMEBA_RCC_MUX_API_NULL == choose_clk_src) {
			return -EFAULT;
		}

		choose_clk_src(cfg->param_value, &(phandle->mux));
	} else { /* DIV */
		pdiv = &(phandle->div);
		if (AMEBA_RCC_DIV_REG_INVALID == pdiv->reg) {
			return 0;
		}

		reg_temp = HAL_READ32(SYSTEM_CTRL_BASE, pdiv->reg);
		reg_temp &= ~(pdiv->mask);
		reg_temp |= ((cfg->param_value) << (pdiv->shift)) & (pdiv->mask);
		HAL_WRITE32(SYSTEM_CTRL_BASE, pdiv->reg, reg_temp);
	}

	return 0;
}

/**
 * @brief Initialize Ameba RCC Driver
 *
 * @param dev Device structure whose driver controls the clock
 *
 * @return 0 on success
 */
static int ameba_clock_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static const struct clock_control_driver_api ameba_clock_driver_api = {
	.on         = ameba_clock_on,
	.off        = ameba_clock_off,
	.get_status = ameba_clock_get_status,
	/*  FIXME support later
		.get_rate   = ameba_clock_get_rate,
		.set_rate   = ameba_clock_set_rate,
		.async_on   = ameba_clock_async_on,
	*/
	.configure  = ameba_clock_configure,
};

DEVICE_DT_INST_DEFINE(0,
					  &ameba_clock_init,
					  NULL,
					  NULL,
					  NULL,
					  PRE_KERNEL_1,
					  CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
					  &ameba_clock_driver_api);

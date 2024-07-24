/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba Clock Control
 */

/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/drivers/clock_control/ameba_clock_control.h>
#include <zephyr/drivers/clock_control.h>

#define DT_DRV_COMPAT                          realtek_ameba_rcc

#define AMEBA_RCC_NO_PARENT                    0xFF
#define AMEBA_RCC_MUX_API_NULL                 NULL

/* clk div */
#define AMEBA_RCC_CKD_GRP0                     0
#define AMEBA_RCC_CKD_GRP1                     1
#define AMEBA_RCC_CKD_GRP_INVALID              2

typedef void(*clk_src_func)(uint32_t src);

static void ameba_rcc_lsys_cksl_uart0(u32 src);
static void ameba_rcc_lsys_cksl_uart1(u32 src);
static void ameba_rcc_lsys_cksl_uart2(u32 src);
static void ameba_rcc_soc_cksl_spic(u32 src);
static void ameba_rcc_soc_cksl_psram(u32 src);
static void ameba_rcc_soc_cksl_sport0(u32 src);
static void ameba_rcc_soc_cksl_sport1(u32 src);
static void ameba_rcc_lsys_cksl_ctc(u32 src);
static void ameba_rcc_lsys_cksl_loguart(u32 src);
static clk_src_func ameba_rcc_get_clk_src_api(u32 clk_idx);

static uint32_t clk_div_group[] = {
	REG_LSYS_CKD_GRP0,
	REG_LSYS_CKD_GRP1,
};

/* Ameba clock control ： cke and fen config reg struct */
struct ameba_clk_ctrl_reg {
	uint32_t cke;   /* clock enable reg */
	uint32_t fen;   /* function enable reg */
};

/* Ameba clock control : clock parents & div info */
struct ameba_clk_cfg_struct {
	uint8_t parent;

	uint8_t div_reg : 2;   /* div group idx */
	uint8_t div_shift : 5; /* div bit offset in group */
	uint8_t div_mask;      /* div mask value */
};

/* clock */
#define RTK_CLK_BASIC(_struct_name, _parent_idx)                   \
	static const struct ameba_clk_cfg_struct _struct_name = {      \
		.parent         = _parent_idx, 	                           \
		.div_reg        = AMEBA_RCC_CKD_GRP_INVALID,               \
		.div_mask       = 0,                                       \
		.div_shift      = 0,                                       \
	}

#define RTK_CLK_DIV(_struct_name, _div_reg, _div_mask, _div_shift) \
	static const struct ameba_clk_cfg_struct _struct_name = {      \
		.parent         = AMEBA_RCC_NO_PARENT,                     \
		.div_reg        = _div_reg,                                \
		.div_mask       = _div_mask,                               \
		.div_shift      = _div_shift,                              \
	}

/* AON Domain Clocks */
RTK_CLK_BASIC(aon_iwdg_clk,      AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(aon_atim_clk,      AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(aon_sdm_clk,       AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(aon_rtc_clk,       AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(aon_otpc_clk,      AMEBA_RCC_NO_PARENT);

/* SYSON Domain Clock */
RTK_CLK_BASIC(lsys_pwm0_clk,     AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_pwm1_clk,     AMEBA_RCC_NO_PARENT);
RTK_CLK_DIV(lsys_htim0_clk,      AMEBA_RCC_CKD_GRP1,          0x3F,         0);
RTK_CLK_DIV(lsys_htim1_clk,      AMEBA_RCC_CKD_GRP1,          0x3F,         0);
RTK_CLK_BASIC(lsys_ledc_clk,     AMEBA_RCC_NO_PARENT);

RTK_CLK_DIV(lsys_uart0_rclk,     AMEBA_RCC_CKD_GRP1,          0x0F,         20);
RTK_CLK_DIV(lsys_uart1_rclk,     AMEBA_RCC_CKD_GRP1,          0x0F,         24);
RTK_CLK_DIV(lsys_uart2_rclk,     AMEBA_RCC_CKD_GRP1,          0x0F,         28);
RTK_CLK_DIV(lsys_loguart_clk,    AMEBA_RCC_CKD_GRP1,          0x0F,         16);
RTK_CLK_DIV(lsys_dtim_clk,       AMEBA_RCC_CKD_GRP1,          0x3F,         6);
RTK_CLK_DIV(lsys_adc_clk,        AMEBA_RCC_CKD_GRP1,          0x0F,         12);
RTK_CLK_BASIC(lsys_gpio_clk,     AMEBA_RCC_NO_PARENT);

RTK_CLK_BASIC(lsys_ltim0_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_ltim1_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_ltim2_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_ltim3_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_ltim4_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_ltim5_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_ltim6_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_ltim7_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_ptim0_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_ptim1_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_wlon_clk,     AMEBA_RCC_NO_PARENT);

RTK_CLK_BASIC(lsys_lpon_clk,     AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(lsys_aipc_clk,     AMEBA_LPON_CLK);
RTK_CLK_BASIC(lsys_kscan_clk,    AMEBA_LPON_CLK);
RTK_CLK_BASIC(lsys_sic_clk,      AMEBA_LPON_CLK);

/* SOC Domain Clock */
RTK_CLK_DIV(soc_hp_clk,          AMEBA_RCC_CKD_GRP0,          0x07,         0);
RTK_CLK_DIV(soc_sram_clk,        AMEBA_RCC_CKD_GRP0,          0x03,         16);

RTK_CLK_DIV(soc_peri_hclk,       AMEBA_RCC_CKD_GRP0,          0x07,         8);
RTK_CLK_BASIC(soc_dmac_clk,      AMEBA_PERI_HCLK);
RTK_CLK_BASIC(soc_lx_clk,        AMEBA_PERI_HCLK);
RTK_CLK_BASIC(soc_sdio_clk,      AMEBA_PERI_HCLK);
RTK_CLK_BASIC(soc_spi0_bclk,     AMEBA_PERI_HCLK);
RTK_CLK_BASIC(soc_spi1_bclk,     AMEBA_PERI_HCLK);
RTK_CLK_BASIC(soc_wmac_bclk,     AMEBA_PERI_HCLK);
RTK_CLK_BASIC(soc_usb_bclk,      AMEBA_PERI_HCLK);

RTK_CLK_BASIC(soc_spic_clk,      AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(soc_psram_clk,     AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(soc_sport0_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(soc_sport1_clk,    AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(soc_ac_clk,        AMEBA_RCC_NO_PARENT);
RTK_CLK_DIV(soc_qspi_clk,        AMEBA_RCC_CKD_GRP0,          0x07,         28);
RTK_CLK_DIV(soc_utmifs_clk,      AMEBA_RCC_CKD_GRP0,          0x0F,         20);
RTK_CLK_DIV(soc_irda_clk,        AMEBA_RCC_CKD_GRP0,          0x07,         12);
RTK_CLK_DIV(soc_lp_clk,          AMEBA_RCC_CKD_GRP0,          0x0F,         4);

RTK_CLK_BASIC(soc_peri_lclk,     AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(soc_i2c0_bclk,     AMEBA_PERI_LCLK);
RTK_CLK_BASIC(soc_i2c1_bclk,     AMEBA_PERI_LCLK);
RTK_CLK_BASIC(soc_trng_bclk,     AMEBA_PERI_LCLK);
RTK_CLK_BASIC(soc_ipc_bclk,      AMEBA_PERI_LCLK);

/* BT Domain Clock */

/* misc clocks */
RTK_CLK_BASIC(bton_clk,          AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(wdg_clk,           AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(ctc_clk,           AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(km0_clk,           AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(km4_clk,           AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(aes_clk,           AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(sha_clk,           AMEBA_RCC_NO_PARENT);
RTK_CLK_BASIC(bor_clk,           AMEBA_RCC_NO_PARENT);

/* Clock Domain End */


/* Rcc clock cke/fen reg array */
static const struct ameba_clk_ctrl_reg ameba_clk_ctrl_reg_array[AMEBA_CLK_MAX] = {
	{/* AMEBA_IWDG_CLK,  */    APBPeriph_CLOCK_NULL,     APBPeriph_IWDG},
	{/* AMEBA_ATIM_CLK,  */    APBPeriph_ATIM_CLOCK,     APBPeriph_ATIM},
	{/* AMEBA_SDM_CLK,   */    APBPeriph_SDM_CLOCK,      APBPeriph_SDM},
	{/* AMEBA_RTC_CLK,   */    APBPeriph_RTC_CLOCK,      APBPeriph_RTC},
	{/* AMEBA_OTPC_CLK,  */    APBPeriph_OTPC_CLOCK,     APBPeriph_OTPC},

	{/* AMEBA_PWM0_CLK,  */    APBPeriph_PWM0_CLOCK,     APBPeriph_PWM0},
	{/* AMEBA_PWM1_CLK,  */    APBPeriph_PWM1_CLOCK,     APBPeriph_PWM1},
	{/* AMEBA_HTIM0_CLK, */    APBPeriph_HTIM0_CLOCK,    APBPeriph_HTIM0},
	{/* AMEBA_HTIM1_CLK, */    APBPeriph_HTIM1_CLOCK,    APBPeriph_HTIM1},
	{/* AMEBA_LEDC_CLK,  */    APBPeriph_LEDC_CLOCK,     APBPeriph_LEDC},
	{/* AMEBA_UART0_RCLK,*/    APBPeriph_UART0_CLOCK,    APBPeriph_UART0},
	{/* AMEBA_UART1_RCLK,*/    APBPeriph_UART1_CLOCK,    APBPeriph_UART1},
	{/* AMEBA_UART2_RCLK,  */  APBPeriph_UART2_CLOCK,    APBPeriph_UART2},
	{/* AMEBA_LOGUART_CLK, */  APBPeriph_LOGUART_CLOCK,  APBPeriph_LOGUART},
	{/* AMEBA_DTIM_CLK,  */    APBPeriph_DTIM_CLOCK,     APBPeriph_DTIM},
	{/* AMEBA_ADC_CLK,   */    APBPeriph_ADC_CLOCK,      APBPeriph_ADC},
	{/* AMEBA_GPIO_CLK,  */    APBPeriph_GPIO_CLOCK,     APBPeriph_GPIO},
	{/* AMEBA_LTIM0_CLK, */    APBPeriph_LTIM0_CLOCK,    APBPeriph_LTIM0},
	{/* AMEBA_LTIM1_CLK, */    APBPeriph_LTIM1_CLOCK,    APBPeriph_LTIM1},
	{/* AMEBA_LTIM2_CLK, */    APBPeriph_LTIM2_CLOCK,    APBPeriph_LTIM2},
	{/* AMEBA_LTIM3_CLK, */    APBPeriph_LTIM3_CLOCK,    APBPeriph_LTIM3},
	{/* AMEBA_LTIM4_CLK, */    APBPeriph_LTIM4_CLOCK,    APBPeriph_LTIM4},
	{/* AMEBA_LTIM5_CLK, */    APBPeriph_LTIM5_CLOCK,    APBPeriph_LTIM5},
	{/* AMEBA_LTIM6_CLK, */    APBPeriph_LTIM6_CLOCK,    APBPeriph_LTIM6},
	{/* AMEBA_LTIM7_CLK, */    APBPeriph_LTIM7_CLOCK,    APBPeriph_LTIM7},
	{/* AMEBA_PTIM0_CLK, */    APBPeriph_PTIM0_CLOCK,    APBPeriph_PTIM0},
	{/* AMEBA_PTIM1_CLK, */    APBPeriph_PTIM1_CLOCK,    APBPeriph_PTIM1},
	{/* AMEBA_WL_SCLK,   */    APBPeriph_WLON_CLOCK,     APBPeriph_WLON},
	{/* AMEBA_LPON_CLK,  */    APBPeriph_LPON_CLOCK,     APBPeriph_NULL},
	{/* AMEBA_AIPC_CLK,  */    APBPeriph_AIPC_CLOCK,     APBPeriph_NULL},
	{/* AMEBA_KSCAN_CLK, */    APBPeriph_KSCAN_CLOCK,    APBPeriph_KSCAN},
	{/* AMEBA_SIC_CLK,   */    APBPeriph_SIC_CLOCK,      APBPeriph_SIC},

	{/* AMEBA_HP_CLK,    */    APBPeriph_HP_CLOCK,       APBPeriph_NULL},
	{/* AMEBA_SRAM_CLK,  */    APBPeriph_SRAM_CLOCK,     APBPeriph_NULL},
	{/* AMEBA_PERI_HCLK, */    APBPeriph_HPERI_CLOCK,    APBPeriph_NULL},
	{/* AMEBA_DMAC_BCLK, */    APBPeriph_DMAC_CLOCK,     APBPeriph_DMAC},
	{/* AMEBA_LX_BCLK,   */    APBPeriph_LX_CLOCK,       APBPeriph_LX},
	{/* AMEBA_SDIO_BCLK, */    APBPeriph_SDIO_CLOCK,     APBPeriph_SDIO},
	{/* AMEBA_SPI0_BCLK, */    APBPeriph_SPI0_CLOCK,     APBPeriph_SPI0},
	{/* AMEBA_SPI1_BCLK, */    APBPeriph_SPI1_CLOCK,     APBPeriph_SPI1},
	{/* AMEBA_WMAC_BCLK, */    APBPeriph_WMAC_CLOCK,     APBPeriph_NULL},

	{/* AMEBA_USB_BCLK,  */    APBPeriph_USB_CLOCK,      APBPeriph_USB},
	{/* AMEBA_SPIC_CLK,  */    APBPeriph_FLASH_CLOCK,    APBPeriph_FLASH},
	{/* AMEBA_PSRAM_CLK, */    APBPeriph_PSRAM_CLOCK,    APBPeriph_PSRAM},
	{/* AMEBA_SPORT0_CLK,*/    APBPeriph_SPORT0_CLOCK,   APBPeriph_SPORT0},
	{/* AMEBA_SPORT1_CLK,*/    APBPeriph_SPORT1_CLOCK,   APBPeriph_SPORT1},
	{/* AMEBA_AC_CLK,    */    APBPeriph_AC_CLOCK,       APBPeriph_AC},
	{/* AMEBA_QSPI_CLK,  */    APBPeriph_QSPI_CLOCK,     APBPeriph_QSPI},
	{/* AMEBA_UTMIFS_CLK,*/    APBPeriph_USB_CLOCK,      APBPeriph_USB},
	{/* AMEBA_IRDA_CLK,  */    APBPeriph_IRDA_CLOCK,     APBPeriph_IRDA},
	{/* AMEBA_LP_CLK,    */    APBPeriph_LP_CLOCK,       APBPeriph_NULL},
	{/* AMEBA_PERI_LCLK, */    APBPeriph_LPERI_CLOCK,    APBPeriph_NULL},
	{/* AMEBA_I2C0_BCLK, */    APBPeriph_I2C0_CLOCK,     APBPeriph_I2C0},
	{/* AMEBA_I2C1_BCLK, */    APBPeriph_I2C1_CLOCK,     APBPeriph_I2C1},

	{/* AMEBA_TRNG_BCLK, */    APBPeriph_TRNG_CLOCK,     APBPeriph_TRNG},
	{/* AMEBA_IPC_BCLK,  */    APBPeriph_IPC_CLOCK,      APBPeriph_IPC},

	{/* AMEBA_CLK_BTON,  */    APBPeriph_BTON_CLOCK,     APBPeriph_BTON},
	{/* AMEBA_CLK_WDG,   */    APBPeriph_WDG_CLOCK,      APBPeriph_NULL},
	{/* AMEBA_CLK_CTC,   */    APBPeriph_CTC_CLOCK,      APBPeriph_NULL},
	{/* AMEBA_CLK_KM0,   */    APBPeriph_KM0_CLOCK,      APBPeriph_KM0},
	{/* AMEBA_CLK_KM4,   */    APBPeriph_KM4_CLOCK,      APBPeriph_KM4},
	{/* AMEBA_CLK_AES,   */    APBPeriph_AES_CLOCK,      APBPeriph_AES},
	{/* AMEBA_CLK_SHA,   */    APBPeriph_SHA_CLOCK,      APBPeriph_SHA},
	{/* AMEBA_CLK_BOR,   */    APBPeriph_CLOCK_NULL,     APBPeriph_BOR},
};

/* Rcc clock array */
static const struct ameba_clk_cfg_struct *ameba_rcc_clk_array[AMEBA_CLK_MAX] = {
	/* AON Domain */
	[AMEBA_IWDG_CLK]       =  &aon_iwdg_clk,
	[AMEBA_ATIM_CLK]       =  &aon_atim_clk,
	[AMEBA_SDM_CLK]        =  &aon_sdm_clk,
	[AMEBA_RTC_CLK]        =  &aon_rtc_clk,
	[AMEBA_OTPC_CLK]       =  &aon_otpc_clk,
	/* SYSON Domain */
	[AMEBA_PWM0_CLK]       =  &lsys_pwm0_clk,
	[AMEBA_PWM1_CLK]       =  &lsys_pwm1_clk,
	[AMEBA_HTIM0_CLK]      =  &lsys_htim0_clk,
	[AMEBA_HTIM1_CLK]      =  &lsys_htim1_clk,
	[AMEBA_LEDC_CLK]       =  &lsys_ledc_clk,
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
	[AMEBA_WL_SCLK]        =  &lsys_wlon_clk,
	[AMEBA_LPON_CLK]       =  &lsys_lpon_clk,
	[AMEBA_AIPC_CLK]       =  &lsys_aipc_clk,
	[AMEBA_KSCAN_CLK]      =  &lsys_kscan_clk,
	[AMEBA_SIC_CLK]        =  &lsys_sic_clk,

	/* SOC Domain */
	[AMEBA_HP_CLK]         =  &soc_hp_clk,
	[AMEBA_SRAM_CLK]       =  &soc_sram_clk,
	[AMEBA_PERI_HCLK]      =  &soc_peri_hclk,
	[AMEBA_DMAC_BCLK]      =  &soc_dmac_clk,
	[AMEBA_LX_BCLK]        =  &soc_lx_clk,
	[AMEBA_SDIO_BCLK]      =  &soc_sdio_clk,
	[AMEBA_SPI0_BCLK]      =  &soc_spi0_bclk,
	[AMEBA_SPI1_BCLK]      =  &soc_spi1_bclk,
	[AMEBA_WMAC_BCLK]      =  &soc_wmac_bclk,
	[AMEBA_USB_BCLK]       =  &soc_usb_bclk,
	[AMEBA_SPIC_CLK]       =  &soc_spic_clk,
	[AMEBA_PSRAM_CLK]      =  &soc_psram_clk,
	[AMEBA_SPORT0_CLK]     =  &soc_sport0_clk,
	[AMEBA_SPORT1_CLK]     =  &soc_sport1_clk,
	[AMEBA_AC_CLK]         =  &soc_ac_clk,
	[AMEBA_QSPI_CLK]       =  &soc_qspi_clk,
	[AMEBA_UTMIFS_CLK]     =  &soc_utmifs_clk,
	[AMEBA_IRDA_CLK]       =  &soc_irda_clk,
	[AMEBA_LP_CLK]         =  &soc_lp_clk,
	[AMEBA_PERI_LCLK]      =  &soc_peri_lclk,
	[AMEBA_I2C0_BCLK]      =  &soc_i2c0_bclk,
	[AMEBA_I2C1_BCLK]      =  &soc_i2c1_bclk,
	[AMEBA_TRNG_BCLK]      =  &soc_trng_bclk,
	[AMEBA_IPC_BCLK]       =  &soc_ipc_bclk,

	/* BT Domain */
	[AMEBA_CLK_BTON]       =  &bton_clk,
	[AMEBA_CLK_WDG]        =  &wdg_clk,
	[AMEBA_CLK_CTC]        =  &ctc_clk,
	[AMEBA_CLK_KM0]        =  &km0_clk,
	[AMEBA_CLK_KM4]        =  &km4_clk,
	[AMEBA_CLK_AES]        =  &aes_clk,
	[AMEBA_CLK_SHA]        =  &sha_clk,
	[AMEBA_CLK_BOR]        =  &bor_clk,

	/* Clock Info End */
};


/* static local APIs */
/**
 * @brief Ameba clock control clock select APIs
 *
 *
 * @param src clock select idx
 *
 */

static void ameba_rcc_lsys_cksl_uart0(u32 src)
{
	RCC_PeriphClockSource_UART_ROM(UART0_DEV, src);
}

static void ameba_rcc_lsys_cksl_uart1(u32 src)
{
	RCC_PeriphClockSource_UART_ROM(UART1_DEV, src);
}

static void ameba_rcc_lsys_cksl_uart2(u32 src)
{
	RCC_PeriphClockSource_UART_ROM(UART2_DEV, src);
}

static void ameba_rcc_lsys_cksl_loguart(u32 src)
{
	RCC_PeriphClockSource_LOGUART_ROM(src);
}

static void ameba_rcc_lsys_cksl_ctc(u32 src)
{
	RCC_PeriphClockSource_CTC(src);
}

static void ameba_rcc_soc_cksl_spic(u32 src)
{
	RCC_PeriphClockSource_SPIC(src);
}

static void ameba_rcc_soc_cksl_psram(u32 src)
{
	RCC_PeriphClockSource_PSRAM(src);
}

static void ameba_rcc_soc_cksl_sport0(u32 src)
{
	RCC_PeriphClockSource_SPORT(AUDIO_SPORT0_DEV, src);
}

static void ameba_rcc_soc_cksl_sport1(u32 src)
{
	RCC_PeriphClockSource_SPORT(AUDIO_SPORT1_DEV, src);
}

static clk_src_func ameba_rcc_get_clk_src_api(u32 clk_idx)
{
	if (clk_idx >= AMEBA_CLK_MAX) {
		return NULL;
	}

	if (AMEBA_UART0_RCLK == clk_idx) {
		return ameba_rcc_lsys_cksl_uart0;
	}

	if (AMEBA_UART1_RCLK == clk_idx) {
		return ameba_rcc_lsys_cksl_uart1;
	}

	if (AMEBA_UART2_RCLK == clk_idx) {
		return ameba_rcc_lsys_cksl_uart2;
	}

	if (AMEBA_SPIC_CLK == clk_idx) {
		return ameba_rcc_soc_cksl_spic;
	}

	if (AMEBA_PSRAM_CLK == clk_idx) {
		return ameba_rcc_soc_cksl_psram;
	}

	if (AMEBA_SPORT0_CLK == clk_idx) {
		return ameba_rcc_soc_cksl_sport0;
	}

	if (AMEBA_SPORT1_CLK == clk_idx) {
		return ameba_rcc_soc_cksl_sport1;
	}

	if (AMEBA_CLK_CTC == clk_idx) {
		return ameba_rcc_lsys_cksl_ctc;
	}

	if (AMEBA_LOGUART_CLK == clk_idx) {
		return ameba_rcc_lsys_cksl_loguart;
	}

	return NULL;
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
	const struct ameba_clk_cfg_struct *pcfg_handle;
	const struct ameba_clk_ctrl_reg *pctrl_handle;
	uint32_t clk_idx = (uint32_t)sub_system;

	ARG_UNUSED(dev);

	if (clk_idx >= AMEBA_CLK_MAX) {
		return -ENOTSUP;
	}

	/* Find the parents,and loop to enable all gating */
	do {
		pctrl_handle = &(ameba_clk_ctrl_reg_array[clk_idx]);
		pcfg_handle = ameba_rcc_clk_array[clk_idx];

		if (pctrl_handle == NULL || pcfg_handle == NULL) {
			return -EFAULT;
		}

		RCC_PeriphClockCmd(pctrl_handle->fen, pctrl_handle->cke, ENABLE);

		clk_idx = pcfg_handle->parent;

		if (clk_idx >= AMEBA_CLK_MAX) {
			break;
		}
	} while (clk_idx != AMEBA_RCC_NO_PARENT);

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
	uint32_t clk_idx = (uint32_t)sub_system;
	const struct ameba_clk_ctrl_reg *phandle;

	ARG_UNUSED(dev);

	if (clk_idx >= AMEBA_CLK_MAX) {
		return -ENOTSUP;
	}

	phandle = &(ameba_clk_ctrl_reg_array[clk_idx]);
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
	uint32_t clk_idx = (uint32_t)sub_system;
	const struct ameba_clk_ctrl_reg *phandle = NULL;

	ARG_UNUSED(dev);

	if (clk_idx >= AMEBA_CLK_MAX) {
		return CLOCK_CONTROL_STATUS_OFF;
	}

	phandle = &(ameba_clk_ctrl_reg_array[clk_idx]);
	if (phandle == NULL) {
		return CLOCK_CONTROL_STATUS_OFF;
	}

	if (RCC_PeriphClockEnableChk(phandle->cke)) {
		return CLOCK_CONTROL_STATUS_ON;
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
	uint32_t clk_idx = (uint32_t)sub_system;
	clk_src_func choose_clk_src = NULL;
	const struct ameba_clk_cfg_struct *phandle;
	struct Rcc_ClkDiv div_param;

	ARG_UNUSED(dev);

	if (clk_idx >= AMEBA_CLK_MAX) {
		return -EOVERFLOW;
	}

	phandle = ameba_rcc_clk_array[clk_idx];
	if (phandle == NULL) {
		return -ENOTSUP;
	}

	if (AMEBA_RCC_SRC_VALUE_INVALID != cfg->src) { /* MUX/SRC */
		choose_clk_src = ameba_rcc_get_clk_src_api(clk_idx);
		if (AMEBA_RCC_MUX_API_NULL == choose_clk_src) {
			return -EFAULT;
		}

		choose_clk_src(cfg->src);
	}

	if (AMEBA_RCC_DIV_VALUE_INVALID != cfg->div) {  /* DIV */
		if (phandle->div_reg >= AMEBA_RCC_CKD_GRP_INVALID) {
			return -EFAULT;
		}

		div_param.CkdGroupOfs = clk_div_group[phandle->div_reg];
		div_param.BitMask = ((u32)(phandle->div_mask)) << phandle->div_shift;
		div_param.DivShift = phandle->div_shift;
		div_param.DivVal = cfg->div;

		RCC_PeriphClockDivSet(&div_param);
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

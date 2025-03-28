/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBAD_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBAD_CLOCK_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Domain clocks
 *
 * AON Domain
 * AON_OTP Domain
 * SYSON Domain
 * SOC Domain
 * BT Domain
 */

/* AON Domain clocks */
#define AMEBA_IWDG_CLK 0 /* iwdg clk idx */
#define AMEBA_ATIM_CLK 1 /* atim clk idx */
#define AMEBA_SDM_CLK  2 /* aon clk idx */
#define AMEBA_RTC_CLK  3 /* sdm clk idx */
#define AMEBA_OTPC_CLK 4 /* optc clk idx */

/* SYSON Domain clocks */
#define AMEBA_PWM0_CLK  5 /* pwm0 clk idx */
#define AMEBA_PWM1_CLK  6 /* pwm1 clk idx */
#define AMEBA_HTIM0_CLK 7 /* htim0 clk idx */
#define AMEBA_HTIM1_CLK 8 /* htim1 clk idx */
#define AMEBA_LEDC_CLK  9 /* ledc clk idx */

#define AMEBA_UART0_RCLK  10 /* uart0 rclk idx */
#define AMEBA_UART1_RCLK  11 /* uart1 rclk idx */
#define AMEBA_LOGUART_CLK 12 /* loguart clk idx */
#define AMEBA_UART3_RCLK  13 /* uart3 rclk idx */
#define AMEBA_DTIM_CLK    14 /* dtim clk idx */
#define AMEBA_ADC_CLK     15 /* adc clk idx */
#define AMEBA_GPIO_CLK    16 /* gpio clk idx */
#define AMEBA_LTIM0_CLK   17 /* ltim0 clk idx */
#define AMEBA_LTIM1_CLK   18 /* ltim1 clk idx */
#define AMEBA_LTIM2_CLK   19 /* ltim2 clk idx */
#define AMEBA_LTIM3_CLK   20 /* ltim3 clk idx */
#define AMEBA_LTIM4_CLK   21 /* ltim4 clk idx */
#define AMEBA_LTIM5_CLK   22 /* ltim5 clk idx */
#define AMEBA_LTIM6_CLK   23 /* ltim6 clk idx */
#define AMEBA_LTIM7_CLK   24 /* ltim7 clk idx */
#define AMEBA_PTIM0_CLK   25 /* ptim0 clk idx */
#define AMEBA_PTIM1_CLK   26 /* ptim1 clk idx */

#define AMEBA_WL_SCLK   27 /* wl clk idx */
#define AMEBA_LPON_CLK  28 /* lpon clk idx */
#define AMEBA_AIPC_CLK  29 /* aipc clk idx */
#define AMEBA_KSCAN_CLK 30 /* kscan clk idx */
#define AMEBA_SIC_CLK   31 /* sic clk idx */

/* SOC Domain clocks */
#define AMEBA_HP_CLK    32 /* hp clk idx */
#define AMEBA_SRAM_CLK  33 /* sram clk idx */
#define AMEBA_PERI_HCLK 34 /* peri clk idx */
#define AMEBA_DMAC_BCLK 35 /* dmac clk idx */
#define AMEBA_LX_BCLK   36 /* lx clk idx */
#define AMEBA_SDIO_BCLK 37 /* sdio clk idx */
#define AMEBA_SPI0_BCLK 38 /* spi0 bclk idx */
#define AMEBA_SPI1_BCLK 39 /* spi1 bclk idx */
#define AMEBA_WMAC_BCLK 40 /* wmac clk idx */
#define AMEBA_USB_BCLK  41 /* usb bclk idx */

#define AMEBA_SPIC_CLK   42 /* spic clk idx */
#define AMEBA_PSRAM_CLK  43 /* psram clk idx */
#define AMEBA_SPORT0_CLK 44 /* sport0 clk idx */
#define AMEBA_SPORT1_CLK 45 /* sport1 clk idx */
#define AMEBA_AC_CLK     46 /* ac clk idx */
#define AMEBA_QSPI_CLK   47 /* qspi clk idx */
#define AMEBA_UTMIFS_CLK 48 /* utmifs clk idx */
#define AMEBA_IRDA_CLK   49 /* irda clk idx */

#define AMEBA_LP_CLK    50 /* lp clk idx */
#define AMEBA_PERI_LCLK 51 /* peri lclk idx */
#define AMEBA_I2C0_BCLK 52 /* i2c0 bclk idx */
#define AMEBA_I2C1_BCLK 53 /* i2c1 bclk idx */
#define AMEBA_TRNG_BCLK 54 /* trng bclk idx */
#define AMEBA_IPC_BCLK  55 /* ipc bclk idx */

/* BT Domain clocks */

/* misc clocks */
#define AMEBA_CLK_BTON 56 /* bton clk idx */
#define AMEBA_CLK_WDG  57 /* wdg clk idx */
#define AMEBA_CLK_CTC  58 /* ctc clk idx */
#define AMEBA_CLK_KM0  59 /* km0 clk idx */
#define AMEBA_CLK_KM4  60 /* km4 clk idx */
#define AMEBA_CLK_AES  61 /* aes clk idx */
#define AMEBA_CLK_SHA  62 /* sha clk idx */
#define AMEBA_CLK_BOR  63 /* bor clk idx */

#define AMEBA_CLK_MAX 64 /* clk idx max */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBAD_CLOCK_H_ */

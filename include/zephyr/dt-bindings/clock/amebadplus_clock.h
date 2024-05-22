/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBADPLUS_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBADPLUS_CLOCK_H_

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
#define AMEBA_AON_CLK        0     /* aon clk idx */
#define AMEBA_AON_WODFT_CLK  1     /* aon wodft clk idx */
#define AMEBA_IWDG_CLK       2     /* iwdg clk idx */
#define AMEBA_ATIM_CLK       3     /* atim clk idx */
#define AMEBA_AUX131K_CLK    4     /* aux131k clk idx */
#define AMEBA_SDM_CLK        5     /* aon clk idx */
#define AMEBA_RTC_CLK        6     /* sdm clk idx */
#define AMEBA_OTPC_CLK       7     /* optc clk idx */

/* SYSON Domain clocks */
#define AMEBA_SDM_XCLK       10     /* sdm clk idx */
#define AMEBA_AUX40M_CLK     11     /* aux40M clk idx */
#define AMEBA_PWM0_CLK       12     /* pwm0 clk idx */
#define AMEBA_PWM1_CLK       13     /* pwm1 clk idx */
#define AMEBA_HTIM0_CLK      14     /* htim0 clk idx */
#define AMEBA_HTIM1_CLK      15     /* htim1 clk idx */
#define AMEBA_LEDC_CLK       16     /* ledc clk idx */

#define AMEBA_UART0_TCLK     17     /* uart0 tclk idx */
#define AMEBA_UART1_TCLK     18     /* uart1 tclk idx */
#define AMEBA_UART2_TCLK     19     /* uart2 tclk idx */
#define AMEBA_UART0_RCLK     20     /* uart0 rclk idx */
#define AMEBA_UART1_RCLK     21     /* uart1 rclk idx */
#define AMEBA_UART2_RCLK     22     /* uart2 rclk idx */
#define AMEBA_LOGUART_CLK    23     /* loguart clk idx */
#define AMEBA_DTIM_CLK       24     /* dtim clk idx */
#define AMEBA_ADC_CLK        25     /* adc clk idx */
#define AMEBA_GPIO_CLK       26     /* gpio clk idx */
#define AMEBA_LTIM0_CLK      27     /* ltim0 clk idx */
#define AMEBA_LTIM1_CLK      28     /* ltim1 clk idx */
#define AMEBA_LTIM2_CLK      29     /* ltim2 clk idx */
#define AMEBA_LTIM3_CLK      30     /* ltim3 clk idx */
#define AMEBA_LTIM4_CLK      31     /* ltim4 clk idx */
#define AMEBA_LTIM5_CLK      32     /* ltim5 clk idx */
#define AMEBA_LTIM6_CLK      33     /* ltim6 clk idx */
#define AMEBA_LTIM7_CLK      34     /* ltim7 clk idx */
#define AMEBA_PTIM0_CLK      35     /* ptim0 clk idx */
#define AMEBA_PTIM1_CLK      36     /* ptim1 clk idx */

#define AMEBA_AUX32K_CLK     37     /* aux32 clk idx */
#define AMEBA_WL_SCLK        38     /* wl clk idx */
#define AMEBA_LPON_CLK       39     /* lpon clk idx */
#define AMEBA_RTC_BCLK       40     /* rtc clk idx */
#define AMEBA_SDM_BCLK       41     /* sdm clk idx */
#define AMEBA_WDG_BCLK       42     /* wdg clk idx */
#define AMEBA_AIPC_CLK       43     /* aipc clk idx */
#define AMEBA_KSCAN_CLK      44     /* kscan clk idx */
#define AMEBA_SIC_CLK        45     /* sic clk idx */

/* SOC Domain clocks */
#define AMEBA_HP_CLK         50     /* hp clk idx */
#define AMEBA_SRAM_CLK       51     /* sram clk idx */

#define AMEBA_PERI_HCLK      52     /* peri clk idx */
#define AMEBA_DMAC_BCLK      53     /* dmac clk idx */
#define AMEBA_LX_BCLK        54     /* lx clk idx */
#define AMEBA_SDIO_BCLK      55     /* sdio clk idx */
#define AMEBA_SPI0_BCLK      56     /* spi0 bclk idx */
#define AMEBA_SPI1_BCLK      57     /* spi1 bclk idx */
#define AMEBA_WMAC_BCLK      58     /* wmac clk idx */
#define AMEBA_SPORT0_BCLK    59     /* sport0 bclk idx */
#define AMEBA_SPORT1_BCLK    60     /* sport1 bclk idx */
#define AMEBA_USB_BCLK       61     /* usb bclk idx */

#define AMEBA_SPIC_CLK          62  /* spic clk idx */
#define AMEBA_SPIC_CLK_NOGATED  63  /* spic_ng clk idx */
#define AMEBA_SPIC_PS_CLK       64  /* spic_ps clk idx */
#define AMEBA_PSRAM_CLK         65  /* psram clk idx */
#define AMEBA_PSRAM_CLK_NOGATED 66  /* psram_ng clk idx */

#define AMEBA_SPRAM_PS_CLK      67  /* soram_ps clk idx */
#define AMEBA_SPORT0_CLK        68  /* sport0 clk idx */
#define AMEBA_SPORT1_CLK        69  /* sport1 clk idx */

#define AMEBA_AC_CLK            70  /* ac clk idx */
#define AMEBA_QSPI_CLK          71  /* qspi clk idx */
#define AMEBA_UTMIFS_CLK        72  /* utmifs clk idx */
#define AMEBA_IRDA_CLK          73  /* irda clk idx */

#define AMEBA_LP_CLK            74  /* lp clk idx */
#define AMEBA_PERI_LCLK_NOGATED 75  /* peri_ng lclk idx */
#define AMEBA_PERI_LCLK         76  /* peri lclk idx */

#define AMEBA_DTIM_BCLK         77  /* dtim bclk idx */
#define AMEBA_PTIM0_BCLK        78  /* ptim0 bclk idx */
#define AMEBA_PTIM1_BCLK        79  /* ptim1 bclk idx */
#define AMEBA_ADC_BCLK          80  /* adc bclk idx */
#define AMEBA_GPIO_BCLK         81  /* gpio bclk idx */
#define AMEBA_LTIM_BCLK         82  /* ltim bclk idx */
#define AMEBA_UART_BCLK         83  /* uart bclk idx */
#define AMEBA_LOGUART_BCLK      84  /* loguart bclk idx */
#define AMEBA_I2C0_BCLK         85  /* i2c0 bclk idx */
#define AMEBA_I2C1_BCLK         86  /* i2c1 bclk idx */
#define AMEBA_LEDC_BCLK         87  /* ledc bclk idx */

#define AMEBA_AC_BCLK           88  /* ac bclk idx */
#define AMEBA_TRNG_BCLK         89  /* trng bclk idx */
#define AMEBA_IPC_BCLK          90  /* ipc bclk idx */
#define AMEBA_HTIM0_BCLK        91  /* htim0 bclk idx */
#define AMEBA_HTIM1_BCLK        92  /* htim1 bclk idx */
#define AMEBA_PWM0_BCLK         93  /* pwm0 bclk idx */
#define AMEBA_PWM1_BCLK         94  /* pwm1 bclk idx */
#define AMEBA_KSCAN_BCLK        95  /* kscan bclk idx */
#define AMEBA_IRDA_BCLK         96  /* irda bclk idx */
#define AMEBA_OTPC_BCLK         97  /* optc bclk idx */
#define AMEBA_SIC_BCLK          98  /* sic bclk idx */

/* BT Domain clocks */
#define AMEBA_CLK_BT_40M        105  /* bt40M clk idx */
#define AMEBA_CLK_BT_32K        106  /* bt32k clk idx */
#define AMEBA_CLK_BT_ANA        107  /* bt_ana clk idx */

/* misc clocks */
#define AMEBA_CLK_BTON          108  /* bton clk idx */
#define AMEBA_CLK_WDG           109  /* wdg clk idx */
#define AMEBA_CLK_CTC           110  /* ctc clk idx */
#define AMEBA_CLK_KM0           111  /* km0 clk idx */
#define AMEBA_CLK_KM4           112  /* km4 clk idx */
#define AMEBA_CLK_AES           113  /* aes clk idx */
#define AMEBA_CLK_SHA           114  /* sha clk idx */
#define AMEBA_CLK_BOR           115  /* bor clk idx */

#define AMEBA_CLK_MAX           116  /* clk idx max */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_AMEBADPLUS_CLOCK_H_ */

/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_lcdc

#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <soc.h>

LOG_MODULE_REGISTER(mipi_dbi_lcdc_ameba, CONFIG_MIPI_DBI_LOG_LEVEL);

enum lcdc_ameba_data_fmt {
	LCDC_AMEBA_DATA_FMT_BYTE = 0,
	LCDC_AMEBA_DATA_FMT_HALFWORD = 1, /* 2 byte */
	LCDC_AMEBA_DATA_FMT_WORD = 2,     /* 4 byte */
};

enum lcdc_ameba_cmd_dc {
	LCDC_AMEBA_COMMAND = 0,
	LCDC_AMEBA_DATA = 1,
};

enum lcdc_ameba_cmd_type {
	LCDC_AMEBA_RX = 0,
	LCDC_AMEBA_TX = 1,
};

enum lcdc_ameba_cmd_te {
	LCDC_AMEBA_TE_NO_SYNC = 0,
	LCDC_AMEBA_TE_RISING_EDGE = 1,
	LCDC_AMEBA_TE_FALLING_EDGE = 2,
};

/* Limit imposed by size of data length field in LCDC_AMEBA command */
#define LCDC_AMEBA_MAX_XFER            0x40000
/* Max reset width (in terms of Timer0_Period, see RST_CTRL register) */
#define LCDC_AMEBA_MAX_RST_WIDTH       0x3F
/* Max reset pulse count */
#define LCDC_AMEBA_MAX_RST_PULSE_COUNT 0x7

/* Descriptor for LCDC_AMEBA command */
union lcdc_ameba_trx_cmd {
	struct {
		/* Data length in bytes. LCDC_AMEBA transfers data_len + 1 */
		uint32_t data_len: 18;
		/* Dummy SCLK cycles between TX and RX (for SPI mode) */
		uint32_t dummy_count: 3;
		uint32_t rsvd: 2;
		/* Use auto repeat mode */
		uint32_t auto_repeat: 1;
		/* Tearing enable sync mode */
		uint32_t te_sync_mode: 2;
		/* TRX command timeout mode */
		uint32_t trx_timeout_mode: 1;
		/* Data format, see lcdc_ameba_data_fmt */
		uint32_t data_format: 2;
		/* Enable command done interrupt */
		uint32_t cmd_done_int: 1;
		/* LCD command or LCD data, see lcdc_ameba_cmd_dc */
		uint32_t cmd_data: 1;
		/* TX or RX command, see lcdc_ameba_cmd_type */
		uint32_t trx: 1;
	} bits;
	uint32_t u32;
};

struct mipi_dbi_lcdc_ameba_config {
	/* LCDC_AMEBA_Type *base; */
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	bool swap_bytes;
	uint8_t write_active_min;
	uint8_t write_inactive_min;
	uint8_t timer0_ratio;
	uint8_t timer1_ratio;
};

#ifdef CONFIG_MIPI_DBI_LCDC_AMEBA_DMA
struct stream {
	const struct device *dma_dev;
	uint32_t channel;
	struct dma_config dma_cfg;
	struct dma_block_config blk_cfg[2];
};
#endif

struct mipi_dbi_lcdc_ameba_data {
	/* Tracks number of bytes remaining in command */
	uint32_t cmd_bytes;
	/* Tracks number of bytes remaining in transfer */
	uint32_t xfer_bytes;
	/* Tracks start of transfer buffer */
	const uint8_t *xfer_buf;
	/* When sending data that does not evenly fit into 4 byte chunks,
	 * this is used to store the last unaligned segment of the data.
	 */
	uint32_t unaligned_word __aligned(4);
	/* Tracks lcdc_ameba_data_fmt value we should use for pixel data */
	uint8_t pixel_fmt;
	/* Tracks TE edge setting we should use for pixel data */
	uint8_t te_edge;
	/* Are we starting a new display frame */
	bool new_frame;
	const struct mipi_dbi_config *active_cfg;
	struct k_sem xfer_sem;
	struct k_sem lock;
#ifdef CONFIG_MIPI_DBI_LCDC_AMEBA_DMA
	struct stream dma_stream;
#endif
};

/* RX and TX FIFO thresholds */
#ifdef CONFIG_MIPI_DBI_LCDC_AMEBA_DMA
#define LCDC_AMEBA_RX_FIFO_THRESH 0x0
#define LCDC_AMEBA_TX_FIFO_THRESH 0x0
#else
#define LCDC_AMEBA_RX_FIFO_THRESH 0x0
#define LCDC_AMEBA_TX_FIFO_THRESH 0x3
#endif

static int mipi_dbi_lcdc_ameba_write_display(const struct device *dev,
					     const struct mipi_dbi_config *dbi_config,
					     const uint8_t *framebuf,
					     struct display_buffer_descriptor *desc,
					     enum display_pixel_format pixfmt)
{
	return 0;
}

static int mipi_dbi_lcdc_ameba_write_cmd(const struct device *dev,
					 const struct mipi_dbi_config *dbi_config, uint8_t cmd,
					 const uint8_t *data, size_t data_len)
{
	return 0;
}

static int mipi_dbi_lcdc_ameba_reset(const struct device *dev, k_timeout_t delay)
{
	return 0;
}

static int mipi_dbi_lcdc_ameba_configure_te(const struct device *dev, uint8_t edge,
					    k_timeout_t delay)
{
	return 0;
}

/* Initializes LCDC_AMEBA peripheral */
static int mipi_dbi_lcdc_ameba_init(const struct device *dev)
{

	return 0;
}

static DEVICE_API(mipi_dbi, mipi_dbi_lcdc_ameba_driver_api) = {
	.command_write = mipi_dbi_lcdc_ameba_write_cmd,
	.write_display = mipi_dbi_lcdc_ameba_write_display,
	.configure_te = mipi_dbi_lcdc_ameba_configure_te,
	.reset = mipi_dbi_lcdc_ameba_reset,
};

static void mipi_dbi_lcdc_ameba_isr(const struct device *dev)
{
}

#ifdef CONFIG_MIPI_DBI_LCDC_AMEBA_DMA
#define LCDC_AMEBA_DMA_CHANNELS(n)                                                                 \
	.dma_stream =                                                                              \
		{                                                                                  \
                                                                                                   \
	},                                                                                         \
	mipi_dbi_lcdc_ameba_config_func
#else
#define LCDC_AMEBA_DMA_CHANNELS(n)
#endif

static void mipi_dbi_lcdc_ameba_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), mipi_dbi_lcdc_ameba_isr,
		    DEVICE_DT_INST_GET(0), 0);

	irq_enable(DT_INST_IRQN(0));
}

PINCTRL_DT_INST_DEFINE(0);

static const struct mipi_dbi_lcdc_ameba_config mipi_dbi_lcdc_config = {
	/* .base = DT_INST_REG_ADDR(0),*/
	.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.irq_config_func = mipi_dbi_lcdc_ameba_config_func,
};

static struct mipi_dbi_lcdc_ameba_data mipi_dbi_lcdc_data = {LCDC_AMEBA_DMA_CHANNELS(0)};

DEVICE_DT_INST_DEFINE(0, mipi_dbi_lcdc_ameba_init, NULL, &mipi_dbi_lcdc_data, &mipi_dbi_lcdc_config,
		      POST_KERNEL, CONFIG_MIPI_DBI_INIT_PRIORITY, &mipi_dbi_lcdc_ameba_driver_api);

/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba SPI interface
 */

#define DT_DRV_COMPAT realtek_amebapro2_spi
/* Include <soc.h> before <ameba_soc.h> to avoid redefining unlikely() macro */
#include <soc.h>
#include <hal_ssi.h>
#include <hal_gpio.h>

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ameba_spi, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

#define SPI_DMA_RX_EN (1 << 0)
#define SPI_DMA_TX_EN (1 << 1)

struct spi_ameba_data {
	bool initialized;
	uint32_t datasize; /* real dfs */
	uint8_t fifo_diff; /* cannot be bigger than FIFO depth */
};

struct spi_ameba_config {

	/* pinctrl info */
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SPI_AMEBA_INTERRUPT
	void (*irq_configure)();
#endif
};

hal_ssi_adaptor_t hal_ssi_adaptor;

#if defined(CONFIG_SPI_AMEBAPRO2_DMA)
hal_gdma_adaptor_t spi_gdma_adp_tx;
hal_gdma_adaptor_t spi_gdma_adp_rx;
#endif

static int spi_dma_en;

/**static int spi_ameba_frame_exchange(const struct device *dev)
 *{
 *	return 0;
 *}
 */

static int spi_ameba_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
	struct spi_ameba_data *data = dev->data;

#ifdef CONFIG_SPI_AMEBAPRO2_DMA

#endif

	if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("Slave mode is not supported on %s", dev->name);
		return -EINVAL;
	}

	if (spi_cfg->operation & SPI_MODE_LOOP) {
		LOG_ERR("Loopback mode is not supported");
		return -EINVAL;
	}

	if (spi_cfg->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("LSB mode is supported");
		return -EINVAL;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    (spi_cfg->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Only single line mode is supported");
		return -EINVAL;
	}

#ifdef CONFIG_SPI_AMEBAPRO2_DMA
	hal_ssi_adaptor.ptx_gdma_adaptor = &spi_gdma_adp_tx;
	hal_ssi_adaptor.prx_gdma_adaptor = &spi_gdma_adp_rx;
#endif
	/* SPI0 (S0)*/
	hal_ssi_adaptor.spi_pin.spi_cs_pin = PIN_E4;
	hal_ssi_adaptor.spi_pin.spi_clk_pin = PIN_E1;
	hal_ssi_adaptor.spi_pin.spi_mosi_pin = PIN_E3;
	hal_ssi_adaptor.spi_pin.spi_miso_pin = PIN_E2;
	hal_ssi_adaptor.index = 0;
	if (data->initialized != true) {
		if ((hal_ssi_init(&hal_ssi_adaptor)) != HAL_OK) {
			LOG_ERR("spi_format(): SPI 0 init fails.\n\r");
			return -1;
		}
	}

	/*Initialize ISR setting to prevent from inadvert effects*/
	hal_ssi_set_interrupt_mask(&hal_ssi_adaptor, 0);

	hal_ssi_set_device_role(&hal_ssi_adaptor, SsiMaster);

	data->datasize = SPI_WORD_SIZE_GET(spi_cfg->operation);

	/* set format */
	hal_spi_format(&hal_ssi_adaptor, (SPI_WORD_SIZE_GET(spi_cfg->operation) - 1),
		       (SPI_MODE_GET(spi_cfg->operation) >> 1));

	if (SPI_OP_MODE_GET(spi_cfg->operation) == SPI_OP_MODE_SLAVE) {
		if ((SPI_MODE_GET(spi_cfg->operation) >> 1) & SPI_MODE_CPOL) {
			hal_gpio_pull_ctrl(hal_ssi_adaptor.spi_pin.spi_clk_pin, Pin_PullUp);
		} else {
			hal_gpio_pull_ctrl(hal_ssi_adaptor.spi_pin.spi_clk_pin, Pin_PullDown);
		}
	}

	/* set frequency */
	hal_ssi_set_sclk(&hal_ssi_adaptor, spi_cfg->frequency);

	data->initialized = true;

	return 0;
}

static int spi_ameba_transceive_impl(const struct device *dev, const struct spi_config *spi_cfg,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs, bool asynchronous,
				     spi_callback_t cb, void *userdata)
{
	int ret = 0;

	spi_ameba_configure(dev, spi_cfg);

	/* wait bus idle*/
	while (hal_ssi_get_busy(&hal_ssi_adaptor)) {
		/* Wait until last frame transfer complete. */
	}

	if (SPI_OP_MODE_GET(spi_cfg->operation) == SPI_OP_MODE_MASTER) {
		/*master*/
		if (tx_bufs->count != 0) {
			/*master Tx*/
#ifdef CONFIG_SPI_AMEBAPRO2_DMA
			if ((spi_dma_en & SPI_DMA_TX_EN) == 0) {
				if (HAL_OK ==
				    hal_ssi_tx_gdma_init(&hal_ssi_adaptor,
							 hal_ssi_adaptor.ptx_gdma_adaptor)) {
					spi_dma_en |= SPI_DMA_TX_EN;
				} else {
					return HAL_BUSY;
				}
			}
			ret = hal_ssi_dma_send(&hal_ssi_adaptor, tx_bufs->buffers->buf,
					       tx_bufs->count);
#else
			ret = hal_ssi_interrupt_init_write(&hal_ssi_adaptor, tx_bufs->buffers->buf,
							   tx_bufs->count);
#endif

		} else {
			/*master Rx*/
#ifdef CONFIG_SPI_AMEBAPRO2_DMA
			if ((spi_dma_en & SPI_DMA_TX_EN) == 0) {
				if (HAL_OK ==
				    hal_ssi_tx_gdma_init(&hal_ssi_adaptor,
							 hal_ssi_adaptor.ptx_gdma_adaptor)) {
					spi_dma_en |= SPI_DMA_TX_EN;
				} else {
					return HAL_BUSY;
				}
			}
			if ((spi_dma_en & SPI_DMA_RX_EN) == 0) {
				if (HAL_OK ==
				    hal_ssi_rx_gdma_init(&hal_ssi_adaptor,
							 hal_ssi_adaptor.prx_gdma_adaptor)) {
					spi_dma_en |= SPI_DMA_RX_EN;
				} else {
					return HAL_BUSY;
				}
			}
			/* as Master mode, sending data will receive data at sametime */
			ret = hal_ssi_dma_recv(&hal_ssi_adaptor, rx_bufs->buffers->buf,
					       rx_bufs->count);
			if (ret == HAL_OK) {
				ret = hal_ssi_dma_send(&hal_ssi_adaptor, tx_bufs->buffers->buf,
						       rx_bufs->count);
			} else {
				return ret;
			}
#else
			ret = hal_ssi_interrupt_init_read(&hal_ssi_adaptor, rx_bufs->buffers->buf,
							  rx_bufs->count);
			/* as Master mode, it need to push data to TX FIFO to generate clock out*/
			/* then the slave can transmit data out */
			/* send some dummy data out*/
			ret = hal_ssi_interrupt_init_write(&hal_ssi_adaptor, NULL, rx_bufs->count);
#endif
		}
	} else {
		/*slave*/
	}

	while (hal_ssi_get_busy(&hal_ssi_adaptor)) {
		/* Wait until last frame transfer complete. */
	}

#ifdef CONFIG_SPI_AMEBAPRO2_DMA
dma_error:
#endif

	return ret;
}

static int spi_ameba_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_ameba_data *data = dev->data;

	hal_ssi_deinit(&hal_ssi_adaptor);
	spi_dma_en = 0;
	data->initialized = false;

	return 0;
}

static int spi_ameba_init(const struct device *dev)
{
#ifdef CONFIG_SPI_AMEBA_INTERRUPT
	cfg->irq_configure(dev);
#endif

	return 0;
}

static int spi_ameba_transceive(const struct device *dev, const struct spi_config *spi_cfg,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	return spi_ameba_transceive_impl(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

static const struct spi_driver_api ameba_spi_api = {.transceive = spi_ameba_transceive,
						    .release = spi_ameba_release};

static struct spi_ameba_data spi_ameba_data = {.initialized = false};

static struct spi_ameba_config spi_ameba_config = {};

DEVICE_DT_INST_DEFINE(0, &spi_ameba_init, NULL, &spi_ameba_data, &spi_ameba_config, POST_KERNEL,
		      CONFIG_SPI_INIT_PRIORITY, &ameba_spi_api);

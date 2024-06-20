/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba SPI interface
 */

#define DT_DRV_COMPAT realtek_ameba_spi


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ameba_spi, CONFIG_SPI_LOG_LEVEL);

#include <zephyr/drivers/spi.h>

#include <zephyr/drivers/clock_control.h>
#include "spi_context.h"

/* Device data structure */
struct spi_ameba_data {
	/* clock device */
	int data;
};

struct spi_ameba_config {
	int irq_source;
};

static int spi_ameba_transfer(const struct device *dev)
{
	return 0;
}

#ifdef CONFIG_SPI_AMEBA_INTERRUPT
static void spi_ameba_isr(void *arg)
{
}
#endif


static int spi_ameba_init(const struct device *dev)
{
	return 0;
}


static int spi_ameba_transceive(const struct device *dev,
								const struct spi_config *spi_cfg,
								const struct spi_buf_set *tx_bufs,
								const struct spi_buf_set *rx_bufs)
{
	return 0;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_ameba_transceive_async(const struct device *dev,
									  const struct spi_config *spi_cfg,
									  const struct spi_buf_set *tx_bufs,
									  const struct spi_buf_set *rx_bufs,
									  spi_callback_t cb,
									  void *userdata)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_ameba_release(const struct device *dev,
							 const struct spi_config *config)
{
	return 0;
}

static const struct spi_driver_api spi_api = {
	.transceive = spi_ameba_transfer,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_ameba_transceive_async,
#endif
	.release = spi_ameba_release
};

#define AMEBA_SPI_INIT(idx)	\
				\
	static struct spi_ameba_data spi_data_##idx = {	\
		.data = 0,	\
	};	\
		\
	static const struct spi_ameba_config spi_config_##idx = {	\
		.irq_source = DT_INST_IRQN(idx), \
	};	\
		\
	DEVICE_DT_INST_DEFINE(idx, &spi_ameba_init,	\
			      NULL, &spi_data_##idx,	\
			      &spi_config_##idx, POST_KERNEL,	\
			      CONFIG_SPI_INIT_PRIORITY, &spi_api);

DT_INST_FOREACH_STATUS_OKAY(AMEBA_SPI_INIT)

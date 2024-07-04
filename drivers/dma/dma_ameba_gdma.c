/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_gdma

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_ameba_gdma, CONFIG_DMA_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/drivers/dma.h>
// #include <zephyr/drivers/dma/dma_ameba.h>
#include <zephyr/drivers/clock_control.h>

struct dma_ameba_data {
	struct dma_context dma_ctx;
};

enum dma_channel_dir {
	DMA_RX,
	DMA_TX,
	DMA_UNCONFIGURED
};

struct dma_ameba_channel {
	uint8_t dir;
	uint8_t channel_id;
	int host_id;
	int periph_id;
	dma_callback_t cb;
	void *user_data;
};

struct dma_ameba_config {
	uint32_t base;
	uint8_t dma_channel_max;
	uint8_t sram_alignment;
	// struct dma_ameba_channel dma_channel[DMA_MAX_CHANNEL];
	void (*config_irq)(const struct device *dev);
	struct device *src_dev;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
};

static void dma_ameba_isr_handler(const struct device *dev, uint32_t channel)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);
}

static int dma_ameba_config(const struct device *dev, uint32_t channel,
							struct dma_config *config_dma)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);
	ARG_UNUSED(config_dma);

	return 0;
}

static int dma_ameba_start(const struct device *dev, uint32_t channel)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);

	return 0;
}

static int dma_ameba_stop(const struct device *dev, uint32_t channel)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);

	return 0;
}

static int dma_ameba_get_status(const struct device *dev, uint32_t channel,
								struct dma_status *status)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);
	ARG_UNUSED(status);

	return 0;
}

static int dma_ameba_reload(const struct device *dev, uint32_t channel, uint32_t src, uint32_t dst,
							size_t size)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);
	ARG_UNUSED(src);
	ARG_UNUSED(dst);
	ARG_UNUSED(size);

	return 0;
}

static int dma_ameba_init(const struct device *dev)
{
	struct dma_ameba_config *config = (struct dma_ameba_config *)dev->config;
	int ret = 0;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("Could not initialize clock (%d)", ret);
		return ret;
	}

	config->config_irq(dev);

	return 0;
}

static const struct dma_driver_api dma_ameba_api = {
	.config = dma_ameba_config,
	.start = dma_ameba_start,
	.stop = dma_ameba_stop,
	.get_status = dma_ameba_get_status,
	.reload = dma_ameba_reload,
};

/*
 * Macro to CONNECT and enable each irq (order is given by the 'listify')
 * chan: channel of the DMA instance (assuming one irq per channel)
 * dma : dma instance (one GDMA instance on ameba)
 */
#define DMA_AMEBA_IRQ_CONNECT_CHANNEL(chan, dma)			\
	do {													\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(dma, chan, irq),		\
			    DT_INST_IRQ_BY_IDX(dma, chan, priority),	\
			    dma_ameba_irq_##dma##_##chan,				\
			    DEVICE_DT_INST_GET(dma), 0);				\
		irq_enable(DT_INST_IRQ_BY_IDX(dma, chan, irq));		\
	} while (0)

/*
 * Macro to configure the irq for each dma instance (n)
 * Loop to CONNECT and enable each irq for each channel
 * Expecting as many irq as property <dma_channels>
 */
#define DMA_AMEBA_IRQ_CONNECT(n)									\
	static void dma_ameba_config_irq_##n(const struct device *dev)	\
	{									\
		ARG_UNUSED(dev);				\
										\
		LISTIFY(DT_INST_PROP(n, dma_channels),			\
			DMA_AMEBA_IRQ_CONNECT_CHANNEL, (;), n);		\
	}

/*
 * Macro to instanciate the irq handler (order is given by the 'listify')
 * chan: channel of the DMA instance (assuming one irq per channel)
 * dma : dma instance (one GDMA instance on ameba)
 */
#define DMA_AMEBA_DEFINE_IRQ_HANDLER(chan, dma)							\
	static void dma_ameba_irq_##dma##_##chan(const struct device *dev)	\
	{											\
		dma_ameba_isr_handler(dev, chan);		\
	}


#define DMA_AMEBA_INIT(n)                                                               \
	BUILD_ASSERT(DT_INST_PROP(n, dma_channels) == DT_NUM_IRQS(DT_DRV_INST(n)),			\
		"Nb of Channels and IRQ mismatch");				\
														\
	LISTIFY(DT_INST_PROP(n, dma_channels),				\
		DMA_AMEBA_DEFINE_IRQ_HANDLER, (;), n);			\
														\
	DMA_AMEBA_IRQ_CONNECT(n);							\
														\
	static const struct dma_ameba_config dma_config_##n = {                             \
		.base = DT_INST_REG_ADDR(n),				\
		.dma_channel_max = DT_INST_PROP(n, dma_channels),                               \
		.sram_alignment = DT_INST_PROP(n, dma_buf_addr_alignment),                      \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                             \
		.clock_subsys = (void *)DT_INST_CLOCKS_CELL(n, idx),                          	\
		.config_irq = dma_ameba_config_irq_##n,										\
	};   																				\
	                                                               						\
	static struct dma_ameba_data dma_data_##n = {                                       \
	};                                                                                  \
                                                                                   		\
	DEVICE_DT_INST_DEFINE(n, &dma_ameba_init, NULL, &dma_data_##n, &dma_config_##n,     \
			      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY, &dma_ameba_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_AMEBA_INIT)

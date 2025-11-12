/*
* Copyright (c) 2025 Realtek Semiconductor Corp.
*
* SPDX-License-Identifier: Apache-2.0
*/

#define DT_DRV_COMPAT realtek_ameba_ppe

#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/misc/ameba_ppe/ameba_ppe.h>

LOG_MODULE_REGISTER(ameba_ppe, CONFIG_AMEBA_PPE_LOG_LEVEL);

struct ameba_ppe_config {
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(const struct device *dev);
};

struct ameba_ppe_data {
	struct k_mutex lock;
	struct k_sem sem;
};

static int ppe_get_px_format(enum ppe_color_format_type cf)
{
	switch(cf) {
		case PPE_COLOR_FORMAT_RGB565:   return PPE_RGB565;
		case PPE_COLOR_FORMAT_RGB888:   return PPE_RGB888;
		case PPE_COLOR_FORMAT_XRGB8888:
		case PPE_COLOR_FORMAT_ARGB8888: return PPE_ARGB8888;
		default: return PPE_ARGB8888;
	}
}

static void ameba_ppe_isr(const struct device *dev)
{
	struct ameba_ppe_data *data = dev->data;
	uint32_t irq_status = PPE_GetAllIntStatus();

	if (irq_status & PPE_BIT_INTR_ST_ALL_OVER) {
		PPE_ClearINTPendingBit(PPE_BIT_INTR_ST_ALL_OVER);
		k_sem_give(&data->sem);
	}
}

static int ameba_ppe_init(const struct device *dev)
{
	const struct ameba_ppe_config *cfg = dev->config;
	struct ameba_ppe_data *data = dev->data;
	int ret = 0;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0) {
		LOG_ERR("Could not initialize clock (%d)", ret);
		return ret;
	}

	k_sem_init(&data->sem, 0, 1);
	k_mutex_init(&data->lock);
	cfg->irq_config_func(dev);

	return ret;
}

int ameba_ppe_configure_and_transfer(const struct device *dev,
				struct ppe_configuration *ppe_draw_conf)
{
	struct ameba_ppe_data *data = dev->data;
	uint8_t input_layer_id = PPE_INPUT_LAYER1_INDEX;
	PPE_InputLayer_InitTypeDef Input_Layer;
	PPE_InputLayer_StructInit(&Input_Layer);
	k_mutex_lock(&data->lock, K_FOREVER);

	Input_Layer.src_addr       = (uint32_t)ppe_draw_conf->src_buf;
	Input_Layer.pic_width      = ppe_draw_conf->src_header->w;
	Input_Layer.pic_height     = ppe_draw_conf->src_header->h;
	Input_Layer.format         = ppe_get_px_format(ppe_draw_conf->src_header->cf);
	Input_Layer.pic_src        = ppe_draw_conf->src_buf ? PPE_LAYER_SRC_FROM_DMA : PPE_LAYER_SRC_CONST;
	Input_Layer.interp         = PPE_INTERP_TYPE_Nearest_Neighbor;
	Input_Layer.key_mode       = PPE_KEY_MODE_DISABLE;
	Input_Layer.line_len       = ppe_draw_conf->src_header->stride;
	Input_Layer.const_ABGR8888_value = ppe_draw_conf->src_header->color;
	Input_Layer.win_min_x      = ppe_draw_conf->src_header->min_x;
	Input_Layer.win_min_y      = ppe_draw_conf->src_header->min_y;
	Input_Layer.win_max_x      = Input_Layer.pic_width;
	Input_Layer.win_max_y      = Input_Layer.pic_height;
	Input_Layer.key_min_bgr    = 0;
	Input_Layer.key_max_bgr    = 0;
	Input_Layer.scale_x        = ppe_draw_conf->scale_x;
	Input_Layer.scale_y        = ppe_draw_conf->scale_y;
	// Layer2 and layer3 can't support rotation
	if (ppe_draw_conf->angle && ppe_draw_conf->src_header->cf != PPE_COLOR_FORMAT_ARGB8888) {
		Input_Layer.angle = ppe_draw_conf->angle;
		if (ppe_draw_conf->angle == 90 || ppe_draw_conf->angle == 270) {
			Input_Layer.win_max_x = ppe_draw_conf->src_header->h;
			Input_Layer.win_max_y = ppe_draw_conf->src_header->w;
		}
	} else {
		Input_Layer.angle = 0;
	}

	if (ppe_draw_conf->opa < OPA_MAX) {
		input_layer_id = PPE_INPUT_LAYER2_INDEX;
		PPE_InputLayer_InitTypeDef BG_Layer;
		PPE_InputLayer_StructInit(&BG_Layer);
		BG_Layer.src_addr               = (uint32_t)ppe_draw_conf->dest_buf;
		BG_Layer.pic_width              = ppe_draw_conf->dest_header->w;
		BG_Layer.pic_height             = ppe_draw_conf->dest_header->h;
		BG_Layer.format                 = ppe_get_px_format(ppe_draw_conf->dest_header->cf);
		BG_Layer.pic_src                = PPE_LAYER_SRC_FROM_DMA;
		BG_Layer.interp                 = PPE_INTERP_TYPE_Nearest_Neighbor;
		BG_Layer.key_mode               = PPE_KEY_MODE_DISABLE;
		BG_Layer.line_len               = ppe_draw_conf->dest_header->stride;
		BG_Layer.const_ABGR8888_value   = 0xFFFFFFFF;
		BG_Layer.win_min_x              = 0;
		BG_Layer.win_min_y              = 0;
		BG_Layer.win_max_x              = BG_Layer.pic_width;
		BG_Layer.win_max_y              = BG_Layer.pic_height;
		BG_Layer.key_min_bgr            = 0;
		BG_Layer.key_max_bgr            = 0;
		BG_Layer.scale_x                = 1;
		BG_Layer.scale_y                = 1;
		BG_Layer.angle                  = 0;
		PPE_InitInputLayer(PPE_INPUT_LAYER1_INDEX, &BG_Layer);
	}

	PPE_ResultLayer_InitTypeDef Result_Layer;
	PPE_ResultLayer_StructInit(&Result_Layer);
	Result_Layer.src_addr       = (uint32_t)ppe_draw_conf->dest_buf;
	Result_Layer.pic_width      = ppe_draw_conf->dest_header->w;
	Result_Layer.pic_height     = ppe_draw_conf->dest_header->h;
	Result_Layer.format         = ppe_get_px_format(ppe_draw_conf->dest_header->cf);
	Result_Layer.bg_src         = PPE_BACKGROUND_SOURCE_LAYER1;
	Result_Layer.line_len       = ppe_draw_conf->dest_header->stride;
	Result_Layer.const_bg       = 0xFFFFFFFF;
	Result_Layer.blk_width      = Result_Layer.pic_width;
	Result_Layer.blk_height     = Result_Layer.pic_height;
	Result_Layer.xor_en         = 0;

	PPE_InitInputLayer(input_layer_id, &Input_Layer);
	PPE_InitResultLayer(&Result_Layer);
	sys_cache_data_flush_and_invd_all();

	if (input_layer_id == PPE_INPUT_LAYER2_INDEX) {
		PPE_LayerEn(PPE_INPUT_LAYER1_BIT | PPE_INPUT_LAYER2_BIT);
	} else {
		PPE_LayerEn(PPE_INPUT_LAYER1_BIT);
	}

	PPE_MaskINTConfig(PPE_BIT_INTR_ST_ALL_OVER, ENABLE);
	PPE_Cmd(ENABLE);
	//while(PPE_GetGlobalState()!=PPE_STATE_DISABLE);
	k_sem_take(&data->sem, K_FOREVER);
	k_mutex_unlock(&data->lock);

	return 0;
}

static DEVICE_API(ppe, ameba_ppe_api_funcs) = {
	.configure_transfer = ameba_ppe_configure_and_transfer,
};

static void ameba_ppe_irq_config(const struct device *dev)
{
	ARG_UNUSED(dev);
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), ameba_ppe_isr,
			DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));
}

static const struct ameba_ppe_config ppe_config = {
	.irq_config_func = ameba_ppe_irq_config,
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clock_subsys = (void *)DT_INST_CLOCKS_CELL(0, idx),
};

static struct ameba_ppe_data ppe_data;

DEVICE_DT_INST_DEFINE(0, &ameba_ppe_init, NULL, &ppe_data, &ppe_config, POST_KERNEL,
			CONFIG_AMEBA_PPE_INIT_PRIORITY, &ameba_ppe_api_funcs);
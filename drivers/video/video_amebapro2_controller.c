/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba VIDEO Controller
 */

#define DT_DRV_COMPAT realtek_amebapro2_video_controller

#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/logging/log.h>
#include <video_api.h>
LOG_MODULE_REGISTER(video_amebapro2_controller, CONFIG_VIDEO_LOG_LEVEL);

#define PRIORITY        5
#define VIDEO_STACKSIZE 16 * 1024

static int video_init_flag;
K_THREAD_STACK_DEFINE(vidoe_init_stack_area, VIDEO_STACKSIZE);
static struct k_thread video_init_thread;
K_THREAD_STACK_DEFINE(video_task_stack_area, VIDEO_STACKSIZE);
static struct k_thread video_task_thread;

void video_task_create(rtos_task_t *pp_handle, const char *p_name, void (*p_routine)(void *),
		       void *p_param, uint16_t stack_size_in_byte, uint16_t priority)
{
	struct k_thread *p_thread = &video_task_thread;

	k_thread_create(p_thread, video_task_stack_area, VIDEO_STACKSIZE,
			(k_thread_entry_t)p_routine, p_param, NULL, NULL, priority, 0, K_NO_WAIT);
	k_thread_name_set(p_thread, p_name);
	*pp_handle = (rtos_task_t)p_thread;
}

static void video_init_task(void *param, void *param1, void *param2)
{
	extern const unsigned char iq_data[];
	extern const unsigned char sensor_data[];

	video_buf_calc(v_buf_cfg.v1_enable, v_buf_cfg.v1_width, v_buf_cfg.v1_height,
		       v_buf_cfg.v2_bps, v_buf_cfg.v1_snapshot, v_buf_cfg.v2_enable,
		       v_buf_cfg.v2_width, v_buf_cfg.v2_height, v_buf_cfg.v2_bps,
		       v_buf_cfg.v2_snapshot, v_buf_cfg.v3_enable, v_buf_cfg.v3_width,
		       v_buf_cfg.v3_height, v_buf_cfg.v3_bps, v_buf_cfg.v3_snapshot,
		       v_buf_cfg.v4_enable, v_buf_cfg.v4_width, v_buf_cfg.v4_height);

	video_init((int)iq_data, (int)sensor_data);
	video_init_flag = 1;
}

static void video_thread_init(void)
{
	k_thread_create(&video_init_thread, vidoe_init_stack_area, VIDEO_STACKSIZE, video_init_task,
			NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
}

static int video_amebapro2_controller_init(const struct device *dev)
{
	int threshold_cnt = 0;

	video_thread_init();
	while (1) {
		if (video_init_flag) {
			break;
		} else if (threshold_cnt >= 10000) {
			LOG_INF("%s fail\r\n", __func__);
			return -1;
		}
		threshold_cnt++;
		k_sleep(K_MSEC(100));
	}
	LOG_INF("%s finish %d\r\n", __func__, video_init_flag);
	return 0;
}

DEVICE_DT_INST_DEFINE(0, video_amebapro2_controller_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_VIDEO_INIT_PRIORITY, NULL);

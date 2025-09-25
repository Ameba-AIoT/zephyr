/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <strings.h>
#include <zephyr/drivers/video-controls.h>

#define VIDEO_DEV_SW "VIDEO_AMEBAPRO2"
#include <os_wrapper.h>
#include <video_api.h>

#define AMEBA_VIDEO_CLOSE 0
#define AMEBA_VIDEO_OPEN  1

LOG_MODULE_REGISTER(video_capture, LOG_LEVEL_DBG);

void mem_dump(const void *addr, size_t len)
{
	const uint8_t *p = (const uint8_t *)addr;
	size_t i;

	for (i = 0; i < len; i++) {
		if (i % 16 == 0) {
			printk("%08x: ", (unsigned int)((uintptr_t)(p + i)));
		}

		printk("%02x ", p[i]);

		if (i % 16 == 15 || i == len - 1) {
			printk("\n");
		}
	}
}
const struct device *video;
static int video_status = AMEBA_VIDEO_CLOSE;

void video_task(void *param)
{
	struct video_buffer buffers[16], *vbuf;
	struct video_format fmt;
	struct video_caps caps;
	unsigned int frame = 0;
	int i = 0;

	if (video == NULL) {
		video = device_get_binding(VIDEO_DEV_SW);
		if (video == NULL) {
			LOG_ERR("Video device %s not found", VIDEO_DEV_SW);
			goto EXIT;
		}
	}

	if (video_get_caps(video, VIDEO_EP_OUT, &caps)) {
		LOG_ERR("Unable to retrieve video capabilities");
		goto EXIT;
	}

	LOG_INF("- Capabilities:\n");
	while (caps.format_caps[i].pixelformat) {
		const struct video_format_cap *fcap = &caps.format_caps[i];

		LOG_INF("  %c%c%c%c width [%u; %u; %u] height [%u; %u; %u]\n",
			(char)fcap->pixelformat, (char)(fcap->pixelformat >> 8),
			(char)(fcap->pixelformat >> 16), (char)(fcap->pixelformat >> 24),
			fcap->width_min, fcap->width_max, fcap->width_step, fcap->height_min,
			fcap->height_max, fcap->height_step);
		i++;
	}

	if (video_get_format(video, VIDEO_EP_OUT, &fmt)) {
		LOG_ERR("Unable to retrieve video format");
		goto EXIT;
	}

	if (video_set_format(video, VIDEO_EP_OUT, &fmt)) {
		LOG_ERR("Unable to retrieve video format");
		return;
	}

	LOG_INF("- Default format: %c%c%c%c %ux%u\n", (char)fmt.pixelformat,
		(char)(fmt.pixelformat >> 8), (char)(fmt.pixelformat >> 16),
		(char)(fmt.pixelformat >> 24), fmt.width, fmt.height);

	memset(buffers, 0, sizeof(buffers));
	for (i = 0; i < ARRAY_SIZE(buffers); i++) {
		video_enqueue(video, VIDEO_EP_OUT, &buffers[i]);
	}

	if (video_stream_start(video)) {
		LOG_ERR("Unable to start capture (interface)");
		goto EXIT;
	}

	LOG_INF("Capture started\n");

	video_status = AMEBA_VIDEO_OPEN;

	while (1) {
		int err;

		err = video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_MSEC(500));
		if (err) {
			LOG_ERR("Unable to dequeue video buf");
			goto EXIT;
		}
		frame++;
		if (frame % 30 == 0) {
			LOG_INF("\rGot frame %u! size: %u; timestamp %u ms", frame, vbuf->size,
				vbuf->timestamp);
			mem_dump(vbuf->buffer, 16);
		}
		err = video_enqueue(video, VIDEO_EP_OUT, vbuf);
		if (err) {
			LOG_ERR("Unable to requeue video buf");
			goto EXIT;
		}
	}
EXIT:
	LOG_INF("The video task is closed\r\n");
	video_status = AMEBA_VIDEO_CLOSE;
}

void video_thread_init(void)
{
	rtos_task_create(NULL, "video", video_task, NULL, 16 * 1024, 3);
}

int main(void)
{
	LOG_INF("amebapro2 video example\r\n");
	LOG_INF("Enter the video start to run the video\r\n");
	LOG_INF("Enter the video stop to stop the video\r\n");
	return 0;
}

static int cmd_video_start(const struct shell *shell, size_t argc, char **argv)
{
	if (video_status == AMEBA_VIDEO_CLOSE) {
		video_thread_init();
		shell_print(shell, "Video started");
	} else {
		shell_print(shell, "Video is open status\r\n");
	}
	return 0;
}

static int cmd_video_stop(const struct shell *shell, size_t argc, char **argv)
{
	if (video_status == AMEBA_VIDEO_OPEN) {
		video_stream_stop(video);
		shell_print(shell, "Video stopped");
	} else {
		shell_print(shell, "Video is close status\r\n");
	}
	return 0;
}

static int cmd_video_ctrl(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(shell, "Usage: video cmd <operatrion> <value>");
		return -EINVAL;
	}
	int ret = 0;
	const char *subcmd = argv[1];
	int val = strtol(argv[2], NULL, 0);

	if (video_status != AMEBA_VIDEO_OPEN) {
		shell_print(shell, "The video is not open\r\n");
		return -EINVAL;
	}

	if (val < 0) {
		shell_error(shell, "Invalid value: %s", argv[2]);
		return -EINVAL;
	}

	if (strcasecmp(subcmd, "gop") == 0) {
		shell_print(shell, "GOP command received with value: %d (0x%x)", val, val);
		ret = video_set_ctrl(video, VIDEO_CID_VENDOR_GOP, &val);
	} else if (strcasecmp(subcmd, "fps") == 0) {
		shell_print(shell, "FPS command received with value: %d (0x%x)", val, val);
		ret = video_set_ctrl(video, VIDEO_CID_VENDOR_FPS, &val);
	} else if (strcasecmp(subcmd, "ispfps") == 0) {
		shell_print(shell, "ISPFPS command received with value: %d (0x%x)", val, val);
		ret = video_set_ctrl(video, VIDEO_CID_VENDOR_ISPFPS, &val);
	} else if (strcasecmp(subcmd, "bps") == 0) {
		shell_print(shell, "BSP command received with value: %d (0x%x)", val, val);
		ret = video_set_ctrl(video, VIDEO_CID_VENDOR_BPS, &val);
	} else if (strcasecmp(subcmd, "forcei") == 0) {
		shell_print(shell, "forcei command received with value: %d (0x%x)", val, val);
		ret = video_set_ctrl(video, VIDEO_CID_VENDOR_FORCE_IFRAME, &val);
	} else {
		shell_error(shell, "Unknown cmd: %s", subcmd);
		return -EINVAL;
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_video, SHELL_CMD(start, NULL, "Start video", cmd_video_start),
			       SHELL_CMD(stop, NULL, "Stop video", cmd_video_stop),
			       SHELL_CMD(cmd, NULL, "Stop video", cmd_video_ctrl),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(video, &sub_video, "Video commands", NULL);

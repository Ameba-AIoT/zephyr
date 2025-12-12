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

#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <video_api.h>

#define VIDEO_CLOSE 0
#define VIDEO_OPEN  1

LOG_MODULE_REGISTER(video_capture, LOG_LEVEL_DBG);

#define MOUNT_POINT      "/SD:"
#define RECORD_FRAME_LEN 300

#define STACKSIZE   10240
#define PRIORITY    5
#define NUM_THREADS 2

#define VIDEO_BUF_SIZE (256 * 1024)

K_THREAD_STACK_ARRAY_DEFINE(thread_stacks, NUM_THREADS, STACKSIZE);

struct thread_context {
	struct k_thread thread_data;
	k_tid_t tid;
	struct k_sem my_sem;
	struct k_sem *other_sem;
	const char *name;
	k_thread_stack_t *stack_ptr;
	int video_status;
};

struct video_sample_context {
	struct thread_context vthread[NUM_THREADS];
	FATFS fat_fs;
	int sdcard_init;
	struct fs_file_t file;
	int video_index;
	uint32_t video_sd_buf_pos;
	struct fs_mount_t fat_mount;
	uint32_t video_record_channel;

	__aligned(32) uint8_t video_buffer[VIDEO_BUF_SIZE];
};

static struct video_sample_context ctxs;

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

int video_get_channel(const char *dev_name)
{
	int video_index = 0;

	if (sscanf(dev_name, "%*[^0-9]%d", &video_index) == 1) {
		LOG_INF("Video index %d", video_index);
	} else {
		return -EINVAL;
	}
	return video_index;
}

static int video_sd_card_init(void)
{
	int rc = 0;

	if (ctxs.sdcard_init) {
		goto EXIT;
	}

	ctxs.fat_mount.type = FS_FATFS;
	ctxs.fat_mount.mnt_point = MOUNT_POINT;
	ctxs.fat_mount.fs_data = &ctxs.fat_fs;
	ctxs.fat_mount.storage_dev = "SD";

	rc = disk_access_init(ctxs.fat_mount.storage_dev);
	if (rc != 0) {
		LOG_ERR("SD disk init failed (%d)", rc);
		goto EXIT;
	} else {
		LOG_INF("SD disk init (%d)", rc);
	}

	rc = fs_mount(&ctxs.fat_mount);
	if (rc < 0) {
		LOG_ERR("Failed to mount filesystem (%d)", rc);
		disk_access_ioctl(ctxs.fat_mount.storage_dev, DISK_IOCTL_CTRL_DEINIT, NULL);
		goto EXIT;
	}
	fs_file_t_init(&ctxs.file);
	ctxs.sdcard_init = 1;
	LOG_INF("Filesystem mounted at %s", MOUNT_POINT);
EXIT:
	return rc;
}

static void video_sd_write(struct fs_file_t *file, char *buf, uint32_t len)
{
	int offset = 0;

	while (len > 0) {
		if (ctxs.video_sd_buf_pos + len >= VIDEO_BUF_SIZE) {
			memcpy(ctxs.video_buffer + ctxs.video_sd_buf_pos, buf + offset,
			       VIDEO_BUF_SIZE - ctxs.video_sd_buf_pos);
			fs_write(file, ctxs.video_buffer, VIDEO_BUF_SIZE);
			offset += VIDEO_BUF_SIZE - ctxs.video_sd_buf_pos;
			len -= VIDEO_BUF_SIZE - ctxs.video_sd_buf_pos;
			ctxs.video_sd_buf_pos = 0;
		} else {
			memcpy(ctxs.video_buffer + ctxs.video_sd_buf_pos, buf + offset, len);
			ctxs.video_sd_buf_pos = ctxs.video_sd_buf_pos + len;
			len = 0;
		}
	}
}

static void video_sd_flush_buf(struct fs_file_t *file)
{
	if (ctxs.video_sd_buf_pos != 0) {
		fs_write(file, ctxs.video_buffer, ctxs.video_sd_buf_pos);
		ctxs.video_sd_buf_pos = 0;
	}
}

static void video_task(void *param, void *param1, void *param2)
{
	struct video_buffer buffers[16], *vbuf;
	struct video_format fmt;
	struct video_caps caps;
	unsigned int frame = 0;
	int i = 0;
	struct thread_context *ctx = (struct thread_context *)param;
	const char *video_dev_name = (const char *)ctx->name;
	const struct device *video = NULL;
	char video_record_filename[32] = {0};
	int rc = 0;
	int video_channel = 0;
	(void)param1;
	(void)param2;

	video = device_get_binding(video_dev_name);
	if (video == NULL) {
		LOG_ERR("Video device %s not found", video_dev_name);
		goto EXIT;
	}

	if (video_get_caps(video, &caps)) {
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

	if (video_get_format(video, &fmt)) {
		LOG_ERR("Unable to retrieve video format");
		goto EXIT;
	}

	if (video_set_format(video, &fmt)) {
		LOG_ERR("Unable to retrieve video format");
		return;
	}

	LOG_INF("- Default format: %c%c%c%c %ux%u\n", (char)fmt.pixelformat,
		(char)(fmt.pixelformat >> 8), (char)(fmt.pixelformat >> 16),
		(char)(fmt.pixelformat >> 24), fmt.width, fmt.height);

	memset(buffers, 0, sizeof(buffers));
	for (i = 0; i < ARRAY_SIZE(buffers); i++) {
		video_enqueue(video, &buffers[i]);
	}

	video_channel = video_get_channel(ctx->name);

	if (video_stream_start(video, VIDEO_BUF_TYPE_OUTPUT)) {
		LOG_ERR("Unable to start capture (interface)");
		goto EXIT;
	}

	LOG_INF("Capture started\n");

	ctx->video_status = VIDEO_OPEN;

	if (ctxs.sdcard_init && (strcmp(video_dev_name, "video_0") == 0)) {

		if (fmt.pixelformat == VIDEO_PIX_FMT_H264) {
			snprintf(video_record_filename, sizeof(video_record_filename),
				 "/SD:/video_%d.h264", ctxs.video_index);
			LOG_INF("%s", video_record_filename);
			rc = fs_open(&ctxs.file, video_record_filename, FS_O_CREATE | FS_O_WRITE);
		} else if (fmt.pixelformat == VIDEO_PIX_FMT_H265) {
			snprintf(video_record_filename, sizeof(video_record_filename),
				 "/SD:/video_%d.h265", ctxs.video_index);
			LOG_INF("%s", video_record_filename);
			rc = fs_open(&ctxs.file, video_record_filename, FS_O_CREATE | FS_O_WRITE);
			ctxs.video_index++;
		}
		if (rc < 0) {
			LOG_ERR("Failed to open file (%d)", rc);
		}
	}

	while (1) {
		int err;

		err = video_dequeue(video, &vbuf, K_MSEC(500));
		if (err) {
			LOG_ERR("Unable to dequeue video buf");
			goto EXIT;
		}
		frame++;
		if (frame % 30 == 0) {
			if (!ctxs.sdcard_init || (ctxs.video_record_channel != video_channel)) {
				LOG_INF("\rGot frame %u! size: %u; timestamp %u ms", frame,
					vbuf->size, vbuf->timestamp);
				mem_dump(vbuf->buffer, 16);
			}
		}
		if (frame < RECORD_FRAME_LEN) {
			if (ctxs.sdcard_init && (ctxs.video_record_channel == video_channel)) {
				video_sd_write(&ctxs.file, vbuf->buffer, vbuf->size);
			}
		} else if (frame == RECORD_FRAME_LEN) {
			if (ctxs.sdcard_init && (ctxs.video_record_channel == video_channel)) {
				video_sd_write(&ctxs.file, vbuf->buffer, vbuf->size);
				video_sd_flush_buf(&ctxs.file);
				fs_close(&ctxs.file);
				fs_unmount(&ctxs.fat_mount);
				disk_access_ioctl(ctxs.fat_mount.storage_dev,
						  DISK_IOCTL_CTRL_DEINIT, NULL);
				ctxs.sdcard_init = 0;
				LOG_INF("Record finish\r\n");
			}
		}

		err = video_enqueue(video, vbuf);
		if (err) {
			LOG_ERR("Unable to requeue video buf");
			goto EXIT;
		}
	}
EXIT:
	LOG_INF("The video task is closed\r\n");
	ctx->video_status = VIDEO_CLOSE;
}

static void video_thread_init(const char *video_dev_name)
{
	int video_index = 0;

	if (strcasecmp(video_dev_name, "video_0") == 0) {
		video_index = 0;
	} else if (strcasecmp(video_dev_name, "video_1") == 0) {
		video_index = 1;
	}

	ctxs.vthread[video_index].tid = k_thread_create(
		&ctxs.vthread[video_index].thread_data, thread_stacks[video_index], STACKSIZE,
		video_task, &ctxs.vthread[video_index], NULL, NULL, PRIORITY, 0, K_NO_WAIT);
}

int main(void)
{
	LOG_INF("amebapro2 video example\r\n");
	LOG_INF("Enter the video start video_0/video_1 to run the video\r\n");
	LOG_INF("Enter the video stop video_0/video_1 to stop the video\r\n");

	for (int i = 0; i < NUM_THREADS; i++) {
		ctxs.vthread[i].name = (i == 0) ? "video_0" : "video_1";
	}

	return 0;
}

static bool is_valid_video_device(const char *dev_name)
{
	return (strcmp(dev_name, "video_0") == 0) || (strcmp(dev_name, "video_1") == 0);
}

static int cmd_video_start(const struct shell *shell, size_t argc, char **argv)
{
	const char *dev_name = argv[1];
	int video_index = 0;

	if (argc != 2) {
		shell_print(shell, "Usage: video start <device_name>, device name should be "
				   "video_0 or video_1");
		return -EINVAL;
	}
	LOG_INF("dev_name %s", dev_name);

	if (!is_valid_video_device(dev_name)) {
		shell_error(shell, "Invalid device name '%s'. Allowed: video_0, video_1", dev_name);
		return -EINVAL;
	}

	video_index = video_get_channel(dev_name);

	if (ctxs.vthread[video_index].video_status == VIDEO_CLOSE) {
		video_thread_init(dev_name);
		shell_print(shell, "Video started");
	} else {
		shell_print(shell, "Video is open status\r\n");
	}
	return 0;
}

static int cmd_video_stop(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *video = NULL;
	const char *dev_name = NULL;
	int video_index = 0;

	if (argc != 2) {
		shell_print(shell, "Usage: video start <device_name> video_0~video_1");
		return -EINVAL;
	}

	dev_name = argv[1];

	if (!is_valid_video_device(dev_name)) {
		shell_error(shell, "Invalid device name '%s'. Allowed: video_0, video_1", dev_name);
		return -EINVAL;
	}

	video_index = video_get_channel(dev_name);

	LOG_INF("Video index %d", video_index);

	video = device_get_binding(dev_name);

	if (video == NULL) {
		LOG_ERR("Video device %s not found", dev_name);
		return 0;
	}

	if (ctxs.vthread[video_index].video_status == VIDEO_OPEN) {
		video_stream_stop(video, VIDEO_BUF_TYPE_OUTPUT);
		shell_print(shell, "Video stopped");
	} else {
		shell_print(shell, "Video is close status\r\n");
	}
	return 0;
}

static int cmd_video_ctrl(const struct shell *shell, size_t argc, char **argv)
{
	int ret = 0;
	const struct device *video = NULL;
	const char *subcmd = argv[1];
	int val = strtol(argv[2], NULL, 0);
	const char *dev_name = NULL;
	int video_index = 0;
	struct video_control control;

	if (argc != 4) {
		shell_error(shell, "Usage: video cmd <operatrion> <value> video_0/video_1");
		return -EINVAL;
	}

	subcmd = argv[1];
	val = strtol(argv[2], NULL, 0);
	dev_name = argv[3];

	if (!is_valid_video_device(dev_name)) {
		shell_error(shell, "Invalid device name '%s'. Allowed: video_0, video_1", dev_name);
		return -EINVAL;
	}

	video_index = video_get_channel(dev_name);

	video = device_get_binding(dev_name);

	if (video == NULL) {
		LOG_ERR("Video device %s not found", dev_name);
		return 0;
	}

	if (ctxs.vthread[video_index].video_status != VIDEO_OPEN) {
		shell_print(shell, "The video is not open\r\n");
		return -EINVAL;
	}

	if (val < 0) {
		shell_error(shell, "Invalid value: %s", argv[2]);
		return -EINVAL;
	}

	if (strcasecmp(subcmd, "gop") == 0) {
		shell_print(shell, "GOP command received with value: %d (0x%x)", val, val);
		control.id = VIDEO_CID_VENDOR_GOP;
		control.val = val;
		ret = video_set_ctrl(video, &control);
	} else if (strcasecmp(subcmd, "fps") == 0) {
		control.id = VIDEO_CID_VENDOR_FPS;
		control.val = val;
		shell_print(shell, "FPS command received with value: %d (0x%x)", val, val);
		ret = video_set_ctrl(video, &control);
	} else if (strcasecmp(subcmd, "ispfps") == 0) {
		control.id = VIDEO_CID_VENDOR_ISPFPS;
		control.val = val;
		shell_print(shell, "ISPFPS command received with value: %d (0x%x)", val, val);
		ret = video_set_ctrl(video, &control);
	} else if (strcasecmp(subcmd, "bps") == 0) {
		control.id = VIDEO_CID_VENDOR_BPS;
		control.val = val;
		shell_print(shell, "BSP command received with value: %d (0x%x)", val, val);
		ret = video_set_ctrl(video, &control);
	} else if (strcasecmp(subcmd, "forcei") == 0) {
		control.id = VIDEO_CID_VENDOR_FORCE_IFRAME;
		control.val = val;
		shell_print(shell, "forcei command received with value: %d (0x%x)", val, val);
		ret = video_set_ctrl(video, &control);
	} else {
		shell_error(shell, "Unknown cmd: %s", subcmd);
		return -EINVAL;
	}
	return 0;
}

static int cmd_video_record_init(const struct shell *shell, size_t argc, char **argv)
{
	video_sd_card_init();
	shell_print(shell, "Enable the sdcard");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_video, SHELL_CMD(start, NULL, "Start video", cmd_video_start),
			       SHELL_CMD(stop, NULL, "Stop video", cmd_video_stop),
			       SHELL_CMD(cmd, NULL, "Video cmd", cmd_video_ctrl),
			       SHELL_CMD(record, NULL, "Record video init", cmd_video_record_init),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(video, &sub_video, "Video commands", NULL);

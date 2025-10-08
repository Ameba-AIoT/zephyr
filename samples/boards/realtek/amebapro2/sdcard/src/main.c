/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include "os_wrapper.h"
#include <ff.h>
#include <zephyr/shell/shell.h>
#include <strings.h>

LOG_MODULE_REGISTER(main);

#define MOUNT_POINT "/SD:"

#define FATFS_IDLE  0
#define FATFS_START 1
static int fatfs_status = FATFS_IDLE;

static FATFS fat_fs;
static struct fs_mount_t fat_mount = {
	.type = FS_FATFS,
	.mnt_point = MOUNT_POINT,
	.fs_data = &fat_fs,
	.storage_dev = "SD",
};

void sd_task(void *param)
{
	int rc;
	struct fs_file_t file;
	const char *text = "Hello Zephyr on SD Card!\n";

	fatfs_status = FATFS_START;
	LOG_INF("Mounting FAT filesystem...");

	rc = disk_access_init(fat_mount.storage_dev);
	if (rc != 0) {
		LOG_ERR("SD disk init failed (%d)", rc);
		goto EXIT;
	} else {
		LOG_INF("SD disk init (%d)", rc);
	}

	rc = fs_mount(&fat_mount);
	if (rc < 0) {
		LOG_ERR("Failed to mount filesystem (%d)", rc);
		disk_access_ioctl(fat_mount.storage_dev, DISK_IOCTL_CTRL_DEINIT, NULL);
		goto EXIT;
	}
	LOG_INF("Filesystem mounted at %s", MOUNT_POINT);

	fs_file_t_init(&file);
	rc = fs_open(&file, MOUNT_POINT "/test.txt", FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		LOG_ERR("Failed to open file (%d)", rc);
	}

	ssize_t written = fs_write(&file, text, strlen(text));

	if (written < 0) {
		LOG_ERR("Failed to write to file (%zd)", written);
	} else {
		LOG_INF("Wrote %zd bytes", written);
	}

	fs_close(&file);

	LOG_INF("File operation done");
	fs_unmount(&fat_mount);
EXIT:
	disk_access_ioctl(fat_mount.storage_dev, DISK_IOCTL_CTRL_DEINIT, NULL);
	LOG_INF("Filesystem unmounted");
	fatfs_status = FATFS_IDLE;
}

int main(void)
{
	rtos_task_create(NULL, "sd", sd_task, NULL, 16 * 1024, 3);
	return 0;
}

static int cmd_sdcard_start(const struct shell *shell, size_t argc, char **argv)
{
	if (fatfs_status == FATFS_IDLE) {
		rtos_task_create(NULL, "sd", sd_task, NULL, 16 * 1024, 3);
	} else {
		LOG_INF("Filesystem is still start\r\n");
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sdcard, SHELL_CMD(start, NULL, "Start sdcard", cmd_sdcard_start),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(sdcard, &sub_sdcard, "Sdcard commands", NULL);

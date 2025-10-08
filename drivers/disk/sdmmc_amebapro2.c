/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba SD Host
 */

#define DT_DRV_COMPAT realtek_amebapro2_sdmmc

#include <zephyr/devicetree.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <stdio.h>
#include "hal_sdhost.h"
#include "os_wrapper.h"
#include "hal_cache.h"
#include "hal.h"
LOG_MODULE_REGISTER(sdmmc_amebapro2, LOG_LEVEL_INF);

#define DEMO_Board_SD_POWER_PIN PIN_F16
#define SD_POWER_ENABLE         0
#define SD_POWER_DISABLE        1
#define _RND(sz, r)             ((((sz) + ((r) - 1)) / (r)) * (r))

static int amebapro2_sdmmc_init(struct disk_info *disk);
static int amebapro2_sdmmc_access_status(struct disk_info *disk);
static int amebapro2_sdmmc_access_read(struct disk_info *disk, uint8_t *data_buf,
				       uint32_t start_sector, uint32_t num_sector);
static int amebapro2_sdmmc_access_write(struct disk_info *disk, const uint8_t *data_buf,
					uint32_t start_sector, uint32_t num_sector);
static int amebapro2_sdmmc_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buff);

struct amebapro2_sdmmc_param {
	struct k_sem sd_sema;
	struct k_mutex sd_mutex;
	int status;
	hal_gpio_adapter_t sd_power_en_gpio;
	hal_sdhost_adapter_t test_sdioh;
	phal_sdhost_adapter_t psdioh_adapter;
};

static const struct disk_operations amebapro2_sdmmc_ops = {
	.init = amebapro2_sdmmc_init,
	.status = amebapro2_sdmmc_access_status,
	.read = amebapro2_sdmmc_access_read,
	.write = amebapro2_sdmmc_access_write,
	.ioctl = amebapro2_sdmmc_access_ioctl,
};

static struct disk_info amebapro2_sdmmc_info = {
	.name = DT_INST_PROP_OR(0, disk_name, "SD"),
	.ops = &amebapro2_sdmmc_ops,
};

static struct amebapro2_sdmmc_param sdmmc_parm;

#define SD_TIMEOUT 1000
void sdio_done_event(void *pdata)
{
	struct amebapro2_sdmmc_param *param = pdata;

	k_sem_give(&param->sd_sema);
}
void sdio_task_wait(void *pdata)
{
	struct amebapro2_sdmmc_param *param = pdata;

	if (k_sem_take(&param->sd_sema, K_MSEC(SD_TIMEOUT)) != 0) {
		LOG_WRN("Timeout for get SD irq\r\n");
	}
}

static void sdh_card_insert_callback(void *pdata)
{
	struct amebapro2_sdmmc_param *param = pdata;
	SDHOST_Type *psdioh = param->psdioh_adapter->base_addr;

	for (int i = 0; i < 50000; i++) {
		__asm__ volatile("nop");
	}

	if (psdioh->card_exist_b.sd_exist) {
		param->status = DISK_STATUS_UNINIT;
	} else {
		param->status = DISK_STATUS_NOMEDIA;
	}
}

static void sdh_card_remove_callback(void *pdata)
{
	struct amebapro2_sdmmc_param *param = pdata;
	SDHOST_Type *psdioh = param->psdioh_adapter->base_addr;

	for (int i = 0; i < 50000; i++) {
		__asm__ volatile("nop");
	}

	if (psdioh->card_exist_b.sd_exist) {
		param->status = DISK_STATUS_UNINIT;
	} else {
		param->status = DISK_STATUS_NOMEDIA;
	}
}

static void amebapro2_sdmmc_power(const struct device *dev, int enable)
{
	struct amebapro2_sdmmc_param *param = dev->data;

	hal_gpio_write(&param->sd_power_en_gpio, enable);
}

static void amebapro2_sdmmc_power_reset(const struct device *dev)
{
	amebapro2_sdmmc_power(dev, SD_POWER_DISABLE);
	k_msleep(100);
	amebapro2_sdmmc_power(dev, SD_POWER_ENABLE);
}

static int amebapro2_sdhost_init(const struct device *dev)
{
	struct amebapro2_sdmmc_param *param = dev->data;
	SDHOST_Type *psdioh = param->psdioh_adapter->base_addr;
	int ret = 0;

	if (psdioh->card_exist_b.sd_exist) {
		ret = hal_sdhost_init_card(param->psdioh_adapter);
		if (ret != HAL_OK) {
			LOG_ERR("failed to init sdcard (ErrorCode 0x%X)", ret);
			ret = -EIO;
		} else {
			param->status = DISK_STATUS_OK;
			LOG_INF("card success %d", ret);
		}
	} else {
		ret = -ENODEV;
	}
	return ret;
}

static int amebapro2_sdhost_deinit(const struct device *dev)
{
	int ret = 0;
	struct amebapro2_sdmmc_param *param = dev->data;

	amebapro2_sdmmc_power_reset(dev);
	k_msleep(10);
	hal_sdhost_deinit(param->psdioh_adapter);
	k_msleep(10);
	ret = hal_sdhost_init_host(param->psdioh_adapter);
	hal_sdhost_card_insert_hook(param->psdioh_adapter, sdh_card_insert_callback, param);
	hal_sdhost_card_remove_hook(param->psdioh_adapter, sdh_card_remove_callback, param);
	hal_sdhost_transfer_done_int_hook(param->psdioh_adapter, sdio_done_event, param);
	hal_sdhost_task_yield_hook(param->psdioh_adapter, sdio_task_wait, param);
	return ret;
}

static int disk_amebapro2_sdmmc_init(const struct device *dev)
{
	int ret = 0;
	struct amebapro2_sdmmc_param *param = dev->data;
	SDHOST_Type *psdioh = NULL;

	param->psdioh_adapter = &param->test_sdioh;
	psdioh = param->psdioh_adapter->base_addr;

	hal_gpio_init(&param->sd_power_en_gpio, DEMO_Board_SD_POWER_PIN);
	hal_gpio_set_dir(&param->sd_power_en_gpio, GPIO_OUT);
	amebapro2_sdmmc_power_reset(dev);
	k_sem_init(&param->sd_sema, 0, 1);
	k_mutex_init(&param->sd_mutex);
	ret = hal_sdhost_init_host(param->psdioh_adapter);

	if (psdioh->card_exist_b.sd_exist) {
		param->status = DISK_STATUS_UNINIT;
	} else {
		param->status = DISK_STATUS_NOMEDIA;
	}
	ret = disk_access_register(&amebapro2_sdmmc_info);
	amebapro2_sdmmc_info.dev = dev;
	hal_sdhost_card_insert_hook(param->psdioh_adapter, sdh_card_insert_callback, param);
	hal_sdhost_card_remove_hook(param->psdioh_adapter, sdh_card_remove_callback, param);
	hal_sdhost_transfer_done_int_hook(param->psdioh_adapter, sdio_done_event, param);
	hal_sdhost_task_yield_hook(param->psdioh_adapter, sdio_task_wait, param);
	return ret;
}

static int amebapro2_sdmmc_init(struct disk_info *disk)
{
	int ret = 0;
	const struct device *dev = disk->dev;
	struct amebapro2_sdmmc_param *param = dev->data;

	k_mutex_lock(&param->sd_mutex, K_FOREVER);
	if (param->status == DISK_STATUS_OK) {
		ret = 0;
	} else {
		ret = amebapro2_sdhost_init(dev);
	}
	k_mutex_unlock(&param->sd_mutex);
	return ret;
}

static int amebapro2_sdmmc_access_status(struct disk_info *disk)
{
	const struct device *dev = disk->dev;
	struct amebapro2_sdmmc_param *param = dev->data;

	return param->status;
}

static int amebapro2_sdmmc_access_read(struct disk_info *disk, uint8_t *data_buf,
				       uint32_t start_sector, uint32_t num_sector)
{
	int ret = 0;
	const struct device *dev = disk->dev;
	struct amebapro2_sdmmc_param *param = dev->data;
	SDHOST_Type *psdioh = param->psdioh_adapter->base_addr;

	k_mutex_lock(&param->sd_mutex, K_FOREVER);
	if (psdioh->card_exist_b.sd_exist == 0) {
		ret = -EIO;
		goto EXIT;
	}

	if (((uint32_t)data_buf) % 32 != 0) {
		u8 *buff_alignment = (u8 *)rtos_mem_malloc(num_sector * 512);

		if (buff_alignment == NULL) {
			LOG_ERR("Fail to malloc cache for SDIO host!!\r\n");
			ret = -EIO;
			goto EXIT;
		}
		if (psdioh->card_exist_b.sd_exist) {
			dcache_clean_by_addr((unsigned int *)buff_alignment, num_sector * 512);
			ret = hal_sdhost_read_data(param->psdioh_adapter, start_sector, num_sector,
						   buff_alignment);
		}
		if (ret) {
			if (psdioh->card_exist_b.sd_exist) {
				ret = hal_sdhost_read_data(param->psdioh_adapter, start_sector,
							   num_sector, buff_alignment);
			}
		}

		memcpy(data_buf, buff_alignment, num_sector * 512);
		if (buff_alignment) {
			rtos_mem_free(buff_alignment);
		}

	} else {
		dcache_clean_by_addr((unsigned int *)data_buf, num_sector * 512);
		if (psdioh->card_exist_b.sd_exist) {
			ret = hal_sdhost_read_data(param->psdioh_adapter, start_sector, num_sector,
						   data_buf);
		}
		if (ret) {
			if (psdioh->card_exist_b.sd_exist) {
				ret = hal_sdhost_read_data(param->psdioh_adapter, start_sector,
							   num_sector, data_buf);
			}
		}
	}

	if (ret != 0) {
		LOG_ERR("Read_ Error start_sector = %u num_sector = %u ret = %d\r\n", start_sector,
			num_sector, ret);
		ret = -EIO;
		;
	}
EXIT:
	k_mutex_unlock(&param->sd_mutex);
	return ret;
}

static int amebapro2_sdmmc_access_write(struct disk_info *disk, const uint8_t *data_buf,
					uint32_t start_sector, uint32_t num_sector)
{
	int ret = 0;
	const struct device *dev = disk->dev;
	struct amebapro2_sdmmc_param *param = dev->data;
	SDHOST_Type *psdioh = param->psdioh_adapter->base_addr;

	k_mutex_lock(&param->sd_mutex, K_FOREVER);
	if (num_sector > 65535) {
		LOG_ERR("num_sector overflow");
		ret = -EIO;
		goto EXIT;
	}

	if (param == NULL) {
		LOG_ERR("The param is not init\r\n");
		ret = -EIO;
		goto EXIT;
	}

	if (psdioh->card_exist_b.sd_exist == 0) {
		ret = -EIO;
		goto EXIT;
	}

	if (((u32)data_buf) % 32 != 0) {
		u8 *buff_create = (u8 *)rtos_mem_malloc(num_sector * 512 + 0x1f);
		u8 *buff_alignment = (u8 *)_RND((u32)buff_create, 32);

		memcpy(buff_alignment, data_buf, num_sector * 512);
		ret = hal_sdhost_write_data(param->psdioh_adapter, start_sector, num_sector,
					    (u8 *)buff_alignment);
		if (ret != HAL_OK) {
			if (psdioh->card_exist_b.sd_exist) {
				ret = hal_sdhost_write_data(param->psdioh_adapter, start_sector,
							    num_sector, (u8 *)buff_alignment);
				LOG_ERR("sdio_sd_write_combine again ret = %d %d\n", ret,
					psdioh->card_exist_b.sd_exist);
				if (ret == HAL_OK) {
					LOG_INF("retry write success!!!!!!\n");
				} else {
					LOG_ERR("Write Error start_sector = %u num_sector = %u ret "
						"= %d\r\n",
						start_sector, num_sector, ret);
					rtos_mem_free(buff_create);
					ret = -EIO;
					goto EXIT;
				}
			} else {
				rtos_mem_free(buff_create);
				ret = -EIO;
				goto EXIT;
			}
		}
		rtos_mem_free(buff_create);

	} else {
		ret = hal_sdhost_write_data(param->psdioh_adapter, start_sector, num_sector,
					    (u8 *)data_buf);
		if (ret != HAL_OK) {
			if (psdioh->card_exist_b.sd_exist) {
				ret = hal_sdhost_write_data(param->psdioh_adapter, start_sector,
							    (num_sector), (u8 *)data_buf);
				if (ret == HAL_OK) {
					LOG_INF("retry write success!!!!!!\n");
				} else {
					LOG_ERR("Write Error start_sector = %u num_sector = %u ret "
						"= %d\r\n",
						start_sector, num_sector, ret);
					ret = -EIO;
					goto EXIT;
				}
			}
		}
	}

	if (ret != 0) {
		LOG_ERR("Write Error start_sector = %u num_sector = %u ret = %d\r\n", start_sector,
			num_sector, ret);
		ret = -EIO;
	}
EXIT:
	k_mutex_unlock(&param->sd_mutex);
	return ret;
}

static void amebapro2_sdmmc_deinit(const struct device *dev)
{
	struct amebapro2_sdmmc_param *param = dev->data;

	k_mutex_lock(&param->sd_mutex, K_FOREVER);
	if (param->status == DISK_STATUS_OK) {
		amebapro2_sdhost_deinit(dev);
	}
	k_mutex_unlock(&param->sd_mutex);
}

static int amebapro2_sdmmc_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buff)
{
	int ret = 0;
	const struct device *dev = disk->dev;
	struct amebapro2_sdmmc_param *param = dev->data;

	if (param == NULL) {
		LOG_ERR("The param is not init\r\n");
		ret = -EIO;
		return ret;
	}
	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		LOG_INF("sector number: %d\r\n", param->psdioh_adapter->blk_capacity);
		LOG_INF("capacity: %d MB\r\n",
			param->psdioh_adapter->blk_capacity / (1024 * 1024 / 512));
		*(uint32_t *)buff = param->psdioh_adapter->blk_capacity;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		LOG_DBG("DISK_IOCTL_GET_SECTOR_SIZE\r\n");
		*(uint32_t *)buff = 512;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		LOG_DBG("DISK_IOCTL_GET_ERASE_BLOCK_SZ\r\n");
		*(uint32_t *)buff = 1;
		break;
	case DISK_IOCTL_CTRL_SYNC:
		LOG_DBG("DISK_IOCTL_CTRL_SYNC\r\n");
		/* we use a blocking API, so nothing to do for sync */
		break;
	case DISK_IOCTL_CTRL_INIT:
		LOG_DBG("DISK_IOCTL_CTRL_INIT\r\n");
		return amebapro2_sdmmc_init(disk);
	case DISK_IOCTL_CTRL_DEINIT:
		LOG_DBG("DISK_IOCTL_CTRL_DEINIT\r\n");
		param->status = DISK_STATUS_UNINIT;
		amebapro2_sdmmc_deinit(dev);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

DEVICE_DT_INST_DEFINE(0, disk_amebapro2_sdmmc_init, NULL, &sdmmc_parm, NULL, POST_KERNEL, 10, NULL);

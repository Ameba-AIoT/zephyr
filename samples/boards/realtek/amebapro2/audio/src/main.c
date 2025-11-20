/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <ff.h>

LOG_MODULE_REGISTER(dmic_sample);

/* WAV file header structure */
struct wav_header {
	/* RIFF Header */
	uint8_t riff[4];    /* "RIFF" */
	uint32_t file_size; /* File size - 8 */
	uint8_t wave[4];    /* "WAVE" */

	/* Format Chunk */
	uint8_t fmt[4];           /* "fmt " */
	uint32_t fmt_size;        /* Format chunk size (16 for PCM) */
	uint16_t audio_format;    /* 1 = PCM */
	uint16_t num_channels;    /* 1 = Mono, 2 = Stereo */
	uint32_t sample_rate;     /* Sampling rate */
	uint32_t byte_rate;       /* sample_rate * num_channels * bits_per_sample / 8 */
	uint16_t block_align;     /* num_channels * bits_per_sample / 8 */
	uint16_t bits_per_sample; /* Bits per sample */

	/* Data Chunk */
	uint8_t data[4];    /* "data" */
	uint32_t data_size; /* Size of audio data */
} __packed;

/* Audio configuration */
#define AUDIO_DEV_NODE   DT_NODELABEL(dmic)
#define SAMPLE_RATE      8000
#define SAMPLE_BIT_WIDTH 16

/* Hardware DMA page size: 640 bytes = 20ms @ 16KHz, 40ms @ 8KHz */
#define AUDIO_DMA_PAGE_SIZE 640
#define DURATION_MS         40
#define BLOCK_COUNT         (1000 / DURATION_MS)

/* Capture settings */
#define READ_TIMEOUT_MS 1000
#define MAX_RETRIES     5
#define MAX_RECORD_MS   30000

/* DMIC status */
#define AMEBAPRO2_DMIC_CLOSE 0
#define AMEBAPRO2_DMIC_OPEN  1

K_MEM_SLAB_DEFINE_STATIC(mem_slab, AUDIO_DMA_PAGE_SIZE, BLOCK_COUNT, 4);

static const struct device *dmic_dev = DEVICE_DT_GET(AUDIO_DEV_NODE);
static int dmic_status = AMEBAPRO2_DMIC_CLOSE;
static uint8_t configured_channels = 1;

static FATFS fat_fs;
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.mnt_point = "/SD:",
	.fs_data = &fat_fs,
	.storage_dev = "SD",
};
static int sd_file_count = 1;

static int do_dmic_init(uint8_t num_channels)
{
	int ret;

	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("%s is not ready", dmic_dev->name);
		return -ENODEV;
	}

	if (num_channels != 1 && num_channels != 2) {
		LOG_ERR("Invalid channel count: %d (must be 1=mono or 2=stereo)", num_channels);
		return -EINVAL;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab = &mem_slab,
	};

	struct dmic_cfg cfg = {
		.streams = &stream,
		.channel = {.req_num_streams = 1, .req_num_chan = num_channels},
	};

	cfg.streams[0].pcm_rate = SAMPLE_RATE;
	cfg.streams[0].block_size = AUDIO_DMA_PAGE_SIZE;

	LOG_INF("Configuring DMIC: %d channel(s) (%s), %d-bit, %d Hz", num_channels,
		num_channels == 1 ? "mono" : "stereo", SAMPLE_BIT_WIDTH, SAMPLE_RATE);

	ret = dmic_configure(dmic_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("dmic_configure() failed with %d", ret);
		dmic_status = AMEBAPRO2_DMIC_CLOSE;
		configured_channels = 0;
		return ret;
	}

	dmic_status = AMEBAPRO2_DMIC_OPEN;
	configured_channels = num_channels;
	return 0;
}

static void create_wav_header(struct wav_header *header, uint32_t data_size, uint16_t num_channels,
			      uint32_t sample_rate, uint16_t bits_per_sample)
{
	/* RIFF Header */
	memcpy(header->riff, "RIFF", 4);
	header->file_size = data_size + sizeof(struct wav_header) - 8;
	memcpy(header->wave, "WAVE", 4);

	/* Format Chunk */
	memcpy(header->fmt, "fmt ", 4);
	header->fmt_size = 16;    /* PCM */
	header->audio_format = 1; /* PCM */
	header->num_channels = num_channels;
	header->sample_rate = sample_rate;
	header->byte_rate = sample_rate * num_channels * bits_per_sample / 8;
	header->block_align = num_channels * bits_per_sample / 8;
	header->bits_per_sample = bits_per_sample;

	/* Data Chunk */
	memcpy(header->data, "data", 4);
	header->data_size = data_size;
}

static void print_audio_stats(const struct shell *shell, void *buffer, size_t size)
{
	int16_t *samples = (int16_t *)buffer;
	size_t num_samples = size / sizeof(int16_t);
	int64_t sum = 0;         /* Use 64-bit to avoid overflow */
	int64_t sum_squares = 0; /* For RMS calculation */
	int16_t min = INT16_MAX;
	int16_t max = INT16_MIN;

	for (size_t i = 0; i < num_samples; i++) {
		int16_t sample = samples[i];

		sum += sample;
		sum_squares += (int64_t)sample * sample;
		if (sample < min) {
			min = sample;
		}
		if (sample > max) {
			max = sample;
		}
	}

	int32_t avg = (int32_t)(sum / num_samples);
	double mean_square = (double)sum_squares / num_samples;
	uint32_t rms = (uint32_t)sqrt(mean_square);

	/* Calculate volume as percentage of max 16-bit range */
	uint32_t volume_percent = (rms * 100) / 32768;

	shell_print(shell, "Audio stats: rms=%u, vol=%u%%, avg=%d, min=%d, max=%d", rms,
		    volume_percent, avg, min, max);
}

static int do_dmic_start(const struct shell *shell, uint32_t duration_ms)
{
	int ret;
	void *buffer;
	size_t size;
	uint32_t blocks_to_capture;

	/* Calculate number of blocks needed for duration_ms */
	blocks_to_capture = (duration_ms + DURATION_MS - 1) / DURATION_MS;

	/* For stereo */
	if (configured_channels == 2) {
		blocks_to_capture *= 2;
	}

	LOG_INF("Starting DMIC capture for %u ms (%u blocks)...", duration_ms, blocks_to_capture);

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("dmic_trigger(START) failed with %d", ret);
		return ret;
	}

	for (uint32_t i = 0; i < blocks_to_capture; i++) {
		int retries = 0;

		do {
			ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT_MS);
			if (ret == -EAGAIN) {
				retries++;
				if (retries > MAX_RETRIES) {
					LOG_ERR("Too many retries, giving up");
					goto stop_dmic;
				}
				LOG_DBG("No data available (retry %d/%d), waiting...", retries,
					MAX_RETRIES);
				k_msleep(100);
			} else if (ret < 0) {
				LOG_ERR("dmic_read() failed with %d", ret);
				goto stop_dmic;
			}
		} while (ret == -EAGAIN);

		LOG_DBG("Block %u/%u: received %zu bytes at %p", i + 1, blocks_to_capture, size,
			buffer);
		if ((i + 1) % BLOCK_COUNT == 0) {
			print_audio_stats(shell, buffer, size);
		}

		k_mem_slab_free(&mem_slab, buffer);
	}
	LOG_INF("Successfully captured %u blocks", blocks_to_capture);

stop_dmic:
	if (dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP) < 0) {
		LOG_ERR("dmic_trigger(STOP) failed");
	} else {
		LOG_INF("DMIC stopped");
	}
	return ret;
}

static int do_dmic_store_to_sd(const struct shell *shell, uint32_t duration_ms)
{
	int ret;
	void *buffer;
	size_t size;
	struct fs_file_t file;
	uint32_t blocks_to_capture;
	uint32_t total_bytes = 0;
	char *file_name = NULL;
	uint8_t *capture_buffer = NULL;
	uint32_t capture_buffer_size = 0;
	uint32_t buffer_idx = 0;

	/* Calculate number of blocks needed for duration_ms */
	blocks_to_capture = (duration_ms + DURATION_MS - 1) / DURATION_MS;

	/* For stereo */
	if (configured_channels == 2) {
		blocks_to_capture *= 2;
	}

	/* Allocate large buffer to hold entire capture in RAM */
	capture_buffer_size = blocks_to_capture * AUDIO_DMA_PAGE_SIZE;
	capture_buffer = k_malloc(capture_buffer_size);
	if (!capture_buffer) {
		LOG_ERR("Failed to allocate capture buffer (%u bytes)", capture_buffer_size);
		return -ENOMEM;
	}
	shell_print(shell, "Allocated %u bytes for capture buffer", capture_buffer_size);

	/* Start DMIC */
	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("dmic_trigger(START) failed with %d", ret);
		k_free(capture_buffer);
		return ret;
	}

	shell_print(shell, "Starting DMIC capture for %u ms (%u blocks)...", duration_ms,
		    blocks_to_capture);

	for (uint32_t i = 0; i < blocks_to_capture; i++) {
		int retries = 0;

		do {
			ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT_MS);
			if (ret == -EAGAIN) {
				retries++;
				if (retries > MAX_RETRIES) {
					LOG_ERR("Too many retries, giving up");
					goto cleanup;
				}
				LOG_DBG("No data available (retry %d/%d), waiting...", retries,
					MAX_RETRIES);
				k_msleep(100);
			} else if (ret < 0) {
				LOG_ERR("dmic_read() failed with %d", ret);
				goto cleanup;
			}
		} while (ret == -EAGAIN);

		/* Copy audio data to capture buffer */
		memcpy(capture_buffer + buffer_idx, buffer, size);
		buffer_idx += size;

		/* Free the buffer back to the memory slab */
		k_mem_slab_free(&mem_slab, buffer);

		if ((i + 1) % BLOCK_COUNT == 0) {
			shell_print(shell, "Captured %u/%u blocks (%u bytes)", i + 1,
				    blocks_to_capture, buffer_idx);
		}
	}

	/* Stop DMIC */
	if (dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP) < 0) {
		LOG_ERR("dmic_trigger(STOP) failed");
	} else {
		LOG_DBG("DMIC stopped");
	}

	shell_print(shell, "Capture complete! Now writing %u bytes to SD card...", buffer_idx);

	/* Now mount SD card and write all data at once */
	ret = disk_access_init(mp.storage_dev);
	if (ret != 0) {
		LOG_ERR("SD disk init failed (%d)", ret);
		goto cleanup;
	}

	ret = fs_mount(&mp);
	if (ret < 0) {
		LOG_ERR("Failed to mount SD card: %d", ret);
		LOG_ERR("Please ensure SD card is inserted and formatted as FAT32");
		goto cleanup;
	}

	struct wav_header wav_hdr;

	create_wav_header(&wav_hdr, buffer_idx, configured_channels, SAMPLE_RATE, SAMPLE_BIT_WIDTH);

	/* Open file for writing */
	file_name = k_malloc(32);
	if (!file_name) {
		LOG_ERR("Failed to allocate filename buffer");
		goto cleanup;
	}
	snprintf(file_name, 32, "/SD:/audio_%s_%d_%d_%d.wav",
		 configured_channels == 1 ? "mono" : "stereo", SAMPLE_RATE, SAMPLE_BIT_WIDTH,
		 sd_file_count++);
	fs_file_t_init(&file);
	ret = fs_open(&file, file_name, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (ret < 0) {
		LOG_ERR("Failed to open file: %d", ret);
		goto cleanup;
	}

	/* Write WAV header first */
	ssize_t written = fs_write(&file, &wav_hdr, sizeof(struct wav_header));

	if (written != sizeof(struct wav_header)) {
		LOG_ERR("Failed to write WAV header: %d", written);
		goto cleanup;
	}
	/* Write audio data */
	written = fs_write(&file, capture_buffer, buffer_idx);
	if (written < 0) {
		LOG_ERR("Failed to write audio data to SD card: %d", written);
		goto cleanup;
	}

	double record_duration =
		(double)buffer_idx / (SAMPLE_RATE * configured_channels * SAMPLE_BIT_WIDTH / 8);

	total_bytes = sizeof(struct wav_header) + written;
	shell_print(shell, "Successfully wrote %u bytes (header + data) to %s", total_bytes,
		    file_name);
	shell_print(shell, "WAV file info: %d Hz, %d-bit, %s, duration: %.1f sec", SAMPLE_RATE,
		    SAMPLE_BIT_WIDTH, configured_channels == 1 ? "mono" : "stereo",
		    record_duration);

cleanup:
	/* Close file and unmount */
	disk_access_ioctl(mp.storage_dev, DISK_IOCTL_CTRL_DEINIT, NULL);
	fs_close(&file);
	fs_unmount(&mp);

	/* Free allocated buffers */
	if (file_name) {
		k_free(file_name);
	}
	if (capture_buffer) {
		k_free(capture_buffer);
	}
	return (ret < 0) ? ret : 0;
}

int main(void)
{
	LOG_INF("===========================================");
	LOG_INF("AmebaPro2 DMIC Example");
	LOG_INF("Commands:");
	LOG_INF("  dmic init [<channels>] - Initialize DMIC (1=mono, 2=stereo, default=1)");
	LOG_INF("                           Example: dmic init 1");
	LOG_INF("  dmic start [<ms>]      - Capture audio (default %d ms)",
		DURATION_MS * BLOCK_COUNT);
	LOG_INF("                           Example: dmic start 5000");
	LOG_INF("  dmic store_to_sd [<ms>] - Record audio to SD card (default 4000 ms)");
	LOG_INF("                           Example: dmic store_to_sd 10000");
	LOG_INF("===========================================");
	return 0;
}

static int cmd_dmic_init(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t num_channels = 1; /* Default to mono */

	if (dmic_status == AMEBAPRO2_DMIC_OPEN) {
		shell_print(shell, "DMIC is already initialized");
		return 0;
	}

	/* If channel count parameter provided */
	if (argc >= 2) {
		num_channels = atoi(argv[1]);
		if (num_channels != 1 && num_channels != 2) {
			shell_error(shell, "Invalid channel count: %d (1=mono or 2=stereo)",
				    num_channels);
			return -1;
		}
	}

	if (do_dmic_init(num_channels) < 0) {
		shell_error(shell, "DMIC initialization failed");
		return -1;
	}

	shell_print(shell, "DMIC initialized successfully: %d channel(s) (%s)", num_channels,
		    num_channels == 1 ? "mono" : "stereo");
	return 0;
}

static int cmd_dmic_start(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t duration_ms;

	if (dmic_status == AMEBAPRO2_DMIC_CLOSE) {
		shell_error(shell, "DMIC not initialized. Run 'dmic init' first");
		return -1;
	}

	/* If duration parameter provided */
	if (argc >= 2) {
		duration_ms = atoi(argv[1]);
		if (duration_ms == 0 || duration_ms > MAX_RECORD_MS) {
			shell_error(shell, "Invalid duration. Must be 1-%d ms", MAX_RECORD_MS);
			return -1;
		}
	} else {
		duration_ms = DURATION_MS * BLOCK_COUNT;
	}

	shell_print(shell, "Capturing %u ms of audio...", duration_ms);

	if (do_dmic_start(shell, duration_ms) < 0) {
		shell_error(shell, "DMIC capture failed");
		return -1;
	}

	shell_print(shell, "DMIC capture completed");
	return 0;
}

static int cmd_dmic_store_to_sd(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t duration_ms;

	if (dmic_status == AMEBAPRO2_DMIC_CLOSE) {
		shell_error(shell, "DMIC not initialized. Run 'dmic init' first");
		return -1;
	}

	/* If duration parameter provided */
	if (argc >= 2) {
		duration_ms = atoi(argv[1]);
		if (duration_ms == 0 || duration_ms > MAX_RECORD_MS) {
			shell_error(shell, "Invalid duration. Must be 1-%d ms", MAX_RECORD_MS);
			return -1;
		}
	} else {
		duration_ms = DURATION_MS * BLOCK_COUNT;
	}

	if (do_dmic_store_to_sd(shell, duration_ms) < 0) {
		shell_error(shell, "Recording failed");
		return -1;
	}

	shell_print(shell, "Recording completed successfully");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_dmic,
			       SHELL_CMD_ARG(init, NULL,
					     "Initialize DMIC [<channels>] (1=mono, 2=stereo)",
					     cmd_dmic_init, 1, 1),
			       SHELL_CMD_ARG(start, NULL, "Capture audio [<duration_ms>]",
					     cmd_dmic_start, 1, 1),
			       SHELL_CMD_ARG(store_to_sd, NULL, "Record to SD card [<duration_ms>]",
					     cmd_dmic_store_to_sd, 1, 1),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dmic, &sub_dmic, "DMIC test commands", NULL);

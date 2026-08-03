/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_mipi_dsi

/* Include <soc.h> before <ameba_soc.h> to avoid redefining the unlikely() macro */
#include <soc.h>
#include <ameba_soc.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/display/mipi_display.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dsi_ameba, CONFIG_MIPI_DSI_LOG_LEVEL);

/*
 * D-PHY HS escape timing (in lane byte clocks) used to size the high-speed
 * data-lane frequency.  Values mirror the validated AmebaSmart raw_mipi
 * reference (ameba_mipi_show.c); they cover the LP->HS entry/exit overhead the
 * link must absorb on top of the active pixel payload.
 */
#define AMEBA_DSI_T_LPX     5
#define AMEBA_DSI_T_HS_PREP 6
#define AMEBA_DSI_T_HS_ZERO 10
#define AMEBA_DSI_T_HS_TRAIL 8
#define AMEBA_DSI_T_HS_EXIT 7
/* Minimum read-trigger interval guard used by the line-length sanity check. */
#define AMEBA_DSI_RTNI      2

/* 1 MHz, matches the SDK "Mhz" scaling constant. */
#define AMEBA_DSI_MHZ       1000000UL

/* Command-mode TX-done poll budget (microseconds). */
#define AMEBA_DSI_CMD_TIMEOUT_US 20000
/* Command-mode RX (BTA read) poll budget (microseconds). */
#define AMEBA_DSI_RX_TIMEOUT_US  20000
/* LCDC upstream "ready" handshake budget before switching to video (us). */
#define AMEBA_DSI_LCDC_READY_TIMEOUT_US 20000

/* Long-packet payload staging buffer (one DCS/GENERIC transaction). */
#define AMEBA_DSI_MAX_PAYLOAD 128

struct mipi_dsi_ameba_config {
	MIPI_TypeDef *base;
	/*
	 * Upstream video source register base; the host uses it to query the
	 * LCDC scan-out "ready" state and re-sync the datapath before switching
	 * to video.  NULL when the lcdc node is absent or disabled.
	 */
	LCDC_TypeDef *lcdc;
	uint16_t frame_rate;
};

struct mipi_dsi_ameba_data {
	MIPI_InitTypeDef init;
	bool attached;
	bool video_on;
};

/*
 * Translate the panel-supplied mipi_dsi_device descriptor into the HAL
 * MIPI_InitTypeDef.  Mirrors MIPI_InitStruct_Config() from the SDK reference:
 * the LP timings (HSA/HBP/HFP) are expressed in bytes (pixels * bpp / 8) while
 * the vertical timings stay in lines, and the HS data-lane frequency is sized
 * from the full per-line bit budget plus the D-PHY escape overhead.
 */
static void mipi_dsi_ameba_fill_init(struct mipi_dsi_ameba_data *data,
				     const struct mipi_dsi_device *mdev,
				     uint16_t frame_rate)
{
	MIPI_InitTypeDef *init = &data->init;
	uint32_t bpp, vtotal, htotal_bits, overhead_bits, total_bits;

	MIPI_StructInit(init);

	switch (mdev->pixfmt) {
	case MIPI_DSI_PIXFMT_RGB565:
		init->MIPI_VideoDataFormat = MIPI_VIDEO_DATA_FORMAT_RGB565;
		bpp = 16U;
		break;
	case MIPI_DSI_PIXFMT_RGB666_PACKED:
		init->MIPI_VideoDataFormat = MIPI_VIDEO_DATA_FORMAT_RGB666_PACKED;
		bpp = 18U;
		break;
	case MIPI_DSI_PIXFMT_RGB666:
		init->MIPI_VideoDataFormat = MIPI_VIDEO_DATA_FORMAT_RGB666_LOOSELY;
		bpp = 24U;
		break;
	case MIPI_DSI_PIXFMT_RGB888:
	default:
		init->MIPI_VideoDataFormat = MIPI_VIDEO_DATA_FORMAT_RGB888;
		bpp = 24U;
		break;
	}

	/* Burst hint from the panel selects the video transmission mode. */
	if (mdev->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
		init->MIPI_VideoModeInterface = MIPI_VIDEO_BURST_MODE;
	} else {
		init->MIPI_VideoModeInterface = MIPI_VIDEO_NON_BURST_MODE_WITH_SYNC_PULSES;
	}

	init->MIPI_LaneNum = mdev->data_lanes;
	init->MIPI_FrameRate = frame_rate;

	init->MIPI_HSA = mdev->timings.hsync * bpp / 8U;
	if (init->MIPI_VideoModeInterface == MIPI_VIDEO_NON_BURST_MODE_WITH_SYNC_PULSES) {
		init->MIPI_HBP = mdev->timings.hbp * bpp / 8U;
	} else {
		init->MIPI_HBP = (mdev->timings.hsync + mdev->timings.hbp) * bpp / 8U;
	}
	init->MIPI_HACT = mdev->timings.hactive;
	init->MIPI_HFP = mdev->timings.hfp * bpp / 8U;

	init->MIPI_VSA = mdev->timings.vsync;
	init->MIPI_VBP = mdev->timings.vbp;
	init->MIPI_VACT = mdev->timings.vactive;
	init->MIPI_VFP = mdev->timings.vfp;

	/* DataLaneFreq * LaneNum = FrameRate * vtotal * (htotal_bits + overhead). */
	vtotal = init->MIPI_VSA + init->MIPI_VBP + init->MIPI_VACT + init->MIPI_VFP;
	htotal_bits = (mdev->timings.hsync + mdev->timings.hbp + init->MIPI_HACT +
		       mdev->timings.hfp) * bpp;
	overhead_bits = (AMEBA_DSI_T_LPX + AMEBA_DSI_T_HS_PREP + AMEBA_DSI_T_HS_ZERO +
			 AMEBA_DSI_T_HS_TRAIL + AMEBA_DSI_T_HS_EXIT) * init->MIPI_LaneNum * 8U;
	total_bits = htotal_bits + overhead_bits;

	init->MIPI_VideDataLaneFreq =
		(uint32_t)(((uint64_t)init->MIPI_FrameRate * total_bits * vtotal) /
			   init->MIPI_LaneNum / AMEBA_DSI_MHZ) + 20U;

	init->MIPI_LineTime = (init->MIPI_VideDataLaneFreq * AMEBA_DSI_MHZ) / 8U /
			      init->MIPI_FrameRate / vtotal;
	init->MIPI_BllpLen = init->MIPI_LineTime / 2U;

	/*
	 * Line-length / line-time sanity guards, mirroring the SDK lcdc_mipi
	 * reference.  A line shorter than the controller's minimum read-trigger
	 * window, or a LineTime too short to clock one line out over the
	 * available lanes, will not display correctly -- flag it during bring-up
	 * rather than letting the panel come up blank.
	 */
	if ((mdev->timings.hsync + mdev->timings.hbp + init->MIPI_HACT +
	     mdev->timings.hfp) < (512U + AMEBA_DSI_RTNI * 16U)) {
		LOG_ERR("line too short (%u px); panel timing unsupported",
			mdev->timings.hsync + mdev->timings.hbp + init->MIPI_HACT +
			mdev->timings.hfp);
	}
	if (init->MIPI_LineTime * init->MIPI_LaneNum < total_bits / 8U) {
		LOG_ERR("LineTime %u too short for a %u-bit line on %u lanes",
			init->MIPI_LineTime, total_bits, init->MIPI_LaneNum);
	}

	LOG_INF("DataLaneFreq: %u Mbps, LineTime: %u", init->MIPI_VideDataLaneFreq,
		init->MIPI_LineTime);
}

/* Poll the raw interrupt status for command-mode TX completion. */
static int mipi_dsi_ameba_wait_txdone(MIPI_TypeDef *base)
{
	uint32_t elapsed = 0U;

	while (!(MIPI_DSI_INTS_Get(base) & MIPI_BIT_CMD_TXDONE)) {
		if (elapsed >= AMEBA_DSI_CMD_TIMEOUT_US) {
			LOG_ERR("command TX timed out (INTS=0x%08x)", MIPI_DSI_INTS_Get(base));
			return -ETIMEDOUT;
		}
		k_busy_wait(10);
		elapsed += 10U;
	}

	MIPI_DSI_INTS_Clr(base, MIPI_BIT_CMD_TXDONE);
	return 0;
}

/* Send a long-packet payload through the IDMA word FIFO, then fire the GO. */
static int mipi_dsi_ameba_send_long(MIPI_TypeDef *base, uint8_t data_id,
				    const uint8_t *payload, size_t len)
{
	uint32_t word0, word1, addr;

	if (len > AMEBA_DSI_MAX_PAYLOAD) {
		return -ENOSPC;
	}

	for (addr = 0U; addr < (len + 7U) / 8U; addr++) {
		size_t idx = addr * 8U;

		word0 = ((idx + 3U < len) ? payload[idx + 3U] << 24 : 0U) |
			((idx + 2U < len) ? payload[idx + 2U] << 16 : 0U) |
			((idx + 1U < len) ? payload[idx + 1U] << 8 : 0U) |
			((idx < len) ? payload[idx] : 0U);
		word1 = ((idx + 7U < len) ? payload[idx + 7U] << 24 : 0U) |
			((idx + 6U < len) ? payload[idx + 6U] << 16 : 0U) |
			((idx + 5U < len) ? payload[idx + 5U] << 8 : 0U) |
			((idx + 4U < len) ? payload[idx + 4U] : 0U);

		MIPI_DSI_CMD_LongPkt_MemQWordRW(base, addr, &word0, &word1, FALSE);
	}

	/* Long-packet word count is 16-bit: low byte in Byte0, high byte in Byte1. */
	MIPI_DSI_CMD_Send(base, data_id, (uint8_t)(len & 0xFFU), (uint8_t)(len >> 8));

	return mipi_dsi_ameba_wait_txdone(base);
}

static ssize_t mipi_dsi_ameba_read(MIPI_TypeDef *base, const struct mipi_dsi_msg *msg)
{
	uint8_t *rx = msg->rx_buf;
	uint32_t elapsed = 0U;
	uint32_t rcmd, status;

	if (rx == NULL || msg->rx_len == 0U) {
		return -EINVAL;
	}

	/* Trigger the read request; BTA hands the link to the panel. */
	MIPI_DSI_CMD_Send(base, msg->type, msg->cmd, 0U);

	while (1) {
		status = MIPI_DSI_INTS_Get(base);
		if (status & (MIPI_BIT_RCMD1 | MIPI_BIT_RX_TRIGGER)) {
			break;
		}
		if (status & MIPI_BIT_LPRX_TIMEOUT) {
			MIPI_DSI_INTS_Clr(base, MIPI_BIT_LPRX_TIMEOUT);
			LOG_ERR("DCS read 0x%02x: LP-RX timeout", msg->cmd);
			return -EIO;
		}
		if (elapsed >= AMEBA_DSI_RX_TIMEOUT_US) {
			LOG_ERR("DCS read 0x%02x: no response (INTS=0x%08x)", msg->cmd, status);
			return -ETIMEDOUT;
		}
		k_busy_wait(10);
		elapsed += 10U;
	}

	rcmd = MIPI_DSI_CMD_Rxcv_CMD(base, 0U);
	MIPI_DSI_INTS_Clr(base, MIPI_BIT_RCMD1 | MIPI_BIT_RX_TRIGGER);

	/*
	 * Only DCS/generic SHORT read responses are handled here.  The response
	 * data type in the RCMD header tells us how many data bytes are valid:
	 *   0x21 -> 1 data byte, 0x22 -> 2 data bytes (DSI spec).
	 * Long read responses (0x1A/0x1C) carry an arbitrary-length payload via
	 * the long-packet path and are not supported.  Return the number of
	 * bytes actually written so callers see the true read length.
	 */
	switch (MIPI_GET_RCMDx_DATAID(rcmd) & 0x3FU) {
	case 0x21U: /* short read response, 1 byte */
		rx[0] = MIPI_GET_RCMDx_BYTE0(rcmd);
		return 1;
	case 0x22U: /* short read response, 2 bytes */
		rx[0] = MIPI_GET_RCMDx_BYTE0(rcmd);
		if (msg->rx_len > 1U) {
			rx[1] = MIPI_GET_RCMDx_BYTE1(rcmd);
			return 2;
		}
		return 1;
	case 0x02U: /* acknowledge and error report */
		LOG_ERR("DCS read 0x%02x: panel error report", msg->cmd);
		return -EIO;
	default:
		LOG_ERR("DCS read 0x%02x: unsupported response type 0x%02x", msg->cmd,
			(unsigned int)MIPI_GET_RCMDx_DATAID(rcmd));
		return -ENOTSUP;
	}
}

/* Promotes the link to video after the panel's display-on; defined below. */
static void mipi_dsi_ameba_enter_video(const struct device *dev);

static ssize_t mipi_dsi_ameba_transfer(const struct device *dev, uint8_t channel,
				       struct mipi_dsi_msg *msg)
{
	const struct mipi_dsi_ameba_config *cfg = dev->config;
	struct mipi_dsi_ameba_data *data = dev->data;
	MIPI_TypeDef *base = cfg->base;
	const uint8_t *tx = msg->tx_buf;
	uint8_t payload[AMEBA_DSI_MAX_PAYLOAD];
	int ret;

	ARG_UNUSED(channel);

	if (!data->attached) {
		return -EPERM;
	}

	switch (msg->type) {
	/* DCS short write, command byte only (e.g. display on/off, sleep out). */
	case MIPI_DSI_DCS_SHORT_WRITE:
		MIPI_DSI_CMD_Send(base, msg->type, msg->cmd, 0U);
		ret = mipi_dsi_ameba_wait_txdone(base);
		if (ret) {
			return ret;
		}
		/* The panel's final LP command is display-on; once it lands the
		 * panel is awake, so promote the link to video (NXP idiom: the
		 * upstream source starts streaming after the panel is ready).
		 */
		if (msg->cmd == MIPI_DCS_SET_DISPLAY_ON) {
			mipi_dsi_ameba_enter_video(dev);
		}
		return (ssize_t)msg->tx_len;

	/* DCS short write, command byte + one parameter (e.g. set address mode). */
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
		MIPI_DSI_CMD_Send(base, msg->type, msg->cmd, tx ? tx[0] : 0U);
		ret = mipi_dsi_ameba_wait_txdone(base);
		return ret ? ret : (ssize_t)msg->tx_len;

	/* Generic short write, 0/1/2 parameter bytes (vendor register pokes). */
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
		MIPI_DSI_CMD_Send(base, msg->type,
				  (msg->tx_len >= 1U) ? tx[0] : 0U,
				  (msg->tx_len >= 2U) ? tx[1] : 0U);
		ret = mipi_dsi_ameba_wait_txdone(base);
		return ret ? ret : (ssize_t)msg->tx_len;

	/* DCS long write: command byte prepended to the payload (gamma/GIP tables). */
	case MIPI_DSI_DCS_LONG_WRITE:
		if (msg->tx_len + 1U > AMEBA_DSI_MAX_PAYLOAD) {
			return -ENOSPC;
		}
		payload[0] = msg->cmd;
		memcpy(&payload[1], tx, msg->tx_len);
		ret = mipi_dsi_ameba_send_long(base, msg->type, payload, msg->tx_len + 1U);
		return ret ? ret : (ssize_t)msg->tx_len;

	/* Generic long write: raw payload, no command byte (bank-select 0xFF seq). */
	case MIPI_DSI_GENERIC_LONG_WRITE:
		ret = mipi_dsi_ameba_send_long(base, msg->type, tx, msg->tx_len);
		return ret ? ret : (ssize_t)msg->tx_len;

	/* DCS read: issue BTA and collect the panel's short response. */
	case MIPI_DSI_DCS_READ:
		return mipi_dsi_ameba_read(base, msg);

	default:
		LOG_ERR("unsupported MIPI-DSI message type 0x%02x", msg->type);
		return -ENOTSUP;
	}
}

/* Cycle the LCDC once against the freshly programmed vo-clock, then wait for
 * it to report ready before switching to video.  No-op when lcdc is NULL.
 */
static void mipi_dsi_ameba_sync_lcdc(LCDC_TypeDef *lcdc)
{
	uint32_t elapsed = 0U;

	if (lcdc == NULL) {
		return;
	}

	LCDC_Cmd(lcdc, DISABLE);
	LCDC_Cmd(lcdc, ENABLE);
	LCDC_TrigerSHWReload(lcdc);

	while (!LCDC_CheckLCDCReady(lcdc)) {
		if (elapsed >= AMEBA_DSI_LCDC_READY_TIMEOUT_US) {
			LOG_WRN("LCDC not ready before video; switching anyway");
			return;
		}
		k_busy_wait(10);
		elapsed += 10U;
	}
}

/* Switch LP → video, called on the panel's final SET_DISPLAY_ON. */
static void mipi_dsi_ameba_enter_video(const struct device *dev)
{
	const struct mipi_dsi_ameba_config *cfg = dev->config;
	struct mipi_dsi_ameba_data *data = dev->data;
	MIPI_TypeDef *base = cfg->base;

	if (data->video_on) {
		return;
	}

	mipi_dsi_ameba_sync_lcdc(cfg->lcdc);
	MIPI_DSI_INTS_ACPU_Clr(base, MIPI_DSI_INTS_ACPU_Get(base));
	MIPI_DSI_Mode_Switch(base, ENABLE);
	data->video_on = true;

	LOG_INF("MIPI-DSI video on (%ux%u)", data->init.MIPI_HACT, data->init.MIPI_VACT);
}

static int mipi_dsi_ameba_attach(const struct device *dev, uint8_t channel,
				 const struct mipi_dsi_device *mdev)
{
	const struct mipi_dsi_ameba_config *cfg = dev->config;
	struct mipi_dsi_ameba_data *data = dev->data;
	MIPI_TypeDef *base = cfg->base;

	ARG_UNUSED(channel);

	mipi_dsi_ameba_fill_init(data, mdev, cfg->frame_rate);

	/* MIPI_Init() sets vo-clock (MIPI_CLK_Set), DPHY, and DSI; link starts in LP mode. */
	MIPI_Init(base, &data->init);

	/* Bound LP escape timeouts so a stuck panel cannot hang command TX. */
	MIPI_DSI_TO1_Set(base, DISABLE, 0U);
	MIPI_DSI_TO2_Set(base, ENABLE, 0x7FFFFFFF);
	MIPI_DSI_TO3_Set(base, DISABLE, 0U);

	/* Command-mode TX/RX is driven by polling raw status, no IRQ needed. */
	MIPI_DSI_INT_Config(base, DISABLE, DISABLE, FALSE);

	data->attached = true;
	data->video_on = false;

	LOG_INF("MIPI-DSI host attached, command mode (%u lanes, %ux%u)", mdev->data_lanes,
		mdev->timings.hactive, mdev->timings.vactive);

	return 0;
}

static int mipi_dsi_ameba_detach(const struct device *dev, uint8_t channel,
				 const struct mipi_dsi_device *mdev)
{
	const struct mipi_dsi_ameba_config *cfg = dev->config;
	struct mipi_dsi_ameba_data *data = dev->data;

	ARG_UNUSED(channel);
	ARG_UNUSED(mdev);

	MIPI_DSI_Mode_Switch(cfg->base, DISABLE);
	data->attached = false;
	data->video_on = false;

	return 0;
}

static int mipi_dsi_ameba_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Clocks are owned by MIPI_Init() (vo-clock + gate) and by the LCDC's own
	 * clock_control request; nothing to bring up until a panel attaches.
	 */
	return 0;
}

static DEVICE_API(mipi_dsi, mipi_dsi_ameba_api) = {
	.attach = mipi_dsi_ameba_attach,
	.detach = mipi_dsi_ameba_detach,
	.transfer = mipi_dsi_ameba_transfer,
};

/*
 * LCDC base resolved via compatible string rather than a phandle on mipi-dsi:
 * a phandle back-reference would create a DT dependency cycle through the panel
 * child (st7701s → mipi-dsi → lcdc) that check_init_priorities rejects.
 */
#define MIPI_DSI_AMEBA_LCDC(n)                                                            \
	COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(realtek_ameba_lcdc_dsi),                   \
		    ((LCDC_TypeDef *)DT_REG_ADDR(                                          \
			    DT_COMPAT_GET_ANY_STATUS_OKAY(realtek_ameba_lcdc_dsi))),       \
		    (NULL))

#define MIPI_DSI_AMEBA_INIT(n)                                                                     \
	static const struct mipi_dsi_ameba_config mipi_dsi_ameba_config_##n = {                    \
		.base = (MIPI_TypeDef *)DT_INST_REG_ADDR(n),                                       \
		.lcdc = MIPI_DSI_AMEBA_LCDC(n),                                                    \
		.frame_rate = DT_INST_PROP(n, realtek_frame_rate),                                 \
	};                                                                                         \
	static struct mipi_dsi_ameba_data mipi_dsi_ameba_data_##n;                                 \
	DEVICE_DT_INST_DEFINE(n, &mipi_dsi_ameba_init, NULL, &mipi_dsi_ameba_data_##n,             \
			      &mipi_dsi_ameba_config_##n, POST_KERNEL,                             \
			      CONFIG_MIPI_DSI_INIT_PRIORITY, &mipi_dsi_ameba_api);

DT_INST_FOREACH_STATUS_OKAY(MIPI_DSI_AMEBA_INIT)

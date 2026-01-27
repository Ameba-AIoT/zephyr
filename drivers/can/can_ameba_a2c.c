/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_ameba_a2c

#include <ameba_soc.h>

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <soc.h>

LOG_MODULE_REGISTER(can_ameba_a2c, CONFIG_CAN_LOG_LEVEL);

#define A2C_TODO 0
static u32 can_ameba_a2c_ram_buffer_map[] = {0x0,  0x6,  0xc,  0x12, 0x18, 0x1e, 0x24, 0x2a, 0x30,
					     0x36, 0x3c, 0x42, 0x48, 0x4e, 0x54, 0x5a, 0x60};

#define CAN_AMEBA_A2C_TIMING_MIN                                                                   \
	{                                                                                          \
		.sjw = 1,                                                                          \
		.prop_seg = 2,                                                                     \
		.phase_seg1 = 2,                                                                   \
		.phase_seg2 = 2,                                                                   \
		.prescaler = 1,                                                                    \
	}

#define CAN_AMEBA_A2C_TIMING_MAX                                                                   \
	{                                                                                          \
		.sjw = 4,                                                                          \
		.prop_seg = 6,                                                                     \
		.phase_seg1 = 8,                                                                   \
		.phase_seg2 = 8,                                                                   \
		.prescaler = 32,                                                                   \
	}

/*
 * Mutex to prevent simultaneous access to filter registers shared between CAN1
 * and CAN2.
 */

/**
 * @brief Ameba-A2C driver internal RX filter structure.
 */
struct can_ameba_a2c_rx_filter {
	struct can_filter filter;
	can_rx_callback_t callback;
	void *user_data;
};

struct can_ameba_a2c_config {
	const struct can_driver_config common;
	A2C_TypeDef *base;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	void (*config_irq)(void);
};

struct can_ameba_a2c_data {
	struct can_driver_data common;
	struct k_mutex inst_mutex;

	ATOMIC_DEFINE(rx_allocs, CONFIG_CAN_MAX_FILTER);
	struct can_ameba_a2c_rx_filter filters[CONFIG_CAN_MAX_FILTER];
	struct k_sem tx_idle;
	can_tx_callback_t tx_callback;
	void *tx_user_data;
	enum can_state state;
};

static void can_ameba_a2c_tx_done(const struct device *dev, int status)
{
	struct can_ameba_a2c_data *a2c_data = dev->data;

	if (a2c_data->tx_callback) {
		a2c_data->tx_callback(dev, status, a2c_data->tx_user_data);
		a2c_data->tx_callback = NULL;
	}

	k_sem_give(&a2c_data->tx_idle);
}

static void can_ameba_a2c_status_reset(A2C_TypeDef *a2c)
{
	A2C_TXErrCntClear(a2c);
	A2C_RXErrCntClear(a2c);
	A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_RX | A2C_BIT_ERROR_TX | A2C_BIT_ERROR_ACK |
					A2C_BIT_ERROR_STUFF | A2C_BIT_ERROR_CRC |
					A2C_BIT_ERROR_FORM | A2C_BIT_ERROR_BIT1 |
					A2C_BIT_ERROR_BIT0);
	A2C_TxMsgBufErrClear(a2c, A2C_MASK_TX_ERROR_FLAG);
	A2C_TxDoneStatusClear(a2c, A2C_MASK_TX_DONE);
	A2C_RxDoneStatusClear(a2c, A2C_MASK_RX_DONE);
}
static int can_ameba_a2c_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_ONE_SHOT |
	       CAN_MODE_3_SAMPLES;

	return 0;
}

static int can_ameba_a2c_start(const struct device *dev)
{
	const struct can_ameba_a2c_config *a2c_config = dev->config;
	struct can_ameba_a2c_data *a2c_data = dev->data;
	int ret = 0;

	k_mutex_lock(&a2c_data->inst_mutex, K_FOREVER);

	if (a2c_data->common.started) {
		ret = -EALREADY;
		goto unlock;
	}

	CAN_STATS_RESET(dev);

	/* 1.Bus off */
	A2C_BusCmd(a2c_config->base, DISABLE);

	/* 2.Clear Tec&Rec count/error status/msg buffer status */
	can_ameba_a2c_status_reset(a2c_config->base);

	/* 3.Bus On */
	A2C_BusCmd(a2c_config->base, ENABLE);

	a2c_data->common.started = true;

unlock:
	k_mutex_unlock(&a2c_data->inst_mutex);

	return ret;
}

static int can_ameba_a2c_stop(const struct device *dev)
{
	const struct can_ameba_a2c_config *a2c_config = dev->config;
	struct can_ameba_a2c_data *a2c_data = dev->data;

	int ret = 0;

	k_mutex_lock(&a2c_data->inst_mutex, K_FOREVER);

	if (!a2c_data->common.started) {
		ret = -EALREADY;
		goto unlock;
	}
	/* 1.Clear irq flags/Tec&Rec count/error status/msg buffer status */
	A2C_ClearAllINT(a2c_config->base);

	/* 2.Close bus */
	A2C_BusCmd(a2c_config->base, DISABLE);

	/* 3.Update status */
	a2c_data->common.started = false;

	can_ameba_a2c_tx_done(dev, -ENETDOWN);

unlock:
	k_mutex_unlock(&a2c_data->inst_mutex);

	return ret;
}

static int can_ameba_a2c_set_mode(const struct device *dev, can_mode_t mode)
{
	can_mode_t supported = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY |
			       CAN_MODE_ONE_SHOT | CAN_MODE_3_SAMPLES;
	const struct can_ameba_a2c_config *a2c_config = dev->config;
	A2C_TypeDef *a2c = a2c_config->base;
	struct can_ameba_a2c_data *data = dev->data;

	LOG_DBG("Set mode %d", mode);

	if ((mode & ~(supported)) != 0) {
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}

	if (data->common.started) {
		return -EBUSY;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	/* set work Mode */
	if ((mode & CAN_MODE_NORMAL) != 0) {
		/* Normal mode */
		a2c->A2C_CTL &= ~A2C_BIT_TEST_MODE_EN;

	} else {
		a2c->A2C_CTL |= A2C_BIT_TEST_MODE_EN;
		a2c->A2C_TEST &= ~A2C_MASK_TEST_CFG;

		/* Loopback mode */
		if ((mode & CAN_MODE_LOOPBACK) != 0) {
			a2c->A2C_TEST |= A2C_TEST_CFG(A2C_EXT_LOOPBACK_MODE);
		}
		/* Silence mode*/
		if ((mode & CAN_MODE_LISTENONLY) != 0) {
			a2c->A2C_TEST |= A2C_TEST_CFG(A2C_SILENCE_MODE);
		}
	}

	/* auto reply */
	if ((mode & CAN_MODE_ONE_SHOT) != 0) {
		/* No automatic retransmission */
		a2c->A2C_CTL &= ~A2C_BIT_AUTO_RE_TX_EN;
	} else {
		a2c->A2C_CTL |= A2C_BIT_AUTO_RE_TX_EN;
	}

	/* tri-sample */
	if ((mode & CAN_MODE_3_SAMPLES) != 0) {
		a2c->A2C_CTL |= A2C_BIT_TRI_SAMPLE;
	} else {
		a2c->A2C_CTL &= ~A2C_BIT_TRI_SAMPLE;
	}

	data->common.mode = mode;

	k_mutex_unlock(&data->inst_mutex);

	return 0;
}

static int can_ameba_a2c_send(const struct device *dev, const struct can_frame *frame,
			      k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	const struct can_ameba_a2c_config *a2c_config = dev->config;
	struct can_ameba_a2c_data *a2c_data = dev->data;
	A2C_TypeDef *a2c = a2c_config->base;
	u32 a2c_ram_cmd, a2c_ram_arb, a2c_ram_cs;

	LOG_DBG("Sending %d bytes on %s. "
		"Id: 0x%x, "
		"ID type: %s, "
		"Remote Frame: %s",
		frame->dlc, dev->name, frame->id,
		(frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
		(frame->flags & CAN_FRAME_RTR) != 0 ? "yes" : "no");

	if ((frame->flags & (CAN_FRAME_FDF | CAN_FRAME_BRS | CAN_FRAME_ESI)) != 0) {
		LOG_ERR("CAN-FD Not Available");
		return -ENOTSUP;
	}

	if (frame->dlc > CAN_MAX_DLC) {
		LOG_ERR("TX frame DLC %u exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
		return -EINVAL;
	}

	if (!a2c_data->common.started) {
		return -ENETDOWN;
	}

	if (a2c_data->state == CAN_STATE_BUS_OFF) {
		LOG_DBG("transmit failed, bus-off");
		return -ENETUNREACH;
	}

	if (k_sem_take(&a2c_data->tx_idle, timeout) != 0) {
		return -EAGAIN;
	}
	k_mutex_lock(&a2c_data->inst_mutex, K_FOREVER);

	/* 1.register tx callback */
	a2c_data->tx_callback = callback;
	a2c_data->tx_user_data = user_data;

	/* 2.Enable msg buffer access */
	a2c_ram_cmd = (A2C_BIT_RAM_BUFFER_EN | A2C_BIT_RAM_ACC_ARB | A2C_BIT_RAM_ACC_CS |
		       A2C_BIT_RAM_ACC_MASK | A2C_BIT_RAM_ACC_DATA_MASK | A2C_BIT_RAM_DIR);
	a2c_ram_cmd |= A2C_RAM_ACC_NUM(0);
	a2c->A2C_RAM_CMD = a2c_ram_cmd;

	/* 3.Config Frame ARB reg */
	a2c_ram_arb = a2c->A2C_RAM_ARB;
	a2c_ram_arb &= (~(A2C_BIT_RAM_RTR | A2C_BIT_RAM_IDE | A2C_MASK_RAM_ID));
	/* 4.RTR/IDE */
	if (frame->flags & CAN_FRAME_RTR) {
		a2c_ram_arb |= A2C_BIT_RAM_RTR;
	}
	/* 5.Extern or stadard frame */
	if (frame->flags & CAN_FRAME_IDE) {
		a2c_ram_arb |= frame->id | A2C_BIT_RAM_IDE;
	} else {
		a2c_ram_arb |= (frame->id & 0x7FF) << 18;
	}
	a2c->A2C_RAM_ARB = a2c_ram_arb;

	/* 6.fill tx msg */
	a2c_ram_cs = a2c->A2C_RAM_CS;
	a2c_ram_cs &= (~(A2C_MASK_RAM_DLC | A2C_BIT_RAM_AUTOREPLY | A2C_BIT_RAM_EDL |
			 A2C_BIT_RAM_BRS | A2C_BIT_RAM_ESI));
	a2c_ram_cs |= A2C_RAM_DLC(frame->dlc);
	/* 0 for rx, 1 for tx */
	a2c_ram_cs |= A2C_BIT_RAM_RXTX;
	a2c->A2C_RAM_CS = a2c_ram_cs;

	if ((frame->flags & CAN_FRAME_RTR) == 0) {
		LOG_DBG("data[0] = %x, data[1] =%x", frame->data_32[0], frame->data_32[1]);
		a2c->A2C_RAM_FDDATA_x[15] = frame->data_32[0];
		a2c->A2C_RAM_FDDATA_x[14] = frame->data_32[1];
	}
	/* 7.send msg */
	a2c->A2C_RAM_CMD |= A2C_BIT_RAM_START;
	LOG_DBG("TX Settings: ARB = %x, CMD = %x, MSK = %x", a2c->A2C_RAM_ARB, a2c->A2C_RAM_CMD,
		a2c->A2C_RAM_MASK);
	while (a2c->A2C_RAM_CMD & A2C_BIT_RAM_START) {
	}
	k_mutex_unlock(&a2c_data->inst_mutex);

	return 0;
}

static int can_ameba_a2c_set_timing(const struct device *dev, const struct can_timing *timing)
{
	const struct can_ameba_a2c_config *a2c_config = dev->config;
	struct can_ameba_a2c_data *a2c_data = dev->data;
	A2C_TypeDef *a2c = a2c_config->base;

	if (a2c_data->common.started) {
		return -EBUSY;
	}

	k_mutex_lock(&a2c_data->inst_mutex, K_FOREVER);

	a2c->A2C_BIT_TIMING &= ~(A2C_MASK_BRP | A2C_MASK_SJW | A2C_MASK_TSEG2 | A2C_MASK_TSEG1);
	a2c->A2C_BIT_TIMING |= A2C_BRP(timing->prescaler - 1) | A2C_SJW(timing->sjw) |
			       A2C_TSEG1(timing->prop_seg + timing->phase_seg1 - 1) |
			       A2C_TSEG2(timing->phase_seg2 - 1);

	if ((a2c_data->common.mode & CAN_MODE_3_SAMPLES) != 0) {
		a2c->A2C_CTL |= A2C_BIT_TRI_SAMPLE;
	}

	k_mutex_unlock(&a2c_data->inst_mutex);

	return 0;
}

static int can_ameba_a2c_get_core_clock(const struct device *dev, uint32_t *rate)
{
	ARG_UNUSED(dev);

	*rate = XTAL_40M;

	return 0;
}
void can_ameba_a2c_read_msg(A2C_TypeDef *A2Cx, struct can_frame *frame, uint8_t msg_idx)
{
	u32 a2c_ram_arb, a2c_ram_cmd, a2c_ram_cs;

	/* Enable msg buffer access*/
	a2c_ram_cmd = (A2C_BIT_RAM_BUFFER_EN | A2C_BIT_RAM_ACC_ARB | A2C_BIT_RAM_ACC_CS |
		       A2C_BIT_RAM_ACC_MASK | A2C_BIT_RAM_ACC_DATA_MASK);
	a2c_ram_cmd |= msg_idx;
	/* Read frame into register from ram message buffer */
	a2c_ram_cmd |= A2C_BIT_RAM_START;
	A2Cx->A2C_RAM_CMD = a2c_ram_cmd;

	/* Read frame flags and ID */
	a2c_ram_arb = A2Cx->A2C_RAM_ARB;
	if (a2c_ram_arb & A2C_BIT_RAM_RTR) {
		frame->flags |= CAN_FRAME_RTR;
	}

	if (a2c_ram_arb & A2C_BIT_RAM_IDE) {
		frame->flags |= CAN_FRAME_IDE;
		frame->id = A2C_GET_RAM_ID(a2c_ram_arb);
	} else {
		frame->id = A2C_GET_RAM_ID(a2c_ram_arb) >> 18;
	}
	/* Read data length of frame*/
	a2c_ram_cs = A2Cx->A2C_RAM_CS;
#if defined(CONFIG_CAN_RX_TIMESTAMP)
	frame->timestamp = A2C_GET_RAM_TIMESTAMP(a2c_ram_cs);
#endif

	if (frame->flags & CAN_FRAME_RTR) {
		frame->dlc = 0;
	} else {
		frame->dlc = A2C_GET_RAM_DLC(a2c_ram_cs);
	}

	/* Get Data: can2.0 8 bytes, can fd 64 bytes */
	if ((frame->flags & CAN_FRAME_RTR) == 0) {
		frame->data_32[0] = A2Cx->A2C_RAM_FDDATA_x[15];
		frame->data_32[1] = A2Cx->A2C_RAM_FDDATA_x[14];
	}
}

static void can_ameba_a2c_rx_msg(const struct device *dev)
{
	const struct can_ameba_a2c_config *a2c_config = dev->config;
	struct can_ameba_a2c_data *a2c_data = dev->data;
	A2C_TypeDef *a2c = a2c_config->base;
	struct can_frame frame;
	can_rx_callback_t callback;

	for (int msg_buf_idx = A2C_MESSAGE_BUFFER_SIZE - 1; msg_buf_idx >= 0; msg_buf_idx--) {
		memset(&frame, 0, sizeof(struct can_frame));
		if (A2C_MsgBufRxDoneStatusGet(a2c, msg_buf_idx)) {
			A2C_MsgBufRxDoneStatusClear(a2c, msg_buf_idx);
			can_ameba_a2c_read_msg(a2c, &frame, msg_buf_idx);

#ifndef CONFIG_CAN_ACCEPT_RTR
			if ((frame.flags & CAN_FRAME_RTR) == 0U) {
#endif /* !CONFIG_CAN_ACCEPT_RTR */
				for (int i = 0; i < ARRAY_SIZE(a2c_data->filters); i++) {
					if (!atomic_test_bit(a2c_data->rx_allocs, i)) {
						continue;
					}
					if (!can_frame_matches_filter(
						    &frame, &a2c_data->filters[i].filter)) {
						continue;
					}

					callback = a2c_data->filters[i].callback;
					if (callback != NULL) {
						callback(dev, &frame,
							 a2c_data->filters[i].user_data);
					}
				}
#ifndef CONFIG_CAN_ACCEPT_RTR
			}
#endif /* !CONFIG_CAN_ACCEPT_RTR */
		}
	}
}

static void can_ameba_a2c_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct can_ameba_a2c_config *a2c_config = dev->config;

	A2C_TypeDef *a2c = a2c_config->base;
	u32 IntStatus, ErrStatus, TxErCnt, RxErCnt, ErrPassive, ErrBusoff, ErrWarning;

	IntStatus = A2C_GetINTStatus(a2c);
	/* ram move done interrupt */
	if (IntStatus & A2C_RAM_MOVE_DONE_INT) {
		A2C_ClearINT(a2c, A2C_BIT_RAM_MOVE_DONE_INT_FLAG);
		LOG_DBG("RAM MOVE DONE INT");
	}

	/* tx interrupt */
	if (IntStatus & A2C_TX_INT) {
		A2C_ClearINT(a2c, A2C_BIT_TX_INT_FLAG);
		can_ameba_a2c_tx_done(dev, 0);
	}

	/* rx interrupt */
	if (IntStatus & A2C_RX_INT) {
		A2C_ClearINT(a2c, A2C_BIT_RX_INT_FLAG);
		LOG_DBG("RX INT");
		/* get current error status */
		TxErCnt = A2C_TXErrCntGet(a2c);
		RxErCnt = A2C_RXErrCntGet(a2c);
		ErrPassive = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_PASSIVE) >> 28;
		ErrBusoff = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_BUSOFF) >> 29;
		ErrWarning = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_WARNING) >> 30;
		can_ameba_a2c_rx_msg(dev);
	}

	/* bus off interrupt */
	if (IntStatus & A2C_BUSOFF_INT) {
		A2C_ClearINT(a2c, A2C_BIT_BUSOFF_INT_FLAG);
		LOG_DBG("A2C: bus off");
	}

	/* wakeup interrupt */
	if (IntStatus & A2C_WKUP_INT) {
		A2C_ClearINT(a2c, A2C_BIT_WAKEUP_INT_FLAG);
		LOG_DBG("A2C: wake up");
	}

	/* error interrupt */
	if (IntStatus & A2C_ERR_INT) {
		A2C_ClearINT(a2c, A2C_BIT_ERROR_INT_FLAG);
		LOG_ERR("A2C: Clear rx Status = %x", A2C_GetINTStatus(a2c));

		ErrStatus = A2C_GetErrStatus(a2c);
		TxErCnt = A2C_TXErrCntGet(a2c);
		RxErCnt = A2C_RXErrCntGet(a2c);
		ErrPassive = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_PASSIVE) >> 28;
		ErrBusoff = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_BUSOFF) >> 29;
		ErrWarning = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_WARNING) >> 30;

		if (ErrStatus & A2C_BIT_ERROR_BIT0) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_BIT0);
			LOG_ERR("bit 0 error: tx = 0, but rx = 1");
		}
		if (ErrStatus & A2C_BIT_ERROR_BIT1) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_BIT1);
			LOG_ERR("bit 1 error: tx = 1, but rx = 0");
		}
		if (ErrStatus & A2C_BIT_ERROR_FORM) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_FORM);
			LOG_ERR("form error");
		}
		if (ErrStatus & A2C_BIT_ERROR_CRC) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_CRC);
			LOG_ERR("CRC error");
		}
		if (ErrStatus & A2C_BIT_ERROR_STUFF) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_STUFF);
			LOG_ERR("stuff error");
		}
		if (ErrStatus & A2C_BIT_ERROR_ACK) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_ACK);
			LOG_ERR("ACK error");
		}
		if (ErrStatus & A2C_BIT_ERROR_TX) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_TX);
			LOG_ERR("tx error");
		}
		if (ErrStatus & A2C_BIT_ERROR_RX) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_RX);
			LOG_ERR("rx error");
		}

		LOG_ERR("ErrStatus = %x, TEC = %d, REC = %d, ErrPassive = %d, ErrBusoff = %d, "
			"ErrWarning = %d",
			ErrStatus, TxErCnt, RxErCnt, ErrPassive, ErrBusoff, ErrWarning);
	}
}

static int can_ameba_a2c_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
				       void *user_data, const struct can_filter *filter)
{
	const struct can_ameba_a2c_config *a2c_config = dev->config;
	struct can_ameba_a2c_data *a2c_data = dev->data;
	A2C_TypeDef *a2c = a2c_config->base;
	uint32_t a2c_ram_cmd, a2c_ram_arb;
	int filter_id = -ENOSPC;
	int i;

	for (i = 0; i < ARRAY_SIZE(a2c_data->filters); i++) {
		if (!atomic_test_and_set_bit(a2c_data->rx_allocs, i)) {
			filter_id = i;
			break;
		}
	}
	k_mutex_lock(&a2c_data->inst_mutex, K_FOREVER);

	if (filter_id >= 0) {
		a2c_data->filters[filter_id].filter = *filter;
		a2c_data->filters[filter_id].user_data = user_data;
		a2c_data->filters[filter_id].callback = callback;

		/* Enable Msg Buffer access*/
		a2c_ram_cmd = (A2C_BIT_RAM_BUFFER_EN | A2C_BIT_RAM_ACC_ARB | A2C_BIT_RAM_ACC_CS |
			       A2C_BIT_RAM_ACC_MASK | A2C_BIT_RAM_ACC_DATA_MASK | A2C_BIT_RAM_DIR);
		a2c_ram_cmd &= ~A2C_MASK_RAM_ACC_NUM;
		a2c_ram_cmd |= A2C_RAM_ACC_NUM(A2C_MESSAGE_BUFFER_SIZE - filter_id - 1);

		/*Config frame header*/
		if (filter->flags & CAN_FILTER_IDE) {
			a2c_ram_arb = A2C_RAM_ID(filter->id) | A2C_BIT_RAM_IDE;
		} else {
			a2c_ram_arb = A2C_RAM_ID((filter->id & 0x7FF) << 18);
		}

		if (filter->flags & CAN_FRAME_RTR) {
			a2c_ram_arb |= A2C_BIT_RAM_RTR;
		}

		a2c->A2C_RAM_ARB = a2c_ram_arb;

		/* Enable the current buffer to receive data */
		a2c->A2C_RAM_CS &= ~A2C_BIT_RAM_RXTX;

		/* Set the mask ID to block unwanted data frames */
		if (filter->flags & CAN_FILTER_IDE) {
			a2c->A2C_RAM_MASK = A2C_BIT_RAM_IDE_MASK | A2C_RAM_ID_MASK(filter->mask);
		} else {
			a2c->A2C_RAM_MASK = A2C_RAM_ID_MASK(filter->mask << 18);
		}

		/* Write RX setting into the ram message buffer */
		a2c_ram_cmd |= A2C_BIT_RAM_START;
		a2c->A2C_RAM_CMD = a2c_ram_cmd;
		LOG_DBG("RX Settings: ARB = %x, CMD = %x, MSK = %x", a2c->A2C_RAM_ARB,
			a2c->A2C_RAM_CMD, a2c->A2C_RAM_MASK);
		while (a2c->A2C_RAM_CMD & A2C_BIT_RAM_START) {
		}
	}
	k_mutex_unlock(&a2c_data->inst_mutex);
	return filter_id;
}

static void can_ameba_a2c_remove_rx_filter(const struct device *dev, int filter_id)
{
	const struct can_ameba_a2c_config *a2c_config = dev->config;
	struct can_ameba_a2c_data *a2c_data = dev->data;
	A2C_TypeDef *a2c = a2c_config->base;
	uint32_t a2c_ram_cmd;
	if (filter_id < 0 || filter_id >= ARRAY_SIZE(a2c_data->filters)) {
		LOG_ERR("filter ID %d out of bounds", filter_id);
		return;
	}

	if (atomic_test_and_clear_bit(a2c_data->rx_allocs, filter_id)) {
		a2c_data->filters[filter_id].callback = NULL;
		a2c_data->filters[filter_id].user_data = NULL;
		a2c_data->filters[filter_id].filter = (struct can_filter){0};

		/* Enable msg buf[x] access */
		a2c_ram_cmd = A2C_RAM_ACC_NUM(A2C_MESSAGE_BUFFER_SIZE - filter_id - 1);

		/* Reset RAM ARB/CS/MASK/ reg */
		a2c->A2C_RAM_ARB = 0;
		a2c->A2C_RAM_CS = 0;
		a2c->A2C_RAM_MASK = 0;

		/* Write RX setting into the ram message buffer */
		a2c_ram_cmd |= A2C_BIT_RAM_START;
		a2c->A2C_RAM_CMD = a2c_ram_cmd;
		LOG_DBG("ARB = %x, CMD = %x, MSK = %x", a2c->A2C_RAM_ARB, a2c->A2C_RAM_CMD,
			a2c->A2C_RAM_MASK);
	}
}

static int can_ameba_a2c_get_state(const struct device *dev, enum can_state *state,
				   struct can_bus_err_cnt *err_cnt)
{
	const struct can_ameba_a2c_config *a2c_config = dev->config;
	struct can_ameba_a2c_data *a2c_data = dev->data;

	if (state != NULL) {
		if (!a2c_data->common.started) {
			*state = CAN_STATE_STOPPED;
		} else {
			*state = a2c_data->state;
		}
	}

	if (err_cnt != NULL) {
		err_cnt->rx_err_cnt = A2C_RXErrCntGet(a2c_config->base);
		err_cnt->tx_err_cnt = A2C_TXErrCntGet(a2c_config->base);
	}

	return 0;
}

static void can_ameba_a2c_set_state_change_callback(const struct device *dev,
						    can_state_change_callback_t callback,
						    void *user_data)
{
	struct can_ameba_a2c_data *a2c_data = dev->data;

	a2c_data->common.state_change_cb = callback;
	a2c_data->common.state_change_cb_user_data = user_data;
}

static int can_ameba_a2c_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ide);

	return CONFIG_CAN_MAX_FILTER;
}

static int can_ameba_a2c_init(const struct device *dev)
{
	const struct can_ameba_a2c_config *a2c_config = dev->config;
	struct can_ameba_a2c_data *a2c_data = dev->data;
	A2C_TypeDef *a2c = a2c_config->base;
	A2C_InitTypeDef A2C_InitStruct;
	int err = 0;

	if (!device_is_ready(a2c_config->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	err = pinctrl_apply_state(a2c_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		LOG_ERR("failed to configure A2C pins (err %d)", err);
		return err;
	}

	err = clock_control_on(a2c_config->clock_dev, a2c_config->clock_subsys);
	if (err != 0) {
		LOG_ERR("failed to enable CAN clock (err %d)", err);
		return err;
	}
	RCC_PeriphClockDividerFENSet(USB_PLL_A2C, DISABLE);
	RCC_PeriphClockDividerFENSet(SYS_PLL_A2C, DISABLE);
	RCC_PeriphClockSourceSet(A2C, XTAL);

	/* Config basic parameters */
	A2C_StructInit(&A2C_InitStruct);
	A2C_Init(a2c, &A2C_InitStruct);

	/* Initialize message buffer addr */
	A2C_RamBufferMapConfig(a2c_config->base, can_ameba_a2c_ram_buffer_map);

	k_sem_init(&a2c_data->tx_idle, 1, 1);
	/* Enable Interrupt */
	a2c_config->config_irq();

	/* Enable A2C */
	A2C_Cmd(a2c, ENABLE);
	k_mutex_init(&a2c_data->inst_mutex);

	return err;
}

DEVICE_API(can, can_ameba_a2c_driver_api) = {
	.get_capabilities = can_ameba_a2c_get_capabilities,
	.start = can_ameba_a2c_start,
	.stop = can_ameba_a2c_stop,
	.set_mode = can_ameba_a2c_set_mode,
#ifdef CONFIG_SOC_SERIES_AMEBA
	.set_timing = can_ameba_a2c_set_timing,
#else
	.set_timing = can_ameba_a2c_set_timing,
#endif /* CONFIG_SOC_SERIES_AMEBA */
	.send = can_ameba_a2c_send,
	.add_rx_filter = can_ameba_a2c_add_rx_filter,
	.remove_rx_filter = can_ameba_a2c_remove_rx_filter,
	.get_state = can_ameba_a2c_get_state,
	.set_state_change_callback = can_ameba_a2c_set_state_change_callback,
	.get_core_clock = can_ameba_a2c_get_core_clock,
	.get_max_filters = can_ameba_a2c_get_max_filters,
	.timing_min = CAN_AMEBA_A2C_TIMING_MIN,
	.timing_max = CAN_AMEBA_A2C_TIMING_MAX,
};

#define CAN_AMEBA_A2C_IRQ_INST(inst)                                                               \
	static void config_can_ameba_a2c_##inst##_irq(void)                                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ(inst, irq), DT_INST_IRQ(inst, priority),                   \
			    can_ameba_a2c_isr, DEVICE_DT_INST_GET(inst), 0);                       \
		irq_enable(DT_INST_IRQ(inst, irq));                                                \
		A2C_INTConfig((A2C_TypeDef *)DT_INST_REG_ADDR(inst),                               \
			      A2C_TX_INT | A2C_RX_INT | A2C_ERR_INT | A2C_WKUP_INT |               \
				      A2C_BUSOFF_INT | A2C_RAM_MOVE_DONE_INT,                      \
			      ENABLE);                                                             \
		A2C_TxMsgBufINTConfig((A2C_TypeDef *)DT_INST_REG_ADDR(inst),                       \
				      A2C_MB_TXINT_EN(0xFFFF), ENABLE);                            \
		A2C_RxMsgBufINTConfig((A2C_TypeDef *)DT_INST_REG_ADDR(inst),                       \
				      A2C_MB_RXINT_EN(0xFFFF), ENABLE);                            \
	}

#define CAN_AMEBA_A2C_INIT(inst)                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	CAN_AMEBA_A2C_IRQ_INST(inst)                                                               \
	const struct can_ameba_a2c_config can_ameba_a2c_config_##inst = {                          \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, 1000000),                         \
		.base = (A2C_TypeDef *)DT_INST_REG_ADDR(inst),                                     \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, idx),            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.config_irq = config_can_ameba_a2c_##inst##_irq};                                  \
                                                                                                   \
	static struct can_ameba_a2c_data can_ameba_a2c_data_##inst = {                             \
		.common = {0},                                                                     \
		.state = 0,                                                                        \
	};                                                                                         \
	CAN_DEVICE_DT_INST_DEFINE(inst, can_ameba_a2c_init, NULL, &can_ameba_a2c_data_##inst,      \
				  &can_ameba_a2c_config_##inst, POST_KERNEL,                       \
				  CONFIG_CAN_INIT_PRIORITY, &can_ameba_a2c_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CAN_AMEBA_A2C_INIT)

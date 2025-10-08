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

#define CAN_AMEBA_A2C_TIMING_MIN                                                                   \
	{                                                                                          \
		.sjw = 1,                                                                          \
		.prop_seg = 1,                                                                     \
		.phase_seg1 = 1,                                                                   \
		.phase_seg2 = 2,                                                                   \
		.prescaler = 1,                                                                    \
	}

#define CAN_AMEBA_A2C_TIMING_MAX                                                                   \
	{                                                                                          \
		.sjw = 4,                                                                          \
		.prop_seg = 1,                                                                     \
		.phase_seg1 = 16,                                                                  \
		.phase_seg2 = 16,                                                                  \
		.prescaler = 256,                                                                  \
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
	enum can_state state;
};

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

	/*1.Bus off*/
	A2C_BusCmd(a2c_config->base, DISABLE);

	/*2.clear Tec&Rec count/error status/msg buffer status*/
	LOG_INF("reg = %x\n", (u32)a2c_config->base);
	can_ameba_a2c_status_reset(a2c_config->base);
	/*3.Enable irq*/
	A2C_INTConfig(a2c_config->base,
		      A2C_TX_INT | A2C_RX_INT | A2C_ERR_INT | A2C_WKUP_INT | A2C_BUSOFF_INT |
			      A2C_RAM_MOVE_DONE_INT,
		      ENABLE);
	/*4.Enable A2C*/
	A2C_Cmd(a2c_config->base, ENABLE);
	/*5.Bus On*/
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
	A2C_TypeDef *a2c = a2c_config->base;

	int ret = 0;

	k_mutex_lock(&a2c_data->inst_mutex, K_FOREVER);

	if (!a2c_data->common.started) {
		ret = -EALREADY;
		goto unlock;
	}

	/*1.mask irq*/
	A2C_INTConfig(a2c_config->base,
		      A2C_TX_INT | A2C_RX_INT | A2C_ERR_INT | A2C_WKUP_INT | A2C_BUSOFF_INT |
			      A2C_RAM_MOVE_DONE_INT,
		      DISABLE);
	/*2.clear irq flags/Tec&Rec count/error status/msg buffer status*/
	A2C_ClearAllINT(a2c_config->base);
	LOG_INF("reg = %x\n", (u32)a2c_config->base);
	can_ameba_a2c_status_reset(a2c_config->base);

	/*3.exit test mode*/
	a2c->A2C_CTL &= ~A2C_BIT_TEST_MODE_EN;
	/*4.close bus*/
	A2C_BusCmd(a2c_config->base, DISABLE);
	/*5.disable a2c*/
	A2C_Cmd(a2c_config->base, DISABLE);

	/*update status*/
	a2c_data->common.started = false;

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

	/*set work Mode*/
	if ((mode & CAN_MODE_NORMAL) != 0) {
		/*Normal mode*/
		a2c->A2C_CTL &= ~A2C_BIT_TEST_MODE_EN;

	} else {
		a2c->A2C_CTL |= A2C_BIT_TEST_MODE_EN;
		a2c->A2C_TEST &= ~A2C_MASK_TEST_CFG;

		/* Loopback mode */
		if ((mode & CAN_MODE_LOOPBACK) != 0) {
			a2c->A2C_TEST |= A2C_TEST_CFG(A2C_INT_LOOPBACK_MODE);
		}
		/**/
		if ((mode & CAN_MODE_LISTENONLY) != 0) {
			a2c->A2C_TEST |= A2C_TEST_CFG(A2C_SILENCE_MODE);
		}
	}

	/*auto reply*/
	if ((mode & CAN_MODE_ONE_SHOT) != 0) {
		/* No automatic retransmission */
		a2c->A2C_CTL &= ~A2C_BIT_AUTO_RE_TX_EN;
	} else {
		a2c->A2C_CTL |= A2C_BIT_AUTO_RE_TX_EN;
	}

	/*tri-sample*/
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
	A2C_TypeDef *a2c = a2c_config->base;
	struct can_ameba_a2c_data *a2c_data = dev->data;

	LOG_DBG("Sending %d bytes on %s. "
		"Id: 0x%x, "
		"ID type: %s, "
		"Remote Frame: %s",
		frame->dlc, dev->name, frame->id,
		(frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
		(frame->flags & CAN_FRAME_RTR) != 0 ? "yes" : "no");

	if (frame->dlc > CAN_MAX_DLC) {
		LOG_ERR("TX frame DLC %u exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
		return -EINVAL;
	}

	if (!a2c_data->common.started) {
		return -ENETDOWN;
	}
	/*return if bus off */
	if (a2c_data->state == CAN_STATE_BUS_OFF) {
		LOG_DBG("transmit failed, bus-off");
		return -ENETUNREACH;
	}

	k_mutex_lock(&a2c_data->inst_mutex, K_FOREVER);
	/*1.Enable access*/
	a2c->A2C_RAM_CMD |= (A2C_BIT_RAM_BUFFER_EN | A2C_BIT_RAM_ACC_ARB | A2C_BIT_RAM_ACC_CS |
			     A2C_BIT_RAM_ACC_MASK | A2C_BIT_RAM_ACC_DATA_MASK | A2C_BIT_RAM_DIR);

	/*2.fill tx msg*/
	a2c->A2C_RAM_CMD &= ~A2C_MASK_RAM_ACC_NUM;
	a2c->A2C_RAM_CS &= ~(A2C_MASK_RAM_DLC | A2C_BIT_RAM_AUTOREPLY | A2C_BIT_RAM_EDL |
			     A2C_BIT_RAM_BRS | A2C_BIT_RAM_ESI);
	a2c->A2C_RAM_CS |= A2C_RAM_DLC(frame->dlc);
	a2c->A2C_RAM_CS |= A2C_BIT_RAM_RXTX; /*0 for rx, 1 for tx*/

	if ((frame->flags & CAN_FRAME_RTR) == 0) {
		for (u32 i = 0; i < CAN_MAX_DLEN / 4; i++) {
			a2c->A2C_RAM_FDDATA_x[i] = frame->data_32[CAN_MAX_DLEN / 4 - i - 1];
		}
	}

	/*3. send msg*/
	a2c->A2C_RAM_CMD |= A2C_BIT_RAM_START;

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
	a2c->A2C_BIT_TIMING |= A2C_BRP(timing->prescaler - 1) | A2C_SJW(timing->sjw - 1) |
			       A2C_TSEG1(timing->phase_seg1 - 1) |
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

static void can_ameba_a2c_check_rx_msg(A2C_TypeDef *A2Cx)
{
	u32 i;
	u32 status = 0;
	A2C_RxMsgTypeDef RxMsg;

	_memset(&RxMsg, 0, sizeof(A2C_RxMsgTypeDef));

	for (i = A2C_MESSAGE_BUFFER_SIZE; i > 0; i--) {
		RxMsg.MsgBufferIdx = i - 1;
		if (A2C_MsgBufRxDoneStatusGet(A2Cx, RxMsg.MsgBufferIdx)) {
			A2C_MsgBufRxDoneStatusClear(A2Cx, RxMsg.MsgBufferIdx);
			A2C_ReadMsg(A2Cx, &RxMsg);
			/*  can_ameba_a2c_dump_rx_msg(&RxMsg); */

			if (RxMsg.StdId != 0x0 && RxMsg.StdId != 0x55) {
				status = 1;
			}
			if (RxMsg.IDE != A2C_STANDARD_FRAME) {
				status = 1;
			}
			if (RxMsg.RTR != A2C_DATA_FRAME) {
				status = 1;
			}
			if (RxMsg.DLC != 8) {
				status = 1;
			}

			for (i = 0; i < 8; i++) {
				if (RxMsg.Data[i] != i) {
					status = 1;
					break;
				}
			}
			if (status == 1) {
				status = 0;
				LOG_INF("ERROR\n");
			}
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
	/*ram move done interrupt */
	if (IntStatus & A2C_RAM_MOVE_DONE_INT) {
		A2C_ClearINT(a2c, A2C_BIT_RAM_MOVE_DONE_INT_FLAG);
	}

	/*tx interrupt*/
	if (IntStatus & A2C_TX_INT) {
		A2C_ClearINT(a2c, A2C_BIT_TX_INT_FLAG);
	}

	/*rx interrupt*/
	if (IntStatus & A2C_RX_INT) {
		A2C_ClearINT(a2c, A2C_BIT_RX_INT_FLAG);

		/* get current error status */
		TxErCnt = A2C_TXErrCntGet(a2c);
		RxErCnt = A2C_RXErrCntGet(a2c);
		ErrPassive = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_PASSIVE) >> 28;
		ErrBusoff = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_BUSOFF) >> 29;
		ErrWarning = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_WARNING) >> 30;

		can_ameba_a2c_check_rx_msg(a2c);
	}

	/* bus off interrupt */
	if (IntStatus & A2C_BUSOFF_INT) {
		A2C_ClearINT(a2c, A2C_BIT_BUSOFF_INT_FLAG);
		LOG_INF("A2C: bus off\n");
	}

	/* wakeup interrupt */
	if (IntStatus & A2C_WKUP_INT) {
		A2C_ClearINT(a2c, A2C_BIT_WAKEUP_INT_FLAG);
		LOG_INF("A2C: wake up\n");
	}

	/* error interrupt */
	if (IntStatus & A2C_ERR_INT) {
		A2C_ClearINT(a2c, A2C_BIT_ERROR_INT_FLAG);
		LOG_INF("A2C: Clear rx Status = %x\n", A2C_GetINTStatus(a2c));

		ErrStatus = A2C_GetErrStatus(a2c);
		TxErCnt = A2C_TXErrCntGet(a2c);
		RxErCnt = A2C_RXErrCntGet(a2c);
		ErrPassive = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_PASSIVE) >> 28;
		ErrBusoff = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_BUSOFF) >> 29;
		ErrWarning = (a2c->A2C_ERR_CNT_STS & A2C_BIT_ERROR_WARNING) >> 30;

		if (ErrStatus & A2C_BIT_ERROR_BIT0) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_BIT0);
			LOG_INF("bit 0 error: tx = 0, but rx = 1\n");
		}
		if (ErrStatus & A2C_BIT_ERROR_BIT1) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_BIT1);
			LOG_INF("bit 1 error: tx = 1, but rx = 0\n");
		}
		if (ErrStatus & A2C_BIT_ERROR_FORM) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_FORM);
			LOG_INF("form error\n");
		}
		if (ErrStatus & A2C_BIT_ERROR_CRC) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_CRC);
			LOG_INF("CRC error\n");
		}
		if (ErrStatus & A2C_BIT_ERROR_STUFF) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_STUFF);
			LOG_INF("stuff error\n");
		}
		if (ErrStatus & A2C_BIT_ERROR_ACK) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_ACK);
			LOG_INF("ACK error\n");
		}
		if (ErrStatus & A2C_BIT_ERROR_TX) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_TX);
			LOG_INF("tx error\n");
		}
		if (ErrStatus & A2C_BIT_ERROR_RX) {
			A2C_ClearErrStatus(a2c, A2C_BIT_ERROR_RX);
			LOG_INF("rx error\n");
		}

		LOG_INF("ErrStatus = %x, TEC = %d, REC = %d, ErrPassive = %d, ErrBusoff = %d, "
			"ErrWarning = %d\n",
			ErrStatus, TxErCnt, RxErCnt, ErrPassive, ErrBusoff, ErrWarning);
	}
}

static int can_ameba_a2c_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
				       void *user_data, const struct can_filter *filter)
{
	struct can_ameba_a2c_data *a2c_data = dev->data;
	int filter_id = -ENOSPC;
	int i;

	for (i = 0; i < ARRAY_SIZE(a2c_data->filters); i++) {
		if (!atomic_test_and_set_bit(a2c_data->rx_allocs, i)) {
			filter_id = i;
			break;
		}
	}

	if (filter_id >= 0) {
		a2c_data->filters[filter_id].filter = *filter;
		a2c_data->filters[filter_id].user_data = user_data;
		a2c_data->filters[filter_id].callback = callback;
	}

	return filter_id;
}

static void can_ameba_a2c_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct can_ameba_a2c_data *a2c_data = dev->data;

	if (filter_id < 0 || filter_id >= ARRAY_SIZE(a2c_data->filters)) {
		LOG_ERR("filter ID %d out of bounds", filter_id);
		return;
	}

	if (atomic_test_and_clear_bit(a2c_data->rx_allocs, filter_id)) {
		a2c_data->filters[filter_id].callback = NULL;
		a2c_data->filters[filter_id].user_data = NULL;
		a2c_data->filters[filter_id].filter = (struct can_filter){0};
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

	/*register isr*/
	a2c_config->config_irq();

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

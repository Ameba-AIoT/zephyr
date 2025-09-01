/*
 * Copyright (c) 2025 Realtek Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include "hci/hci_common.h"
#include "hci/hci_if_zephyr.h"
#include "hci/hci_transport.h"
#include "hci_uart.h"
#include "hci_platform.h"
#include "bt_debug.h"
#include <zephyr/drivers/bluetooth.h>
/* #include <zephyr_log.h> */

#define DT_DRV_COMPAT realtek_ameba_bt_hci

struct bt_rtk_data {
	bt_hci_recv_t recv;
};

static void zephyr_recv(struct hci_rx_packet_t *pkt)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	struct bt_rtk_data *hci = dev->data;
	struct net_buf *buf = NULL;
	uint8_t type;

	if (pkt->type == HCI_EVT) {
		buf = bt_buf_get_evt(((struct bt_hci_evt_hdr *)(pkt->buf))->evt, pkt->discardable,
				     pkt->discardable ? K_NO_WAIT : K_FOREVER);
		type = BT_BUF_EVT;
	} else if (pkt->type == HCI_ACL) {
		buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);
		type = BT_BUF_ACL_IN;
	} else if (pkt->type == HCI_ISO) {
		buf = bt_buf_get_rx(BT_BUF_ISO_IN, K_FOREVER);
		type = BT_BUF_ISO_IN;
	} /* else if (pkt->type == HCI_SCO) {
	   *	buf = bt_buf_get_rx(BT_BUF_SCO_IN, K_FOREVER);
	   *	type = BT_BUF_SCO_IN;
	   * }
	   */

	if (!buf) {
		return;
	}

	if (buf->size < pkt->len) {
		net_buf_unref(buf);
		return;
	}

	bt_buf_set_type(buf, type);

	/* btsnoop_send(pkt->type, pkt->buf, pkt->len, true); */

	net_buf_add_mem(buf, pkt->buf, pkt->len);
	hci->recv(dev, buf);
}

static struct hci_transport_cb zephyr_stack_cb = {
	.recv = zephyr_recv,
};

static int hci_open(const struct device *dev, bt_hci_recv_t recv)
{
	struct bt_rtk_data *hci = dev->data;

	hci->recv = recv;

	if (!hci_controller_enable()) {
		return -1;
	}

	/* HCI Transport Bridge to Zephyr Stack */
	hci_transport_register(&zephyr_stack_cb);

	return 0;
}

static int hci_send(const struct device *dev, struct net_buf *buf)
{
	ARG_UNUSED(dev);
	uint8_t type;

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT:
		type = HCI_ACL;
		break;
	case BT_BUF_CMD:
		type = HCI_CMD;
		break;
	case BT_BUF_ISO_OUT:
		type = HCI_ISO;
		break;
	/* case BT_BUF_SCO_OUT:
	 *	   type = HCI_SCO;
	 *	   break;
	 */
	default:
		return -EINVAL;
	}

	if (hci_transport_send(type, buf->data, buf->len, true) != buf->len) {
		return -EIO;
	}

	/* btsnoop_send(type, buf->data, buf->len, false); */
	net_buf_unref(buf);

	return 0;
}

static int hci_close(const struct device *dev)
{
	struct bt_rtk_data *hci = dev->data;

	hci_controller_disable();
	hci_controller_free();

	hci->recv = NULL;
	return 0;
}

static DEVICE_API(bt_hci, drv) = {
	.open = hci_open,
	.send = hci_send,
	.close = hci_close,
};

static struct bt_rtk_data bt_rtk_data = {};                                      
DEVICE_DT_INST_DEFINE(0, NULL, NULL, &bt_rtk_data, NULL, 
                      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &drv)


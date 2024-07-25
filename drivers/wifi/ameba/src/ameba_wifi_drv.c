/*
 * Copyright (c) 2024 Realtek Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ameba_wifi, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/kernel.h>

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/device.h>
#include <soc.h>
#include "ameba_wifi.h"

#ifndef ZEPHYR_WIFI_TODO
/* header files add */
#endif

#define DT_DRV_COMPAT realtek_ameba_wifi

/* zephyr wifi todo, temp */
#define CONFIG_RTK_WIFI_EVENT_TASK_STACK_SIZE	4096
//#define CONFIG_RTK_WIFI_EVENT_TASK_PRIO		K_IDLE_PRIO + 6

#define DHCPV4_MASK (NET_EVENT_IPV4_DHCP_BOUND | NET_EVENT_IPV4_DHCP_STOP)


/* use global iface pointer to support any ethernet driver */
/* necessary for wifi callback functions */
static struct net_if *rtk_wifi_iface[NET_IF_MAX_CONFIGS];

/* zephyr wifi todo, temp */
static struct rtk_wifi_runtime rtk_data;

/* global para for wifi connect and ap info now */
_Alignas(4) static rtw_network_info_t1 wifi = {0};
static unsigned char password[129] = {0};
static rtw_softap_info_t1 ap = {0};

static int init_done = 0;
static unsigned char if_idx = 0;

static void rtk_wifi_event_task(void);

static char iface_name[2][10] = {"wlan0",  "wlan1"};

K_MSGQ_DEFINE(rtk_wifi_msgq, sizeof(struct rtk_system_event), 10, 4);
K_THREAD_STACK_DEFINE(rtk_wifi_event_stack, CONFIG_RTK_WIFI_EVENT_TASK_STACK_SIZE);

static struct k_thread rtk_wifi_event_thread;
/* for add dhcp callback in mgmt_thread in net_mgmt */
static struct net_mgmt_event_callback rtk_dhcp_cb = {0};

/* called in mgmt_thread in net_mgmt */
static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
							   struct net_if *iface)
{
	//const struct wifi_status *status = (const struct wifi_status *)cb->info;

	/* zephyr wifi todo, temp */
	switch (mgmt_event) {
	case NET_EVENT_IPV4_DHCP_BOUND:
		wifi_mgmt_raise_connect_result_event(iface, 0);
		break;
	default:
		break;
	}
}

/* for sema give to rtk_wifi_event_task */
int rtk_event_send_internal(int32_t event_id, void *event_data, size_t event_data_size,
							uint32_t ticks_to_wait)
{
	struct rtk_system_event evt;
	evt.event_id = event_id;

	if (event_data_size > sizeof(evt.event_info)) {
		LOG_ERR("MSG %d wont find %d > %d",
				event_id, event_data_size, sizeof(evt.event_info));
		return -EIO;
	}

	memcpy(&evt.event_info, event_data, event_data_size);
	k_msgq_put(&rtk_wifi_msgq, &evt, K_FOREVER);
}

static uint8_t rtk_wifi_getidx(const char *buf)
{
#if 0
	struct ethhdr etherhdr;
	int idx = STA_WLAN_INDEX;
	if (memcmp(dev_data->mac_addr[STA_WLAN_INDEX], &etherhdr.h_source, ETH_ALEN) == 0) {
		return STA_WLAN_INDEX;
	} else if (memcmp(dev_data->mac_addr[SOFTAP_WLAN_INDEX], &etherhdr.h_source, ETH_ALEN) == 0) {
		return SOFTAP_WLAN_INDEX;
	} else {
		return STA_WLAN_INDEX;
	}
#endif
	return STA_WLAN_INDEX;

}

int rtk_wifi_internal_tx(const char *buf, size_t buf_len)
{
	uint32_t wlan_idx = STA_WLAN_INDEX; //rtk_wifi_getidx(buf);
	int ret = 0;
	struct eth_drv_sg sg_list;
	sg_list.buf = (unsigned int)buf;
	sg_list.len = buf_len;

#ifndef ZEPHYR_WIFI_TODO
	/* zephyr wifi todo 这边目前buf copy了两次，第一次net_pkt_read到buf，第二次copy到skb, 待优化 */
#endif
	ret = inic_host_send(wlan_idx, &sg_list, 1, buf_len, NULL);
	return ret;
}


/* zephyr wifi todo, now for sta only */
static int rtk_wifi_send(const struct device *dev, struct net_pkt *pkt)
{

	struct rtk_wifi_runtime *data = dev->data;
	const int pkt_len = net_pkt_get_len(pkt);

	uint8_t *buf = rtos_mem_malloc(NET_ETH_MAX_FRAME_SIZE);
	/* Read the packet payload */
	if (net_pkt_read(pkt, buf, pkt_len) < 0) {
		goto out;
	}

	/* zephyr wifi todo 这边用两个frame buf是为了防止ethernet上面task交互, ap 和sta 打架, frame buf目前没有保护, 后面会拿掉, todo */
	if (rtk_wifi_internal_tx((void *)buf, pkt_len) != 0) {
		goto out;
	}

#if defined(CONFIG_NET_STATISTICS_WIFI)
	data->stats.bytes.sent += pkt_len;
	data->stats.pkts.tx++;
#endif
	k_free(buf);
	LOG_DBG("pkt sent %p len %d", pkt, pkt_len);
	return 0;

out:
	k_free(buf);

	LOG_ERR("Failed to send packet");
#if defined(CONFIG_NET_STATISTICS_WIFI)
	data->stats.errors.tx++;
#endif
	return -EIO;
}

static void dump_buf(char *info, uint8_t *buf, uint32_t len)
{
	DiagPrintf("%s", info);
	for (int i = 0; i < len; i++) {
		DiagPrintf("%s0x%02X%s", i % 16 == 0 ? "\n     " : ",",
				   buf[i], i == len - 1 ? "\n" : "");
	}
}

/* should called in driver after rx, can call rx_callback_ptr in driver */
static int eth_rtk_rx(uint8_t idx, void *buffer, uint16_t len)
{
	struct net_pkt *pkt;

	if (rtk_wifi_iface[idx] == NULL) {
		LOG_ERR("network interface unavailable");
		return -EIO;
	}
	//dump_buf("recv", buffer, len);

	pkt = net_pkt_rx_alloc_with_buffer(rtk_wifi_iface[idx], len, AF_UNSPEC, 0, K_MSEC(100));
	if (!pkt) {
		LOG_ERR("Failed to get net buffer");
		return -EIO;
	}

	if (net_pkt_write(pkt, buffer, len) < 0) {
		LOG_ERR("Failed to write pkt");
		goto pkt_unref;
	}
	if (net_recv_data(rtk_wifi_iface[idx], pkt) < 0) {

		printf("%s %d \r\n", __func__, __LINE__);
		LOG_ERR("Failed to push received data");
		goto pkt_unref;
	}

#if defined(CONFIG_NET_STATISTICS_WIFI)
	rtk_data.stats.bytes.received += len;
	rtk_data.stats.pkts.rx++;
#endif

	rtos_mem_free(buffer);

	return 0;

pkt_unref:
	net_pkt_unref(pkt);
	rtos_mem_free(buffer);
#if defined(CONFIG_NET_STATISTICS_WIFI)
	rtk_data.stats.errors.rx++;
#endif

	return -EIO;
}

static void scan_done_handler(unsigned int scanned_AP_num, void *user_data)
{
	struct wifi_scan_result res = { 0 };
	rtw_scan_result_t *scanned_AP_info;
	char *scan_buf = NULL;
	unsigned int i = 0;
	int ssid_len;

	/* scanned no AP*/
	if (scanned_AP_num == 0) {
		LOG_INF("No Wi-Fi AP found");
		goto out;
	}

	scan_buf = rtos_mem_zmalloc(scanned_AP_num * sizeof(rtw_scan_result_t));
	if (scan_buf == NULL) {
		LOG_INF("Failed to malloc buffer to print scan results");
		goto out;
	}

	if (wifi_get_scan_records(&scanned_AP_num, scan_buf) < 0) {
		LOG_INF("Unable to retrieve AP records");
		goto out;
	}

	//if (rtk_data.scan_cb) {
	for (i = 0; i < scanned_AP_num; i++) {
		scanned_AP_info = (rtw_scan_result_t *)(scan_buf + i * (sizeof(rtw_scan_result_t)));
		scanned_AP_info->SSID.val[scanned_AP_info->SSID.len] = 0; /* Ensure the SSID is null terminated */

		memset(&res, 0, sizeof(struct wifi_scan_result));
		ssid_len = scanned_AP_info->SSID.len;

		res.ssid_length = scanned_AP_info->SSID.len;
		strncpy(res.ssid, scanned_AP_info->SSID.val, ssid_len);
		res.rssi = scanned_AP_info->signal_strength;
		res.channel = scanned_AP_info->channel;
		res.security = WIFI_SECURITY_TYPE_NONE;
		if (scanned_AP_info->security > RTW_SECURITY_OPEN) {
			res.security = WIFI_SECURITY_TYPE_PSK;
		}

		print_scan_result(scanned_AP_info);
		//rtk_data.scan_cb(rtk_wifi_iface[STA_WLAN_INDEX], 0, &res);
		/* ensure notifications get delivered */
		k_yield();
	}
	//}

out:
	if (scan_buf) {
		k_free(scan_buf);
	}
	/* report end of scan event */
//	rtk_data.scan_cb(rtk_wifi_iface[STA_WLAN_INDEX], 0, NULL);	// callback in mgmt.c to inform upper
//	rtk_data.scan_cb = NULL;
}

static void rtk_wifi_handle_connect_event(void)
{
	if (IS_ENABLED(CONFIG_RTK_WIFI_STA_AUTO_DHCPV4)) {
		net_dhcpv4_start(rtk_wifi_iface[STA_WLAN_INDEX]);
	} else {
		wifi_mgmt_raise_connect_result_event(rtk_wifi_iface[STA_WLAN_INDEX], 0);
	}

	rtk_data.state = RTK_STA_CONNECTED;
}

static void rtk_wifi_handle_disconnect_event(void)
{
	if (rtk_data.state == RTK_STA_CONNECTED) {
		if (IS_ENABLED(CONFIG_RTK_WIFI_STA_AUTO_DHCPV4)) {
			net_dhcpv4_stop(rtk_wifi_iface[STA_WLAN_INDEX]);
		}
		wifi_mgmt_raise_disconnect_result_event(rtk_wifi_iface[STA_WLAN_INDEX], 0);
	} else {
		wifi_mgmt_raise_disconnect_result_event(rtk_wifi_iface[STA_WLAN_INDEX], -1);
	}

	/* zephyr wifi todo, Note: reconnect is no need for rtk, auto reconnect set in driver, remove note after reconnect test pass */
}

int rtk_wifi_connect_test(struct wifi_connect_req_params *params)
{
	int ret;
	memcpy(wifi.ssid.val, params->ssid, params->ssid_length);
	wifi.ssid.val[params->ssid_length] = '\0';
	wifi.ssid.len = params->ssid_length;

	//wifi.channel = params->channel;

	if (params->security == WIFI_SECURITY_TYPE_PSK) {
		memcpy(password, params->psk, params->psk_length);
		password[params->psk_length] = '\0';
		wifi.security_type = RTW_SECURITY_WPA2_AES_PSK;
		wifi.password = password;
	} else if (params->security == WIFI_SECURITY_TYPE_NONE) {
		wifi.security_type = RTW_SECURITY_OPEN;
		wifi.password = NULL;
	} else {
		LOG_ERR("Authentication method not supported");
		return -EIO;
	}

	if (params->channel) {
		wifi.channel = params->channel;
	}

	ret = wifi_connect(&wifi, 1);

	if (ret != RTW_SUCCESS) {
		LOG_ERR("Failed to connect to Wi-Fi access point");
		return -EAGAIN;
	} else {
		rtk_wifi_handle_connect_event();
		LOG_INF("assoc successed \r\n");
	}

	return 0;
}


static void rtk_wifi_event_task(void)
{
	struct rtk_system_event evt;
	uint8_t s_con_cnt = 0;

	while (1) {
		k_msgq_get(&rtk_wifi_msgq, &evt, K_FOREVER);

		switch (evt.event_id) {
		case RTK_WIFI_EVENT_STA_START:
			rtk_data.state = RTK_STA_STARTED;
			net_eth_carrier_on(rtk_wifi_iface[STA_WLAN_INDEX]);
			break;
		case RTK_WIFI_EVENT_STA_STOP:
			rtk_data.state = RTK_STA_STOPPED;
			net_eth_carrier_off(rtk_wifi_iface[STA_WLAN_INDEX]);
			break;
		case RTK_WIFI_EVENT_STA_CONNECTED:
			rtk_wifi_handle_connect_event();
			break;
		case RTK_WIFI_EVENT_STA_DISCONNECTED:
			rtk_wifi_handle_disconnect_event();
			break;
		case RTK_WIFI_EVENT_SCAN_DONE:
			//scan_done_handler();
			break;
		case RTK_WIFI_EVENT_AP_STOP:
			rtk_data.state = RTK_AP_STOPPED;
			net_eth_carrier_off(rtk_wifi_iface[SOFTAP_WLAN_INDEX]);
			break;
		case RTK_WIFI_EVENT_AP_STACONNECTED:
			rtk_data.state = RTK_AP_CONNECTED;
			s_con_cnt++;
			break;
		case RTK_WIFI_EVENT_AP_STADISCONNECTED:
			rtk_data.state = RTK_AP_DISCONNECTED;
			break;
		default:
			break;
		}
	}
}

static int rtk_wifi_disconnect(const struct device *dev)
{
	int ret = 0;

	if (wifi_is_connected_to_ap() != RTW_SUCCESS) {
		printf("\n\rnot connected yet");
		return -EALREADY;
	}

	ret = wifi_disconnect();
	return ret;

}

/* zephyr wifi todo, wep todo, no key idx in wifi_connect_req_params?? */
int rtk_wifi_connect(const struct device *dev, struct wifi_connect_req_params *params)
{
	struct rtk_wifi_runtime *data = dev->data;
	int ret;

	if (data->state == RTK_STA_CONNECTING || data->state == RTK_STA_CONNECTED) {
		wifi_mgmt_raise_connect_result_event(rtk_wifi_iface[STA_WLAN_INDEX], -1);
		return -EALREADY;
	}

	if (data->state != RTK_STA_STARTED) {
		LOG_ERR("Wi-Fi not in station mode");
		wifi_mgmt_raise_connect_result_event(rtk_wifi_iface[STA_WLAN_INDEX], -1);
		return -EIO;
	}

	data->state = RTK_STA_CONNECTING;

	memcpy(data->status.ssid, params->ssid, params->ssid_length);
	data->status.ssid[params->ssid_length] = '\0';

	memcpy(wifi.ssid.val, params->ssid, params->ssid_length);
	wifi.ssid.val[params->ssid_length] = '\0';

	wifi.channel = params->channel;

	if (params->security == WIFI_SECURITY_TYPE_PSK) {
		memcpy(password, params->psk, params->psk_length);
		password[params->psk_length] = '\0';
		wifi.security_type = RTW_SECURITY_WPA2_AES_PSK;
		wifi.password = password;
	} else if (params->security == WIFI_SECURITY_TYPE_NONE) {
		wifi.security_type = RTW_SECURITY_OPEN;
		wifi.password = NULL;
	} else {
		LOG_ERR("Authentication method not supported");
		return -EIO;
	}

	if (params->channel) {
		wifi.channel = params->channel;
	}

	ret = wifi_connect(&wifi, 1);

	if (ret != RTW_SUCCESS) {
		LOG_ERR("Failed to connect to Wi-Fi access point");
		return -EAGAIN;
	} else {
		rtk_wifi_handle_connect_event();
		LOG_INF("assoc successed \r\n");
	}

	return 0;
}

static int rtk_wifi_scan(const struct device *dev, struct wifi_scan_params *params,
						 scan_result_cb_t cb)
{
	struct rtk_wifi_runtime *data = dev->data;
	int ret = 0;
	rtw_join_status_t join_status = RTW_JOINSTATUS_UNKNOWN;

	if (data->scan_cb != NULL) {
		LOG_INF("Scan callback in progress");
		return -EINPROGRESS;
	}

	join_status = wifi_get_join_status();
	if ((join_status > RTW_JOINSTATUS_UNKNOWN) && (join_status < RTW_JOINSTATUS_SUCCESS)) {
		return -EINPROGRESS;
	}

	data->scan_cb = cb;

	if ((ret = wifi_scan_zephyr((void *)scan_done_handler)) != RTW_SUCCESS) {
		LOG_ERR("Failed  to start Wi-Fi scanning");
		return -EAGAIN;
	}


	return 0;
}

/* zephyr wifi todo, remove to driver later */
static int rtk_wifi_ap_enable(const struct device *dev, struct wifi_connect_req_params *params)
{
	struct rtk_wifi_runtime *data = dev->data;
	int ret = 0;
	uint32_t ip_addr;
	uint32_t netmask;
	uint32_t gw;

	/* Build Wi-Fi configuration for AP mode */
	memcpy(data->status.ssid, params->ssid, params->ssid_length);
	data->status.ssid[params->ssid_length] = '\0';

	strncpy((char *)ap.ssid.val, params->ssid, params->ssid_length);

	if (params->psk_length == 0) {
		ap.password_len = 0;
		ap.security_type = RTW_SECURITY_OPEN;
		//data->status.security = RTW_SECURITY_OPEN;
	} else {
		strncpy((char *) password, params->psk, params->psk_length);
		ap.password = password;
		ap.password_len = params->psk_length;
		ap.security_type = RTW_SECURITY_WPA2_AES_PSK;
		//data->status.security = RTW_SECURITY_WPA2_AES_PSK;
	}

	/* Start Wi-Fi in AP mode */
	wifi_stop_ap();

	if ((ret = wifi_start_ap(&ap)) < 0) {
		LOG_ERR("Failed to enable Wi-Fi AP mode");
		return -EAGAIN;
	}

	net_eth_carrier_on(rtk_wifi_iface[1]);

	return 0;
}

void rtk_wifi_ap_disable(const struct device *dev)
{
	(void) dev;

	wifi_stop_ap();

	/* zephyr wifi todo, need?? es32 no why? by sema? */
	net_eth_carrier_off(rtk_wifi_iface[1]);

}

/* zephyr wifi todo, 怎么从dev中获得iface的信息 */
static int rtk_wifi_status(const struct device *dev, struct wifi_iface_status *status)
{
	struct rtk_wifi_runtime *data = dev->data;
	//wifi_mode_t mode;
	//wifi_config_t conf;
	///wifi_ap_record_t ap_info;

	switch (data->state) {
	case RTK_STA_STOPPED:
	case RTK_AP_STOPPED:
		status->state = WIFI_STATE_INACTIVE;
		break;
	case RTK_STA_STARTED:
	case RTK_AP_DISCONNECTED:
		status->state = WIFI_STATE_DISCONNECTED;
		break;
	case RTK_STA_CONNECTING:
		status->state = WIFI_STATE_SCANNING;
		break;
	case RTK_STA_CONNECTED:
	case RTK_AP_CONNECTED:
		status->state = WIFI_STATE_COMPLETED;
		break;
	default:
		break;
	}

	/* zephyr wifi todo, get iface */
	strncpy(status->ssid, data->status.ssid, WIFI_SSID_MAX_LEN);
	status->ssid_len = strnlen(data->status.ssid, WIFI_SSID_MAX_LEN);
	status->band = WIFI_FREQ_BAND_2_4_GHZ; //todo
	status->link_mode = WIFI_LINK_MODE_UNKNOWN;
	status->mfp = WIFI_MFP_DISABLE;

#if 0
	switch (data->status.security) {
	case WIFI_AUTH_OPEN:
		status->security = WIFI_SECURITY_TYPE_NONE;
		break;
	case WIFI_AUTH_WPA2_PSK:
		status->security = WIFI_SECURITY_TYPE_PSK;
		break;
	default:
		status->security = WIFI_SECURITY_TYPE_UNKNOWN;
	}
#endif
	return 0;
}

typedef int (*wifi_rxcb_t)(uint8_t idx, void *buffer, uint16_t len);
extern int (*rx_callback_ptr)(uint8_t idx, void *buffer, uint16_t len);

static void rtk_wifi_internal_reg_rxcb(uint32_t idx, wifi_rxcb_t cb)
{
	rx_callback_ptr = cb;
}

static void rtk_wifi_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct rtk_wifi_runtime *dev_data = dev->data;
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);

	eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;
	rtk_wifi_iface[if_idx % NET_IF_MAX_CONFIGS] = iface;
	dev_data->state = RTK_STA_STOPPED;

	if (!init_done) {
		wlan_initialize();
		init_done = 1;
	}

	// TBD
	/* Start interface when we are actually connected with Wi-Fi network */
	wifi_get_mac_address(if_idx, (struct _rtw_mac_t *)dev_data->mac_addr[if_idx], 1);

	/* Assign link local address. */
	net_if_set_link_addr(iface, dev_data->mac_addr[if_idx], 6, NET_LINK_ETHERNET);

	ethernet_init(iface);
	net_if_carrier_off(iface);

	rtk_wifi_internal_reg_rxcb(0, eth_rtk_rx);

	DiagPrintf("addr: %02x:%02x:%02x:%02x:%02x:%02x \r\n", dev_data->mac_addr[0][0],
			   dev_data->mac_addr[0][1], dev_data->mac_addr[0][2], dev_data->mac_addr[0][3],
			   dev_data->mac_addr[0][4], dev_data->mac_addr[0][5]);

	/* seperate ap and sta */
	net_if_set_name(iface, iface_name[if_idx]);

	if_idx++;

}

static int rtk_wifi_dev_init(const struct device *dev)
{
	k_tid_t tid = k_thread_create(&rtk_wifi_event_thread, rtk_wifi_event_stack,
								  CONFIG_RTK_WIFI_EVENT_TASK_STACK_SIZE,
								  (k_thread_entry_t)rtk_wifi_event_task, NULL, NULL, NULL,
								  CONFIG_RTK_WIFI_EVENT_TASK_PRIO, K_INHERIT_PERMS,
								  K_NO_WAIT);

	k_thread_name_set(tid, "rtk_event");

	/* add event call back in net_mgmt */
	if (IS_ENABLED(CONFIG_RTK_WIFI_STA_AUTO_DHCPV4)) {
		net_mgmt_init_event_callback(&rtk_dhcp_cb, wifi_event_handler, DHCPV4_MASK);
		net_mgmt_add_event_callback(&rtk_dhcp_cb);
	}
	return 0;
}

static const struct wifi_mgmt_ops rtk_wifi_mgmt = {
	.scan		   = rtk_wifi_scan,
	.connect	   = rtk_wifi_connect,
	.disconnect	   = rtk_wifi_disconnect,
	.ap_enable	   = rtk_wifi_ap_enable,
	.ap_disable	   = rtk_wifi_ap_disable,
	.iface_status	   = rtk_wifi_status,
#if defined(CONFIG_NET_STATISTICS_WIFI)
	.get_stats	   = rtk_wifi_stats,
#endif
};

static const struct net_wifi_mgmt_offload rtk_api = {
	.wifi_iface.iface_api.init	  = rtk_wifi_init,
	.wifi_iface.send = rtk_wifi_send,
	.wifi_mgmt_api = &rtk_wifi_mgmt,
};

/* inst replace by DT_DRV_COMPAT(inst) */
NET_DEVICE_DT_INST_DEFINE(0,
						  rtk_wifi_dev_init, NULL,
						  &rtk_data, NULL, CONFIG_WIFI_INIT_PRIORITY,
						  &rtk_api, ETHERNET_L2,
						  NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);


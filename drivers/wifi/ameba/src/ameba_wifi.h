/*
 * Copyright (c) 2024 Realtek Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define WEP_ENABLED		0x0001		/**< wep enable */
#define TKIP_ENABLED		0x0002		/**< tkip enable */
#define AES_ENABLED		0x0004		/**< aes enable */
#define WSEC_SWFLAG		0x0008		/**< WSEC SWFLAG */
#define AES_CMAC_ENABLED	0x0010		/**< aes cmac enable */
#define ENTERPRISE_ENABLED	0x0020		/**< enterprise enable */
#define OWE_ENABLED		0X0040		/**< owe enable */
#define SHARED_ENABLED		0x00008000	/**< shared enable */
#define WPA_SECURITY		0x00200000	/**< wpa */
#define WPA2_SECURITY		0x00400000	/**< wpa2 */
#define WPA3_SECURITY		0x00800000	/**< wpa3 */
#define WPS_ENABLED		0x10000000	/**< wps  enable*/

#if defined(__IAR_SYSTEMS_ICC__) || defined (__GNUC__) || defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
/* SET pack mode 1-alignment for the following area. */
#pragma pack(1)
#endif

enum rtw_security {
	RTW_SECURITY_OPEN               = 0,                                                            /**< Open security                           */
	RTW_SECURITY_WEP_PSK            = (WEP_ENABLED),                                                /**< WEP Security with open authentication   */
	RTW_SECURITY_WEP_SHARED         = (WEP_ENABLED | SHARED_ENABLED),                               /**< WEP Security with shared authentication */

	RTW_SECURITY_WPA_TKIP_PSK       = (WPA_SECURITY | TKIP_ENABLED),                                /**< WPA Security with TKIP                  */
	RTW_SECURITY_WPA_AES_PSK        = (WPA_SECURITY | AES_ENABLED),                                 /**< WPA Security with AES                   */
	RTW_SECURITY_WPA_MIXED_PSK      = (WPA_SECURITY | AES_ENABLED | TKIP_ENABLED),                  /**< WPA Security with AES & TKIP            */
	RTW_SECURITY_WPA2_TKIP_PSK		= (WPA2_SECURITY | TKIP_ENABLED),								/**< WPA2 Security with TKIP				 */
	RTW_SECURITY_WPA2_AES_PSK       = (WPA2_SECURITY | AES_ENABLED),                                /**< WPA2 Security with AES                  */
	RTW_SECURITY_WPA2_MIXED_PSK     = (WPA2_SECURITY | AES_ENABLED | TKIP_ENABLED),                 /**< WPA2 Security with AES & TKIP           */
	RTW_SECURITY_WPA_WPA2_TKIP_PSK  = (WPA_SECURITY | WPA2_SECURITY | TKIP_ENABLED),                /**< WPA/WPA2 Security with TKIP             */
	RTW_SECURITY_WPA_WPA2_AES_PSK   = (WPA_SECURITY | WPA2_SECURITY | AES_ENABLED),                 /**< WPA/WPA2 Security with AES              */
	RTW_SECURITY_WPA_WPA2_MIXED_PSK = (WPA_SECURITY  | WPA2_SECURITY | TKIP_ENABLED | AES_ENABLED), /**< WPA/WPA2 Security with AES & TKIP       */
	RTW_SECURITY_WPA3_AES_PSK	 = (WPA3_SECURITY | AES_ENABLED),				/**< WPA3-SAE with AES security			   */
	RTW_SECURITY_WPA3_OWE	 = (WPA3_SECURITY | OWE_ENABLED | AES_ENABLED),				/**< WPA3-OWE with AES security			   */
	RTW_SECURITY_WPA2_WPA3_MIXED = (WPA2_SECURITY | WPA3_SECURITY | AES_ENABLED), /**< WPA3-SAE/WPA2 with AES security		   */
	RTW_SECURITY_WPA2_AES_CMAC      = (WPA2_SECURITY | AES_CMAC_ENABLED),                           /**< WPA2 Security with AES and Management Frame Protection */
};

struct _rtw_ssid_t {
	unsigned char		len;     /**< SSID length */
	unsigned char		val[33]; /**< SSID name (AP name)  */
};

struct _rtw_mac_t {
	unsigned char		octet[6]; /**< Unique 6-byte MAC address */
};

typedef void (*rtw_joinstatus_callback_t)(enum rtw_join_status_type join_status);

typedef struct _rtw_network_info_t {
	struct _rtw_ssid_t					ssid;
	struct _rtw_mac_t					bssid;
	uint32_t
	security_type;	/* because enum rtw_security type would occupy 8 bytes on PC/Raspi, so use u32 instead of enum to keep structure consistent */
	unsigned char				*password;
	int 						password_len;
	int 						key_id;
	unsigned char
	channel;		/**< set to 0 means full channel scan, set to other value means only scan on the specified channel */
	unsigned char
	pscan_option;	/**< used when the specified channel is set, set to 0 for normal partial scan, set to PSCAN_FAST_SURVEY for fast survey*/
	unsigned char 				is_wps_trigger;	/**< connection triggered by WPS process**/
	rtw_joinstatus_callback_t
	joinstatus_user_callback;	/**< user callback for processing joinstatus, please set to NULL if not use it */
	struct _rtw_mac_t		prev_bssid;
} rtw_network_info_t1;

typedef struct _rtw_softap_info_t {
	struct _rtw_ssid_t		ssid;
	unsigned char		hidden_ssid;
	enum rtw_security		security_type;
	unsigned char 		*password;
	unsigned char 		password_len;
	unsigned char		channel;
} rtw_softap_info_t1;

#ifndef STA_WLAN_INDEX
#define STA_WLAN_INDEX	0
#endif

#ifndef SOFTAP_WLAN_INDEX
#define SOFTAP_WLAN_INDEX	1
#endif

struct eth_drv_sg {
	unsigned int    buf;
	unsigned int     len;
};

enum rtw_bss_type {
	RTW_BSS_TYPE_INFRASTRUCTURE 	= 0, /**< Denotes infrastructure network                  */
	RTW_BSS_TYPE_ADHOC          		= 1, /**< Denotes an 802.11 ad-hoc IBSS network           */
	RTW_BSS_TYPE_ANY            			= 2, /**< Denotes either infrastructure or ad-hoc network */
	RTW_BSS_TYPE_UNKNOWN        		= 0xFFFFFFFF /**< May be returned by scan function if BSS type is unknown. Do not pass this to the Join function */
};

enum rtw_802_11_band {
	RTW_802_11_BAND_5GHZ   = 0, /**< Denotes 5GHz radio band */
	RTW_802_11_BAND_2_4GHZ = 1,  /**< Denotes 2.4GHz radio band */
	RTW_802_11_BAND_NOUSE = 0xFFFFFFFF
};

/**
  * @brief  The enumeration lists the WPS types.
  */
enum rtw_wps_type {
	RTW_WPS_TYPE_DEFAULT 		    		= 0x0000,	/**< default type */
	RTW_WPS_TYPE_USER_SPECIFIED 		= 0x0001,	/**< user specified type */
	RTW_WPS_TYPE_MACHINE_SPECIFIED   	= 0x0002,	/**< machine specified type */
	RTW_WPS_TYPE_REKEY 			        	= 0x0003,	/**< retry key type */
	RTW_WPS_TYPE_PUSHBUTTON 		    	= 0x0004,	/**< push button type */
	RTW_WPS_TYPE_REGISTRAR_SPECIFIED 	= 0x0005,	/**< register specified type */
	RTW_WPS_TYPE_NONE                   		= 0x0006, 	/**< none */
	RTW_WPS_TYPE_WSC                    		= 0x0007,	/**< wsc type */
	RTW_WPS_TYPE_NOUSE					= 0xffffffff		/**< unsed type */
};

typedef struct rtw_scan_result {
	struct _rtw_ssid_t
		SSID;             /**< Service Set Identification (i.e. Name of Access Point)                    */
	struct _rtw_mac_t
		BSSID;            /**< Basic Service Set Identification (i.e. MAC address of Access Point)       */
	signed short
	signal_strength;  /**< Receive Signal Strength Indication in dBm. <-90=Very poor, >-30=Excellent */
	enum rtw_bss_type
	bss_type;         /**< Network type                                                              */
	enum rtw_security
	security;         /**< Security type                                                             */
	enum rtw_wps_type
	wps_type;         /**< WPS type                                                                  */
	unsigned int
	channel;          /**< Radio channel that the AP beacon was received on                          */
	enum rtw_802_11_band
	band;             /**< Radio band                                                                */
} rtw_scan_result_t;

typedef enum _rtw_result_t {
	RTW_SUCCESS                      = 0,    /**< Success */
	RTW_TIMEOUT                      = 2,    /**< Timeout */
	RTW_INVALID_KEY                  = 4,        /**< Invalid key */

	RTW_ERROR                        = -1,   /**< Generic Error */
	RTW_BADARG                       = -2,   /**< Bad Argument */
	RTW_BUSY                         = -16,  /**< Busy */
	RTW_NOMEM                        = -27,  /**< No Memory */
} rtw_result_t;

typedef enum rtw_join_status_type rtw_join_status_t;
enum rtw_join_status_type {
	RTW_JOINSTATUS_UNKNOWN = 0,				/**< unknown */

	/* The intermediate states of Linking should be added
		in front of RTW_JOINSTATUS_SUCCESS */
	RTW_JOINSTATUS_STARTING,				/**< starting phase */
	RTW_JOINSTATUS_SCANNING,				/**< scanning phase */
	RTW_JOINSTATUS_AUTHENTICATING,			/**< authenticating phase */
	RTW_JOINSTATUS_AUTHENTICATED,			/**< authenticated phase */
	RTW_JOINSTATUS_ASSOCIATING,				/**< associating phase */
	RTW_JOINSTATUS_ASSOCIATED,			 	/**< associated phase */
	RTW_JOINSTATUS_4WAY_HANDSHAKING,		/**< 4 way handshaking phase */
	RTW_JOINSTATUS_4WAY_HANDSHAKE_DONE,	/**< 4 way handshake done phase */
	RTW_JOINSTATUS_SUCCESS,					/**< join success  */

	/* The other result states of Linking should be added
		in back of RTW_JOINSTATUS_SUCCESS */
	RTW_JOINSTATUS_FAIL,						/**< join fail  */
	RTW_JOINSTATUS_DISCONNECT,				/**< disconnect */
};

struct _rtw_channel_scan_time_t {
	unsigned short
	active_scan_time;      /**< active scan time per channel, units: millisecond, default is 100ms */
	unsigned short
	passive_scan_time;     /**< passive scan time per channel, units: millisecond, default is 110ms */
};

/**
  * @brief  The enumeration lists the scan options.
  */
enum rtw_scan_option {
	RTW_SCAN_NOUSE			= 0x00,  /**< default value */
	RTW_SCAN_ACTIVE              	= 0x01,     /**< active scan */
	RTW_SCAN_PASSIVE             	= 0x02,    /**< passive scan*/
	RTW_SCAN_NO_HIDDEN_SSID	= 0x04, /**< Filter hidden ssid APs*/
	RTW_SCAN_REPORT_EACH	= 0x08,    /**< report each */
	RTW_SCAN_WITH_P2P		= 0x10    /**< for P2P usage */
};

struct rtk_wifi_status {
	char ssid[WIFI_SSID_MAX_LEN + 1];
	char pass[WIFI_PSK_MAX_LEN + 1];
	//wifi_auth_mode_t security;
	bool connected;
	uint8_t channel;
	int rssi;
};

struct rtk_wifi_runtime {
	uint8_t mac_addr[2][6];
	uint8_t frame_buf[2][NET_ETH_MAX_FRAME_SIZE];
#if defined(CONFIG_NET_STATISTICS_WIFI)
	struct net_stats_wifi stats;
#endif
	struct rtk_wifi_status status;
	scan_result_cb_t scan_cb;
	uint8_t state;
};

typedef enum {
	SYSTEM_EVENT_WIFI_READY = 0, /*WiFi 准备好*/
	SYSTEM_EVENT_SCAN_DONE, /*扫描 AP 完成*/
	SYSTEM_EVENT_STA_START, /*作为 STA 开始工作*/
	SYSTEM_EVENT_STA_STOP, /*作为 STA 结束工作*/
	SYSTEM_EVENT_STA_CONNECTED, /*作为 STA 连接上 AP*/
	SYSTEM_EVENT_STA_DISCONNECTED, /*作为 STA 断开 AP*/

	RTK_WIFI_EVENT_STA_START,
	RTK_WIFI_EVENT_STA_STOP,
	RTK_WIFI_EVENT_STA_CONNECTED,
	RTK_WIFI_EVENT_STA_DISCONNECTED,
	RTK_WIFI_EVENT_SCAN_DONE,
	RTK_WIFI_EVENT_AP_STOP,
	RTK_WIFI_EVENT_AP_STACONNECTED,
	RTK_WIFI_EVENT_AP_STADISCONNECTED,
} system_event_id_t;

typedef struct {
	uint8_t bssid[6];       /*!< BSSID of the connected AP */
	uint8_t ssid[32];       /*!< SSID of the connected AP */
	uint8_t ssid_len;       /*!< Length of the SSID */
	uint8_t channel;        /*!< Channel of the connected AP */
	//wifi_auth_mode_t authmode;  /*!< Authentication mode of the connected AP */
} wifi_event_sta_connected_t;

typedef union {
	wifi_event_sta_connected_t sta_connected;  /*!< station connected to AP */
} system_event_info_t;

struct rtk_system_event {
	system_event_id_t event_id;
	system_event_info_t event_info;
};

enum rtk_state_flag {
	RTK_STA_STOPPED,
	RTK_STA_STARTED,
	RTK_STA_CONNECTING,
	RTK_STA_CONNECTED,
	RTK_AP_CONNECTED,
	RTK_AP_DISCONNECTED,
	RTK_AP_STOPPED,
};


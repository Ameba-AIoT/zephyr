/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>
#include <ameba_soc.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>

void z_arm_reset(void);

IMAGE2_ENTRY_SECTION
RAM_START_FUNCTION Img2EntryFun0 = {z_arm_reset, NULL, /* BOOT_RAM_WakeFromPG, */
				    (uint32_t)NewVectorTable};

static void app_vdd1833_detect(void)
{
#if AMEBAD_ZEPHYR_TODO

#endif
}

static int amebad_app_init(void)
{
#if AMEBAD_ZEPHYR_TODO
	/* Register IPC interrupt */
	IRQ_CONNECT(IPC_KM4_IRQ, INT_PRI_MIDDLE, IPC_INTHandler, (uint32_t)IPCKM4_DEV, 0);
	irq_enable(IPC_KM4_IRQ);

	/* IPC table initialization */
	ipc_table_init(IPCKM4_DEV);
#endif

	app_vdd1833_detect();
	return 0;
}

/* Disable KM0 Loguart Interrupt */
void shell_loguratRx_Ipc_Tx(u32 ipc_dir, u32 ipc_ch)
{
	IPC_MSG_STRUCT ipc_msg_temp;

	ipc_msg_temp.msg_type = IPC_USER_POINT;
	ipc_msg_temp.msg = 0;
	ipc_msg_temp.msg_len = 1;
	ipc_msg_temp.rsvd = 0; /* for coverity init issue */
	ipc_send_message(ipc_dir, ipc_ch, &ipc_msg_temp);
}

static int amebad_init(void)
{
	/*
	 * Cache is enabled by default at reset, disable it before
	 * sys_cache*-functions can enable them.
	 */
	Cache_Enable(DISABLE);
	sys_cache_data_enable();
	sys_cache_instr_enable();

	shell_loguratRx_Ipc_Tx((u32)NULL, IPC_INT_CHAN_SHELL_SWITCH);

#if AMEBAD_ZEPHYR_TODO

	/* do xtal/osc clk init */
	SystemCoreClockUpdate();

	XTAL_INIT();

	if (SYSCFG_CHIPType_Get() == CHIP_TYPE_ASIC_POSTSIM) { /* Only Asic need OSC Calibration */
		OSC4M_Init();
		OSC4M_Calibration(30000);
		if ((((BOOT_Reason()) & AON_BIT_RSTF_DSLP) == FALSE) &&
		    (RTCIO_IsEnabled() == FALSE)) {
			OSC131K_Calibration(30000); /* PPM=30000=3% */ /* 7.5ms */
		}
	}
#endif

	return 0;
}

SYS_INIT(amebad_init, PRE_KERNEL_1, 0);
SYS_INIT(amebad_app_init, APPLICATION, 0);

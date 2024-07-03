/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Realtek Ameba I2C interface
 */
#define DT_DRV_COMPAT realtek_ameba_i2c

#include <ameba_soc.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/clock_control/ameba_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2c_ameba, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

struct i2c_ameba_data {
	uint32_t master_mode;
	uint32_t addr_mode;
	uint32_t slave_address;
	struct i2c_msg *current;
	volatile int flag_done;
#if defined(CONFIG_I2C_AMEBA_INTERRUPT)
	I2C_IntModeCtrl i2c_intctrl;
#endif
};

typedef void (*irq_connect_cb)(void);

struct i2c_ameba_config {
	int index;
	I2C_TypeDef *I2Cx;
	uint32_t bitrate;
	const struct pinctrl_dev_config *pcfg;
	const clock_control_subsys_t clock_subsys;
	const struct device *clock_dev;
#if defined(CONFIG_I2C_AMEBA_INTERRUPT)
	void (*irq_cfg_func)(void);
#endif
};

#if defined(CONFIG_I2C_AMEBA_INTERRUPT)
struct k_sem txSemaphore;
struct k_sem rxSemaphore;

static void i2c_give_sema(u32 IsWrite)
{
	if (IsWrite) {
		k_sem_give(&txSemaphore);
	} else {
		k_sem_give(&rxSemaphore);
	}
}

static void i2c_take_sema(u32 IsWrite)
{
	if (IsWrite) {
		k_sem_take(&txSemaphore, K_FOREVER);
	} else {
		k_sem_take(&rxSemaphore, K_FOREVER);
	}
}
#endif

static void i2c_ameba_isr(const struct device *dev)
{
	struct i2c_ameba_data *data = dev->data;

	uint32_t intr_status = I2C_GetINT(data->i2c_intctrl.I2Cx);

	if (intr_status & I2C_BIT_R_STOP_DET) {
		data->flag_done = 1;
		/* Clear I2C interrupt */
		I2C_ClearINT(data->i2c_intctrl.I2Cx, I2C_BIT_R_STOP_DET);
	}

	I2C_ISRHandle(&data->i2c_intctrl);
}

static int i2c_ameba_configure(const struct device *dev, uint32_t dev_config)
{
	I2C_InitTypeDef I2C_InitStruct;

	struct i2c_ameba_data *data = dev->data;
	const struct i2c_ameba_config *config = dev->config;

	I2C_TypeDef *i2c = config->I2Cx;
	int err = 0;

	/* Disable I2C device */
	I2C_Cmd(i2c, DISABLE);

	I2C_StructInit(&I2C_InitStruct);

	if (dev_config & I2C_MODE_CONTROLLER) {
		I2C_InitStruct.I2CMaster = I2C_MASTER_MODE;
	} else {
		I2C_InitStruct.I2CMaster = I2C_SLAVE_MODE;
	}

	if (dev_config & I2C_ADDR_10_BITS) {
		I2C_InitStruct.I2CAddrMod = I2C_ADDR_10BIT;
	} else {
		I2C_InitStruct.I2CAddrMod = I2C_ADDR_7BIT;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		I2C_InitStruct.I2CSpdMod = I2C_SS_MODE;
		I2C_InitStruct.I2CClk = (I2C_BITRATE_STANDARD / 1000);
		break;
	case I2C_SPEED_FAST:
		I2C_InitStruct.I2CSpdMod = I2C_FS_MODE;
		I2C_InitStruct.I2CClk = (I2C_BITRATE_FAST / 1000);
		break;
	case I2C_SPEED_HIGH:
		I2C_InitStruct.I2CSpdMod = I2C_HS_MODE;
		I2C_InitStruct.I2CClk = (I2C_BITRATE_HIGH / 1000);
		break;
	default:
		err = -EINVAL;
		goto error;
	}

	data->master_mode = I2C_InitStruct.I2CMaster;
	data->addr_mode = I2C_InitStruct.I2CAddrMod;

	I2C_Init(i2c, &I2C_InitStruct);

	I2C_Cmd(i2c, ENABLE);

error:
	return err;
}

static int i2c_ameba_transfer(const struct device *dev, struct i2c_msg *msgs,
							  uint8_t num_msgs, uint16_t addr)
{
	struct i2c_ameba_data *data = dev->data;
	const struct i2c_ameba_config *config = dev->config;

	I2C_TypeDef *i2c = config->I2Cx;
	struct i2c_msg *current, *next;
	int err = 0;

	current = msgs;

	/* First message flags implicitly contain I2C_MSG_RESTART flag. */
	current->flags |= I2C_MSG_RESTART;
	/*restart check for write or read dirction*/
	for (uint8_t i = 1; i <= num_msgs; i++) {
		if (i < num_msgs) {
			next = current + 1;

			/*
			* If there have a R/W transfer state change between messages,
			* An explicit I2C_MSG_RESTART flag is needed for the second message.
			*/
			if ((current->flags & I2C_MSG_RW_MASK) !=
				(next->flags & I2C_MSG_RW_MASK)) {
				if ((next->flags & I2C_MSG_RESTART) == 0U) {
					return -EINVAL;
				}
			}

			/* Only the last message need I2C_MSG_STOP flag to free the Bus. */
			if (current->flags & I2C_MSG_STOP) {
				return -EINVAL;
			}
		} else {
			/* Last message flags implicitly contain I2C_MSG_STOP flag. */
			current->flags |= I2C_MSG_STOP;
		}

		if ((current->buf == NULL) ||
			(current->len == 0U)) {
			return -EINVAL;
		}

		current++;
	}

	/* Enable i2c device */
	I2C_Cmd(i2c, ENABLE);

	I2C_SetSlaveAddress(i2c, addr);
	data->slave_address = addr;

#if defined(CONFIG_I2C_AMEBA_INTERRUPT)
	for (uint8_t i = 0; i < num_msgs; ++i) {
		data->current = &msgs[i];
		if (data->master_mode == 1) {
			data->flag_done = 0;
			I2C_INTConfig(i2c, I2C_BIT_R_STOP_DET, ENABLE);

			if ((data->current->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
				I2C_MasterWriteInt(i2c, &data->i2c_intctrl, data->current->buf,  data->current->len);
				while (data->flag_done == 0);
			} else {
				I2C_MasterReadInt(i2c,   &data->i2c_intctrl, data->current->buf,  data->current->len);
				while (data->flag_done == 0);
			}
		}
	}
#else
	for (uint8_t i = 0; i < num_msgs; ++i) {
		k_sleep(K_MSEC(5));
		data->current = &msgs[i];
		if (data->master_mode == 1) {
			if ((data->current->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
				I2C_MasterWrite(i2c,  data->current->buf,  data->current->len);
			} else {
				I2C_MasterRead(i2c,  data->current->buf,  data->current->len);
			}
		} else {
			if ((data->current->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
				I2C_SlaveWrite(i2c,  data->current->buf,  data->current->len);
			} else {
				I2C_SlaveRead(i2c,  data->current->buf,  data->current->len);
			}
		}
	}
#endif
	/* Disable I2C device */
	I2C_Cmd(i2c, DISABLE);

	return err;
}

static const struct i2c_driver_api i2c_ameba_driver_api = {
	.configure = i2c_ameba_configure,
	.transfer = i2c_ameba_transfer,
};

static int i2c_ameba_init(const struct device *dev)
{
	struct i2c_ameba_data *data = dev->data;
	const struct i2c_ameba_config *config = dev->config;

	uint32_t bitrate_cfg;
	int err = 0;

	/*clk enable*/
	clock_control_on(config->clock_dev, (clock_control_subsys_t)config->clock_subsys);
	/* Configure pinmux  */
	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

#if defined(CONFIG_I2C_AMEBA_INTERRUPT)
	k_sem_init(&txSemaphore, 1, 1);
	k_sem_init(&rxSemaphore, 1, 1);

	k_sem_take(&txSemaphore, K_FOREVER);
	k_sem_take(&rxSemaphore, K_FOREVER);

	data->i2c_intctrl.I2Cx = config->I2Cx;
	data->i2c_intctrl.I2CSendSem =  i2c_give_sema;
	data->i2c_intctrl.I2CWaitSem = i2c_take_sema;
	config->irq_cfg_func();
#endif

	bitrate_cfg = i2c_map_dt_bitrate(config->bitrate);
	i2c_ameba_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);

	return 0;
}


#define AMEBA_I2C_INIT(n)						   \
    PINCTRL_DT_INST_DEFINE(n);  \
    static void i2c_ameba_irq_cfg_func_##n(void)             \
    {  \
          IRQ_CONNECT(DT_INST_IRQN(n),        \
                    DT_INST_IRQ(n, priority),       \
                    i2c_ameba_isr,                   \
                    DEVICE_DT_INST_GET(n),              \
                    0);                         \
        irq_enable(DT_INST_IRQN(n));        \
    } \
												   \
	static struct i2c_ameba_data i2c_ameba_data_##n; \
												   \
	static const struct i2c_ameba_config i2c_ameba_config_##n = {				   \
		.index = n,		\
        .I2Cx=(I2C_TypeDef *)DT_INST_REG_ADDR(n), 							   \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),          \
        .bitrate = DT_INST_PROP(n, clock_frequency),          \
        .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                     \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, idx),    \
        .irq_cfg_func = i2c_ameba_irq_cfg_func_##n,          \
	};					\
						\
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_ameba_init, NULL, &i2c_ameba_data_##n, \
			     &i2c_ameba_config_##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,	   \
			     &i2c_ameba_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMEBA_I2C_INIT)
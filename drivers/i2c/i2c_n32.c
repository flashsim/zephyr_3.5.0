/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/dt-bindings/pinctrl/n32-pinctrl.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>

#include <n32g45x_i2c.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

// #include <soc.h>



#include <zephyr/sys/sys_io.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/init.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/linker/sections.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT nations_n32_i2c


LOG_MODULE_REGISTER(i2c_n32, CONFIG_I2C_LOG_LEVEL);

// #include "i2c-priv.h"

struct i2c_n32_config
{
    uint32_t reg_base;
    uint32_t apb_clk_en;
    uint32_t i2c_bitrate;
    //uint32_t cctl_offset;
    //uint32_t cctl_mask;
    const struct pinctrl_dev_config *pinctrl_config;
    void (*irq_config_func)(void);
};

struct i2c_n32_data
{
    struct k_sem bus_mutex;
    struct k_sem sync_sem;
    uint32_t dev_config;
    struct i2c_msg *msgs;
    uint32_t msg_num;
    uint32_t msg_idx;
    uint32_t buf_idx;
    
    uint16_t slave_addr;
    
};



#define I2CT_FLAG_TIMEOUT ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT ((uint32_t)(20 * I2CT_FLAG_TIMEOUT)) ///((uint32_t)(20 * I2C_FLAG_TIMOUT))



/**
 *  @brief      The function of this API is to ensure that the data can be successfully sent out.
 *  @param[in]  id - to set the slave ID.for kite slave ID=0x5c,for eagle slave ID=0x5a.
 *  @param[in]  data - The data to be sent, The first three bytes can be set as the RAM address of the slave.
 *  @param[in]  len - This length is the total length, including both the length of the slave RAM address and the length of the data to be sent.
 * @retval 0 successfully
 * @retval other fail
 */
// static int i2c_master_write(const struct device *dev, uint8_t id, const uint8_t *data, uint8_t len)
// {
    // const struct i2c_n32_config *cfg = dev->config;
    // I2C_Module *i2c_inst = (I2C_Module *)cfg->reg_base;
    // const uint8_t *ptr = data;
    // uint32_t timeout;

    // timeout = I2CT_LONG_TIMEOUT;

    // while (I2C_GetFlag(i2c_inst, I2C_FLAG_BUSY))
    // {
        // if ((timeout--) == 0)
        // {
            // return 4;
        // }
    // }

    // I2C_ConfigAck(i2c_inst, ENABLE);

    // I2C_GenerateStart(i2c_inst, ENABLE);

    // timeout = I2C_FLAG_TIMOUT;

    // while (!I2C_CheckEvent(i2c_inst, I2C_EVT_MASTER_MODE_FLAG)) // EV5
    // {
        // if ((timeout--) == 0)
        // {
            // return 5;
        // }
    // }

    // I2C_SendAddr7bit(i2c_inst, id, I2C_DIRECTION_SEND);

    // timeout = I2C_FLAG_TIMOUT;

    // while (!I2C_CheckEvent(i2c_inst, I2C_EVT_MASTER_TXMODE_FLAG)) // EV6
    // {
        // if ((timeout--) == 0)
        // {
            // return 6;
        // }
    // }

    // send data
    // while (len-- > 0)
    // {
        // I2C_SendData(i2c_inst, *ptr++);
        // timeout = I2C_FLAG_TIMOUT;

        // while (!I2C_CheckEvent(i2c_inst, I2C_EVT_MASTER_DATA_SENDING)) // EV8
        // {
            // if ((timeout--) == 0)
            // {
                // return 7;
            // }
        // }
    // }

    // timeout = I2C_FLAG_TIMOUT;

    // while (!I2C_CheckEvent(i2c_inst, I2C_EVT_MASTER_DATA_SENDED)) // EV8-2
    // {
        // if ((timeout--) == 0)
        // {
            // return 8;
        // }
    // }

    // I2C_GenerateStop(i2c_inst, ENABLE);

    // return 0;
// }

/**
 * @brief      This function serves to read a packet of data from the specified address of slave device
 * @param[in]  id - to set the slave ID.for kite slave ID=0x5c,for eagle slave ID=0x5a.
 * @param[in|out]  data - Store the read data
 * @param[in]  len - The total length of the data read back.
 * @retval 0 successfully
 * @retval other fail
 */
// static int i2c_master_read(const struct device *dev, uint8_t id, uint8_t *data, uint8_t len)
// {
    // const struct i2c_n32_config *cfg = dev->config;
    // I2C_Module *i2c_inst = (I2C_Module *)cfg->reg_base;
    // uint8_t *ptr = data;
    // uint32_t timeout;

    // timeout = I2CT_LONG_TIMEOUT;

    // while (I2C_GetFlag(i2c_inst, I2C_FLAG_BUSY))
    // {
        // if ((timeout--) == 0)
        // {
            // return 9;
        // }
    // }

    // I2C_ConfigAck(i2c_inst, ENABLE);

    // send start
    // I2C_GenerateStart(i2c_inst, ENABLE);

    // timeout = I2C_FLAG_TIMOUT;

    // while (!I2C_CheckEvent(i2c_inst, I2C_EVT_MASTER_MODE_FLAG)) // EV5
    // {
        // if ((timeout--) == 0)
        // {
            // return 10;
        // }
    // }

    // send addr
    // I2C_SendAddr7bit(i2c_inst, id, I2C_DIRECTION_RECV);

    // timeout = I2C_FLAG_TIMOUT;

    // while (!I2C_CheckEvent(i2c_inst, I2C_EVT_MASTER_RXMODE_FLAG)) // EV6
    // {
        // if ((timeout--) == 0)
        // {
            // return 6;
        // }
    // }

    // recv data
    // while (len-- > 0)
    // {
        // timeout = I2CT_LONG_TIMEOUT;

        // while (!I2C_CheckEvent(i2c_inst, I2C_EVT_MASTER_DATA_RECVD_FLAG)) // EV7
        // {
            // if ((timeout--) == 0)
            // {
                // return 14;
            // }
        // }

        // if (len == 1)
        // {
            // I2C_ConfigAck(I2C1, DISABLE);
            // I2C_GenerateStop(I2C1, ENABLE);
        // }

        // if (len == 0)
        // {
            // I2C_GenerateStop(I2C1, ENABLE);
        // }

        // *ptr++ = I2C_RecvData(I2C1);
    // }

    // return 0;
// }





static int i2c_n32_configure(const struct device *dev, uint32_t dev_config)
{
    const struct i2c_n32_config *cfg = dev->config;
    struct i2c_n32_data *data = dev->data;
    
    k_sem_take(&data->bus_mutex, K_FOREVER);
    
    I2C_DeInit((I2C_Module *)cfg->reg_base);
    
    I2C_InitType i2c_init = {
        .BusMode     = I2C_BUSMODE_I2C,
        .FmDutyCycle = I2C_FMDUTYCYCLE_2,
        .AckEnable   = I2C_ACKEN,
        .AddrMode    = I2C_ADDR_MODE_7BIT        
    };

    /* check i2c speed */
    i2c_init.ClkSpeed = dev_config;
    

    /* init i2c */
    I2C_Init((I2C_Module *)cfg->reg_base, &i2c_init);
    I2C_Enable((I2C_Module *)cfg->reg_base, ENABLE);
    
       /* release the mutex */
    k_sem_give(&data->bus_mutex);

    return 0;
}

static void n32_i2c_event_isr(const struct device *dev)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *config = dev->config;
	I2C_Module *i2c_port = (I2C_Module*)config->reg_base;

    unsigned int last_event = I2C_GetLastEvent(i2c_port);
    
    if ((last_event & I2C_ROLE_MASTER) == I2C_ROLE_MASTER) // master mode
    {
        switch (last_event)
        {
            case I2C_EVT_MASTER_MODE_FLAG: // 0x00030001.EV5 start
                if (data->msgs->flags & I2C_MSG_READ) {    // read
                
                     uint8_t flags = data->msgs->flags ;
                     uint32_t len = data->msgs->len ;
                     
                    //Memset(rx_buf, 0, TEST_BUFFER_SIZE); // clear recv buf, ready to recv data
                    I2C_SendAddr7bit(i2c_port, data->slave_addr, I2C_DIRECTION_RECV);
                    //rx_num = 0;
                }
                else  { // write
                    I2C_SendAddr7bit(i2c_port, data->slave_addr, I2C_DIRECTION_SEND);
                    //tx_num = 0;
                }
                break;
            case I2C_EVT_MASTER_TXMODE_FLAG: // 0x00070082.EV8 justafter EV6
                //Comm_Flag = C_READY;
                //I2C_SendData(i2c_port, tx_buf[tx_num++]);
                I2C_SendData(i2c_port,  *data->msgs->buf);
                data->msgs->buf++;
                data->msgs->len--;
                break;
            case I2C_EVT_MASTER_DATA_SENDING: // 0x00070080. transmitting data
                //if (tx_num < TEST_BUFFER_SIZE)
                if (data->msgs->len > 0)
                {
                    //I2C_SendData(i2c_port, tx_buf[tx_num++]);
                    I2C_SendData(i2c_port, *data->msgs->buf);
                    data->msgs->buf++;
                    data->msgs->len--;
                }
                break;
            case I2C_EVT_MASTER_DATA_SENDED: // 0x00070084.byte data send finish
                                             // bit2    BSF (Byte transfer finished)
                //if (tx_num == TEST_BUFFER_SIZE)  // data send finish
                if (data->msgs->len == 0)  // data send finish
                {
                    //if (Comm_Flag == C_READY)
                    //{
                    //    Comm_Flag = C_STOP_BIT;
                        I2C_GenerateStop(i2c_port, ENABLE);
                    //    flag_master_send_finish = 1;
                    //}
                    k_sem_give(&data->sync_sem);
                }
                break;
                // Master Receiver
            case I2C_EVT_MASTER_RXMODE_FLAG: // 0x00030002.EV6
                //Comm_Flag = C_READY;
                //if (TEST_BUFFER_SIZE == 1)
                if (data->msgs->len == 1)
                {
                    I2C_ConfigAck(i2c_port, DISABLE);
                    //if (Comm_Flag == C_READY)
                    //{
                    //    Comm_Flag = C_STOP_BIT;
                        I2C_GenerateStop(i2c_port, ENABLE);
                    //}
                    k_sem_give(&data->sync_sem);
                }
                //else if (TEST_BUFFER_SIZE == 2)
                else if (data->msgs->len  == 2)
                {
                    i2c_port->CTRL1 |= I2C_NACK_POS_NEXT; /// set ACKPOS
                    I2C_ConfigAck(i2c_port, DISABLE);
                    k_sem_give(&data->sync_sem);
                }
                break;
            case I2C_EVT_MASTER_DATA_RECVD_FLAG: // one byte recved :EV7.//BUSY,MSL and RXDATNE flags.
            case I2C_EVT_MASTER_SFT_DATA_RECVD_FLAG: // BUSY, MSMODE(Master) and Data register not empty, BSF(Byte transfer finished)flags.
                //rx_buf[rx_num++] = I2C_RecvData(i2c_port);
                *data->msgs->buf = I2C_RecvData(i2c_port);
                data->msgs->buf++;
                data->msgs->len--;
                
                //if (rx_num == (TEST_BUFFER_SIZE - 1))
                if (data->msgs->len == 1)
                {
                    I2C_ConfigAck(i2c_port, DISABLE);   // Disable I2C acknowledgement.
                    //if (Comm_Flag == C_READY)
                    {
                    //    Comm_Flag = C_STOP_BIT;
                        I2C_GenerateStop(i2c_port, ENABLE); // Send I2C STOP Condition.
                        k_sem_give(&data->sync_sem);
                    }
                }
                //else if (rx_num == TEST_BUFFER_SIZE)
                else if (data->msgs->len == 0)
                {
                    k_sem_give(&data->sync_sem);
                    //flag_master_recv_finish = 1;
                }
                break;
            case 0x00030201: // Arbitration lost
            case 0x00030401: // Acknowledge failure
            case 0x00030501: // Acknowledge failure and Bus error
                I2C_Enable(i2c_port, DISABLE);
                I2C_Enable(i2c_port, ENABLE);
                break;
            default:
                //log_info("I2C error status:0x%x\r\n", last_event);
                break;
        }
    }
}


static void n32_i2c_error_isr(const struct device *dev)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *config = dev->config;
    I2C_Module *i2c_port = (I2C_Module*)config->reg_base;
    
    
	// uint32_t stat;

	// stat = I2C_STAT0(cfg->reg);

	// if (stat & I2C_STAT0_BERR) {
		// I2C_STAT0(cfg->reg) &= ~I2C_STAT0_BERR;
		// data->errs |= I2C_GD32_ERR_BERR;
	// }

	// if (stat & I2C_STAT0_LOSTARB) {
		// I2C_STAT0(cfg->reg) &= ~I2C_STAT0_LOSTARB;
		// data->errs |= I2C_GD32_ERR_LARB;
	// }

	// if (stat & I2C_STAT0_AERR) {
		// I2C_STAT0(cfg->reg) &= ~I2C_STAT0_AERR;
		// data->errs |= I2C_GD32_ERR_AERR;
	// }

	// if (data->errs != 0U) {
		// /* Enter stop condition */
		// I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;

		// k_sem_give(&data->sync_sem);
	// }
    

    
    if(I2C_GetFlag(i2c_port, I2C_FLAG_ACKFAIL))
    {
        I2C_ClrFlag(i2c_port, I2C_FLAG_ACKFAIL);
        I2C_GenerateStop(i2c_port, ENABLE); // Send I2C1 STOP Condition.
        k_sem_give(&data->sync_sem);
    }
    
}


static int i2c_n32_transfer_end(const struct device *dev)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *cfg = dev->config;

	I2C_ConfigInt(I2C1, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, DISABLE);

	/* Wait for stop condition is done. */
	while (I2C_GetFlag(I2C1, I2C_FLAG_BUSY)) {
		/* NOP */
	}

	// if (data->errs) {
		// return -EIO;
	// }

	return 0;
}


static int i2c_n32_msg_read(const struct device *dev, struct i2c_msg *msg,
			                uint8_t *next_msg_flags, uint16_t slave_addr)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *cfg = dev->config;
    I2C_Module *i2c_port = (I2C_Module *)cfg->reg_base;
    //I2C_TypeDef *i2c = cfg->i2c;

	// if (I2C_STAT1(cfg->reg) & I2C_STAT1_I2CBSY) {
		// data->errs = I2C_GD32_ERR_BUSY;
		// return -EBUSY;
	// }

	// i2c_gd32_xfer_begin(dev);

	// k_sem_take(&data->sync_sem, K_FOREVER);

	// return i2c_gd32_xfer_end(dev);
    
    k_sem_reset(&data->sync_sem);
    
    I2C_ConfigInt(I2C1, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, ENABLE); // interrupt enable
 
    
    //I2C_TypeDef *i2c = cfg->i2c;
    I2C_ConfigPecLocation(i2c_port, I2C_PEC_POS_CURRENT);
    
    if (msg->flags & I2C_MSG_RESTART) {
		I2C_GenerateStart(i2c_port, ENABLE);
	}

    k_sem_take(&data->sync_sem, K_FOREVER);

    return i2c_n32_transfer_end(dev);
}

static int i2c_n32_msg_write(const struct device *dev, struct i2c_msg *msg,
			                 uint8_t *next_msg_flags, uint16_t saddr)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *config = dev->config;
    I2C_Module *i2c_port = (I2C_Module *)config->reg_base;

	// if (I2C_STAT1(reg_base) & I2C_STAT1_I2CBSY) {
		// data->errs = I2C_GD32_ERR_BUSY;
		// return -EBUSY;
	// }

	// i2c_gd32_xfer_begin(dev);

	// k_sem_take(&data->sync_sem, K_FOREVER);

	// return i2c_gd32_xfer_end(dev);
    
    k_sem_reset(&data->sync_sem);
    I2C_ConfigInt(I2C1, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, ENABLE); // interrupt enable
    
    I2C_ConfigPecLocation(i2c_port, I2C_PEC_POS_CURRENT);
    
    if (msg->flags & I2C_MSG_RESTART) {
		I2C_GenerateStart(i2c_port, ENABLE);
	}
    
    k_sem_take(&data->sync_sem, K_FOREVER);

	return i2c_n32_transfer_end(dev);
   
}

static int i2c_n32_transfer(const struct device *dev, struct i2c_msg *msgs,
                            uint8_t num_msgs, uint16_t slave_addr)
{
    //int status = 0;
    //uint8_t send_stop = 0;
    struct i2c_n32_data *data = dev->data;
    struct i2c_msg *current, *next;
    int ret = 0;

	current = msgs;

	/* First message flags set I2C_MSG_RESTART flag. */
	current->flags |= I2C_MSG_RESTART;

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
			/* Last message flags contain I2C_MSG_STOP flag. */
			current->flags |= I2C_MSG_STOP;
		}

		if ((current->buf == NULL) ||
		    (current->len == 0U)) {
			return -EINVAL;
		}

		current++;
	}

    k_sem_take(&data->bus_mutex, K_FOREVER);

    current = msgs;
    uint8_t *next_msg_flags = NULL;

    for (int i = 0; i < num_msgs; i++)
    {
        data->msgs = &msgs[i];
        data->slave_addr = slave_addr;
		//data->len = msgs[i].len;
 
        
        if (i > 0) {
            next = current + 1;
            next_msg_flags = &(next->flags);
        }
        //ret = stm32_i2c_transaction(dev, *current, next_msg_flags, slave);
        
        if ((current->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) 
        {
            ret = i2c_n32_msg_write(dev, current, next_msg_flags, slave_addr);
        } 
        else {
            
            ret = i2c_n32_msg_read(dev, current, next_msg_flags, slave_addr);
        }
        
        
        if (ret < 0) {
            break;
        }
        current++;
    }

    /* release the mutex */
    k_sem_give(&data->bus_mutex);

    return 0;
};

/* API implementation: init */
static int i2c_n32_init(const struct device *dev)
{
    int status = 0;

    const struct i2c_n32_config *config = dev->config;
    struct i2c_n32_data *data = dev->data;
    //uint32_t dev_config = (I2C_MODE_MASTER); // | i2c_map_dt_bitrate(cfg->bitrate));
    
    /* Mutex semaphore to protect the i2c api in multi-thread env. */
    k_sem_init(&data->bus_mutex, 1, 1);
    
    /* Sync semaphore to sync i2c state between isr and transfer api. */
	k_sem_init(&data->sync_sem, 0, K_SEM_MAX_LIMIT);
    
    int ret = pinctrl_apply_state(config->pinctrl_config, PINCTRL_STATE_DEFAULT);
    if(ret != 0)
    {
        LOG_ERR("Failed to configure I2C clock");
        return ret;
    }

    ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)), (clock_control_subsys_t *)(&config->apb_clk_en));
    
    if (ret < 0) {
        return -EIO;
    }
    
    
    uint32_t bitrate_cfg = config->i2c_bitrate;;

    status = i2c_n32_configure(dev, bitrate_cfg);
    if (status != 0)
    {
        LOG_ERR("Failed to configure I2C on init");
        return status;
    }

    config->irq_config_func();
    
    return 0;
}

static const struct i2c_driver_api i2c_n32_api = {
    .configure = i2c_n32_configure,
    .transfer = i2c_n32_transfer,
};

 


#define N32_I2C_INIT(n)        \
                               \
    PINCTRL_DT_INST_DEFINE(n); \
    static void n32_i2c_irq_config_##n(void) \
    { \
        IRQ_CONNECT( \
            DT_INST_IRQ_BY_NAME(n, event, irq),    \
            DT_INST_IRQ_BY_NAME(n, event, priority),\
            n32_i2c_event_isr,     \
            DEVICE_DT_INST_GET(n), \
            0); \
        irq_enable(DT_INST_IRQ_BY_NAME(n, event, irq)); \
                                                    \
        IRQ_CONNECT(                                \
            DT_INST_IRQ_BY_NAME(n, error, irq),	    \
			DT_INST_IRQ_BY_NAME(n, error, priority),\
			n32_i2c_error_isr,			\
			DEVICE_DT_INST_GET(n), 0);		\
		irq_enable(DT_INST_IRQ_BY_NAME(n, error, irq));	\
    };  \
        \
    static struct i2c_n32_data i2c_n32_dev_data_##n;  \
    const static  struct i2c_n32_config i2c_n32_dev_cfg_##n = {  \
        .reg_base   = DT_INST_REG_ADDR(n), \
        .apb_clk_en = DT_INST_CLOCKS_CELL(n, bits),	\
        .i2c_bitrate = DT_INST_PROP(n, clock_frequency),	\
        .pinctrl_config  = PINCTRL_DT_INST_DEV_CONFIG_GET(n),  \
        .irq_config_func = n32_i2c_irq_config_##n \
    };  \
        \
    I2C_DEVICE_DT_INST_DEFINE(n, \
                              i2c_n32_init,              \
                              NULL,                      \
                              &i2c_n32_dev_data_##n,     \
                              &i2c_n32_dev_cfg_##n,      \
                              POST_KERNEL,               \
                              CONFIG_I2C_INIT_PRIORITY,  \
                              &i2c_n32_api);

DT_INST_FOREACH_STATUS_OKAY(N32_I2C_INIT);



/*
 * SPDX-License-IdentifINTE: Apache-2.0
 */


#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>


#include <n32g45x.h>


#define DT_DRV_COMPAT nations_n32_bxcan

LOG_MODULE_REGISTER(can_n32, CONFIG_CAN_LOG_LEVEL);

#define SP_IS_SET(n) DT_INST_NODE_HAS_PROP(n, sample_point) ||

#define USE_SP_ALGO (DT_INST_FOREACH_STATUS_OKAY(SP_IS_SET) 0)


#define CAN_N32_MAX_FILTER_ID   (CONFIG_CAN_MAX_EXTENDED_ID_FILTER + CONFIG_CAN_MAX_STANDARD_ID_FILTER * 2)

#define CAN_N32_NUM_FILTER_BANKS (14)

#define CAN_INIT_TIMEOUT  (10 * (sys_clock_hw_cycles_per_sec() / MSEC_PER_SEC))
/* Time out for INAK bit */
#define INIAK_TIMEOUT ((uint32_t)0x0000FFFF)


#define CAN_N32_FIRX_STD_IDE_POS   (3U)
#define CAN_N32_FIRX_STD_RTR_POS   (4U)
#define CAN_N32_FIRX_STD_ID_POS    (5U)

#define CAN_N32_FIRX_EXT_IDE_POS    (2U)
#define CAN_N32_FIRX_EXT_RTR_POS    (1U)
#define CAN_N32_FIRX_EXT_STD_ID_POS (21U)
#define CAN_N32_FIRX_EXT_EXT_ID_POS (3U)


struct can_n32_mailbox {
    can_tx_callback_t tx_callback;
    void *callback_arg;
};


#define CAN1  DT_NODELABEL(can1)


static int can_n32_leave_init_mode(CAN_Module *can);
static int can_n32_enter_init_mode(CAN_Module *can);
static void can_n32_signal_tx_complete(const struct device *dev, struct can_n32_mailbox *mb,
                     int status);
static int can_n32_get_state(const struct device *dev, enum can_state *state,
                               struct can_bus_err_cnt *err_cnt);



struct can_n32_data 
{
    struct k_mutex inst_mutex;
    struct k_sem tx_allocs_sem;
    enum can_state state;
    struct can_n32_mailbox mb0;
    struct can_n32_mailbox mb1;
    struct can_n32_mailbox mb2;
    can_rx_callback_t rx_cb_std[CONFIG_CAN_MAX_STANDARD_ID_FILTER];
    can_rx_callback_t rx_cb_ext[CONFIG_CAN_MAX_EXTENDED_ID_FILTER];
    
    void *cb_arg_std[CONFIG_CAN_MAX_STANDARD_ID_FILTER];
    void *cb_arg_ext[CONFIG_CAN_MAX_EXTENDED_ID_FILTER];
    void *state_change_cb_user_data;
    can_state_change_callback_t state_change_cb;
    bool started;
};




struct can_n32_config {

    CAN_Module *can;
    CAN_Module *master_can;
    uint32_t sjw;
    uint8_t prop_ts1;
    uint8_t ts2;
    uint32_t can_apb_clk_en;
    uint32_t bus_speed;
    uint16_t sample_point;
    const struct device *phy;
    uint32_t max_bitrate;
    const struct pinctrl_dev_config *pin_cfg;
    void (*irq_config_func)(CAN_Module *can);
    
};



static struct k_mutex filter_mutex;



static void can_n32_rx_fifo_pop(CAN_FIFOMailBox_Param *mbox, struct can_frame *frame)
{
    memset(frame, 0, sizeof(*frame));

    if (mbox->RMI & CAN_RMI0_IDE) {
        frame->id = mbox->RMI >> 3;
        frame->flags |= CAN_FRAME_IDE;
    } else {
        frame->id = mbox->RMI >> 21;
    }

    if ((mbox->RMI & CAN_RMI0_RTRQ) != 0) {
        frame->flags |= CAN_FRAME_RTR;
    } else {
        frame->data_32[0] = mbox->RMDL;
        frame->data_32[1] = mbox->RMDH;
    }

    frame->dlc = mbox->RMDT & (CAN_RMDT0_DLC);
#ifdef CONFIG_CAN_RX_TIMESTAMP
    frame->timestamp = ((mbox->RMDT & CAN_RDT0R_TIME) >> CAN_RDT0R_TIME_Pos);
#endif
}


static inline void can_n32_tx_isr_handler(const struct device *dev)
{
    struct can_n32_data *data = dev->data;
    const struct can_n32_config *cfg = dev->config;
    CAN_Module *can = cfg->can;
    uint32_t bus_off;
    int status;

    bus_off = can->ESTS & CAN_ESTS_BOFFL;

    if ((can->TSTS & CAN_TSTS_RQCPM0) | bus_off) {
        status = can->TSTS & CAN_TSTS_TXOKM0 ? 0  :
             can->TSTS & CAN_TSTS_TERRM0 ? -EIO :
             can->TSTS & CAN_TSTS_ALSTM0 ? -EBUSY :    bus_off ? -ENETUNREACH : -EIO;
        /* clear the request. */
        can->TSTS |= CAN_TSTS_RQCPM0;
        can_n32_signal_tx_complete(dev, &data->mb0, status);
    }

    if ((can->TSTS & CAN_TSTS_RQCPM1) | bus_off) {
        status = can->TSTS & CAN_TSTS_TXOKM1 ? 0  :
             can->TSTS & CAN_TSTS_TERRM1 ? -EIO :
             can->TSTS & CAN_TSTS_ALSTM1 ? -EBUSY :    bus_off  ? -ENETUNREACH :  -EIO;
        /* clear the request. */
        can->TSTS |= CAN_TSTS_RQCPM1;
        can_n32_signal_tx_complete(dev, &data->mb1, status);
    }

    if ((can->TSTS & CAN_TSTS_RQCPM2) | bus_off) {
        status = can->TSTS & CAN_TSTS_TXOKM2 ? 0  :
             can->TSTS & CAN_TSTS_TERRM2 ? -EIO :
             can->TSTS & CAN_TSTS_ALSTM2 ? -EBUSY :    bus_off ? -ENETUNREACH : -EIO;
        /* clear the request. */
        can->TSTS |= CAN_TSTS_RQCPM2;
        can_n32_signal_tx_complete(dev, &data->mb2, status);
    }

    if (can->TSTS & CAN_TSTS_TMEM) {
        k_sem_give(&data->tx_allocs_sem);
    }
}


static inline void can_n32_rx_isr_handler(const struct device *dev)
{
    struct can_n32_data *data = dev->data;
    const struct can_n32_config *cfg = dev->config;
    CAN_Module *can = cfg->can;
    CAN_FIFOMailBox_Param *mbox;
    int filter_id, index;
    struct can_frame frame;
    can_rx_callback_t callback = NULL;
    void *cb_arg;

    while (can->RFF0 & CAN_RFF0_FFMP0) {
        mbox = &can->sFIFOMailBox[0];
        filter_id = ((mbox->RMDT & CAN_RMDT0_FMI) >> 8);

        LOG_DBG("Message on filter_id %d", filter_id);

        can_n32_rx_fifo_pop(mbox, &frame);

        if (filter_id < CONFIG_CAN_MAX_EXTENDED_ID_FILTER) {
            callback = data->rx_cb_ext[filter_id];
            cb_arg = data->cb_arg_ext[filter_id];
        } else if (filter_id < CAN_N32_MAX_FILTER_ID) {
            index = filter_id - CONFIG_CAN_MAX_EXTENDED_ID_FILTER;
            callback = data->rx_cb_std[index];
            cb_arg = data->cb_arg_std[index];
        }

        if (callback) {
            callback(dev, &frame, cb_arg);
        }

        /* Release message */
        can->RFF0 |= CAN_RFF0_RFFOM0;
    }

    if (can->RFF0 & CAN_RFF0_FFOVR0) {
        LOG_ERR("RX FIFO Overflow");
        CAN_STATS_RX_OVERRUN_INC(dev);
    }
}


static inline void can_n32_bus_state_change_isr(const struct device *dev)
{
    struct can_n32_data *data = dev->data;
    struct can_bus_err_cnt err_cnt;
    enum can_state state;
    const can_state_change_callback_t cb = data->state_change_cb;
    void *state_change_cb_user_data = data->state_change_cb_user_data;

#ifdef CONFIG_CAN_STATS
    const struct can_n32_config *cfg = dev->config;
    CAN_TypeDef *can = cfg->can;

    switch (can->ESTS & CAN_ESTS_LEC) {
    case (CAN_ESTS_LEC_0):
        CAN_STATS_STUFF_ERROR_INC(dev);
        break;
    case (CAN_ESTS_LEC_1):
        CAN_STATS_FORM_ERROR_INC(dev);
        break;
    case (CAN_ESTS_LEC_1 | CAN_ESTS_LEC_0):
        CAN_STATS_ACK_ERROR_INC(dev);
        break;
    case (CAN_ESTS_LEC_2):
        CAN_STATS_BIT1_ERROR_INC(dev);
        break;
    case (CAN_ESTS_LEC_2 | CAN_ESTS_LEC_0):
        CAN_STATS_BIT0_ERROR_INC(dev);
        break;
    case (CAN_ESTS_LEC_2 | CAN_ESR_LEC_1):
        CAN_STATS_CRC_ERROR_INC(dev);
        break;
    default:
        break;
    }

    /* Clear last error code flag */
    can->ESTS |= CAN_ESTS_LEC;
#endif /* CONFIG_CAN_STATS */

    (void)can_n32_get_state(dev, &state, &err_cnt);

    if (state != data->state) {
        data->state = state;

        if (cb != NULL) {
            cb(dev, state, err_cnt, state_change_cb_user_data);
        }
    }
}



static void can_n32_rx_isr(const struct device *dev)
{
    can_n32_rx_isr_handler(dev);
}

static void can_n32_tx_isr(const struct device *dev)
{
    can_n32_tx_isr_handler(dev);
}

static void can_n32_state_change_isr(const struct device *dev)
{
    const struct can_n32_config *cfg = dev->config;
    CAN_Module *can = cfg->can;

    /* Signal bus-off to waiting tx */
    if (can->MSTS & CAN_MSTS_ERRINT) {
        can_n32_tx_isr_handler(dev);
        can_n32_bus_state_change_isr(dev);
        can->MSTS |= CAN_MSTS_ERRINT;
    }
}



static int can_n32_get_capabilities(const struct device *dev, can_mode_t *cap)
{
    ARG_UNUSED(dev);

    *cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_ONE_SHOT;

    return 0;
}


static int can_n32_start(const struct device *dev)
{
    const struct can_n32_config *cfg = dev->config;
    struct can_n32_data *data = dev->data;
    CAN_Module *can = cfg->can;
    int ret = 0;

    k_mutex_lock(&data->inst_mutex, K_FOREVER);

    if (data->started) {
        ret = -EALREADY;
        goto unlock;
    }

    if (cfg->phy != NULL) {
        ret = can_transceiver_enable(cfg->phy);
        if (ret != 0) {
            LOG_ERR("failed to enable CAN transceiver (err %d)", ret);
            goto unlock;
        }
    }

    CAN_STATS_RESET(dev);

    __NOP();

    ret = can_n32_leave_init_mode(can);
    if (ret < 0) {
        LOG_ERR("Failed to leave init mode");

        if (cfg->phy != NULL) {
            /* Attempt to disable the CAN transceiver in case of error */
            (void)can_transceiver_disable(cfg->phy);
        }

        ret = -EIO;
        goto unlock;
    }

    data->started = true;

unlock:
    k_mutex_unlock(&data->inst_mutex);

    return ret;
}



static int can_n32_stop(const struct device *dev)
{
    const struct can_n32_config *cfg = dev->config;
    struct can_n32_data *data = dev->data;
    CAN_Module *can = cfg->can;
    int ret = 0;

    k_mutex_lock(&data->inst_mutex, K_FOREVER);

    if (!data->started) {
        ret = -EALREADY;
        goto unlock;
    }

    ret = can_n32_enter_init_mode(can);
    if (ret < 0) {
        LOG_ERR("Failed to enter init mode");
        ret = -EIO;
        goto unlock;
    }

    /* Abort any pending transmissions */
    can_n32_signal_tx_complete(dev, &data->mb0, -ENETDOWN);
    can_n32_signal_tx_complete(dev, &data->mb1, -ENETDOWN);
    can_n32_signal_tx_complete(dev, &data->mb2, -ENETDOWN);
    
    can->TSTS |= CAN_TSTS_ABRQM2 | CAN_TSTS_ABRQM1 | CAN_TSTS_ABRQM0;
    
    
    

    if (cfg->phy != NULL) {
        ret = can_transceiver_disable(cfg->phy);
        if (ret != 0) {
            LOG_ERR("failed to enable CAN transceiver (err %d)", ret);
            goto unlock;
        }
    }

    data->started = false;

unlock:
    k_mutex_unlock(&data->inst_mutex);

    return ret;
}


static void can_n32_signal_tx_complete(const struct device *dev, struct can_n32_mailbox *mb,
                     int status)
{
    can_tx_callback_t callback = mb->tx_callback;

    if (callback != NULL) {
        callback(dev, status, mb->callback_arg);
        mb->tx_callback = NULL;
    }
}



static int can_n32_set_mode(const struct device *dev, can_mode_t mode)
{
    const struct can_n32_config *cfg = dev->config;
    CAN_Module *can = cfg->can;
    struct can_n32_data *data = dev->data;

    LOG_DBG("Set mode %d", mode);

    if ((mode & ~(CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_ONE_SHOT)) != 0) {
        LOG_ERR("unsupported mode: 0x%08x", mode);
        return -ENOTSUP;
    }

    if (data->started) {
        return -EBUSY;
    }

    k_mutex_lock(&data->inst_mutex, K_FOREVER);

    if ((mode & CAN_MODE_LOOPBACK) != 0) {
        /* Loopback mode */
        can->BTIM |= CAN_BTIM_LBM;
    } else {
        can->BTIM &= ~CAN_BTIM_LBM;
    }

    if ((mode & CAN_MODE_LISTENONLY) != 0) {
        /* Silent mode */
        can->BTIM |= CAN_BTIM_SLM;
    } else {
        can->BTIM &= ~CAN_BTIM_SLM;
    }

    if ((mode & CAN_MODE_ONE_SHOT) != 0) {
        /* No automatic retransmission */
        can->MCTRL |= CAN_MCTRL_NART;
    } else {
        can->MCTRL &= ~CAN_MCTRL_NART;
    }

    k_mutex_unlock(&data->inst_mutex);

    return 0;
}


static int can_n32_set_timing(const struct device *dev,  const struct can_timing *timing)
{
    const struct can_n32_config *cfg = dev->config;
    CAN_Module *can = cfg->can;
    struct can_n32_data *data = dev->data;

    k_mutex_lock(&data->inst_mutex, K_FOREVER);

    if (data->started) {
        k_mutex_unlock(&data->inst_mutex);
        return -EBUSY;
    }

    can->BTIM = (can->BTIM & ~(CAN_BTIM_RSJW | CAN_BTIM_BRTP |
                 CAN_BTIM_TBS1 | CAN_BTIM_TBS2)) |
         (((timing->sjw        - 1) << 24) & CAN_BTIM_RSJW) |
         (((timing->phase_seg1 - 1) << 16) & CAN_BTIM_TBS1) |
         (((timing->phase_seg2 - 1) << 20) & CAN_BTIM_TBS2) |
         (((timing->prescaler  - 1) << 0) & CAN_BTIM_BRTP);

    k_mutex_unlock(&data->inst_mutex);

    return 0;
}


static int can_n32_send(const struct device *dev, const struct can_frame *frame,
                         k_timeout_t timeout, can_tx_callback_t callback,
                        void *user_data)
{
    const struct can_n32_config *cfg = dev->config;
    struct can_n32_data *data = dev->data;
    CAN_Module *can = cfg->can;
    uint32_t transmit_status_register = can->TSTS;
    CAN_TxMailBox_Param *mailbox = NULL;
    
    
    struct can_n32_mailbox *mb = NULL;

    LOG_DBG("Sending %d bytes on %s. "
            "Id: 0x%x, "
            "ID type: %s, "
            "Remote Frame: %s"
            , frame->dlc, dev->name
            , frame->id
            , (frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard"
            , (frame->flags & CAN_FRAME_RTR) != 0 ? "yes" : "no");

    __ASSERT_NO_MSG(callback != NULL);
    __ASSERT(frame->dlc == 0U || frame->data != NULL, "Dataptr is null");

    if (frame->dlc > CAN_MAX_DLC) {
        LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
        return -EINVAL;
    }

    if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0) {
        LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
        return -ENOTSUP;
    }

    if (!data->started) {
        return -ENETDOWN;
    }

    if (can->ESTS & CAN_ESTS_BOFFL) {
        return -ENETUNREACH;
    }

    k_mutex_lock(&data->inst_mutex, K_FOREVER);
    while (!(transmit_status_register & CAN_TSTS_TMEM)) {
        k_mutex_unlock(&data->inst_mutex);
        LOG_DBG("Transmit buffer full");
        if (k_sem_take(&data->tx_allocs_sem, timeout)) {
            return -EAGAIN;
        }

        k_mutex_lock(&data->inst_mutex, K_FOREVER);
        transmit_status_register = can->TSTS;
    }

    if (transmit_status_register & CAN_TSTS_TMEM0) {
        LOG_DBG("Using TX mailbox 0");
        mailbox = &can->sTxMailBox[0];
        mb = &(data->mb0);
    } else if (transmit_status_register & CAN_TSTS_TMEM1) {
        LOG_DBG("Using TX mailbox 1");
        mailbox = &can->sTxMailBox[1];
        mb = &data->mb1;
    } else if (transmit_status_register & CAN_TSTS_TMEM2) {
        LOG_DBG("Using TX mailbox 2");
        mailbox = &can->sTxMailBox[2];
        mb = &data->mb2;
    }

    mb->tx_callback = callback;
    mb->callback_arg = user_data;

    /* mailbox identifINTE register setup */
    mailbox->TMI &= CAN_TMI0_TXRQ;

    if ((frame->flags & CAN_FRAME_IDE) != 0) {
        mailbox->TMI |= (frame->id << 3) | CAN_TMI0_IDE;
    } else {
        mailbox->TMI |= (frame->id << 21);
    }

    if ((frame->flags & CAN_FRAME_RTR) != 0) {
        mailbox->TMI |= CAN_TMI1_RTRQ;
    } else {
        mailbox->TMDL = frame->data_32[0];
        mailbox->TMDH = frame->data_32[1];
    }

    mailbox->TMDT = (mailbox->TMDT & ~CAN_TMDT0_DLC) |
            ((frame->dlc & 0xF));

    mailbox->TMI |= CAN_TMI0_TXRQ;
    k_mutex_unlock(&data->inst_mutex);

    return 0;
}



static void can_n32_set_filter_bank(int filter_id, CAN_FilterRegister_Param *filter_reg,
                      bool ide, uint32_t id, uint32_t mask)
{
    if (ide) {
        filter_reg->FR1 = id;
        filter_reg->FR2 = mask;
    } else {
        if ((filter_id - CONFIG_CAN_MAX_EXTENDED_ID_FILTER) % 2 == 0) {
            /* even std filter id: first 1/2 bank */
            filter_reg->FR1 = id | (mask << 16);
        } else {
            /* uneven std filter id: first 1/2 bank */
            filter_reg->FR2 = id | (mask << 16);
        }
    }
}


static inline uint32_t can_n32_filter_to_std_mask(const struct can_filter *filter)
{
    uint32_t rtr_mask = !IS_ENABLED(CONFIG_CAN_ACCEPT_RTR);

    return  (filter->mask << CAN_N32_FIRX_STD_ID_POS) |
        (rtr_mask << CAN_N32_FIRX_STD_RTR_POS) |
        (1U << CAN_N32_FIRX_STD_IDE_POS);
}

static inline uint32_t can_n32_filter_to_ext_mask(const struct can_filter *filter)
{
    uint32_t rtr_mask = !IS_ENABLED(CONFIG_CAN_ACCEPT_RTR);

    return  (filter->mask << CAN_N32_FIRX_EXT_EXT_ID_POS) |
        (rtr_mask << CAN_N32_FIRX_EXT_RTR_POS) |
        (1U << CAN_N32_FIRX_EXT_IDE_POS);
}

static inline uint32_t can_n32_filter_to_std_id(const struct can_filter *filter)
{
    return  (filter->id  << CAN_N32_FIRX_STD_ID_POS);
}

static inline uint32_t can_n32_filter_to_ext_id(const struct can_filter *filter)
{
    return  (filter->id << CAN_N32_FIRX_EXT_EXT_ID_POS) |
        (1U << CAN_N32_FIRX_EXT_IDE_POS);
}

static inline int can_n32_set_filter(const struct device *dev, const struct can_filter *filter)
{
    const struct can_n32_config *cfg = dev->config;
    struct can_n32_data *data = dev->data;
    CAN_Module *can = cfg->master_can;
    uint32_t mask = 0U;
    uint32_t id = 0U;
    int filter_id = -ENOSPC;
    int bank_offset = 0;
    int bank_num;

    if (cfg->can != cfg->master_can) {
        /* CAN slave instance: start with offset */
        bank_offset = CAN_N32_NUM_FILTER_BANKS;
    }

    if ((filter->flags & CAN_FILTER_IDE) != 0) {
        for (int i = 0; i < CONFIG_CAN_MAX_EXTENDED_ID_FILTER; i++) {
            if (data->rx_cb_ext[i] == NULL) {
                id = can_n32_filter_to_ext_id(filter);
                mask = can_n32_filter_to_ext_mask(filter);
                filter_id = i;
                bank_num = bank_offset + i;
                break;
            }
        }
    } else {
        for (int i = 0; i < CONFIG_CAN_MAX_STANDARD_ID_FILTER; i++) {
            if (data->rx_cb_std[i] == NULL) {
                id = can_n32_filter_to_std_id(filter);
                mask = can_n32_filter_to_std_mask(filter);
                filter_id = CONFIG_CAN_MAX_EXTENDED_ID_FILTER + i;
                bank_num = bank_offset + CONFIG_CAN_MAX_EXTENDED_ID_FILTER + i / 2;
                break;
            }
        }
    }

    if (filter_id != -ENOSPC) {
        LOG_DBG("Adding filter_id %d, CAN ID: 0x%x, mask: 0x%x",
            filter_id, filter->id, filter->mask);

        /* set the filter init mode */
        can->FMC |= CAN_FMC_FINITM;

        can_n32_set_filter_bank(filter_id, &can->sFilterRegister[bank_num],
                      (filter->flags & CAN_FILTER_IDE) != 0,
                      id, mask);

        can->FA1 |= 1U << bank_num;
        can->FMC &= ~(CAN_FMC_FINITM);
    } else {
        LOG_WRN("No free filter left");
    }

    return filter_id;
}


static int can_n32_add_rx_filter(const struct device *dev, can_rx_callback_t cb,
                      void *cb_arg, const struct can_filter *filter)
{
    struct can_n32_data *data = dev->data;
    int filter_id;

    if ((filter->flags & ~(CAN_FILTER_IDE)) != 0) {
        LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
        return -ENOTSUP;
    }

    k_mutex_lock(&filter_mutex, K_FOREVER);
    k_mutex_lock(&data->inst_mutex, K_FOREVER);

    filter_id = can_n32_set_filter(dev, filter);
    if (filter_id >= 0) {
        if ((filter->flags & CAN_FILTER_IDE) != 0) {
            data->rx_cb_ext[filter_id] = cb;
            data->cb_arg_ext[filter_id] = cb_arg;
        } else {
            data->rx_cb_std[filter_id - CONFIG_CAN_MAX_EXTENDED_ID_FILTER] = cb;
            data->cb_arg_std[filter_id - CONFIG_CAN_MAX_EXTENDED_ID_FILTER] = cb_arg;
        }
    }

    k_mutex_unlock(&data->inst_mutex);
    k_mutex_unlock(&filter_mutex);

    return filter_id;
}



static void can_n32_remove_rx_filter(const struct device *dev, int filter_id)
{
    const struct can_n32_config *cfg = dev->config;
    struct can_n32_data *data = dev->data;
    CAN_Module *can = cfg->master_can;
    bool ide;
    int bank_offset = 0;
    int bank_num;
    bool bank_unused;

    if (filter_id < 0 || filter_id >= CAN_N32_MAX_FILTER_ID) {
        LOG_ERR("filter ID %d out of bounds", filter_id);
        return;
    }

    k_mutex_lock(&filter_mutex, K_FOREVER);
    k_mutex_lock(&data->inst_mutex, K_FOREVER);

    if (cfg->can != cfg->master_can) {
        bank_offset = CAN_N32_NUM_FILTER_BANKS;
    }

    if (filter_id < CONFIG_CAN_MAX_EXTENDED_ID_FILTER) {
        ide = true;
        bank_num = bank_offset + filter_id;

        data->rx_cb_ext[filter_id] = NULL;
        data->cb_arg_ext[filter_id] = NULL;

        bank_unused = true;
    } else {
        int filter_index = filter_id - CONFIG_CAN_MAX_EXTENDED_ID_FILTER;

        ide = false;
        bank_num = bank_offset + CONFIG_CAN_MAX_EXTENDED_ID_FILTER +
              (filter_id - CONFIG_CAN_MAX_EXTENDED_ID_FILTER) / 2;

        data->rx_cb_std[filter_index] = NULL;
        data->cb_arg_std[filter_index] = NULL;

        if (filter_index % 2 == 1) {
            bank_unused = data->rx_cb_std[filter_index - 1] == NULL;
        } else if (filter_index + 1 < CONFIG_CAN_MAX_STANDARD_ID_FILTER) {
            bank_unused = data->rx_cb_std[filter_index + 1] == NULL;
        } else {
            bank_unused = true;
        }
    }

    LOG_DBG("Removing filter_id %d, ide %d", filter_id, ide);

    can->FMC |= CAN_FM1_FB0;

    can_n32_set_filter_bank(filter_id, &can->sFilterRegister[bank_num],
                  ide, 0, 0xFFFFFFFF);

    if (bank_unused) {
        can->FS1 &= ~(1U << bank_num);
        LOG_DBG("Filter bank %d is unused -> deactivate", bank_num);
    }

    can->FMC &= ~(CAN_FM1_FB0);

    k_mutex_unlock(&data->inst_mutex);
    k_mutex_unlock(&filter_mutex);
}



static int can_n32_get_state(const struct device *dev, enum can_state *state,
                               struct can_bus_err_cnt *err_cnt)
{
    const struct can_n32_config *cfg = dev->config;
    struct can_n32_data *data = dev->data;
    CAN_Module *can = cfg->can;

    if (state != NULL) {
        if (!data->started) {
            *state = CAN_STATE_STOPPED;
        } else if (can->ESTS & CAN_ESTS_BOFFL) {
            *state = CAN_STATE_BUS_OFF;
        } else if (can->ESTS & CAN_ESTS_EPVFL) {
            *state = CAN_STATE_ERROR_PASSIVE;
        } else if (can->ESTS & CAN_ESTS_EWGFL) {
            *state = CAN_STATE_ERROR_WARNING;
        } else {
            *state = CAN_STATE_ERROR_ACTIVE;
        }
    }

    if (err_cnt != NULL) {
        err_cnt->tx_err_cnt =
            ((can->ESTS & CAN_ESTS_TXEC) >> 16);
        err_cnt->rx_err_cnt =
            ((can->ESTS & CAN_ESTS_RXEC) >> 24);
    }

    return 0;
}



#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
static int can_n32_recover(const struct device *dev, k_timeout_t timeout)
{
    const struct can_n32_config *cfg = dev->config;
    struct can_n32_data *data = dev->data;
    CAN_Module *can = cfg->can;
    int ret = -EAGAIN;
    int64_t start_time;

    if (!data->started) {
        return -ENETDOWN;
    }

    if (!(can->ESTS & CAN_ESTS_BOFFL)) {
        return 0;
    }

    if (k_mutex_lock(&data->inst_mutex, K_FOREVER)) {
        return -EAGAIN;
    }

    ret = can_n32_enter_init_mode(can);
    if (ret) {
        goto done;
    }

    can_n32_leave_init_mode(can);

    start_time = k_uptime_ticks();

    while (can->ESTS & CAN_ESTS_BOFFL) {
        if (!K_TIMEOUT_EQ(timeout, K_FOREVER) &&
            k_uptime_ticks() - start_time >= timeout.ticks) {
            goto done;
        }
    }

    ret = 0;

done:
    k_mutex_unlock(&data->inst_mutex);
    return ret;
}
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */



static void can_n32_set_state_change_callback(const struct device *dev,
                        can_state_change_callback_t callback,
                        void *user_data)
{
    struct can_n32_data *data = dev->data;
    const struct can_n32_config *cfg = dev->config;
    CAN_Module *can = cfg->can;

    data->state_change_cb = callback;
    data->state_change_cb_user_data = user_data;

    if (callback == NULL) {
        can->INTE &= ~(CAN_INTE_BOFITE | CAN_INTE_EPVITE | CAN_INTE_EWGITE);
    } else {
        can->INTE |= CAN_INTE_BOFITE | CAN_INTE_EPVITE | CAN_INTE_EWGITE;
    }
}




static int can_n32_get_core_clock(const struct device *dev, uint32_t *rate)
{
    const struct can_n32_config *cfg = dev->config;
    const struct device *clock;
    int ret = 0;

    clock = DEVICE_DT_GET(DT_NODELABEL(rcc));
    
    
    struct n32_clock_control_subsys {
        uint32_t rcc_offset;
        uint32_t mask;
    } subsys;
    
    subsys.rcc_offset  = cfg->can_apb_clk_en;

    ret = clock_control_get_rate(clock, (clock_control_subsys_t)&subsys, rate);

 
    if (ret != 0) {
        LOG_ERR("Failed call clock_control_get_rate: return [%d]", ret);
        return -EIO;
    }

    return 0;
}


static int can_n32_get_max_bitrate(const struct device *dev, uint32_t *max_bitrate)
{
    const struct can_n32_config *config = dev->config;

    *max_bitrate = config->max_bitrate;

    return 0;
}



static int can_n32_get_max_filters(const struct device *dev, bool ide)
{
    ARG_UNUSED(dev);

    if (ide) {
        return CONFIG_CAN_MAX_EXTENDED_ID_FILTER;
    } else {
        return CONFIG_CAN_MAX_STANDARD_ID_FILTER;
    }
}



static int can_n32_enter_init_mode(CAN_Module *can)
{
    uint32_t start_time;

    can->MCTRL |= CAN_MCTRL_INIRQ;
    start_time = k_cycle_get_32();

    while ((can->MSTS & CAN_MSTS_INIAK) == 0U) {
        if (k_cycle_get_32() - start_time > CAN_INIT_TIMEOUT) {
            can->MCTRL &= ~CAN_MCTRL_INIRQ;
            return -EAGAIN;
        }
    }

    return 0;
}

static int can_n32_leave_init_mode(CAN_Module *can)
{
    uint32_t start_time;

    can->MCTRL &=  ~(uint32_t)CAN_MCTRL_INIRQ;
    
    start_time = k_cycle_get_32();

    while ((can->MSTS & CAN_MSTS_INIAK) != 0U) {
        if (k_cycle_get_32() - start_time > CAN_INIT_TIMEOUT) {
            return -EAGAIN;
        }
    }

    return 0;
}


// static int can_n32_leave_sleep_mode(CAN_Module *can)
// {
    // uint32_t start_time;

    // can->MCTRL &= ~CAN_MCTRL_SLPRQ;
    // start_time = k_cycle_get_32();

    // while ((can->MSTS & CAN_MSTS_SLPAK) != 0) {
        // if (k_cycle_get_32() - start_time > CAN_INIT_TIMEOUT) {
            // return -EAGAIN;
        // }
    // }

    // return 0;
// }



static int can_n32_init(const struct device *dev)
{
    const struct can_n32_config *cfg = dev->config;
    struct can_n32_data *data = dev->data;
    
    CAN_Module *can = cfg->can;
    
    struct can_timing timing = { 0 };
    uint32_t bank_offset;
    int ret;

    k_mutex_init(&filter_mutex);
    k_mutex_init(&data->inst_mutex);
    k_sem_init(&data->tx_allocs_sem, 0, 1);

    if (cfg->phy != NULL) {
        if (!device_is_ready(cfg->phy)) {
            LOG_ERR("CAN transceiver not ready");
            return -ENODEV;
        }
    }


    ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)), (clock_control_subsys_t *)(&cfg->can_apb_clk_en));
    
    if (ret != 0) {
        LOG_ERR("HAL_CAN_Init clock control on failed: %d", ret);
        return -EIO;
    }

    /* Configure dt provided device signals when available */
    ret = pinctrl_apply_state(cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("CAN pinctrl setup failed (%d)", ret);
        return ret;
    }
    
    /* Exit from sleep mode */
    can->MCTRL &= (~(uint32_t)CAN_MCTRL_SLPRQ);
    uint32_t start_time = k_cycle_get_32();

    while ((can->MSTS & CAN_MSTS_SLPAK) != 0) {
        if (k_cycle_get_32() - start_time > INIAK_TIMEOUT) {
            __NOP();
            return -EAGAIN;
        }
    }
  
   /* Request initialisation */
    can->MCTRL |= CAN_MCTRL_INIRQ;
    start_time = k_cycle_get_32();
    while ((can->MSTS & CAN_MSTS_INIAK) == 0) {
        if (k_cycle_get_32() - start_time > INIAK_TIMEOUT) {
             return -EAGAIN;
        }
    }

    /* configure scale of filter banks < CONFIG_CAN_MAX_EXTENDED_ID_FILTER for ext ids */
    bank_offset = (cfg->can == cfg->master_can) ? 0 : CAN_N32_NUM_FILTER_BANKS;
    cfg->master_can->FMC |= CAN_FMC_FINITM;
    cfg->master_can->FS1 |= ((1U << CONFIG_CAN_MAX_EXTENDED_ID_FILTER) - 1) << bank_offset;
    cfg->master_can->FMC &= ~CAN_FMC_FINITM;

    can->MCTRL &= ~CAN_MCTRL_TTCM & ~CAN_MCTRL_ABOM & ~CAN_MCTRL_AWKUM &
            ~CAN_MCTRL_NART & ~CAN_MCTRL_RFLM & ~CAN_MCTRL_TXFP;
#ifdef CONFIG_CAN_RX_TIMESTAMP
    can->MCTRL |= CAN_MCTRL_TTCM;
#endif
#ifdef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
    can->MCTRL |= CAN_MCTRL_ABOM;
#endif

    if (cfg->sample_point && USE_SP_ALGO) {
        ret = can_calc_timing(dev, &timing, cfg->bus_speed,
                      cfg->sample_point);
        __NOP();          
        if (ret == -EINVAL) {
            __NOP();
            LOG_ERR("Can't find timing for given param");
            return -EIO;
        }
        LOG_DBG("Presc: %d, TS1: %d, TS2: %d",
            timing.prescaler, timing.phase_seg1, timing.phase_seg2);
        LOG_DBG("Sample-point err : %d", ret);        
    } 
    else {
        timing.sjw = cfg->sjw;
        timing.prop_seg = 0;
        timing.phase_seg1 = cfg->prop_ts1;
        timing.phase_seg2 = cfg->ts2;
        ret = can_calc_prescaler(dev, &timing, cfg->bus_speed);
        if (ret) {
            LOG_WRN("Bitrate error: %d", ret);
        }
    }

    ret = can_set_timing(dev, &timing);
    if (ret) {
        return ret;
    }
    
    ret = can_n32_set_mode(dev, CAN_MODE_NORMAL);
    if (ret) {
        return ret;
    }
 

    (void)can_n32_get_state(dev, &data->state, NULL);

    cfg->irq_config_func(can);
    can->INTE |= CAN_INTE_TMEITE;  

    return 0;
}





static const struct can_driver_api can_n32_api = {
    .get_capabilities = can_n32_get_capabilities,
    .start = can_n32_start,
    .stop = can_n32_stop,
    .set_mode = can_n32_set_mode,
    .set_timing = can_n32_set_timing,
    .send = can_n32_send,
    .add_rx_filter = can_n32_add_rx_filter,
    .remove_rx_filter = can_n32_remove_rx_filter,
    .get_state = can_n32_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
    .recover = can_n32_recover,
#endif
    .set_state_change_callback = can_n32_set_state_change_callback,
    .get_core_clock = can_n32_get_core_clock,
    .get_max_bitrate = can_n32_get_max_bitrate,
    .get_max_filters = can_n32_get_max_filters,
    .timing_min = {
        .sjw = 0x1,
        .prop_seg = 0x00,
        .phase_seg1 = 0x01,
        .phase_seg2 = 0x01,
        .prescaler = 0x01
    },
    .timing_max = {
        .sjw = 0x04,
        .prop_seg = 0x00,
        .phase_seg1 = 0x10,
        .phase_seg2 = 0x08,
        .prescaler = 0x400
    }
};




#define CAN_N32_INIT(n)            \
                                         \
    PINCTRL_DT_INST_DEFINE(n);           \
    static void config_can_##n##_irq(CAN_Module *canx)    \
    {                                                    \
        IRQ_CONNECT( \
            DT_INST_IRQ_BY_NAME(n, rx, irq),             \
            DT_INST_IRQ_BY_NAME(n, rx, priority),        \
            can_n32_rx_isr, DEVICE_DT_INST_GET(n),    \
            0);  \
        irq_enable(DT_INST_IRQ_BY_NAME(n, rx, irq));     \
                     \
        IRQ_CONNECT(  \
                DT_INST_IRQ_BY_NAME(n, tx, irq),    \
                DT_INST_IRQ_BY_NAME(n, tx, priority),   \
                can_n32_tx_isr, DEVICE_DT_INST_GET(n),  \
                0);  \
        irq_enable(DT_INST_IRQ_BY_NAME(n, tx, irq));      \
                                                          \
        IRQ_CONNECT(   \
                DT_INST_IRQ_BY_NAME(n, sce, irq),             \
                DT_INST_IRQ_BY_NAME(n, sce, priority),        \
                can_n32_state_change_isr,                      \
                DEVICE_DT_INST_GET(n), 0);                    \
        irq_enable(DT_INST_IRQ_BY_NAME(n, sce, irq));             \
                     \
        canx->INTE |= CAN_INT_TME | CAN_INT_ERR | CAN_INT_FMP0 | \
                CAN_INT_FMP1 | CAN_INT_BOF;                  \
        if (IS_ENABLED(CONFIG_CAN_STATS)) {                          \
            canx->INTE |= CAN_INT_LEC;                           \
        }                                            \
    };  \
        \
    static const struct can_n32_config can_n32_cfg_##n = {            \
        .bus_speed = DT_INST_PROP(n, bus_speed),                      \
        .sample_point = DT_INST_PROP_OR(n, sample_point, 0),          \
        .can    = (CAN_Module *)DT_INST_REG_ADDR(n),             \
        .master_can = (CAN_Module *)DT_INST_PROP_OR(n,               \
                      master_can_reg, DT_INST_REG_ADDR(n)),         \
        .sjw = DT_INST_PROP_OR(n, sjw, 1),                            \
        .prop_ts1 = DT_INST_PROP_OR(n, prop_seg, 0) +           \
                    DT_INST_PROP_OR(n, phase_seg1, 0),         \
        .ts2     = DT_INST_PROP_OR(n, phase_seg2, 0),           \
        .can_apb_clk_en  = DT_INST_CLOCKS_CELL(n, bits),      \
        .irq_config_func = config_can_##n##_irq,                \
        .pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                     \
        .phy = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(n, phys)),       \
        .max_bitrate = DT_INST_CAN_TRANSCEIVER_MAX_BITRATE(n, 1000000), \
    };  \
    static struct can_n32_data can_n32_dev_data_##n; \
             \
    CAN_DEVICE_DT_INST_DEFINE(                  \
                    n,                          \
                    can_n32_init,             \
                    NULL,                       \
                    &can_n32_dev_data_##n,    \
                    &can_n32_cfg_##n,         \
                    POST_KERNEL,                 \
                    CONFIG_CAN_INIT_PRIORITY,    \
                    &can_n32_api);


DT_INST_FOREACH_STATUS_OKAY(CAN_N32_INIT)

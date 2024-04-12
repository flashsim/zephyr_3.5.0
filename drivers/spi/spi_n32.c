/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */


#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_n32);


#include "spi_context.h"
#include <n32g45x_spi.h>


#define DT_DRV_COMPAT nations_n32_spi



struct spi_n32_cfg
{
    uint32_t reg_base;
    uint32_t apb_clk_en;
   
    const struct pinctrl_dev_config *pinctrl_cfg;
    void (*irq_config_func)(void);
     
    //struct spi_config spi_config;
};

struct spi_n32_data
{
    struct spi_context ctx;
};


static bool spi_n32_transfer_ongoing(struct spi_n32_data *data)
{
    return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}


static int spi_n32_tx_and_rx(const struct device *dev)
{
    const struct spi_n32_cfg *config = dev->config;
    struct spi_n32_data *data = dev->data;
    uint16_t tx_frame = 0, rx_frame = 0;
    
    const uint8_t data_size = SPI_WORD_SIZE_GET(data->ctx.config->operation);
    
    
    /* check  Send buffer not emplty */
    while (SPI_I2S_GetStatus((SPI_Module*)(config->reg_base), SPI_I2S_TE_FLAG) == RESET)
        ;   
    
    if (data_size == 8) {
        if (spi_context_tx_buf_on(&data->ctx)) {
            tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
        }
    
        /*!< Send byte through the SPI peripheral */
        SPI_I2S_TransmitData((SPI_Module*)(config->reg_base), tx_frame);
        spi_context_update_tx(&data->ctx, 1, 1);
    } 
    else {
        if (spi_context_tx_buf_on(&data->ctx)) {
            tx_frame = UNALIGNED_GET((uint16_t *)(data->ctx.tx_buf));
        }
        SPI_I2S_TransmitData((SPI_Module*)(config->reg_base), tx_frame);
        spi_context_update_tx(&data->ctx, 2, 1);
    }
    
    /* Wait to receive */
    while (SPI_I2S_GetStatus((SPI_Module*)(config->reg_base), SPI_I2S_RNE_FLAG) == RESET)
        ;
    
    if (data_size == 8) {
        rx_frame = SPI_I2S_ReceiveData((SPI_Module*)(config->reg_base));
        if (spi_context_rx_buf_on(&data->ctx)) {
            UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
        }
        spi_context_update_rx(&data->ctx, 1, 1);
    } 
    else {
        rx_frame = SPI_I2S_ReceiveData((SPI_Module*)(config->reg_base));
        if (spi_context_rx_buf_on(&data->ctx)) {
            UNALIGNED_PUT(rx_frame, (uint16_t *)data->ctx.rx_buf);
        }
        spi_context_update_rx(&data->ctx, 2, 1);
    }
    
    return 0;
}


static int spi_n32_init(const struct device *dev)
{
    //const struct device *pinmux = DEVICE_DT_GET(DT_NODELABEL(pinmux));
    const struct spi_n32_cfg *config = dev->config;
    struct spi_n32_data *data = dev->data;
    
    int ret = pinctrl_apply_state(config->pinctrl_cfg, PINCTRL_STATE_DEFAULT);
    
    ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)), (clock_control_subsys_t *)&config->apb_clk_en);
    if (ret < 0) {
        LOG_ERR("Could not enable SPI clock");
        return ret;
    }

    ret = spi_context_cs_configure_all(&data->ctx);
    if (ret < 0) {
        return ret;
    }

    spi_context_unlock_unconditionally(&data->ctx);

    return 0;
}



static int spi_n32_config(const struct device *dev,    const struct spi_config *config)
{
    struct spi_n32_data *data = dev->data;
    const struct spi_n32_cfg *cfg = dev->config;  
    
    if (spi_context_configured(&data->ctx, config)) {
        return 0;
    }
    
    if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
        LOG_ERR("Slave mode not supported");
        return -ENOTSUP;
    }
    SPI_I2S_DeInit((SPI_Module*)cfg->reg_base);
    
    SPI_InitType spi_init = {
        .DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX,
        .SpiMode       = SPI_MODE_MASTER,
        .DataLen       = SPI_DATA_SIZE_8BITS,
        .CLKPOL        = SPI_CLKPOL_HIGH,
        .CLKPHA        = SPI_CLKPHA_SECOND_EDGE,
        .NSS           = SPI_NSS_SOFT,
    
        .BaudRatePres = SPI_BR_PRESCALER_4,
    
        .FirstBit = SPI_FB_MSB,
        .CRCPoly  = 7    
    };
    
    if (SPI_WORD_SIZE_GET(config->operation) == 8) {
        spi_init.DataLen  = SPI_DATA_SIZE_8BITS;
    } 
    else {
        spi_init.DataLen  = SPI_DATA_SIZE_16BITS;
    }
    
    if (spi_cs_is_gpio(config)) {
        spi_init.NSS = SPI_NSS_SOFT;
    }
    else {
        spi_init.NSS = SPI_NSS_HARD;
    }
    
    if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
        // /*LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_HIGH);*/
        spi_init.CLKPOL = SPI_CLKPOL_HIGH;
    } 
    else {
        spi_init.CLKPOL = SPI_CLKPOL_LOW;
    }

    if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
        //LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_2EDGE);
        spi_init.CLKPHA = SPI_CLKPHA_SECOND_EDGE;
    } 
    else {
       // /*LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_1EDGE);*/
        spi_init.CLKPHA = SPI_CLKPHA_FIRST_EDGE;
    }
    
    if (config->operation & SPI_TRANSFER_LSB) {
        // /*LL_SPI_SetTransferBitOrder(spi, LL_SPI_LSB_FIRST);*/
        spi_init.FirstBit = SPI_FB_LSB;
    } 
    else {
        // /*LL_SPI_SetTransferBitOrder(spi, LL_SPI_MSB_FIRST);*/
        spi_init.FirstBit = SPI_FB_MSB;
    }
        

    // uint32_t clk_freq = 0;
    // clock_control_get_rate(DEVICE_DT_GET(DT_NODELABEL(rcc)),
                           // (clock_control_subsys_t)&cfg->apb_clk_en,
                           // &clk_freq);
    
    // spi_init.BaudRatePres = clk_freq;
    
    SPI_Init((SPI_Module*)cfg->reg_base, &spi_init);

    data->ctx.config = config;

    return 0;
}

static int spi_n32_transceive(const struct device *dev,
                              const struct spi_config *config,
                              const struct spi_buf_set *tx_bufs,
                              const struct spi_buf_set *rx_bufs)
{
    int ret = 0;
    struct spi_n32_data *data = dev->data;
    const struct spi_n32_cfg *cfg = dev->config;

    /* context setup */
    spi_context_lock(&data->ctx, false, NULL, NULL, config);
    

    /* set configuration */
    ret = spi_n32_config(dev, config);
    
    /*!< Enable the SPI  */
    SPI_Enable((SPI_Module*)cfg->reg_base, ENABLE);
    
    if (ret < 0)
    {
        return ret;
    }

    spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
    spi_context_cs_control(&data->ctx, true);
    

    
#ifdef CONFIG_SPI_N32_INTERRUPT
#ifdef CONFIG_SPI_N32_DMA
    if (spi_n32_dma_enabled(dev)) {
        for (size_t i = 0; i < ARRAY_SIZE(data->dma); i++) {
            data->dma[i].count = 0;
        }

        ret = spi_n32_start_dma_transceive(dev);
        if (ret < 0) {
            goto dma_error;
        }
    } else
#endif
    {
        ret = spi_context_wait_for_completion(&data->ctx);
    }
#else
    do {
        ret = spi_n32_tx_and_rx(dev);
        if (ret < 0) {
            break;
        }
    } while (spi_n32_transfer_ongoing(data));

    
#endif
    
    
    
    //SPI_CTL0(cfg->reg) &= ~(SPI_CTL0_SPIEN | SPI_CTL1_DMATEN | SPI_CTL1_DMAREN);
    
    
    spi_context_cs_control(&data->ctx, false);
    SPI_Enable((SPI_Module*)cfg->reg_base, DISABLE);



    /* release context */
    spi_context_release(&data->ctx, ret);

    return ret;
}

#ifdef CONFIG_SPI_ASYNC
/* API implementation: transceive_async */
static int spi_n32_transceive_async(const struct device *dev,
                                    const struct spi_config *config,
                                    const struct spi_buf_set *tx_bufs,
                                    const struct spi_buf_set *rx_bufs,
                                    struct k_poll_signal *async)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(config);
    ARG_UNUSED(tx_bufs);
    ARG_UNUSED(rx_bufs);
    ARG_UNUSED(async);

    return -ENOTSUP;
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_n32_release(const struct device *dev, const struct spi_config *config)
{
    struct spi_n32_data *data = dev->data;
    spi_context_unlock_unconditionally(&data->ctx);

    return 0;
}

static const struct spi_driver_api spi_n32_api = {
    .transceive = spi_n32_transceive,
    .release = spi_n32_release,
#ifdef CONFIG_SPI_ASYNC
    .transceive_async = spi_n32_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
};

#define SPI_N32_DEVICE(n)                                      \
    PINCTRL_DT_INST_DEFINE(n);                           \
                          \
    static const struct spi_n32_cfg spi_n32_dev_cfg_##n = {    \
        .reg_base     = DT_INST_REG_ADDR(n),                   \
        .apb_clk_en   = DT_INST_CLOCKS_CELL(n, bits),           \
        .pinctrl_cfg  = PINCTRL_DT_INST_DEV_CONFIG_GET(n),  \
                     \
    };                     \
    static struct spi_n32_data spi_n32_dev_data_##n = { \
        SPI_CONTEXT_INIT_LOCK(spi_n32_dev_data_##n, ctx),	   \
        SPI_CONTEXT_INIT_SYNC(spi_n32_dev_data_##n, ctx),	   \
        SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)   \
    }; \
                                                               \
    DEVICE_DT_INST_DEFINE(n, \
                          &spi_n32_init,              \
                          NULL,                      \
                          &spi_n32_dev_data_##n,     \
                          &spi_n32_dev_cfg_##n,      \
                          POST_KERNEL,               \
                          CONFIG_SPI_INIT_PRIORITY,  \
                          &spi_n32_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_N32_DEVICE);

/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/dt-bindings/pinctrl/n32-pinctrl.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_n32, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"
#include <n32g45x_adc.h>


#define DT_DRV_COMPAT nations_n32_adc



static const uint8_t acq_time_table[8] = {
    ADC_SAMP_TIME_1CYCLES5,  
    ADC_SAMP_TIME_7CYCLES5,
    ADC_SAMP_TIME_13CYCLES5,
    ADC_SAMP_TIME_28CYCLES5,
    ADC_SAMP_TIME_41CYCLES5,
    ADC_SAMP_TIME_55CYCLES5,
    ADC_SAMP_TIME_71CYCLES5,
    ADC_SAMP_TIME_239CYCLES5
};



struct adc_n32_config 
{
    uint32_t reg_base;
    uint32_t ahb_clk_en;

    uint8_t channels;
    const struct pinctrl_dev_config *pinctrl_cfg;
    uint8_t irq_num;
    void (*irq_config_func)(void);
};

struct adc_n32_data 
{
    struct adc_context ctx;
    const struct device *dev;
    uint16_t *buffer;
    uint16_t *repeat_buffer;
};



static void adc_n32_isr(const struct device *dev)
{
    struct adc_n32_data *data = dev->data;
    const struct adc_n32_config *cfg = dev->config;
    ADC_Module *ADCx = (ADC_Module *)cfg->reg_base;

    if (ADC_GetFlagStatus(ADCx, ADC_FLAG_ENDC))
    {
        *data->buffer++ = ADC_GetDat(ADCx);
        
        ADC_ClearFlag(ADCx, ADC_FLAG_STR);
 
        /* Disable ENDC interrupt. */
        ADC_ConfigInt(ADCx, ADC_INT_ENDC, DISABLE);
        /* Clear ENDC bit. */
        ADC_ClearFlag(ADCx, ADC_FLAG_ENDC);
        adc_context_on_sampling_done(&data->ctx, dev);
    }
}





static void adc_context_start_sampling(struct adc_context *ctx)
{
    struct adc_n32_data *data = CONTAINER_OF(ctx, struct adc_n32_data, ctx);
    const struct device *dev = data->dev;
    const struct adc_n32_config *cfg = dev->config;
    ADC_Module *ADCx = (ADC_Module *)cfg->reg_base;

    data->repeat_buffer = data->buffer;

     /* Enable ENDC interrupt. */
    ADC_ConfigInt(ADCx, ADC_INT_ENDC, ENABLE);
    
     /* Start ADC Software Conversion */
    ADC_EnableSoftwareStartConv(ADCx, ENABLE);

    
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
                          bool repeat_sampling)
{
    struct adc_n32_data *data = CONTAINER_OF(ctx, struct adc_n32_data, ctx);

    if (repeat_sampling) {
        data->buffer = data->repeat_buffer;
    }
}


static int adc_n32_init(const struct device *dev)
{
    struct adc_n32_data *data = dev->data;
    const struct adc_n32_config *config = dev->config;
    int ret;
    ADC_Module *ADCx = (ADC_Module*)config->reg_base;
   
    data->dev = dev;

    ret = pinctrl_apply_state(config->pinctrl_cfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        return ret;
    }


    ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)), (clock_control_subsys_t *)&config->ahb_clk_en);
    
    if (ret < 0) {
        return ret;
    }


   /* RCC_ADCHCLK_DIV16*/
    ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV16);
    RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8);  //selsect HSE as RCC ADC1M CLK Source    


    /* ADC configuration ------------------------------------------------------*/
    ADC_InitType adc_init = {
        .WorkMode       = ADC_WORKMODE_INDEPENDENT,
        .MultiChEn      = DISABLE,
        .ContinueConvEn = DISABLE,
        .ExtTrigSelect  = ADC_EXT_TRIGCONV_NONE,
        .DatAlign       = ADC_DAT_ALIGN_R,
        .ChsNumber      = 1,
    };
    ADC_Init(ADCx, &adc_init);


    /* Enable ADC */
    ADC_Enable(ADCx, ENABLE);


     /*Check ADC Ready*/
    while(ADC_GetFlagStatusNew(ADCx, ADC_FLAG_RDY) == RESET)
        ;

   /* Start ADC calibration */
    ADC_StartCalibration(ADCx);
    
    /* Check the end of ADC calibration */
    while (ADC_GetCalibrationStatus(ADCx))
        ;

    config->irq_config_func();

    adc_context_unlock_unconditionally(&data->ctx);

    return 0;
}




static int adc_n32_configure_sampling_time(const struct device *dev,
                    uint8_t channel, uint16_t acq_time)
{
    uint8_t index = 0;

    const struct adc_n32_config *config = (const struct adc_n32_config *)dev->config;
    struct adc_n32_data *data = dev->data;


    if (acq_time != ADC_ACQ_TIME_DEFAULT) {
        /* Acquisition time unit is adc clock cycle. */
        if (ADC_ACQ_TIME_UNIT(acq_time) != ADC_ACQ_TIME_TICKS) {
            printf("char acqisition time err\r");
            return -EINVAL;
        }

        for (index=0; index < ARRAY_SIZE(acq_time_table); index++) {
            
            if (acq_time == ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS,  acq_time_table[index]))
            {
                acq_time = acq_time_table[index];
                break;                
            }
        }
    }
    else {
        printf("acq_time equal ADC_ACQ_TIME_DEFAULT \r");
        return 0;
    }

    ARG_UNUSED(data);
    
    if(index > ARRAY_SIZE(acq_time_table))
    {
        LOG_ERR("Sampling time value not supported.");
        return -EINVAL;
    }
 

    ADC_ConfigRegularChannel(((ADC_Module*)config->reg_base), channel, 1, acq_time);

    return 0;
}

static int adc_n32_channel_setup(const struct device *dev,    const struct adc_channel_cfg *chan_cfg)
{
    const struct adc_n32_config *config = dev->config;

    if (chan_cfg->gain != ADC_GAIN_1) {
        LOG_ERR("Gain is not valid");
        return -ENOTSUP;
    }

    //if (chan_cfg->reference != ADC_REF_INTERNAL) {
     //   LOG_ERR("Reference is not valid");
    //return -ENOTSUP;
    //}

    if (chan_cfg->differential) {
        LOG_ERR("Differential sampling not supported");
        return -ENOTSUP;
    }

    if (chan_cfg->channel_id >= config->channels) {
        LOG_ERR("Invalid channel (%u)", chan_cfg->channel_id);
        return -EINVAL;
    }

    return adc_n32_configure_sampling_time(dev, chan_cfg->channel_id,
                    chan_cfg->acquisition_time);
}












static int adc_n32_start_read(const struct device *dev,
                   const struct adc_sequence *sequence)
{
    struct adc_n32_data *data = dev->data;
    const struct adc_n32_config *config = dev->config;
    uint8_t resolution_id;
    uint32_t index;
    ADC_Module * ADCx = (ADC_Module*)config->reg_base;
    

    index = find_lsb_set(sequence->channels) - 1;
    
    if (sequence->channels > BIT(index)) {
        LOG_ERR("Only single channel supported");
        return -ENOTSUP;
    }

    switch (sequence->resolution) {
    case 12U:
        resolution_id = 3U;
        break;
    case 10U:
        resolution_id = 2U;
        break;
    case 8U:
        resolution_id = 1U;
        break;
    case 6U:
        resolution_id = 0U;
        break;
    default:
        return -EINVAL;
    }

    ADC_SetConvResultBitNum(ADCx, resolution_id);

    if (sequence->calibrate) {
        ADC_StartCalibration(ADCx);
        /* Check the end of ADC calibration */
        while (ADC_GetCalibrationStatus(ADCx))
            ;
    }

    /* Signle conversion mode with regular group. */ 
    ADCx->RSEQ3 &= ~ADC_RSEQ3_SEQ1;
    ADCx->RSEQ3 = index;
    
    data->buffer = sequence->buffer;
    adc_context_start_read(&data->ctx, sequence);

    return adc_context_wait_for_completion(&data->ctx);
}



static int adc_n32_read(const struct device *dev, const struct adc_sequence *sequence)
{
    struct adc_n32_data *data = dev->data;
    int error;

    adc_context_lock(&data->ctx, false, NULL);
    error = adc_n32_start_read(dev, sequence);
    adc_context_release(&data->ctx, error);

    return error;
}




#ifdef CONFIG_ADC_ASYNC
static int adc_n32_read_async(const struct device *dev,
                   const struct adc_sequence *sequence,
                   struct k_poll_signal *async)
{
    struct adc_n32_data *data = dev->data;
    int error;

    // adc_context_lock(&data->ctx, true, async);
    // error = adc_gd32_start_read(dev, sequence);
    // adc_context_release(&data->ctx, error);

    return error;
}
#endif /* CONFIG_ADC_ASYNC */



static struct adc_driver_api adc_n32_driver_api = {
    .channel_setup = adc_n32_channel_setup,
    .read = adc_n32_read,
#ifdef CONFIG_ADC_ASYNC
    .read_async = adc_n32_read_async,
#endif /* CONFIG_ADC_ASYNC */
};




#define HANDLE_SHARED_IRQ(n, active_irq)                            \
    static const struct device *const dev_##n = DEVICE_DT_INST_GET(n);            \
    const struct adc_n32_config *cfg_##n = dev_##n->config;                \
    ADC_Module *ADCx = (ADC_Module*)cfg_##n->reg_base;        \
                                                \
    if ((cfg_##n->irq_num == active_irq) && (ADCx->CTRL1 & ADC_FLAG_ENDC)) {    \
        adc_n32_isr(dev_##n);                                \
    }


static void adc_n32_global_irq_handler(const struct device *dev)
{
    const struct adc_n32_config *cfg = dev->config;

    LOG_DBG("global irq handler: %u", cfg->irq_num);

    //DT_INST_FOREACH_STATUS_OKAY_VARGS(HANDLE_SHARED_IRQ, (cfg->irq_num));
    
    adc_n32_isr(dev);
   
}

#define ADC1_ENABLE 1


static void adc_n32_global_irq_cfg(void)
{
    static bool global_irq_init = true;

    if (!global_irq_init) {
        return;
    }

    global_irq_init = false;
    
    //printf("DT_IRQ: %d\r\n", DT_IRQN(DT_NODELABEL(adc1)));
    

#if ADC1_ENABLE
    /* Shared irq config default to adc0. */
    IRQ_CONNECT(
                DT_IRQN(DT_NODELABEL(adc1)),
                DT_IRQ(DT_NODELABEL(adc1), priority),
                adc_n32_global_irq_handler,
                DEVICE_DT_GET(DT_NODELABEL(adc1)),
                0);
    irq_enable(DT_IRQN(DT_NODELABEL(adc1)));
#elif ADC2_ENABLE
    IRQ_CONNECT(
                DT_IRQN(DT_NODELABEL(adc2)),
                DT_IRQ(DT_NODELABEL(adc2), priority),
                adc_n32_global_irq_handler,
                DEVICE_DT_GET(DT_NODELABEL(adc2)),
                0);
    irq_enable(DT_IRQN(DT_NODELABEL(adc2)));
#endif

#if  ADC3_ENABLE
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc3)),
                DT_IRQ(DT_NODELABEL(adc3), priority),
                adc_n32_global_irq_handler,
                DEVICE_DT_GET(DT_NODELABEL(adc3)),
                0);
    irq_enable(DT_IRQN(DT_NODELABEL(adc3)))
#elif ADC4_ENABLE
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc4)),
                DT_IRQ(DT_NODELABEL(adc4), priority),
                adc_n32_global_irq_handler,
                DEVICE_DT_GET(DT_NODELABEL(adc4)),
                0);
    irq_enable(DT_IRQN(DT_NODELABEL(adc4)))
#endif
}




#ifdef CONFIG_SOC_SERIES_N32F3X0
#define ADC_CLOCK_SOURCE(n)                                    \
    .rcu_clock_source = DT_INST_PROP(n, rcu_clock_source)
#else
#define ADC_CLOCK_SOURCE(n)
#endif



#define ADC_N32_INIT(n)                                    \
    PINCTRL_DT_INST_DEFINE(n);                             \
                                                        \
    const static struct adc_n32_config adc_n32_config_##n = {       \
        .reg_base = DT_INST_REG_ADDR(n),                   \
        .ahb_clk_en  = DT_INST_CLOCKS_CELL(n, bits),       \
        .channels  = DT_INST_PROP(n, channels),              \
        .pinctrl_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),    \
        .irq_num  = DT_INST_IRQN(n),                        \
        .irq_config_func = adc_n32_global_irq_cfg,      \
        ADC_CLOCK_SOURCE(n)                                \
    };                                            \
                                                      \
    static struct adc_n32_data adc_n32_data_##n = {        \
        ADC_CONTEXT_INIT_TIMER(adc_n32_data_##n, ctx),     \
        ADC_CONTEXT_INIT_LOCK(adc_n32_data_##n, ctx),         \
        ADC_CONTEXT_INIT_SYNC(adc_n32_data_##n, ctx),         \
    };                                            \
    \
    DEVICE_DT_INST_DEFINE(n,                  \
                  &adc_n32_init,            \
                  NULL,                        \
                  &adc_n32_data_##n,        \
                  &adc_n32_config_##n,        \
                  POST_KERNEL,              \
                  CONFIG_ADC_INIT_PRIORITY,    \
                  &adc_n32_driver_api);    \

DT_INST_FOREACH_STATUS_OKAY(ADC_N32_INIT)



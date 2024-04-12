/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nations_n32_rcc

#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/sys/util.h>


#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr\drivers\clock_control\arm_clock_control.h>


#define GET_CLOCK_ID_OFFSET(id)  (((id) >> 6U) & 0xFFU)
#define GET_CLOCK_ID_BIT(id)     ((id)&0x1FU)

struct n32_clock_control_config 
{
    uint32_t base;
};

// static inline void n32set_clock(volatile uint32_t *base,
                    // uint8_t bit, enum arm_soc_state_t state)
// {
    // uint32_t key;

    // key = irq_lock();

    // switch (state) {
    // case SOC_ACTIVE:
        // base[0] |= (1 << bit);
        // break;
    // case SOC_SLEEP:
        // base[2] |= (1 << bit);
        // break;
    // case SOC_DEEPSLEEP:
        // base[4] |= (1 << bit);
        // break;
    // default:
        // break;
    // }

    // irq_unlock(key);
// }

static inline void n32ahb_set_clock_on(uint8_t bit,     enum arm_soc_state_t state)
{
     //n32set_clock((volatile uint32_t *)&(__N32_SYSCON->ahbclkcfg0set), bit, state);
}

static inline void n32ahb_set_clock_off(uint8_t bit, enum arm_soc_state_t state)
{
     //n32set_clock((volatile uint32_t *)&(__N32_SYSCON->ahbclkcfg0clr),  bit, state);
}

static inline void n32apb_set_clock_on(uint8_t bit,     enum arm_soc_state_t state)
{
     //n32set_clock((volatile uint32_t *)&(__N32_SYSCON->apbclkcfg0set),  bit, state);
}

static inline void n32apb_set_clock_off(uint8_t bit, enum arm_soc_state_t state)
{
     //n32set_clock((volatile uint32_t *)&(__N32_SYSCON->apbclkcfg0clr), bit, state);
}

static inline int n32_cc_on(const struct device *dev,
                            clock_control_subsys_t sub_system)
{
    const struct n32_clock_control_config *config = dev->config;
    uint32_t bits = *(uint32_t *)sub_system;
 
    sys_set_bit(config->base + GET_CLOCK_ID_OFFSET(bits), GET_CLOCK_ID_BIT(bits));


    return 0;
}

static inline int n32_cc_off(const struct device *dev,
                             clock_control_subsys_t sub_system)
{
    const struct n32_clock_control_config *config = dev->config;
    uint32_t bits = *(uint32_t *)sub_system;

    sys_clear_bit(config->base + GET_CLOCK_ID_OFFSET(bits),
            GET_CLOCK_ID_BIT(bits));

    return 0;
}

static int n32_cc_get_subsys_rate(const struct device *clock,
                                  clock_control_subsys_t sub_system,
                                  uint32_t *rate)
{
    int rc = 0;
    uint32_t bits = *(uint32_t *)sub_system;

    RCC_ClocksType RCC_ClockFreq;
    RCC_GetClocksFreqValue(&RCC_ClockFreq);

    switch (GET_CLOCK_ID_OFFSET(bits))
    {
    case N32_AHBPCLKEN_OFFSET:
        *rate = RCC_ClockFreq.HclkFreq;
        break;
    case N32_APB2PCLKEN_OFFSET:
        *rate = RCC_ClockFreq.Pclk2Freq;
        break;
    case N32_APB1PCLKEN_OFFSET:
        *rate = RCC_ClockFreq.Pclk1Freq;
        break;
    default:
        rc = -EINVAL;
        break;
    }

    return rc;
}

static const struct clock_control_driver_api n32_cc_api = {
    .on = n32_cc_on,
    .off = n32_cc_off,
    .get_rate = n32_cc_get_subsys_rate,
};

#ifdef CONFIG_CLOCK_CONTROL_N32_ENABLE_PLL

static uint32_t n32round_freq(uint32_t mainclk)
{
    uint32_t nc_mainclk = 0U;

    /*
    * Verify that the frequency is in the supported range otherwise
    * round it to the next closer one.
    */
    if (mainclk <= N32_PLL_FREQUENCY_12MHZ)
    {
        nc_mainclk = N32_PLL_FREQUENCY_12MHZ;
    }
    else if (mainclk <= N32_PLL_FREQUENCY_24MHZ)
    {
        nc_mainclk = N32_PLL_FREQUENCY_24MHZ;
    }
    else if (mainclk <= N32_PLL_FREQUENCY_36MHZ)
    {
        nc_mainclk = N32_PLL_FREQUENCY_36MHZ;
    }
    else
    {
        nc_mainclk = N32_PLL_FREQUENCY_48MHZ;
    }

    return nc_mainclk;
}

static uint32_t n32get_prescaler(uint32_t mainclk)
{
    uint32_t pre_mainclk = 0U;

    // /*
     // * Verify that the frequency is in the supported range otherwise
     // * round it to the next closer one.
     // */
    if (mainclk <= N32_PLL_FREQUENCY_12MHZ)
    {
        pre_mainclk = N32_PLL_PRESCALER_12MHZ;
    }
    else if (mainclk <= N32_PLL_FREQUENCY_24MHZ)
    {
        pre_mainclk = N32_PLL_PRESCALER_24MHZ;
    }
    else if (mainclk <= N32_PLL_FREQUENCY_36MHZ)
    {
        pre_mainclk = N32_PLL_PRESCALER_36MHZ;
    }
    else
    {
        pre_mainclk = N32_PLL_PRESCALER_48MHZ;
    }

    return pre_mainclk;
    

}

static int n32pll_enable(uint32_t mainclk)
{
    uint32_t pre_mainclk = n32get_prescaler(mainclk);

    // /* Set PLLCTRL Register */
    __N32_SYSCON->pllctrl = N32_PLL_CONFIGURATION;

    // /* Switch the the Main clock to PLL and set prescaler */
    __N32_SYSCON->mainclk = pre_mainclk;

    while (!__N32_SYSCON->pllstatus)
    {
        /* Wait for PLL to lock */
    }

    return 0;
}
#endif /* CONFIG_CLOCK_CONTROL_N32_ENABLE_PLL */

static int n32_cc_init(const struct device *dev)
{
#ifdef CONFIG_CLOCK_CONTROL_N32_ENABLE_PLL
    const struct n32_clock_control_cfg_t *const cfg = dev->config;

    /*
     * Enable PLL if Beetle is configured to run at a different
     * frequency than 24Mhz.
     */
    if (cfg->freq != MAINCLK_BASE_FREQ)
    {
        n32pll_enable(cfg->freq);
    }
#endif /* CONFIG_CLOCK_CONTROL_N32_ENABLE_PLL */

    return 0;
}

static const struct n32_clock_control_config n32_cc_cfg = {
    .base = DT_INST_REG_ADDR(0),
};

/**
 * @brief Clock Control device init
 *
 */
DEVICE_DT_INST_DEFINE(0, \
                    &n32_cc_init, \
                    NULL, \
                    NULL, \
                    &n32_cc_cfg, \
                    PRE_KERNEL_1, \
                    CONFIG_CLOCK_CONTROL_INIT_PRIORITY,  \
                    &n32_cc_api);

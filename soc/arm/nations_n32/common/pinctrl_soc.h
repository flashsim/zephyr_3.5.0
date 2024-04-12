/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_SOC_ARM_NATIONS_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_NATIONS_COMMON_PINCTRL_SOC_H_


#include <zephyr/devicetree.h>
#include <zephyr/types.h>
#include <zephyr/dt-bindings/pinctrl/n32-pinctrl.h>




#ifdef __cplusplus
extern "C" {
#endif



#define N32_NO_PULL     0x0
#define N32_PULL_UP     0x1
#define N32_PULL_DOWN   0x2
#define N32_PUSH_PULL   0x0
#define N32_OPEN_DRAIN  0x1
#define N32_OUTPUT_LOW  0x0
#define N32_OUTPUT_HIGH 0x1
#define N32_GPIO_OUTPUT 0x1









/** Type for N32 pin. */
typedef struct pinctrl_soc_pin {
	/** Pinmux settings (port, pin and function, Pin configuration (bias, drive and slew rate ). */
	uint32_t pinmux;

} pinctrl_soc_pin_t;





/**
 * @brief Utility macro to initialize pinmux field in #pinctrl_pin_t.
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_N32_PINMUX_INIT(node_id) DT_PROP(node_id, pinmux)





/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param state_prop State property name.
 * @param idx State property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, state_prop, idx) \
	{ .pinmux = Z_PINCTRL_N32_PINMUX_INIT(			   \
		DT_PROP_BY_IDX(node_id, state_prop, idx)),	       \
    },
    
    
 
/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			       \
	{DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_NATIONS_COMMON_PINCTRL_SOC_H_ */

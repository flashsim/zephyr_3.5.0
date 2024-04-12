/*
 * 
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/device.h>
#include <zephyr/init.h>

#include <cmsis_core.h>
#include <soc.h>

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int n32g45x_init(void)
{
	/* Update CMSIS SystemCoreClock variable (HCLK) */
	/* At reset, system core clock is set to 8 MHz from HSI */
	//SystemCoreClock = 8000000;
    SystemInit();
	return 0;
}

SYS_INIT(n32g45x_init, PRE_KERNEL_1, 0);

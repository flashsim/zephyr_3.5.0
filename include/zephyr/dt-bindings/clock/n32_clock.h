/*
 * Copyright (c) 2023, Quincy.W <wangqyfm@foxmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_N32_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_N32_CLOCK_H_

#define N32_AHBPCLKEN_OFFSET  0x14
#define N32_APB2PCLKEN_OFFSET 0x18
#define N32_APB1PCLKEN_OFFSET 0x1c

#define N32_CLOCK_DMA1   ((N32_AHBPCLKEN_OFFSET << 6) | 0) // RCC_AHBPCLKEN.0
#define N32_CLOCK_DMA2   ((N32_AHBPCLKEN_OFFSET << 6) | 1) // RCC_AHBPCLKEN.1
#define N32_CLOCK_SRAM   ((N32_AHBPCLKEN_OFFSET << 6) | 2) // RCC_AHBPCLKEN.2
#define N32_CLOCK_FLITF  ((N32_AHBPCLKEN_OFFSET << 6) | 4) // RCC_AHBPCLKEN.4
#define N32_CLOCK_CRC    ((N32_AHBPCLKEN_OFFSET << 6) | 6) // RCC_AHBPCLKEN.6
#define N32_CLOCK_RNG    ((N32_AHBPCLKEN_OFFSET << 6) | 9) // RCC_AHBPCLKEN.9
#define N32_CLOCK_SDIO   ((N32_AHBPCLKEN_OFFSET << 6) | 10) // RCC_AHBPCLKEN.10
#define N32_CLOCK_SCA    ((N32_AHBPCLKEN_OFFSET << 6) | 11) // RCC_AHBPCLKEN.11
#define N32_CLOCK_ADC1   ((N32_AHBPCLKEN_OFFSET << 6) | 12) // RCC_AHBPCLKEN.12
#define N32_CLOCK_ADC2   ((N32_AHBPCLKEN_OFFSET << 6) | 13) // RCC_AHBPCLKEN.13

#define N32_CLOCK_AFIO   ((N32_APB2PCLKEN_OFFSET << 6) | 0) // RCC_APB2PCLKEN.0
#define N32_CLOCK_GPIOA  ((N32_APB2PCLKEN_OFFSET << 6) | 2) // RCC_APB2PCLKEN.2
#define N32_CLOCK_GPIOB  ((N32_APB2PCLKEN_OFFSET << 6) | 3) // RCC_APB2PCLKEN.3
#define N32_CLOCK_GPIOC  ((N32_APB2PCLKEN_OFFSET << 6) | 4) // RCC_APB2PCLKEN.4
#define N32_CLOCK_GPIOD  ((N32_APB2PCLKEN_OFFSET << 6) | 5) // RCC_APB2PCLKEN.5
#define N32_CLOCK_GPIOE  ((N32_APB2PCLKEN_OFFSET << 6) | 6) // RCC_APB2PCLKEN.6
#define N32_CLOCK_GPIOF  ((N32_APB2PCLKEN_OFFSET << 6) | 7) // RCC_APB2PCLKEN.7
#define N32_CLOCK_GPIOG  ((N32_APB2PCLKEN_OFFSET << 6) | 8) // RCC_APB2PCLKEN.8
#define N32_CLOCK_TIM1   ((N32_APB2PCLKEN_OFFSET << 6) | 11) // RCC_APB2PCLKEN.11
#define N32_CLOCK_SPI1   ((N32_APB2PCLKEN_OFFSET << 6) | 12) // RCC_APB2PCLKEN.12
#define N32_CLOCK_TIM8   ((N32_APB2PCLKEN_OFFSET << 6) | 13) // RCC_APB2PCLKEN.13
#define N32_CLOCK_USART1 ((N32_APB2PCLKEN_OFFSET << 6) | 14) // RCC_APB2PCLKEN.14
#define N32_CLOCK_DVP    ((N32_APB2PCLKEN_OFFSET << 6) | 16) // RCC_APB2PCLKEN.16
#define N32_CLOCK_UART6  ((N32_APB2PCLKEN_OFFSET << 6) | 17) // RCC_APB2PCLKEN.17
#define N32_CLOCK_UART7  ((N32_APB2PCLKEN_OFFSET << 6) | 18) // RCC_APB2PCLKEN.18
#define N32_CLOCK_I2C3   ((N32_APB2PCLKEN_OFFSET << 6) | 19) // RCC_APB2PCLKEN.19

#define N32_CLOCK_TIM2   ((N32_APB1PCLKEN_OFFSET << 6) | 0) // RCC_APB1PCLKEN.0
#define N32_CLOCK_TIM3   ((N32_APB1PCLKEN_OFFSET << 6) | 1) // RCC_APB1PCLKEN.1
#define N32_CLOCK_TIM4   ((N32_APB1PCLKEN_OFFSET << 6) | 2) // RCC_APB1PCLKEN.2
#define N32_CLOCK_TIM5   ((N32_APB1PCLKEN_OFFSET << 6) | 3) // RCC_APB1PCLKEN.3
#define N32_CLOCK_TIM6   ((N32_APB1PCLKEN_OFFSET << 6) | 4) // RCC_APB1PCLKEN.4
#define N32_CLOCK_TIM7   ((N32_APB1PCLKEN_OFFSET << 6) | 5) // RCC_APB1PCLKEN.5
#define N32_CLOCK_TSC    ((N32_APB1PCLKEN_OFFSET << 6) | 10) // RCC_APB1PCLKEN.10
#define N32_CLOCK_WWDG   ((N32_APB1PCLKEN_OFFSET << 6) | 11) // RCC_APB1PCLKEN.11
#define N32_CLOCK_SPI2   ((N32_APB1PCLKEN_OFFSET << 6) | 14) // RCC_APB1PCLKEN.14
#define N32_CLOCK_SPI3   ((N32_APB1PCLKEN_OFFSET << 6) | 15) // RCC_APB1PCLKEN.15
#define N32_CLOCK_USART2 ((N32_APB1PCLKEN_OFFSET << 6) | 17) // RCC_APB1PCLKEN.17
#define N32_CLOCK_USART3 ((N32_APB1PCLKEN_OFFSET << 6) | 18) // RCC_APB1PCLKEN.18
#define N32_CLOCK_UART4  ((N32_APB1PCLKEN_OFFSET << 6) | 19) // RCC_APB1PCLKEN.19
#define N32_CLOCK_UART5  ((N32_APB1PCLKEN_OFFSET << 6) | 20) // RCC_APB1PCLKEN.20
#define N32_CLOCK_I2C1   ((N32_APB1PCLKEN_OFFSET << 6) | 21) // RCC_APB1PCLKEN.21
#define N32_CLOCK_I2C2   ((N32_APB1PCLKEN_OFFSET << 6) | 22) // RCC_APB1PCLKEN.22
#define N32_CLOCK_USB    ((N32_APB1PCLKEN_OFFSET << 6) | 23) // RCC_APB1PCLKEN.23
#define N32_CLOCK_CAN1   ((N32_APB1PCLKEN_OFFSET << 6) | 25) // RCC_APB1PCLKEN.25
#define N32_CLOCK_CAN2   ((N32_APB1PCLKEN_OFFSET << 6) | 26) // RCC_APB1PCLKEN.26
#define N32_CLOCK_BKP    ((N32_APB1PCLKEN_OFFSET << 6) | 27) // RCC_APB1PCLKEN.27
#define N32_CLOCK_PWR    ((N32_APB1PCLKEN_OFFSET << 6) | 28) // RCC_APB1PCLKEN.28
#define N32_CLOCK_DAC    ((N32_APB1PCLKEN_OFFSET << 6) | 29) // RCC_APB1PCLKEN.29

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_N32_CLOCK_H_ */
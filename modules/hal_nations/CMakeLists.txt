#
# Copyright (c) 2024, Nationstech
# SPDX-License-Identifier: Apache-2.0
#

if(CONFIG_HAS_NATIONS_N32_HAL)
  add_subdirectory(${ZEPHYR_CURRENT_MODULE_DIR} hal_nations)
  string(TOUPPER ${CONFIG_SOC} n32_soc)
  
  set(n32_soc_dir  ${ZEPHYR_CURRENT_MODULE_DIR}/${CONFIG_SOC_SERIES})
  set(n32_soc_sys_dir   ${n32_soc_dir}/CMSIS/device)
  set(n32_peripherals_driver_dir   ${n32_soc_dir}/drivers)
  set(n32_peripherals_driver_src_dir   ${n32_soc_dir}/drivers/src)
  
 message(---------------------------------------------------------------------------------------------------------)
#  message(n32_soc_dir:, ${n32_soc_dir})
#  message(n32_soc_sys_dir:, ${n32_soc_sys_dir})
#  message(ZEPHYR_CURRENT_MODULE_DIR:, ${ZEPHYR_CURRENT_MODULE_DIR})
#  message(n32_soc:, ${CONFIG_SOC})
 message(n32_peripherals_driver_dir:, ${n32_peripherals_driver_dir})
 message(n32_peripherals_driver_src_dir:, ${n32_peripherals_driver_src_dir})
 message(---------------------------------------------------------------------------------------------------------)
 
    

    
# Global includes to be used outside hal_nations
zephyr_include_directories(${n32_soc_sys_dir})
zephyr_include_directories(${n32_peripherals_driver_dir}/inc)

zephyr_library_sources(${n32_soc_sys_dir}/system_${CONFIG_SOC_SERIES}.c)



zephyr_library_sources_ifdef(CONFIG_USE_N32_GPIO ${n32_peripherals_driver_src_dir}/n32g45x_gpio.c)
zephyr_library_sources_ifdef(CONFIG_USE_N32_RCC  ${n32_peripherals_driver_src_dir}/n32g45x_rcc.c)
zephyr_library_sources_ifdef(CONFIG_USE_N32_UART ${n32_peripherals_driver_src_dir}/n32g45x_usart.c)
zephyr_library_sources_ifdef(CONFIG_USE_N32_DMA  ${n32_peripherals_driver_src_dir}/n32g45x_dma.c)
zephyr_library_sources_ifdef(CONFIG_USE_N32_I2C  ${n32_peripherals_driver_src_dir}/n32g45x_i2c.c)
zephyr_library_sources_ifdef(CONFIG_USE_N32_SPI  ${n32_peripherals_driver_src_dir}/n32g45x_spi.c)
zephyr_library_sources_ifdef(CONFIG_USE_N32_BXCAN  ${n32_peripherals_driver_src_dir}/n32g45x_can.c)
zephyr_library_sources_ifdef(CONFIG_USE_N32_ADC  ${n32_peripherals_driver_src_dir}/n32g45x_adc.c)
zephyr_library_sources_ifdef(CONFIG_USE_N32_FLASH  ${n32_peripherals_driver_src_dir}/n32g45x_flash.c)
zephyr_library_sources_ifdef(CONFIG_USE_N32_TIM  ${n32_peripherals_driver_src_dir}/n32g45x_tim.c)
zephyr_library_sources_ifdef(CONFIG_USE_N32_EXTI  ${n32_peripherals_driver_src_dir}/n32g45x_exti.c)





endif()

.. _n32g45xv_stb:

NATIONS-N32G45XV-STB
########################

Overview
********

The N32G45XV-STB board is a hardware platform that enables design and debug
of the Cortex-M4F High Performance MCU.There are multiple version of this board .

.. image:: img/n32g45xv_stb.jpg
     :align: center
     :alt: n32g45xv_stb



Hardware
********
- 2 user LEDs
- 2 user push buttons
- Reset Button
- USB VBUS or external source (3.3V, 5V)
- vbat
- usb to serial port
- NS-Link interface


For more information about the N32G45X SoC and N32G45X-STB board:

- `N32G45X Cortex-M4F High Performance SoC Website`_
- `N32G457 Datasheet`_
- `N32G457 Reference Manual`_
- `GD32A503V Eval Schematics`_




Supported Features
==================

The Zephyr n32g45xv_stb board configuration supports the following hardware features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | serial port-polling;                |
|           |            | serial port-interrupt               |
+-----------+------------+-------------------------------------+
| PINMUX    | on-chip    | pinmux                              |
+-----------+------------+-------------------------------------+
| GPIO      | on-chip    | gpio                                |
+-----------+------------+-------------------------------------+
| CLOCK     | on-chip    | reset and clock control             |
+-----------+------------+-------------------------------------+
| FLASH     | on-chip    | flash memory                        |
+-----------+------------+-------------------------------------+
| WATCHDOG  | on-chip    | independent watchdog                |
+-----------+------------+-------------------------------------+
| ADC       | on-chip    | ADC Controller                      |
+-----------+------------+-------------------------------------+
| PWM       | on-chip    | pwm                                 |
+-----------+------------+-------------------------------------+
| SPI       | on-chip    | spi                                 |
+-----------+------------+-------------------------------------+
| USB       | on-chip    | USB device                          |
+-----------+------------+-------------------------------------+
| COUNTER   | on-chip    | rtc                                 |
+-----------+------------+-------------------------------------+
| RTC       | on-chip    | rtc                                 |
+-----------+------------+-------------------------------------+


The default configuration can be found in the defconfig file:
``boards/arm/nations_n32g45xv_stb/nations_n32g45xv_stb_defconfig``




Default Zephyr Peripheral Mapping:
----------------------------------

- USART_1 TX/RX: PA9/PA10 (NS-Link Virtual COM Port)

- SPI1 NSS/SCK/MISO/MOSI: PA4/PA5/PA6/PA7
- SPI2 NSS/SCK/MISO/MOSI: PB12/PB13/PB14/PB15
- I2C1 SDA/SCL: PB9/PB8
- USER_PB: PC13
- LED1: PA8
- LED2: PB4
- LED3: PB5
- KEY1: PA4
- KEY2: PA5
- KEY3: PA6
- USB_DC DM/DP: PA11/PA12



System Clock
------------

The on-board 8MHz crystal is used to produce a 144MHz system clock with PLL.





Flashing
========

Follow the :ref:`getting_started` instructions for Zephyr application
development.

For example, to build and flash the :ref:`hello_world` application for the
MSP-EXP432P401R LaunchXL:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: msp_exp432p401r_launchxl
   :goals: flash

This will load the image into flash.

To see program output from UART0, connect a separate terminal window:

.. code-block:: console

  % screen /dev/ttyACM0 115200 8N1

Then press the reset button (S3) on the board to run the program.

Debugging
=========

To debug a previously flashed image, after resetting the board, use the 'debug'
build target:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: msp_exp432p401r_launchxl
   :maybe-skip-config:
   :goals: debug

References
**********

.. Nationstech Cortex-M4 High Performance SoC Website:
	https://www.gigadevice.com.cn/product/mcu/arm-cortex-m33/gd32a503vdt3


TI MSP432P401R Product Page:
   http://www.ti.com/product/msp432p401r

TI MSP432 SDK:
   http://www.ti.com/tool/SIMPLELINK-MSP432-SDK

.. _UniFlash:
   http://processors.wiki.ti.com/index.php/UniFlash_v4_Quick_Guide#Command_Line_Interface

.. _CCS IDE:
   http://www.ti.com/tool/ccstudio

..  _XDS-110 emulation package:
   http://processors.wiki.ti.com/index.php/XDS_Emulation_Software_Package#XDS_Emulation_Software_.28emupack.29_Download

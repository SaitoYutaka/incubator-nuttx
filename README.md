# NuttX for micro:bit

This file will show that how to build nuttx for micro:bit.

## Help

Please give me some advice on how to do this.
If so, please send me on https://github.com/SaitoYutaka/incubator-nuttx/issues .

## Why

Just my study.

## Satus

in progress.

## Step1
### About micro:bit

#### Hardware
 * https://tech.microbit.org/hardware/

#### Cortex-M0
 * https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M0

#### What are the differences between Cortex-M0 and Cortex-M0+?
 * https://community.cypress.com/docs/DOC-10652

## Step2
### Dodument

https://github.com/SaitoYutaka/incubator-nuttx/blob/master/Documentation/NuttxPortingGuide.html#L930

```
2.5.4 Adding a New Board Configuration
Okay, so you have created a new board configuration directory. Now, how do you hook this board into the configuration system so that you can select with make menuconfig?

You will need modify the file boards/Kconfig. Let's look at the STM32F4-Discovery configuration in the Kconfig file and see how we would add a new board directory to the configuration. For this configuration let's say that you new board resides in the directory boards/myarch/mychip/myboard; It uses an MCU selected with CONFIG_ARCH_CHIP_MYMCU; and you want the board to be selected with CONFIG_ARCH_BOARD_MYBOARD. Then here is how you can clone the STM32F4-Discovery configuration in boards/Kconfig to support your new board configuration.

In boards/Kconfig for the stm32f4-discovery, you will see a configuration definition like this:

config ARCH_BOARD_STM32F4_DISCOVERY
    bool "STMicro STM32F4-Discovery board"
    depends on ARCH_CHIP_STM32F407VG
    select ARCH_HAVE_LEDS
    select ARCH_HAVE_BUTTONS
    select ARCH_HAVE_IRQBUTTONS
    ---help---
        STMicro STM32F4-Discovery board based on the STMicro STM32F407VGT6 MCU.
```

## Step3
### Edit boards/Kconfig

...




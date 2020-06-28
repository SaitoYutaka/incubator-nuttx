# NuttX for micro:bit

This file will show that how to build nuttx for micro:bit.

## Debut on micro:bit

1.  Run pyocd
```
$ sudo pyocd gdbserver -t nrf51
```
2. Run Debug on VSCode 

## build and run qemu

```
$ git clone https://github.com/SaitoYutaka/incubator-nuttx.git nuttx
$ cd nuttx
$ ./tools/configure.sh microbit:nsh
$ make
$ qemu-system-arm -M microbit -device loader,file=nuttx -serial stdio

NuttShell (NSH)
nsh> ls
/:
 dev/
nsh> help
help usage:  help [-v] [<cmd>]

  [         cd        echo      hexdump   mkfatfs   mw        sh        uname     
  ?         cp        exec      kill      mkrd      pwd       sleep     umount    
  basename  cmp       exit      ls        mh        rm        test      unset     
  break     dirname   false     mb        mount     rmdir     time      usleep    
  cat       dd        help      mkdir     mv        set       true      xd        

Builtin Apps:
  hello  nsh    
nsh> hello
Hello, World!!
```

## Debug

```
$ qemu-system-arm -M microbit -device loader,file=nuttx -s -serial stdio
NuttShell (NSH)
>
```

```
$ arm-none-eabi-gdb
GNU gdb (GNU Tools for Arm Embedded Processors 7-2018-q2-update) 8.1.0.20180315-git
Copyright (C) 2018 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
and "show warranty" for details.
This GDB was configured as "--host=x86_64-linux-gnu --target=arm-none-eabi".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
<http://www.gnu.org/software/gdb/documentation/>.
For help, type "help".
Type "apropos word" to search for commands related to "word".
(gdb) target remote tcp:127.0.0.1:1234
Remote debugging using tcp:127.0.0.1:1234
warning: No executable has been specified and target does not support
determining executable automatically.  Try using the "file" command.
0x00004280 in ?? ()
(gdb) source ~/nuttx-microbit/nuttx/
(gdb) symbol-file ./nuttx
(gdb) b nsh_main
(gdb) c
Continuing.
```

## About qemu

https://www.qemu.org/2019/05/22/microbit/

## Why

Just my study.

## Satus

in progress.

## Memo
### About micro:bit

#### Hardware
 * https://tech.microbit.org/hardware/

#### Nordic nRF51822-QFAA-R rev 3
 * https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF51822

####  Standalone drivers for peripherals present in Nordic SoCs
 * https://github.com/NordicSemiconductor/nrfx

#### Cortex-M0
 * https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M0

#### What are the differences between Cortex-M0 and Cortex-M0+?
 * https://community.cypress.com/docs/DOC-10652

## Files

```
$ git diff 66401c8960c5211ded41ae4f60bfb1746a84bd50 --name-only
.vscode/launch.json
README.md
arch/arm/Kconfig
arch/arm/include/nrf51/chip.h
arch/arm/include/nrf51/irq.h
arch/arm/include/nrf51/nrf51_irq.h
arch/arm/src/nrf51/Kconfig
arch/arm/src/nrf51/Make.defs
arch/arm/src/nrf51/chip.h
arch/arm/src/nrf51/hardware/.gitkeep
arch/arm/src/nrf51/hardware/nrf51_clock.h
arch/arm/src/nrf51/hardware/nrf51_ficr.h
arch/arm/src/nrf51/hardware/nrf51_gpio.h
arch/arm/src/nrf51/hardware/nrf51_memorymap.h
arch/arm/src/nrf51/hardware/nrf51_nvmc.h
arch/arm/src/nrf51/hardware/nrf51_uarte.h
arch/arm/src/nrf51/hardware/nrf51_utils.h
arch/arm/src/nrf51/hardware/nrf51_wdt.h
arch/arm/src/nrf51/nrf51_allocateheap.c
arch/arm/src/nrf51/nrf51_clockconfig.c
arch/arm/src/nrf51/nrf51_clockconfig.h
arch/arm/src/nrf51/nrf51_config.h
arch/arm/src/nrf51/nrf51_gpio.c
arch/arm/src/nrf51/nrf51_gpio.h
arch/arm/src/nrf51/nrf51_idle.c
arch/arm/src/nrf51/nrf51_irq.c
arch/arm/src/nrf51/nrf51_irq.h
arch/arm/src/nrf51/nrf51_lowputc.c
arch/arm/src/nrf51/nrf51_lowputc.h
arch/arm/src/nrf51/nrf51_nvmc.c
arch/arm/src/nrf51/nrf51_nvmc.h
arch/arm/src/nrf51/nrf51_rtc.h
arch/arm/src/nrf51/nrf51_serial.c
arch/arm/src/nrf51/nrf51_serial.h
arch/arm/src/nrf51/nrf51_start.c
arch/arm/src/nrf51/nrf51_start.h
arch/arm/src/nrf51/nrf51_timerisr.c
arch/arm/src/nrf51/nrf51_utils.c
boards/Kconfig
boards/arm/nrf51/drivers/Kconfig
boards/arm/nrf51/microbit/Kconfig
boards/arm/nrf51/microbit/README.txt
boards/arm/nrf51/microbit/configs/nsh/.gitkeep
boards/arm/nrf51/microbit/configs/nsh/defconfig
boards/arm/nrf51/microbit/include/.gitkeep
boards/arm/nrf51/microbit/include/board.h
boards/arm/nrf51/microbit/scripts/.gitkeep
boards/arm/nrf51/microbit/scripts/Make.defs
boards/arm/nrf51/microbit/scripts/microbit.ld
boards/arm/nrf51/microbit/src/.gitkeep
boards/arm/nrf51/microbit/src/Makefile
boards/arm/nrf51/microbit/src/nrf51-generic.h
boards/arm/nrf51/microbit/src/nrf51_appinit.c
boards/arm/nrf51/microbit/src/nrf51_autoleds.c
boards/arm/nrf51/microbit/src/nrf51_boot.c
boards/arm/nrf51/microbit/src/nrf51_bringup.c
boards/arm/nrf51/microbit/src/nrf51_buttons.c
boards/arm/nrf51/microbit/src/nrf51_userleds.c
tools/configure.sh
```





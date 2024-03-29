#!/bin/sh
make distclean
make stm32mp1_atk_defconfig
make menuconfig
make uImage dtbs LOADADDR=0XC2000040 -j12


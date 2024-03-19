#!/bin/bash

make distclean
make stm32mp157_atk_trusted_defconfig
make DEVICE_TREE=stm32mp157d-atk all -j12

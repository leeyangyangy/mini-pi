#!/bin/sh
#ATK-STM32MP157 SD卡启动系统烧录脚本
#版本V1.0
#Author:ALIENTEK
VERSION="1.0"

execute ()
{
    $* >/dev/null
    if [ $? -ne 0 ]; then
        echo
        echo "错误: 执行 $*"
        echo
        exit 1
    fi
}

device=$1

#判断选择的块设备是否存在及是否是一个块设备
if [ ! -b $device ]; then
  echo "错误: $device 不是一个块设备文件"
  exit 1
fi

#这里防止选错设备，否则会影响Ubuntu系统的启动
if [ $device = '/dev/sda' ];then
  echo "请不要选择sda设备，/dev/sda通常是您的Ubuntu硬盘!
继续操作你的系统将会受到影响！脚本已自动退出"
  exit 1 
fi

#将制作好的整个系统烧写到SD卡
dd if=./flashlayout_atk_sdcard-stm32mp157d-atk-qt.raw of=$device bs=1M conv=fdatasync status=progress
sync

sleep 1

echo "正在重定义分区大小..."
sgdisk -d 5 ${device}
sgdisk -a 1 -n 5:136226:0 -c 5:rootfs -t 5:8300 -u 5:e91c4e10-16e6-4c0e-bd0e-77becf4a3582 ${device}
sync
echo "SD卡启动系统烧录完成，请退出!"

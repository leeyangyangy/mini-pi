#!/bin/sh
#ATK-STM32MP157 eMMC启动系统烧录脚本
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

#这里防止选错设备，否则会影响SD卡系统的启动
if [ $device = '/dev/mmcblk1' ];then
  echo "请不要选择mmcblk1设备，/dev/mmcblk1通常是您的SD卡系统所在设备!
继续操作您的SD卡系统将会受到影响！脚本已自动退出"
  exit 1 
fi

echo "使能eMMC写flag"
echo 0 > /sys/class/block/mmcblk2boot0/force_ro
echo 0 > /sys/class/block/mmcblk2boot1/force_ro

echo "正在烧录tf-a，请稍候..."
execute "dd if=./tf-a/tf-a-stm32mp157d-atk-trusted.stm32 of=${device}boot0 conv=fsync"
execute "dd if=./tf-a/tf-a-stm32mp157d-atk-trusted.stm32 of=${device}boot1 conv=fsync"

echo "关闭eMMC写flag"
echo 1 > /sys/class/block/mmcblk2boot0/force_ro
echo 1 > /sys/class/block/mmcblk2boot1/force_ro
execute "mmc bootpart enable 1 1 ${device}"

echo "正在烧录bootfs和根文件系统，请稍候..."
execute "dd if=./atk_emmc-stm32mp157d-atk-qt.raw of=${device} bs=8M conv=sync status=progress"

echo "正在重定义分区大小..."
sgdisk -d 3 ${device}
sync
sgdisk -a 1 -n 3:136192:0 -c 3:rootfs -t 3:8300 -u 3:491f6117-415d-4f53-88c9-6e0de54deac6 ${device}
sync

echo "eMMC启动系统烧录完成！请拨码至010，从eMMC启动系统"



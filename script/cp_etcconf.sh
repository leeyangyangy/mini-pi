#!/bin/sh
cp ../gen/rcS ~/linux/nfs/rootfs/etc/init.d/
chmod 777 ~/linux/nfs/rootfs/etc/init.d/rcS
cp ../gen/fstab ~/linux/nfs/rootfs/etc/
cp ../gen/inittab ~/linux/nfs/rootfs/etc/

#!/bin/bash

kernel='Image.gz-dtb-meizu'
ramdisk='twrp.360.img.gz'
dt='sm7325-idp.dtb'
#dtbs='qcom/sm7125-idp.dtb qcom/sdm636-meizu-E3.dtb'
#dtbs='qcom/sc7180-idp.dtb qcom/sdm636-meizu-E3.dtb'
#dtbs='joyeuse_dtb.bin'
dtbs='qcom/sm7125-idp.dtb'
echo twrp boot

cat arch/arm64/boot/Image.gz $dtbs > /tmp/$kernel
cat $dtbs > /tmp/$dt

#cat arch/arm64/boot/Image.gz qcom/sdm630-sony-xperia-nile-pioneer.dtb  > /tmp/Image.gz-dtb-meizu
#cat arch/arm64/boot/Image.gz qcom/sdm636-asus-x00td.dtb > /tmp/Image.gz-dtb-meizu
#cp twrp.331.img.gz /tmp/twrp.360.img.gz
#cp pmos.img.gz /tmp/twrp.360.img.gz
#cp twrp.350_mod.img.gz /tmp/twrp.360.img.gz
#cp ramdisk.331.mod_sh.gz /tmp/twrp.360.img.gz
#cp twrp.341.img.gz /tmp/twrp.360.img.gz
#cp twrp.361.img.gz /tmp/twrp.360.img.gz
#cp ramdisk_v8000.img.gz  /tmp/twrp.360.img.gz
#cp ramdisk_pl2_los18.1.img.gz  /tmp/twrp.360.img.gz
#cp orangefox_12.img.gz /tmp/twrp.360.img.gz
#cp ramdisk.miatoll.img.gz /tmp/twrp.360.img.gz
#cp ramdisk.orange.x2.img.gz /tmp/twrp.360.img.gz

# the recovery.fstab contain block device mount option.
# vayu can mis-match and cause error so skip mounting joyeuse partition
# thus save time.

#find lib | cpio -o -Hnewc |gzip -9 > /tmp/firmware.cpio.gz

cp ramdisk.los.miatoll.nonfs.img.gz /tmp/$ramdisk
#cp ramdisk_los_boot.img.gz /tmp/$ramdisk

cd ~/source/
find lib | cpio -o -Hnewc |gzip -9 >> /tmp/$ramdisk

#cp -u ramdisk.los.vayu.img.gz /tmp/$ramdisk
#cp ramdisk.los.fp4.img.gz /tmp/$ramdisk
#cp ramdisk.los.berlin.img.lz4 /tmp/$ramdisk
#cp ramdisk.los.miatoll.img.gz /tmp/$ramdisk
cd /tmp/

#boot image:
# boot KERNEL [RAMDISK [SECOND]]
#                            Download and boot kernel from RAM.
# flash:raw PARTITION KERNEL [RAMDISK [SECOND]]
#                            Create boot image and flash it.
# --dtb DTB                  Specify path to DTB for boot image header version 2.
# --cmdline CMDLINE          Override kernel command line.
# --base ADDRESS             Set kernel base address (default: 0x10000000).
# --kernel-offset            Set kernel offset (default: 0x00008000).
# --ramdisk-offset           Set ramdisk offset (default: 0x01000000).
# --tags-offset              Set tags offset (default: 0x00000100).
# --dtb-offset               Set dtb offset (default: 0x01100000).
# --page-size BYTES          Set flash page size (default: 2048).
# --header-version VERSION   Set boot image header version.
# --os-version MAJOR[.MINOR[.PATCH]]
#                            Set boot image OS version (default: 0.0.0).
# --os-patch-level YYYY-MM-DD
#                            Set boot image OS security patch level.

#cmdline='console=pstore1 clk_ignore_unused firmware_class.path=/vendor/firmware/ androidboot.hardware=qcom init=/init 
# androidboot.boot_devices=soc@0/1d84000.ufshc printk.devkmsg=on deferred_probe_timeout=30 buildvariant=userdebug'
# console=ttyMSM0,115200,n8 androidboot.console=ttyMSM0 earlycon=msm_serial_dm,0xc170000 androidboot.hardware=qcom user_debug=31 msm_rtb.filter=0x37 ehci-hcd.park=3 lpm_levels.sleep_disabled=1 sched_enable_hmp=1 sched_enable_power_aware=1 service_locator.enable=1 swiotlb=1 androidboot.configfs=true androidboot.usbcontroller=a800000.dwc3 androidboot.selinux=permissive loop.max_part=9 buildvariant=userdebug'
#cmdline='console=ttyMSM0,115200,n8 androidboot.console=ttyMSM0 earlycon=msm_serial_dm,0xc170000 maxcpus=1 clk_ignore_unused androidboot.hardware=qcom androidboot.configfs=true androidboot.usbcontroller=a600000.dwc3 printk.devkmsg=on androidboot.selinux=permissive'
#cmdline='twrpfastboot=1 console=ttyMSM0,115200n8 androidboot.hardware=qcom androidboot.console=ttyMSM0 androidboot.memcg=1 lpm_levels.sleep_disabled=1 video=vfb:640x400,bpp=32,memsize=3072000 msm_rtb.filter=0x237 service_locator.enable=1 androidboot.usbcontroller=a600000.dwc3 swiotlb=0 loop.max_part=7 cgroup.memory=nokmem,nosocket pcie_ports=compat loop.max_part=7 iptable_raw.raw_before_defrag=1 ip6table_raw.raw_before_defrag=1 buildvariant=user'
random=$RANDOM
cmdline_0='init=/init pd_ignore_unused no_console_suspend kpti=off nokaslr ignore_loglevel clk_ignore_unused '
cmdline_0='root=/dev/ram0 rw androidboot.hardware=qcom androidboot.usbcontroller=a600000.usb service_locator.enable=1 androidboot.keymaster=1 androidboot.fstab_suffix=default androidboot.multisim_config=dsds androidboot.selinux=permissive'
cmdline=$cmdline_0

kernel_offset='32768'
#kernel_offset='2097152'
ramdisk_offset='16777216'
# @2f0,0000
#ramdisk_offset='32505856'
loadBase='0'
tag_offset='256'
dt_offset='32505856'
# @3f0 0000
#dt_offset='66060288'
# @300 0000
#dt_offset='50331648'
osVersion='11.0.0'
osPatchLevel='2020-06-00'
headerVersion='2'
#ramdisk='twrp.img.gz'
#ramdisk='mk10.img.gz'
# --dtb $dt --dtb-offset $dt_offset
echo $cmdline
~/fastboot boot $kernel $ramdisk --dtb $dt --dtb-offset $dt_offset --base $loadBase --kernel-offset $kernel_offset --ramdisk-offset $ramdisk_offset --tags-offset $tag_offset --page-size 4096 --os-version $osVersion --os-patch-level $osPatchLevel --header-version $headerVersion --cmdline "$cmdline"

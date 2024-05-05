adb shell rm /system/bin/shutdown
adb push shutdown /system/bin
adb push arch/arm64/boot/Image /
adb push arch/arm64/boot/dts/qcom/sm7125-xiaomi-joyeuse-cust.dtb /
adb push ramdisk.img.gz /
adb shell kexec -s Image --dtb sm7125-xiaomi-joyeuse-cust.dtb --ramdisk ramdisk.img.gz --reuse-cmdline
echo adb shell kexec -e

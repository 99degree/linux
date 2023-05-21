# linux
Linux kernel source tree

This repo is tracking latest mainline kernel as updated as possible. The idea is to work with sm7125 soc and mainline [1]. 

The development machine is a XiaoMi redmi note 9 pro int'l version. Currently the working branch is based on latest code
of Linus's public tree with my own periodic merge into local change and git rebase for every push.

This repo have a simple philosophy, which is basically minimal change to mainline source. To archeive that, I had introduced
dts changes from mainly map220v [2] and the sm7125.dtsi within, together with my local changes aimed 
to sc7180.dtsi after all (assumed the great commonality of sc7180 and sm7125). I really limited the change to sc7180.dtsi 
and to archeive, i rather choose to override the value/node in sm7125.dtsi instead. the effective main dts is named as 
sm7125-idp.dts and comtaining all machine specific node (the difference between redmi note 9 pro and smasung A72/A52), 
unless there is any dt node missing by sm7125-idp.dts but present in arch/arm64/boot/dts/qcom/sm7125-samsung-a52q.dts 
or fail merge, most of the common functionality are paired.

Here are functionality not tested:
modem
rmtfs
slpi(sensors, largely broken, not supported)
bluetooth
fingerprint(largely broken, not supported)
sound/mic/headphone/ampifier dai
gpu(display comtroller is working, drm)
camera(largely broken, not supported)
pwm
lcd backlight
sd card

Tested good:
UFS
LCD display and its dsi drm controller
touchscreen
cpufreq
usb peri/adb shell
spi/dma
pon/pwr button/vol button

So with vendor nt36xxx_spi driver it is barely meet the requirement to Android/LOS. The success story of mainline similar to 
xiaomi phone is from Linaro [3] and the overall procedure is compile full Android/LOS by them-selves.

Currently the most mainline kernel friendly OS, postmarketOS, is not support with dynamic partition/virtual A/B scheme so the idea
of flashing the rootfs into data partition is no-op unless flashing to super partition (the dynamic partition and wipe all 
the sys/vendor/odm/system_ext itself) and lost the dual boot with LOS20 for ramoops reboot debug.

I am sticking with LOS 19 recovery img [4] with the resulting mainline kernel image.gz-dtb by fastboot. The idea is to unpack the recovery
initrd img and fastboot with the compiled zImage.gz-dtb as result. In addition, I also make use of the vendor bootloader capability
of concat gz/tar img, simplify the androidboot img as below, and boot script is provided later as named boot_joyeuse.sh by the time.

[androidboot hdr][zImaage.gz-dtb][LOS recovery initrd.img.gz][firmware-mbn-tree.gz]

The repo arrangement is simple too. Basically the branch 'working' is a force pushed workspace. Usually I will leave and branch another
working-2023xxxx branch for each kernel release or important commit. So if there are leave message, it will got wiped too. Please
feel free to leave a issue ticket and reference to specific commit from within my tree in case of question and query. I will response 
as I back online and try to answer questions in my own knowledge. And this is happened once, I wanted to say sorry to Aarqw12 [5] for 
his(?)/her(?) question/comment on the working branch, it is highly likely got losted. And this is also encurrge me to breate this 
front page branch README and make it more visible to other people.

Anyway, welcome to everybody come and visit here please find my changes useful, you are also welcome to refine the changeset 
and make it up-stream, provided that quoting and credit me. I didnt add myself any copy-right-years into those files, but i assumed they are.
It is nice to use my github nickname 99degree as copyright reference (and include full identification as [6] too). 

Please also Feel free to get a glimpse of my youtube channel [7] for Meizu E3 porting LOS-17, and not-that-active twitter [8]

[1] https://github.com/torvalds/linux
[2] https://github.com/map220v/sm7125-mainline
[3] https://www.linaro.org/blog/aosp-on-pixel3-pocof1-running-aosp-with-mainline-kernel-on-form-factor-devices/
[4] https://download.lineageos.org/devices/miatoll/builds
[5] https://github.com/Aarqw12
[6] https://github.com/99degree
[7] https://www.youtube.com/@99degree14
[8] https://twitter.com/99degree2

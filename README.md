# linux
Linux kernel source tree

This repo is tracking latest mainline kernel as updated as possible. The idea is to work with sm7125 soc and mainline [1]. 
The development machine is a XiaoMi redmi note 9 pro int'l version. Currently the working branch is based on latest code
of Linus's public tree with my own periodic merge into local change and git rebase for every push.

## Tested good:
```
UFS
sd card
LCD display and its dsi drm display controller (tianma panel)
touchscreen nt36675 of my own written regmap driver, yay! 
cpufreq
usb peri/adb shell
spi/dma/i2c
pon/pwr button/vol button
lcd backlight
camera_cci bus
camss subsys 
camera s5k5e9 sensor
pd-mapper(software to bringup sound/wifi/bt)
```

## Yet to test 
This repo have a simple philosophy, which is basically minimal change to mainline source. Here are functionality not tested:
```
LCD panel from branded huaxing panel
modem
rmtfs
bluetooth/wifi
sound/mic/headphone/ampifier dai (ported device tree, boots up, not tested)
tas2562 amplifier ic (compiled as modules already)
gpu 3d (display comtroller is working, drm)
camera cmos sensors except s5k5e9
pwm
```

## Do not work atm
```
slpi(sensors, largely broken, not supported)
focaltech touch panel
fingerprint(largely broken, not supported)
```

## Wiki, Wiki, Wiki
Wiki[12] is available for common porting question.

So with rewrite of nt36xxx_spi driver[11], it is barely meet the requirement to Android/LOS/PostmarketOS. Success story of 
mainline similar to xiaomi phone is from Linaro [3] and the overall procedure is compile full Android/LOS by them-selves.

So if there are leave message, please feel free to leave a issue ticket, and then reference to specific commit from within the 
tree in case of question and query. I will response  as I back online and try to answer questions in my own knowledge.

## Freely redistribute
The repo arrangement is simple too. Basically the branch 'working' is a force pushed workspace. Usually latest mainline code will get a local branch-out and leave 
a branch to the repo. Typical name of this local branch is as working-2023xxxx or next-2024xxxx, roughly for each kernel release with important commit. 

Other short-named branches are for integrating as merge rebase. Lists as below are for merge use. So individual changes for particular device driver can be pickup more cleanly for other projects. The important thing is, just like nt36xxx branch, there are some changes stack up into one big change-set and might have different commit id by time-to-time. 
>
	sm7125 
	clk 
	power 
	gpu 
	panel 
	nt36xxx 
	wled 
	camss 
	cmos 
	dm-user 
	misc 
	sound

Anyway, welcome to everybody come and visit here please find my changes useful, you are also welcome to refine the changeset 
and make it up-stream, provided that quoting and credit me. I didnt add myself any copy-right-years into those files, but 
i assumed they are. It is nice to use my github nickname 99degree as copyright reference (and include full identification as [6] too). 

## PostmarkerOS image download for Miatoll
A dual boot Lineageos on ufs and postmarketos on sdcard is available for Joyeuse[9]. And the pmOS binary release is at here[10] for download.
Further trial of booting Android out of sdcard is under attempt, but failed as result boot media not found in first-stage-init.
Again, a cloud build bundle with custom postmarketOS is available for rought testing at [10]

## Afterward
Please also Feel free to get a glimpse of my youtube channel [7] for Meizu E3 porting LOS-17, and not-that-active twitter [8]

 - [1] https://github.com/torvalds/linux
 - [2] https://github.com/map220v/sm7125-mainline
 - [3] https://www.linaro.org/blog/aosp-on-pixel3-pocof1-running-aosp-with-mainline-kernel-on-form-factor-devices/
 - [4] https://download.lineageos.org/devices/miatoll/builds
 - [5] https://github.com/Aarqw12
 - [6] https://github.com/99degree
 - [7] https://www.youtube.com/@99degree14
 - [8] https://twitter.com/99degree2
 - [9] https://www.youtube.com/watch?v=fz7Zj4eY4iY
 - [10] https://github.com/99degree/postmarket-nightly-builds/actions
 - [11] https://github.com/99degree/linux/commits/nt36xxx/
 - [12] https://github.com/99degree/linux/wiki/

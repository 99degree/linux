#ccflags-y += -g

ccflags-y += -Idrivers/media/i2c/mtk/
#ccflags-y += -Idrivers/media/i2c/mtk/frame-sync
ccflags-y += -Idrivers/media/i2c/mtk/virt-sensor

imgsensor-objs := \
        adaptor-hw.o \
        adaptor-drv.o \
        adaptor-i2c.o \
        adaptor-ctrls.o \
        adaptor-ioctl.o \
        adaptor-common-ctrl.o \

obj-m += gc8034_mipi_Sensor.o imgsensor.o
#obj-m += frame-sync/

#ov16a1qofilmmipiraw_Sensor.o
#ov16a1qsunnymipiraw_Sensor.o
#s5k5e8yxmipiraw_Sensor.o
#s5k5e9sunnymipiraw_Sensor.o
#s5kgw1ofilmmipiraw_Sensor.o

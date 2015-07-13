include $(srctree)/drivers/misc/mediatek/Makefile.custom

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))

obj-y += mau.o
#obj-y += spc.o

ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)

obj-y += smi_debug.o

obj-y += smi_common_decouple.o

# drivers/ext_disp

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/misc/mediatek/ext_disp/  \
             -I$(srctree)/drivers/misc/mediatek/video/     \
             -I$(srctree)/drivers/misc/mediatek/videox/$(MTK_PLATFORM)/ \
             -I$(srctree)/drivers/misc/mediatek/dispsys/ \
             -I$(srctree)/drivers/misc/mediatek/hdmi/inc/ \
             -I$(srctree)/drivers/staging/android/ion \
             -I$(srctree)/drivers/misc/mediatek/sync/ \
             -I$(srctree)/drivers/misc/mediatek/cmdq/ \
             -I$(srctree)/drivers/misc/mediatek/hdmi/inc/ \
             -I$(srctree)/drivers/misc/mediatek/lcm/inc \
             -I$(srctree)/drivers/misc/mediatek/hdmi/$(CONFIG_CUSTOM_KERNEL_HDMI)/

ifeq ($(CONFIG_CUSTOM_KERNEL_HDMI),nxp_tda19989)
	ccflags-y += 	-DHDMI_TDA19989
endif

ifeq ($(CONFIG_CUSTOM_KERNEL_HDMI),"Sii8338")
	ccflags-y += 	-DMHL_SII8338
endif

obj-y	:= hdmitx.o debug.o  hdmitx_test.o





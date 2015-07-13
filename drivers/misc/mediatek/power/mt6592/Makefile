include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_PMIC)          += pmic.o upmu_common.o ncp6335.o fan53555.o

ifeq ($(CONFIG_MTK_MT6333_SUPPORT),y)
    obj-$(CONFIG_MTK_PMIC) += mt6333.o
endif

ifeq ($(CONFIG_MTK_TPS61280_SUPPORT),y)
  obj-$(CONFIG_MTK_PMIC) += tps6128x.o
endif

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_hal.o 

ifeq ($(CONFIG_MTK_BQ24296_SUPPORT),y) 
    obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_bq24296.o bq24296.o tbl_helper_func.o
else
    ifeq ($(CONFIG_MTK_BQ24196_SUPPORT),y) 
        obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_bq24196.o bq24196.o tbl_helper_func.o
    else
      ifeq ($(CONFIG_MTK_NCP1851_SUPPORT),y) 
        #obj-$(CONFIG_MTK_SMART_BATTERY) += mt_battery_ncp1851.o mt6320_fuel_gauge_ncp1851.o ncp1851.o
        obj-$(CONFIG_MTK_SMART_BATTERY) += addon/
      else
          ifeq ($(CONFIG_MTK_FAN5402_SUPPORT),y)
               obj-$(CONFIG_MTK_SMART_BATTERY) += fan5402.o charging_hw_fan5402.o
          else ifeq ($(CONFIG_MTK_FAN5405_SUPPORT),y)
               obj-$(CONFIG_MTK_SMART_BATTERY) += fan5405.o charging_hw_fan5405.o
          else
               ifeq ($(CONFIG_MTK_SWCHR_SUPPORT),y)
                    obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_6333.o                
               else
                    obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_pmic.o 
               endif
          endif
      endif
    endif
endif

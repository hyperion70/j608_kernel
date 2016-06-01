#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <linux/kernel.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif
#define CAMERA_POWER_ON_PIN GPIO115
kal_bool searchMainSensor = KAL_TRUE;

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
u32 pinSetIdx = 0;//default main sensor
u32 pinSetIdxTmp = 0;

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4

#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


u32 pinSet[2][8] = {
                    //for main sensor
                    {GPIO_CAMERA_CMRST_PIN,
                        GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,                   /* ON state */
                        GPIO_OUT_ZERO,                  /* OFF state */
                     GPIO_CAMERA_CMPDN_PIN,
                        GPIO_CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                    },
                    //for sub sensor
                    {GPIO_CAMERA_CMRST1_PIN,
                     GPIO_CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     GPIO_CAMERA_CMPDN1_PIN,
                        GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                    },
                   };


    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
		searchMainSensor = KAL_TRUE;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
		searchMainSensor = KAL_FALSE;
    }
   
    //power ON
    if (On) {
		PK_DBG("kdCISModulePowerOn -on:currSensorName=%s\n",currSensorName);
		PK_DBG("kdCISModulePowerOn -on:pinSetIdx=%d\n",pinSetIdx);

		if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV2685_YUV,currSensorName)))
		{
            PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_OV2685_YUV \n");

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
			{
				PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(5);

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D,VOL_1800,mode_name))
            {
                PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(5);

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
			{
				PK_ERR("[CAMERA SENSOR] Fail to enable analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(5);
            
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]){
				//PD pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_ERR("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_ERR("[CAMERA LENS] set gpio failed!! \n");}
				mdelay(5);
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_ERR("[CAMERA LENS] set gpio failed!! \n");}
				mdelay(5);
			}

            //disable inactive sensor
    		if(pinSetIdx == 0 || pinSetIdx == 2) {//disable sub
    			if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
    			if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    			if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
    			if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    			if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
    			if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
    		}
    		else
    		{
    			if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
    			if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
    			if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
    			if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
    			if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
    			if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
    		} 
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_SP2529_YUV,currSensorName)))
        {
            PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_SP2529_YUV \n");

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
			{
				PK_ERR("[CAMERA SENSOR] Fail to enable analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(5);

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
			{
				PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(5);

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D,VOL_1800,mode_name))
            {
                PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(5);

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]){
				//PD pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_ERR("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_ERR("[CAMERA LENS] set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_ERR("[CAMERA LENS] set gpio failed!! \n");}
                mdelay(5);

                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_ERR("[CAMERA LENS] set gpio failed!! \n");}
				mdelay(100);
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_ERR("[CAMERA LENS] set gpio failed!! \n");}
				mdelay(5);
                //RST pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("[CAMERA SENSOR] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_ERR("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_ERR("[CAMERA SENSOR] set gpio failed!! \n");}
				mdelay(5);
				
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_ERR("[CAMERA SENSOR] set gpio failed!! \n");}
				mdelay(10);
			}

            //disable inactive sensor
			if(pinSetIdx == 0 || pinSetIdx == 2) {//disable sub
				if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
				if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
				if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
				if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
			}
			else
			{
				if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
				if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
				if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
				if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
			}
        }
		else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX179_MIPI_RAW,currSensorName)))			  
		  {
		printk("[sym] 123123power on SENSOR_DRVNAME_IMX179_MIPI_RAW\n");
		   printk("kdCISModulePowerOn   SENSOR_DRVNAME_IMX179_MIPI_RAW_2LANE\n");
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
			{
			printk("[CAMERA SENSOR] Fail to enable analog power\n");
			//return -EIO;
			//goto _kdCISModulePowerOn_exit_;
			}
			mdelay(10);
			//DOVDD
			printk("[ON_general 1.8V]sensorIdx:%d \n",SensorIdx);
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
			{
				printk("[CAMERA SENSOR] Fail to enable digital power\n");
				//return -EIO;
				//goto _kdCISModulePowerOn_exit_;
			}
			mdelay(10);
			//DVDD
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
			{
				printk("[CAMERA SENSOR] Fail to enable digital power\n");
				//return -EIO;
				//goto _kdCISModulePowerOn_exit_;
			}
			mdelay(10);
			//AF_VCC
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
			{
				printk("[CAMERA SENSOR] Fail to enable analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(10);
	        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
			{//disable sub
			//printk("imx179_mipi_raw_2lane cmpdn=%d,reset=%d \n",pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMRST]);
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				mdelay(10);
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON]))
					printk("imx179_mipi_raw_2lane set IDX_PS_CMPDN failed!! \n");
				else
					printk("imx179_mipi_raw_2lane set IDX_PS_CMPDN  success!! \n");
				mdelay(1);
				printk("imx179_mipi_raw_2lane cmpdn = %d\n",mt_get_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN]));
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				mdelay(10);
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON]))
					PK_DBG("imx179_mipi_raw_2lane set IDX_PS_CMRST failed!! \n");
				else
					PK_DBG("imx179_mipi_raw_2lane set IDX_PS_CMRST  success!! \n");
				mdelay(1);
				printk("imx179_mipi_raw_2lane  IDX_PS_CMRST = %d\n",mt_get_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST]));
		   if(pinSetIdx == 0) {//disable sub
			   pinSetIdxTmp = 1;
				}
		   else{
			   pinSetIdxTmp = 0;
			}
		   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
				}
				mdelay(1);
			}
		  }	
else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX135_MIPI_RAW,currSensorName)))			  
		  {
		printk("[sym] 123123power on SENSOR_DRVNAME_IMX135_MIPI_RAW\n");
		   printk("kdCISModulePowerOn   SENSOR_DRVNAME_IMX135_MIPI_RAW_2LANE\n");
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
			{
			printk("[CAMERA SENSOR] Fail to enable analog power\n");
			//return -EIO;
			//goto _kdCISModulePowerOn_exit_;
			}
			mdelay(10);
			//DOVDD
			printk("[ON_general 1.8V]sensorIdx:%d \n",SensorIdx);
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
			{
				printk("[CAMERA SENSOR] Fail to enable digital power\n");
				//return -EIO;
				//goto _kdCISModulePowerOn_exit_;
			}
			mdelay(10);
			//DVDD
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
			{
				printk("[CAMERA SENSOR] Fail to enable digital power\n");
				//return -EIO;
				//goto _kdCISModulePowerOn_exit_;
			}
			mdelay(10);
			//AF_VCC
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
			{
				printk("[CAMERA SENSOR] Fail to enable analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(10);
			mt_set_gpio_mode(CAMERA_POWER_ON_PIN,GPIO_CAMERA_AF_EN_PIN_M_GPIO);
       		        mt_set_gpio_dir(CAMERA_POWER_ON_PIN,GPIO_DIR_OUT);
        		mt_set_gpio_out(CAMERA_POWER_ON_PIN,1);
	        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
			{//disable sub
			//printk("imx179_mipi_raw_2lane cmpdn=%d,reset=%d \n",pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMRST]);
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				mdelay(10);
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON]))
					printk("IMX135_mipi_raw_2lane set IDX_PS_CMPDN failed!! \n");
				else
					printk("IMX135_mipi_raw_2lane set IDX_PS_CMPDN  success!! \n");
				mdelay(1);
				printk("IMX135_mipi_raw_2lane cmpdn = %d\n",mt_get_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN]));
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				mdelay(10);
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON]))
					PK_DBG("IMX135_mipi_raw_2lane set IDX_PS_CMRST failed!! \n");
				else
					PK_DBG("IMX135_mipi_raw_2lane set IDX_PS_CMRST  success!! \n");
				mdelay(1);
				printk("IMX1359_mipi_raw_2lane  IDX_PS_CMRST = %d\n",mt_get_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST]));
		   if(pinSetIdx == 0) {//disable sub
			   pinSetIdxTmp = 1;
				}
		   else{
			   pinSetIdxTmp = 0;
			}
		   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			   if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			   if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
			   if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
				}
				mdelay(1);
			}
		  }	
		else  if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV8825_MIPI_RAW,currSensorName)))
        {
            PK_DBG("[sym]kdCISModulePowerOn get in---  SENSOR_DRVNAME_OV8825_MIPI_RAW cmpdn=%d,reset=%d \n",pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMRST]);
            PK_DBG("[ON_general 2.8V]sensorIdx:%d \n",SensorIdx);
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(5);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                PK_DBG("[sym] OV8825_MIPI_RAW cmpdn=%d,reset=%d \n",pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMRST]);
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                mdelay(10);
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON]))
                    PK_DBG("[CAMERA LENS] set IDX_PS_CMPDN failed!! \n");
                else
                    PK_DBG("[sym][CAMERA LENS] set IDX_PS_CMPDN  success!! \n");
                mdelay(1);

                printk("[sym] cmpdn = %d\n",mt_get_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN]));
                //	mt_set_gpio_out(10,1);

                //RST pin
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
                mdelay(10);
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON]))
                    PK_DBG("[CAMERA SENSOR] set IDX_PS_CMRST failed!! \n");
                else
                    PK_DBG("[sym][CAMERA LENS] set IDX_PS_CMRST  success!! \n");
                mdelay(1);

                printk("[sym] IDX_PS_CMRST = %d\n",mt_get_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST]));
                //	mt_set_gpio_out(9,1);
                mdelay(1);
            }

            //disable inactive sensor
            if(pinSetIdx == 0) {//disable sub
                pinSetIdxTmp = 1;
            }
            else{
                pinSetIdxTmp = 0;
            }
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdxTmp][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_mode(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMRST],pinSet[pinSetIdxTmp][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                if(mt_set_gpio_out(pinSet[pinSetIdxTmp][IDX_PS_CMPDN],pinSet[pinSetIdxTmp][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
            }
        }
	
	else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV8865_MIPI_RAW,currSensorName)))
	{
		if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}

		if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
        
		mdelay(50);
	
		PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_OV8865_MIPI_RAW \n");
		PK_DBG("[ON_general 2.8V]sensorIdx:%d \n",SensorIdx);

		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}
		mdelay(10);
		
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}
		mdelay(10);

		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
		{
			 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			 //return -EIO;
			 goto _kdCISModulePowerOn_exit_;
		}
		mdelay(10);

		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
		}
		mdelay(10);

        
		mdelay(50);
		//PDN/STBY pin
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
		{
		
			//RST pin  
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			mdelay(5);
		
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			mdelay(5);

		}

	//disable inactive sensor
	if(pinSetIdx == 0 || pinSetIdx == 2) {//disable sub
		if (GPIO_CAMERA_INVALID != pinSet[1][IDX_PS_CMRST]) {
			if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
			if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
		}
	}
	else {
		if (GPIO_CAMERA_INVALID != pinSet[0][IDX_PS_CMRST]) {
			if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
			if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
		}
		if (GPIO_CAMERA_INVALID != pinSet[2][IDX_PS_CMRST]) {
		   /*
			if(mt_set_gpio_mode(pinSet[2][IDX_PS_CMRST],pinSet[2][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_mode(pinSet[2][IDX_PS_CMPDN],pinSet[2][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[2][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_dir(pinSet[2][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[2][IDX_PS_CMRST],pinSet[2][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
			if(mt_set_gpio_out(pinSet[2][IDX_PS_CMPDN],pinSet[2][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
		   */
		}
	}

}		
    }
    else {//power OFF
		PK_DBG("kdCISModulePowerOn -off:currSensorName=%s\n",currSensorName);
		if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV2685_YUV,currSensorName)))
		{
            PK_DBG("kdCISModulePower--off get in---SENSOR_DRVNAME_OV2685_YUV \n");
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_ERR("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_ERR("[CAMERA SENSOR] set gpio failed!! \n");} //high == pwnd sensor
            }

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
            {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name))
            {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_SP2529_YUV,currSensorName)))
        {
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_ERR("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_ERR("[CAMERA SENSOR] set gpio failed!! \n");} //high == pwnd sensor
                mdelay(1); 
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_ERR("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_ERR("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor             
            }

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name))
            {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
            {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}
        }
		else  if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX179_MIPI_RAW,currSensorName)))	
		 {
			printk("[sym] power off SENSOR_DRVNAME_IMX179_MIPI_RAW\n");
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
			{
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_ERR("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_ERR("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_ERR("[CAMERA SENSOR] set gpio failed!! \n");} //high == pwnd sensor
			mdelay(1);
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			mdelay(5);
			}
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name))
			{
			PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name)) {
			PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name)) {
			PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
			}
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
			{
			PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
			}			
		}
else  if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX135_MIPI_RAW,currSensorName)))	
		 {
			mt_set_gpio_out(CAMERA_POWER_ON_PIN,0);
			printk("[sym] power off SENSOR_DRVNAME_IMX135_MIPI_RAW\n");
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
			{
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			mdelay(5);
			}
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name))
			{
			PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name)) {
			PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name)) {
			PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
			}
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
			{
			PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
			//return -EIO;
			goto _kdCISModulePowerOn_exit_;
			}			
		}
	else   if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV8825_MIPI_RAW,currSensorName)))
        {
            PK_DBG("kdCISModulePower--off get in---SENSOR_DRVNAME_OV8825_MIPI_RAW reset = %d,pwd=%d \n",pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMPDN]);
            //reset pull down
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
            }

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //PDN pull down
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
            }
        }
	else  if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV8865_MIPI_RAW,currSensorName)))
		{
			PK_DBG("kdCISModulePower--off get in---SENSOR_DRVNAME_OV8865_MIPI_RAW \n");

			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor

			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

		}


        //For Power Saving
		if(pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF] != GPIO_OUT_ZERO)
		{
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO);
		}
		if(pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF] != GPIO_OUT_ZERO)
		{
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ZERO);
		}

		if(pinSet[1-pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF] != GPIO_OUT_ZERO)
		{
			   mt_set_gpio_out(pinSet[1-pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO);
		}
		if(pinSet[1-pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF] != GPIO_OUT_ZERO)
		{
			   mt_set_gpio_out(pinSet[1-pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ZERO);
		}

		//~For Power Saving
    }
	return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);


//!--
//





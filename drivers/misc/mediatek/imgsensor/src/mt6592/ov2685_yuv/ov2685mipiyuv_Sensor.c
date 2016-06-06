/* 613Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 10 12 2010 sean.cheng
 * [ALPS00021722] [Need Patch] [Volunteer Patch][Camera]MT6573 Camera related function
 * .rollback the lib3a for mt6573 camera related files
 *
 * 09 10 2010 jackie.su
 * [ALPS00002279] [Need Patch] [Volunteer Patch] ALPS.Wxx.xx Volunteer patch for
 * .alps dual sensor
 *
 * 09 02 2010 jackie.su
 * [ALPS00002279] [Need Patch] [Volunteer Patch] ALPS.Wxx.xx Volunteer patch for
 * .roll back dual sensor
 *
 * 07 27 2010 sean.cheng
 * [ALPS00003112] [Need Patch] [Volunteer Patch] ISP/Sensor driver modification for Customer support
 * .1. add master clock switcher 
 *  2. add master enable/disable 
 *  3. add dummy line/pixel for sensor 
 *  4. add sensor driving current setting
 *
 * 07 01 2010 sean.cheng
 * [ALPS00121215][Camera] Change color when switch low and high 
 * .Add video delay frame.
 *
 * 07 01 2010 sean.cheng
 * [ALPS00002805][Need Patch] [Volunteer Patch]10X Patch for DS269 Video Frame Rate 
 * .Change the sensor clock to let frame rate to be 30fps in vidoe mode
 *
 * 06 13 2010 sean.cheng
 * [ALPS00002514][Need Patch] [Volunteer Patch] ALPS.10X.W10.11 Volunteer patch for E1k Camera 
 * .
 * 1. Add set zoom factor and capdelay frame for YUV sensor 
 * 2. Modify e1k sensor setting
 *
 * 05 25 2010 sean.cheng
 * [ALPS00002250][Need Patch] [Volunteer Patch] ALPS.10X.W10.11 Volunteer patch for YUV video frame rate 
 * .
 * Add 15fps option for video mode
 *
 * 05 03 2010 sean.cheng
 * [ALPS00001357][Meta]CameraTool 
 * .
 * Fix OV2685 MIPI YUV sensor frame rate to 30fps in vidoe mode
 *
 * Mar 4 2010 mtk70508
 * [DUMA00154792] Sensor driver
 * 
 *
 * Mar 4 2010 mtk70508
 * [DUMA00154792] Sensor driver
 * 
 *
 * Mar 1 2010 mtk01118
 * [DUMA00025869] [Camera][YUV I/F & Query feature] check in camera code
 * 
 *
 * Feb 24 2010 mtk01118
 * [DUMA00025869] [Camera][YUV I/F & Query feature] check in camera code
 * 
 *
 * Nov 24 2009 mtk02204
 * [DUMA00015869] [Camera Driver] Modifiy camera related drivers for dual/backup sensor/lens drivers.
 * 
 *
 * Oct 29 2009 mtk02204
 * [DUMA00015869] [Camera Driver] Modifiy camera related drivers for dual/backup sensor/lens drivers.
 * 
 *
 * Oct 27 2009 mtk02204
 * [DUMA00015869] [Camera Driver] Modifiy camera related drivers for dual/backup sensor/lens drivers.
 * 
 *
 * Aug 13 2009 mtk01051
 * [DUMA00009217] [Camera Driver] CCAP First Check In
 * 
 *
 * Aug 5 2009 mtk01051
 * [DUMA00009217] [Camera Driver] CCAP First Check In
 * 
 *
 * Jul 17 2009 mtk01051
 * [DUMA00009217] [Camera Driver] CCAP First Check In
 * 
 *
 * Jul 7 2009 mtk01051
 * [DUMA00008051] [Camera Driver] Add drivers for camera high ISO binning mode.
 * Add ISO query info for Sensor
 *
 * May 18 2009 mtk01051
 * [DUMA00005921] [Camera] LED Flashlight first check in
 * 
 *
 * May 16 2009 mtk01051
 * [DUMA00005921] [Camera] LED Flashlight first check in
 * 
 *
 * May 16 2009 mtk01051
 * [DUMA00005921] [Camera] LED Flashlight first check in
 * 
 *
 * Apr 7 2009 mtk02204
 * [DUMA00004012] [Camera] Restructure and rename camera related custom folders and folder name of came
 * 
 *
 * Mar 27 2009 mtk02204
 * [DUMA00002977] [CCT] First check in of MT6516 CCT drivers
 *
 *
 * Mar 25 2009 mtk02204
 * [DUMA00111570] [Camera] The system crash after some operations
 *
 *
 * Mar 20 2009 mtk02204
 * [DUMA00002977] [CCT] First check in of MT6516 CCT drivers
 *
 *
 * Mar 2 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 *
 *
 * Feb 24 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 *
 *
 * Dec 27 2008 MTK01813
 * DUMA_MBJ CheckIn Files
 * created by clearfsimport
 *
 * Dec 10 2008 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 *
 *
 * Oct 27 2008 mtk01051
 * [DUMA00000851] Camera related drivers check in
 * Modify Copyright Header
 *
 * Oct 24 2008 mtk02204
 * [DUMA00000851] Camera related drivers check in
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
//#include <windows.h>
//#include <memory.h>
//#include <nkintr.h>
//#include <ceddk.h>
//#include <ceddk_exp.h>

//#include "kal_release.h"
//#include "i2c_exp.h"
//#include "gpio_exp.h"
//#include "msdk_exp.h"
//#include "msdk_sensor_exp.h"
//#include "msdk_isp_exp.h"
//#include "base_regs.h"
//#include "Sensor.h"
//#include "camera_sensor_para.h"
//#include "CameraCustomized.h"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "ov2685mipiyuv_Sensor.h"
#include "ov2685mipiyuv_Camera_Sensor_para.h"
#include "ov2685mipiyuv_CameraCustomized.h"

#define OV2685MIPIYUV_DEBUG
#ifdef OV2685MIPIYUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

#define __SLT_DRV_OV3660_BURST_SHOTS__	//Debug: Brightless became lower and lower in burst shot mode
#ifdef __SLT_DRV_OV3660_BURST_SHOTS__	//wujinyou, 2011.11.21
static kal_uint8 preview_init_flag = 0;
#endif


static DEFINE_SPINLOCK(ov2685_drv_lock);

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define OV2685_MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,OV2685_MIPI_WRITE_ID)
#define OV2685_MIPI_write_cmos_sensor_2(addr, para, bytes) iWriteReg((u16) addr , (u32) para ,bytes,OV2685_MIPI_WRITE_ID)
kal_uint16 OV2685_MIPI_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
iReadReg((u16) addr ,(u8*)&get_byte,OV2685_MIPI_WRITE_ID);
return get_byte;
}


/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/


#define	OV2685_MIPI_LIMIT_EXPOSURE_LINES				        (1253)
#define	OV2685_MIPI_VIDEO_NORMALMODE_30FRAME_RATE       (30)
#define	OV2685_MIPI_VIDEO_NORMALMODE_FRAME_RATE         (15)
#define	OV2685_MIPI_VIDEO_NIGHTMODE_FRAME_RATE          (7.5)
#define BANDING50_30HZ
/* Global Valuable */

static kal_uint32 zoom_factor = 0; 
static kal_uint8 OV2685_MIPI_exposure_line_h = 0,OV2685_MIPI_exposure_line_m = 0, OV2685_MIPI_exposure_line_l = 0;

static kal_bool OV2685_MIPI_gPVmode = KAL_TRUE; //PV size or Full size
static kal_bool OV2685_MIPI_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool OV2685_MIPI_sensor_cap_state = KAL_FALSE; //Preview or Capture

static kal_uint16 OV2685_MIPI_dummy_pixels=0, OV2685_MIPI_dummy_lines=0;

static kal_uint16 OV2685_MIPI_exposure_lines=0, OV2685_MIPI_extra_exposure_lines = 0;


static kal_int8 OV2685_MIPI_DELAY_AFTER_PREVIEW = -1;

static kal_uint8 OV2685_MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;  //Wonder add

/****** OVT 6-18******/
static kal_uint16 OV2685_MIPI_Capture_Max_Gain16= 6*16;
static kal_uint16 OV2685_MIPI_Capture_Gain16=0 ;    
static kal_uint16 OV2685_MIPI_Capture_Shutter=0;
static kal_uint16 OV2685_MIPI_Capture_Extra_Lines=0;

static kal_uint16  OV2685_MIPI_PV_Dummy_Pixels =0,OV2685_MIPI_Capture_Dummy_Pixels =0, OV2685_MIPI_Capture_Dummy_Lines =0;
static kal_uint16  OV2685_MIPI_PV_Gain16 = 0;
static kal_uint16  OV2685_MIPI_PV_Shutter = 0;
static kal_uint16  OV2685_MIPI_PV_Extra_Lines = 0;

/*
kal_uint16 OV2685_MIPI_sensor_gain_base=0,OV2685_MIPI_FAC_SENSOR_REG=0,OV2685_MIPI_iOV2685_MIPI_Mode=0,OV2685_MIPI_max_exposure_lines=0;
kal_uint32 OV2685_MIPI_capture_pclk_in_M=520,OV2685_MIPI_preview_pclk_in_M=390,OV2685_MIPI_PV_dummy_pixels=0,OV2685_MIPI_PV_dummy_lines=0,OV2685_MIPI_isp_master_clock=0;
*/
kal_uint16 OV2685_MIPI_iOV2685_MIPI_Mode=0;
kal_uint32 OV2685_MIPI_capture_pclk_in_M=520,OV2685_MIPI_preview_pclk_in_M=390,OV2685_MIPI_PV_dummy_pixels=0,OV2685_MIPI_PV_dummy_lines=0,OV2685_MIPI_isp_master_clock=0;

static kal_uint32  OV2685_MIPI_sensor_pclk=390;
static kal_bool OV2685_MIPI_AWB_ENABLE = KAL_TRUE; 
static kal_bool OV2685_MIPI_AE_ENABLE = KAL_TRUE; 

static kal_uint32 Capture_Shutter = 0; 
static kal_uint32 Capture_Gain = 0; 

#if WINMO_USE
kal_uint8 OV2685_MIPI_sensor_write_I2C_address = OV2685_MIPI_WRITE_ID;
kal_uint8 OV2685_MIPI_sensor_read_I2C_address = OV2685_MIPI_READ_ID;

UINT32 OV2685GPIOBaseAddr;
HANDLE OV2685hGPIO;
HANDLE OV2685hDrvI2C;
I2C_TRANSACTION OV2685I2CConfig;
#endif 
UINT8 OV2685_MIPI_PixelClockDivider=0;

//SENSOR_REG_STRUCT OV2685SensorCCT[FACTORY_END_ADDR]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
//SENSOR_REG_STRUCT OV2685SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
//	camera_para.SENSOR.cct	SensorCCT	=> SensorCCT
//	camera_para.SENSOR.reg	SensorReg
MSDK_SENSOR_CONFIG_STRUCT OV2685SensorConfigData;

static struct
{
	//kal_uint8   Banding;
	kal_bool	  NightMode;
	kal_bool	  VideoMode;
	kal_uint16  Fps;
	kal_uint16  ShutterStep;
	kal_uint8   IsPVmode;
	kal_uint32  PreviewDummyPixels;
	kal_uint32  PreviewDummyLines;
	kal_uint32  CaptureDummyPixels;
	kal_uint32  CaptureDummyLines;
	kal_uint32  PreviewPclk;
	kal_uint32  CapturePclk;
	kal_uint32  ZsdturePclk;
	kal_uint32  PreviewShutter;
	kal_uint32  SensorGain;
	kal_bool    	manualAEStart;
	kal_bool    	userAskAeLock;
       kal_bool    	userAskAwbLock;
	kal_uint32      currentExposureTime;
       kal_uint32      currentShutter;
	kal_uint32      currentextshutter;
       kal_uint32      currentAxDGain;
	kal_uint32  	sceneMode;
       unsigned char isoSpeed;
	unsigned char zsd_flag;
	kal_uint32      AF_window_x;
	kal_uint32      AF_window_y;
	unsigned char   awbMode;
	UINT16 iWB;
	//OV2685MIPI_SENSOR_MODE SensorMode;
} OV2685MIPISensor;

#if WINMO_USE
void OV2685_MIPI_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	OV2685I2CConfig.operation=I2C_OP_WRITE;
	OV2685I2CConfig.slaveAddr=OV2685_MIPI_sensor_write_I2C_address>>1;
	OV2685I2CConfig.transfer_num=1;	/* TRANSAC_LEN */
	OV2685I2CConfig.transfer_len=3;
	OV2685I2CConfig.buffer[0]=(UINT8)(addr>>8);
	OV2685I2CConfig.buffer[1]=(UINT8)(addr&0xFF);
	OV2685I2CConfig.buffer[2]=(UINT8)para;
	DRV_I2CTransaction(OV2685hDrvI2C, &OV2685I2CConfig);

}	/* OV2685_MIPI_write_cmos_sensor() */

kal_uint32 OV2685_MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint8 get_byte=0xFF;

	OV2685I2CConfig.operation=I2C_OP_WRITE;
	OV2685I2CConfig.slaveAddr=OV2685_MIPI_sensor_write_I2C_address>>1;
	OV2685I2CConfig.transfer_num=1;	/* TRANSAC_LEN */
	OV2685I2CConfig.transfer_len=2;
	OV2685I2CConfig.buffer[0]=(UINT8)(addr>>8);
	OV2685I2CConfig.buffer[1]=(UINT8)(addr&0xFF);
	DRV_I2CTransaction(OV2685hDrvI2C, &OV2685I2CConfig);

	OV2685I2CConfig.operation=I2C_OP_READ;
	OV2685I2CConfig.slaveAddr=OV2685_MIPI_sensor_read_I2C_address>>1;
	OV2685I2CConfig.transfer_num=1;	/* TRANSAC_LEN */
	OV2685I2CConfig.transfer_len=1;
	DRV_I2CTransaction(OV2685hDrvI2C, &OV2685I2CConfig);
	get_byte=OV2685I2CConfig.buffer[0];

	return get_byte;
}	/* OV2685_MIPI_read_cmos_sensor() */
#endif 

void OV2685MIPIGetExifInfo(UINT32 exifAddr)
{
	 SENSORDB("[OV2685MIPI]enter OV2685MIPIGetExifInfo function\n");
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 20; //Aperture
    pExifInfo->AEISOSpeed = OV2685MIPISensor.isoSpeed;
    pExifInfo->AWBMode = OV2685MIPISensor.awbMode;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->CapExposureTime = OV2685MIPISensor.currentExposureTime;
    pExifInfo->RealISOValue = OV2685MIPISensor.isoSpeed;
	SENSORDB("[OV2685MIPI]exit OV2685MIPIGetExifInfo function %d\n", pExifInfo->CapExposureTime);
}

void OV2685_MIPI_set_dummy(kal_uint16 pixels, kal_uint16 lines)
{
    if (OV2685_MIPI_gPVmode)
    {
    	  pixels = pixels+0x06ac; 
        OV2685_MIPI_write_cmos_sensor(0x380D,( pixels&0xFF));         
        OV2685_MIPI_write_cmos_sensor(0x380C,(( pixels&0xFF00)>>8)); 
      
        lines= lines+0x0284; 
        OV2685_MIPI_write_cmos_sensor(0x380F,(lines&0xFF));       
        OV2685_MIPI_write_cmos_sensor(0x380E,((lines&0xFF00)>>8));  
    	
    }
    else
    {
        pixels = pixels+0x06a4; 
        OV2685_MIPI_write_cmos_sensor(0x380D,( pixels&0xFF));         
        OV2685_MIPI_write_cmos_sensor(0x380C,(( pixels&0xFF00)>>8)); 
      
        lines= lines+0x050e; 
        OV2685_MIPI_write_cmos_sensor(0x380F,(lines&0xFF));       
        OV2685_MIPI_write_cmos_sensor(0x380E,((lines&0xFF00)>>8));    	
    }    
}    /* OV2685_MIPI_set_dummy */
void OV2685_MIPI_globle_init(void)
{
    SENSORDB("OV2685_MIPI_global_init begin \n");
    OV2685_MIPI_write_cmos_sensor(0x0100,0x00);

    OV2685_MIPI_write_cmos_sensor(0x3002,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3016,0x1c);
    OV2685_MIPI_write_cmos_sensor(0x3018,0x44);
    OV2685_MIPI_write_cmos_sensor(0x301d,0xf0);
    OV2685_MIPI_write_cmos_sensor(0x3020,0x00);        
    
    OV2685_MIPI_write_cmos_sensor(0x3501,0x54);
    OV2685_MIPI_write_cmos_sensor(0x3502,0x30);
    OV2685_MIPI_write_cmos_sensor(0x3503,0x00);
    OV2685_MIPI_write_cmos_sensor(0x350b,0x2b);
    OV2685_MIPI_write_cmos_sensor(0x3600,0xb4);
    OV2685_MIPI_write_cmos_sensor(0x3603,0x35);
    OV2685_MIPI_write_cmos_sensor(0x3604,0x24);
    OV2685_MIPI_write_cmos_sensor(0x3605,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3620,0x26);
    OV2685_MIPI_write_cmos_sensor(0x3621,0x37);
    OV2685_MIPI_write_cmos_sensor(0x3622,0x04);
    OV2685_MIPI_write_cmos_sensor(0x3628,0x10);
    OV2685_MIPI_write_cmos_sensor(0x3705,0x3c);
    OV2685_MIPI_write_cmos_sensor(0x370a,0x23);
    OV2685_MIPI_write_cmos_sensor(0x370c,0x50);
    OV2685_MIPI_write_cmos_sensor(0x370d,0xc0);
    OV2685_MIPI_write_cmos_sensor(0x3717,0x58);
    OV2685_MIPI_write_cmos_sensor(0x3718,0x88);
    OV2685_MIPI_write_cmos_sensor(0x3720,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3721,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3722,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3723,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3738,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3781,0x80);
    OV2685_MIPI_write_cmos_sensor(0x3789,0x60);
    
    OV2685_MIPI_write_cmos_sensor(0x3800,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3801,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3802,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3803,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3804,0x06);
    OV2685_MIPI_write_cmos_sensor(0x3805,0x4f);
    OV2685_MIPI_write_cmos_sensor(0x3806,0x04);
    OV2685_MIPI_write_cmos_sensor(0x3807,0xbf);
    OV2685_MIPI_write_cmos_sensor(0x3808,0x03);
    OV2685_MIPI_write_cmos_sensor(0x3809,0x20);
    OV2685_MIPI_write_cmos_sensor(0x380a,0x02);
    OV2685_MIPI_write_cmos_sensor(0x380b,0x58);
    OV2685_MIPI_write_cmos_sensor(0x3810,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3811,0x04);
    OV2685_MIPI_write_cmos_sensor(0x3812,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3813,0x04);
    OV2685_MIPI_write_cmos_sensor(0x3814,0x31);
    OV2685_MIPI_write_cmos_sensor(0x3815,0x31);
    OV2685_MIPI_write_cmos_sensor(0x3819,0x04);
    
    OV2685_MIPI_write_cmos_sensor(0x3820,0xc2);//c2
    OV2685_MIPI_write_cmos_sensor(0x3821,0x05);//01
    
    OV2685_MIPI_write_cmos_sensor(0x3082,0x2c);
    OV2685_MIPI_write_cmos_sensor(0x3083,0x03);
    OV2685_MIPI_write_cmos_sensor(0x3084,0x0f);
    OV2685_MIPI_write_cmos_sensor(0x3085,0x03);
    OV2685_MIPI_write_cmos_sensor(0x3086,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3087,0x00);
    
    OV2685_MIPI_write_cmos_sensor(0x380c,0x06);
    OV2685_MIPI_write_cmos_sensor(0x380d,0xac);
    OV2685_MIPI_write_cmos_sensor(0x380e,0x02);
    OV2685_MIPI_write_cmos_sensor(0x380f,0x84);
    
    OV2685_MIPI_write_cmos_sensor(0x3a02,0x81);
    OV2685_MIPI_write_cmos_sensor(0x3a06,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3a07,0xc1);
    OV2685_MIPI_write_cmos_sensor(0x3a08,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3a09,0xa1);
    
    OV2685_MIPI_write_cmos_sensor(0x3a0e,0x02);
    OV2685_MIPI_write_cmos_sensor(0x3a0f,0x43);
    OV2685_MIPI_write_cmos_sensor(0x3a10,0x02);
    OV2685_MIPI_write_cmos_sensor(0x3a11,0x84);
    
    OV2685_MIPI_write_cmos_sensor(0x3a00,0x43);
    OV2685_MIPI_write_cmos_sensor(0x382a,0x09);
    OV2685_MIPI_write_cmos_sensor(0x3a0a,0x07);
    OV2685_MIPI_write_cmos_sensor(0x3a0b,0x8c);
    OV2685_MIPI_write_cmos_sensor(0x3a0c,0x07);
    OV2685_MIPI_write_cmos_sensor(0x3a0d,0x8c);
    
    OV2685_MIPI_write_cmos_sensor(0x4000,0x81);
    OV2685_MIPI_write_cmos_sensor(0x4001,0x40);
    OV2685_MIPI_write_cmos_sensor(0x4008,0x00);
    OV2685_MIPI_write_cmos_sensor(0x4009,0x03);
    
    OV2685_MIPI_write_cmos_sensor(0x4300,0x32);
    OV2685_MIPI_write_cmos_sensor(0x430e,0x00);
    OV2685_MIPI_write_cmos_sensor(0x4602,0x02);
    OV2685_MIPI_write_cmos_sensor(0x4837,0x1e);
    
    OV2685_MIPI_write_cmos_sensor(0x5000,0xff);
    OV2685_MIPI_write_cmos_sensor(0x5001,0x05);
    OV2685_MIPI_write_cmos_sensor(0x5002,0x33); //32
    OV2685_MIPI_write_cmos_sensor(0x5003,0x04);
    OV2685_MIPI_write_cmos_sensor(0x5004,0xff);
    OV2685_MIPI_write_cmos_sensor(0x5005,0x12);

    OV2685_MIPI_write_cmos_sensor(0x5606,0x20);//0x24
    OV2685_MIPI_write_cmos_sensor(0x5605,0x00);//0x10

    
    OV2685_MIPI_write_cmos_sensor(0x0100,0x01);
    
    OV2685_MIPI_write_cmos_sensor(0x5180,0xf4);//f4); //f4
    OV2685_MIPI_write_cmos_sensor(0x5181,0x11);//11);
    OV2685_MIPI_write_cmos_sensor(0x5182,0x41);//41);
    OV2685_MIPI_write_cmos_sensor(0x5183,0x42);//42);
    OV2685_MIPI_write_cmos_sensor(0x5184,0x7c);//81);
    OV2685_MIPI_write_cmos_sensor(0x5185,0x65);//67);
    OV2685_MIPI_write_cmos_sensor(0x5186,0x85);//85);
    OV2685_MIPI_write_cmos_sensor(0x5187,0x14);//90);
    OV2685_MIPI_write_cmos_sensor(0x5188,0x13);//11);
    OV2685_MIPI_write_cmos_sensor(0x5189,0x15);//15);
    OV2685_MIPI_write_cmos_sensor(0x518a,0x11);//11);
    OV2685_MIPI_write_cmos_sensor(0x518b,0x57);//60); //  4d);
    OV2685_MIPI_write_cmos_sensor(0x518c,0x33);//33);
    OV2685_MIPI_write_cmos_sensor(0x518d,0xf8);//f8);
    OV2685_MIPI_write_cmos_sensor(0x518e,0x4 );//04);
    OV2685_MIPI_write_cmos_sensor(0x518f,0x7f);//7f);
    OV2685_MIPI_write_cmos_sensor(0x5190,0x40);//40);
    OV2685_MIPI_write_cmos_sensor(0x5191,0x5f);//5f);
    OV2685_MIPI_write_cmos_sensor(0x5192,0x40);//40);
    OV2685_MIPI_write_cmos_sensor(0x5193,0xff);//ff);
    OV2685_MIPI_write_cmos_sensor(0x5194,0x40);//40);
    OV2685_MIPI_write_cmos_sensor(0x5195,0x5 );//04);
    OV2685_MIPI_write_cmos_sensor(0x5196,0xc5);//0e);
    OV2685_MIPI_write_cmos_sensor(0x5197,0x4 );//04);
    OV2685_MIPI_write_cmos_sensor(0x5198,0x0 );//00);
    OV2685_MIPI_write_cmos_sensor(0x5199,0x6 );//07);
    OV2685_MIPI_write_cmos_sensor(0x519a,0x43);//e8);
    OV2685_MIPI_write_cmos_sensor(0x519b,0x10);//10);

/*
    OV2685_MIPI_write_cmos_sensor(0x5180,0xf4);
    OV2685_MIPI_write_cmos_sensor(0x5181,0x11);
    OV2685_MIPI_write_cmos_sensor(0x5182,0x41);
    OV2685_MIPI_write_cmos_sensor(0x5183,0x42);
    OV2685_MIPI_write_cmos_sensor(0x5184,0x79);
    OV2685_MIPI_write_cmos_sensor(0x5185,0x5b);
    OV2685_MIPI_write_cmos_sensor(0x5186,0x9d);
    OV2685_MIPI_write_cmos_sensor(0x5187,0xa6);
    OV2685_MIPI_write_cmos_sensor(0x5188,0x13);
    OV2685_MIPI_write_cmos_sensor(0x5189,0x1b);
    OV2685_MIPI_write_cmos_sensor(0x518a,0x16);
    OV2685_MIPI_write_cmos_sensor(0x518b,0x47);
    OV2685_MIPI_write_cmos_sensor(0x518c,0x25);
    OV2685_MIPI_write_cmos_sensor(0x518d,0xf8);
    OV2685_MIPI_write_cmos_sensor(0x518e,0x04);
    OV2685_MIPI_write_cmos_sensor(0x518f,0x7f);
    OV2685_MIPI_write_cmos_sensor(0x5190,0x40);
    OV2685_MIPI_write_cmos_sensor(0x5191,0x5f);
    OV2685_MIPI_write_cmos_sensor(0x5192,0x40);
    OV2685_MIPI_write_cmos_sensor(0x5193,0xff);
    OV2685_MIPI_write_cmos_sensor(0x5194,0x40);
    OV2685_MIPI_write_cmos_sensor(0x5195,0x05);
    OV2685_MIPI_write_cmos_sensor(0x5196,0xf2);
    OV2685_MIPI_write_cmos_sensor(0x5197,0x04);
    OV2685_MIPI_write_cmos_sensor(0x5198,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5199,0x06);
    OV2685_MIPI_write_cmos_sensor(0x519a,0xc);
    OV2685_MIPI_write_cmos_sensor(0x519b,0x10);

    OV2685_MIPI_write_cmos_sensor(0x5180,0xf4);
    OV2685_MIPI_write_cmos_sensor(0x5181,0x11);
    OV2685_MIPI_write_cmos_sensor(0x5182,0x41);
    OV2685_MIPI_write_cmos_sensor(0x5183,0x42);
    OV2685_MIPI_write_cmos_sensor(0x5184,0x78);
    OV2685_MIPI_write_cmos_sensor(0x5185,0x52);
    OV2685_MIPI_write_cmos_sensor(0x5186,0x86);
    OV2685_MIPI_write_cmos_sensor(0x5187,0xa6);
    OV2685_MIPI_write_cmos_sensor(0x5188,0x0c);
    OV2685_MIPI_write_cmos_sensor(0x5189,0x0e);
    OV2685_MIPI_write_cmos_sensor(0x518a,0x0d);
    OV2685_MIPI_write_cmos_sensor(0x518b,0x4f);
    OV2685_MIPI_write_cmos_sensor(0x518c,0x3c);
    OV2685_MIPI_write_cmos_sensor(0x518d,0xf8);
    OV2685_MIPI_write_cmos_sensor(0x518e,0x04);
    OV2685_MIPI_write_cmos_sensor(0x518f,0x7f);
    OV2685_MIPI_write_cmos_sensor(0x5190,0x40);
    OV2685_MIPI_write_cmos_sensor(0x5191,0x5f);
    OV2685_MIPI_write_cmos_sensor(0x5192,0x40);
    OV2685_MIPI_write_cmos_sensor(0x5193,0xff);
    OV2685_MIPI_write_cmos_sensor(0x5194,0x40);
    OV2685_MIPI_write_cmos_sensor(0x5195,0x05);
    OV2685_MIPI_write_cmos_sensor(0x5196,0xa7);
    OV2685_MIPI_write_cmos_sensor(0x5197,0x04);
    OV2685_MIPI_write_cmos_sensor(0x5198,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5199,0x06);
    OV2685_MIPI_write_cmos_sensor(0x519a,0xac);
    OV2685_MIPI_write_cmos_sensor(0x519b,0x04);
*/

    
    OV2685_MIPI_write_cmos_sensor(0x5200,0x09);
    OV2685_MIPI_write_cmos_sensor(0x5201,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5202,0x06);
    OV2685_MIPI_write_cmos_sensor(0x5203,0x20);
    OV2685_MIPI_write_cmos_sensor(0x5204,0x41);
    OV2685_MIPI_write_cmos_sensor(0x5205,0x16);
    OV2685_MIPI_write_cmos_sensor(0x5206,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5207,0x05);
    OV2685_MIPI_write_cmos_sensor(0x520b,0x30);
    OV2685_MIPI_write_cmos_sensor(0x520c,0x75);
    OV2685_MIPI_write_cmos_sensor(0x520d,0x00);
    OV2685_MIPI_write_cmos_sensor(0x520e,0x30);
    OV2685_MIPI_write_cmos_sensor(0x520f,0x75);
    OV2685_MIPI_write_cmos_sensor(0x5210,0x00);
    
    OV2685_MIPI_write_cmos_sensor(0x5280,0x17);
    OV2685_MIPI_write_cmos_sensor(0x5281,0x03);
    OV2685_MIPI_write_cmos_sensor(0x5282,0x06);
    OV2685_MIPI_write_cmos_sensor(0x5283,0x0a);
    OV2685_MIPI_write_cmos_sensor(0x5284,0x10);
    OV2685_MIPI_write_cmos_sensor(0x5285,0x18);
    OV2685_MIPI_write_cmos_sensor(0x5286,0x20);
    OV2685_MIPI_write_cmos_sensor(0x5287,0x10);
    OV2685_MIPI_write_cmos_sensor(0x5288,0x02);
    OV2685_MIPI_write_cmos_sensor(0x5289,0x12);
    
    OV2685_MIPI_write_cmos_sensor(0x5300,0xc5);
    OV2685_MIPI_write_cmos_sensor(0x5301,0xa0);
    OV2685_MIPI_write_cmos_sensor(0x5302,0x02);
    OV2685_MIPI_write_cmos_sensor(0x5303,0x14);
    OV2685_MIPI_write_cmos_sensor(0x5304,0x18);
    OV2685_MIPI_write_cmos_sensor(0x5305,0x30);
    OV2685_MIPI_write_cmos_sensor(0x5306,0x60);
    OV2685_MIPI_write_cmos_sensor(0x5307,0xc0);
    OV2685_MIPI_write_cmos_sensor(0x5308,0x82);
    OV2685_MIPI_write_cmos_sensor(0x5309,0x26); //00
    OV2685_MIPI_write_cmos_sensor(0x530a,0x26); //26
    OV2685_MIPI_write_cmos_sensor(0x530b,0x02);
    OV2685_MIPI_write_cmos_sensor(0x530c,0x02);
    OV2685_MIPI_write_cmos_sensor(0x530d,0x00);
    OV2685_MIPI_write_cmos_sensor(0x530e,0x0c);
    OV2685_MIPI_write_cmos_sensor(0x530f,0x14);
    OV2685_MIPI_write_cmos_sensor(0x5310,0x3a);
    OV2685_MIPI_write_cmos_sensor(0x5311,0x20);
    OV2685_MIPI_write_cmos_sensor(0x5312,0x80);
    OV2685_MIPI_write_cmos_sensor(0x5313,0x50);

    /*


    OV2685_MIPI_write_cmos_sensor(0x5380,0x01);
    OV2685_MIPI_write_cmos_sensor(0x5381,0x92);
    OV2685_MIPI_write_cmos_sensor(0x5382,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5383,0x20);
    OV2685_MIPI_write_cmos_sensor(0x5384,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5385,0x8d);
    OV2685_MIPI_write_cmos_sensor(0x5386,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5387,0x87);
    OV2685_MIPI_write_cmos_sensor(0x5388,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5389,0x43);
    OV2685_MIPI_write_cmos_sensor(0x538a,0x01);
    OV2685_MIPI_write_cmos_sensor(0x538b,0xcb);
    OV2685_MIPI_write_cmos_sensor(0x538c,0x00);
    
    */
    //Saturation
    OV2685_MIPI_write_cmos_sensor(0x5380,0x01);
    OV2685_MIPI_write_cmos_sensor(0x5381,0x52);
    OV2685_MIPI_write_cmos_sensor(0x5382,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5383,0x4a);
    OV2685_MIPI_write_cmos_sensor(0x5384,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5385,0xb6);
    OV2685_MIPI_write_cmos_sensor(0x5386,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5387,0x8d);
    OV2685_MIPI_write_cmos_sensor(0x5388,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5389,0x3a);
    OV2685_MIPI_write_cmos_sensor(0x538a,0x00);
    OV2685_MIPI_write_cmos_sensor(0x538b,0xa6);
    OV2685_MIPI_write_cmos_sensor(0x538c,0x00);

    //gamma
    /*
    OV2685_MIPI_write_cmos_sensor(0x5400,0x08);
    OV2685_MIPI_write_cmos_sensor(0x5401,0x16);
    OV2685_MIPI_write_cmos_sensor(0x5402,0x28);
    OV2685_MIPI_write_cmos_sensor(0x5403,0x4a);
    OV2685_MIPI_write_cmos_sensor(0x5404,0x58);
    OV2685_MIPI_write_cmos_sensor(0x5405,0x64);
    OV2685_MIPI_write_cmos_sensor(0x5406,0x6d);
    OV2685_MIPI_write_cmos_sensor(0x5407,0x78);
    OV2685_MIPI_write_cmos_sensor(0x5408,0x82);
    OV2685_MIPI_write_cmos_sensor(0x5409,0x8c);
    OV2685_MIPI_write_cmos_sensor(0x540a,0x9c);
    OV2685_MIPI_write_cmos_sensor(0x540b,0xa9);
    OV2685_MIPI_write_cmos_sensor(0x540c,0xc0);
    OV2685_MIPI_write_cmos_sensor(0x540d,0xd4);
    OV2685_MIPI_write_cmos_sensor(0x540e,0xe5);
    OV2685_MIPI_write_cmos_sensor(0x540f,0xa0);
    OV2685_MIPI_write_cmos_sensor(0x5410,0x5e);
    OV2685_MIPI_write_cmos_sensor(0x5411,0x06);
     */
    OV2685_MIPI_write_cmos_sensor(0x5400,0x0d); //08
    OV2685_MIPI_write_cmos_sensor(0x5401,0x1a); ///1a
    OV2685_MIPI_write_cmos_sensor(0x5402,0x32); //32
    OV2685_MIPI_write_cmos_sensor(0x5403,0x59); //59
    OV2685_MIPI_write_cmos_sensor(0x5404,0x66); ////68
    OV2685_MIPI_write_cmos_sensor(0x5405,0x70); ////76
    OV2685_MIPI_write_cmos_sensor(0x5406,0x7f); //82
    OV2685_MIPI_write_cmos_sensor(0x5407,0x8a); //8c
    OV2685_MIPI_write_cmos_sensor(0x5408,0x94);
    OV2685_MIPI_write_cmos_sensor(0x5409,0x9c);
    OV2685_MIPI_write_cmos_sensor(0x540a,0xa9);
    OV2685_MIPI_write_cmos_sensor(0x540b,0xb6);
    OV2685_MIPI_write_cmos_sensor(0x540c,0xcc);
    OV2685_MIPI_write_cmos_sensor(0x540d,0xdd);
    OV2685_MIPI_write_cmos_sensor(0x540e,0xeb);
    OV2685_MIPI_write_cmos_sensor(0x540f,0xa0); //a0
    OV2685_MIPI_write_cmos_sensor(0x5410,0x6e); //6e
    OV2685_MIPI_write_cmos_sensor(0x5411,0x06); //06
    
    
    OV2685_MIPI_write_cmos_sensor(0x5480,0x19);
    OV2685_MIPI_write_cmos_sensor(0x5481,0x0a);
    OV2685_MIPI_write_cmos_sensor(0x5482,0x0a);
    OV2685_MIPI_write_cmos_sensor(0x5483,0x12);
    OV2685_MIPI_write_cmos_sensor(0x5484,0x04);
    OV2685_MIPI_write_cmos_sensor(0x5485,0x08);
    OV2685_MIPI_write_cmos_sensor(0x5486,0x10);
    OV2685_MIPI_write_cmos_sensor(0x5487,0x18);
    OV2685_MIPI_write_cmos_sensor(0x5488,0x20);
    OV2685_MIPI_write_cmos_sensor(0x5489,0x1e);
    
    OV2685_MIPI_write_cmos_sensor(0x5500,0x02);
    OV2685_MIPI_write_cmos_sensor(0x5501,0x03);
    OV2685_MIPI_write_cmos_sensor(0x5502,0x05);
    OV2685_MIPI_write_cmos_sensor(0x5503,0x06);
    OV2685_MIPI_write_cmos_sensor(0x5504,0x07);
    OV2685_MIPI_write_cmos_sensor(0x5505,0x08);
    OV2685_MIPI_write_cmos_sensor(0x5506,0x00);     
    
    OV2685_MIPI_write_cmos_sensor(0x5600,0x06);

    OV2685_MIPI_write_cmos_sensor(0x560b,0x01);

    OV2685_MIPI_write_cmos_sensor(0x5603, 0x30); //50 
    OV2685_MIPI_write_cmos_sensor(0x5604, 0x40); //38 
    OV2685_MIPI_write_cmos_sensor(0x5609,0x20);
    OV2685_MIPI_write_cmos_sensor(0x560a,0x60);
    
    OV2685_MIPI_write_cmos_sensor(0x5800,0x03);//02);
    OV2685_MIPI_write_cmos_sensor(0x5801,0x2F);//F5);
    OV2685_MIPI_write_cmos_sensor(0x5802,0x02);//02);
    OV2685_MIPI_write_cmos_sensor(0x5803,0x4F);//4E);
    OV2685_MIPI_write_cmos_sensor(0x5804,0x22);//3C);
    OV2685_MIPI_write_cmos_sensor(0x5805,0x05);//05);
    OV2685_MIPI_write_cmos_sensor(0x5806,0x50);//1A);
    OV2685_MIPI_write_cmos_sensor(0x5807,0x05);//05);
                                            
    OV2685_MIPI_write_cmos_sensor(0x5808,0x03);//03);
    OV2685_MIPI_write_cmos_sensor(0x5809,0x3C);//09);
    OV2685_MIPI_write_cmos_sensor(0x580A,0x02);//02);
    OV2685_MIPI_write_cmos_sensor(0x580B,0x4B);//4A);
    OV2685_MIPI_write_cmos_sensor(0x580C,0x21);//34); //37
    OV2685_MIPI_write_cmos_sensor(0x580D,0x05);//05);
    OV2685_MIPI_write_cmos_sensor(0x580E,0x32);//8C);
    OV2685_MIPI_write_cmos_sensor(0x580F,0x05);//05);
                                            
    OV2685_MIPI_write_cmos_sensor(0x5810,0x03);//02);
    OV2685_MIPI_write_cmos_sensor(0x5811,0x23);//F1);
    OV2685_MIPI_write_cmos_sensor(0x5812,0x02);//02);
    OV2685_MIPI_write_cmos_sensor(0x5813,0x4A);//4B);
    OV2685_MIPI_write_cmos_sensor(0x5814,0x1A);//2c); //2c
    OV2685_MIPI_write_cmos_sensor(0x5815,0x05);//05);
    OV2685_MIPI_write_cmos_sensor(0x5816,0x3F);//1E);
    OV2685_MIPI_write_cmos_sensor(0x5817,0x05);//05);
    
    OV2685_MIPI_write_cmos_sensor(0x5818,0x0d);
    OV2685_MIPI_write_cmos_sensor(0x5819,0x40);
    OV2685_MIPI_write_cmos_sensor(0x581a,0x04);
    OV2685_MIPI_write_cmos_sensor(0x581b,0x0c);
    
    OV2685_MIPI_write_cmos_sensor(0x3a03,0x48); //4c
    OV2685_MIPI_write_cmos_sensor(0x3a04,0x38); //3c
    
    OV2685_MIPI_write_cmos_sensor(0x3a12,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3a13,0x8e);
   
    SENSORDB("OV2685_MIPI_global_init end! \n");
}

kal_uint16 OV2685_MIPI_read_OV2685_MIPI_gain(void)
{
    kal_uint8  temp_reg;
    kal_uint16 sensor_gain;

    temp_reg=OV2685_MIPI_read_cmos_sensor(0x350b);  


    sensor_gain=(16+(temp_reg&0x0F));
    if(temp_reg&0x10)
        sensor_gain<<=1;
    if(temp_reg&0x20)
        sensor_gain<<=1;

    if(temp_reg&0x40)
        sensor_gain<<=1;

    if(temp_reg&0x80)
        sensor_gain<<=1;

    return sensor_gain;
}  /* OV2685_MIPI_read_OV2685_MIPI_gain */
kal_uint16 OV2685_MIPI_read_shutter(void)
{
    kal_uint16 temp_reg1, temp_reg2, temp_reg3;
    kal_uint16 temp_reg;

    temp_reg1 = OV2685_MIPI_read_cmos_sensor(0x3500);   
    temp_reg2 = OV2685_MIPI_read_cmos_sensor(0x3501);  
    temp_reg3 = OV2685_MIPI_read_cmos_sensor(0x3502);  
    
    temp_reg = (temp_reg1<<12)|(temp_reg2<<4)|(temp_reg3>>4);    

    spin_lock(&ov2685_drv_lock);
    OV2685_MIPI_PV_Shutter = temp_reg ;   
    spin_unlock(&ov2685_drv_lock);
    
    return temp_reg;
}    /* OV2685_MIPI_read_shutter */

void OV2685_MIPI_write_OV2685_MIPI_gain(kal_uint16 gain)
{    
    kal_uint16 temp_reg;

    RETAILMSG(1, (TEXT("OV2685 write gain: %d\r\n"), gain));

    if(gain > 248)  return ;//ASSERT(0);

    temp_reg = 0;
    if (gain > 31)
    {
        temp_reg |= 0x10;
        gain = gain >> 1;
    }
    if (gain > 31)
    {
        temp_reg |= 0x20;
        gain = gain >> 1;
    }

    if (gain > 31)
    {
        temp_reg |= 0x40;
        gain = gain >> 1;
    }
    if (gain > 31)
    {
        temp_reg |= 0x80;
        gain = gain >> 1;
    }

    if (gain > 16)
    {
        temp_reg |= ((gain -16) & 0x0f);
    }   

    OV2685_MIPI_write_cmos_sensor(0x350b,temp_reg);
}  /* OV2685_MIPI_write_OV2685_MIPI_gain */

static void OV2685_MIPI_write_shutter(kal_uint16 shutter)
{
	  kal_uint32 OV2685_MIPI_extra_exposure_lines = 0;
	
	  if (shutter < 1)
	  {
		    shutter = 1;
	  }

        if (shutter > OV2685_MIPI_FULL_EXPOSURE_LIMITATION) 
        {
          OV2685_MIPI_extra_exposure_lines = shutter+4;
	  OV2685_MIPI_write_cmos_sensor(0x380f, OV2685_MIPI_extra_exposure_lines & 0xFF);       
	  OV2685_MIPI_write_cmos_sensor(0x380e, (OV2685_MIPI_extra_exposure_lines & 0xFF00)>>8);  
	  OV2685_MIPI_write_cmos_sensor(0x350d, 0x00);
          OV2685_MIPI_write_cmos_sensor(0x350c, 0x00);
    }

  #if 0
    if (OV2685_MIPI_gPVmode) 
    {
        if (shutter <= OV2685_MIPI_PV_EXPOSURE_LIMITATION) 
        {
            spin_lock(&ov2685_drv_lock);
            OV2685_MIPI_extra_exposure_lines = 0;
	          spin_unlock(&ov2685_drv_lock);
        }
        else 
        {
	          spin_lock(&ov2685_drv_lock);
            OV2685_MIPI_extra_exposure_lines=shutter - OV2685_MIPI_PV_EXPOSURE_LIMITATION;
	          spin_unlock(&ov2685_drv_lock);
        }

        if (shutter > OV2685_MIPI_PV_EXPOSURE_LIMITATION) 
        {
            shutter = OV2685_MIPI_PV_EXPOSURE_LIMITATION;
        }
    }
    else 
    {
        if (shutter <= OV2685_MIPI_FULL_EXPOSURE_LIMITATION) 
        {
	          spin_lock(&ov2685_drv_lock);
            OV2685_MIPI_extra_exposure_lines = 0;
	          spin_unlock(&ov2685_drv_lock);
    }
        else 
        {
	          spin_lock(&ov2685_drv_lock);
            OV2685_MIPI_extra_exposure_lines = shutter - OV2685_MIPI_FULL_EXPOSURE_LIMITATION;
	          spin_unlock(&ov2685_drv_lock);
        }

        if (shutter > OV2685_MIPI_FULL_EXPOSURE_LIMITATION) 
        {
            shutter = OV2685_MIPI_FULL_EXPOSURE_LIMITATION;
        }
    }
   #endif
	  //AEC PK EXPOSURE
	  shutter*=16;
	  OV2685_MIPI_write_cmos_sensor(0x3502, (shutter&0x00FF));       
	  OV2685_MIPI_write_cmos_sensor(0x3501, ((shutter&0x0FF00)>>8));  
	  OV2685_MIPI_write_cmos_sensor(0x3500, ((shutter&0xFF0000)>>16));
	  
	  //OV2685_MIPI_set_dummy(0, OV2685_MIPI_extra_exposure_lines);
	  
	  //OV2685_MIPI_write_cmos_sensor(0x350D, OV2685_MIPI_extra_exposure_lines&0xFF);        
	  //OV2685_MIPI_write_cmos_sensor(0x350C, (OV2685_MIPI_extra_exposure_lines&0xFF00)>> 8);     

}    /* OV2685_MIPI_write_shutter */
/*
void OV2685_MIPI_Computer_AEC(kal_uint16 preview_clk_in_M, kal_uint16 capture_clk_in_M)
{
    kal_uint16 PV_Line_Width;
    kal_uint16 Capture_Line_Width;
    kal_uint16 Capture_Maximum_Shutter;
    kal_uint16 Capture_Exposure;
    kal_uint16 Capture_Gain16;
    kal_uint16 Capture_Banding_Filter;
    kal_uint32 Gain_Exposure=0;

    PV_Line_Width = OV2685_MIPI_PV_PERIOD_PIXEL_NUMS + OV2685_MIPI_PV_Dummy_Pixels;   

    Capture_Line_Width = OV2685_MIPI_FULL_PERIOD_PIXEL_NUMS + OV2685_MIPI_Capture_Dummy_Pixels;
    Capture_Maximum_Shutter = OV2685_MIPI_FULL_EXPOSURE_LIMITATION + OV2685_MIPI_Capture_Dummy_Lines;
    Gain_Exposure = 1;
    ///////////////////////
    Gain_Exposure *=(OV2685_MIPI_PV_Shutter+OV2685_MIPI_PV_Extra_Lines);
    Gain_Exposure *=PV_Line_Width;  //970
    //   Gain_Exposure /=g_Preview_PCLK_Frequency;
    Gain_Exposure /=Capture_Line_Width;//1940
    Gain_Exposure = Gain_Exposure*capture_clk_in_M/preview_clk_in_M;// for clock   

    //OV2685_MIPI_Capture_Gain16 = Capture_Gain16;
    OV2685_MIPI_Capture_Extra_Lines = (Gain_Exposure > Capture_Maximum_Shutter)?
            (Gain_Exposure - Capture_Maximum_Shutter):0;     
    
    OV2685_MIPI_Capture_Shutter = Gain_Exposure - OV2685_MIPI_Capture_Extra_Lines;
}
*/

void OV2685_MIPI_Computer_AECAGC(kal_uint16 preview_clk_in_M, kal_uint16 capture_clk_in_M)
{
    kal_uint16 PV_Line_Width;
    kal_uint16 Capture_Line_Width;
    kal_uint16 Capture_Maximum_Shutter;
    kal_uint16 Capture_Exposure;
    kal_uint16 Capture_Gain16;
    kal_uint32 Capture_Banding_Filter;
    kal_uint32 Gain_Exposure=0;

    PV_Line_Width = OV2685_MIPI_PV_PERIOD_PIXEL_NUMS + OV2685_MIPI_PV_Dummy_Pixels;   

    Capture_Line_Width = OV2685_MIPI_FULL_PERIOD_PIXEL_NUMS + OV2685_MIPI_Capture_Dummy_Pixels;
    Capture_Maximum_Shutter = OV2685_MIPI_FULL_EXPOSURE_LIMITATION + OV2685_MIPI_Capture_Dummy_Lines;

    if (OV2685_MIPI_Banding_setting == AE_FLICKER_MODE_50HZ)
#if WINMO_USE        
        Capture_Banding_Filter = (kal_uint32)(capture_clk_in_M*1000000/100/(2*Capture_Line_Width)+0.5);
#else 
        Capture_Banding_Filter = (kal_uint32)(capture_clk_in_M*100000/100/(2*Capture_Line_Width));
#endif 
    else
#if WINMO_USE
        Capture_Banding_Filter = (kal_uint16)(capture_clk_in_M*1000000/120/(2*Capture_Line_Width)+0.5);
#else 
        Capture_Banding_Filter = (kal_uint32)(capture_clk_in_M*100000/120/(2*Capture_Line_Width) );
#endif 

    /*   Gain_Exposure = OV2685_MIPI_PV_Gain16*(OV2685_MIPI_PV_Shutter+OV2685_MIPI_PV_Extra_Lines)*PV_Line_Width/g_Preview_PCLK_Frequency/Capture_Line_Width*g_Capture_PCLK_Frequency
    ;*/
spin_lock(&ov2685_drv_lock);
    //OV2685_MIPI_PV_Gain16 = OV2685_MIPI_read_OV2685_MIPI_gain();
spin_unlock(&ov2685_drv_lock);
    Gain_Exposure = 1 * OV2685_MIPI_PV_Gain16;  //For OV2685
    ///////////////////////
    Gain_Exposure *=(OV2685_MIPI_PV_Shutter+OV2685_MIPI_PV_Extra_Lines);
    Gain_Exposure *=PV_Line_Width;  //970
    //   Gain_Exposure /=g_Preview_PCLK_Frequency;
    Gain_Exposure /=Capture_Line_Width;//1940
    Gain_Exposure = Gain_Exposure*capture_clk_in_M/preview_clk_in_M;// for clock   

    //redistribute gain and exposure
    if (Gain_Exposure < (kal_uint32)(Capture_Banding_Filter * 16))     // Exposure < 1/100/120
    {
       if(Gain_Exposure<16){//exposure line smaller than 2 lines and gain smaller than 0x08 
            Gain_Exposure = Gain_Exposure*4;     
            Capture_Exposure = 1;
            Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2/4;
        }
        else
        {
            Capture_Exposure = Gain_Exposure /16;
            Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2;
        }
    }
    else 
    {
        if (Gain_Exposure >(kal_uint32)( Capture_Maximum_Shutter * 16)) // Exposure > Capture_Maximum_Shutter
        {
           
            Capture_Exposure = Capture_Maximum_Shutter/Capture_Banding_Filter*Capture_Banding_Filter;
            Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2;
            if (Capture_Gain16 > OV2685_MIPI_Capture_Max_Gain16) 
            {
                // gain reach maximum, insert extra line
                Capture_Exposure = (kal_uint16)(Gain_Exposure*11 /10 /OV2685_MIPI_Capture_Max_Gain16);
                
                // Exposure = n/100/120
                Capture_Exposure = Capture_Exposure/Capture_Banding_Filter * Capture_Banding_Filter;
                Capture_Gain16 = ((Gain_Exposure *4)/ Capture_Exposure+3)/4;
            }
        }
        else  // 1/100 < Exposure < Capture_Maximum_Shutter, Exposure = n/100/120
        {
            Capture_Exposure = Gain_Exposure/16/Capture_Banding_Filter;
            Capture_Exposure = Capture_Exposure * Capture_Banding_Filter;
            Capture_Gain16 = (Gain_Exposure*2 +1) / Capture_Exposure/2;
        }
    }

    	spin_lock(&ov2685_drv_lock);
    OV2685_MIPI_Capture_Gain16 = Capture_Gain16;
    OV2685_MIPI_Capture_Extra_Lines = (Capture_Exposure > Capture_Maximum_Shutter)?
            (Capture_Exposure - Capture_Maximum_Shutter/Capture_Banding_Filter*Capture_Banding_Filter):0;     
    

    OV2685_MIPI_Capture_Shutter = Capture_Exposure - OV2685_MIPI_Capture_Extra_Lines;
	  spin_unlock(&ov2685_drv_lock);
}

/* void OV2685_MIPI_set_isp_driving_current(kal_uint8 current)
{
	
}        */
static void OV2685_MIPI_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 temp_AE_reg = 0;

    if (AE_enable == KAL_TRUE)
    {
        // turn on AEC/AGC
        temp_AE_reg = OV2685_MIPI_read_cmos_sensor(0x3503);
        OV2685_MIPI_write_cmos_sensor(0x3503, temp_AE_reg&0xfc);
    }
    else
    {
        // turn off AEC/AGC
        temp_AE_reg = OV2685_MIPI_read_cmos_sensor(0x3503);
        OV2685_MIPI_write_cmos_sensor(0x3503, temp_AE_reg|0x03);
    }
}
static void OV2685_MIPI_set_AWB_mode(kal_bool AWB_enable)
{
    kal_uint8 temp_AWB_reg = 0;

    //return ;

    if (AWB_enable == KAL_TRUE)
    {
        //enable Auto WB
        temp_AWB_reg = OV2685_MIPI_read_cmos_sensor(0x5180);
        OV2685_MIPI_write_cmos_sensor(0x5180, temp_AWB_reg&0xfd);        
    }
    else
    {
        //turn off AWB
        temp_AWB_reg = OV2685_MIPI_read_cmos_sensor(0x5180);
        OV2685_MIPI_write_cmos_sensor(0x5180, temp_AWB_reg|0x02);        
    }
}


/*************************************************************************
* FUNCTION
*	OV2685_MIPI_night_mode
*
* DESCRIPTION
*	This function night mode of OV2685.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL OV2685_MIPI_set_param_banding(UINT16 para); 
void OV2685_MIPI_night_mode(kal_bool enable)
{
	  kal_uint8 night = OV2685_MIPI_read_cmos_sensor(0x3a00); 
	
		if (OV2685_MIPI_sensor_cap_state == KAL_FALSE) 
		{
			if (enable) 
			{
			  if (OV2685_MIPI_VEDIO_encode_mode == KAL_TRUE) 
				{
				    OV2685_MIPI_write_cmos_sensor(0x3082, 0x2c);
            OV2685_MIPI_write_cmos_sensor(0x3083, 0x03);
            OV2685_MIPI_write_cmos_sensor(0x3084, 0x0f);
            OV2685_MIPI_write_cmos_sensor(0x3085, 0x03);
            OV2685_MIPI_write_cmos_sensor(0x3086, 0x00);
            OV2685_MIPI_write_cmos_sensor(0x3087, 0x00);
           
            OV2685_MIPI_write_cmos_sensor(0x380c, 0x0d);
            OV2685_MIPI_write_cmos_sensor(0x380d, 0x58);
            OV2685_MIPI_write_cmos_sensor(0x380e, 0x02);
            OV2685_MIPI_write_cmos_sensor(0x380f, 0x84);                                        
                           
            OV2685_MIPI_write_cmos_sensor(0x3a06, 0x00);
            OV2685_MIPI_write_cmos_sensor(0x3a07, 0x60);
            OV2685_MIPI_write_cmos_sensor(0x3a08, 0x00);
            OV2685_MIPI_write_cmos_sensor(0x3a09, 0x50);
          
            OV2685_MIPI_write_cmos_sensor(0x3a0e, 0x02);
            OV2685_MIPI_write_cmos_sensor(0x3a0f, 0x40);
            OV2685_MIPI_write_cmos_sensor(0x3a10, 0x02);
            OV2685_MIPI_write_cmos_sensor(0x3a11, 0x80);
            
            OV2685_MIPI_write_cmos_sensor(0x3a00, night&0xfd);                                     
            OV2685_MIPI_write_cmos_sensor(0x3a0a, 0x02);
				    OV2685_MIPI_write_cmos_sensor(0x3a0b, 0x84);
			      OV2685_MIPI_write_cmos_sensor(0x3a0c, 0x02);
				    OV2685_MIPI_write_cmos_sensor(0x3a0d, 0x84);           
        }
				else 
				{
					  /* Camera mode only */	
            OV2685_MIPI_write_cmos_sensor(0x3a00, night|0x02);                                     
            OV2685_MIPI_write_cmos_sensor(0x3a0a, 0x0f);
				    OV2685_MIPI_write_cmos_sensor(0x3a0b, 0x18);
			      OV2685_MIPI_write_cmos_sensor(0x3a0c, 0x0f);
				    OV2685_MIPI_write_cmos_sensor(0x3a0d, 0x18);
				}	   
			}
			else 
			{
				/* when enter normal mode (disable night mode) without light, the AE vibrate */
				if (OV2685_MIPI_VEDIO_encode_mode == KAL_TRUE) 
				{
					  /* MJPEG or MPEG4 Apps */
				    OV2685_MIPI_write_cmos_sensor(0x3082, 0x2c);
            OV2685_MIPI_write_cmos_sensor(0x3083, 0x03);
            OV2685_MIPI_write_cmos_sensor(0x3084, 0x0f);
            OV2685_MIPI_write_cmos_sensor(0x3085, 0x03);
            OV2685_MIPI_write_cmos_sensor(0x3086, 0x00);
            OV2685_MIPI_write_cmos_sensor(0x3087, 0x00);
           
            OV2685_MIPI_write_cmos_sensor(0x380c, 0x06);
            OV2685_MIPI_write_cmos_sensor(0x380d, 0xac);
            OV2685_MIPI_write_cmos_sensor(0x380e, 0x02);
            OV2685_MIPI_write_cmos_sensor(0x380f, 0x84);                                        
                           
            OV2685_MIPI_write_cmos_sensor(0x3a06, 0x00);
            OV2685_MIPI_write_cmos_sensor(0x3a07, 0xc1);
            OV2685_MIPI_write_cmos_sensor(0x3a08, 0x00);
            OV2685_MIPI_write_cmos_sensor(0x3a09, 0xa1);
          
            OV2685_MIPI_write_cmos_sensor(0x3a0e, 0x02);
            OV2685_MIPI_write_cmos_sensor(0x3a0f, 0x43);
            OV2685_MIPI_write_cmos_sensor(0x3a10, 0x02);
            OV2685_MIPI_write_cmos_sensor(0x3a11, 0x84);
            
            OV2685_MIPI_write_cmos_sensor(0x3a00, night&0xfd);                                     
            OV2685_MIPI_write_cmos_sensor(0x3a0a, 0x02);
				    OV2685_MIPI_write_cmos_sensor(0x3a0b, 0x84);
			      OV2685_MIPI_write_cmos_sensor(0x3a0c, 0x02);
				    OV2685_MIPI_write_cmos_sensor(0x3a0d, 0x84);       
				 }
				else 
				{
					  /* Camera mode only */					                     
         
					  OV2685_MIPI_write_cmos_sensor(0x3a00, night|0x02);                                     
            OV2685_MIPI_write_cmos_sensor(0x3a0a, 0x07);
				    OV2685_MIPI_write_cmos_sensor(0x3a0b, 0x8c);
			      OV2685_MIPI_write_cmos_sensor(0x3a0c, 0x07);
				    OV2685_MIPI_write_cmos_sensor(0x3a0d, 0x8c);
				}					
		  }
	 }       
        OV2685_MIPI_set_param_banding(OV2685_MIPI_Banding_setting); 
}	/* OV2685_MIPI_night_mode */

/*************************************************************************
* FUNCTION
*	OV2685_MIPI_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 OV2685_MIPI_GetSensorID(kal_uint32 *sensorID)
{
	  volatile signed char i;
		kal_uint32 sensor_id=0;
		kal_uint8 temp_sccb_addr = 0;
		//s_move to here from CISModulePowerOn()

		OV2685_MIPI_write_cmos_sensor(0x0103,0x01);// Reset sensor
		mDELAY(10);		
		
			//	Read sensor ID to adjust I2C is OK?
			for(i=0;i<3;i++)
			{
				sensor_id = (OV2685_MIPI_read_cmos_sensor(0x300A) << 8) | OV2685_MIPI_read_cmos_sensor(0x300B);
				if(sensor_id == 0x2685)
				{
                    break;
                    SENSORDB("++++OV2685_MIPI_GetSensorID,read id = 0x%x\n", sensor_id);
				}
			}
            if(0x2685 == sensor_id)
            {
			    *sensorID = sensor_id;
            }
            else
            {
                *sensorID = 0xFFFFFFFF;
                SENSORDB("[OV2685_MIPI_GetSensorID] sensor_id :0x%02x is err !!!\n",sensor_id);
                return ERROR_SENSOR_CONNECT_FAIL;
            }

		RETAILMSG(1, (TEXT("OV2685 Sensor Read ID OK \r\n")));
		
    return ERROR_NONE;    
}   
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	OV2685Open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV2685Open(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

#if WINMO_USE	
	OV2685hDrvI2C=DRV_I2COpen(0);
	DRV_I2CSetParam(OV2685hDrvI2C, I2C_VAL_FS_SAMPLE_DIV, 3);
	DRV_I2CSetParam(OV2685hDrvI2C, I2C_VAL_FS_STEP_DIV, 8);
	DRV_I2CSetParam(OV2685hDrvI2C, I2C_VAL_DELAY_LEN, 2);
	DRV_I2CSetParam(OV2685hDrvI2C, I2C_VAL_CLK_EXT, I2C_CLKEXT_DISABLE);
	OV2685I2CConfig.trans_mode=I2C_TRANSFER_FAST_MODE;
	OV2685I2CConfig.slaveAddr=OV2685_MIPI_sensor_write_I2C_address>>1;
	OV2685I2CConfig.RS_ST=I2C_TRANSFER_STOP;	/* for fast mode */

	CISModulePowerOn(KAL_TRUE);      // Power On CIS Power
	Sleep(10);							// To wait for Stable Power
#endif 
	zoom_factor = 0; 
	OV2685_MIPI_write_cmos_sensor(0x0103,0x01);// Reset sensor
  Sleep(10);

	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
        sensor_id = (OV2685_MIPI_read_cmos_sensor(0x300A) << 8) | OV2685_MIPI_read_cmos_sensor(0x300B);
        if(sensor_id == 0x2685)
        {
            SENSORDB("++++OV2685Open,read id = 0x%x\n", sensor_id);
            break;
        }
    }
    if(0x2685 != sensor_id)
    {
        SENSORDB("[OV2685Open] sensor_id :0x%02x is err !!!!\n",sensor_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }
	
RETAILMSG(1, (TEXT("OV2685 Sensor Read ID OK \r\n")));
	
//init MIPI
    OV2685_MIPI_globle_init();

    return ERROR_NONE;
}	/* OV2685Open() */

/*************************************************************************
* FUNCTION
*	OV2685Close
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV2685Close(void)
{
//	CISModulePowerOn(FALSE);

#if WINMO_USE
	#ifndef SOFTWARE_I2C_INTERFACE	/* software I2c MODE */
	DRV_I2CClose(OV2685hDrvI2C);
	#endif
#endif 	
	return ERROR_NONE;
}	/* OV2685Close() */

/*************************************************************************
* FUNCTION
*	OV2685Preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV2685Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint8 iTemp, temp_AE_reg, temp_AWB_reg;
  kal_uint16 iDummyPixels = 0, iDummyLines = 0, iStartX = 0, iStartY = 0;
   
	spin_lock(&ov2685_drv_lock);
	OV2685_MIPI_sensor_cap_state = KAL_FALSE;
	spin_unlock(&ov2685_drv_lock);  

#ifdef __SLT_DRV_OV3660_BURST_SHOTS__	//wujinyou, 2011.11.21
	preview_init_flag = 1;
#endif

    //OV2685_MIPI_set_AE_mode(KAL_FALSE);   
  
    //OV2685_MIPI_write_cmos_sensor(0x3500, OV2685_MIPI_exposure_line_h); 
    //OV2685_MIPI_write_cmos_sensor(0x3501, OV2685_MIPI_exposure_line_m);  
    //OV2685_MIPI_write_cmos_sensor(0x3502, OV2685_MIPI_exposure_line_l);
   
    //OV2685_MIPI_write_OV2685_MIPI_gain(OV2685_MIPI_PV_Gain16);    

	  spin_lock(&ov2685_drv_lock);
	       OV2685_MIPI_sensor_pclk=390;
	  spin_unlock(&ov2685_drv_lock);
   
    //YUV SVGA (800x600)      
    OV2685_MIPI_write_cmos_sensor(0x3620,0x26);
    OV2685_MIPI_write_cmos_sensor(0x3621,0x37);
    OV2685_MIPI_write_cmos_sensor(0x3622,0x04);
    OV2685_MIPI_write_cmos_sensor(0x370a,0x23);
    OV2685_MIPI_write_cmos_sensor(0x3718,0x88);
    OV2685_MIPI_write_cmos_sensor(0x3721,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3722,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3723,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3738,0x00);
    OV2685_MIPI_write_cmos_sensor(0x4008,0x00);
    OV2685_MIPI_write_cmos_sensor(0x4009,0x03);
    
    OV2685_MIPI_write_cmos_sensor(0x3820,0xc2);//c2
    OV2685_MIPI_write_cmos_sensor(0x3821,0x01);//01
    
    OV2685_MIPI_write_cmos_sensor(0x3808,0x03);
    OV2685_MIPI_write_cmos_sensor(0x3809,0x20);

    OV2685_MIPI_write_cmos_sensor(0x380a,0x02);
    OV2685_MIPI_write_cmos_sensor(0x380b,0x58);
    OV2685_MIPI_write_cmos_sensor(0x3811,0x04);
    OV2685_MIPI_write_cmos_sensor(0x3813,0x04);
    OV2685_MIPI_write_cmos_sensor(0x3814,0x31);
    OV2685_MIPI_write_cmos_sensor(0x3815,0x31);
    
    OV2685_MIPI_write_cmos_sensor(0x3082,0x2c);
    OV2685_MIPI_write_cmos_sensor(0x3083,0x03);
    OV2685_MIPI_write_cmos_sensor(0x3084,0x0f);
    OV2685_MIPI_write_cmos_sensor(0x3085,0x03);
    OV2685_MIPI_write_cmos_sensor(0x3086,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3087,0x00);
    
    OV2685_MIPI_write_cmos_sensor(0x380c,0x06);
    OV2685_MIPI_write_cmos_sensor(0x380d,0xac);
    OV2685_MIPI_write_cmos_sensor(0x380e,0x02);
    OV2685_MIPI_write_cmos_sensor(0x380f,0x84);
    
    OV2685_MIPI_write_cmos_sensor(0x3a06,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3a07,0xc1);
    OV2685_MIPI_write_cmos_sensor(0x3a08,0x00);
    OV2685_MIPI_write_cmos_sensor(0x3a09,0xa1);
    
    OV2685_MIPI_write_cmos_sensor(0x3a0e,0x02);
    OV2685_MIPI_write_cmos_sensor(0x3a0f,0x43);
    OV2685_MIPI_write_cmos_sensor(0x3a10,0x02);
    OV2685_MIPI_write_cmos_sensor(0x3a11,0x84);      

    OV2685_MIPI_write_cmos_sensor(0x5608, 0x00);
    OV2685_MIPI_write_cmos_sensor(0x5607, 0x00); ///10


    OV2685_MIPI_write_cmos_sensor(0x5908,0x00);
    OV2685_MIPI_write_cmos_sensor(0x5909,0x00);
    OV2685_MIPI_write_cmos_sensor(0x590a,0x10);
    OV2685_MIPI_write_cmos_sensor(0x590b,0x01);
    OV2685_MIPI_write_cmos_sensor(0x590c,0x10);
    OV2685_MIPI_write_cmos_sensor(0x590d,0x01);
    OV2685_MIPI_write_cmos_sensor(0x590e,0x00);
    OV2685_MIPI_write_cmos_sensor(0x590f,0x00);


    
    OV2685_MIPI_write_cmos_sensor(0x5281,0x03);
    OV2685_MIPI_write_cmos_sensor(0x5282,0x06);
    OV2685_MIPI_write_cmos_sensor(0x5283,0x0a);
    OV2685_MIPI_write_cmos_sensor(0x5302,0x06);
    OV2685_MIPI_write_cmos_sensor(0x5303,0x0a);
    OV2685_MIPI_write_cmos_sensor(0x5484,0x04);
    OV2685_MIPI_write_cmos_sensor(0x5485,0x08);
    OV2685_MIPI_write_cmos_sensor(0x5486,0x10);
    OV2685_MIPI_write_cmos_sensor(0x5488,0x20);
    OV2685_MIPI_write_cmos_sensor(0x5503,0x06);
    OV2685_MIPI_write_cmos_sensor(0x5504,0x07);     
    
	  spin_lock(&ov2685_drv_lock);
        OV2685_MIPI_gPVmode = KAL_TRUE;
	  spin_unlock(&ov2685_drv_lock);

    if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        RETAILMSG(1,(TEXT("Camera Video preview\r\n")));
	      spin_lock(&ov2685_drv_lock);
             OV2685_MIPI_VEDIO_encode_mode = KAL_TRUE;
	      spin_unlock(&ov2685_drv_lock);

        iDummyPixels = 0;
        iDummyLines = 0;

        /* to fix VSYNC, to fix frame rate */
        iTemp = OV2685_MIPI_read_cmos_sensor(0x3a00);
        OV2685_MIPI_write_cmos_sensor(0x3a00, iTemp&0xfd); //Disable night mode
        OV2685_MIPI_write_cmos_sensor(0x3a0a, 0x02);
        OV2685_MIPI_write_cmos_sensor(0x3a0b, 0x84);
        OV2685_MIPI_write_cmos_sensor(0x3a0c, 0x02);
        OV2685_MIPI_write_cmos_sensor(0x3a0d, 0x84);
        
        if (image_window->ImageTargetWidth <= OV2685_MIPI_VIDEO_QCIF_WIDTH)
        {
		        spin_lock(&ov2685_drv_lock);
            OV2685_MIPI_iOV2685_MIPI_Mode = OV2685_MIPI_MODE_QCIF_VIDEO;
		        spin_unlock(&ov2685_drv_lock);
        }
        else
        {
		        spin_lock(&ov2685_drv_lock);
            OV2685_MIPI_iOV2685_MIPI_Mode = OV2685_MIPI_MODE_QVGA_VIDEO;
		        spin_unlock(&ov2685_drv_lock);
        }
        //image_window->wait_stable_frame = 3;	
    }
    else
    {
        RETAILMSG(1,(TEXT("Camera preview\r\n")));
        //sensor_config_data->frame_rate == 30
        //ISP_PREVIEW_MODE
        //4  <2> if preview of capture PICTURE

        /* preview: 30 fps with 36M PCLK */
	      spin_lock(&ov2685_drv_lock);
             OV2685_MIPI_VEDIO_encode_mode = KAL_FALSE;
	      spin_unlock(&ov2685_drv_lock);

        iDummyPixels = 0; 
        iDummyLines = 0; 

	      spin_lock(&ov2685_drv_lock);
              OV2685_MIPI_iOV2685_MIPI_Mode = OV2685_MIPI_MODE_PREVIEW;
	      spin_unlock(&ov2685_drv_lock);        	
    }
        if(1)//modify
        {
        	iStartX = 13;
          iStartY = 1;
          
          OV2685_MIPI_write_cmos_sensor(0x3820, 0xc2);
          OV2685_MIPI_write_cmos_sensor(0x3821, 0x01);         
        }
        else
        {
          iStartX = 1;
          iStartY = 1;          
          
          OV2685_MIPI_write_cmos_sensor(0x3820, 0xc6);
          OV2685_MIPI_write_cmos_sensor(0x3821, 0x05);     
        }

    //4 <6> set dummy
	  spin_lock(&ov2685_drv_lock);
         OV2685_MIPI_PV_Dummy_Pixels = iDummyPixels;
	  spin_unlock(&ov2685_drv_lock);
   // OV2685_MIPI_set_dummy(iDummyPixels, iDummyLines);
   
    temp_AE_reg = OV2685_MIPI_read_cmos_sensor(0x3503);
    OV2685_MIPI_write_cmos_sensor(0x3503, temp_AE_reg&0xfc);
    
    Sleep(200);  

    //4 <7> set shutter
    image_window->GrabStartX = iStartX;
    image_window->GrabStartY = iStartY;
    image_window->ExposureWindowWidth = OV2685_MIPI_IMAGE_SENSOR_PV_WIDTH - iStartX -2;
    image_window->ExposureWindowHeight = OV2685_MIPI_IMAGE_SENSOR_PV_HEIGHT- iStartY -2;
            
	  spin_lock(&ov2685_drv_lock);
         OV2685_MIPI_DELAY_AFTER_PREVIEW = 1;
	  spin_unlock(&ov2685_drv_lock);

  	memcpy(&OV2685SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	
  	return ERROR_NONE;
}	/* OV2685Preview() */

UINT32 OV2685Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	
 //if(OV2685_MIPI_sensor_cap_state = KAL_FALSE)
 {
    kal_uint32 shutter = 0;
    kal_uint32 temp_reg;
 
	  spin_lock(&ov2685_drv_lock);
          OV2685_MIPI_sensor_cap_state = KAL_TRUE;
	  spin_unlock(&ov2685_drv_lock);  
 
    //OV2685_MIPI_set_AE_mode(KAL_FALSE);  
   //temp_reg = OV2685_MIPI_read_cmos_sensor(0x3a00);
    //OV2685_MIPI_write_cmos_sensor(0x3a00, temp_reg&0xfd); //Disable night mode        
  
	  spin_lock(&ov2685_drv_lock);
    OV2685_MIPI_exposure_line_h = OV2685_MIPI_read_cmos_sensor(0x3500);
    OV2685_MIPI_exposure_line_m = OV2685_MIPI_read_cmos_sensor(0x3501);
    OV2685_MIPI_exposure_line_l = OV2685_MIPI_read_cmos_sensor(0x3502);
	  spin_unlock(&ov2685_drv_lock);
    OV2685_MIPI_PV_Gain16 = OV2685_MIPI_read_OV2685_MIPI_gain();
    shutter = OV2685_MIPI_read_shutter();
    OV2685MIPISensor.currentExposureTime = 52*shutter;  //add jiabaojun
    if ((image_window->ImageTargetWidth<=OV2685_MIPI_IMAGE_SENSOR_PV_WIDTH)&&
        (image_window->ImageTargetHeight<=OV2685_MIPI_IMAGE_SENSOR_PV_HEIGHT))
    {    /* Less than PV Mode */

	      spin_lock(&ov2685_drv_lock);
        OV2685_MIPI_gPVmode=KAL_TRUE;
        OV2685_MIPI_dummy_pixels = 0;
        OV2685_MIPI_dummy_lines = 0;
	      spin_unlock(&ov2685_drv_lock);
        
        OV2685_MIPI_write_shutter(shutter);
        
        image_window->GrabStartX = 1;
        image_window->GrabStartY = 1;
        image_window->ExposureWindowWidth= OV2685_MIPI_IMAGE_SENSOR_PV_WIDTH - 3;
        image_window->ExposureWindowHeight = OV2685_MIPI_IMAGE_SENSOR_PV_HEIGHT - 3;
    }
    else 
    {    
    	  /* 2M FULL Mode */
        image_window->GrabStartX=1;
        image_window->GrabStartY=6;
        image_window->ExposureWindowWidth=OV2685_MIPI_IMAGE_SENSOR_FULL_WIDTH - image_window->GrabStartX - 2;
        image_window->ExposureWindowHeight=OV2685_MIPI_IMAGE_SENSOR_FULL_HEIGHT -image_window->GrabStartY - 2;    	 

        // 1600x1200      
        OV2685_MIPI_write_cmos_sensor(0x3620,0x24);
        OV2685_MIPI_write_cmos_sensor(0x3621,0x34);
        OV2685_MIPI_write_cmos_sensor(0x3622,0x03);
        OV2685_MIPI_write_cmos_sensor(0x370a,0x21);
        OV2685_MIPI_write_cmos_sensor(0x3718,0x80);
        OV2685_MIPI_write_cmos_sensor(0x3721,0x09);
        OV2685_MIPI_write_cmos_sensor(0x3722,0x06);
        OV2685_MIPI_write_cmos_sensor(0x3723,0x59);
        OV2685_MIPI_write_cmos_sensor(0x3738,0x99);
        OV2685_MIPI_write_cmos_sensor(0x4008,0x02);
        OV2685_MIPI_write_cmos_sensor(0x4009,0x09);
        
        OV2685_MIPI_write_cmos_sensor(0x3820,0xc0);//c0
        OV2685_MIPI_write_cmos_sensor(0x3821,0x00);//00
        
        OV2685_MIPI_write_cmos_sensor(0x3808,0x06);
        OV2685_MIPI_write_cmos_sensor(0x3809,0x40);
        OV2685_MIPI_write_cmos_sensor(0x380a,0x04);
        OV2685_MIPI_write_cmos_sensor(0x380b,0xb0);
        OV2685_MIPI_write_cmos_sensor(0x3811,0x08);
        OV2685_MIPI_write_cmos_sensor(0x3813,0x08);
        OV2685_MIPI_write_cmos_sensor(0x3814,0x11);
        OV2685_MIPI_write_cmos_sensor(0x3815,0x11);
        
        OV2685_MIPI_write_cmos_sensor(0x3082,0x2c);
        OV2685_MIPI_write_cmos_sensor(0x3083,0x03);
        OV2685_MIPI_write_cmos_sensor(0x3084,0x0f);
        OV2685_MIPI_write_cmos_sensor(0x3085,0x03);
        OV2685_MIPI_write_cmos_sensor(0x3086,0x00);
        OV2685_MIPI_write_cmos_sensor(0x3087,0x00);
        
        OV2685_MIPI_write_cmos_sensor(0x380c,0x06);
        OV2685_MIPI_write_cmos_sensor(0x380d,0xa4);
        OV2685_MIPI_write_cmos_sensor(0x380e,0x05);
        OV2685_MIPI_write_cmos_sensor(0x380f,0x0e);
        
        OV2685_MIPI_write_cmos_sensor(0x3a06,0x00);
        OV2685_MIPI_write_cmos_sensor(0x3a07,0xc2);
        OV2685_MIPI_write_cmos_sensor(0x3a08,0x00);
        OV2685_MIPI_write_cmos_sensor(0x3a09,0xa1);
        
        OV2685_MIPI_write_cmos_sensor(0x3a0e,0x04);
        OV2685_MIPI_write_cmos_sensor(0x3a0f,0x8c);
        OV2685_MIPI_write_cmos_sensor(0x3a10,0x05);
        OV2685_MIPI_write_cmos_sensor(0x3a11,0x0e);
        
        OV2685_MIPI_write_cmos_sensor(0x5281,0x07);
        OV2685_MIPI_write_cmos_sensor(0x5282,0x07);
        OV2685_MIPI_write_cmos_sensor(0x5283,0x0c);
        OV2685_MIPI_write_cmos_sensor(0x5302,0x07);
        OV2685_MIPI_write_cmos_sensor(0x5303,0x0b);
        OV2685_MIPI_write_cmos_sensor(0x5484,0x06);
        OV2685_MIPI_write_cmos_sensor(0x5485,0x0b);
        OV2685_MIPI_write_cmos_sensor(0x5486,0x11);
        OV2685_MIPI_write_cmos_sensor(0x5488,0x18);
        OV2685_MIPI_write_cmos_sensor(0x5503,0x05);
        OV2685_MIPI_write_cmos_sensor(0x5504,0x06);            
       
	      spin_lock(&ov2685_drv_lock);
          OV2685_MIPI_gPVmode = KAL_FALSE;
	      spin_unlock(&ov2685_drv_lock);
           
        if ((image_window->ImageTargetWidth<=OV2685_MIPI_IMAGE_SENSOR_FULL_WIDTH)&&
        (image_window->ImageTargetHeight<=OV2685_MIPI_IMAGE_SENSOR_FULL_HEIGHT))
        {     
	        if (zoom_factor  <  3) 
	        {  
			        shutter = shutter;          
	        }
	        else 
	        {
						  shutter = shutter;
	        }                  
        }
        else//Interpolate to 3M
        {
  	      if (image_window->ZoomFactor >= 340)
	        {  	        
					    shutter = shutter;       
			    }
	        else if (image_window->ZoomFactor >= 240) 
	        {          
					    shutter = shutter;
	        }
	        else 
	        {  
					    shutter = shutter;
			    }                  
        }
       	spin_lock(&ov2685_drv_lock);
          OV2685_MIPI_Capture_Dummy_Pixels = OV2685_MIPI_dummy_pixels ;
          OV2685_MIPI_Capture_Dummy_Lines = OV2685_MIPI_dummy_lines;
	      spin_unlock(&ov2685_drv_lock);   
        
        //OV2685_MIPI_set_dummy(OV2685_MIPI_dummy_pixels, OV2685_MIPI_dummy_lines);
       
        OV2685_MIPI_write_shutter(shutter);		       
      }
 }
	  spin_lock(&ov2685_drv_lock);
        OV2685_MIPI_DELAY_AFTER_PREVIEW = 2;
	  spin_unlock(&ov2685_drv_lock);

	  memcpy(&OV2685SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	  return ERROR_NONE;
}	/* OV2685Capture() */

UINT32 OV2685GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=OV2685_MIPI_IMAGE_SENSOR_FULL_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X - 8;  //modify by yanxu
	pSensorResolution->SensorFullHeight=OV2685_MIPI_IMAGE_SENSOR_FULL_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y -6;
	pSensorResolution->SensorPreviewWidth=OV2685_MIPI_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X -8;
	pSensorResolution->SensorPreviewHeight=OV2685_MIPI_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y -6;
	pSensorResolution->SensorVideoWidth=OV2685_MIPI_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X -8;
	pSensorResolution->SensorVideoHeight=OV2685_MIPI_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y -6;

	return ERROR_NONE;
}	/* OV2685GetResolution() */

UINT32 OV2685GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=OV2685_MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=OV2685_MIPI_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX=OV2685_MIPI_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=OV2685_MIPI_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	/*??? */
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;    //SENSOR_INTERFACE_TYPE_PARALLEL
/*
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_5M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_5M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=TRUE;

	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_1M_WIDTH;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_1M_HEIGHT;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=TRUE;
	pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=TRUE;
*/
	pSensorInfo->CaptureDelayFrame = 2; 
	pSensorInfo->PreviewDelayFrame = 3; 
	pSensorInfo->VideoDelayFrame = 4; 		
	pSensorInfo->SensorMasterClockSwitch = 0; 
  pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;  	

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			   pSensorInfo->SensorClockFreq=24;
			   pSensorInfo->SensorClockDividCount=	3;
			   pSensorInfo->SensorClockRisingCount= 0;
			   pSensorInfo->SensorClockFallingCount= 2;
			   pSensorInfo->SensorPixelClockCount= 3;
			   pSensorInfo->SensorDataLatchCount= 2;
         pSensorInfo->SensorGrabStartX = 4; 
         pSensorInfo->SensorGrabStartY = 2;        
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		 pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		 pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
		     break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			   pSensorInfo->SensorClockFreq=24;
			   pSensorInfo->SensorClockDividCount=	3;
			   pSensorInfo->SensorClockRisingCount= 0;
			   pSensorInfo->SensorClockFallingCount= 2;
			   pSensorInfo->SensorPixelClockCount= 3;
			   pSensorInfo->SensorDataLatchCount= 2;
         pSensorInfo->SensorGrabStartX = 4; 
         pSensorInfo->SensorGrabStartY = 2;   
		 
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		 pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		 pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
				 break;
		default:
			   pSensorInfo->SensorClockFreq=24;
			   pSensorInfo->SensorClockDividCount=3;
			   pSensorInfo->SensorClockRisingCount=0;
			   pSensorInfo->SensorClockFallingCount=2;
			   pSensorInfo->SensorPixelClockCount=3;
			   pSensorInfo->SensorDataLatchCount=2;
         pSensorInfo->SensorGrabStartX = 4; 
         pSensorInfo->SensorGrabStartY = 2;      
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		    pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		    pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
	       break;
	}
	spin_lock(&ov2685_drv_lock);
	OV2685_MIPI_PixelClockDivider=pSensorInfo->SensorPixelClockCount;
	spin_unlock(&ov2685_drv_lock);
	memcpy(pSensorConfigData, &OV2685SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* OV2685GetInfo() */

UINT32 OV2685Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			   OV2685Preview(pImageWindow, pSensorConfigData);
		     break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			   OV2685Capture(pImageWindow, pSensorConfigData);         
		     break;
		default:
		     break; 
	}
	return TRUE;
}	/* OV2685Control() */

#if WINMO_USE
void OV2685Query(PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo)
{
	MSDK_FEATURE_TYPE_RANGE_STRUCT *pFeatureRange;
	MSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT *pFeatureMultiSelection;
	switch (pSensorFeatureInfo->FeatureId)
	{
		case ISP_FEATURE_DSC_MODE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_SCENE_MODE_MAX;
			pFeatureMultiSelection->DefaultSelection = CAM_AUTO_DSC_MODE;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_AUTO_DSC_MODE)|
				CAMERA_FEATURE_SUPPORT(CAM_NIGHTSCENE_MODE));			
		break;
		case ISP_FEATURE_WHITEBALANCE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_WB;
			pFeatureMultiSelection->DefaultSelection = CAM_WB_AUTO;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_WB_AUTO)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_CLOUD)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_DAYLIGHT)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_INCANDESCENCE)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_TUNGSTEN)|
				CAMERA_FEATURE_SUPPORT(CAM_WB_FLUORESCENT));
		break;
		case ISP_FEATURE_IMAGE_EFFECT:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_EFFECT_ENC;
			pFeatureMultiSelection->DefaultSelection = CAM_EFFECT_ENC_NORMAL;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_NORMAL)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_GRAYSCALE)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_COLORINV)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIAGREEN)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIABLUE)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIA));	
		break;
		case ISP_FEATURE_AE_METERING_MODE:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_BRIGHTNESS:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_RANGE;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureRange = (PMSDK_FEATURE_TYPE_RANGE_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureRange);
			pFeatureRange->MinValue = CAM_EV_NEG_4_3;
			pFeatureRange->MaxValue = CAM_EV_POS_4_3;
			pFeatureRange->StepValue = CAMERA_FEATURE_ID_EV_STEP;
			pFeatureRange->DefaultValue = CAM_EV_ZERO;
		break;
		case ISP_FEATURE_BANDING_FREQ:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_BANDING;
			pFeatureMultiSelection->DefaultSelection = CAM_BANDING_50HZ;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_BANDING_50HZ)|
				CAMERA_FEATURE_SUPPORT(CAM_BANDING_60HZ));
		break;
		case ISP_FEATURE_AF_OPERATION:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_AF_RANGE_CONTROL:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;
		break;
		case ISP_FEATURE_FLASH:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
		case ISP_FEATURE_VIDEO_SCENE_MODE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_VIDEO_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_SCENE_MODE_MAX;
			pFeatureMultiSelection->DefaultSelection = CAM_VIDEO_AUTO_MODE;
			pFeatureMultiSelection->SupportedSelection = 
				(CAMERA_FEATURE_SUPPORT(CAM_VIDEO_AUTO_MODE)|
				CAMERA_FEATURE_SUPPORT(CAM_VIDEO_NIGHT_MODE));
		break;
		case ISP_FEATURE_ISO:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
		default:
			pSensorFeatureInfo->FeatureSupported = MSDK_FEATURE_NOT_SUPPORTED;			
		break;
	}
}
#endif 

BOOL OV2685_MIPI_set_param_wb(UINT16 para)
{
    kal_uint8  temp_reg,temp_AE_reg;

    temp_reg=OV2685_MIPI_read_cmos_sensor(0x5180);

    switch (para)
    {
        case AWB_MODE_OFF:
            //OV2685_MIPI_AWB_ENABLE = KAL_FALSE; 
            //OV2685_MIPI_set_AWB_mode(OV2685_MIPI_AWB_ENABLE);            
            //break;                     
        case AWB_MODE_AUTO:
		        spin_lock(&ov2685_drv_lock);
                 OV2685_MIPI_AWB_ENABLE = KAL_TRUE;  
		        spin_unlock(&ov2685_drv_lock);
            OV2685_MIPI_set_AWB_mode(OV2685_MIPI_AWB_ENABLE);
            OV2685_MIPI_write_cmos_sensor(0x5180, temp_reg&0xfd);              
            break;

        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5180, temp_reg|0x02); 
            OV2685_MIPI_write_cmos_sensor(0x5195, 0x06); 
            OV2685_MIPI_write_cmos_sensor(0x5196, 0x7c);
            OV2685_MIPI_write_cmos_sensor(0x5197, 0x03);  
            OV2685_MIPI_write_cmos_sensor(0x5198, 0xa0); 
            OV2685_MIPI_write_cmos_sensor(0x5199, 0x05);
            OV2685_MIPI_write_cmos_sensor(0x519a, 0xd3); 
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;

        case AWB_MODE_DAYLIGHT: //sunny
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5180, temp_reg|0x02); 
            OV2685_MIPI_write_cmos_sensor(0x5195, 0x06); //79c
            OV2685_MIPI_write_cmos_sensor(0x5196, 0x2c);
            OV2685_MIPI_write_cmos_sensor(0x5197, 0x03);  
            OV2685_MIPI_write_cmos_sensor(0x5198, 0xa0); 
            OV2685_MIPI_write_cmos_sensor(0x5199, 0x05);
            OV2685_MIPI_write_cmos_sensor(0x519a, 0xf3); 
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;

        case AWB_MODE_INCANDESCENT: //office
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5180, temp_reg|0x02); 
            OV2685_MIPI_write_cmos_sensor(0x5195, 0x05); 
            OV2685_MIPI_write_cmos_sensor(0x5196, 0x00);
            OV2685_MIPI_write_cmos_sensor(0x5197, 0x03);  
            OV2685_MIPI_write_cmos_sensor(0x5198, 0xb0); 
            OV2685_MIPI_write_cmos_sensor(0x5199, 0x07);
            OV2685_MIPI_write_cmos_sensor(0x519a, 0xdf); 
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;

        case AWB_MODE_TUNGSTEN: //home
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5180, temp_reg|0x02); 
            OV2685_MIPI_write_cmos_sensor(0x5195, 0x04); 
            OV2685_MIPI_write_cmos_sensor(0x5196, 0x90);
            OV2685_MIPI_write_cmos_sensor(0x5197, 0x04);  
            OV2685_MIPI_write_cmos_sensor(0x5198, 0x00); 
            OV2685_MIPI_write_cmos_sensor(0x5199, 0x09);
            OV2685_MIPI_write_cmos_sensor(0x519a, 0x20); 
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;

        case AWB_MODE_FLUORESCENT:
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5180, temp_reg|0x02); 
            OV2685_MIPI_write_cmos_sensor(0x5195, 0x04); 
            OV2685_MIPI_write_cmos_sensor(0x5196, 0xec);
            OV2685_MIPI_write_cmos_sensor(0x5197, 0x04);  
            OV2685_MIPI_write_cmos_sensor(0x5198, 0x00); 
            OV2685_MIPI_write_cmos_sensor(0x5199, 0x07); //6d3
            OV2685_MIPI_write_cmos_sensor(0x519a, 0xe3); 
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;
#if WINMO_USE
        case AWB_MODE_MANUAL:
            break;
#endif 
        default:
            return FALSE;
    }
    return TRUE;
} /* OV2685_MIPI_set_param_wb */

BOOL OV2685_MIPI_set_param_effect(UINT16 para)
{
    BOOL  ret = TRUE;   
    switch (para)
    {
        case MEFFECT_OFF:
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5600, 0x06); 

            OV2685_MIPI_write_cmos_sensor(0x560b, 0x01); 

            OV2685_MIPI_write_cmos_sensor(0x5603, 0x30); //50
            OV2685_MIPI_write_cmos_sensor(0x5604, 0x40); //38
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);  
            break;

        case MEFFECT_SEPIA:
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5600, 0x1e); 
            OV2685_MIPI_write_cmos_sensor(0x5603, 0x60); //40
            OV2685_MIPI_write_cmos_sensor(0x5604, 0xa0); 
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;

        case MEFFECT_NEGATIVE:
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5600, 0x46); 
            OV2685_MIPI_write_cmos_sensor(0x5603, 0x40); 
            OV2685_MIPI_write_cmos_sensor(0x5604, 0x28); 
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;

        case MEFFECT_SEPIAGREEN:
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5600, 0x1e); 
            OV2685_MIPI_write_cmos_sensor(0x5603, 0x60); 
            OV2685_MIPI_write_cmos_sensor(0x5604, 0x60); 
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;

        case MEFFECT_SEPIABLUE:
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5600, 0x1e); 
            OV2685_MIPI_write_cmos_sensor(0x5603, 0xa0); 
            OV2685_MIPI_write_cmos_sensor(0x5604, 0x40); 
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;
		case MEFFECT_MONO: //B&W
			      OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
			
            OV2685_MIPI_write_cmos_sensor(0x5600, 0x1e); 
            OV2685_MIPI_write_cmos_sensor(0x5603, 0x80); 
            OV2685_MIPI_write_cmos_sensor(0x5604, 0x80); 
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
			break;
#if WINMO_USE
        case CAM_EFFECT_ENC_GRAYINV:
        case CAM_EFFECT_ENC_COPPERCARVING:
        case CAM_EFFECT_ENC_BLUECARVING:
        case CAM_EFFECT_ENC_EMBOSSMENT:
        case CAM_EFFECT_ENC_SKETCH:
        case CAM_EFFECT_ENC_BLACKBOARD:
        case CAM_EFFECT_ENC_WHITEBOARD:
        case CAM_EFFECT_ENC_JEAN:
        case CAM_EFFECT_ENC_OIL:
#endif 
        default:
            ret = FALSE;
    }
    return ret;

} /* OV2685_MIPI_set_param_effect */

BOOL OV2685_MIPI_set_param_banding(UINT16 para)
{
    kal_uint8 banding;
    banding = OV2685_MIPI_read_cmos_sensor(0x3a02);
    
    switch (para)
    {
    case AE_FLICKER_MODE_50HZ:
		                spin_lock(&ov2685_drv_lock);		
		                OV2685_MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;		
		                spin_unlock(&ov2685_drv_lock);
		
		                OV2685_MIPI_write_cmos_sensor(0x3a02,banding|0x80);    /* enable banding and 50 Hz */				
                    break;    
    case AE_FLICKER_MODE_60HZ:
		                spin_lock(&ov2685_drv_lock);
		                OV2685_MIPI_Banding_setting = AE_FLICKER_MODE_60HZ;	
		                spin_unlock(&ov2685_drv_lock);	
	 
                    OV2685_MIPI_write_cmos_sensor(0x3a02,(banding&0x7f)); /* enable banding and 60 Hz */            
                    break;
            default:
                    return FALSE;
    }
    return TRUE;
} /* OV2685_MIPI_set_param_banding */

void OV2685MIPI_set_iso(UINT16 para)
{
    spin_lock(&ov2685_drv_lock);
    //OV2685MIPISensor.isoSpeed = para;
    spin_unlock(&ov2685_drv_lock);   
    switch (para)
    {
        case AE_ISO_100:
             OV2685_MIPI_write_cmos_sensor(0x3a12, 0x00);
             OV2685_MIPI_write_cmos_sensor(0x3a13, 0x60);
             break;
        case AE_ISO_200:
             OV2685_MIPI_write_cmos_sensor(0x3a12, 0x00);
             OV2685_MIPI_write_cmos_sensor(0x3a13, 0x90);
             break;
        case AE_ISO_400:
             OV2685_MIPI_write_cmos_sensor(0x3a12, 0x00);
             OV2685_MIPI_write_cmos_sensor(0x3a13, 0xc0);
             break;
        default:
             break;
    }
    return;
}

BOOL OV2685_MIPI_set_param_exposure(UINT16 para)
{
    kal_uint8  temp_reg;

    temp_reg=OV2685_MIPI_read_cmos_sensor(0x5600);

    switch (para)
    {
        case AE_EV_COMP_n20:
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5600, temp_reg|0x4);
            OV2685_MIPI_write_cmos_sensor(0x5608, 0x08);
            OV2685_MIPI_write_cmos_sensor(0x5607, 0x30);
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;

        case AE_EV_COMP_n10:    
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5600, temp_reg|0x4);
            OV2685_MIPI_write_cmos_sensor(0x5608, 0x08);
            OV2685_MIPI_write_cmos_sensor(0x5607, 0x18);
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;



        case AE_EV_COMP_00:
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5600, temp_reg|0x4);
            OV2685_MIPI_write_cmos_sensor(0x5608, 0x00);
            OV2685_MIPI_write_cmos_sensor(0x5607, 0x00); //10
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;


        case AE_EV_COMP_10:
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5600, temp_reg|0x4);
            OV2685_MIPI_write_cmos_sensor(0x5608, 0x00);
            OV2685_MIPI_write_cmos_sensor(0x5607, 0x18);
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;

        case AE_EV_COMP_20:
        	  OV2685_MIPI_write_cmos_sensor(0x3208, 0x00);
        	
            OV2685_MIPI_write_cmos_sensor(0x5600, temp_reg|0x4);
            OV2685_MIPI_write_cmos_sensor(0x5608, 0x00);
            OV2685_MIPI_write_cmos_sensor(0x5607, 0x30);
            
            OV2685_MIPI_write_cmos_sensor(0x3208, 0x10); 
            OV2685_MIPI_write_cmos_sensor(0x3208, 0xa0);
            break;

        default:
            return FALSE;
    }

    return TRUE;
} /* OV2685_MIPI_set_param_exposure */

UINT32 OV2685MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	switch (iCmd) {
	case FID_SCENE_MODE:	  
	                   if (iPara == SCENE_MODE_OFF)
	                   {
	                      OV2685_MIPI_night_mode(0); 
	                   }
	                   else if (iPara == SCENE_MODE_NIGHTSCENE)
	                   {
                        OV2685_MIPI_night_mode(1); 
	                   }	    
	                   break; 	    
	case FID_AWB_MODE:
	                   OV2685_MIPI_set_param_wb(iPara);
	                   break;
	case FID_COLOR_EFFECT:	  
                     OV2685_MIPI_set_param_effect(iPara);
	                   break;
	case FID_AE_EV:
                     #if WINMO_USE	    
	case ISP_FEATURE_EXPOSURE:
                     #endif 	    
	                   OV2685_MIPI_set_param_exposure(iPara);
	                   break;
	case FID_AE_FLICKER:  	    	    
                     OV2685_MIPI_set_param_banding(iPara);
	                   break;
       case FID_AE_SCENE_MODE: 
                     if (iPara == AE_MODE_OFF)
                     {
		                 spin_lock(&ov2685_drv_lock);
                     // OV2685_MIPI_AE_ENABLE = KAL_FALSE; 
                        OV2685_MIPI_AE_ENABLE = KAL_TRUE;
		                 spin_unlock(&ov2685_drv_lock);
                     }
                     else
                     {
		                 spin_lock(&ov2685_drv_lock);
                        OV2685_MIPI_AE_ENABLE = KAL_TRUE; 
		                 spin_unlock(&ov2685_drv_lock);
	                   }
                     OV2685_MIPI_set_AE_mode(OV2685_MIPI_AE_ENABLE);
                     break; 
	  case FID_ZOOM_FACTOR:
		                 spin_lock(&ov2685_drv_lock);
		                    zoom_factor = iPara; 
		                 spin_unlock(&ov2685_drv_lock);
                     break; 
          case FID_AE_ISO:
                  SENSORDB("FID_AE_ISO setting d%\n", iPara);
                  OV2685MIPI_set_iso(iPara);
                  break;
      default:
	                   break;
	}
	return TRUE;
}   /* OV2685MIPIYUVSensorSetting */

UINT32 OV2685MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 iTemp;
     
    iTemp = OV2685_MIPI_read_cmos_sensor(0x3a00);
    OV2685_MIPI_write_cmos_sensor(0x3a00, iTemp&0xfd); //Disable night mode

    if (u2FrameRate == 30)
    {
    	  OV2685_MIPI_write_cmos_sensor(0x3082, 0x2c);
        OV2685_MIPI_write_cmos_sensor(0x3083, 0x03);
        OV2685_MIPI_write_cmos_sensor(0x3084, 0x0f);
        OV2685_MIPI_write_cmos_sensor(0x3085, 0x03);
        OV2685_MIPI_write_cmos_sensor(0x3086, 0x00);
        OV2685_MIPI_write_cmos_sensor(0x3087, 0x00);
        
        OV2685_MIPI_write_cmos_sensor(0x380c, 0x06);
        OV2685_MIPI_write_cmos_sensor(0x380d, 0xac);
        OV2685_MIPI_write_cmos_sensor(0x380e, 0x02);
        OV2685_MIPI_write_cmos_sensor(0x380f, 0x84);                                        
                       
        OV2685_MIPI_write_cmos_sensor(0x3a06, 0x00);
        OV2685_MIPI_write_cmos_sensor(0x3a07, 0xc1);
        OV2685_MIPI_write_cmos_sensor(0x3a08, 0x00);
        OV2685_MIPI_write_cmos_sensor(0x3a09, 0xa1);
        
        OV2685_MIPI_write_cmos_sensor(0x3a0e, 0x02);
        OV2685_MIPI_write_cmos_sensor(0x3a0f, 0x43);
        OV2685_MIPI_write_cmos_sensor(0x3a10, 0x02);
        OV2685_MIPI_write_cmos_sensor(0x3a11, 0x84);
                                   
        OV2685_MIPI_write_cmos_sensor(0x3a0a, 0x02);
				OV2685_MIPI_write_cmos_sensor(0x3a0b, 0x84);
			  OV2685_MIPI_write_cmos_sensor(0x3a0c, 0x02);
				OV2685_MIPI_write_cmos_sensor(0x3a0d, 0x84); 
    }
    else if (u2FrameRate == 15)       
    {
    	  OV2685_MIPI_write_cmos_sensor(0x3082, 0x2c);
        OV2685_MIPI_write_cmos_sensor(0x3083, 0x03);
        OV2685_MIPI_write_cmos_sensor(0x3084, 0x0f);
        OV2685_MIPI_write_cmos_sensor(0x3085, 0x03);
        OV2685_MIPI_write_cmos_sensor(0x3086, 0x00);
        OV2685_MIPI_write_cmos_sensor(0x3087, 0x00);
        
        OV2685_MIPI_write_cmos_sensor(0x380c, 0x0d);
        OV2685_MIPI_write_cmos_sensor(0x380d, 0x58);
        OV2685_MIPI_write_cmos_sensor(0x380e, 0x02);
        OV2685_MIPI_write_cmos_sensor(0x380f, 0x84);                                        
                       
        OV2685_MIPI_write_cmos_sensor(0x3a06, 0x00);
        OV2685_MIPI_write_cmos_sensor(0x3a07, 0x60);
        OV2685_MIPI_write_cmos_sensor(0x3a08, 0x00);
        OV2685_MIPI_write_cmos_sensor(0x3a09, 0x50);
        
        OV2685_MIPI_write_cmos_sensor(0x3a0e, 0x02);
        OV2685_MIPI_write_cmos_sensor(0x3a0f, 0x40);
        OV2685_MIPI_write_cmos_sensor(0x3a10, 0x02);
        OV2685_MIPI_write_cmos_sensor(0x3a11, 0x80);
                                   
        OV2685_MIPI_write_cmos_sensor(0x3a0a, 0x02);
				OV2685_MIPI_write_cmos_sensor(0x3a0b, 0x84);
			  OV2685_MIPI_write_cmos_sensor(0x3a0c, 0x02);
				OV2685_MIPI_write_cmos_sensor(0x3a0d, 0x84);    
    }
    else 
    {
        SENSORDB("Wrong frame rate setting \n");
    }

	spin_lock(&ov2685_drv_lock);
	OV2685_MIPI_VEDIO_encode_mode = KAL_TRUE;
	spin_unlock(&ov2685_drv_lock);
        
    return TRUE;
}

UINT32 OV2685MIPIYUVSetSoftwarePWDNMode(kal_bool bEnable)
{
    SENSORDB("[OV2685MIPIYUVSetSoftwarePWDNMode] Software Power down enable:%d\n", bEnable);
    if(bEnable) {   // enable software power down mode   
	  //OV2685_MIPI_write_cmos_sensor(0x0100, 0x00);
         OV2685_MIPI_write_cmos_sensor(0x301c, 0xf4);
    } else {
       //OV2685_MIPI_write_cmos_sensor(0x0100, 0x01);  
       OV2685_MIPI_write_cmos_sensor(0x301c, 0xf0);
    }
    return TRUE;
}
/*************************************************************************
* FUNCTION
*OV2685MIPIClose
*
* DESCRIPTION
* This HI257SetMaxFramerateByScenario is to turn off sensor module power.
*
* PARAMETERS
* None
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
  UINT32 OV2685MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	SENSORDB("HI257SetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	/*switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = 134200000;
			lineLength = IMX111MIPI_PV_LINE_LENGTH_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - IMX111MIPI_PV_FRAME_LENGTH_LINES;
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			break;		
		default:
			break;
	}	*/
	return ERROR_NONE;
}
  /*************************************************************************
  * FUNCTION
  * HI257GetDefaultFramerateByScenario
  *
  * DESCRIPTION
  * This function is to turn off sensor module power.
  * RETURNS
  * None
  *
  * GLOBALS AFFECTED
  *
  *************************************************************************/
UINT32 OV2685MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 220;
			break;		//hhl 2-28
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}

  	}

UINT32 OV2685FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

#if WINMO_USE	
	PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo=(PMSDK_FEATURE_INFO_STRUCT) pFeaturePara;
#endif 

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=OV2685_MIPI_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=OV2685_MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=OV2685_MIPI_PV_PERIOD_PIXEL_NUMS+OV2685_MIPI_PV_dummy_pixels;
			*pFeatureReturnPara16=OV2685_MIPI_PV_PERIOD_LINE_NUMS+OV2685_MIPI_PV_dummy_lines;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = OV2685_MIPI_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			OV2685_MIPI_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&ov2685_drv_lock);
			OV2685_MIPI_isp_master_clock=*pFeatureData32;
			spin_unlock(&ov2685_drv_lock);
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			OV2685_MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = OV2685_MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &OV2685SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:

		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
                        *pFeatureReturnPara32++=0;
                        *pFeatureParaLen=4;	    
		    break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			 OV2685_MIPI_GetSensorID(pFeatureData32);
			 break;
		case SENSOR_FEATURE_SET_YUV_CMD:
//		       SENSORDB("OV2685 MIPI YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			OV2685MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
#if WINMO_USE		    		
		case SENSOR_FEATURE_QUERY:
			OV2685Query(pSensorFeatureInfo);
			*pFeatureParaLen = sizeof(MSDK_FEATURE_INFO_STRUCT);
		break;		
		case SENSOR_FEATURE_SET_YUV_CAPTURE_RAW_SUPPORT:
			/* update yuv capture raw support flag by *pFeatureData16 */
		break;		
#endif 			
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		       OV2685MIPIYUVSetVideoMode(*pFeatureData16);
		       break; 
	        case SENSOR_FEATURE_SET_SOFTWARE_PWDN:
	            OV2685MIPIYUVSetSoftwarePWDNMode((BOOL)*pFeatureData16);        	        	
	            break;
		  case SENSOR_FEATURE_GET_EXIF_INFO:
			SENSORDB("[OV2685MIPI]SENSOR_FEATURE_GET_EXIF_INFO\n");
                     OV2685MIPIGetExifInfo(*pFeatureData32);
                     break;   
		default:
			break;			
	}
	return ERROR_NONE;
}	/* OV2685FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncOV2685=
{
	OV2685Open,
	OV2685GetInfo,
	OV2685GetResolution,
	OV2685FeatureControl,
	OV2685Control,
	OV2685Close
};

UINT32 OV2685_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncOV2685;

	return ERROR_NONE;
}	/* SensorInit() */




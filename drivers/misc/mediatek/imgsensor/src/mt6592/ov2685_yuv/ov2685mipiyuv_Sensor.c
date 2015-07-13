/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
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
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/system.h>	 
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
#define OV2685MIPISENSORDB printk
#else
#define OV2685MIPISENSORDB(x,...)
#endif
static DEFINE_SPINLOCK(ov2685mipi_drv_lock);
static bool AF_Power = false;
static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define OV2685MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,OV2685MIPI_WRITE_ID)
//#define msleep(ms)  mdelay(ms)
kal_uint8 OV2685MIPI_sensor_socket = DUAL_CAMERA_NONE_SENSOR;
static int Iszsd = false;
int flash_mode = 2;	//video-mode-flashlight 2; capture-mode-flashlight 1
typedef enum
{
    PRV_W=1280,
    PRV_H=960
}PREVIEW_VIEW_SIZE;
kal_uint16 OV2685MIPIYUV_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV2685MIPI_WRITE_ID);
    return get_byte;
}
#define OV2685MIPI_MAX_AXD_GAIN (32) //max gain = 32
#define OV2685MIPI_MAX_EXPOSURE_TIME (1968) // preview:984,capture 984*2
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
	OV2685MIPI_SENSOR_MODE SensorMode;
} OV2685MIPISensor;
/* Global Valuable */
static kal_uint32 zoom_factor = 0; 
static kal_int8 OV2685MIPI_DELAY_AFTER_PREVIEW = -1;
static kal_uint8 OV2685MIPI_Banding_setting = AE_FLICKER_MODE_50HZ; 
static kal_bool OV2685MIPI_AWB_ENABLE = KAL_TRUE; 
static kal_bool OV2685MIPI_AE_ENABLE = KAL_TRUE; 
MSDK_SENSOR_CONFIG_STRUCT OV2685MIPISensorConfigData;
#define OV2685_TEST_PATTERN_CHECKSUM (0x7ba87eae)
#define OV2685_TAF_TOLERANCE (100)
void OV2685MIPI_set_scene_mode(UINT16 para);
BOOL OV2685MIPI_set_param_wb(UINT16 para);


typedef enum
{
    AE_SECTION_INDEX_BEGIN=0, 
    AE_SECTION_INDEX_1=AE_SECTION_INDEX_BEGIN, 
    AE_SECTION_INDEX_2, 
    AE_SECTION_INDEX_3, 
    AE_SECTION_INDEX_4, 
    AE_SECTION_INDEX_5, 
    AE_SECTION_INDEX_6, 
    AE_SECTION_INDEX_7, 
    AE_SECTION_INDEX_8, 
    AE_SECTION_INDEX_9, 
    AE_SECTION_INDEX_10, 
    AE_SECTION_INDEX_11, 
    AE_SECTION_INDEX_12, 
    AE_SECTION_INDEX_13, 
    AE_SECTION_INDEX_14, 
    AE_SECTION_INDEX_15, 
    AE_SECTION_INDEX_16,  
    AE_SECTION_INDEX_MAX
}AE_SECTION_INDEX;
typedef enum
{
    AE_VERTICAL_BLOCKS=4,
    AE_VERTICAL_BLOCKS_MAX,
    AE_HORIZONTAL_BLOCKS=4,
    AE_HORIZONTAL_BLOCKS_MAX
}AE_VERTICAL_HORIZONTAL_BLOCKS;
static UINT32 line_coordinate[AE_VERTICAL_BLOCKS_MAX] = {0};//line[0]=0      line[1]=160     line[2]=320     line[3]=480     line[4]=640
static UINT32 row_coordinate[AE_HORIZONTAL_BLOCKS_MAX] = {0};//line[0]=0       line[1]=120     line[2]=240     line[3]=360     line[4]=480
static BOOL AE_1_ARRAY[AE_SECTION_INDEX_MAX] = {FALSE};
static BOOL AE_2_ARRAY[AE_HORIZONTAL_BLOCKS][AE_VERTICAL_BLOCKS] = {{FALSE},{FALSE},{FALSE},{FALSE}};//how to ....
//=====================touch AE begin==========================//
void writeAEReg(void)
{	
    UINT8 temp;
    //write 1280X960
     OV2685MIPI_write_cmos_sensor(0x501d,0x10);
     OV2685MIPI_write_cmos_sensor(0x5680,0x00); 
     OV2685MIPI_write_cmos_sensor(0x5681,0x00);
     OV2685MIPI_write_cmos_sensor(0x5682,0x00);  
     OV2685MIPI_write_cmos_sensor(0x5683,0x00);
     OV2685MIPI_write_cmos_sensor(0x5684,0x05); //width=256  
     OV2685MIPI_write_cmos_sensor(0x5685,0x00);
     OV2685MIPI_write_cmos_sensor(0x5686,0x03); //heght=256
     OV2685MIPI_write_cmos_sensor(0x5687,0xc0);
    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_1]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_2]==TRUE)    { temp=temp|0xF0;}
    //write 0x5688
    OV2685MIPI_write_cmos_sensor(0x5688,temp);
    
    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_3]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_4]==TRUE)    { temp=temp|0xF0;}
    //write 0x5689
    OV2685MIPI_write_cmos_sensor(0x5689,temp);

    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_5]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_6]==TRUE)    { temp=temp|0xF0;}
    //write 0x568A
    OV2685MIPI_write_cmos_sensor(0x568A,temp);
    
    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_7]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_8]==TRUE)    { temp=temp|0xF0;}
    //write 0x568B
    OV2685MIPI_write_cmos_sensor(0x568B,temp);

    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_9]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_10]==TRUE)  { temp=temp|0xF0;}
	//write 0x568C
    OV2685MIPI_write_cmos_sensor(0x568C,temp);

    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_11]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_12]==TRUE)    { temp=temp|0xF0;}
    //write 0x568D
    OV2685MIPI_write_cmos_sensor(0x568D,temp);    
    
    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_13]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_14]==TRUE)    { temp=temp|0xF0;}
	//write 0x568E
    OV2685MIPI_write_cmos_sensor(0x568E,temp);

    temp=0x11;
    if(AE_1_ARRAY[AE_SECTION_INDEX_15]==TRUE)    { temp=temp|0x0F;}
    if(AE_1_ARRAY[AE_SECTION_INDEX_16]==TRUE)    { temp=temp|0xF0;}
	//write 0x568F
    OV2685MIPI_write_cmos_sensor(0x568F,temp);
}

void printAE_1_ARRAY(void)
{
    UINT32 i;
    for(i=0; i<AE_SECTION_INDEX_MAX; i++)
    {
        OV2685MIPISENSORDB("AE_1_ARRAY[%2d]=%d\n", i, AE_1_ARRAY[i]);
    }
}

void printAE_2_ARRAY(void)
{
    UINT32 i, j;
    OV2685MIPISENSORDB("\t\t");
    for(i=0; i<AE_VERTICAL_BLOCKS; i++)
    {
        OV2685MIPISENSORDB("      line[%2d]", i);
    }
    printk("\n");
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        OV2685MIPISENSORDB("\trow[%2d]", j);
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        {
            //SENSORDB("AE_2_ARRAY[%2d][%2d]=%d\n", j,i,AE_2_ARRAY[j][i]);
            OV2685MIPISENSORDB("  %7d", AE_2_ARRAY[j][i]);
        }
        OV2685MIPISENSORDB("\n");
    }
}

void clearAE_2_ARRAY(void)
{
    UINT32 i, j;
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        {AE_2_ARRAY[j][i]=FALSE;}
    }
}

void mapAE_2_ARRAY_To_AE_1_ARRAY(void)
{
    UINT32 i, j;
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        { AE_1_ARRAY[j*AE_VERTICAL_BLOCKS+i] = AE_2_ARRAY[j][i];}
    }
}

void mapMiddlewaresizePointToPreviewsizePoint(
    UINT32 mx,
    UINT32 my,
    UINT32 mw,
    UINT32 mh,
    UINT32 * pvx,
    UINT32 * pvy,
    UINT32 pvw,
    UINT32 pvh
)
{
    *pvx = pvw * mx / mw;
    *pvy = pvh * my / mh;
    OV2685MIPISENSORDB("mapping middlware x[%d],y[%d], [%d X %d]\n\t\tto x[%d],y[%d],[%d X %d]\n ",
        mx, my, mw, mh, *pvx, *pvy, pvw, pvh);
}


void calcLine(void)
{//line[5]
    UINT32 i;
    UINT32 step = PRV_W / AE_VERTICAL_BLOCKS;
    for(i=0; i<=AE_VERTICAL_BLOCKS; i++)
    {
        *(&line_coordinate[0]+i) = step*i;
        OV2685MIPISENSORDB("line[%d]=%d\t",i, *(&line_coordinate[0]+i));
    }
    OV2685MIPISENSORDB("\n");
}

void calcRow(void)
{//row[5]
    UINT32 i;
    UINT32 step = PRV_H / AE_HORIZONTAL_BLOCKS;
    for(i=0; i<=AE_HORIZONTAL_BLOCKS; i++)
    {
        *(&row_coordinate[0]+i) = step*i;
        OV2685MIPISENSORDB("row[%d]=%d\t",i,*(&row_coordinate[0]+i));
    }
    OV2685MIPISENSORDB("\n");
}

void calcPointsAELineRowCoordinate(UINT32 x, UINT32 y, UINT32 * linenum, UINT32 * rownum)
{
    UINT32 i;
    i = 1;
    while(i<=AE_VERTICAL_BLOCKS)
    {
        if(x<line_coordinate[i])
        {
            *linenum = i;
            break;
        }
        *linenum = i++;
    }
    
    i = 1;
    while(i<=AE_HORIZONTAL_BLOCKS)
    {
        if(y<row_coordinate[i])
        {
            *rownum = i;
            break;
        }
        *rownum = i++;
    }
    OV2685MIPISENSORDB("PV point [%d, %d] to section line coordinate[%d] row[%d]\n",x,y,*linenum,*rownum);
}



MINT32 clampSection(UINT32 x, UINT32 min, UINT32 max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

void mapCoordinate(UINT32 linenum, UINT32 rownum, UINT32 * sectionlinenum, UINT32 * sectionrownum)
{
    *sectionlinenum = clampSection(linenum-1,0,AE_VERTICAL_BLOCKS-1);
    *sectionrownum = clampSection(rownum-1,0,AE_HORIZONTAL_BLOCKS-1);	
    OV2685MIPISENSORDB("mapCoordinate from[%d][%d] to[%d][%d]\n",
		linenum, rownum,*sectionlinenum,*sectionrownum);
}

void mapRectToAE_2_ARRAY(UINT32 x0, UINT32 y0, UINT32 x1, UINT32 y1)
{
    UINT32 i, j;
    OV2685MIPISENSORDB("([%d][%d]),([%d][%d])\n", x0,y0,x1,y1);
    clearAE_2_ARRAY();
    x0=clampSection(x0,0,AE_VERTICAL_BLOCKS-1);
    y0=clampSection(y0,0,AE_HORIZONTAL_BLOCKS-1);
    x1=clampSection(x1,0,AE_VERTICAL_BLOCKS-1);
    y1=clampSection(y1,0,AE_HORIZONTAL_BLOCKS-1);

    for(j=y0; j<=y1; j++)
    {
        for(i=x0; i<=x1; i++)
        {
            AE_2_ARRAY[j][i]=TRUE;
        }
    }
}

void resetPVAE_2_ARRAY(void)
{
    mapRectToAE_2_ARRAY(1,1,2,2);
}

//update ae window
//@input zone[] addr
void OV2685_FOCUS_Set_AE_Window(UINT32 zone_addr)
{//update global zone
    OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685_FOCUS_Set_AE_Window function\n");
    //input:
    UINT32 FD_XS;
    UINT32 FD_YS;	
    UINT32 x0, y0, x1, y1;
    UINT32 pvx0, pvy0, pvx1, pvy1;
    UINT32 linenum, rownum;
    UINT32 rightbottomlinenum,rightbottomrownum;
    UINT32 leftuplinenum,leftuprownum;
    UINT32* zone = (UINT32*)zone_addr;
    x0 = *zone;
    y0 = *(zone + 1);
    x1 = *(zone + 2);
    y1 = *(zone + 3);	
    FD_XS = *(zone + 4);
    FD_YS = *(zone + 5);

    OV2685MIPISENSORDB("AE x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",
    x0, y0, x1, y1, FD_XS, FD_YS);	
    
    //print_sensor_ae_section();
    //print_AE_section();	

    //1.transfer points to preview size
    //UINT32 pvx0, pvy0, pvx1, pvy1;
    mapMiddlewaresizePointToPreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, PRV_W, PRV_H);
    mapMiddlewaresizePointToPreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, PRV_W, PRV_H);
    
    //2.sensor AE line and row coordinate
    calcLine();
    calcRow();

    //3.calc left up point to section
    //UINT32 linenum, rownum;
    calcPointsAELineRowCoordinate(pvx0,pvy0,&linenum,&rownum);    
    //UINT32 leftuplinenum,leftuprownum;
    mapCoordinate(linenum, rownum, &leftuplinenum, &leftuprownum);
    //SENSORDB("leftuplinenum=%d,leftuprownum=%d\n",leftuplinenum,leftuprownum);

    //4.calc right bottom point to section
    calcPointsAELineRowCoordinate(pvx1,pvy1,&linenum,&rownum);    
    //UINT32 rightbottomlinenum,rightbottomrownum;
    mapCoordinate(linenum, rownum, &rightbottomlinenum, &rightbottomrownum);
    //SENSORDB("rightbottomlinenum=%d,rightbottomrownum=%d\n",rightbottomlinenum,rightbottomrownum);

    //5.update global section array
    mapRectToAE_2_ARRAY(leftuplinenum, leftuprownum, rightbottomlinenum, rightbottomrownum);
    //print_AE_section();

    //6.write to reg
    mapAE_2_ARRAY_To_AE_1_ARRAY();
    //printAE_1_ARRAY();
    printAE_2_ARRAY();
    writeAEReg();
	 OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685_FOCUS_Set_AE_Window function\n");
}
//=====================touch AE end==========================//
static void OV2685MIPIinitalvariable()
{
	spin_lock(&ov2685mipi_drv_lock);
	OV2685MIPISensor.VideoMode = KAL_FALSE;
	OV2685MIPISensor.NightMode = KAL_FALSE;
	OV2685MIPISensor.Fps = 100;
	OV2685MIPISensor.ShutterStep= 0xde;
	OV2685MIPISensor.CaptureDummyPixels = 0;
	OV2685MIPISensor.CaptureDummyLines = 0;
	OV2685MIPISensor.PreviewDummyPixels = 0;
	OV2685MIPISensor.PreviewDummyLines = 0;
	OV2685MIPISensor.SensorMode= SENSOR_MODE_INIT;
	OV2685MIPISensor.IsPVmode= KAL_TRUE;	
	OV2685MIPISensor.PreviewPclk= 560;
	OV2685MIPISensor.CapturePclk= 900;
	OV2685MIPISensor.ZsdturePclk= 900;
	OV2685MIPISensor.PreviewShutter=0x5c4; 
	OV2685MIPISensor.SensorGain=0x38;
	OV2685MIPISensor.manualAEStart=0;
	OV2685MIPISensor.isoSpeed=AE_ISO_100;
	OV2685MIPISensor.userAskAeLock=KAL_FALSE;
    OV2685MIPISensor.userAskAwbLock=KAL_FALSE;
	OV2685MIPISensor.currentExposureTime=0;
    OV2685MIPISensor.currentShutter=0;
	OV2685MIPISensor.zsd_flag=0;
	OV2685MIPISensor.currentextshutter=0;
	OV2685MIPISensor.AF_window_x=0;
	OV2685MIPISensor.AF_window_y=0;
	OV2685MIPISensor.awbMode = AWB_MODE_AUTO;
	OV2685MIPISensor.iWB=AWB_MODE_AUTO;
	spin_unlock(&ov2685mipi_drv_lock);
}
void OV2685MIPIGetExifInfo(UINT32 exifAddr)
{
	 OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIGetExifInfo function\n");
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 20;
    pExifInfo->AEISOSpeed = OV2685MIPISensor.isoSpeed;
	pExifInfo->AWBMode = OV2685MIPISensor.awbMode;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = OV2685MIPISensor.isoSpeed;
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIGetExifInfo function\n");
}
static void OV2685MIPISetDummy(kal_uint32 dummy_pixels, kal_uint32 dummy_lines)
{
		OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPISetDummy function:dummy_pixels=%d,dummy_lines=%d\n",dummy_pixels,dummy_lines);
		if (OV2685MIPISensor.IsPVmode)  
        {
            dummy_pixels = dummy_pixels+OV2685MIPI_PV_PERIOD_PIXEL_NUMS; 
            OV2685MIPI_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));         
            OV2685MIPI_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
      
            dummy_lines= dummy_lines+OV2685MIPI_PV_PERIOD_LINE_NUMS; 
            OV2685MIPI_write_cmos_sensor(0x380F,(dummy_lines&0xFF));       
            OV2685MIPI_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
        } 
        else
        {
            dummy_pixels = dummy_pixels+OV2685MIPI_FULL_PERIOD_PIXEL_NUMS; 
            OV2685MIPI_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));         
            OV2685MIPI_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
      
            dummy_lines= dummy_lines+OV2685MIPI_FULL_PERIOD_LINE_NUMS; 
            OV2685MIPI_write_cmos_sensor(0x380F,(dummy_lines&0xFF));       
            OV2685MIPI_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
        } 
		OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPISetDummy function:\n ");
}    /* OV2685MIPI_set_dummy */

/*************************************************************************
* FUNCTION
*	OV2685MIPIWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void OV2685MIPIWriteShutter(kal_uint32 shutter)
{
	kal_uint32 extra_exposure_vts = 0;
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIWriteShutter function:shutter=%d\n ",shutter);
	if (shutter < 1)
	{
		shutter = 1;
	}
	if (shutter > OV2685MIPI_FULL_EXPOSURE_LIMITATION) 
	{
		extra_exposure_vts =shutter+4;
		OV2685MIPI_write_cmos_sensor(0x380f, extra_exposure_vts & 0xFF);          // EXVTS[b7~b0]
		OV2685MIPI_write_cmos_sensor(0x380e, (extra_exposure_vts & 0xFF00) >> 8); // EXVTS[b15~b8]
		OV2685MIPI_write_cmos_sensor(0x350D,0x00); 
		OV2685MIPI_write_cmos_sensor(0x350C,0x00);
	}
	shutter*=16;
	OV2685MIPI_write_cmos_sensor(0x3502, (shutter & 0x00FF));           //AEC[7:0]
	OV2685MIPI_write_cmos_sensor(0x3501, ((shutter & 0x0FF00) >>8));  //AEC[15:8]
	OV2685MIPI_write_cmos_sensor(0x3500, ((shutter & 0xFF0000) >> 16));	
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIWriteShutter function:\n ");
}    /* OV2685MIPI_write_shutter */
/*************************************************************************
* FUNCTION
*	OV2685MIPIExpWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void OV2685MIPIWriteExpShutter(kal_uint32 shutter)
{
	shutter*=16;
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIWriteExpShutter function:shutter=%d\n ",shutter);
	OV2685MIPI_write_cmos_sensor(0x3502, (shutter & 0x00FF));           //AEC[7:0]
	OV2685MIPI_write_cmos_sensor(0x3501, ((shutter & 0x0FF00) >>8));  //AEC[15:8]
	OV2685MIPI_write_cmos_sensor(0x3500, ((shutter & 0xFF0000) >> 16));	
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIWriteExpShutter function:\n ");
}    /* OV2685MIPI_write_shutter */


/*************************************************************************
* FUNCTION
*	OV2685MIPIWriteSensorGain
*
* DESCRIPTION
*	This function used to write the sensor gain.
*
* PARAMETERS
*	1. kal_uint32 : The sensor gain want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void OV2685MIPIWriteSensorGain(kal_uint32 gain)
{
	kal_uint16 temp_reg ;
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIWriteSensorGain function:gain=%d\n",gain);
	if(gain > 1024)  ASSERT(0);
	temp_reg = 0;
	temp_reg=gain&0x0FF;	
	OV2685MIPI_write_cmos_sensor(0x350B,temp_reg);
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIWriteSensorGain function:\n ");
}  /* OV2685MIPI_write_sensor_gain */

/*************************************************************************
* FUNCTION
*	OV2685MIPIReadShutter
*
* DESCRIPTION
*	This function read current shutter for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current shutter value.
*
*************************************************************************/
static kal_uint32 OV2685MIPIReadShutter(void)
{
	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIReadShutter function:\n ");
	temp_reg1 = OV2685MIPIYUV_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV2685MIPIYUV_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV2685MIPIYUV_read_cmos_sensor(0x3502);    // AEC[b7~b0]

	spin_lock(&ov2685mipi_drv_lock);
	OV2685MIPISensor.PreviewShutter  = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);
	spin_unlock(&ov2685mipi_drv_lock);
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIReadShutter function:\n ");	
	return OV2685MIPISensor.PreviewShutter;
} /* OV2685MIPI_read_shutter */


/*************************************************************************
* FUNCTION
*	OV2685MIPIReadSensorGain
*
* DESCRIPTION
*	This function read current sensor gain for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current sensor gain value.
*
*************************************************************************/
static kal_uint32 OV2685MIPIReadSensorGain(void)
{
	kal_uint32 sensor_gain = 0;
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIReadSensorGain function:\n ");
	sensor_gain=(OV2685MIPIYUV_read_cmos_sensor(0x350B)&0xFF); 
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIReadSensorGain function:\n ");
	return sensor_gain;
}  /* OV2685MIPIReadSensorGain */
/*************************************************************************
* FUNCTION
*	OV2685MIPI_set_AE_mode
*
* DESCRIPTION
*	This function OV2685MIPI_set_AE_mode.
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
static void OV2685MIPI_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 AeTemp;
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_set_AE_mode function:\n ");
    AeTemp = OV2685MIPIYUV_read_cmos_sensor(0x3503);
    if (AE_enable == KAL_TRUE)
    {
        // turn on AEC/AGC
        OV2685MIPI_write_cmos_sensor(0x3503,(AeTemp&(~0x03)));
    }
    else
    {
        // turn off AEC/AGC
      OV2685MIPI_write_cmos_sensor(0x3503,(AeTemp|0x03));
    }
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_set_AE_mode function:\n ");
}

/*************************************************************************
* FUNCTION
*	OV2685MIPI_set_AWB_mode
*
* DESCRIPTION
*	This function OV2685MIPI_set_AWB_mode.
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
static void OV2685MIPI_set_AWB_mode(kal_bool AWB_enable)
{
    kal_uint8 AwbTemp;
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_set_AWB_mode function:\n ");
	  AwbTemp = OV2685MIPIYUV_read_cmos_sensor(0x5180);   

    if (AWB_enable == KAL_TRUE)
    {
             
		OV2685MIPI_write_cmos_sensor(0x5180,AwbTemp&0xFD); 
		
    }
    else
    {             
		OV2685MIPI_write_cmos_sensor(0x5180,AwbTemp|0x02); 
		
    }
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_set_AWB_mode function:\n ");
}
static void OV2685MIPI_set_AWB_mode_UNLOCK()
{
    OV2685MIPI_set_AWB_mode(KAL_TRUE);
    if (!((SCENE_MODE_OFF == OV2685MIPISensor.sceneMode) || (SCENE_MODE_NORMAL == 
    OV2685MIPISensor.sceneMode) || (SCENE_MODE_HDR == OV2685MIPISensor.sceneMode)))
    {
      OV2685MIPI_set_scene_mode(OV2685MIPISensor.sceneMode);        
    }
    if (!((AWB_MODE_OFF == OV2685MIPISensor.iWB) || (AWB_MODE_AUTO == OV2685MIPISensor.iWB)))
    {
	   OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_set_AWB_mode_UNLOCK function:iWB=%d\n ",OV2685MIPISensor.iWB);
	   OV2685MIPI_set_param_wb(OV2685MIPISensor.iWB);
    }
    return;
}

/*************************************************************************
* FUNCTION
*	OV2685MIPI_GetSensorID
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
//static 
kal_uint32 OV2685MIPI_GetSensorID(kal_uint32 *sensorID)
{
    volatile signed char i;
	kal_uint32 sensor_id=0;
	kal_uint8 temp_sccb_addr = 0;
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_GetSensorID function:\n ");
	OV2685MIPI_write_cmos_sensor(0103,0x01);// Reset sensor
	msleep(5);
	for(i=0;i<3;i++)
	{
		sensor_id = (OV2685MIPIYUV_read_cmos_sensor(0x300A) << 8) | OV2685MIPIYUV_read_cmos_sensor(0x300B);
		OV2685MIPISENSORDB("OV2685MIPI READ ID: %x",sensor_id);
		if(sensor_id != OV2685MIPI_SENSOR_ID)
		{	
			*sensorID =0xffffffff;
			return ERROR_SENSOR_CONNECT_FAIL;
		}
		else
			{
			*sensorID=OV2685MIPI_SENSOR_ID;
		        break;
			}
	}
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_GetSensorID function:\n ");
   return ERROR_NONE;    

}   

/*************************************************************************
* FUNCTION
*    OV2685MIPIInitialSetting
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
//static 
void OV2685MIPIInitialSetting(void)
{
	//;OV2685MIPI 1280x960,30fps
	//56Mhz, 224Mbps/Lane, 2 Lane
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIInitialSetting function:\n ");

OV2685MIPI_write_cmos_sensor(0x0103,0x01);
OV2685MIPI_write_cmos_sensor(0x3002,0x00);
OV2685MIPI_write_cmos_sensor(0x3016,0x3c);// 1c
OV2685MIPI_write_cmos_sensor(0x3018,0x84);
OV2685MIPI_write_cmos_sensor(0x301d,0xf0);
OV2685MIPI_write_cmos_sensor(0x3020,0x00);
OV2685MIPI_write_cmos_sensor(0x3082,0x2c);
OV2685MIPI_write_cmos_sensor(0x3083,0x03);
OV2685MIPI_write_cmos_sensor(0x3084,0x07);
OV2685MIPI_write_cmos_sensor(0x3085,0x03);
OV2685MIPI_write_cmos_sensor(0x3086,0x00);
OV2685MIPI_write_cmos_sensor(0x3087,0x00);
OV2685MIPI_write_cmos_sensor(0x3501,0x4e);
OV2685MIPI_write_cmos_sensor(0x3502,0xe0);
OV2685MIPI_write_cmos_sensor(0x3503,0x03);
OV2685MIPI_write_cmos_sensor(0x350b,0x36);
OV2685MIPI_write_cmos_sensor(0x3600,0xb4);
OV2685MIPI_write_cmos_sensor(0x3603,0x39);
OV2685MIPI_write_cmos_sensor(0x3604,0x24);
OV2685MIPI_write_cmos_sensor(0x3605,0x00);
OV2685MIPI_write_cmos_sensor(0x3620,0x24);
OV2685MIPI_write_cmos_sensor(0x3621,0x34);
OV2685MIPI_write_cmos_sensor(0x3622,0x03);
OV2685MIPI_write_cmos_sensor(0x3628,0x10);
OV2685MIPI_write_cmos_sensor(0x3705,0x3c);
OV2685MIPI_write_cmos_sensor(0x370a,0x21);
OV2685MIPI_write_cmos_sensor(0x370c,0x50);
OV2685MIPI_write_cmos_sensor(0x370d,0xc0);
OV2685MIPI_write_cmos_sensor(0x3717,0x58);
OV2685MIPI_write_cmos_sensor(0x3718,0x80);
OV2685MIPI_write_cmos_sensor(0x3720,0x00);
OV2685MIPI_write_cmos_sensor(0x3721,0x09);
OV2685MIPI_write_cmos_sensor(0x3722,0x06);
OV2685MIPI_write_cmos_sensor(0x3723,0x59);
OV2685MIPI_write_cmos_sensor(0x3738,0x99);
OV2685MIPI_write_cmos_sensor(0x3781,0x80);
OV2685MIPI_write_cmos_sensor(0x3789,0x60);
OV2685MIPI_write_cmos_sensor(0x3800,0x00);
OV2685MIPI_write_cmos_sensor(0x3801,0x00);
OV2685MIPI_write_cmos_sensor(0x3802,0x00);
OV2685MIPI_write_cmos_sensor(0x3803,0x00);
OV2685MIPI_write_cmos_sensor(0x3804,0x06);
OV2685MIPI_write_cmos_sensor(0x3805,0x4f);
OV2685MIPI_write_cmos_sensor(0x3806,0x04);
OV2685MIPI_write_cmos_sensor(0x3807,0xbf);
OV2685MIPI_write_cmos_sensor(0x3808,0x06);
OV2685MIPI_write_cmos_sensor(0x3809,0x40);
OV2685MIPI_write_cmos_sensor(0x380a,0x04);
OV2685MIPI_write_cmos_sensor(0x380b,0xb0);
OV2685MIPI_write_cmos_sensor(0x380c,0x06);
OV2685MIPI_write_cmos_sensor(0x380d,0xa4);
OV2685MIPI_write_cmos_sensor(0x380e,0x05);
OV2685MIPI_write_cmos_sensor(0x380f,0x0e);
OV2685MIPI_write_cmos_sensor(0x3810,0x00);
OV2685MIPI_write_cmos_sensor(0x3811,0x08);
OV2685MIPI_write_cmos_sensor(0x3812,0x00);
OV2685MIPI_write_cmos_sensor(0x3813,0x08);
OV2685MIPI_write_cmos_sensor(0x3814,0x11);
OV2685MIPI_write_cmos_sensor(0x3815,0x11);
OV2685MIPI_write_cmos_sensor(0x3819,0x04);
OV2685MIPI_write_cmos_sensor(0x3820,0xc0);//c4
OV2685MIPI_write_cmos_sensor(0x3821,0x00);//04
OV2685MIPI_write_cmos_sensor(0x4000,0x81);
OV2685MIPI_write_cmos_sensor(0x4001,0x40);
OV2685MIPI_write_cmos_sensor(0x4008,0x02);
OV2685MIPI_write_cmos_sensor(0x4009,0x09);
OV2685MIPI_write_cmos_sensor(0x4300,0x30);
OV2685MIPI_write_cmos_sensor(0x430e,0x00);
OV2685MIPI_write_cmos_sensor(0x4602,0x02);
OV2685MIPI_write_cmos_sensor(0x4837,0x1e);
OV2685MIPI_write_cmos_sensor(0x5000,0xff);
OV2685MIPI_write_cmos_sensor(0x5001,0x05);
OV2685MIPI_write_cmos_sensor(0x5002,0x32);
OV2685MIPI_write_cmos_sensor(0x5003,0x04);
OV2685MIPI_write_cmos_sensor(0x5004,0xff);
OV2685MIPI_write_cmos_sensor(0x5005,0x12);
OV2685MIPI_write_cmos_sensor(0x0100,0x01);
                                   
OV2685MIPI_write_cmos_sensor(0x5180,0xf4);
OV2685MIPI_write_cmos_sensor(0x5181,0x11);
OV2685MIPI_write_cmos_sensor(0x5182,0x41);
OV2685MIPI_write_cmos_sensor(0x5183,0x42);
OV2685MIPI_write_cmos_sensor(0x5184,0x78);
OV2685MIPI_write_cmos_sensor(0x5185,0x58);
OV2685MIPI_write_cmos_sensor(0x5186,0xb5);
OV2685MIPI_write_cmos_sensor(0x5187,0xb2);
OV2685MIPI_write_cmos_sensor(0x5188,0x08);
OV2685MIPI_write_cmos_sensor(0x5189,0x0e);
OV2685MIPI_write_cmos_sensor(0x518a,0x0c);
OV2685MIPI_write_cmos_sensor(0x518b,0x4c);
OV2685MIPI_write_cmos_sensor(0x518c,0x38);
OV2685MIPI_write_cmos_sensor(0x518d,0xf8);
OV2685MIPI_write_cmos_sensor(0x518e,0x04);
OV2685MIPI_write_cmos_sensor(0x518f,0x7f);
OV2685MIPI_write_cmos_sensor(0x5190,0x40);
OV2685MIPI_write_cmos_sensor(0x5191,0x5f);
OV2685MIPI_write_cmos_sensor(0x5192,0x40);
OV2685MIPI_write_cmos_sensor(0x5193,0xff);
OV2685MIPI_write_cmos_sensor(0x5194,0x40);
OV2685MIPI_write_cmos_sensor(0x5195,0x07);
OV2685MIPI_write_cmos_sensor(0x5196,0x04);
OV2685MIPI_write_cmos_sensor(0x5197,0x04);
OV2685MIPI_write_cmos_sensor(0x5198,0x00);
OV2685MIPI_write_cmos_sensor(0x5199,0x05);
OV2685MIPI_write_cmos_sensor(0x519a,0xd2);
OV2685MIPI_write_cmos_sensor(0x519b,0x10);
OV2685MIPI_write_cmos_sensor(0x5200,0x09);
OV2685MIPI_write_cmos_sensor(0x5201,0x02);
OV2685MIPI_write_cmos_sensor(0x5202,0x06);
OV2685MIPI_write_cmos_sensor(0x5203,0x20);
OV2685MIPI_write_cmos_sensor(0x5204,0x41);
OV2685MIPI_write_cmos_sensor(0x5205,0x16);
OV2685MIPI_write_cmos_sensor(0x5206,0x00);
OV2685MIPI_write_cmos_sensor(0x5207,0x05);
OV2685MIPI_write_cmos_sensor(0x520b,0x30);
OV2685MIPI_write_cmos_sensor(0x520c,0x75);
OV2685MIPI_write_cmos_sensor(0x520d,0x00);
OV2685MIPI_write_cmos_sensor(0x520e,0x30);
OV2685MIPI_write_cmos_sensor(0x520f,0x75);
OV2685MIPI_write_cmos_sensor(0x5210,0x00);
OV2685MIPI_write_cmos_sensor(0x5280,0x14);
OV2685MIPI_write_cmos_sensor(0x5281,0x02);
OV2685MIPI_write_cmos_sensor(0x5282,0x02);
OV2685MIPI_write_cmos_sensor(0x5283,0x04);
OV2685MIPI_write_cmos_sensor(0x5284,0x06);
OV2685MIPI_write_cmos_sensor(0x5285,0x08);
OV2685MIPI_write_cmos_sensor(0x5286,0x0c);
OV2685MIPI_write_cmos_sensor(0x5287,0x10);
OV2685MIPI_write_cmos_sensor(0x5300,0xc5);
OV2685MIPI_write_cmos_sensor(0x5301,0xa0);
OV2685MIPI_write_cmos_sensor(0x5302,0x06);
OV2685MIPI_write_cmos_sensor(0x5303,0x08);
OV2685MIPI_write_cmos_sensor(0x5304,0x18);
OV2685MIPI_write_cmos_sensor(0x5305,0x30);
OV2685MIPI_write_cmos_sensor(0x5306,0x60);
OV2685MIPI_write_cmos_sensor(0x5307,0xc0);
OV2685MIPI_write_cmos_sensor(0x5308,0x82);
OV2685MIPI_write_cmos_sensor(0x5309,0x00);
OV2685MIPI_write_cmos_sensor(0x530a,0x26);
OV2685MIPI_write_cmos_sensor(0x530b,0x02);
OV2685MIPI_write_cmos_sensor(0x530c,0x02);
OV2685MIPI_write_cmos_sensor(0x530d,0x00);
OV2685MIPI_write_cmos_sensor(0x530e,0x0c);
OV2685MIPI_write_cmos_sensor(0x530f,0x14);
OV2685MIPI_write_cmos_sensor(0x5310,0x1a);
OV2685MIPI_write_cmos_sensor(0x5311,0x20);
OV2685MIPI_write_cmos_sensor(0x5312,0x80);
OV2685MIPI_write_cmos_sensor(0x5313,0x4b);
OV2685MIPI_write_cmos_sensor(0x5380,0x01);
OV2685MIPI_write_cmos_sensor(0x5381,0x52);
OV2685MIPI_write_cmos_sensor(0x5382,0x00);
OV2685MIPI_write_cmos_sensor(0x5383,0x4a);
OV2685MIPI_write_cmos_sensor(0x5384,0x00);
OV2685MIPI_write_cmos_sensor(0x5385,0xb6);
OV2685MIPI_write_cmos_sensor(0x5386,0x00);
OV2685MIPI_write_cmos_sensor(0x5387,0x8d);
OV2685MIPI_write_cmos_sensor(0x5388,0x00);
OV2685MIPI_write_cmos_sensor(0x5389,0x3a);
OV2685MIPI_write_cmos_sensor(0x538a,0x00);
OV2685MIPI_write_cmos_sensor(0x538b,0xa6);
OV2685MIPI_write_cmos_sensor(0x538c,0x00);
OV2685MIPI_write_cmos_sensor(0x5400,0x0d);
OV2685MIPI_write_cmos_sensor(0x5401,0x18);
OV2685MIPI_write_cmos_sensor(0x5402,0x31);
OV2685MIPI_write_cmos_sensor(0x5403,0x5a);
OV2685MIPI_write_cmos_sensor(0x5404,0x65);
OV2685MIPI_write_cmos_sensor(0x5405,0x6f);
OV2685MIPI_write_cmos_sensor(0x5406,0x77);
OV2685MIPI_write_cmos_sensor(0x5407,0x80);
OV2685MIPI_write_cmos_sensor(0x5408,0x87);
OV2685MIPI_write_cmos_sensor(0x5409,0x8f);
OV2685MIPI_write_cmos_sensor(0x540a,0xa2);
OV2685MIPI_write_cmos_sensor(0x540b,0xb2);
OV2685MIPI_write_cmos_sensor(0x540c,0xcc);
OV2685MIPI_write_cmos_sensor(0x540d,0xe4);
OV2685MIPI_write_cmos_sensor(0x540e,0xf0);
OV2685MIPI_write_cmos_sensor(0x540f,0xa0);
OV2685MIPI_write_cmos_sensor(0x5410,0x6e);
OV2685MIPI_write_cmos_sensor(0x5411,0x06);
OV2685MIPI_write_cmos_sensor(0x5480,0x19);
OV2685MIPI_write_cmos_sensor(0x5481,0x00);
OV2685MIPI_write_cmos_sensor(0x5482,0x09);
OV2685MIPI_write_cmos_sensor(0x5483,0x12);
OV2685MIPI_write_cmos_sensor(0x5484,0x04);
OV2685MIPI_write_cmos_sensor(0x5485,0x06);
OV2685MIPI_write_cmos_sensor(0x5486,0x08);
OV2685MIPI_write_cmos_sensor(0x5487,0x0c);
OV2685MIPI_write_cmos_sensor(0x5488,0x10);
OV2685MIPI_write_cmos_sensor(0x5489,0x18);
OV2685MIPI_write_cmos_sensor(0x5500,0x02);
OV2685MIPI_write_cmos_sensor(0x5501,0x03);
OV2685MIPI_write_cmos_sensor(0x5502,0x04);
OV2685MIPI_write_cmos_sensor(0x5503,0x05);
OV2685MIPI_write_cmos_sensor(0x5504,0x06);
OV2685MIPI_write_cmos_sensor(0x5505,0x08);
OV2685MIPI_write_cmos_sensor(0x5506,0x00);
OV2685MIPI_write_cmos_sensor(0x5600,0x02);
OV2685MIPI_write_cmos_sensor(0x5603,0x40);
OV2685MIPI_write_cmos_sensor(0x5604,0x38);
OV2685MIPI_write_cmos_sensor(0x5609,0x20);
OV2685MIPI_write_cmos_sensor(0x560a,0x80);
OV2685MIPI_write_cmos_sensor(0x5800,0x03);
OV2685MIPI_write_cmos_sensor(0x5801,0x24);
OV2685MIPI_write_cmos_sensor(0x5802,0x02);
OV2685MIPI_write_cmos_sensor(0x5803,0x40);
OV2685MIPI_write_cmos_sensor(0x5804,0x34);
OV2685MIPI_write_cmos_sensor(0x5805,0x05);
OV2685MIPI_write_cmos_sensor(0x5806,0x12);
OV2685MIPI_write_cmos_sensor(0x5807,0x05);
OV2685MIPI_write_cmos_sensor(0x5808,0x03);
OV2685MIPI_write_cmos_sensor(0x5809,0x3c);
OV2685MIPI_write_cmos_sensor(0x580a,0x02);
OV2685MIPI_write_cmos_sensor(0x580b,0x40);
OV2685MIPI_write_cmos_sensor(0x580c,0x26);
OV2685MIPI_write_cmos_sensor(0x580d,0x05);
OV2685MIPI_write_cmos_sensor(0x580e,0x52);
OV2685MIPI_write_cmos_sensor(0x580f,0x06);
OV2685MIPI_write_cmos_sensor(0x5810,0x03);
OV2685MIPI_write_cmos_sensor(0x5811,0x28);
OV2685MIPI_write_cmos_sensor(0x5812,0x02);
OV2685MIPI_write_cmos_sensor(0x5813,0x40);
OV2685MIPI_write_cmos_sensor(0x5814,0x24);
OV2685MIPI_write_cmos_sensor(0x5815,0x05);
OV2685MIPI_write_cmos_sensor(0x5816,0x42);
OV2685MIPI_write_cmos_sensor(0x5817,0x06);
OV2685MIPI_write_cmos_sensor(0x5818,0x0d);
OV2685MIPI_write_cmos_sensor(0x5819,0x40);
OV2685MIPI_write_cmos_sensor(0x581a,0x04);
OV2685MIPI_write_cmos_sensor(0x581b,0x0c);
OV2685MIPI_write_cmos_sensor(0x3a03,0x4a);
OV2685MIPI_write_cmos_sensor(0x3a04,0x40);
OV2685MIPI_write_cmos_sensor(0x3503,0x00);       
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIInitialSetting function:\n ");
} 
/*****************************************************************
* FUNCTION
*    OV2685MIPIPreviewSetting
*
* DESCRIPTION
*    This function config Preview setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void OV2685MIPIPreviewSetting_SVGA(void)
{
	//;OV2685MIPI 1280x960,30fps
	//56Mhz, 224Mbps/Lane, 2Lane.
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIPreviewSetting_SVGA function:\n ");
OV2685MIPI_write_cmos_sensor(0x0103,0x01);
OV2685MIPI_write_cmos_sensor(0x3002,0x00);
OV2685MIPI_write_cmos_sensor(0x3016,0x3c);// 1c //3c
OV2685MIPI_write_cmos_sensor(0x3018,0x84);//84
OV2685MIPI_write_cmos_sensor(0x301d,0xf0);
OV2685MIPI_write_cmos_sensor(0x3020,0x00);
OV2685MIPI_write_cmos_sensor(0x3082,0x2c);
OV2685MIPI_write_cmos_sensor(0x3083,0x03);
OV2685MIPI_write_cmos_sensor(0x3084,0x07);
OV2685MIPI_write_cmos_sensor(0x3085,0x03);
OV2685MIPI_write_cmos_sensor(0x3086,0x00);
OV2685MIPI_write_cmos_sensor(0x3087,0x00);
OV2685MIPI_write_cmos_sensor(0x3501,0x4e); ///4e
OV2685MIPI_write_cmos_sensor(0x3502,0xe0); ///e0
OV2685MIPI_write_cmos_sensor(0x3503,0x03);
OV2685MIPI_write_cmos_sensor(0x350b,0x36);
OV2685MIPI_write_cmos_sensor(0x3600,0xb4);
OV2685MIPI_write_cmos_sensor(0x3603,0x39);
OV2685MIPI_write_cmos_sensor(0x3604,0x24);
OV2685MIPI_write_cmos_sensor(0x3605,0x00);
OV2685MIPI_write_cmos_sensor(0x3620,0x26);
OV2685MIPI_write_cmos_sensor(0x3621,0x37);
OV2685MIPI_write_cmos_sensor(0x3622,0x04);
OV2685MIPI_write_cmos_sensor(0x3628,0x10);
OV2685MIPI_write_cmos_sensor(0x3705,0x3c);
OV2685MIPI_write_cmos_sensor(0x370a,0x23); //21
OV2685MIPI_write_cmos_sensor(0x370c,0x50);
OV2685MIPI_write_cmos_sensor(0x370d,0xc0);
OV2685MIPI_write_cmos_sensor(0x3717,0x58);
OV2685MIPI_write_cmos_sensor(0x3718,0x88);
OV2685MIPI_write_cmos_sensor(0x3720,0x00);
OV2685MIPI_write_cmos_sensor(0x3721,0x00);
OV2685MIPI_write_cmos_sensor(0x3722,0x00);
OV2685MIPI_write_cmos_sensor(0x3723,0x00);
OV2685MIPI_write_cmos_sensor(0x3738,0x00);
OV2685MIPI_write_cmos_sensor(0x3781,0x80);
OV2685MIPI_write_cmos_sensor(0x3789,0x60);

/*
OV2685MIPI_write_cmos_sensor(0x3800,0x00);
OV2685MIPI_write_cmos_sensor(0x3801,0xa0);
OV2685MIPI_write_cmos_sensor(0x3802,0x00);
OV2685MIPI_write_cmos_sensor(0x3803,0x78);
OV2685MIPI_write_cmos_sensor(0x3804,0x05);
OV2685MIPI_write_cmos_sensor(0x3805,0xaf);
OV2685MIPI_write_cmos_sensor(0x3806,0x04);
OV2685MIPI_write_cmos_sensor(0x3807,0x47);
OV2685MIPI_write_cmos_sensor(0x3808,0x05);
OV2685MIPI_write_cmos_sensor(0x3809,0x00);
OV2685MIPI_write_cmos_sensor(0x380a,0x03);
OV2685MIPI_write_cmos_sensor(0x380b,0xc0);
OV2685MIPI_write_cmos_sensor(0x380c,0x06); //1700
OV2685MIPI_write_cmos_sensor(0x380d,0xa4);
OV2685MIPI_write_cmos_sensor(0x380e,0x05); //1294
OV2685MIPI_write_cmos_sensor(0x380f,0x0e);
OV2685MIPI_write_cmos_sensor(0x3810,0x00);
OV2685MIPI_write_cmos_sensor(0x3811,0x08);
OV2685MIPI_write_cmos_sensor(0x3812,0x00);
OV2685MIPI_write_cmos_sensor(0x3813,0x08);
OV2685MIPI_write_cmos_sensor(0x3814,0x11);
OV2685MIPI_write_cmos_sensor(0x3815,0x11);
*/
OV2685MIPI_write_cmos_sensor(0x3800,0x00);
OV2685MIPI_write_cmos_sensor(0x3801,0x00);
OV2685MIPI_write_cmos_sensor(0x3802,0x00);
OV2685MIPI_write_cmos_sensor(0x3803,0x00);
OV2685MIPI_write_cmos_sensor(0x3804,0x06);
OV2685MIPI_write_cmos_sensor(0x3805,0x4f);
OV2685MIPI_write_cmos_sensor(0x3806,0x04);
OV2685MIPI_write_cmos_sensor(0x3807,0xbf);
OV2685MIPI_write_cmos_sensor(0x3808,0x03);
OV2685MIPI_write_cmos_sensor(0x3809,0x20);
OV2685MIPI_write_cmos_sensor(0x380a,0x02);
OV2685MIPI_write_cmos_sensor(0x380b,0x58);
OV2685MIPI_write_cmos_sensor(0x380c,0x06); //1700
OV2685MIPI_write_cmos_sensor(0x380d,0xac);
OV2685MIPI_write_cmos_sensor(0x380e,0x02); //644
OV2685MIPI_write_cmos_sensor(0x380f,0x84);
OV2685MIPI_write_cmos_sensor(0x3810,0x00);
OV2685MIPI_write_cmos_sensor(0x3811,0x04);
OV2685MIPI_write_cmos_sensor(0x3812,0x00);
OV2685MIPI_write_cmos_sensor(0x3813,0x04);
OV2685MIPI_write_cmos_sensor(0x3814,0x31);
OV2685MIPI_write_cmos_sensor(0x3815,0x31);


OV2685MIPI_write_cmos_sensor(0x3819,0x04);
OV2685MIPI_write_cmos_sensor(0x3820,0xc0);//c0//c4 //c6
OV2685MIPI_write_cmos_sensor(0x3821,0x00);//00 //04 //05
OV2685MIPI_write_cmos_sensor(0x4000,0x81);
OV2685MIPI_write_cmos_sensor(0x4001,0x40);
OV2685MIPI_write_cmos_sensor(0x4008,0x00);//02
OV2685MIPI_write_cmos_sensor(0x4009,0x03);//09
OV2685MIPI_write_cmos_sensor(0x4300,0x30);
OV2685MIPI_write_cmos_sensor(0x430e,0x00);
OV2685MIPI_write_cmos_sensor(0x4602,0x02);
OV2685MIPI_write_cmos_sensor(0x4837,0x1e);
OV2685MIPI_write_cmos_sensor(0x5000,0xff);
OV2685MIPI_write_cmos_sensor(0x5001,0x05);
OV2685MIPI_write_cmos_sensor(0x5002,0x32);
OV2685MIPI_write_cmos_sensor(0x5003,0x04);
OV2685MIPI_write_cmos_sensor(0x5004,0xff);
OV2685MIPI_write_cmos_sensor(0x5005,0x12);
OV2685MIPI_write_cmos_sensor(0x0100,0x01);

OV2685MIPI_write_cmos_sensor(0x5180,0xf4);
OV2685MIPI_write_cmos_sensor(0x5181,0x11);
OV2685MIPI_write_cmos_sensor(0x5182,0x41);
OV2685MIPI_write_cmos_sensor(0x5183,0x42);
OV2685MIPI_write_cmos_sensor(0x5184,0x78);
OV2685MIPI_write_cmos_sensor(0x5185,0x58);
OV2685MIPI_write_cmos_sensor(0x5186,0xb5);
OV2685MIPI_write_cmos_sensor(0x5187,0xb2);
OV2685MIPI_write_cmos_sensor(0x5188,0x08);
OV2685MIPI_write_cmos_sensor(0x5189,0x0e);
OV2685MIPI_write_cmos_sensor(0x518a,0x0c);
OV2685MIPI_write_cmos_sensor(0x518b,0x4c);
OV2685MIPI_write_cmos_sensor(0x518c,0x38);
OV2685MIPI_write_cmos_sensor(0x518d,0xf8);
OV2685MIPI_write_cmos_sensor(0x518e,0x04);
OV2685MIPI_write_cmos_sensor(0x518f,0x7f);
OV2685MIPI_write_cmos_sensor(0x5190,0x40);
OV2685MIPI_write_cmos_sensor(0x5191,0x5f);
OV2685MIPI_write_cmos_sensor(0x5192,0x40);
OV2685MIPI_write_cmos_sensor(0x5193,0xff);
OV2685MIPI_write_cmos_sensor(0x5194,0x40);
OV2685MIPI_write_cmos_sensor(0x5195,0x07);
OV2685MIPI_write_cmos_sensor(0x5196,0x04);
OV2685MIPI_write_cmos_sensor(0x5197,0x04);
OV2685MIPI_write_cmos_sensor(0x5198,0x00);
OV2685MIPI_write_cmos_sensor(0x5199,0x05);
OV2685MIPI_write_cmos_sensor(0x519a,0xd2);
OV2685MIPI_write_cmos_sensor(0x519b,0x10);
OV2685MIPI_write_cmos_sensor(0x5200,0x09);
OV2685MIPI_write_cmos_sensor(0x5201,0x02);
OV2685MIPI_write_cmos_sensor(0x5202,0x06);
OV2685MIPI_write_cmos_sensor(0x5203,0x20);
OV2685MIPI_write_cmos_sensor(0x5204,0x41);
OV2685MIPI_write_cmos_sensor(0x5205,0x16);
OV2685MIPI_write_cmos_sensor(0x5206,0x00);
OV2685MIPI_write_cmos_sensor(0x5207,0x05);
OV2685MIPI_write_cmos_sensor(0x520b,0x30);
OV2685MIPI_write_cmos_sensor(0x520c,0x75);
OV2685MIPI_write_cmos_sensor(0x520d,0x00);
OV2685MIPI_write_cmos_sensor(0x520e,0x30);
OV2685MIPI_write_cmos_sensor(0x520f,0x75);
OV2685MIPI_write_cmos_sensor(0x5210,0x00);
OV2685MIPI_write_cmos_sensor(0x5280,0x14);
OV2685MIPI_write_cmos_sensor(0x5281,0x02);
OV2685MIPI_write_cmos_sensor(0x5282,0x02);
OV2685MIPI_write_cmos_sensor(0x5283,0x04);
OV2685MIPI_write_cmos_sensor(0x5284,0x06);
OV2685MIPI_write_cmos_sensor(0x5285,0x08);
OV2685MIPI_write_cmos_sensor(0x5286,0x0c);
OV2685MIPI_write_cmos_sensor(0x5287,0x10);
OV2685MIPI_write_cmos_sensor(0x5300,0xc5);
OV2685MIPI_write_cmos_sensor(0x5301,0xa0);
OV2685MIPI_write_cmos_sensor(0x5302,0x06);
OV2685MIPI_write_cmos_sensor(0x5303,0x08);
OV2685MIPI_write_cmos_sensor(0x5304,0x18);
OV2685MIPI_write_cmos_sensor(0x5305,0x30);
OV2685MIPI_write_cmos_sensor(0x5306,0x60);
OV2685MIPI_write_cmos_sensor(0x5307,0xc0);
OV2685MIPI_write_cmos_sensor(0x5308,0x82);
OV2685MIPI_write_cmos_sensor(0x5309,0x00);
OV2685MIPI_write_cmos_sensor(0x530a,0x26);

OV2685MIPI_write_cmos_sensor(0x5309,0x3f);
OV2685MIPI_write_cmos_sensor(0x530a,0x3f);


OV2685MIPI_write_cmos_sensor(0x530b,0x02);
OV2685MIPI_write_cmos_sensor(0x530c,0x02);
OV2685MIPI_write_cmos_sensor(0x530d,0x00);
OV2685MIPI_write_cmos_sensor(0x530e,0x0c);
OV2685MIPI_write_cmos_sensor(0x530f,0x14);
OV2685MIPI_write_cmos_sensor(0x5310,0x1a);
OV2685MIPI_write_cmos_sensor(0x5311,0x20);
OV2685MIPI_write_cmos_sensor(0x5312,0x80);
OV2685MIPI_write_cmos_sensor(0x5313,0x4b); ///4b
OV2685MIPI_write_cmos_sensor(0x5380,0x01);
OV2685MIPI_write_cmos_sensor(0x5381,0x52);
OV2685MIPI_write_cmos_sensor(0x5382,0x00);
OV2685MIPI_write_cmos_sensor(0x5383,0x4a);
OV2685MIPI_write_cmos_sensor(0x5384,0x00);
OV2685MIPI_write_cmos_sensor(0x5385,0xb6);
OV2685MIPI_write_cmos_sensor(0x5386,0x00);
OV2685MIPI_write_cmos_sensor(0x5387,0x8d);
OV2685MIPI_write_cmos_sensor(0x5388,0x00);
OV2685MIPI_write_cmos_sensor(0x5389,0x3a);
OV2685MIPI_write_cmos_sensor(0x538a,0x00);
OV2685MIPI_write_cmos_sensor(0x538b,0xa6);
OV2685MIPI_write_cmos_sensor(0x538c,0x00);
OV2685MIPI_write_cmos_sensor(0x5400,0x0d);
OV2685MIPI_write_cmos_sensor(0x5401,0x18);
OV2685MIPI_write_cmos_sensor(0x5402,0x31);
OV2685MIPI_write_cmos_sensor(0x5403,0x5a);
OV2685MIPI_write_cmos_sensor(0x5404,0x65);
OV2685MIPI_write_cmos_sensor(0x5405,0x6f);
OV2685MIPI_write_cmos_sensor(0x5406,0x77);
OV2685MIPI_write_cmos_sensor(0x5407,0x80);
OV2685MIPI_write_cmos_sensor(0x5408,0x87);
OV2685MIPI_write_cmos_sensor(0x5409,0x8f);
OV2685MIPI_write_cmos_sensor(0x540a,0xa2);
OV2685MIPI_write_cmos_sensor(0x540b,0xb2);
OV2685MIPI_write_cmos_sensor(0x540c,0xcc);
OV2685MIPI_write_cmos_sensor(0x540d,0xe4);
OV2685MIPI_write_cmos_sensor(0x540e,0xf0);
OV2685MIPI_write_cmos_sensor(0x540f,0xa0);
OV2685MIPI_write_cmos_sensor(0x5410,0x6e);
OV2685MIPI_write_cmos_sensor(0x5411,0x06);
OV2685MIPI_write_cmos_sensor(0x5480,0x19);
OV2685MIPI_write_cmos_sensor(0x5481,0x00);
OV2685MIPI_write_cmos_sensor(0x5482,0x09);
OV2685MIPI_write_cmos_sensor(0x5483,0x12);
OV2685MIPI_write_cmos_sensor(0x5484,0x04);
OV2685MIPI_write_cmos_sensor(0x5485,0x06);
OV2685MIPI_write_cmos_sensor(0x5486,0x08);
OV2685MIPI_write_cmos_sensor(0x5487,0x0c);
OV2685MIPI_write_cmos_sensor(0x5488,0x10);
OV2685MIPI_write_cmos_sensor(0x5489,0x18);
OV2685MIPI_write_cmos_sensor(0x5500,0x02);
OV2685MIPI_write_cmos_sensor(0x5501,0x03);
OV2685MIPI_write_cmos_sensor(0x5502,0x04);
OV2685MIPI_write_cmos_sensor(0x5503,0x05);
OV2685MIPI_write_cmos_sensor(0x5504,0x06);
OV2685MIPI_write_cmos_sensor(0x5505,0x08);
OV2685MIPI_write_cmos_sensor(0x5506,0x00);
OV2685MIPI_write_cmos_sensor(0x5600,0x02);
OV2685MIPI_write_cmos_sensor(0x5603,0x40);
OV2685MIPI_write_cmos_sensor(0x5604,0x38);
OV2685MIPI_write_cmos_sensor(0x5609,0x20);
OV2685MIPI_write_cmos_sensor(0x560a,0x80);
OV2685MIPI_write_cmos_sensor(0x5800,0x03);
OV2685MIPI_write_cmos_sensor(0x5801,0x24);
OV2685MIPI_write_cmos_sensor(0x5802,0x02);
OV2685MIPI_write_cmos_sensor(0x5803,0x40);
OV2685MIPI_write_cmos_sensor(0x5804,0x34);
OV2685MIPI_write_cmos_sensor(0x5805,0x05);
OV2685MIPI_write_cmos_sensor(0x5806,0x12);
OV2685MIPI_write_cmos_sensor(0x5807,0x05);
OV2685MIPI_write_cmos_sensor(0x5808,0x03);
OV2685MIPI_write_cmos_sensor(0x5809,0x3c);
OV2685MIPI_write_cmos_sensor(0x580a,0x02);
OV2685MIPI_write_cmos_sensor(0x580b,0x40);
OV2685MIPI_write_cmos_sensor(0x580c,0x26);
OV2685MIPI_write_cmos_sensor(0x580d,0x05);
OV2685MIPI_write_cmos_sensor(0x580e,0x52);
OV2685MIPI_write_cmos_sensor(0x580f,0x06);
OV2685MIPI_write_cmos_sensor(0x5810,0x03);
OV2685MIPI_write_cmos_sensor(0x5811,0x28);
OV2685MIPI_write_cmos_sensor(0x5812,0x02);
OV2685MIPI_write_cmos_sensor(0x5813,0x40);
OV2685MIPI_write_cmos_sensor(0x5814,0x24);
OV2685MIPI_write_cmos_sensor(0x5815,0x05);
OV2685MIPI_write_cmos_sensor(0x5816,0x42);
OV2685MIPI_write_cmos_sensor(0x5817,0x06);
OV2685MIPI_write_cmos_sensor(0x5818,0x0d);
OV2685MIPI_write_cmos_sensor(0x5819,0x40);
OV2685MIPI_write_cmos_sensor(0x581a,0x04);
OV2685MIPI_write_cmos_sensor(0x581b,0x0c);
OV2685MIPI_write_cmos_sensor(0x3a03,0x4a);
OV2685MIPI_write_cmos_sensor(0x3a04,0x40);
OV2685MIPI_write_cmos_sensor(0x3503,0x00);

    OV2685MIPI_write_cmos_sensor(0x382a,0x08); 
	OV2685MIPI_write_cmos_sensor(0x3a00,0x43); 
	OV2685MIPI_write_cmos_sensor(0x3a0a,0x0a); 
	OV2685MIPI_write_cmos_sensor(0x3a0b,0x10); 
	OV2685MIPI_write_cmos_sensor(0x3a0c,0x0a); 
	OV2685MIPI_write_cmos_sensor(0x3a0d,0x10); 

	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIPreviewSetting_SVGA function:\n ");
}
/*************************************************************************
* FUNCTION
*     OV2685MIPIFullSizeCaptureSetting
*
* DESCRIPTION
*    This function config full size capture setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void OV2685MIPIFullSizeCaptureSetting(void)
{
	//OV2685MIPI 2592x1944,10fps
	//90Mhz, 360Mbps/Lane, 2Lane.15
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIFullSizeCaptureSetting function:\n ");
OV2685MIPI_write_cmos_sensor(0x0103,0x01);
OV2685MIPI_write_cmos_sensor(0x3002,0x00);
OV2685MIPI_write_cmos_sensor(0x3016,0x3c);// 1c
OV2685MIPI_write_cmos_sensor(0x3018,0x84);
OV2685MIPI_write_cmos_sensor(0x301d,0xf0);
OV2685MIPI_write_cmos_sensor(0x3020,0x00);
OV2685MIPI_write_cmos_sensor(0x3082,0x2c);
OV2685MIPI_write_cmos_sensor(0x3083,0x03);
OV2685MIPI_write_cmos_sensor(0x3084,0x07);
OV2685MIPI_write_cmos_sensor(0x3085,0x03);
OV2685MIPI_write_cmos_sensor(0x3086,0x01); //00
OV2685MIPI_write_cmos_sensor(0x3087,0x00);
OV2685MIPI_write_cmos_sensor(0x3501,0x4e);
OV2685MIPI_write_cmos_sensor(0x3502,0xe0);
OV2685MIPI_write_cmos_sensor(0x3503,0x03);
OV2685MIPI_write_cmos_sensor(0x350b,0x36);
OV2685MIPI_write_cmos_sensor(0x3600,0xb4);
OV2685MIPI_write_cmos_sensor(0x3603,0x39);
OV2685MIPI_write_cmos_sensor(0x3604,0x24);
OV2685MIPI_write_cmos_sensor(0x3605,0x00);
OV2685MIPI_write_cmos_sensor(0x3620,0x26);
OV2685MIPI_write_cmos_sensor(0x3621,0x37);
OV2685MIPI_write_cmos_sensor(0x3622,0x04);
OV2685MIPI_write_cmos_sensor(0x3628,0x10);
OV2685MIPI_write_cmos_sensor(0x3705,0x3c);
OV2685MIPI_write_cmos_sensor(0x370a,0x21);
OV2685MIPI_write_cmos_sensor(0x370c,0x50);
OV2685MIPI_write_cmos_sensor(0x370d,0xc0);
OV2685MIPI_write_cmos_sensor(0x3717,0x58);
OV2685MIPI_write_cmos_sensor(0x3718,0x88);
OV2685MIPI_write_cmos_sensor(0x3720,0x00);
OV2685MIPI_write_cmos_sensor(0x3721,0x00);
OV2685MIPI_write_cmos_sensor(0x3722,0x00);
OV2685MIPI_write_cmos_sensor(0x3723,0x00);
OV2685MIPI_write_cmos_sensor(0x3738,0x00);
OV2685MIPI_write_cmos_sensor(0x3781,0x80);
OV2685MIPI_write_cmos_sensor(0x3789,0x60);
OV2685MIPI_write_cmos_sensor(0x3800,0x00);
OV2685MIPI_write_cmos_sensor(0x3801,0x00);
OV2685MIPI_write_cmos_sensor(0x3802,0x00);
OV2685MIPI_write_cmos_sensor(0x3803,0x00);
OV2685MIPI_write_cmos_sensor(0x3804,0x06);
OV2685MIPI_write_cmos_sensor(0x3805,0x4f);
OV2685MIPI_write_cmos_sensor(0x3806,0x04);
OV2685MIPI_write_cmos_sensor(0x3807,0xbf);
OV2685MIPI_write_cmos_sensor(0x3808,0x06); //1600
OV2685MIPI_write_cmos_sensor(0x3809,0x40);
OV2685MIPI_write_cmos_sensor(0x380a,0x04); //1200
OV2685MIPI_write_cmos_sensor(0x380b,0xb0);
OV2685MIPI_write_cmos_sensor(0x380c,0x06); //1700
OV2685MIPI_write_cmos_sensor(0x380d,0xa4); 
OV2685MIPI_write_cmos_sensor(0x380e,0x05); //1294
OV2685MIPI_write_cmos_sensor(0x380f,0x0e);
OV2685MIPI_write_cmos_sensor(0x3810,0x00);
OV2685MIPI_write_cmos_sensor(0x3811,0x08); //00
OV2685MIPI_write_cmos_sensor(0x3812,0x00);
OV2685MIPI_write_cmos_sensor(0x3813,0x08); //00
OV2685MIPI_write_cmos_sensor(0x3814,0x11);
OV2685MIPI_write_cmos_sensor(0x3815,0x11);
OV2685MIPI_write_cmos_sensor(0x3819,0x04);
OV2685MIPI_write_cmos_sensor(0x3820,0xc0);//c4
OV2685MIPI_write_cmos_sensor(0x3821,0x00);//04
OV2685MIPI_write_cmos_sensor(0x4000,0x81);
OV2685MIPI_write_cmos_sensor(0x4001,0x40);
OV2685MIPI_write_cmos_sensor(0x4008,0x00);
OV2685MIPI_write_cmos_sensor(0x4009,0x0b);
OV2685MIPI_write_cmos_sensor(0x4300,0x30);
OV2685MIPI_write_cmos_sensor(0x430e,0x00);
OV2685MIPI_write_cmos_sensor(0x4602,0x02);
OV2685MIPI_write_cmos_sensor(0x4837,0x1e);
OV2685MIPI_write_cmos_sensor(0x5000,0xff);
OV2685MIPI_write_cmos_sensor(0x5001,0x05);
OV2685MIPI_write_cmos_sensor(0x5002,0x32);
OV2685MIPI_write_cmos_sensor(0x5003,0x04);
OV2685MIPI_write_cmos_sensor(0x5004,0xff);
OV2685MIPI_write_cmos_sensor(0x5005,0x12);
OV2685MIPI_write_cmos_sensor(0x0100,0x01);

OV2685MIPI_write_cmos_sensor(0x5180,0xf4);
OV2685MIPI_write_cmos_sensor(0x5181,0x11);
OV2685MIPI_write_cmos_sensor(0x5182,0x41);
OV2685MIPI_write_cmos_sensor(0x5183,0x42);
OV2685MIPI_write_cmos_sensor(0x5184,0x78);
OV2685MIPI_write_cmos_sensor(0x5185,0x58);
OV2685MIPI_write_cmos_sensor(0x5186,0xb5);
OV2685MIPI_write_cmos_sensor(0x5187,0xb2);
OV2685MIPI_write_cmos_sensor(0x5188,0x08);
OV2685MIPI_write_cmos_sensor(0x5189,0x0e);
OV2685MIPI_write_cmos_sensor(0x518a,0x0c);
OV2685MIPI_write_cmos_sensor(0x518b,0x4c);
OV2685MIPI_write_cmos_sensor(0x518c,0x38);
OV2685MIPI_write_cmos_sensor(0x518d,0xf8);
OV2685MIPI_write_cmos_sensor(0x518e,0x04);
OV2685MIPI_write_cmos_sensor(0x518f,0x7f);
OV2685MIPI_write_cmos_sensor(0x5190,0x40);
OV2685MIPI_write_cmos_sensor(0x5191,0x5f);
OV2685MIPI_write_cmos_sensor(0x5192,0x40);
OV2685MIPI_write_cmos_sensor(0x5193,0xff);
OV2685MIPI_write_cmos_sensor(0x5194,0x40);
OV2685MIPI_write_cmos_sensor(0x5195,0x07);
OV2685MIPI_write_cmos_sensor(0x5196,0x04);
OV2685MIPI_write_cmos_sensor(0x5197,0x04);
OV2685MIPI_write_cmos_sensor(0x5198,0x00);
OV2685MIPI_write_cmos_sensor(0x5199,0x05);
OV2685MIPI_write_cmos_sensor(0x519a,0xd2);
OV2685MIPI_write_cmos_sensor(0x519b,0x10);
OV2685MIPI_write_cmos_sensor(0x5200,0x09);
OV2685MIPI_write_cmos_sensor(0x5201,0x02);
OV2685MIPI_write_cmos_sensor(0x5202,0x06);
OV2685MIPI_write_cmos_sensor(0x5203,0x20);
OV2685MIPI_write_cmos_sensor(0x5204,0x41);
OV2685MIPI_write_cmos_sensor(0x5205,0x16);
OV2685MIPI_write_cmos_sensor(0x5206,0x00);
OV2685MIPI_write_cmos_sensor(0x5207,0x05);
OV2685MIPI_write_cmos_sensor(0x520b,0x30);
OV2685MIPI_write_cmos_sensor(0x520c,0x75);
OV2685MIPI_write_cmos_sensor(0x520d,0x00);
OV2685MIPI_write_cmos_sensor(0x520e,0x30);
OV2685MIPI_write_cmos_sensor(0x520f,0x75);
OV2685MIPI_write_cmos_sensor(0x5210,0x00);
OV2685MIPI_write_cmos_sensor(0x5280,0x14);
OV2685MIPI_write_cmos_sensor(0x5281,0x02);
OV2685MIPI_write_cmos_sensor(0x5282,0x02);
OV2685MIPI_write_cmos_sensor(0x5283,0x04);
OV2685MIPI_write_cmos_sensor(0x5284,0x06);
OV2685MIPI_write_cmos_sensor(0x5285,0x08);
OV2685MIPI_write_cmos_sensor(0x5286,0x0c);
OV2685MIPI_write_cmos_sensor(0x5287,0x10);
OV2685MIPI_write_cmos_sensor(0x5300,0xc5);
OV2685MIPI_write_cmos_sensor(0x5301,0xa0);
OV2685MIPI_write_cmos_sensor(0x5302,0x06);
OV2685MIPI_write_cmos_sensor(0x5303,0x08);
OV2685MIPI_write_cmos_sensor(0x5304,0x18);
OV2685MIPI_write_cmos_sensor(0x5305,0x30);
OV2685MIPI_write_cmos_sensor(0x5306,0x60);
OV2685MIPI_write_cmos_sensor(0x5307,0xc0);
OV2685MIPI_write_cmos_sensor(0x5308,0x82);
OV2685MIPI_write_cmos_sensor(0x5309,0x00);
OV2685MIPI_write_cmos_sensor(0x530a,0x26);
OV2685MIPI_write_cmos_sensor(0x530b,0x02);
OV2685MIPI_write_cmos_sensor(0x530c,0x02);
OV2685MIPI_write_cmos_sensor(0x530d,0x00);
OV2685MIPI_write_cmos_sensor(0x530e,0x0c);
OV2685MIPI_write_cmos_sensor(0x530f,0x14);
OV2685MIPI_write_cmos_sensor(0x5310,0x1a);
OV2685MIPI_write_cmos_sensor(0x5311,0x20);
OV2685MIPI_write_cmos_sensor(0x5312,0x80);
OV2685MIPI_write_cmos_sensor(0x5313,0x4b);
OV2685MIPI_write_cmos_sensor(0x5380,0x01);
OV2685MIPI_write_cmos_sensor(0x5381,0x52);
OV2685MIPI_write_cmos_sensor(0x5382,0x00);
OV2685MIPI_write_cmos_sensor(0x5383,0x4a);
OV2685MIPI_write_cmos_sensor(0x5384,0x00);
OV2685MIPI_write_cmos_sensor(0x5385,0xb6);
OV2685MIPI_write_cmos_sensor(0x5386,0x00);
OV2685MIPI_write_cmos_sensor(0x5387,0x8d);
OV2685MIPI_write_cmos_sensor(0x5388,0x00);
OV2685MIPI_write_cmos_sensor(0x5389,0x3a);
OV2685MIPI_write_cmos_sensor(0x538a,0x00);
OV2685MIPI_write_cmos_sensor(0x538b,0xa6);
OV2685MIPI_write_cmos_sensor(0x538c,0x00);
OV2685MIPI_write_cmos_sensor(0x5400,0x0d);
OV2685MIPI_write_cmos_sensor(0x5401,0x18);
OV2685MIPI_write_cmos_sensor(0x5402,0x31);
OV2685MIPI_write_cmos_sensor(0x5403,0x5a);
OV2685MIPI_write_cmos_sensor(0x5404,0x65);
OV2685MIPI_write_cmos_sensor(0x5405,0x6f);
OV2685MIPI_write_cmos_sensor(0x5406,0x77);
OV2685MIPI_write_cmos_sensor(0x5407,0x80);
OV2685MIPI_write_cmos_sensor(0x5408,0x87);
OV2685MIPI_write_cmos_sensor(0x5409,0x8f);
OV2685MIPI_write_cmos_sensor(0x540a,0xa2);
OV2685MIPI_write_cmos_sensor(0x540b,0xb2);
OV2685MIPI_write_cmos_sensor(0x540c,0xcc);
OV2685MIPI_write_cmos_sensor(0x540d,0xe4);
OV2685MIPI_write_cmos_sensor(0x540e,0xf0);
OV2685MIPI_write_cmos_sensor(0x540f,0xa0);
OV2685MIPI_write_cmos_sensor(0x5410,0x6e);
OV2685MIPI_write_cmos_sensor(0x5411,0x06);
OV2685MIPI_write_cmos_sensor(0x5480,0x19);
OV2685MIPI_write_cmos_sensor(0x5481,0x00);
OV2685MIPI_write_cmos_sensor(0x5482,0x09);
OV2685MIPI_write_cmos_sensor(0x5483,0x12);
OV2685MIPI_write_cmos_sensor(0x5484,0x04);
OV2685MIPI_write_cmos_sensor(0x5485,0x06);
OV2685MIPI_write_cmos_sensor(0x5486,0x08);
OV2685MIPI_write_cmos_sensor(0x5487,0x0c);
OV2685MIPI_write_cmos_sensor(0x5488,0x10);
OV2685MIPI_write_cmos_sensor(0x5489,0x18);
OV2685MIPI_write_cmos_sensor(0x5500,0x02);
OV2685MIPI_write_cmos_sensor(0x5501,0x03);
OV2685MIPI_write_cmos_sensor(0x5502,0x04);
OV2685MIPI_write_cmos_sensor(0x5503,0x05);
OV2685MIPI_write_cmos_sensor(0x5504,0x06);
OV2685MIPI_write_cmos_sensor(0x5505,0x08);
OV2685MIPI_write_cmos_sensor(0x5506,0x00);
OV2685MIPI_write_cmos_sensor(0x5600,0x02);
OV2685MIPI_write_cmos_sensor(0x5603,0x40);
OV2685MIPI_write_cmos_sensor(0x5604,0x38);
OV2685MIPI_write_cmos_sensor(0x5609,0x20);
OV2685MIPI_write_cmos_sensor(0x560a,0x80);
OV2685MIPI_write_cmos_sensor(0x5800,0x03);
OV2685MIPI_write_cmos_sensor(0x5801,0x24);
OV2685MIPI_write_cmos_sensor(0x5802,0x02);
OV2685MIPI_write_cmos_sensor(0x5803,0x40);
OV2685MIPI_write_cmos_sensor(0x5804,0x34);
OV2685MIPI_write_cmos_sensor(0x5805,0x05);
OV2685MIPI_write_cmos_sensor(0x5806,0x12);
OV2685MIPI_write_cmos_sensor(0x5807,0x05);
OV2685MIPI_write_cmos_sensor(0x5808,0x03);
OV2685MIPI_write_cmos_sensor(0x5809,0x3c);
OV2685MIPI_write_cmos_sensor(0x580a,0x02);
OV2685MIPI_write_cmos_sensor(0x580b,0x40);
OV2685MIPI_write_cmos_sensor(0x580c,0x26);
OV2685MIPI_write_cmos_sensor(0x580d,0x05);
OV2685MIPI_write_cmos_sensor(0x580e,0x52);
OV2685MIPI_write_cmos_sensor(0x580f,0x06);
OV2685MIPI_write_cmos_sensor(0x5810,0x03);
OV2685MIPI_write_cmos_sensor(0x5811,0x28);
OV2685MIPI_write_cmos_sensor(0x5812,0x02);
OV2685MIPI_write_cmos_sensor(0x5813,0x40);
OV2685MIPI_write_cmos_sensor(0x5814,0x24);
OV2685MIPI_write_cmos_sensor(0x5815,0x05);
OV2685MIPI_write_cmos_sensor(0x5816,0x42);
OV2685MIPI_write_cmos_sensor(0x5817,0x06);
OV2685MIPI_write_cmos_sensor(0x5818,0x0d);
OV2685MIPI_write_cmos_sensor(0x5819,0x40);
OV2685MIPI_write_cmos_sensor(0x581a,0x04);
OV2685MIPI_write_cmos_sensor(0x581b,0x0c);
OV2685MIPI_write_cmos_sensor(0x3a03,0x4a);
OV2685MIPI_write_cmos_sensor(0x3a04,0x40);
OV2685MIPI_write_cmos_sensor(0x3503,0x00);
	spin_lock(&ov2685mipi_drv_lock);
	OV2685MIPISensor.IsPVmode = KAL_FALSE;
	OV2685MIPISensor.CapturePclk= 900;	
	spin_unlock(&ov2685mipi_drv_lock);
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIFullSizeCaptureSetting function:\n ");
}
/*************************************************************************
* FUNCTION
*     OV2685MIPIFullSizeZSDSetting
*
* DESCRIPTION
*    This function config full size capture setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void OV2685MIPIFullSizeZSDSetting(void)
{
    OV2685MIPI_write_cmos_sensor(0x0103,0x01);
    OV2685MIPI_write_cmos_sensor(0x3002,0x00);
    OV2685MIPI_write_cmos_sensor(0x3016,0x3c);// 1c
    OV2685MIPI_write_cmos_sensor(0x3018,0x84);
    OV2685MIPI_write_cmos_sensor(0x301d,0xf0);
    OV2685MIPI_write_cmos_sensor(0x3020,0x00);
    OV2685MIPI_write_cmos_sensor(0x3082,0x2c);
    OV2685MIPI_write_cmos_sensor(0x3083,0x03);
    OV2685MIPI_write_cmos_sensor(0x3084,0x07);
    OV2685MIPI_write_cmos_sensor(0x3085,0x03);
    OV2685MIPI_write_cmos_sensor(0x3086,0x01); //00
    OV2685MIPI_write_cmos_sensor(0x3087,0x00);
    OV2685MIPI_write_cmos_sensor(0x3501,0x4e);
    OV2685MIPI_write_cmos_sensor(0x3502,0xe0);
    OV2685MIPI_write_cmos_sensor(0x3503,0x03);
    OV2685MIPI_write_cmos_sensor(0x350b,0x36);
    OV2685MIPI_write_cmos_sensor(0x3600,0xb4);
    OV2685MIPI_write_cmos_sensor(0x3603,0x39);
    OV2685MIPI_write_cmos_sensor(0x3604,0x24);
    OV2685MIPI_write_cmos_sensor(0x3605,0x00);
    OV2685MIPI_write_cmos_sensor(0x3620,0x26);
    OV2685MIPI_write_cmos_sensor(0x3621,0x37);
    OV2685MIPI_write_cmos_sensor(0x3622,0x04);
    OV2685MIPI_write_cmos_sensor(0x3628,0x10);
    OV2685MIPI_write_cmos_sensor(0x3705,0x3c);
    OV2685MIPI_write_cmos_sensor(0x370a,0x21);
    OV2685MIPI_write_cmos_sensor(0x370c,0x50);
    OV2685MIPI_write_cmos_sensor(0x370d,0xc0);
    OV2685MIPI_write_cmos_sensor(0x3717,0x58);
    OV2685MIPI_write_cmos_sensor(0x3718,0x88);
    OV2685MIPI_write_cmos_sensor(0x3720,0x00);
    OV2685MIPI_write_cmos_sensor(0x3721,0x00);
    OV2685MIPI_write_cmos_sensor(0x3722,0x00);
    OV2685MIPI_write_cmos_sensor(0x3723,0x00);
    OV2685MIPI_write_cmos_sensor(0x3738,0x00);
    OV2685MIPI_write_cmos_sensor(0x3781,0x80);
    OV2685MIPI_write_cmos_sensor(0x3789,0x60);
    OV2685MIPI_write_cmos_sensor(0x3800,0x00);
    OV2685MIPI_write_cmos_sensor(0x3801,0x00);
    OV2685MIPI_write_cmos_sensor(0x3802,0x00);
    OV2685MIPI_write_cmos_sensor(0x3803,0x00);
    OV2685MIPI_write_cmos_sensor(0x3804,0x06);
    OV2685MIPI_write_cmos_sensor(0x3805,0x4f);
    OV2685MIPI_write_cmos_sensor(0x3806,0x04);
    OV2685MIPI_write_cmos_sensor(0x3807,0xbf);
    OV2685MIPI_write_cmos_sensor(0x3808,0x06); //1600
    OV2685MIPI_write_cmos_sensor(0x3809,0x40);
    OV2685MIPI_write_cmos_sensor(0x380a,0x04); //1200
    OV2685MIPI_write_cmos_sensor(0x380b,0xb0);
    OV2685MIPI_write_cmos_sensor(0x380c,0x06); //1700
    OV2685MIPI_write_cmos_sensor(0x380d,0xa4); 
    OV2685MIPI_write_cmos_sensor(0x380e,0x0a); //1294 //50e
    OV2685MIPI_write_cmos_sensor(0x380f,0x0e);
    OV2685MIPI_write_cmos_sensor(0x3810,0x00);
    OV2685MIPI_write_cmos_sensor(0x3811,0x08); //00
    OV2685MIPI_write_cmos_sensor(0x3812,0x00);
    OV2685MIPI_write_cmos_sensor(0x3813,0x08); //00
    OV2685MIPI_write_cmos_sensor(0x3814,0x11);
    OV2685MIPI_write_cmos_sensor(0x3815,0x11);
    OV2685MIPI_write_cmos_sensor(0x3819,0x04);
    OV2685MIPI_write_cmos_sensor(0x3820,0xc0);//c4
    OV2685MIPI_write_cmos_sensor(0x3821,0x00);//04
    OV2685MIPI_write_cmos_sensor(0x4000,0x81);
    OV2685MIPI_write_cmos_sensor(0x4001,0x40);
    OV2685MIPI_write_cmos_sensor(0x4008,0x00);
    OV2685MIPI_write_cmos_sensor(0x4009,0x0b);
    OV2685MIPI_write_cmos_sensor(0x4300,0x30);
    OV2685MIPI_write_cmos_sensor(0x430e,0x00);
    OV2685MIPI_write_cmos_sensor(0x4602,0x02);
    OV2685MIPI_write_cmos_sensor(0x4837,0x1e);
    OV2685MIPI_write_cmos_sensor(0x5000,0xff);
    OV2685MIPI_write_cmos_sensor(0x5001,0x05);
    OV2685MIPI_write_cmos_sensor(0x5002,0x32);
    OV2685MIPI_write_cmos_sensor(0x5003,0x04);
    OV2685MIPI_write_cmos_sensor(0x5004,0xff);
    OV2685MIPI_write_cmos_sensor(0x5005,0x12);
    OV2685MIPI_write_cmos_sensor(0x0100,0x01);
    
    OV2685MIPI_write_cmos_sensor(0x5180,0xf4);
    OV2685MIPI_write_cmos_sensor(0x5181,0x11);
    OV2685MIPI_write_cmos_sensor(0x5182,0x41);
    OV2685MIPI_write_cmos_sensor(0x5183,0x42);
    OV2685MIPI_write_cmos_sensor(0x5184,0x78);
    OV2685MIPI_write_cmos_sensor(0x5185,0x58);
    OV2685MIPI_write_cmos_sensor(0x5186,0xb5);
    OV2685MIPI_write_cmos_sensor(0x5187,0xb2);
    OV2685MIPI_write_cmos_sensor(0x5188,0x08);
    OV2685MIPI_write_cmos_sensor(0x5189,0x0e);
    OV2685MIPI_write_cmos_sensor(0x518a,0x0c);
    OV2685MIPI_write_cmos_sensor(0x518b,0x4c);
    OV2685MIPI_write_cmos_sensor(0x518c,0x38);
    OV2685MIPI_write_cmos_sensor(0x518d,0xf8);
    OV2685MIPI_write_cmos_sensor(0x518e,0x04);
    OV2685MIPI_write_cmos_sensor(0x518f,0x7f);
    OV2685MIPI_write_cmos_sensor(0x5190,0x40);
    OV2685MIPI_write_cmos_sensor(0x5191,0x5f);
    OV2685MIPI_write_cmos_sensor(0x5192,0x40);
    OV2685MIPI_write_cmos_sensor(0x5193,0xff);
    OV2685MIPI_write_cmos_sensor(0x5194,0x40);
    OV2685MIPI_write_cmos_sensor(0x5195,0x07);
    OV2685MIPI_write_cmos_sensor(0x5196,0x04);
    OV2685MIPI_write_cmos_sensor(0x5197,0x04);
    OV2685MIPI_write_cmos_sensor(0x5198,0x00);
    OV2685MIPI_write_cmos_sensor(0x5199,0x05);
    OV2685MIPI_write_cmos_sensor(0x519a,0xd2);
    OV2685MIPI_write_cmos_sensor(0x519b,0x10);
    OV2685MIPI_write_cmos_sensor(0x5200,0x09);
    OV2685MIPI_write_cmos_sensor(0x5201,0x02);
    OV2685MIPI_write_cmos_sensor(0x5202,0x06);
    OV2685MIPI_write_cmos_sensor(0x5203,0x20);
    OV2685MIPI_write_cmos_sensor(0x5204,0x41);
    OV2685MIPI_write_cmos_sensor(0x5205,0x16);
    OV2685MIPI_write_cmos_sensor(0x5206,0x00);
    OV2685MIPI_write_cmos_sensor(0x5207,0x05);
    OV2685MIPI_write_cmos_sensor(0x520b,0x30);
    OV2685MIPI_write_cmos_sensor(0x520c,0x75);
    OV2685MIPI_write_cmos_sensor(0x520d,0x00);
    OV2685MIPI_write_cmos_sensor(0x520e,0x30);
    OV2685MIPI_write_cmos_sensor(0x520f,0x75);
    OV2685MIPI_write_cmos_sensor(0x5210,0x00);
    OV2685MIPI_write_cmos_sensor(0x5280,0x14);
    OV2685MIPI_write_cmos_sensor(0x5281,0x02);
    OV2685MIPI_write_cmos_sensor(0x5282,0x02);
    OV2685MIPI_write_cmos_sensor(0x5283,0x04);
    OV2685MIPI_write_cmos_sensor(0x5284,0x06);
    OV2685MIPI_write_cmos_sensor(0x5285,0x08);
    OV2685MIPI_write_cmos_sensor(0x5286,0x0c);
    OV2685MIPI_write_cmos_sensor(0x5287,0x10);
    OV2685MIPI_write_cmos_sensor(0x5300,0xc5);
    OV2685MIPI_write_cmos_sensor(0x5301,0xa0);
    
    //OV2685MIPI_write_cmos_sensor(0x5302,0x06);
    //OV2685MIPI_write_cmos_sensor(0x5303,0x08);
    //OV2685MIPI_write_cmos_sensor(0x5304,0x18);
    //OV2685MIPI_write_cmos_sensor(0x5305,0x30);
    //OV2685MIPI_write_cmos_sensor(0x5306,0x60);
    //OV2685MIPI_write_cmos_sensor(0x5307,0xc0);

    OV2685MIPI_write_cmos_sensor(0x5302,0x80);
    OV2685MIPI_write_cmos_sensor(0x5303,0x80);
    OV2685MIPI_write_cmos_sensor(0x5304,0x80);
    OV2685MIPI_write_cmos_sensor(0x5305,0x80);
    OV2685MIPI_write_cmos_sensor(0x5306,0x80);
    OV2685MIPI_write_cmos_sensor(0x5307,0x80);

    
    OV2685MIPI_write_cmos_sensor(0x5308,0x82);
    OV2685MIPI_write_cmos_sensor(0x5309,0x00);
    OV2685MIPI_write_cmos_sensor(0x530a,0x26);
    OV2685MIPI_write_cmos_sensor(0x530b,0x02);
    OV2685MIPI_write_cmos_sensor(0x530c,0x02);
    OV2685MIPI_write_cmos_sensor(0x530d,0x00);
    OV2685MIPI_write_cmos_sensor(0x530e,0x0c);
    OV2685MIPI_write_cmos_sensor(0x530f,0x14);
    OV2685MIPI_write_cmos_sensor(0x5310,0x1a);
    OV2685MIPI_write_cmos_sensor(0x5311,0x20);
    OV2685MIPI_write_cmos_sensor(0x5312,0x80);
    OV2685MIPI_write_cmos_sensor(0x5313,0x4b);
    OV2685MIPI_write_cmos_sensor(0x5380,0x01);
    OV2685MIPI_write_cmos_sensor(0x5381,0x52);
    OV2685MIPI_write_cmos_sensor(0x5382,0x00);
    OV2685MIPI_write_cmos_sensor(0x5383,0x4a);
    OV2685MIPI_write_cmos_sensor(0x5384,0x00);
    OV2685MIPI_write_cmos_sensor(0x5385,0xb6);
    OV2685MIPI_write_cmos_sensor(0x5386,0x00);
    OV2685MIPI_write_cmos_sensor(0x5387,0x8d);
    OV2685MIPI_write_cmos_sensor(0x5388,0x00);
    OV2685MIPI_write_cmos_sensor(0x5389,0x3a);
    OV2685MIPI_write_cmos_sensor(0x538a,0x00);
    OV2685MIPI_write_cmos_sensor(0x538b,0xa6);
    OV2685MIPI_write_cmos_sensor(0x538c,0x00);
    OV2685MIPI_write_cmos_sensor(0x5400,0x0d);
    OV2685MIPI_write_cmos_sensor(0x5401,0x18);
    OV2685MIPI_write_cmos_sensor(0x5402,0x31);
    OV2685MIPI_write_cmos_sensor(0x5403,0x5a);
    OV2685MIPI_write_cmos_sensor(0x5404,0x65);
    OV2685MIPI_write_cmos_sensor(0x5405,0x6f);
    OV2685MIPI_write_cmos_sensor(0x5406,0x77);
    OV2685MIPI_write_cmos_sensor(0x5407,0x80);
    OV2685MIPI_write_cmos_sensor(0x5408,0x87);
    OV2685MIPI_write_cmos_sensor(0x5409,0x8f);
    OV2685MIPI_write_cmos_sensor(0x540a,0xa2);
    OV2685MIPI_write_cmos_sensor(0x540b,0xb2);
    OV2685MIPI_write_cmos_sensor(0x540c,0xcc);
    OV2685MIPI_write_cmos_sensor(0x540d,0xe4);
    OV2685MIPI_write_cmos_sensor(0x540e,0xf0);
    OV2685MIPI_write_cmos_sensor(0x540f,0xa0);
    OV2685MIPI_write_cmos_sensor(0x5410,0x6e);
    OV2685MIPI_write_cmos_sensor(0x5411,0x06);
    OV2685MIPI_write_cmos_sensor(0x5480,0x19);
    OV2685MIPI_write_cmos_sensor(0x5481,0x00);
    OV2685MIPI_write_cmos_sensor(0x5482,0x09);
    OV2685MIPI_write_cmos_sensor(0x5483,0x12);
    OV2685MIPI_write_cmos_sensor(0x5484,0x04);
    OV2685MIPI_write_cmos_sensor(0x5485,0x06);
    OV2685MIPI_write_cmos_sensor(0x5486,0x08);
    OV2685MIPI_write_cmos_sensor(0x5487,0x0c);
    OV2685MIPI_write_cmos_sensor(0x5488,0x10);
    OV2685MIPI_write_cmos_sensor(0x5489,0x18);
    
    //OV2685MIPI_write_cmos_sensor(0x5500,0x02);
    //OV2685MIPI_write_cmos_sensor(0x5501,0x03);
    //OV2685MIPI_write_cmos_sensor(0x5502,0x04);
    //OV2685MIPI_write_cmos_sensor(0x5503,0x05);
    //OV2685MIPI_write_cmos_sensor(0x5504,0x06);
    //OV2685MIPI_write_cmos_sensor(0x5505,0x08);
    //OV2685MIPI_write_cmos_sensor(0x5506,0x00);

    OV2685MIPI_write_cmos_sensor(0x5500,0x60);
    OV2685MIPI_write_cmos_sensor(0x5501,0x60);
    OV2685MIPI_write_cmos_sensor(0x5502,0x60);
    OV2685MIPI_write_cmos_sensor(0x5503,0x60);
    OV2685MIPI_write_cmos_sensor(0x5504,0x60);
    OV2685MIPI_write_cmos_sensor(0x5505,0x60);
    OV2685MIPI_write_cmos_sensor(0x5506,0x60);

    
    OV2685MIPI_write_cmos_sensor(0x5600,0x02);
    OV2685MIPI_write_cmos_sensor(0x5603,0x40);
    OV2685MIPI_write_cmos_sensor(0x5604,0x38);
    OV2685MIPI_write_cmos_sensor(0x5609,0x20);
    OV2685MIPI_write_cmos_sensor(0x560a,0x80);
    OV2685MIPI_write_cmos_sensor(0x5800,0x03);
    OV2685MIPI_write_cmos_sensor(0x5801,0x24);
    OV2685MIPI_write_cmos_sensor(0x5802,0x02);
    OV2685MIPI_write_cmos_sensor(0x5803,0x40);
    OV2685MIPI_write_cmos_sensor(0x5804,0x34);
    OV2685MIPI_write_cmos_sensor(0x5805,0x05);
    OV2685MIPI_write_cmos_sensor(0x5806,0x12);
    OV2685MIPI_write_cmos_sensor(0x5807,0x05);
    OV2685MIPI_write_cmos_sensor(0x5808,0x03);
    OV2685MIPI_write_cmos_sensor(0x5809,0x3c);
    OV2685MIPI_write_cmos_sensor(0x580a,0x02);
    OV2685MIPI_write_cmos_sensor(0x580b,0x40);
    OV2685MIPI_write_cmos_sensor(0x580c,0x26);
    OV2685MIPI_write_cmos_sensor(0x580d,0x05);
    OV2685MIPI_write_cmos_sensor(0x580e,0x52);
    OV2685MIPI_write_cmos_sensor(0x580f,0x06);
    OV2685MIPI_write_cmos_sensor(0x5810,0x03);
    OV2685MIPI_write_cmos_sensor(0x5811,0x28);
    OV2685MIPI_write_cmos_sensor(0x5812,0x02);
    OV2685MIPI_write_cmos_sensor(0x5813,0x40);
    OV2685MIPI_write_cmos_sensor(0x5814,0x24);
    OV2685MIPI_write_cmos_sensor(0x5815,0x05);
    OV2685MIPI_write_cmos_sensor(0x5816,0x42);
    OV2685MIPI_write_cmos_sensor(0x5817,0x06);
    OV2685MIPI_write_cmos_sensor(0x5818,0x0d);
    OV2685MIPI_write_cmos_sensor(0x5819,0x40);
    OV2685MIPI_write_cmos_sensor(0x581a,0x04);
    OV2685MIPI_write_cmos_sensor(0x581b,0x0c);
    OV2685MIPI_write_cmos_sensor(0x3a03,0x4a);
    OV2685MIPI_write_cmos_sensor(0x3a04,0x40);
    OV2685MIPI_write_cmos_sensor(0x3503,0x00);


	OV2685MIPI_write_cmos_sensor(0x3a00,0x43); 
    OV2685MIPI_write_cmos_sensor(0x382a,0x08); 
	OV2685MIPI_write_cmos_sensor(0x3a0a,0x0a); 
	OV2685MIPI_write_cmos_sensor(0x3a0b,0x10); 
	OV2685MIPI_write_cmos_sensor(0x3a0c,0x0a); 
	OV2685MIPI_write_cmos_sensor(0x3a0d,0x10); 

spin_lock(&ov2685mipi_drv_lock);
OV2685MIPISensor.IsPVmode = KAL_FALSE;
OV2685MIPISensor.CapturePclk= 900;  
spin_unlock(&ov2685mipi_drv_lock);

}

static UINT32 OV2685read_vcm_adc()  
{

}

static UINT32 OV2685set_vcm_adc(adcvalue) 
{

}

static void OV2685_FOCUS_OVT_AFC_Init(void)
{
 
}


static void OV2685_FOCUS_OVT_AFC_Constant_Focus(void)
{

}   

static void OV2685_FOCUS_OVT_AFC_Single_Focus()
{

}

static void OV2685_FOCUS_OVT_AFC_Pause_Focus()
{

}
static void OV2685_FOCUS_Get_AF_Max_Num_Focus_Areas(UINT32 *pFeatureReturnPara32)
{ 	  

}

static void OV2685_FOCUS_Get_AE_Max_Num_Metering_Areas(UINT32 *pFeatureReturnPara32)
{ 	

}
static void OV2685_FOCUS_OVT_AFC_Touch_AF(UINT32 x,UINT32 y)
{

}

static void OV2685_FOCUS_Set_AF_Window(UINT32 zone_addr)
{

}

static void OV2685_FOCUS_Get_AF_Macro(UINT32 *pFeatureReturnPara32)
{

}
static void OV2685_FOCUS_Get_AF_Inf(UINT32 * pFeatureReturnPara32)
{

}

                       
static void OV2685_FOCUS_OVT_AFC_Get_AF_Status(UINT32 *pFeatureReturnPara32)
{

}

/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Cancel_Focus
* DESCRIPTION
*   cancel af 
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/     
static void OV2685_FOCUS_OVT_AFC_Cancel_Focus()
{

}

/*************************************************************************
* FUNCTION
*   OV2685WBcalibattion
* DESCRIPTION
*   color calibration
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void OV2685WBcalibattion(kal_uint32 color_r_gain,kal_uint32 color_b_gain)
{
		kal_uint32 color_r_gain_w = 0;
		kal_uint32 color_b_gain_w = 0;
		OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685WBcalibattion function:\n ");
		kal_uint8 temp = OV2685MIPIYUV_read_cmos_sensor(0x350b); 
		
		if(temp>=0xb0)
		{	
			color_r_gain_w=color_r_gain*97/100;																																														
			color_b_gain_w=color_b_gain*99/100;  
		}
		else if (temp>=0x70)
		{
			color_r_gain_w=color_r_gain *97/100;																																														
			color_b_gain_w=color_b_gain*99/100;
		}
		else if (temp>=0x30)
		{
			color_r_gain_w=color_r_gain*97/100;																																														
			color_b_gain_w=color_b_gain*99/100;
		}
		else
		{
			color_r_gain_w=color_r_gain*97/100;																																														
			color_b_gain_w=color_b_gain*99/100; 
		}																																																																						
		OV2685MIPI_write_cmos_sensor(0x5195,(color_r_gain_w & 0xff00)>>8);																																														
		OV2685MIPI_write_cmos_sensor(0x5196,color_r_gain_w & 0xff); 			
		OV2685MIPI_write_cmos_sensor(0x5199,(color_b_gain_w & 0xff00)>>8);																																														
		OV2685MIPI_write_cmos_sensor(0x519a,color_b_gain_w & 0xff); 
		OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685WBcalibattion function:\n ");
}	
/*************************************************************************
* FUNCTION
*	OV2685MIPIOpen
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
UINT32 OV2685MIPIOpen(void)
{
	volatile signed int i;
	kal_uint16 sensor_id = 0;
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIOpen function:\n ");
	//OV2685MIPI_write_cmos_sensor(0x3103,0x11);
	OV2685MIPI_write_cmos_sensor(0x0103,0x01);
    msleep(5);
	for(i=0;i<3;i++)
	{
		sensor_id = (OV2685MIPIYUV_read_cmos_sensor(0x300A) << 8) | OV2685MIPIYUV_read_cmos_sensor(0x300B);
		OV2685MIPISENSORDB("OV2685MIPI READ ID :%x",sensor_id);
		if(sensor_id != OV2685MIPI_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	OV2685MIPIinitalvariable();
	OV2685MIPIInitialSetting();
	//OV2685_FOCUS_OVT_AFC_Init();
	OV2685MIPI_set_AWB_mode(KAL_TRUE);
	if(false == AF_Power)
	{
		OV2685MIPISENSORDB("[OV2685Sensor] AF Power on.\n");
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,"OV2685_AF"))
		{
			printk("[CAMERA SENSOR AF] Fail to enable analog power\n");
			return -EIO;
		}  
		AF_Power = true;
	}
	else
	{
		OV2685MIPISENSORDB("[OV2685Sensor] AF Power has already on.\n");
	}
	mdelay(8);	
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIOpen function:\n ");
	return ERROR_NONE;
}	/* OV2685MIPIOpen() */

/*************************************************************************
* FUNCTION
*	OV2685MIPIClose
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
UINT32 OV2685MIPIClose(void)
{
	flash_mode = 2;	//set default flash-mode as video-mode-flashlight 2
	OV2685MIPI_write_cmos_sensor(0x3023,0x01);
	OV2685MIPI_write_cmos_sensor(0x3022,0x06); 
 	kal_uint16 lastadc, num, i, now, step = 100;
    lastadc = OV2685read_vcm_adc();
 	num = lastadc/step;
 	for(i = 1; i <= num; i++)
  	{
  		OV2685set_vcm_adc(lastadc-i*step);
		mdelay(30);
	}

	OV2685set_vcm_adc(0);
	OV2685MIPI_write_cmos_sensor(0x3023,0x01);
	OV2685MIPI_write_cmos_sensor(0x3022,0x08); 

	if(true == AF_Power)
	{
		OV2685MIPISENSORDB("[OV2685Sensor] AF Power down.\n");
	    if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,"OV2685_AF"))
	    {
		 	printk("[CAMERA SENSOR AF] Fail to enable analog power\n");
			return -EIO;
		}
		AF_Power = false;
	}
	else
	{
		OV2685MIPISENSORDB("[OV2685Sensor] AF Power is already off.\n");
	}
	return ERROR_NONE;
}	/* OV2685MIPIClose() */
/*************************************************************************
* FUNCTION
*	OV2685MIPIPreview
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
UINT32 OV2685MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	//OV2685MIPIPreviewSetting_SVGA();
    OV2685MIPIFullSizeZSDSetting();
	msleep(100);	
	OV2685MIPI_set_AE_mode(KAL_TRUE);
	OV2685MIPI_set_AWB_mode(KAL_TRUE);
	msleep(20);
	//OV2685_FOCUS_OVT_AFC_Constant_Focus();
	spin_lock(&ov2685mipi_drv_lock);
	OV2685MIPISensor.SensorMode= SENSOR_MODE_PREVIEW;
	OV2685MIPISensor.IsPVmode = KAL_TRUE;
	OV2685MIPISensor.PreviewPclk= 560;	
	OV2685MIPISensor.zsd_flag=0;
	spin_unlock(&ov2685mipi_drv_lock);
	return ERROR_NONE ;
}	/* OV2685MIPIPreview() */
/*************************************************************************
* FUNCTION
*	OV2685MIPIZSDPreview
*
* DESCRIPTION
*	This function start the sensor ZSD preview.
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
UINT32 OV2685MIPIZSDPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	OV2685MIPIFullSizeZSDSetting();
	msleep(50);
	OV2685MIPI_set_AE_mode(KAL_TRUE);
	OV2685MIPI_set_AWB_mode(KAL_TRUE);
	msleep(20);
	//OV2685_FOCUS_OVT_AFC_Constant_Focus();		
	spin_lock(&ov2685mipi_drv_lock);
	OV2685MIPISensor.zsd_flag=1;
	OV2685MIPISensor.SensorMode= SENSOR_MODE_PREVIEW;
	spin_unlock(&ov2685mipi_drv_lock);
	return ERROR_NONE ;
}	/* OV2685MIPIPreview() */
/*************************************************************************
* FUNCTION
*	OV2685MIPICapture
*
* DESCRIPTION
*	This function start the sensor OV2685MIPICapture
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

UINT32 OV2685MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint32 shutter = 0;    
    kal_uint32 extshutter = 0;
    kal_uint32 color_r_gain = 0;
    kal_uint32 color_b_gain = 0;
    kal_uint32 readgain=0;
    OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPICapture function:\n ");
    if(SENSOR_MODE_PREVIEW == OV2685MIPISensor.SensorMode )
    {        
    shutter=OV2685MIPIReadShutter();
    //extshutter=OV2685MIPIReadExtraShutter();
    readgain=OV2685MIPIReadSensorGain();
    spin_lock(&ov2685mipi_drv_lock);
    OV2685MIPISensor.PreviewShutter=shutter;
    //OV2685MIPISensor.PreviewExtraShutter=extshutter;    
    OV2685MIPISensor.SensorGain=readgain;
    spin_unlock(&ov2685mipi_drv_lock);    
    OV2685MIPI_set_AE_mode(KAL_FALSE);
    OV2685MIPI_set_AWB_mode(KAL_FALSE);
    color_r_gain=((OV2685MIPIYUV_read_cmos_sensor(0x5196)&0xFF)+((OV2685MIPIYUV_read_cmos_sensor(0x5195)&0xFF)*256));  
    color_b_gain=((OV2685MIPIYUV_read_cmos_sensor(0x519a)&0xFF)+((OV2685MIPIYUV_read_cmos_sensor(0x5199)&0xFF)*256)); 
    OV2685MIPIFullSizeCaptureSetting();
    spin_lock(&ov2685mipi_drv_lock);    
    OV2685MIPISensor.SensorMode= SENSOR_MODE_CAPTURE;
    spin_unlock(&ov2685mipi_drv_lock);
    OV2685WBcalibattion(color_r_gain,color_b_gain);      
    OV2685MIPISENSORDB("[OV2685MIPI]Before shutter=%d:\n",shutter);
    if(OV2685MIPISensor.zsd_flag==0)
    {
        shutter = shutter*2;
    }
    if (SCENE_MODE_HDR == OV2685MIPISensor.sceneMode)
    {
        spin_lock(&ov2685mipi_drv_lock);
        OV2685MIPISensor.currentExposureTime=shutter;
        //OV2685MIPISensor.currentextshutter=extshutter;
        OV2685MIPISensor.currentAxDGain=readgain;
        spin_unlock(&ov2685mipi_drv_lock);
    }
    else
    {
        OV2685MIPIWriteSensorGain(OV2685MIPISensor.SensorGain);    
        OV2685MIPIWriteShutter(shutter);
    }
    OV2685MIPISENSORDB("[OV2685MIPI]after shutter=%d:\n",shutter);
    OV2685MIPISENSORDB("[OV2685MIPI]after shutter=%d:\n",OV2685MIPISensor.SensorGain);
    msleep(20);
	//mDELAY(200);
    }
    OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPICapture function:\n ");
    return ERROR_NONE; 
}/* OV2685MIPICapture() */

BOOL OV2685MIPI_set_param_exposure_for_HDR(UINT16 para)
{
    kal_uint32 totalGain = 0, exposureTime = 0;
	
	OV2685MIPISENSORDB("[OV2685MIPI]OV2685MIPI_set_param_exposure_for_HDRenter para=%d,manualAEStart%d\n",para,OV2685MIPISensor.manualAEStart);
    if (0 == OV2685MIPISensor.manualAEStart)
    {       
        OV2685MIPI_set_AE_mode(KAL_FALSE);//Manual AE enable
        spin_lock(&ov2685mipi_drv_lock);	
        OV2685MIPISensor.manualAEStart = 1;
		spin_unlock(&ov2685mipi_drv_lock);
    }
	totalGain = OV2685MIPISensor.currentAxDGain;
    exposureTime = OV2685MIPISensor.currentExposureTime;
	switch (para)
	{
	   case AE_EV_COMP_20:	//+2 EV
       case AE_EV_COMP_10:	// +1 EV
		   totalGain = totalGain<<1;
           exposureTime = exposureTime<<1;
           OV2685MIPISENSORDB("[4EC] HDR AE_EV_COMP_20\n");
		 break;
	   case AE_EV_COMP_00:	// +0 EV
           OV2685MIPISENSORDB("[4EC] HDR AE_EV_COMP_00\n");
		 break;
	   case AE_EV_COMP_n10:  // -1 EV
	   case AE_EV_COMP_n20:  // -2 EV
		   totalGain = totalGain >> 1;
           exposureTime = exposureTime >> 1;
           OV2685MIPISENSORDB("[4EC] HDR AE_EV_COMP_n20\n");
		 break;
	   default:
		 break;//return FALSE;
	}
	totalGain = (totalGain > OV2685MIPI_MAX_AXD_GAIN) ? OV2685MIPI_MAX_AXD_GAIN : totalGain;
    exposureTime = (exposureTime > OV2685MIPI_MAX_EXPOSURE_TIME) ? OV2685MIPI_MAX_EXPOSURE_TIME : exposureTime;
    OV2685MIPIWriteSensorGain(totalGain);	
	OV2685MIPIWriteShutter(exposureTime);	
//	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_set_param_exposure_for_HDR function:\n ");
	return TRUE;
}
UINT32 OV2685MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIGetResolution function:\n ");
	pSensorResolution->SensorPreviewWidth=  OV2685MIPI_IMAGE_SENSOR_SVGA_WIDTH-2*OV2685MIPI_PV_GRAB_START_X;
	pSensorResolution->SensorPreviewHeight= OV2685MIPI_IMAGE_SENSOR_SVGA_HEIGHT-2*OV2685MIPI_PV_GRAB_START_Y;
	pSensorResolution->SensorFullWidth= OV2685MIPI_IMAGE_SENSOR_QSXGA_WITDH-2*OV2685MIPI_FULL_GRAB_START_X; 
	pSensorResolution->SensorFullHeight= OV2685MIPI_IMAGE_SENSOR_QSXGA_HEIGHT-2*OV2685MIPI_FULL_GRAB_START_Y;
	pSensorResolution->SensorVideoWidth= OV2685MIPI_IMAGE_SENSOR_SVGA_WIDTH-2*OV2685MIPI_PV_GRAB_START_X; 
	pSensorResolution->SensorVideoHeight= OV2685MIPI_IMAGE_SENSOR_SVGA_HEIGHT-2*OV2685MIPI_PV_GRAB_START_Y;;
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIGetResolution function:\n ");
	return ERROR_NONE;
}	/* OV2685MIPIGetResolution() */

UINT32 OV2685MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,MSDK_SENSOR_INFO_STRUCT *pSensorInfo,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIGetInfo function:\n ");
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorPreviewResolutionX=OV2685MIPI_IMAGE_SENSOR_QSXGA_WITDH-2*OV2685MIPI_FULL_GRAB_START_X;//OV2685MIPI_IMAGE_SENSOR_QSXGA_WITDH ;
			pSensorInfo->SensorPreviewResolutionY=OV2685MIPI_IMAGE_SENSOR_QSXGA_HEIGHT-2*OV2685MIPI_FULL_GRAB_START_Y;//OV2685MIPI_IMAGE_SENSOR_QSXGA_HEIGHT ;
			pSensorInfo->SensorCameraPreviewFrameRate=15;
			break;
		default:
			pSensorInfo->SensorPreviewResolutionX=OV2685MIPI_IMAGE_SENSOR_SVGA_WIDTH-2*OV2685MIPI_PV_GRAB_START_X; ;
			pSensorInfo->SensorPreviewResolutionY=OV2685MIPI_IMAGE_SENSOR_SVGA_HEIGHT-2*OV2685MIPI_PV_GRAB_START_Y;
			pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
	}		 		
	pSensorInfo->SensorFullResolutionX= OV2685MIPI_IMAGE_SENSOR_QSXGA_WITDH-2*OV2685MIPI_FULL_GRAB_START_X;
	pSensorInfo->SensorFullResolutionY= OV2685MIPI_IMAGE_SENSOR_QSXGA_HEIGHT-2*OV2685MIPI_FULL_GRAB_START_Y;
	//pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=5;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=4;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;  
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 2;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
	pSensorInfo->CaptureDelayFrame = 2; //2
	pSensorInfo->PreviewDelayFrame = 5; //3
	pSensorInfo->VideoDelayFrame = 3; 		
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->YUVAwbDelayFrame = 3;
	pSensorInfo->YUVEffectDelayFrame= 3; 
	pSensorInfo->AEShutDelayFrame= 0;
 	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;   		
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = OV2685MIPI_PV_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = OV2685MIPI_PV_GRAB_START_Y;   
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;  	
			pSensorInfo->SensorPacketECCOrder = 1;		
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = OV2685MIPI_FULL_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = OV2685MIPI_FULL_GRAB_START_Y;             
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount =14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;
			pSensorInfo->SensorPacketECCOrder = 1;
			break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=5;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = OV2685MIPI_PV_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = OV2685MIPI_PV_GRAB_START_Y; 			
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;
			pSensorInfo->SensorHightSampling = 0;	
			pSensorInfo->SensorPacketECCOrder = 1;
		  break;
	}
	memcpy(pSensorConfigData, &OV2685MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));	
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIGetInfo function:\n ");	
	return ERROR_NONE;
}	/* OV2685MIPIGetInfo() */

UINT32 OV2685MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	  OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIControl function:\n ");
	  spin_lock(&ov2685mipi_drv_lock);
	  CurrentScenarioId = ScenarioId;
	  spin_unlock(&ov2685mipi_drv_lock);
	  switch (ScenarioId)
	  {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 OV2685MIPIPreview(pImageWindow, pSensorConfigData);
			 Iszsd = false;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 //OV2685MIPIZSDPreview(pImageWindow, pSensorConfigData);
			 Iszsd = true;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			 OV2685MIPICapture(pImageWindow, pSensorConfigData);
	  	     break;
		default:
			return ERROR_INVALID_SCENARIO_ID;
	}
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIControl function:\n ");
	return ERROR_NONE;
}	
BOOL OV2685MIPI_set_param_wb(UINT16 para)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_set_param_wb function:\n ");
	spin_lock(&ov2685mipi_drv_lock);
    OV2685MIPISensor.awbMode = para;
    spin_unlock(&ov2685mipi_drv_lock);
	switch (para)
    {
        case AWB_MODE_OFF:
							spin_lock(&ov2685mipi_drv_lock);
							OV2685MIPI_AWB_ENABLE = KAL_FALSE; 
							spin_unlock(&ov2685mipi_drv_lock);
							OV2685MIPI_set_AWB_mode(OV2685MIPI_AWB_ENABLE);
							break;                    
        case AWB_MODE_AUTO:
							spin_lock(&ov2685mipi_drv_lock);
							OV2685MIPI_AWB_ENABLE = KAL_TRUE; 
							spin_unlock(&ov2685mipi_drv_lock);
							OV2685MIPI_set_AWB_mode(OV2685MIPI_AWB_ENABLE);
							break;
        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
        					OV2685MIPI_write_cmos_sensor(0x3208,0x00);
							OV2685MIPI_set_AWB_mode(KAL_FALSE);         	                
							OV2685MIPI_write_cmos_sensor(0x5195,0x07); 
							OV2685MIPI_write_cmos_sensor(0x5196,0xdc); 
							OV2685MIPI_write_cmos_sensor(0x5197,0x04); 
							OV2685MIPI_write_cmos_sensor(0x5198,0x00); 
							OV2685MIPI_write_cmos_sensor(0x5199,0x05); //04
							OV2685MIPI_write_cmos_sensor(0x519a,0xd3); //30 
							OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
        	    			OV2685MIPI_write_cmos_sensor(0x3208,0xa0);                                                               
              				break;
        case AWB_MODE_DAYLIGHT: //sunny
        	    			OV2685MIPI_write_cmos_sensor(0x3208,0x00); 
							OV2685MIPI_set_AWB_mode(KAL_FALSE);                           
							OV2685MIPI_write_cmos_sensor(0x5195,0x07); 
							OV2685MIPI_write_cmos_sensor(0x5196,0x9c); 
							OV2685MIPI_write_cmos_sensor(0x5197,0x04); 
							OV2685MIPI_write_cmos_sensor(0x5198,0x00); 
							OV2685MIPI_write_cmos_sensor(0x5199,0x05); 
							OV2685MIPI_write_cmos_sensor(0x519a,0xf3);                           
							OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
        	    			OV2685MIPI_write_cmos_sensor(0x3208,0xa0);                        
							break;
        case AWB_MODE_INCANDESCENT: //office baichid
        	    			OV2685MIPI_write_cmos_sensor(0x3208,0x00); 
							OV2685MIPI_set_AWB_mode(KAL_FALSE);                           
							OV2685MIPI_write_cmos_sensor(0x5195,0x06); 
							OV2685MIPI_write_cmos_sensor(0x5196,0xb8); //e0
							OV2685MIPI_write_cmos_sensor(0x5197,0x04); 
							OV2685MIPI_write_cmos_sensor(0x5198,0x00); 
							OV2685MIPI_write_cmos_sensor(0x5199,0x06); //05a0
							OV2685MIPI_write_cmos_sensor(0x519a,0x5f);
							OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
        	    			OV2685MIPI_write_cmos_sensor(0x3208,0xa0);                        
							break; 
		case AWB_MODE_TUNGSTEN:
			        		OV2685MIPI_write_cmos_sensor(0x3208,0x00); 
							OV2685MIPI_set_AWB_mode(KAL_FALSE);                         
							OV2685MIPI_write_cmos_sensor(0x5195,0x04); //548
							OV2685MIPI_write_cmos_sensor(0x5196,0x90); 
							OV2685MIPI_write_cmos_sensor(0x5197,0x04); 
							OV2685MIPI_write_cmos_sensor(0x5198,0x00); 
							OV2685MIPI_write_cmos_sensor(0x5199,0x09); //05
							OV2685MIPI_write_cmos_sensor(0x519a,0x20); 
							OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
        	    			OV2685MIPI_write_cmos_sensor(0x3208,0xa0);                        
							break;
        case AWB_MODE_FLUORESCENT:
        	    			OV2685MIPI_write_cmos_sensor(0x3208,0x00); 
							OV2685MIPI_set_AWB_mode(KAL_FALSE);                           
							OV2685MIPI_write_cmos_sensor(0x5195,0x04); //548
							OV2685MIPI_write_cmos_sensor(0x5196,0x90); 
							OV2685MIPI_write_cmos_sensor(0x5197,0x04); 
							OV2685MIPI_write_cmos_sensor(0x5198,0x00); 
							OV2685MIPI_write_cmos_sensor(0x5199,0x09); //05
							OV2685MIPI_write_cmos_sensor(0x519a,0xa0);                       
							OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
        	    			OV2685MIPI_write_cmos_sensor(0x3208,0xa0);                        
							break;
        default:
			break;
    }
	spin_lock(&ov2685mipi_drv_lock);
    OV2685MIPISensor.iWB = para;
    spin_unlock(&ov2685mipi_drv_lock);
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_set_param_wb function:\n ");
       return TRUE;
} /* OV2685MIPI_set_param_wb */
void OV2685MIPI_set_contrast(UINT16 para)
{   
    OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_set_contrast function:\n ");
    switch (para)
    {
        case ISP_CONTRAST_LOW:
             OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			 OV2685MIPI_write_cmos_sensor(0x5002,0x33);
			 OV2685MIPI_write_cmos_sensor(0x5606,0x14);
			 OV2685MIPI_write_cmos_sensor(0x5605,0x14);
			 OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			 OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
             break;
        case ISP_CONTRAST_HIGH:
             OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			 OV2685MIPI_write_cmos_sensor(0x5586,0x2c);
			 OV2685MIPI_write_cmos_sensor(0x5585,0x1c);
			 OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			 OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
             break;
        case ISP_CONTRAST_MIDDLE:
			 OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			 OV2685MIPI_write_cmos_sensor(0x5586,0x20);
			 OV2685MIPI_write_cmos_sensor(0x5585,0x00);
			 OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			 OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
			 break;
        default:
             break;
    }
    OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_set_contrast function:\n ");
    return;
}

void OV2685MIPI_set_brightness(UINT16 para)
{
    OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_set_brightness function:\n ");
    switch (para)
    {
        case ISP_BRIGHT_LOW:
             OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			 OV2685MIPI_write_cmos_sensor(0x5607,0x30);
			 OV2685MIPI_write_cmos_sensor(0x5608,0x08);
			 OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			 OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
             break;
        case ISP_BRIGHT_HIGH:
             OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			 OV2685MIPI_write_cmos_sensor(0x5587,0x30);
			 OV2685MIPI_write_cmos_sensor(0x5588,0x00);
			 OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			 OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
             break;
        case ISP_BRIGHT_MIDDLE:
			 OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			 OV2685MIPI_write_cmos_sensor(0x5587,0x00);
			 OV2685MIPI_write_cmos_sensor(0x5588,0x00);
			 OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			 OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
			 break;
        default:
             return KAL_FALSE;
             break;
    }
    OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_set_brightness function:\n ");
    return;
}
void OV2685MIPI_set_saturation(UINT16 para)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_set_saturation function:\n ");
    switch (para)
    {
        case ISP_SAT_HIGH:
			OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			
			OV2685MIPI_write_cmos_sensor(0x5380,0x01);
			OV2685MIPI_write_cmos_sensor(0x5381,0xb1);//ccm
			OV2685MIPI_write_cmos_sensor(0x5382,0x00);//
			OV2685MIPI_write_cmos_sensor(0x5383,0x23);//
			OV2685MIPI_write_cmos_sensor(0x5384,0x00 );//
			OV2685MIPI_write_cmos_sensor(0x5385,0x98);//
			OV2685MIPI_write_cmos_sensor(0x5386,0x00);//
			OV2685MIPI_write_cmos_sensor(0x5387,0x92);//
			OV2685MIPI_write_cmos_sensor(0x5388,0x00);//
			OV2685MIPI_write_cmos_sensor(0x5389,0x48);//    
			OV2685MIPI_write_cmos_sensor(0x538a,0x01);//      
			OV2685MIPI_write_cmos_sensor(0x538b,0xee);// 
			OV2685MIPI_write_cmos_sensor(0x538c,0x10);
			     			                                           
			OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
             break;
        case ISP_SAT_LOW:
			OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			
			OV2685MIPI_write_cmos_sensor(0x5380,0x01);
			OV2685MIPI_write_cmos_sensor(0x5381,0x55);//ccm
			OV2685MIPI_write_cmos_sensor(0x5382,0x00);//
			OV2685MIPI_write_cmos_sensor(0x5383,0x1b);//
			OV2685MIPI_write_cmos_sensor(0x5384,0x00 );//
			OV2685MIPI_write_cmos_sensor(0x5385,0x78);//
			OV2685MIPI_write_cmos_sensor(0x5386,0x00);//
			OV2685MIPI_write_cmos_sensor(0x5387,0x72);//
			OV2685MIPI_write_cmos_sensor(0x5388,0x00);//
			OV2685MIPI_write_cmos_sensor(0x5389,0x38);//    
			OV2685MIPI_write_cmos_sensor(0x538a,0x01);//      
			OV2685MIPI_write_cmos_sensor(0x538b,0x84);// 
			OV2685MIPI_write_cmos_sensor(0x538c,0x10);
			     			                                           
			OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
             break;
        case ISP_SAT_MIDDLE:
			OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			
			OV2685MIPI_write_cmos_sensor(0x5380,0x01);
			OV2685MIPI_write_cmos_sensor(0x5381,0x83);//ccm
			OV2685MIPI_write_cmos_sensor(0x5382,0x00);//
			OV2685MIPI_write_cmos_sensor(0x5383,0x1f);//
			OV2685MIPI_write_cmos_sensor(0x5384,0x00 );//
			OV2685MIPI_write_cmos_sensor(0x5385,0x88);//
			OV2685MIPI_write_cmos_sensor(0x5386,0x00);//
			OV2685MIPI_write_cmos_sensor(0x5387,0x82);//
			OV2685MIPI_write_cmos_sensor(0x5388,0x00);//
			OV2685MIPI_write_cmos_sensor(0x5389,0x40);//    
			OV2685MIPI_write_cmos_sensor(0x538a,0x01);//      
			OV2685MIPI_write_cmos_sensor(0x538b,0xb9);// 
			OV2685MIPI_write_cmos_sensor(0x538c,0x10);
			     			                                           
			OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
			 break;
        default:
			return KAL_FALSE;
			 break;
    }
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_set_saturation function:\n ");
     return;
}
void OV2685MIPI_scene_mode_PORTRAIT()
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_scene_mode_PORTRAIT function:\n ");
	OV2685MIPI_write_cmos_sensor(0x3208,0x00);  
	/*FRAME rate*/
	OV2685MIPI_write_cmos_sensor(0x3a00,0x43);//night mode enable, band enable//fps30-10
	OV2685MIPI_write_cmos_sensor(0x3a02,0x90);//50Hz
	OV2685MIPI_write_cmos_sensor(0x3a03,0x4e);//AEC target H
	OV2685MIPI_write_cmos_sensor(0x3a04,0x40);//AEC target L
	OV2685MIPI_write_cmos_sensor(0x3a06,0x00);//B50 H
	OV2685MIPI_write_cmos_sensor(0x3a07,0xc1);//B50 L
	OV2685MIPI_write_cmos_sensor(0x3a08,0x00);//B60 H
	OV2685MIPI_write_cmos_sensor(0x3a09,0xa1);//B60 L
	OV2685MIPI_write_cmos_sensor(0x3a0a,0x07);//max exp 50 H
	OV2685MIPI_write_cmos_sensor(0x3a0b,0x8a);//max exp 50 L
	OV2685MIPI_write_cmos_sensor(0x3a0c,0x07);//max exp 60 H
	OV2685MIPI_write_cmos_sensor(0x3a0d,0x8c);//max exp 60 L
	OV2685MIPI_write_cmos_sensor(0x3a0e,0x02);//VTS band 50 H
	OV2685MIPI_write_cmos_sensor(0x3a0f,0x43);//VTS band 50 L
	OV2685MIPI_write_cmos_sensor(0x3a10,0x02);//VTS band 60 H
	OV2685MIPI_write_cmos_sensor(0x3a11,0x84);//VTS band 60 L
	OV2685MIPI_write_cmos_sensor(0x3a13,0x80);//gain ceiling = 8x                                      			 			 	 		 		 				 			 			 		 			 
	OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
	OV2685MIPI_write_cmos_sensor(0x3208,0xa0); 
}
void OV2685MIPI_scene_mode_LANDSCAPE()
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_scene_mode_LANDSCAPE function:\n ");
	OV2685MIPI_write_cmos_sensor(0x3208,0x00);  
	/*FRAME rate*/
	OV2685MIPI_write_cmos_sensor(0x3a00,0x43);//night mode enable, band enable
	OV2685MIPI_write_cmos_sensor(0x3a02,0x90);//50Hz
	OV2685MIPI_write_cmos_sensor(0x3a03,0x4e);//AEC target H
	OV2685MIPI_write_cmos_sensor(0x3a04,0x40);//AEC target L
	OV2685MIPI_write_cmos_sensor(0x3a06,0x00);//B50 H
	OV2685MIPI_write_cmos_sensor(0x3a07,0xc1);//B50 L
	OV2685MIPI_write_cmos_sensor(0x3a08,0x00);//B60 H
	OV2685MIPI_write_cmos_sensor(0x3a09,0xa1);//B60 L
	OV2685MIPI_write_cmos_sensor(0x3a0a,0x07);//max exp 50 H
	OV2685MIPI_write_cmos_sensor(0x3a0b,0x8a);//max exp 50 L
	OV2685MIPI_write_cmos_sensor(0x3a0c,0x07);//max exp 60 H
	OV2685MIPI_write_cmos_sensor(0x3a0d,0x8c);//max exp 60 L
	OV2685MIPI_write_cmos_sensor(0x3a0e,0x02);//VTS band 50 H
	OV2685MIPI_write_cmos_sensor(0x3a0f,0x43);//VTS band 50 L
	OV2685MIPI_write_cmos_sensor(0x3a10,0x02);//VTS band 60 H
	OV2685MIPI_write_cmos_sensor(0x3a11,0x84);//VTS band 60 L
	OV2685MIPI_write_cmos_sensor(0x3a13,0x80);//gain ceiling = 8x                                      			 			 	 		 		 				 			 			 		 			 
	OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
	OV2685MIPI_write_cmos_sensor(0x3208,0xa0); 
}
void OV2685MIPI_scene_mode_SUNSET()
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_scene_mode_SUNSET function:\n ");
	OV2685MIPI_write_cmos_sensor(0x3208,0x00);  
	/*FRAME rate*/
	OV2685MIPI_write_cmos_sensor(0x3a00,0x43);//night mode enable, band enable
	OV2685MIPI_write_cmos_sensor(0x3a02,0x90);//50Hz
	OV2685MIPI_write_cmos_sensor(0x3a03,0x4e);//AEC target H
	OV2685MIPI_write_cmos_sensor(0x3a04,0x40);//AEC target L
	OV2685MIPI_write_cmos_sensor(0x3a06,0x00);//B50 H
	OV2685MIPI_write_cmos_sensor(0x3a07,0xc1);//B50 L
	OV2685MIPI_write_cmos_sensor(0x3a08,0x00);//B60 H
	OV2685MIPI_write_cmos_sensor(0x3a09,0xa1);//B60 L
	OV2685MIPI_write_cmos_sensor(0x3a0a,0x07);//max exp 50 H
	OV2685MIPI_write_cmos_sensor(0x3a0b,0x8a);//max exp 50 L
	OV2685MIPI_write_cmos_sensor(0x3a0c,0x07);//max exp 60 H
	OV2685MIPI_write_cmos_sensor(0x3a0d,0x8c);//max exp 60 L
	OV2685MIPI_write_cmos_sensor(0x3a0e,0x02);//VTS band 50 H
	OV2685MIPI_write_cmos_sensor(0x3a0f,0x43);//VTS band 50 L
	OV2685MIPI_write_cmos_sensor(0x3a10,0x02);//VTS band 60 H
	OV2685MIPI_write_cmos_sensor(0x3a11,0x84);//VTS band 60 L
	OV2685MIPI_write_cmos_sensor(0x3a13,0x80);//gain ceiling = 8x                                      			 			 	 		 		 				 			 			 		 			 
	OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
	OV2685MIPI_write_cmos_sensor(0x3208,0xa0); 
}
void OV2685MIPI_scene_mode_SPORTS()
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_scene_mode_SPORTS function:\n ");
	OV2685MIPI_write_cmos_sensor(0x3208,0x00);  
	/*FRAME rate*/
	OV2685MIPI_write_cmos_sensor(0x3a00,0x41);//night mode enable, band enable
	OV2685MIPI_write_cmos_sensor(0x3a02,0x90);//50Hz
	OV2685MIPI_write_cmos_sensor(0x3a03,0x4e);//AEC target H
	OV2685MIPI_write_cmos_sensor(0x3a04,0x40);//AEC target L
	OV2685MIPI_write_cmos_sensor(0x3a06,0x00);//B50 H
	OV2685MIPI_write_cmos_sensor(0x3a07,0xc1);//B50 L
	OV2685MIPI_write_cmos_sensor(0x3a08,0x00);//B60 H
	OV2685MIPI_write_cmos_sensor(0x3a09,0xa1);//B60 L
	OV2685MIPI_write_cmos_sensor(0x3a0a,0x07);//max exp 50 H
	OV2685MIPI_write_cmos_sensor(0x3a0b,0x8a);//max exp 50 L
	OV2685MIPI_write_cmos_sensor(0x3a0c,0x07);//max exp 60 H
	OV2685MIPI_write_cmos_sensor(0x3a0d,0x8c);//max exp 60 L
	OV2685MIPI_write_cmos_sensor(0x3a0e,0x02);//VTS band 50 H
	OV2685MIPI_write_cmos_sensor(0x3a0f,0x43);//VTS band 50 L
	OV2685MIPI_write_cmos_sensor(0x3a10,0x02);//VTS band 60 H
	OV2685MIPI_write_cmos_sensor(0x3a11,0x84);//VTS band 60 L
	OV2685MIPI_write_cmos_sensor(0x3a13,0x80);//gain ceiling = 8x                                      			 			 	 		 		 				 			 			 		 			 
	OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
	OV2685MIPI_write_cmos_sensor(0x3208,0xa0); 
}
void OV2685MIPI_scene_mode_OFF()
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_scene_mode_OFF function:\n ");
	OV2685MIPI_write_cmos_sensor(0x3a00,0x43); 
    OV2685MIPI_write_cmos_sensor(0x382a,0x08); 
	OV2685MIPI_write_cmos_sensor(0x3a0a,0x0a); 
	OV2685MIPI_write_cmos_sensor(0x3a0b,0x10); 
	OV2685MIPI_write_cmos_sensor(0x3a0c,0x0a); 
	OV2685MIPI_write_cmos_sensor(0x3a0d,0x10); 
}    
void OV2685MIPI_scene_mode_NIGHT()
{
	OV2685MIPI_write_cmos_sensor(0x3208,0x00);  
	/*FRAME rate*/
	OV2685MIPI_write_cmos_sensor(0x3a00,0x43);//night mode enable, band enable
	OV2685MIPI_write_cmos_sensor(0x3a02,0x90);//50Hz
	OV2685MIPI_write_cmos_sensor(0x3a03,0x4e);//AEC target H
	OV2685MIPI_write_cmos_sensor(0x3a04,0x40);//AEC target L
	OV2685MIPI_write_cmos_sensor(0x3a06,0x00);//B50 H
	OV2685MIPI_write_cmos_sensor(0x3a07,0xc1);//B50 L
	OV2685MIPI_write_cmos_sensor(0x3a08,0x00);//B60 H
	OV2685MIPI_write_cmos_sensor(0x3a09,0xa1);//B60 L
	OV2685MIPI_write_cmos_sensor(0x3a0a,0x0c);//max exp 50 H
	OV2685MIPI_write_cmos_sensor(0x3a0b,0x10);//max exp 50 L
	OV2685MIPI_write_cmos_sensor(0x3a0c,0x0a);//max exp 60 H
	OV2685MIPI_write_cmos_sensor(0x3a0d,0x0d);//max exp 60 L
	OV2685MIPI_write_cmos_sensor(0x3a0e,0x02);//VTS band 50 H
	OV2685MIPI_write_cmos_sensor(0x3a0f,0x43);//VTS band 50 L
	OV2685MIPI_write_cmos_sensor(0x3a10,0x02);//VTS band 60 H
	OV2685MIPI_write_cmos_sensor(0x3a11,0x84);//VTS band 60 L
	OV2685MIPI_write_cmos_sensor(0x3a13,0x80);//gain ceiling = 8x                                      			 			 	 		 		 				 			 			 		 			 
	OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
	OV2685MIPI_write_cmos_sensor(0x3208,0xa0); 
}
void OV2685MIPI_set_scene_mode(UINT16 para)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_set_scene_mode function:\n ");	
	OV2685MIPISENSORDB("[OV2685MIPI] OV2685MIPI_set_scene_mode=%d",para);	

	spin_lock(&ov2685mipi_drv_lock);
	OV2685MIPISensor.sceneMode=para;
	spin_unlock(&ov2685mipi_drv_lock);
	
    switch (para)
    { 

		case SCENE_MODE_NIGHTSCENE:
		     OV2685MIPI_scene_mode_NIGHT();	
			break;
        case SCENE_MODE_PORTRAIT:
			 OV2685MIPI_scene_mode_PORTRAIT();		 
             break;
        case SCENE_MODE_LANDSCAPE:
			 OV2685MIPI_scene_mode_LANDSCAPE();		 
             break;
        case SCENE_MODE_SUNSET:
			 OV2685MIPI_scene_mode_SUNSET();		 
            break;
        case SCENE_MODE_SPORTS:
            OV2685MIPI_scene_mode_SPORTS();		 
            break;
        case SCENE_MODE_HDR:
            if (1 == OV2685MIPISensor.manualAEStart)
            {
                OV2685MIPI_set_AE_mode(KAL_TRUE);//Manual AE disable
                spin_lock(&ov2685mipi_drv_lock);
            	OV2685MIPISensor.manualAEStart = 0;
                OV2685MIPISensor.currentExposureTime = 0;
                OV2685MIPISensor.currentAxDGain = 0;
				spin_unlock(&ov2685mipi_drv_lock);
            }
            break;
        case SCENE_MODE_OFF:
			OV2685MIPI_scene_mode_OFF();
			break;
        default:
            break;
    }
	msleep(100);
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_set_scene_mode function:\n ");
	return;
}

void OV2685MIPI_set_iso(UINT16 para)
{
    spin_lock(&ov2685mipi_drv_lock);
    OV2685MIPISensor.isoSpeed = para;
    spin_unlock(&ov2685mipi_drv_lock);   
    switch (para)
    {
        case AE_ISO_100:
             OV2685MIPI_write_cmos_sensor(0x3a13, 0x80);
             break;
        case AE_ISO_200:
             OV2685MIPI_write_cmos_sensor(0x3a13, 0xc0);
             break;
        case AE_ISO_400:
             OV2685MIPI_write_cmos_sensor(0x3a13, 0xf0);
             break;
        default:
             break;
    }
    return;
}

BOOL OV2685MIPI_set_param_effect(UINT16 para)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_set_param_effect function:\n ");
	switch (para)
    {
				case MEFFECT_OFF:    
				OV2685MIPI_write_cmos_sensor(0x3208,0x00);              
				OV2685MIPI_write_cmos_sensor(0x5600,0x06); 
				OV2685MIPI_write_cmos_sensor(0x5603,0x40); 
				OV2685MIPI_write_cmos_sensor(0x5604,0x28);
				OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
				OV2685MIPI_write_cmos_sensor(0x3208,0xa0); 
				break;
				case MEFFECT_SEPIA:  
				OV2685MIPI_write_cmos_sensor(0x3208,0x00);                    
				OV2685MIPI_write_cmos_sensor(0x5600,0x1c);
				OV2685MIPI_write_cmos_sensor(0x5603,0x40); 
				OV2685MIPI_write_cmos_sensor(0x5604,0xa0); 
				OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
				OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
				break;
				case MEFFECT_NEGATIVE:
				OV2685MIPI_write_cmos_sensor(0x3208,0x00);
				OV2685MIPI_write_cmos_sensor(0x5600,0x46); 
				OV2685MIPI_write_cmos_sensor(0x5603,0x40); 
				OV2685MIPI_write_cmos_sensor(0x5604,0x28); 	
				OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
				OV2685MIPI_write_cmos_sensor(0x3208,0xa0);									                                                            
				break;
				case MEFFECT_SEPIAGREEN:    
				OV2685MIPI_write_cmos_sensor(0x3208,0x00);                  			 
				OV2685MIPI_write_cmos_sensor(0x5600,0x1c);			 
				OV2685MIPI_write_cmos_sensor(0x5603,0x60); 
				OV2685MIPI_write_cmos_sensor(0x5604,0x60);   
				OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
				OV2685MIPI_write_cmos_sensor(0x3208,0xa0);                   
				break;
				case MEFFECT_SEPIABLUE:
				OV2685MIPI_write_cmos_sensor(0x3208,0x00);
				OV2685MIPI_write_cmos_sensor(0x5600,0x1c);             
				OV2685MIPI_write_cmos_sensor(0x5603,0xa0); //a0 
				OV2685MIPI_write_cmos_sensor(0x5604,0x40); 
				OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
				OV2685MIPI_write_cmos_sensor(0x3208,0xa0);                     
				break;
				case MEFFECT_MONO: //B&W
				OV2685MIPI_write_cmos_sensor(0x3208,0x00);
				OV2685MIPI_write_cmos_sensor(0x5600,0x1c);      		
				OV2685MIPI_write_cmos_sensor(0x5603,0x80); 
				OV2685MIPI_write_cmos_sensor(0x5604,0x80); 
				OV2685MIPI_write_cmos_sensor(0x3208,0x10); 
				OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
				break;
        default:
         break;
    }    
	msleep(100);
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_set_param_effect function:\n ");
    return KAL_FALSE;
} /* OV2685MIPI_set_param_effect */

BOOL OV2685MIPI_set_param_banding(UINT16 para)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_set_param_banding function:\n ");
	switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
						spin_lock(&ov2685mipi_drv_lock);
						OV2685MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;
						spin_unlock(&ov2685mipi_drv_lock);
	
						OV2685MIPI_write_cmos_sensor(0x3a02,0x90);
            break;
        case AE_FLICKER_MODE_60HZ:			
						spin_lock(&ov2685mipi_drv_lock);
						OV2685MIPI_Banding_setting = AE_FLICKER_MODE_60HZ;
						spin_unlock(&ov2685mipi_drv_lock);
		
						OV2685MIPI_write_cmos_sensor(0x3a02,0x10);
            break;
            default:
            break;
    }
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_set_param_banding function:\n ");
        return TRUE;
} /* OV2685MIPI_set_param_banding */

BOOL OV2685MIPI_set_param_exposure(UINT16 para)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_set_param_exposure function:\n ");
	OV2685MIPISENSORDB("[OV2685MIPI]para=%d:\n",para);
	//spin_lock(&ov2685mipi_drv_lock);
   if (SCENE_MODE_HDR == OV2685MIPISensor.sceneMode && 
    SENSOR_MODE_CAPTURE == OV2685MIPISensor.SensorMode)
   {
   	   //spin_unlock(&ov2685mipi_drv_lock);
       OV2685MIPI_set_param_exposure_for_HDR(para);
       return TRUE;
   }
   //spin_unlock(&ov2685mipi_drv_lock);
	switch (para)
    {	
       case AE_EV_COMP_20:	                   

			 OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			 OV2685MIPI_write_cmos_sensor(0x3a03,0x5a);
			 OV2685MIPI_write_cmos_sensor(0x3a04,0x50);
			 OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			 OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
				
				break;
		case AE_EV_COMP_10:	                     
				
			 OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			 OV2685MIPI_write_cmos_sensor(0x3a03,0x52);
			 OV2685MIPI_write_cmos_sensor(0x3a04,0x48);
			 OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			 OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
				
			  break;
		case AE_EV_COMP_00: 
								
			 OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			 OV2685MIPI_write_cmos_sensor(0x3a03,0x4e);
			 OV2685MIPI_write_cmos_sensor(0x3a04,0x40);
			 OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			 OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
			  break;
   		 case AE_EV_COMP_n10:
								
			 OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			 OV2685MIPI_write_cmos_sensor(0x3a03,0x42);
			 OV2685MIPI_write_cmos_sensor(0x3a04,0x38);
			 OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			 OV2685MIPI_write_cmos_sensor(0x3208,0xa0);
							
			  break;
      	case AE_EV_COMP_n20:  
								
			 OV2685MIPI_write_cmos_sensor(0x3208,0x00);
			 OV2685MIPI_write_cmos_sensor(0x3a03,0x3a);
			 OV2685MIPI_write_cmos_sensor(0x3a04,0x30);
			 OV2685MIPI_write_cmos_sensor(0x3208,0x10);
			 OV2685MIPI_write_cmos_sensor(0x3208,0xa0);				
				
        	 break;
                default:
               break;
    }
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_set_param_exposure function:\n ");
    return TRUE;
} /* OV2685MIPI_set_param_exposure */

UINT32 OV2685MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	OV2685MIPISENSORDB("OV2685MIPIYUVSensorSetting:iCmd=%d,iPara=%d, %d \n",iCmd, iPara);
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIYUVSensorSetting function:\n ");
	switch (iCmd) {
		case FID_SCENE_MODE:
			OV2685MIPISENSORDB("[OV2685MIPI]FID_SCENE_MODE para\n",iPara);
			//OV2685MIPI_set_scene_mode(iPara);
	    	break; 	    
		case FID_AWB_MODE:
			OV2685MIPISENSORDB("[OV2685MIPI]FID_AWB_MODE para=%d\n", iPara);
			//OV2685MIPI_set_param_wb(iPara);
			break;
		case FID_COLOR_EFFECT:	    	    
			OV2685MIPISENSORDB("[OV2685MIPI]FID_COLOR_EFFECT para=%d\n", iPara);
			//OV2685MIPI_set_param_effect(iPara);
		 	break;
		case FID_AE_EV:   
			//OV2685MIPI_set_param_exposure(iPara);
			OV2685MIPISENSORDB("[OV2685MIPI]FID_AE_EV para=%d\n", iPara);
		    break;
		case FID_AE_FLICKER:    	    	    
			OV2685MIPISENSORDB("[OV2685MIPI]FID_AE_FLICKER para=%d\n", iPara);
			//OV2685MIPI_set_param_banding(iPara);
		 	break;
		case FID_AE_SCENE_MODE: 
			OV2685MIPISENSORDB("[OV2685MIPI]FID_AE_SCENE_MODE para=%d\n", iPara);
        	break; 
		case FID_ISP_CONTRAST:
			OV2685MIPISENSORDB("[OV2685MIPI]FID_ISP_CONTRAST para=%d\n", iPara);
            //OV2685MIPI_set_contrast(iPara);
            break;
        case FID_ISP_BRIGHT:
			OV2685MIPISENSORDB("[OV2685MIPI]FID_ISP_BRIGHT para=%d\n", iPara);
            //OV2685MIPI_set_brightness(iPara);
            break;
        case FID_ISP_SAT:
			OV2685MIPISENSORDB("[OV2685MIPI]FID_ISP_SAT para=%d\n", iPara);
            //OV2685MIPI_set_saturation(iPara);
	        break; 
	    case FID_ZOOM_FACTOR:
   		    OV2685MIPISENSORDB("[OV2685MIPI]FID_ZOOM_FACTOR:%d\n", iPara); 	    
			spin_lock(&ov2685mipi_drv_lock);
	        zoom_factor = iPara; 
			spin_unlock(&ov2685mipi_drv_lock);
            break; 
		case FID_AE_ISO:
			OV2685MIPISENSORDB("[OV2685MIPI]FID_AE_ISO:%d\n", iPara);
            //OV2685MIPI_set_iso(iPara);
            break;         
	  	default:
		  break;
	}
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIYUVSensorSetting function:\n ");
	  return TRUE;
}   /* OV2685MIPIYUVSensorSetting */

UINT32 OV2685MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIYUVSetVideoMode function:\n ");
	flash_mode = 2;
	if (u2FrameRate == 30)
	{

		}
    else if (u2FrameRate == 15)   
	{

	}   
    else 
    {
        printk("Wrong frame rate setting \n");
    } 
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIYUVSetVideoMode function:\n ");
    return TRUE; 
}

#if 0

/**************************/
static void OV2685MIPIGetEvAwbRef(UINT32 pSensorAEAWBRefStruct)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIGetEvAwbRef function:\n ");
	PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
	Ref->SensorAERef.AeRefLV05Shutter=0x170c;
	Ref->SensorAERef.AeRefLV05Gain=0x30;
	Ref->SensorAERef.AeRefLV13Shutter=0x24e;
	Ref->SensorAERef.AeRefLV13Gain=0x10;
	Ref->SensorAwbGainRef.AwbRefD65Rgain=0x610;
	Ref->SensorAwbGainRef.AwbRefD65Bgain=0x448;
	Ref->SensorAwbGainRef.AwbRefCWFRgain=0x4e0;
	Ref->SensorAwbGainRef.AwbRefCWFBgain=0x5a0;
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIGetEvAwbRef function:\n ");
}

static void OV2685MIPIGetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIGetCurAeAwbInfo function:\n ");
	PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
	Info->SensorAECur.AeCurShutter=OV2685MIPIReadShutter();
	Info->SensorAECur.AeCurGain=OV2685MIPIReadSensorGain() ;
	Info->SensorAwbGainCur.AwbCurRgain=((OV2685MIPIYUV_read_cmos_sensor(0x3401)&&0xff)+((OV2685MIPIYUV_read_cmos_sensor(0x3400)&&0xff)*256));
	Info->SensorAwbGainCur.AwbCurBgain=((OV2685MIPIYUV_read_cmos_sensor(0x3405)&&0xff)+((OV2685MIPIYUV_read_cmos_sensor(0x3404)&&0xff)*256));
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIGetCurAeAwbInfo function:\n ");
}
#endif
UINT32 OV2685MIPIMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
		kal_uint32 pclk;
		kal_int16 dummyLine;
		kal_uint16 lineLength,frameHeight;
		OV2685MIPISENSORDB("OV2685MIPIMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
		OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIMaxFramerateByScenario function:\n ");
		switch (scenarioId) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				pclk = 56000000;
				lineLength = OV2685MIPI_IMAGE_SENSOR_SVGA_WIDTH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - OV2685MIPI_IMAGE_SENSOR_SVGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&ov2685mipi_drv_lock);
				OV2685MIPISensor.SensorMode= SENSOR_MODE_PREVIEW;
				OV2685MIPISensor.PreviewDummyLines = dummyLine;
				spin_unlock(&ov2685mipi_drv_lock);
				//OV2685MIPISetDummy(OV2685MIPISensor.PreviewDummyPixels, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pclk = 56000000;
				lineLength = OV2685MIPI_IMAGE_SENSOR_VIDEO_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - OV2685MIPI_IMAGE_SENSOR_VIDEO_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				//spin_lock(&ov2685mipi_drv_lock);
				//ov8825.sensorMode = SENSOR_MODE_VIDEO;
				//spin_unlock(&ov2685mipi_drv_lock);
				//OV2685MIPISetDummy(0, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:			
				pclk = 90000000;
				lineLength = OV2685MIPI_IMAGE_SENSOR_QSXGA_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - OV2685MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&ov2685mipi_drv_lock);
				OV2685MIPISensor.CaptureDummyLines = dummyLine;
				OV2685MIPISensor.SensorMode= SENSOR_MODE_CAPTURE;
				spin_unlock(&ov2685mipi_drv_lock);
				//OV2685MIPISetDummy(OV2685MIPISensor.CaptureDummyPixels, dummyLine);			
				break;		
			case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
				break;		
			default:
				break;
		}	
		OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIMaxFramerateByScenario function:\n ");
		return ERROR_NONE;
}
UINT32 OV2685MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPIGetDefaultFramerateByScenario function:\n ");
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 150;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIGetDefaultFramerateByScenario function:\n ");
	return ERROR_NONE;
}
void OV2685MIPI_get_AEAWB_lock(UINT32 *pAElockRet32, UINT32 *pAWBlockRet32)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_get_AEAWB_lock function:\n ");
	*pAElockRet32 =1;
	*pAWBlockRet32=1;
	OV2685MIPISENSORDB("[OV2685MIPI]OV2685MIPI_get_AEAWB_lock,AE=%d,AWB=%d\n",*pAElockRet32,*pAWBlockRet32);
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_get_AEAWB_lock function:\n ");
}
void OV2685MIPI_GetDelayInfo(UINT32 delayAddr)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter OV2685MIPI_GetDelayInfo function:\n ");
	SENSOR_DELAY_INFO_STRUCT *pDelayInfo=(SENSOR_DELAY_INFO_STRUCT*)delayAddr;
	pDelayInfo->InitDelay=0;
	pDelayInfo->EffectDelay=0;
	pDelayInfo->AwbDelay=0;
	pDelayInfo->AFSwitchDelayFrame=50;
	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPI_GetDelayInfo function:\n ");
}

void OV2685MIPI_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
	OV2685MIPISENSORDB("[OV2685MIPI]enter ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          spin_lock(&ov2685mipi_drv_lock);
          OV2685MIPISensor.userAskAeLock = TRUE;
          spin_unlock(&ov2685mipi_drv_lock);
          OV2685MIPI_set_AE_mode(KAL_FALSE);
      break;
      case SENSOR_3A_AE_UNLOCK:
          spin_lock(&ov2685mipi_drv_lock);
          OV2685MIPISensor.userAskAeLock = FALSE;
          spin_unlock(&ov2685mipi_drv_lock);
          OV2685MIPI_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
          spin_lock(&ov2685mipi_drv_lock);
          OV2685MIPISensor.userAskAwbLock = TRUE;
          spin_unlock(&ov2685mipi_drv_lock);
          OV2685MIPI_set_AWB_mode(KAL_FALSE);
      break;

      case SENSOR_3A_AWB_UNLOCK:
          spin_lock(&ov2685mipi_drv_lock);
          OV2685MIPISensor.userAskAwbLock = FALSE;
          spin_unlock(&ov2685mipi_drv_lock);
          OV2685MIPI_set_AWB_mode_UNLOCK();
      break;
      default:
      	break;
   }
   OV2685MIPISENSORDB("[OV2685MIPI]exit ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   return;
}

#if 0
#define FLASH_BV_THRESHOLD 0x10 
static void OV2685MIPI_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{
	unsigned int NormBr;	   
	NormBr = OV2685MIPIYUV_read_cmos_sensor(0x56A1); 
	if (NormBr > FLASH_BV_THRESHOLD)
	{
	   *pFeatureReturnPara32 = FALSE;
		return;
	}
	*pFeatureReturnPara32 = TRUE;
	return;
}
#endif

UINT32 OV2685MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	UINT32 Tony_Temp1 = 0;
	UINT32 Tony_Temp2 = 0;
	Tony_Temp1 = pFeaturePara[0];
	Tony_Temp2 = pFeaturePara[1];
	
	//OV2685MIPISENSORDB("[OV2685MIPI]enter[OV2685MIPIFeatureControl]feature id=%d \n",FeatureId);
	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=OV2685MIPI_IMAGE_SENSOR_QSXGA_WITDH;
			*pFeatureReturnPara16=OV2685MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=OV2685MIPI_FULL_PERIOD_PIXEL_NUMS + OV2685MIPISensor.CaptureDummyPixels;
					*pFeatureReturnPara16=OV2685MIPI_FULL_PERIOD_LINE_NUMS + OV2685MIPISensor.CaptureDummyLines;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara16++=OV2685MIPI_PV_PERIOD_PIXEL_NUMS + OV2685MIPISensor.PreviewDummyPixels;
					*pFeatureReturnPara16=OV2685MIPI_PV_PERIOD_LINE_NUMS + OV2685MIPISensor.PreviewDummyLines;
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV2685MIPISensor.ZsdturePclk * 1000 *100;	 //unit: Hz				
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV2685MIPISensor.PreviewPclk * 1000 *100;	 //unit: Hz
					*pFeatureParaLen=4;
					break;
			}
			break;
		/**********************Strobe Ctrl Start *******************************/
		case SENSOR_FEATURE_SET_ESHUTTER:
			OV2685MIPISENSORDB("[OV2685MIPI] F_SET_ESHUTTER: Not Support\n");
			break;
		case SENSOR_FEATURE_SET_GAIN:
			OV2685MIPISENSORDB("[OV2685MIPI] F_SET_GAIN: Not Support\n");
			break;
		case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
			OV2685MIPISENSORDB("[OV2685MIPI] F_GET_AE_FLASHLIGHT_INFO: Not Support\n");
			break;
	    case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
            //OV2685MIPI_FlashTriggerCheck(pFeatureData32);
            OV2685MIPISENSORDB("[OV2685MIPI] F_GET_TRIGGER_FLASHLIGHT_INFO: %d\n", pFeatureData32);
            break;		
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			OV2685MIPISENSORDB("OV2685MIPI SENSOR_FEATURE_SET_FLASHLIGHT\n");
			break;
		/**********************Strobe Ctrl End *******************************/
		
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			OV2685MIPISENSORDB("[OV2685MIPI] F_SET_ISP_MASTER_CLOCK_FREQ\n");
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			OV2685MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = OV2685MIPIYUV_read_cmos_sensor(pSensorRegData->RegAddr);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &OV2685MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			OV2685MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_SET_YUV_3A_CMD:
            OV2685MIPI_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
            break;			
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_SET_VIDEO_MODE\n");
		    OV2685MIPIYUVSetVideoMode(*pFeatureData16);
		    break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_CHECK_SENSOR_ID\n");
			OV2685MIPI_GetSensorID(pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_GET_EV_AWB_REF\n");
			//OV2685MIPIGetEvAwbRef(*pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN\n");
			//OV2685MIPIGetCurAeAwbInfo(*pFeatureData32);			
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_GET_EXIF_INFO\n");
            OV2685MIPIGetExifInfo(*pFeatureData32);
            break;
		case SENSOR_FEATURE_GET_DELAY_INFO:
			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_GET_DELAY_INFO\n");
			OV2685MIPI_GetDelayInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_SET_SLAVE_I2C_ID:
             OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_SET_SLAVE_I2C_ID\n");
             OV2685MIPI_sensor_socket = *pFeatureData32;
             break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			OV2685MIPISENSORDB("[OV2685MIPI]OV2685_TEST_PATTERN_CHECKSUM\n");
			*pFeatureReturnPara32=OV2685_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;				
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:\
//			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO\n");
			OV2685MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,(MUINT32 *)*(pFeatureData32+1));
			break;
	    /**********************below is AF control**********************/	
		case SENSOR_FEATURE_GET_AF_STATUS:
		//	OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_GET_AF_STATUS\n");
            OV2685_FOCUS_OVT_AFC_Get_AF_Status(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;
		case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
	//		OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_SINGLE_FOCUS_MODE\n");
			OV2685_FOCUS_OVT_AFC_Single_Focus();
            break;
		case SENSOR_FEATURE_CONSTANT_AF:
//			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_CONSTANT_AF\n");
			//OV2685_FOCUS_OVT_AFC_Constant_Focus();
			break;
		case SENSOR_FEATURE_CANCEL_AF:
//			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_CANCEL_AF\n");
            //OV2685_FOCUS_OVT_AFC_Cancel_Focus();
            break;
		case SENSOR_FEATURE_GET_AF_INF:
//			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_GET_AF_INF\n");
            OV2685_FOCUS_Get_AF_Inf(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;
		case SENSOR_FEATURE_GET_AF_MACRO:
			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_GET_AF_MACRO\n");
            OV2685_FOCUS_Get_AF_Macro(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;
		case SENSOR_FEATURE_SET_AF_WINDOW: 
			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_SET_AF_WINDOW\n");
			OV2685_FOCUS_Set_AF_Window(*pFeatureData32);
            break;       					
        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS\n");
            OV2685_FOCUS_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break; 			
		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			OV2685MIPISENSORDB("[OV2685MIPI]SENSOR_FEATURE_GET_AF_STATUS\n");
			OV2685MIPI_get_AEAWB_lock(*pFeatureData32, *(pFeatureData32+1));
			break;					                              	               
        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			OV2685MIPISENSORDB("[OV2685MIPI]AE zone addr = 0x%x\n",*pFeatureData32);
            OV2685_FOCUS_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;        
        case SENSOR_FEATURE_SET_AE_WINDOW:
            OV2685MIPISENSORDB("[OV2685MIPI]AE zone addr = 0x%x\n",*pFeatureData32);			
            OV2685_FOCUS_Set_AE_Window(*pFeatureData32);
            break; 
		default:
			break;			
	}
//	OV2685MIPISENSORDB("[OV2685MIPI]exit OV2685MIPIFeatureControl function:\n ");
	return ERROR_NONE;
}	/* OV2685MIPIFeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncOV2685MIPI=
{
	OV2685MIPIOpen,
	OV2685MIPIGetInfo,
	OV2685MIPIGetResolution,
	OV2685MIPIFeatureControl,
	OV2685MIPIControl,
	OV2685MIPIClose
};

UINT32 OV2685_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)    
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncOV2685MIPI;
	return ERROR_NONE;
}

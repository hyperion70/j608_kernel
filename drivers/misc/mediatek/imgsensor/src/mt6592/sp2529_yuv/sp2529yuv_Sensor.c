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
 *  
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Leo Lee
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * [SP2529YUV V1.0.0]
 * 8.17.2012 Leo.Lee
 * .First Release
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by GalaxyCoreinc. DO NOT MODIFY!!
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

#include "sp2529yuv_Sensor.h"
#include "sp2529yuv_Camera_Sensor_para.h"
#include "sp2529yuv_CameraCustomized.h"

#define SP2529YUV_DEBUG
#ifdef SP2529YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif



  // #define  scaler_preview

#define  SP2529_SET_PAGE0    SP2529_write_cmos_sensor(0xfd,0x00)
#define  SP2529_SET_PAGE1    SP2529_write_cmos_sensor(0xfd,0x01)
#define  SP2529_SET_PAGE2    SP2529_write_cmos_sensor(0xfd,0x02)
#define  SP2529_SET_PAGE3    SP2529_write_cmos_sensor(0xfd,0x03)

#define WINDOW_SIZE_UXGA	0
#define WINDOW_SIZE_720P 	1
#define WINDOW_SIZE_SVGA 	2
#define WINDOW_SIZE_VGA	 	3


extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
/*************************************************************************
* FUNCTION
*    SP2529_write_cmos_sensor
*
* DESCRIPTION
*    This function wirte data to CMOS sensor through I2C
*
* PARAMETERS
*    addr: the 16bit address of register
*    para: the 8bit value of register
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void SP2529_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), SP2529_WRITE_ID); 

#if (defined(__SP2529_DEBUG_TRACE__))
    if (sizeof(out_buff) != rt) printk("I2C write %x, %x error\n", addr, para);
#endif
}

/*************************************************************************
* FUNCTION
*    SP2529_read_cmos_sensor
*
* DESCRIPTION
*    This function read data from CMOS sensor through I2C.
*
* PARAMETERS
*    addr: the 16bit address of register
*
* RETURNS
*    8bit data read through I2C
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint8 SP2529_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;

    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), SP2529_WRITE_ID)) {
        SENSORDB("ERROR: SP2529_read_cmos_sensor \n");
    }

#if (defined(__SP2529_DEBUG_TRACE__))
  if (size != rt) printk("I2C read %x error\n", addr);
#endif

  return in_buff[0];
}

//#define DEBUG_SENSOR_SP2529
#ifdef DEBUG_SENSOR_SP2529
#define SP2529_OP_CODE_INI		0x00		/* Initial value. */
#define SP2529_OP_CODE_REG		0x01		/* Register */
#define SP2529_OP_CODE_DLY		0x02		/* Delay */
#define SP2529_OP_CODE_END		0x03		/* End of initial setting. */
static kal_uint16 fromsd;

typedef struct
{
	u16 init_reg;
	u16 init_val;	/* Save the register value and delay tick */
	u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
} SP2529_initial_set_struct;

SP2529_initial_set_struct SP2529_Init_Reg[1000];

static u32 strtol(const char *nptr, u8 base)
{
	u8 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{
		if((base==16 && *nptr>='A' && *nptr<='F') || 
				(base==16 && *nptr>='a' && *nptr<='f') || 
				(base>=10 && *nptr>='0' && *nptr<='9') ||
				(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

static u8 SP2529_Initialize_from_T_Flash()
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0, j = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */


	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static u8 data_buff[10*1024] ;

	fp = filp_open("/mnt/sdcard/sp2529_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) { 
		printk("create file error\n"); 
		return -1; 
	} 
	fs = get_fs(); 
	set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	//printk("%s %d %d\n", buf,iFileLen,pos); 
	filp_close(fp, NULL); 
	set_fs(fs);





	/* Start parse the setting witch read from t-flash. */
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size))
	{
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
		{
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
			{
				curr_ptr++;		/* Skip block comment code. */
			}

			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}

		if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
		{
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}
		/* This just content one enter line. */
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		//printk(" curr_ptr1 = %s\n",curr_ptr);
		memcpy(func_ind, curr_ptr, 3);


		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */
			SP2529_Init_Reg[i].op_code = SP2529_OP_CODE_REG;

			SP2529_Init_Reg[i].init_reg = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */

			SP2529_Init_Reg[i].init_val = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */

		}
		else									/* DLY */
		{
			/* Need add delay for this setting. */ 
			curr_ptr += 4;	
			SP2529_Init_Reg[i].op_code = SP2529_OP_CODE_DLY;

			SP2529_Init_Reg[i].init_reg = 0xFF;
			SP2529_Init_Reg[i].init_val = strtol((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
		}
		i++;


		/* Skip to next line directly. */
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}

	/* (0xFFFF, 0xFFFF) means the end of initial setting. */
	SP2529_Init_Reg[i].op_code = SP2529_OP_CODE_END;
	SP2529_Init_Reg[i].init_reg = 0xFF;
	SP2529_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
	//printk(" %x  ==  %x\n",SP2529_Init_Reg[j].init_reg, SP2529_Init_Reg[j].init_val);

	/* Start apply the initial setting to sensor. */
#if 1
	for (j=0; j<i; j++)
	{
		if (SP2529_Init_Reg[j].op_code == SP2529_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (SP2529_Init_Reg[j].op_code == SP2529_OP_CODE_DLY)
		{
			msleep(SP2529_Init_Reg[j].init_val);		/* Delay */
		}
		else if (SP2529_Init_Reg[j].op_code == SP2529_OP_CODE_REG)
		{

			SP2529_write_cmos_sensor(SP2529_Init_Reg[j].init_reg, SP2529_Init_Reg[j].init_val);
		}
		else
		{
			printk("REG ERROR!\n");
		}
	}
#endif
	return 1;	
}

#endif

/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/
/* Global Valuable */

static kal_uint32 zoom_factor = 0; 

static kal_bool SP2529_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool SP2529_sensor_cap_state = KAL_FALSE; //Preview or Capture

static kal_uint16 SP2529_exposure_lines=0, SP2529_extra_exposure_lines = 0;

static kal_uint16 SP2529_Capture_Shutter=0;
static kal_uint16 SP2529_Capture_Extra_Lines=0;

kal_uint32 SP2529_capture_pclk_in_M=520,SP2529_preview_pclk_in_M=390,SP2529_PV_dummy_pixels=0,SP2529_PV_dummy_lines=0,SP2529_isp_master_clock=0;

static kal_uint32  SP2529_sensor_pclk=390;


MSDK_SENSOR_CONFIG_STRUCT SP2529SensorConfigData;
static kal_bool	setshutter = KAL_FALSE;
kal_bool SP2529_MPEG4_encode_mode = KAL_FALSE;
kal_bool   SP2529_CAPTURE_MODE = KAL_FALSE;
kal_bool   SP2529_CAM_Nightmode = 0;
kal_bool   SP2529_CAM_BANDING_50HZ = KAL_TRUE;
static kal_uint16 G_shutter;
static kal_uint8 G_Gain;

/*************************************************************************
 * FUNCTION
 *	SP2529_SetShutter
 *
 * DESCRIPTION
 *	This function set e-shutter of SP2529 to change exposure time.
 *
 * PARAMETERS
 *   iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void SP2529_Set_Shutter(kal_uint16 iShutter)
{
	kal_uint8 temp_reg_L, temp_reg_H;
	temp_reg_L = iShutter & 0xff;
	temp_reg_H = (iShutter >>8) & 0xff;
	SP2529_write_cmos_sensor(0xfd,0x00); 
	SP2529_write_cmos_sensor(0x03,temp_reg_H);
	SP2529_write_cmos_sensor(0x04,temp_reg_L);
	SENSORDB(" SP2529_Set_Shutter\r\n");
} /* Set_SP2529_Shutter */


/*************************************************************************
 * FUNCTION
 *	SP2529_read_Shutter
 *
 * DESCRIPTION
 *	This function read e-shutter of SP2529 .
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 SP2529_Read_Shutter(void)
{
	kal_uint8 temp_reg_L, temp_reg_H;
	kal_uint16 shutter;
	SP2529_write_cmos_sensor(0xfd,0x00); 
	temp_reg_L = SP2529_read_cmos_sensor(0x04);
	temp_reg_H = SP2529_read_cmos_sensor(0x03);

	shutter = (temp_reg_L & 0xFF) | (temp_reg_H << 8);

	SENSORDB(" SP2529_Read_Shutter %x \r\n",shutter);
	return shutter;
} /* SP2529_read_shutter */


static void SP2529_set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 SP2529_HV_Mirror;

	switch (image_mirror) 
	{
		case IMAGE_NORMAL:
			//SP2529_HV_Mirror = 0x14; 
		    break;
		case IMAGE_H_MIRROR:
			//SP2529_HV_Mirror = 0x15;
		    break;
		case IMAGE_V_MIRROR:
			//SP2529_HV_Mirror = 0x16; 
		    break;
		case IMAGE_HV_MIRROR:
			//SP2529_HV_Mirror = 0x17;
		    break;
		default:
		    break;
	}
	//SP2529_write_cmos_sensor(0x17, SP2529_HV_Mirror);
}

static void SP2529_set_AE_mode(kal_bool AE_enable)
{
	kal_uint8 temp_AE_reg = 0;

	//SP2529_write_cmos_sensor(0xfe, 0x00);
	if (AE_enable == KAL_TRUE)
	{
		// turn on AEC/AGC
	//	SP2529_write_cmos_sensor(0xb6, 0x03);
	}
	else
	{
		// turn off AEC/AGC
	//	SP2529_write_cmos_sensor(0xb6, 0x00);
	}
}


static void SP2529_set_AWB_mode(kal_bool AWB_enable)
{
	kal_uint8 temp_AWB_reg = 0;

	//SP2529_write_cmos_sensor(0xfe, 0x00);
	if (AWB_enable == KAL_TRUE)
	{
		//enable Auto WB
		//SP2529_write_cmos_sensor(0x82, 0xfe);
	}
	else
	{
		//turn off AWB
		//SP2529_write_cmos_sensor(0x82, 0xfc);
	}
}


/*************************************************************************
* FUNCTION
*	SP2529_night_mode
*
* DESCRIPTION
*	This function night mode of SP2529.
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
void SP2529_night_mode(kal_bool enable)
{
	//kal_uint16 night = SP2529_read_cmos_sensor(0x20);
	SENSORDB("Ronlus SP2529_night_mode\r\n");//¨°¨2D??Y3??|ìSP2529      a8-10 ????o¨?¨??810 3p  
	//sensorlist.cpp kd_imagesensor.h add related files  
#if 1//
	if (enable)/*night mode settings*/
	{
		SENSORDB(" night mode\r\n");
		SP2529_CAM_Nightmode = 1;
		//SP2529_write_cmos_sensor(0xfd,0x00);
		//SP2529_write_cmos_sensor(0xb2,SP2529_LOWLIGHT_Y0ffset);
		if(SP2529_MPEG4_encode_mode == KAL_TRUE) /*video night mode*/
		{
			SENSORDB("Ronlus video night mode\r\n");

			if(SP2529_CAM_BANDING_50HZ == KAL_TRUE)/*video night mode 50hz*/
			{	
				// 1.5PLL  fix10.0243fps   video night mode 50hz  
				SENSORDB(" video night mode 50hz\r\n");

				SP2529_write_cmos_sensor(0xfd,0x00);
				SP2529_write_cmos_sensor(0x03,0x02);
				SP2529_write_cmos_sensor(0x04,0xe8);
				SP2529_write_cmos_sensor(0x05,0x00);
				SP2529_write_cmos_sensor(0x06,0x00);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x00);
				SP2529_write_cmos_sensor(0x09,0x03);
				SP2529_write_cmos_sensor(0x0a,0xa7);
				SP2529_write_cmos_sensor(0xfd,0x01);
				SP2529_write_cmos_sensor(0xf0,0x00);
				SP2529_write_cmos_sensor(0xf7,0x7c);
				SP2529_write_cmos_sensor(0xf8,0x67);
				SP2529_write_cmos_sensor(0x02,0x0a);
				SP2529_write_cmos_sensor(0x03,0x01);
				SP2529_write_cmos_sensor(0x06,0x7c);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x01);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0x3d,0x0c);
				SP2529_write_cmos_sensor(0x3e,0x67);
				SP2529_write_cmos_sensor(0x3f,0x00);
				SP2529_write_cmos_sensor(0x88,0x21);
				SP2529_write_cmos_sensor(0x89,0xf8);
				SP2529_write_cmos_sensor(0x8a,0x44);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0xbe,0xd8);
				SP2529_write_cmos_sensor(0xbf,0x04);
				SP2529_write_cmos_sensor(0xd0,0xd8);
				SP2529_write_cmos_sensor(0xd1,0x04);
				SP2529_write_cmos_sensor(0xc9,0xd8);
				SP2529_write_cmos_sensor(0xca,0x04);
	

				SENSORDB("Ronlus video night mode 50hz\r\n");
			}
			else/*video night mode 60hz*/
			{ 
				SENSORDB("Ronlus video night mode 60hz\r\n");     
			    // 1.5pll  fix 10.0889 fps video night mode 60hz 

				SP2529_write_cmos_sensor(0xfd,0x00);
				SP2529_write_cmos_sensor(0x03,0x02);
				SP2529_write_cmos_sensor(0x04,0x70);
				SP2529_write_cmos_sensor(0x05,0x00);
				SP2529_write_cmos_sensor(0x06,0x00);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x00);
				SP2529_write_cmos_sensor(0x09,0x03);
				SP2529_write_cmos_sensor(0x0a,0x9d);
				SP2529_write_cmos_sensor(0xfd,0x01);
				SP2529_write_cmos_sensor(0xf0,0x00);
				SP2529_write_cmos_sensor(0xf7,0x68);
				SP2529_write_cmos_sensor(0xf8,0x68);
				SP2529_write_cmos_sensor(0x02,0x0c);
				SP2529_write_cmos_sensor(0x03,0x01);
				SP2529_write_cmos_sensor(0x06,0x68);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x01);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0x3d,0x0c);
				SP2529_write_cmos_sensor(0x3e,0x68);
				SP2529_write_cmos_sensor(0x3f,0x00);
				SP2529_write_cmos_sensor(0x88,0xec);
				SP2529_write_cmos_sensor(0x89,0xec);
				SP2529_write_cmos_sensor(0x8a,0x44);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0xbe,0xe0);
				SP2529_write_cmos_sensor(0xbf,0x04);
				SP2529_write_cmos_sensor(0xd0,0xe0);
				SP2529_write_cmos_sensor(0xd1,0x04);
				SP2529_write_cmos_sensor(0xc9,0xe0);
				SP2529_write_cmos_sensor(0xca,0x04);

			}
		}/*capture night mode*/
		else 
		{   
			SENSORDB("Ronlus capture night mode\r\n");
			if(SP2529_CAM_BANDING_50HZ == KAL_TRUE)/*capture night mode 50hz*/
			{	
			     /*capture night mode 50hz 1.5PLL 5-22.065fps*/
				SENSORDB(" capture night mode 50hz\r\n");//3                            

				SP2529_write_cmos_sensor(0xfd,0x00);
				SP2529_write_cmos_sensor(0x03,0x06);
				SP2529_write_cmos_sensor(0x04,0x66);
				SP2529_write_cmos_sensor(0x05,0x00);
				SP2529_write_cmos_sensor(0x06,0x00);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x00);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0x0a,0x8e);
				SP2529_write_cmos_sensor(0xfd,0x01);
				SP2529_write_cmos_sensor(0xf0,0x01);
				SP2529_write_cmos_sensor(0xf7,0x11);
				SP2529_write_cmos_sensor(0xf8,0xe4);
				SP2529_write_cmos_sensor(0x02,0x14);
				SP2529_write_cmos_sensor(0x03,0x01);
				SP2529_write_cmos_sensor(0x06,0x11);
				SP2529_write_cmos_sensor(0x07,0x01);
				SP2529_write_cmos_sensor(0x08,0x01);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0x3d,0x18);
				SP2529_write_cmos_sensor(0x3e,0xe4);
				SP2529_write_cmos_sensor(0x3f,0x00);
				SP2529_write_cmos_sensor(0x88,0xe0);
				SP2529_write_cmos_sensor(0x89,0x3e);
				SP2529_write_cmos_sensor(0x8a,0x21);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0xbe,0x54);
				SP2529_write_cmos_sensor(0xbf,0x15);
				SP2529_write_cmos_sensor(0xd0,0x54);
				SP2529_write_cmos_sensor(0xd1,0x15);
				SP2529_write_cmos_sensor(0xc9,0x54);
				SP2529_write_cmos_sensor(0xca,0x15);

			}
			else/*capture night mode 60hz*/
			{ 
			    /*capture night mode 60hz 1.5PLL 5-22.0211fps*/
				SENSORDB(" capture night mode 60hz\r\n");                                             

				SP2529_write_cmos_sensor(0xfd,0x00);
				SP2529_write_cmos_sensor(0x03,0x05);
				SP2529_write_cmos_sensor(0x04,0x52);
				SP2529_write_cmos_sensor(0x05,0x00);
				SP2529_write_cmos_sensor(0x06,0x00);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x00);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0x0a,0x90);
				SP2529_write_cmos_sensor(0xfd,0x01);
				SP2529_write_cmos_sensor(0xf0,0x00);
				SP2529_write_cmos_sensor(0xf7,0xe3);
				SP2529_write_cmos_sensor(0xf8,0xe3);
				SP2529_write_cmos_sensor(0x02,0x18);
				SP2529_write_cmos_sensor(0x03,0x01);
				SP2529_write_cmos_sensor(0x06,0xe3);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x01);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0x3d,0x18);
				SP2529_write_cmos_sensor(0x3e,0xe3);
				SP2529_write_cmos_sensor(0x3f,0x00);
				SP2529_write_cmos_sensor(0x88,0x41);
				SP2529_write_cmos_sensor(0x89,0x41);
				SP2529_write_cmos_sensor(0x8a,0x22);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0xbe,0x48);
				SP2529_write_cmos_sensor(0xbf,0x15);
				SP2529_write_cmos_sensor(0xd0,0x48);
				SP2529_write_cmos_sensor(0xd1,0x15);
				SP2529_write_cmos_sensor(0xc9,0x48);
				SP2529_write_cmos_sensor(0xca,0x15);
			}
		}
	}
	else /*normal mode settings*/
	{
		SENSORDB("Ronlus normal mode\r\n");
		SP2529_CAM_Nightmode = 0;
	//	SP2529_write_cmos_sensor(0xfd,0x00);
	//	SP2529_write_cmos_sensor(0xb2,SP2529_NORMAL_Y0ffset);
		if (SP2529_MPEG4_encode_mode == KAL_TRUE) 
		{
			SENSORDB("Ronlus video normal mode\r\n");
			if(SP2529_CAM_BANDING_50HZ == KAL_TRUE)/*video normal mode 50hz*/
			{
				SENSORDB("Ronlus video normal mode 50hz\r\n");
				/*capture night mode 60hz 1.5PLL 5-22.065fps*/
				SENSORDB(" capture night mode 50hz\r\n");                                             

				SP2529_write_cmos_sensor(0xfd,0x00);
				SP2529_write_cmos_sensor(0x03,0x05);
				SP2529_write_cmos_sensor(0x04,0x52);
				SP2529_write_cmos_sensor(0x05,0x00);
				SP2529_write_cmos_sensor(0x06,0x00);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x00);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0x0a,0x90);
				SP2529_write_cmos_sensor(0xfd,0x01);
				SP2529_write_cmos_sensor(0xf0,0x00);
				SP2529_write_cmos_sensor(0xf7,0xe3);
				SP2529_write_cmos_sensor(0xf8,0xe3);
				SP2529_write_cmos_sensor(0x02,0x18);
				SP2529_write_cmos_sensor(0x03,0x01);
				SP2529_write_cmos_sensor(0x06,0xe3);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x01);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0x3d,0x18);
				SP2529_write_cmos_sensor(0x3e,0xe3);
				SP2529_write_cmos_sensor(0x3f,0x00);
				SP2529_write_cmos_sensor(0x88,0x41);
				SP2529_write_cmos_sensor(0x89,0x41);
				SP2529_write_cmos_sensor(0x8a,0x22);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0xbe,0x48);
				SP2529_write_cmos_sensor(0xbf,0x15);
				SP2529_write_cmos_sensor(0xd0,0x48);
				SP2529_write_cmos_sensor(0xd1,0x15);
				SP2529_write_cmos_sensor(0xc9,0x48);
				SP2529_write_cmos_sensor(0xca,0x15); 

			}
			else/*video normal mode 60hz*/
			{
				SENSORDB(" video normal mode 60hz\r\n");  
				/*video normal mode 1.5Pll fix 22.0211fps 60hz*/

				SP2529_write_cmos_sensor(0xfd,0x00);
				SP2529_write_cmos_sensor(0x03,0x05);
				SP2529_write_cmos_sensor(0x04,0x52);
				SP2529_write_cmos_sensor(0x05,0x00);
				SP2529_write_cmos_sensor(0x06,0x00);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x00);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0x0a,0x90);
				SP2529_write_cmos_sensor(0xfd,0x01);
				SP2529_write_cmos_sensor(0xf0,0x00);
				SP2529_write_cmos_sensor(0xf7,0xe3);
				SP2529_write_cmos_sensor(0xf8,0xe3);
				SP2529_write_cmos_sensor(0x02,0x05);
				SP2529_write_cmos_sensor(0x03,0x01);
				SP2529_write_cmos_sensor(0x06,0xe3);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x01);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0x3d,0x05);
				SP2529_write_cmos_sensor(0x3e,0xe3);
				SP2529_write_cmos_sensor(0x3f,0x00);
				SP2529_write_cmos_sensor(0x88,0x41);
				SP2529_write_cmos_sensor(0x89,0x41);
				SP2529_write_cmos_sensor(0x8a,0x22);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0xbe,0x6f);
				SP2529_write_cmos_sensor(0xbf,0x04);
				SP2529_write_cmos_sensor(0xd0,0x6f);
				SP2529_write_cmos_sensor(0xd1,0x04);
				SP2529_write_cmos_sensor(0xc9,0x6f);
				SP2529_write_cmos_sensor(0xca,0x04);


			}
		}
		else/*capture normal mode*/
		{
			SENSORDB("Ronlus capture normal mode\r\n");
			if(SP2529_CAM_BANDING_50HZ == KAL_TRUE)/*capture normal mode 50hz*/
			{
				SENSORDB(" capture normal mode 50hz\r\n");
				/*capture normal mode 1.5pll 10-22.0695fps 50hz*/

				SP2529_write_cmos_sensor(0xfd,0x00);
				SP2529_write_cmos_sensor(0x03,0x06);
				SP2529_write_cmos_sensor(0x04,0x66);
				SP2529_write_cmos_sensor(0x05,0x00);
				SP2529_write_cmos_sensor(0x06,0x00);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x00);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0x0a,0x8e);
				SP2529_write_cmos_sensor(0xfd,0x01);
				SP2529_write_cmos_sensor(0xf0,0x01);
				SP2529_write_cmos_sensor(0xf7,0x11);
				SP2529_write_cmos_sensor(0xf8,0xe4);
				SP2529_write_cmos_sensor(0x02,0x0a);
				SP2529_write_cmos_sensor(0x03,0x01);
				SP2529_write_cmos_sensor(0x06,0x11);
				SP2529_write_cmos_sensor(0x07,0x01);
				SP2529_write_cmos_sensor(0x08,0x01);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0x3d,0x0c);
				SP2529_write_cmos_sensor(0x3e,0xe4);
				SP2529_write_cmos_sensor(0x3f,0x00);
				SP2529_write_cmos_sensor(0x88,0xe0);
				SP2529_write_cmos_sensor(0x89,0x3e);
				SP2529_write_cmos_sensor(0x8a,0x21);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0xbe,0xaa);
				SP2529_write_cmos_sensor(0xbf,0x0a);
				SP2529_write_cmos_sensor(0xd0,0xaa);
				SP2529_write_cmos_sensor(0xd1,0x0a);
				SP2529_write_cmos_sensor(0xc9,0xaa);
				SP2529_write_cmos_sensor(0xca,0x0a);

			}
			else/*video normal mode 60hz*/
			{
				SENSORDB(" capture normal mode 60hz\r\n");
				/*capture normal mode 1.5pll 10-22.0211fps 60hz*/

				SP2529_write_cmos_sensor(0xfd,0x00);
				SP2529_write_cmos_sensor(0x03,0x05);
				SP2529_write_cmos_sensor(0x04,0x52);
				SP2529_write_cmos_sensor(0x05,0x00);
				SP2529_write_cmos_sensor(0x06,0x00);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x00);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0x0a,0x90);
				SP2529_write_cmos_sensor(0xfd,0x01);
				SP2529_write_cmos_sensor(0xf0,0x00);
				SP2529_write_cmos_sensor(0xf7,0xe3);
				SP2529_write_cmos_sensor(0xf8,0xe3);
				SP2529_write_cmos_sensor(0x02,0x0c);
				SP2529_write_cmos_sensor(0x03,0x01);
				SP2529_write_cmos_sensor(0x06,0xe3);
				SP2529_write_cmos_sensor(0x07,0x00);
				SP2529_write_cmos_sensor(0x08,0x01);
				SP2529_write_cmos_sensor(0x09,0x00);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0x3d,0x0c);
				SP2529_write_cmos_sensor(0x3e,0xe3);
				SP2529_write_cmos_sensor(0x3f,0x00);
				SP2529_write_cmos_sensor(0x88,0x41);
				SP2529_write_cmos_sensor(0x89,0x41);
				SP2529_write_cmos_sensor(0x8a,0x22);
				SP2529_write_cmos_sensor(0xfd,0x02);
				SP2529_write_cmos_sensor(0xbe,0xa4);
				SP2529_write_cmos_sensor(0xbf,0x0a);
				SP2529_write_cmos_sensor(0xd0,0xa4);
				SP2529_write_cmos_sensor(0xd1,0x0a);
				SP2529_write_cmos_sensor(0xc9,0xa4);
				SP2529_write_cmos_sensor(0xca,0x0a);
			}
		}
	}

#endif
}	/* SP2529_night_mode */



/*************************************************************************
* FUNCTION
*	SP2529_GetSensorID
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
static kal_uint32 SP2529_GetSensorID(kal_uint32 *sensorID)

{
	kal_uint16 sensor_id=0;
    int i;
    
	SENSORDB("xieyang SP2529GetSensorID ");
	//SENSORDB("xieyang in GPIO_CAMERA_CMPDN_PIN=%d,GPIO_CAMERA_CMPDN1_PIN=%d", 
	//	mt_get_gpio_out(GPIO_CAMERA_CMPDN_PIN),mt_get_gpio_out(GPIO_CAMERA_CMPDN1_PIN));

	for(i=0;i<3;i++)
	{
		SP2529_write_cmos_sensor(0xfd, 0x00); 
		sensor_id = SP2529_read_cmos_sensor(0x02);
		SENSORDB("%s sensor_id=%d\n", __func__, sensor_id);

		if (sensor_id == SP2529_SENSOR_ID)
		{
			break;
		}
	}

	if(sensor_id != SP2529_SENSOR_ID)
	{
		*sensorID = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	else
	{
		*sensorID = SP2529_SENSOR_ID;
	}

	return ERROR_NONE;    
}   /* SP2529Open  */
static void SP2529_Sensor_Init(void)
{
    //bining 800 600
    SP2529_write_cmos_sensor(0xfd,0x01);
    SP2529_write_cmos_sensor(0x36,0x02);     
    SP2529_write_cmos_sensor(0xfd,0x00);
    SP2529_write_cmos_sensor(0x2f,0x09);
    SP2529_write_cmos_sensor(0x1d,0x00);        
    SP2529_write_cmos_sensor(0x11,0x40);
    SP2529_write_cmos_sensor(0x18,0x00);
    SP2529_write_cmos_sensor(0x1a,0x49);
    SP2529_write_cmos_sensor(0x1e,0x01);
    SP2529_write_cmos_sensor(0x0c,0x55);
    SP2529_write_cmos_sensor(0x21,0x10);
    SP2529_write_cmos_sensor(0x22,0x2a);
    SP2529_write_cmos_sensor(0x25,0xad);
    SP2529_write_cmos_sensor(0x27,0xa1);
    SP2529_write_cmos_sensor(0x1f,0xc0);
    SP2529_write_cmos_sensor(0x28,0x0b);
    SP2529_write_cmos_sensor(0x2b,0x8c);
    SP2529_write_cmos_sensor(0x26,0x09);
    SP2529_write_cmos_sensor(0x2c,0x45);
    SP2529_write_cmos_sensor(0x37,0x00);
    SP2529_write_cmos_sensor(0x16,0x01);
    SP2529_write_cmos_sensor(0x17,0x2f);

    SP2529_write_cmos_sensor(0x69,0x01);
    SP2529_write_cmos_sensor(0x6a,0x2d);
    SP2529_write_cmos_sensor(0x13,0x4f);
    SP2529_write_cmos_sensor(0x6b,0x50);
    SP2529_write_cmos_sensor(0x6c,0x50);
    SP2529_write_cmos_sensor(0x6f,0x50);
    SP2529_write_cmos_sensor(0x73,0x51);

    SP2529_write_cmos_sensor(0x7a,0x41);
    SP2529_write_cmos_sensor(0x70,0x41);
    SP2529_write_cmos_sensor(0x7d,0x40);
    SP2529_write_cmos_sensor(0x74,0x40);
    SP2529_write_cmos_sensor(0x75,0x40);
    SP2529_write_cmos_sensor(0x14,0x01);
    SP2529_write_cmos_sensor(0x15,0x20);
    SP2529_write_cmos_sensor(0x71,0x22);
    SP2529_write_cmos_sensor(0x76,0x22);
    SP2529_write_cmos_sensor(0x7c,0x22);

    SP2529_write_cmos_sensor(0x7e,0x21);
    SP2529_write_cmos_sensor(0x72,0x21);
    SP2529_write_cmos_sensor(0x77,0x20);

    SP2529_write_cmos_sensor(0xfd,0x00);

    SP2529_write_cmos_sensor(0xfd,0x01);
    SP2529_write_cmos_sensor(0x32,0x00);
    SP2529_write_cmos_sensor(0xfb,0x21); 
    SP2529_write_cmos_sensor(0xfd,0x02);
    SP2529_write_cmos_sensor(0x85,0x00); 
    SP2529_write_cmos_sensor(0x00,0x82);
    SP2529_write_cmos_sensor(0x01,0x82);
    //ae setting    1.5pll 10-22.0695fps 
    SP2529_write_cmos_sensor(0xfd,0x00);
    SP2529_write_cmos_sensor(0x03,0x06);
    SP2529_write_cmos_sensor(0x04,0x66);
    SP2529_write_cmos_sensor(0x05,0x00);
    SP2529_write_cmos_sensor(0x06,0x00);
    SP2529_write_cmos_sensor(0x07,0x00);
    SP2529_write_cmos_sensor(0x08,0x00);
    SP2529_write_cmos_sensor(0x09,0x00);
    SP2529_write_cmos_sensor(0x0a,0x8e);
    SP2529_write_cmos_sensor(0xfd,0x01);
    SP2529_write_cmos_sensor(0xf0,0x01);
    SP2529_write_cmos_sensor(0xf7,0x11);
    SP2529_write_cmos_sensor(0xf8,0xe4);
    SP2529_write_cmos_sensor(0x02,0x0a);
    SP2529_write_cmos_sensor(0x03,0x01);
    SP2529_write_cmos_sensor(0x06,0x11);
    SP2529_write_cmos_sensor(0x07,0x01);
    SP2529_write_cmos_sensor(0x08,0x01);
    SP2529_write_cmos_sensor(0x09,0x00);
    SP2529_write_cmos_sensor(0xfd,0x02);
    SP2529_write_cmos_sensor(0x3d,0x0c);
    SP2529_write_cmos_sensor(0x3e,0xe4);
    SP2529_write_cmos_sensor(0x3f,0x00);
    SP2529_write_cmos_sensor(0x88,0xe0);
    SP2529_write_cmos_sensor(0x89,0x3e);
    SP2529_write_cmos_sensor(0x8a,0x21);
    SP2529_write_cmos_sensor(0xfd,0x02);
    SP2529_write_cmos_sensor(0xbe,0xaa);
    SP2529_write_cmos_sensor(0xbf,0x0a);
    SP2529_write_cmos_sensor(0xd0,0xaa);
    SP2529_write_cmos_sensor(0xd1,0x0a);
    SP2529_write_cmos_sensor(0xc9,0xaa);
    SP2529_write_cmos_sensor(0xca,0x0a);

    SP2529_write_cmos_sensor(0xb8,0x90);  //mean_nr_dummy                                                                     
    SP2529_write_cmos_sensor(0xb9,0xa0);  //mean_dummy_nr                                                                     
    SP2529_write_cmos_sensor(0xba,0x30);  //mean_dummy_low                                                                    
    SP2529_write_cmos_sensor(0xbb,0x45);  //mean_low_dummy                                                                    
    SP2529_write_cmos_sensor(0xbc,0x90);  //rpc_heq_low                                                                       
    SP2529_write_cmos_sensor(0xbd,0x50);  //rpc_heq_dummy                                                                     
    SP2529_write_cmos_sensor(0xfd,0x03);                                                                                     
    SP2529_write_cmos_sensor(0x77,0x50);	 //rpc_heq_nr2                                                                     
    //rpc                                                                                              
    SP2529_write_cmos_sensor(0xfd,0x01);                                                                                     
    SP2529_write_cmos_sensor(0xe0,0x48);                                                                             
    SP2529_write_cmos_sensor(0xe1,0x38);                                                                             
    SP2529_write_cmos_sensor(0xe2,0x30);                                                                             
    SP2529_write_cmos_sensor(0xe3,0x2c);                                                                             
    SP2529_write_cmos_sensor(0xe4,0x2c);                                                                             
    SP2529_write_cmos_sensor(0xe5,0x2a);                                                                             
    SP2529_write_cmos_sensor(0xe6,0x2a);                                                                             
    SP2529_write_cmos_sensor(0xe7,0x28);                                                                             
    SP2529_write_cmos_sensor(0xe8,0x28);                                                                             
    SP2529_write_cmos_sensor(0xe9,0x28);                                                                             
    SP2529_write_cmos_sensor(0xea,0x26);                                                                             
    SP2529_write_cmos_sensor(0xf3,0x26);                                                                             
    SP2529_write_cmos_sensor(0xf4,0x26);                                                                             
    SP2529_write_cmos_sensor(0xfd,0x01);  //ae min gain                                                                       
    SP2529_write_cmos_sensor(0x04,0xa0); //rpc_max_indr                                                                      
    SP2529_write_cmos_sensor(0x05,0x26);  //rpc_min_indr                                                                   
    SP2529_write_cmos_sensor(0x0a,0x48);  //rpc_max_outdr                                                                     
    SP2529_write_cmos_sensor(0x0b,0x26);//rpc_min_outdr                                                                     
                                                         
    SP2529_write_cmos_sensor(0xfd,0x01);  //ae target                                                                         
    SP2529_write_cmos_sensor(0xeb,0x7a);  //target_indr		                                                          
    SP2529_write_cmos_sensor(0xec,0x7a);  //target_outdr		                                                          
    SP2529_write_cmos_sensor(0xed,0x04);  //lock_range                                                                        
    SP2529_write_cmos_sensor(0xee,0x08);  //hold_range                                                                        
                                                
    //去坏像素    
    SP2529_write_cmos_sensor(0xfd,0x03);
    SP2529_write_cmos_sensor(0x52,0xff);//dpix_wht_ofst_outdoor                                                                    
    SP2529_write_cmos_sensor(0x53,0x60);//dpix_wht_ofst_normal1
    SP2529_write_cmos_sensor(0x94,0x00);//dpix_wht_ofst_normal2
    SP2529_write_cmos_sensor(0x54,0x00);//dpix_wht_ofst_dummy                                                                                       
    SP2529_write_cmos_sensor(0x55,0x00);//dpix_wht_ofst_low

    SP2529_write_cmos_sensor(0x56,0x80);//dpix_blk_ofst_outdoor
    SP2529_write_cmos_sensor(0x57,0x80);//dpix_blk_ofst_normal1
    SP2529_write_cmos_sensor(0x95,0x00);//dpix_blk_ofst_normal2
    SP2529_write_cmos_sensor(0x58,0x00);//dpix_blk_ofst_dummy  
    SP2529_write_cmos_sensor(0x59,0x00);//dpix_blk_ofst_low    

    SP2529_write_cmos_sensor(0x5a,0xf6);//dpix_wht_ratio
    SP2529_write_cmos_sensor(0x5b,0x00);
    SP2529_write_cmos_sensor(0x5c,0x88);//dpix_blk_ratio
    SP2529_write_cmos_sensor(0x5d,0x00);
    SP2529_write_cmos_sensor(0x96,0x00);//dpix_wht/blk_ratio_nr2
                                                       
    SP2529_write_cmos_sensor(0xfd,0x03);
    SP2529_write_cmos_sensor(0x8a,0x00);
    SP2529_write_cmos_sensor(0x8b,0x00);
    SP2529_write_cmos_sensor(0x8c,0xff);

    SP2529_write_cmos_sensor(0x22,0xff);//dem_gdif_thr_outdoor
    SP2529_write_cmos_sensor(0x23,0xff);//dem_gdif_thr_normal
    SP2529_write_cmos_sensor(0x24,0xff);//dem_gdif_thr_dummy
    SP2529_write_cmos_sensor(0x25,0xff);//dem_gdif_thr_low

    SP2529_write_cmos_sensor(0x5e,0xff);//dem_gwnd_wht_outdoor
    SP2529_write_cmos_sensor(0x5f,0xff);//dem_gwnd_wht_normal 
    SP2529_write_cmos_sensor(0x60,0xff);//dem_gwnd_wht_dummy  
    SP2529_write_cmos_sensor(0x61,0xff);//dem_gwnd_wht_low    
    SP2529_write_cmos_sensor(0x62,0x00);//dem_gwnd_blk_outdoor
    SP2529_write_cmos_sensor(0x63,0x00);//dem_gwnd_blk_normal 
    SP2529_write_cmos_sensor(0x64,0x00);//dem_gwnd_blk_dummy  
    SP2529_write_cmos_sensor(0x65,0x00);//dem_gwnd_blk_low 

    //lsc                                                                                             
    SP2529_write_cmos_sensor(0xfd,0x01);                                                                                     
    SP2529_write_cmos_sensor(0x21,0x00);  //lsc_sig_ru lsc_sig_lu                                                             
    SP2529_write_cmos_sensor(0x22,0x00);  //lsc_sig_rd lsc_sig_ld                                                             
    SP2529_write_cmos_sensor(0x26,0xa0);  //lsc_gain_thr                                                                      
    SP2529_write_cmos_sensor(0x27,0x14);  //lsc_exp_thrl                                                                      
    SP2529_write_cmos_sensor(0x28,0x05);  //lsc_exp_thrh                                                                      
    SP2529_write_cmos_sensor(0x29,0x20);  //lsc_dec_fac                                                                       
    SP2529_write_cmos_sensor(0x2a,0x01);  //lsc_rpc_en lens衰减自适应                                                         
                                                 
    SP2529_write_cmos_sensor(0xfd,0x01);                                                                     
    SP2529_write_cmos_sensor(0xa1,0x15);                                                                     
    SP2529_write_cmos_sensor(0xa2,0x15);                                                                  
    SP2529_write_cmos_sensor(0xa3,0x15);                                                                     
    SP2529_write_cmos_sensor(0xa4,0x15);                                                                     
    SP2529_write_cmos_sensor(0xa5,0x12);                                                                  
    SP2529_write_cmos_sensor(0xa6,0x14);                                                                     
    SP2529_write_cmos_sensor(0xa7,0x0e);                                                                     
    SP2529_write_cmos_sensor(0xa8,0x0d);                                                                     
    SP2529_write_cmos_sensor(0xa9,0x11);                                                                  
    SP2529_write_cmos_sensor(0xaa,0x16);                                                                  
    SP2529_write_cmos_sensor(0xab,0x0d);                                                                     
    SP2529_write_cmos_sensor(0xac,0x0b);                                                                     
    SP2529_write_cmos_sensor(0xad,0x13);                                                                  
    SP2529_write_cmos_sensor(0xae,0x08);                                                                     
    SP2529_write_cmos_sensor(0xaf,0x05);                                                                     
    SP2529_write_cmos_sensor(0xb0,0x04);                                                                      
    SP2529_write_cmos_sensor(0xb1,0x10);                                                                   
    SP2529_write_cmos_sensor(0xb2,0x0a);                                                                   
    SP2529_write_cmos_sensor(0xb3,0x05);                                                                      
    SP2529_write_cmos_sensor(0xb4,0x07);                                                                      
    SP2529_write_cmos_sensor(0xb5,0x10);                                                                   
    SP2529_write_cmos_sensor(0xb6,0x0a);                                                                      
    SP2529_write_cmos_sensor(0xb7,0x07);                                                                      
    SP2529_write_cmos_sensor(0xb8,0x07); 

    //AWB                                                                                            
    SP2529_write_cmos_sensor(0xfd,0x02);                                                                                     
    SP2529_write_cmos_sensor(0x26,0xac); //Red channel gain                                                                   
    SP2529_write_cmos_sensor(0x27,0x91); //Blue channel gain                                                                  
    SP2529_write_cmos_sensor(0x28,0xcc); //Y top value limit                                                                  
    SP2529_write_cmos_sensor(0x29,0x01); //Y bot value limit                                                                  
    SP2529_write_cmos_sensor(0x2a,0x02); //rg_limit_log                                                                       
    SP2529_write_cmos_sensor(0x2b,0x16); //bg_limit_log                                                                       
    SP2529_write_cmos_sensor(0x2c,0x20); //Awb image center row start                                                         
    SP2529_write_cmos_sensor(0x2d,0xdc); //Awb image center row end                                                           
    SP2529_write_cmos_sensor(0x2e,0x20); //Awb image center col start                                                         
    SP2529_write_cmos_sensor(0x2f,0x96); //Awb image center col end                                                           
    SP2529_write_cmos_sensor(0x1b,0x80); //b,g mult a constant for detect white pixel                                         
    SP2529_write_cmos_sensor(0x1a,0x80); //r,g mult a constant for detect white pixel                                         
    SP2529_write_cmos_sensor(0x18,0x16); //wb_fine_gain_step,wb_rough_gain_step                                               
    SP2529_write_cmos_sensor(0x19,0x26); //wb_dif_fine_th, wb_dif_rough_th                                                    
                                                             
    //d65 10                                                                                 
    SP2529_write_cmos_sensor(0x66,0x35);//41//3a                                                                            
    SP2529_write_cmos_sensor(0x67,0x5f);//6d//6a                                                                             
    SP2529_write_cmos_sensor(0x68,0xb5);//a9//b3                                                                              
    SP2529_write_cmos_sensor(0x69,0xdb);//d1//de                                                                            
    SP2529_write_cmos_sensor(0x6a,0xa5);                                                                              
                                                             
    //indoor 11                                                                                        
    SP2529_write_cmos_sensor(0x7c,0x33);// 0b                                                                             
    SP2529_write_cmos_sensor(0x7d,0x58);//51                                                                               
    SP2529_write_cmos_sensor(0x7e,0xe7);//e7                                                                         
    SP2529_write_cmos_sensor(0x7f,0x0c);//06                                                                                     
    SP2529_write_cmos_sensor(0x80,0xa6);                                                                                     
                                                           
    //cwf   12                                                                             
    SP2529_write_cmos_sensor(0x70,0x20);//21//1f                                                                             
    SP2529_write_cmos_sensor(0x71,0x45);// 0b//49                                                                            
    SP2529_write_cmos_sensor(0x72,0x08);//fd//05                                                                            
    SP2529_write_cmos_sensor(0x73,0x2a);//22//2e                                                                             
    SP2529_write_cmos_sensor(0x74,0xaa);//a6                                                                            
                                                     
    //tl84     13                                                                              
    SP2529_write_cmos_sensor(0x6b,0x00);//09                                                                             
    SP2529_write_cmos_sensor(0x6c,0x20);//32                                                                            
    SP2529_write_cmos_sensor(0x6d,0x12);//0b                                                                             
    SP2529_write_cmos_sensor(0x6e,0x36);//34                                                                             
    SP2529_write_cmos_sensor(0x6f,0xaa);//aa                                                                             
                                                     
    //f        14                                                                              
    SP2529_write_cmos_sensor(0x61,0xeb);//f3//e9                                                                             
    SP2529_write_cmos_sensor(0x62,0x13);// 0e//16                                                                             
    SP2529_write_cmos_sensor(0x63,0x29);//20//2a                                                                            
    SP2529_write_cmos_sensor(0x64,0x4e);//48//4c                                                                            
    SP2529_write_cmos_sensor(0x65,0x6a);                                                                             
                                                           
    SP2529_write_cmos_sensor(0x75,0x00);                                                                                     
    SP2529_write_cmos_sensor(0x76,0x09);                                                                                     
    SP2529_write_cmos_sensor(0x77,0x02);                                                                                     
    SP2529_write_cmos_sensor(0x0e,0x16);                                                                                     
    SP2529_write_cmos_sensor(0x3b,0x09);//awb                                                                                 
    SP2529_write_cmos_sensor(0xfd,0x02); //awb outdoor mode                                                                  
    SP2529_write_cmos_sensor(0x02,0x00);//outdoor exp 5msb                                                                    
    SP2529_write_cmos_sensor(0x03,0x10);//outdoor exp 8lsb                                                                    
    SP2529_write_cmos_sensor(0x04,0xf0);//outdoor rpc                                                                         
    SP2529_write_cmos_sensor(0xf5,0xb3);//outdoor rgain top                                                                   
    SP2529_write_cmos_sensor(0xf6,0x80);//outdoor rgain bot                                                                   
    SP2529_write_cmos_sensor(0xf7,0xe0);//outdoor bgain top                                                                   
    SP2529_write_cmos_sensor(0xf8,0x89);//outdoor bgain bot   
                             
    //skin detect                                                                                    
    SP2529_write_cmos_sensor(0xfd,0x02);                                                                                     
    SP2529_write_cmos_sensor(0x08,0x00);                                                                                     
    SP2529_write_cmos_sensor(0x09,0x04);                                                                                     
                                                       
    SP2529_write_cmos_sensor(0xfd,0x02);                                                                                     
    SP2529_write_cmos_sensor(0xdd,0x0f); //raw smooth en                                                                      
    SP2529_write_cmos_sensor(0xde,0x0f); //sharpen en                                                                         
                                                       
    SP2529_write_cmos_sensor(0xfd,0x02);  // sharp                                                                            
    SP2529_write_cmos_sensor(0x57,0x1a);  //raw_sharp_y_base                                                                  
    SP2529_write_cmos_sensor(0x58,0x10);  //raw_sharp_y_min                                                                   
    SP2529_write_cmos_sensor(0x59,0xe0);  //raw_sharp_y_max                                                                   
    SP2529_write_cmos_sensor(0x5a,0x20);//30  //raw_sharp_rangek_neg                                                           
    SP2529_write_cmos_sensor(0x5b,0x20);  //raw_sharp_rangek_pos                                                              
                                                       
    SP2529_write_cmos_sensor(0xcb,0x10);//18	 //raw_sharp_range_base_outdoor	                                          
    SP2529_write_cmos_sensor(0xcc,0x10);//18	 //raw_sharp_range_base_nr 	                                          
    SP2529_write_cmos_sensor(0xcd,0x28);//18	 //raw_sharp_range_base_dummy	                                          
    SP2529_write_cmos_sensor(0xce,0x80);//18	 //raw_sharp_range_base_low	                                          
                                                            
    SP2529_write_cmos_sensor(0xfd,0x03);                                                                                     
    SP2529_write_cmos_sensor(0x87,0x0a);			//raw_sharp_range_ofst1	4x                                        
    SP2529_write_cmos_sensor(0x88,0x14);			//raw_sharp_range_ofst2	8x                                        
    SP2529_write_cmos_sensor(0x89,0x18);			//raw_sharp_range_ofst3	16x                                       
                                                        
    SP2529_write_cmos_sensor(0xfd,0x02);
    SP2529_write_cmos_sensor(0xe8,0x30);   //sharpness gain for increasing pixel’s Y, in outdoor                             
    SP2529_write_cmos_sensor(0xec,0x40);   //sharpness gain for decreasing pixel’s Y, in outdoor                             
    SP2529_write_cmos_sensor(0xe9,0x30);   //sharpness gain for increasing pixel’s Y, in normal                           
    SP2529_write_cmos_sensor(0xed,0x40);   //sharpness gain for decreasing pixel’s Y, in normal                           
    SP2529_write_cmos_sensor(0xea,0x28);   //sharpness gain for increasing pixel’s Y,in dummy                                
    SP2529_write_cmos_sensor(0xee,0x38);   //sharpness gain for decreasing pixel’s Y, in dummy                               
    SP2529_write_cmos_sensor(0xeb,0x20);   //sharpness gain for increasing pixel’s Y,in lowlight                             
    SP2529_write_cmos_sensor(0xef,0x30);   //sharpness gain for decreasing pixel’s Y, in low light                           
                                                           
    SP2529_write_cmos_sensor(0xfd,0x02);		//skin sharpen                                                             
    SP2529_write_cmos_sensor(0xdc,0x03);		//skin_sharp_sel肤色降锐化                                                 
    SP2529_write_cmos_sensor(0x05,0x00);		//skin_num_th2排除肤色降锐化对分辨率卡引起的干扰                           
                                                             
                                                            
    //平滑自适应                                                                                       
    SP2529_write_cmos_sensor(0xfd,0x02);                                                                                     
    SP2529_write_cmos_sensor(0xf4,0x30);  //raw_ymin                                                                          
    SP2529_write_cmos_sensor(0xfd,0x03);                                                                                     
    SP2529_write_cmos_sensor(0x97,0x80);  //raw_ymax_outdoor                                                                  
    SP2529_write_cmos_sensor(0x98,0x80);  //raw_ymax_normal                                                                   
    SP2529_write_cmos_sensor(0x99,0x80);  //raw_ymax_dummy                                                                    
    SP2529_write_cmos_sensor(0x9a,0x80);  //raw_ymax_low                                                                      
    SP2529_write_cmos_sensor(0xfd,0x02);                                                                                     
    SP2529_write_cmos_sensor(0xe4,0xff);//40 //raw_yk_fac_outdoor                                                              
    SP2529_write_cmos_sensor(0xe5,0xff);//40 //raw_yk_fac_normal                                                               
    SP2529_write_cmos_sensor(0xe6,0xff);//40 //raw_yk_fac_dummy                                                                
    SP2529_write_cmos_sensor(0xe7,0xff);//40 //raw_yk_fac_low                                                                  
                                                       
    SP2529_write_cmos_sensor(0xfd,0x03);                                                                                     
    SP2529_write_cmos_sensor(0x72,0x00);  //raw_lsc_fac_outdoor                                                               
    SP2529_write_cmos_sensor(0x73,0x2d);  //raw_lsc_fac_normal                                                                
    SP2529_write_cmos_sensor(0x74,0x2d);  //raw_lsc_fac_dummy                                                                 
    SP2529_write_cmos_sensor(0x75,0x2d);  //raw_lsc_fac_low                                                                   
                                                              
    //四个通道内阈值                                                                                   
    SP2529_write_cmos_sensor(0xfd,0x02); //raw_dif_thr                                                                        
    SP2529_write_cmos_sensor(0x78,0x20);//20 //raw_dif_thr_outdoor                                                                
    SP2529_write_cmos_sensor(0x79,0x20);//18//0a                                                                                  
    SP2529_write_cmos_sensor(0x7a,0x14);//10//10                                                                                  
    SP2529_write_cmos_sensor(0x7b,0x08);//08//20                                                                                  
                                                            
    //Gr、Gb通道銄兄                                                                                
    SP2529_write_cmos_sensor(0x81,0x40);//18 //10//raw_grgb_thr_outdoor                                                            
    SP2529_write_cmos_sensor(0x82,0x40);//10 //10                                                                                 
    SP2529_write_cmos_sensor(0x83,0x50);//08 //10                                                                                 
    SP2529_write_cmos_sensor(0x84,0x50);//08 //10                                                                                 
                                                          
    SP2529_write_cmos_sensor(0xfd,0x03);                                                                                     
    SP2529_write_cmos_sensor(0x7e,0x10);     //raw_noise_base_outdoor                                                         
    SP2529_write_cmos_sensor(0x7f,0x18);     //raw_noise_base_normal                                                          
    SP2529_write_cmos_sensor(0x80,0x20);     //raw_noise_base_dummy                                                           
    SP2529_write_cmos_sensor(0x81,0x30);     //raw_noise_base_low                                                             
    SP2529_write_cmos_sensor(0x7c,0xff);     //raw_noise_base_dark                                                            
    SP2529_write_cmos_sensor(0x82,0x44);     //raw_dns_fac_outdoor,raw_dns_fac_normal}                                       
    SP2529_write_cmos_sensor(0x83,0x22);     //raw_dns_fac_dummy,raw_dns_fac_low}                                            
    SP2529_write_cmos_sensor(0x84,0x08);			//raw_noise_ofst1 	4x                                
    SP2529_write_cmos_sensor(0x85,0x40);			//raw_noise_ofst2	8x                                
    SP2529_write_cmos_sensor(0x86,0x80); 		//raw_noise_ofst3	16x                                       
                                                          
    //去紫边功能                                                                                       
    SP2529_write_cmos_sensor(0xfd,0x03);                                                                                    
    SP2529_write_cmos_sensor(0x66,0x18); //pf_bg_thr_normal b-g>thr                                                          
    SP2529_write_cmos_sensor(0x67,0x28); //pf_rg_thr_normal r-g<thr                                                          
    SP2529_write_cmos_sensor(0x68,0x20); //pf_delta_thr_normal |val|>thr                                                     
    SP2529_write_cmos_sensor(0x69,0x88); //pf_k_fac val/16                                                                   
    SP2529_write_cmos_sensor(0x9b,0x18); //pf_bg_thr_outdoor                                                                 
    SP2529_write_cmos_sensor(0x9c,0x28); //pf_rg_thr_outdoor                                                                 
    SP2529_write_cmos_sensor(0x9d,0x20); //pf_delta_thr_outdoor                                                              
                                                              
    //Gamma                                                                                            
    SP2529_write_cmos_sensor(0xfd,0x01);//0x01//gamma                                                                  
    SP2529_write_cmos_sensor(0x8b,0x00);//0x00//                                                                    
    SP2529_write_cmos_sensor(0x8c,0x0f);//0x0e//                                                                    
    SP2529_write_cmos_sensor(0x8d,0x21);//0x21//                                                                    
    SP2529_write_cmos_sensor(0x8e,0x2c);//0x2c//                                                                    
    SP2529_write_cmos_sensor(0x8f,0x37);//0x37//                                                                    
    SP2529_write_cmos_sensor(0x90,0x46);//0x49//                                                                    
    SP2529_write_cmos_sensor(0x91,0x53);//0x5c//                                                                    
    SP2529_write_cmos_sensor(0x92,0x5e);//0x67//                                                                    
    SP2529_write_cmos_sensor(0x93,0x6a);//0x73//                                                                    
    SP2529_write_cmos_sensor(0x94,0x7d);//0x89//                                                                    
    SP2529_write_cmos_sensor(0x95,0x8d);//0x98//                                                                    
    SP2529_write_cmos_sensor(0x96,0x9e);//0xa8//                                                                    
    SP2529_write_cmos_sensor(0x97,0xac);//0xb5//                                                                    
    SP2529_write_cmos_sensor(0x98,0xba);//0xc1//                                                                    
    SP2529_write_cmos_sensor(0x99,0xc6);//0xcb//                                                                    
    SP2529_write_cmos_sensor(0x9a,0xd1);//0xd4//                                                                    
    SP2529_write_cmos_sensor(0x9b,0xda);//0xdc//                                                                    
    SP2529_write_cmos_sensor(0x9c,0xe4);//0xe4//                                                                    
    SP2529_write_cmos_sensor(0x9d,0xeb);//0xeb//                                                                    
    SP2529_write_cmos_sensor(0x9e,0xf2);//0xf2//                                                                    
    SP2529_write_cmos_sensor(0x9f,0xf9);//0xf9//                                                                    
    SP2529_write_cmos_sensor(0xa0,0xff);//0xff// 
                               
                                                           
    //CCM                                                                                            
    SP2529_write_cmos_sensor(0xfd,0x02);                                                                                     
    SP2529_write_cmos_sensor(0x15,0xC8);                                                                                     
    SP2529_write_cmos_sensor(0x16,0x95);                                                                                     
    //!F                                                                                             
    SP2529_write_cmos_sensor(0xa0,0x80);//8c                                                                         
    SP2529_write_cmos_sensor(0xa1,0xf4);//fa                                                                         
    SP2529_write_cmos_sensor(0xa2,0x0c);//fa                                                                         
    SP2529_write_cmos_sensor(0xa3,0xe7);//ed                                                                         
    SP2529_write_cmos_sensor(0xa4,0x99);//a6                                                                         
    SP2529_write_cmos_sensor(0xa5,0x00);//ed                                                                         
    SP2529_write_cmos_sensor(0xa6,0xf4);//e7                                                                         
    SP2529_write_cmos_sensor(0xa7,0xda);//f4                                                                         
    SP2529_write_cmos_sensor(0xa8,0xb3);//a6                                                                         
    SP2529_write_cmos_sensor(0xa9,0x0c);//3c                                                                         
    SP2529_write_cmos_sensor(0xaa,0x03);//33                                                                         
    SP2529_write_cmos_sensor(0xab,0x0f);//0f                                                                         
    //F                                                                                        
    SP2529_write_cmos_sensor(0xac,0x99);//0x8c                                                                                  
    SP2529_write_cmos_sensor(0xad,0xf4);//0xf4                                                                                  
    SP2529_write_cmos_sensor(0xae,0xf4);//0x00                                                                                  
    SP2529_write_cmos_sensor(0xaf,0xf4);//0xe7                                                                                  
    SP2529_write_cmos_sensor(0xb0,0x99);//0xc0                                                                                  
    SP2529_write_cmos_sensor(0xb1,0xf4);//0xda                                                                                  
    SP2529_write_cmos_sensor(0xb2,0xf4);//0xf4                                                                                  
    SP2529_write_cmos_sensor(0xb3,0xda);//0xcd                                                                                  
    SP2529_write_cmos_sensor(0xb4,0xb3);//0xc0                                                                                  
    SP2529_write_cmos_sensor(0xb5,0x3c);//0x0c                                                                                  
    SP2529_write_cmos_sensor(0xb6,0x33);//0x33                                                                                  
    SP2529_write_cmos_sensor(0xb7,0x0f);//0x0f                                                                                  
                                                            
    SP2529_write_cmos_sensor(0xfd,0x01);		//auto_sat                                                                 
    SP2529_write_cmos_sensor(0xd2,0x2f);		//autosa_en[0]                                                             
    SP2529_write_cmos_sensor(0xd1,0x38);		//lum thr in green enhance                                                 
    SP2529_write_cmos_sensor(0xdd,0x1b);                                                                                     
    SP2529_write_cmos_sensor(0xde,0x1b);   

                                               
    //auto sat                                                                                        
    SP2529_write_cmos_sensor(0xfd,0x02);                                                                                     
    SP2529_write_cmos_sensor(0xc1,0x40);                                                                                     
    SP2529_write_cmos_sensor(0xc2,0x40);                                                                                     
    SP2529_write_cmos_sensor(0xc3,0x40);                                                                                     
    SP2529_write_cmos_sensor(0xc4,0x40);                                                                                     
    SP2529_write_cmos_sensor(0xc5,0x80);                                                                                     
    SP2529_write_cmos_sensor(0xc6,0x80);                                                                                     
    SP2529_write_cmos_sensor(0xc7,0x80);                                                                                     
    SP2529_write_cmos_sensor(0xc8,0x80);                                                                                     
                                                             
    //sat u                                                                                           
    SP2529_write_cmos_sensor(0xfd,0x01);                                                                                     
    SP2529_write_cmos_sensor(0xd3,0x70);                                                                                     
    SP2529_write_cmos_sensor(0xd4,0x70);                                                                                     
    SP2529_write_cmos_sensor(0xd5,0x70);                                                                                     
    SP2529_write_cmos_sensor(0xd6,0x70);                                                                                     
    //sat v     7                                                                                     
    SP2529_write_cmos_sensor(0xd7,0x70);                                                                                    
    SP2529_write_cmos_sensor(0xd8,0x70);                                                                                    
    SP2529_write_cmos_sensor(0xd9,0x70);                                                                                    
    SP2529_write_cmos_sensor(0xda,0x70);                                                                                    
                                                    
    SP2529_write_cmos_sensor(0xfd,0x03);                                                                                     
    SP2529_write_cmos_sensor(0x76,0x10);                                                                                     
    SP2529_write_cmos_sensor(0x7a,0x40);                                                                                     
    SP2529_write_cmos_sensor(0x7b,0x40);                                                                                     
    //auto_sat                                                                                     
    SP2529_write_cmos_sensor(0xfd,0x01);                                                                                     
    SP2529_write_cmos_sensor(0xc2,0xff);//aa //u_v_th_outdoor白色物体表面有彩色噪声降低此值                                       
    SP2529_write_cmos_sensor(0xc3,0xff);//aa //u_v_th_nr                                                                          
    SP2529_write_cmos_sensor(0xc4,0xaa);//44 //u_v_th_dummy                                                                       
    SP2529_write_cmos_sensor(0xc5,0xaa);//44 //u_v_th_low                                                                         
                                                            
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
    //low_lum_offset                                                                                   
    SP2529_write_cmos_sensor(0xfd,0x01);                                                                                     
    SP2529_write_cmos_sensor(0xcd,0x09);                                                                                    
    SP2529_write_cmos_sensor(0xce,0x10);                                                                                     
                                                           
    //gw                                                                                            
    SP2529_write_cmos_sensor(0xfd,0x02);                                                                                     
    SP2529_write_cmos_sensor(0x35,0x6f);//uv_fix_dat                                                                          
    SP2529_write_cmos_sensor(0x37,0x13);                                                                                     
                                                             
    //heq                                                                                             
                                                             
    SP2529_write_cmos_sensor(0xfd,0x01);                                                                                     
    SP2529_write_cmos_sensor(0xdb,0x00);//buf_heq_offset                                                                      
                                                           
    SP2529_write_cmos_sensor(0x10,0x80);//ku_outdoor                                                                          
    SP2529_write_cmos_sensor(0x11,0x80);//ku_nr
    SP2529_write_cmos_sensor(0x12,0x80);//ku_dummy
    SP2529_write_cmos_sensor(0x13,0x80);//ku_low 
    SP2529_write_cmos_sensor(0x14,0xa0);//kl_outdoor                                                                              
    SP2529_write_cmos_sensor(0x15,0xa0);//kl_nr                                                                               
    SP2529_write_cmos_sensor(0x16,0xa0);//kl_dummy                                                                            
    SP2529_write_cmos_sensor(0x17,0xa0);//kl_low                                                                                           
                                                          
    SP2529_write_cmos_sensor(0xfd,0x03);                                                                                     
    SP2529_write_cmos_sensor(0x00,0x80);  //ctf_heq_mean	                                                                  
    SP2529_write_cmos_sensor(0x03,0x30);  //ctf_range_thr   可以排除灰板场景的阈值                                            
    SP2529_write_cmos_sensor(0x06,0xf0);  //ctf_reg_max	                                                                  
    SP2529_write_cmos_sensor(0x07,0x08);  //ctf_reg_min	                                                                  
    SP2529_write_cmos_sensor(0x0a,0x00);  //ctf_lum_ofst                                                                      
    SP2529_write_cmos_sensor(0x01,0x00);  //ctf_posk_fac_outdoor                                                              
    SP2529_write_cmos_sensor(0x02,0x00);  //ctf_posk_fac_nr                                                                   
    SP2529_write_cmos_sensor(0x04,0x00);  //ctf_posk_fac_dummy                                                                
    SP2529_write_cmos_sensor(0x05,0x00);  //ctf_posk_fac_low                                                                  
    SP2529_write_cmos_sensor(0x0b,0x00);  //ctf_negk_fac_outdoor                                                              
    SP2529_write_cmos_sensor(0x0c,0x00);  //ctf_negk_fac_nr                                                                   
    SP2529_write_cmos_sensor(0x0d,0x00);  //ctf_negk_fac_dummy                                                                
    SP2529_write_cmos_sensor(0x0e,0x00);  //ctf_negk_fac_low
    SP2529_write_cmos_sensor(0x08,0x10);                                                                  
    SP2529_write_cmos_sensor(0x09,0x10);                                                                                                 
    //CNR                               
    SP2529_write_cmos_sensor(0xfd,0x02); //cnr
    SP2529_write_cmos_sensor(0x8e,0x0a); //cnr_uv_grad_thr                
    SP2529_write_cmos_sensor(0x8f,0x03); //[0]0 vertical,1 horizontal
    SP2529_write_cmos_sensor(0x90,0x40); //cnrH_thr_outdoor   
    SP2529_write_cmos_sensor(0x91,0x40); //cnrH_thr_nr        
    SP2529_write_cmos_sensor(0x92,0x60); //cnrH_thr_dummy     
    SP2529_write_cmos_sensor(0x93,0x80); //cnrH_thr_low 
    SP2529_write_cmos_sensor(0x94,0x80); //cnrV_thr_outdoor
    SP2529_write_cmos_sensor(0x95,0x80); //cnrV_thr_nr     
    SP2529_write_cmos_sensor(0x96,0x80); //cnrV_thr_dummy  
    SP2529_write_cmos_sensor(0x97,0x80); //cnrV_thr_low

    SP2529_write_cmos_sensor(0x98,0x80); //cnr_grad_thr_outdoor                  
    SP2529_write_cmos_sensor(0x99,0x80); //cnr_grad_thr_nr                  
    SP2529_write_cmos_sensor(0x9a,0x80); //cnr_grad_thr_dummy                  
    SP2529_write_cmos_sensor(0x9b,0x80); //cnr_grad_thr_low                  

    SP2529_write_cmos_sensor(0x9e,0x44);                    
    SP2529_write_cmos_sensor(0x9f,0x44);                    
                                                             
    SP2529_write_cmos_sensor(0xfd,0x02);	//auto                                                                             
    SP2529_write_cmos_sensor(0x85,0x00);	//enable 50Hz/60Hz function[4]  [3:0] interval_line                                
    SP2529_write_cmos_sensor(0xfd,0x01);                                                                                     
    SP2529_write_cmos_sensor(0x00,0x00); 	//fix mode                                                                         
    SP2529_write_cmos_sensor(0x32,0x15);//	//ae en                                                                            
    SP2529_write_cmos_sensor(0x33,0xef);//ef		//lsc\bpc en                                                       
    SP2529_write_cmos_sensor(0x34,0xef);		//ynr[4]\cnr[0]\gamma[2]\colo[1]                                           
    SP2529_write_cmos_sensor(0x35 , 0x41);//		;YUYV   																														
    SP2529_write_cmos_sensor(0xfd,0x00);		                                                                                                                                             
    SP2529_write_cmos_sensor(0x3f , 0x00);// 	;mirror/flip                          																														
                                                        
    SP2529_write_cmos_sensor(0xfd,0x01);                                                                                     
    SP2529_write_cmos_sensor(0x50,0x00);   //heq_auto_mode 读状态                                                             
    SP2529_write_cmos_sensor(0x66,0x00);		//effect 
    SP2529_write_cmos_sensor(0xfd,0x02); 
    SP2529_write_cmos_sensor(0xd6,0x0f); 
    SP2529_write_cmos_sensor(0xfd,0x00);
    SP2529_write_cmos_sensor(0x92,0x81);
    SP2529_write_cmos_sensor(0x98,0x39);
    SP2529_write_cmos_sensor(0xfd,0x00);
    SP2529_write_cmos_sensor(0x1d,0x01);
    SP2529_write_cmos_sensor(0xfd,0x01);
    SP2529_write_cmos_sensor(0x36,0x00);    

    //MIPI																									
    SP2529_write_cmos_sensor(0xfd , 0x00);																														
    SP2529_write_cmos_sensor(0x95 , 0x03);																														
    SP2529_write_cmos_sensor(0x94 , 0x20);		   																												
    SP2529_write_cmos_sensor(0x97 , 0x02);																														
    SP2529_write_cmos_sensor(0x96 , 0x58);																														

}
  
void SP2529_config_window(kal_uint8 index)
{
	SENSORDB(" SP2529_config_window,index = %d\r\n",index);
	switch(index)
	{
		case WINDOW_SIZE_UXGA:
			#if 1
			/*1600x1200 this is for resize para*/
			//uxga 
			SP2529_write_cmos_sensor(0xfd,0x00); 
			SP2529_write_cmos_sensor(0x19,0x00);
			SP2529_write_cmos_sensor(0x30,0x00);//00 
			SP2529_write_cmos_sensor(0x31,0x00); 
			SP2529_write_cmos_sensor(0x33,0x00);
			//MIPI      
			SP2529_write_cmos_sensor(0xfd,0x00); 
			SP2529_write_cmos_sensor(0x95,0x06);
			SP2529_write_cmos_sensor(0x94,0x40); 
			SP2529_write_cmos_sensor(0x97,0x04); 
			SP2529_write_cmos_sensor(0x96,0xb0);
			#endif
			break;
		case WINDOW_SIZE_720P:
	
			/**1280*720  this is for crop para*/
	
			break;
		case WINDOW_SIZE_SVGA:
			#if 1
			/*SVGA this is for resize para*/	
			//binning svga
			SP2529_write_cmos_sensor(0xfd , 0x00);	
			SP2529_write_cmos_sensor(0x19 , 0x03);
			SP2529_write_cmos_sensor(0x30 , 0x00);
			SP2529_write_cmos_sensor(0x31 , 0x04);
			SP2529_write_cmos_sensor(0x33 , 0x01);
			SP2529_write_cmos_sensor(0xfd , 0x00);																														
			SP2529_write_cmos_sensor(0x95 , 0x03);																														
			SP2529_write_cmos_sensor(0x94 , 0x20);																														
			SP2529_write_cmos_sensor(0x97 , 0x02);																														
			SP2529_write_cmos_sensor(0x96 , 0x58);
			#endif
			break;
		case WINDOW_SIZE_VGA:
		
			/*VGA this is for resize para*/							 
		
			break;
		default:
			break;
	}
}

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	SP2529Open
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
UINT32 SP2529Open(void)
{
	kal_uint16 sensor_id=0;
	int i;
    
	SENSORDB("Ronlus SP2529Open\r\n");
	// check if sensor ID correct
	for(i = 0; i < 3; i++)
	{
		SP2529_write_cmos_sensor(0xfd,0x00);
		sensor_id = SP2529_read_cmos_sensor(0x02);
		SENSORDB("Ronlus SP2529 Sensor id = %x\n", sensor_id);
		if (sensor_id == SP2529_SENSOR_ID)
		{
			break;
		}
	}
	mdelay(50);
	if(sensor_id != SP2529_SENSOR_ID)
	{
		SENSORDB("SP2529 Sensor id read failed, ID = %x\n", sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}
#ifdef DEBUG_SENSOR_SP2529  //gepeiwei   120903
	//?D??¨o??¨2??¨?|??????¨o??¤?¨?D???asp2528_sd |ì????t,??¨?D??¨¨?2?¨oy

	//?¨|¨?¨2??????-¨°¨°?ê??à?????à?3?¨o???￥2?¨oy?¨2_s_fmt?D??ê
	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static char buf[10*1024] ;

	fp = filp_open("/mnt/sdcard/sp2529_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) { 
		fromsd = 0;   
		printk("open file error\n");

	} 
	else 
	{
		fromsd = 1;
		printk("open file ok\n");

		//SP2529_Initialize_from_T_Flash();


		filp_close(fp, NULL); 
		set_fs(fs);
	}

	if(fromsd == 1)//¨o??¤???¨?SD?¨￠¨¨?//gepeiwei   120903
	{
		printk("________________from t!\n");
		SP2529_Initialize_from_T_Flash();//??¨?SD??§?¨￠¨¨?|ì????¨°ao?￥¨oy
	}
	else
	{
		SP2529_Sensor_Init();
		//SP2529_Write_More_Registers();//added for FAE to debut
	}
#else  
	SP2529_Sensor_Init();


#endif

	SENSORDB("SP2529 Sensor Read ID OK \r\n");
	//SP2529_Sensor_Init();
	
	return ERROR_NONE;
}	/* SP2529Open() */

/*************************************************************************
* FUNCTION
*	SP2529Close
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
UINT32 SP2529Close(void)
{
//	CISModulePowerOn(FALSE);
	return ERROR_NONE;
}	/* SP2529Close() */

LOCAL void SP2529_AfterSnapshot(void)
{
    kal_uint8  	AE_tmp;
	#if 1
	kal_uint8   tmp1,tmp2,tmp3;
	#endif
	

	SP2529_write_cmos_sensor(0xfd,0x01);
	AE_tmp=SP2529_read_cmos_sensor(0x32);
	AE_tmp=AE_tmp&0xfa;
	
	SP2529_write_cmos_sensor(0x32,AE_tmp);
	SP2529_write_cmos_sensor(0xfd,0x00);
	SP2529_write_cmos_sensor(0xe7,0x03);
	SP2529_write_cmos_sensor(0xe7,0x00);
	
	SP2529_Set_Shutter(G_shutter);
	
	SP2529_write_cmos_sensor(0xfd,0x00); 
	SP2529_write_cmos_sensor(0x24,G_Gain);
	
	SP2529_write_cmos_sensor(0xfd,0x00);
	SP2529_write_cmos_sensor(0xe7,0x03);
	SP2529_write_cmos_sensor(0xe7,0x00);

	SP2529_write_cmos_sensor(0xfd,0x01);
	AE_tmp=SP2529_read_cmos_sensor(0x32);
	AE_tmp=AE_tmp|0x05;
	SP2529_write_cmos_sensor(0x32,AE_tmp);
	SP2529_write_cmos_sensor(0xfd,0x00);
	SP2529_write_cmos_sensor(0xe7,0x03);
	SP2529_write_cmos_sensor(0xe7,0x00);

	return 0;
}  

LOCAL void SP2529_BeforeSnapshot(void)
{	
	kal_uint32 Shutter_temp;
    kal_uint16 Shutter, HB;
    kal_uint8  	HB_L,HB_H,Gain,AE_tmp;

	SP2529_write_cmos_sensor(0xfd,0x01);
	AE_tmp=SP2529_read_cmos_sensor(0x32);
	SP2529_write_cmos_sensor(0xfd,0x00); 
	Gain=SP2529_read_cmos_sensor(0x23);

	HB_H=SP2529_read_cmos_sensor(0x09);
	HB_L=SP2529_read_cmos_sensor(0x0a);

	G_Gain=Gain;
	HB = (HB_L & 0xFF) | (HB_H << 8);
	AE_tmp=AE_tmp&0xfa;
	Shutter = SP2529_Read_Shutter();
	G_shutter = Shutter;
	
	Shutter_temp = Shutter;	
	Shutter_temp = Shutter_temp*(517+HB)/2/(922+HB);	
	Shutter = Shutter_temp&0xffff;

	if(Shutter<1)
	{
		Shutter=1;
	}
	SP2529_write_cmos_sensor(0xfd,0x01);
	SP2529_write_cmos_sensor(0x32,AE_tmp);  
	SP2529_write_cmos_sensor(0xfd,0x00);
	SP2529_write_cmos_sensor(0xe7,0x03);
	SP2529_write_cmos_sensor(0xe7,0x00);
	
	SP2529_Set_Shutter(Shutter);	
	SP2529_write_cmos_sensor(0xfd,0x00); 
	SP2529_write_cmos_sensor(0x24,Gain);
	
	SP2529_write_cmos_sensor(0xfd,0x00);
	SP2529_write_cmos_sensor(0xe7,0x03);
	SP2529_write_cmos_sensor(0xe7,0x00);

return 0;  
} 

/*************************************************************************
* FUNCTION
*	SP2529Preview
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
UINT32 SP2529Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 iTemp;
	kal_uint16 iStartX = 0, iStartY = 1;
    kal_uint8  	AE_tmp;
	SENSORDB(" SP2529Preview fun start\r\n");

	//SP2529_PV_setting();

	SP2529_config_window(WINDOW_SIZE_SVGA);

    if(setshutter)
    {
	    SP2529_AfterSnapshot();  //xg test
    }
	setshutter = KAL_FALSE;
	if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
	{
		SENSORDB("Ronlus video preview\r\n");
		SP2529_MPEG4_encode_mode = KAL_TRUE;		
	}
	else
	{
		SENSORDB("Ronlus capture preview\r\n");
		SP2529_MPEG4_encode_mode = KAL_FALSE;
	}
	image_window->GrabStartX= IMAGE_SENSOR_START_GRAB_X;
	image_window->GrabStartY= IMAGE_SENSOR_START_GRAB_Y;
	image_window->ExposureWindowWidth = SP2529_IMAGE_SENSOR_PV_WIDTH; //IMAGE_SENSOR_FULL_WIDTH
	image_window->ExposureWindowHeight =SP2529_IMAGE_SENSOR_PV_HEIGHT;//IMAGE_SENSOR_FULL_HEIGHT
	// copy sensor_config_data
	memcpy(&SP2529SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));//rotation
	SENSORDB("Ronlus SP2529Preview fun end\r\n");
	return ERROR_NONE;
}	/* SP2529Preview() */




UINT32 SP2529Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	SENSORDB("Ronlus SP2529Capture fun start\r\n");
	SP2529_CAPTURE_MODE=KAL_TRUE;

	// SP2529_CAP_setting();
	 SP2529_config_window(WINDOW_SIZE_UXGA);
	 SP2529_BeforeSnapshot(); 
	 msleep(10);
#if 0
	if ((image_window->ImageTargetWidth<=IMAGE_SENSOR_SVGA_WIDTH)&&
			(image_window->ImageTargetHeight<=IMAGE_SENSOR_SVGA_HEIGHT))
	{    /* Less than PV Mode */
		image_window->GrabStartX = IMAGE_SENSOR_SVGA_GRAB_PIXELS;
		image_window->GrabStartY = IMAGE_SENSOR_SVGA_GRAB_LINES;
		image_window->ExposureWindowWidth= IMAGE_SENSOR_PV_WIDTH;
		image_window->ExposureWindowHeight = IMAGE_SENSOR_PV_HEIGHT;
	}
	else
#endif

	{
		kal_uint32 shutter, cap_dummy_pixels = 0; 
		if(!setshutter)
		{
			//SP2529_ae_enable(KAL_FALSE);
			//SP2529_awb_enable(KAL_FALSE);
			//shutter = SP2529_Read_Shutter();
			//SP2529_CAP_setting();
			//SP2529_set_hb_shutter(cap_dummy_pixels, shutter);
		}	
	 image_window->GrabStartX=1;
        image_window->GrabStartY=1;
        image_window->ExposureWindowWidth=SP2529_IMAGE_SENSOR_FULL_WIDTH - image_window->GrabStartX - 2;
        image_window->ExposureWindowHeight=SP2529_IMAGE_SENSOR_FULL_HEIGHT -image_window->GrabStartY - 2;    	 
	}
	setshutter = KAL_TRUE;
	// copy sensor_config_data
	memcpy(&SP2529SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	SENSORDB("Ronlus SP2529Capture fun end\r\n");
	return ERROR_NONE;
}	/* SP2529Capture() */

  
UINT32 SP2529GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=SP2529_IMAGE_SENSOR_FULL_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorFullHeight=SP2529_IMAGE_SENSOR_FULL_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	pSensorResolution->SensorPreviewWidth=SP2529_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorPreviewHeight=SP2529_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	pSensorResolution->SensorVideoWidth=SP2529_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	pSensorResolution->SensorVideoHeight=SP2529_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	return ERROR_NONE;
}	/* SP2529GetResolution() */

UINT32 SP2529GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=SP2529_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=SP2529_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX=SP2529_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=SP2529_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	/*??? */
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->CaptureDelayFrame = 4; 
	pSensorInfo->PreviewDelayFrame = 1; 
	pSensorInfo->VideoDelayFrame = 0; 
       pSensorInfo->YUVAwbDelayFrame = 2;  // add by lanking
	pSensorInfo->YUVEffectDelayFrame = 2;  // add by lanking
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_6MA;


	pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_MIPI;

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
                     pSensorInfo->SensorGrabStartX = 2; 
                     pSensorInfo->SensorGrabStartY = 2;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;

	
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
                     pSensorInfo->SensorGrabStartX = 2; 
                     pSensorInfo->SensorGrabStartY = 2;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;

			
		break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
                     pSensorInfo->SensorGrabStartX = 2; 
                     pSensorInfo->SensorGrabStartY = 2;               
			
		break;
	}
	memcpy(pSensorConfigData, &SP2529SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* SP2529GetInfo() */


UINT32 SP2529Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			SP2529Preview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			SP2529Capture(pImageWindow, pSensorConfigData);
		break;
		default:
		    break; 
	}
	return TRUE;
}	/* SP2529Control() */

BOOL SP2529_set_param_wb(UINT16 para)
{
	switch (para)
	{
		case AWB_MODE_AUTO:
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x26,0xac);
			SP2529_write_cmos_sensor(0x27,0x91);	   
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x32,0x15);
		break;
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x32,0x05);
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x26,0xe2);
			SP2529_write_cmos_sensor(0x27,0x82);
			SP2529_write_cmos_sensor(0xfd,0x00);
		break;
		case AWB_MODE_DAYLIGHT: //sunny
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x32,0x05);
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x26,0xc1);
			SP2529_write_cmos_sensor(0x27,0x88);
			SP2529_write_cmos_sensor(0xfd,0x00);
		break;
		case AWB_MODE_INCANDESCENT: //office
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x32,0x05);
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x26,0x7b);
			SP2529_write_cmos_sensor(0x27,0xd3);
			SP2529_write_cmos_sensor(0xfd,0x00);
		break;
		case AWB_MODE_TUNGSTEN: //home
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x32,0x05);
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x26,0xae);
			SP2529_write_cmos_sensor(0x27,0xcc);
			SP2529_write_cmos_sensor(0xfd,0x00);
		break;
		case AWB_MODE_FLUORESCENT:
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x32,0x05);
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x26,0xb4);
			SP2529_write_cmos_sensor(0x27,0xc4);
			SP2529_write_cmos_sensor(0xfd,0x00);
		break;	
		default:
		return FALSE;
	}
	return TRUE;
} /* SP2529_set_param_wb */

BOOL SP2529_set_param_effect(UINT16 para)
{
	kal_uint32 ret = KAL_TRUE;
	SP2529_write_cmos_sensor(0xfd,0x01);
	SP2529_write_cmos_sensor(0x36,0x02);
	switch (para)
	{
		case MEFFECT_OFF:
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x66,0x00);
			SP2529_write_cmos_sensor(0x67,0x80);
			SP2529_write_cmos_sensor(0x68,0x80);
			SP2529_write_cmos_sensor(0xdb,0x00);
			SP2529_write_cmos_sensor(0x34,0xc7);
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x14,0x00);
		break;

		case MEFFECT_SEPIA:
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x66,0x10);
			SP2529_write_cmos_sensor(0x67,0x98);
			SP2529_write_cmos_sensor(0x68,0x58);
			SP2529_write_cmos_sensor(0xdb,0x00);
			SP2529_write_cmos_sensor(0x34,0xc7);
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x14,0x00);
		break;  

		case MEFFECT_NEGATIVE:		
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x66,0x08);
			SP2529_write_cmos_sensor(0x67,0x80);
			SP2529_write_cmos_sensor(0x68,0x80);
			SP2529_write_cmos_sensor(0xdb,0x00);
			SP2529_write_cmos_sensor(0x34,0xc7);
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x14,0x00);
		break; 

		case MEFFECT_SEPIAGREEN:		
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x66,0x10);
			SP2529_write_cmos_sensor(0x67,0x50);
			SP2529_write_cmos_sensor(0x68,0x50);
			SP2529_write_cmos_sensor(0xdb,0x00);
			SP2529_write_cmos_sensor(0x34,0xc7);
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x14,0x00);
		break;

		case MEFFECT_SEPIABLUE:	
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x66,0x10);
			SP2529_write_cmos_sensor(0x67,0x80);
			SP2529_write_cmos_sensor(0x68,0xb0);
			SP2529_write_cmos_sensor(0xdb,0x00);
			SP2529_write_cmos_sensor(0x34,0xc7);
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x14,0x00);
		break;

		case MEFFECT_MONO:				
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0x66,0x20);
			SP2529_write_cmos_sensor(0x67,0x80);
			SP2529_write_cmos_sensor(0x68,0x80);
			SP2529_write_cmos_sensor(0xdb,0x00);
			SP2529_write_cmos_sensor(0x34,0xc7);
			SP2529_write_cmos_sensor(0xfd,0x02);
			SP2529_write_cmos_sensor(0x14,0x00);
		break;

		default:
		return FALSE;
	}
	SP2529_write_cmos_sensor(0xfd,0x00);
	SP2529_write_cmos_sensor(0xe7,0x03);
	SP2529_write_cmos_sensor(0xe7,0x00);
	mdelay(200); 
	SP2529_write_cmos_sensor(0xfd,0x01);
	SP2529_write_cmos_sensor(0x36,0x00);
	return ret;
} /* SP2529_set_param_effect */

BOOL SP2529_set_param_banding(UINT16 para)
{
	SENSORDB(" SP2529_set_param_banding\r\n");
	//SENSORDB("Ronlus SP2529_set_param_banding para = %d ---- index = %d\r\n",para,index); 
	SENSORDB("Ronlus SP2529_set_param_banding ---- SP2529_MPEG4_encode_mode = %d\r\n",SP2529_MPEG4_encode_mode);
  
    switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
		
		SENSORDB(" SP2529_set_param_banding 50hz\r\n");

		SP2529_CAM_BANDING_50HZ = KAL_TRUE;

            break;

        case AE_FLICKER_MODE_60HZ:
		
		SENSORDB(" SP2529_set_param_banding 60hz\r\n");

		SP2529_CAM_BANDING_50HZ = KAL_FALSE;
	break;
  
          default:
		
		SENSORDB(" SP2529_set_param_banding default\r\n");

		SP2529_CAM_BANDING_50HZ = KAL_TRUE; 
	//return FALSE;
	  break;
    }
 
    return TRUE;
} /* SP2529_set_param_banding */

BOOL SP2529_set_param_exposure(UINT16 para)
{
	SENSORDB("Ronlus SP2529_set_param_exposure\r\n");
	switch (para)
	{
		case AE_EV_COMP_n13:              /* EV -2 */
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0xdb,0xe0);
			break;
		case AE_EV_COMP_n10:              /* EV -1.5 */
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0xdb,0xe8);
			break;
		case AE_EV_COMP_n07:              /* EV -1 */
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0xdb,0xf0);
			break;
		case AE_EV_COMP_n03:              /* EV -0.5 */
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0xdb,0xf8);
			break;
		case AE_EV_COMP_00:                /* EV 0 */
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0xdb,0x00);
			break;
		case AE_EV_COMP_03:              /* EV +0.5 */
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0xdb,0x08);
			break;
		case AE_EV_COMP_07:              /* EV +1 */
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0xdb,0x10);
			break;
		case AE_EV_COMP_10:              /* EV +1.5 */
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0xdb,0x18);
			break;
		case AE_EV_COMP_13:              /* EV +2 */
			SP2529_write_cmos_sensor(0xfd,0x01);
			SP2529_write_cmos_sensor(0xdb,0x20);
			break;
		default:
			return FALSE;
	}
	return TRUE;
} /* SP2529_set_param_exposure */

UINT32 SP2529YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
#ifdef DEBUG_SENSOR_SP2529
	return TRUE;
#endif

	switch (iCmd) {
	case FID_SCENE_MODE:	    
//	    printk("Set Scene Mode:%d\n", iPara); 
	    if (iPara == SCENE_MODE_OFF)
	    {
	        SP2529_night_mode(0); 
	    }
	    else if (iPara == SCENE_MODE_NIGHTSCENE)
	    {
               SP2529_night_mode(1); 
	    }	    
	    break; 	    
	case FID_AWB_MODE:
//	    printk("Set AWB Mode:%d\n", iPara); 	    
           SP2529_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
//	    printk("Set Color Effect:%d\n", iPara); 	    	    
           SP2529_set_param_effect(iPara);
	break;
	case FID_AE_EV:
//           printk("Set EV:%d\n", iPara); 	    	    
           SP2529_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
//           printk("Set Flicker:%d\n", iPara); 	    	    	    
            SP2529_set_param_banding(iPara);
	    SP2529_night_mode(SP2529_CAM_Nightmode); 

	break;
/*
	case FID_AE_SCENE_MODE: 
            if (iPara == AE_MODE_OFF) {
                SP2529_set_AE_mode(KAL_FALSE);
            }
            else {
                SP2529_set_AE_mode(KAL_TRUE);
	    }
            break; 
	case FID_ZOOM_FACTOR:
	    zoom_factor = iPara; 
        break; 
        */
	default:
	break;
	}
	return TRUE;
}   /* SP2529YUVSensorSetting */

UINT32 SP2529YUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 iTemp;
    /* to fix VSYNC, to fix frame rate */
    //printk("Set YUV Video Mode \n");  

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
    SP2529_VEDIO_encode_mode = KAL_TRUE; 
        
    return TRUE;
}

UINT32 SP2529FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=SP2529_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=SP2529_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=SP2529_IMAGE_SENSOR_PV_WIDTH;
			*pFeatureReturnPara16=SP2529_IMAGE_SENSOR_PV_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			//*pFeatureReturnPara32 = SP2529_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
#ifndef DEBUG_SENSOR_SP2529		
			SP2529_night_mode((BOOL) *pFeatureData16);
#endif
		break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			SP2529_isp_master_clock=*pFeatureData32;
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			SP2529_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = SP2529_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &SP2529SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
			 SP2529_GetSensorID(pFeatureData32);
			 break;
		case SENSOR_FEATURE_SET_YUV_CMD:
		       //printk("SP2529 YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			SP2529YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		       SP2529YUVSetVideoMode(*pFeatureData16);
		       break; 
		default:
			break;			
	}
	return ERROR_NONE;
}	/* SP2529FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncSP2529=
{
	SP2529Open,
	SP2529GetInfo,
	SP2529GetResolution,
	SP2529FeatureControl,
	SP2529Control,
	SP2529Close
};

UINT32 SP2529_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncSP2529;

	return ERROR_NONE;
}	/* SensorInit() */

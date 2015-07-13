/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------

 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------

 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 *
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
#include <linux/xlog.h>

#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"
   
#include "sp2529_mipi_yuv_Sensor.h"
#include "sp2529_mipi_yuv_Camera_Sensor_para.h"
#include "sp2529_mipi_yuv_CameraCustomized.h"

#define SP2529MIPI_DEBUG
#ifdef SP2529MIPI_DEBUG
#define SENSORDB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[SP2529MIPI]", fmt, ##arg)

#else
#define SENSORDB(x,...)
#endif
  
//#define DEBUGE_SENSOR_SP0A28// add by zhaofei - 2013-10-08-15-47 for t-card debug
#ifdef DEBUGE_SENSOR_SP0A28 
	UINT32 fromsd;//SP0A28 FOR  T FLSAH
#endif

typedef struct
{
  UINT16  iSensorVersion;
  UINT16  iNightMode;
  UINT16  iWB;
  UINT16  iEffect;
  UINT16  iEV;
  UINT16  iBanding;
  UINT16  iMirror;
  UINT16  iFrameRate;
} SP2529MIPIStatus;

SP2529MIPIStatus SP2529MIPICurrentStatus;
static DEFINE_SPINLOCK(ov9740mipi_yuv_drv_lock);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//static int sensor_id_fail = 0; 
static kal_uint32 zoom_factor = 0; 
static UINT16 G_shutter; // add by zhaofei - 2013-10-09-14-34
static UINT8 G_Gain;

static kal_uint8 SP2529MIPI_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;

    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), SP2529MIPI_WRITE_ID)) {
        SENSORDB("ERROR: GC2235MIPI_read_cmos_sensor \n");
    }
	 printk("SP2529MIPI_read_cmos_sensor, addr:%x;get_byte:%x \n",addr,in_buff[0]);
  return in_buff[0];
}
/*
kal_uint8 SP2529MIPI_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint8 get_byte=0;
	char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,SP2529MIPI_WRITE_ID);

	 SENSORDB("SP2529MIPI_read_cmos_sensor, addr:%x;get_byte:%x \n",addr,get_byte);
	
	return get_byte;
}
 inline void SP2529MIPI_write_cmos_sensor(u16 addr, u32 para)
{
   char puSendCmd[3] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
   iWriteRegI2C(puSendCmd , 3,SP2529MIPI_WRITE_ID);
    //SENSORDB("SP2529MIPI_write_cmos_sensor, addr:%x; para:%x \n",addr,para);
} 
	
*/
static void SP2529MIPI_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), SP2529MIPI_WRITE_ID); 
}

#ifdef DEBUGE_SENSOR_SP0A28
/******SP0A28 FOR  T FLSAH**********/
	#define SP0A28_OP_CODE_INI		0x00		/* Initial value. */
	#define SP0A28_OP_CODE_REG		0x01		/* Register */
	#define SP0A28_OP_CODE_DLY		0x02		/* Delay */
	#define SP0A28_OP_CODE_END		0x03		/* End of initial setting. */
	

		typedef struct
	{
		u16 init_reg;
		u16 init_val;	/* Save the register value and delay tick */
		u8 op_code;		/* 0 - Initial value, 1 - Register, 2 - Delay, 3 - End of setting. */
	} SP0A28_initial_set_struct;

	SP0A28_initial_set_struct SP0A28_Init_Reg[1000];
	
 u32 strtol_1(const char *nptr, u8 base)
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

 u8 SP0A28_Initialize_from_T_Flash()
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
 
    fp = filp_open("/storage/sdcard0/sp2529_sd", O_RDONLY , 0); 
    if (IS_ERR(fp)) { 
        printk("create file from shouji error\n"); 
        fp = filp_open("/storage/sdcard1/sp2529_sd", O_RDONLY , 0); 
        if (IS_ERR(fp))
        {
        	printk("create file from T error\n"); 	
        	return -1; 
        }
        
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
			SP0A28_Init_Reg[i].op_code = SP0A28_OP_CODE_REG;
			
			SP0A28_Init_Reg[i].init_reg = strtol_1((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */
		
			SP0A28_Init_Reg[i].init_val = strtol_1((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */
		
		}
		else									/* DLY */
		{
			/* Need add delay for this setting. */
			curr_ptr += 4;	
			SP0A28_Init_Reg[i].op_code = SP0A28_OP_CODE_DLY;
			
			SP0A28_Init_Reg[i].init_reg = 0xFF;
			SP0A28_Init_Reg[i].init_val = strtol_1((const char *)curr_ptr,  10);	/* Get the delay ticks, the delay should less then 50 */
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
	SP0A28_Init_Reg[i].op_code = SP0A28_OP_CODE_END;
	SP0A28_Init_Reg[i].init_reg = 0xFF;
	SP0A28_Init_Reg[i].init_val = 0xFF;
	i++;
	//for (j=0; j<i; j++)
		//printk(" %x  ==  %x\n",SP0A28_Init_Reg[j].init_reg, SP0A28_Init_Reg[j].init_val);

	/* Start apply the initial setting to sensor. */
	#if 1
	for (j=0; j<i; j++)
	{
		if (SP0A28_Init_Reg[j].op_code == SP0A28_OP_CODE_END)	/* End of the setting. */
		{
			break ;
		}
		else if (SP0A28_Init_Reg[j].op_code == SP0A28_OP_CODE_DLY)
		{
			msleep(SP0A28_Init_Reg[j].init_val);		/* Delay */
		}
		else if (SP0A28_Init_Reg[j].op_code == SP0A28_OP_CODE_REG)
		{
		
			SP2529MIPI_write_cmos_sensor(SP0A28_Init_Reg[j].init_reg, SP0A28_Init_Reg[j].init_val);
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
/******SP0A28 FOR  T FLSAH**********/
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
/* Global Valuable */

kal_bool SP2529MIPI_MPEG4_encode_mode = KAL_FALSE, SP2529MIPI_MJPEG_encode_mode = KAL_FALSE;
static kal_bool   SP2529MIPI_VEDIO_encode_mode = KAL_FALSE; 
//SP_CMSTART
static kal_bool	setshutter = KAL_FALSE;
//SP_CMEND
static kal_bool   SP2529MIPI_sensor_cap_state = KAL_FALSE; 
static kal_uint32 SP2529MIPI_sensor_pclk=480;//720;
static kal_bool   SP2529MIPI_AE_ENABLE = KAL_TRUE; 
MSDK_SENSOR_CONFIG_STRUCT SP2529MIPISensorConfigData;

kal_bool   SP2529_CAM_BANDING_50HZ = KAL_TRUE;
kal_bool   SP2529_CAM_Nightmode = 0;

//SP_CMSTART
/*************************************************************************
 * FUNCTION
 *	SP2529MIPI_SetShutter
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
void SP2529MIPI_Set_Shutter(kal_uint16 iShutter)
{
	kal_uint8 temp_reg_L, temp_reg_H;
	temp_reg_L = iShutter & 0xff;
	temp_reg_H = (iShutter >>8) & 0xff;
	SP2529MIPI_write_cmos_sensor(0xfd,0x00); 
	SP2529MIPI_write_cmos_sensor(0x03,temp_reg_H);
	SP2529MIPI_write_cmos_sensor(0x04,temp_reg_L);
	SENSORDB(" SP2529MIPI_Set_Shutter\r\n");
} /* Set_SP2529MIPI_Shutter */


/*************************************************************************
 * FUNCTION
 *	SP2529MIPI_read_Shutter
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
kal_uint16 SP2529MIPI_Read_Shutter(void)
{
	kal_uint8 temp_reg_L, temp_reg_H;
	kal_uint16 shutter;
	SP2529MIPI_write_cmos_sensor(0xfd,0x00); 
	temp_reg_L = SP2529MIPI_read_cmos_sensor(0x04);
	temp_reg_H = SP2529MIPI_read_cmos_sensor(0x03);

	shutter = (temp_reg_L & 0xFF) | (temp_reg_H << 8);

	SENSORDB(" SP2529MIPI_Read_Shutter %x \r\n",shutter);
	return shutter;
} /* SP2529MIPI_read_shutter */
//SP_CMEND
/*************************************************************************
* FUNCTION
*	SP2529MIPIInitialPara
*
* DESCRIPTION
*	This function initialize the global status of  MT9V114
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
static void SP2529MIPIInitialPara(void)
{
  spin_lock(&ov9740mipi_yuv_drv_lock);
  SP2529MIPICurrentStatus.iNightMode = 0;
  SP2529MIPICurrentStatus.iWB = AWB_MODE_AUTO;
  SP2529MIPICurrentStatus.iEffect = MEFFECT_OFF;
  SP2529MIPICurrentStatus.iBanding = AE_FLICKER_MODE_50HZ;
  SP2529MIPICurrentStatus.iEV = AE_EV_COMP_n03;
  SP2529MIPICurrentStatus.iMirror = IMAGE_NORMAL;
  SP2529MIPICurrentStatus.iFrameRate = 0;
  spin_unlock(&ov9740mipi_yuv_drv_lock);
}


void SP2529MIPI_set_mirror(kal_uint8 image_mirror)
{

		if(SP2529MIPICurrentStatus.iMirror == image_mirror)
		  return;
		//rotation 180 case
		switch (image_mirror)  
		{
			case IMAGE_NORMAL:
				SP2529MIPI_write_cmos_sensor(0xfd, 0x00); 
				SP2529MIPI_write_cmos_sensor(0x3f, 0x00); 
				break;
			case IMAGE_H_MIRROR:
				SP2529MIPI_write_cmos_sensor(0xfd, 0x00); 
				SP2529MIPI_write_cmos_sensor(0x3f, 0x01);  //mirror : bit0
				break;
			case IMAGE_V_MIRROR:
				SP2529MIPI_write_cmos_sensor(0xfd, 0x00); 
				SP2529MIPI_write_cmos_sensor(0x3f, 0x02);   //mirror : bit 1
				break;
			case IMAGE_HV_MIRROR:
				SP2529MIPI_write_cmos_sensor(0xfd, 0x00); 
				SP2529MIPI_write_cmos_sensor(0x3f, 0x03); 
				break;
			default:
				ASSERT(0);
				break;
		} 

		/* //general case.
		switch (image_mirror)  
		{
			case IMAGE_NORMAL:
				SP2529MIPI_write_cmos_sensor(0x0101, 0x00);
				break;
			case IMAGE_H_MIRROR:
				SP2529MIPI_write_cmos_sensor(0x0101, 0x01);  //mirror : bit 0 
				break;
			case IMAGE_V_MIRROR:
				SP2529MIPI_write_cmos_sensor(0x0101, 0x02);  //mirror : bit1	
				break;
			case IMAGE_HV_MIRROR:
				SP2529MIPI_write_cmos_sensor(0x0101, 0x03); 	
				break;
			default:
				ASSERT(0);
				break;
		} */
		spin_lock(&ov9740mipi_yuv_drv_lock);
		SP2529MIPICurrentStatus.iMirror = image_mirror;
		spin_unlock(&ov9740mipi_yuv_drv_lock);

}
//SP_CMSTART
/*************************************************************************
 * FUNCTION
 *	SP2529_set_hb_shutter
 *
 * DESCRIPTION
 *	This function set the dummy pixels(Horizontal Blanking) when capturing, it can be
 *	used to adjust the frame rate  for back-end process.
 *	
 *	IMPORTANT NOTICE: the base shutter need re-calculate for some sensor, or else flicker may occur.
 *
 * PARAMETERS
 *	1. kal_uint32 : Dummy Pixels (Horizontal Blanking)
 *	2. kal_uint32 : shutter (Vertical Blanking)
 *
 * RETURNS
 *	None
 *
 *************************************************************************/
static void SP2529MIPI_set_hb_shutter(kal_uint32 hb_add,  kal_uint32 shutter)
{
	kal_uint32 hb_ori, hb_total;
	kal_uint32 temp_reg, banding_step;
	SENSORDB("Ronlus SP2529MIPI_set_hb_shutter\r\n");
}    /* SP2529MIPI_set_dummy */
//SP_CMEND


/*****************************************************************************
 * FUNCTION
 *  SP2529MIPI_set_dummy
 * DESCRIPTION
 *
 * PARAMETERS
 *  pixels      [IN]
 *  lines       [IN]
 * RETURNS
 *  void
 *****************************************************************************/
void SP2529MIPI_set_dummy(kal_uint16 dummy_pixels, kal_uint16 dummy_lines)
{
		/****************************************************
		  * Adjust the extra H-Blanking & V-Blanking.
		  *****************************************************/
		SP2529MIPI_write_cmos_sensor(0xfd, 0x00);  
		SP2529MIPI_write_cmos_sensor(0x05, (dummy_lines>>4)  ); 	// Extra V-Blanking
		SP2529MIPI_write_cmos_sensor(0x06, (dummy_lines&0xFF)); 	// Extra V-Blanking
		SP2529MIPI_write_cmos_sensor(0x09, (dummy_pixels>>4)	);	// Extra HBlanking
		SP2529MIPI_write_cmos_sensor(0x0a, (dummy_pixels&0xFF));	// Extra H-Blanking


}   /* SP2529MIPI_set_dummy */

/*****************************************************************************
 * FUNCTION
 *  SP2529MIPI_Initialize_Setting
 * DESCRIPTION
 *    SP2529 DVP 1280x720 Full at 30FPS in YUV format
 *    24Mhz input, system clock 76Mhz
 * PARAMETERS
 *  void
 * RETURNS
 *  void
 *****************************************************************************/
void SP2529MIPI_Initialize_Setting(void)
{
#ifdef DEBUGE_SENSOR_SP0A28
			if(fromsd == 1)//ê?·?'óSD?áè?//gepeiwei   120903
				{
				
			printk("________________from t!\n");
			SP0A28_Initialize_from_T_Flash();//'óSD?¨?áè?µ??÷òao¯êy
				}
			else
#endif//SP0A28 FOR  T FLSAH
{
//binning 800 600 pll 1.5 fix 10-22fps
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);
SP2529MIPI_write_cmos_sensor(0x36 , 0x02);
SP2529MIPI_write_cmos_sensor(0xfd , 0x00);
SP2529MIPI_write_cmos_sensor(0x1c , 0x17);
SP2529MIPI_write_cmos_sensor(0x30 , 0x0c);
SP2529MIPI_write_cmos_sensor(0x2f , 0x09);
SP2529MIPI_write_cmos_sensor(0xfd , 0x00);
SP2529MIPI_write_cmos_sensor(0x0c , 0x55);
SP2529MIPI_write_cmos_sensor(0x27 , 0xa5);
SP2529MIPI_write_cmos_sensor(0x1a , 0x4b);
SP2529MIPI_write_cmos_sensor(0x20 , 0x2f);
SP2529MIPI_write_cmos_sensor(0x22 , 0x5a);
SP2529MIPI_write_cmos_sensor(0x25 , 0xbd);
SP2529MIPI_write_cmos_sensor(0x21 , 0x10);
SP2529MIPI_write_cmos_sensor(0x28 , 0x08);
SP2529MIPI_write_cmos_sensor(0x1d , 0x01);
SP2529MIPI_write_cmos_sensor(0x7a , 0x5d);
SP2529MIPI_write_cmos_sensor(0x70 , 0x41);
SP2529MIPI_write_cmos_sensor(0x74 , 0x40);
SP2529MIPI_write_cmos_sensor(0x75 , 0x40);
SP2529MIPI_write_cmos_sensor(0x15 , 0x3e);
SP2529MIPI_write_cmos_sensor(0x71 , 0x3f);
SP2529MIPI_write_cmos_sensor(0x7c , 0x3f);
SP2529MIPI_write_cmos_sensor(0x76 , 0x3f);
SP2529MIPI_write_cmos_sensor(0x7e , 0x29);
SP2529MIPI_write_cmos_sensor(0x72 , 0x29);
SP2529MIPI_write_cmos_sensor(0x77 , 0x28);
SP2529MIPI_write_cmos_sensor(0x1e , 0x01);
SP2529MIPI_write_cmos_sensor(0x1c , 0x0f);
SP2529MIPI_write_cmos_sensor(0x2e , 0xc0);
SP2529MIPI_write_cmos_sensor(0x1f , 0xc0);
SP2529MIPI_write_cmos_sensor(0x6c , 0x00);
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);
SP2529MIPI_write_cmos_sensor(0x32 , 0x00);
SP2529MIPI_write_cmos_sensor(0xfd , 0x02);
SP2529MIPI_write_cmos_sensor(0x85 , 0x00);     

//ae setting    1.5pll 10-22.0695fps 
SP2529MIPI_write_cmos_sensor(0xfd,0x00);
SP2529MIPI_write_cmos_sensor(0x03,0x05);
SP2529MIPI_write_cmos_sensor(0x04,0x46);
SP2529MIPI_write_cmos_sensor(0x05,0x00);
SP2529MIPI_write_cmos_sensor(0x06,0x00);
SP2529MIPI_write_cmos_sensor(0x07,0x00);
SP2529MIPI_write_cmos_sensor(0x08,0x00);
SP2529MIPI_write_cmos_sensor(0x09,0x01);
SP2529MIPI_write_cmos_sensor(0x0a,0x1b);
SP2529MIPI_write_cmos_sensor(0xfd,0x01);
SP2529MIPI_write_cmos_sensor(0xf0,0x00);
SP2529MIPI_write_cmos_sensor(0xf7,0xe1);
SP2529MIPI_write_cmos_sensor(0xf8,0xbc);
SP2529MIPI_write_cmos_sensor(0x02,0x0b);
SP2529MIPI_write_cmos_sensor(0x03,0x01);
SP2529MIPI_write_cmos_sensor(0x06,0xe1);
SP2529MIPI_write_cmos_sensor(0x07,0x00);
SP2529MIPI_write_cmos_sensor(0x08,0x01);
SP2529MIPI_write_cmos_sensor(0x09,0x00);
SP2529MIPI_write_cmos_sensor(0xfd,0x02);
SP2529MIPI_write_cmos_sensor(0x3d,0x0d);
SP2529MIPI_write_cmos_sensor(0x3e,0xbc);
SP2529MIPI_write_cmos_sensor(0x3f,0x00);
SP2529MIPI_write_cmos_sensor(0x88,0x46);
SP2529MIPI_write_cmos_sensor(0x89,0xb9);
SP2529MIPI_write_cmos_sensor(0x8a,0x22);
SP2529MIPI_write_cmos_sensor(0xfd,0x02);
SP2529MIPI_write_cmos_sensor(0xbe,0xab);
SP2529MIPI_write_cmos_sensor(0xbf,0x09);
SP2529MIPI_write_cmos_sensor(0xd0,0xab);
SP2529MIPI_write_cmos_sensor(0xd1,0x09);
SP2529MIPI_write_cmos_sensor(0xc9,0xab);
SP2529MIPI_write_cmos_sensor(0xca,0x09); 
                                    
SP2529MIPI_write_cmos_sensor(0xb8 , 0x90);   //mean_nr_dummy    
SP2529MIPI_write_cmos_sensor(0xb9 , 0x85);   //mean_dummy_nr    
SP2529MIPI_write_cmos_sensor(0xba , 0x30);   //mean_dummy_low   
SP2529MIPI_write_cmos_sensor(0xbb , 0x45);   //mean_low_dummy   
SP2529MIPI_write_cmos_sensor(0xbc , 0xc0);   //rpc_heq_low      
SP2529MIPI_write_cmos_sensor(0xbd , 0x60);   //rpc_heq_dummy    
SP2529MIPI_write_cmos_sensor(0xfd , 0x03);                     
SP2529MIPI_write_cmos_sensor(0x77 , 0x48);   //rpc_heq_nr2      
//rpc                               
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);
SP2529MIPI_write_cmos_sensor(0xe0 , 0x48);
SP2529MIPI_write_cmos_sensor(0xe1 , 0x38);
SP2529MIPI_write_cmos_sensor(0xe2 , 0x30);
SP2529MIPI_write_cmos_sensor(0xe3 , 0x2c);
SP2529MIPI_write_cmos_sensor(0xe4 , 0x2c);
SP2529MIPI_write_cmos_sensor(0xe5 , 0x2a);
SP2529MIPI_write_cmos_sensor(0xe6 , 0x2a);
SP2529MIPI_write_cmos_sensor(0xe7 , 0x28);
SP2529MIPI_write_cmos_sensor(0xe8 , 0x28);
SP2529MIPI_write_cmos_sensor(0xe9 , 0x28);
SP2529MIPI_write_cmos_sensor(0xea , 0x26);
SP2529MIPI_write_cmos_sensor(0xf3 , 0x26);
SP2529MIPI_write_cmos_sensor(0xf4 , 0x26);
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);  //ae min gain    
SP2529MIPI_write_cmos_sensor(0x04 , 0xb0);  //rpc_max_indr   
SP2529MIPI_write_cmos_sensor(0x05 , 0x20);  //rpc_min_indr   
SP2529MIPI_write_cmos_sensor(0x0a , 0x48);  //rpc_max_outdr  
SP2529MIPI_write_cmos_sensor(0x0b , 0x26);  //rpc_min_outdr  
                                    
                                    
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);   //ae target                     
SP2529MIPI_write_cmos_sensor(0xf2 , 0x09);                
SP2529MIPI_write_cmos_sensor(0xeb , 0x98);   //target_indr	
SP2529MIPI_write_cmos_sensor(0xec , 0x78);   //target_outdr	
SP2529MIPI_write_cmos_sensor(0xed , 0x06);   //lock_range    
SP2529MIPI_write_cmos_sensor(0xee , 0x0a);   //hold_range    
                                    
SP2529MIPI_write_cmos_sensor(0xfd , 0x02);      
SP2529MIPI_write_cmos_sensor(0x4f , 0x46);    //dem_morie_thr              
                                 
  //去坏像素                     
SP2529MIPI_write_cmos_sensor(0xfd , 0x03);
SP2529MIPI_write_cmos_sensor(0x52 , 0xff); //dpix_wht_ofst_outdoor        
SP2529MIPI_write_cmos_sensor(0x53 , 0x60); //dpix_wht_ofst_normal1        
SP2529MIPI_write_cmos_sensor(0x94 , 0x00);//20 //dpix_wht_ofst_normal2        
SP2529MIPI_write_cmos_sensor(0x54 , 0x00); //dpix_wht_ofst_dummy          
SP2529MIPI_write_cmos_sensor(0x55 , 0x00); //dpix_wht_ofst_low            
                                       
SP2529MIPI_write_cmos_sensor(0x56 , 0x80); //dpix_blk_ofst_outdoor        
SP2529MIPI_write_cmos_sensor(0x57 , 0x80); //dpix_blk_ofst_normal1        
SP2529MIPI_write_cmos_sensor(0x95 , 0x00);//80 //dpix_blk_ofst_normal2        
SP2529MIPI_write_cmos_sensor(0x58 , 0x00); //dpix_blk_ofst_dummy          
SP2529MIPI_write_cmos_sensor(0x59 , 0x00); //dpix_blk_ofst_low            
                                      
SP2529MIPI_write_cmos_sensor(0x5a , 0xf6); //dpix_wht_ratio               
SP2529MIPI_write_cmos_sensor(0x5b , 0x00);                               
SP2529MIPI_write_cmos_sensor(0x5c , 0x88); //dpix_blk_ratio               
SP2529MIPI_write_cmos_sensor(0x5d , 0x00);                               
SP2529MIPI_write_cmos_sensor(0x96 , 0x00);//68 //dpix_wht/blk_ratio_nr2       
                                   
                                   
SP2529MIPI_write_cmos_sensor(0xfd , 0x03);
SP2529MIPI_write_cmos_sensor(0x8a , 0x00);
SP2529MIPI_write_cmos_sensor(0x8b , 0x00);
SP2529MIPI_write_cmos_sensor(0x8c , 0xff);
                                  
SP2529MIPI_write_cmos_sensor(0x22 , 0xff); //dem_gdif_thr_outdoor    
SP2529MIPI_write_cmos_sensor(0x23 , 0xff); //dem_gdif_thr_normal     
SP2529MIPI_write_cmos_sensor(0x24 , 0xff); //dem_gdif_thr_dummy      
SP2529MIPI_write_cmos_sensor(0x25 , 0xff); //dem_gdif_thr_low        
                                    
SP2529MIPI_write_cmos_sensor(0x5e , 0xff); //dem_gwnd_wht_outdoor    
SP2529MIPI_write_cmos_sensor(0x5f , 0xff); //dem_gwnd_wht_normal     
SP2529MIPI_write_cmos_sensor(0x60 , 0xff); //dem_gwnd_wht_dummy      
SP2529MIPI_write_cmos_sensor(0x61 , 0xff); //dem_gwnd_wht_low        
SP2529MIPI_write_cmos_sensor(0x62 , 0x00); //dem_gwnd_blk_outdoor    
SP2529MIPI_write_cmos_sensor(0x63 , 0x00); //dem_gwnd_blk_normal     
SP2529MIPI_write_cmos_sensor(0x64 , 0x00); //dem_gwnd_blk_dummy      
SP2529MIPI_write_cmos_sensor(0x65 , 0x00); //dem_gwnd_blk_low        
//lsc                              
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);
SP2529MIPI_write_cmos_sensor(0x21 , 0x00);  //lsc_sig_ru lsc_sig_lu             
SP2529MIPI_write_cmos_sensor(0x22 , 0x00);  //lsc_sig_rd lsc_sig_ld             
SP2529MIPI_write_cmos_sensor(0x26 , 0xa0);  //lsc_gain_thr                      
SP2529MIPI_write_cmos_sensor(0x27 , 0x14);  //lsc_exp_thrl                      
SP2529MIPI_write_cmos_sensor(0x28 , 0x05);  //lsc_exp_thrh                      
SP2529MIPI_write_cmos_sensor(0x29 , 0x20);  //lsc_dec_fac     进dummy态退shading 功能有问题，需关掉                  
SP2529MIPI_write_cmos_sensor(0x2a , 0x01);  //lsc_rpc_en lens 衰减自适应        
//LSC for CHT813                                            
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);                                     
SP2529MIPI_write_cmos_sensor(0xa1 , 0x15);  //lsc_rsx_l                                       
SP2529MIPI_write_cmos_sensor(0xa2 , 0x15);  //lsc_rsx_r                         
SP2529MIPI_write_cmos_sensor(0xa3 , 0x15);  //lsc_rsy_u                         
SP2529MIPI_write_cmos_sensor(0xa4 , 0x15);  //lsc_rsy_d                         
SP2529MIPI_write_cmos_sensor(0xa5 , 0x12);  //lsc_gxy_l                         
SP2529MIPI_write_cmos_sensor(0xa6 , 0x14);  //lsc_gxy_r                         
SP2529MIPI_write_cmos_sensor(0xa7 , 0x0e);  //lsc_gxy_l                         
SP2529MIPI_write_cmos_sensor(0xa8 , 0x0d);  //lsc_gxy_r                         
SP2529MIPI_write_cmos_sensor(0xa9 , 0x11);  //lsc_bsx_l                         
SP2529MIPI_write_cmos_sensor(0xaa , 0x16);  //lsc_bsx_r                         
SP2529MIPI_write_cmos_sensor(0xab , 0x0d);  //lsc_bsy_u                         
SP2529MIPI_write_cmos_sensor(0xac , 0x0b);  //lsc_bsy_d                         
SP2529MIPI_write_cmos_sensor(0xad , 0x13);  //lsc_rxy_lu                        
SP2529MIPI_write_cmos_sensor(0xae , 0x08);  //lsc_rxy_ru                        
SP2529MIPI_write_cmos_sensor(0xaf , 0x05);  //lsc_rxy_ld                        
SP2529MIPI_write_cmos_sensor(0xb0 , 0x04);  //lsc_rxy_rd                        
SP2529MIPI_write_cmos_sensor(0xb1 , 0x10);  //lsc_gsx_lu                        
SP2529MIPI_write_cmos_sensor(0xb2 , 0x0a);  //lsc_gsx_ru                        
SP2529MIPI_write_cmos_sensor(0xb3 , 0x05);  //lsc_gsy_ud                        
SP2529MIPI_write_cmos_sensor(0xb4 , 0x07);  //lsc_gsy_dd                        
SP2529MIPI_write_cmos_sensor(0xb5 , 0x10);  //lsc_bxy_lu                        
SP2529MIPI_write_cmos_sensor(0xb6 , 0x0a);  //lsc_bxy_ru                        
SP2529MIPI_write_cmos_sensor(0xb7 , 0x07);  //lsc_bxy_ld                        
SP2529MIPI_write_cmos_sensor(0xb8 , 0x07);  //lsc_bxy_rd                         
//awb                                
SP2529MIPI_write_cmos_sensor(0xfd , 0x02);
SP2529MIPI_write_cmos_sensor(0x26 , 0xae);  //Red channel gain                                 
SP2529MIPI_write_cmos_sensor(0x27 , 0x91);  //Blue channel gain                                
SP2529MIPI_write_cmos_sensor(0x28 , 0xcc);  //Y top value limit                                
SP2529MIPI_write_cmos_sensor(0x29 , 0x01);  //Y bot value limit                                
SP2529MIPI_write_cmos_sensor(0x2a , 0x02);  //rg_limit_log                                     
SP2529MIPI_write_cmos_sensor(0x2b , 0x16);  //bg_limit_log                                     
SP2529MIPI_write_cmos_sensor(0x2c , 0x20);  //Awb image center row start                       
SP2529MIPI_write_cmos_sensor(0x2d , 0xdc);  //Awb image center row end                         
SP2529MIPI_write_cmos_sensor(0x2e , 0x20);  //Awb image center col start                       
SP2529MIPI_write_cmos_sensor(0x2f , 0x96);  //Awb image center col end                         
SP2529MIPI_write_cmos_sensor(0x1b , 0x80);  //b,g mult a constant for detect white pixel       
SP2529MIPI_write_cmos_sensor(0x1a , 0x80);  //r,g mult a constant for detect white pixel       
SP2529MIPI_write_cmos_sensor(0x18 , 0x16);  //wb_fine_gain_step,wb_rough_gain_step             
SP2529MIPI_write_cmos_sensor(0x19 , 0x26);  //wb_dif_fine_th, wb_dif_rough_th                  
SP2529MIPI_write_cmos_sensor(0x1d , 0x04);  //skin detect u bot                                 
SP2529MIPI_write_cmos_sensor(0x1f , 0x0c);  //skin detect v bot                                
 //d65 10                           
SP2529MIPI_write_cmos_sensor(0x66 , 0x36);
SP2529MIPI_write_cmos_sensor(0x67 , 0x5c);
SP2529MIPI_write_cmos_sensor(0x68 , 0xbb);
SP2529MIPI_write_cmos_sensor(0x69 , 0xdf);
SP2529MIPI_write_cmos_sensor(0x6a , 0xa5);
 //indoor                          
SP2529MIPI_write_cmos_sensor(0x7c , 0x26);
SP2529MIPI_write_cmos_sensor(0x7d , 0x4a);
SP2529MIPI_write_cmos_sensor(0x7e , 0xe0);
SP2529MIPI_write_cmos_sensor(0x7f , 0x05);
SP2529MIPI_write_cmos_sensor(0x80 , 0xa6);
 //cwf   12                         
SP2529MIPI_write_cmos_sensor(0x70 , 0x21);
SP2529MIPI_write_cmos_sensor(0x71 , 0x41);
SP2529MIPI_write_cmos_sensor(0x72 , 0x05);
SP2529MIPI_write_cmos_sensor(0x73 , 0x25);
SP2529MIPI_write_cmos_sensor(0x74 , 0xaa);
//tl84                              
SP2529MIPI_write_cmos_sensor(0x6b , 0x00);
SP2529MIPI_write_cmos_sensor(0x6c , 0x20);
SP2529MIPI_write_cmos_sensor(0x6d , 0x0e);
SP2529MIPI_write_cmos_sensor(0x6e , 0x2a);
SP2529MIPI_write_cmos_sensor(0x6f , 0xaa);
                                  
SP2529MIPI_write_cmos_sensor(0x61 , 0xdb);
SP2529MIPI_write_cmos_sensor(0x62 , 0xfe);
SP2529MIPI_write_cmos_sensor(0x63 , 0x37);
SP2529MIPI_write_cmos_sensor(0x64 , 0x56);
SP2529MIPI_write_cmos_sensor(0x65 , 0x6a);
 //f                                
SP2529MIPI_write_cmos_sensor(0x75 , 0x00);
SP2529MIPI_write_cmos_sensor(0x76 , 0x09);
SP2529MIPI_write_cmos_sensor(0x77 , 0x02);
SP2529MIPI_write_cmos_sensor(0x0e , 0x16);
SP2529MIPI_write_cmos_sensor(0x3b , 0x09);
                                   
SP2529MIPI_write_cmos_sensor(0xfd , 0x02); //awb outdoor mode               
SP2529MIPI_write_cmos_sensor(0x02 , 0x00); //outdoor exp 5msb   
SP2529MIPI_write_cmos_sensor(0x03 , 0x10); //outdoor exp 8lsb   
SP2529MIPI_write_cmos_sensor(0x04 , 0xf0); //outdoor rpc        
SP2529MIPI_write_cmos_sensor(0xf5 , 0xb3); //outdoor rgain top  
SP2529MIPI_write_cmos_sensor(0xf6 , 0x80); //outdoor rgain bot  
SP2529MIPI_write_cmos_sensor(0xf7 , 0xe0); //outdoor bgain top  
SP2529MIPI_write_cmos_sensor(0xf8 , 0x89); //outdoor bgain bot  
                                   
 //skin detect                     
SP2529MIPI_write_cmos_sensor(0xfd , 0x02);
SP2529MIPI_write_cmos_sensor(0x08 , 0x00);
SP2529MIPI_write_cmos_sensor(0x09 , 0x04);
                                   
SP2529MIPI_write_cmos_sensor(0xfd , 0x02);
SP2529MIPI_write_cmos_sensor(0xdd , 0x0f); //raw smooth en 
SP2529MIPI_write_cmos_sensor(0xde , 0x0f); //sharpen en    
                                   
SP2529MIPI_write_cmos_sensor(0xfd , 0x02); // sharp               
SP2529MIPI_write_cmos_sensor(0x57 , 0x25); //raw_sharp_y_base     
SP2529MIPI_write_cmos_sensor(0x58 , 0x10); //raw_sharp_y_min      
SP2529MIPI_write_cmos_sensor(0x59 , 0xe0); //raw_sharp_y_max      
SP2529MIPI_write_cmos_sensor(0x5a , 0x10); //raw_sharp_rangek_neg 
SP2529MIPI_write_cmos_sensor(0x5b , 0x1c); //raw_sharp_rangek_pos 
                                   
SP2529MIPI_write_cmos_sensor(0xcb , 0x0a); //raw_sharp_range_base_outdoor	
SP2529MIPI_write_cmos_sensor(0xcc , 0x06); //raw_sharp_range_base_nr 	
SP2529MIPI_write_cmos_sensor(0xcd , 0x0c); //raw_sharp_range_base_dummy	
SP2529MIPI_write_cmos_sensor(0xce , 0x12); //raw_sharp_range_base_low	
                                   
SP2529MIPI_write_cmos_sensor(0xfd , 0x03);
SP2529MIPI_write_cmos_sensor(0x87 , 0x08); //raw_sharp_range_ofst1	4x  
SP2529MIPI_write_cmos_sensor(0x88 , 0x10); //raw_sharp_range_ofst2	8x  
SP2529MIPI_write_cmos_sensor(0x89 , 0x14); //raw_sharp_range_ofst3	16x 
                                   
                                   
SP2529MIPI_write_cmos_sensor(0xfd , 0x02); 
SP2529MIPI_write_cmos_sensor(0xe8 , 0x5c); //sharpness gain for increasing pixel’s Y, in outdoor        
SP2529MIPI_write_cmos_sensor(0xec , 0x5c); //sharpness gain for decreasing pixel’s Y, in outdoor        
SP2529MIPI_write_cmos_sensor(0xe9 , 0x48); //sharpness gain for increasing pixel’s Y, in normal         
SP2529MIPI_write_cmos_sensor(0xed , 0x60); //sharpness gain for decreasing pixel’s Y, in normal         
SP2529MIPI_write_cmos_sensor(0xea , 0x34); //sharpness gain for increasing pixel’s Y,in dummy           
SP2529MIPI_write_cmos_sensor(0xee , 0x44); //sharpness gain for decreasing pixel’s Y, in dummy          
SP2529MIPI_write_cmos_sensor(0xeb , 0x24); //sharpness gain for increasing pixel’s Y,in lowlight        
SP2529MIPI_write_cmos_sensor(0xef , 0x2c); //sharpness gain for decreasing pixel’s Y, in low light      
                                    
SP2529MIPI_write_cmos_sensor(0xfd , 0x02); //skin sharpen                                   
SP2529MIPI_write_cmos_sensor(0xdc , 0x04); //skin_sharp_sel肤色降锐化                       
SP2529MIPI_write_cmos_sensor(0x05 , 0x00); //skin_num_th2排除肤色降锐化对分辨率卡引起的干扰 
                                    
//平滑自适应                        
SP2529MIPI_write_cmos_sensor(0xfd , 0x02);
SP2529MIPI_write_cmos_sensor(0xf4 , 0x30);  //raw_ymin           
SP2529MIPI_write_cmos_sensor(0xfd , 0x03);                      
SP2529MIPI_write_cmos_sensor(0x97 , 0x80);  //raw_ymax_outdoor   
SP2529MIPI_write_cmos_sensor(0x98 , 0x80);  //raw_ymax_normal    
SP2529MIPI_write_cmos_sensor(0x99 , 0x80);  //raw_ymax_dummy     
SP2529MIPI_write_cmos_sensor(0x9a , 0x80);  //raw_ymax_low       
SP2529MIPI_write_cmos_sensor(0xfd , 0x02);                      
SP2529MIPI_write_cmos_sensor(0xe4 , 0xff);  //raw_yk_fac_outdoor 
SP2529MIPI_write_cmos_sensor(0xe5 , 0xff);  //raw_yk_fac_normal  
SP2529MIPI_write_cmos_sensor(0xe6 , 0xff);  //raw_yk_fac_dummy   
SP2529MIPI_write_cmos_sensor(0xe7 , 0xff);  //raw_yk_fac_low     
                                  
SP2529MIPI_write_cmos_sensor(0xfd , 0x03);
SP2529MIPI_write_cmos_sensor(0x72 , 0x00);  //raw_lsc_fac_outdoor 
SP2529MIPI_write_cmos_sensor(0x73 , 0x2a);  //raw_lsc_fac_normal  
SP2529MIPI_write_cmos_sensor(0x74 , 0x2a);  //raw_lsc_fac_dummy   
SP2529MIPI_write_cmos_sensor(0x75 , 0x2d);  //raw_lsc_fac_low     
                                   
//四个通道内阈值                   
SP2529MIPI_write_cmos_sensor(0xfd , 0x02);
SP2529MIPI_write_cmos_sensor(0x78 , 0x20);
SP2529MIPI_write_cmos_sensor(0x79 , 0x20);
SP2529MIPI_write_cmos_sensor(0x7a , 0x14);
SP2529MIPI_write_cmos_sensor(0x7b , 0x08);
                                   
SP2529MIPI_write_cmos_sensor(0x81 , 0x40);//raw_grgb_thr_outdoor  
SP2529MIPI_write_cmos_sensor(0x82 , 0x40);
SP2529MIPI_write_cmos_sensor(0x83 , 0x50);
SP2529MIPI_write_cmos_sensor(0x84 , 0x50);
                                   
SP2529MIPI_write_cmos_sensor(0xfd , 0x03);
SP2529MIPI_write_cmos_sensor(0x7e , 0x0c); //raw_noise_base_outdoor                   
SP2529MIPI_write_cmos_sensor(0x7f , 0x12); //raw_noise_base_normal                    
SP2529MIPI_write_cmos_sensor(0x80 , 0x24); //raw_noise_base_dummy                     
SP2529MIPI_write_cmos_sensor(0x81 , 0x34); //raw_noise_base_low                       
SP2529MIPI_write_cmos_sensor(0x7c , 0xff); //raw_noise_base_dark                      
SP2529MIPI_write_cmos_sensor(0x82 , 0x65); //raw_dns_fac_outdoor,raw_dns_fac_normal} 
SP2529MIPI_write_cmos_sensor(0x83 , 0x54); //raw_dns_fac_dummy,raw_dns_fac_low}         
SP2529MIPI_write_cmos_sensor(0x84 , 0x08);  //raw_noise_ofst1 	4x                   
SP2529MIPI_write_cmos_sensor(0x85 , 0x40);  //raw_noise_ofst2	8x                   
SP2529MIPI_write_cmos_sensor(0x86 , 0x80); //raw_noise_ofst3	16x    
                                   
//去紫边功能                       
SP2529MIPI_write_cmos_sensor(0xfd , 0x03);
SP2529MIPI_write_cmos_sensor(0x66 , 0x18); //pf_bg_thr_normal b-g>thr      
SP2529MIPI_write_cmos_sensor(0x67 , 0x28); //pf_rg_thr_normal r-g<thr      
SP2529MIPI_write_cmos_sensor(0x68 , 0x20); //pf_delta_thr_normal |val|>thr 
SP2529MIPI_write_cmos_sensor(0x69 , 0x88); //pf_k_fac val/16               
SP2529MIPI_write_cmos_sensor(0x9b , 0x18); //pf_bg_thr_outdoor             
SP2529MIPI_write_cmos_sensor(0x9c , 0x28); //pf_rg_thr_outdoor             
SP2529MIPI_write_cmos_sensor(0x9d , 0x20); //pf_delta_thr_outdoor          
                                   
//Gamma                            
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);
SP2529MIPI_write_cmos_sensor(0x8b , 0x00);
SP2529MIPI_write_cmos_sensor(0x8c , 0x0f);
SP2529MIPI_write_cmos_sensor(0x8d , 0x21);
SP2529MIPI_write_cmos_sensor(0x8e , 0x2c);
SP2529MIPI_write_cmos_sensor(0x8f , 0x37);
SP2529MIPI_write_cmos_sensor(0x90 , 0x46);
SP2529MIPI_write_cmos_sensor(0x91 , 0x53);
SP2529MIPI_write_cmos_sensor(0x92 , 0x5e);
SP2529MIPI_write_cmos_sensor(0x93 , 0x6a);
SP2529MIPI_write_cmos_sensor(0x94 , 0x7d);
SP2529MIPI_write_cmos_sensor(0x95 , 0x8d);
SP2529MIPI_write_cmos_sensor(0x96 , 0x9e);
SP2529MIPI_write_cmos_sensor(0x97 , 0xac);
SP2529MIPI_write_cmos_sensor(0x98 , 0xba);
SP2529MIPI_write_cmos_sensor(0x99 , 0xc6);
SP2529MIPI_write_cmos_sensor(0x9a , 0xd1);
SP2529MIPI_write_cmos_sensor(0x9b , 0xda);
SP2529MIPI_write_cmos_sensor(0x9c , 0xe4);
SP2529MIPI_write_cmos_sensor(0x9d , 0xeb);
SP2529MIPI_write_cmos_sensor(0x9e , 0xf2);
SP2529MIPI_write_cmos_sensor(0x9f , 0xf9);
SP2529MIPI_write_cmos_sensor(0xa0 , 0xff); 
                                   
//CCM                              
SP2529MIPI_write_cmos_sensor(0xfd , 0x02); 
SP2529MIPI_write_cmos_sensor(0x15 , 0xc8); 
SP2529MIPI_write_cmos_sensor(0x16 , 0x95); 
 //!F                               
SP2529MIPI_write_cmos_sensor(0xa0 , 0x84);
SP2529MIPI_write_cmos_sensor(0xa1 , 0xf4);
SP2529MIPI_write_cmos_sensor(0xa2 , 0x0c);
SP2529MIPI_write_cmos_sensor(0xa3 , 0xe7);
SP2529MIPI_write_cmos_sensor(0xa4 , 0x99);
SP2529MIPI_write_cmos_sensor(0xa5 , 0x00);
SP2529MIPI_write_cmos_sensor(0xa6 , 0xf4);
SP2529MIPI_write_cmos_sensor(0xa7 , 0xda);
SP2529MIPI_write_cmos_sensor(0xa8 , 0xb3);
SP2529MIPI_write_cmos_sensor(0xa9 , 0x0c);
SP2529MIPI_write_cmos_sensor(0xaa , 0x03);
SP2529MIPI_write_cmos_sensor(0xab , 0x0f); 
   //F                              
SP2529MIPI_write_cmos_sensor(0xac , 0x9c);
SP2529MIPI_write_cmos_sensor(0xad , 0xf4);
SP2529MIPI_write_cmos_sensor(0xae , 0xf4);
SP2529MIPI_write_cmos_sensor(0xaf , 0xf4);
SP2529MIPI_write_cmos_sensor(0xb0 , 0x99);
SP2529MIPI_write_cmos_sensor(0xb1 , 0xf4);
SP2529MIPI_write_cmos_sensor(0xb2 , 0xf4);
SP2529MIPI_write_cmos_sensor(0xb3 , 0xda);
SP2529MIPI_write_cmos_sensor(0xb4 , 0xb3);
SP2529MIPI_write_cmos_sensor(0xb5 , 0x3c);
SP2529MIPI_write_cmos_sensor(0xb6 , 0x33);
SP2529MIPI_write_cmos_sensor(0xb7 , 0x0f);
                                   
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);  //auto_sat                 
SP2529MIPI_write_cmos_sensor(0xd2 , 0x2f);  //autosat_en[0]             
SP2529MIPI_write_cmos_sensor(0xd1 , 0x38);  //lum thr in green enhance 
SP2529MIPI_write_cmos_sensor(0xdd , 0x1b); 
SP2529MIPI_write_cmos_sensor(0xde , 0x1b); 
                                    
//auto sat                          
SP2529MIPI_write_cmos_sensor(0xfd , 0x02);
SP2529MIPI_write_cmos_sensor(0xc1 , 0x40);
SP2529MIPI_write_cmos_sensor(0xc2 , 0x40);
SP2529MIPI_write_cmos_sensor(0xc3 , 0x40);
SP2529MIPI_write_cmos_sensor(0xc4 , 0x40);
SP2529MIPI_write_cmos_sensor(0xc5 , 0x80);
SP2529MIPI_write_cmos_sensor(0xc6 , 0x80);
SP2529MIPI_write_cmos_sensor(0xc7 , 0x80);
SP2529MIPI_write_cmos_sensor(0xc8 , 0x80);
                                   
//sat u                            
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);
SP2529MIPI_write_cmos_sensor(0xd3 , 0x90);
SP2529MIPI_write_cmos_sensor(0xd4 , 0x90);
SP2529MIPI_write_cmos_sensor(0xd5 , 0x90);
SP2529MIPI_write_cmos_sensor(0xd6 , 0x90);
//sat v                            
SP2529MIPI_write_cmos_sensor(0xd7 , 0x98);
SP2529MIPI_write_cmos_sensor(0xd8 , 0x98);
SP2529MIPI_write_cmos_sensor(0xd9 , 0x98);
SP2529MIPI_write_cmos_sensor(0xda , 0x98);
                                    
SP2529MIPI_write_cmos_sensor(0xfd , 0x03);
SP2529MIPI_write_cmos_sensor(0x76 , 0x0a);
SP2529MIPI_write_cmos_sensor(0x7a , 0x40);
SP2529MIPI_write_cmos_sensor(0x7b , 0x40);
 //auto_sat                        
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);
SP2529MIPI_write_cmos_sensor(0xc2 , 0xee);  //u_v_th_outdoor白色物体表面有彩色噪声降低此值    
SP2529MIPI_write_cmos_sensor(0xc3 , 0xee);  //u_v_th_nr                                       
SP2529MIPI_write_cmos_sensor(0xc4 , 0xaa);  //u_v_th_dummy                                    
SP2529MIPI_write_cmos_sensor(0xc5 , 0xaa);  //u_v_th_low          
                                    
//low_lum_offset                    
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);
SP2529MIPI_write_cmos_sensor(0xcd , 0x09);
SP2529MIPI_write_cmos_sensor(0xce , 0x10);
//gw                                
SP2529MIPI_write_cmos_sensor(0xfd , 0x02);
SP2529MIPI_write_cmos_sensor(0x32 , 0x60);
SP2529MIPI_write_cmos_sensor(0x35 , 0x60); //uv_fix_dat 
SP2529MIPI_write_cmos_sensor(0x37 , 0x13);
                                   
//heq                              
SP2529MIPI_write_cmos_sensor(0xfd , 0x01); 
SP2529MIPI_write_cmos_sensor(0xdb , 0x00); //buf_heq_offset    
SP2529MIPI_write_cmos_sensor(0x10 , 0x78); //ku_outdoor       
SP2529MIPI_write_cmos_sensor(0x11 , 0x8c); //ku_nr            
SP2529MIPI_write_cmos_sensor(0x12 , 0x84); //ku_dummy         
SP2529MIPI_write_cmos_sensor(0x13 , 0x80); //ku_low           
SP2529MIPI_write_cmos_sensor(0x14 , 0xb0); //kl_outdoor       
SP2529MIPI_write_cmos_sensor(0x15 , 0xc0); //kl_nr            
SP2529MIPI_write_cmos_sensor(0x16 , 0xac); //kl_dummy         
SP2529MIPI_write_cmos_sensor(0x17 , 0xa4); //kl_low           
                                   
SP2529MIPI_write_cmos_sensor(0xfd , 0x03);
SP2529MIPI_write_cmos_sensor(0x00 , 0x80); //ctf_heq_mean	                          
SP2529MIPI_write_cmos_sensor(0x03 , 0x30); //ctf_range_thr   可以排除灰板场景的阈值   
SP2529MIPI_write_cmos_sensor(0x06 , 0xf0); //ctf_reg_max	                          
SP2529MIPI_write_cmos_sensor(0x07 , 0x08); //ctf_reg_min	                          
SP2529MIPI_write_cmos_sensor(0x0a , 0x00); //ctf_lum_ofst                             
SP2529MIPI_write_cmos_sensor(0x01 , 0x00); //ctf_posk_fac_outdoor                     
SP2529MIPI_write_cmos_sensor(0x02 , 0x00); //ctf_posk_fac_nr                          
SP2529MIPI_write_cmos_sensor(0x04 , 0x00); //ctf_posk_fac_dummy                       
SP2529MIPI_write_cmos_sensor(0x05 , 0x00); //ctf_posk_fac_low                         
SP2529MIPI_write_cmos_sensor(0x0b , 0x00); //ctf_negk_fac_outdoor                     
SP2529MIPI_write_cmos_sensor(0x0c , 0x00); //ctf_negk_fac_nr                          
SP2529MIPI_write_cmos_sensor(0x0d , 0x00); //ctf_negk_fac_dummy                       
SP2529MIPI_write_cmos_sensor(0x0e , 0x00); //ctf_negk_fac_low                         
SP2529MIPI_write_cmos_sensor(0x08 , 0x10); 
SP2529MIPI_write_cmos_sensor(0x09 , 0x10); 
                                    
SP2529MIPI_write_cmos_sensor(0xfd , 0x02); //cnr                   
SP2529MIPI_write_cmos_sensor(0x8e , 0x0a); //cnr_grad_thr_dummy    
SP2529MIPI_write_cmos_sensor(0x90 , 0x40); //20 //cnr_thr_outdoor   
SP2529MIPI_write_cmos_sensor(0x91 , 0x40); //20 //cnr_thr_nr        
SP2529MIPI_write_cmos_sensor(0x92 , 0x60); //60 //cnr_thr_dummy     
SP2529MIPI_write_cmos_sensor(0x93 , 0x80); //80 //cnr_thr_low       
SP2529MIPI_write_cmos_sensor(0x9e , 0x44);
SP2529MIPI_write_cmos_sensor(0x9f , 0x44);
                                   
SP2529MIPI_write_cmos_sensor(0xfd , 0x02); //auto                                                
SP2529MIPI_write_cmos_sensor(0x85 , 0x00); //enable 50Hz/60Hz function[4]  [3:0] interval_line   
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);
SP2529MIPI_write_cmos_sensor(0x00 , 0x00); //fix mode 
SP2529MIPI_write_cmos_sensor(0xfb , 0x25); 
SP2529MIPI_write_cmos_sensor(0x32 , 0x15); //ae en 
SP2529MIPI_write_cmos_sensor(0x33 , 0xef); //lsc\bpc en
SP2529MIPI_write_cmos_sensor(0x34 , 0xef);  //ynr[4]\cnr[0]\gamma[2]\colo[1]  
SP2529MIPI_write_cmos_sensor(0x35 , 0x40);  //YUYV                            
SP2529MIPI_write_cmos_sensor(0xfd , 0x00);        
SP2529MIPI_write_cmos_sensor(0x3f , 0x00); //mirror/flip    
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);
SP2529MIPI_write_cmos_sensor(0x50 , 0x00); //heq_auto_mode 读状态 
SP2529MIPI_write_cmos_sensor(0x66 , 0x00); //effect               
SP2529MIPI_write_cmos_sensor(0xfd , 0x02);                                                                                     
SP2529MIPI_write_cmos_sensor(0xd6 , 0x0f);  
                                    
                                    
SP2529MIPI_write_cmos_sensor(0xfd , 0x00);
SP2529MIPI_write_cmos_sensor(0x92 , 0x81);
SP2529MIPI_write_cmos_sensor(0x98 , 0x3a);
SP2529MIPI_write_cmos_sensor(0xfd , 0x00);
SP2529MIPI_write_cmos_sensor(0xfd , 0x01);
SP2529MIPI_write_cmos_sensor(0x36 , 0x00);
  
//binning
SP2529MIPI_write_cmos_sensor(0xfd , 0x00);
SP2529MIPI_write_cmos_sensor(0x19 , 0x03);
SP2529MIPI_write_cmos_sensor(0x31 , 0x04);
SP2529MIPI_write_cmos_sensor(0x33 , 0x01); 
//MIPI
SP2529MIPI_write_cmos_sensor(0xfd , 0x00);
SP2529MIPI_write_cmos_sensor(0x95 , 0x03);
SP2529MIPI_write_cmos_sensor(0x94 , 0x20);
SP2529MIPI_write_cmos_sensor(0x97 , 0x02);
SP2529MIPI_write_cmos_sensor(0x96 , 0x58);

}
}

/*****************************************************************************
 * FUNCTION
 *  SP2529MIPI_PV_Mode
 * DESCRIPTION
 *
 * PARAMETERS
 *  void
 * RETURNS
 *  void
 *****************************************************************************/
void SP2529MIPI_PV_Mode(void)
{
//bininig svga 
SP2529MIPI_write_cmos_sensor(0xfd,0x00);
SP2529MIPI_write_cmos_sensor(0x19,0x03);
SP2529MIPI_write_cmos_sensor(0x30,0x00);//01 00
SP2529MIPI_write_cmos_sensor(0x31,0x04); 
SP2529MIPI_write_cmos_sensor(0x33,0x01);                                    
//MIPI                              
SP2529MIPI_write_cmos_sensor(0x95,0x03); 
SP2529MIPI_write_cmos_sensor(0x94,0x20); 
SP2529MIPI_write_cmos_sensor(0x97,0x02); 
SP2529MIPI_write_cmos_sensor(0x96,0x58);

}

/*****************************************************************************
 * FUNCTION
 *  SP2529MIPI_CAP_Mode
 * DESCRIPTION
 *
 * PARAMETERS
 *  void
 * RETURNS
 *  void
 *****************************************************************************/

void SP2529MIPI_CAP_Mode(void)
{
//uxga 
SP2529MIPI_write_cmos_sensor(0xfd,0x00); 
SP2529MIPI_write_cmos_sensor(0x19,0x00);
SP2529MIPI_write_cmos_sensor(0x30,0x00);//00 
SP2529MIPI_write_cmos_sensor(0x31,0x00); 
SP2529MIPI_write_cmos_sensor(0x33,0x00);
//MIPI                                 
SP2529MIPI_write_cmos_sensor(0x95,0x06);
SP2529MIPI_write_cmos_sensor(0x94,0x40); 
SP2529MIPI_write_cmos_sensor(0x97,0x04); 
SP2529MIPI_write_cmos_sensor(0x96,0xb0);
}

static void SP2529MIPI_set_AE_mode(kal_bool AE_enable)
{
     if(AE_enable==KAL_TRUE)
     {
       SP2529MIPI_write_cmos_sensor(0xfd, 0x01);  
     	SP2529MIPI_write_cmos_sensor(0x32, 0x15);  
     }
     else
     {
     	SP2529MIPI_write_cmos_sensor(0xfd, 0x01);  
     	SP2529MIPI_write_cmos_sensor(0x32, 0x10);  
     }
}


/*************************************************************************
* FUNCTION
*	SP2529MIPI_night_mode
*
* DESCRIPTION
*	This function night mode of SP2529MIPI.
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


void SP2529MIPI_night_mode(kal_bool enable)
{
#ifdef DEBUGE_SENSOR_SP0A28
			return;
#endif

	//kal_uint16 night = SP2529_read_cmos_sensor(0x20);
	SENSORDB("Ronlus SP2529_night_mode\r\n");//¨°¨2D??Y3??|ìSP2529      a8-10 ????o¨?¨??810 3p  
	//sensorlist.cpp kd_imagesensor.h add related files  
#if 1//
	if (enable)/*night mode settings*/
	{
		SENSORDB(" night mode\r\n");
		SP2529_CAM_Nightmode = 1;
		//SP2529MIPI_write_cmos_sensor(0xfd,0x00);
		//SP2529MIPI_write_cmos_sensor(0xb2,SP2529_LOWLIGHT_Y0ffset);
		if(SP2529MIPI_MPEG4_encode_mode == KAL_TRUE) /*video night mode*/
		{
			SENSORDB("Ronlus video night mode\r\n");

			if(SP2529_CAM_BANDING_50HZ == KAL_TRUE)/*video night mode 50hz*/
			{	
				 // 1.5PLL  fix10-10fps   video night mode 50hz  
				SENSORDB(" video night mode 50hz\r\n");
				//luo-0924
				SP2529MIPI_write_cmos_sensor(0xfd,0x00);
				SP2529MIPI_write_cmos_sensor(0x03,0x02);
				SP2529MIPI_write_cmos_sensor(0x04,0xee);
				SP2529MIPI_write_cmos_sensor(0x05,0x00);
				SP2529MIPI_write_cmos_sensor(0x06,0x00);
				SP2529MIPI_write_cmos_sensor(0x07,0x00);
				SP2529MIPI_write_cmos_sensor(0x08,0x00);
				SP2529MIPI_write_cmos_sensor(0x09,0x03);
				SP2529MIPI_write_cmos_sensor(0x0a,0x9b);
				SP2529MIPI_write_cmos_sensor(0xfd,0x01);
				SP2529MIPI_write_cmos_sensor(0xf0,0x00);
				SP2529MIPI_write_cmos_sensor(0xf7,0x7d);
				SP2529MIPI_write_cmos_sensor(0xf8,0x68);
				SP2529MIPI_write_cmos_sensor(0x02,0x0a);
				SP2529MIPI_write_cmos_sensor(0x03,0x01);
				SP2529MIPI_write_cmos_sensor(0x06,0x7d);
				SP2529MIPI_write_cmos_sensor(0x07,0x00);
				SP2529MIPI_write_cmos_sensor(0x08,0x01);
				SP2529MIPI_write_cmos_sensor(0x09,0x00);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0x3d,0x0c);
				SP2529MIPI_write_cmos_sensor(0x3e,0x68);
				SP2529MIPI_write_cmos_sensor(0x3f,0x00);
				SP2529MIPI_write_cmos_sensor(0x88,0x18);
				SP2529MIPI_write_cmos_sensor(0x89,0xec);
				SP2529MIPI_write_cmos_sensor(0x8a,0x44);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0xbe,0xe2);
				SP2529MIPI_write_cmos_sensor(0xbf,0x04);
				SP2529MIPI_write_cmos_sensor(0xd0,0xe2);
				SP2529MIPI_write_cmos_sensor(0xd1,0x04);
				SP2529MIPI_write_cmos_sensor(0xc9,0xe2);
				SP2529MIPI_write_cmos_sensor(0xca,0x04);
	

				SENSORDB("Ronlus video night mode 50hz\r\n");
			}
			else/*video night mode 60hz*/
			{ 
				SENSORDB("Ronlus video night mode 60hz\r\n");     
			    // 1.5pll  fix 10-10 fps video night mode 60hz 
				//luo-0924
				SP2529MIPI_write_cmos_sensor(0xfd,0x00);
				SP2529MIPI_write_cmos_sensor(0x03,0x02);
				SP2529MIPI_write_cmos_sensor(0x04,0x70);
				SP2529MIPI_write_cmos_sensor(0x05,0x00);
				SP2529MIPI_write_cmos_sensor(0x06,0x00);
				SP2529MIPI_write_cmos_sensor(0x07,0x00);
				SP2529MIPI_write_cmos_sensor(0x08,0x00);
				SP2529MIPI_write_cmos_sensor(0x09,0x03);
				SP2529MIPI_write_cmos_sensor(0x0a,0x9d);
				SP2529MIPI_write_cmos_sensor(0xfd,0x01);
				SP2529MIPI_write_cmos_sensor(0xf0,0x00);
				SP2529MIPI_write_cmos_sensor(0xf7,0x68);
				SP2529MIPI_write_cmos_sensor(0xf8,0x68);
				SP2529MIPI_write_cmos_sensor(0x02,0x0c);
				SP2529MIPI_write_cmos_sensor(0x03,0x01);
				SP2529MIPI_write_cmos_sensor(0x06,0x68);
				SP2529MIPI_write_cmos_sensor(0x07,0x00);
				SP2529MIPI_write_cmos_sensor(0x08,0x01);
				SP2529MIPI_write_cmos_sensor(0x09,0x00);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0x3d,0x0c);
				SP2529MIPI_write_cmos_sensor(0x3e,0x68);
				SP2529MIPI_write_cmos_sensor(0x3f,0x00);
				SP2529MIPI_write_cmos_sensor(0x88,0xec);
				SP2529MIPI_write_cmos_sensor(0x89,0xec);
				SP2529MIPI_write_cmos_sensor(0x8a,0x44);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0xbe,0xe0);
				SP2529MIPI_write_cmos_sensor(0xbf,0x04);
				SP2529MIPI_write_cmos_sensor(0xd0,0xe0);
				SP2529MIPI_write_cmos_sensor(0xd1,0x04);
				SP2529MIPI_write_cmos_sensor(0xc9,0xe0);
				SP2529MIPI_write_cmos_sensor(0xca,0x04);

			}
		}/*capture night mode*/
		else 
		{   
			SENSORDB("Ronlus capture night mode\r\n");
			if(SP2529_CAM_BANDING_50HZ == KAL_TRUE)/*capture night mode 50hz*/
			{	
			    /*capture night mode 50hz 1.5PLL 5-15fps*/
				SENSORDB(" capture night mode 50hz\r\n");//3                            
				//luo-0924
				SP2529MIPI_write_cmos_sensor(0xfd,0x00); 
				SP2529MIPI_write_cmos_sensor(0x03,0x04); 
				SP2529MIPI_write_cmos_sensor(0x04,0x5c); 
				SP2529MIPI_write_cmos_sensor(0x05,0x00); 
				SP2529MIPI_write_cmos_sensor(0x06,0x00); 
				SP2529MIPI_write_cmos_sensor(0x07,0x00); 
				SP2529MIPI_write_cmos_sensor(0x08,0x00); 
				SP2529MIPI_write_cmos_sensor(0x09,0x01); 
				SP2529MIPI_write_cmos_sensor(0x0a,0xc3); 
				SP2529MIPI_write_cmos_sensor(0xfd,0x01); 
				SP2529MIPI_write_cmos_sensor(0xf0,0x00); 
				SP2529MIPI_write_cmos_sensor(0xf7,0xba); 
				SP2529MIPI_write_cmos_sensor(0xf8,0x9b); 
				SP2529MIPI_write_cmos_sensor(0x02,0x14); 
				SP2529MIPI_write_cmos_sensor(0x03,0x01); 
				SP2529MIPI_write_cmos_sensor(0x06,0xba); 
				SP2529MIPI_write_cmos_sensor(0x07,0x00); 
				SP2529MIPI_write_cmos_sensor(0x08,0x01); 
				SP2529MIPI_write_cmos_sensor(0x09,0x00); 
				SP2529MIPI_write_cmos_sensor(0xfd,0x02); 
				SP2529MIPI_write_cmos_sensor(0x3d,0x18); 
				SP2529MIPI_write_cmos_sensor(0x3e,0x9b); 
				SP2529MIPI_write_cmos_sensor(0x3f,0x00); 
				SP2529MIPI_write_cmos_sensor(0x88,0xc0); 
				SP2529MIPI_write_cmos_sensor(0x89,0x4d); 
				SP2529MIPI_write_cmos_sensor(0x8a,0x32); 
				SP2529MIPI_write_cmos_sensor(0xfd,0x02); 
				SP2529MIPI_write_cmos_sensor(0xbe,0x88); 
				SP2529MIPI_write_cmos_sensor(0xbf,0x0e); 
				SP2529MIPI_write_cmos_sensor(0xd0,0x88); 
				SP2529MIPI_write_cmos_sensor(0xd1,0x0e); 
				SP2529MIPI_write_cmos_sensor(0xc9,0x88); 
				SP2529MIPI_write_cmos_sensor(0xca,0x0e); 



			}
			else/*capture night mode 60hz*/
			{ 
			    /*capture night mode 60hz 1.5PLL 5-15fps*/
				SENSORDB(" capture night mode 60hz\r\n");                                             
				//luo-0924
				SP2529MIPI_write_cmos_sensor(0xfd,0x00);
				SP2529MIPI_write_cmos_sensor(0x03,0x03);
				SP2529MIPI_write_cmos_sensor(0x04,0xa2);
				SP2529MIPI_write_cmos_sensor(0x05,0x00);
				SP2529MIPI_write_cmos_sensor(0x06,0x00);
				SP2529MIPI_write_cmos_sensor(0x07,0x00);
				SP2529MIPI_write_cmos_sensor(0x08,0x00);
				SP2529MIPI_write_cmos_sensor(0x09,0x01);
				SP2529MIPI_write_cmos_sensor(0x0a,0xc3);
				SP2529MIPI_write_cmos_sensor(0xfd,0x01);
				SP2529MIPI_write_cmos_sensor(0xf0,0x00);
				SP2529MIPI_write_cmos_sensor(0xf7,0x9b);
				SP2529MIPI_write_cmos_sensor(0xf8,0x9b);
				SP2529MIPI_write_cmos_sensor(0x02,0x18);
				SP2529MIPI_write_cmos_sensor(0x03,0x01);
				SP2529MIPI_write_cmos_sensor(0x06,0x9b);
				SP2529MIPI_write_cmos_sensor(0x07,0x00);
				SP2529MIPI_write_cmos_sensor(0x08,0x01);
				SP2529MIPI_write_cmos_sensor(0x09,0x00);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0x3d,0x18);
				SP2529MIPI_write_cmos_sensor(0x3e,0x9b);
				SP2529MIPI_write_cmos_sensor(0x3f,0x00);
				SP2529MIPI_write_cmos_sensor(0x88,0x4d);
				SP2529MIPI_write_cmos_sensor(0x89,0x4d);
				SP2529MIPI_write_cmos_sensor(0x8a,0x33);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0xbe,0x88);
				SP2529MIPI_write_cmos_sensor(0xbf,0x0e);
				SP2529MIPI_write_cmos_sensor(0xd0,0x88);
				SP2529MIPI_write_cmos_sensor(0xd1,0x0e);
				SP2529MIPI_write_cmos_sensor(0xc9,0x88);
				SP2529MIPI_write_cmos_sensor(0xca,0x0e);



			}
		}
	}
	else /*normal mode settings*/
	{
		SENSORDB("Ronlus normal mode\r\n");
		SP2529_CAM_Nightmode = 0;
	//	SP2529MIPI_write_cmos_sensor(0xfd,0x00);
	//	SP2529MIPI_write_cmos_sensor(0xb2,SP2529_NORMAL_Y0ffset);
		if (SP2529MIPI_MPEG4_encode_mode == KAL_TRUE) 
		{
			SENSORDB("Ronlus video normal mode\r\n");
			if(SP2529_CAM_BANDING_50HZ == KAL_TRUE)/*video normal mode 50hz*/
			{
				SENSORDB("Ronlus video normal mode 50hz\r\n");
				/*video normal mode 1.5Pll fix 15-15fps 50hz*/
				//luo-0924
				SP2529MIPI_write_cmos_sensor(0xfd,0x00); 
				SP2529MIPI_write_cmos_sensor(0x03,0x04); 
				SP2529MIPI_write_cmos_sensor(0x04,0x5c); 
				SP2529MIPI_write_cmos_sensor(0x05,0x00); 
				SP2529MIPI_write_cmos_sensor(0x06,0x00); 
				SP2529MIPI_write_cmos_sensor(0x07,0x00); 
				SP2529MIPI_write_cmos_sensor(0x08,0x00); 
				SP2529MIPI_write_cmos_sensor(0x09,0x01); 
				SP2529MIPI_write_cmos_sensor(0x0a,0xc3); 
				SP2529MIPI_write_cmos_sensor(0xfd,0x01); 
				SP2529MIPI_write_cmos_sensor(0xf0,0x00); 
				SP2529MIPI_write_cmos_sensor(0xf7,0xba); 
				SP2529MIPI_write_cmos_sensor(0xf8,0x9b); 
				SP2529MIPI_write_cmos_sensor(0x02,0x06); 
				SP2529MIPI_write_cmos_sensor(0x03,0x01); 
				SP2529MIPI_write_cmos_sensor(0x06,0xba); 
				SP2529MIPI_write_cmos_sensor(0x07,0x00); 
				SP2529MIPI_write_cmos_sensor(0x08,0x01); 
				SP2529MIPI_write_cmos_sensor(0x09,0x00); 
				SP2529MIPI_write_cmos_sensor(0xfd,0x02); 
				SP2529MIPI_write_cmos_sensor(0x3d,0x08); 
				SP2529MIPI_write_cmos_sensor(0x3e,0x9b); 
				SP2529MIPI_write_cmos_sensor(0x3f,0x00); 
				SP2529MIPI_write_cmos_sensor(0x88,0xc0); 
				SP2529MIPI_write_cmos_sensor(0x89,0x4d); 
				SP2529MIPI_write_cmos_sensor(0x8a,0x32); 
				SP2529MIPI_write_cmos_sensor(0xfd,0x02); 
				SP2529MIPI_write_cmos_sensor(0xbe,0x5c); 
				SP2529MIPI_write_cmos_sensor(0xbf,0x04); 
				SP2529MIPI_write_cmos_sensor(0xd0,0x5c); 
				SP2529MIPI_write_cmos_sensor(0xd1,0x04); 
				SP2529MIPI_write_cmos_sensor(0xc9,0x5c); 
				SP2529MIPI_write_cmos_sensor(0xca,0x04); 

			}
			else/*video normal mode 60hz*/
			{
				SENSORDB(" video normal mode 60hz\r\n");  
				/*video normal mode 1.5Pll fix 15-15fps 60hz*/
				//luo-0924
				SP2529MIPI_write_cmos_sensor(0xfd,0x00);
				SP2529MIPI_write_cmos_sensor(0x03,0x03);
				SP2529MIPI_write_cmos_sensor(0x04,0xa2);
				SP2529MIPI_write_cmos_sensor(0x05,0x00);
				SP2529MIPI_write_cmos_sensor(0x06,0x00);
				SP2529MIPI_write_cmos_sensor(0x07,0x00);
				SP2529MIPI_write_cmos_sensor(0x08,0x00);
				SP2529MIPI_write_cmos_sensor(0x09,0x01);
				SP2529MIPI_write_cmos_sensor(0x0a,0xc3);
				SP2529MIPI_write_cmos_sensor(0xfd,0x01);
				SP2529MIPI_write_cmos_sensor(0xf0,0x00);
				SP2529MIPI_write_cmos_sensor(0xf7,0x9b);
				SP2529MIPI_write_cmos_sensor(0xf8,0x9b);
				SP2529MIPI_write_cmos_sensor(0x02,0x08);
				SP2529MIPI_write_cmos_sensor(0x03,0x01);
				SP2529MIPI_write_cmos_sensor(0x06,0x9b);
				SP2529MIPI_write_cmos_sensor(0x07,0x00);
				SP2529MIPI_write_cmos_sensor(0x08,0x01);
				SP2529MIPI_write_cmos_sensor(0x09,0x00);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0x3d,0x08);
				SP2529MIPI_write_cmos_sensor(0x3e,0x9b);
				SP2529MIPI_write_cmos_sensor(0x3f,0x00);
				SP2529MIPI_write_cmos_sensor(0x88,0x4d);
				SP2529MIPI_write_cmos_sensor(0x89,0x4d);
				SP2529MIPI_write_cmos_sensor(0x8a,0x33);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0xbe,0xd8);
				SP2529MIPI_write_cmos_sensor(0xbf,0x04);
				SP2529MIPI_write_cmos_sensor(0xd0,0xd8);
				SP2529MIPI_write_cmos_sensor(0xd1,0x04);
				SP2529MIPI_write_cmos_sensor(0xc9,0xd8);
				SP2529MIPI_write_cmos_sensor(0xca,0x04);


			}
		}
		else/*capture normal mode*/
		{
			SENSORDB("Ronlus capture normal mode\r\n");
			if(SP2529_CAM_BANDING_50HZ == KAL_TRUE)/*capture normal mode 50hz*/
			{
				SENSORDB(" capture normal mode 50hz\r\n");
				/*capture normal mode 1.5pll 10-22.0695fps 50hz*/

				SP2529MIPI_write_cmos_sensor(0xfd,0x00);
				SP2529MIPI_write_cmos_sensor(0x03,0x06);
				SP2529MIPI_write_cmos_sensor(0x04,0x66);
				SP2529MIPI_write_cmos_sensor(0x05,0x00);
				SP2529MIPI_write_cmos_sensor(0x06,0x00);
				SP2529MIPI_write_cmos_sensor(0x07,0x00);
				SP2529MIPI_write_cmos_sensor(0x08,0x00);
				SP2529MIPI_write_cmos_sensor(0x09,0x00);
				SP2529MIPI_write_cmos_sensor(0x0a,0x8e);
				SP2529MIPI_write_cmos_sensor(0xfd,0x01);
				SP2529MIPI_write_cmos_sensor(0xf0,0x01);
				SP2529MIPI_write_cmos_sensor(0xf7,0x11);
				SP2529MIPI_write_cmos_sensor(0xf8,0xe4);
				SP2529MIPI_write_cmos_sensor(0x02,0x0a);
				SP2529MIPI_write_cmos_sensor(0x03,0x01);
				SP2529MIPI_write_cmos_sensor(0x06,0x11);
				SP2529MIPI_write_cmos_sensor(0x07,0x01);
				SP2529MIPI_write_cmos_sensor(0x08,0x01);
				SP2529MIPI_write_cmos_sensor(0x09,0x00);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0x3d,0x0c);
				SP2529MIPI_write_cmos_sensor(0x3e,0xe4);
				SP2529MIPI_write_cmos_sensor(0x3f,0x00);
				SP2529MIPI_write_cmos_sensor(0x88,0xe0);
				SP2529MIPI_write_cmos_sensor(0x89,0x3e);
				SP2529MIPI_write_cmos_sensor(0x8a,0x21);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0xbe,0xaa);
				SP2529MIPI_write_cmos_sensor(0xbf,0x0a);
				SP2529MIPI_write_cmos_sensor(0xd0,0xaa);
				SP2529MIPI_write_cmos_sensor(0xd1,0x0a);
				SP2529MIPI_write_cmos_sensor(0xc9,0xaa);
				SP2529MIPI_write_cmos_sensor(0xca,0x0a);

			}
			else/*video normal mode 60hz*/
			{
				SENSORDB(" capture normal mode 60hz\r\n");
				/*capture normal mode 1.5pll 10-15fps 60hz*/
				//luo-0924
				SP2529MIPI_write_cmos_sensor(0xfd,0x00);
				SP2529MIPI_write_cmos_sensor(0x03,0x03);
				SP2529MIPI_write_cmos_sensor(0x04,0xa2);
				SP2529MIPI_write_cmos_sensor(0x05,0x00);
				SP2529MIPI_write_cmos_sensor(0x06,0x00);
				SP2529MIPI_write_cmos_sensor(0x07,0x00);
				SP2529MIPI_write_cmos_sensor(0x08,0x00);
				SP2529MIPI_write_cmos_sensor(0x09,0x01);
				SP2529MIPI_write_cmos_sensor(0x0a,0xc3);
				SP2529MIPI_write_cmos_sensor(0xfd,0x01);
				SP2529MIPI_write_cmos_sensor(0xf0,0x00);
				SP2529MIPI_write_cmos_sensor(0xf7,0x9b);
				SP2529MIPI_write_cmos_sensor(0xf8,0x9b);
				SP2529MIPI_write_cmos_sensor(0x02,0x0c);
				SP2529MIPI_write_cmos_sensor(0x03,0x01);
				SP2529MIPI_write_cmos_sensor(0x06,0x9b);
				SP2529MIPI_write_cmos_sensor(0x07,0x00);
				SP2529MIPI_write_cmos_sensor(0x08,0x01);
				SP2529MIPI_write_cmos_sensor(0x09,0x00);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0x3d,0x0c);
				SP2529MIPI_write_cmos_sensor(0x3e,0x9b);
				SP2529MIPI_write_cmos_sensor(0x3f,0x00);
				SP2529MIPI_write_cmos_sensor(0x88,0x4d);
				SP2529MIPI_write_cmos_sensor(0x89,0x4d);
				SP2529MIPI_write_cmos_sensor(0x8a,0x33);
				SP2529MIPI_write_cmos_sensor(0xfd,0x02);
				SP2529MIPI_write_cmos_sensor(0xbe,0x44);
				SP2529MIPI_write_cmos_sensor(0xbf,0x07);
				SP2529MIPI_write_cmos_sensor(0xd0,0x44);
				SP2529MIPI_write_cmos_sensor(0xd1,0x07);
				SP2529MIPI_write_cmos_sensor(0xc9,0x44);
				SP2529MIPI_write_cmos_sensor(0xca,0x07);



			}
		}
	}

#endif
}	/* SP2529MIPI_night_mode */

//shaohui add for DEVINFO CMM
#ifdef SLT_DEVINFO_CMM 
#include  <linux/dev_info.h>
static struct devinfo_struct *s_DEVINFO_ccm;   //suppose 10 max lcm device 

#endif
/*************************************************************************
* FUNCTION
*	SP2529MIPI_GetSensorID
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
static kal_uint32 SP2529MIPI_GetSensorID(kal_uint32 *sensorID)
{
    //volatile signed char i;
	kal_uint16 sensor_id=0,sensor_id_2=0;

     // check if sensor ID correct


#ifdef SLT_DEVINFO_CMM 
	char mid_info=0;
	char dev_vendorlist[0x20][20]={"null","sunny","truly","A-kerr","LiteArray","Darling","Qtech",
		"OFlim","Huaquan"};
 s_DEVINFO_ccm =(struct devinfo_struct*) kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);	
	s_DEVINFO_ccm->device_type = "CCM";
	s_DEVINFO_ccm->device_module = "T8200 R1.0";//can change if got module id
	s_DEVINFO_ccm->device_vendor = "KARR";
	s_DEVINFO_ccm->device_ic = "SP2529";
	s_DEVINFO_ccm->device_version = "SuperPix";
	s_DEVINFO_ccm->device_info = "200W";
	
#endif
	SP2529MIPI_write_cmos_sensor(0xfd,0x00); 	
	 sensor_id  =SP2529MIPI_read_cmos_sensor(0xA0);
	// sensor_id_2=SP2529MIPI_read_cmos_sensor(0x0001);
	 //sensor_id = (sensor_id<<8)+sensor_id_2;
	 printk("GetID Sensor Read SP2529MIPI ID 0x%x  \r\n", (unsigned int)sensor_id);
	 *sensorID=sensor_id;
	if (sensor_id != SP2529MIPI_SENSOR_ID)
	{
	    *sensorID=0xFFFFFFFF;
#ifdef SLT_DEVINFO_CMM 
	s_DEVINFO_ccm->device_used = DEVINFO_UNUSED;
	devinfo_check_add_device(s_DEVINFO_ccm);
#endif
	    SENSORDB("Sensor Read ByeBye \r\n");
		return ERROR_SENSOR_CONNECT_FAIL;
	}
#ifdef SLT_DEVINFO_CMM 
	s_DEVINFO_ccm->device_used = DEVINFO_USED;
	devinfo_check_add_device(s_DEVINFO_ccm);
#endif
    return ERROR_NONE;    
}  


/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	SP2529MIPIOpen
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
UINT32 SP2529MIPIOpen(void)
{
	//volatile signed char i;
	kal_uint16 sensor_id=0,sensor_id_2=0;
	SP2529MIPI_write_cmos_sensor(0xfd,0x00); 	
	sensor_id  =SP2529MIPI_read_cmos_sensor(0xA0);
	//sensor_id_2=SP2529MIPI_read_cmos_sensor(0x0001);
	//sensor_id = (sensor_id<<8)+sensor_id_2;
	printk("Open Sensor Read SP2529MIPI ID %x \r\n",(unsigned int)sensor_id);
	
	if (sensor_id != SP2529MIPI_SENSOR_ID)
	{
	    SENSORDB("Sensor Read ByeBye \r\n");
		return ERROR_SENSOR_CONNECT_FAIL;
	}

#ifdef DEBUGE_SENSOR_SP0A28//gepeiwei   120903

	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static char buf[10*1024] ;

	fp = filp_open("/storage/sdcard0/sp2529_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) 
	{ 
		printk("open file form shouji error\n"); 
		fp = filp_open("/storage/sdcard1/sp2529_sd", O_RDONLY , 0); 
		if (IS_ERR(fp))
		{
			 fromsd = 0; 
			 printk("open file form T error\n"); 
		}
		else
		 {
			fromsd = 1;
			printk("open file ok\n");
			//SP0A28_Initialize_from_T_Flash();
			filp_close(fp, NULL); 
			set_fs(fs);
		}

	} 

	else 
	{
		fromsd = 1;
		printk("open file ok\n");

		//SP0A28_Initialize_from_T_Flash();


		filp_close(fp, NULL); 
		set_fs(fs);
	}
#endif
	
    SP2529MIPIInitialPara(); 
	
	SP2529MIPI_Initialize_Setting();
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	SP2529MIPIClose
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
UINT32 SP2529MIPIClose(void)
{
	return ERROR_NONE;
}	/* SP2529MIPIClose() */

//SP_CMSTART
LOCAL void SP2529MIPI_AfterSnapshot(void)
{
    kal_uint8  	AE_tmp;
	#if 1
	kal_uint8   tmp1,tmp2,tmp3;
	#endif
	

	SP2529MIPI_write_cmos_sensor(0xfd,0x01);
	AE_tmp=SP2529MIPI_read_cmos_sensor(0x32);
	AE_tmp=AE_tmp&0xfa;
	
	SP2529MIPI_write_cmos_sensor(0x32,AE_tmp);
	SP2529MIPI_write_cmos_sensor(0xfd,0x00);
	SP2529MIPI_write_cmos_sensor(0xe7,0x03);
	SP2529MIPI_write_cmos_sensor(0xe7,0x00);
	
	SP2529MIPI_Set_Shutter(G_shutter);
	
	SP2529MIPI_write_cmos_sensor(0xfd,0x00); 
	SP2529MIPI_write_cmos_sensor(0x24,G_Gain);
	
	SP2529MIPI_write_cmos_sensor(0xfd,0x00);
	SP2529MIPI_write_cmos_sensor(0xe7,0x03);
	SP2529MIPI_write_cmos_sensor(0xe7,0x00);

	SP2529MIPI_write_cmos_sensor(0xfd,0x01);
	AE_tmp=SP2529MIPI_read_cmos_sensor(0x32);
	AE_tmp=AE_tmp|0x05;
	SP2529MIPI_write_cmos_sensor(0x32,AE_tmp);
	SP2529MIPI_write_cmos_sensor(0xfd,0x00);
	SP2529MIPI_write_cmos_sensor(0xe7,0x03);
	SP2529MIPI_write_cmos_sensor(0xe7,0x00);

	return 0;
}  


LOCAL void SP2529MIPI_BeforeSnapshot(void)
{	
	kal_uint32 Shutter_temp;
    kal_uint16 Shutter, HB;
    kal_uint8  	HB_L,HB_H,Gain,AE_tmp;

	SP2529MIPI_write_cmos_sensor(0xfd,0x01);
	AE_tmp=SP2529MIPI_read_cmos_sensor(0x32);
	SP2529MIPI_write_cmos_sensor(0xfd,0x00); 
	Gain=SP2529MIPI_read_cmos_sensor(0x23);

	HB_H=SP2529MIPI_read_cmos_sensor(0x09);
	HB_L=SP2529MIPI_read_cmos_sensor(0x0a);

	G_Gain=Gain;
	HB = (HB_L & 0xFF) | (HB_H << 8);
	AE_tmp=AE_tmp&0xfa;
	Shutter = SP2529MIPI_Read_Shutter();
	G_shutter = Shutter;
	
	Shutter_temp = Shutter;	
	Shutter_temp = Shutter_temp*(517+HB)/2/(922+HB);	
	Shutter = Shutter_temp&0xffff;

	if(Shutter<1)
	{
		Shutter=1;
	}
	SP2529MIPI_write_cmos_sensor(0xfd,0x01);
	SP2529MIPI_write_cmos_sensor(0x32,AE_tmp);  
	SP2529MIPI_write_cmos_sensor(0xfd,0x00);
	SP2529MIPI_write_cmos_sensor(0xe7,0x03);
	SP2529MIPI_write_cmos_sensor(0xe7,0x00);
	
	SP2529MIPI_Set_Shutter(Shutter);	
	SP2529MIPI_write_cmos_sensor(0xfd,0x00); 
	SP2529MIPI_write_cmos_sensor(0x24,Gain);
	
	SP2529MIPI_write_cmos_sensor(0xfd,0x00);
	SP2529MIPI_write_cmos_sensor(0xe7,0x03);
	SP2529MIPI_write_cmos_sensor(0xe7,0x00);

return 0;  
} 
//SP_CMEND
/*************************************************************************
* FUNCTION
*	SP2529MIPIPreview
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
UINT32 SP2529MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	spin_lock(&ov9740mipi_yuv_drv_lock);
//SP_CMSTART  
	SP2529MIPI_PV_Mode();

    if(setshutter)
    {
	    SP2529MIPI_AfterSnapshot();  //xg test
    }
	setshutter = KAL_FALSE;
//SP_CMEND	
	SP2529MIPI_sensor_cap_state = KAL_FALSE;
	
	if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
	{
		SP2529MIPI_MPEG4_encode_mode = KAL_TRUE;		
		SP2529MIPI_MJPEG_encode_mode = KAL_FALSE;
	}
	else
	{
		SP2529MIPI_MPEG4_encode_mode = KAL_FALSE;		
		SP2529MIPI_MJPEG_encode_mode = KAL_FALSE;
		
	}
	spin_unlock(&ov9740mipi_yuv_drv_lock);
	
	
	SP2529MIPI_night_mode(SP2529MIPICurrentStatus.iNightMode);
	SP2529MIPI_set_mirror(IMAGE_NORMAL);
	
	//just add for porting,please delete this when release
	//SP2529MIPI_set_mirror(IMAGE_HV_MIRROR);

    image_window->ExposureWindowWidth = SP2529MIPI_IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight = SP2529MIPI_IMAGE_SENSOR_PV_HEIGHT;
	
	// copy sensor_config_data
	memcpy(&SP2529MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
  	return ERROR_NONE;
}	/* SP2529MIPIPreview() */

UINT32 SP2529MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
     //kal_uint32 pv_integration_time = 0;       // Uinit - us
     //kal_uint32 cap_integration_time = 0;
     //kal_uint16 PV_line_len = 0;
     //kal_uint16 CAP_line_len = 0;
	 SP2529MIPI_CAP_Mode();   

	SP2529MIPI_BeforeSnapshot();
	mdelay(10);

	spin_lock(&ov9740mipi_yuv_drv_lock);
	SP2529MIPI_sensor_cap_state = KAL_TRUE;
	spin_unlock(&ov9740mipi_yuv_drv_lock);
               
         

    image_window->GrabStartX = SP2529MIPI_IMAGE_SENSOR_FULL_INSERTED_PIXELS;
    image_window->GrabStartY = SP2529MIPI_IMAGE_SENSOR_FULL_INSERTED_LINES;
    image_window->ExposureWindowWidth= SP2529MIPI_IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = SP2529MIPI_IMAGE_SENSOR_FULL_HEIGHT;
//SP_CMSTART
	if(!setshutter)
	{
		//SP2529_ae_enable(KAL_FALSE);
		//SP2529_awb_enable(KAL_FALSE);
		//shutter = SP2529_Read_Shutter();
		//SP2529_CAP_setting();
		//SP2529_set_hb_shutter(cap_dummy_pixels, shutter);
	}

	setshutter = KAL_TRUE;
//SP_CMEND
     // copy sensor_config_data
     memcpy(&SP2529MIPISensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
     return ERROR_NONE;
}        /* SP2529MIPICapture() */

UINT32 SP2529MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=SP2529MIPI_IMAGE_SENSOR_FULL_WIDTH;  //modify by yanxu
	pSensorResolution->SensorFullHeight=SP2529MIPI_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth=SP2529MIPI_IMAGE_SENSOR_PV_WIDTH; 
	pSensorResolution->SensorPreviewHeight=SP2529MIPI_IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth=SP2529MIPI_IMAGE_SENSOR_PV_WIDTH; 
	pSensorResolution->SensorVideoHeight=SP2529MIPI_IMAGE_SENSOR_PV_HEIGHT;
	return ERROR_NONE;
}	/* SP2529MIPIGetResolution() */

UINT32 SP2529MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	#if 0
    switch(ScenarioId)
    {
        
    	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 pSensorInfo->SensorPreviewResolutionX=SP2529MIPI_IMAGE_SENSOR_FULL_WIDTH;
	         pSensorInfo->SensorPreviewResolutionY=SP2529MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			 pSensorInfo->SensorCameraPreviewFrameRate=15;
			 break;
		
		default:
	#else
	{
	#endif
	
			 pSensorInfo->SensorPreviewResolutionX=SP2529MIPI_IMAGE_SENSOR_PV_WIDTH;
	         pSensorInfo->SensorPreviewResolutionY=SP2529MIPI_IMAGE_SENSOR_PV_HEIGHT;
			 pSensorInfo->SensorCameraPreviewFrameRate=30;
    }
	pSensorInfo->SensorFullResolutionX=SP2529MIPI_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=SP2529MIPI_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YVYU;//SENSOR_OUTPUT_FORMAT_YUYV;
	
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;

	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	
	#ifdef MIPI_INTERFACE
   		pSensorInfo->SensroInterfaceType        = SENSOR_INTERFACE_TYPE_MIPI;
   	#else
   		pSensorInfo->SensroInterfaceType		= SENSOR_INTERFACE_TYPE_PARALLEL;
   	#endif
	
#ifdef MIPI_INTERFACE
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
#endif
	pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;//ISP_DRIVING_4MA;   	
	pSensorInfo->CaptureDelayFrame = 1;////3; 
	pSensorInfo->PreviewDelayFrame = 1;////3; 
	pSensorInfo->VideoDelayFrame = 2;////4; 
	
	switch (ScenarioId) 
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			
			pSensorInfo->SensorDataLatchCount= 2;
			
			pSensorInfo->SensorGrabStartX = 2; 
			pSensorInfo->SensorGrabStartY = 2; 
#ifdef MIPI_INTERFACE
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
#endif
		break;

		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = 2; 
			pSensorInfo->SensorGrabStartY = 2; 					
			#ifdef MIPI_INTERFACE
	            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
	            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
	            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
	            pSensorInfo->SensorPacketECCOrder = 1;
	        	#endif			
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
	memcpy(pSensorConfigData, &SP2529MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* SP2529MIPIGetInfo() */


UINT32 SP2529MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			SP2529MIPIPreview(pImageWindow, pSensorConfigData);
		break;
		
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			SP2529MIPICapture(pImageWindow, pSensorConfigData);
		break;
		
		#if 0
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			   SP2529MIPICapture(pImageWindow, pSensorConfigData);
			break;
		#endif
		
        default:
            return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* SP2529MIPIControl() */

/* [TC] YUV sensor */	
#if WINMO_USE
void SP2529MIPIQuery(PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo)
{
	MSDK_FEATURE_TYPE_RANGE_STRUCT *pFeatureRange;
	MSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT *pFeatureMultiSelection;
	switch (pSensorFeatureInfo->FeatureId)
	{
		case ISP_FEATURE_DSC_MODE:
			pSensorFeatureInfo->FeatureType = MSDK_FEATURE_TYPE_MULTI_SELECTION;
			pSensorFeatureInfo->FeatureSupported = (UINT8)(MSDK_SET_GET_FEATURE_SUPPORTED|MSDK_QUERY_CAMERA_SUPPORTED);
			pFeatureMultiSelection = (PMSDK_FEATURE_TYPE_MULTI_SELECTION_STRUCT)(&pSensorFeatureInfo->FeatureInformation.FeatureMultiSelection);
			pFeatureMultiSelection->TotalSelection = CAM_NO_OF_SCENE_MODE_MAX-2;
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
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_GRAYINV)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SEPIABLUE)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_SKETCH)|
				CAMERA_FEATURE_SUPPORT(CAM_EFFECT_ENC_EMBOSSMENT)|
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
			pFeatureMultiSelection->TotalSelection = 2;
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

BOOL SP2529MIPI_set_param_wb(UINT16 para)
{

	//if(SP2529MIPICurrentStatus.iWB == para)
	//	return TRUE;
	SENSORDB("[Enter]SP2529MIPI set_param_wb func:para = %d\n",para);

	switch (para)
	{
		case AWB_MODE_AUTO:
			SP2529MIPI_write_cmos_sensor(0xfd,0x02);
			SP2529MIPI_write_cmos_sensor(0x26,0xac);
			SP2529MIPI_write_cmos_sensor(0x27,0x91);	   
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0x32,0x15);
		break;
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0x32,0x05);
			SP2529MIPI_write_cmos_sensor(0xfd,0x02);
			SP2529MIPI_write_cmos_sensor(0x26,0xe2);
			SP2529MIPI_write_cmos_sensor(0x27,0x82);
			SP2529MIPI_write_cmos_sensor(0xfd,0x00);
		break;
		case AWB_MODE_DAYLIGHT: //sunny
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0x32,0x05);
			SP2529MIPI_write_cmos_sensor(0xfd,0x02);
			SP2529MIPI_write_cmos_sensor(0x26,0xc1);
			SP2529MIPI_write_cmos_sensor(0x27,0x88);
			SP2529MIPI_write_cmos_sensor(0xfd,0x00);
		break;
		case AWB_MODE_INCANDESCENT: //office
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0x32,0x05);
			SP2529MIPI_write_cmos_sensor(0xfd,0x02);
			SP2529MIPI_write_cmos_sensor(0x26,0x7b);
			SP2529MIPI_write_cmos_sensor(0x27,0xd3);
			SP2529MIPI_write_cmos_sensor(0xfd,0x00);
		break;
		case AWB_MODE_TUNGSTEN: //home
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0x32,0x05);
			SP2529MIPI_write_cmos_sensor(0xfd,0x02);
			SP2529MIPI_write_cmos_sensor(0x26,0xae);
			SP2529MIPI_write_cmos_sensor(0x27,0xcc);
			SP2529MIPI_write_cmos_sensor(0xfd,0x00);
		break;
		case AWB_MODE_FLUORESCENT:
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0x32,0x05);
			SP2529MIPI_write_cmos_sensor(0xfd,0x02);
			SP2529MIPI_write_cmos_sensor(0x26,0xb4);
			SP2529MIPI_write_cmos_sensor(0x27,0xc4);
			SP2529MIPI_write_cmos_sensor(0xfd,0x00);
		break;	
		default:
		return FALSE;
	}
		spin_lock(&ov9740mipi_yuv_drv_lock);
	    SP2529MIPICurrentStatus.iWB = para;
		spin_unlock(&ov9740mipi_yuv_drv_lock);
return TRUE;
}

BOOL SP2529MIPI_set_param_effect(UINT16 para)
{
	/*----------------------------------------------------------------*/
	   /* Local Variables												 */
	   /*----------------------------------------------------------------*/
	   kal_uint32 ret = KAL_TRUE;
	   kal_uint8  SDE_1, SDE_2;
	   /*----------------------------------------------------------------*/
	   /* Code Body 													 */
	   /*----------------------------------------------------------------*/
	 //  if(SP2529MIPICurrentStatus.iEffect == para)
	//	  return TRUE;
	   SENSORDB("[Enter]ov9740mipi_yuv set_param_effect func:para = %d\n",para);



	   switch (para)
	   {
		   case MEFFECT_OFF:
			   SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			   SP2529MIPI_write_cmos_sensor(0x66,0x00);
			   SP2529MIPI_write_cmos_sensor(0x67,0x80);
			   SP2529MIPI_write_cmos_sensor(0x68,0x80);
			   SP2529MIPI_write_cmos_sensor(0xdb,0x00);
			  // SP2529MIPI_write_cmos_sensor(0x34,0xc7);
			 //  SP2529MIPI_write_cmos_sensor(0xfd,0x02);
			  // SP2529MIPI_write_cmos_sensor(0x14,0x00);
			   break;
		   case MEFFECT_MONO:
			   SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			   SP2529MIPI_write_cmos_sensor(0x66,0x20);
			   SP2529MIPI_write_cmos_sensor(0x67,0x80);
			   SP2529MIPI_write_cmos_sensor(0x68,0x80);
			   SP2529MIPI_write_cmos_sensor(0xdb,0x00);
			   //SP2529MIPI_write_cmos_sensor(0x34,0xc7);
			  // SP2529MIPI_write_cmos_sensor(0xfd,0x02);
			   //SP2529MIPI_write_cmos_sensor(0x14,0x00);
			   break;
		   case MEFFECT_SEPIA:
			   SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			   SP2529MIPI_write_cmos_sensor(0x66,0x10);
			   SP2529MIPI_write_cmos_sensor(0x67,0x98);
			   SP2529MIPI_write_cmos_sensor(0x68,0x58);
			   SP2529MIPI_write_cmos_sensor(0xdb,0x00);
			 //  SP2529MIPI_write_cmos_sensor(0x34,0xc7);
			 //  SP2529MIPI_write_cmos_sensor(0xfd,0x02);
			 //  SP2529MIPI_write_cmos_sensor(0x14,0x00);
			   break;
		   case MEFFECT_SEPIABLUE:
			   SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			   SP2529MIPI_write_cmos_sensor(0x66,0x10);
			   SP2529MIPI_write_cmos_sensor(0x67,0x80);
			   SP2529MIPI_write_cmos_sensor(0x68,0xb0);
			   SP2529MIPI_write_cmos_sensor(0xdb,0x00);
			//   SP2529MIPI_write_cmos_sensor(0x34,0xc7);
			//   SP2529MIPI_write_cmos_sensor(0xfd,0x02);
			//   SP2529MIPI_write_cmos_sensor(0x14,0x00);
			   break;
		   case MEFFECT_NEGATIVE:
			   SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			   SP2529MIPI_write_cmos_sensor(0x66,0x08);
			   SP2529MIPI_write_cmos_sensor(0x67,0x80);
			   SP2529MIPI_write_cmos_sensor(0x68,0x80);
			   SP2529MIPI_write_cmos_sensor(0xdb,0x00);
			//   SP2529MIPI_write_cmos_sensor(0x34,0xc7);
			//   SP2529MIPI_write_cmos_sensor(0xfd,0x02);
			//   SP2529MIPI_write_cmos_sensor(0x14,0x00);
			   break;
		   default:
			   ret = KAL_FALSE;
			   break;
	   }
	   
	   spin_lock(&ov9740mipi_yuv_drv_lock);
	   SP2529MIPICurrentStatus.iEffect = para;
	   spin_unlock(&ov9740mipi_yuv_drv_lock);
	   return ret;
} 


BOOL SP2529MIPI_set_param_banding(UINT16 para)
{
	kal_uint8 m_banding_auto;
	kal_uint8 m_banding_sel;  
	kal_uint8 m_banding_sel_set;  

//	if(SP2529MIPICurrentStatus.iBanding == para)
//		return TRUE;
	
	SENSORDB("SP2529MIPI_set_param_banding %d  \r\n", para);
	SENSORDB("50hz %d  \r\n", AE_FLICKER_MODE_50HZ);
	SENSORDB("60hz %d  \r\n", AE_FLICKER_MODE_60HZ);
#if 0
	m_banding_auto  =SP2529MIPI_read_cmos_sensor(0x3C01);
	//m_banding_sel   =SP2529MIPI_read_cmos_sensor(0x3C0C);//read only
	m_banding_sel_set = SP2529MIPI_read_cmos_sensor(0x3C00);
	

	m_banding_auto = m_banding_auto & 0x7F;
	//m_banding_sel = m_banding_sel   & 0xFA;
	m_banding_sel_set &= 0xfb;
	
#endif

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
	spin_lock(&ov9740mipi_yuv_drv_lock);
    SP2529MIPICurrentStatus.iBanding = para;
	spin_unlock(&ov9740mipi_yuv_drv_lock);
	return TRUE;
} /* SP2529MIPI_set_param_banding */

BOOL SP2529MIPI_set_param_exposure(UINT16 para)
{

	//if(SP2529MIPICurrentStatus.iEV == para)
	//	return TRUE;
	
	SENSORDB("[Enter]ov9740mipi_yuv set_param_exposure func:para = %d\n",para);
	  
	//SP2529MIPI_write_cmos_sensor(0x0028, 0x7000);	//question  这里原来是打开的
    switch (para)
	{
		case AE_EV_COMP_n13:              /* EV -2 */
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0xdb,0xe0);
			break;
		case AE_EV_COMP_n10:              /* EV -1.5 */
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0xdb,0xe8);
			break;
		case AE_EV_COMP_n07:              /* EV -1 */
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0xdb,0xf0);
			break;
		case AE_EV_COMP_n03:              /* EV -0.5 */
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0xdb,0xf8);
			break;
		case AE_EV_COMP_00:                /* EV 0 */
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0xdb,0x00);
			break;
		case AE_EV_COMP_03:              /* EV +0.5 */
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0xdb,0x08);
			break;
		case AE_EV_COMP_07:              /* EV +1 */
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0xdb,0x10);
			break;
		case AE_EV_COMP_10:              /* EV +1.5 */
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0xdb,0x18);
			break;
		case AE_EV_COMP_13:              /* EV +2 */
			SP2529MIPI_write_cmos_sensor(0xfd,0x01);
			SP2529MIPI_write_cmos_sensor(0xdb,0x20);
			break;
		default:
			return FALSE;
	}
	spin_lock(&ov9740mipi_yuv_drv_lock);
	SP2529MIPICurrentStatus.iEV = para;
	spin_unlock(&ov9740mipi_yuv_drv_lock);
	return TRUE;

}/* SP2529MIPI_set_param_exposure */


UINT32 SP2529MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{

#ifdef DEBUGE_SENSOR_SP0A28
	return TRUE;
#endif

	switch (iCmd) {
	case FID_SCENE_MODE:
	    if (iPara == SCENE_MODE_OFF)
	    {
	        SP2529MIPI_night_mode(0); 
	    }

         else if (iPara == SCENE_MODE_NIGHTSCENE)			
	    {
            SP2529MIPI_night_mode(1); 
	    }	    
	    break; 	    
	case FID_AWB_MODE:
         SP2529MIPI_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
         SP2529MIPI_set_param_effect(iPara);
	break;
	case FID_AE_EV:  	    
         SP2529MIPI_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
         SP2529MIPI_set_param_banding(iPara);
		  SP2529MIPI_night_mode(SP2529_CAM_Nightmode); 
	break;
    case FID_AE_SCENE_MODE:  
		spin_lock(&ov9740mipi_yuv_drv_lock);
      if (iPara == AE_MODE_OFF) {
          SP2529MIPI_AE_ENABLE = KAL_FALSE; 
      }
      else {
          SP2529MIPI_AE_ENABLE = KAL_TRUE; 
      }
	  spin_unlock(&ov9740mipi_yuv_drv_lock);
      SP2529MIPI_set_AE_mode(SP2529MIPI_AE_ENABLE);
    break; 
	case FID_ZOOM_FACTOR:
		spin_lock(&ov9740mipi_yuv_drv_lock);
	    zoom_factor = iPara; 
		spin_unlock(&ov9740mipi_yuv_drv_lock);
	break; 
	default:
	break;
	}
	return ERROR_NONE;
}   /* SP2529MIPIYUVSensorSetting */


UINT32 SP2529MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{

    if(SP2529MIPICurrentStatus.iFrameRate == u2FrameRate)
      return ERROR_NONE;

	spin_lock(&ov9740mipi_yuv_drv_lock);
    SP2529MIPI_VEDIO_encode_mode = KAL_TRUE; 
	SP2529MIPI_MPEG4_encode_mode = KAL_TRUE;
	spin_unlock(&ov9740mipi_yuv_drv_lock);
	
/*
	SP2529MIPI_write_cmos_sensor(0x3A00,0x78); 
	SP2529MIPI_write_cmos_sensor(0x3a14,0x02);//15);	// 50Hz Max exposure
	SP2529MIPI_write_cmos_sensor(0x3a15,0xf0);//c6);	// 50Hz Max exposure
	SP2529MIPI_write_cmos_sensor(0x3a02,0x02);//18);	// 60Hz Max exposure
	SP2529MIPI_write_cmos_sensor(0x3a03,0xf0);//20);	// 60Hz Max exposure */

    if(20<=u2FrameRate && u2FrameRate<=30) //fix 30
    {		
		//SP2529MIPI_write_cmos_sensor(0x0303,0x01);	// PLL control
    }
    else if(5<=u2FrameRate && u2FrameRate<20 )// fix 15
    {
		//SP2529MIPI_write_cmos_sensor(0x0303,0x02);	// PLL control
    }
    else 
    {
        printk("Wrong Frame Rate \n"); 
    }
    return ERROR_NONE;
}


kal_uint16 SP2529MIPIReadShutter(void)
{
   #if 0
   kal_uint16 temp_msb=0x0000,temp_lsb=0x0000;
   
   SP2529MIPI_write_cmos_sensor(0xfd,0x00);

   temp_msb=SP2529MIPI_read_cmos_sensor(0x03);
   temp_msb = (temp_msb<<8)&0xff00;
   temp_lsb=SP2529MIPI_read_cmos_sensor(0x04);
   temp_lsb = temp_lsb&0x00ff;
   
   return (temp_msb|temp_lsb);
   #endif
}
kal_uint16 SP2529MIPIReadGain(void)
{
#if 0
	kal_uint16 temp_msb=0x0000,temp_lsb=0x0000;
	
	  SP2529MIPI_write_cmos_sensor(0xfd,0x00);
	
	temp_msb=SP2529MIPI_read_cmos_sensor(0x23);
	//temp_msb = (temp_msb<<8)&0xff00;
	//temp_lsb=SP2529MIPI_read_cmos_sensor(0x0205);
	//temp_lsb = temp_lsb&0x00ff;
	
	//return (temp_msb|temp_lsb);
	return  temp_msb;
#endif

}
kal_uint16 SP2529MIPIReadAwbRGain(void)
{
#if 0
	kal_uint16 temp_msb=0x0000,temp_lsb=0x0000;
	temp_msb = SP2529MIPI_read_cmos_sensor(0x3400);
	temp_msb = (temp_msb<<8)&0xff00;
	temp_lsb = SP2529MIPI_read_cmos_sensor(0x3401);
	temp_lsb = temp_lsb&0x00ff;

	return (temp_msb|temp_lsb);
#endif
}
kal_uint16 SP2529MIPIReadAwbBGain(void)
{
#if 0
	kal_uint16 temp_msb=0x0000,temp_lsb=0x0000;
	temp_msb = SP2529MIPI_read_cmos_sensor(0x3404);
	temp_msb = (temp_msb<<8)&0xff00;
	temp_lsb = SP2529MIPI_read_cmos_sensor(0x3405);
	temp_lsb = temp_lsb&0x00ff;
	return (temp_msb|temp_lsb);
#endif

}
#if 0

/*************************************************************************
* FUNCTION
*    SP2529MIPIGetEvAwbRef
*
* DESCRIPTION
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void SP2529MIPIGetEvAwbRef(UINT32 pSensorAEAWBRefStruct/*PSENSOR_AE_AWB_REF_STRUCT Ref*/)
{
    PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
    SENSORDB("SP2529MIPIGetEvAwbRef ref = 0x%x \n", Ref);
    	
	Ref->SensorAERef.AeRefLV05Shutter = 5422;
    Ref->SensorAERef.AeRefLV05Gain = 478; /* 128 base */
    Ref->SensorAERef.AeRefLV13Shutter = 80;
    Ref->SensorAERef.AeRefLV13Gain = 128; /*  128 base */
    Ref->SensorAwbGainRef.AwbRefD65Rgain = 186; /* 128 base */
    Ref->SensorAwbGainRef.AwbRefD65Bgain = 158; /* 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFRgain = 196; /* 1.25x, 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFBgain = 278; /* 1.28125x, 128 base */
}
/*************************************************************************
* FUNCTION
*    SP2529MIPIGetCurAeAwbInfo
*
* DESCRIPTION
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void SP2529MIPIGetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct/*PSENSOR_AE_AWB_CUR_STRUCT Info*/)
{
    PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
    SENSORDB("SP2529MIPIGetCurAeAwbInfo Info = 0x%x \n", Info);

    Info->SensorAECur.AeCurShutter = SP2529MIPIReadShutter();
    Info->SensorAECur.AeCurGain = SP2529MIPIReadGain() * 2; /* 128 base */
    
    Info->SensorAwbGainCur.AwbCurRgain = SP2529MIPIReadAwbRGain()<< 1; /* 128 base */
    
    Info->SensorAwbGainCur.AwbCurBgain = SP2529MIPIReadAwbBGain()<< 1; /* 128 base */
}

#endif

void SP2529MIPIGetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;    
    SENSORDB("SP2529MIPIGetAFMaxNumFocusAreas, *pFeatureReturnPara32 = %d\n",*pFeatureReturnPara32);

}

void SP2529MIPIGetAFMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;    
    SENSORDB("SP2529MIPIGetAFMaxNumMeteringAreas,*pFeatureReturnPara32 = %d\n",*pFeatureReturnPara32);

}

void SP2529MIPIGetAEAWBLock(UINT32 *pAElockRet32,UINT32 *pAWBlockRet32)
{
    *pAElockRet32 = 1;
	*pAWBlockRet32 = 1;
    SENSORDB("SP2529MIPIGetAEAWBLock,AE=%d ,AWB=%d\n,",*pAElockRet32,*pAWBlockRet32);
}


void SP2529MIPIGetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = SP2529MIPICurrentStatus.iWB;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}

UINT32 SP2529MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	SENSORDB("SP2529MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = SP2529MIPI_sensor_pclk/10;
			lineLength = SP2529MIPI_SXGA_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - SP2529MIPI_SXGA_PERIOD_LINE_NUMS;
			SP2529MIPI_set_dummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = SP2529MIPI_sensor_pclk/10;
			lineLength = SP2529MIPI_SXGA_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - SP2529MIPI_SXGA_PERIOD_LINE_NUMS;
			SP2529MIPI_set_dummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = SP2529MIPI_sensor_pclk/10;
			lineLength = SP2529MIPI_SXGA_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - SP2529MIPI_SXGA_PERIOD_LINE_NUMS;
			SP2529MIPI_set_dummy(0, dummyLine);			
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
	return ERROR_NONE;
}


UINT32 SP2529MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 300;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}

UINT32 SP2529MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	//PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;
#if WINMO_USE	
	PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo=(PMSDK_FEATURE_INFO_STRUCT) pFeaturePara;
#endif 


	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=SP2529MIPI_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=SP2529MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = SP2529MIPI_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			SP2529MIPI_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			SP2529MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = SP2529MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &SP2529MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
		case SENSOR_FEATURE_SET_YUV_CMD:
			//SP2529MIPIYUVSensorSetting((MSDK_ISP_FEATURE_ENUM)*pFeatureData16, *(pFeatureData16+1));
			SP2529MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		break;
#if WINMO_USE		
		case SENSOR_FEATURE_QUERY:
			SP2529MIPIQuery(pSensorFeatureInfo);
			*pFeatureParaLen = sizeof(MSDK_FEATURE_INFO_STRUCT);
		break;		
		case SENSOR_FEATURE_SET_YUV_CAPTURE_RAW_SUPPORT:
			/* update yuv capture raw support flag by *pFeatureData16 */
		break;
#endif 				
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		    SP2529MIPIYUVSetVideoMode(*pFeatureData16);
		    break; 
		#if 0
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			 SP2529MIPIGetEvAwbRef(*pFeatureData32);
				break;
  		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			   SP2529MIPIGetCurAeAwbInfo(*pFeatureData32);	
			break;
		#endif
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
            SP2529MIPI_GetSensorID(pFeatureData32); 
            break; 	

		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			SP2529MIPIGetAFMaxNumFocusAreas(pFeatureReturnPara32); 		   
			*pFeatureParaLen=4;
			break;	   
		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			SP2529MIPIGetAFMaxNumMeteringAreas(pFeatureReturnPara32);			  
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
			SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO\n");
			SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32); 		 
			SP2529MIPIGetExifInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			SP2529MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			SP2529MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			SP2529MIPIGetAEAWBLock((*pFeatureData32),*(pFeatureData32+1));
			break;			
		default:
			break;			
	}
	return ERROR_NONE;
}

SENSOR_FUNCTION_STRUCT	SensorFuncSP2529MIPI=
{
	SP2529MIPIOpen,
	SP2529MIPIGetInfo,
	SP2529MIPIGetResolution,
	SP2529MIPIFeatureControl,
	SP2529MIPIControl,
	SP2529MIPIClose
};

UINT32 SP2529_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncSP2529MIPI;
	return ERROR_NONE;
}	/* SensorInit() */

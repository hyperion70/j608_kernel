#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
#include <cust_adc.h>    	// zhoulidong  add for lcm detect
#define MIN_VOLTAGE (0)		//++++rgk bug-id:no modify by yangjuwei 20140401
#define MAX_VOLTAGE (100)	//++++rgk bug-id:no modify by yangjuwei 20140401

//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)
#define LCM_ID		0x94	//++++rgk bug-id:no modify by yangjuwei 20140401

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

unsigned static int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util ;

#define SET_RESET_PIN(v)    			(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 				(lcm_util.udelay(n))
#define MDELAY(n) 				(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)			lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)		lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)			lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)							lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)							lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE						0

// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static LCM_setting_table_V3 lcm_initialization_setting[] = {
	
	{0x39,0xB9,3,{0xFF,0x83,0x94}},

	{0x15,0xBA,1,{0x13}},
        {0x15,0x53,1,{0x24}},
	{0x39,0xB1,16,{0x01,0x00,0x07,0x86,0x01,0x11,0x11,0x2A,0x32,0x3F,0x3F,0x47,0x12,0x01,0xE6,0xE2}},//SET POWER
	
	{0x39,0xB4,22,{0x80,0x06,0x32,0x10,0x03,0x32,0x15,0x08,0x32,0x10,0x08,0x33,0x04,0x43,0x05,0x37,0x04,0x3F,0x06,0x61,0x61,0x06}},//SET CYC

	{0x39,0xB2,6,{0x00,0xC8,0x08,0x04,0x00,0x22}},

	{0x39,0xD5,32,{0x00,0x00,0x00,0x00,0x0A,0x00,0x01,0x00,0xCC,0x00,0x00,0x00,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x01,0x67,0x45,0x23,0x01,0x23,0x88,0x88,0x88,0x88}},

	
	{0x39,0xC7,4,{0x00,0x10,0x00,0x10}},
	{0x39,0xBF,4,{0x06,0x02,0x10,0x04}},

	{0x15,0xCC,1,{0x09}},
	{0x39,0xE0,42,{0x00,0x01,0x02,0x2B,0x33,0x3F,0x0C,0x32,0x05,0x0B,0x0D,0x10,0x12,0x10,0x12,0x10,0x17,0x00,0x01,0x02,0x2B,0x33,0x3F,0x0C,0x32,0x05,0x0B,0x0D,0x10,0x12,0x10,0x12,0x10,0x17,0x09,0x17,0x06,0x11,0x09,0x17,0x06,0x11}},
   /*{0x39,0xC1,127,{0x01,
		              0x00,0x09,0x11,0x18,0x20,0x28,0x31,0x39,0x41,0x47,0x4E,0x56,0x5E,0x65,0x6d,0x75,0x7d,0x84,0x8c,0x94,0x9c,0xa3,0xab,0xb4,0xbc,0xc4,0xcd,0xd5,0xdd,0xe5,0xed,0xf6,0xff,0x03,0xe6,0x3e,0x15,0x24,0x3d,0x12,0x69,0xc0,
		              0x00,0x08,0x0F,0x15,0x1C,0x24,0x2B,0x32,0x3A,0x40,0x46,0x4C,0x53,0x5A,0x60,0x67,0x6E,0x75,0x7C,0x83,0x8A,0x90,0x97,0x9E,0xA5,0xAC,0xB4,0xBB,0xC2,0xCA,0xD1,0xD9,0xDF,0x03,0xC7,0x3E,0x9E,0xA9,0x3F,0xB4,0x58,0xc0,
		              0x00,0x08,0x0F,0x15,0x1C,0x24,0x2B,0x32,0x3A,0x40,0x46,0x4C,0x53,0x5A,0x60,0x67,0x6E,0x75,0x7C,0x83,0x8A,0x90,0x97,0x9E,0xA5,0xAC,0xB4,0xBB,0xC2,0xCA,0xD1,0xD9,0xDF,0x03,0xC7,0x3E,0x9E,0xA9,0x3F,0xB4,0x58,0xc0}},*/

	//{0x15,0xB6,1,{0x09}},
	{0x15,0xD4,1,{0x32}},
	{0x15,0x35,1,{0x00}},
	
	{0x05,0x11,	0,	{0x00}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,200,{}},
	{0x05,0x29,	0,	{0x00}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,50,{}},

};

static void lcm_register()
{
	unsigned int data_array[35];
	data_array[0] = 0x00043902;
	data_array[1] = 0x9483ffb9;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00113902;
	data_array[1] = 0x008213ba;
	data_array[2] = 0x1000c516;
	data_array[3] = 0x03240fff;
	data_array[4] = 0x20252421;
	data_array[5] = 0x00000008;
	dsi_set_cmdq(data_array, 6, 1);
	data_array[0] = 0x00113902;
	data_array[1] = 0x040001b1;
	data_array[2] = 0x1111018a;
	data_array[3] = 0x3f3f372f;
	data_array[4] = 0xe6011247;
	data_array[5] = 0x000000e2;
	dsi_set_cmdq(data_array, 6, 1);
	data_array[0] = 0x00073902;
	data_array[1] = 0x08c800b2;
	data_array[2] = 0x00220004;
	dsi_set_cmdq(data_array, 3, 1);
	data_array[0] = 0x00173902;
	data_array[1] = 0x320680b4;
	data_array[2] = 0x15320310;
	data_array[3] = 0x08103208;
	data_array[4] = 0x05430433;
	data_array[5] = 0x06430437;
	data_array[6] = 0x00066161;
	dsi_set_cmdq(data_array, 7, 1);
	data_array[0] = 0x00053902;
	data_array[1] = 0x100006bf;
	data_array[2] = 0x00000004;
	dsi_set_cmdq(data_array, 3, 1);
	data_array[0] = 0x00033902;
	data_array[1] = 0x00170cc0;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00023902;
	data_array[1] = 0x000000b6;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00213902;
	data_array[1] = 0x000000d5;
	data_array[2] = 0x01000a00;
	data_array[3] = 0x0000cc00;
	data_array[4] = 0x88888800;
	data_array[5] = 0x88888888;
	data_array[6] = 0x01888888;
	data_array[7] = 0x01234567;
	data_array[8] = 0x88888823;
	data_array[9] = 0x00000088;
	dsi_set_cmdq(data_array, 10, 1);
	data_array[0] = 0x00023902;
	data_array[1] = 0x000009cc;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00053902;
	data_array[1] = 0x001000c7;
	data_array[2] = 0x00000010;
	dsi_set_cmdq(data_array, 3, 1);
	data_array[0] = 0x002b3902;
	data_array[1] = 0x060400e0;
	data_array[2] = 0x173f332b;
	data_array[3] = 0x0d0e0a34;
	data_array[4] = 0x13111311;
	data_array[5] = 0x04001710;
	data_array[6] = 0x3f332b06;
	data_array[7] = 0x0e0a3417;
	data_array[8] = 0x1113110d;
	data_array[9] = 0x0b171013;
	data_array[10] = 0x0b110717;
	data_array[11] = 0x00110717;
	dsi_set_cmdq(data_array, 12, 1);
	data_array[0] = 0x00023902;
	data_array[1] = 0x000032d4;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(250);
	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

	memset(params, 0, sizeof(LCM_PARAMS));
	
	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
#endif
	
	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 8;
	params->dsi.vertical_frontporch					= 6;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 40;
	params->dsi.horizontal_backporch				= 86;
	params->dsi.horizontal_frontporch				= 86;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	
	//params->dsi.LPX=8; 

	// Bit rate calculation
	params->dsi.PLL_CLOCK = 240;
	//1 Every lane speed
	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
	params->dsi.fbk_div =9;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	


}


static void lcm_init(void)
{
	 
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	//lcm_register();
	dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);

	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static  LCM_setting_table_V3 lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x05, 0x28, 0, {}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 10, {}},

	// Sleep Mode On
	{0x05, 0x10, 0, {}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 120, {}},
};
static void lcm_suspend(void)
{	
		
	dsi_set_cmdq_V3(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting)/sizeof(lcm_deep_sleep_mode_in_setting[0]), 1);
	
	SET_RESET_PIN(1);     
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(100);
}


static void lcm_resume(void)
{
	lcm_init();
	
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif


static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);

	array[0] = 0x00043902;
	array[1] = 0x9483FFB9;
	dsi_set_cmdq(array, 2, 1);	

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; //we only need ID
	
	#ifdef BUILD_LK
		printf("hx8394 uboot %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("hx8394 kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	#endif
	   
	return (id == LCM_ID) ? 1 : 0;
}

// zhoulidong  add for lcm detect (start)
static int rgk_lcm_compare_id(void)
{
	int data[4] = {0,0,0,0};
	int res = 0;
	int rawdata = 0;
	int lcm_vol = 0;

#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
	res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
	if(res < 0)
	{ 
	#ifdef BUILD_LK
		printf("[adc_uboot]: get data error\n");
	#endif
		return 0;
	}
#endif

	lcm_vol = data[0]*1000+data[1]*10;

#ifdef BUILD_LK
	printf("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
#endif
	
	if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE && lcm_compare_id())
	{
		return 1;
	}

	return 0;
}

// zhoulidong  add for lcm detect (end)


// zhoulidong add for eds(start)
static unsigned int lcm_esd_check(void)
{
#ifdef BUILD_LK
	//printf("lcm_esd_check()\n");
#else
	//printk("lcm_esd_check()\n");
#endif 
#ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);
	if(buffer[0]==0x1c)
	{
		//#ifdef BUILD_LK
		//printf("%s %d\n FALSE", __func__, __LINE__);
		//#else
		//printk("%s %d\n FALSE", __func__, __LINE__);
		//#endif
		return FALSE;
	}
	else
	{	
		//#ifdef BUILD_LK
		//printf("%s %d\n FALSE", __func__, __LINE__);
		//#else
		//printk("%s %d\n FALSE", __func__, __LINE__);
		//#endif		 
		return TRUE;
	}
#endif

}

static unsigned int lcm_esd_recover(void)
{
	
	#ifdef BUILD_LK
		printf("lcm_esd_recover()\n");
	#else
		printk("lcm_esd_recover()\n");
	#endif	
	
	lcm_init();	

	return TRUE;
}
// zhoulidong add for eds(end)

LCM_DRIVER hx8394_dsi_vdo_truly_hd720_ips_lcm_drv = 
{
	.name			= "hx8394_dsi_vdo_truly_hd720_ips",
	.set_util_funcs		= lcm_set_util_funcs,
	.get_params		= lcm_get_params,
	.init			= lcm_init,
	.suspend		= lcm_suspend,
	.resume			= lcm_resume,
	.compare_id		= rgk_lcm_compare_id,
	.esd_check		= lcm_esd_check,
	.esd_recover		= lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
	//.update		= lcm_update,
#endif
};


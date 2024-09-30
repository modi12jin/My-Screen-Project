/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifdef BUILD_LK
#include <string.h>
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#include <platform/sync_write.h>
#include <platform/ddp_hal.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#endif
#endif
#include "lcm_drv.h"

#ifndef BUILD_LK 
extern int inettek_gpio_set(const char *name);
extern int lcm_vgp_supply_enable(void);
extern int lcm_vgp_supply_disable(void);
#endif

#ifdef BUILD_LK
#define LCD_2V8_EN                 (38 | 0x80000000)
#define LCD_RST_PIN                (66 | 0x80000000)
#define LCD_GAMA_EN1                (20 | 0x80000000)
#endif

/* --------------------------------------------------------------- */
/*  Local Constants */
/* --------------------------------------------------------------- */
#define FRAME_WIDTH		(720)
#define FRAME_HEIGHT	(1280)
#define GPIO_OUT_ONE		1
#define GPIO_OUT_ZERO		0

#define REGFLAG_DELAY            0xFE
#define REGFLAG_END_OF_TABLE     0x00

/* --------------------------------------------------------------- */
/*  Local Variables */
/* --------------------------------------------------------------- */





extern void mdelay(unsigned long msec);

#ifdef BUILD_LK
static LCM_UTIL_FUNCS lcm_util = {
    .set_reset_pin = NULL,
    .udelay = NULL,
    .mdelay = NULL,
};
#else
static LCM_UTIL_FUNCS lcm_util = { 0 };

#endif

#define SET_RESET_PIN(v)        (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                (lcm_util.udelay(n))
#define MDELAY(n)                (lcm_util.mdelay(n))

/* --------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		 (lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update))
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		 (lcm_util.dsi_set_cmdq(pdata, queue_size, force_update))
#define wrtie_cmd(cmd) \
		 (lcm_util.dsi_write_cmd(cmd))
#define write_regs(addr, pdata, byte_nums) \
		 (lcm_util.dsi_write_regs(addr, pdata, byte_nums))
#define read_reg \
		 (lcm_util.dsi_read_reg())
#define read_reg_v2(cmd, buffer, buffer_size) \
		 (lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size))

#ifdef BUILD_LK
static void pmic_vgp2_enable(bool en, int vol_mv)
{
	unsigned int val;
	if (!en)
		mt6392_upmu_set_rg_vgp2_en(0x0);
	else {
		switch (vol_mv) {
			case 1200:
				val = 0x00;
				break;
			case 1300:
				val = 0x01;
				break;
			case 1500:
				val = 0x02;
				break;
			case 1800:
				val = 0x03;
				break;
			case 2000:
				val = 0x04;
				break;
			case 2800:
				val = 0x05;
				break;
			case 3000:
				val = 0x06;
				break;
			case 3300:
				val = 0x07;
				break;
			default:
				val = 0x05;
				break;
		}
		//VGP2 0x00=1.2V, 0x01=1.3V, 0x02=1.5V, 0x03=1.8V, 0x04=2.0V, 0x05=2.8V, 0x06=3.0V, 0x07=3.3V
		
		mt6392_upmu_set_rg_vgp2_vosel(val);
		mt6392_upmu_set_rg_vgp2_en(0x1);
	}
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
    mt_set_gpio_mode(GPIO, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO, output);
}
#endif

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[128];
};



//update initial param for IC boe_nt71391_dsi_vdo 0.01
static struct LCM_setting_table lcm_initialization_setting[] = 
{
	{0x11,	0,	{}},
	{REGFLAG_DELAY, 250, {}},	
	{0xB9,0x3,{0xF1,0x12,0x83}},
	{0xBA,0x1B,{0x33,0x81,0x05,0xF9,0x0E,0x0E,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x25,0x00,0x91,0x0A,0x00,0x00,0x02,0x4F,0xC1,0x00,0x00,0x37}},
	{0xB8,0x4,{0x75,0x22,0x20,0x03}},
	{0xBF,0x3,{0x02,0x10,0x00}},
	{0xB3,0xA,{0x07,0x0B,0x1E,0x1E,0x03,0xFF,0x00,0x00,0x00,0x00}},
	{0xC0,0x9,{0x73,0x73,0x50,0x50,0x00,0x00,0x08,0xA0,0x00}},
	{0xBC,0x1,{0x46}},
	{0xCC,0x1,{0x07}},
	{0xB4,0x1,{0x80}},
	{0xB2,0x3,{0xC8,0x02,0xF0}},
	{0xC6,0x6,{0x01,0x00,0xFF,0xFF,0x00,0x20}},
	{0xE3,0xE,{0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,0xFF,0x80,0xC0,0x14}},
	{0xC1,0xC,{0x54,0x00,0x32,0x32,0x77,0xF1,0xFF,0xFF,0xCC,0xCC,0x77,0x77}},
	{0xB5,0x2,{0x10,0x10}},
	{0xB6,0x2,{0x8E,0x8E}},
	{0xE9,0x3F,{0xC8,0x10,0x09,0x00,0x00,0x08,0xE9,0x12,0x30,0x00,0x27,0x85,0x08,0xE9,0x27,0x18,0x00,0x81,0x00,0x00,0x00,0x00,0x00,0x81,0x00,0x00,0x00,0x00,0xF8,0xBA,0x46,0x02,0x08,0x88,0x88,0x82,0x88,0x88,0x88,0xF8,0xBA,0x57,0x13,0x18,0x88,0x88,0x83,0x88,0x88,0x88,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
	{0xEA,0x3D,{0x07,0x12,0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x8F,0xBA,0x31,0x75,0x38,0x88,0x88,0x81,0x88,0x88,0x88,0x8F,0xBA,0x20,0x64,0x28,0x88,0x88,0x80,0x88,0x88,0x88,0x23,0x30,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xE0,0x22,{0x00,0x06,0x0B,0x24,0x2B,0x3F,0x39,0x33,0x05,0x0A,0x0C,0x10,0x11,0x0F,0x12,0x16,0x1C,0x00,0x06,0x0B,0x24,0x2B,0x3F,0x39,0x33,0x05,0x0A,0x0C,0x10,0x11,0x0F,0x12,0x16,0x1C}},  
	{0x11,	0,	{}},	//Sleep Out
	{REGFLAG_DELAY, 250, {}},
	{0x29,	0,	{}},	//Display ON
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};

#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = 
{
	{0x32,	0,	{0x00}},
  	{REGFLAG_DELAY, 10, {}},
    //Sleep Out
    {0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    // Display ON
    {0x29, 0, {0x00}},
    {REGFLAG_DELAY, 150, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = 
{
    // Display off sequence
    {0x28, 0, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    // Sleep Mode On
    {0x10, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
    
};
#endif

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
    for(i = 0; i < count; i++) {
        unsigned cmd;
        cmd = table[i].cmd;
        switch (cmd) {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
            case REGFLAG_END_OF_TABLE :
                break;
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				//MDELAY(10);//soso add or it will fail to send register
       	}
    }
}

/* --------------------------------------------------------------- */
/*  LCM Driver Implementations */
/* --------------------------------------------------------------- */
#ifdef BUILD_LK
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}
#else
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}
#endif




static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* Video mode setting */
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=FRAME_WIDTH*3;

	params->dsi.vertical_sync_active		= 3; 
	params->dsi.vertical_backporch			= 11;
	params->dsi.vertical_frontporch			= 16;
	params->dsi.vertical_active_line		= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active		= 4;
	params->dsi.horizontal_backporch		= 40;
	params->dsi.horizontal_frontporch		= 40;
	params->dsi.horizontal_active_pixel		= FRAME_WIDTH;

//	params->dsi.ssc_disable	= 1;	//0表示开展频，1表示关展频
//	params->dsi.ssc_range 	= 6;	//6为展频幅度6%，此值可以修改
    params->dsi.PLL_CLOCK 	= 208;	
//	params->dsi.cont_clock	= 1;
	
	params->dsi.clk_lp_per_line_enable = 1;
}
static unsigned int lcm_compare_id(void);
static void lcm_init(void)
{
	#ifdef BUILD_LK
	printf("[LK/LCM]xxxxxxx %s enter\n", __func__);
	#else
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	#endif
	
	#ifdef BUILD_LK
	lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ONE);
	MDELAY(10);
	lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ZERO);	
	MDELAY(10);
	lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ONE);
	MDELAY(150);
	
//	MDELAY(50);	
//	lcm_compare_id();

	#else
	inettek_gpio_set("lcm_rst_en1");
	MDELAY(10);
	inettek_gpio_set("lcm_rst_en0");
	MDELAY(5);
	inettek_gpio_set("lcm_rst_en1");
	MDELAY(5);
	#endif
	
	
	//init_karnak_fiti_kd_lcm(); /*FITI KD panel*/
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1); 
}

static void lcm_resume(void)
{
	#ifdef BUILD_LK
	printf("[LK/LCM]xxxxxxx %s enter\n", __func__);
	#else
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	#endif
	//init_karnak_fiti_kd_lcm(); /* TPV panel */
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1); 
}

static void lcm_init_power(void)
{
	#ifdef BUILD_LK
	printf("[LK/LCM]xxxxxxx %s enter\n", __func__);
	#else
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	#endif
	
	MDELAY(1);
	#ifdef BUILD_LK
	pmic_vgp2_enable(false, 1800);	//VCC_LCD8_1V8
	lcm_set_gpio_output(LCD_2V8_EN, GPIO_OUT_ONE);	//VCC_LCD_3V3
    pmic_vgp2_enable(true, 1800);	//VCC_LCD8_1V8
	MDELAY(10);
	lcm_set_gpio_output(LCD_GAMA_EN1, GPIO_OUT_ONE);
	
	#else
	inettek_gpio_set("lcm_vcc_en1");
	inettek_gpio_set("lcm_avdd_en1");
	lcm_vgp_supply_enable();
	#endif


}

static void lcm_resume_power(void)
{
	#ifdef BUILD_LK
	printf("[LK/LCM]xxxxxxx %s enter\n", __func__);
	#else
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	#endif

	MDELAY(1);
	#ifdef BUILD_LK
	lcm_set_gpio_output(LCD_2V8_EN, GPIO_OUT_ONE);	//VCC_LCD_3V3
	pmic_vgp2_enable(true, 1800);	//VCC_LCD8_1V8
	lcm_set_gpio_output(LCD_GAMA_EN1, GPIO_OUT_ONE);
	
	#else
	inettek_gpio_set("lcm_vcc_en1");
	inettek_gpio_set("lcm_avdd_en1");
	lcm_vgp_supply_enable();
	#endif

	#ifdef BUILD_LK
	lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ONE);
	MDELAY(10);
	lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ZERO);
	MDELAY(5);
	lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ONE);
	MDELAY(5);
	#else
	inettek_gpio_set("lcm_rst_en1");
	MDELAY(10);
	inettek_gpio_set("lcm_rst_en0");
	MDELAY(5);
	inettek_gpio_set("lcm_rst_en1");
	MDELAY(5);
	#endif
}

static void lcm_suspend(void)
{
	#ifndef BUILD_LK
	unsigned int data_array[16];
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	data_array[0] = 0x00280500; /* Display Off */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);
	data_array[0] = 0x00100500; /* Sleep In */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	#endif
}

static void lcm_suspend_power(void)
{
	#ifdef BUILD_LK
	lcm_set_gpio_output(LCD_2V8_EN, GPIO_OUT_ZERO);	//VCC_LCD_3V3
    pmic_vgp2_enable(false,1800);
	MDELAY(10);
	lcm_set_gpio_output(LCD_GAMA_EN1, GPIO_OUT_ZERO);
    lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ZERO);
	#else
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	inettek_gpio_set("lcm_rst_en0");
	MDELAY(10);
	inettek_gpio_set("lcm_avdd_en0");
	lcm_vgp_supply_disable();
	inettek_gpio_set("lcm_vcc_en0");
	MDELAY(10);
	#endif
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[3];

	unsigned int data_array[16];
	
	lcm_init_power();
	
	/* NOTE:should reset LCM firstly */

	// lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ONE);
	// MDELAY(30);
	// lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ZERO);
	// MDELAY(20);
	// lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ONE);
	// MDELAY(30);
	
	data_array[0] = 0x00043902;
	data_array[1] = 0x8312F1B9;   
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00013700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0xD0, &buffer[0], 1);

	id = buffer[0]; //we only need ID
#ifdef BUILD_LK
	printf("Read ILI9881C ID, ID=0x%X\n",id);
#else
	printk("Read ILI9881C ID, ID=0x%X\n",id);
#endif 
	return 1;
//	return (LCM_ID_ILI9881C == id) ? 1 : 0;
}


LCM_DRIVER st7703_hsd055bhw3_c00_t20_dsi_vdo_lcm_drv = {
	.name			= "st7703_hsd055bhw3_c00_t20_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.init_power	= lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power	= lcm_resume_power,
	.suspend_power	= lcm_suspend_power,
	.compare_id = lcm_compare_id,
};

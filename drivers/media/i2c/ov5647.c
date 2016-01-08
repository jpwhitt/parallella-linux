/*
 * ov5647.c
 *
 * Copyright (C) 2015 Jason Whittaker <jpwhitt@yahoo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h> 


#define DRIVER_NAME	"ov5647"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");


/* Data ------------------------------------------------------------------- */

struct ov5647 {
	struct v4l2_subdev sd;		/* Must be first ! */
	struct media_pad pad;
	struct i2c_client *client;
	struct gpio_desc *gpio_ena;
	struct v4l2_mbus_framefmt format;
	u32 xclk_freq;
};


#define REG_DLY  0xffff


struct regval_list {
	unsigned short addr;
	unsigned char data;
};

static struct regval_list sensor_common[] = {
	{ 0x0100, 0x00 },// ; software standby
	{ 0x0103, 0x01 },// ; software reset
	//delay(5ms) 
	{ REG_DLY,0x05 }, //must delay
	{ 0x3034, 0x1A },
//	{ 0x3035, 0x21 },
//	{ 0x3036, 0x46 },
//	{ 0x303c, 0x11 },
	{ 0x3106, 0xf5 },
//	{ 0x3821, 0x07 },
//	{ 0x3820, 0x41 },
	{ 0x3827, 0xec },
	{ 0x370c, 0x0f },
//	{ 0x3612, 0x59 },
//	{ 0x3618, 0x00 },
	{ 0x5000, 0x06 },
	{ 0x5001, 0x01 },
	{ 0x5002, 0x40 },
	{ 0x5003, 0x08 },
	{ 0x5a00, 0x08 },
	{ 0x3000, 0x00 },
	{ 0x3001, 0x00 },
	{ 0x3002, 0x00 },
	{ 0x3016, 0x08 },
	{ 0x3017, 0x10 },
	{ 0x3018, 0x44 },
	{ 0x301c, 0xf8 },
	{ 0x301d, 0xf0 },
	{ 0x3a18, 0x00 },
	{ 0x3a19, 0xf8 },
	{ 0x3c01, 0x80 },
	{ 0x3b07, 0x0c },
//	{ 0x380c, 0x07 },
//	{ 0x380d, 0x68 },
//	{ 0x380e, 0x03 },
//	{ 0x380f, 0xd8 },
//	{ 0x3814, 0x31 },
//	{ 0x3815, 0x31 },
//	{ 0x3708, 0x64 },
//	{ 0x3709, 0x52 },
	{ 0x3630, 0x2e },
	{ 0x3632, 0xe2 },
	{ 0x3633, 0x23 },
	{ 0x3634, 0x44 },
	{ 0x3636, 0x06 },
	{ 0x3620, 0x64 },
	{ 0x3621, 0xe0 },
	{ 0x3600, 0x37 },
	{ 0x3704, 0xa0 },
	{ 0x3703, 0x5a },
	{ 0x3715, 0x78 },
	{ 0x3717, 0x01 },
	{ 0x3731, 0x02 },
	{ 0x370b, 0x60 },
	{ 0x3705, 0x1a },
	{ 0x3f05, 0x02 },
	{ 0x3f06, 0x10 },
	{ 0x3f01, 0x0a },
//	{ 0x3a08, 0x01 },
//	{ 0x3a09, 0x27 },
//	{ 0x3a0a, 0x00 },
//	{ 0x3a0b, 0xf6 },
//	{ 0x3a0d, 0x04 },
//	{ 0x3a0e, 0x03 },
	{ 0x3a0f, 0x58 },
	{ 0x3a10, 0x50 },
	{ 0x3a1b, 0x58 },
	{ 0x3a1e, 0x50 },
	{ 0x3a11, 0x60 },
	{ 0x3a1f, 0x28 },
	{ 0x4001, 0x02 },
//	{ 0x4004, 0x02 },
	{ 0x4000, 0x09 },
//	{ 0x4837, 0x24 },
	{ 0x4050, 0x6e },
	{ 0x4051, 0x8f },
	{ 0x3022, 0x07 },
};


/* 2592 x 1944 @ 15 fps */
	/*
	 * MIPI Link   : 425.000 Mbps
	 * Pixel clock : 85.000 MHz
	 * Timing zone : 2752 x 1974
	 * FPS         : 15.6
	*/
static struct regval_list sensor_2592_1944_15[] = {
	{ 0x3035, 0x21 },
	{ 0x3036, 0x66 },
	{ 0x303c, 0x11 },
	{ 0x3821, 0x06 },
	{ 0x3820, 0x00 },
	{ 0x3612, 0x5b },
	{ 0x3618, 0x04 },
	{ 0x380c, 0x0a },
	{ 0x380d, 0xc0 },
	{ 0x380e, 0x07 },
	{ 0x380f, 0xb6 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3708, 0x64 },
	{ 0x3709, 0x12 },
	{ 0x3808, 0x0a },
	{ 0x3809, 0x20 },
	{ 0x380a, 0x07 },
	{ 0x380b, 0x98 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x0c },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x04 },
	{ 0x3804, 0x0a },
	{ 0x3805, 0x33 },
	{ 0x3806, 0x07 },
	{ 0x3807, 0xa3 },
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0x28 },
	{ 0x3a0a, 0x00 },
	{ 0x3a0b, 0xf6 },
	{ 0x3a0d, 0x07 },
	{ 0x3a0e, 0x06 },
	{ 0x4004, 0x04 },
	{ 0x4837, 0x19 },
};

/* 1936 x 1088 @ 30 fps */
	/*
	 * MIPI Link   : 416.667 Mbps
	 * Pixel clock : 83.333 MHz
	 * Timing zone : 2416 x 1104
	 * FPS         : 31.2
	 */
static struct regval_list sensor_1936_1088_30[] = {
	{ 0x3035, 0x21 },
	{ 0x3036, 0x64 },
	{ 0x303c, 0x11 },
	{ 0x3821, 0x06 },
	{ 0x3820, 0x00 },
	{ 0x3612, 0x5b },
	{ 0x3618, 0x04 },
	{ 0x380c, 0x09 },
	{ 0x380d, 0x70 },
	{ 0x380e, 0x04 },
	{ 0x380f, 0x50 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3708, 0x64 },
	{ 0x3709, 0x12 },
	{ 0x3808, 0x07 },
	{ 0x3809, 0x90 },	/* 80 */
	{ 0x380a, 0x04 },
	{ 0x380b, 0x40 },	/* 38 */
	{ 0x3800, 0x01 },
	{ 0x3801, 0x54 },	/* 5c */
	{ 0x3802, 0x01 },
	{ 0x3803, 0xb0 },	/* b2 */
	{ 0x3804, 0x08 },
	{ 0x3805, 0xeb },	/* e3 */
	{ 0x3806, 0x05 },
	{ 0x3807, 0xf3 },	/* f1 */
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0x4b },
	{ 0x3a0a, 0x01 },
	{ 0x3a0b, 0x13 },
	{ 0x3a0d, 0x04 },
	{ 0x3a0e, 0x03 },
	{ 0x4004, 0x04 },
	{ 0x4837, 0x19 },
};

/* 1296 x 968 @ 30 fps */
	/*
	 * MIPI Link   : 291.667 Mbps
	 * Pixel clock : 58.333 MHz
	 * Timing zone : 1896 x 984
	 * FPS         : 31.3
	 */
static struct regval_list sensor_1296_968_30[] = {
	{ 0x3035, 0x21 },
	{ 0x3036, 0x46 },
	{ 0x303c, 0x11 },
	{ 0x3821, 0x07 },
	{ 0x3820, 0x41 },
	{ 0x3612, 0x59 },
	{ 0x3618, 0x00 },
	{ 0x380c, 0x07 },
	{ 0x380d, 0x68 },
	{ 0x380e, 0x03 },
	{ 0x380f, 0xd8 },
	{ 0x3814, 0x31 },
	{ 0x3815, 0x31 },
	{ 0x3708, 0x64 },
	{ 0x3709, 0x52 },
	{ 0x3808, 0x05 },
	{ 0x3809, 0x10 },	/* 00 */
	{ 0x380a, 0x03 },
	{ 0x380b, 0xc8 },	/* c0 */
	{ 0x3800, 0x00 },
	{ 0x3801, 0x10 },	/* 18 */
	{ 0x3802, 0x00 },
	{ 0x3803, 0x08 },	/* 0e */
	{ 0x3804, 0x0a },
	{ 0x3805, 0x4f },	/* 27 */
	{ 0x3806, 0x07 },
	{ 0x3807, 0x9b },	/* 95 */
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0x27 },
	{ 0x3a0a, 0x00 },
	{ 0x3a0b, 0xf6 },
	{ 0x3a0d, 0x04 },
	{ 0x3a0e, 0x03 },
	{ 0x4004, 0x02 },
	{ 0x4837, 0x24 },
};

/* 1296 x 728 @ 30 fps */
	/*
	 * MIPI Link   : TBD Mbps
	 * Pixel clock : TBD MHz
	 * Timing zone : 1896 x 984
	 * FPS         : 31.3
	 */
static struct regval_list sensor_1296_728_30_regs[] = { //720: 1280*720@30fps
	{ 0x3035, 0x21 }, //clk                
	{ 0x3036, 0x32 }, //clk                
	{ 0x303c, 0x11 }, //clk                
	{ 0x3820, 0x41 }, //vbin               
	{ 0x3821, 0x07 }, //hbin               
	{ 0x3612, 0x49 }, //                   
	{ 0x3618, 0x00 }, //                   
	{ 0x3708, 0x22 }, //                   
	{ 0x3709, 0x52 }, //                   
	{ 0x370c, 0x03 }, //                   
	{ 0x380c, 0x06 }, //[4:0]hts high      
	{ 0x380d, 0xd6 }, //[7:0]hts low       
	{ 0x380e, 0x03 }, //[4:0]vts high      
	{ 0x380f, 0x20 }, //[7:0]vts low       
	{ 0x3814, 0x31 }, //h subsample inc    
	{ 0x3815, 0x31 }, //v subsample inc    
	{ 0x3808, 0x05 }, //[4:0]dvp h out high
	{ 0x3809, 0x10 }, //[7:0]dvp h out low 
	{ 0x380a, 0x02 }, //[4:0]dvp v out high
	{ 0x380b, 0xd8 }, //[7:0]dvp v out low 
	{ 0x3800, 0x00 }, //[4:0]dvp h start   
	{ 0x3801, 0x10 }, //[7:0]dvp h start   
	{ 0x3802, 0x00 }, //[4:0]dvp v start   
	{ 0x3803, 0xe8 }, //[7:0]dvp v start   
	{ 0x3804, 0x0a }, //[4:0]dvp h end     
	{ 0x3805, 0x4f }, //[7:0]dvp h end     
	{ 0x3806, 0x06 }, //[4:0]dvp v end     
	{ 0x3807, 0xb7 }, //[7:0]dvp v end     
	{ 0x3a08, 0x00 }, //
	{ 0x3a09, 0xdf }, //
	{ 0x3a0a, 0x00 }, //
	{ 0x3a0b, 0xba }, //
	{ 0x3a0d, 0x04 }, //
	{ 0x3a0e, 0x03 }, //
};



static struct regval_list sensor_644_484_60_regs[] = { //VGA: 640*480@60fps
	{ 0x3035, 0x21 }, //clk                
	{ 0x3036, 0x64 }, //clk                
	{ 0x303c, 0x11 }, //clk                
	{ 0x3820, 0x41 }, //vbin               
	{ 0x3821, 0x07 }, //hbin               
	{ 0x3612, 0x49 }, //                   
	{ 0x3618, 0x00 }, //                   
	{ 0x3708, 0x22 }, //                   
	{ 0x3709, 0x52 }, //                   
	{ 0x370c, 0x03 }, //                    
	{ 0x380c, 0x06 }, //[4:0]hts high      
	{ 0x380d, 0xd6 }, //[7:0]hts low       
	{ 0x380e, 0x03 }, //[4:0]vts high      
	{ 0x380f, 0x20 }, //[7:0]vts low       
	{ 0x3814, 0x71 }, //h subsample inc    
	{ 0x3815, 0x71 }, //v subsample inc    
	{ 0x3808, 0x02 }, //[4:0]dvp h out high
	{ 0x3809, 0x84 }, //[7:0]dvp h out low 
	{ 0x380a, 0x01 }, //[4:0]dvp v out high
	{ 0x380b, 0xe4 }, //[7:0]dvp v out low 
	{ 0x3800, 0x00 }, //[4:0]dvp h start   
	{ 0x3801, 0x10 }, //[7:0]dvp h start   
	{ 0x3802, 0x00 }, //[4:0]dvp v start   
	{ 0x3803, 0x00 }, //[7:0]dvp v start   
	{ 0x3804, 0x0a }, //[4:0]dvp h end     
	{ 0x3805, 0x3f }, //[7:0]dvp h end     
	{ 0x3806, 0x07 }, //[4:0]dvp v end     
	{ 0x3807, 0xaf }, //[7:0]dvp v end     
	{ 0x3a08, 0x01 },
	{ 0x3a09, 0x28 },
	{ 0x3a0a, 0x00 },
	{ 0x3a0b, 0xf6 },
	{ 0x3a0d, 0x07 },
	{ 0x3a0e, 0x06 },
	{ 0x4004, 0x04 },
	{ 0x4837, 0x19 },
	{ 0x5003, 0x0e }, //
};


struct ov5647_framesize {
	u16 width;
	u16 height;
	u8  fps;
		struct regval_list *regs;
		int regs_size;
};

static const struct ov5647_framesize ov5647_framesizes[] = {
	{
		.width  = 644,
		.height = 484,
		.fps    = 60,
		.regs   = sensor_644_484_60_regs,
		.regs_size = ARRAY_SIZE(sensor_644_484_60_regs)
	}, {
		.width  = 1296,
		.height = 728,
		.fps    = 30,
		.regs   = sensor_1296_728_30_regs,
		.regs_size  = ARRAY_SIZE(sensor_1296_728_30_regs)
	}, {
		.width  = 1296,
		.height = 968,
		.fps    = 30,
		.regs   = sensor_1296_968_30,
		.regs_size = ARRAY_SIZE(sensor_1296_968_30)
	}, {
		.width  = 1936,
		.height	= 1088,
		.fps    = 30,
		.regs   = sensor_1936_1088_30,
		.regs_size = ARRAY_SIZE(sensor_1936_1088_30)
	}, {
		.width  = 2592,
		.height	= 1944,
		.fps    = 15,
		.regs   = sensor_2592_1944_15,
		.regs_size = ARRAY_SIZE(sensor_2592_1944_15)
	}
};


/* Helpers ---------------------------------------------------------------- */

static uint8_t
ov5647_reg_read(struct ov5647 *s, uint16_t reg)
{
	int rv;
	u8 buf[3];
	struct i2c_msg msgs[2] = {
		{
			.addr  = s->client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = &buf[0],
		},
		{
			.addr  = s->client->addr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = &buf[2],
		},
	};

	buf[0] = (reg >> 8) & 0xff;
	buf[1] =  reg       & 0xff;

	rv = i2c_transfer(s->client->adapter, msgs, 2);

	v4l2_dbg(2, debug, s->client, "%s: 0x%02hhx @ 0x%04hx. (%d)\n",
		__func__, buf[2], reg, rv);

	return (rv == 2) ? buf[2] : 0x00;
}

static int
ov5647_reg_write(struct ov5647 *s, uint16_t reg, uint8_t val)
{
	int rv;
	u8 buf[3];
	struct i2c_msg msg = {
		.addr  = s->client->addr,
		.flags = 0,
		.len   = 3,
		.buf   = &buf[0],
	};

	buf[0] = (reg >> 8) & 0xff;
	buf[1] =  reg       & 0xff;
	buf[2] =  val;

	rv = i2c_transfer(s->client->adapter, &msg, 1);

	v4l2_dbg(2, debug, s->client, "%s: 0x%02hhx @ 0x%04hx. (%d)\n",
		__func__, val, reg, rv);

	return rv != 1;
}

static int ov5647_reg_write_array(struct ov5647 *s, struct regval_list *regs, int array_size)
{
	int i=0;
	
	if(!regs)
		return -EINVAL;
	
	while(i<array_size)
	{
	if(regs->addr == REG_DLY) {
		v4l2_dbg(2, debug, s->client, "%s: delay %dms\n",
		__func__, regs->data);
			msleep(regs->data);
		} 
		else {  
				if(ov5647_reg_write(s, regs->addr, regs->data)) {
				v4l_err(s->client, "i2c write failed.\n");
				}
		}
		i++;
		regs++;
	}
	return 0;
}

static void
ov5647_power_on(struct ov5647 *s)
{
	if (s->gpio_ena) {
		gpiod_set_value_cansleep(s->gpio_ena, 1);
		msleep(30); /* 5ms ramp-up, 5ms pwdn, 20ms boot */
		v4l2_dbg(2, debug, s->client, "%s\n", __func__);
	}
}

static void
ov5647_power_off(struct ov5647 *s)
{
	if (s->gpio_ena)
		gpiod_set_value_cansleep(s->gpio_ena, 0);
		v4l2_dbg(2, debug, s->client, "%s\n", __func__);
}

static int 
ov5647_get_sysclk(struct ov5647 *s)
{
		 /* calculate sysclk */
		int xvclk = s->xclk_freq / 10000;
		int temp1, temp2;
		int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv, Bit_div2x = 1, sclk_rdiv, sysclk;

		int sclk_rdiv_map[] = {1, 2, 4, 8};

		temp1 = ov5647_reg_read(s, 0x3034);
		temp2 = temp1 & 0x0f;
		if (temp2 == 8 || temp2 == 10) {
				Bit_div2x = temp2 / 2;
		}

		temp1 = ov5647_reg_read(s, 0x3035);
		SysDiv = temp1>>4;
		if (SysDiv == 0) {
					 SysDiv = 16;
		}

		temp1 = ov5647_reg_read(s, 0x3036);
		Multiplier = temp1;

		temp1 = ov5647_reg_read(s, 0x3037);
		PreDiv = temp1 & 0x0f;
		Pll_rdiv = ((temp1 >> 4) & 0x01) + 1;

		temp1 = ov5647_reg_read(s, 0x3108);
		temp2 = temp1 & 0x03;
		sclk_rdiv = sclk_rdiv_map[temp2];

		VCO = xvclk * Multiplier / PreDiv;

		sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;

		pr_info("sysclk is %ikHz\n", sysclk);

		return sysclk;
}


/* V4L sub device --------------------------------------------------------- */

 /* Core ops */

static inline struct ov5647 *to_sensor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5647, sd);
}


static int
ov5647_co_get_register(struct v4l2_subdev *sd,
											 struct v4l2_dbg_register *reg)
{
	struct ov5647 *s = to_sensor(sd);
	reg->val = ov5647_reg_read(s, reg->reg & 0xffff);
	reg->size = 1;
	return 0;
}

static int
ov5647_co_set_register(struct v4l2_subdev *sd,
											 const struct v4l2_dbg_register *reg)
{
	struct ov5647 *s = to_sensor(sd);
	ov5647_reg_write(s, reg->reg & 0xffff, reg->val & 0xff);
	return 0;
}

static int
ov5647_co_set_power(struct v4l2_subdev *sd, int on)
{
	struct ov5647 *s = to_sensor(sd);
	int ret = 0;

	v4l2_dbg(1, debug, s->client, "%s: on: %d\n", __func__, on);

	if (on) {
		ov5647_power_on(s);

		ret = ov5647_reg_write_array(s, sensor_common, ARRAY_SIZE(sensor_common));
		if(ret < 0) {
				v4l_err(s->client, "sensor initialisation error\n");
				return ret;
			}	
	} else
		ov5647_power_off(s);

	return 0;
}

static const struct v4l2_subdev_core_ops ov5647_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register     = ov5647_co_get_register,
	.s_register     = ov5647_co_set_register,
#endif
	.s_power        = ov5647_co_set_power,
};


 /* Pad ops */

static int
ov5647_po_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
												 struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int
ov5647_po_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
													struct v4l2_subdev_frame_size_enum *fse)
{
	
	if (fse->index >= ARRAY_SIZE(ov5647_framesizes))
		return -EINVAL;

	fse->max_width  = ov5647_framesizes[fse->index].width;
	fse->max_height = ov5647_framesizes[fse->index].height;

	return 0; 
}

static int
ov5647_po_enum_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
															struct v4l2_subdev_frame_interval_enum *fie)
{
	const struct ov5647_framesize *fs;
	int i;

	if (fie->index >= ARRAY_SIZE(ov5647_framesizes))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ov5647_framesizes); i++) 
		{
			fs = &ov5647_framesizes[i];
			if (fie->width <= fs->width && fie->height <= fs->height)
				break;
	}

	fie->interval.numerator = 1;
	fie->interval.denominator = fs->fps;

	return 0;
}

static int
ov5647_try_format(struct ov5647 *s, struct v4l2_mbus_framefmt *fmt, struct ov5647_framesize **fsize)
{
	
	const struct ov5647_framesize *fs;
	int i;

	v4l2_dbg(1, debug, s->client, "%s: (%dx%d)\n",
		__func__, fmt->width, fmt->height);

	/*
		Check requested frame sizes against available and 
		choose the smallest that still fits the frame.
	*/
	for (i = 0; i < ARRAY_SIZE(ov5647_framesizes); i++) 
		{
			fs = &ov5647_framesizes[i];
			if (fmt->width <= fs->width && fmt->height <= fs->height)
				break;
	}

	fmt->width  = fs->width;
	fmt->height = fs->height;
	fmt->field  = V4L2_FIELD_NONE;

	*fsize = (struct ov5647_framesize *)fs;


	return 0;
}

static int
ov5647_po_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
									struct v4l2_subdev_format *format)
{
	struct ov5647 *s = to_sensor(sd);
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct ov5647_framesize *fsize;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		ov5647_try_format(s, mf, &fsize);
		return 0;
	}

	mf = &s->format;

	return 0; 
}

static int
ov5647_po_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
									struct v4l2_subdev_format *format)
{

	struct ov5647 *s = to_sensor(sd);
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct ov5647_framesize *fsize;
	int ret;

	v4l2_dbg(1, debug, s->client, "%s: (%dx%d)\n",
		__func__, mf->width, mf->height);

	ov5647_try_format(s, mf, &fsize);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0; 

	s->format.width = fsize->width;
	s->format.height = fsize->height;

	ov5647_co_set_power(sd, 1);

	ret = ov5647_reg_write_array(s, fsize->regs, fsize->regs_size);
	if(ret < 0) {
			v4l_err(s->client, "set format error\n");
			return ret;
		}

		/* read PCLK */
		ov5647_get_sysclk(s);  	

	return 0;
}

static int
ov5647_po_set_crop(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
									 struct v4l2_subdev_crop *crop)
{
	return 0; /* FIXME */
}

static int
ov5647_po_get_crop(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
									 struct v4l2_subdev_crop *crop)
{
	return 0; /* FIXME */
}

static const struct v4l2_subdev_pad_ops ov5647_subdev_pad_ops = {
	.enum_mbus_code	     = ov5647_po_enum_mbus_code,
	.enum_frame_size     = ov5647_po_enum_frame_size,
	.enum_frame_interval = ov5647_po_enum_frame_interval,
	.get_fmt             = ov5647_po_get_fmt,
	.set_fmt             = ov5647_po_set_fmt,
	.get_crop            = ov5647_po_get_crop,
	.set_crop            = ov5647_po_set_crop,
};


 /* Video ops */

static int
ov5647_vo_set_crystal_freq(struct v4l2_subdev *sd, u32 freq, u32 flags)
{
	struct ov5647 *s = to_sensor(sd);

	if ((freq < 6000000) || (freq > 27000000))
		return -EINVAL;

	s->xclk_freq = freq;

	return 0;
}

static int
ov5647_vo_set_stream(struct v4l2_subdev *sd, int on)
{
	struct ov5647 *s = to_sensor(sd);

	v4l2_dbg(1, debug, s->client, "%s: on: %d\n", __func__, on);

	if (on)
		ov5647_reg_write(s, 0x0100, 0x01);
	else {
		ov5647_reg_write(s, 0x0100, 0x00);
		ov5647_power_off(s);
	}

	return 0;
}

static const struct v4l2_subdev_video_ops ov5647_subdev_video_ops = {
	.s_crystal_freq = ov5647_vo_set_crystal_freq,
	.s_stream       = ov5647_vo_set_stream,
};


 /* SubDev ops */

static const struct v4l2_subdev_ops ov5647_ops = {
	.core  = &ov5647_subdev_core_ops,
	.pad   = &ov5647_subdev_pad_ops,
	.video = &ov5647_subdev_video_ops,
};


/* I2C Driver  ------------------------------------------------------------ */

static int
ov5647_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct ov5647 *s;
	int rv;

	/* Get private sensor struct */
	s = devm_kzalloc(&client->dev, sizeof(struct ov5647), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->client = client;
	s->client->flags = I2C_CLIENT_SCCB;

	/* Get GPIO */
	s->gpio_ena = devm_gpiod_get(&client->dev, "enable");
	if (!IS_ERR(s->gpio_ena)) {
		gpiod_direction_output(s->gpio_ena, 0);
	} else {
		s->gpio_ena = NULL;
	}

	/* Get XCLK frequency */
	rv = of_property_read_u32(client->dev.of_node,
			"clock-frequency", &s->xclk_freq);
	if (rv < 0)
		s->xclk_freq = 25000000; /* 25 MHz default */

	/* Power-on briefly to check it's the right sensor */
	ov5647_power_on(s);

	if ((ov5647_reg_read(s, 0x300A) != 0x56) ||
			(ov5647_reg_read(s, 0x300B) != 0x47))
	{
		v4l_err(client, "chip not found\n");
		return -ENODEV;
	}

	ov5647_power_off(s);

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
		client->addr << 1, client->adapter->name);

	/* V4L init */
	sd = &s->sd;
	v4l2_i2c_subdev_init(sd, client, &ov5647_ops);
	strlcpy(sd->name, DRIVER_NAME, sizeof(sd->name));

	s->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	rv = media_entity_init(&sd->entity, 1, &s->pad, 0);
	if (rv) {
		v4l_err(client, "Failed to initialise pad\n");
		goto done;
	}

	rv = v4l2_async_register_subdev(&s->sd);
	if (rv) {
		v4l_err(client, "Failed to register subdev for async detection\n");
		goto done;
	}

	rv = 0;

done:
	if (rv > 0) {
		media_entity_cleanup(&sd->entity);
	}

	return rv;
}

static int
ov5647_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

	return 0;
}


static const struct i2c_device_id ov5647_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5647_id);

static const struct of_device_id ov5647_of_match[] = {
	{ .compatible = "omnivision,ov5647" },
	{ }
};
MODULE_DEVICE_TABLE(of, ov5647_of_match);

static struct i2c_driver ov5647_driver = {
	.probe = ov5647_probe,
	.remove	= ov5647_remove,
	.id_table	= ov5647_id,
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
		.of_match_table	= ov5647_of_match,
	},
};
module_i2c_driver(ov5647_driver);

MODULE_AUTHOR("Jason Whittaker <jpwhitt@yahoo.com>");
MODULE_DESCRIPTION("Omnivision OV5647 5MP sensor driver (MIPI CSI-2 mode)");
MODULE_LICENSE("GPL");

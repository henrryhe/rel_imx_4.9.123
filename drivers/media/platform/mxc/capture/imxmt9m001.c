/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


/*
 * copied from ov5640.c; to integrate mt9m001 to imx6 video capture.
 * all driver codes is copied from 
 	drivers/media/i2c/soc_camera/mt9m001.c
 	参考ov5640驱动结构,加入mt9m001驱动
 * chuan.he 2018.12.29
 * 
 */


#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"
#include <media/soc_camera.h>



#define DEFAULT_FPS 30

/* mt9m001 selected register addresses */
#define MT9M001_CHIP_VERSION		0x00
#define MT9M001_ROW_START		0x01
#define MT9M001_COLUMN_START		0x02
#define MT9M001_WINDOW_HEIGHT		0x03
#define MT9M001_WINDOW_WIDTH		0x04
#define MT9M001_HORIZONTAL_BLANKING	0x05
#define MT9M001_VERTICAL_BLANKING	0x06
#define MT9M001_OUTPUT_CONTROL		0x07
#define MT9M001_SHUTTER_WIDTH		0x09
#define MT9M001_FRAME_RESTART		0x0b
#define MT9M001_SHUTTER_DELAY		0x0c
#define MT9M001_RESET			0x0d
#define MT9M001_READ_OPTIONS1		0x1e
#define MT9M001_READ_OPTIONS2		0x20
#define MT9M001_GLOBAL_GAIN		0x35
#define MT9M001_CHIP_ENABLE		0xF1

#define MT9M001_MAX_WIDTH		1280
#define MT9M001_MAX_HEIGHT		1024
#define MT9M001_MIN_WIDTH		48
#define MT9M001_MIN_HEIGHT		32
#define MT9M001_COLUMN_SKIP		20
#define MT9M001_ROW_SKIP		12


#define MT9M001_XCLK_MIN 6000000
#define MT9M001_XCLK_MAX 24000000



static int reg_read(struct i2c_client *client, const u8 reg)
{
	return i2c_smbus_read_word_swapped(client, reg);
}

static int reg_write(struct i2c_client *client, const u8 reg,
		     const u16 data)
{
	return i2c_smbus_write_word_swapped(client, reg, data);
}

static int reg_set(struct i2c_client *client, const u8 reg,
		   const u16 data)
{
	int ret;

	ret = reg_read(client, reg);
	if (ret < 0)
		return ret;
	return reg_write(client, reg, ret | data);
}

static int reg_clear(struct i2c_client *client, const u8 reg,
		     const u16 data)
{
	int ret;

	ret = reg_read(client, reg);
	if (ret < 0)
		return ret;
	return reg_write(client, reg, ret & ~data);
}


static int imxmt9m001_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int imxmt9m001_remove(struct i2c_client *client);


static struct sensor_data imxmt9m001_data;


static const struct i2c_device_id imxmt9m001_id[] = {
	{"imxmt9m001", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, imxmt9m001_id);

static struct i2c_driver imxmt9m001_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "imxmt9m001",
		  },
	.probe  = imxmt9m001_probe,
	.remove = imxmt9m001_remove,
	.id_table = imxmt9m001_id,
};


static int imxmt9m001_init(struct i2c_client *client)
{
	int ret;

	dev_dbg(&client->dev, "%s\n", __func__);
	KER_LOG("\n");

	/*
	 * We don't know, whether platform provides reset, issue a soft reset
	 * too. This returns all registers to their default values.
	 */
	ret = reg_write(client, MT9M001_RESET, 1);
	if (!ret)
		ret = reg_write(client, MT9M001_RESET, 0);

	/* Disable chip, synchronous option update */
	if (!ret)
		ret = reg_write(client, MT9M001_OUTPUT_CONTROL, 0);

	return ret;
}

static inline void imxmt9m001_reset(void)
{
	imxmt9m001_init(imxmt9m001_data.i2c_client);
}


/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}
	KER_LOG("\n");
	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = imxmt9m001_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", imxmt9m001_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = MT9M001_XCLK_MIN;
	p->u.bt656.clock_max = MT9M001_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */
	p->u.bt656.latch_clk_inv = 1;

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;
	KER_LOG("\n");
	sensor->on = on;
	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;

}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	int ret = 0;
	KER_LOG("\n");

	struct v4l2_rect rect;
	const u16 hblank = 9, vblank = 25;

	rect.top = 0;
	rect.left = 0;
	rect.width = 1280;
	rect.height = 1024;

	/* Datasheet requirement: see register description */
	rect.width = ALIGN(rect.width, 2);
	rect.left = ALIGN(rect.left, 2);

	soc_camera_limit_side(&rect.left, &rect.width,
		     MT9M001_COLUMN_SKIP, MT9M001_MIN_WIDTH, MT9M001_MAX_WIDTH);

	soc_camera_limit_side(&rect.top, &rect.height,
		     MT9M001_ROW_SKIP, MT9M001_MIN_HEIGHT, MT9M001_MAX_HEIGHT);


	/* Blanking and start values - default... */
	ret = reg_write(imxmt9m001_data.i2c_client, MT9M001_HORIZONTAL_BLANKING, hblank);
	if (!ret)
		ret = reg_write(imxmt9m001_data.i2c_client, MT9M001_VERTICAL_BLANKING, vblank);

	/*
	 * The caller provides a supported format, as verified per
	 * call to .try_mbus_fmt()
	 */
	if (!ret)
		ret = reg_write(imxmt9m001_data.i2c_client, MT9M001_COLUMN_START, rect.left);
	if (!ret)
		ret = reg_write(imxmt9m001_data.i2c_client, MT9M001_ROW_START, rect.top);
	if (!ret)
		ret = reg_write(imxmt9m001_data.i2c_client, MT9M001_WINDOW_WIDTH, rect.width - 1);
	if (!ret)
		ret = reg_write(imxmt9m001_data.i2c_client, MT9M001_WINDOW_HEIGHT,
				rect.height - 1);

				/*
	if (!ret && v4l2_ctrl_g_ctrl(mt9m001->autoexposure) == V4L2_EXPOSURE_AUTO)
		ret = reg_write(imxmt9m001_data.i2c_client, MT9M001_SHUTTER_WIDTH, mt9m001->total_h);
		*/

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;
	KER_LOG("sensor->pix.priv = %d\n",sensor->pix.priv);
	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;
	KER_LOG("\n");
	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	KER_LOG("\n");
	pr_debug("In imxmt9m001:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	KER_LOG("\n");
	if (fsize->index > 1)
		return -EINVAL;

	fsize->pixel_format = imxmt9m001_data.pix.pixelformat;
	fsize->discrete.width = 1280;
	fsize->discrete.height = 1024;

	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	int i, j, count;

	KER_LOG("\n");

	return -EINVAL;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	KER_LOG("\n");
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "mt9m001_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	KER_LOG("\n");
	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	KER_LOG("\n");
	if (fmt->index > 1)
		return -EINVAL;

	fmt->pixelformat = imxmt9m001_data.pix.pixelformat;
	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{

	int ret = 0;

	KER_LOG("\n");
	return ret;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	KER_LOG("\n");
	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc imxmt9m001_ioctl_desc[] = {
	{ vidioc_int_dev_init_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ vidioc_int_dev_exit_num,
	  ioctl_dev_exit},
	{ vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)ioctl_s_power },
	{ vidioc_int_g_ifparm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ifparm },
	{ vidioc_int_init_num,
	  (v4l2_int_ioctl_func *)ioctl_init },
	{ vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ vidioc_int_g_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ vidioc_int_g_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ vidioc_int_s_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_s_ctrl },
	{ vidioc_int_enum_framesizes_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ vidioc_int_enum_frameintervals_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
	{ vidioc_int_g_chip_ident_num,
	  (v4l2_int_ioctl_func *)ioctl_g_chip_ident },
};

static struct v4l2_int_slave imxmt9m001_slave = {
	.ioctls = imxmt9m001_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(imxmt9m001_ioctl_desc),
};

static struct v4l2_int_device imxmt9m001_int_device = {
	.module = THIS_MODULE,
	.name = "imxmt9m001",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &imxmt9m001_slave,
	},
};

//#define IIC_LOOP_TEST

/*!
 * imxmt9m001 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int imxmt9m001_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high, chip_id_low;
	s32 data;

	/* imxmt9m001 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

		/* Enable the chip */
	data = reg_write(client, MT9M001_CHIP_ENABLE, 1);
	dev_dbg(&client->dev, "write: %d\n", data);

	/* Read out the chip version register */
	data = reg_read(client, MT9M001_CHIP_VERSION);
	dev_info(&client->dev, "Detected a MT9M001 chip ID %x (%s)\n", data,
				data == 0x8431 ? "C12STM" : "C12ST");

	KER_LOG("\n");

	retval = of_property_read_u32(dev->of_node, "mclk",
					&imxmt9m001_data.mclk);
	if (retval) {
		dev_err(dev, "mclk frequency is invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(imxmt9m001_data.csi));
	if (retval) {
		dev_err(dev, "csi_id invalid\n");
		return retval;
	}

	//imxmt9m001_init(client);

#ifdef IIC_LOOP_TEST
	s32 data1;
	int i = 0;
	for (;i < 10000;i++){
		data1 = reg_read(client, MT9M001_CHIP_VERSION);
		KER_LOG("read data = 0x%x\n",data1);
		KER_LOG("read data = %d\n",data1);
		mdelay(500);
	} 
#endif

	clk_prepare_enable(imxmt9m001_data.sensor_clk);

	imxmt9m001_data.io_init = imxmt9m001_reset;
	
	imxmt9m001_data.i2c_client = client;
	imxmt9m001_data.pix.pixelformat = V4L2_PIX_FMT_GREY;
	imxmt9m001_data.pix.width = 1280;
	imxmt9m001_data.pix.height = 1024;
	imxmt9m001_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	imxmt9m001_data.streamcap.capturemode = 0;
	imxmt9m001_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	imxmt9m001_data.streamcap.timeperframe.numerator = 1;


	clk_disable_unprepare(imxmt9m001_data.sensor_clk);

	imxmt9m001_int_device.priv = &imxmt9m001_data;
	retval = v4l2_int_device_register(&imxmt9m001_int_device);

	pr_info("camera imxmt9m001 is found\n");
	return retval;
}

/*!
 * ov5640 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int imxmt9m001_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&imxmt9m001_int_device);

	return 0;
}

module_i2c_driver(imxmt9m001_i2c_driver);

MODULE_AUTHOR("chuan.he");
MODULE_DESCRIPTION("Micron MT9M001 Camera driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");

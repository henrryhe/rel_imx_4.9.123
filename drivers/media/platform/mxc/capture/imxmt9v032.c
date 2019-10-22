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
 * copied from ov5640.c; to integrate mt9v032 to imx6 video capture.
 * all driver codes is copied from 
 	drivers/media/i2c/mt9v032.c
 	参考ov5640驱动结构,加入mt9v032驱动
 * chuan.he 2019.03.04
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

/* The first four rows are black rows. The active area spans 753x481 pixels. */
#define MT9V032_PIXEL_ARRAY_HEIGHT			485
#define MT9V032_PIXEL_ARRAY_WIDTH			753

#define MT9V032_SYSCLK_FREQ_DEF				26600000

#define MT9V032_CHIP_VERSION				0x00
#define		MT9V032_CHIP_ID_REV1			0x1311
#define		MT9V032_CHIP_ID_REV3			0x1313
#define		MT9V034_CHIP_ID_REV1			0X1324
#define MT9V032_COLUMN_START				0x01
#define		MT9V032_COLUMN_START_MIN		1
#define		MT9V032_COLUMN_START_DEF		1
#define		MT9V032_COLUMN_START_MAX		752
#define MT9V032_ROW_START				0x02
#define		MT9V032_ROW_START_MIN			4
#define		MT9V032_ROW_START_DEF			5
#define		MT9V032_ROW_START_MAX			482
#define MT9V032_WINDOW_HEIGHT				0x03
#define		MT9V032_WINDOW_HEIGHT_MIN		1
#define		MT9V032_WINDOW_HEIGHT_DEF		480
#define		MT9V032_WINDOW_HEIGHT_MAX		480
#define MT9V032_WINDOW_WIDTH				0x04
#define		MT9V032_WINDOW_WIDTH_MIN		1
#define		MT9V032_WINDOW_WIDTH_DEF		752
#define		MT9V032_WINDOW_WIDTH_MAX		752
#define MT9V032_HORIZONTAL_BLANKING			0x05
#define		MT9V032_HORIZONTAL_BLANKING_MIN		43
#define		MT9V034_HORIZONTAL_BLANKING_MIN		61
#define		MT9V032_HORIZONTAL_BLANKING_DEF		94
#define		MT9V032_HORIZONTAL_BLANKING_MAX		1023
#define MT9V032_VERTICAL_BLANKING			0x06
#define		MT9V032_VERTICAL_BLANKING_MIN		4
#define		MT9V034_VERTICAL_BLANKING_MIN		2
#define		MT9V032_VERTICAL_BLANKING_DEF		45
#define		MT9V032_VERTICAL_BLANKING_MAX		3000
#define		MT9V034_VERTICAL_BLANKING_MAX		32288
#define MT9V032_CHIP_CONTROL				0x07
#define		MT9V032_CHIP_CONTROL_MASTER_MODE	(1 << 3)
#define		MT9V032_CHIP_CONTROL_DOUT_ENABLE	(1 << 7)
#define		MT9V032_CHIP_CONTROL_SEQUENTIAL		(1 << 8)
#define MT9V032_SHUTTER_WIDTH1				0x08
#define MT9V032_SHUTTER_WIDTH2				0x09
#define MT9V032_SHUTTER_WIDTH_CONTROL			0x0a
#define MT9V032_TOTAL_SHUTTER_WIDTH			0x0b
#define		MT9V032_TOTAL_SHUTTER_WIDTH_MIN		1
#define		MT9V034_TOTAL_SHUTTER_WIDTH_MIN		0
#define		MT9V032_TOTAL_SHUTTER_WIDTH_DEF		480
#define		MT9V032_TOTAL_SHUTTER_WIDTH_MAX		32767
#define		MT9V034_TOTAL_SHUTTER_WIDTH_MAX		32765
#define MT9V032_RESET					0x0c
#define MT9V032_READ_MODE				0x0d
#define		MT9V032_READ_MODE_ROW_BIN_MASK		(3 << 0)
#define		MT9V032_READ_MODE_ROW_BIN_SHIFT		0
#define		MT9V032_READ_MODE_COLUMN_BIN_MASK	(3 << 2)
#define		MT9V032_READ_MODE_COLUMN_BIN_SHIFT	2
#define		MT9V032_READ_MODE_ROW_FLIP		(1 << 4)
#define		MT9V032_READ_MODE_COLUMN_FLIP		(1 << 5)
#define		MT9V032_READ_MODE_DARK_COLUMNS		(1 << 6)
#define		MT9V032_READ_MODE_DARK_ROWS		(1 << 7)
#define		MT9V032_READ_MODE_RESERVED		0x0300
#define MT9V032_PIXEL_OPERATION_MODE			0x0f
#define		MT9V034_PIXEL_OPERATION_MODE_HDR	(1 << 0)
#define		MT9V034_PIXEL_OPERATION_MODE_COLOR	(1 << 1)
#define		MT9V032_PIXEL_OPERATION_MODE_COLOR	(1 << 2)
#define		MT9V032_PIXEL_OPERATION_MODE_HDR	(1 << 6)
#define MT9V032_ANALOG_GAIN				0x35
#define		MT9V032_ANALOG_GAIN_MIN			16
#define		MT9V032_ANALOG_GAIN_DEF			16
#define		MT9V032_ANALOG_GAIN_MAX			64
#define MT9V032_MAX_ANALOG_GAIN				0x36
#define		MT9V032_MAX_ANALOG_GAIN_MAX		127
#define MT9V032_FRAME_DARK_AVERAGE			0x42
#define MT9V032_DARK_AVG_THRESH				0x46
#define		MT9V032_DARK_AVG_LOW_THRESH_MASK	(255 << 0)
#define		MT9V032_DARK_AVG_LOW_THRESH_SHIFT	0
#define		MT9V032_DARK_AVG_HIGH_THRESH_MASK	(255 << 8)
#define		MT9V032_DARK_AVG_HIGH_THRESH_SHIFT	8
#define MT9V032_ROW_NOISE_CORR_CONTROL			0x70
#define		MT9V034_ROW_NOISE_CORR_ENABLE		(1 << 0)
#define		MT9V034_ROW_NOISE_CORR_USE_BLK_AVG	(1 << 1)
#define		MT9V032_ROW_NOISE_CORR_ENABLE		(1 << 5)
#define		MT9V032_ROW_NOISE_CORR_USE_BLK_AVG	(1 << 7)
#define MT9V032_PIXEL_CLOCK				0x74
#define MT9V034_PIXEL_CLOCK				0x72
#define		MT9V032_PIXEL_CLOCK_INV_LINE		(1 << 0)
#define		MT9V032_PIXEL_CLOCK_INV_FRAME		(1 << 1)
#define		MT9V032_PIXEL_CLOCK_XOR_LINE		(1 << 2)
#define		MT9V032_PIXEL_CLOCK_CONT_LINE		(1 << 3)
#define		MT9V032_PIXEL_CLOCK_INV_PXL_CLK		(1 << 4)
#define MT9V032_TEST_PATTERN				0x7f
#define		MT9V032_TEST_PATTERN_DATA_MASK		(1023 << 0)
#define		MT9V032_TEST_PATTERN_DATA_SHIFT		0
#define		MT9V032_TEST_PATTERN_USE_DATA		(1 << 10)
#define		MT9V032_TEST_PATTERN_GRAY_MASK		(3 << 11)
#define		MT9V032_TEST_PATTERN_GRAY_NONE		(0 << 11)
#define		MT9V032_TEST_PATTERN_GRAY_VERTICAL	(1 << 11)
#define		MT9V032_TEST_PATTERN_GRAY_HORIZONTAL	(2 << 11)
#define		MT9V032_TEST_PATTERN_GRAY_DIAGONAL	(3 << 11)
#define		MT9V032_TEST_PATTERN_ENABLE		(1 << 13)
#define		MT9V032_TEST_PATTERN_FLIP		(1 << 14)
#define MT9V032_AEC_AGC_ENABLE				0xaf
#define		MT9V032_AEC_ENABLE			(1 << 0)
#define		MT9V032_AGC_ENABLE			(1 << 1)
#define MT9V032_THERMAL_INFO				0xc1


//add by chuan.he
#define MT9V032_XCLK_MIN	(13000000)
#define MT9V032_XCLK_MAX	(27000000)



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


static int imxmt9v032_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int imxmt9v032_remove(struct i2c_client *client);


static struct sensor_data imxmt9v032_data;


static const struct i2c_device_id imxmt9v032_id[] = {
	{"imxmt9v032", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, imxmt9v032_id);

static struct i2c_driver imxmt9v032_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "imxmt9v032",
		  },
	.probe  = imxmt9v032_probe,
	.remove = imxmt9v032_remove,
	.id_table = imxmt9v032_id,
};


static int imxmt9v032_init(struct i2c_client *client)
{
	int ret;

	dev_dbg(&client->dev, "%s\n", __func__);
	KER_LOG("\n");

	/*
	 * We don't know, whether platform provides reset, issue a soft reset
	 * too. This returns all registers to their default values.
	 */
	ret = reg_write(client, MT9V032_RESET, 1);
	if (!ret)
		ret = reg_write(client, MT9V032_RESET, 0);

	/* Disable chip, synchronous option update */
	if (!ret)
		ret = reg_write(client, MT9V032_CHIP_CONTROL, 0);

	return ret;
}

static inline void imxmt9v032_reset(void)
{
	imxmt9v032_init(imxmt9v032_data.i2c_client);
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
	p->u.bt656.clock_curr = imxmt9v032_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", imxmt9v032_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = MT9V032_XCLK_MIN;
	p->u.bt656.clock_max = MT9V032_XCLK_MAX;
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
	//暂时不用设置,用默认参数
	return ret;

	struct v4l2_rect rect;
	const u16 hblank = 9, vblank = 25;

	rect.top = 0;
	rect.left = 0;
	rect.width = MT9V032_WINDOW_WIDTH_DEF;
	rect.height = MT9V032_WINDOW_HEIGHT_DEF;

	/* Datasheet requirement: see register description */
	rect.width = ALIGN(rect.width, 2);
	rect.left = ALIGN(rect.left, 2);

	/*
	soc_camera_limit_side(&rect.left, &rect.width,
		     MT9M001_COLUMN_SKIP, MT9M001_MIN_WIDTH, MT9M001_MAX_WIDTH);

	soc_camera_limit_side(&rect.top, &rect.height,
		     MT9M001_ROW_SKIP, MT9V032_WINDOW_HEIGHT_MIN, MT9V032_WINDOW_HEIGHT_MAX);
	*/

	/* Blanking and start values - default... */
	ret = reg_write(imxmt9v032_data.i2c_client, MT9V032_HORIZONTAL_BLANKING, hblank);
	if (!ret)
		ret = reg_write(imxmt9v032_data.i2c_client, MT9V032_VERTICAL_BLANKING, vblank);

	/*
	 * The caller provides a supported format, as verified per
	 * call to .try_mbus_fmt()
	 */
	if (!ret)
		ret = reg_write(imxmt9v032_data.i2c_client, MT9V032_COLUMN_START, rect.left);
	if (!ret)
		ret = reg_write(imxmt9v032_data.i2c_client, MT9V032_ROW_START, rect.top);
	if (!ret)
		ret = reg_write(imxmt9v032_data.i2c_client, MT9V032_WINDOW_WIDTH, rect.width - 1);
	if (!ret)
		ret = reg_write(imxmt9v032_data.i2c_client, MT9V032_WINDOW_HEIGHT,
				rect.height - 1);

				/*
	if (!ret && v4l2_ctrl_g_ctrl(mt9m001->autoexposure) == V4L2_EXPOSURE_AUTO)
		ret = reg_write(imxmt9v032_data.i2c_client, MT9M001_SHUTTER_WIDTH, mt9m001->total_h);
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


static int set_auto_gain_exposure_on(bool status)
{
	s32 data = 0, ret = 0, bits = 0x03;
	data = reg_read(imxmt9v032_data.i2c_client, MT9V032_AEC_AGC_ENABLE);

	if (status == true) {
		data = data | bits;
		ret = reg_write(imxmt9v032_data.i2c_client, MT9V032_AEC_AGC_ENABLE, data);
	} else {
		data = data & (~bits);
		ret = reg_write(imxmt9v032_data.i2c_client, MT9V032_AEC_AGC_ENABLE, data);
	}

	if (ret < 0) {
		KER_LOG("set auto gain and exposure error ret = [0x%08x]\n", ret);
	}
	
	return ret;
}
static int ioctl_get_gain(void)
{
	s32 data = 0;
	data = reg_read(imxmt9v032_data.i2c_client, MT9V032_ANALOG_GAIN);

	return data;
}

static int ioctl_get_exposure(void)
{
	s32 data = 0;

	data = reg_read(imxmt9v032_data.i2c_client, MT9V032_TOTAL_SHUTTER_WIDTH);
	
	return data;
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
	//KER_LOG("\n");
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
		vc->value = ioctl_get_exposure();
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		vc->value = ioctl_get_gain();
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		ret = -EPERM;
		break;
	}
	return ret;
}

/*
* set vertical or horizontal flip or both
*/
static void ioctl_set_flip(int flipcontrol)
{
	int ret = 0;
	s32 data = 0;
	data = reg_read(imxmt9v032_data.i2c_client, MT9V032_READ_MODE);
	KER_LOG("data=[0x%8x]\n",data);
	ret = reg_write(imxmt9v032_data.i2c_client, MT9V032_READ_MODE, data|flipcontrol);
	if (ret < 0) {
		KER_LOG("datawrite error: %d \n", data|flipcontrol);
	}
}

static int ioctl_set_gain(int gain)
{
	int ret = 0;
	
	ret = set_auto_gain_exposure_on(false);
	if (ret < 0) {
		KER_LOG("set_auto_gain_exposure_on error ret = [0x%08x]\n", ret);
		return ret;
	}
	
	ret = reg_write(imxmt9v032_data.i2c_client, MT9V032_ANALOG_GAIN, gain);
	if (ret < 0) {
		KER_LOG("datawrite error ret = [0x%08x]\n", ret);
	}

	
	return ret;
}

static int ioctl_set_exposure(int exposure)
{
	int ret = 0;

	ret = set_auto_gain_exposure_on(false);
	if (ret < 0) {
		KER_LOG("set_auto_gain_exposure_on error ret = [0x%08x]\n", ret);
		return ret;
	}
	
	ret = reg_write(imxmt9v032_data.i2c_client, MT9V032_TOTAL_SHUTTER_WIDTH, exposure);
	if (ret < 0) {
		KER_LOG("datawrite error ret = [0x%08x]\n",ret);
	}
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
	//KER_LOG("\n");
	//pr_debug("In imxmt9v032:ioctl_s_ctrl %d\n", vc->id);

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
		retval = ioctl_set_exposure(vc->value);
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		retval = ioctl_set_gain(vc->value);
		break;
	case V4L2_CID_HFLIP:
		ioctl_set_flip(MT9V032_READ_MODE_ROW_FLIP);
		break;
	case V4L2_CID_VFLIP:
		ioctl_set_flip(MT9V032_READ_MODE_COLUMN_FLIP);
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

	fsize->pixel_format = imxmt9v032_data.pix.pixelformat;
	fsize->discrete.width = MT9V032_WINDOW_WIDTH_DEF;
	fsize->discrete.height = MT9V032_WINDOW_HEIGHT_DEF;

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
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "mt9v032_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	KER_LOG("\n");
	ioctl_set_flip(MT9V032_READ_MODE_ROW_FLIP|MT9V032_READ_MODE_COLUMN_FLIP);
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

	fmt->pixelformat = imxmt9v032_data.pix.pixelformat;
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
static struct v4l2_int_ioctl_desc imxmt9v032_ioctl_desc[] = {
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

static struct v4l2_int_slave imxmt9v032_slave = {
	.ioctls = imxmt9v032_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(imxmt9v032_ioctl_desc),
};

static struct v4l2_int_device imxmt9v032_int_device = {
	.module = THIS_MODULE,
	.name = "imxmt9v032",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &imxmt9v032_slave,
	},
};

//#define IIC_LOOP_TEST

/*!
 * imxmt9v032 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int imxmt9v032_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high, chip_id_low;
	s32 data;

	/* imxmt9v032 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

		/* Enable the chip */
	/*data = reg_write(client, MT9M001_CHIP_ENABLE, 1);
	dev_dbg(&client->dev, "write: %d\n", data);*/

	/* Read out the chip version register */
	data = reg_read(client, MT9V032_CHIP_VERSION);
	dev_info(&client->dev, "Detected a MT9V032 chip ID %x (%s)\n", data,
				data == MT9V032_CHIP_ID_REV1 ? "REV1" : "REV3");

	KER_LOG("client.addr = %2x\n", client->addr);

	retval = of_property_read_u32(dev->of_node, "mclk",
					&imxmt9v032_data.mclk);
	if (retval) {
		dev_err(dev, "mclk frequency is invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(imxmt9v032_data.csi));
	if (retval) {
		dev_err(dev, "csi_id invalid\n");
		return retval;
	}

#ifdef IIC_LOOP_TEST
	s32 data1;
	int i = 0;
	for (;i < 10000;i++){
		data1 = reg_read(client, MT9V032_CHIP_VERSION);
		KER_LOG("read data = 0x%x\n",data1);
		KER_LOG("read data = %d\n",data1);
		mdelay(500);
	} 
#endif

	clk_prepare_enable(imxmt9v032_data.sensor_clk);

	imxmt9v032_data.io_init = imxmt9v032_reset;
	
	imxmt9v032_data.i2c_client = client;
	imxmt9v032_data.pix.pixelformat = V4L2_PIX_FMT_GREY;
	imxmt9v032_data.pix.width = MT9V032_WINDOW_WIDTH_DEF;
	imxmt9v032_data.pix.height = MT9V032_WINDOW_HEIGHT_DEF;
	imxmt9v032_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	imxmt9v032_data.streamcap.capturemode = 0;
	imxmt9v032_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	imxmt9v032_data.streamcap.timeperframe.numerator = 1;


	clk_disable_unprepare(imxmt9v032_data.sensor_clk);

	imxmt9v032_int_device.priv = &imxmt9v032_data;
	retval = v4l2_int_device_register(&imxmt9v032_int_device);

	pr_info("camera imxmt9v032 is found\n");
	return retval;
}

/*!
 * ov5640 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int imxmt9v032_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&imxmt9v032_int_device);

	return 0;
}

module_i2c_driver(imxmt9v032_i2c_driver);

MODULE_AUTHOR("chuan.he");
MODULE_DESCRIPTION("Aptina MT9V032 Camera driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");

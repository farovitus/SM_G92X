// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S5K4E6 5MP (2608x1960) MIPI CSI-2 image sensor driver
 *
 * Extracted from the Galaxy S6 (SM-G920F / zeroflte) FIMC-IS firmware
 * sequences for 2-lane RAW10 MIPI Normal Mode on CSIS1 -> FIMC-LITE B.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <asm/unaligned.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define S5K4E6_DRV_NAME			"s5k4e6"

#define S5K4E6_MCLK_FREQ_HZ		24000000UL
#define S5K4E6_LINK_FREQ_HZ		504000000ULL
#define S5K4E6_PIXEL_RATE_HZ		180000000ULL

#define S5K4E6_NATIVE_WIDTH		2608U
#define S5K4E6_NATIVE_HEIGHT		1960U

#define S5K4E6_REG_CHIP_ID		0x0000
#define S5K4E6_REG_REV_NUM		0x0002
#define S5K4E6_REG_FRAME_COUNT		0x0005
#define S5K4E6_REG_MODE_SELECT		0x0100
#define S5K4E6_REG_PARAM_HOLD		0x0104
#define S5K4E6_REG_COARSE_INT_TIME	0x0202
#define S5K4E6_REG_ANALOGUE_GAIN	0x0204
#define S5K4E6_REG_DIGITAL_GAIN_GR	0x020e
#define S5K4E6_REG_FRAME_LENGTH_LINES	0x0340
#define S5K4E6_REG_VFLIP		0x5333
#define S5K4E6_REG_SW_RESET		0x6010
#define S5K4E6_REG_PAGE_SELECT		0x6028

#define S5K4E6_VTS_MAX			0xffff
#define S5K4E6_VBLANK_MIN		40
#define S5K4E6_EXPO_MIN			4
#define S5K4E6_EXPO_DEFAULT		1000
#define S5K4E6_EXPO_MARGIN		4
#define S5K4E6_AGAIN_MIN		32	/* 1.0x in Q5 fixed point */
#define S5K4E6_AGAIN_DEFAULT		32
#define S5K4E6_AGAIN_MAX		512	/* 16.0x */
#define S5K4E6_DGAIN_MIN		256	/* 1.0x in Q8 fixed point */
#define S5K4E6_DGAIN_DEFAULT		256
/*
 * Keep hardware digital gain at unity (1.0x) and apply extra digital gain
 * cleanly in SoftISP after per-channel AWB and black-level subtraction.
 */
#define S5K4E6_DGAIN_MAX		256

#define S5K4E6_VFLIP_NORMAL		0xe0
#define S5K4E6_VFLIP_MIRROR		0xe2

struct s5k4e6_i2c_op {
	u8 len;
	u8 data[18];
};

#define S5K4E6_W8(reg, val) { \
	.len = 3, \
	.data = { ((reg) >> 8) & 0xff, (reg) & 0xff, (val) & 0xff }, \
}

#define S5K4E6_W16(reg, val) { \
	.len = 4, \
	.data = { ((reg) >> 8) & 0xff, (reg) & 0xff, \
		  ((val) >> 8) & 0xff, (val) & 0xff }, \
}

#define S5K4E6_BURST(n, ...) { \
	.len = (n), \
	.data = { __VA_ARGS__ }, \
}

static const char * const s5k4e6_supply_names[] = {
	"vt-vdda",
	"vt-vddio",
	"vt-vddd",
};

#define S5K4E6_NUM_SUPPLIES ARRAY_SIZE(s5k4e6_supply_names)

struct s5k4e6_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts_def;
	u16 reg_066x;
	u16 reg_6f12;
	u8 reg_32aa;
	u8 reg_3500;
	u8 reg_3089;
	bool sram_066x_before;
};

static const struct s5k4e6_i2c_op s5k4e6_init_pre[] = {
	S5K4E6_W16(0x6004, 0x0001),
	S5K4E6_W8(0x535b, 0x00),
	S5K4E6_W8(0x5402, 0x15),
	S5K4E6_W8(0x5401, 0x1d),
	S5K4E6_W16(0x6102, 0xc000),
	S5K4E6_W16(0x614c, 0x25aa),
	S5K4E6_W16(0x614e, 0x25b8),
	S5K4E6_W16(0x618c, 0x08d4),
	S5K4E6_W16(0x618e, 0x08d6),
	S5K4E6_W16(0x6004, 0x0001),
	S5K4E6_W16(0x6028, 0x2000),
	S5K4E6_W16(0x602a, 0x11a8),
	S5K4E6_BURST(10, 0x6f, 0x12, 0x3a, 0xf9, 0x14, 0x10, 0x39, 0xf9, 0x14, 0x10),
	S5K4E6_W16(0x6028, 0x2000),
};

static const struct s5k4e6_i2c_op s5k4e6_init_sram[] = {
	S5K4E6_W16(0x602a, 0x12bc),
	S5K4E6_W16(0x6f12, 0x1020),
	S5K4E6_W16(0x602a, 0x12c2),
	S5K4E6_BURST(6, 0x6f, 0x12, 0x10, 0x20, 0x10, 0x20),
	S5K4E6_W16(0x602a, 0x12ca),
	S5K4E6_BURST(6, 0x6f, 0x12, 0x10, 0x20, 0x10, 0x10),
	S5K4E6_W16(0x602a, 0x12fc),
	S5K4E6_W16(0x6f12, 0x1020),
	S5K4E6_W16(0x602a, 0x1302),
	S5K4E6_BURST(6, 0x6f, 0x12, 0x10, 0x20, 0x10, 0x20),
	S5K4E6_W16(0x602a, 0x130a),
	S5K4E6_BURST(6, 0x6f, 0x12, 0x10, 0x20, 0x10, 0x10),
	S5K4E6_W16(0x602a, 0x14b8),
	S5K4E6_W8(0x6f12, 0x01),
	S5K4E6_W16(0x602a, 0x14b9),
	S5K4E6_W8(0x6f12, 0x01),
	S5K4E6_W16(0x602a, 0x14c0),
	S5K4E6_BURST(18, 0x6f, 0x12,
		     0x00, 0x00, 0xff, 0xda, 0xff, 0xda, 0x00, 0x00,
		     0x00, 0x00, 0xff, 0xda, 0xff, 0xda, 0x00, 0x00),
	S5K4E6_W16(0x602a, 0x1488),
	S5K4E6_BURST(10, 0x6f, 0x12, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80),
	S5K4E6_W16(0x602a, 0x1496),
	S5K4E6_BURST(10, 0x6f, 0x12, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80, 0xff, 0x80),
	S5K4E6_W16(0x602a, 0x14a4),
	S5K4E6_BURST(10, 0x6f, 0x12, 0xff, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xff, 0xc0),
	S5K4E6_W16(0x602a, 0x147a),
	S5K4E6_BURST(8, 0x6f, 0x12, 0x00, 0x00, 0x00, 0x02, 0xff, 0xfc),
	S5K4E6_W16(0x602a, 0x0512),
	S5K4E6_W16(0x6f12, 0x0111),
};

static const struct s5k4e6_i2c_op s5k4e6_init_post_sram[] = {
	S5K4E6_W16(0x602a, 0x14ac),
	S5K4E6_W16(0x6f12, 0x0000),
	S5K4E6_W16(0x602a, 0x0524),
	S5K4E6_W8(0x6f12, 0x00),
	S5K4E6_W8(0x3285, 0x00),
	S5K4E6_W16(0x327a, 0x0001),
	S5K4E6_W8(0x3283, 0x0a),
	S5K4E6_W8(0x3297, 0x18),
	S5K4E6_W8(0x32e1, 0x00),
	S5K4E6_W16(0x3286, 0x9000),
	S5K4E6_W8(0x3298, 0x40),
};

static const struct s5k4e6_i2c_op s5k4e6_init_analog[] = {
	S5K4E6_W16(0x327c, 0x0400),
	S5K4E6_W16(0x328a, 0x0800),
	S5K4E6_W8(0x3284, 0x37),
	S5K4E6_W8(0x32a1, 0x20),
	S5K4E6_W8(0x32a2, 0x10),
	S5K4E6_W8(0x32a4, 0x0c),
	S5K4E6_W16(0x3204, 0x000c),
	S5K4E6_W16(0x3206, 0x000b),
	S5K4E6_W16(0x3208, 0x0009),
	S5K4E6_W16(0x3210, 0x0007),
	S5K4E6_W16(0x3212, 0x0007),
	S5K4E6_W16(0x0200, 0x0408),
	S5K4E6_W8(0x3219, 0x1c),
	S5K4E6_W8(0x321a, 0x32),
	S5K4E6_W8(0x321b, 0x24),
	S5K4E6_W8(0x321c, 0x07),
	S5K4E6_W8(0x321e, 0x08),
	S5K4E6_W8(0x3220, 0x13),
	S5K4E6_W8(0x3226, 0x52),
	S5K4E6_W8(0x3227, 0x5c),
	S5K4E6_W8(0x3228, 0x03),
	S5K4E6_W8(0x0305, 0x06),
	S5K4E6_W8(0x0307, 0xb4),
	S5K4E6_W8(0x5363, 0x00),
	S5K4E6_W8(0x5364, 0x42),
	S5K4E6_W8(0x5365, 0x98),
	S5K4E6_W8(0x534e, 0x49),
	S5K4E6_W8(0x534f, 0x10),
};

static const struct s5k4e6_mode s5k4e6_modes[] = {
	{
		/* 2608x1960 @ 30 fps, 1008 Mbps 2-lane */
		.width = 2608,
		.height = 1960,
		.hts = 0x0b68,
		.vts_def = 0x07f8,
		.reg_066x = 0x0668,
		.reg_6f12 = 0x4010,
		.reg_32aa = 0x00,
		.reg_3500 = 0x00,
		.reg_3089 = 0x00,
		.sram_066x_before = true,
	},
	{
		/* 1304x980 @ 30 fps (2x2 binned), 1008 Mbps 2-lane */
		.width = 1304,
		.height = 980,
		.hts = 0x1612,
		.vts_def = 0x0424,
		.reg_066x = 0x066a,
		.reg_6f12 = 0x4110,
		.reg_32aa = 0x01,
		.reg_3500 = 0x01,
		.reg_3089 = 0x01,
		.sram_066x_before = false,
	},
};

static const s64 s5k4e6_link_freq_menu[] = {
	S5K4E6_LINK_FREQ_HZ,
};

struct s5k4e6 {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct regulator_bulk_data supplies[S5K4E6_NUM_SUPPLIES];
	struct clk *mclk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_active;
	struct gpio_desc *reset;
	struct gpio_desc *standby;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *dgain;
	struct v4l2_ctrl *vflip;

	const struct s5k4e6_mode *mode;
	u8 rev;
	bool streaming;
};

static inline struct s5k4e6 *to_s5k4e6(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k4e6, sd);
}

static int s5k4e6_write_raw(struct i2c_client *client, const u8 *buf, size_t len)
{
	int ret;

	ret = i2c_master_send(client, buf, len);
	if (ret < 0)
		return ret;
	if (ret != len)
		return -EIO;
	return 0;
}

static int s5k4e6_write8(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };

	return s5k4e6_write_raw(client, buf, sizeof(buf));
}

static int s5k4e6_write16(struct i2c_client *client, u16 reg, u16 val)
{
	u8 buf[4] = { reg >> 8, reg & 0xff, val >> 8, val & 0xff };

	return s5k4e6_write_raw(client, buf, sizeof(buf));
}

static int s5k4e6_read(struct i2c_client *client, u16 reg, u8 *buf, size_t len)
{
	u8 wbuf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .len = 2, .buf = wbuf },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = len, .buf = buf },
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		return ret;
	return ret == 2 ? 0 : -EIO;
}

static int s5k4e6_write_seq(struct i2c_client *client,
			    const struct s5k4e6_i2c_op *ops, size_t count)
{
	size_t i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = s5k4e6_write_raw(client, ops[i].data, ops[i].len);
		if (ret)
			return ret;
	}
	return 0;
}

static int s5k4e6_apply_mode(struct s5k4e6 *sensor, const struct s5k4e6_mode *m)
{
	struct i2c_client *client = sensor->client;
	int ret;

	ret = s5k4e6_write_seq(client, s5k4e6_init_pre,
			       ARRAY_SIZE(s5k4e6_init_pre));
	if (ret)
		return ret;

	if (m->sram_066x_before) {
		ret = s5k4e6_write16(client, 0x602a, m->reg_066x);
		ret = ret ?: s5k4e6_write16(client, 0x6f12, m->reg_6f12);
		if (ret)
			return ret;
	}

	ret = s5k4e6_write_seq(client, s5k4e6_init_sram,
			       ARRAY_SIZE(s5k4e6_init_sram));
	if (ret)
		return ret;

	if (!m->sram_066x_before) {
		ret = s5k4e6_write16(client, 0x602a, m->reg_066x);
		ret = ret ?: s5k4e6_write16(client, 0x6f12, m->reg_6f12);
		if (ret)
			return ret;
	}

	ret = s5k4e6_write_seq(client, s5k4e6_init_post_sram,
			       ARRAY_SIZE(s5k4e6_init_post_sram));
	if (ret)
		return ret;

	ret = s5k4e6_write8(client, 0x32aa, m->reg_32aa);
	if (ret)
		return ret;

	ret = s5k4e6_write_seq(client, s5k4e6_init_analog,
			       ARRAY_SIZE(s5k4e6_init_analog));
	if (ret)
		return ret;

	ret = s5k4e6_write16(client, S5K4E6_REG_FRAME_LENGTH_LINES, m->vts_def);
	ret = ret ?: s5k4e6_write16(client, 0x0342, m->hts);
	ret = ret ?: s5k4e6_write16(client, 0x021e, 0x03fc);
	ret = ret ?: s5k4e6_write16(client, 0x0344, 0x0000);
	ret = ret ?: s5k4e6_write16(client, 0x0346, 0x0000);
	ret = ret ?: s5k4e6_write16(client, 0x0348, 0x0a2f);
	ret = ret ?: s5k4e6_write16(client, 0x034a, 0x07a7);
	ret = ret ?: s5k4e6_write16(client, 0x034c, m->width);
	ret = ret ?: s5k4e6_write16(client, 0x034e, m->height);
	ret = ret ?: s5k4e6_write8(client, 0x3500, m->reg_3500);
	ret = ret ?: s5k4e6_write8(client, 0x3089, m->reg_3089);
	ret = ret ?: s5k4e6_write8(client, 0x0216, 0x01);
	ret = ret ?: s5k4e6_write8(client, S5K4E6_REG_VFLIP, S5K4E6_VFLIP_NORMAL);
	ret = ret ?: s5k4e6_write8(client, 0x5080, 0x01);
	ret = ret ?: s5k4e6_write16(client, 0x6004, 0x0000);
	return ret;
}

static u32 s5k4e6_get_format_code(struct s5k4e6 *sensor)
{
	/*
	 * Native S5K4E6 Bayer order (0x5333=0xe0) is GRBG; toggling VFLIP shifts
	 * rows by 1 line and turns GRBG into BGGR.
	 */
	return sensor->vflip->val ? MEDIA_BUS_FMT_SBGGR10_1X10 :
				    MEDIA_BUS_FMT_SGRBG10_1X10;
}

static void s5k4e6_fill_format(struct s5k4e6 *sensor,
			       const struct s5k4e6_mode *mode,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5k4e6_get_format_code(sensor);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k4e6_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4e6 *sensor = to_s5k4e6(sd);
	int i, ret;

	/* Hold normal-mode pin states before rails rise. */
	gpiod_set_value_cansleep(sensor->standby, 0);
	gpiod_set_value_cansleep(sensor->reset, 1);

	for (i = 0; i < S5K4E6_NUM_SUPPLIES; i++) {
		ret = regulator_enable(sensor->supplies[i].consumer);
		if (ret)
			goto err_reg;
	}
	usleep_range(1000, 1500);

	ret = clk_set_rate(sensor->mclk, S5K4E6_MCLK_FREQ_HZ);
	ret = ret ?: clk_prepare_enable(sensor->mclk);
	if (ret)
		goto err_reg;

	if (sensor->pins_active) {
		ret = pinctrl_select_state(sensor->pinctrl, sensor->pins_active);
		if (ret)
			goto err_clk;
	}
	usleep_range(1000, 1500);

	/* Release reset with standby kept low to select MIPI Normal Mode. */
	gpiod_set_value_cansleep(sensor->reset, 0);
	usleep_range(1000, 2000);
	return 0;

err_clk:
	clk_disable_unprepare(sensor->mclk);
err_reg:
	while (i--)
		regulator_disable(sensor->supplies[i].consumer);
	return ret;
}

static int s5k4e6_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4e6 *sensor = to_s5k4e6(sd);
	int i;

	gpiod_set_value_cansleep(sensor->reset, 1);
	usleep_range(10, 20);
	if (sensor->pins_default)
		pinctrl_select_state(sensor->pinctrl, sensor->pins_default);
	clk_disable_unprepare(sensor->mclk);
	for (i = S5K4E6_NUM_SUPPLIES - 1; i >= 0; i--)
		regulator_disable(sensor->supplies[i].consumer);
	gpiod_set_value_cansleep(sensor->standby, 0);

	return 0;
}

static int s5k4e6_identify(struct s5k4e6 *sensor)
{
	u8 id_buf[2];
	u8 rev = 0;
	int ret;

	ret = s5k4e6_read(sensor->client, S5K4E6_REG_CHIP_ID, id_buf, 2);
	if (ret) {
		dev_err(sensor->dev, "failed to read chip id at 0x%02x: %d\n",
			sensor->client->addr, ret);
		return ret;
	}

	ret = s5k4e6_read(sensor->client, S5K4E6_REG_REV_NUM, &rev, 1);
	if (ret) {
		dev_err(sensor->dev, "failed to read revision: %d\n", ret);
		return ret;
	}

	sensor->rev = rev;
	dev_info(sensor->dev, "s5k4e6 detected: id=0x%04x rev=%u\n",
		 get_unaligned_be16(id_buf), rev);

	return 0;
}

static int s5k4e6_write_expo_again(struct i2c_client *client, u16 expo, u16 again)
{
	/*
	 * S5K4E6 firmware (fimc_is_fw2 0x8b88e..0x8b93c) updates both the
	 * primary (0x0202/0x0204) and secondary (0x021e/0x0220) exposure/gain
	 * register banks as 6-byte bursts because 0x0216=0x01 stays active.
	 */
	static const u16 bases[] = { S5K4E6_REG_COARSE_INT_TIME, 0x021e };
	size_t i;

	for (i = 0; i < ARRAY_SIZE(bases); i++) {
		u8 buf[6] = {
			bases[i] >> 8, bases[i] & 0xff,
			expo >> 8, expo & 0xff,
			again >> 8, again & 0xff,
		};
		int ret = s5k4e6_write_raw(client, buf, sizeof(buf));
		if (ret)
			return ret;
	}
	return 0;
}

static int s5k4e6_set_dgain(struct i2c_client *client, u16 gain)
{
	/*
	 * Mirror 10-byte quad-channel digital gain bursts to both 0x020e..0x0214
	 * and 0x0222..0x0228 matching fimc_is_fw2 0x8b8c8..0x8b97c.
	 */
	static const u16 bases[] = { S5K4E6_REG_DIGITAL_GAIN_GR, 0x0222 };
	size_t i;

	for (i = 0; i < ARRAY_SIZE(bases); i++) {
		u8 hi = gain >> 8;
		u8 lo = gain & 0xff;
		u8 buf[10] = {
			bases[i] >> 8, bases[i] & 0xff,
			hi, lo, hi, lo, hi, lo, hi, lo,
		};
		int ret = s5k4e6_write_raw(client, buf, sizeof(buf));
		if (ret)
			return ret;
	}
	return 0;
}

static int s5k4e6_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4e6 *sensor =
		container_of(ctrl->handler, struct s5k4e6, ctrl_handler);
	struct i2c_client *client = sensor->client;
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int expo_max = sensor->mode->height + ctrl->val -
			       S5K4E6_EXPO_MARGIN;
		int expo_def = min(expo_max, S5K4E6_EXPO_DEFAULT);

		__v4l2_ctrl_modify_range(sensor->exposure,
					 sensor->exposure->minimum,
					 expo_max, sensor->exposure->step,
					 expo_def);
	}

	ret = pm_runtime_get_if_in_use(sensor->dev);
	if (ret <= 0)
		return ret == -EINVAL ? 0 : ret;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = s5k4e6_write_expo_again(client, ctrl->val,
					      sensor->again->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = s5k4e6_write_expo_again(client, sensor->exposure->val,
					      ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = s5k4e6_set_dgain(client, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = s5k4e6_write16(client, S5K4E6_REG_FRAME_LENGTH_LINES,
				     sensor->mode->height + ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		ret = s5k4e6_write8(client, S5K4E6_REG_VFLIP,
				    ctrl->val ? S5K4E6_VFLIP_MIRROR :
						S5K4E6_VFLIP_NORMAL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(sensor->dev);
	return ret;
}

static const struct v4l2_ctrl_ops s5k4e6_ctrl_ops = {
	.s_ctrl = s5k4e6_set_ctrl,
};

static int s5k4e6_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	struct s5k4e6 *sensor = to_s5k4e6(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;

	fmt = v4l2_subdev_get_pad_format(sd, state, 0);
	s5k4e6_fill_format(sensor, &s5k4e6_modes[0], fmt);

	crop = v4l2_subdev_get_pad_crop(sd, state, 0);
	crop->left = 0;
	crop->top = 0;
	crop->width = S5K4E6_NATIVE_WIDTH;
	crop->height = S5K4E6_NATIVE_HEIGHT;

	return 0;
}

static int s5k4e6_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k4e6 *sensor = to_s5k4e6(sd);

	if (code->index > 0)
		return -EINVAL;

	code->code = s5k4e6_get_format_code(sensor);
	return 0;
}

static int s5k4e6_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k4e6 *sensor = to_s5k4e6(sd);

	if (fse->index >= ARRAY_SIZE(s5k4e6_modes))
		return -EINVAL;
	if (fse->code != s5k4e6_get_format_code(sensor))
		return -EINVAL;

	fse->min_width = s5k4e6_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k4e6_modes[fse->index].height;
	fse->max_height = fse->min_height;
	return 0;
}

static int s5k4e6_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k4e6 *sensor = to_s5k4e6(sd);
	const struct s5k4e6_mode *mode;
	struct v4l2_mbus_framefmt *pad_fmt;

	mode = v4l2_find_nearest_size(s5k4e6_modes, ARRAY_SIZE(s5k4e6_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5k4e6_fill_format(sensor, mode, &fmt->format);
	pad_fmt = v4l2_subdev_get_pad_format(sd, state, 0);
	*pad_fmt = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		int vblank_def = mode->vts_def - mode->height;
		int expo_max = mode->vts_def - S5K4E6_EXPO_MARGIN;
		int hblank = mode->hts - mode->width;

		sensor->mode = mode;
		__v4l2_ctrl_modify_range(sensor->vblank, S5K4E6_VBLANK_MIN,
					 S5K4E6_VTS_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(sensor->vblank, vblank_def);
		__v4l2_ctrl_modify_range(sensor->hblank, hblank, hblank, 1,
					 hblank);
		__v4l2_ctrl_modify_range(sensor->exposure, S5K4E6_EXPO_MIN,
					 expo_max, 1,
					 min(expo_max, S5K4E6_EXPO_DEFAULT));
	}

	return 0;
}

static int s5k4e6_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_get_pad_crop(sd, state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5K4E6_NATIVE_WIDTH;
		sel->r.height = S5K4E6_NATIVE_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int s5k4e6_start_streaming(struct s5k4e6 *sensor)
{
	struct i2c_client *client = sensor->client;
	int ret;

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		return ret;

	ret = s5k4e6_write8(client, S5K4E6_REG_PARAM_HOLD, 0x01);
	ret = ret ?: s5k4e6_write16(client, S5K4E6_REG_SW_RESET, 0x0001);
	if (ret)
		goto err_pm;

	usleep_range(5000, 6000);

	ret = s5k4e6_apply_mode(sensor, sensor->mode);
	ret = ret ?: s5k4e6_write16(client, S5K4E6_REG_PAGE_SELECT, 0x4000);
	if (ret)
		goto err_pm;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrl_handler);
	if (ret)
		goto err_pm;

	ret = s5k4e6_write8(client, S5K4E6_REG_PARAM_HOLD, 0x00);
	usleep_range(4000, 5000);
	ret = ret ?: s5k4e6_write8(client, S5K4E6_REG_MODE_SELECT, 0x01);
	if (ret)
		goto err_pm;

	__v4l2_ctrl_grab(sensor->vflip, true);
	return 0;

err_pm:
	pm_runtime_put(sensor->dev);
	return ret;
}

static void s5k4e6_stop_streaming(struct s5k4e6 *sensor)
{
	struct i2c_client *client = sensor->client;
	u8 frame_cnt = 0;
	int ret;

	ret = s5k4e6_write8(client, S5K4E6_REG_MODE_SELECT, 0x00);
	if (ret) {
		dev_err(sensor->dev, "failed to stop stream: %d\n", ret);
	} else {
		ret = read_poll_timeout(s5k4e6_read, ret,
					ret || frame_cnt == 0xff,
					2000, 500000, false, client,
					S5K4E6_REG_FRAME_COUNT, &frame_cnt, 1);
		if (ret || frame_cnt != 0xff)
			dev_warn(sensor->dev,
				 "stream-off poll timed out (cnt=0x%02x)\n",
				 frame_cnt);
	}

	__v4l2_ctrl_grab(sensor->vflip, false);
	pm_runtime_put(sensor->dev);
}

static int s5k4e6_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct s5k4e6 *sensor = to_s5k4e6(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	if (sensor->streaming == !!enable)
		goto unlock;

	if (enable)
		ret = s5k4e6_start_streaming(sensor);
	else
		s5k4e6_stop_streaming(sensor);

	if (!ret)
		sensor->streaming = !!enable;

unlock:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static const struct v4l2_subdev_core_ops s5k4e6_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops s5k4e6_video_ops = {
	.s_stream = s5k4e6_s_stream,
};

static const struct v4l2_subdev_pad_ops s5k4e6_pad_ops = {
	.init_cfg = s5k4e6_init_cfg,
	.enum_mbus_code = s5k4e6_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k4e6_set_fmt,
	.get_selection = s5k4e6_get_selection,
	.enum_frame_size = s5k4e6_enum_frame_size,
};

static const struct v4l2_subdev_ops s5k4e6_subdev_ops = {
	.core = &s5k4e6_core_ops,
	.video = &s5k4e6_video_ops,
	.pad = &s5k4e6_pad_ops,
};

static int s5k4e6_init_controls(struct s5k4e6 *sensor)
{
	struct v4l2_ctrl_handler *hdl = &sensor->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	const struct s5k4e6_mode *mode = sensor->mode;
	int hblank = mode->hts - mode->width;
	int vblank = mode->vts_def - mode->height;
	int expo_max = mode->vts_def - S5K4E6_EXPO_MARGIN;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 10);
	if (ret)
		return ret;

	sensor->pixel_rate = v4l2_ctrl_new_std(hdl, &s5k4e6_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       S5K4E6_PIXEL_RATE_HZ,
					       S5K4E6_PIXEL_RATE_HZ, 1,
					       S5K4E6_PIXEL_RATE_HZ);

	sensor->link_freq =
		v4l2_ctrl_new_int_menu(hdl, &s5k4e6_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(s5k4e6_link_freq_menu) - 1,
				       0, s5k4e6_link_freq_menu);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->vblank = v4l2_ctrl_new_std(hdl, &s5k4e6_ctrl_ops,
					   V4L2_CID_VBLANK,
					   S5K4E6_VBLANK_MIN,
					   S5K4E6_VTS_MAX - mode->height,
					   1, vblank);

	sensor->hblank = v4l2_ctrl_new_std(hdl, &s5k4e6_ctrl_ops,
					   V4L2_CID_HBLANK,
					   hblank, hblank, 1, hblank);
	if (sensor->hblank)
		sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->exposure = v4l2_ctrl_new_std(hdl, &s5k4e6_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K4E6_EXPO_MIN, expo_max, 1,
					     min(expo_max, S5K4E6_EXPO_DEFAULT));
	if (sensor->exposure)
		sensor->exposure->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	sensor->again = v4l2_ctrl_new_std(hdl, &s5k4e6_ctrl_ops,
					  V4L2_CID_ANALOGUE_GAIN,
					  S5K4E6_AGAIN_MIN, S5K4E6_AGAIN_MAX,
					  1, S5K4E6_AGAIN_DEFAULT);
	if (sensor->again)
		sensor->again->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	sensor->dgain = v4l2_ctrl_new_std(hdl, &s5k4e6_ctrl_ops,
					  V4L2_CID_DIGITAL_GAIN,
					  S5K4E6_DGAIN_MIN, S5K4E6_DGAIN_MAX,
					  1, S5K4E6_DGAIN_DEFAULT);
	if (sensor->dgain)
		sensor->dgain->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	sensor->vflip = v4l2_ctrl_new_std(hdl, &s5k4e6_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (sensor->vflip)
		sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret)
		goto err_free;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &s5k4e6_ctrl_ops, &props);
	if (ret)
		goto err_free;

	sensor->sd.ctrl_handler = hdl;
	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int s5k4e6_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *ep;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!ep)
		return dev_err_probe(dev, -EINVAL, "missing endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &ep_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "expected 2 CSI-2 data lanes, got %u\n",
			ep_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
	} else if (ep_cfg.nr_of_link_frequencies &&
		   ep_cfg.link_frequencies[0] != S5K4E6_LINK_FREQ_HZ) {
		dev_err(dev, "unsupported link frequency %llu\n",
			ep_cfg.link_frequencies[0]);
		ret = -EINVAL;
	}

	v4l2_fwnode_endpoint_free(&ep_cfg);
	return ret;
}

static int s5k4e6_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k4e6 *sensor;
	int i, ret;

	ret = s5k4e6_check_hwcfg(dev);
	if (ret)
		return ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = dev;
	sensor->client = client;
	sensor->mode = &s5k4e6_modes[0];

	v4l2_i2c_subdev_init(&sensor->sd, client, &s5k4e6_subdev_ops);
	dev_set_drvdata(dev, &sensor->sd);

	for (i = 0; i < S5K4E6_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = s5k4e6_supply_names[i];
	ret = devm_regulator_bulk_get(dev, S5K4E6_NUM_SUPPLIES,
				      sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	sensor->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(sensor->mclk))
		return dev_err_probe(dev, PTR_ERR(sensor->mclk),
				     "failed to get mclk\n");

	sensor->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(sensor->pinctrl))
		return dev_err_probe(dev, PTR_ERR(sensor->pinctrl),
				     "failed to get pinctrl\n");
	sensor->pins_default = pinctrl_lookup_state(sensor->pinctrl, "default");
	if (IS_ERR(sensor->pins_default))
		sensor->pins_default = NULL;
	sensor->pins_active = pinctrl_lookup_state(sensor->pinctrl, "active");
	if (IS_ERR(sensor->pins_active))
		sensor->pins_active = NULL;

	sensor->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(dev, PTR_ERR(sensor->reset),
				     "failed to get reset gpio\n");

	sensor->standby = devm_gpiod_get(dev, "standby", GPIOD_OUT_LOW);
	if (IS_ERR(sensor->standby))
		return dev_err_probe(dev, PTR_ERR(sensor->standby),
				     "failed to get standby gpio\n");

	ret = s5k4e6_power_on(dev);
	if (ret)
		return ret;

	ret = s5k4e6_identify(sensor);
	if (ret)
		goto err_power;

	ret = s5k4e6_init_controls(sensor);
	if (ret)
		goto err_power;

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto err_ctrls;

	sensor->sd.state_lock = sensor->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto err_pm;

	pm_runtime_idle(dev);
	return 0;

err_pm:
	pm_runtime_disable(dev);
	v4l2_subdev_cleanup(&sensor->sd);
err_entity:
	media_entity_cleanup(&sensor->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
err_power:
	s5k4e6_power_off(dev);
	return ret;
}

static void s5k4e6_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4e6 *sensor = to_s5k4e6(sd);

	v4l2_async_unregister_subdev(sd);
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		s5k4e6_power_off(sensor->dev);
	pm_runtime_set_suspended(sensor->dev);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
}

static const struct dev_pm_ops s5k4e6_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k4e6_power_off, s5k4e6_power_on, NULL)
};

static const struct of_device_id s5k4e6_of_match[] = {
	{ .compatible = "samsung,s5k4e6" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k4e6_of_match);

static struct i2c_driver s5k4e6_driver = {
	.driver = {
		.name = S5K4E6_DRV_NAME,
		.of_match_table = s5k4e6_of_match,
		.pm = &s5k4e6_pm_ops,
	},
	.probe = s5k4e6_probe,
	.remove = s5k4e6_remove,
};

module_i2c_driver(s5k4e6_driver);

MODULE_DESCRIPTION("Samsung S5K4E6 MIPI CSI-2 image sensor driver");
MODULE_LICENSE("GPL");

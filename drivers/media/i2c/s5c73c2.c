// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S5C73C2 ("73C2") camera companion of the Galaxy S6 rear module.
 *
 * The rear sensor (Sony IMX240, or Samsung S5K2P2 on some units) sends its
 * MIPI stream through this chip, which applies the module calibration and
 * forwards the stream to CSIS0. It has no bypass: nothing reaches the SoC
 * until its firmware runs, and the firmware is loaded by the host on every
 * power-up.
 *
 * The chip answers on the rear camera I2C bus (ID 0x73c2 at register 0,
 * 16-bit big-endian addresses and data), and takes register writes and its
 * firmware over SPI:
 *   write: 0x02, addr[15:8], addr[7:0], data[15:8], data[7:0]
 *   burst: 0x02, 0x6f, 0x12, bytes...   (0x6f12 is the RAM data port)
 * Reads here go over I2C. The SPI pins are bit-banged, which moves the
 * 200 KB firmware in about 5 s.
 *
 * The node owns the whole rear module power sequence for now: probe powers
 * it in the stock order, identifies the sensor, uploads the firmware for it,
 * has the chip compute the CRC32 of what it received and compares it with
 * the file, starts the chip's ARM, applies the master and mode setfiles and
 * powers the module off again. Streaming and the V4L2 side come with the
 * capture path.
 */

#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <asm/unaligned.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define S5C73C2_CHIP_ID			0x73c2

#define S5C73C2_REG_ID			0x0000
#define S5C73C2_REG_VERSION		0x0002
#define S5C73C2_REG_DOWNLOAD		0x0008
#define S5C73C2_REG_SFLASH_CMD		0x0014
#define S5C73C2_REG_SFLASH_SRAM_ADDR	0x001c
#define S5C73C2_REG_SFLASH_SIZE		0x0020
#define S5C73C2_REG_SFLASH_CRC32	0x002c
#define S5C73C2_REG_M2M_CAL		0x0044
#define S5C73C2_REG_MODE		0x0042
#define S5C73C2_REG_FRAME_STATE		0x0050
#define S5C73C2_REG_WDR_AE_CTRL0	0x0b78
#define S5C73C2_REG_WDR_AE_CTRL1	0x0b7c
#define S5C73C2_REG_WDR_MODE		0x0b9c
#define S5C73C2_REG_PAF_DISABLE		0x0ba8
#define S5C73C2_REG_CAL_BYPASS		0x1b34
#define S5C73C2_REG_ARM_GO		0x6014
#define S5C73C2_REG_ARM_REMAP		0x6048
#define S5C73C2_REG_RAM_ADDR_HI		0x6428
#define S5C73C2_REG_RAM_ADDR_LO		0x642a
#define S5C73C2_REG_STREAM		0x6800
#define S5C73C2_REG_MODE_APPLY		0x6802
#define S5C73C2_REG_HOST_INTRP		0x6806
#define S5C73C2_REG_RAM_DATA		0x6f12

#define IMX240_REG_FRAME_COUNT		0x0005
#define IMX240_REG_MODE_SELECT		0x0100
#define IMX240_REG_GROUP_HOLD		0x0104
#define IMX240_REG_COARSE_INT_TIME	0x0202
#define IMX240_REG_ANA_GAIN_GLOBAL	0x0204
#define IMX240_REG_DIG_GAIN_GR		0x020e
#define IMX240_REG_FRAME_LENGTH_LINES	0x0340

#define S5C73C2_NATIVE_WIDTH		5328
#define S5C73C2_NATIVE_HEIGHT		3000
#define S5C73C2_VTS_MAX			65535
#define S5C73C2_VBLANK_MIN		32
#define S5C73C2_EXPO_MIN		4
#define S5C73C2_EXPO_MARGIN		10
#define S5C73C2_EXPO_DEFAULT		3050
#define S5C73C2_AGAIN_MIN		0
#define S5C73C2_AGAIN_MAX		480
#define S5C73C2_AGAIN_DEFAULT		0
#define S5C73C2_DGAIN_MIN		256
#define S5C73C2_DGAIN_MAX		4095
#define S5C73C2_DGAIN_DEFAULT		256
#define S5C73C2_PIXEL_RATE_MIN		560000000ULL
#define S5C73C2_PIXEL_RATE_MAX		584000000ULL

#define S5K2P2_REG_REV_ID		0x0002
#define S5K2P2_REG_B0E_CTRL		0x0b0e
#define S5K2P2_REG_SW_RESET		0x6010
#define S5K2P2_REG_PAGE_SELECT		0x6028
#define S5K2P2_REG_PTR_SELECT		0x602a
#define S5K2P2_REG_RAM_DATA		0x6f12
#define S5K2P2_REG_6004			0x6004
#define S5K2P2_REG_6214			0x6214
#define S5K2P2_REG_6218			0x6218
#define S5K2P2_REG_6226			0x6226
#define S5K2P2_REG_3050			0x3050
#define S5K2P2_REG_3068			0x3068
#define S5K2P2_REG_306A			0x306a
#define S5K2P2_REG_306C			0x306c
#define S5K2P2_REG_70B6			0x70b6
#define S5K2P2_REG_70D4			0x70d4
#define S5K2P2_REV_EVT3			0xc7
#define S5K2P2_REV_EVT4			0xf0
#define S5K2P2_REV_EVT5			0xe2
#define S5K2P2_EXPO_MIN			3
#define S5K2P2_EXPO_MARGIN		4
#define S5K2P2_AGAIN_MIN		32
#define S5K2P2_AGAIN_MAX		512
#define S5K2P2_AGAIN_DEFAULT		32

#define S5C73C2_VERSION_EVT1		0x00b0
#define S5C73C2_FW_ADDR			0x00080000
#define S5C73C2_SFLASH_CMD_CRC32	0x000c
#define S5C73C2_SIGNATURE_CLEAR		0xbeef

#define S5C73C2_SPI_WRITE		0x02
/* One SPI packet: the command, the data port address, 253 data bytes. */
#define S5C73C2_BURST_LEN		256
#define S5C73C2_BURST_HDR		3
/* The firmware ends with its version string; the last 4 bytes are a CRC. */
#define S5C73C2_FW_TAIL			16
#define S5C73C2_FW_CRC_LEN		4
/* A setfile is (addr, value) pairs followed by a 16-byte version. */
#define S5C73C2_SETFILE_TAIL		16

struct s5c73c2_supply {
	const char *name;
	int microvolt;		/* 0: keep the rail's fixed level */
	unsigned int delay_us;	/* after enabling */
};

/* Stock power-on order of the IMX240 module; IMX240 levels. */
static const struct s5c73c2_supply s5c73c2_supplies[] = {
	{ "vddaf",	2800000, 2000 },
	{ "vdda",	2800000, 0 },
	{ "ois-vdd",	0, 0 },
	{ "ois-vm",	0, 2500 },
	{ "vddio",	0, 0 },
	{ "vddd",	1050000, 0 },
	{ "comp-ret",	1000000, 0 },
	{ "comp-vddio",	0, 0 },
	{ "comp-vdda",	0, 0 },
	{ "comp-vddd",	0, 0 },
	{ "comp-noret",	0, 0 },
	{ "comp-core",	800000, 0 },
};

struct s5c73c2_sensor {
	const char *name;
	u16 addr;
	u16 id_reg;
	u16 id;
};

static const struct s5c73c2_sensor s5c73c2_sensors[] = {
	{ "imx240", 0x1a, 0x0016, 0x0240 },
	/* S5K2P2: its ID is not known here, so any answer at its address. */
	{ "2p2", 0x2d, 0x0000, 0 },
};

struct s5c73c2_sensor_reg {
	u16 reg;
	u8 val;
};

struct s5c73c2_reg {
	u16 reg;
	u16 val;
	u8 width;
};

#include "s5c73c2_2p2_regs.h"

struct s5c73c2_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts_def;
	u32 expo_def;
	u16 c73c2_mode;
	bool paf_en;
	u32 link_freq_index;
	u64 pixel_rate;
	const struct s5c73c2_sensor_reg *regs;
	unsigned int num_regs;
};

struct s5c73c2_sensor_info {
	const struct s5c73c2_mode *modes;
	unsigned int num_modes;
	u32 default_mbus_code;
	s32 expo_min;
	s32 expo_margin;
	s32 again_min;
	s32 again_max;
	s32 again_default;
	bool is_2p2;
};

struct s5c73c2 {
	struct device *dev;
	struct i2c_client *client;
	struct spi_device *spi;
	struct regulator *supplies[ARRAY_SIZE(s5c73c2_supplies)];
	struct clk *mclk;
	struct clk *sensor_mclk;
	struct gpio_desc *reset;
	struct gpio_desc *sensor_reset;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_off;
	struct pinctrl_state *pins_on;
	u8 *tx;
	const struct s5c73c2_sensor *sensor;
	const struct s5c73c2_sensor_info *info;
	const struct s5c73c2_mode *mode;
	u8 sensor_rev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *dgain;
	bool streaming;
};

static inline struct s5c73c2 *to_s5c73c2(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5c73c2, sd);
}

static const struct s5c73c2_sensor_reg imx240_common_regs[] = {
	{ 0x0101, 0x00 }, { 0x0136, 0x18 }, { 0x0137, 0x00 }, { 0x4128, 0x00 },
	{ 0x4129, 0x0f }, { 0x412e, 0x00 }, { 0x412f, 0x0e }, { 0x45d0, 0x01 },
	{ 0x4612, 0x30 }, { 0x4619, 0x38 }, { 0x461a, 0x7c }, { 0x461b, 0x2a },
	{ 0x463f, 0x18 }, { 0x4653, 0x29 }, { 0x4657, 0x29 }, { 0x465b, 0x28 },
	{ 0x465f, 0x23 }, { 0x4667, 0x22 }, { 0x466b, 0x22 }, { 0x4673, 0x1a },
	{ 0x4904, 0x00 }, { 0x4905, 0xa6 }, { 0x4906, 0x00 }, { 0x4907, 0x8e },
	{ 0x4908, 0x01 }, { 0x4909, 0x18 }, { 0x490a, 0x00 }, { 0x490b, 0xd7 },
	{ 0x4a72, 0x00 }, { 0x4a73, 0x8c }, { 0x4a76, 0x01 }, { 0x4a77, 0x07 },
	{ 0x4a7a, 0x01 }, { 0x4a7b, 0x07 }, { 0x4a5c, 0x01 }, { 0x4a5d, 0x0a },
	{ 0x4a62, 0x01 }, { 0x4a63, 0x0a }, { 0x4a84, 0x01 }, { 0x4a85, 0x0a },
	{ 0xa900, 0x7f }, { 0xa908, 0x00 }, { 0xa909, 0x38 }, { 0xae1c, 0x05 },
	{ 0xae1d, 0x30 }, { 0xae1e, 0x04 }, { 0xae1f, 0xa4 }, { 0xae20, 0x04 },
	{ 0xae21, 0xa4 }, { 0xae3a, 0x05 }, { 0xae3b, 0x30 }, { 0x4600, 0x08 },
	{ 0x5041, 0x00 }, { 0x4550, 0x02 }, { 0x622b, 0x01 }, { 0x6227, 0x11 },
	{ 0x4647, 0x49 }, { 0x4645, 0xca }, { 0x4713, 0x3b }, { 0x4711, 0xa2 },
	{ 0x471b, 0x31 }, { 0x4719, 0x87 }, { 0x4723, 0x2a }, { 0x4721, 0x74 },
	{ 0x472b, 0x25 }, { 0x4729, 0x66 }, { 0x4733, 0x21 }, { 0x4731, 0x5b },
	{ 0x473b, 0x1e }, { 0x4739, 0x52 }, { 0x4616, 0x03 }, { 0x467e, 0x03 },
};

static const struct s5c73c2_sensor_reg imx240_5328x3000_regs[] = {
	{ 0x0114, 0x03 }, { 0x0220, 0x23 }, { 0x0221, 0x11 }, { 0x0222, 0x01 },
	{ 0x0340, 0x0b }, { 0x0341, 0xf4 }, { 0x0342, 0x18 }, { 0x0343, 0x00 },
	{ 0x0344, 0x00 }, { 0x0345, 0x00 }, { 0x0346, 0x00 }, { 0x0347, 0x00 },
	{ 0x0348, 0x14 }, { 0x0349, 0xdf }, { 0x034a, 0x0b }, { 0x034b, 0xb7 },
	{ 0x0381, 0x01 }, { 0x0383, 0x01 }, { 0x0385, 0x01 }, { 0x0387, 0x01 },
	{ 0x0900, 0x00 }, { 0x0901, 0x11 }, { 0x0902, 0x00 }, { 0x3000, 0x04 },
	{ 0x3008, 0x00 }, { 0x3009, 0x66 }, { 0x3029, 0x00 }, { 0x305c, 0x11 },
	{ 0x6283, 0x04 }, { 0x403c, 0x08 }, { 0x622d, 0x02 }, { 0x622e, 0x01 },
	{ 0x620e, 0x0e }, { 0x0112, 0x0a }, { 0x0113, 0x0a }, { 0x034c, 0x15 },
	{ 0x034d, 0x84 }, { 0x034e, 0x0b }, { 0x034f, 0xb8 }, { 0x0401, 0x00 },
	{ 0x0404, 0x00 }, { 0x0405, 0x10 }, { 0x0408, 0x00 }, { 0x0409, 0x00 },
	{ 0x040a, 0x00 }, { 0x040b, 0x00 }, { 0x040c, 0x14 }, { 0x040d, 0xe0 },
	{ 0x040e, 0x0b }, { 0x040f, 0xb8 }, { 0x0301, 0x04 }, { 0x0303, 0x02 },
	{ 0x0305, 0x0f }, { 0x0306, 0x02 }, { 0x0307, 0xc4 }, { 0x0309, 0x0a },
	{ 0x030b, 0x01 }, { 0x030d, 0x0f }, { 0x030e, 0x03 }, { 0x030f, 0x4d },
	{ 0x0310, 0x01 }, { 0x0820, 0x15 }, { 0x0821, 0x20 }, { 0x0822, 0x00 },
	{ 0x0823, 0x00 }, { 0x3034, 0x00 }, { 0x3035, 0xc7 }, { 0x0b06, 0x00 },
	{ 0x3001, 0x07 }, { 0x3014, 0x01 }, { 0x3015, 0x01 }, { 0x3020, 0x01 },
	{ 0x3021, 0x00 }, { 0x3024, 0x00 }, { 0x3025, 0x00 }, { 0x30c5, 0x00 },
	{ 0x30b4, 0x01 }, { 0x0202, 0x0b }, { 0x0203, 0xea }, { 0x0224, 0x05 },
	{ 0x0225, 0xf6 }, { 0x0204, 0x00 }, { 0x0205, 0x00 }, { 0x020e, 0x01 },
	{ 0x020f, 0x00 }, { 0x0210, 0x01 }, { 0x0211, 0x00 }, { 0x0212, 0x01 },
	{ 0x0213, 0x00 }, { 0x0214, 0x01 }, { 0x0215, 0x00 }, { 0x0216, 0x00 },
	{ 0x0217, 0x00 },
};

static const struct s5c73c2_sensor_reg imx240_2664x1500_regs[] = {
	{ 0x0114, 0x03 }, { 0x0220, 0x00 }, { 0x0221, 0x11 }, { 0x0222, 0x01 },
	{ 0x0340, 0x06 }, { 0x0341, 0x30 }, { 0x0342, 0x18 }, { 0x0343, 0x00 },
	{ 0x0344, 0x00 }, { 0x0345, 0x00 }, { 0x0346, 0x00 }, { 0x0347, 0x00 },
	{ 0x0348, 0x14 }, { 0x0349, 0xdf }, { 0x034a, 0x0b }, { 0x034b, 0xb7 },
	{ 0x0381, 0x01 }, { 0x0383, 0x01 }, { 0x0385, 0x01 }, { 0x0387, 0x01 },
	{ 0x0900, 0x01 }, { 0x0901, 0x22 }, { 0x0902, 0x00 }, { 0x3000, 0x26 },
	{ 0x3008, 0x00 }, { 0x3009, 0x66 }, { 0x3029, 0x00 }, { 0x305c, 0x11 },
	{ 0x6283, 0x04 }, { 0x403c, 0x08 }, { 0x622d, 0x02 }, { 0x622e, 0x01 },
	{ 0x620e, 0x0c }, { 0x0112, 0x0a }, { 0x0113, 0x0a }, { 0x034c, 0x0a },
	{ 0x034d, 0x70 }, { 0x034e, 0x05 }, { 0x034f, 0xdc }, { 0x0401, 0x00 },
	{ 0x0404, 0x00 }, { 0x0405, 0x10 }, { 0x0408, 0x00 }, { 0x0409, 0x00 },
	{ 0x040a, 0x00 }, { 0x040b, 0x00 }, { 0x040c, 0x0a }, { 0x040d, 0x70 },
	{ 0x040e, 0x05 }, { 0x040f, 0xdc }, { 0x0301, 0x04 }, { 0x0303, 0x02 },
	{ 0x0305, 0x0f }, { 0x0306, 0x02 }, { 0x0307, 0xda }, { 0x0309, 0x0a },
	{ 0x030b, 0x01 }, { 0x030d, 0x0f }, { 0x030e, 0x02 }, { 0x030f, 0xdb },
	{ 0x0310, 0x00 }, { 0x0820, 0x12 }, { 0x0821, 0x40 }, { 0x0822, 0x00 },
	{ 0x0823, 0x00 }, { 0x3034, 0x13 }, { 0x3035, 0x3f }, { 0x0b06, 0x00 },
	{ 0x3001, 0x07 }, { 0x3014, 0x01 }, { 0x3015, 0x01 }, { 0x3020, 0x00 },
	{ 0x3021, 0x00 }, { 0x3024, 0x00 }, { 0x3025, 0x00 }, { 0x30c5, 0x00 },
	{ 0x30b4, 0x00 }, { 0x0202, 0x06 }, { 0x0203, 0x28 }, { 0x0224, 0x01 },
	{ 0x0225, 0xf4 }, { 0x0204, 0x00 }, { 0x0205, 0x00 }, { 0x020e, 0x01 },
	{ 0x020f, 0x00 }, { 0x0210, 0x01 }, { 0x0211, 0x00 }, { 0x0212, 0x01 },
	{ 0x0213, 0x00 }, { 0x0214, 0x01 }, { 0x0215, 0x00 }, { 0x0216, 0x00 },
	{ 0x0217, 0x00 },
};

static const s64 s5c73c2_link_freq_menu[] = {
	676000000ULL,
	584800000ULL,
};

static const u32 s5c73c2_mbus_codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
};

static const struct s5c73c2_mode s5c73c2_modes[] = {
	{
		.width = 5328,
		.height = 3000,
		.hts = 6144,
		.vts_def = 3060,
		.expo_def = 3050,
		.c73c2_mode = 0,
		.paf_en = true,
		.link_freq_index = 0,
		.pixel_rate = 566400000ULL,
		.regs = imx240_5328x3000_regs,
		.num_regs = ARRAY_SIZE(imx240_5328x3000_regs),
	},
	{
		.width = 2664,
		.height = 1500,
		.hts = 6144,
		.vts_def = 1584,
		.expo_def = 1576,
		.c73c2_mode = 6,
		.paf_en = false,
		.link_freq_index = 1,
		.pixel_rate = 584000000ULL,
		.regs = imx240_2664x1500_regs,
		.num_regs = ARRAY_SIZE(imx240_2664x1500_regs),
	},
};

static const struct s5c73c2_mode s5k2p2_modes[] = {
	{
		.width = 5328,
		.height = 3000,
		.hts = 5880,
		.vts_def = 3168,
		.expo_def = 3050,
		.c73c2_mode = 0,
		.paf_en = true,
		.link_freq_index = 0,
		.pixel_rate = 560000000ULL,
	},
	{
		.width = 2664,
		.height = 1500,
		.hts = 5880,
		.vts_def = 1586,
		.expo_def = 1570,
		.c73c2_mode = 6,
		.paf_en = false,
		.link_freq_index = 1,
		.pixel_rate = 560000000ULL,
	},
};

static const struct s5c73c2_sensor_info s5c73c2_imx240_info = {
	.modes = s5c73c2_modes,
	.num_modes = ARRAY_SIZE(s5c73c2_modes),
	.default_mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
	.expo_min = S5C73C2_EXPO_MIN,
	.expo_margin = S5C73C2_EXPO_MARGIN,
	.again_min = S5C73C2_AGAIN_MIN,
	.again_max = S5C73C2_AGAIN_MAX,
	.again_default = S5C73C2_AGAIN_DEFAULT,
	.is_2p2 = false,
};

static const struct s5c73c2_sensor_info s5c73c2_2p2_info = {
	.modes = s5k2p2_modes,
	.num_modes = ARRAY_SIZE(s5k2p2_modes),
	.default_mbus_code = MEDIA_BUS_FMT_SGRBG10_1X10,
	.expo_min = S5K2P2_EXPO_MIN,
	.expo_margin = S5K2P2_EXPO_MARGIN,
	.again_min = S5K2P2_AGAIN_MIN,
	.again_max = S5K2P2_AGAIN_MAX,
	.again_default = S5K2P2_AGAIN_DEFAULT,
	.is_2p2 = true,
};

static const struct v4l2_subdev_ops s5c73c2_subdev_ops;
static int s5c73c2_check_hwcfg(struct device *dev);
static void s5c73c2_detect_sensor(struct s5c73c2 *c2);
static int s5c73c2_init_controls(struct s5c73c2 *c2);

static int s5c73c2_read(struct s5c73c2 *c2, u16 addr, u16 reg, u16 *val)
{
	u8 wbuf[2], rbuf[2];
	struct i2c_msg msgs[2] = {
		{ .addr = addr, .len = 2, .buf = wbuf },
		{ .addr = addr, .flags = I2C_M_RD, .len = 2, .buf = rbuf },
	};
	int ret;

	put_unaligned_be16(reg, wbuf);
	ret = i2c_transfer(c2->client->adapter, msgs, 2);
	if (ret != 2)
		return ret < 0 ? ret : -EIO;
	*val = get_unaligned_be16(rbuf);
	return 0;
}

static int s5c73c2_write(struct s5c73c2 *c2, u16 reg, u16 val)
{
	c2->tx[0] = S5C73C2_SPI_WRITE;
	put_unaligned_be16(reg, &c2->tx[1]);
	put_unaligned_be16(val, &c2->tx[3]);
	return spi_write(c2->spi, c2->tx, 5);
}

/* Write a 32-bit value as two 16-bit registers, high half first. */
static int s5c73c2_write32(struct s5c73c2 *c2, u16 reg, u32 val)
{
	int ret = s5c73c2_write(c2, reg, val >> 16);

	return ret ?: s5c73c2_write(c2, reg + 2, val & 0xffff);
}

static int s5c73c2_burst(struct s5c73c2 *c2, const u8 *data, size_t size)
{
	const size_t chunk = S5C73C2_BURST_LEN - S5C73C2_BURST_HDR;
	size_t off, len;
	int ret;

	c2->tx[0] = S5C73C2_SPI_WRITE;
	put_unaligned_be16(S5C73C2_REG_RAM_DATA, &c2->tx[1]);
	for (off = 0; off < size; off += len) {
		len = min(chunk, size - off);
		memcpy(&c2->tx[S5C73C2_BURST_HDR], data + off, len);
		ret = spi_write(c2->spi, c2->tx, S5C73C2_BURST_HDR + len);
		if (ret)
			return ret;
	}
	return 0;
}

static void s5c73c2_power_off(struct s5c73c2 *c2, int nsup)
{
	gpiod_set_value_cansleep(c2->sensor_reset, 1);
	gpiod_set_value_cansleep(c2->reset, 1);
	usleep_range(10, 20);
	pinctrl_select_state(c2->pinctrl, c2->pins_off);
	clk_disable_unprepare(c2->sensor_mclk);
	clk_disable_unprepare(c2->mclk);
	while (nsup--)
		regulator_disable(c2->supplies[nsup]);
}

static int s5c73c2_power_on(struct s5c73c2 *c2)
{
	int i, ret;

	gpiod_set_value_cansleep(c2->sensor_reset, 1);
	gpiod_set_value_cansleep(c2->reset, 1);

	for (i = 0; i < ARRAY_SIZE(s5c73c2_supplies); i++) {
		const struct s5c73c2_supply *s = &s5c73c2_supplies[i];

		if (s->microvolt) {
			ret = regulator_set_voltage(c2->supplies[i],
						    s->microvolt, s->microvolt);
			if (ret)
				goto err_supply;
		}
		ret = regulator_enable(c2->supplies[i]);
		if (ret)
			goto err_supply;
		if (s->delay_us)
			usleep_range(s->delay_us, s->delay_us + 100);
	}
	usleep_range(1000, 1100);

	ret = clk_set_rate(c2->mclk, 24000000);
	ret = ret ?: clk_prepare_enable(c2->mclk);
	if (ret)
		goto err_supply;
	ret = clk_set_rate(c2->sensor_mclk, 24000000);
	ret = ret ?: clk_prepare_enable(c2->sensor_mclk);
	if (ret)
		goto err_mclk;
	ret = pinctrl_select_state(c2->pinctrl, c2->pins_on);
	if (ret)
		goto err_sensor_mclk;
	usleep_range(2000, 2100);

	gpiod_set_value_cansleep(c2->reset, 0);
	usleep_range(300, 400);
	gpiod_set_value_cansleep(c2->sensor_reset, 0);
	usleep_range(2000, 2100);
	return 0;

err_sensor_mclk:
	clk_disable_unprepare(c2->sensor_mclk);
err_mclk:
	clk_disable_unprepare(c2->mclk);
err_supply:
	dev_err(c2->dev, "power on at %s: %d\n",
		i < ARRAY_SIZE(s5c73c2_supplies) ? s5c73c2_supplies[i].name :
		"mclk", ret);
	while (i--)
		regulator_disable(c2->supplies[i]);
	return ret;
}

static const struct s5c73c2_sensor *s5c73c2_find_sensor(struct s5c73c2 *c2)
{
	int i;
	u16 id;

	for (i = 0; i < ARRAY_SIZE(s5c73c2_sensors); i++) {
		const struct s5c73c2_sensor *s = &s5c73c2_sensors[i];

		if (s5c73c2_read(c2, s->addr, s->id_reg, &id))
			continue;
		if (!s->id || id == s->id)
			return s;
		dev_warn(c2->dev, "0x%02x: ID 0x%04x, not %s\n", s->addr, id,
			 s->name);
	}
	return NULL;
}

/* Have the chip compute the CRC32 of its RAM and compare it with the file. */
static int s5c73c2_check_crc(struct s5c73c2 *c2, u32 addr, u32 size, u32 want)
{
	u16 cmd, hi, lo;
	int ret, i;
	u32 got;

	ret = s5c73c2_write32(c2, S5C73C2_REG_SFLASH_CRC32, 0);
	ret = ret ?: s5c73c2_write32(c2, S5C73C2_REG_SFLASH_SRAM_ADDR, addr);
	ret = ret ?: s5c73c2_write32(c2, S5C73C2_REG_SFLASH_SIZE, size);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_SFLASH_CMD,
				   S5C73C2_SFLASH_CMD_CRC32);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_HOST_INTRP, 1);
	if (ret)
		return ret;

	for (i = 0; i < 1000; i++) {
		ret = s5c73c2_read(c2, c2->client->addr,
				   S5C73C2_REG_SFLASH_CMD, &cmd);
		if (!ret && !cmd)
			break;
		usleep_range(1000, 1100);
	}
	if (ret || cmd) {
		dev_err(c2->dev, "CRC command not done (cmd 0x%04x, %d)\n",
			cmd, ret);
		return ret ?: -ETIMEDOUT;
	}

	ret = s5c73c2_read(c2, c2->client->addr, S5C73C2_REG_SFLASH_CRC32,
			   &hi);
	ret = ret ?: s5c73c2_read(c2, c2->client->addr,
				  S5C73C2_REG_SFLASH_CRC32 + 2, &lo);
	if (ret)
		return ret;
	/* High half first; equal to the file's trailer read little-endian. */
	got = (u32)hi << 16 | lo;

	if (got != want) {
		dev_err(c2->dev, "CRC32 %08x, file %08x\n", got, want);
		return -EBADMSG;
	}
	return 0;
}

static int s5c73c2_load_fw(struct s5c73c2 *c2, const struct firmware *fw)
{
	size_t len = fw->size - S5C73C2_FW_CRC_LEN;
	u32 want = get_unaligned_le32(fw->data + len);
	ktime_t start;
	int ret;
	u16 val;

	ret = s5c73c2_write(c2, S5C73C2_REG_DOWNLOAD, 1);
	if (ret)
		return ret;
	usleep_range(1000, 1100);
	ret = s5c73c2_write32(c2, S5C73C2_REG_RAM_ADDR_HI, S5C73C2_FW_ADDR);
	if (ret)
		return ret;

	/* Fail now rather than after the upload if SPI writes do not land. */
	ret = s5c73c2_read(c2, c2->client->addr, S5C73C2_REG_RAM_ADDR_HI, &val);
	if (ret)
		return ret;
	if (val != S5C73C2_FW_ADDR >> 16) {
		dev_err(c2->dev, "SPI write not seen over I2C (0x%04x)\n", val);
		return -EIO;
	}

	start = ktime_get();
	ret = s5c73c2_burst(c2, fw->data, fw->size);
	if (ret)
		return ret;
	dev_dbg(c2->dev, "firmware %zu bytes in %lld ms\n", fw->size,
		ktime_ms_delta(ktime_get(), start));

	return s5c73c2_check_crc(c2, S5C73C2_FW_ADDR, len, want);
}

static int s5c73c2_load_setfile(struct s5c73c2 *c2, const char *name)
{
	const struct firmware *fw;
	size_t i, len;
	int ret;

	ret = request_firmware(&fw, name, c2->dev);
	if (ret)
		return ret;
	if (fw->size < S5C73C2_SETFILE_TAIL ||
	    (fw->size - S5C73C2_SETFILE_TAIL) % 4) {
		ret = -EINVAL;
		goto out;
	}
	len = fw->size - S5C73C2_SETFILE_TAIL;
	for (i = 0; i < len && !ret; i += 4)
		ret = s5c73c2_write(c2, get_unaligned_be16(fw->data + i),
				    get_unaligned_be16(fw->data + i + 2));
	dev_dbg(c2->dev, "%s: %zu writes, %.16s\n", name, len / 4,
		fw->data + len);
out:
	release_firmware(fw);
	return ret;
}

static int s5c73c2_boot(struct s5c73c2 *c2, const char *sensor)
{
	const struct firmware *fw;
	char name[48];
	int ret, try;
	u16 val;

	snprintf(name, sizeof(name), "companion_fw_%s_evt1.bin", sensor);
	ret = request_firmware(&fw, name, c2->dev);
	if (ret)
		return ret;
	if (fw->size <= S5C73C2_FW_TAIL) {
		ret = -EINVAL;
		goto out;
	}
	if ((crc32_le(~0, fw->data, fw->size - S5C73C2_FW_CRC_LEN) ^ ~0) !=
	    get_unaligned_le32(fw->data + fw->size - S5C73C2_FW_CRC_LEN)) {
		dev_err(c2->dev, "%s: CRC does not match its contents\n", name);
		ret = -EBADMSG;
		goto out;
	}
	dev_info(c2->dev, "%s: version %.12s\n", name,
		 fw->data + fw->size - S5C73C2_FW_TAIL);

	for (try = 0; try < 2; try++) {
		ret = s5c73c2_load_fw(c2, fw);
		if (ret != -EBADMSG)
			break;
	}
	if (ret)
		goto out;

	ret = s5c73c2_write(c2, S5C73C2_REG_ARM_REMAP, 1);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_ID, S5C73C2_SIGNATURE_CLEAR);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_ARM_GO, 1);
	if (ret)
		goto out;
	usleep_range(1000, 1100);
	/* The firmware replaces the signature with the chip ID once it runs. */
	ret = s5c73c2_read(c2, c2->client->addr, S5C73C2_REG_ID, &val);
	if (ret)
		goto out;
	if (val != S5C73C2_CHIP_ID) {
		dev_err(c2->dev, "firmware did not start (0x%04x)\n", val);
		ret = -EIO;
		goto out;
	}
	/* No calibration from the module's FROM yet. */
	ret = s5c73c2_write(c2, S5C73C2_REG_M2M_CAL, 0);
	if (ret)
		goto out;

	snprintf(name, sizeof(name), "companion_%s_master_setfile.bin", sensor);
	ret = s5c73c2_load_setfile(c2, name);
	if (ret)
		goto out;
	usleep_range(1000, 1100);
	snprintf(name, sizeof(name), "companion_%s_mode_setfile.bin", sensor);
	ret = s5c73c2_load_setfile(c2, name);
out:
	release_firmware(fw);
	return ret;
}

static int s5c73c2_identify_and_boot(struct s5c73c2 *c2)
{
	const struct s5c73c2_sensor *sensor;
	u16 id, ver;
	int ret;

	ret = s5c73c2_read(c2, c2->client->addr, S5C73C2_REG_ID, &id);
	ret = ret ?: s5c73c2_read(c2, c2->client->addr, S5C73C2_REG_VERSION,
				  &ver);
	if (ret)
		return dev_err_probe(c2->dev, ret, "no answer on I2C\n");
	if (id != S5C73C2_CHIP_ID)
		return dev_err_probe(c2->dev, -ENODEV, "ID 0x%04x\n", id);
	if (ver != S5C73C2_VERSION_EVT1)
		return dev_err_probe(c2->dev, -ENODEV,
				     "version 0x%04x, only EVT1 is known\n",
				     ver);

	sensor = s5c73c2_find_sensor(c2);
	if (!sensor)
		return dev_err_probe(c2->dev, -ENODEV, "no rear sensor\n");
	dev_info(c2->dev, "73C2 EVT1, sensor %s\n", sensor->name);

	ret = s5c73c2_boot(c2, sensor->name);
	if (ret)
		return dev_err_probe(c2->dev, ret, "firmware start failed\n");
	dev_info(c2->dev, "firmware running\n");
	return 0;
}

static void s5c73c2_put_spi(void *data)
{
	put_device(data);
}

static int s5c73c2_get_spi(struct s5c73c2 *c2)
{
	struct device_node *np;
	struct device *dev;
	int ret;

	np = of_parse_phandle(c2->dev->of_node, "samsung,spi-device", 0);
	if (!np)
		return dev_err_probe(c2->dev, -EINVAL, "no samsung,spi-device\n");
	dev = bus_find_device_by_of_node(&spi_bus_type, np);
	of_node_put(np);
	if (!dev)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(c2->dev, s5c73c2_put_spi, dev);
	if (ret)
		return ret;
	c2->spi = to_spi_device(dev);
	return 0;
}

static int s5c73c2_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5c73c2 *c2;
	int i, ret;

	c2 = devm_kzalloc(dev, sizeof(*c2), GFP_KERNEL);
	if (!c2)
		return -ENOMEM;
	c2->dev = dev;
	c2->client = client;

	c2->tx = devm_kmalloc(dev, S5C73C2_BURST_LEN, GFP_KERNEL);
	if (!c2->tx)
		return -ENOMEM;

	ret = s5c73c2_get_spi(c2);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(s5c73c2_supplies); i++) {
		c2->supplies[i] = devm_regulator_get(dev,
						     s5c73c2_supplies[i].name);
		if (IS_ERR(c2->supplies[i]))
			return dev_err_probe(dev, PTR_ERR(c2->supplies[i]),
					     "%s supply\n",
					     s5c73c2_supplies[i].name);
	}

	c2->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(c2->mclk))
		return dev_err_probe(dev, PTR_ERR(c2->mclk), "mclk\n");
	c2->sensor_mclk = devm_clk_get(dev, "sensor-mclk");
	if (IS_ERR(c2->sensor_mclk))
		return dev_err_probe(dev, PTR_ERR(c2->sensor_mclk),
				     "sensor-mclk\n");

	c2->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(c2->reset))
		return dev_err_probe(dev, PTR_ERR(c2->reset), "reset\n");
	c2->sensor_reset = devm_gpiod_get(dev, "sensor-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(c2->sensor_reset))
		return dev_err_probe(dev, PTR_ERR(c2->sensor_reset),
				     "sensor-reset\n");

	c2->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(c2->pinctrl))
		return dev_err_probe(dev, PTR_ERR(c2->pinctrl), "pinctrl\n");
	c2->pins_off = pinctrl_lookup_state(c2->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(c2->pins_off))
		return dev_err_probe(dev, PTR_ERR(c2->pins_off), "default pins\n");
	c2->pins_on = pinctrl_lookup_state(c2->pinctrl, "active");
	if (IS_ERR(c2->pins_on))
		return dev_err_probe(dev, PTR_ERR(c2->pins_on), "active pins\n");

	ret = s5c73c2_check_hwcfg(c2->dev);
	if (ret)
		return ret;

	c2->info = &s5c73c2_imx240_info;
	c2->mode = &c2->info->modes[0];

	ret = s5c73c2_power_on(c2);
	if (ret)
		return ret;

	ret = s5c73c2_identify_and_boot(c2);
	if (ret)
		goto err_power;

	s5c73c2_detect_sensor(c2);
	c2->mode = &c2->info->modes[0];

	v4l2_i2c_subdev_init(&c2->sd, c2->client, &s5c73c2_subdev_ops);
	dev_set_drvdata(c2->dev, &c2->sd);

	ret = s5c73c2_init_controls(c2);
	if (ret)
		goto err_power;

	c2->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			V4L2_SUBDEV_FL_HAS_EVENTS;
	c2->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	c2->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&c2->sd.entity, 1, &c2->pad);
	if (ret)
		goto err_ctrls;

	c2->sd.state_lock = c2->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&c2->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(c2->dev);
	pm_runtime_get_noresume(c2->dev);
	pm_runtime_enable(c2->dev);
	pm_runtime_set_autosuspend_delay(c2->dev, 3000);
	pm_runtime_use_autosuspend(c2->dev);

	ret = v4l2_async_register_subdev_sensor(&c2->sd);
	if (ret)
		goto err_pm;

	pm_runtime_mark_last_busy(c2->dev);
	pm_runtime_put_autosuspend(c2->dev);
	return 0;

err_pm:
	pm_runtime_dont_use_autosuspend(c2->dev);
	pm_runtime_disable(c2->dev);
	pm_runtime_put_noidle(c2->dev);
	v4l2_subdev_cleanup(&c2->sd);
err_entity:
	media_entity_cleanup(&c2->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&c2->ctrl_handler);
err_power:
	s5c73c2_power_off(c2, ARRAY_SIZE(s5c73c2_supplies));
	return ret;
}

static int s5c73c2_read32(struct s5c73c2 *c2, u16 addr, u16 reg, u32 *val)
{
	u8 wbuf[2], rbuf[4];
	struct i2c_msg msgs[2] = {
		{ .addr = addr, .len = 2, .buf = wbuf },
		{ .addr = addr, .flags = I2C_M_RD, .len = 4, .buf = rbuf },
	};
	int ret;

	put_unaligned_be16(reg, wbuf);
	ret = i2c_transfer(c2->client->adapter, msgs, 2);
	if (ret != 2)
		return ret < 0 ? ret : -EIO;
	*val = get_unaligned_be32(rbuf);
	return 0;
}

static int s5c73c2_sensor_read8(struct s5c73c2 *c2, u16 reg, u8 *val)
{
	u8 wbuf[2];
	struct i2c_msg msgs[2] = {
		{ .addr = c2->sensor->addr, .len = 2, .buf = wbuf },
		{ .addr = c2->sensor->addr, .flags = I2C_M_RD, .len = 1, .buf = val },
	};
	int ret;

	put_unaligned_be16(reg, wbuf);
	ret = i2c_transfer(c2->client->adapter, msgs, 2);
	if (ret != 2)
		return ret < 0 ? ret : -EIO;
	return 0;
}

static int s5c73c2_sensor_write_raw(struct s5c73c2 *c2, const u8 *buf,
				    size_t len)
{
	struct i2c_msg msg = {
		.addr = c2->sensor->addr,
		.len = len,
		.buf = (u8 *)buf,
	};
	int ret;

	ret = i2c_transfer(c2->client->adapter, &msg, 1);
	if (ret != 1)
		return ret < 0 ? ret : -EIO;
	return 0;
}

static int s5c73c2_sensor_write8(struct s5c73c2 *c2, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };

	return s5c73c2_sensor_write_raw(c2, buf, sizeof(buf));
}

static int s5c73c2_sensor_write16(struct s5c73c2 *c2, u16 reg, u16 val)
{
	u8 buf[4] = { reg >> 8, reg & 0xff, val >> 8, val & 0xff };

	return s5c73c2_sensor_write_raw(c2, buf, sizeof(buf));
}

static int s5c73c2_sensor_set_dgain(struct s5c73c2 *c2, u16 gain)
{
	u8 buf[10] = {
		IMX240_REG_DIG_GAIN_GR >> 8,
		IMX240_REG_DIG_GAIN_GR & 0xff,
		gain >> 8, gain & 0xff,
		gain >> 8, gain & 0xff,
		gain >> 8, gain & 0xff,
		gain >> 8, gain & 0xff,
	};

	return s5c73c2_sensor_write_raw(c2, buf, sizeof(buf));
}

static int s5c73c2_sensor_write_table(struct s5c73c2 *c2,
				      const struct s5c73c2_sensor_reg *regs,
				      unsigned int count)
{
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = s5c73c2_sensor_write8(c2, regs[i].reg, regs[i].val);
		if (ret) {
			dev_err(c2->dev,
				"sensor reg 0x%04x = 0x%02x failed: %d\n",
				regs[i].reg, regs[i].val, ret);
			return ret;
		}
	}

	return 0;
}

static int s5c73c2_sensor_write_regs(struct s5c73c2 *c2,
				     const struct s5c73c2_reg *regs,
				     unsigned int count)
{
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		if (regs[i].width == 1)
			ret = s5c73c2_sensor_write8(c2, regs[i].reg,
						    regs[i].val);
		else
			ret = s5c73c2_sensor_write16(c2, regs[i].reg,
						     regs[i].val);
		if (ret) {
			dev_err(c2->dev,
				"sensor w%u reg 0x%04x = 0x%04x failed: %d\n",
				regs[i].width, regs[i].reg, regs[i].val, ret);
			return ret;
		}
	}

	return 0;
}

enum s5c73c2_step_kind {
	STEP_W8,
	STEP_W16,
	STEP_REGS,
	STEP_BURST,
};

struct s5c73c2_step {
	enum s5c73c2_step_kind kind;
	u16 reg;
	u16 val;
	const void *ptr;
	unsigned int len;
};

#define S2_W8(r, v)		{ STEP_W8, (r), (v), NULL, 0 }
#define S2_W16(r, v)		{ STEP_W16, (r), (v), NULL, 0 }
#define S2_REGS(arr)		{ STEP_REGS, 0, 0, (arr), ARRAY_SIZE(arr) }
#define S2_BURST(arr)		{ STEP_BURST, 0, 0, (arr), sizeof(arr) }

static int s5c73c2_run_steps(struct s5c73c2 *c2,
			     const struct s5c73c2_step *steps,
			     unsigned int count)
{
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		switch (steps[i].kind) {
		case STEP_W8:
			ret = s5c73c2_sensor_write8(c2, steps[i].reg,
						    steps[i].val);
			break;
		case STEP_W16:
			ret = s5c73c2_sensor_write16(c2, steps[i].reg,
						     steps[i].val);
			break;
		case STEP_REGS:
			ret = s5c73c2_sensor_write_regs(c2, steps[i].ptr,
							steps[i].len);
			break;
		case STEP_BURST:
			ret = s5c73c2_sensor_write_raw(c2, steps[i].ptr,
						       steps[i].len);
			break;
		default:
			return -EINVAL;
		}
		if (ret)
			return ret;
	}

	return 0;
}

static int s5c73c2_2p2_start_evt3(struct s5c73c2 *c2, bool full_res)
{
	const struct s5c73c2_reg *mregs = full_res ?
		s5k2p2_evt3_5328x3000_regs : s5k2p2_evt3_2664x1500_regs;
	unsigned int mlen = full_res ?
		ARRAY_SIZE(s5k2p2_evt3_5328x3000_regs) :
		ARRAY_SIZE(s5k2p2_evt3_2664x1500_regs);
	const struct s5c73c2_step steps[] = {
		S2_REGS(s5k2p2_evt3_init0),
		S2_REGS(s5k2p2_evt3_init1),
		S2_REGS(s5k2p2_evt3_init2),
		S2_REGS(s5k2p2_evt3_init3),
		S2_REGS(s5k2p2_evt3_init4),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x4000),
		{ STEP_REGS, 0, 0, mregs, mlen },
		S2_REGS(s5k2p2_evt3_init_tail),
		S2_REGS(s5k2p2_evt3_mode_tail),
		S2_W16(S5K2P2_REG_6004, 0x0001),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x2000),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x2e00),
		S2_BURST(s5k2p2_evt3_sram_1),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x4000),
		S2_W16(S5K2P2_REG_70B6, 0x0001),
		S2_W16(S5K2P2_REG_70D4, 0x0001),
		S2_W8(S5K2P2_REG_3050, 0x00),
		S2_W16(S5K2P2_REG_306A, 0x8000),
		S2_W16(S5K2P2_REG_3068, 0x0000),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x2000),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x1410),
		S2_W8(S5K2P2_REG_RAM_DATA, 0x01),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x1416),
		S2_BURST(s5k2p2_evt3_sram_2),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x8000),
		S2_BURST(s5k2p2_evt3_sram_3),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x3228),
		S2_BURST(s5k2p2_evt3_sram_4),
		S2_REGS(s5k2p2_evt3_init3),
		S2_REGS(s5k2p2_evt3_post_reg),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x3dd4),
		S2_BURST(s5k2p2_evt3_sram_5),
		S2_W8(0x3e76, 0x0a),
		S2_W16(0x3ed6, 0x0105),
		S2_W16(0x3ed8, 0x0101),
		S2_W16(0x3ede, 0x00f0),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x3f44),
		S2_BURST(s5k2p2_evt3_sram_6),
		S2_W16(0x306e, 0x039c),
		S2_W16(0x3072, 0x00ff),
		S2_W16(0x30c4, 0x0001),
		S2_W16(0x30c6, 0x0001),
		S2_W8(0x315c, 0x00),
		S2_W8(0x0b05, 0x01),
		S2_W16(0xb0ca, 0x4990),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x4000),
		{ STEP_REGS, 0, 0, mregs, mlen },
		S2_REGS(s5k2p2_evt3_mode_tail),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x4000),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x3a70),
		S2_BURST(s5k2p2_evt3_sram_7),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x3ab2),
		S2_BURST(s5k2p2_evt3_sram_8),
		S2_W16(S5K2P2_REG_6004, 0x0000),
	};

	return s5c73c2_run_steps(c2, steps, ARRAY_SIZE(steps));
}

static int s5c73c2_2p2_start_evt4(struct s5c73c2 *c2, bool full_res)
{
	const struct s5c73c2_step init_steps[] = {
		S2_REGS(s5k2p2_evt4_pre_regs),
		S2_REGS(s5k2p2_evt4_common_regs),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x2000),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x11d0),
		S2_W8(S5K2P2_REG_RAM_DATA, 0x01),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x4000),
		S2_W16(S5K2P2_REG_70D4, 0x0001),
		S2_W16(S5K2P2_REG_306C, 0x4000),
		S2_W16(S5K2P2_REG_306A, 0x0000),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x2000),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x11d0),
		S2_W8(S5K2P2_REG_RAM_DATA, 0x01),
		S2_W16(S5K2P2_REG_6004, 0x0001),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x4000),
		S2_BURST(s5k2p2_evt4_sram_patch),
		S2_REGS(s5k2p2_evt4_pre_regs),
		S2_REGS(s5k2p2_evt4_common_regs),
	};
	const struct s5c73c2_step m0_steps[] = {
		S2_REGS(s5k2p2_evt4_5328x3000_p1),
		S2_BURST(s5k2p2_evt4_5328x3000_b1),
		S2_BURST(s5k2p2_evt4_5328x3000_b2),
		S2_REGS(s5k2p2_evt4_5328x3000_p2),
		S2_REGS(s5k2p2_evt4_mode_tail),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x2000),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x11d0),
		S2_W8(S5K2P2_REG_RAM_DATA, 0x01),
	};
	const struct s5c73c2_step m6_steps[] = {
		S2_REGS(s5k2p2_evt4_2664x1500_p1),
		S2_BURST(s5k2p2_evt4_2664x1500_sram),
		S2_REGS(s5k2p2_evt4_2664x1500_p2),
		S2_BURST(s5k2p2_evt4_2664x1500_b1),
		S2_BURST(s5k2p2_evt4_2664x1500_b2),
		S2_REGS(s5k2p2_evt4_2664x1500_p3),
		S2_REGS(s5k2p2_evt4_mode_tail),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x2000),
		S2_W16(S5K2P2_REG_PTR_SELECT, 0x11d0),
		S2_W8(S5K2P2_REG_RAM_DATA, 0x01),
	};
	int ret;

	ret = s5c73c2_run_steps(c2, init_steps, ARRAY_SIZE(init_steps));
	if (ret)
		return ret;
	return full_res ?
		s5c73c2_run_steps(c2, m0_steps, ARRAY_SIZE(m0_steps)) :
		s5c73c2_run_steps(c2, m6_steps, ARRAY_SIZE(m6_steps));
}

static int s5c73c2_2p2_start_evt5(struct s5c73c2 *c2, bool full_res)
{
	const struct s5c73c2_step steps[] = {
		S2_REGS(s5k2p2_evt5_common_regs),
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x4000),
		{ STEP_REGS, 0, 0,
		  full_res ? s5k2p2_evt5_5328x3000_regs :
			     s5k2p2_evt5_2664x1500_regs,
		  full_res ? ARRAY_SIZE(s5k2p2_evt5_5328x3000_regs) :
			     ARRAY_SIZE(s5k2p2_evt5_2664x1500_regs) },
	};

	return s5c73c2_run_steps(c2, steps, ARRAY_SIZE(steps));
}

static int s5c73c2_2p2_start(struct s5c73c2 *c2)
{
	const struct s5c73c2_step reset_steps[] = {
		S2_W16(S5K2P2_REG_SW_RESET, 0x0001),
	};
	const struct s5c73c2_step pll_steps[] = {
		S2_W16(S5K2P2_REG_PAGE_SELECT, 0x4000),
		S2_W16(S5K2P2_REG_6214, 0x7970),
		S2_W16(S5K2P2_REG_6218, 0x7150),
		S2_W16(S5K2P2_REG_6226, 0x0001),
	};
	bool full_res = c2->mode->c73c2_mode == 0;
	int ret;

	ret = s5c73c2_run_steps(c2, reset_steps, ARRAY_SIZE(reset_steps));
	if (ret)
		return ret;
	usleep_range(2000, 2500);

	ret = s5c73c2_run_steps(c2, pll_steps, ARRAY_SIZE(pll_steps));
	if (ret)
		return ret;

	switch (c2->sensor_rev) {
	case S5K2P2_REV_EVT3:
		ret = s5c73c2_2p2_start_evt3(c2, full_res);
		break;
	case S5K2P2_REV_EVT4:
		ret = s5c73c2_2p2_start_evt4(c2, full_res);
		break;
	case S5K2P2_REV_EVT5:
		ret = s5c73c2_2p2_start_evt5(c2, full_res);
		break;
	default:
		dev_err(c2->dev, "unsupported 2p2 revision 0x%02x\n",
			c2->sensor_rev);
		return -EOPNOTSUPP;
	}
	if (ret)
		return ret;

	ret = s5c73c2_sensor_write8(c2, S5K2P2_REG_B0E_CTRL, 0x00);
	return ret ?: s5c73c2_sensor_write16(c2, S5K2P2_REG_PAGE_SELECT,
					       0x4000);
}

static void s5c73c2_detect_sensor(struct s5c73c2 *c2)
{
	unsigned int i;
	u16 val;
	int ret;

	for (i = 0; i < ARRAY_SIZE(s5c73c2_sensors); i++) {
		const struct s5c73c2_sensor *s = &s5c73c2_sensors[i];

		val = 0;
		ret = s5c73c2_read(c2, s->addr, s->id_reg, &val);
		if (ret)
			continue;
		if (!s->id || val == s->id) {
			c2->sensor = s;
			if (!strcmp(s->name, "2p2")) {
				c2->info = &s5c73c2_2p2_info;
				if (!s5c73c2_sensor_read8(c2, S5K2P2_REG_REV_ID,
							  &c2->sensor_rev))
					dev_info(c2->dev,
						 "2p2 id=0x%04x rev=0x%02x\n",
						 val, c2->sensor_rev);
			} else {
				c2->info = &s5c73c2_imx240_info;
			}
			return;
		}
	}
}

static u32 s5c73c2_find_mbus_code(const struct s5c73c2 *c2, u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(s5c73c2_mbus_codes); i++) {
		if (s5c73c2_mbus_codes[i] == code)
			return code;
	}

	return c2->info->default_mbus_code;
}

static void s5c73c2_fill_format(const struct s5c73c2 *c2,
				const struct s5c73c2_mode *mode, u32 code,
				struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5c73c2_find_mbus_code(c2, code);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5c73c2_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5c73c2 *c2 =
		container_of(ctrl->handler, struct s5c73c2, ctrl_handler);
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int expo_max = c2->mode->height + ctrl->val -
			       c2->info->expo_margin;
		int expo_def = min(expo_max, (int)c2->mode->expo_def);

		__v4l2_ctrl_modify_range(c2->exposure,
					 c2->exposure->minimum,
					 expo_max, c2->exposure->step,
					 expo_def);
	}

	ret = pm_runtime_get_if_in_use(c2->dev);
	if (ret <= 0)
		return ret == -EINVAL ? 0 : ret;

	if (!c2->sensor) {
		pm_runtime_mark_last_busy(c2->dev);
		pm_runtime_put_autosuspend(c2->dev);
		return -ENODEV;
	}

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = s5c73c2_sensor_write16(c2, IMX240_REG_COARSE_INT_TIME,
					     ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = s5c73c2_sensor_write16(c2, IMX240_REG_ANA_GAIN_GLOBAL,
					     ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		if (c2->info->is_2p2)
			ret = s5c73c2_sensor_write16(c2,
						     IMX240_REG_DIG_GAIN_GR,
						     ctrl->val);
		else
			ret = s5c73c2_sensor_set_dgain(c2, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = s5c73c2_sensor_write16(c2, IMX240_REG_FRAME_LENGTH_LINES,
					     c2->mode->height + ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(c2->dev);
	pm_runtime_put_autosuspend(c2->dev);
	return ret;
}

static const struct v4l2_ctrl_ops s5c73c2_ctrl_ops = {
	.s_ctrl = s5c73c2_set_ctrl,
};

static int s5c73c2_init_cfg(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state)
{
	struct s5c73c2 *c2 = to_s5c73c2(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;

	fmt = v4l2_subdev_get_pad_format(sd, state, 0);
	s5c73c2_fill_format(c2, &c2->info->modes[0],
			    c2->info->default_mbus_code, fmt);

	crop = v4l2_subdev_get_pad_crop(sd, state, 0);
	crop->left = 0;
	crop->top = 0;
	crop->width = S5C73C2_NATIVE_WIDTH;
	crop->height = S5C73C2_NATIVE_HEIGHT;

	return 0;
}

static int s5c73c2_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(s5c73c2_mbus_codes))
		return -EINVAL;

	code->code = s5c73c2_mbus_codes[code->index];
	return 0;
}

static int s5c73c2_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5c73c2 *c2 = to_s5c73c2(sd);
	unsigned int i;

	if (fse->index >= c2->info->num_modes)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(s5c73c2_mbus_codes); i++) {
		if (fse->code == s5c73c2_mbus_codes[i])
			break;
	}
	if (i == ARRAY_SIZE(s5c73c2_mbus_codes))
		return -EINVAL;

	fse->min_width = c2->info->modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = c2->info->modes[fse->index].height;
	fse->max_height = fse->min_height;
	return 0;
}

static int s5c73c2_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	struct s5c73c2 *c2 = to_s5c73c2(sd);
	const struct s5c73c2_mode *mode;
	struct v4l2_mbus_framefmt *pad_fmt;

	mode = v4l2_find_nearest_size(c2->info->modes, c2->info->num_modes,
				      width, height,
				      fmt->format.width, fmt->format.height);

	s5c73c2_fill_format(c2, mode, fmt->format.code, &fmt->format);
	pad_fmt = v4l2_subdev_get_pad_format(sd, state, 0);
	*pad_fmt = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		int vblank_def = mode->vts_def - mode->height;
		int expo_max = mode->vts_def - c2->info->expo_margin;
		int hblank = mode->hts - mode->width;

		c2->mode = mode;
		__v4l2_ctrl_s_ctrl(c2->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(c2->pixel_rate, mode->pixel_rate);
		__v4l2_ctrl_modify_range(c2->vblank, S5C73C2_VBLANK_MIN,
					 S5C73C2_VTS_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(c2->vblank, vblank_def);
		__v4l2_ctrl_modify_range(c2->hblank, hblank, hblank, 1,
					 hblank);
		__v4l2_ctrl_modify_range(c2->exposure, c2->info->expo_min,
					 expo_max, 1,
					 min(expo_max, (int)mode->expo_def));
	}

	return 0;
}

static int s5c73c2_get_selection(struct v4l2_subdev *sd,
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
		sel->r.width = S5C73C2_NATIVE_WIDTH;
		sel->r.height = S5C73C2_NATIVE_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int s5c73c2_companion_set_mode(struct s5c73c2 *c2,
				      const struct s5c73c2_mode *mode)
{
	int ret;

	ret = s5c73c2_write(c2, S5C73C2_REG_MODE, mode->c73c2_mode);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_WDR_AE_CTRL0, 0x0000);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_WDR_AE_CTRL1, 0x0000);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_WDR_MODE, 0x0100);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_CAL_BYPASS, 0x0000);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_PAF_DISABLE,
				   mode->paf_en ? 0x0000 : 0x0001);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_MODE_APPLY, 0x0001);
	if (!ret)
		usleep_range(1000, 2000);

	return ret;
}

static int s5c73c2_start_streaming(struct s5c73c2 *c2)
{
	int ret;

	if (!c2->sensor)
		return -ENODEV;

	ret = pm_runtime_resume_and_get(c2->dev);
	if (ret < 0)
		return ret;

	if (c2->info->is_2p2) {
		ret = s5c73c2_2p2_start(c2);
	} else {
		ret = s5c73c2_sensor_write_table(c2, imx240_common_regs,
						 ARRAY_SIZE(imx240_common_regs));
		ret = ret ?: s5c73c2_sensor_write_table(c2, c2->mode->regs,
							c2->mode->num_regs);
	}
	ret = ret ?: s5c73c2_sensor_write8(c2, IMX240_REG_GROUP_HOLD, 0x01);
	ret = ret ?: __v4l2_ctrl_handler_setup(&c2->ctrl_handler);
	ret = ret ?: s5c73c2_sensor_write8(c2, IMX240_REG_GROUP_HOLD, 0x00);
	ret = ret ?: s5c73c2_companion_set_mode(c2, c2->mode);
	ret = ret ?: s5c73c2_write(c2, S5C73C2_REG_STREAM, 0x0001);
	if (c2->info->is_2p2)
		ret = ret ?: s5c73c2_sensor_write16(c2, S5K2P2_REG_PAGE_SELECT,
						     0x4000);
	ret = ret ?: s5c73c2_sensor_write8(c2, IMX240_REG_MODE_SELECT, 0x01);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	pm_runtime_mark_last_busy(c2->dev);
	pm_runtime_put_autosuspend(c2->dev);
	return ret;
}

static void s5c73c2_stop_streaming(struct s5c73c2 *c2)
{
	u32 frame_state = 0;
	u8 frame_cnt = 0;
	int ret;

	if (c2->info->is_2p2)
		s5c73c2_sensor_write16(c2, S5K2P2_REG_PAGE_SELECT, 0x4000);
	ret = s5c73c2_sensor_write8(c2, IMX240_REG_MODE_SELECT, 0x00);
	if (ret) {
		dev_err(c2->dev, "failed to stop sensor stream: %d\n", ret);
	} else {
		ret = read_poll_timeout(s5c73c2_sensor_read8, ret,
					ret || frame_cnt == 0xff,
					2000, 500000, false, c2,
					IMX240_REG_FRAME_COUNT, &frame_cnt);
		if (ret || frame_cnt != 0xff)
			dev_warn(c2->dev,
				 "sensor stream-off poll timed out (cnt=0x%02x)\n",
				 frame_cnt);
	}

	ret = s5c73c2_write(c2, S5C73C2_REG_STREAM, 0x0000);
	if (ret) {
		dev_err(c2->dev, "failed to stop companion stream: %d\n", ret);
	} else {
		ret = read_poll_timeout(s5c73c2_read32, ret,
					ret || frame_state == 0,
					2000, 500000, false, c2,
					c2->client->addr,
					S5C73C2_REG_FRAME_STATE, &frame_state);
		if (ret || frame_state != 0)
			dev_warn(c2->dev,
				 "companion stream-off timed out (state=0x%08x)\n",
				 frame_state);
	}

	pm_runtime_mark_last_busy(c2->dev);
	pm_runtime_put_autosuspend(c2->dev);
}

static int s5c73c2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct s5c73c2 *c2 = to_s5c73c2(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	if (c2->streaming == !!enable)
		goto unlock;

	if (enable)
		ret = s5c73c2_start_streaming(c2);
	else
		s5c73c2_stop_streaming(c2);

	if (!ret)
		c2->streaming = !!enable;

unlock:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static const struct v4l2_subdev_core_ops s5c73c2_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops s5c73c2_video_ops = {
	.s_stream = s5c73c2_s_stream,
};

static const struct v4l2_subdev_pad_ops s5c73c2_pad_ops = {
	.init_cfg = s5c73c2_init_cfg,
	.enum_mbus_code = s5c73c2_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5c73c2_set_fmt,
	.get_selection = s5c73c2_get_selection,
	.enum_frame_size = s5c73c2_enum_frame_size,
};

static const struct v4l2_subdev_ops s5c73c2_subdev_ops = {
	.core = &s5c73c2_core_ops,
	.video = &s5c73c2_video_ops,
	.pad = &s5c73c2_pad_ops,
};

static int s5c73c2_init_controls(struct s5c73c2 *c2)
{
	struct v4l2_ctrl_handler *hdl = &c2->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	const struct s5c73c2_mode *mode = c2->mode;
	int hblank = mode->hts - mode->width;
	int vblank = mode->vts_def - mode->height;
	int expo_max = mode->vts_def - c2->info->expo_margin;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 9);
	if (ret)
		return ret;

	c2->pixel_rate = v4l2_ctrl_new_std(hdl, &s5c73c2_ctrl_ops,
					   V4L2_CID_PIXEL_RATE,
					   S5C73C2_PIXEL_RATE_MIN,
					   S5C73C2_PIXEL_RATE_MAX, 1,
					   mode->pixel_rate);

	c2->link_freq =
		v4l2_ctrl_new_int_menu(hdl, &s5c73c2_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(s5c73c2_link_freq_menu) - 1,
				       mode->link_freq_index,
				       s5c73c2_link_freq_menu);
	if (c2->link_freq)
		c2->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	c2->vblank = v4l2_ctrl_new_std(hdl, &s5c73c2_ctrl_ops,
				       V4L2_CID_VBLANK,
				       S5C73C2_VBLANK_MIN,
				       S5C73C2_VTS_MAX - mode->height,
				       1, vblank);

	c2->hblank = v4l2_ctrl_new_std(hdl, &s5c73c2_ctrl_ops,
				       V4L2_CID_HBLANK,
				       hblank, hblank, 1, hblank);
	if (c2->hblank)
		c2->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	c2->exposure = v4l2_ctrl_new_std(hdl, &s5c73c2_ctrl_ops,
					 V4L2_CID_EXPOSURE,
					 c2->info->expo_min, expo_max, 1,
					 min(expo_max, (int)mode->expo_def));
	if (c2->exposure)
		c2->exposure->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	c2->again = v4l2_ctrl_new_std(hdl, &s5c73c2_ctrl_ops,
				      V4L2_CID_ANALOGUE_GAIN,
				      c2->info->again_min, c2->info->again_max,
				      1, c2->info->again_default);
	if (c2->again)
		c2->again->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	c2->dgain = v4l2_ctrl_new_std(hdl, &s5c73c2_ctrl_ops,
				      V4L2_CID_DIGITAL_GAIN,
				      S5C73C2_DGAIN_MIN, S5C73C2_DGAIN_MAX,
				      1, S5C73C2_DGAIN_DEFAULT);
	if (c2->dgain)
		c2->dgain->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	ret = v4l2_fwnode_device_parse(c2->dev, &props);
	if (ret)
		goto err_free;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &s5c73c2_ctrl_ops, &props);
	if (ret)
		goto err_free;

	c2->sd.ctrl_handler = hdl;
	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int s5c73c2_check_hwcfg(struct device *dev)
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

	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(dev, "expected 4 CSI-2 data lanes, got %u\n",
			ep_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
	} else if (ep_cfg.nr_of_link_frequencies &&
		   ep_cfg.link_frequencies[0] != s5c73c2_link_freq_menu[0]) {
		dev_err(dev, "unsupported link frequency %llu\n",
			ep_cfg.link_frequencies[0]);
		ret = -EINVAL;
	}

	v4l2_fwnode_endpoint_free(&ep_cfg);
	return ret;
}

static int s5c73c2_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5c73c2 *c2 = to_s5c73c2(sd);

	s5c73c2_power_off(c2, ARRAY_SIZE(s5c73c2_supplies));
	return 0;
}

static int s5c73c2_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5c73c2 *c2 = to_s5c73c2(sd);
	int ret;

	ret = s5c73c2_power_on(c2);
	if (ret)
		return ret;

	ret = s5c73c2_identify_and_boot(c2);
	if (ret) {
		s5c73c2_power_off(c2, ARRAY_SIZE(s5c73c2_supplies));
		return ret;
	}

	return 0;
}

static void s5c73c2_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5c73c2 *c2 = to_s5c73c2(sd);

	v4l2_async_unregister_subdev(sd);
	pm_runtime_dont_use_autosuspend(c2->dev);
	pm_runtime_disable(c2->dev);
	if (!pm_runtime_status_suspended(c2->dev))
		s5c73c2_power_off(c2, ARRAY_SIZE(s5c73c2_supplies));
	pm_runtime_set_suspended(c2->dev);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&c2->ctrl_handler);
}

static const struct dev_pm_ops s5c73c2_pm_ops = {
	SET_RUNTIME_PM_OPS(s5c73c2_runtime_suspend,
			   s5c73c2_runtime_resume, NULL)
};

static const struct of_device_id s5c73c2_of_match[] = {
	{ .compatible = "samsung,s5c73c2" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5c73c2_of_match);

static struct i2c_driver s5c73c2_driver = {
	.driver = {
		.name = "s5c73c2",
		.of_match_table = s5c73c2_of_match,
		.pm = &s5c73c2_pm_ops,
	},
	.probe = s5c73c2_probe,
	.remove = s5c73c2_remove,
};
module_i2c_driver(s5c73c2_driver);

MODULE_DESCRIPTION("Samsung S5C73C2 camera companion");
MODULE_LICENSE("GPL");

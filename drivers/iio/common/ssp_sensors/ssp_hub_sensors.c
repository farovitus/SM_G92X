// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * IIO devices for the SSP hub sensors beyond the accelerometer and gyroscope:
 * magnetometer, pressure, light, proximity, rotation vectors, step counter,
 * step detector and significant motion.
 *
 * Each is a buffer-only IIO device fed by the hub. A hub sample is packed and
 * unaligned; it is unpacked into the IIO scan layout and stamped with the
 * time the hub core gives. The scan buffer lives as long as the device, so
 * a sample that races with buffer disable never lands in freed memory.
 */

#include <linux/iio/buffer.h>
#include <linux/iio/common/ssp_sensors.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "ssp_iio_sensor.h"

#define SSP_HUB_MAX_CHANNELS	8
/*
 * Largest scan record: the IIO core pads a record to its largest element,
 * so the 16-byte quaternion makes a 32-byte record with the timestamp at
 * byte 16, the next 8-byte boundary, where readers of the scan_elements
 * ABI look for it. iio_push_to_buffers_with_timestamp() would put it in
 * the last 8 bytes (24) instead, so the driver places it itself.
 */
#define SSP_HUB_SCAN_SIZE	64

struct ssp_hub_sensor_desc {
	enum ssp_sensor_type type;
	const struct iio_chan_spec *channels;
	unsigned int num_channels;	/* including the timestamp */
	const unsigned long *scan_masks;
	const u8 *src;			/* sample offset of each data channel */
	int scale[SSP_HUB_MAX_CHANNELS][2]; /* IIO_VAL_FRACTIONAL, 0/0: none */
	int offset[SSP_HUB_MAX_CHANNELS];
};

/* iio_priv(): struct ssp_sensor_data must stay first for the hub core. */
struct ssp_hub_sensor {
	struct ssp_sensor_data spd;
	const struct ssp_hub_sensor_desc *desc;
	u8 dst[SSP_HUB_MAX_CHANNELS];
	u8 len[SSP_HUB_MAX_CHANNELS];
	u8 ts_off;			/* timestamp offset in the record */
};

#define SSP_HUB_CHAN(_type, _mod, _chan, _idx, _sign, _bits, _repeat, _info) \
{									\
	.type = (_type),						\
	.modified = (_mod) != IIO_NO_MOD,				\
	.channel2 = (_mod),						\
	.indexed = (_mod) == IIO_NO_MOD && (_chan) >= 0,		\
	.channel = (_chan) < 0 ? 0 : (_chan),				\
	.info_mask_separate = (_info),					\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_idx),						\
	.scan_type = {							\
		.sign = (_sign),					\
		.realbits = (_bits),					\
		.storagebits = (_bits),					\
		.repeat = (_repeat),					\
		.endianness = IIO_LE,					\
	},								\
}

#define SSP_HUB_SCALE		BIT(IIO_CHAN_INFO_SCALE)
#define SSP_HUB_SCALE_OFFSET	(BIT(IIO_CHAN_INFO_SCALE) | \
				 BIT(IIO_CHAN_INFO_OFFSET))

static const unsigned long ssp_hub_mask_1[] = { GENMASK(0, 0), 0 };
static const unsigned long ssp_hub_mask_2[] = { GENMASK(1, 0), 0 };
static const unsigned long ssp_hub_mask_3[] = { GENMASK(2, 0), 0 };
static const unsigned long ssp_hub_mask_6[] = { GENMASK(5, 0), 0 };

/* Calibrated field, 3 x s16; the trailing accuracy byte is dropped. */
static const struct iio_chan_spec ssp_magn_channels[] = {
	SSP_HUB_CHAN(IIO_MAGN, IIO_MOD_X, -1, 0, 's', 16, 0, 0),
	SSP_HUB_CHAN(IIO_MAGN, IIO_MOD_Y, -1, 1, 's', 16, 0, 0),
	SSP_HUB_CHAN(IIO_MAGN, IIO_MOD_Z, -1, 2, 's', 16, 0, 0),
	SSP_CHAN_TIMESTAMP(3),
};

/* LPS25H: s32 at 4096 counts/hPa, s16 temperature 42.5 C + t / 480. */
static const struct iio_chan_spec ssp_pressure_channels[] = {
	SSP_HUB_CHAN(IIO_PRESSURE, IIO_NO_MOD, -1, 0, 's', 32, 0,
		     SSP_HUB_SCALE),
	SSP_HUB_CHAN(IIO_TEMP, IIO_NO_MOD, -1, 1, 's', 16, 0,
		     SSP_HUB_SCALE_OFFSET),
	SSP_CHAN_TIMESTAMP(2),
};

/* TMD4903 raw colour counts with the integration time and gain codes. */
static const struct iio_chan_spec ssp_light_channels[] = {
	SSP_HUB_CHAN(IIO_INTENSITY, IIO_MOD_LIGHT_RED, -1, 0, 'u', 16, 0, 0),
	SSP_HUB_CHAN(IIO_INTENSITY, IIO_MOD_LIGHT_GREEN, -1, 1, 'u', 16, 0, 0),
	SSP_HUB_CHAN(IIO_INTENSITY, IIO_MOD_LIGHT_BLUE, -1, 2, 'u', 16, 0, 0),
	SSP_HUB_CHAN(IIO_INTENSITY, IIO_MOD_LIGHT_CLEAR, -1, 3, 'u', 16, 0, 0),
	/* in_intensity0_raw: atime code, in_intensity1_raw: again code */
	SSP_HUB_CHAN(IIO_INTENSITY, IIO_NO_MOD, 0, 4, 'u', 8, 0, 0),
	SSP_HUB_CHAN(IIO_INTENSITY, IIO_NO_MOD, 1, 5, 'u', 8, 0, 0),
	SSP_CHAN_TIMESTAMP(6),
};

/* proximity0: 1 near, 0 far; proximity1: the adc count. */
static const struct iio_chan_spec ssp_prox_channels[] = {
	SSP_HUB_CHAN(IIO_PROXIMITY, IIO_NO_MOD, 0, 0, 'u', 8, 0, 0),
	SSP_HUB_CHAN(IIO_PROXIMITY, IIO_NO_MOD, 1, 1, 'u', 16, 0, 0),
	SSP_CHAN_TIMESTAMP(2),
};

/* Quaternion, 4 x s32; the trailing accuracy byte is dropped. */
static const struct iio_chan_spec ssp_rot_channels[] = {
	SSP_HUB_CHAN(IIO_ROT, IIO_MOD_QUATERNION, -1, 0, 's', 32, 4, 0),
	SSP_CHAN_TIMESTAMP(1),
};

/* Steps since the previous sample. */
static const struct iio_chan_spec ssp_step_counter_channels[] = {
	SSP_HUB_CHAN(IIO_STEPS, IIO_NO_MOD, -1, 0, 'u', 32, 0, 0),
	SSP_CHAN_TIMESTAMP(1),
};

/* One sample per event; the byte is always 1. */
static const struct iio_chan_spec ssp_step_detector_channels[] = {
	SSP_HUB_CHAN(IIO_STEPS, IIO_NO_MOD, -1, 0, 'u', 8, 0, 0),
	SSP_CHAN_TIMESTAMP(1),
};

static const struct iio_chan_spec ssp_sig_motion_channels[] = {
	SSP_HUB_CHAN(IIO_ACTIVITY, IIO_NO_MOD, -1, 0, 'u', 8, 0, 0),
	SSP_CHAN_TIMESTAMP(1),
};

static const u8 ssp_src_magn[] = { 0, 2, 4 };
static const u8 ssp_src_pressure[] = { 0, 4 };
static const u8 ssp_src_light[] = { 0, 2, 4, 6, 8, 9 };
static const u8 ssp_src_prox[] = { 0, 1 };
static const u8 ssp_src_first[] = { 0 };

#define SSP_HUB_DESC(_type, _chans, _mask, _src)			\
	.type = (_type),						\
	.channels = (_chans),						\
	.num_channels = ARRAY_SIZE(_chans),				\
	.scan_masks = (_mask),						\
	.src = (_src)

static const struct ssp_hub_sensor_desc ssp_magn_desc = {
	SSP_HUB_DESC(SSP_GEOMAGNETIC_SENSOR, ssp_magn_channels,
		     ssp_hub_mask_3, ssp_src_magn),
};

static const struct ssp_hub_sensor_desc ssp_pressure_desc = {
	SSP_HUB_DESC(SSP_PRESSURE_SENSOR, ssp_pressure_channels,
		     ssp_hub_mask_2, ssp_src_pressure),
	/* kPa per count; milli-degrees C per count, offset 42.5 C */
	.scale = { { 1, 40960 }, { 1000, 480 } },
	.offset = { 0, 20400 },
};

static const struct ssp_hub_sensor_desc ssp_light_desc = {
	SSP_HUB_DESC(SSP_LIGHT_SENSOR, ssp_light_channels,
		     ssp_hub_mask_6, ssp_src_light),
};

static const struct ssp_hub_sensor_desc ssp_prox_desc = {
	SSP_HUB_DESC(SSP_PROXIMITY_SENSOR, ssp_prox_channels,
		     ssp_hub_mask_2, ssp_src_prox),
};

static const struct ssp_hub_sensor_desc ssp_rv_desc = {
	SSP_HUB_DESC(SSP_ROTATION_VECTOR, ssp_rot_channels,
		     ssp_hub_mask_1, ssp_src_first),
};

static const struct ssp_hub_sensor_desc ssp_grv_desc = {
	SSP_HUB_DESC(SSP_GAME_ROTATION_VECTOR, ssp_rot_channels,
		     ssp_hub_mask_1, ssp_src_first),
};

static const struct ssp_hub_sensor_desc ssp_step_counter_desc = {
	SSP_HUB_DESC(SSP_STEP_COUNTER, ssp_step_counter_channels,
		     ssp_hub_mask_1, ssp_src_first),
};

static const struct ssp_hub_sensor_desc ssp_step_detector_desc = {
	SSP_HUB_DESC(SSP_STEP_DETECTOR, ssp_step_detector_channels,
		     ssp_hub_mask_1, ssp_src_first),
};

static const struct ssp_hub_sensor_desc ssp_sig_motion_desc = {
	SSP_HUB_DESC(SSP_SIG_MOTION_SENSOR, ssp_sig_motion_channels,
		     ssp_hub_mask_1, ssp_src_first),
};

static int ssp_hub_process_data(struct iio_dev *indio_dev, void *buf,
				int64_t timestamp)
{
	struct ssp_hub_sensor *s = iio_priv(indio_dev);
	const struct ssp_hub_sensor_desc *desc = s->desc;
	unsigned int i;

	for (i = 0; i < desc->num_channels - 1; i++)
		memcpy(s->spd.buffer + s->dst[i], (u8 *)buf + desc->src[i],
		       s->len[i]);

	if (indio_dev->scan_timestamp)
		memcpy(s->spd.buffer + s->ts_off, &timestamp, sizeof(timestamp));

	return iio_push_to_buffers(indio_dev, s->spd.buffer);
}

static int ssp_hub_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	struct ssp_hub_sensor *s = iio_priv(indio_dev);
	struct ssp_data *data = dev_get_drvdata(indio_dev->dev.parent->parent);
	int i = chan->scan_index;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ssp_convert_to_freq(ssp_get_sensor_delay(data, s->spd.type),
				    val, val2);
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SCALE:
		if (!s->desc->scale[i][1])
			return -EINVAL;
		*val = s->desc->scale[i][0];
		*val2 = s->desc->scale[i][1];
		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_OFFSET:
		*val = s->desc->offset[i];
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ssp_hub_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct ssp_hub_sensor *s = iio_priv(indio_dev);
	struct ssp_data *data = dev_get_drvdata(indio_dev->dev.parent->parent);

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;

	return ssp_change_delay(data, s->spd.type,
				ssp_convert_to_time(val, val2));
}

static const struct iio_info ssp_hub_iio_info = {
	.read_raw = ssp_hub_read_raw,
	.write_raw = ssp_hub_write_raw,
};

static int ssp_hub_postenable(struct iio_dev *indio_dev)
{
	struct ssp_hub_sensor *s = iio_priv(indio_dev);
	struct ssp_data *data = dev_get_drvdata(indio_dev->dev.parent->parent);

	return ssp_enable_sensor(data, s->spd.type,
				 ssp_get_sensor_delay(data, s->spd.type));
}

static int ssp_hub_postdisable(struct iio_dev *indio_dev)
{
	struct ssp_hub_sensor *s = iio_priv(indio_dev);
	struct ssp_data *data = dev_get_drvdata(indio_dev->dev.parent->parent);

	return ssp_disable_sensor(data, s->spd.type);
}

static const struct iio_buffer_setup_ops ssp_hub_buffer_ops = {
	.postenable = ssp_hub_postenable,
	.postdisable = ssp_hub_postdisable,
};

static int ssp_hub_sensor_probe(struct platform_device *pdev)
{
	const struct ssp_hub_sensor_desc *desc =
		(const void *)platform_get_device_id(pdev)->driver_data;
	struct ssp_hub_sensor *s;
	struct iio_dev *indio_dev;
	unsigned int i, off = 0, size, largest = sizeof(s64);
	int ret;

	if (WARN_ON(desc->num_channels - 1 > SSP_HUB_MAX_CHANNELS))
		return -EINVAL;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*s));
	if (!indio_dev)
		return -ENOMEM;

	s = iio_priv(indio_dev);
	s->desc = desc;
	s->spd.type = desc->type;
	s->spd.process_data = ssp_hub_process_data;

	/* Scan layout with every channel on: each naturally aligned. */
	for (i = 0; i < desc->num_channels - 1; i++) {
		const typeof(desc->channels[i].scan_type) *st =
			&desc->channels[i].scan_type;

		size = st->storagebits / 8;
		off = ALIGN(off, size);
		s->dst[i] = off;
		s->len[i] = size * (st->repeat ? st->repeat : 1);
		off += s->len[i];
		largest = max_t(unsigned int, largest, s->len[i]);
	}

	/* data, the s64 timestamp, padded as iio_compute_scan_bytes() does */
	s->ts_off = ALIGN(off, sizeof(s64));
	off = ALIGN(s->ts_off + sizeof(s64), largest);
	if (WARN_ON(off > SSP_HUB_SCAN_SIZE))
		return -EINVAL;

	s->spd.buffer = devm_kzalloc(&pdev->dev, SSP_HUB_SCAN_SIZE, GFP_KERNEL);
	if (!s->spd.buffer)
		return -ENOMEM;

	indio_dev->name = platform_get_device_id(pdev)->name;
	indio_dev->info = &ssp_hub_iio_info;
	indio_dev->channels = desc->channels;
	indio_dev->num_channels = desc->num_channels;
	indio_dev->available_scan_masks = desc->scan_masks;

	ret = devm_iio_kfifo_buffer_setup(&pdev->dev, indio_dev,
					  &ssp_hub_buffer_ops);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, indio_dev);

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	/* ssp registering should be done after all iio setup */
	ssp_register_consumer(indio_dev, desc->type);

	return 0;
}

static const struct platform_device_id ssp_hub_sensor_ids[] = {
	{ "ssp-magnetometer", (kernel_ulong_t)&ssp_magn_desc },
	{ "ssp-pressure", (kernel_ulong_t)&ssp_pressure_desc },
	{ "ssp-light", (kernel_ulong_t)&ssp_light_desc },
	{ "ssp-proximity", (kernel_ulong_t)&ssp_prox_desc },
	{ "ssp-rotation-vector", (kernel_ulong_t)&ssp_rv_desc },
	{ "ssp-game-rotvec", (kernel_ulong_t)&ssp_grv_desc },
	{ "ssp-step-counter", (kernel_ulong_t)&ssp_step_counter_desc },
	{ "ssp-step-detector", (kernel_ulong_t)&ssp_step_detector_desc },
	{ "ssp-sig-motion", (kernel_ulong_t)&ssp_sig_motion_desc },
	{ }
};
MODULE_DEVICE_TABLE(platform, ssp_hub_sensor_ids);

static struct platform_driver ssp_hub_sensor_driver = {
	.driver = {
		.name = "ssp-hub-sensors",
	},
	.probe = ssp_hub_sensor_probe,
	.id_table = ssp_hub_sensor_ids,
};
module_platform_driver(ssp_hub_sensor_driver);

MODULE_DESCRIPTION("Samsung sensorhub magnetometer, pressure, light, proximity and motion sensors");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_SSP_SENSORS);

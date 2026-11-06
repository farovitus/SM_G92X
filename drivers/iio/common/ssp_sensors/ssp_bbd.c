// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Galaxy S6 (zeroflte, zerolte) sensor hub.
 *
 * The SSP hub firmware runs on the Broadcom BCM4773 GNSS chip. It reaches the
 * AP through the BCM4773 bridge byte stream instead of a dedicated SPI link:
 * the bridge carries the SSP headers and frames unchanged, and reports when
 * userspace (lhd) has loaded the chip's RAM patch and the hub is running.
 */

#include <linux/bcm4773.h>
#include <linux/mfd/core.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <asm/unaligned.h>

#include "ssp.h"

#define SSP_BBD_TIMEOUT_MS	1000
#define SSP_BBD_SETTLE_MS	1000
#define SSP_BBD_PROX_HUB_ID	7

/* Hub firmware ids; the S6 has a grip sensor at 11, so the rest move up. */
static const u8 ssp_bbd_hub_ids[SSP_SENSOR_MAX] = {
	[SSP_ACCELEROMETER_SENSOR]		= 0,
	[SSP_GYROSCOPE_SENSOR]			= 1,
	[SSP_GEOMAGNETIC_UNCALIB_SENSOR]	= 2,
	[SSP_GEOMAGNETIC_RAW]			= 3,
	[SSP_GEOMAGNETIC_SENSOR]		= 4,
	[SSP_PRESSURE_SENSOR]			= 5,
	[SSP_GESTURE_SENSOR]			= 6,
	[SSP_PROXIMITY_SENSOR]			= 7,
	[SSP_TEMPERATURE_HUMIDITY_SENSOR]	= 8,
	[SSP_LIGHT_SENSOR]			= 9,
	[SSP_PROXIMITY_RAW]			= 10,
	[SSP_ORIENTATION_SENSOR]		= 12,
	[SSP_STEP_DETECTOR]			= 13,
	[SSP_SIG_MOTION_SENSOR]			= 14,
	[SSP_GYRO_UNCALIB_SENSOR]		= 15,
	[SSP_GAME_ROTATION_VECTOR]		= 16,
	[SSP_ROTATION_VECTOR]			= 17,
	[SSP_STEP_COUNTER]			= 18,
	/* The S6 heart rate sensor is an AP I2C device, not on the hub. */
	[SSP_BIO_HRM_RAW]			= SSP_NO_HUB_ID,
	[SSP_BIO_HRM_RAW_FAC]			= SSP_NO_HUB_ID,
	[SSP_BIO_HRM_LIB]			= SSP_NO_HUB_ID,
};

/* Sample sizes by hub id, without the 4-byte hub time; 0: unknown. */
static const u8 ssp_bbd_frame_sizes[] = {
	[0]  = 6,	/* accelerometer, 3 x s16 */
	[1]  = 6,	/* gyroscope, 3 x s16 */
	[2]  = 12,	/* magnetometer uncalibrated, 6 x s16 */
	[3]  = 6,	/* magnetometer raw */
	[4]  = 7,	/* magnetometer, 3 x s16 + accuracy */
	[5]  = 6,	/* pressure, s32 Pa/100 + s16 temperature */
	[6]  = 20,	/* gesture */
	[7]  = 3,	/* proximity, u8 state + u16 adc */
	[8]  = 5,	/* temperature/humidity */
	[9]  = 10,	/* light, r g b w u16 + atime + again */
	[10] = 2,	/* proximity raw */
	[11] = 9,	/* grip */
	[13] = 1,	/* step detector */
	[14] = 1,	/* significant motion */
	[15] = 12,	/* gyroscope uncalibrated */
	[16] = 17,	/* game rotation vector, 4 x s32 + accuracy */
	[17] = 17,	/* rotation vector */
	[18] = 4,	/* step counter, u32 steps since the last sample */
	[23] = 1,	/* shake */
	[24] = 12,	/* light IR */
	[26] = 1,	/* tilt */
	[27] = 1,	/* pickup */
};

static int ssp_bbd_setup(struct ssp_data *data);

static const struct ssp_sensorhub_info ssp_bbd_info = {
	.hub_ids = ssp_bbd_hub_ids,
	.frame_sizes = ssp_bbd_frame_sizes,
	.num_hub_ids = ARRAY_SIZE(ssp_bbd_frame_sizes),
	.frame_count = true,
	.ap_timestamp = true,
	.short_inst = true,
	.no_dump_check = true,
	.resync_enabled = true,
	.setup = ssp_bbd_setup,
};

static const struct mfd_cell ssp_bbd_cells[] = {
	{ .name = "ssp-accelerometer", },
	{ .name = "ssp-gyroscope", },
	{ .name = "ssp-magnetometer", },
	{ .name = "ssp-pressure", },
	{ .name = "ssp-light", },
	{ .name = "ssp-proximity", },
	{ .name = "ssp-rotation-vector", },
	{ .name = "ssp-game-rotvec", },
	{ .name = "ssp-step-counter", },
	{ .name = "ssp-step-detector", },
	{ .name = "ssp-sig-motion", },
};

static int ssp_bbd_send(struct ssp_data *data, const void *buf, size_t len)
{
	return bcm4773_send(data->hub, buf, len);
}

static int ssp_bbd_read(struct ssp_data *data, void *buf, size_t len)
{
	ssize_t ret = bcm4773_recv(data->hub, buf, len, SSP_BBD_TIMEOUT_MS);

	return ret < 0 ? ret : 0;
}

/* lhd power-cycles the chip and reloads its patch; ready() follows. */
static void ssp_bbd_reset(struct ssp_data *data)
{
	int ret = bcm4773_request_reset(data->hub);

	if (ret)
		dev_err(data->dev, "hub reset request failed %d\n", ret);
}

static const struct ssp_bus_ops ssp_bbd_bus_ops = {
	.send_header = ssp_bbd_send,
	.read = ssp_bbd_read,
	.write = ssp_bbd_send,
	.reset = ssp_bbd_reset,
};

static int ssp_bbd_setup(struct ssp_data *data)
{
	u8 pos[3] = { data->accel_position, data->accel_position,
		      data->mag_position };
	u8 glass = data->glass_type;
	u8 prox[4] = {
		data->prox_thresh[0] >> 8, data->prox_thresh[0],
		data->prox_thresh[1] >> 8, data->prox_thresh[1],
	};
	int ret;

	ret = ssp_write_cmd(data, SSP_MSG2SSP_AP_SENSOR_FORMATION, pos,
			    sizeof(pos));
	if (ret < 0)
		return ret;

	ret = ssp_write_cmd(data, SSP_MSG2SSP_AP_GLASS_TYPE, &glass, 1);
	if (ret < 0)
		return ret;

	if (memchr_inv(data->mag_matrix, 0, sizeof(data->mag_matrix))) {
		ret = ssp_write_cmd(data,
				    SSP_MSG2SSP_AP_SET_MAGNETIC_STATIC_MATRIX,
				    data->mag_matrix,
				    sizeof(data->mag_matrix));
		if (ret < 0)
			dev_err(data->dev, "magnetometer matrix failed %d\n",
				ret);
	}

	if (data->available_sensors & BIT(SSP_BBD_PROX_HUB_ID)) {
		ret = ssp_write_cmd(data, SSP_MSG2SSP_AP_SENSOR_PROXTHRESHOLD,
				    prox, sizeof(prox));
		if (ret < 0)
			dev_err(data->dev, "proximity thresholds failed %d\n",
				ret);
	}

	return 0;
}

static void ssp_bbd_rx_work(struct work_struct *work)
{
	struct ssp_data *data = container_of(work, struct ssp_data, work_rx);
	int ret;

	while (bcm4773_rx_avail(data->hub) >= SSP_HEADER_BUFFER_SIZE) {
		ret = ssp_irq_msg(data);
		/* -EPROTO: a bad message, but read whole */
		if (ret < 0 && ret != -EPROTO) {
			/* A message was cut short: resync on the next one. */
			dev_err(data->dev, "stream lost %d, flushing\n", ret);
			bcm4773_flush(data->hub);
			break;
		}
	}
}

static void ssp_bbd_ready_work(struct work_struct *work)
{
	struct ssp_data *data = container_of(work, struct ssp_data,
					     work_ready);
	int ret = -ENODEV, retries;

	if (!READ_ONCE(data->hub_ready)) {
		dev_info(data->dev, "hub stopped\n");
		ssp_enable_mcu(data, false);
		ssp_clean_pending_list(data);
		return;
	}

	/* The hub firmware needs a moment after ESW:READY, as on stock. */
	msleep(SSP_BBD_SETTLE_MS);
	if (!READ_ONCE(data->hub_ready))
		return;

	ssp_clean_pending_list(data);
	ssp_enable_mcu(data, true);

	for (retries = 0; retries < SSP_DEFAULT_RETRIES; retries++) {
		ret = ssp_initialize_mcu(data);
		if (ret >= 0 || !READ_ONCE(data->hub_ready))
			break;
		msleep(100);
	}

	if (ret < 0) {
		dev_err(data->dev, "hub init failed %d\n", ret);
		ssp_enable_mcu(data, false);
		return;
	}

	dev_info(data->dev, "hub ready, sensors 0x%08x\n",
		 data->available_sensors);

	ssp_resync_sensors(data);
	if (data->last_resume_state)
		ssp_command(data, data->last_resume_state, 0);
}

/* Bridge callbacks run in atomic context. */
static void ssp_bbd_rx(void *priv)
{
	struct ssp_data *data = priv;

	queue_work(system_highpri_wq, &data->work_rx);
}

static void ssp_bbd_ready(void *priv, bool ready)
{
	struct ssp_data *data = priv;

	WRITE_ONCE(data->hub_ready, ready);
	queue_work(system_power_efficient_wq, &data->work_ready);
}

static const struct bcm4773_client_ops ssp_bbd_client_ops = {
	.rx = ssp_bbd_rx,
	.ready = ssp_bbd_ready,
};

static int ssp_bbd_parse_dt(struct ssp_data *data)
{
	struct device *dev = data->dev;
	s32 matrix[9];
	u32 val;
	int i, ret;

	if (!device_property_read_u32(dev, "samsung,acc-position", &val))
		data->accel_position = val;
	if (!device_property_read_u32(dev, "samsung,mag-position", &val))
		data->mag_position = val;
	device_property_read_u32(dev, "samsung,glass-type", &data->glass_type);

	data->prox_thresh[0] = 2000;
	data->prox_thresh[1] = 1400;
	device_property_read_u32_array(dev, "samsung,prox-thresholds",
				       data->prox_thresh, 2);

	if (!device_property_present(dev, "samsung,mag-matrix"))
		return 0;

	ret = device_property_read_u32_array(dev, "samsung,mag-matrix",
					     (u32 *)matrix, ARRAY_SIZE(matrix));
	if (ret)
		return dev_err_probe(dev, ret, "bad samsung,mag-matrix\n");

	for (i = 0; i < ARRAY_SIZE(matrix); i++)
		put_unaligned_le16((s16)matrix[i], &data->mag_matrix[2 * i]);

	return 0;
}

static int ssp_bbd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ssp_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->hub = dev->parent;
	data->bus = &ssp_bbd_bus_ops;
	data->sensorhub_info = &ssp_bbd_info;

	ret = ssp_bbd_parse_dt(data);
	if (ret)
		return ret;

	ssp_data_init(data);
	/* Nothing reaches the hub until the bridge says it is up. */
	data->shut_down = true;
	INIT_WORK(&data->work_rx, ssp_bbd_rx_work);
	INIT_WORK(&data->work_ready, ssp_bbd_ready_work);
	platform_set_drvdata(pdev, data);

	ret = mfd_add_devices(dev, PLATFORM_DEVID_AUTO, ssp_bbd_cells,
			      ARRAY_SIZE(ssp_bbd_cells), NULL, 0, NULL);
	if (ret) {
		dev_err(dev, "mfd add devices fail\n");
		goto err_release;
	}

	/* Calls ssp_bbd_ready() at once if the hub already runs. */
	ret = bcm4773_register_client(data->hub, &ssp_bbd_client_ops, data);
	if (ret) {
		dev_err_probe(dev, ret, "cannot attach to the BCM4773\n");
		goto err_mfd;
	}

	return 0;

err_mfd:
	mfd_remove_devices(dev);
err_release:
	ssp_data_release(data);
	return ret;
}

static void ssp_bbd_remove(struct platform_device *pdev)
{
	struct ssp_data *data = platform_get_drvdata(pdev);

	mfd_remove_devices(&pdev->dev);

	bcm4773_unregister_client(data->hub);
	cancel_work_sync(&data->work_ready);
	cancel_work_sync(&data->work_rx);

	ssp_enable_mcu(data, false);
	ssp_data_release(data);
}

static const struct of_device_id ssp_bbd_of_match[] = {
	{ .compatible = "samsung,zeroflte-sensorhub" },
	{ }
};
MODULE_DEVICE_TABLE(of, ssp_bbd_of_match);

static struct platform_driver ssp_bbd_driver = {
	.probe = ssp_bbd_probe,
	.remove_new = ssp_bbd_remove,
	.driver = {
		.name = "ssp-bcm4773",
		.of_match_table = ssp_bbd_of_match,
		.pm = pm_sleep_ptr(&ssp_pm_ops),
	},
};

int ssp_bbd_register(void)
{
	return platform_driver_register(&ssp_bbd_driver);
}

void ssp_bbd_unregister(void)
{
	platform_driver_unregister(&ssp_bbd_driver);
}

MODULE_IMPORT_NS(BCM4773);

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2014, Samsung Electronics Co. Ltd. All Rights Reserved.
 */

#include <asm/unaligned.h>

#include "ssp.h"

#define SSP_DEV (data->dev)
#define SSP_GET_MESSAGE_TYPE(data) (data & (3 << SSP_RW))

/*
 * SSP -> AP Instruction
 * They tell what packet type can be expected. In the future there will
 * be less of them. BYPASS means common sensor packets with accel, gyro,
 * hrm etc. data. LIBRARY and META are mock-up's for now.
 */
#define SSP_MSG2AP_INST_BYPASS_DATA		0x37
#define SSP_MSG2AP_INST_LIBRARY_DATA		0x01
#define SSP_MSG2AP_INST_DEBUG_DATA		0x03
#define SSP_MSG2AP_INST_BIG_DATA		0x04
#define SSP_MSG2AP_INST_META_DATA		0x05
#define SSP_MSG2AP_INST_TIME_SYNC		0x06
#define SSP_MSG2AP_INST_RESET			0x07
#define SSP_MSG2AP_INST_GYRO_CAL		0x08
#define SSP_MSG2AP_GYRO_CAL_START		0x43
#define SSP_MSG2AP_GYRO_CAL_STOP		0x44
#define SSP_MSG2AP_GYRO_CAL_EVENT		0x45

#define SSP_UNIMPLEMENTED -1

struct ssp_msg_header {
	u8 cmd;
	__le16 length;
	__le16 options;
	__le32 data;
} __attribute__((__packed__));

struct ssp_msg {
	u16 length;
	u16 options;
	struct list_head list;
	struct completion *done;
	struct ssp_msg_header *h;
	char *buffer;
};

static const int ssp_offset_map[SSP_SENSOR_MAX] = {
	[SSP_ACCELEROMETER_SENSOR] =		SSP_ACCELEROMETER_SIZE +
						SSP_TIME_SIZE,
	[SSP_GYROSCOPE_SENSOR] =		SSP_GYROSCOPE_SIZE +
						SSP_TIME_SIZE,
	[SSP_GEOMAGNETIC_UNCALIB_SENSOR] =	SSP_UNIMPLEMENTED,
	[SSP_GEOMAGNETIC_RAW] =			SSP_UNIMPLEMENTED,
	[SSP_GEOMAGNETIC_SENSOR] =		SSP_UNIMPLEMENTED,
	[SSP_PRESSURE_SENSOR] =			SSP_UNIMPLEMENTED,
	[SSP_GESTURE_SENSOR] =			SSP_UNIMPLEMENTED,
	[SSP_PROXIMITY_SENSOR] =		SSP_UNIMPLEMENTED,
	[SSP_TEMPERATURE_HUMIDITY_SENSOR] =	SSP_UNIMPLEMENTED,
	[SSP_LIGHT_SENSOR] =			SSP_UNIMPLEMENTED,
	[SSP_PROXIMITY_RAW] =			SSP_UNIMPLEMENTED,
	[SSP_ORIENTATION_SENSOR] =		SSP_UNIMPLEMENTED,
	[SSP_STEP_DETECTOR] =			SSP_UNIMPLEMENTED,
	[SSP_SIG_MOTION_SENSOR] =		SSP_UNIMPLEMENTED,
	[SSP_GYRO_UNCALIB_SENSOR] =		SSP_UNIMPLEMENTED,
	[SSP_GAME_ROTATION_VECTOR] =		SSP_UNIMPLEMENTED,
	[SSP_ROTATION_VECTOR] =			SSP_UNIMPLEMENTED,
	[SSP_STEP_COUNTER] =			SSP_UNIMPLEMENTED,
	[SSP_BIO_HRM_RAW] =			SSP_BIO_HRM_RAW_SIZE +
						SSP_TIME_SIZE,
	[SSP_BIO_HRM_RAW_FAC] =			SSP_BIO_HRM_RAW_FAC_SIZE +
						SSP_TIME_SIZE,
	[SSP_BIO_HRM_LIB] =			SSP_BIO_HRM_LIB_SIZE +
						SSP_TIME_SIZE,
};

#define SSP_HEADER_SIZE		(sizeof(struct ssp_msg_header))
#define SSP_HEADER_SIZE_ALIGNED	(ALIGN(SSP_HEADER_SIZE, 4))

static struct ssp_msg *ssp_create_msg(u8 cmd, u16 len, u16 opt, u32 data)
{
	struct ssp_msg_header h;
	struct ssp_msg *msg;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return NULL;

	h.cmd = cmd;
	h.length = cpu_to_le16(len);
	h.options = cpu_to_le16(opt);
	h.data = cpu_to_le32(data);

	msg->buffer = kzalloc(SSP_HEADER_SIZE_ALIGNED + len,
			      GFP_KERNEL | GFP_DMA);
	if (!msg->buffer) {
		kfree(msg);
		return NULL;
	}

	msg->length = len;
	msg->options = opt;
	INIT_LIST_HEAD(&msg->list);

	memcpy(msg->buffer, &h, SSP_HEADER_SIZE);

	return msg;
}

/*
 * It is a bit heavy to do it this way but often the function is used to compose
 * the message from smaller chunks which are placed on the stack.  Often the
 * chunks are small so memcpy should be optimalized.
 */
static inline void ssp_fill_buffer(struct ssp_msg *m, unsigned int offset,
				   const void *src, unsigned int len)
{
	memcpy(&m->buffer[SSP_HEADER_SIZE_ALIGNED + offset], src, len);
}

static inline void ssp_get_buffer(struct ssp_msg *m, unsigned int offset,
				  void *dest, unsigned int len)
{
	memcpy(dest, &m->buffer[SSP_HEADER_SIZE_ALIGNED + offset],  len);
}

#define SSP_GET_BUFFER_AT_INDEX(m, index) \
	(m->buffer[SSP_HEADER_SIZE_ALIGNED + index])
#define SSP_SET_BUFFER_AT_INDEX(m, index, val) \
	(m->buffer[SSP_HEADER_SIZE_ALIGNED + index] = val)

static void ssp_clean_msg(struct ssp_msg *m)
{
	kfree(m->buffer);
	kfree(m);
}

static int ssp_print_mcu_debug(char *data_frame, int *data_index,
			       int received_len)
{
	int length = data_frame[(*data_index)++];

	if (length > received_len - *data_index || length <= 0) {
		ssp_dbg("[SSP]: MSG From MCU-invalid debug length(%d/%d)\n",
			length, received_len);
		return -EPROTO;
	}

	ssp_dbg("[SSP]: MSG From MCU - %s\n", &data_frame[*data_index]);

	*data_index += length;

	return 0;
}

/*
 * It was designed that way - additional lines to some kind of handshake,
 * please do not ask why - only the firmware guy can know it.
 */
static int ssp_check_lines(struct ssp_data *data, bool state)
{
	int delay_cnt = 0;

	gpiod_set_value_cansleep(data->ap_mcu_gpiod, state);

	while (gpiod_get_value_cansleep(data->mcu_ap_gpiod) != state) {
		usleep_range(3000, 3500);

		if (data->shut_down || delay_cnt++ > 500) {
			dev_err(SSP_DEV, "%s:timeout, hw ack wait fail %d\n",
				__func__, state);

			if (!state)
				gpiod_set_value_cansleep(data->ap_mcu_gpiod, 1);

			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int ssp_spi_send_header(struct ssp_data *data, const void *buf,
			       size_t len)
{
	int status;

	status = ssp_check_lines(data, false);
	if (status < 0)
		return status;

	status = spi_write(to_spi_device(data->dev), buf, len);
	if (status < 0) {
		gpiod_set_value_cansleep(data->ap_mcu_gpiod, 1);
		dev_err(SSP_DEV, "%s spi_write fail\n", __func__);
		return status;
	}

	return ssp_check_lines(data, true);
}

static int ssp_spi_read(struct ssp_data *data, void *buf, size_t len)
{
	return spi_read(to_spi_device(data->dev), buf, len);
}

static int ssp_spi_write(struct ssp_data *data, const void *buf, size_t len)
{
	return spi_write(to_spi_device(data->dev), buf, len);
}

static void ssp_spi_reset(struct ssp_data *data)
{
	gpiod_set_value(data->mcu_reset_gpiod, 0);
	usleep_range(1000, 1200);
	gpiod_set_value(data->mcu_reset_gpiod, 1);
	msleep(50);
}

const struct ssp_bus_ops ssp_spi_bus_ops = {
	.send_header = ssp_spi_send_header,
	.read = ssp_spi_read,
	.write = ssp_spi_write,
	.reset = ssp_spi_reset,
};

static int ssp_do_transfer(struct ssp_data *data, struct ssp_msg *msg,
			   struct completion *done, int timeout)
{
	int status;
	/*
	 * check if this is a short one way message or the whole transfer has
	 * second part after an interrupt
	 */
	const bool use_no_irq = msg->length == 0;

	if (data->shut_down)
		return -EPERM;

	msg->done = done;

	mutex_lock(&data->comm_lock);

	/* Queue first: the answer may arrive before send_header returns. */
	if (!use_no_irq) {
		mutex_lock(&data->pending_lock);
		list_add_tail(&msg->list, &data->pending_list);
		mutex_unlock(&data->pending_lock);
	}

	status = data->bus->send_header(data, msg->buffer, SSP_HEADER_SIZE);
	if (status < 0) {
		if (!use_no_irq) {
			mutex_lock(&data->pending_lock);
			list_del_init(&msg->list);
			mutex_unlock(&data->pending_lock);
		}
		goto _error_locked;
	}

	mutex_unlock(&data->comm_lock);

	if (!use_no_irq && done)
		if (wait_for_completion_timeout(done,
						msecs_to_jiffies(timeout)) ==
		    0) {
			/* A late reply may have taken it off the list already. */
			mutex_lock(&data->pending_lock);
			list_del_init(&msg->list);
			mutex_unlock(&data->pending_lock);

			data->timeout_cnt++;
			return -ETIMEDOUT;
		}

	return 0;

_error_locked:
	mutex_unlock(&data->comm_lock);
	data->timeout_cnt++;
	return status;
}

static inline int ssp_spi_sync_command(struct ssp_data *data,
				       struct ssp_msg *msg)
{
	return ssp_do_transfer(data, msg, NULL, 0);
}

static int ssp_spi_sync(struct ssp_data *data, struct ssp_msg *msg,
			int timeout)
{
	DECLARE_COMPLETION_ONSTACK(done);

	if (WARN_ON(!msg->length))
		return -EPERM;

	return ssp_do_transfer(data, msg, &done, timeout);
}

static int ssp_handle_big_data(struct ssp_data *data, char *dataframe, int *idx)
{
	/* mock-up, it will be changed with adding another sensor types */
	*idx += 8;
	return 0;
}

/* Galaxy S6 data frame: hub id, le16 count, count x (sample + hub time) */
static int ssp_parse_counted(struct ssp_data *data, char *frame, int *idx,
			     int len)
{
	const struct ssp_sensorhub_info *info = data->sensorhub_info;
	struct ssp_sensor_data *spd = NULL;
	struct iio_dev *indio_dev = NULL;
	unsigned int hub, size, count;
	int type;

	if (len - *idx < 3)
		return -EPROTO;

	hub = (u8)frame[(*idx)++];
	count = get_unaligned_le16(&frame[*idx]);
	*idx += 2;

	if (hub >= info->num_hub_ids || !info->frame_sizes[hub]) {
		dev_err(SSP_DEV, "unknown hub sensor %u\n", hub);
		return -EPROTO;
	}

	size = info->frame_sizes[hub] + SSP_TIME_SIZE;
	if (!count || count * size > len - *idx) {
		dev_err(SSP_DEV, "bad frame, sensor %u count %u\n", hub, count);
		return -EPROTO;
	}

	type = data->hub_to_type[hub];
	if (type >= 0)
		indio_dev = data->sensor_devs[type];
	if (indio_dev)
		spd = iio_priv(indio_dev);

	for (; count; count--, *idx += size) {
		if (!spd || !spd->process_data)
			continue;
		if (info->ap_timestamp)
			memset(&frame[*idx + size - SSP_TIME_SIZE], 0,
			       SSP_TIME_SIZE);
		spd->process_data(indio_dev, &frame[*idx], data->timestamp);
	}

	return 0;
}

static int ssp_parse_s6_inst(struct ssp_data *data, char *frame, int *idx,
			     int len, u8 inst)
{
	unsigned int skip;

	switch (inst) {
	case SSP_MSG2AP_INST_BYPASS_DATA:
		return ssp_parse_counted(data, frame, idx, len);
	case SSP_MSG2AP_INST_LIBRARY_DATA:
		if (len - *idx < 2)
			return -EPROTO;
		skip = get_unaligned_le16(&frame[*idx]) + 2;
		break;
	case SSP_MSG2AP_INST_META_DATA:
		/* flush complete: what, hub sensor id */
		skip = 2;
		break;
	case SSP_MSG2AP_INST_GYRO_CAL:
		/* new gyro offsets, 3 x s16; userspace owns calibration */
		skip = 6;
		break;
	case SSP_MSG2AP_GYRO_CAL_START:
	case SSP_MSG2AP_GYRO_CAL_STOP:
	case SSP_MSG2AP_GYRO_CAL_EVENT:
		skip = 0;
		break;
	default:
		dev_err(SSP_DEV, "unknown hub instruction 0x%02x\n", inst);
		return -EPROTO;
	}

	if (skip > len - *idx)
		return -EPROTO;
	*idx += skip;

	return 0;
}

static int ssp_parse_dataframe(struct ssp_data *data, char *dataframe, int len)
{
	const struct ssp_sensorhub_info *info = data->sensorhub_info;
	int idx, sd, ret;
	struct ssp_sensor_data *spd;
	struct iio_dev **indio_devs = data->sensor_devs;

	if (info->ap_timestamp)
		data->timestamp = ktime_get_boottime_ns();

	for (idx = 0; idx < len;) {
		u8 inst = dataframe[idx++];

		if (info->frame_count &&
		    inst != SSP_MSG2AP_INST_DEBUG_DATA &&
		    inst != SSP_MSG2AP_INST_BIG_DATA &&
		    inst != SSP_MSG2AP_INST_TIME_SYNC &&
		    inst != SSP_MSG2AP_INST_RESET) {
			ret = ssp_parse_s6_inst(data, dataframe, &idx, len,
						inst);
			if (ret)
				return ret;
			continue;
		}

		switch (inst) {
		case SSP_MSG2AP_INST_BYPASS_DATA:
			if (idx >= len)
				return -EPROTO;
			sd = dataframe[idx++];
			if (sd < 0 || sd >= SSP_SENSOR_MAX) {
				dev_err(SSP_DEV,
					"Mcu data frame1 error %d\n", sd);
				return -EPROTO;
			}

			if (indio_devs[sd]) {
				spd = iio_priv(indio_devs[sd]);
				if (spd->process_data) {
					if (idx >= len)
						return -EPROTO;
					spd->process_data(indio_devs[sd],
							  &dataframe[idx],
							  data->timestamp);
				}
			} else {
				dev_err(SSP_DEV, "no client for frame\n");
			}

			idx += ssp_offset_map[sd];
			break;
		case SSP_MSG2AP_INST_DEBUG_DATA:
			if (idx >= len)
				return -EPROTO;
			sd = ssp_print_mcu_debug(dataframe, &idx, len);
			if (sd) {
				dev_err(SSP_DEV,
					"Mcu data frame3 error %d\n", sd);
				return sd;
			}
			break;
		case SSP_MSG2AP_INST_LIBRARY_DATA:
			idx += len;
			break;
		case SSP_MSG2AP_INST_BIG_DATA:
			ssp_handle_big_data(data, dataframe, &idx);
			break;
		case SSP_MSG2AP_INST_TIME_SYNC:
			data->time_syncing = true;
			break;
		case SSP_MSG2AP_INST_RESET:
			ssp_queue_ssp_refresh_task(data, 0);
			break;
		}
	}

	if (data->time_syncing && !info->ap_timestamp)
		data->timestamp = ktime_get_real_ns();

	return 0;
}

/* threaded irq */
int ssp_irq_msg(struct ssp_data *data)
{
	char *buffer;
	u8 msg_type;
	int ret;
	u16 length, msg_options;
	struct ssp_msg *msg = NULL, *iter, *n;

	ret = data->bus->read(data, data->header_buffer, SSP_HEADER_BUFFER_SIZE);
	if (ret < 0) {
		dev_err(SSP_DEV, "header read fail\n");
		return ret;
	}

	length = le16_to_cpu(data->header_buffer[1]);
	msg_options = le16_to_cpu(data->header_buffer[0]);
	msg_type = SSP_GET_MESSAGE_TYPE(msg_options);

	if (length == 0 && msg_type != SSP_AP2HUB_WRITE) {
		dev_err(SSP_DEV, "length received from mcu is 0\n");
		return -EINVAL;
	}

	switch (msg_type) {
	case SSP_AP2HUB_READ:
	case SSP_AP2HUB_WRITE:
		/*
		 * this is a small list, a few elements - the packets can be
		 * received with no order
		 */
		mutex_lock(&data->pending_lock);
		list_for_each_entry_safe(iter, n, &data->pending_list, list) {
			if (iter->options == msg_options) {
				list_del_init(&iter->list);
				msg = iter;
				break;
			}
		}

		if (!msg) {
			/*
			 * here can be implemented dead messages handling
			 * but the slave should not send such ones - it is to
			 * check but let's handle this
			 */
			dev_err(SSP_DEV, "No match error %x\n", msg_options);

			/* On the S6 a write request carries no payload. */
			if (msg_type == SSP_AP2HUB_WRITE &&
			    data->sensorhub_info->frame_count) {
				ret = -EPROTO;
				goto _unlock;
			}

			buffer = kmalloc(length, GFP_KERNEL | GFP_DMA);
			if (!buffer) {
				ret = -ENOMEM;
				goto _unlock;
			}

			/* got dead packet so it is always an error */
			ret = data->bus->read(data, buffer, length);
			if (ret >= 0)
				ret = -EPROTO;

			kfree(buffer);

			goto _unlock;
		}

		if (msg_type == SSP_AP2HUB_READ)
			ret = data->bus->read(data,
					      &msg->buffer[SSP_HEADER_SIZE_ALIGNED],
					      msg->length);

		if (msg_type == SSP_AP2HUB_WRITE) {
			ret = data->bus->write(data,
					       &msg->buffer[SSP_HEADER_SIZE_ALIGNED],
					       msg->length);
			if (msg_options & SSP_AP2HUB_RETURN) {
				msg->options =
					SSP_AP2HUB_READ | SSP_AP2HUB_RETURN;
				msg->length = 1;

				list_add_tail(&msg->list, &data->pending_list);
				goto _unlock;
			}
		}

		if (msg->done)
			if (!completion_done(msg->done))
				complete(msg->done);
_unlock:
		mutex_unlock(&data->pending_lock);
		break;
	case SSP_HUB2AP_WRITE:
		buffer = kzalloc(length, GFP_KERNEL | GFP_DMA);
		if (!buffer)
			return -ENOMEM;

		ret = data->bus->read(data, buffer, length);
		if (ret < 0) {
			dev_err(SSP_DEV, "frame read fail\n");
			kfree(buffer);
			break;
		}

		ret = ssp_parse_dataframe(data, buffer, length);

		kfree(buffer);
		break;

	default:
		dev_err(SSP_DEV, "unknown msg type\n");
		return -EPROTO;
	}

	return ret;
}

void ssp_clean_pending_list(struct ssp_data *data)
{
	struct ssp_msg *msg, *n;

	mutex_lock(&data->pending_lock);
	list_for_each_entry_safe(msg, n, &data->pending_list, list) {
		list_del_init(&msg->list);

		if (msg->done)
			if (!completion_done(msg->done))
				complete(msg->done);
	}
	mutex_unlock(&data->pending_lock);
}

int ssp_command(struct ssp_data *data, char command, int arg)
{
	int ret;
	struct ssp_msg *msg;

	msg = ssp_create_msg(command, 0, SSP_AP2HUB_WRITE, arg);
	if (!msg)
		return -ENOMEM;

	ssp_dbg("%s - command 0x%x %d\n", __func__, command, arg);

	ret = ssp_spi_sync_command(data, msg);
	ssp_clean_msg(msg);

	return ret;
}

/**
 * ssp_hub_id() - hub firmware id of a sensor type
 * @data:	sensorhub structure
 * @type:	SSP sensor type
 *
 * Returns the id or -ENODEV if this hub has no such sensor.
 */
int ssp_hub_id(struct ssp_data *data, enum ssp_sensor_type type)
{
	const u8 *ids = data->sensorhub_info->hub_ids;

	if (!ids)
		return type;

	return ids[type] == SSP_NO_HUB_ID ? -ENODEV : ids[type];
}

int ssp_send_instruction(struct ssp_data *data, u8 inst, u8 sensor_type,
			 u8 *send_buf, u8 length)
{
	int ret, hub_id;
	u8 id;
	struct ssp_msg *msg;

	hub_id = ssp_hub_id(data, sensor_type);
	if (hub_id < 0)
		return hub_id;
	id = hub_id;

	if (data->fw_dl_state == SSP_FW_DL_STATE_DOWNLOADING) {
		dev_err(SSP_DEV, "%s - Skip Inst! DL state = %d\n",
			__func__, data->fw_dl_state);
		return -EBUSY;
	} else if (!(data->available_sensors & BIT(id)) &&
		   (inst <= SSP_MSG2SSP_INST_CHANGE_DELAY)) {
		dev_err(SSP_DEV, "%s - Bypass Inst Skip! - %u\n",
			__func__, sensor_type);
		return -EIO; /* just fail */
	}

	msg = ssp_create_msg(inst, length + (data->sensorhub_info->short_inst ?
					     1 : 2),
			     SSP_AP2HUB_WRITE, 0);
	if (!msg)
		return -ENOMEM;

	ssp_fill_buffer(msg, 0, &id, 1);
	ssp_fill_buffer(msg, 1, send_buf, length);

	ssp_dbg("%s - Inst = 0x%x, Sensor Type = 0x%x, data = %u\n",
		__func__, inst, sensor_type, send_buf[1]);

	ret = ssp_spi_sync(data, msg, 1000);
	ssp_clean_msg(msg);

	return ret;
}

int ssp_get_chipid(struct ssp_data *data)
{
	int ret;
	char buffer;
	struct ssp_msg *msg;

	msg = ssp_create_msg(SSP_MSG2SSP_AP_WHOAMI, 1, SSP_AP2HUB_READ, 0);
	if (!msg)
		return -ENOMEM;

	ret = ssp_spi_sync(data, msg, 1000);

	buffer = SSP_GET_BUFFER_AT_INDEX(msg, 0);

	ssp_clean_msg(msg);

	return ret < 0 ? ret : buffer;
}

/**
 * ssp_write_cmd() - send a command with a payload the hub reads
 * @data:	sensorhub structure
 * @cmd:	SSP_MSG2SSP_AP_* command
 * @buf:	payload
 * @len:	payload length, not 0
 *
 * Returns 0 or negative value in case of error
 */
int ssp_write_cmd(struct ssp_data *data, u8 cmd, const void *buf, u8 len)
{
	struct ssp_msg *msg;
	int ret;

	msg = ssp_create_msg(cmd, len, SSP_AP2HUB_WRITE, 0);
	if (!msg)
		return -ENOMEM;

	ssp_fill_buffer(msg, 0, buf, len);

	ret = ssp_spi_sync(data, msg, 1000);
	ssp_clean_msg(msg);

	return ret;
}

int ssp_set_magnetic_matrix(struct ssp_data *data)
{
	int ret;
	struct ssp_msg *msg;

	if (!data->sensorhub_info->mag_length)
		return 0;

	msg = ssp_create_msg(SSP_MSG2SSP_AP_SET_MAGNETIC_STATIC_MATRIX,
			     data->sensorhub_info->mag_length, SSP_AP2HUB_WRITE,
			     0);
	if (!msg)
		return -ENOMEM;

	ssp_fill_buffer(msg, 0, data->sensorhub_info->mag_table,
			data->sensorhub_info->mag_length);

	ret = ssp_spi_sync(data, msg, 1000);
	ssp_clean_msg(msg);

	return ret;
}

unsigned int ssp_get_sensor_scanning_info(struct ssp_data *data)
{
	int ret;
	__le32 result;
	u32 cpu_result = 0;

	struct ssp_msg *msg = ssp_create_msg(SSP_MSG2SSP_AP_SENSOR_SCANNING, 4,
					     SSP_AP2HUB_READ, 0);
	if (!msg)
		return 0;

	ret = ssp_spi_sync(data, msg, 1000);
	if (ret < 0) {
		dev_err(SSP_DEV, "%s - spi read fail %d\n", __func__, ret);
		goto _exit;
	}

	ssp_get_buffer(msg, 0, &result, 4);
	cpu_result = le32_to_cpu(result);

	dev_info(SSP_DEV, "%s state: 0x%08x\n", __func__, cpu_result);

_exit:
	ssp_clean_msg(msg);
	return cpu_result;
}

unsigned int ssp_get_firmware_rev(struct ssp_data *data)
{
	int ret;
	__le32 result;

	struct ssp_msg *msg = ssp_create_msg(SSP_MSG2SSP_AP_FIRMWARE_REV, 4,
					     SSP_AP2HUB_READ, 0);
	if (!msg)
		return SSP_INVALID_REVISION;

	ret = ssp_spi_sync(data, msg, 1000);
	if (ret < 0) {
		dev_err(SSP_DEV, "%s - transfer fail %d\n", __func__, ret);
		ret = SSP_INVALID_REVISION;
		goto _exit;
	}

	ssp_get_buffer(msg, 0, &result, 4);
	ret = le32_to_cpu(result);

_exit:
	ssp_clean_msg(msg);
	return ret;
}

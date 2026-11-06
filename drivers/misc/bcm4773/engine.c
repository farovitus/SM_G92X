// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom BCM4773 bridge engine: connects the Broadcom transport layer to
 * the SSI bus, to lhd's character devices and to the sensor hub client.
 *
 * Everything that touches the bridge runs under the engine mutex. The
 * transport timer is a delayed work that ticks the transport layer itself;
 * stock woke lhd with "BBD:tick" so that lhd's next read did it.
 */

#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "bbd_internal.h"
#include "transport/rpc_codec_c.h"
#include "transport/transport_layer_c.h"
#include "transport/bbd_packet_layer_c.h"
#include "utils/stream_codec_c.h"

bool BbdEngine_Lock(int from)
{
	struct bbd_device *bbd = gpbbd_dev;

	if (!bbd || !bbd->bbd_engine.constructed)
		return false;
	mutex_lock(&bbd->bbd_engine.lock);
	return true;
}

void BbdEngine_Unlock(void)
{
	mutex_unlock(&gpbbd_dev->bbd_engine.lock);
}

static struct bbd_device *cb_to_bbd(struct sITransportLayerCb *cb)
{
	return cb->m_callbackData;
}

static void bbd_tick_work(struct work_struct *work)
{
	struct bbd_device *bbd = container_of(to_delayed_work(work),
					      struct bbd_device, tick_work);

	mutex_lock(&bbd->bbd_engine.lock);
	if (bbd->bbd_engine.open)
		TransportLayer_Tick(&bbd->bbd_engine.bridge.m_otTL);
	mutex_unlock(&bbd->bbd_engine.lock);
}

static void bbd_cb_timer_set(struct sITransportLayerCb *cb, long ms)
{
	struct bbd_device *bbd = cb_to_bbd(cb);

	if (ms > 0)
		mod_delayed_work(system_wq, &bbd->tick_work,
				 msecs_to_jiffies(ms));
	else
		cancel_delayed_work(&bbd->tick_work);
}

static unsigned long bbd_cb_time_ms(struct sITransportLayerCb *cb)
{
	return jiffies_to_msecs(jiffies);
}

static void bbd_cb_data_to_send(struct sITransportLayerCb *cb,
				unsigned char *data, unsigned long len)
{
	struct bbd_device *bbd = cb_to_bbd(cb);

	bbd_ssi_send(bbd->ssi, data, len);
}

static void bbd_cb_comm_error(struct sITransportLayerCb *cb)
{
	bbd_on_read(BBD_MINOR_CONTROL, BBD_CTRL_PL_ON_COMMUNICATION_ERROR,
		    sizeof(BBD_CTRL_PL_ON_COMMUNICATION_ERROR));
}

static void bbd_cb_packet_received(struct sITransportLayerCb *cb,
				   unsigned char *data, unsigned short size)
{
	struct bbd_device *bbd = cb_to_bbd(cb);

	BbdBridge_OnPacketReceived(&bbd->bbd_engine.bridge.m_otTL, data, size);
}

static void bbd_cb_exception(struct sITransportLayerCb *cb,
			     const char filename[], unsigned int line)
{
	const char *base = kbasename(filename);
	char buf[80];

	pr_debug("bcm4773: transport exception at %s:%u\n", base, line);
	snprintf(buf, sizeof(buf), BBD_CTRL_PL_ON_EXCEPTION "%s/%u", base, line);
	bbd_on_read(BBD_MINOR_CONTROL, buf, strlen(buf) + 1);
}

static void bbd_cb_remote_sync_complete(struct sITransportLayerCb *cb)
{
	bbd_on_read(BBD_MINOR_CONTROL, BBD_CTRL_PL_REMOTE_SYNC_COMPLETE,
		    sizeof(BBD_CTRL_PL_REMOTE_SYNC_COMPLETE));
}

static void bbd_cb_reliable_ack(struct sITransportLayerCb *cb,
				struct sBbdReliableTransaction *trans)
{
	bbd_on_read(BBD_MINOR_RELIABLE, (const unsigned char *)trans,
		    sizeof(*trans));
}

static void bbd_cb_data_to_lhd(struct sITransportLayerCb *cb,
			       const unsigned char *data, unsigned short size)
{
	bbd_on_read(BBD_MINOR_PACKET, data, size);
}

/*
 * IRpcSensorResponse::Data carries one chunk of the SSP byte stream.
 * BbdBridge_OnRpcReceived() passes the decoder itself, not its data.
 */
static bool bbd_sensor_rpc(void *priv, unsigned short id,
			   struct sStreamDecoder *str)
{
	struct sRpcDecoder *dec = priv;
	struct bbd_device *bbd = dec->data;
	unsigned short size;
	unsigned char *data;

	if (id != RPC_DEFINITION(IRpcSensorResponse, Data))
		return false;

	size = StreamDecoder_GetU16(str);
	data = StreamDecoder_GetBuffer(str, size);
	if (!StreamCodec_Fail(str))
		bbd_sensor_rx(bbd, data, size);
	return true;
}

/* Called with the engine lock held. */
static void bbd_engine_setup_locked(struct bbd_device *bbd)
{
	struct BbdEngine *eng = &bbd->bbd_engine;
	struct sITransportLayerCb *cb = &eng->callback;

	cb->m_callbackData = bbd;
	cb->OnTimerSet = bbd_cb_timer_set;
	cb->OnTimeMsRequest = bbd_cb_time_ms;
	cb->OnDataToSend = bbd_cb_data_to_send;
	cb->OnCommunicationError = bbd_cb_comm_error;
	cb->OnPacketReceived = bbd_cb_packet_received;
	cb->OnException = bbd_cb_exception;
	cb->OnRemoteSyncComplete = bbd_cb_remote_sync_complete;
	cb->OnReliableAck = bbd_cb_reliable_ack;
	cb->OnDataToLhd = bbd_cb_data_to_lhd;

	eng->m_rpcSensor.cb = bbd_sensor_rpc;
	eng->m_rpcSensor.data = bbd;
	eng->m_rpcSensor.m_pNext = NULL;

	if (eng->open)
		BbdBridge_RegisterRpcDecoder(&eng->bridge,
					     bbd->shmd ? &eng->m_rpcSensor : NULL);
}

void bbd_engine_setup(struct bbd_device *bbd)
{
	mutex_lock(&bbd->bbd_engine.lock);
	bbd_engine_setup_locked(bbd);
	mutex_unlock(&bbd->bbd_engine.lock);
}

/* lhd opened /dev/bbd_packet: start a fresh transport. */
void bbd_engine_open(struct bbd_device *bbd)
{
	struct BbdEngine *eng = &bbd->bbd_engine;

	mutex_lock(&eng->lock);
	if (!eng->open) {
		bbd_engine_setup_locked(bbd);
		BbdBridge_BbdBridge(&eng->bridge, &eng->callback);
		eng->open = true;
		bbd_engine_setup_locked(bbd);
	}
	mutex_unlock(&eng->lock);
}

void bbd_engine_close(struct bbd_device *bbd)
{
	struct BbdEngine *eng = &bbd->bbd_engine;
	bool was_open;

	mutex_lock(&eng->lock);
	was_open = eng->open;
	if (was_open) {
		eng->open = false;
		BbdBridge_dtor(&eng->bridge);
	}
	mutex_unlock(&eng->lock);

	if (was_open)
		cancel_delayed_work_sync(&bbd->tick_work);
}

void bbd_engine_init(struct bbd_device *bbd)
{
	mutex_init(&bbd->bbd_engine.lock);
	INIT_DELAYED_WORK(&bbd->tick_work, bbd_tick_work);
	bbd->bbd_engine.constructed = true;
}

void bbd_engine_exit(struct bbd_device *bbd)
{
	bbd_engine_close(bbd);
	bbd->bbd_engine.constructed = false;
}

int bbd_engine_send_sensor(struct bbd_device *bbd, const void *buf,
			   size_t len)
{
	if (len > MAX_RPC_PAYLOAD - sizeof(u16))
		return -EMSGSIZE;

	/* Takes the engine lock and fails while the transport is closed. */
	if (!BbdBridge_SendSensorData(&bbd->bbd_engine.bridge,
				      (unsigned char *)buf, len))
		return -EIO;
	return 0;
}

/* Bytes read from the hub over SSI. */
void bbd_engine_rx(struct bbd_device *bbd, const u8 *buf, size_t len)
{
	BbdBridge_SetData(&bbd->bbd_engine.bridge, buf, len);
}

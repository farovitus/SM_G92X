/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Engine that drives the Broadcom bridge. The imported bridge code needs
 * only the lock helpers and the open/constructed flags.
 */
#ifndef BBD_ENGINE_H_
#define BBD_ENGINE_H_

#include <linux/mutex.h>

#include "bbd_bridge_c.h"

struct BbdEngine {
	struct BbdBridge bridge;
	struct mutex lock;
	struct sRpcDecoder m_rpcSensor;
	struct sITransportLayerCb callback;
	bool constructed;
	bool open;
};

bool BbdEngine_Lock(int from);
void BbdEngine_Unlock(void);

#endif /* BBD_ENGINE_H_ */

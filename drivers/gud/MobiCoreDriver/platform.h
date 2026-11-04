/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos 7420 platform settings for the t-base driver. The SSIQ interrupt
 * (GIC SPI 214) comes from the device tree.
 */
#ifndef _MC_DRV_PLATFORM_H_
#define _MC_DRV_PLATFORM_H_

#define MC_MEM_TRACES

#define MC_VM_UNMAP

#define MC_FASTCALL_WORKER_THREAD

#define LPAE_SUPPORT

/* t-base 302A takes the AArch32 SMC function IDs. */
#define MC_AARCH32_FC

#define DUMP_TBASE_HALT_STATUS

#endif /* _MC_DRV_PLATFORM_H_ */

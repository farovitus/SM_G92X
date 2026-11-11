// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos7 (Exynos 7420) system sleep.
 *
 * The secure firmware implements PSCI 0.1 with function IDs from DT, so it
 * has no SYSTEM_SUSPEND and the PSCI core offers only suspend-to-idle.
 * Samsung's firmware enters system sleep from CPU_SUSPEND instead, issued
 * by the last online CPU with affinity level 3 (power_state 0x03000000)
 * and the generic cpu_resume entry point. Offer that as "deep" sleep.
 *
 * Before the call, as Samsung's kernel did: set the PMU wakeup masks (the
 * pin controller has already written EINT_WAKEUP_MASK from the enabled
 * wake interrupts) and tell the firmware through the non-secure SYSRAM
 * INFORM words where to resume and that this is a sleep.
 *
 * The PMU is then set up for SLEEP as Samsung's pmu_cal_sys_exynos7420.c
 * did: every block's sys_pwr register takes its SLEEP value, the central
 * sequencers are enabled, and the sequencer steps are reordered for the
 * DRAM self-refresh (CKE). The firmware's CPU_SUSPEND then lets the
 * sequencer take the SoC down. A wakeup event brings the boot CPU back
 * through the boot ROM and the firmware to INFORM0.
 *
 * If the sequencer never ran (an event arrived before the power-down),
 * CENTRAL_SEQ_CONFIGURATION still has its PWR_CFG bit clear: that is an
 * early wake, and only the sequencer is restored.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>
#include <linux/psci.h>
#include <linux/soc/samsung/exynos-pmu.h>
#include <linux/suspend.h>
#include <asm/memory.h>
#include <asm/suspend.h>

#include "exynos-pmu.h"

#define EXYNOS7_PSCI_SYSTEM_SLEEP	0x03000000

/* PMU */
#define EXYNOS7_WAKEUP_STAT		0x0600
#define EXYNOS7_WAKEUP_MASK		0x0610
#define EXYNOS7_WAKEUP_MASK2		0x0614
#define EXYNOS7_WAKEUP_MASK3		0x0618
/* Sleep: every internal source masked, event monitor off (bit 31 clear). */
#define EXYNOS7_SLEEP_WAKEUP_MASK	0x7fffffff
#define EXYNOS7_SLEEP_WAKEUP_MASK2	0xffff0000
#define EXYNOS7_SLEEP_WAKEUP_MASK3	0xfffe0000

#define EXYNOS7_CENTRAL_SEQ_CONFIGURATION	0x0200
#define EXYNOS7_CENTRAL_SEQ_MIF_CONFIGURATION	0x0240
#define EXYNOS7_CENTRALSEQ_PWR_CFG		BIT(16)
#define EXYNOS7_SW_MTUP(n)			(0x5108 + (n) * 4)
#define EXYNOS7_SW_MTUP_MIF(n)			(0x5308 + (n) * 4)
#define EXYNOS7_SW_MTUP_STEPS			13
#define EXYNOS7_PMU_DEBUG			0x0a00
#define EXYNOS7_CLKOUT_DISABLE			BIT(0)
#define EXYNOS7_XXTI_SYS_PWR_REG		0x1284

/* ATLAS cluster low-power interface */
#define EXYNOS7_LPI_ATLAS_BASE			0x11820000
#define EXYNOS7_LPI_MASK_ATLAS_ASYNCBRIDGE	0x0020
#define EXYNOS7_ASATBZ_MSTIF_ATLAS_AUD		BIT(5)
#define EXYNOS7_ASATBZ_MSTIF_ATLAS_CAM1		BIT(4)

struct exynos7_pmu_sleep {
	u16 offset;
	u32 value;
};

/* SLEEP column of Samsung's exynos7420_pmu_config and _option tables. */
static const struct exynos7_pmu_sleep exynos7_pmu_sleep[] = {
	{ 0x1000, 0x00000008 },	/* ATLAS_CPU0_SYS_PWR_REG */
	{ 0x1004, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU0_LOCAL_SYS_PWR_REG */
	{ 0x1008, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU0_CENTRAL_SYS_PWR_REG */
	{ 0x100c, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU0_CPUSEQUENCER_SYS_PWR_REG */
	{ 0x1010, 0x00000008 },	/* ATLAS_CPU1_SYS_PWR_REG */
	{ 0x1014, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU1_LOCAL_SYS_PWR_REG */
	{ 0x1018, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU1_CENTRAL_SYS_PWR_REG */
	{ 0x101c, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU1_CPUSEQUENCER_SYS_PWR_REG */
	{ 0x1020, 0x00000008 },	/* ATLAS_CPU2_SYS_PWR_REG */
	{ 0x1024, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU2_LOCAL_SYS_PWR_REG */
	{ 0x1028, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU2_CENTRAL_SYS_PWR_REG */
	{ 0x102c, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU2_CPUSEQUENCER_SYS_PWR_REG */
	{ 0x1030, 0x00000008 },	/* ATLAS_CPU3_SYS_PWR_REG */
	{ 0x1034, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU3_LOCAL_SYS_PWR_REG */
	{ 0x1038, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU3_CENTRAL_SYS_PWR_REG */
	{ 0x103c, 0x00000000 },	/* DIS_IRQ_ATLAS_CPU3_CPUSEQUENCER_SYS_PWR_REG */
	{ 0x1040, 0x00000008 },	/* APOLLO_CPU0_SYS_PWR_REG */
	{ 0x1044, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU0_LOCAL_SYS_PWR_REG */
	{ 0x1048, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU0_CENTRAL_SYS_PWR_REG */
	{ 0x104c, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU0_CPUSEQUENCER_SYS_PWR_REG */
	{ 0x1050, 0x00000008 },	/* APOLLO_CPU1_SYS_PWR_REG */
	{ 0x1054, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU1_LOCAL_SYS_PWR_REG */
	{ 0x1058, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU1_CENTRAL_SYS_PWR_REG */
	{ 0x105c, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU1_CPUSEQUENCER_SYS_PWR_REG */
	{ 0x1060, 0x00000008 },	/* APOLLO_CPU2_SYS_PWR_REG */
	{ 0x1064, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU2_LOCAL_SYS_PWR_REG */
	{ 0x1068, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU2_CENTRAL_SYS_PWR_REG */
	{ 0x106c, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU2_CPUSEQUENCER_SYS_PWR_REG */
	{ 0x1070, 0x00000008 },	/* APOLLO_CPU3_SYS_PWR_REG */
	{ 0x1074, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU3_LOCAL_SYS_PWR_REG */
	{ 0x1078, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU3_CENTRAL_SYS_PWR_REG */
	{ 0x107c, 0x00000000 },	/* DIS_IRQ_APOLLO_CPU3_CPUSEQUENCER_SYS_PWR_REG */
	{ 0x1080, 0x00000008 },	/* ATLAS_NONCPU_SYS_PWR_REG */
	{ 0x1084, 0x00000008 },	/* APOLLO_NONCPU_SYS_PWR_REG */
	{ 0x1088, 0x00000002 },	/* ATLAS_DBG_SYS_PWR_REG */
	{ 0x10a0, 0x00000000 },	/* CORTEXM3_APM_SYS_PWR_REG */
	{ 0x10b0, 0x00000000 },	/* A7IS_SYS_PWR_REG */
	{ 0x10b4, 0x00000000 },	/* DIS_IRQ_A7IS_LOCAL_SYS_PWR_REG */
	{ 0x10b8, 0x00000000 },	/* DIS_IRQ_A7IS_CENTRAL_SYS_PWR_REG */
	{ 0x10c0, 0x00000007 },	/* ATLAS_L2_SYS_PWR_REG */
	{ 0x10c4, 0x00000007 },	/* APOLLO_L2_SYS_PWR_REG */
	{ 0x1100, 0x00000000 },	/* CLKSTOP_CMU_TOP_SYS_PWR_REG */
	{ 0x1104, 0x00000000 },	/* CLKRUN_CMU_TOP_SYS_PWR_REG */
	{ 0x110c, 0x00000000 },	/* RESET_CMU_TOP_SYS_PWR_REG */
	{ 0x111c, 0x00000000 },	/* RESET_CPUCLKSTOP_SYS_PWR_REG */
	{ 0x1120, 0x00000000 },	/* CLKSTOP_CMU_MIF_SYS_PWR_REG */
	{ 0x1124, 0x00000000 },	/* CLKRUN_CMU_MIF_SYS_PWR_REG */
	{ 0x112c, 0x00000000 },	/* RESET_CMU_MIF_SYS_PWR_REG */
	{ 0x1138, 0x00000001 },	/* DDRPHY_DLLLOCK_SYS_PWR_REG */
	{ 0x1140, 0x00000000 },	/* DISABLE_PLL_CMU_TOP_SYS_PWR_REG */
	{ 0x1160, 0x00000000 },	/* DISABLE_PLL_CMU_MIF_SYS_PWR_REG */
	{ 0x1180, 0x00000000 },	/* TOP_BUS_SYS_PWR_REG */
	{ 0x1184, 0x00000001 },	/* TOP_RETENTION_SYS_PWR_REG */
	{ 0x1188, 0x00000003 },	/* TOP_PWR_SYS_PWR_REG */
	{ 0x1190, 0x00000000 },	/* TOP_BUS_MIF_SYS_PWR_REG */
	{ 0x1194, 0x00000001 },	/* TOP_RETENTION_MIF_SYS_PWR_REG */
	{ 0x1198, 0x00000003 },	/* TOP_PWR_MIF_SYS_PWR_REG */
	{ 0x119c, 0x00000001 },	/* RET_OSCCLK_GATE_SYS_PWR_REG */
	{ 0x11a0, 0x00000000 },	/* LOGIC_RESET_SYS_PWR_REG */
	{ 0x11a4, 0x00000001 },	/* OSCCLK_GATE_SYS_PWR_REG */
	{ 0x11a8, 0x00000000 },	/* SLEEP_RESET_SYS_PWR_REG */
	{ 0x11b0, 0x00000000 },	/* LOGIC_RESET_MIF_SYS_PWR_REG */
	{ 0x11b4, 0x00000001 },	/* OSCCLK_GATE_MIF_SYS_PWR_REG */
	{ 0x11b8, 0x00000000 },	/* SLEEP_RESET_MIF_SYS_PWR_REG */
	{ 0x11bc, 0x00000001 },	/* RET_OSCCLK_GATE_MIF_SYS_PWR_REG */
	{ 0x11c0, 0x00000000 },	/* MEMORY_TOP_SYS_PWR_REG */
	{ 0x11c4, 0x00000000 },	/* MEMORY_TOP_ALV_SYS_PWR_REG */
	{ 0x1200, 0x00000000 },	/* PAD_RETENTION_LPDDR4_SYS_PWR_REG */
	{ 0x1204, 0x00000000 },	/* PAD_RETENTION_AUD_SYS_PWR_REG */
	{ 0x1208, 0x00000000 },	/* PAD_RETENTION_JTAG_SYS_PWR_REG */
	{ 0x120c, 0x00000000 },	/* PAD_RETENTION_BUS0_TOP_SYS_PWR_REG */
	{ 0x1218, 0x00000000 },	/* PAD_RETENTION_MMC2_SYS_PWR_REG */
	{ 0x1220, 0x00000000 },	/* PAD_RETENTION_TOP_SYS_PWR_REG */
	{ 0x1224, 0x00000000 },	/* PAD_RETENTION_UART_SYS_PWR_REG */
	{ 0x1228, 0x00000000 },	/* PAD_RETENTION_MMC0_SYS_PWR_REG */
	{ 0x122c, 0x00000000 },	/* PAD_RETENTION_MMC1_SYS_PWR_REG */
	{ 0x1230, 0x00000000 },	/* PAD_RETENTION_EBIA_SYS_PWR_REG */
	{ 0x1234, 0x00000000 },	/* PAD_RETENTION_EBIB_SYS_PWR_REG */
	{ 0x1238, 0x00000000 },	/* PAD_RETENTION_SPI_SYS_PWR_REG */
	{ 0x123c, 0x00000000 },	/* PAD_RETENTION_MIF_SYS_PWR_REG */
	{ 0x1240, 0x00000000 },	/* PAD_ISOLATION_SYS_PWR_REG */
	{ 0x1244, 0x00000000 },	/* PAD_RETENTION_LLI_SYS_PWR_REG */
	{ 0x1248, 0x00000000 },	/* PAD_RETENTION_UFS_SYS_PWR_REG */
	{ 0x1250, 0x00000000 },	/* PAD_ISOLATION_MIF_SYS_PWR_REG */
	{ 0x1254, 0x00000000 },	/* PAD_RETENTION_FSYSGENIO_SYS_PWR_REG */
	{ 0x1258, 0x00000000 },	/* PAD_RETENTION_BOOTLDO_0_SYS_PWR_REG */
	{ 0x125c, 0x00000000 },	/* PAD_RETENTION_BOOTLDO_1_SYS_PWR_REG */
	{ 0x1260, 0x00000000 },	/* PAD_ALV_SEL_SYS_PWR_REG */
	{ 0x1284, 0x00000000 },	/* XXTI_SYS_PWR_REG */
	{ 0x1288, 0x00000000 },	/* XXTI26_SYS_PWR_REG */
	{ 0x12c0, 0x00000000 },	/* EXT_REGULATOR_SYS_PWR_REG */
	{ 0x1300, 0x00000000 },	/* GPIO_MODE_SYS_PWR_REG */
	{ 0x1304, 0x00000000 },	/* GPIO_MODE_FSYS0_SYS_PWR_REG */
	{ 0x1308, 0x00000000 },	/* GPIO_MODE_FSYS1_SYS_PWR_REG */
	{ 0x130c, 0x00000000 },	/* GPIO_MODE_BUS0_SYS_PWR_REG */
	{ 0x1320, 0x00000000 },	/* GPIO_MODE_MIF_SYS_PWR_REG */
	{ 0x1340, 0x00000000 },	/* GPIO_MODE_AUD_SYS_PWR_REG */
	{ 0x1350, 0x00000000 },	/* ATLAS_SYS_PWR_REG */
	{ 0x1354, 0x00000000 },	/* APOLLO_SYS_PWR_REG */
	{ 0x1358, 0x00000000 },	/* CLKRUN_CMU_ATLAS_SYS_PWR_REG */
	{ 0x135c, 0x00000000 },	/* CLKRUN_CMU_APOLLO_SYS_PWR_REG */
	{ 0x1360, 0x00000000 },	/* CLKSTOP_CMU_ATLAS_SYS_PWR_REG */
	{ 0x1364, 0x00000000 },	/* CLKSTOP_CMU_APOLLO_SYS_PWR_REG */
	{ 0x1368, 0x00000000 },	/* DISABLE_PLL_CMU_ATLAS_SYS_PWR_REG */
	{ 0x136c, 0x00000000 },	/* DISABLE_PLL_CMU_APOLLO_SYS_PWR_REG */
	{ 0x1370, 0x00000000 },	/* RESET_LOGIC_ATLAS_SYS_PWR_REG */
	{ 0x1374, 0x00000000 },	/* RESET_LOGIC_APOLLO_SYS_PWR_REG */
	{ 0x1378, 0x00000000 },	/* MEMORY_ATLAS_SYS_PWR_REG */
	{ 0x137c, 0x00000000 },	/* RESET_CMU_ATLAS_SYS_PWR_REG */
	{ 0x1380, 0x00000000 },	/* RESET_CMU_APOLLO_SYS_PWR_REG */
	{ 0x1384, 0x00000000 },	/* RESET_SLEEP_ATLAS_SYS_PWR_REG */
	{ 0x1388, 0x00000000 },	/* RESET_SLEEP_APOLLO_SYS_PWR_REG */
	{ 0x1400, 0x00000000 },	/* AUD_SYS_PWR_REG */
	{ 0x1404, 0x00000000 },	/* BUS0_SYS_PWR_REG */
	{ 0x1408, 0x00000000 },	/* CAM0_SYS_PWR_REG */
	{ 0x140c, 0x00000000 },	/* CAM1_SYS_PWR_REG */
	{ 0x1410, 0x00000000 },	/* DISP_SYS_PWR_REG */
	{ 0x1414, 0x00000000 },	/* FSYS0_SYS_PWR_REG */
	{ 0x1418, 0x00000000 },	/* FSYS1_SYS_PWR_REG */
	{ 0x1420, 0x00000000 },	/* G3D_SYS_PWR_REG */
	{ 0x1428, 0x00000000 },	/* ISP0_SYS_PWR_REG */
	{ 0x142c, 0x00000000 },	/* ISP1_SYS_PWR_REG */
	{ 0x1430, 0x00000000 },	/* MFC_SYS_PWR_REG */
	{ 0x1434, 0x00000000 },	/* MSCL_SYS_PWR_REG */
	{ 0x1438, 0x00000000 },	/* VPP_SYS_PWR_REG */
	{ 0x1440, 0x00000000 },	/* CLKRUN_CMU_AUD_SYS_PWR_REG */
	{ 0x1444, 0x00000000 },	/* CLKRUN_CMU_BUS0_SYS_PWR_REG */
	{ 0x1448, 0x00000000 },	/* CLKRUN_CMU_CAM0_SYS_PWR_REG */
	{ 0x144c, 0x00000000 },	/* CLKRUN_CMU_CAM1_SYS_PWR_REG */
	{ 0x1450, 0x00000000 },	/* CLKRUN_CMU_DISP_SYS_PWR_REG */
	{ 0x1454, 0x00000000 },	/* CLKRUN_CMU_FSYS0_SYS_PWR_REG */
	{ 0x1458, 0x00000000 },	/* CLKRUN_CMU_FSYS1_SYS_PWR_REG */
	{ 0x1460, 0x00000000 },	/* CLKRUN_CMU_G3D_SYS_PWR_REG */
	{ 0x1468, 0x00000000 },	/* CLKRUN_CMU_ISP0_SYS_PWR_REG */
	{ 0x146c, 0x00000000 },	/* CLKRUN_CMU_ISP1_SYS_PWR_REG */
	{ 0x1470, 0x00000000 },	/* CLKRUN_CMU_MFC_SYS_PWR_REG */
	{ 0x1474, 0x00000000 },	/* CLKRUN_CMU_MSCL_SYS_PWR_REG */
	{ 0x1478, 0x00000000 },	/* CLKRUN_CMU_VPP_SYS_PWR_REG */
	{ 0x1480, 0x00000000 },	/* CLKSTOP_CMU_AUD_SYS_PWR_REG */
	{ 0x1484, 0x00000000 },	/* CLKSTOP_CMU_BUS0_SYS_PWR_REG */
	{ 0x1488, 0x00000000 },	/* CLKSTOP_CMU_CAM0_SYS_PWR_REG */
	{ 0x148c, 0x00000000 },	/* CLKSTOP_CMU_CAM1_SYS_PWR_REG */
	{ 0x1490, 0x00000000 },	/* CLKSTOP_CMU_DISP_SYS_PWR_REG */
	{ 0x1494, 0x00000000 },	/* CLKSTOP_CMU_FSYS0_SYS_PWR_REG */
	{ 0x1498, 0x00000000 },	/* CLKSTOP_CMU_FSYS1_SYS_PWR_REG */
	{ 0x14a0, 0x00000000 },	/* CLKSTOP_CMU_G3D_SYS_PWR_REG */
	{ 0x14a8, 0x00000000 },	/* CLKSTOP_CMU_ISP0_SYS_PWR_REG */
	{ 0x14ac, 0x00000000 },	/* CLKSTOP_CMU_ISP1_SYS_PWR_REG */
	{ 0x14b0, 0x00000000 },	/* CLKSTOP_CMU_MFC_SYS_PWR_REG */
	{ 0x14b4, 0x00000000 },	/* CLKSTOP_CMU_MSCL_SYS_PWR_REG */
	{ 0x14b8, 0x00000000 },	/* CLKSTOP_CMU_VPP_SYS_PWR_REG */
	{ 0x14c0, 0x00000000 },	/* DISABLE_PLL_CMU_AUD_SYS_PWR_REG */
	{ 0x14c4, 0x00000000 },	/* DISABLE_PLL_CMU_BUS0_SYS_PWR_REG */
	{ 0x14c8, 0x00000000 },	/* DISABLE_PLL_CMU_CAM0_SYS_PWR_REG */
	{ 0x14cc, 0x00000000 },	/* DISABLE_PLL_CMU_CAM1_SYS_PWR_REG */
	{ 0x14d0, 0x00000000 },	/* DISABLE_PLL_CMU_DISP_SYS_PWR_REG */
	{ 0x14d4, 0x00000000 },	/* DISABLE_PLL_CMU_FSYS0_SYS_PWR_REG */
	{ 0x14d8, 0x00000000 },	/* DISABLE_PLL_CMU_FSYS1_SYS_PWR_REG */
	{ 0x14e0, 0x00000000 },	/* DISABLE_PLL_CMU_G3D_SYS_PWR_REG */
	{ 0x14e8, 0x00000000 },	/* DISABLE_PLL_CMU_ISP0_SYS_PWR_REG */
	{ 0x14ec, 0x00000000 },	/* DISABLE_PLL_CMU_ISP1_SYS_PWR_REG */
	{ 0x14f0, 0x00000000 },	/* DISABLE_PLL_CMU_MFC_SYS_PWR_REG */
	{ 0x14f4, 0x00000000 },	/* DISABLE_PLL_CMU_MSCL_SYS_PWR_REG */
	{ 0x14f8, 0x00000000 },	/* DISABLE_PLL_CMU_VPP_SYS_PWR_REG */
	{ 0x1500, 0x00000000 },	/* RESET_LOGIC_AUD_SYS_PWR_REG */
	{ 0x1504, 0x00000000 },	/* RESET_LOGIC_BUS0_SYS_PWR_REG */
	{ 0x1508, 0x00000000 },	/* RESET_LOGIC_CAM0_SYS_PWR_REG */
	{ 0x150c, 0x00000000 },	/* RESET_LOGIC_CAM1_SYS_PWR_REG */
	{ 0x1510, 0x00000000 },	/* RESET_LOGIC_DISP_SYS_PWR_REG */
	{ 0x1514, 0x00000000 },	/* RESET_LOGIC_FSYS0_SYS_PWR_REG */
	{ 0x1518, 0x00000000 },	/* RESET_LOGIC_FSYS1_SYS_PWR_REG */
	{ 0x1520, 0x00000000 },	/* RESET_LOGIC_G3D_SYS_PWR_REG */
	{ 0x1528, 0x00000000 },	/* RESET_LOGIC_ISP0_SYS_PWR_REG */
	{ 0x152c, 0x00000000 },	/* RESET_LOGIC_ISP1_SYS_PWR_REG */
	{ 0x1530, 0x00000000 },	/* RESET_LOGIC_MFC_SYS_PWR_REG */
	{ 0x1534, 0x00000000 },	/* RESET_LOGIC_MSCL_SYS_PWR_REG */
	{ 0x1538, 0x00000000 },	/* RESET_LOGIC_VPP_SYS_PWR_REG */
	{ 0x1540, 0x00000000 },	/* MEMORY_AUD_SYS_PWR_REG */
	{ 0x1548, 0x00000000 },	/* MEMORY_CAM0_SYS_PWR_REG */
	{ 0x154c, 0x00000000 },	/* MEMORY_CAM1_SYS_PWR_REG */
	{ 0x1550, 0x00000000 },	/* MEMORY_DISP_SYS_PWR_REG */
	{ 0x1554, 0x00000000 },	/* MEMORY_FSYS0_SYS_PWR_REG */
	{ 0x1558, 0x00000000 },	/* MEMORY_FSYS1_SYS_PWR_REG */
	{ 0x1560, 0x00000000 },	/* MEMORY_G3D_SYS_PWR_REG */
	{ 0x1568, 0x00000000 },	/* MEMORY_ISP0_SYS_PWR_REG */
	{ 0x156c, 0x00000000 },	/* MEMORY_ISP1_SYS_PWR_REG */
	{ 0x1570, 0x00000000 },	/* MEMORY_MFC_SYS_PWR_REG */
	{ 0x1574, 0x00000000 },	/* MEMORY_MSCL_SYS_PWR_REG */
	{ 0x1578, 0x00000000 },	/* MEMORY_VPP_SYS_PWR_REG */
	{ 0x1580, 0x00000000 },	/* RESET_CMU_AUD_SYS_PWR_REG */
	{ 0x1584, 0x00000000 },	/* RESET_CMU_BUS0_SYS_PWR_REG */
	{ 0x1588, 0x00000000 },	/* RESET_CMU_CAM0_SYS_PWR_REG */
	{ 0x158c, 0x00000000 },	/* RESET_CMU_CAM1_SYS_PWR_REG */
	{ 0x1590, 0x00000000 },	/* RESET_CMU_DISP_SYS_PWR_REG */
	{ 0x1594, 0x00000000 },	/* RESET_CMU_FSYS0_SYS_PWR_REG */
	{ 0x1598, 0x00000000 },	/* RESET_CMU_FSYS1_SYS_PWR_REG */
	{ 0x15a0, 0x00000000 },	/* RESET_CMU_G3D_SYS_PWR_REG */
	{ 0x15a8, 0x00000000 },	/* RESET_CMU_ISP0_SYS_PWR_REG */
	{ 0x15ac, 0x00000000 },	/* RESET_CMU_ISP1_SYS_PWR_REG */
	{ 0x15b0, 0x00000000 },	/* RESET_CMU_MFC_SYS_PWR_REG */
	{ 0x15b4, 0x00000000 },	/* RESET_CMU_MSCL_SYS_PWR_REG */
	{ 0x15b8, 0x00000000 },	/* RESET_CMU_VPP_SYS_PWR_REG */
	{ 0x15c4, 0x00000000 },	/* RESET_SLEEP_BUS0_SYS_PWR_REG */
	{ 0x15d4, 0x00000000 },	/* RESET_SLEEP_FSYS0_SYS_PWR_REG */
	{ 0x15d8, 0x00000000 },	/* RESET_SLEEP_FSYS1_SYS_PWR_REG */
	{ 0x0020, 0x00000000 },	/* PMU_SYNC_CTRL */
	{ 0x0248, 0x00001000 },	/* CENTRAL_SEQ_MIF_OPTION */
	{ 0x0628, 0x00100013 },	/* WAKEUP_MASK_MIF */
	{ 0x2408, 0x00000001 },	/* ATLAS_NONCPU_OPTION */
	{ 0x2e08, 0x00000001 },	/* MEMORY_TOP_OPTION */
	{ 0x2e28, 0x00000011 },	/* MEMORY_TOP_ALV_OPTION */
	{ 0x2868, 0x00000000 },	/* RESET_CMU_TOP_OPTION */
	{ 0x3a08, 0xc0001102 },	/* ATLAS_OPTION */
	{ 0x3a28, 0xc0001102 },	/* APOLLO_OPTION */
	{ 0x40a8, 0xc0001102 },	/* FSYS0_OPTION */
	{ 0x40c8, 0xc0001102 },	/* FSYS1_OPTION */
	{ 0x3068, 0x00000000 },	/* PAD_RETENTION_BUS0_TOP_OPTION */
	{ 0x2c48, 0x00000002 },	/* TOP_PWR_OPTION */
};

/* Sequencer steps for sleep (CKE first) and the default order. */
static const u32 exynos7_mtup_sleep[EXYNOS7_SW_MTUP_STEPS] = {
	0x0d0c0b09, 0x11100f0e, 0x15141312, 0x19181716, 0x1d1c1b1a,
	0x21201f1e, 0x25242322, 0x29282726, 0x2d2c2b2a, 0x31302f2e,
	0x35343332, 0x39383736, 0x3c3b0a3a,
};

static const u32 exynos7_mtup_normal[EXYNOS7_SW_MTUP_STEPS] = {
	0x0c0b0a09, 0x100f0e0d, 0x14131211, 0x18171615, 0x1c1b1a19,
	0x201f1e1d, 0x24232221, 0x28272625, 0x2c2b2a29, 0x302f2e2d,
	0x34333231, 0x38373635, 0x3c3b3a39,
};

static void __iomem *exynos7_lpi_atlas;

/*
 * CPU power state. Index = core + 4 * cluster from MPIDR: ATLAS (A57,
 * cluster 0) is 0-3, APOLLO (A53, cluster 1, the boot cluster) is 4-7.
 */
#define EXYNOS7_CPU_STATUS(n)			(0x2004 + (n) * 0x80)
#define EXYNOS7_LOCAL_PWR_CFG			0xf
#define EXYNOS7_BOOT_CPU_INDEX			4
#define EXYNOS7_ATLAS_NONCPU_STATUS		0x2404
#define EXYNOS7_APOLLO_NONCPU_STATUS		0x2424

/*
 * Samsung's kernel waited until PSCI CPU_OFF had really powered every
 * non-boot core down before it issued the sleep call: a core still in
 * WFI keeps its cluster's L2 up, and the central sequencer then never
 * completes the power-down handshake.
 */
static int exynos7_pm_wait_cpus_off(void)
{
	/* Interrupts are off: count polls, 50000 x 10 us = 500 ms. */
	int polls = 50000;
	u32 busy, status;
	int n;

	do {
		busy = 0;
		for (n = 0; n < 8; n++) {
			if (n == EXYNOS7_BOOT_CPU_INDEX)
				continue;
			status = pmu_raw_readl(EXYNOS7_CPU_STATUS(n));
			if (status & EXYNOS7_LOCAL_PWR_CFG)
				busy |= BIT(n);
		}
		if (!busy)
			return 0;
		udelay(10);
	} while (--polls);

	for (n = 0; n < 8; n++)
		if (busy & BIT(n))
			pr_err("exynos7-pm: cpu index %d still powered: %#x\n",
			       n, pmu_raw_readl(EXYNOS7_CPU_STATUS(n)));
	return -EBUSY;
}

/*
 * Cluster power control in CMU_ATLAS and CMU_APOLLO. Samsung's kernel
 * cleared both (USE_L2QACTIVE, bit 14, among them) before sleep, so the L2
 * Q-channel clock-down cannot hold off the L2 power-down, and set
 * USE_L2QACTIVE again after; here the saved values are put back. CMU_ATLAS is written while the A57 cluster is
 * still up: in prepare_late, before the non-boot CPUs go offline.
 */
#define EXYNOS7_ATLAS_PWR_CTRL			0x11801020
#define EXYNOS7_APOLLO_PWR_CTRL			0x11901020

static void __iomem *exynos7_atlas_pwr_ctrl;
static void __iomem *exynos7_apollo_pwr_ctrl;
static u32 exynos7_atlas_pwr_saved, exynos7_apollo_pwr_saved;

static int exynos7_pm_prepare_late(void)
{
	exynos7_atlas_pwr_saved = readl(exynos7_atlas_pwr_ctrl);
	exynos7_apollo_pwr_saved = readl(exynos7_apollo_pwr_ctrl);
	pr_info("exynos7-pm: PWR_CTRL atlas %#x apollo %#x\n",
		exynos7_atlas_pwr_saved, exynos7_apollo_pwr_saved);
	writel(0, exynos7_atlas_pwr_ctrl);
	writel(0, exynos7_apollo_pwr_ctrl);
	return 0;
}

static void exynos7_pm_wake(void)
{
	writel(exynos7_atlas_pwr_saved, exynos7_atlas_pwr_ctrl);
	writel(exynos7_apollo_pwr_saved, exynos7_apollo_pwr_ctrl);
}

/*
 * Bring-up: arm the watchdog with reset for the length of the sleep call.
 * If the SoC never powers down, it resets by itself (RST_STAT bit 23); if
 * the power-down completes, the watchdog stops with the TOP domain.
 */
static bool sleep_watchdog = true;
module_param(sleep_watchdog, bool, 0644);

/*
 * Bring-up: wait this long before the sleep call, so a wakeup event armed
 * just before suspend (an RTC alarm) is already pending at the firmware.
 * A clean early-wake return then proves the firmware entry/abort path.
 */
/* Bring-up: run the whole sleep path but skip the firmware call. */
static bool dry_run;
module_param(dry_run, bool, 0644);

static uint early_wake_ms;
module_param(early_wake_ms, uint, 0644);

/* gpa3-7 is the s2mps15 interrupt line (active low). */
#define EXYNOS7_ALIVE_PINCTRL			0x10580000
#define EXYNOS7_GPA3_DAT			0x0064
#define EXYNOS7_GPA3_EPEND			0x0a0c

static void __iomem *exynos7_alive_pinctrl;

static u32 exynos7_pm_gpa3_pend(void)
{
	return readl(exynos7_alive_pinctrl + EXYNOS7_GPA3_EPEND) << 16 |
	       readl(exynos7_alive_pinctrl + EXYNOS7_GPA3_DAT);
}

#define EXYNOS7_WDT_BASE			0x101d0000
#define EXYNOS7_WTCON				0x00
#define EXYNOS7_WTDAT				0x04
#define EXYNOS7_WTCNT				0x08
/* Prescaler 256, divider 128, enable, reset: about 32 s at 66 MHz. */
#define EXYNOS7_WTCON_SLEEP			0xff39
#define EXYNOS7_ENABLE_PCLK_PERIS		0x10040900
#define EXYNOS7_PCLK_WDT			BIT(6)
#define EXYNOS7_WDT_DISABLE			0x0408
#define EXYNOS7_WDT_MASK_RESET			0x040c
#define EXYNOS7_WDT_RESET_BIT			BIT(23)

static void __iomem *exynos7_wdt;
static void __iomem *exynos7_pclk_peris;
static u32 exynos7_wdt_saved[3];

static void exynos7_pm_watchdog(bool arm)
{
	u32 reg;

	if (arm) {
		exynos7_wdt_saved[0] = readl(exynos7_pclk_peris);
		exynos7_wdt_saved[1] = pmu_raw_readl(EXYNOS7_WDT_DISABLE);
		exynos7_wdt_saved[2] = pmu_raw_readl(EXYNOS7_WDT_MASK_RESET);
		writel(exynos7_wdt_saved[0] | EXYNOS7_PCLK_WDT, exynos7_pclk_peris);
		reg = exynos7_wdt_saved[1] & ~EXYNOS7_WDT_RESET_BIT;
		pmu_raw_writel(reg, EXYNOS7_WDT_DISABLE);
		reg = exynos7_wdt_saved[2] & ~EXYNOS7_WDT_RESET_BIT;
		pmu_raw_writel(reg, EXYNOS7_WDT_MASK_RESET);
		writel(0, exynos7_wdt + EXYNOS7_WTCON);
		writel(0xffff, exynos7_wdt + EXYNOS7_WTDAT);
		writel(0xffff, exynos7_wdt + EXYNOS7_WTCNT);
		writel(EXYNOS7_WTCON_SLEEP, exynos7_wdt + EXYNOS7_WTCON);
	} else {
		writel(0, exynos7_wdt + EXYNOS7_WTCON);
		pmu_raw_writel(exynos7_wdt_saved[1], EXYNOS7_WDT_DISABLE);
		pmu_raw_writel(exynos7_wdt_saved[2], EXYNOS7_WDT_MASK_RESET);
		writel(exynos7_wdt_saved[0], exynos7_pclk_peris);
	}
}

/*
 * Bring-up: power these block domains off right before the sleep call and
 * back on after it. Bit n is the domain at PMU 0x4000 + n * 0x20 (AUD 0,
 * CAM0 2, CAM1 3, DISP 4, G3D 8, ISP0 10, ISP1 12, MFC 13, MSCL 14, VPP
 * 15). The power-off runs
 * children first (ISP0, CAM0, ISP1, CAM1).
 */
static uint pd_off_mask;
module_param(pd_off_mask, uint, 0644);

static const u16 exynos7_pd_off_order[] = {
	0x4140, 0x4040, 0x4180, 0x4060, 0x4000, 0x41a0, 0x41c0,
	0x4080, 0x41e0, 0x4100,
};
static u32 exynos7_pd_was_off;

static int exynos7_pd_bit(u16 off)
{
	return (off - 0x4000) / 0x20;
}

static void exynos7_pm_pd_set(u16 off, u32 val)
{
	int polls = 1000;

	pmu_raw_writel(val, off);
	while ((pmu_raw_readl(off + 4) & 0xf) != val && --polls)
		udelay(10);
	if (!polls)
		pr_err("exynos7-pm: domain %#x stuck at %#x (want %#x)\n",
		       off, pmu_raw_readl(off + 4), val);
}

static void exynos7_pm_pd_off(void)
{
	int i;

	exynos7_pd_was_off = 0;
	for (i = 0; i < ARRAY_SIZE(exynos7_pd_off_order); i++) {
		u16 off = exynos7_pd_off_order[i];

		if (!(pd_off_mask & BIT(exynos7_pd_bit(off))) ||
		    !(pmu_raw_readl(off + 4) & 0xf))
			continue;
		exynos7_pm_pd_set(off, 0);
		exynos7_pd_was_off |= BIT(exynos7_pd_bit(off));
	}
}

static void exynos7_pm_pd_on(void)
{
	int i;

	for (i = ARRAY_SIZE(exynos7_pd_off_order) - 1; i >= 0; i--) {
		u16 off = exynos7_pd_off_order[i];

		if (exynos7_pd_was_off & BIT(exynos7_pd_bit(off)))
			exynos7_pm_pd_set(off, 0xf);
	}
}

/* CORTEXM3_APM power control; see exynos7_pm_apm_power_down(). */
#define EXYNOS7_APM_CONFIGURATION		0x2500
#define EXYNOS7_APM_STATUS			0x2504
#define EXYNOS7_APM_OPTION			0x2508
#define EXYNOS7_APM_LOCAL_PWR_RUN		BIT(0)

/* Bring-up: block domain and A7IS state right before the sleep call. */
static void exynos7_pm_dump_domains(void)
{
	static const struct { const char *name; u16 off; } dom[] = {
		{ "AUD", 0x4000 }, { "BUS0", 0x4020 }, { "CAM0", 0x4040 },
		{ "CAM1", 0x4060 }, { "DISP", 0x4080 }, { "FSYS0", 0x40a0 },
		{ "FSYS1", 0x40c0 }, { "G3D", 0x4100 }, { "ISP0", 0x4140 },
		{ "ISP1", 0x4180 }, { "MFC", 0x41a0 }, { "MSCL", 0x41c0 },
		{ "VPP", 0x41e0 },
	};
	char buf[160];
	int i, len = 0;

	for (i = 0; i < ARRAY_SIZE(dom); i++)
		len += scnprintf(buf + len, sizeof(buf) - len, " %s=%x",
				 dom[i].name,
				 pmu_raw_readl(dom[i].off + 4) & 0xf);
	pr_info("exynos7-pm: domains%s\n", buf);
	pr_info("exynos7-pm: A7IS status %#x option %#x\n",
		pmu_raw_readl(0x2584), pmu_raw_readl(0x2588));
	pr_info("exynos7-pm: APM cfg %#x status %#x option %#x\n",
		pmu_raw_readl(EXYNOS7_APM_CONFIGURATION),
		pmu_raw_readl(EXYNOS7_APM_STATUS),
		pmu_raw_readl(EXYNOS7_APM_OPTION));
}

static void exynos7_pm_set_mtup(const u32 *steps)
{
	int i;

	for (i = 0; i < EXYNOS7_SW_MTUP_STEPS; i++) {
		pmu_raw_writel(steps[i], EXYNOS7_SW_MTUP(i));
		pmu_raw_writel(steps[i], EXYNOS7_SW_MTUP_MIF(i));
	}
}

static void exynos7_pm_central_seq(bool enable)
{
	u32 reg;

	reg = pmu_raw_readl(EXYNOS7_CENTRAL_SEQ_CONFIGURATION);
	reg = enable ? reg & ~EXYNOS7_CENTRALSEQ_PWR_CFG :
		       reg | EXYNOS7_CENTRALSEQ_PWR_CFG;
	pmu_raw_writel(reg, EXYNOS7_CENTRAL_SEQ_CONFIGURATION);

	reg = pmu_raw_readl(EXYNOS7_CENTRAL_SEQ_MIF_CONFIGURATION);
	reg = enable ? reg & ~EXYNOS7_CENTRALSEQ_PWR_CFG :
		       reg | EXYNOS7_CENTRALSEQ_PWR_CFG;
	pmu_raw_writel(reg, EXYNOS7_CENTRAL_SEQ_MIF_CONFIGURATION);
}

static void exynos7_pm_lpi_mask(void)
{
	u32 reg;

	reg = readl(exynos7_lpi_atlas + EXYNOS7_LPI_MASK_ATLAS_ASYNCBRIDGE);
	reg |= EXYNOS7_ASATBZ_MSTIF_ATLAS_AUD | EXYNOS7_ASATBZ_MSTIF_ATLAS_CAM1;
	writel(reg, exynos7_lpi_atlas + EXYNOS7_LPI_MASK_ATLAS_ASYNCBRIDGE);
}

static void exynos7_pm_prepare_sleep(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(exynos7_pmu_sleep); i++)
		pmu_raw_writel(exynos7_pmu_sleep[i].value,
			       exynos7_pmu_sleep[i].offset);

	/* Keep the 24 MHz oscillator running while CLKOUT is in use. */
	if (!(pmu_raw_readl(EXYNOS7_PMU_DEBUG) & EXYNOS7_CLKOUT_DISABLE))
		pmu_raw_writel(1, EXYNOS7_XXTI_SYS_PWR_REG);

	exynos7_pm_central_seq(true);
	exynos7_pm_lpi_mask();
	exynos7_pm_set_mtup(exynos7_mtup_sleep);
}

/* Returns true when the SoC went down, false on an early wake. */
static bool exynos7_pm_finish_sleep(void)
{
	bool slept = pmu_raw_readl(EXYNOS7_CENTRAL_SEQ_CONFIGURATION) &
		     EXYNOS7_CENTRALSEQ_PWR_CFG;

	if (slept) {
		/* Pad retention is released by the pin controller's resume. */
		exynos7_pm_lpi_mask();
	} else {
		exynos7_pm_central_seq(false);
	}
	exynos7_pm_set_mtup(exynos7_mtup_normal);
	return slept;
}

/*
 * Clocks for sleep, from Samsung's exynos-powermode.c: before the power-
 * down every gate and mux enable in the listed CMUs is opened (and three
 * mux selects moved), so each block can complete its low-power handshake.
 * The SYSREG root clock gating list is lost in sleep and put back after.
 */
struct exynos7_sfr {
	u32 pa;
	u32 mask;
	u32 val;
	u32 saved;
	void __iomem *va;
};

#define EXYNOS7_SYSREG_APOLLO_SRAM	0x10050054	/* 0: dual rail */

/*
 * The APM, a Cortex-M3 in the PMU's ALIVE domain, ran Samsung's CL-DVFS
 * firmware. Samsung's kernel powered it up and out of reset only with the
 * screen on and the firmware loaded, and powered it down again on screen
 * off, before any sleep, and on reboot. S-Boot leaves it powered but held
 * in reset, so it never signals STANDBYWFI, and Linux has no use for it:
 * power it down for good, as Samsung's kernel had it whenever it slept.
 */
static void exynos7_pm_apm_power_down(void)
{
	u32 cfg = pmu_raw_readl(EXYNOS7_APM_CONFIGURATION);
	int i;

	pr_info("exynos7-pm: APM cfg %#x status %#x option %#x, powering down\n",
		cfg, pmu_raw_readl(EXYNOS7_APM_STATUS),
		pmu_raw_readl(EXYNOS7_APM_OPTION));
	pmu_raw_writel(cfg & ~EXYNOS7_APM_LOCAL_PWR_RUN,
		       EXYNOS7_APM_CONFIGURATION);
	for (i = 0; i < 1000; i++) {
		if (!(pmu_raw_readl(EXYNOS7_APM_STATUS) &
		      EXYNOS7_APM_LOCAL_PWR_RUN))
			break;
		udelay(10);
	}
	pr_info("exynos7-pm: APM status %#x after power-down\n",
		pmu_raw_readl(EXYNOS7_APM_STATUS));
}

/*
 * Boot settings from Samsung's PMU init: no clock freeze around the ARM
 * clamp of either cluster.
 */
#define EXYNOS7_ATLAS_ARMCLK_STOPCTRL		0x11801000
#define EXYNOS7_APOLLO_ARMCLK_STOPCTRL		0x11901000

static void exynos7_pm_clear_stopctrl(void)
{
	static const u32 pa[] = {
		EXYNOS7_ATLAS_ARMCLK_STOPCTRL, EXYNOS7_APOLLO_ARMCLK_STOPCTRL,
	};
	void __iomem *va;
	int i;

	for (i = 0; i < ARRAY_SIZE(pa); i++) {
		va = ioremap(pa[i], 4);
		if (!va)
			continue;
		pr_info("exynos7-pm: ARMCLK_STOPCTRL %#x was %#x\n",
			pa[i], readl(va));
		writel(0, va);
		iounmap(va);
	}
}

/* Saved, then (value & mask) forced, before sleep; restored after. */
static struct exynos7_sfr exynos7_sleep_clk[] = {
	{ 0x10570300, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOPC0 */
	{ 0x10570304, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOPC1 */
	{ 0x10570308, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOPC2 */
	{ 0x1057030c, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOPC3 */
	{ 0x10570310, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOPC4 */
	{ 0x10570314, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOPC5 */
	{ 0x10570800, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_TOPC0 */
	{ 0x10570804, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_TOPC1 */
	{ 0x10570808, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_TOPC2 */
	{ 0x10570a00, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOPC0 */
	{ 0x10570a04, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOPC1 */
	{ 0x105d0300, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP00 */
	{ 0x105d030c, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP03 */
	{ 0x105d0310, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP04 */
	{ 0x105d0314, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP05 */
	{ 0x105d0318, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP06 */
	{ 0x105d031c, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP07 */
	{ 0x105d0320, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP0_DISP */
	{ 0x105d0324, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP0_CAM10 */
	{ 0x105d0328, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP0_CAM11 */
	{ 0x105d0330, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP0_PERIC0 */
	{ 0x105d0334, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP0_PERIC1 */
	{ 0x105d0338, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP0_PERIC2 */
	{ 0x105d033c, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP0_PERIC3 */
	{ 0x105d0808, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_TOP02 */
	{ 0x105d080c, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_TOP03 */
	{ 0x105d0810, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_TOP04 */
	{ 0x105d0814, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_TOP05 */
	{ 0x105d0818, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_TOP06 */
	{ 0x105d081c, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_TOP07 */
	{ 0x105d0a20, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOP0_DISP */
	{ 0x105d0a24, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOP0_CAM10 */
	{ 0x105d0a28, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOP0_CAM11 */
	{ 0x105d0a30, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOP0_PERIC0 */
	{ 0x105d0a34, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOP0_PERIC1 */
	{ 0x105d0a38, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOP0_PERIC2 */
	{ 0x105d0a3c, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOP0_PERIC3 */
	{ 0x105e0300, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP10 */
	{ 0x105e030c, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP13 */
	{ 0x105e0324, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP1_FSYS0 */
	{ 0x105e0328, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP1_FSYS1 */
	{ 0x105e032c, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_TOP1_FSYS11 */
	{ 0x105e0808, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_TOP12 */
	{ 0x105e080c, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_TOP13 */
	{ 0x105e0a24, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOP1_FSYS0 */
	{ 0x105e0a28, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOP1_FSYS1 */
	{ 0x105e0a2c, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_TOP1_FSYS11 */
	{ 0x105d0200, 0x00000001, 0x00000000 },	/* MUX_SEL_TOP00 */
	{ 0x105d0220, 0x00007000, 0x00003000 },	/* MUX_SEL_TOP0_DISP */
	{ 0x105d0230, 0x00300370, 0x00100130 },	/* MUX_SEL_TOP0_PERIC0 */
	{ 0x10850300, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_MIF0 */
	{ 0x10950300, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_MIF1 */
	{ 0x10a50300, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_MIF2 */
	{ 0x10b50300, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_MIF3 */
	{ 0x105b0800, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_CCORE0 */
	{ 0x105b0804, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_CCORE1 */
	{ 0x11060300, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_IMEM */
	{ 0x11060800, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_IMEM0 */
	{ 0x11060804, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_IMEM1 */
	{ 0x11060900, 0xffffffff, 0xffffffff },	/* ENABLE_PCLK_IMEM */
	{ 0x13610300, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_PERIC0 */
	{ 0x13610a00, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_PERIC0 */
	{ 0x14c80304, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_PERIC11 */
	{ 0x14c80a00, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_PERIC10 */
	{ 0x13400800, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_BUS0 */
	{ 0x14800204, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_BUS1 */
	{ 0x14800800, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_BUS1 */
	{ 0x10e90300, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_FSYS00 */
	{ 0x10e90304, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_FSYS01 */
	{ 0x10e90800, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_FSYS00 */
	{ 0x10e90804, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_FSYS01 */
	{ 0x10e90a04, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_FSYS01 */
	{ 0x10e90a10, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_FSYS04 */
	{ 0x156e0304, 0xffffffff, 0xffffffff },	/* MUX_ENABLE_FSYS11 */
	{ 0x156e0800, 0xffffffff, 0xffffffff },	/* ENABLE_ACLK_FSYS1 */
	/*
	 * BLK_AUD stays powered here, but LPASS gates its bridge to the
	 * top bus while idle and I2S/PCM may run from the codec's clock.
	 * The block then cannot finish its power-down handshake and the
	 * sleep sequence stalls. Samsung's AUD power-off opened the gate
	 * and moved both muxes to the PLL first.
	 */
	{ 0x114c0800, BIT(28), BIT(28) },	/* ENABLE_ACLK_AUD: LH_ASYNC_SI_TOP */
	{ 0x114c0200, BIT(16) | BIT(12), 0 },	/* MUX_SEL_AUD: PCM, I2S from PLL */
	{ 0x10050038, 0x00000000, 0x00000000 },	/* APOLLO_EMA_CON */
	{ 0x10050138, 0x00000000, 0x00000000 },	/* ATLAS_EMA_CON */
	{ 0x10052908, 0x00000000, 0x00000000 },	/* CPU_EMA_REG1 */
	{ 0x10052718, 0x00000000, 0x00000000 },	/* EMA_RF2_HS_CON */
	{ 0x10052758, 0x00000000, 0x00000000 },	/* EMA_RF2_HS_CON_INT */
	{ 0x10051424, 0x00000000, 0x00000000 },	/* IMEM_EMA_VROMP_HD_CON */
	{ 0x1005275c, 0x00000000, 0x00000000 },	/* EMA_RF2_UHD_CON_INT */
	{ 0x1005271c, 0x00000000, 0x00000000 },	/* EMA_RF2_UHD_CON */
	{ 0x156e0a04, 0xffffffff, 0xffffffff },	/* ENABLE_SCLK_FSYS11 */
};

/* SYSREG root clock gating, restored after a real sleep. */
static struct exynos7_sfr exynos7_sleep_rcg[] = {
	{ 0x10050018 },
	{ 0x10050118 },
	{ 0x1005016c },
	{ 0x10050218 },
	{ 0x1005021c },
	{ 0x1005023c },
	{ 0x10050240 },
	{ 0x10050318 },
	{ 0x10050320 },
	{ 0x10050418 },
	{ 0x10050420 },
	{ 0x10050518 },
	{ 0x1005051c },
	{ 0x10050524 },
	{ 0x10050540 },
	{ 0x10050544 },
	{ 0x10050618 },
	{ 0x10050624 },
	{ 0x1005064c },
	{ 0x10050650 },
	{ 0x10050714 },
	{ 0x1005071c },
	{ 0x10050724 },
	{ 0x10050818 },
	{ 0x1005081c },
	{ 0x100508d0 },
	{ 0x100508d4 },
	{ 0x100508d8 },
	{ 0x100508dc },
	{ 0x10051018 },
	{ 0x1005101c },
	{ 0x10051020 },
	{ 0x10051068 },
	{ 0x1005106c },
	{ 0x10051074 },
	{ 0x10051078 },
	{ 0x10051134 },
	{ 0x10051138 },
	{ 0x10051218 },
	{ 0x10051240 },
	{ 0x10051244 },
	{ 0x1005124c },
	{ 0x10051418 },
	{ 0x1005141c },
	{ 0x10051420 },
	{ 0x10051434 },
	{ 0x10051444 },
	{ 0x10051448 },
	{ 0x1005151c },
	{ 0x10051524 },
	{ 0x10051538 },
	{ 0x1005153c },
	{ 0x10051618 },
	{ 0x1005162c },
	{ 0x10051630 },
	{ 0x10051634 },
	{ 0x10051724 },
	{ 0x10051738 },
	{ 0x10051818 },
	{ 0x10051824 },
	{ 0x10051828 },
	{ 0x10051918 },
	{ 0x10051924 },
	{ 0x10051928 },
	{ 0x10052018 },
	{ 0x10052024 },
	{ 0x10052028 },
	{ 0x10052118 },
	{ 0x10052124 },
	{ 0x10052128 },
	{ 0x10052218 },
	{ 0x1005221c },
	{ 0x1005222c },
	{ 0x10052230 },
	{ 0x10052234 },
	{ 0x10052318 },
	{ 0x10052418 },
	{ 0x10052440 },
	{ 0x10052518 },
	{ 0x1005251c },
	{ 0x10052550 },
	{ 0x10052554 },
	{ 0x10052558 },
	{ 0x10052618 },
	{ 0x1005261c },
	{ 0x10052620 },
	{ 0x10052624 },
	{ 0x10052640 },
	{ 0x10052648 },
	{ 0x10052818 },
	{ 0x10570d08 },
	{ 0x105d0d04 },
	{ 0x105d0d08 },
	{ 0x105e0d04 },
};

static void __iomem *exynos7_apollo_sram;

static int exynos7_pm_map_sfrs(struct exynos7_sfr *sfr, int n)
{
	int i;

	for (i = 0; i < n; i++) {
		void __iomem *page = ioremap(sfr[i].pa & PAGE_MASK, PAGE_SIZE);

		if (!page)
			return -ENOMEM;
		sfr[i].va = page + (sfr[i].pa & ~PAGE_MASK);
	}
	return 0;
}

static void exynos7_pm_save_sfrs(struct exynos7_sfr *sfr, int n)
{
	int i;

	for (i = 0; i < n; i++)
		sfr[i].saved = readl_relaxed(sfr[i].va);
}

static void exynos7_pm_restore_sfrs(struct exynos7_sfr *sfr, int n)
{
	int i;

	for (i = 0; i < n; i++)
		writel_relaxed(sfr[i].saved, sfr[i].va);
}

static void exynos7_pm_force_sfrs(struct exynos7_sfr *sfr, int n)
{
	int i;

	for (i = 0; i < n; i++)
		if (sfr[i].mask)
			writel_relaxed((sfr[i].saved & ~sfr[i].mask) | sfr[i].val,
				       sfr[i].va);
}

/* Non-secure SYSRAM */
#define EXYNOS7_INFORM0			0x08	/* resume address */
#define EXYNOS7_INFORM1			0x0c	/* sleep flag */
#define EXYNOS7_CHECK_SLEEP		0x00000bad

static void __iomem *exynos7_sysram_ns;

/* Bring-up: marks the ramoops console, then branches to cpu_resume. */
extern void exynos7_pm_resume_entry(void);

/* The firmware's return value when it came back without losing context. */
static int exynos7_pm_psci_ret;

static int exynos7_pm_finisher(unsigned long power_state)
{
	exynos7_pm_psci_ret = psci_ops.cpu_suspend(power_state,
				__pa_symbol(exynos7_pm_resume_entry));
	return exynos7_pm_psci_ret;
}

static int exynos7_pm_enter(suspend_state_t state)
{
	u32 mask, mask2, mask3;
	int ret;

	mask = pmu_raw_readl(EXYNOS7_WAKEUP_MASK);
	mask2 = pmu_raw_readl(EXYNOS7_WAKEUP_MASK2);
	mask3 = pmu_raw_readl(EXYNOS7_WAKEUP_MASK3);
	pmu_raw_writel(EXYNOS7_SLEEP_WAKEUP_MASK, EXYNOS7_WAKEUP_MASK);
	pmu_raw_writel(EXYNOS7_SLEEP_WAKEUP_MASK2, EXYNOS7_WAKEUP_MASK2);
	pmu_raw_writel(EXYNOS7_SLEEP_WAKEUP_MASK3, EXYNOS7_WAKEUP_MASK3);

	writel_relaxed(__pa_symbol(exynos7_pm_resume_entry),
		       exynos7_sysram_ns + EXYNOS7_INFORM0);
	writel(EXYNOS7_CHECK_SLEEP, exynos7_sysram_ns + EXYNOS7_INFORM1);

	exynos7_pm_prepare_sleep();
	exynos7_pm_save_sfrs(exynos7_sleep_clk, ARRAY_SIZE(exynos7_sleep_clk));
	exynos7_pm_force_sfrs(exynos7_sleep_clk, ARRAY_SIZE(exynos7_sleep_clk));
	exynos7_pm_save_sfrs(exynos7_sleep_rcg, ARRAY_SIZE(exynos7_sleep_rcg));

	ret = exynos7_pm_wait_cpus_off();
	pr_info("exynos7-pm: noncpu status atlas %#x apollo %#x\n",
		pmu_raw_readl(EXYNOS7_ATLAS_NONCPU_STATUS),
		pmu_raw_readl(EXYNOS7_APOLLO_NONCPU_STATUS));
	if (ret)
		goto restore;

	/* syscore_suspend() has already run cpu_pm_enter() and the cluster one. */
	exynos7_pm_psci_ret = 0;
	pr_info("exynos7-pm: entering sleep%s\n",
		sleep_watchdog ? " (watchdog armed)" : "");
	if (sleep_watchdog)
		exynos7_pm_watchdog(true);
	exynos7_pm_pd_off();
	exynos7_pm_dump_domains();
	if (early_wake_ms) {
		mdelay(early_wake_ms);
		pr_info("exynos7-pm: gpa3 pend<<16|dat %#x, sleep call now\n",
			exynos7_pm_gpa3_pend());
	}
	if (dry_run) {
		pr_info("exynos7-pm: dry run, not calling the firmware\n");
		ret = -EBUSY;
	} else {
		ret = cpu_suspend(EXYNOS7_PSCI_SYSTEM_SLEEP, exynos7_pm_finisher);
	}
	if (sleep_watchdog)
		exynos7_pm_watchdog(false);
	exynos7_pm_pd_on();
	if (!ret) {
		pr_info("exynos7-pm: resumed through cpu_resume\n");
	} else if (ret == -EOPNOTSUPP && !exynos7_pm_psci_ret) {
		/* The firmware returned success without a power-down. */
		pr_info("exynos7-pm: firmware returned without power-down\n");
		ret = 0;
	} else {
		pr_warn("exynos7-pm: sleep failed: %d (PSCI %d)\n",
			ret, exynos7_pm_psci_ret);
	}

restore:
	if (exynos7_pm_finish_sleep()) {
		exynos7_pm_restore_sfrs(exynos7_sleep_rcg,
					ARRAY_SIZE(exynos7_sleep_rcg));
		writel(0, exynos7_apollo_sram);
	} else {
		pr_info("exynos7-pm: early wake, the SoC stayed up\n");
	}
	exynos7_pm_restore_sfrs(exynos7_sleep_clk, ARRAY_SIZE(exynos7_sleep_clk));
	writel(0, exynos7_sysram_ns + EXYNOS7_INFORM1);
	pr_info("exynos7-pm: wakeup status %#x\n",
		pmu_raw_readl(EXYNOS7_WAKEUP_STAT));
	pmu_raw_writel(mask, EXYNOS7_WAKEUP_MASK);
	pmu_raw_writel(mask2, EXYNOS7_WAKEUP_MASK2);
	pmu_raw_writel(mask3, EXYNOS7_WAKEUP_MASK3);
	return ret;
}

static const struct platform_suspend_ops exynos7_pm_ops = {
	.valid		= suspend_valid_only_mem,
	.prepare_late	= exynos7_pm_prepare_late,
	.enter		= exynos7_pm_enter,
	.wake		= exynos7_pm_wake,
};

void exynos7_pm_init(void)
{
	struct device_node *np;

	if (!psci_ops.cpu_suspend) {
		pr_warn("exynos7-pm: no PSCI CPU_SUSPEND, deep sleep disabled\n");
		return;
	}

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos4210-sysram-ns");
	exynos7_sysram_ns = of_iomap(np, 0);
	of_node_put(np);
	if (!exynos7_sysram_ns) {
		pr_warn("exynos7-pm: no non-secure SYSRAM, deep sleep disabled\n");
		return;
	}

	exynos7_lpi_atlas = ioremap(EXYNOS7_LPI_ATLAS_BASE, SZ_4K);
	exynos7_apollo_sram = ioremap(EXYNOS7_SYSREG_APOLLO_SRAM, 4);
	exynos7_atlas_pwr_ctrl = ioremap(EXYNOS7_ATLAS_PWR_CTRL, 4);
	exynos7_apollo_pwr_ctrl = ioremap(EXYNOS7_APOLLO_PWR_CTRL, 4);
	exynos7_wdt = ioremap(EXYNOS7_WDT_BASE, SZ_4K);
	exynos7_pclk_peris = ioremap(EXYNOS7_ENABLE_PCLK_PERIS, 4);
	exynos7_alive_pinctrl = ioremap(EXYNOS7_ALIVE_PINCTRL, SZ_4K);
	if (!exynos7_lpi_atlas || !exynos7_apollo_sram ||
	    !exynos7_atlas_pwr_ctrl || !exynos7_apollo_pwr_ctrl ||
	    !exynos7_wdt || !exynos7_pclk_peris || !exynos7_alive_pinctrl ||
	    exynos7_pm_map_sfrs(exynos7_sleep_clk, ARRAY_SIZE(exynos7_sleep_clk)) ||
	    exynos7_pm_map_sfrs(exynos7_sleep_rcg, ARRAY_SIZE(exynos7_sleep_rcg))) {
		pr_warn("exynos7-pm: cannot map the sleep registers\n");
		return;
	}

	exynos7_pm_apm_power_down();
	exynos7_pm_clear_stopctrl();
	writel(0, exynos7_apollo_sram);

	/*
	 * Deep sleep is offered but not the default until it is proven on
	 * the board: "echo deep > /sys/power/mem_sleep" selects it. A
	 * mem_sleep_default= argument still takes precedence.
	 */
	if (mem_sleep_default == PM_SUSPEND_MAX)
		mem_sleep_default = PM_SUSPEND_TO_IDLE;
	suspend_set_ops(&exynos7_pm_ops);
}

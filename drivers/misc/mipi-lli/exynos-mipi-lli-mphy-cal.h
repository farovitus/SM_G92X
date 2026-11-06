/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos7420 MIPI-LLI combo M-PHY calibration interface.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 */

#ifndef __DRIVERS_EXYNOS_MIPI_LLI_MPHY_CAL_H
#define __DRIVERS_EXYNOS_MIPI_LLI_MPHY_CAL_H

#include <linux/types.h>

enum lli_mphy_referclk_in_freq {
	LLI_MPHY_REFERENCE_CLK_IN_24MHZ  = 0x00ul,
	LLI_MPHY_REFERENCE_CLK_IN_26MHZ  = 0x01ul,
};

enum lli_mphy_ls_hs_mode {
	LLI_MPHY_PWM_MODE  = 0x01ul,
	LLI_MPHY_HS_MODE  = 0x02ul,
};

enum lli_mphy_gear {
	LLI_MPHY_G1  = 0x01ul,
	LLI_MPHY_G2  = 0x02ul,
	LLI_MPHY_G3  = 0x03ul,
	LLI_MPHY_G4  = 0x04ul,
	LLI_MPHY_G5  = 0x05ul,
};

enum lli_mphy_series {
	LLI_MPHY_RATEA  = 0x01ul,
	LLI_MPHY_RATEB  = 0x02ul,
};

struct refclk_info {
	enum lli_mphy_referclk_in_freq freq;
	bool is_shared;
};

/* @info: OPMODE_* | GEAR_* | HS_RATE_* from exynos-mipi-lli-mphy.h */
u32 exynos_mphy_cal_init(void __iomem *mphy_addr, u32 info);
u32 exynos_mphy_cal_cmn_init(void __iomem *mphy_addr,
			     struct refclk_info *refclk_path);
u32 exynos_mphy_cal_ovtm_init(void __iomem *mphy_addr);
int exynos_mphy_cal_pma_init(void __iomem *pma_addr, void __iomem *mphy_addr,
			     struct refclk_info *refclk_path, u32 info);

#endif /* __DRIVERS_EXYNOS_MIPI_LLI_MPHY_CAL_H */

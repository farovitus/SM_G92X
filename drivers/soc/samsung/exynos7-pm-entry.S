/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos7 sleep resume entry, for bring-up: the firmware enters here with
 * the MMU off. Append a mark to the exynos-snapshot kernel log ring that
 * TWRP shows as /proc/last_kmsg (write pointer at 0x46000200), so a
 * failed resume shows whether Linux was reached, then continue at
 * cpu_resume. Byte stores: with the MMU off, memory is Device and an
 * unaligned wider store would fault. Uses x9-x13 only.
 */
#include <linux/linkage.h>

#define ESS_LAST_LOGBUF_PA	0x46000200

	.text
SYM_CODE_START(exynos7_pm_resume_entry)
	ldr	x9, =ESS_LAST_LOGBUF_PA
	ldr	w11, [x9]
	adr	x12, 2f
	mov	x13, #16
1:	ldrb	w10, [x12], #1
	strb	w10, [x11], #1
	subs	x13, x13, #1
	b.ne	1b
	str	w11, [x9]
	dsb	sy
	b	cpu_resume
	.ltorg
2:	.ascii	"NKRESUMED-ENTRY\n"
SYM_CODE_END(exynos7_pm_resume_entry)

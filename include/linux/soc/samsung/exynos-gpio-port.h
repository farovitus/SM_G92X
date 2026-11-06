/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Direct access to a Samsung pinctrl GPIO bank's data register.
 *
 * For bit-banging drivers that need more than gpiolib's one locked
 * read-modify-write per call. Hold @lock (IRQs off) around every access
 * to @dat, and leave the bits of other lines in the bank as they are:
 * other drivers share the register through gpiolib under the same lock.
 */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS_GPIO_PORT_H
#define __LINUX_SOC_SAMSUNG_EXYNOS_GPIO_PORT_H

#include <linux/errno.h>
#include <linux/spinlock_types.h>
#include <linux/types.h>

struct gpio_desc;

struct samsung_gpio_port {
	void __iomem *dat;	/* the bank's DAT register */
	raw_spinlock_t *lock;	/* the bank lock gpiolib takes */
	u32 mask;		/* the line's bit in @dat */
};

#ifdef CONFIG_PINCTRL_SAMSUNG
int samsung_gpio_get_port(struct gpio_desc *desc,
			  struct samsung_gpio_port *port);
#else
static inline int samsung_gpio_get_port(struct gpio_desc *desc,
					struct samsung_gpio_port *port)
{
	return -ENODEV;
}
#endif

#endif /* __LINUX_SOC_SAMSUNG_EXYNOS_GPIO_PORT_H */

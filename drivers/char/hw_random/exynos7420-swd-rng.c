// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Exynos7420 secure-world TRNG
 *
 * The Exynos7420 TRNG belongs to the secure world. The EL3 firmware that
 * S-Boot loads (CryptoManager) exposes it through one SiP SMC: open it,
 * read 64 bits per call (returned in x2 and x3), close it. Before the
 * first read the driver runs the firmware's FIPS 140-2 start-up test,
 * repeated reads with x2 = 1, until the firmware reports it done.
 *
 * Without this source the kernel has no seeded entropy until about 1 s
 * after /init starts, and the first blocking getrandom() in init stalls
 * the boot until the jitter entropy loop has seeded the CRNG.
 *
 * Interface facts from Samsung's 3.10 exyswd-rng driver (SM-G920F).
 */

#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <linux/hw_random.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#define SWD_RNG_SMC		0x82001012
#define SWD_RNG_INIT		0x0
#define SWD_RNG_EXIT		0x1
#define SWD_RNG_GET_DATA	0x2
#define SWD_RNG_RESUME		0x3

#define SWD_RNG_OK		0
#define SWD_RNG_RETRY		2
#define SWD_RNG_TEST_ERROR	4
#define SWD_RNG_START_UP_DONE	5

#define SWD_RNG_RETRIES		1000
#define SWD_RNG_START_UP_BYTES	(4096 + 1)

struct swd_rng {
	struct hwrng rng;
	struct device *dev;
	bool start_up_done;
	bool failed;
};

static unsigned long swd_rng_smc(unsigned long cmd, unsigned long arg,
				 u64 *data)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SWD_RNG_SMC, cmd, arg, 0, 0, 0, 0, 0, &res);
	if (data)
		*data = (u64)(u32)res.a3 << 32 | (u32)res.a2;
	return res.a0;
}

/* Call until the firmware stops asking for a retry. */
static unsigned long swd_rng_call(unsigned long cmd, unsigned long arg,
				  u64 *data)
{
	unsigned long ret;
	int tries = 0;

	while ((ret = swd_rng_smc(cmd, arg, data)) == SWD_RNG_RETRY &&
	       tries++ < SWD_RNG_RETRIES)
		usleep_range(50, 100);
	return ret;
}

static int swd_rng_start_up_test(struct swd_rng *p)
{
	int left = SWD_RNG_START_UP_BYTES;
	bool rerun = false;
	unsigned long ret;

	while (left > 0) {
		ret = swd_rng_call(SWD_RNG_GET_DATA, 1, NULL);
		if (ret == SWD_RNG_START_UP_DONE)
			return 0;
		if (ret == SWD_RNG_TEST_ERROR && !rerun) {
			/* One false positive in about 10^9: test once more. */
			rerun = true;
			left = SWD_RNG_START_UP_BYTES;
			continue;
		}
		if (ret != SWD_RNG_OK) {
			dev_err(p->dev, "start-up test failed: %lu\n", ret);
			return -EIO;
		}
		left -= 32;
	}
	return 0;
}

static int swd_rng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	struct swd_rng *p = container_of(rng, struct swd_rng, rng);
	size_t done = 0;
	unsigned long ret;
	int err = 0;
	u64 v;

	if (p->failed)
		return -EIO;

	ret = swd_rng_call(SWD_RNG_INIT, 0, NULL);
	if (ret != SWD_RNG_OK)
		return -EIO;

	if (!p->start_up_done) {
		err = swd_rng_start_up_test(p);
		if (err) {
			p->failed = true;
			goto out;
		}
		p->start_up_done = true;
	}

	while (done < max) {
		size_t n = min_t(size_t, max - done, sizeof(v));

		ret = swd_rng_call(SWD_RNG_GET_DATA, 0, &v);
		if (ret != SWD_RNG_OK) {
			/* A failed continuous test: drop this block. */
			err = ret == SWD_RNG_TEST_ERROR ? -EAGAIN : -EIO;
			break;
		}
		memcpy(buf + done, &v, n);
		done += n;
	}
	memzero_explicit(&v, sizeof(v));
out:
	if (swd_rng_call(SWD_RNG_EXIT, 0, NULL) != SWD_RNG_OK)
		dev_warn(p->dev, "close failed\n");
	return done ? done : err;
}

static int swd_rng_probe(struct platform_device *pdev)
{
	struct swd_rng *p;
	int ret;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->dev = &pdev->dev;
	p->rng.name = "exynos7420-swd";
	p->rng.read = swd_rng_read;
	p->rng.quality = 500;	/* Samsung's estimate, per 1024 bits */
	platform_set_drvdata(pdev, p);

	ret = devm_hwrng_register(&pdev->dev, &p->rng);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "hwrng_register\n");
	return 0;
}

static int swd_rng_resume(struct device *dev)
{
	/* Tell the firmware it lost the TRNG's state, as the 3.10 driver did. */
	swd_rng_smc(SWD_RNG_RESUME, 0, NULL);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(swd_rng_pm_ops, NULL, swd_rng_resume);

static const struct of_device_id swd_rng_of_match[] = {
	{ .compatible = "samsung,exynos7420-swd-rng" },
	{ }
};
MODULE_DEVICE_TABLE(of, swd_rng_of_match);

static struct platform_driver swd_rng_driver = {
	.probe = swd_rng_probe,
	.driver = {
		.name = "exynos7420-swd-rng",
		.of_match_table = swd_rng_of_match,
		.pm = pm_sleep_ptr(&swd_rng_pm_ops),
	},
};
module_platform_driver(swd_rng_driver);

MODULE_DESCRIPTION("Exynos7420 secure-world TRNG");
MODULE_LICENSE("GPL");

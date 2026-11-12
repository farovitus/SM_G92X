// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung modem interface (modem_v1), Shannon 333 over MIPI-LLI with an
 * SPI boot link.
 *
 * Copyright (C) 2010 Samsung Electronics.
 * 6.6 port: DT only, gpiod, devm, deferred probe on the LLI driver, and
 * no BUG()/panic on a bad configuration. The platform driver keeps the
 * stock name "mif_sipc5".
 *
 * Fail-soft: probe never waits for the CP. The CP stays powered off until
 * cbd issues IOCTL_MODEM_ON, so a missing or broken modem cannot stall
 * the boot.
 */

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/mipi-lli.h>
#include "modem_prj.h"
#include "modem_utils.h"

#define FMT_WAKE_TIME	(HZ / 2)
#define RAW_WAKE_TIME	(HZ * 6)

static struct modem_shared *create_modem_shared_data(struct device *dev)
{
	struct modem_shared *msd;

	msd = devm_kzalloc(dev, sizeof(*msd), GFP_KERNEL);
	if (!msd)
		return NULL;

	INIT_LIST_HEAD(&msd->link_dev_list);
	msd->iodevs_tree_fmt = RB_ROOT;
	spin_lock_init(&msd->lock);

	return msd;
}

static struct modem_ctl *create_modemctl_device(struct device *dev,
						struct modem_shared *msd,
						struct modem_data *pdata)
{
	struct modem_ctl *mc;
	int ret;

	mc = devm_kzalloc(dev, sizeof(*mc), GFP_KERNEL);
	if (!mc)
		return ERR_PTR(-ENOMEM);

	mc->dev = dev;
	mc->name = pdata->name;
	mc->mdm_data = pdata;
	mc->msd = msd;
	mc->phone_state = STATE_OFFLINE;

	spin_lock_init(&mc->lock);
	init_completion(&mc->init_cmpl);
	init_completion(&mc->off_cmpl);

	ret = ss333_init_modemctl_device(mc, pdata);
	if (ret) {
		mif_err("%s: ss333_init_modemctl_device fail (%d)\n",
			pdata->name, ret);
		return ERR_PTR(ret);
	}

	mif_info("%s is created!!!\n", pdata->name);
	return mc;
}

static struct link_device *create_link_device(struct platform_device *pdev,
					      struct modem_data *pdata,
					      enum modem_link link_type)
{
	switch (link_type) {
	case LINKDEV_LLI:
		return lli_create_link_device(pdev, pdata);
	case LINKDEV_SPI:
		return spi_create_link_device(pdev, pdata);
	default:
		mif_err("%s: link type %d not supported\n", pdata->name,
			link_type);
		return ERR_PTR(-ENODEV);
	}
}

static long iod_waketime(struct io_device *iod)
{
	switch (iod->format) {
	case IPC_FMT:
		return FMT_WAKE_TIME;
	case IPC_BOOT:
	case IPC_DUMP:
		return RAW_WAKE_TIME;
	default:
		break;
	}

	switch (iod->id) {
	case SIPC5_CH_ID_FMT_0 ... SIPC5_CH_ID_FMT_9:
		return FMT_WAKE_TIME;
	case SIPC5_CH_ID_BOOT_0 ... SIPC5_CH_ID_DUMP_9:
		return RAW_WAKE_TIME;
	default:
		return 0;
	}
}

/*
 * Pick the TX link of @iod: its only link, or @tx_link when it has
 * several (umts_boot0 starts on LLI; cbd moves it to SPI with
 * IOCTL_MODEM_SET_TX_LINK). Stock BUG()ed when none matched.
 */
static struct link_device *find_iod_link(struct modem_shared *msd,
					 struct io_device *iod,
					 enum modem_link tx_link)
{
	struct link_device *ld, *found = NULL;

	list_for_each_entry(ld, &msd->link_dev_list, list) {
		if (!IS_CONNECTED(iod, ld))
			continue;
		if (count_bits(iod->link_types) <= 1 ||
		    tx_link == ld->link_type)
			found = ld;
	}
	return found;
}

static void iod_release(void *data)
{
	sipc5_deinit_io_device(data);
}

static struct io_device *create_io_device(struct device *dev,
					  struct modem_io_t *io_t,
					  struct modem_shared *msd,
					  struct modem_ctl *mc,
					  struct modem_data *pdata)
{
	struct link_device *ld;
	struct io_device *iod;
	int ret;

	iod = devm_kzalloc(dev, sizeof(*iod), GFP_KERNEL);
	if (!iod)
		return ERR_PTR(-ENOMEM);

	RB_CLEAR_NODE(&iod->node_fmt);

	iod->name = io_t->name;
	iod->id = io_t->id;
	iod->format = io_t->format;
	iod->io_typ = io_t->io_type;
	iod->link_types = io_t->links;
	iod->attrs = io_t->attrs;
	iod->app = io_t->app;
	iod->net_typ = pdata->modem_net;
	iod->use_handover = pdata->use_handover;
	iod->ipc_version = pdata->ipc_version;
	atomic_set(&iod->opened, 0);
	iod->mc = mc;
	iod->msd = msd;

	ld = find_iod_link(msd, iod, io_t->tx_link);
	if (!ld) {
		mif_err("%s: no link device for links 0x%x, skipped\n",
			iod->name, iod->link_types);
		return ERR_PTR(-ENODEV);
	}
	set_current_link(iod, ld);
	iod->waketime = iod_waketime(iod);

	ret = sipc5_init_io_device(iod);
	if (ret) {
		mif_err("%s: sipc5_init_io_device fail (%d)\n", iod->name, ret);
		return ERR_PTR(ret);
	}
	ret = devm_add_action_or_reset(dev, iod_release, iod);
	if (ret)
		return ERR_PTR(ret);

	if (iod->format == IPC_FMT && iod->id == SIPC5_CH_ID_FMT_0)
		mc->iod = iod;
	if (iod->format == IPC_FMT && iod->id == SIPC5_CH_ID_FMT_1)
		mc->iod_ds = iod;
	if (iod->format == IPC_BOOT) {
		mc->bootd = iod;
		mif_info("BOOT device = %s\n", iod->name);
	}

	if (iod->format != IPC_RAW)
		insert_iod_with_format(msd, iod->format, iod);
	if (sipc5_is_not_reserved_channel(iod->id))
		insert_iod_with_channel(msd, iod->id, iod);

	mif_info("%s created on %s\n", iod->name, ld->name);
	return iod;
}

/* Stock "mif,*" properties are kept, without the "mif," prefix */
static int parse_dt_common_pdata(struct device_node *np,
				 struct modem_data *pdata)
{
	mif_dt_read_string(np, "samsung,modem-name", pdata->name);
	mif_dt_read_enum(np, "samsung,modem-net", pdata->modem_net);
	mif_dt_read_enum(np, "samsung,modem-type", pdata->modem_type);
	mif_dt_read_bool(np, "samsung,use-handover", pdata->use_handover);
	mif_dt_read_enum(np, "samsung,ipc-version", pdata->ipc_version);
	mif_dt_read_u32(np, "samsung,link-types", pdata->link_types);
	mif_dt_read_string(np, "samsung,link-name", pdata->link_name);
	mif_dt_read_u32(np, "samsung,link-attrs", pdata->link_attrs);
	return 0;
}

static int parse_dt_gpio_pdata(struct device *dev, struct modem_data *pdata)
{
	struct device_node *pm_node;
	int ret;

	/* Outputs start low: the CP stays off and in reset until cbd */
	pdata->gpio_cp_on = devm_gpiod_get(dev, "cp-on", GPIOD_OUT_LOW);
	if (IS_ERR(pdata->gpio_cp_on))
		return dev_err_probe(dev, PTR_ERR(pdata->gpio_cp_on), "cp-on\n");

	pdata->gpio_cp_reset = devm_gpiod_get(dev, "cp-reset", GPIOD_OUT_LOW);
	if (IS_ERR(pdata->gpio_cp_reset))
		return dev_err_probe(dev, PTR_ERR(pdata->gpio_cp_reset),
				     "cp-reset\n");

	pdata->gpio_pda_active = devm_gpiod_get(dev, "pda-active",
						GPIOD_OUT_LOW);
	if (IS_ERR(pdata->gpio_pda_active))
		return dev_err_probe(dev, PTR_ERR(pdata->gpio_pda_active),
				     "pda-active\n");

	pdata->gpio_phone_active = devm_gpiod_get(dev, "phone-active",
						  GPIOD_IN);
	if (IS_ERR(pdata->gpio_phone_active))
		return dev_err_probe(dev, PTR_ERR(pdata->gpio_phone_active),
				     "phone-active\n");

	pdata->gpio_ipc_int2cp = devm_gpiod_get_optional(dev, "ipc-int2cp",
							 GPIOD_OUT_LOW);
	if (IS_ERR(pdata->gpio_ipc_int2cp))
		return dev_err_probe(dev, PTR_ERR(pdata->gpio_ipc_int2cp),
				     "ipc-int2cp\n");

	pdata->gpio_ap_dump_int = devm_gpiod_get(dev, "ap-dump-int",
						 GPIOD_OUT_LOW);
	if (IS_ERR(pdata->gpio_ap_dump_int))
		return dev_err_probe(dev, PTR_ERR(pdata->gpio_ap_dump_int),
				     "ap-dump-int\n");

	pdata->gpio_cp_dump_int = devm_gpiod_get(dev, "cp-dump-int", GPIOD_IN);
	if (IS_ERR(pdata->gpio_cp_dump_int))
		return dev_err_probe(dev, PTR_ERR(pdata->gpio_cp_dump_int),
				     "cp-dump-int\n");

	/* gpa1-0: the modem's cp_status and the link PM's cp2ap_status */
	pdata->gpio_cp_status = devm_gpiod_get(dev, "cp-status", GPIOD_IN);
	if (IS_ERR(pdata->gpio_cp_status))
		return dev_err_probe(dev, PTR_ERR(pdata->gpio_cp_status),
				     "cp-status\n");

	pm_node = of_get_child_by_name(dev->of_node, PM_DT_NODE_NAME);
	if (!pm_node)
		return dev_err_probe(dev, -ENODEV, "no %s node\n",
				     PM_DT_NODE_NAME);

	ret = link_pm_parse_dt_gpio_pdata(dev, pm_node, pdata);
	of_node_put(pm_node);
	return ret;
}

static int parse_dt_iodevs_pdata(struct device *dev, struct device_node *np,
				 struct modem_data *pdata)
{
	struct device_node *child;
	unsigned int i = 0;

	pdata->num_iodevs = of_get_available_child_count(np);
	if (!pdata->num_iodevs)
		return -ENODEV;

	pdata->iodevs = devm_kcalloc(dev, pdata->num_iodevs,
				     sizeof(*pdata->iodevs), GFP_KERNEL);
	if (!pdata->iodevs)
		return -ENOMEM;

	for_each_available_child_of_node(np, child) {
		struct modem_io_t *iod = &pdata->iodevs[i];
		int ret = -EINVAL;

		if (of_property_read_string(child, "iod,name", &iod->name) ||
		    of_property_read_u32(child, "iod,id", &iod->id) ||
		    of_property_read_u32(child, "iod,format",
					 (u32 *)&iod->format) ||
		    of_property_read_u32(child, "iod,io-type",
					 (u32 *)&iod->io_type) ||
		    of_property_read_u32(child, "iod,links", &iod->links) ||
		    of_property_read_u32(child, "iod,attrs", &iod->attrs)) {
			mif_err("%pOF: missing iod property\n", child);
			of_node_put(child);
			return ret;
		}

		if (count_bits(iod->links) > 1 &&
		    of_property_read_u32(child, "iod,tx-link",
					 (u32 *)&iod->tx_link)) {
			mif_err("%s: 2+ links and no iod,tx-link\n", iod->name);
			of_node_put(child);
			return ret;
		}

		of_property_read_string(child, "iod,app", &iod->app);

		if (iod->attrs & IODEV_ATTR(ATTR_SBD_IPC)) {
			of_property_read_u32(child, "iod,ul-num-buffers",
					     &iod->ul_num_buffers);
			of_property_read_u32(child, "iod,ul-buffer-size",
					     &iod->ul_buffer_size);
			of_property_read_u32(child, "iod,dl-num-buffers",
					     &iod->dl_num_buffers);
			of_property_read_u32(child, "iod,dl-buffer-size",
					     &iod->dl_buffer_size);
		}
		i++;
	}

	return 0;
}

static struct modem_data *modem_parse_dt(struct device *dev)
{
	struct device_node *iodevs;
	struct modem_data *pdata;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	ret = parse_dt_common_pdata(dev->of_node, pdata);
	if (ret) {
		mif_err("DT error: failed to parse common\n");
		return ERR_PTR(ret);
	}

	ret = parse_dt_gpio_pdata(dev, pdata);
	if (ret)
		return ERR_PTR(ret);

	iodevs = of_get_child_by_name(dev->of_node, "iodevs");
	if (!iodevs) {
		mif_err("DT error: no iodevs node\n");
		return ERR_PTR(-EINVAL);
	}
	ret = parse_dt_iodevs_pdata(dev, iodevs, pdata);
	of_node_put(iodevs);
	if (ret) {
		mif_err("DT error: failed to parse iodevs (%d)\n", ret);
		return ERR_PTR(ret);
	}

	mif_info("DT parse complete!\n");
	return pdata;
}

static int modem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct modem_data *pdata;
	struct modem_shared *msd;
	struct modem_ctl *mc;
	struct link_device *ld;
	unsigned int i, created = 0;

	pdata = modem_parse_dt(dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);

	msd = create_modem_shared_data(dev);
	if (!msd)
		return -ENOMEM;

	mc = create_modemctl_device(dev, msd, pdata);
	if (IS_ERR(mc))
		return PTR_ERR(mc);

	for (i = 0; i < LINKDEV_MAX; i++) {
		if (!(pdata->link_types & LINKTYPE(i)))
			continue;

		ld = create_link_device(pdev, pdata, i);
		if (IS_ERR(ld)) {
			if (PTR_ERR(ld) == -EPROBE_DEFER)
				return dev_err_probe(dev, -EPROBE_DEFER,
						     "link %d not ready\n", i);
			mif_err("%s: link %d creation failed (%ld), continue\n",
				pdata->name, i, PTR_ERR(ld));
			continue;
		}

		spin_lock_init(&ld->lock);
		ld->link_type = i;
		ld->mc = mc;
		ld->msd = msd;
		ld->dev = dev;
		ld->state = LINK_STATE_OFFLINE;
		list_add(&ld->list, &msd->link_dev_list);
		mif_info("%s: %s link created\n", pdata->name, ld->name);
	}

	if (list_empty(&msd->link_dev_list))
		return dev_err_probe(dev, -ENODEV, "no link device\n");

	for (i = 0; i < pdata->num_iodevs; i++) {
		struct io_device *iod;

		iod = create_io_device(dev, &pdata->iodevs[i], msd, mc, pdata);
		if (IS_ERR(iod))
			continue;	/* fail-soft: keep the other nodes */
		created++;
	}

	if (!mc->iod || !mc->bootd)
		return dev_err_probe(dev, -ENODEV,
				     "no FMT (umts_ipc0) or BOOT (umts_boot0) node\n");

	platform_set_drvdata(pdev, mc);
	mif_info("%s: %u/%u io devices\n", pdata->name, created,
		 pdata->num_iodevs);
	return 0;
}

static void modem_shutdown(struct platform_device *pdev)
{
	struct modem_ctl *mc = platform_get_drvdata(pdev);

	if (!mc)
		return;

	mc->ops.modem_off(mc);
	mc->phone_state = STATE_OFFLINE;
	mif_info("%s(%s)\n", mc->name, FUNC);
}

/* PDA_ACTIVE tells the CP whether the AP is awake */
static int modem_suspend(struct device *dev)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);
	struct link_device *ld = mc->iod ? get_current_link(mc->iod) : NULL;
	struct modem_link_pm *pm = ld ? &ld->pm : NULL;

	/*
	 * The link-PM wakeup source keeps the system up while the link is
	 * mounted, but a suspend that does not check wakeup counts goes ahead
	 * anyway. Lowering PDA_ACTIVE then makes the CP release a link whose
	 * LLI is about to lose power: the unmount never completes, and the
	 * link-PM watchdog crash path touches the dead LLI (SError).
	 */
	if (pm && pm->link_mounted && pm->link_mounted(pm)) {
		dev_info_ratelimited(dev, "%s link mounted (%s), not suspending\n",
				     pm->link_name,
				     pm_state2str(pm->fsm.state));
		return -EBUSY;
	}

	gpiod_set_value(mc->gpio_pda_active, 0);
	mif_info("%s: %s\n", FUNC, mc->name);
	return 0;
}

static int modem_resume(struct device *dev)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);

	gpiod_set_value(mc->gpio_pda_active, 1);
	mif_info("%s: %s\n", FUNC, mc->name);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(modem_pm_ops, modem_suspend, modem_resume);

/*
 * cbd polls <device>/cp_status/value while the CP boot loader starts. The
 * stock kernel made that path with gpio_export_link(); offer the same
 * read-only file as an attribute group instead of the GPIO sysfs ABI.
 */
static ssize_t value_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);

	if (!mc)
		return -ENODEV;
	return sysfs_emit(buf, "%d\n",
			  gpiod_get_value_cansleep(mc->gpio_cp_status));
}
static DEVICE_ATTR_RO(value);

static struct attribute *cp_status_attrs[] = {
	&dev_attr_value.attr,
	NULL
};

static const struct attribute_group cp_status_group = {
	.name = "cp_status",
	.attrs = cp_status_attrs,
};

static const struct attribute_group *modem_groups[] = {
	&cp_status_group,
	NULL
};

static const struct of_device_id modem_of_match[] = {
	{ .compatible = "samsung,ss333-modem" },
	{ }
};
MODULE_DEVICE_TABLE(of, modem_of_match);

static struct platform_driver modem_driver = {
	.probe = modem_probe,
	.shutdown = modem_shutdown,
	.driver = {
		.name = "mif_sipc5",
		.pm = pm_sleep_ptr(&modem_pm_ops),
		.of_match_table = modem_of_match,
		.dev_groups = modem_groups,
		/* io devices, net devices and IRQs cannot be torn down */
		.suppress_bind_attrs = true,
	},
};

static int __init modem_init(void)
{
	int ret;

	ret = spi_boot_link_register();
	if (ret)
		return ret;

	ret = platform_driver_register(&modem_driver);
	if (ret)
		spi_boot_link_unregister();
	return ret;
}
module_init(modem_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung Shannon 333 modem interface (modem_v1)");

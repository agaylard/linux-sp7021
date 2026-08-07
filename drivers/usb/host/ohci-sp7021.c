// SPDX-License-Identifier: GPL-2.0
/*
 * Sunplus SP7021 OHCI host controller driver
 *
 * Copyright (C) 2021 Sunplus Technology Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "ohci.h"

static struct hc_driver __read_mostly ohci_sp7021_hc_driver;

static const struct ohci_driver_overrides sp7021_overrides __initconst = {
	.product_desc	= "SP7021 OHCI",
	.reset		= ohci_setup,
};

static int ohci_sp7021_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct resource *res;
	struct clk *clk;
	int irq, err;

	if (usb_disabled())
		return -ENODEV;

	clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(clk),
				     "failed to enable clock\n");

	{
		struct reset_control *rst =
			devm_reset_control_get_optional_shared(&pdev->dev, NULL);
		if (!IS_ERR_OR_NULL(rst))
			reset_control_deassert(rst);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	hcd = usb_create_hcd(&ohci_sp7021_hc_driver, &pdev->dev,
			     dev_name(&pdev->dev));
	if (!hcd)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hcd->regs)) {
		err = PTR_ERR(hcd->regs);
		goto put_hcd;
	}

	err = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (err)
		goto put_hcd;

	platform_set_drvdata(pdev, hcd);
	return 0;

put_hcd:
	usb_put_hcd(hcd);
	return err;
}

static void ohci_sp7021_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
}

#ifdef CONFIG_PM_SLEEP
static int ohci_sp7021_suspend(struct device *dev)
{
	return ohci_suspend(dev_get_drvdata(dev), device_may_wakeup(dev));
}

static int ohci_sp7021_resume(struct device *dev)
{
	ohci_resume(dev_get_drvdata(dev), false);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ohci_sp7021_pm_ops,
			 ohci_sp7021_suspend, ohci_sp7021_resume);

static const struct of_device_id ohci_sp7021_dt_ids[] = {
	{ .compatible = "sunplus,sp7021-usb-ohci0" },
	{ .compatible = "sunplus,sp7021-usb-ohci1" },
	{ }
};
MODULE_DEVICE_TABLE(of, ohci_sp7021_dt_ids);

static struct platform_driver ohci_sp7021_driver = {
	.probe		= ohci_sp7021_probe,
	.remove		= ohci_sp7021_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver = {
		.name		= "ohci-sp7021",
		.of_match_table	= ohci_sp7021_dt_ids,
		.pm		= &ohci_sp7021_pm_ops,
	},
};

static int __init ohci_sp7021_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	ohci_init_driver(&ohci_sp7021_hc_driver, &sp7021_overrides);
	return platform_driver_register(&ohci_sp7021_driver);
}
module_init(ohci_sp7021_init);

static void __exit ohci_sp7021_exit(void)
{
	platform_driver_unregister(&ohci_sp7021_driver);
}
module_exit(ohci_sp7021_exit);

MODULE_AUTHOR("Sunplus Technology Inc.");
MODULE_DESCRIPTION("Sunplus SP7021 OHCI host controller driver");
MODULE_LICENSE("GPL");

// SPDX-License-Identifier: GPL-2.0
/*
 * Sunplus SP7021 EHCI host controller driver
 *
 * Copyright (C) 2021 Sunplus Technology Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/of.h>

#include "ehci.h"

struct ehci_sp7021_priv {
	struct phy *phy;
};

#define hcd_to_sp7021_priv(h) ((struct ehci_sp7021_priv *)hcd_to_ehci(h)->priv)

static struct hc_driver __read_mostly ehci_sp7021_hc_driver;

static int ehci_sp7021_reset(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	ehci->caps = hcd->regs;
	return ehci_setup(hcd);
}

static const struct ehci_driver_overrides sp7021_overrides __initconst = {
	.reset			= ehci_sp7021_reset,
	.extra_priv_size	= sizeof(struct ehci_sp7021_priv),
};

static int ehci_sp7021_probe(struct platform_device *pdev)
{
	struct ehci_sp7021_priv *priv;
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

	hcd = usb_create_hcd(&ehci_sp7021_hc_driver, &pdev->dev,
			     dev_name(&pdev->dev));
	if (!hcd)
		return -ENOMEM;

	priv = hcd_to_sp7021_priv(hcd);

	priv->phy = devm_phy_optional_get(&pdev->dev, "usb");
	if (IS_ERR(priv->phy)) {
		err = PTR_ERR(priv->phy);
		goto put_hcd;
	}

	dev_info(&pdev->dev, "PHY %s\n", priv->phy ? "found" : "absent");

	err = phy_init(priv->phy);
	if (err) {
		dev_err(&pdev->dev, "phy_init failed: %d\n", err);
		goto put_hcd;
	}

	err = phy_power_on(priv->phy);
	if (err) {
		dev_err(&pdev->dev, "phy_power_on failed: %d\n", err);
		goto phy_exit;
	}

	dev_info(&pdev->dev, "PHY powered on\n");
	msleep(50);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hcd->regs)) {
		err = PTR_ERR(hcd->regs);
		goto phy_off;
	}

	hcd->tpl_support = of_usb_host_tpl_support(pdev->dev.of_node);

	err = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (err)
		goto phy_off;

	platform_set_drvdata(pdev, hcd);

	return 0;

phy_off:
	phy_power_off(priv->phy);
phy_exit:
	phy_exit(priv->phy);
put_hcd:
	usb_put_hcd(hcd);
	return err;
}

static void ehci_sp7021_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_sp7021_priv *priv = hcd_to_sp7021_priv(hcd);

	usb_remove_hcd(hcd);
	phy_power_off(priv->phy);
	phy_exit(priv->phy);
	usb_put_hcd(hcd);
}

#ifdef CONFIG_PM_SLEEP
static int ehci_sp7021_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct ehci_sp7021_priv *priv = hcd_to_sp7021_priv(hcd);
	int ret;

	ret = ehci_suspend(hcd, device_may_wakeup(dev));
	if (ret)
		return ret;

	phy_power_off(priv->phy);
	return 0;
}

static int ehci_sp7021_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct ehci_sp7021_priv *priv = hcd_to_sp7021_priv(hcd);
	int ret;

	ret = phy_power_on(priv->phy);
	if (ret)
		return ret;

	ehci_resume(hcd, false);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ehci_sp7021_pm_ops,
			 ehci_sp7021_suspend, ehci_sp7021_resume);

static const struct of_device_id ehci_sp7021_dt_ids[] = {
	{ .compatible = "sunplus,sp7021-usb-ehci0" },
	{ .compatible = "sunplus,sp7021-usb-ehci1" },
	{ }
};
MODULE_DEVICE_TABLE(of, ehci_sp7021_dt_ids);

static struct platform_driver ehci_sp7021_driver = {
	.probe		= ehci_sp7021_probe,
	.remove		= ehci_sp7021_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver = {
		.name		= "ehci-sp7021",
		.of_match_table	= ehci_sp7021_dt_ids,
		.pm		= &ehci_sp7021_pm_ops,
	},
};

static int __init ehci_sp7021_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	ehci_init_driver(&ehci_sp7021_hc_driver, &sp7021_overrides);
	return platform_driver_register(&ehci_sp7021_driver);
}
module_init(ehci_sp7021_init);

static void __exit ehci_sp7021_exit(void)
{
	platform_driver_unregister(&ehci_sp7021_driver);
}
module_exit(ehci_sp7021_exit);

MODULE_AUTHOR("Sunplus Technology Inc.");
MODULE_DESCRIPTION("Sunplus SP7021 EHCI host controller driver");
MODULE_LICENSE("GPL");

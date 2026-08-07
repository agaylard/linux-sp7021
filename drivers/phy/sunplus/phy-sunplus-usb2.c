// SPDX-License-Identifier: GPL-2.0

/*
 * Sunplus SP7021 USB 2.0 phy driver
 *
 * Copyright (C) 2022 Sunplus Technology Inc., All rights reserved.
 *
 * Ported from 5.10 vendor driver (phy0-sunplus.c / phy1-sunplus.c)
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

/* Mask-write helpers (SP7021 register format: bits 31:16 = mask, bits 15:0 = value) */
#define RF_MASK_V(mask, val)		(((mask) << 16) | (val))
#define RF_MASK_V_SET(bits)		RF_MASK_V(bits, bits)
#define RF_MASK_V_CLR(bits)		RF_MASK_V(bits, 0)

/* GROUP UPHY (phy_regs offsets) */
#define DISC_LEVEL_OFFSET		0x1c	/* CONFIG7 */
#define OTP_DISC_LEVEL_BIT		0x1f
#define OTP_DISC_LEVEL_DEFAULT		0xd
#define ECO_PATH_OFFSET			0x24	/* CONFIG9 */
#define ECO_PATH_SET			BIT(6)
#define POWER_SAVING_OFFSET		0x4	/* CONFIG1 */
#define POWER_SAVING_SET		BIT(5)
#define APHY_PROBE_OFFSET		0x5c	/* CONFIG23 */
#define APHY_PROBE_CTRL_MASK		GENMASK(5, 3)
#define APHY_PROBE_CTRL			FIELD_PREP(APHY_PROBE_CTRL_MASK, 7)
#define CDP_REG_OFFSET			0x40	/* CONFIG16 */
#define DCP_REG_OFFSET			0x44	/* CONFIG17 */
#define UPHY_INTER_SIGNAL_REG_OFFSET	0xc	/* CONFIG3 */

/* GROUP MOON0 (moon0_regs offsets) */
#define USB_RESET_OFFSET		0x5c	/* G0.23 */
#define UPHY0_RST_BIT			BIT(13)
#define USBC0_RST_BIT			BIT(10)
#define UPHY1_RST_BIT			BIT(14)	/* G0.23 bit 14 for UPHY1 */
#define USBC1_RST_BIT			BIT(11)	/* G0.23 bit 11 for USBC1 */

/* GROUP MOON4 (moon4_regs offsets, from G4.0 base = 0x9c000200) */
#define USBC_CTL_OFFSET			0x44	/* G4.17 */
#define UPHY0_CTL0_OFFSET		0x48	/* G4.18 */
#define UPHY0_CTL1_OFFSET		0x4c	/* G4.19 */
#define UPHY0_CTL2_OFFSET		0x50	/* G4.20 */
#define UPHY0_CTL3_OFFSET		0x54	/* G4.21 */
#define UPHY1_CTL0_OFFSET		0x58	/* G4.22 */
#define UPHY1_CTL1_OFFSET		0x5c	/* G4.23 */
#define UPHY1_CTL2_OFFSET		0x60	/* G4.24 */
#define UPHY1_CTL3_OFFSET		0x64	/* G4.25 */

struct sp_usbphy {
	struct device		*dev;
	struct clk		*phy_clk;
	void __iomem		*phy_regs;
	void __iomem		*moon0_regs;
	void __iomem		*moon4_regs;
	u32			uphy_index; /* 0 or 1 */
	u32			disc_vol_addr_off;
};

static int update_disc_vol(struct sp_usbphy *usbphy)
{
	struct nvmem_cell *cell;
	ssize_t otp_l = 0;
	char *otp_v;
	u32 val, set = 0;

	cell = nvmem_cell_get(usbphy->dev, "disc_vol");
	if (!IS_ERR_OR_NULL(cell)) {
		otp_v = nvmem_cell_read(cell, &otp_l);
		nvmem_cell_put(cell);
		if (!IS_ERR(otp_v)) {
			set = *otp_v & OTP_DISC_LEVEL_BIT;
			kfree(otp_v);
		}
	}

	if (set == 0)
		set = OTP_DISC_LEVEL_DEFAULT;

	val = readl(usbphy->phy_regs + DISC_LEVEL_OFFSET);
	val = (val & ~OTP_DISC_LEVEL_BIT) | set;
	writel(val, usbphy->phy_regs + DISC_LEVEL_OFFSET);

	return 0;
}

static int sp_uphy_init(struct phy *phy)
{
	struct sp_usbphy *usbphy = phy_get_drvdata(phy);
	u32 uphy_rst_bit  = usbphy->uphy_index ? UPHY1_RST_BIT  : UPHY0_RST_BIT;
	u32 usbc_rst_bit  = usbphy->uphy_index ? USBC1_RST_BIT  : USBC0_RST_BIT;
	u32 ctl0_off = usbphy->uphy_index ? UPHY1_CTL0_OFFSET : UPHY0_CTL0_OFFSET;
	u32 ctl1_off = usbphy->uphy_index ? UPHY1_CTL1_OFFSET : UPHY0_CTL1_OFFSET;
	u32 ctl2_off = usbphy->uphy_index ? UPHY1_CTL2_OFFSET : UPHY0_CTL2_OFFSET;
	u32 ctl3_off = usbphy->uphy_index ? UPHY1_CTL3_OFFSET : UPHY0_CTL3_OFFSET;
	u32 val;

	/* 1. Reset UPHY */
	writel(RF_MASK_V_SET(uphy_rst_bit), usbphy->moon0_regs + USB_RESET_OFFSET);
	writel(RF_MASK_V_CLR(uphy_rst_bit), usbphy->moon0_regs + USB_RESET_OFFSET);
	mdelay(1);

	/* 2. Default value modification (MOON4 UPHY CTL0/1) */
	writel(RF_MASK_V(0xffff, 0x4002), usbphy->moon4_regs + ctl0_off);
	writel(RF_MASK_V(0xffff, 0x8747), usbphy->moon4_regs + ctl1_off);

	/* 3. PLL power off/on twice (MOON4 UPHY CTL3) */
	writel(RF_MASK_V(0xffff, 0x88), usbphy->moon4_regs + ctl3_off);
	mdelay(1);
	writel(RF_MASK_V(0xffff, 0x80), usbphy->moon4_regs + ctl3_off);
	mdelay(1);
	writel(RF_MASK_V(0xffff, 0x88), usbphy->moon4_regs + ctl3_off);
	mdelay(1);
	writel(RF_MASK_V(0xffff, 0x80), usbphy->moon4_regs + ctl3_off);
	mdelay(1);
	writel(RF_MASK_V(0xffff, 0x00), usbphy->moon4_regs + ctl3_off);

	/* 4. PHY internal register modifications */
	update_disc_vol(usbphy);

	val = readl(usbphy->phy_regs + ECO_PATH_OFFSET);
	val &= ~ECO_PATH_SET;
	writel(val, usbphy->phy_regs + ECO_PATH_OFFSET);

	val = readl(usbphy->phy_regs + POWER_SAVING_OFFSET);
	val &= ~POWER_SAVING_SET;
	writel(val, usbphy->phy_regs + POWER_SAVING_OFFSET);

	val = readl(usbphy->phy_regs + APHY_PROBE_OFFSET);
	val = (val & ~APHY_PROBE_CTRL_MASK) | APHY_PROBE_CTRL;
	writel(val, usbphy->phy_regs + APHY_PROBE_OFFSET);

	/* 5. Reset USBC */
	writel(RF_MASK_V_SET(usbc_rst_bit), usbphy->moon0_regs + USB_RESET_OFFSET);
	writel(RF_MASK_V_CLR(usbc_rst_bit), usbphy->moon0_regs + USB_RESET_OFFSET);

	/* 6. UPHY clock fix (set RX_CLK_SEL in CTL2) */
	writel(RF_MASK_V_SET(BIT(6)), usbphy->moon4_regs + ctl2_off);

	/* 7. Host mode: clear USB_CTRL bit so OTG hardware routes to EHCI.
	 * Port 0 clears bit 4 (USB0_CTRL); port 1 clears bits 13:12 (USB1_SEL+CTRL).
	 * Matches the state left by the 5.10 OTG driver after it deregisters SW control. */
	if (usbphy->uphy_index == 0)
		writel(RF_MASK_V_CLR(BIT(4)), usbphy->moon4_regs + USBC_CTL_OFFSET);
	else
		writel(RF_MASK_V_CLR(3 << 12), usbphy->moon4_regs + USBC_CTL_OFFSET);

	/* 8. AC & ACB charge current settings */
	writel(RF_MASK_V_SET(BIT(11)), usbphy->moon4_regs + ctl3_off);
	writel(RF_MASK_V_SET(BIT(14)), usbphy->moon4_regs + ctl3_off);

	/* 9. Additional PHY internal signal settings */
	writel(0x19, usbphy->phy_regs + CDP_REG_OFFSET);
	writel(0x92, usbphy->phy_regs + DCP_REG_OFFSET);
	writel(0x21, usbphy->phy_regs + UPHY_INTER_SIGNAL_REG_OFFSET);

	return 0;
}

static int sp_uphy_power_on(struct phy *phy)
{
	struct sp_usbphy *usbphy = phy_get_drvdata(phy);

	return clk_prepare_enable(usbphy->phy_clk);
}

static int sp_uphy_power_off(struct phy *phy)
{
	struct sp_usbphy *usbphy = phy_get_drvdata(phy);

	clk_disable_unprepare(usbphy->phy_clk);
	return 0;
}

static int sp_uphy_exit(struct phy *phy)
{
	return 0;
}

static const struct phy_ops sp_uphy_ops = {
	.init		= sp_uphy_init,
	.power_on	= sp_uphy_power_on,
	.power_off	= sp_uphy_power_off,
	.exit		= sp_uphy_exit,
};

static const struct of_device_id sp_uphy_dt_ids[] = {
	{.compatible = "sunplus,sp7021-usb2-phy", },
	{ }
};
MODULE_DEVICE_TABLE(of, sp_uphy_dt_ids);

static int sp_usb_phy_probe(struct platform_device *pdev)
{
	struct sp_usbphy *usbphy;
	struct phy_provider *phy_provider;
	struct phy *phy;
	struct resource *res;

	usbphy = devm_kzalloc(&pdev->dev, sizeof(*usbphy), GFP_KERNEL);
	if (!usbphy)
		return -ENOMEM;

	usbphy->dev = &pdev->dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy");
	usbphy->phy_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(usbphy->phy_regs))
		return PTR_ERR(usbphy->phy_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "moon0");
	if (!res)
		return -EINVAL;
	usbphy->moon0_regs = devm_ioremap(&pdev->dev, res->start,
					  resource_size(res));
	if (!usbphy->moon0_regs)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "moon4");
	if (!res)
		return -EINVAL;
	usbphy->moon4_regs = devm_ioremap(&pdev->dev, res->start,
					  resource_size(res));
	if (!usbphy->moon4_regs)
		return -ENOMEM;

	usbphy->phy_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(usbphy->phy_clk))
		return PTR_ERR(usbphy->phy_clk);

	/* Determine port index from node address: 0x4a80 = port0, 0x4b00 = port1 */
	{
		struct resource *phy_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy");
		usbphy->uphy_index = (phy_res->start == 0x9c004b00) ? 1 : 0;
	}

	of_property_read_u32(pdev->dev.of_node, "sunplus,disc-vol-addr-off",
			     &usbphy->disc_vol_addr_off);

	/* Enable clock before init */
	if (clk_prepare_enable(usbphy->phy_clk))
		return -EIO;

	phy = devm_phy_create(&pdev->dev, NULL, &sp_uphy_ops);
	if (IS_ERR(phy)) {
		clk_disable_unprepare(usbphy->phy_clk);
		return PTR_ERR(phy);
	}

	phy_set_drvdata(phy, usbphy);
	phy_provider = devm_of_phy_provider_register(&pdev->dev, of_phy_simple_xlate);

	dev_info(&pdev->dev, "USB 2.0 PHY port %d registered\n", usbphy->uphy_index);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver sunplus_usb_phy_driver = {
	.probe		= sp_usb_phy_probe,
	.driver		= {
		.name		= "sunplus-usb2-phy",
		.of_match_table	= sp_uphy_dt_ids,
	},
};
module_platform_driver(sunplus_usb_phy_driver);

MODULE_AUTHOR("Sunplus Technology Inc.");
MODULE_DESCRIPTION("Sunplus SP7021 USB 2.0 PHY driver");
MODULE_LICENSE("GPL");

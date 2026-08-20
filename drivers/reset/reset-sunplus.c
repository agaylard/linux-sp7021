// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
/*
 * SP7021 reset driver
 *
 * Copyright (C) Sunplus Technology Co., Ltd.
 *       All rights reserved.
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/reboot.h>
#include <linux/soc/sunplus/sp7021.h>
#include <dt-bindings/reset/sunplus,sp7021-reset.h>


/*
 * Reset register bit positions, encoded as (register * MOON_REG_MASK_SHIFT + bit).
 * Indexed by the RST_* constants from <dt-bindings/reset/sunplus,sp7021-reset.h>.
 * Registers are mo_reset0~9 (G0.21~G0.30) in the SP7021 Moon register block.
 */
static const u32 sp_resets[] = {
	[RST_SYSTEM]        = 0x00,
	[RST_RTC]           = 0x02,
	[RST_IOCTL]         = 0x03,
	[RST_IOP]           = 0x04,
	[RST_OTPRX]         = 0x05,
	[RST_NOC]           = 0x06,
	[RST_BR]            = 0x07,
	[RST_RBUS_L00]      = 0x08,
	[RST_SPIFL]         = 0x09,
	[RST_SDCTRL0]       = 0x0a,
	[RST_PERI0]         = 0x0b,
	[RST_A926]          = 0x0d,
	[RST_UMCTL2]        = 0x0e,
	[RST_PERI1]         = 0x0f,
	[RST_DDR_PHY0]      = 0x10,
	[RST_ACHIP]         = 0x12,
	[RST_STC0]          = 0x14,
	[RST_STC_AV0]       = 0x15,
	[RST_STC_AV1]       = 0x16,
	[RST_STC_AV2]       = 0x17,
	[RST_UA0]           = 0x18,
	[RST_UA1]           = 0x19,
	[RST_UA2]           = 0x1a,
	[RST_UA3]           = 0x1b,
	[RST_UA4]           = 0x1c,
	[RST_HWUA]          = 0x1d,
	[RST_DDC0]          = 0x1e,
	[RST_UADMA]         = 0x1f,
	[RST_CBDMA0]        = 0x20,
	[RST_CBDMA1]        = 0x21,
	[RST_SPI_COMBO_0]   = 0x22,
	[RST_SPI_COMBO_1]   = 0x23,
	[RST_SPI_COMBO_2]   = 0x24,
	[RST_SPI_COMBO_3]   = 0x25,
	[RST_AUD]           = 0x26,
	[RST_USBC0]         = 0x2a,
	[RST_USBC1]         = 0x2b,
	[RST_UPHY0]         = 0x2d,
	[RST_UPHY1]         = 0x2e,
	[RST_I2CM0]         = 0x30,
	[RST_I2CM1]         = 0x31,
	[RST_I2CM2]         = 0x32,
	[RST_I2CM3]         = 0x33,
	[RST_PMC]           = 0x3d,
	[RST_CARD_CTL0]     = 0x3e,
	[RST_CARD_CTL1]     = 0x3f,
	[RST_CARD_CTL4]     = 0x42,
	[RST_BCH]           = 0x44,
	[RST_DDFCH]         = 0x4b,
	[RST_CSIIW0]        = 0x4c,
	[RST_CSIIW1]        = 0x4d,
	[RST_MIPICSI0]      = 0x4e,
	[RST_MIPICSI1]      = 0x4f,
	[RST_HDMI_TX]       = 0x50,
	[RST_VPOST]         = 0x55,
	[RST_TGEN]          = 0x60,
	[RST_DMIX]          = 0x61,
	[RST_TCON]          = 0x6a,
	[RST_INTERRUPT]     = 0x6f,
	[RST_RGST]          = 0x70,
	[RST_GPIO]          = 0x73,
	[RST_RBUS_TOP]      = 0x74,
	[RST_MAILBOX]       = 0x86,
	[RST_SPIND]         = 0x8a,
	[RST_I2C2CBUS]      = 0x8b,
	[RST_SEC]           = 0x8d,
	[RST_DVE]           = 0x8e,
	[RST_GPOST0]        = 0x8f,
	[RST_OSD0]          = 0x90,
	[RST_DISP_PWM]      = 0x92,
	[RST_UADBG]         = 0x93,
	[RST_DUMMY_MASTER]  = 0x94,
	[RST_FIO_CTL]       = 0x95,
	[RST_FPGA]          = 0x96,
	[RST_L2SW]          = 0x97,
	[RST_ICM]           = 0x98,
	[RST_AXI_GLOBAL]    = 0x99,
};

struct sp_reset {
	struct reset_controller_dev rcdev;
	void __iomem *base;
};

static inline struct sp_reset *to_sp_reset(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct sp_reset, rcdev);
}

static int sp_reset_update(struct reset_controller_dev *rcdev,
			   unsigned long id, bool assert)
{
	struct sp_reset *reset = to_sp_reset(rcdev);
	int index = sp_resets[id] / MOON_REG_MASK_SHIFT;
	int shift = sp_resets[id] % MOON_REG_MASK_SHIFT;

	writel(MOON_REG_WRITE(BIT(shift), assert ? BIT(shift) : 0),
	       reset->base + (index * 4));

	return 0;
}

static int sp_reset_assert(struct reset_controller_dev *rcdev,
			   unsigned long id)
{
	return sp_reset_update(rcdev, id, true);
}

static int sp_reset_deassert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	return sp_reset_update(rcdev, id, false);
}

static int sp_reset_status(struct reset_controller_dev *rcdev,
			   unsigned long id)
{
	struct sp_reset *reset = to_sp_reset(rcdev);
	int index = sp_resets[id] / MOON_REG_MASK_SHIFT;
	int shift = sp_resets[id] % MOON_REG_MASK_SHIFT;
	u32 reg;

	reg = readl(reset->base + (index * 4));

	return !!(reg & BIT(shift));
}

static const struct reset_control_ops sp_reset_ops = {
	.assert   = sp_reset_assert,
	.deassert = sp_reset_deassert,
	.status   = sp_reset_status,
};

static int sp_restart(struct sys_off_data *data)
{
	struct sp_reset *reset = data->cb_data;

	sp_reset_assert(&reset->rcdev, 0);
	sp_reset_deassert(&reset->rcdev, 0);

	return NOTIFY_DONE;
}

static int sp_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sp_reset *reset;
	struct resource *res;
	int ret;

	reset = devm_kzalloc(dev, sizeof(*reset), GFP_KERNEL);
	if (!reset)
		return -ENOMEM;

	reset->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(reset->base))
		return PTR_ERR(reset->base);

	reset->rcdev.ops = &sp_reset_ops;
	reset->rcdev.owner = THIS_MODULE;
	reset->rcdev.of_node = dev->of_node;
	reset->rcdev.nr_resets = resource_size(res) / 4 * MOON_REG_MASK_SHIFT;

	ret = devm_reset_controller_register(dev, &reset->rcdev);
	if (ret)
		return ret;

	return devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_RESTART,
					     192, sp_restart, reset);
}

static const struct of_device_id sp_reset_dt_ids[] = {
	{.compatible = "sunplus,sp7021-reset",},
	{ /* sentinel */ },
};

static struct platform_driver sp_reset_driver = {
	.probe = sp_reset_probe,
	.driver = {
		.name			= "sunplus-reset",
		.of_match_table		= sp_reset_dt_ids,
		.suppress_bind_attrs	= true,
	},
};
builtin_platform_driver(sp_reset_driver);

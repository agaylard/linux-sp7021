// SPDX-License-Identifier: GPL-2.0-only
/*
 * Thermal sensor driver for Sunplus SP7021 SoC.
 * Copyright (c) Sunplus Inc.
 * Author: Li-hao Kuo <lhjeff911@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include "thermal_hwmon.h"

#define ENABLE_THERMAL		BIT(31)

#define SP_THERMAL_MASK		GENMASK(10, 0)
#define SP_TCODE_HIGH_MASK	GENMASK(15, 8)
#define SP_TCODE_LOW_MASK	GENMASK(7, 0)

#define SP_THERMAL_CTL0_REG	0x0000
#define SP_THERMAL_STS0_REG	0x0030

/* Sensor constants from factory characterisation */
#define SP7021_TEMP_BASE	3500	/* centidegrees at reference ADC code */
#define SP7021_TEMP_RATE	608	/* ADC counts per 100 °C */
#define SP7021_OTP_BASE		1518	/* default reference ADC code if OTP absent */

struct sp_thermal_data {
	struct thermal_zone_device *tz;
	void __iomem *regs;
	int otp_temp0;
};

static void sp_thermal_init(struct sp_thermal_data *sp)
{
	writel(ENABLE_THERMAL, sp->regs + SP_THERMAL_CTL0_REG);
	msleep(1);
}

static void sp_get_otp_temp_coef(struct sp_thermal_data *sp, struct device *dev)
{
	struct nvmem_cell *cell;
	char *buf;
	ssize_t len;

	cell = nvmem_cell_get(dev, "therm_calib");
	if (IS_ERR(cell)) {
		dev_warn(dev, "failed to get therm_calib NVMEM cell, using default\n");
		sp->otp_temp0 = SP7021_OTP_BASE;
		return;
	}

	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);

	if (IS_ERR(buf) || len < 2) {
		dev_warn(dev, "failed to read therm_calib, using default\n");
		sp->otp_temp0 = SP7021_OTP_BASE;
		return;
	}

	sp->otp_temp0 = FIELD_PREP(SP_TCODE_LOW_MASK, buf[0]) |
			FIELD_PREP(SP_TCODE_HIGH_MASK, buf[1]);
	kfree(buf);

	sp->otp_temp0 = FIELD_GET(SP_THERMAL_MASK, sp->otp_temp0);

	if (!sp->otp_temp0) {
		dev_warn(dev, "therm_calib is zero, using default\n");
		sp->otp_temp0 = SP7021_OTP_BASE;
	}
}

static int sp_thermal_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct sp_thermal_data *sp = thermal_zone_device_priv(tz);
	int t_code;

	t_code = FIELD_GET(SP_THERMAL_MASK,
			   readl(sp->regs + SP_THERMAL_STS0_REG));
	*temp = ((sp->otp_temp0 - t_code) * 10000 / SP7021_TEMP_RATE
		 + SP7021_TEMP_BASE) * 10;

	return 0;
}

static const struct thermal_zone_device_ops sp_thermal_ops = {
	.get_temp = sp_thermal_get_temp,
};

static int sp_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sp_thermal_data *sp;

	sp = devm_kzalloc(dev, sizeof(*sp), GFP_KERNEL);
	if (!sp)
		return -ENOMEM;

	sp->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sp->regs))
		return dev_err_probe(dev, PTR_ERR(sp->regs), "failed to map registers\n");

	platform_set_drvdata(pdev, sp);

	sp_thermal_init(sp);
	sp_get_otp_temp_coef(sp, dev);

	sp->tz = devm_thermal_of_zone_register(dev, 0, sp, &sp_thermal_ops);
	if (IS_ERR(sp->tz))
		return dev_err_probe(dev, PTR_ERR(sp->tz), "failed to register thermal zone\n");

	if (devm_thermal_add_hwmon_sysfs(dev, sp->tz))
		dev_warn(dev, "failed to add hwmon sysfs\n");

	dev_info(dev, "thermal sensor registered, calibration: otp_temp0=%d\n", sp->otp_temp0);

	return 0;
}

static int __maybe_unused sp_thermal_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused sp_thermal_resume(struct device *dev)
{
	struct sp_thermal_data *sp = dev_get_drvdata(dev);

	sp_thermal_init(sp);
	return 0;
}

static const struct dev_pm_ops sp_thermal_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sp_thermal_suspend, sp_thermal_resume)
};

static const struct of_device_id sp_thermal_ids[] = {
	{ .compatible = "sunplus,sp7021-thermal" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sp_thermal_ids);

static struct platform_driver sp_thermal_driver = {
	.probe	= sp_thermal_probe,
	.driver	= {
		.name		= "sp7021-thermal",
		.of_match_table	= sp_thermal_ids,
		.pm		= &sp_thermal_pm_ops,
	},
};
module_platform_driver(sp_thermal_driver);

MODULE_AUTHOR("Li-hao Kuo <lhjeff911@gmail.com>");
MODULE_DESCRIPTION("Thermal sensor driver for Sunplus SP7021 SoC");
MODULE_LICENSE("GPL");

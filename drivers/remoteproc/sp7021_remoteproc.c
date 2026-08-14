// SPDX-License-Identifier: GPL-2.0
/*
 * Sunplus SP7021 ARM926 Remote Processor driver
 *
 * Copyright (C) 2021 Sunplus Technology Co., Ltd.
 *
 * Based on Zynq Remote Processor driver
 * Copyright (C) 2012 Michal Simek <monstr@monstr.eu>
 * Copyright (C) 2012 PetaLogix
 */

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/workqueue.h>

#include "remoteproc_internal.h"

#define MAX_NUM_VRINGS	2

struct ipi_info {
	u32 notifyid;
	bool pending;
};

struct sp_rproc_pdata {
	struct rproc *rproc;
	struct ipi_info ipis[MAX_NUM_VRINGS];
	u32 __iomem *mbox0to2;	/* CPU0 → A926 mailbox (kick) */
	u32 __iomem *boot;	/* A926 boot address register */
};

static bool autoboot __read_mostly;

/* Single-instance globals (SP7021 has one A926) */
static struct rproc *g_rproc;
static struct work_struct workqueue;

static void handle_event(struct work_struct *work)
{
	struct sp_rproc_pdata *local = g_rproc->priv;

	if (rproc_vq_interrupt(local->rproc, local->ipis[0].notifyid) == IRQ_NONE)
		dev_dbg(g_rproc->dev.parent, "no message found in vqid 0\n");
}

static void ipi_kick(void)
{
	dev_dbg(g_rproc->dev.parent, "kick from A926\n");
	schedule_work(&workqueue);
}

static void kick_pending_ipi(struct rproc *rproc)
{
	struct sp_rproc_pdata *local = rproc->priv;
	int i;

	for (i = 0; i < MAX_NUM_VRINGS; i++) {
		if (local->ipis[i].pending) {
			writel(local->ipis[i].notifyid, local->mbox0to2);
			local->ipis[i].pending = false;
		}
	}
}

static int sp_rproc_start(struct rproc *rproc)
{
	struct sp_rproc_pdata *local = rproc->priv;

	INIT_WORK(&workqueue, handle_event);
	kick_pending_ipi(rproc);
	writel(rproc->bootaddr, local->boot);
	dev_info(rproc->dev.parent, "A926 started, boot addr 0x%llx\n",
		 rproc->bootaddr);
	return 0;
}

static void sp_rproc_kick(struct rproc *rproc, int vqid)
{
	struct sp_rproc_pdata *local = rproc->priv;
	struct rproc_vdev *rvdev, *rvtmp;
	int i;

	list_for_each_entry_safe(rvdev, rvtmp, &rproc->rvdevs, node) {
		for (i = 0; i < MAX_NUM_VRINGS; i++) {
			struct rproc_vring *rvring = &rvdev->vring[i];

			if (rvring->notifyid == vqid) {
				local->ipis[i].notifyid = vqid;
				if (rproc->state == RPROC_RUNNING)
					writel(vqid, local->mbox0to2);
				else
					local->ipis[i].pending = true;
			}
		}
	}
}

static int sp_rproc_stop(struct rproc *rproc)
{
	struct sp_rproc_pdata *local = rproc->priv;

	writel(0xDEADC0DE, local->mbox0to2);
	return 0;
}

static int sp_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	int num_mems, i, ret;

	num_mems = of_count_phandle_with_args(np, "memory-region", NULL);
	if (num_mems <= 0)
		return 0;

	for (i = 0; i < num_mems; i++) {
		struct device_node *node;
		struct reserved_mem *rmem;
		struct rproc_mem_entry *mem;

		node = of_parse_phandle(np, "memory-region", i);
		rmem = of_reserved_mem_lookup(node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region %d\n", i);
			return -EINVAL;
		}

		if (strstr(node->name, "vdev") && strstr(node->name, "buffer")) {
			mem = rproc_mem_entry_init(dev, NULL,
						   (dma_addr_t)rmem->base,
						   rmem->size, rmem->base,
						   NULL, NULL, node->name);
			if (!mem)
				return -ENOMEM;
			rproc_add_carveout(rproc, mem);
		} else if (strstr(node->name, "vdev") && strstr(node->name, "vring")) {
			mem = rproc_mem_entry_init(dev, NULL,
						   (dma_addr_t)rmem->base,
						   rmem->size, rmem->base,
						   NULL, NULL, node->name);
			if (!mem)
				return -ENOMEM;
			mem->va = devm_ioremap_wc(dev, rmem->base, rmem->size);
			if (!mem->va)
				return -ENOMEM;
			rproc_add_carveout(rproc, mem);
		} else {
			mem = rproc_of_resm_mem_entry_init(dev, i, rmem->size,
							   rmem->base, node->name);
			if (!mem)
				return -ENOMEM;
			mem->va = devm_ioremap_wc(dev, rmem->base, rmem->size);
			if (!mem->va)
				return -ENOMEM;
			rproc_add_carveout(rproc, mem);
		}
	}

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret == -EINVAL)
		ret = 0;
	return ret;
}

static const struct rproc_ops sp_rproc_ops = {
	.start			= sp_rproc_start,
	.stop			= sp_rproc_stop,
	.kick			= sp_rproc_kick,
	.parse_fw		= sp_parse_fw,
	.find_loaded_rsc_table	= rproc_elf_find_loaded_rsc_table,
	.get_boot_addr		= rproc_elf_get_boot_addr,
	.load			= rproc_elf_load_segments,
};

static irqreturn_t sp_remoteproc_interrupt(int irq, void *dev_id)
{
	ipi_kick();
	return IRQ_HANDLED;
}

static int sp_remoteproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sp_rproc_pdata *local;
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, sp_remoteproc_interrupt,
			       IRQF_TRIGGER_NONE, dev_name(dev), NULL);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	{
		const char *fw_name;

		if (of_property_read_string(dev->of_node, "firmware", &fw_name))
			fw_name = NULL;
		g_rproc = rproc_alloc(dev, dev_name(dev), &sp_rproc_ops,
				      fw_name, sizeof(*local));
	}
	if (!g_rproc)
		return -ENOMEM;

	local = g_rproc->priv;
	local->rproc = g_rproc;
	platform_set_drvdata(pdev, g_rproc);

	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "dma_set_coherent_mask failed: %d\n", ret);
		goto free_rproc;
	}

	local->mbox0to2 = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(local->mbox0to2)) {
		ret = PTR_ERR(local->mbox0to2);
		goto free_rproc;
	}

	local->boot = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(local->boot)) {
		ret = PTR_ERR(local->boot);
		goto free_rproc;
	}

	g_rproc->auto_boot = autoboot;

	ret = rproc_add(g_rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed: %d\n", ret);
		goto free_rproc;
	}

	dev_info(dev, "SP7021 A926 remoteproc registered\n");
	return 0;

free_rproc:
	rproc_free(g_rproc);
	return ret;
}

static void sp_remoteproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	of_reserved_mem_device_release(&pdev->dev);
	rproc_free(rproc);
}

static const struct of_device_id sp_remoteproc_match[] = {
	{ .compatible = "sunplus,sp-rproc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sp_remoteproc_match);

static struct platform_driver sp_remoteproc_driver = {
	.probe	= sp_remoteproc_probe,
	.remove = sp_remoteproc_remove,
	.driver	= {
		.name		= "sp7021-remoteproc",
		.of_match_table	= sp_remoteproc_match,
	},
};
module_platform_driver(sp_remoteproc_driver);

module_param_named(autoboot, autoboot, bool, 0444);
MODULE_PARM_DESC(autoboot, "Automatically boot A926 firmware (default: false)");

MODULE_AUTHOR("Sunplus Technology Co., Ltd.");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Sunplus SP7021 ARM926 remote processor driver");

// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Sunplus Technology Co., Ltd.
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/smp.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>

#define OF_CPU_BOOT	"cpu-boot-reg"

static DEFINE_SPINLOCK(boot_lock);

/*
 * pen_release holds the hardware core ID of the secondary CPU being
 * brought up.  The secondary clears it to -1 once it is running.
 */
int pentagram_pen_release = -1;

static void write_pen_release(int val)
{
	pentagram_pen_release = val;
	smp_wmb();
	sync_cache_w(&pentagram_pen_release);
}

#include "platsmp.h"

#ifdef CONFIG_HOTPLUG_CPU

static bool sc_smp_cpu_can_disable(unsigned int cpu)
{
	return true;
}
#endif

static void sc_smp_secondary_init(unsigned int cpu)
{
	write_pen_release(-1);
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

static u32 secondary_boot_addr_for(unsigned int cpu)
{
	u32 secondary_boot_addr = 0;
	struct device_node *cpu_node = of_get_cpu_node(cpu, NULL);

	if (!cpu_node) {
		pr_err("Failed to find device tree node for CPU%u\n", cpu);
		return 0;
	}

	if (of_property_read_u32(cpu_node, OF_CPU_BOOT, &secondary_boot_addr))
		pr_err("missed cpu-boot-reg in DT for CPU%u\n", cpu);

	of_node_put(cpu_node);
	return secondary_boot_addr;
}

static void __init sc_smp_prepare_cpus(unsigned int max_cpus)
{
	/* Cortex-A7 has no SCU */
}

static int sc_smp_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	const u32 wait_addr = secondary_boot_addr_for(cpu);
	u32 mpidr = cpu_logical_map(cpu);
	u32 core_id = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	unsigned long timeout;
	void __iomem *va;

	if (!wait_addr)
		return -EINVAL;

	spin_lock(&boot_lock);

	write_pen_release(core_id);

	va = ioremap((phys_addr_t)wait_addr, sizeof(phys_addr_t));
	if (!va) {
		pr_warn("unable to ioremap CPU%u boot address\n", cpu);
		spin_unlock(&boot_lock);
		return -ENOMEM;
	}

	/* Write the physical address of the kernel secondary entry point */
	writel(__pa_symbol(secondary_startup), va);
	smp_wmb();
	iounmap(va);

	/* Wake any CPUs sleeping in WFI */
	sev();

	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		arch_send_wakeup_ipi_mask(cpumask_of(cpu));
		if (pentagram_pen_release == -1)
			break;
		udelay(10);
	}

	spin_unlock(&boot_lock);

	return pentagram_pen_release != -1 ? -ETIMEDOUT : 0;
}

static const struct smp_operations sc_smp_ops __initconst = {
	.smp_prepare_cpus	= sc_smp_prepare_cpus,
	.smp_secondary_init	= sc_smp_secondary_init,
	.smp_boot_secondary	= sc_smp_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_can_disable	= sc_smp_cpu_can_disable,
	.cpu_die		= sc_smp_cpu_die,
#endif
};
CPU_METHOD_OF_DECLARE(sc_smp, "sunplus,sc-smp", &sc_smp_ops);

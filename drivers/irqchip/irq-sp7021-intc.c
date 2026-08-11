// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
/*
 * Copyright (C) Sunplus Technology Co., Ltd.
 *       All rights reserved.
 */
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define SP_INTC_HWIRQ_MIN	0
#define SP_INTC_HWIRQ_MAX	223

#define SP_INTC_NR_IRQS		(SP_INTC_HWIRQ_MAX - SP_INTC_HWIRQ_MIN + 1)
#define SP_INTC_NR_GROUPS	DIV_ROUND_UP(SP_INTC_NR_IRQS, 32)
#define SP_INTC_REG_SIZE	(SP_INTC_NR_GROUPS * 4)

/* REG_GROUP_0 regs */
#define REG_INTR_TYPE		(sp_intc.g0)
#define REG_INTR_POLARITY	(REG_INTR_TYPE     + SP_INTC_REG_SIZE)
#define REG_INTR_PRIORITY	(REG_INTR_POLARITY + SP_INTC_REG_SIZE)
#define REG_INTR_MASK		(REG_INTR_PRIORITY + SP_INTC_REG_SIZE)

/* REG_GROUP_1 regs */
#define REG_INTR_CLEAR		(sp_intc.g1)
#define REG_MASKED_EXT1		(REG_INTR_CLEAR    + SP_INTC_REG_SIZE)
#define REG_MASKED_EXT0		(REG_MASKED_EXT1   + SP_INTC_REG_SIZE)
#define REG_INTR_GROUP		(REG_INTR_CLEAR    + 31 * 4)

#define GROUP_MASK		(BIT(SP_INTC_NR_GROUPS) - 1)
#define GROUP_SHIFT_EXT1	(0)
#define GROUP_SHIFT_EXT0	(8)

/*
 * When GPIO_INT0~7 set to edge trigger, doesn't work properly.
 * WORKAROUND: change it to level trigger, and toggle the polarity
 * at ACK/Handler to make the HW work.
 */
#define GPIO_INT0_HWIRQ		120
#define GPIO_INT7_HWIRQ		127
#define IS_GPIO_INT(irq)					\
({								\
	u32 i = irq;						\
	(i >= GPIO_INT0_HWIRQ) && (i <= GPIO_INT7_HWIRQ);	\
})

/* index of states */
enum {
	_IS_EDGE = 0,	/* edge-triggered (vs level) */
	_IS_LOW,	/* current armed polarity: 1=low-active, 0=high-active */
	_IS_ACTIVE,	/* return-edge is in flight (single-edge mode only) */
	_IS_BOTH,	/* both-edges mode: report every transition */
};

#define STATE_BITS_PER_IRQ	4
#define STATE_BIT(irq, idx)	(((irq) - GPIO_INT0_HWIRQ) * STATE_BITS_PER_IRQ + (idx))
#define ASSIGN_STATE(irq, idx, v)	assign_bit(STATE_BIT(irq, idx), sp_intc.states, v)
#define TEST_STATE(irq, idx)		test_bit(STATE_BIT(irq, idx), sp_intc.states)

static struct sp_intctl {
	/*
	 * REG_GROUP_0: include type/polarity/priority/mask regs.
	 * REG_GROUP_1: include clear/masked_ext0/masked_ext1/group regs.
	 */
	void __iomem *g0; // REG_GROUP_0 base
	void __iomem *g1; // REG_GROUP_1 base

	struct irq_domain *domain;
	raw_spinlock_t lock;

	/*
	 * store GPIO_INT states
	 * each interrupt has 4 states: is_edge, is_low, is_active, is_both
	 */
	DECLARE_BITMAP(states, (GPIO_INT7_HWIRQ - GPIO_INT0_HWIRQ + 1) * STATE_BITS_PER_IRQ);
} sp_intc;

static struct irq_chip sp_intc_chip;

static void sp_intc_assign_bit(u32 hwirq, void __iomem *base, bool value)
{
	u32 offset, mask;
	unsigned long flags;
	void __iomem *reg;

	offset = (hwirq / 32) * 4;
	reg = base + offset;

	raw_spin_lock_irqsave(&sp_intc.lock, flags);
	mask = readl_relaxed(reg);
	if (value)
		mask |= BIT(hwirq % 32);
	else
		mask &= ~BIT(hwirq % 32);
	writel_relaxed(mask, reg);
	raw_spin_unlock_irqrestore(&sp_intc.lock, flags);
}

static void sp_intc_ack_irq(struct irq_data *d)
{
	u32 hwirq = d->hwirq;

	if (unlikely(IS_GPIO_INT(hwirq) && TEST_STATE(hwirq, _IS_EDGE))) { // WORKAROUND
		bool new_pol = !TEST_STATE(hwirq, _IS_LOW);

		sp_intc_assign_bit(hwirq, REG_INTR_POLARITY, new_pol);

		if (TEST_STATE(hwirq, _IS_BOTH)) {
			/*
			 * Both-edges mode: update _IS_LOW to track the new
			 * hardware polarity so the next ack flips it again.
			 * Never set _IS_ACTIVE — every level detection is a
			 * real edge and must reach the handler.
			 */
			ASSIGN_STATE(hwirq, _IS_LOW, new_pol);
		} else {
			/*
			 * Single-edge mode: _IS_LOW stays constant (it records
			 * the original requested polarity).  _IS_ACTIVE signals
			 * the handler to swallow the return-edge interrupt.
			 */
			ASSIGN_STATE(hwirq, _IS_ACTIVE, true);
		}
	}

	sp_intc_assign_bit(hwirq, REG_INTR_CLEAR, 1);
}

static void sp_intc_mask_irq(struct irq_data *d)
{
	sp_intc_assign_bit(d->hwirq, REG_INTR_MASK, 0);
}

static void sp_intc_unmask_irq(struct irq_data *d)
{
	sp_intc_assign_bit(d->hwirq, REG_INTR_MASK, 1);
}

static int sp_intc_set_type(struct irq_data *d, unsigned int type)
{
	u32 hwirq = d->hwirq;
	bool is_edge = !(type & IRQ_TYPE_LEVEL_MASK);
	bool is_low = (type == IRQ_TYPE_LEVEL_LOW || type == IRQ_TYPE_EDGE_FALLING);
	bool is_both = (type == IRQ_TYPE_EDGE_BOTH);

	irq_set_handler_locked(d, is_edge ? handle_edge_irq : handle_level_irq);

	if (unlikely(IS_GPIO_INT(hwirq) && is_edge)) { // WORKAROUND
		/*
		 * The GPIO_INT hardware re-asserts the interrupt at level after
		 * each edge, causing a spurious second fire that disables the IRQ.
		 * Emulate edge detection using level mode + polarity toggling on
		 * each ack.  For both-edges, _IS_LOW alternates so every
		 * transition is reported; for single-edge, _IS_ACTIVE suppresses
		 * the return-edge.
		 *
		 * For IRQ_TYPE_EDGE_BOTH arm high-active first (is_low=false).
		 */
		ASSIGN_STATE(hwirq, _IS_EDGE, true);
		ASSIGN_STATE(hwirq, _IS_LOW, is_both ? false : is_low);
		ASSIGN_STATE(hwirq, _IS_ACTIVE, false);
		ASSIGN_STATE(hwirq, _IS_BOTH, is_both);
		/* change to level */
		is_edge = false;
		is_low = is_both ? false : is_low;
	}

	sp_intc_assign_bit(hwirq, REG_INTR_TYPE, is_edge);
	sp_intc_assign_bit(hwirq, REG_INTR_POLARITY, is_low);

	return 0;
}

static int sp_intc_get_ext_irq(int ext_num)
{
	void __iomem *base = ext_num ? REG_MASKED_EXT1 : REG_MASKED_EXT0;
	u32 shift = ext_num ? GROUP_SHIFT_EXT1 : GROUP_SHIFT_EXT0;
	u32 groups;
	u32 pending_group;
	u32 group;
	u32 pending_irq;

	groups = readl_relaxed(REG_INTR_GROUP);
	pending_group = (groups >> shift) & GROUP_MASK;
	if (!pending_group)
		return -1;

	group = fls(pending_group) - 1;
	pending_irq = readl_relaxed(base + group * 4);
	if (!pending_irq)
		return -1;

	return (group * 32) + fls(pending_irq) - 1;
}

static void sp_intc_handle_ext_cascaded(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int ext_num = (uintptr_t)irq_desc_get_handler_data(desc);
	int hwirq;

	chained_irq_enter(chip, desc);

	while ((hwirq = sp_intc_get_ext_irq(ext_num)) >= 0) {
		if (unlikely(IS_GPIO_INT(hwirq) && TEST_STATE(hwirq, _IS_ACTIVE))) { // WORKAROUND
			ASSIGN_STATE(hwirq, _IS_ACTIVE, false);
			sp_intc_assign_bit(hwirq, REG_INTR_POLARITY, TEST_STATE(hwirq, _IS_LOW));
		} else {
			generic_handle_domain_irq(sp_intc.domain, hwirq);
		}
	}

	chained_irq_exit(chip, desc);
}

static struct irq_chip sp_intc_chip = {
	.name		= "sp_intc",
	.irq_ack	= sp_intc_ack_irq,
	.irq_mask	= sp_intc_mask_irq,
	.irq_unmask	= sp_intc_unmask_irq,
	.irq_set_type	= sp_intc_set_type,
};

static int sp_intc_irq_domain_map(struct irq_domain *domain,
				  unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &sp_intc_chip, handle_level_irq);
	irq_set_chip_data(irq, &sp_intc_chip);
	irq_set_noprobe(irq);

	return 0;
}

static const struct irq_domain_ops sp_intc_dm_ops = {
	.xlate = irq_domain_xlate_twocell,
	.map = sp_intc_irq_domain_map,
};

static int sp_intc_irq_map(struct device_node *node, int i)
{
	unsigned int irq;

	irq = irq_of_parse_and_map(node, i);
	if (!irq)
		return -ENOENT;

	irq_set_chained_handler_and_data(irq, sp_intc_handle_ext_cascaded, (void *)(uintptr_t)i);

	return 0;
}

static int __init sp_intc_init_dt(struct device_node *node, struct device_node *parent)
{
	int i, ret;

	sp_intc.g0 = of_iomap(node, 0);
	if (!sp_intc.g0)
		return -ENXIO;

	sp_intc.g1 = of_iomap(node, 1);
	if (!sp_intc.g1) {
		ret = -ENXIO;
		goto out_unmap0;
	}

	ret = sp_intc_irq_map(node, 0); // EXT_INT0
	if (ret)
		goto out_unmap1;

	ret = sp_intc_irq_map(node, 1); // EXT_INT1
	if (ret)
		goto out_unmap1;

	/* initial regs */
	for (i = 0; i < SP_INTC_NR_GROUPS; i++) {
		/* all mask */
		writel_relaxed(0, REG_INTR_MASK + i * 4);
		/* all edge */
		writel_relaxed(~0, REG_INTR_TYPE + i * 4);
		/* all high-active */
		writel_relaxed(0, REG_INTR_POLARITY + i * 4);
		/* all EXT_INT0 */
		writel_relaxed(~0, REG_INTR_PRIORITY + i * 4);
		/* all clear */
		writel_relaxed(~0, REG_INTR_CLEAR + i * 4);
	}

	sp_intc.domain = irq_domain_create_linear(of_fwnode_handle(node), SP_INTC_NR_IRQS,
						  &sp_intc_dm_ops, &sp_intc);
	if (!sp_intc.domain) {
		ret = -ENOMEM;
		goto out_unmap1;
	}

	raw_spin_lock_init(&sp_intc.lock);

	return 0;

out_unmap1:
	iounmap(sp_intc.g1);
out_unmap0:
	iounmap(sp_intc.g0);

	return ret;
}

IRQCHIP_DECLARE(sp_intc, "sunplus,sp7021-intc", sp_intc_init_dt);

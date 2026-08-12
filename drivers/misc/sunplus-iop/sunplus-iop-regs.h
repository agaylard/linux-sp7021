/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __SUNPLUS_IOP_REGS_H__
#define __SUNPLUS_IOP_REGS_H__

struct regs_iop_t {
	u32 iop_control;	/* 00 */
	u32 iop_reg1;		/* 01 */
	u32 iop_bp;		/* 02 */
	u32 iop_regsel;		/* 03 */
	u32 iop_regout;		/* 04 */
	u32 iop_reg5;		/* 05 */
	u32 iop_resume_pcl;	/* 06 */
	u32 iop_resume_pch;	/* 07 */
	u32 iop_data0;		/* 08 */
	u32 iop_data1;		/* 09 */
	u32 iop_data2;		/* 10 */
	u32 iop_data3;		/* 11 */
	u32 iop_data4;		/* 12 */
	u32 iop_data5;		/* 13 */
	u32 iop_data6;		/* 14 */
	u32 iop_data7;		/* 15 */
	u32 iop_data8;		/* 16 */
	u32 iop_data9;		/* 17 */
	u32 iop_data10;		/* 18 */
	u32 iop_data11;		/* 19 */
	u32 iop_base_adr_l;	/* 20 */
	u32 iop_base_adr_h;	/* 21 */
	u32 memory_bridge_control; /* 22 */
	u32 iop_regmap_adr_l;	/* 23 */
	u32 iop_regmap_adr_h;	/* 24 */
	u32 iop_direct_adr;	/* 25 */
	u32 reserved[6];	/* 26~31 */
};

struct regs_iop_pmc_t {
	u32 PMC_TIMER;		/* 00 */
	u32 PMC_CTRL;		/* 01 */
	u32 XTAL27M_PASSWORD_I;	/* 02 */
	u32 XTAL27M_PASSWORD_II;/* 03 */
	u32 XTAL32K_PASSWORD_I;	/* 04 */
	u32 XTAL32K_PASSWORD_II;/* 05 */
	u32 CLK27M_PASSWORD_I;	/* 06 */
	u32 CLK27M_PASSWORD_II;	/* 07 */
	u32 PMC_TIMER2;		/* 08 */
	u32 reserved[23];	/* 09~31 */
};

#endif /* __SUNPLUS_IOP_REGS_H__ */

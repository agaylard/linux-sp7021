/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright (C) Sunplus Technology Co., Ltd.
 *       All rights reserved.
 */
#ifndef _DT_BINDINGS_CLOCK_SUNPLUS_SP7021_H
#define _DT_BINDINGS_CLOCK_SUNPLUS_SP7021_H

/* gates — DT-visible clocks; indices must match sp_clk_gates[] in clk-sp7021.c */
#define CLK_RTC         0
#define CLK_OTPRX       1
/* index 2 was CLK_NOC — now infrastructure-only, not DT-visible */
#define CLK_BR          2
#define CLK_SPIFL       3
#define CLK_PERI0       4
#define CLK_PERI1       5
#define CLK_STC0        6
#define CLK_STC_AV0     7
#define CLK_STC_AV1     8
#define CLK_STC_AV2     9
#define CLK_UA0         10
#define CLK_UA1         11
#define CLK_UA2         12
#define CLK_UA3         13
#define CLK_UA4         14
#define CLK_HWUA        15
#define CLK_DDC0        16
#define CLK_UADMA       17
#define CLK_CBDMA0      18
#define CLK_CBDMA1      19
#define CLK_SPI_COMBO_0 20
#define CLK_SPI_COMBO_1 21
#define CLK_SPI_COMBO_2 22
#define CLK_SPI_COMBO_3 23
#define CLK_AUD         24
#define CLK_USBC0       25
#define CLK_USBC1       26
#define CLK_UPHY0       27
#define CLK_UPHY1       28
#define CLK_I2CM0       29
#define CLK_I2CM1       30
#define CLK_I2CM2       31
#define CLK_I2CM3       32
#define CLK_PMC         33
#define CLK_CARD_CTL0   34
#define CLK_CARD_CTL1   35
#define CLK_CARD_CTL4   36
#define CLK_BCH         37
#define CLK_DDFCH       38
#define CLK_CSIIW0      39
#define CLK_CSIIW1      40
#define CLK_MIPICSI0    41
#define CLK_MIPICSI1    42
#define CLK_HDMI_TX     43
#define CLK_VPOST       44
#define CLK_TGEN        45
#define CLK_DMIX        46
#define CLK_TCON        47
#define CLK_GPIO        48
#define CLK_MAILBOX     49
#define CLK_SPIND       50
#define CLK_I2C2CBUS    51
#define CLK_SEC         52
#define CLK_DVE         53
#define CLK_GPOST0      54
#define CLK_OSD0        55
#define CLK_DISP_PWM    56
#define CLK_UADBG       57
#define CLK_FIO_CTL     58
#define CLK_FPGA        59
#define CLK_L2SW        60
#define CLK_ICM         61
/* index 62 was CLK_AXI_GLOBAL — now infrastructure-only, not DT-visible */

/* plls */
#define PLL_A           62
#define PLL_E           63
#define PLL_E_2P5       64
#define PLL_E_25        65
#define PLL_E_112P5     66
#define PLL_F           67
#define PLL_TV          68
#define PLL_TV_A        69
#define PLL_SYS         70

#define CLK_MAX         71

#endif

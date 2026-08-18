// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Sunplus Inc.
 * Author: Li-hao Kuo <lhjeff911@gmail.com>
 *
 * Ported from Linux 5.10 sunplus_sd2.c to 7.1.5 by Andrew Gilmore.
 * Handles SP7021 CARD1 (SD card slot, Group 125/126 registers at 0x9C003E80).
 * This is a separate driver from sunplus-mmc.c (CARD0/eMMC).
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/semaphore.h>
#include <linux/reset.h>

#define SPSDC_MIN_CLK			400000
#define SPSDC_MAX_CLK			52000000
#define SPSDC_50M_CLK			50000000
#define SPSDC_MAX_BLK_COUNT		65536

/* Group 125 registers (base + offset) */
#define SPSD2_MEDIA_TYPE_REG		0x0000
#define SPSDC_MEDIA_MASK		GENMASK(2, 0)
#define SPSDC_MEDIA_NONE		0
#define SPSDC_MEDIA_SD			6

#define SPSD2_SDRAM_SECTOR_SIZE_REG	0x0010
#define SPSDC_MAX_DMA_MEMORY_SECTORS	8

#define SPSD2_SDRAM_SECTOR_ADDR_REG	0x001C

/* Group 126 registers */
#define SPSD2_SD_INT_REG		0x00B0
#define SPSDC_SDINT_SDCMPEN		BIT(0)
#define SPSDC_SDINT_SDCMP		BIT(1)
#define SPSDC_SDINT_SDCMP_CLR		BIT(2)
#define SPSDC_SDINT_SDIOEN		BIT(4)
#define SPSDC_SDINT_SDIO		BIT(5)
#define SPSDC_SDINT_SDIO_CLR		BIT(6)

#define SPSD2_SD_PAGE_NUM_REG		0x00B4
#define SPSD2_SD_CONF0_REG		0x00B8
#define SPSDC_CONF0_SDPIO_MODE		BIT(0)
#define SPSDC_CONF0_SDLEN_MODE		BIT(2)
#define SPSDC_CONF0_TRANS_MODE		GENMASK(5, 4)
#define SPSDC_TRANS_CMD_MODE		0
#define SPSDC_TRANS_WR_MODE		1
#define SPSDC_TRANS_RD_MODE		2
#define SPSDC_CONF0_AUTORSP		BIT(6)
#define SPSDC_CONF0_CMDDUMMY		BIT(7)
#define SPSDC_CONF0_RSPCHK		BIT(8)

#define SPSD2_SDIO_CTRL_REG		0x00BC
#define SPSDC_SDIO_CTRL_MULTI_TRIG	BIT(6)

#define SPSD2_SD_RST_REG		0x00C0
#define SPSDC_RST_ALL			0x07

#define SPSD2_SD_CONF_REG		0x00C4
#define SPSDC_CONF_CLK_DIV		GENMASK(11, 0)
#define SPSDC_CONF_4BIT_MODE		BIT(12)
#define SPSDC_CONF_SDRSP_TYPE		BIT(13)
/*
 * SPSDC_CONF_SD_MODE (bit 16): must be SET for SD card operation.
 * When clear, the controller samples data on the falling clock edge,
 * violating the SD specification. Setting this bit switches to rising-edge
 * sampling. Despite the register guide naming it "MMC card mode", the
 * vendor driver always sets it for SD card use.
 */
#define SPSDC_CONF_SD_MODE		BIT(16)
#define SPSDC_CONF_MMC8BIT		BIT(18)
#define SPSDC_CONF_SDIO_MODE		BIT(20)

#define SPSD2_SD_CTRL_REG		0x00C8
#define SPSDC_CTRL_CMD_TRIG		BIT(0)
#define SPSDC_CTRL_TXDUMMY_TRIG		BIT(1)

#define SPSD2_SD_STATUS_REG		0x00CC
#define SPSDC_STS_DUMMY_RDY		BIT(0)
#define SPSDC_STS_RSP_BUF_FULL		BIT(1)
#define SPSDC_STS_TX_BUF_EMP		BIT(2)
#define SPSDC_STS_RX_BUF_FULL		BIT(3)
#define SPSDC_STS_CMD_PIN_STS		BIT(4)
#define SPSDC_STS_DAT0_PIN_STS		BIT(5)
#define SPSDC_STS_RSP_TIMEOUT		BIT(6)
#define SPSDC_STS_CARD_CRC_TIMEOUT	BIT(7)
#define SPSDC_STS_STB_TIMEOUT		BIT(8)
#define SPSDC_STS_RSP_CRC7_ERR		BIT(9)
#define SPSDC_STS_CRC_TOKEN_ERR		BIT(10)
#define SPSDC_STS_RDATA_CRC16_ERR	BIT(11)
#define SPSDC_STS_SUSPEND_STATE_RDY	BIT(12)
#define SPSDC_STS_BUSY_CYCLE		BIT(13)

#define SPSD2_SD_STATE_REG		0x00D0
#define SPSDC_STATE_ERROR		BIT(13)
#define SPSDC_STATE_FINISH		BIT(14)

#define SPSD2_BLOCKSIZE_REG		0x00D4
#define SPSD2_SD_TIMING_CONF0_REG	0x00DC
#define SPSDC_TIMING_CONF0_HS_EN	BIT(11)
#define SPSDC_TIMING_CONF0_WRTD		GENMASK(14, 12)

#define SPSD2_SD_TIMING_CONF1_REG	0x00E0
#define SPSDC_TIMING_CONF1_RDTD		GENMASK(15, 13)

#define SPSD2_SD_PIO_TX_REG		0x00E4
#define SPSD2_SD_PIO_RX_REG		0x00E8
#define SPSD2_SD_CMD_BUF0_REG		0x00EC
#define SPSD2_SD_CMD_BUF1_REG		0x00F0
#define SPSD2_SD_CMD_BUF2_REG		0x00F4
#define SPSD2_SD_CMD_BUF3_REG		0x00F8
#define SPSD2_SD_CMD_BUF4_REG		0x00FC
#define SPSD2_SD_RSP_BUF0_3_REG	0x0100
#define SPSD2_SD_RSP_BUF4_5_REG	0x0104

/* DMA engine registers (separate block) */
#define SPSD2_DMA_SRCDST_REG		0x0204
#define SPSD2_DMA_SIZE_REG		0x0208
#define SPSD2_DMA_STOP_RST_REG		0x020C
#define SPSD2_DMA_CTRL_REG		0x0210
#define SPSD2_DMA_BASE_ADDR0_REG	0x0214
#define SPSD2_DMA_BASE_ADDR16_REG	0x0218

struct spsdc_tuning_info {
	int need_tuning;
#define SPSDC_MAX_RETRIES (8 * 8)
	int retried;
	u32 wr_dly:3;
	u32 rd_dly:3;
	u32 clk_dly:3;
};

enum {
	SPSDC_DMA_MODE = 0,
	SPSDC_PIO_MODE = 1,
};

enum spsdc_mode {
	SPSDC_MODE_SD = 0,
	SPSDC_MODE_SDIO = 1,
};

struct spsdc_host {
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rstc;
	spinlock_t lock;
	struct mmc_host *mmc;
	struct semaphore mrq_lock;	/* binary; allows cross-context unlock (irq→process) */
	struct mmc_request *mrq;
	struct sg_mapping_iter sg_miter;
	struct spsdc_tuning_info tuning_info;
	struct tasklet_struct tsklet_finish_req;
	int irq;
	int mode;
	int use_int;
	int power_state;
	int dmapio_mode;
	int dma_use_int;
	int dma_int_threshold;
	int restore_4bit_sdio_bus;
};

static inline int spsdc_wait_finish(struct spsdc_host *host)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(5000);

	while (!time_after(jiffies, timeout)) {
		if (readl(host->base + SPSD2_SD_STATE_REG) & SPSDC_STATE_FINISH)
			return 0;
		if (readl(host->base + SPSD2_SD_STATE_REG) & SPSDC_STATE_ERROR)
			return -EIO;
	}
	return -ETIMEDOUT;
}

static inline int spsdc_wait_sdstatus(struct spsdc_host *host, unsigned int status_bit)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(5000);

	while (!time_after(jiffies, timeout)) {
		if (readl(host->base + SPSD2_SD_STATUS_REG) & status_bit)
			return 0;
		if (readl(host->base + SPSD2_SD_STATE_REG) & SPSDC_STATE_ERROR)
			return -EIO;
	}
	return -ETIMEDOUT;
}

#define spsdc_wait_rspbuf_full(host) spsdc_wait_sdstatus(host, SPSDC_STS_RSP_BUF_FULL)
#define spsdc_wait_rxbuf_full(host)  spsdc_wait_sdstatus(host, SPSDC_STS_RX_BUF_FULL)
#define spsdc_wait_txbuf_empty(host) spsdc_wait_sdstatus(host, SPSDC_STS_TX_BUF_EMP)

static void spsdc_get_rsp(struct spsdc_host *host, struct mmc_command *cmd)
{
	u32 value0_3, value4_5;

	if (unlikely(!(cmd->flags & MMC_RSP_PRESENT)))
		return;
	if (unlikely(cmd->flags & MMC_RSP_136)) {
		if (spsdc_wait_rspbuf_full(host))
			return;
		value0_3 = readl(host->base + SPSD2_SD_RSP_BUF0_3_REG);
		value4_5 = readl(host->base + SPSD2_SD_RSP_BUF4_5_REG) & 0xffff;
		cmd->resp[0] = (value0_3 << 8) | (value4_5 >> 8);
		cmd->resp[1] = value4_5 << 24;
		if (spsdc_wait_rspbuf_full(host))
			return;
		value0_3 = readl(host->base + SPSD2_SD_RSP_BUF0_3_REG);
		value4_5 = readl(host->base + SPSD2_SD_RSP_BUF4_5_REG) & 0xffff;
		cmd->resp[1] |= value0_3 >> 8;
		cmd->resp[2] = value0_3 << 24;
		cmd->resp[2] |= value4_5 << 8;
		if (spsdc_wait_rspbuf_full(host))
			return;
		value0_3 = readl(host->base + SPSD2_SD_RSP_BUF0_3_REG);
		value4_5 = readl(host->base + SPSD2_SD_RSP_BUF4_5_REG) & 0xffff;
		cmd->resp[2] |= value0_3 >> 24;
		cmd->resp[3] = value0_3 << 8;
		cmd->resp[3] |= value4_5 >> 8;
	} else {
		if (spsdc_wait_rspbuf_full(host))
			return;
		value0_3 = readl(host->base + SPSD2_SD_RSP_BUF0_3_REG);
		value4_5 = readl(host->base + SPSD2_SD_RSP_BUF4_5_REG) & 0xffff;
		cmd->resp[0] = (value0_3 << 8) | (value4_5 >> 8);
		cmd->resp[1] = value4_5 << 24;
	}
}

static void spsdc_set_bus_clk(struct spsdc_host *host, int clk)
{
	unsigned int clkdiv;
	int f_min = host->mmc->f_min;
	int f_max = host->mmc->f_max;
	u32 value = readl(host->base + SPSD2_SD_CONF_REG);

	if (clk < f_min)
		clk = f_min;
	if (clk > f_max)
		clk = f_max;
	if (clk >= SPSDC_50M_CLK)
		clk = f_max;

	clkdiv = (clk_get_rate(host->clk) + clk) / clk - 1;
	if (clkdiv > 0xfff)
		clkdiv = 0xfff;

	value &= ~SPSDC_CONF_CLK_DIV;
	value |= FIELD_PREP(SPSDC_CONF_CLK_DIV, clkdiv);
	writel(value, host->base + SPSD2_SD_CONF_REG);

	if (clk > 25000000)
		host->use_int = 0;
	else
		host->use_int = 1;
}

static void spsdc_set_bus_timing(struct spsdc_host *host, unsigned int timing)
{
	u32 value = readl(host->base + SPSD2_SD_TIMING_CONF0_REG);
	int clkdiv = FIELD_GET(SPSDC_CONF_CLK_DIV, readl(host->base + SPSD2_SD_CONF_REG));
	int delay = (clkdiv / 2 < 7) ? clkdiv / 2 : 7;

	switch (timing) {
	case MMC_TIMING_LEGACY:
		value &= ~SPSDC_TIMING_CONF0_HS_EN;
		break;
	case MMC_TIMING_SD_HS:
	case MMC_TIMING_MMC_HS:
		value |= SPSDC_TIMING_CONF0_HS_EN |
			FIELD_PREP(SPSDC_TIMING_CONF0_WRTD, delay);
		break;
	}
	writel(value, host->base + SPSD2_SD_TIMING_CONF0_REG);
}

static void spsdc_set_bus_width(struct spsdc_host *host, int width)
{
	u32 value = readl(host->base + SPSD2_SD_CONF_REG);

	switch (width) {
	case MMC_BUS_WIDTH_8:
		value &= ~SPSDC_CONF_4BIT_MODE;
		value |= SPSDC_CONF_MMC8BIT;
		break;
	case MMC_BUS_WIDTH_4:
		value |= SPSDC_CONF_4BIT_MODE;
		value &= ~SPSDC_CONF_MMC8BIT;
		break;
	default:
		value &= ~SPSDC_CONF_4BIT_MODE;
		value &= ~SPSDC_CONF_MMC8BIT;
		break;
	}
	writel(value, host->base + SPSD2_SD_CONF_REG);
}

static void spsdc_select_mode(struct spsdc_host *host)
{
	u32 value = readl(host->base + SPSD2_SD_CONF_REG);

	value |= SPSDC_CONF_SD_MODE;
	switch (host->mode) {
	case SPSDC_MODE_SDIO:
		value |= SPSDC_CONF_SDIO_MODE;
		writel(value, host->base + SPSD2_SD_CONF_REG);
		value = readl(host->base + SPSD2_SDIO_CTRL_REG);
		value |= SPSDC_SDIO_CTRL_MULTI_TRIG;
		writel(value, host->base + SPSD2_SDIO_CTRL_REG);
		break;
	case SPSDC_MODE_SD:
	default:
		value &= ~SPSDC_CONF_SDIO_MODE;
		host->mode = SPSDC_MODE_SD;
		writel(value, host->base + SPSD2_SD_CONF_REG);
		break;
	}
}

static void spsdc_sw_reset(struct spsdc_host *host)
{
	writel(SPSDC_RST_ALL, host->base + SPSD2_SD_RST_REG);
	writel(0x6, host->base + SPSD2_DMA_STOP_RST_REG);
	while (readl(host->base + SPSD2_DMA_STOP_RST_REG) & BIT(2))
		;
	writel(0x0, host->base + SPSD2_DMA_CTRL_REG);
	writel(0x1, host->base + SPSD2_DMA_CTRL_REG);
	writel(0x0, host->base + SPSD2_DMA_CTRL_REG);
}

static void spsdc_prepare_cmd(struct spsdc_host *host, struct mmc_command *cmd)
{
	u32 value;

	writeb((cmd->opcode | 0x40), host->base + SPSD2_SD_CMD_BUF0_REG);
	writeb(((cmd->arg >> 24) & 0xff), host->base + SPSD2_SD_CMD_BUF1_REG);
	writeb(((cmd->arg >> 16) & 0xff), host->base + SPSD2_SD_CMD_BUF2_REG);
	writeb(((cmd->arg >>  8) & 0xff), host->base + SPSD2_SD_CMD_BUF3_REG);
	writeb(((cmd->arg >>  0) & 0xff), host->base + SPSD2_SD_CMD_BUF4_REG);

	value = readl(host->base + SPSD2_SD_INT_REG);
	value |= SPSDC_SDINT_SDCMP_CLR;
	if (likely(!host->use_int || cmd->flags & MMC_RSP_136))
		value &= ~SPSDC_SDINT_SDCMPEN;
	else
		value |= SPSDC_SDINT_SDCMPEN;
	writel(value, host->base + SPSD2_SD_INT_REG);

	value = readl(host->base + SPSD2_SD_CONF0_REG);
	value &= ~SPSDC_CONF0_TRANS_MODE;
	value |= SPSDC_CONF0_CMDDUMMY;
	if (likely(cmd->flags & MMC_RSP_PRESENT)) {
		value |= SPSDC_CONF0_AUTORSP;
	} else {
		value &= ~SPSDC_CONF0_AUTORSP;
		writel(value, host->base + SPSD2_SD_CONF0_REG);
		return;
	}

	if (likely(cmd->flags & MMC_RSP_CRC && !(cmd->flags & MMC_RSP_136)))
		value |= SPSDC_CONF0_RSPCHK;
	else
		value &= ~SPSDC_CONF0_RSPCHK;
	writel(value, host->base + SPSD2_SD_CONF0_REG);

	value = readl(host->base + SPSD2_SD_CONF_REG);
	if (unlikely(cmd->flags & MMC_RSP_136))
		value |= SPSDC_CONF_SDRSP_TYPE;
	else
		value &= ~SPSDC_CONF_SDRSP_TYPE;
	writel(value, host->base + SPSD2_SD_CONF_REG);
}

static void spsdc_prepare_data(struct spsdc_host *host, struct mmc_data *data)
{
	u32 value;

	writel(data->blocks - 1, host->base + SPSD2_SD_PAGE_NUM_REG);
	writel(data->blksz - 1, host->base + SPSD2_BLOCKSIZE_REG);
	value = readl(host->base + SPSD2_SD_CONF0_REG);
	value &= ~SPSDC_CONF0_TRANS_MODE;
	if (data->flags & MMC_DATA_READ) {
		value &= ~(SPSDC_CONF0_AUTORSP | SPSDC_CONF0_CMDDUMMY);
		value |= FIELD_PREP(SPSDC_CONF0_TRANS_MODE, SPSDC_TRANS_RD_MODE);
		writel(0x12, host->base + SPSD2_DMA_SRCDST_REG);
	} else {
		value |= FIELD_PREP(SPSDC_CONF0_TRANS_MODE, SPSDC_TRANS_WR_MODE);
		writel(0x21, host->base + SPSD2_DMA_SRCDST_REG);
	}
	value |= SPSDC_CONF0_SDLEN_MODE;
	if (likely(host->dmapio_mode == SPSDC_DMA_MODE)) {
		struct scatterlist *sg;
		dma_addr_t dma_addr;
		unsigned int dma_size;
		void __iomem *reg_addr;
		int dma_direction = data->flags & MMC_DATA_READ ?
				    DMA_FROM_DEVICE : DMA_TO_DEVICE;
		int i, count = dma_map_sg(host->mmc->parent, data->sg,
					  data->sg_len, dma_direction);

		if (unlikely(!count || count > SPSDC_MAX_DMA_MEMORY_SECTORS)) {
			data->error = -EINVAL;
			return;
		}
		for_each_sg(data->sg, sg, count, i) {
			dma_addr = sg_dma_address(sg);
			dma_size = sg_dma_len(sg) / data->blksz - 1;
			if (i == 0) {
				writel(dma_addr,
				       host->base + SPSD2_DMA_BASE_ADDR0_REG);
				writel(dma_addr >> 16,
				       host->base + SPSD2_DMA_BASE_ADDR16_REG);
				writel(dma_size,
				       host->base + SPSD2_SDRAM_SECTOR_SIZE_REG);
			} else {
				reg_addr = host->base +
					   SPSD2_SDRAM_SECTOR_ADDR_REG +
					   (i - 1) * 8;
				writel(dma_addr, reg_addr);
				writel(dma_size, reg_addr + 4);
			}
		}
		value &= ~SPSDC_CONF0_SDPIO_MODE;
		writel(value, host->base + SPSD2_SD_CONF0_REG);
		writel(data->blksz - 1, host->base + SPSD2_DMA_SIZE_REG);
		if (!host->use_int &&
		    data->blksz * data->blocks > host->dma_int_threshold) {
			host->dma_use_int = 1;
			value = readl(host->base + SPSD2_SD_INT_REG);
			value |= SPSDC_SDINT_SDCMPEN;
			writel(value, host->base + SPSD2_SD_INT_REG);
		}
	} else {
		value |= SPSDC_CONF0_SDPIO_MODE;
		writel(value, host->base + SPSD2_SD_CONF0_REG);
	}
}

static inline void spsdc_trigger_transaction(struct spsdc_host *host)
{
	u32 value = readl(host->base + SPSD2_SD_CTRL_REG);

	value |= SPSDC_CTRL_CMD_TRIG;
	writel(value, host->base + SPSD2_SD_CTRL_REG);
}

static int __send_stop_cmd(struct spsdc_host *host, struct mmc_command *stop)
{
	u32 value;

	spsdc_prepare_cmd(host, stop);
	value = readl(host->base + SPSD2_SD_INT_REG);
	value &= ~SPSDC_SDINT_SDCMPEN;
	writel(value, host->base + SPSD2_SD_INT_REG);
	spsdc_trigger_transaction(host);
	if (spsdc_wait_finish(host)) {
		value = readl(host->base + SPSD2_SD_STATUS_REG);
		if (value & SPSDC_STS_RSP_CRC7_ERR)
			stop->error = -EILSEQ;
		else
			stop->error = -ETIMEDOUT;
		return -1;
	}
	spsdc_get_rsp(host, stop);
	return 0;
}

static int __switch_sdio_bus_width(struct spsdc_host *host, int width)
{
	struct mmc_command cmd = {};
	int ret;
	u32 value;
	u8 ctrl;

	cmd.opcode = SD_IO_RW_DIRECT;
	cmd.arg |= SDIO_CCCR_IF << 9;
	cmd.flags = MMC_RSP_R5;
	spsdc_prepare_cmd(host, &cmd);
	value = readl(host->base + SPSD2_SD_INT_REG);
	value &= ~SPSDC_SDINT_SDCMPEN;
	writel(value, host->base + SPSD2_SD_INT_REG);
	spsdc_trigger_transaction(host);
	ret = spsdc_wait_finish(host);
	if (ret) {
		spsdc_sw_reset(host);
		return ret;
	}
	spsdc_get_rsp(host, &cmd);
	ctrl = cmd.resp[0] & 0xff;

	ctrl &= ~SDIO_BUS_WIDTH_MASK;
	if (width == MMC_BUS_WIDTH_4)
		ctrl |= SDIO_BUS_WIDTH_4BIT;

	cmd.arg |= 0x80000000;
	cmd.arg |= ctrl;
	spsdc_prepare_cmd(host, &cmd);
	value = readl(host->base + SPSD2_SD_INT_REG);
	value &= ~SPSDC_SDINT_SDCMPEN;
	writel(value, host->base + SPSD2_SD_INT_REG);
	spsdc_trigger_transaction(host);
	ret = spsdc_wait_finish(host);
	if (ret) {
		spsdc_sw_reset(host);
		return ret;
	}
	spsdc_get_rsp(host, &cmd);
	spsdc_set_bus_width(host, width);

	return ret;
}

static int spsdc_check_error(struct spsdc_host *host, struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	int ret = 0;
	u32 value = readl(host->base + SPSD2_SD_STATE_REG);

	if (unlikely(value & SPSDC_STATE_ERROR)) {
		u32 timing_cfg0, timing_cfg1;

		value = readl(host->base + SPSD2_SD_STATUS_REG);
		timing_cfg0 = readl(host->base + SPSD2_SD_TIMING_CONF0_REG);
		host->tuning_info.wr_dly =
			FIELD_GET(SPSDC_TIMING_CONF0_WRTD, timing_cfg0);
		timing_cfg1 = readl(host->base + SPSD2_SD_TIMING_CONF1_REG);
		host->tuning_info.rd_dly =
			FIELD_GET(SPSDC_TIMING_CONF1_RDTD, timing_cfg1);

		if (value & SPSDC_STS_RSP_TIMEOUT) {
			ret = -ETIMEDOUT;
			host->tuning_info.wr_dly++;
		} else if (value & SPSDC_STS_RSP_CRC7_ERR) {
			ret = -EILSEQ;
			host->tuning_info.rd_dly++;
		}
		if (data) {
			if ((value & SPSDC_STS_STB_TIMEOUT) ||
			    (value & SPSDC_STS_CARD_CRC_TIMEOUT)) {
				ret = -ETIMEDOUT;
				host->tuning_info.rd_dly++;
			} else if (value & SPSDC_STS_CRC_TOKEN_ERR) {
				ret = -EILSEQ;
				host->tuning_info.wr_dly++;
			} else if (value & SPSDC_STS_RDATA_CRC16_ERR) {
				ret = -EILSEQ;
				host->tuning_info.rd_dly++;
			}
			data->error = ret;
			data->bytes_xfered = 0;
		}
		cmd->error = ret;
		if (!host->tuning_info.need_tuning)
			cmd->retries = SPSDC_MAX_RETRIES;
		spsdc_sw_reset(host);
		timing_cfg0 |= FIELD_PREP(SPSDC_TIMING_CONF0_WRTD,
					   host->tuning_info.wr_dly);
		writel(timing_cfg0, host->base + SPSD2_SD_TIMING_CONF0_REG);
		timing_cfg1 |= FIELD_PREP(SPSDC_TIMING_CONF1_RDTD,
					   host->tuning_info.rd_dly);
		writel(timing_cfg1, host->base + SPSD2_SD_TIMING_CONF1_REG);
	} else if (data) {
		data->bytes_xfered = data->blocks * data->blksz;
	}
	host->tuning_info.need_tuning = ret;
	return ret;
}

static void spsdc_xfer_data_pio(struct spsdc_host *host, struct mmc_data *data)
{
	int data_left = data->blocks * data->blksz;
	int consumed, remain;
	struct sg_mapping_iter *sg_miter = &host->sg_miter;
	unsigned int flags = 0;
	u16 *buf;

	if (data->flags & MMC_DATA_WRITE)
		flags |= SG_MITER_FROM_SG;
	else
		flags |= SG_MITER_TO_SG;
	sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
	while (data_left > 0) {
		consumed = 0;
		if (!sg_miter_next(sg_miter))
			break;
		buf = sg_miter->addr;
		remain = sg_miter->length;
		do {
			if (data->flags & MMC_DATA_WRITE) {
				if (spsdc_wait_txbuf_empty(host))
					goto done;
				writel(*buf, host->base + SPSD2_SD_PIO_TX_REG);
			} else {
				if (spsdc_wait_rxbuf_full(host))
					goto done;
				*buf = readl(host->base + SPSD2_SD_PIO_RX_REG);
			}
			buf++;
			consumed += 2;
			remain -= 2;
		} while (remain);
		sg_miter->consumed = consumed;
		data_left -= consumed;
	}
done:
	sg_miter_stop(sg_miter);
}

static void spsdc_controller_init(struct spsdc_host *host)
{
	u32 value;

	/*
	 * Use software reset instead of hardware assert/deassert so that the
	 * DMA stop/reset register is properly cleared.  A hardware reset via
	 * reset_control_assert leaves SPSD2_DMA_STOP_RST_REG in an
	 * indeterminate state that causes subsequent DMA transfers to hang.
	 */
	spsdc_sw_reset(host);

	value = readl(host->base + SPSD2_MEDIA_TYPE_REG);
	value &= ~SPSDC_MEDIA_MASK;
	value |= FIELD_PREP(SPSDC_MEDIA_MASK, SPSDC_MEDIA_SD);
	writel(value, host->base + SPSD2_MEDIA_TYPE_REG);
}

static void spsdc_set_power_mode(struct spsdc_host *host, struct mmc_ios *ios)
{
	if (host->power_state == ios->power_mode)
		return;

	switch (ios->power_mode) {
	case MMC_POWER_ON:
		spsdc_controller_init(host);
		pm_runtime_get_sync(host->mmc->parent);
		break;
	case MMC_POWER_UP:
		break;
	case MMC_POWER_OFF:
		pm_runtime_put(host->mmc->parent);
		break;
	}
	host->power_state = ios->power_mode;
}

static void spsdc_finish_request(struct spsdc_host *host,
				 struct mmc_request *mrq)
{
	struct mmc_command *cmd;
	struct mmc_data *data;

	if (!mrq)
		return;

	cmd = mrq->cmd;
	data = mrq->data;
	if (data && SPSDC_DMA_MODE == host->dmapio_mode) {
		int dma_direction = data->flags & MMC_DATA_READ ?
				    DMA_FROM_DEVICE : DMA_TO_DEVICE;

		dma_unmap_sg(host->mmc->parent, data->sg, data->sg_len,
			     dma_direction);
		host->dma_use_int = 0;
	}
	spsdc_get_rsp(host, cmd);
	spsdc_check_error(host, mrq);
	if (mrq->stop) {
		if (__send_stop_cmd(host, mrq->stop))
			spsdc_sw_reset(host);
	}
	host->mrq = NULL;

	if (host->restore_4bit_sdio_bus) {
		__switch_sdio_bus_width(host, MMC_BUS_WIDTH_4);
		host->restore_4bit_sdio_bus = 0;
	}
	up(&host->mrq_lock);
	mmc_request_done(host->mmc, mrq);
}

static irqreturn_t spsdc_irq(int irq, void *dev_id)
{
	struct spsdc_host *host = dev_id;
	u32 value = readl(host->base + SPSD2_SD_INT_REG);

	spin_lock(&host->lock);
	if ((value & SPSDC_SDINT_SDCMP) && (value & SPSDC_SDINT_SDCMPEN)) {
		value &= ~SPSDC_SDINT_SDCMP;
		value |= SPSDC_SDINT_SDCMP_CLR;
		writel(value, host->base + SPSD2_SD_INT_REG);
		if (host->mrq && host->mrq->stop)
			tasklet_schedule(&host->tsklet_finish_req);
		else
			spsdc_finish_request(host, host->mrq);
	}
	if ((value & SPSDC_SDINT_SDIO) && (value & SPSDC_SDINT_SDIOEN))
		mmc_signal_sdio_irq(host->mmc);
	spin_unlock(&host->lock);
	return IRQ_HANDLED;
}

static void spsdc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct spsdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
	int bus_width = mmc->ios.bus_width;
	int ret;

	ret = down_interruptible(&host->mrq_lock);
	if (ret)
		return;

	host->mrq = mrq;
	data = mrq->data;
	cmd = mrq->cmd;
	if (cmd->opcode == SD_IO_RW_EXTENDED && bus_width == MMC_BUS_WIDTH_4 &&
	    data && data->blocks * data->blksz <= 4) {
		if (__switch_sdio_bus_width(host, MMC_BUS_WIDTH_1)) {
			cmd->error = -1;
			host->mrq = NULL;
			up(&host->mrq_lock);
			mmc_request_done(host->mmc, mrq);
			return;
		}
		host->restore_4bit_sdio_bus = 1;
	}

	spsdc_prepare_cmd(host, cmd);
	if (unlikely(cmd->flags & MMC_RSP_136)) {
		spsdc_trigger_transaction(host);
		spsdc_get_rsp(host, cmd);
		spsdc_wait_finish(host);
		spsdc_check_error(host, mrq);
		host->mrq = NULL;
		up(&host->mrq_lock);
		mmc_request_done(host->mmc, mrq);
	} else {
		if (data)
			spsdc_prepare_data(host, data);
		if (host->dmapio_mode == SPSDC_PIO_MODE && data) {
			u32 value;

			value = readl(host->base + SPSD2_SD_INT_REG);
			value &= ~SPSDC_SDINT_SDCMPEN;
			writel(value, host->base + SPSD2_SD_INT_REG);
			spsdc_trigger_transaction(host);
			spsdc_xfer_data_pio(host, data);
			spsdc_wait_finish(host);
			spsdc_finish_request(host, mrq);
		} else {
			if (!(host->use_int || host->dma_use_int)) {
				spsdc_trigger_transaction(host);
				spsdc_wait_finish(host);
				spsdc_finish_request(host, mrq);
			} else {
				spsdc_trigger_transaction(host);
			}
		}
	}
}

static void spsdc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct spsdc_host *host = mmc_priv(mmc);

	down(&host->mrq_lock);
	spsdc_set_power_mode(host, ios);
	spsdc_set_bus_clk(host, ios->clock);
	spsdc_set_bus_timing(host, ios->timing);
	spsdc_set_bus_width(host, ios->bus_width);
	spsdc_select_mode(host);
	up(&host->mrq_lock);
}

static int spsdc_get_cd(struct mmc_host *mmc)
{
	int ret = 0;

	if (mmc_host_can_gpio_cd(mmc))
		ret = mmc_gpio_get_cd(mmc);
	if (ret < 0)
		ret = 0;

	return ret;
}

static void spsdc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct spsdc_host *host = mmc_priv(mmc);
	u32 value = readl(host->base + SPSD2_SD_INT_REG);

	value |= SPSDC_SDINT_SDIO_CLR;
	if (enable)
		value |= SPSDC_SDINT_SDIOEN;
	else
		value &= ~SPSDC_SDINT_SDIOEN;
	writel(value, host->base + SPSD2_SD_INT_REG);
}

static const struct mmc_host_ops spsdc_ops = {
	.request        = spsdc_request,
	.set_ios        = spsdc_set_ios,
	.get_cd         = spsdc_get_cd,
	.enable_sdio_irq = spsdc_enable_sdio_irq,
};

static void tsklet_func_finish_req(struct tasklet_struct *t)
{
	struct spsdc_host *host = from_tasklet(host, t, tsklet_finish_req);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	spsdc_finish_request(host, host->mrq);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void spsdc_disable_unprepare(void *data)
{
	clk_disable_unprepare(data);
}

static void spsdc_reset_control_assert(void *data)
{
	reset_control_assert(data);
}

static int spsdc_drv_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct spsdc_host *host;
	int ret;

	mmc = mmc_alloc_host(sizeof(*host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->power_state = MMC_POWER_OFF;
	host->dma_int_threshold = 1024;
	host->dmapio_mode = SPSDC_DMA_MODE;

	host->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto free_host;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		ret = host->irq;
		goto free_host;
	}

	ret = devm_request_irq(&pdev->dev, host->irq, spsdc_irq,
			       IRQF_SHARED, dev_name(&pdev->dev), host);
	if (ret)
		goto free_host;

	host->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(host->clk)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(host->clk),
				    "clk get fail\n");
		goto free_host;
	}

	host->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(host->rstc)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(host->rstc),
				    "rst get fail\n");
		goto free_host;
	}

	ret = clk_prepare_enable(host->clk);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "failed to enable clk\n");
		goto free_host;
	}

	ret = devm_add_action_or_reset(&pdev->dev, spsdc_disable_unprepare,
				       host->clk);
	if (ret)
		goto free_host;

	ret = reset_control_deassert(host->rstc);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "failed to deassert reset\n");
		goto free_host;
	}

	ret = devm_add_action_or_reset(&pdev->dev, spsdc_reset_control_assert,
				       host->rstc);
	if (ret)
		goto free_host;

	ret = mmc_of_parse(mmc);
	if (ret)
		goto free_host;

	spin_lock_init(&host->lock);
	sema_init(&host->mrq_lock, 1);
	tasklet_setup(&host->tsklet_finish_req, tsklet_func_finish_req);

	mmc->ops = &spsdc_ops;
	mmc->f_min = SPSDC_MIN_CLK;
	if (mmc->f_max > SPSDC_MAX_CLK)
		mmc->f_max = SPSDC_MAX_CLK;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->max_seg_size = SPSDC_MAX_BLK_COUNT * 512;
	mmc->max_segs = SPSDC_MAX_DMA_MEMORY_SECTORS;
	mmc->max_req_size = SPSDC_MAX_BLK_COUNT * 512;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = SPSDC_MAX_BLK_COUNT;

	if (mmc->caps2 & MMC_CAP2_NO_SDIO)
		host->mode = SPSDC_MODE_SD;
	else
		host->mode = SPSDC_MODE_SDIO;

	dev_set_drvdata(&pdev->dev, host);
	spsdc_controller_init(host);
	spsdc_select_mode(host);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = mmc_add_host(mmc);
	if (ret)
		goto disable_pm;

	return 0;

disable_pm:
	pm_runtime_disable(&pdev->dev);
free_host:
	mmc_free_host(mmc);
	return ret;
}

static void spsdc_drv_remove(struct platform_device *pdev)
{
	struct spsdc_host *host = platform_get_drvdata(pdev);

	mmc_remove_host(host->mmc);
	tasklet_kill(&host->tsklet_finish_req);
	pm_runtime_disable(&pdev->dev);
	mmc_free_host(host->mmc);
}


static int spsdc_pm_runtime_suspend(struct device *dev)
{
	struct spsdc_host *host = dev_get_drvdata(dev);

	clk_disable_unprepare(host->clk);
	return 0;
}

static int spsdc_pm_runtime_resume(struct device *dev)
{
	struct spsdc_host *host = dev_get_drvdata(dev);

	return clk_prepare_enable(host->clk);
}

static DEFINE_RUNTIME_DEV_PM_OPS(spsdc_pm_ops, spsdc_pm_runtime_suspend,
				 spsdc_pm_runtime_resume, NULL);

static const struct of_device_id spsdc_of_table[] = {
	{ .compatible = "sunplus,sp7021-sdhci" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spsdc_of_table);

static struct platform_driver spsdc_driver = {
	.probe    = spsdc_drv_probe,
	.remove   = spsdc_drv_remove,
	.driver   = {
		.name           = "spsdc",
		.pm             = pm_ptr(&spsdc_pm_ops),
		.of_match_table = spsdc_of_table,
	},
};
module_platform_driver(spsdc_driver);

MODULE_AUTHOR("Li-hao Kuo <lhjeff911@gmail.com>");
MODULE_DESCRIPTION("Sunplus SP7021 SD card controller driver");
MODULE_LICENSE("GPL");

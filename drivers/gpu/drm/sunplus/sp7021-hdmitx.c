// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Sunplus SP7021 HDMI TX DRM bridge driver
 *
 * The SP7021 contains an integrated HDMI 1.4 transmitter. This driver
 * implements a DRM bridge supporting 480p, 576p, 720p60 and 1080p60 output.
 * HPD and EDID are handled via the hardware DDC FIFO controller.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hdmi.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>

/* -------------------------------------------------------------------------
 * Register offsets (relative to HDMI TX base at 0x9c00be00 = group 380)
 * -------------------------------------------------------------------------
 */
/* Group 380 */
#define PWR_CTRL		0x014
#define SW_RESET		0x018
#define SYSTEM_STATUS		0x01c
#define SYSTEM_CTRL1		0x020
/* Group 381 */
#define VIDEO_CTRL1		0x0c0
/* Group 382 */
#define VIDEO_FORMAT		0x130
#define AUDIO_CTRL1		0x140
#define AUDIO_CTRL2		0x144
#define AUDIO_SPDIF_CTRL	0x148
#define AUDIO_CHNL_STS2		0x160
#define ACR_CONFIG1		0x168
#define ACR_N_VALUE1		0x16c
/* Group 383 */
#define INTR0_UNMASK		0x184
#define INTR1_UNMASK		0x188
#define INTR0_STS		0x190
#define INTR1_STS		0x194
#define DDC_SLV_ADDR		0x1a0
#define DDC_SLV_OFFSET		0x1a8
#define DDC_DATA_CNT		0x1ac
#define DDC_CMD			0x1b0
#define DDC_STS			0x1b4
#define DDC_DATA		0x1b8
#define DDC_FIFO_CNT		0x1bc
/* Group 385 */
#define INFO_FRAME_CTRL1	0x280
#define INFO_FRAME_CTRL2	0x284
#define AVI_INFO_FRAME01	0x298
#define AVI_INFO_FRAME23	0x29c
#define AVI_INFO_FRAME45	0x2a0
#define AVI_INFO_FRAME67	0x2a4
#define AVI_INFO_FRAME89	0x2a8
#define AVI_INFO_FRAME1011	0x2ac
#define AVI_INFO_FRAME1213	0x2b0
#define AUDIO_INFO_FRAME01	0x2b4
#define AUDIO_INFO_FRAME23	0x2b8
#define AUDIO_INFO_FRAME45	0x2bc
#define AUDIO_INFO_FRAME67	0x2c0
#define AUDIO_INFO_FRAME89	0x2c4
#define AUDIO_INFO_FRAME1011	0x2c8
/* Group 387 (TMDS TX) */
#define TMDSTX_CTRL1		0x3e8
#define TMDSTX_CTRL2		0x3ec
#define TMDSTX_CTRL3		0x3f0
#define TMDSTX_CTRL4		0x3f4
#define TMDSTX_CTRL5		0x3f8

/* MOON4 offsets (PLLTV — second reg region) */
#define PLLTV_CTL0		0x00
#define PLLTV_CTL1		0x04
#define PLLTV_CTL2		0x08

/* MOON5 offset (TMDS L2SW — third reg region) */
#define TMDS_L2SW_CTL		0x00

/* Interrupt0 mask bits */
#define INT0_HPD		BIT(0)
#define INT0_RSEN		BIT(1)

/* Interrupt1 mask bits */
#define INT1_DDC_FIFO_FULL	BIT(2)

/* SYSTEM_STATUS bits */
#define STUS_RSEN_IN		BIT(0)
#define STUS_HPD_IN		BIT(1)

/* DDC commands */
#define DDC_CMD_SEQ_READ	1
#define DDC_CMD_CLEAR_FIFO	5

/* DDC_STS bits */
#define DDC_STS_CMD_DONE	BIT(0)
#define DDC_STS_BUS_LOW		BIT(1)
#define DDC_STS_BUS_NONACK	BIT(2)
#define DDC_STS_FIFO_READ	BIT(4)
#define DDC_STS_FIFO_FULL	BIT(5)
#define DDC_STS_FIFO_EMPTY	BIT(6)

#define DDC_FIFO_SIZE		16
#define DDC_SLAVE_ADDR		0xa0	/* EDID I2C addr 0x50, 8-bit write form */
#define EDID_SIZE		256
#define DDC_POLL_TIMEOUT_US	200000	/* 200ms */

/* -------------------------------------------------------------------------
 * PLL and PHY tables (derived from 5.10 vendor param_cfg.inc, 24-bit depth)
 * -------------------------------------------------------------------------
 */
enum sp7021_timing {
	SP7021_TIMING_480P = 0,
	SP7021_TIMING_576P,
	SP7021_TIMING_720P60,
	SP7021_TIMING_1080P60,
	SP7021_TIMING_MAX,
};

struct pll_tv_param {
	bool bypass;
	u8 r, m, n;
};

static const struct pll_tv_param pll_cfg[SP7021_TIMING_MAX] = {
	[SP7021_TIMING_480P]    = { .bypass = true  },			/* 27 MHz bypass */
	[SP7021_TIMING_576P]    = { .bypass = true  },			/* 27 MHz bypass */
	[SP7021_TIMING_720P60]  = { .bypass = false, .r = 1, .m = 1, .n = 0x0a },	/* 74.25 MHz */
	[SP7021_TIMING_1080P60] = { .bypass = false, .r = 0, .m = 1, .n = 0x0a },	/* 148.5 MHz */
};

struct phy_param {
	u8 aclk_mode, is_data_double, kv_mode, term_mode;
	u8 ectr_mode, is_emp, is_clk_detector, fckdv_mode;
	u8 ckinv_mode, is_from_odd, is_clk_inv;
	u8 icp_mod_mode, pd_d_mode, cpst_mode, icp_mode;
	u8 bgr_mode, sw_ctrl, dsel_mode;
	u8 irt_mode, dcnst_mode, rv_model;
};

static const struct phy_param phy_cfg[SP7021_TIMING_MAX] = {
	[SP7021_TIMING_480P] = {
		.kv_mode = 0, .term_mode = 0, .ectr_mode = 5,
		.is_clk_detector = 1, .fckdv_mode = 1,
		.icp_mod_mode = 5, .cpst_mode = 7, .icp_mode = 1,
		.bgr_mode = 3, .sw_ctrl = 5, .dsel_mode = 0x15,
		.dcnst_mode = 0x1f, .rv_model = 1,
	},
	[SP7021_TIMING_576P] = {
		.kv_mode = 0, .term_mode = 0, .ectr_mode = 0,
		.is_clk_detector = 1, .fckdv_mode = 1,
		.icp_mod_mode = 5, .cpst_mode = 7, .icp_mode = 1,
		.bgr_mode = 3, .sw_ctrl = 5, .dsel_mode = 0x15,
		.dcnst_mode = 0x1f, .rv_model = 0,
	},
	[SP7021_TIMING_720P60] = {
		.kv_mode = 1, .term_mode = 0, .ectr_mode = 5,
		.is_clk_detector = 1, .fckdv_mode = 1,
		.icp_mod_mode = 5, .cpst_mode = 7, .icp_mode = 1,
		.bgr_mode = 3, .sw_ctrl = 5, .dsel_mode = 0x15,
		.dcnst_mode = 0x1f, .rv_model = 1,
	},
	[SP7021_TIMING_1080P60] = {
		.kv_mode = 3, .term_mode = 1, .ectr_mode = 7,
		.is_clk_detector = 1, .fckdv_mode = 1,
		.icp_mod_mode = 0xf, .cpst_mode = 7, .icp_mode = 1,
		.bgr_mode = 3, .sw_ctrl = 6, .dsel_mode = 0x15,
		.dcnst_mode = 0x1f, .rv_model = 1,
	},
};

/* Pixel clock in kHz for each timing (used for mode matching) */
static const unsigned int timing_pclk_khz[SP7021_TIMING_MAX] = {
	[SP7021_TIMING_480P]    = 27000,
	[SP7021_TIMING_576P]    = 27000,
	[SP7021_TIMING_720P60]  = 74250,
	[SP7021_TIMING_1080P60] = 148500,
};

/* Horizontal active pixels for each timing */
static const unsigned int timing_hdisplay[SP7021_TIMING_MAX] = {
	[SP7021_TIMING_480P]    = 720,
	[SP7021_TIMING_576P]    = 720,
	[SP7021_TIMING_720P60]  = 1280,
	[SP7021_TIMING_1080P60] = 1920,
};

/* -------------------------------------------------------------------------
 * Device structure
 * -------------------------------------------------------------------------
 */
struct sp7021_hdmitx {
	struct drm_bridge bridge;
	struct device *dev;
	void __iomem *base;	/* HDMI TX group 380 base */
	void __iomem *moon4;	/* PLLTV control */
	void __iomem *moon5;	/* TMDS L2SW control */
	struct clk *clk;
	struct reset_control *rst;
	int irq;
	struct mutex lock;
	enum sp7021_timing timing;
	bool hpd_active;
};

static inline struct sp7021_hdmitx *bridge_to_sp7021(struct drm_bridge *b)
{
	return container_of(b, struct sp7021_hdmitx, bridge);
}

/* -------------------------------------------------------------------------
 * Hardware access
 * -------------------------------------------------------------------------
 */
static void sp7021_apply_pll(struct sp7021_hdmitx *tx)
{
	const struct pll_tv_param *p = &pll_cfg[tx->timing];

	if (p->bypass) {
		writel(0x80000000 | BIT(15), tx->moon4 + PLLTV_CTL0);
	} else {
		writel(0x80000000, tx->moon4 + PLLTV_CTL0);
		writel(0x01800000 | ((u32)p->r << 7), tx->moon4 + PLLTV_CTL1);
		writel(0x7fff0000 | ((u32)p->m << 8) | p->n, tx->moon4 + PLLTV_CTL2);
	}
}

static void sp7021_apply_phy(struct sp7021_hdmitx *tx)
{
	const struct phy_param *p = &phy_cfg[tx->timing];
	u32 val, mask;

	writel(0x1f70000 |
	       ((u32)(p->aclk_mode & 0x3) << 7) |
	       ((u32)(p->is_data_double & 0x1) << 6) |
	       ((u32)(p->kv_mode & 0x3) << 4) |
	       ((u32)(p->term_mode & 0x7) << 0),
	       tx->moon5 + TMDS_L2SW_CTL);

	val = readl(tx->base + TMDSTX_CTRL1);
	mask = 0xfff3;
	writel((val & ~mask) |
	       ((u32)(p->ectr_mode & 0xf) << 12) |
	       ((u32)(p->is_emp & 0x1) << 11) |
	       ((u32)(p->is_clk_detector & 0x1) << 10) |
	       ((u32)(p->fckdv_mode & 0x3) << 8) |
	       ((u32)(p->ckinv_mode & 0xf) << 4) |
	       ((u32)(p->is_from_odd & 0x1) << 1) |
	       ((u32)(p->is_clk_inv & 0x1) << 0),
	       tx->base + TMDSTX_CTRL1);

	val = readl(tx->base + TMDSTX_CTRL2);
	mask = 0xf7f2;
	writel((val & ~mask) |
	       ((u32)(p->icp_mod_mode & 0xf) << 12) |
	       ((u32)(p->pd_d_mode & 0x7) << 8) |
	       ((u32)(p->cpst_mode & 0xf) << 4) |
	       ((u32)(p->icp_mode & 0x1) << 1),
	       tx->base + TMDSTX_CTRL2);

	val = readl(tx->base + TMDSTX_CTRL3);
	mask = 0xef3f;
	writel((val & ~mask) |
	       ((u32)(p->bgr_mode & 0x7) << 13) |
	       ((u32)(p->sw_ctrl & 0xf) << 8) |
	       ((u32)(p->dsel_mode & 0x3f) << 0),
	       tx->base + TMDSTX_CTRL3);

	val = readl(tx->base + TMDSTX_CTRL4);
	mask = 0xfc3f;
	writel((val & ~mask) |
	       ((u32)(p->irt_mode & 0x3f) << 10) |
	       ((u32)(p->dcnst_mode & 0x3f) << 0),
	       tx->base + TMDSTX_CTRL4);

	val = readl(tx->base + TMDSTX_CTRL5);
	writel((val & ~0x2) | ((u32)(p->rv_model & 0x1) << 1), tx->base + TMDSTX_CTRL5);
}

static void sp7021_apply_video(struct sp7021_hdmitx *tx)
{
	u32 hv_pol;

	switch (tx->timing) {
	case SP7021_TIMING_720P60:
	case SP7021_TIMING_1080P60:
		hv_pol = 0;
		break;
	default:
		hv_pol = 3;
		break;
	}

	/* 24-bit color depth (cd=4), H/V polarity */
	writel((4 << 4) | hv_pol, tx->base + VIDEO_FORMAT);

	/* HDMI mode, output enable */
	writel(BIT(12) | 1, tx->base + SYSTEM_CTRL1);

	/* Enable infoframe transmission */
	writel(0x1013, tx->base + INFO_FRAME_CTRL2);
}

static void sp7021_apply_audio(struct sp7021_hdmitx *tx)
{
	/* I2S, 2ch, 48kHz LPCM — fixed default for now */
	writel((3 << 14) | (2 << 12) | (1 << 10) | (1 << 4) | 1,
	       tx->base + AUDIO_CTRL1);
	writel(0x01b5, tx->base + AUDIO_CTRL2);
	writel(0x200, tx->base + AUDIO_CHNL_STS2);	/* 48kHz */
	writel((0x3f << 1) | 0, tx->base + AUDIO_SPDIF_CTRL);
	writel(0x5, tx->base + ACR_CONFIG1);
	writel(0x1800, tx->base + ACR_N_VALUE1);	/* N=6144 for 48kHz */
}

static void sp7021_apply_avi_infoframe(struct sp7021_hdmitx *tx)
{
	struct hdmi_avi_infoframe frame;
	u8 buf[HDMI_INFOFRAME_SIZE(AVI)];
	ssize_t len;

	hdmi_avi_infoframe_init(&frame);
	frame.colorspace = HDMI_COLORSPACE_RGB;
	frame.quantization_range = HDMI_QUANTIZATION_RANGE_LIMITED;

	switch (tx->timing) {
	case SP7021_TIMING_480P:
		frame.picture_aspect = HDMI_PICTURE_ASPECT_4_3;
		frame.colorimetry = HDMI_COLORIMETRY_ITU_601;
		frame.video_code = 2;
		break;
	case SP7021_TIMING_576P:
		frame.picture_aspect = HDMI_PICTURE_ASPECT_4_3;
		frame.colorimetry = HDMI_COLORIMETRY_ITU_601;
		frame.video_code = 17;
		break;
	case SP7021_TIMING_720P60:
		frame.picture_aspect = HDMI_PICTURE_ASPECT_16_9;
		frame.colorimetry = HDMI_COLORIMETRY_ITU_709;
		frame.video_code = 4;
		break;
	case SP7021_TIMING_1080P60:
	default:
		frame.picture_aspect = HDMI_PICTURE_ASPECT_16_9;
		frame.colorimetry = HDMI_COLORIMETRY_ITU_709;
		frame.video_code = 16;
		break;
	}

	len = hdmi_avi_infoframe_pack(&frame, buf, sizeof(buf));
	if (len < 0)
		return;

	/*
	 * Each AVI_INFO_FRAME register holds two infoframe bytes:
	 * bits[7:0] = buf[n], bits[15:8] = buf[n+1], starting from buf[3]
	 * (the checksum byte; buf[0..2] are type/version/length).
	 */
	writel(buf[3] | ((u32)buf[4] << 8),  tx->base + AVI_INFO_FRAME01);
	writel(buf[5] | ((u32)buf[6] << 8),  tx->base + AVI_INFO_FRAME23);
	writel(buf[7] | ((u32)buf[8] << 8),  tx->base + AVI_INFO_FRAME45);
	writel(buf[9] | ((u32)buf[10] << 8), tx->base + AVI_INFO_FRAME67);
	writel(buf[11] | ((u32)buf[12] << 8), tx->base + AVI_INFO_FRAME89);
	writel(buf[13] | ((u32)buf[14] << 8), tx->base + AVI_INFO_FRAME1011);
	writel(buf[15] | ((u32)buf[16] << 8), tx->base + AVI_INFO_FRAME1213);
}

static void sp7021_apply_audio_infoframe(struct sp7021_hdmitx *tx)
{
	struct hdmi_audio_infoframe frame;
	u8 buf[HDMI_INFOFRAME_SIZE(AUDIO)];
	ssize_t len;

	hdmi_audio_infoframe_init(&frame);
	frame.channels = 2;

	len = hdmi_audio_infoframe_pack(&frame, buf, sizeof(buf));
	if (len < 0)
		return;

	writel(buf[3] | ((u32)buf[4] << 8),  tx->base + AUDIO_INFO_FRAME01);
	writel(buf[5] | ((u32)buf[6] << 8),  tx->base + AUDIO_INFO_FRAME23);
	writel(buf[7] | ((u32)buf[8] << 8),  tx->base + AUDIO_INFO_FRAME45);
	writel(buf[9] | ((u32)buf[10] << 8), tx->base + AUDIO_INFO_FRAME67);
	writel(buf[11] | ((u32)buf[12] << 8), tx->base + AUDIO_INFO_FRAME89);
	writel(buf[13],                        tx->base + AUDIO_INFO_FRAME1011);
}

static void sp7021_hdmitx_hw_init(struct sp7021_hdmitx *tx)
{
	/* Power on all blocks */
	writel(0x1f, tx->base + PWR_CTRL);

	/* Software reset — brief delay for hardware to settle */
	writel(0x00, tx->base + SW_RESET);
	udelay(100);
	writel(0xff, tx->base + SW_RESET);
	udelay(500);

	/* Enable HPD and RSEN interrupts */
	writel(readl(tx->base + INTR0_UNMASK) | INT0_HPD | INT0_RSEN,
	       tx->base + INTR0_UNMASK);
	writel(readl(tx->base + INTR1_UNMASK) | INT1_DDC_FIFO_FULL,
	       tx->base + INTR1_UNMASK);

	/* DDC: set EDID slave address and FIFO transfer size */
	writel(DDC_SLAVE_ADDR, tx->base + DDC_SLV_ADDR);
	writel(DDC_FIFO_SIZE, tx->base + DDC_DATA_CNT);
}

static void sp7021_hdmitx_hw_start(struct sp7021_hdmitx *tx)
{
	sp7021_apply_pll(tx);
	sp7021_apply_phy(tx);
	sp7021_apply_avi_infoframe(tx);
	sp7021_apply_audio_infoframe(tx);
	sp7021_apply_video(tx);
	sp7021_apply_audio(tx);
	writel(readl(tx->base + INFO_FRAME_CTRL1) | 0x1b1b,
	       tx->base + INFO_FRAME_CTRL1);
}

static void sp7021_hdmitx_hw_stop(struct sp7021_hdmitx *tx)
{
	writel(0x1010, tx->base + INFO_FRAME_CTRL1);
	writel(0x1010, tx->base + INFO_FRAME_CTRL2);
}

/* -------------------------------------------------------------------------
 * EDID reading via hardware DDC FIFO
 *
 * The DDC controller is a simple I2C master with a 16-byte FIFO.  We
 * follow the same sequence as the 5.10 vendor driver:
 *   1. CLEAR_FIFO, poll CMD_DONE
 *   2. SEQ_READ at offset, poll FIFO_FULL or CMD_DONE
 *   3. Drain FIFO, repeat
 * -------------------------------------------------------------------------
 */
static int sp7021_ddc_wait(struct sp7021_hdmitx *tx, u32 ok_mask)
{
	unsigned int timeout = DDC_POLL_TIMEOUT_US / 100;
	u32 sts;

	do {
		udelay(100);
		sts = readl(tx->base + DDC_STS);
		if (sts & (DDC_STS_BUS_LOW | DDC_STS_BUS_NONACK)) {
			dev_dbg(tx->dev, "DDC bus error sts=0x%02x\n", sts);
			return -EIO;
		}
		if (sts & ok_mask)
			return 0;
	} while (--timeout);

	dev_dbg(tx->dev, "DDC timeout sts=0x%02x\n", readl(tx->base + DDC_STS));
	return -ETIMEDOUT;
}

static int sp7021_read_edid(struct sp7021_hdmitx *tx, u8 *buf)
{
	unsigned int offset, i;
	int ret;

	/* Ensure data count is set */
	writel(DDC_FIFO_SIZE, tx->base + DDC_DATA_CNT);

	/* Clear FIFO; wait for CMD_DONE before issuing reads */
	writel(DDC_CMD_CLEAR_FIFO, tx->base + DDC_CMD);
	ret = sp7021_ddc_wait(tx, DDC_STS_CMD_DONE);
	if (ret)
		return ret;

	for (offset = 0; offset < EDID_SIZE; offset += DDC_FIFO_SIZE) {
		/* Issue sequential read: command then offset (vendor order) */
		writel(DDC_CMD_SEQ_READ, tx->base + DDC_CMD);
		writel(offset, tx->base + DDC_SLV_OFFSET);

		/* Wait for FIFO_FULL (16 bytes ready) or CMD_DONE */
		ret = sp7021_ddc_wait(tx, DDC_STS_FIFO_FULL | DDC_STS_CMD_DONE);
		if (ret)
			return ret;

		for (i = 0; i < DDC_FIFO_SIZE; i++)
			buf[offset + i] = readl(tx->base + DDC_DATA) & 0xff;
	}

	return 0;
}

/* -------------------------------------------------------------------------
 * Mode matching
 * -------------------------------------------------------------------------
 */
static enum sp7021_timing sp7021_mode_to_timing(const struct drm_display_mode *mode)
{
	unsigned int i;

	for (i = 0; i < SP7021_TIMING_MAX; i++) {
		unsigned int pclk = timing_pclk_khz[i];

		if (mode->hdisplay != timing_hdisplay[i])
			continue;
		/* Allow ±1% pixel clock tolerance for fractional rates */
		if (mode->clock >= pclk - pclk / 100 &&
		    mode->clock <= pclk + pclk / 100)
			return i;
	}

	return SP7021_TIMING_MAX;
}

/* -------------------------------------------------------------------------
 * DRM bridge callbacks
 * -------------------------------------------------------------------------
 */
static int sp7021_hdmitx_bridge_attach(struct drm_bridge *bridge,
				       struct drm_encoder *encoder,
				       enum drm_bridge_attach_flags flags)
{
	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR))
		return -EINVAL;

	return 0;
}

static enum drm_mode_status
sp7021_hdmitx_bridge_mode_valid(struct drm_bridge *bridge,
				const struct drm_display_info *info,
				const struct drm_display_mode *mode)
{
	return sp7021_mode_to_timing(mode) < SP7021_TIMING_MAX ?
		MODE_OK : MODE_NOMODE;
}

static void sp7021_hdmitx_bridge_mode_set(struct drm_bridge *bridge,
					  const struct drm_display_mode *mode,
					  const struct drm_display_mode *adj)
{
	struct sp7021_hdmitx *tx = bridge_to_sp7021(bridge);
	enum sp7021_timing t = sp7021_mode_to_timing(adj);

	if (t < SP7021_TIMING_MAX)
		tx->timing = t;
}

static void sp7021_hdmitx_bridge_atomic_enable(struct drm_bridge *bridge,
					       struct drm_atomic_state *state)
{
	struct sp7021_hdmitx *tx = bridge_to_sp7021(bridge);

	mutex_lock(&tx->lock);
	sp7021_hdmitx_hw_start(tx);
	mutex_unlock(&tx->lock);
}

static void sp7021_hdmitx_bridge_atomic_disable(struct drm_bridge *bridge,
						struct drm_atomic_state *state)
{
	struct sp7021_hdmitx *tx = bridge_to_sp7021(bridge);

	mutex_lock(&tx->lock);
	sp7021_hdmitx_hw_stop(tx);
	mutex_unlock(&tx->lock);
}

static enum drm_connector_status
sp7021_hdmitx_bridge_detect(struct drm_bridge *bridge,
			    struct drm_connector *connector)
{
	/*
	 * HPD is not wired to the HDMI TX HPD input on the LTPP3G2 board —
	 * the 5.10 vendor driver also treats the monitor as always connected
	 * (CONFIG_HPD_DETECTION defaults to n). Report connected unconditionally
	 * until proper HPD GPIO support is added.
	 */
	return connector_status_connected;
}

static const struct drm_edid *
sp7021_hdmitx_bridge_edid_read(struct drm_bridge *bridge,
				struct drm_connector *connector)
{
	struct sp7021_hdmitx *tx = bridge_to_sp7021(bridge);
	u8 *buf;
	const struct drm_edid *drm_edid;
	int ret;

	buf = kmalloc(EDID_SIZE, GFP_KERNEL);
	if (!buf)
		return NULL;

	mutex_lock(&tx->lock);
	ret = sp7021_read_edid(tx, buf);
	mutex_unlock(&tx->lock);

	if (ret) {
		dev_err(tx->dev, "EDID read failed: %d\n", ret);
		kfree(buf);
		return NULL;
	}

	drm_edid = drm_edid_alloc(buf, EDID_SIZE);
	kfree(buf);
	return drm_edid;
}

static int sp7021_hdmitx_bridge_get_modes(struct drm_bridge *bridge,
					  struct drm_connector *connector)
{
	/*
	 * Fallback fixed modes matching the hardware's supported timings.
	 * Used when EDID cannot be read (e.g. DDC EEPROM not powered).
	 * 720p60 is preferred as the safe default.
	 */
	static const struct {
		unsigned int clock;
		int hdisplay, hss, hse, htotal;
		int vdisplay, vss, vse, vtotal;
		unsigned int flags;
		bool preferred;
	} fixed_modes[] = {
		{ 74250, 1280, 1390, 1430, 1650,  720,  725,  730,  750,
		  DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC, true  },
		{ 148500, 1920, 2008, 2052, 2200, 1080, 1084, 1089, 1125,
		  DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC, false },
		{ 27000,  720,  736,  798,  858,  480,  489,  495,  525,
		  DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC, false },
		{ 27000,  720,  732,  796,  864,  576,  581,  586,  625,
		  DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC, false },
	};
	int i, count = 0;

	for (i = 0; i < ARRAY_SIZE(fixed_modes); i++) {
		struct drm_display_mode *mode = drm_mode_create(connector->dev);

		if (!mode)
			continue;

		mode->clock       = fixed_modes[i].clock;
		mode->hdisplay    = fixed_modes[i].hdisplay;
		mode->hsync_start = fixed_modes[i].hss;
		mode->hsync_end   = fixed_modes[i].hse;
		mode->htotal      = fixed_modes[i].htotal;
		mode->vdisplay    = fixed_modes[i].vdisplay;
		mode->vsync_start = fixed_modes[i].vss;
		mode->vsync_end   = fixed_modes[i].vse;
		mode->vtotal      = fixed_modes[i].vtotal;
		mode->flags       = fixed_modes[i].flags;
		mode->type        = DRM_MODE_TYPE_DRIVER;
		if (fixed_modes[i].preferred)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
		count++;
	}

	return count;
}

static void sp7021_hdmitx_bridge_hpd_enable(struct drm_bridge *bridge)
{
	struct sp7021_hdmitx *tx = bridge_to_sp7021(bridge);

	writel(readl(tx->base + INTR0_UNMASK) | INT0_HPD, tx->base + INTR0_UNMASK);
}

static void sp7021_hdmitx_bridge_hpd_disable(struct drm_bridge *bridge)
{
	struct sp7021_hdmitx *tx = bridge_to_sp7021(bridge);

	writel(readl(tx->base + INTR0_UNMASK) & ~INT0_HPD, tx->base + INTR0_UNMASK);
}

static const struct drm_bridge_funcs sp7021_hdmitx_bridge_funcs = {
	.attach          = sp7021_hdmitx_bridge_attach,
	.mode_valid      = sp7021_hdmitx_bridge_mode_valid,
	.mode_set        = sp7021_hdmitx_bridge_mode_set,
	.atomic_enable   = sp7021_hdmitx_bridge_atomic_enable,
	.atomic_disable  = sp7021_hdmitx_bridge_atomic_disable,
	.detect          = sp7021_hdmitx_bridge_detect,
	.get_modes       = sp7021_hdmitx_bridge_get_modes,
	.edid_read       = sp7021_hdmitx_bridge_edid_read,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset           = drm_atomic_helper_bridge_reset,
};

/* -------------------------------------------------------------------------
 * IRQ handler
 * -------------------------------------------------------------------------
 */
static irqreturn_t sp7021_hdmitx_irq(int irq, void *data)
{
	struct sp7021_hdmitx *tx = data;
	u32 intr0 = readl(tx->base + INTR0_STS);
	enum drm_connector_status status;

	if (intr0 & INT0_HPD) {
		bool hpd = !!(readl(tx->base + SYSTEM_STATUS) & STUS_HPD_IN);

		if (hpd != tx->hpd_active) {
			tx->hpd_active = hpd;
			status = hpd ? connector_status_connected
				     : connector_status_disconnected;
			drm_bridge_hpd_notify(&tx->bridge, status);
			dev_dbg(tx->dev, "HPD %s\n", hpd ? "connected" : "disconnected");
		}

		writel(intr0 & ~INT0_HPD, tx->base + INTR0_STS);
	}

	if (intr0 & INT0_RSEN)
		writel(intr0 & ~INT0_RSEN, tx->base + INTR0_STS);

	return IRQ_HANDLED;
}

/* -------------------------------------------------------------------------
 * Platform driver
 * -------------------------------------------------------------------------
 */
static int sp7021_hdmitx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sp7021_hdmitx *tx;
	int ret;

	tx = devm_drm_bridge_alloc(dev, struct sp7021_hdmitx, bridge,
				   &sp7021_hdmitx_bridge_funcs);
	if (IS_ERR(tx))
		return PTR_ERR(tx);

	tx->dev = dev;
	mutex_init(&tx->lock);
	tx->timing = SP7021_TIMING_1080P60;

	tx->base = devm_platform_ioremap_resource_byname(pdev, "hdmitx");
	if (IS_ERR(tx->base))
		return dev_err_probe(dev, PTR_ERR(tx->base), "failed to map hdmitx regs\n");

	/*
	 * MOON4 and MOON5 are shared with the clock controller — use
	 * devm_ioremap without request_mem_region to avoid EBUSY.
	 */
	{
		struct resource *r;

		r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "plltv");
		if (!r)
			return dev_err_probe(dev, -ENODEV, "no plltv resource\n");
		tx->moon4 = devm_ioremap(dev, r->start, resource_size(r));
		if (!tx->moon4)
			return dev_err_probe(dev, -ENOMEM, "failed to map plltv regs\n");

		r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "tmds");
		if (!r)
			return dev_err_probe(dev, -ENODEV, "no tmds resource\n");
		tx->moon5 = devm_ioremap(dev, r->start, resource_size(r));
		if (!tx->moon5)
			return dev_err_probe(dev, -ENOMEM, "failed to map tmds regs\n");
	}

	tx->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(tx->clk))
		return dev_err_probe(dev, PTR_ERR(tx->clk), "failed to get clock\n");

	tx->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(tx->rst))
		return dev_err_probe(dev, PTR_ERR(tx->rst), "failed to get reset\n");

	reset_control_deassert(tx->rst);

	tx->irq = platform_get_irq(pdev, 0);
	if (tx->irq < 0)
		return tx->irq;

	ret = devm_request_irq(dev, tx->irq, sp7021_hdmitx_irq,
			       IRQF_SHARED, dev_name(dev), tx);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	sp7021_hdmitx_hw_init(tx);

	tx->bridge.of_node = dev->of_node;
	tx->bridge.type = DRM_MODE_CONNECTOR_HDMIA;
	tx->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID | DRM_BRIDGE_OP_MODES;

	drm_bridge_add(&tx->bridge);
	platform_set_drvdata(pdev, tx);

	dev_info(dev, "HDMI TX registered\n");
	return 0;
}

static void sp7021_hdmitx_remove(struct platform_device *pdev)
{
	struct sp7021_hdmitx *tx = platform_get_drvdata(pdev);

	drm_bridge_remove(&tx->bridge);
	reset_control_assert(tx->rst);
}

static const struct of_device_id sp7021_hdmitx_of_match[] = {
	{ .compatible = "sunplus,sp7021-hdmitx" },
	{ }
};
MODULE_DEVICE_TABLE(of, sp7021_hdmitx_of_match);

static struct platform_driver sp7021_hdmitx_driver = {
	.probe  = sp7021_hdmitx_probe,
	.remove = sp7021_hdmitx_remove,
	.driver = {
		.name           = "sp7021-hdmitx",
		.of_match_table = sp7021_hdmitx_of_match,
	},
};
module_platform_driver(sp7021_hdmitx_driver);

MODULE_AUTHOR("Sunplus Technology");
MODULE_DESCRIPTION("Sunplus SP7021 HDMI TX DRM bridge driver");
MODULE_LICENSE("GPL");

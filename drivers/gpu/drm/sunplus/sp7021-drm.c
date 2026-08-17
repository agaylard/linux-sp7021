// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Sunplus SP7021 DRM display driver
 *
 * Drives the OSD0 layer through DMIX and TGEN to the HDMI TX bridge.
 * Supports 480p, 576p, 720p60 and 1080p60 via the sp7021-hdmitx bridge.
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mode_config.h>
#include <drm/drm_of.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

/* -------------------------------------------------------------------------
 * Register offsets within the display register block (base 0x9c005c80)
 *
 * Block layout (derived from struct DISP_REG_t in 5.10 vendor reg_disp.h):
 *   G185 DDFCH  @ 0x000  (1 group = 128 B)
 *   [10 reserved groups]
 *   G196 OSD    @ 0x580  (1 group)
 *   [2 reserved groups]
 *   G199 VPOST  @ 0x700  (1 group)
 *   [6 reserved groups]
 *   G206 GPOST  @ 0xA80  (1 group)
 *   [6 reserved groups]
 *   G213 TGEN   @ 0xE00  (1 group)
 *   [3 reserved groups]
 *   G217 DMIX   @ 0x1000 (1 group)
 *   [16 reserved groups]
 *   G234 DVE    @ 0x1880 (2 groups)
 * -------------------------------------------------------------------------
 */

/* TGEN registers */
#define TGEN_CONFIG		0x0e00	/* latch mode */
#define TGEN_RESET		0x0e04
#define TGEN_USER_INT1		0x0e08	/* vsync interrupt line */
#define TGEN_USER_INT2		0x0e0c
#define TGEN_DTG_CONFIG		0x0e10	/* timing format + fps */
#define TGEN_DTG_ADJ3		0x0e64	/* OSD0 pipeline delay */
#define TGEN_DTG_ADJ4		0x0e68	/* PTG pipeline delay */
#define TGEN_SOURCE_SEL		0x0e74

/* DMIX registers */
#define DMIX_CONFIG0		0x1000	/* BG and L1/L2 input selection */
#define DMIX_CONFIG1		0x1004	/* layer blend mode */
#define DMIX_CONFIG2		0x1008	/* L3-L6 input selection */
#define DMIX_PTG_CONFIG		0x1020	/* pattern generator */
#define DMIX_PTG_CFG4		0x102c	/* PTG Y value */
#define DMIX_PTG_CFG5		0x1030	/* PTG Cb value */
#define DMIX_PTG_CFG6		0x1034	/* PTG Cr value */
#define DMIX_PIX_EN_SEL		0x1068

/* OSD registers */
#define OSD_CTRL		0x0580
#define OSD_EN			0x0584
#define OSD_BASE_ADDR		0x0588	/* DMA address of OSD header */
#define OSD_HVLD_OFFSET		0x05c0
#define OSD_HVLD_WIDTH		0x05c4
#define OSD_VVLD_OFFSET		0x05c8
#define OSD_VVLD_HEIGHT		0x05cc
#define OSD_BIST_CTRL		0x05d4
#define OSD_3D_H_OFFSET		0x05ec
#define OSD_SRC_DECIM		0x05f4

/* GPOST registers */
#define GPOST_CONFIG		0x0a80
#define GPOST_BG1		0x0a94
#define GPOST_BG2		0x0a98
#define GPOST_CONTRAST		0x0a9c
#define GPOST_MASTER_EN		0x0ac8

/* OSD control bits */
#define OSD_CTRL_RGB_MODE	BIT(10)
#define OSD_CTRL_CLUT_ARGB	BIT(7)
#define OSD_CTRL_LATCH_EN	BIT(5)
#define OSD_CTRL_A32B32_EN	BIT(4)
#define OSD_CTRL_FIFO_DEPTH	(7 << 0)

/*
 * OSD header format code for XRGB8888 / ARGB8888.
 * The header lives in DMA-coherent memory and is read by the hardware in
 * big-endian byte order, so all header words are written with cpu_to_be32().
 *
 * Word 0 layout (big-endian): [config0][reserved][config1][blend_level]
 *   config0[3:0] = 0x8  (ARGB8888 pixel format)
 *   config1[4]   = 1    (byte-swap within each 32-bit word)
 */
#define OSD_HDR_FMT_ARGB8888	0x8
#define OSD_HDR_W0		cpu_to_be32((OSD_HDR_FMT_ARGB8888 << 24) | 0x00001000)
#define OSD_HDR_W5(width)	cpu_to_be32(0x00010000 | (width))
#define OSD_HDR_W6_END		cpu_to_be32(0xffffffe0)	/* link_next = end-of-list */

/*
 * TGEN dtg_config encoding:
 *   bits [10:8] = format (0=480P, 1=576P, 2=720P, 3=1080P)
 *   bits [5:4]  = fps    (0=60Hz, 1=50Hz)
 */
static const u32 tgen_dtg_cfg[] = {
	[0] = (0 << 8) | (0 << 4),	/* 480p60 */
	[1] = (1 << 8) | (1 << 4),	/* 576p50 */
	[2] = (2 << 8) | (0 << 4),	/* 720p60 */
	[3] = (3 << 8) | (0 << 4),	/* 1080p60 */
};

/* Pixel clock in kHz — matches sp7021-hdmitx.c timing table */
static const unsigned int disp_pclk_khz[] = { 27000, 27000, 74250, 148500 };
static const unsigned int disp_hdisplay[] = { 720, 720, 1280, 1920 };

/* -------------------------------------------------------------------------
 * Device structure
 * -------------------------------------------------------------------------
 */
#define NUM_CLKS  5
#define NUM_RSTS  5

static const char * const sp7021_clk_names[NUM_CLKS] = {
	"tgen", "dmix", "osd0", "gpost0", "dve"
};
static const char * const sp7021_rst_names[NUM_RSTS] = {
	"tgen", "dmix", "osd0", "gpost0", "dve"
};

struct sp7021_drm {
	struct drm_device	drm;
	struct device		*dev;
	void __iomem		*base;

	struct clk		*clks[NUM_CLKS];
	struct reset_control	*rsts[NUM_RSTS];

	/* OSD header in DMA-coherent memory (128 bytes) */
	u32			*osd_hdr;
	dma_addr_t		osd_hdr_phys;

	struct drm_crtc		crtc;
	struct drm_plane	primary;
	struct drm_encoder	encoder;

	int			vsync_irq;
	spinlock_t		lock;

	unsigned int		timing;	/* index into tgen_dtg_cfg[] */
};

DEFINE_DRM_GEM_DMA_FOPS(sp7021_drm_fops);

static struct drm_driver sp7021_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.fops			= &sp7021_drm_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	.name			= "sp7021-drm",
	.desc			= "Sunplus SP7021 display",
	.major			= 1,
	.minor			= 0,
};

static inline struct sp7021_drm *crtc_to_sp7021(struct drm_crtc *c)
{
	return container_of(c, struct sp7021_drm, crtc);
}

static inline struct sp7021_drm *plane_to_sp7021(struct drm_plane *p)
{
	return container_of(p, struct sp7021_drm, primary);
}

/* -------------------------------------------------------------------------
 * Hardware helpers
 * -------------------------------------------------------------------------
 */
static void sp7021_hw_write(struct sp7021_drm *d, u32 reg, u32 val)
{
	writel(val, d->base + reg);
}

static int sp7021_timing_for_mode(const struct drm_display_mode *mode)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(disp_pclk_khz); i++) {
		unsigned int pclk = disp_pclk_khz[i];

		if (mode->hdisplay != disp_hdisplay[i])
			continue;
		if (mode->clock >= pclk - pclk / 100 &&
		    mode->clock <= pclk + pclk / 100)
			return i;
	}
	return -EINVAL;
}

static void sp7021_tgen_init(struct sp7021_drm *d, unsigned int timing)
{
	sp7021_hw_write(d, TGEN_CONFIG, 0x0007);	/* latch on */
	sp7021_hw_write(d, TGEN_SOURCE_SEL, 0);
	sp7021_hw_write(d, TGEN_USER_INT1, 0);		/* vsync at line 0 */
	sp7021_hw_write(d, TGEN_USER_INT2, 400);

	sp7021_hw_write(d, TGEN_DTG_CONFIG, tgen_dtg_cfg[timing]);

	/* Pipeline delay for OSD0 on layer L6 and PTG on BG */
	sp7021_hw_write(d, TGEN_DTG_ADJ3, 6);
	sp7021_hw_write(d, TGEN_DTG_ADJ4, 0x1000);

	/* Reset TGEN to apply new timing */
	sp7021_hw_write(d, TGEN_RESET, 1);
	sp7021_hw_write(d, TGEN_RESET, 0);
}

static void sp7021_dmix_init(struct sp7021_drm *d)
{
	sp7021_hw_write(d, DMIX_PIX_EN_SEL, 0x0006);
	/*
	 * config0: BG=PTG (bits[6:4]=7), L1=VPP0 (bits[10:8]=0)
	 * config1: L6=OSD0 opacity (bits[11:10]=2=opacity), L1=opacity (bits[1:0]=2)
	 *          0x8156 from vendor: BG mode | L6=opacity | L1=opacity
	 * config2: L6 input = OSD0 (bits[15:12] = 3 = OSD0)
	 */
	sp7021_hw_write(d, DMIX_CONFIG0, 0x0070);
	sp7021_hw_write(d, DMIX_CONFIG1, 0x8156);
	sp7021_hw_write(d, DMIX_CONFIG2, 0x3000);

	/* PTG: solid background color black (Y=16, Cb=128, Cr=128 for limited range) */
	sp7021_hw_write(d, DMIX_PTG_CONFIG, 0x2000);
	sp7021_hw_write(d, DMIX_PTG_CFG4, (1 << 8) | 0x10);
	sp7021_hw_write(d, DMIX_PTG_CFG5, (1 << 8) | 0x80);
	sp7021_hw_write(d, DMIX_PTG_CFG6, (1 << 8) | 0x80);
}

static void sp7021_gpost_bypass(struct sp7021_drm *d)
{
	sp7021_hw_write(d, GPOST_CONFIG, 0);
	sp7021_hw_write(d, GPOST_MASTER_EN, 0);
	sp7021_hw_write(d, GPOST_BG1, 0x8010);
	sp7021_hw_write(d, GPOST_BG2, 0x0080);
	sp7021_hw_write(d, GPOST_CONTRAST, 0);
}

static void sp7021_osd_setup(struct sp7021_drm *d,
			     dma_addr_t fb_addr, unsigned int w, unsigned int h)
{
	u32 *hdr = d->osd_hdr;

	/* Build OSD header in big-endian byte order for display hardware */
	hdr[0] = OSD_HDR_W0;
	hdr[1] = cpu_to_be32((h << 16) | w);
	hdr[2] = 0;
	hdr[3] = 0;
	hdr[4] = 0;
	hdr[5] = OSD_HDR_W5(w);
	hdr[6] = OSD_HDR_W6_END;
	hdr[7] = cpu_to_be32((u32)fb_addr);

	sp7021_hw_write(d, OSD_CTRL,
			OSD_CTRL_RGB_MODE | OSD_CTRL_CLUT_ARGB |
			OSD_CTRL_LATCH_EN | OSD_CTRL_A32B32_EN |
			OSD_CTRL_FIFO_DEPTH);
	sp7021_hw_write(d, OSD_BASE_ADDR, (u32)d->osd_hdr_phys);
	sp7021_hw_write(d, OSD_HVLD_OFFSET, 0);
	sp7021_hw_write(d, OSD_HVLD_WIDTH, w);
	sp7021_hw_write(d, OSD_VVLD_OFFSET, 0);
	sp7021_hw_write(d, OSD_VVLD_HEIGHT, h);
	sp7021_hw_write(d, OSD_BIST_CTRL, 0);
	sp7021_hw_write(d, OSD_3D_H_OFFSET, 0);
	sp7021_hw_write(d, OSD_SRC_DECIM, 0);
	sp7021_hw_write(d, OSD_EN, 1);
}

/* -------------------------------------------------------------------------
 * IRQ — TGEN user_int1 fires at scan line 0 (vsync)
 * -------------------------------------------------------------------------
 */
static irqreturn_t sp7021_drm_irq(int irq, void *data)
{
	struct sp7021_drm *d = data;
	struct drm_crtc *crtc = &d->crtc;
	unsigned long flags;

	drm_crtc_handle_vblank(crtc);

	spin_lock_irqsave(&d->lock, flags);
	if (crtc->state && crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
		drm_crtc_vblank_put(crtc);
	}
	spin_unlock_irqrestore(&d->lock, flags);

	return IRQ_HANDLED;
}

/* -------------------------------------------------------------------------
 * CRTC
 * -------------------------------------------------------------------------
 */
static int sp7021_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct sp7021_drm *d = crtc_to_sp7021(crtc);

	enable_irq(d->vsync_irq);
	return 0;
}

static void sp7021_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct sp7021_drm *d = crtc_to_sp7021(crtc);

	disable_irq_nosync(d->vsync_irq);
}

static void sp7021_crtc_atomic_enable(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct sp7021_drm *d = crtc_to_sp7021(crtc);
	const struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	int timing, i, ret;

	timing = sp7021_timing_for_mode(mode);
	if (timing < 0)
		timing = 2;	/* fall back to 720p60 */
	d->timing = timing;

	for (i = 0; i < NUM_RSTS; i++)
		reset_control_deassert(d->rsts[i]);

	for (i = 0; i < NUM_CLKS; i++) {
		ret = clk_prepare_enable(d->clks[i]);
		if (ret)
			dev_err(d->dev, "failed to enable clock %s: %d\n",
				sp7021_clk_names[i], ret);
	}

	sp7021_tgen_init(d, timing);
	sp7021_dmix_init(d);
	sp7021_gpost_bypass(d);

	drm_crtc_vblank_on(crtc);
}

static void sp7021_crtc_atomic_disable(struct drm_crtc *crtc,
				       struct drm_atomic_state *state)
{
	struct sp7021_drm *d = crtc_to_sp7021(crtc);
	int i;

	drm_crtc_vblank_off(crtc);

	sp7021_hw_write(d, OSD_EN, 0);

	for (i = 0; i < NUM_CLKS; i++)
		clk_disable_unprepare(d->clks[i]);

	for (i = 0; i < NUM_RSTS; i++)
		reset_control_assert(d->rsts[i]);
}

static void sp7021_crtc_atomic_flush(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct sp7021_drm *d = crtc_to_sp7021(crtc);
	unsigned long flags;

	spin_lock_irqsave(&d->lock, flags);
	if (crtc->state->event) {
		if (drm_crtc_vblank_get(crtc) == 0) {
			/* event will be signalled in the IRQ handler */
		} else {
			drm_crtc_send_vblank_event(crtc, crtc->state->event);
			crtc->state->event = NULL;
		}
	}
	spin_unlock_irqrestore(&d->lock, flags);
}

static const struct drm_crtc_helper_funcs sp7021_crtc_helper_funcs = {
	.atomic_enable  = sp7021_crtc_atomic_enable,
	.atomic_disable = sp7021_crtc_atomic_disable,
	.atomic_flush   = sp7021_crtc_atomic_flush,
};

static const struct drm_crtc_funcs sp7021_crtc_funcs = {
	.reset                  = drm_atomic_helper_crtc_reset,
	.destroy                = drm_crtc_cleanup,
	.set_config             = drm_atomic_helper_set_config,
	.page_flip              = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank          = sp7021_crtc_enable_vblank,
	.disable_vblank         = sp7021_crtc_disable_vblank,
};

/* -------------------------------------------------------------------------
 * Primary plane (OSD0)
 * -------------------------------------------------------------------------
 */
static const u32 sp7021_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static void sp7021_plane_atomic_update(struct drm_plane *plane,
				       struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct sp7021_drm *d = plane_to_sp7021(plane);
	struct drm_framebuffer *fb = new_state->fb;
	dma_addr_t addr;

	if (!fb)
		return;

	addr = drm_fb_dma_get_gem_addr(fb, new_state, 0);

	/* Update only the framebuffer pointer — hardware latches on next vsync */
	d->osd_hdr[7] = cpu_to_be32((u32)addr);

	if (!plane->state || !plane->state->fb) {
		/* First update: program all OSD registers */
		sp7021_osd_setup(d, addr, fb->width, fb->height);
	}
}

static void sp7021_plane_atomic_disable(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	struct sp7021_drm *d = plane_to_sp7021(plane);

	sp7021_hw_write(d, OSD_EN, 0);
}

static const struct drm_plane_helper_funcs sp7021_plane_helper_funcs = {
	.atomic_update  = sp7021_plane_atomic_update,
	.atomic_disable = sp7021_plane_atomic_disable,
};

static const struct drm_plane_funcs sp7021_plane_funcs = {
	.update_plane           = drm_atomic_helper_update_plane,
	.disable_plane          = drm_atomic_helper_disable_plane,
	.destroy                = drm_plane_cleanup,
	.reset                  = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_plane_destroy_state,
};

/* -------------------------------------------------------------------------
 * Encoder (pass-through to HDMI TX bridge)
 * -------------------------------------------------------------------------
 */
static const struct drm_encoder_funcs sp7021_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

/* -------------------------------------------------------------------------
 * Mode config
 * -------------------------------------------------------------------------
 */
static const struct drm_mode_config_funcs sp7021_mode_config_funcs = {
	.fb_create      = drm_gem_fb_create,
	.atomic_check   = drm_atomic_helper_check,
	.atomic_commit  = drm_atomic_helper_commit,
};

/* -------------------------------------------------------------------------
 * Platform driver
 * -------------------------------------------------------------------------
 */
static int sp7021_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sp7021_drm *d;
	struct drm_device *drm;
	struct drm_bridge *bridge;
	struct drm_connector *connector;
	int i, ret;

	d = devm_drm_dev_alloc(dev, &sp7021_drm_driver,
			       struct sp7021_drm, drm);
	if (IS_ERR(d))
		return PTR_ERR(d);

	drm = &d->drm;
	d->dev = dev;
	spin_lock_init(&d->lock);
	platform_set_drvdata(pdev, d);

	d->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(d->base))
		return PTR_ERR(d->base);

	for (i = 0; i < NUM_CLKS; i++) {
		d->clks[i] = devm_clk_get(dev, sp7021_clk_names[i]);
		if (IS_ERR(d->clks[i]))
			return dev_err_probe(dev, PTR_ERR(d->clks[i]),
					     "failed to get clock %s\n",
					     sp7021_clk_names[i]);
	}

	for (i = 0; i < NUM_RSTS; i++) {
		d->rsts[i] = devm_reset_control_get_exclusive(dev, sp7021_rst_names[i]);
		if (IS_ERR(d->rsts[i]))
			return dev_err_probe(dev, PTR_ERR(d->rsts[i]),
					     "failed to get reset %s\n",
					     sp7021_rst_names[i]);
	}

	d->osd_hdr = dmam_alloc_coherent(dev, 128, &d->osd_hdr_phys, GFP_KERNEL | __GFP_ZERO);
	if (!d->osd_hdr)
		return -ENOMEM;

	d->vsync_irq = platform_get_irq(pdev, 0);
	if (d->vsync_irq < 0)
		return d->vsync_irq;

	ret = devm_request_irq(dev, d->vsync_irq, sp7021_drm_irq,
			       IRQF_NO_AUTOEN, dev_name(dev), d);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	/* Find the HDMI TX bridge via DT port graph */
	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge), "no HDMI bridge found\n");

	/* DRM mode config */
	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width  = 720;
	drm->mode_config.min_height = 480;
	drm->mode_config.max_width  = 1920;
	drm->mode_config.max_height = 1080;
	drm->mode_config.funcs = &sp7021_mode_config_funcs;
	drm->mode_config.preferred_depth = 32;
	drm->mode_config.prefer_shadow = 0;

	/* Primary plane */
	ret = drm_universal_plane_init(drm, &d->primary, 0,
				       &sp7021_plane_funcs,
				       sp7021_plane_formats,
				       ARRAY_SIZE(sp7021_plane_formats),
				       NULL,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;

	drm_plane_helper_add(&d->primary, &sp7021_plane_helper_funcs);

	/* CRTC */
	ret = drm_crtc_init_with_planes(drm, &d->crtc, &d->primary, NULL,
					&sp7021_crtc_funcs, NULL);
	if (ret)
		return ret;

	drm_crtc_helper_add(&d->crtc, &sp7021_crtc_helper_funcs);

	/* Encoder */
	ret = drm_encoder_init(drm, &d->encoder, &sp7021_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS, NULL);
	if (ret)
		return ret;

	d->encoder.possible_crtcs = drm_crtc_mask(&d->crtc);

	/* Attach HDMI TX bridge */
	ret = drm_bridge_attach(&d->encoder, bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return dev_err_probe(dev, ret, "failed to attach bridge\n");

	/* Create connector from bridge */
	connector = drm_bridge_connector_init(drm, &d->encoder);
	if (IS_ERR(connector))
		return dev_err_probe(dev, PTR_ERR(connector),
				     "failed to create bridge connector\n");

	drm_connector_attach_encoder(connector, &d->encoder);

	drm_mode_config_reset(drm);

	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;

	drm_kms_helper_poll_init(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	drm_kms_helper_poll_enable(drm);

	dev_info(dev, "SP7021 display registered\n");
	return 0;
}

static void sp7021_drm_remove(struct platform_device *pdev)
{
	struct sp7021_drm *d = platform_get_drvdata(pdev);

	drm_kms_helper_poll_fini(&d->drm);
	drm_dev_unregister(&d->drm);
	drm_atomic_helper_shutdown(&d->drm);
}

static void sp7021_drm_shutdown(struct platform_device *pdev)
{
	struct sp7021_drm *d = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&d->drm);
}

static const struct of_device_id sp7021_drm_of_match[] = {
	{ .compatible = "sunplus,sp7021-display" },
	{ }
};
MODULE_DEVICE_TABLE(of, sp7021_drm_of_match);

static struct platform_driver sp7021_drm_platform_driver = {
	.probe    = sp7021_drm_probe,
	.remove   = sp7021_drm_remove,
	.shutdown = sp7021_drm_shutdown,
	.driver   = {
		.name           = "sp7021-drm",
		.of_match_table = sp7021_drm_of_match,
	},
};
module_platform_driver(sp7021_drm_platform_driver);

MODULE_AUTHOR("Sunplus Technology");
MODULE_DESCRIPTION("Sunplus SP7021 DRM display driver");
MODULE_LICENSE("GPL");

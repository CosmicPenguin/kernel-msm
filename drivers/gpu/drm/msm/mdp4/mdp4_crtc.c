/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kfifo.h>

#include "mdp4_kms.h"

#include <drm/drm_mode.h>
#include "drm_crtc.h"
#include "drm_crtc_helper.h"


struct mdp4_crtc {
	struct drm_crtc base;
	char name[8];
	struct drm_plane *plane;
	int id;
	int ovlp;
	enum mdp4_dma dma;
	bool enabled;

	/* if there is a pending flip, these will be non-null: */
	struct drm_pending_vblank_event *event;
	struct drm_framebuffer *old_fb;

	/* we can't free fb from irq, so deferred to worker: */
	DECLARE_KFIFO_PTR(unref_fifo, struct drm_framebuffer *);
	struct work_struct work;

	struct mdp4_irq irq;
};
#define to_mdp4_crtc(x) container_of(x, struct mdp4_crtc, base)

static struct mdp4_kms *get_kms(struct drm_crtc *crtc)
{
	struct msm_drm_private *priv = crtc->dev->dev_private;
	return to_mdp4_kms(priv->kms);
}

static void update_fb(struct mdp4_crtc *mdp4_crtc,
		struct drm_framebuffer *new_fb, struct drm_framebuffer *old_fb)
{
	if (WARN_ON(mdp4_crtc->old_fb))
		drm_framebuffer_unreference(mdp4_crtc->old_fb);

	/* keep track of the previously scanned out buffer to unref: */
	mdp4_crtc->old_fb = old_fb;

	/* grab reference to incoming scanout fb: */
	drm_framebuffer_reference(new_fb);
	mdp4_crtc->base.fb = new_fb;
}

static void unref_worker(struct work_struct *work)
{
	struct mdp4_crtc *mdp4_crtc = container_of(work, struct mdp4_crtc, work);
	struct drm_device *dev = mdp4_crtc->base.dev;
	struct drm_framebuffer *fb;

	mutex_lock(&dev->mode_config.mutex);
	while (kfifo_get(&mdp4_crtc->unref_fifo, &fb))
		drm_framebuffer_unreference(fb);
	mutex_unlock(&dev->mode_config.mutex);
}

static void mdp4_crtc_destroy(struct drm_crtc *crtc)
{
	struct mdp4_crtc *mdp4_crtc = to_mdp4_crtc(crtc);

	mdp4_crtc->plane->funcs->destroy(mdp4_crtc->plane);
	drm_crtc_cleanup(crtc);

	WARN_ON(!kfifo_is_empty(&mdp4_crtc->unref_fifo));
	kfifo_free(&mdp4_crtc->unref_fifo);

	kfree(mdp4_crtc);
}

static void mdp4_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct mdp4_crtc *mdp4_crtc = to_mdp4_crtc(crtc);
	struct mdp4_kms *mdp4_kms = get_kms(crtc);
	bool enabled = (mode == DRM_MODE_DPMS_ON);

	DBG("%s: mode=%d", mdp4_crtc->name, mode);

	if (enabled == mdp4_crtc->enabled)
		return;

	if (enabled)
		mdp4_irq_register(mdp4_kms, &mdp4_crtc->irq);
	else
		mdp4_irq_unregister(mdp4_kms, &mdp4_crtc->irq);

	mdp4_crtc->enabled = enabled;
}

static bool mdp4_crtc_mode_fixup(struct drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void blend_setup(struct drm_crtc *crtc)
{
	struct mdp4_crtc *mdp4_crtc = to_mdp4_crtc(crtc);
	struct mdp4_kms *mdp4_kms = get_kms(crtc);
	int i, ovlp = mdp4_crtc->ovlp;

	/*
	 * This probably would also need to be triggered by any attached
	 * plane when it changes.. for now since we are only using a single
	 * private plane, the configuration is hard-coded:
	 */

	mdp4_write(mdp4_kms, REG_MDP4_OVLP_TRANSP_LOW0(ovlp), 0);
	mdp4_write(mdp4_kms, REG_MDP4_OVLP_TRANSP_LOW1(ovlp), 0);
	mdp4_write(mdp4_kms, REG_MDP4_OVLP_TRANSP_HIGH0(ovlp), 0);
	mdp4_write(mdp4_kms, REG_MDP4_OVLP_TRANSP_HIGH1(ovlp), 0);

	for (i = 0; i < 4; i++) {
		mdp4_write(mdp4_kms, REG_MDP4_OVLP_STAGE_FG_ALPHA(ovlp, i), 0);
		mdp4_write(mdp4_kms, REG_MDP4_OVLP_STAGE_BG_ALPHA(ovlp, i), 0);
		mdp4_write(mdp4_kms, REG_MDP4_OVLP_STAGE_OP(ovlp, i),
				MDP4_OVLP_STAGE_OP_FG_ALPHA(FG_CONST) |
				MDP4_OVLP_STAGE_OP_BG_ALPHA(FG_CONST));
		mdp4_write(mdp4_kms, REG_MDP4_OVLP_STAGE_CO3(ovlp, i), 0);
		mdp4_write(mdp4_kms, REG_MDP4_OVLP_STAGE_TRANSP_LOW0(ovlp, i), 0);
		mdp4_write(mdp4_kms, REG_MDP4_OVLP_STAGE_TRANSP_LOW1(ovlp, i), 0);
		mdp4_write(mdp4_kms, REG_MDP4_OVLP_STAGE_TRANSP_HIGH0(ovlp, i), 0);
		mdp4_write(mdp4_kms, REG_MDP4_OVLP_STAGE_TRANSP_HIGH1(ovlp, i), 0);
	}

	/* XXX hard code for pipe2 (RGB1).. we need to figure this out from
	 * what plane(s) are attached..
	 */
	mdp4_write(mdp4_kms, REG_MDP4_LAYERMIXER_IN_CFG,
			MDP4_LAYERMIXER_IN_CFG_PIPE2(STAGE_BASE) |
			MDP4_LAYERMIXER_IN_CFG_PIPE2_MIXER1);
}

static int mdp4_crtc_mode_set(struct drm_crtc *crtc,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode,
		int x, int y,
		struct drm_framebuffer *old_fb)
{
	struct mdp4_crtc *mdp4_crtc = to_mdp4_crtc(crtc);
	struct mdp4_kms *mdp4_kms = get_kms(crtc);
	enum mdp4_dma dma = mdp4_crtc->dma;
	int ret, ovlp = mdp4_crtc->ovlp;

	mode = adjusted_mode;

	DBG("%s: set mode: %d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x",
			mdp4_crtc->name, mode->base.id, mode->name,
			mode->vrefresh, mode->clock,
			mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal,
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal,
			mode->type, mode->flags);

	mdp4_write(mdp4_kms, REG_MDP4_DMA_SRC_SIZE(dma),
			MDP4_DMA_SRC_SIZE_WIDTH(mode->hdisplay) |
			MDP4_DMA_SRC_SIZE_HEIGHT(mode->vdisplay));

	/* take data from pipe: */
	mdp4_write(mdp4_kms, REG_MDP4_DMA_SRC_BASE(dma), 0);
	mdp4_write(mdp4_kms, REG_MDP4_DMA_SRC_STRIDE(dma),
			crtc->fb->pitches[0]);
	mdp4_write(mdp4_kms, REG_MDP4_DMA_DST_SIZE(dma),
			MDP4_DMA_DST_SIZE_WIDTH(0) |
			MDP4_DMA_DST_SIZE_HEIGHT(0));

	mdp4_write(mdp4_kms, REG_MDP4_OVLP_BASE(ovlp), 0);
	mdp4_write(mdp4_kms, REG_MDP4_OVLP_SIZE(ovlp),
			MDP4_OVLP_SIZE_WIDTH(mode->hdisplay) |
			MDP4_OVLP_SIZE_HEIGHT(mode->vdisplay));
	mdp4_write(mdp4_kms, REG_MDP4_OVLP_STRIDE(ovlp),
			crtc->fb->pitches[0]);

	mdp4_write(mdp4_kms, REG_MDP4_OVLP_CFG(ovlp), 1);

	update_fb(mdp4_crtc, crtc->fb, old_fb);

	ret = mdp4_plane_mode_set(mdp4_crtc->plane, crtc, crtc->fb,
			0, 0, mode->hdisplay, mode->vdisplay,
			x << 16, y << 16,
			mode->hdisplay << 16, mode->vdisplay << 16);
	if (ret) {
		dev_err(crtc->dev->dev, "%s: failed to set mode on plane: %d\n",
				mdp4_crtc->name, ret);
		return ret;
	}

	blend_setup(crtc);

	/*
	 * I believe this is the *output* format to the encoder.. so possibly
	 * we need to be finding this out from the encoder:
	 */
	mdp4_write(mdp4_kms, REG_MDP4_DMA_CONFIG(dma),
			MDP4_DMA_CONFIG_R_BPC(DBPC8) |
			MDP4_DMA_CONFIG_G_BPC(DBPC8) |
			MDP4_DMA_CONFIG_B_BPC(DBPC8) |
			MDP4_DMA_CONFIG_PACK(0x21));

	if (dma == DMA_E) {
		mdp4_write(mdp4_kms, REG_MDP4_DMA_E_QUANT(0), 0x00ff0000);
		mdp4_write(mdp4_kms, REG_MDP4_DMA_E_QUANT(1), 0x00ff0000);
		mdp4_write(mdp4_kms, REG_MDP4_DMA_E_QUANT(2), 0x00ff0000);
	}

	return 0;
}

static void mdp4_crtc_flush(struct drm_crtc *crtc)
{
	struct mdp4_crtc *mdp4_crtc = to_mdp4_crtc(crtc);
	struct mdp4_kms *mdp4_kms = get_kms(crtc);
	uint32_t flush = 0;

	flush |= pipe2flush(mdp4_plane_pipe(mdp4_crtc->plane));
	flush |= ovlp2flush(mdp4_crtc->ovlp);

	DBG("%s: flush=%08x", mdp4_crtc->name, flush);

	mdp4_write(mdp4_kms, REG_MDP4_OVERLAY_FLUSH, flush);
}

static void mdp4_crtc_prepare(struct drm_crtc *crtc)
{
	struct mdp4_crtc *mdp4_crtc = to_mdp4_crtc(crtc);
	DBG("%s", mdp4_crtc->name);
	mdp4_crtc_dpms(crtc, DRM_MODE_DPMS_OFF);
}

static void mdp4_crtc_commit(struct drm_crtc *crtc)
{
	mdp4_crtc_dpms(crtc, DRM_MODE_DPMS_ON);
	mdp4_crtc_flush(crtc);
}

static int mdp4_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
		struct drm_framebuffer *old_fb)
{
	struct mdp4_crtc *mdp4_crtc = to_mdp4_crtc(crtc);
	struct drm_plane *plane = mdp4_crtc->plane;
	struct drm_display_mode *mode = &crtc->mode;

	update_fb(mdp4_crtc, crtc->fb, old_fb);

	return mdp4_plane_mode_set(plane, crtc, crtc->fb,
			0, 0, mode->hdisplay, mode->vdisplay,
			x << 16, y << 16,
			mode->hdisplay << 16, mode->vdisplay << 16);
}

static void mdp4_crtc_load_lut(struct drm_crtc *crtc)
{
}

static int mdp4_crtc_page_flip(struct drm_crtc *crtc,
		struct drm_framebuffer *new_fb,
		struct drm_pending_vblank_event *event)
{
	struct mdp4_crtc *mdp4_crtc = to_mdp4_crtc(crtc);
	struct drm_device *dev = crtc->dev;

	if (mdp4_crtc->old_fb) {
		dev_err(dev->dev, "already pending page flip!\n");
		return -EBUSY;
	}

	mdp4_crtc->event = event;

	update_fb(mdp4_crtc, new_fb, crtc->fb);

	mdp4_plane_set_scanout(mdp4_crtc->plane, new_fb);
	mdp4_crtc_flush(crtc);

	return 0;
}

static int mdp4_crtc_set_property(struct drm_crtc *crtc,
		struct drm_property *property, uint64_t val)
{
	// XXX
	return -EINVAL;
}

static const struct drm_crtc_funcs mdp4_crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.destroy = mdp4_crtc_destroy,
	.page_flip = mdp4_crtc_page_flip,
	.set_property = mdp4_crtc_set_property,
};

static const struct drm_crtc_helper_funcs mdp4_crtc_helper_funcs = {
	.dpms = mdp4_crtc_dpms,
	.mode_fixup = mdp4_crtc_mode_fixup,
	.mode_set = mdp4_crtc_mode_set,
	.prepare = mdp4_crtc_prepare,
	.commit = mdp4_crtc_commit,
	.mode_set_base = mdp4_crtc_mode_set_base,
	.load_lut = mdp4_crtc_load_lut,
};

static void mdp4_crtc_irq(struct mdp4_irq *irq, uint32_t irqstatus)
{
	struct mdp4_crtc *mdp4_crtc = container_of(irq, struct mdp4_crtc, irq);
	struct drm_device *dev = mdp4_crtc->base.dev;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	if (mdp4_crtc->old_fb) {
		if (kfifo_put(&mdp4_crtc->unref_fifo,
				(const struct drm_framebuffer **)&mdp4_crtc->old_fb)) {
			struct msm_drm_private *priv = dev->dev_private;
			queue_work(priv->wq, &mdp4_crtc->work);
		} else {
			dev_err(dev->dev, "unref fifo full!\n");
			drm_framebuffer_unreference(mdp4_crtc->old_fb);
		}
		mdp4_crtc->old_fb = NULL;
	}

	spin_lock_irqsave(&dev->event_lock, flags);
	event = mdp4_crtc->event;
	mdp4_crtc->event = NULL;
	if (event)
		drm_send_vblank_event(dev, mdp4_crtc->id, event);
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

uint32_t mdp4_crtc_vblank(struct drm_crtc *crtc)
{
	struct mdp4_crtc *mdp4_crtc = to_mdp4_crtc(crtc);
	return mdp4_crtc->irq.irqmask;
}

/* set interface for routing crtc->encoder: */
void mdp4_crtc_set_intf(struct drm_crtc *crtc, enum mdp4_intf intf)
{
	struct mdp4_crtc *mdp4_crtc = to_mdp4_crtc(crtc);
	struct mdp4_kms *mdp4_kms = get_kms(crtc);
	uint32_t intf_sel;

	intf_sel = mdp4_read(mdp4_kms, REG_MDP4_DISP_INTF_SEL);

	switch (mdp4_crtc->dma) {
	case DMA_P:
		intf_sel &= ~MDP4_DISP_INTF_SEL_PRIM__MASK;
		intf_sel |= MDP4_DISP_INTF_SEL_PRIM(intf);
		break;
	case DMA_S:
		intf_sel &= ~MDP4_DISP_INTF_SEL_SEC__MASK;
		intf_sel |= MDP4_DISP_INTF_SEL_SEC(intf);
		break;
	case DMA_E:
		intf_sel &= ~MDP4_DISP_INTF_SEL_EXT__MASK;
		intf_sel |= MDP4_DISP_INTF_SEL_EXT(intf);
		break;
	}

	DBG("%s: intf_sel=%08x", mdp4_crtc->name, intf_sel);

	mdp4_write(mdp4_kms, REG_MDP4_DISP_INTF_SEL, intf_sel);
}

static const char *dma_names[] = {
		"DMA_P", "DMA_S", "DMA_E",
};

/* initialize crtc */
struct drm_crtc *mdp4_crtc_init(struct drm_device *dev,
		struct drm_plane *plane, int id, int ovlp_id,
		enum mdp4_dma dma_id)
{
	struct drm_crtc *crtc = NULL;
	struct mdp4_crtc *mdp4_crtc;
	int ret;

	mdp4_crtc = kzalloc(sizeof(*mdp4_crtc), GFP_KERNEL);
	if (!mdp4_crtc) {
		ret = -ENOMEM;
		goto fail;
	}

	crtc = &mdp4_crtc->base;

	mdp4_crtc->plane = plane;
	mdp4_crtc->plane->crtc = crtc;

	mdp4_crtc->ovlp = ovlp_id;
	mdp4_crtc->dma = dma_id;

	mdp4_crtc->irq.irqmask = dma2irq(mdp4_crtc->dma);
	mdp4_crtc->irq.irq = mdp4_crtc_irq;

	snprintf(mdp4_crtc->name, sizeof(mdp4_crtc->name), "%s:%d",
			dma_names[dma_id], ovlp_id);

	ret = kfifo_alloc(&mdp4_crtc->unref_fifo, 16, GFP_KERNEL);
	if (ret) {
		dev_err(dev->dev, "could not allocate unref FIFO\n");
		goto fail;
	}

	INIT_WORK(&mdp4_crtc->work, unref_worker);

	drm_crtc_init(dev, crtc, &mdp4_crtc_funcs);
	drm_crtc_helper_add(crtc, &mdp4_crtc_helper_funcs);

	mdp4_plane_install_properties(mdp4_crtc->plane, &crtc->base);

	return crtc;

fail:
	if (crtc)
		mdp4_crtc_destroy(crtc);

	return ERR_PTR(ret);
}

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

#ifndef __MDP4_KMS_H__
#define __MDP4_KMS_H__

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include "msm_drv.h"
#include "mdp4.xml.h"


/* For transiently registering for different MDP4 irqs that various parts
 * of the KMS code need during setup/configuration.  We these are not
 * necessarily the same as what drm_vblank_get/put() are requesting, and
 * the hysteresis in drm_vblank_put() is not necessarily desirable for
 * internal housekeeping related irq usage.
 */
struct mdp4_irq {
	struct list_head node;
	uint32_t irqmask;
	bool registered;
	void (*irq)(struct mdp4_irq *irq, uint32_t irqstatus);
};

struct mdp4_kms {
	struct msm_kms base;

	struct drm_device *dev;

	int rev;

	/* mapper-id used to request GEM buffer mapped for scanout: */
	int id;

	void __iomem *mmio;

	struct regulator *dsi_pll_vdda;
	struct regulator *dsi_pll_vddio;
	struct regulator *vdd;

	struct clk *clk;
	struct clk *pclk;
	struct clk *lut_clk;

	/* irq handling: */
	struct list_head irq_list;    /* list of mdp4_irq */
	uint32_t vblank_mask;         /* irq bits set for userspace vblank */
	struct mdp4_irq error_handler;
};
#define to_mdp4_kms(x) container_of(x, struct mdp4_kms, base)

static inline void mdp4_write(struct mdp4_kms *mdp4_kms, u32 reg, u32 data)
{
	msm_writel(data, mdp4_kms->mmio + reg);
}

static inline u32 mdp4_read(struct mdp4_kms *mdp4_kms, u32 reg)
{
	return msm_readl(mdp4_kms->mmio + reg);
}

static inline uint32_t pipe2flush(enum mpd4_pipe pipe)
{
	switch (pipe) {
	case VG1:      return MDP4_OVERLAY_FLUSH_VG1;
	case VG2:      return MDP4_OVERLAY_FLUSH_VG2;
	case RGB1:     return MDP4_OVERLAY_FLUSH_RGB1;
	case RGB2:     return MDP4_OVERLAY_FLUSH_RGB1;
	default:       return 0;
	}
}

static inline uint32_t ovlp2flush(int ovlp)
{
	switch (ovlp) {
	case 0:        return MDP4_OVERLAY_FLUSH_OVLP0;
	case 1:        return MDP4_OVERLAY_FLUSH_OVLP1;
	default:       return 0;
	}
}

static inline uint32_t dma2irq(enum mdp4_dma dma)
{
	switch (dma) {
	case DMA_P:    return MDP4_IRQ_DMA_P_DONE | MDP4_IRQ_PRIMARY_VSYNC;
	case DMA_S:    return MDP4_IRQ_DMA_S_DONE;
	case DMA_E:    return MDP4_IRQ_DMA_E_DONE | MDP4_IRQ_EXTERNAL_VSYNC;
	default:       return 0;
	}
}

void mdp4_irq_preinstall(struct msm_kms *kms);
int mdp4_irq_postinstall(struct msm_kms *kms);
void mdp4_irq_uninstall(struct msm_kms *kms);
irqreturn_t mdp4_irq(struct msm_kms *kms);
void mdp4_irq_wait(struct mdp4_kms *mdp4_kms, uint32_t irqmask);
void mdp4_irq_register(struct mdp4_kms *mdp4_kms, struct mdp4_irq *irq);
void mdp4_irq_unregister(struct mdp4_kms *mdp4_kms, struct mdp4_irq *irq);
int mdp4_enable_vblank(struct msm_kms *kms, struct drm_crtc *crtc);
void mdp4_disable_vblank(struct msm_kms *kms, struct drm_crtc *crtc);

void mdp4_plane_install_properties(struct drm_plane *plane,
		struct drm_mode_object *obj);
void mdp4_plane_set_scanout(struct drm_plane *plane,
		struct drm_framebuffer *fb);
int mdp4_plane_mode_set(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		int crtc_x, int crtc_y,
		unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y,
		uint32_t src_w, uint32_t src_h);
enum mpd4_pipe mdp4_plane_pipe(struct drm_plane *plane);
struct drm_plane *mdp4_plane_init(struct drm_device *dev,
		enum mpd4_pipe pipe_id, bool private_plane);

uint32_t mdp4_crtc_vblank(struct drm_crtc *crtc);
void mdp4_crtc_set_intf(struct drm_crtc *crtc, enum mdp4_intf intf);
struct drm_crtc *mdp4_crtc_init(struct drm_device *dev,
		struct drm_plane *plane, int id, int ovlp_id,
		enum mdp4_dma dma_id);

struct drm_encoder *mdp4_lcdc_encoder_init(struct drm_device *dev);
struct drm_encoder *mdp4_dtv_encoder_init(struct drm_device *dev);
struct drm_encoder *mdp4_dsi_encoder_init(struct drm_device *dev);

#ifdef CONFIG_MSM_BUS_SCALING
static inline int match_dev_name(struct device *dev, void *data)
{
	return !strcmp(dev_name(dev), data);
}
/* bus scaling data is associated with extra pointless platform devices,
 * "dtv", etc.. this is a bit of a hack, but we need a way for encoders
 * to find their pdata to make the bus-scaling stuff work.
 */
static inline void *mdp4_find_pdata(const char *devname)
{
	struct device *dev;
	dev = bus_find_device(&platform_bus_type, NULL,
			(void *)devname, match_dev_name);
	return dev ? dev->platform_data : NULL;
}
#endif

#endif /* __MDP4_KMS_H__ */

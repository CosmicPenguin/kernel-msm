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


#include "msm_drv.h"
#include "mdp4_kms.h"

#include <mach/iommu.h>
#include <mach/iommu_domains.h>


static int mdp4_hw_init(struct msm_kms *kms)
{
	struct mdp4_kms *mdp4_kms = to_mdp4_kms(kms);
	struct drm_device *dev = mdp4_kms->dev;
	uint32_t version, major, minor, dmap_cfg, vg_cfg;
	unsigned long clk;
	int ret = 0;

	pm_runtime_get_sync(dev->dev);

	version = mdp4_read(mdp4_kms, REG_MDP4_VERSION);

	major = FIELD(version, MDP4_VERSION_MAJOR);
	minor = FIELD(version, MDP4_VERSION_MINOR);

	DBG("found MDP version v%d.%d", major, minor);

	if (major != 4) {
		dev_err(dev->dev, "unexpected MDP version: v%d.%d\n",
				major, minor);
		ret = -ENXIO;
		goto out;
	}

	mdp4_kms->rev = minor;

	if (mdp4_kms->dsi_pll_vdda) {
		if ((mdp4_kms->rev == 2) || (mdp4_kms->rev == 4)) {
			ret = regulator_set_voltage(mdp4_kms->dsi_pll_vdda,
					1200000, 1200000);
			if (ret) {
				dev_err(dev->dev,
					"failed to set dsi_pll_vdda voltage: %d\n", ret);
				goto out;
			}
		}
	}

	if (mdp4_kms->dsi_pll_vddio) {
		if (mdp4_kms->rev == 2) {
			ret = regulator_set_voltage(mdp4_kms->dsi_pll_vddio,
					1800000, 1800000);
			if (ret) {
				dev_err(dev->dev,
					"failed to set dsi_pll_vddio voltage: %d\n", ret);
				goto out;
			}
		}
	}

	if (mdp4_kms->rev > 1) {
		mdp4_write(mdp4_kms, REG_MDP4_CS_CONTROLLER0, 0x0707ffff);
		mdp4_write(mdp4_kms, REG_MDP4_CS_CONTROLLER1, 0x03073f3f);
	}

	/* make sure things are off: */
	mdp4_write(mdp4_kms, REG_MDP4_DTV_ENABLE, 0);
	mdp4_write(mdp4_kms, REG_MDP4_LCDC_ENABLE, 0);
	mdp4_write(mdp4_kms, REG_MDP4_DSI_ENABLE, 0);

	mdp4_write(mdp4_kms, REG_MDP4_PORTMAP_MODE, 0x3);

	/* max read pending cmd config, 3 pending requests: */
	mdp4_write(mdp4_kms, REG_MDP4_READ_CNFG, 0x02222);

	clk = clk_get_rate(mdp4_kms->clk);

	if ((mdp4_kms->rev >= 1) || (clk >= 90000000)) {
		dmap_cfg = 0x47;     /* 16 bytes-burst x 8 req */
		vg_cfg = 0x47;       /* 16 bytes-burs x 8 req */
	} else {
		dmap_cfg = 0x27;     /* 8 bytes-burst x 8 req */
		vg_cfg = 0x43;       /* 16 bytes-burst x 4 req */
	}

	DBG("fetch config: dmap=%02x, vg=%02x", dmap_cfg, vg_cfg);

	mdp4_write(mdp4_kms, REG_MDP4_DMA_FETCH_CONFIG(DMA_P), dmap_cfg);
	mdp4_write(mdp4_kms, REG_MDP4_DMA_FETCH_CONFIG(DMA_E), dmap_cfg);

	mdp4_write(mdp4_kms, REG_MDP4_PIPE_FETCH_CONFIG(VG1), vg_cfg);
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_FETCH_CONFIG(VG2), vg_cfg);
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_FETCH_CONFIG(RGB1), vg_cfg);
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_FETCH_CONFIG(RGB2), vg_cfg);

	if (mdp4_kms->rev >= 2)
		mdp4_write(mdp4_kms, REG_MDP4_LAYERMIXER_IN_CFG_UPDATE_METHOD, 1);

	/* disable CSC matrix / YUV by default: */
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_OP_MODE(VG1), 0);
	mdp4_write(mdp4_kms, REG_MDP4_PIPE_OP_MODE(VG2), 0);
	mdp4_write(mdp4_kms, REG_MDP4_DMA_P_OP_MODE, 0);
	mdp4_write(mdp4_kms, REG_MDP4_DMA_S_OP_MODE, 0);
	mdp4_write(mdp4_kms, REG_MDP4_OVLP_CSC_CONFIG(1), 0);
	mdp4_write(mdp4_kms, REG_MDP4_OVLP_CSC_CONFIG(2), 0);

	if (mdp4_kms->rev > 1)
		mdp4_write(mdp4_kms, REG_MDP4_RESET_STATUS, 1);

out:
	pm_runtime_put_sync(dev->dev);

	return ret;
}

static int mdp4_pm_suspend(struct msm_kms *kms)
{
	struct mdp4_kms *mdp4_kms = to_mdp4_kms(kms);

	DBG("");

	clk_disable_unprepare(mdp4_kms->clk);
	if (mdp4_kms->pclk)
		clk_disable_unprepare(mdp4_kms->pclk);
	clk_disable_unprepare(mdp4_kms->lut_clk);

	return 0;
}

static int mdp4_pm_resume(struct msm_kms *kms)
{
	struct mdp4_kms *mdp4_kms = to_mdp4_kms(kms);

	DBG("");

	clk_prepare_enable(mdp4_kms->clk);
	if (mdp4_kms->pclk)
		clk_prepare_enable(mdp4_kms->pclk);
	clk_prepare_enable(mdp4_kms->lut_clk);

	return 0;
}

static void mdp4_preclose(struct msm_kms *kms, struct drm_file *file)
{
	/* cancel or wait for pending page flips.. */
	DBG(""); // XXX
}

static void mdp4_destroy(struct msm_kms *kms)
{
	struct mdp4_kms *mdp4_kms = to_mdp4_kms(kms);
	kfree(mdp4_kms);
}

static const struct msm_kms_funcs kms_funcs = {
		.hw_init         = mdp4_hw_init,
		.pm_suspend      = mdp4_pm_suspend,
		.pm_resume       = mdp4_pm_resume,
		.preclose        = mdp4_preclose,
		.irq_preinstall  = mdp4_irq_preinstall,
		.irq_postinstall = mdp4_irq_postinstall,
		.irq_uninstall   = mdp4_irq_uninstall,
		.irq             = mdp4_irq,
		.enable_vblank   = mdp4_enable_vblank,
		.disable_vblank  = mdp4_disable_vblank,
		.destroy         = mdp4_destroy,
};

static int modeset_init(struct mdp4_kms *mdp4_kms)
{
	struct drm_device *dev = mdp4_kms->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct drm_plane *plane;
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	int ret;

	/*
	 *  NOTE: this is a bit simplistic until we add support
	 * for more than just RGB1->DMA_E->DTV->HDMI
	 */

	/* the CRTCs get constructed with a private plane: */
	plane = mdp4_plane_init(dev, RGB1, true);
	if (IS_ERR(plane)) {
		dev_err(dev->dev, "failed to construct plane for RGB1\n");
		ret = PTR_ERR(plane);
		goto fail;
	}

	crtc  = mdp4_crtc_init(dev, plane, priv->num_crtcs, 1, DMA_E);
	if (IS_ERR(crtc)) {
		dev_err(dev->dev, "failed to construct crtc for DMA_E\n");
		ret = PTR_ERR(crtc);
		goto fail;
	}
	priv->crtcs[priv->num_crtcs++] = crtc;

	encoder = mdp4_dtv_encoder_init(dev);
	if (IS_ERR(encoder)) {
		dev_err(dev->dev, "failed to construct DTV encoder\n");
		ret = PTR_ERR(encoder);
		goto fail;
	}
	encoder->possible_crtcs = 0x1;     /* DTV can be hooked to DMA_E */
	priv->encoders[priv->num_encoders++] = encoder;

	connector = hdmi_connector_init(dev, encoder);
	if (IS_ERR(connector)) {
		dev_err(dev->dev, "failed to construct HDMI connector\n");
		ret = PTR_ERR(connector);
		goto fail;
	}
	priv->connectors[priv->num_connectors++] = connector;

	return 0;

fail:
	return ret;
}

static const char *iommu_ports[] = {
		"mdp_port0_cb0", "mdp_port1_cb0",
};

struct msm_kms *mdp4_kms_init(struct drm_device *dev)
{
	struct platform_device *pdev = dev->platformdev;
	struct resource *res;
	struct mdp4_kms *mdp4_kms;
	struct msm_kms *kms = NULL;
	struct iommu_domain *iommu;
	uint32_t max_clk;
	int i, ret;

	mdp4_kms = kzalloc(sizeof(*mdp4_kms), GFP_KERNEL);
	if (!mdp4_kms) {
		dev_err(dev->dev, "failed to allocate kms\n");
		ret = -ENOMEM;
		goto fail;
	}

	kms = &mdp4_kms->base;
	kms->funcs = &kms_funcs;

	mdp4_kms->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev->dev, "failed to get memory resource\n");
		ret = -EINVAL;
		goto fail;
	}

	mdp4_kms->mmio = msm_ioremap(&pdev->dev,
			res->start, resource_size(res), "MDP4");
	if (!mdp4_kms->mmio) {
		dev_err(dev->dev, "failed to ioremap\n");
		ret = -ENOMEM;
		goto fail;
	}

	mdp4_kms->dsi_pll_vdda = devm_regulator_get(&pdev->dev, "dsi_pll_vdda");
	if (IS_ERR(mdp4_kms->dsi_pll_vdda))
		mdp4_kms->dsi_pll_vdda = NULL;

	mdp4_kms->dsi_pll_vddio = devm_regulator_get(&pdev->dev, "dsi_pll_vddio");
	if (IS_ERR(mdp4_kms->dsi_pll_vddio))
		mdp4_kms->dsi_pll_vddio = NULL;

	mdp4_kms->vdd = devm_regulator_get(&pdev->dev, "vdd");
	if (IS_ERR(mdp4_kms->vdd))
		mdp4_kms->vdd = NULL;

	if (mdp4_kms->vdd)
		regulator_enable(mdp4_kms->vdd);

	mdp4_kms->clk = devm_clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(mdp4_kms->clk)) {
		dev_err(dev->dev, "failed to get core_clk\n");
		ret = PTR_ERR(mdp4_kms->clk);
		goto fail;
	}

	mdp4_kms->pclk = devm_clk_get(&pdev->dev, "iface_clk");
	if (IS_ERR(mdp4_kms->pclk))
		mdp4_kms->pclk = NULL;

	// XXX if (rev >= MDP_REV_42) { ???
	mdp4_kms->lut_clk = devm_clk_get(&pdev->dev, "lut_clk");
	if (IS_ERR(mdp4_kms->lut_clk)) {
		dev_err(dev->dev, "failed to get lut_clk\n");
		ret = PTR_ERR(mdp4_kms->lut_clk);
		goto fail;
	}

	if (cpu_is_apq8064())
		max_clk = 266667000;
	else
		max_clk = 200000000;

	clk_set_rate(mdp4_kms->clk, max_clk);
	clk_set_rate(mdp4_kms->lut_clk, max_clk);

	iommu = msm_get_iommu_domain(DISPLAY_READ_DOMAIN);
	if (!iommu) {
		dev_err(dev->dev, "failed to get mdp4 iommu\n");
		ret = -ENXIO;
		goto fail;
	}

	for (i = 0; i < ARRAY_SIZE(iommu_ports); i++) {
		struct device *ctx = msm_iommu_get_ctx(iommu_ports[i]);
		if (!ctx)
			continue;
		ret = iommu_attach_device(iommu, ctx);
		if (ret) {
			dev_warn(dev->dev, "could not attach iommu to %s", iommu_ports[i]);
			goto fail;
		}
	}

	mdp4_kms->id = msm_register_iommu(dev, iommu);
	if (mdp4_kms->id < 0) {
		ret = mdp4_kms->id;
		dev_err(dev->dev, "failed to register mdp4 iommu: %d\n", ret);
		goto fail;
	}

	ret = modeset_init(mdp4_kms);
	if (ret) {
		dev_err(dev->dev, "modeset_init failed: %d\n", ret);
		goto fail;
	}

	return kms;

fail:
	if (kms)
		mdp4_destroy(kms);
	return ERR_PTR(ret);
}

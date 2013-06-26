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

#include <mach/board.h>
#include <mach/socinfo.h>

#include "hdmi_connector.h"

static struct platform_device *hdmi_pdev;


static void set_mode(struct hdmi_connector *hdmi_connector, bool power_on)
{
	uint32_t ctrl = 0;

	if (power_on) {
		ctrl |= HDMI_CTRL_ENABLE;
		if (!hdmi_connector->hdmi) {
			ctrl |= HDMI_CTRL_HDMI;
			hdmi_write(hdmi_connector, REG_HDMI_CTRL, ctrl);
			ctrl &= ~HDMI_CTRL_HDMI;
		} else {
			ctrl |= HDMI_CTRL_HDMI;
		}
	} else {
		ctrl = HDMI_CTRL_HDMI;
	}

	hdmi_write(hdmi_connector, REG_HDMI_CTRL, ctrl);
	DBG("HDMI Core: %s, HDMI_CTRL=0x%08x",
			power_on ? "Enable" : "Disable", ctrl);
}

static int hpd_enable(struct hdmi_connector *hdmi_connector)
{
	struct drm_device *dev = hdmi_connector->base.base.dev;
	struct msm_hdmi_platform_data *pd =
			hdmi_connector->pdev->dev.platform_data;
	struct hdmi_phy *phy = hdmi_connector->phy;
	uint32_t hpd_ctrl;
	int ret;

	ret = pd->gpio_config(1);
	if (ret) {
		dev_err(dev->dev, "failed to configure GPIOs: %d\n", ret);
		goto fail;
	}

	ret = clk_prepare_enable(hdmi_connector->clk);
	if (ret) {
		dev_err(dev->dev, "failed to enable 'clk': %d\n", ret);
		goto fail;
	}

	ret = clk_prepare_enable(hdmi_connector->m_pclk);
	if (ret) {
		dev_err(dev->dev, "failed to enable 'm_pclk': %d\n", ret);
		goto fail;
	}

	ret = clk_prepare_enable(hdmi_connector->s_pclk);
	if (ret) {
		dev_err(dev->dev, "failed to enable 's_pclk': %d\n", ret);
		goto fail;
	}

	if (hdmi_connector->mpp0)
		ret = regulator_enable(hdmi_connector->mpp0);
	if (!ret)
		ret = regulator_enable(hdmi_connector->mvs);
	if (ret) {
		dev_err(dev->dev, "failed to enable regulators: %d\n", ret);
		goto fail;
	}

	set_mode(hdmi_connector, false);
	phy->funcs->reset(phy);
	set_mode(hdmi_connector, true);

	hdmi_write(hdmi_connector, REG_HDMI_USEC_REFTIMER, 0x0001001b);

	/* enable HPD events: */
	hdmi_write(hdmi_connector, REG_HDMI_HPD_INT_CTRL,
			HDMI_HPD_INT_CTRL_INT_CONNECT |
			HDMI_HPD_INT_CTRL_INT_EN);

	/* set timeout to 4.1ms (max) for hardware debounce */
	hpd_ctrl = hdmi_read(hdmi_connector, REG_HDMI_HPD_CTRL);
	hpd_ctrl |= HDMI_HPD_CTRL_TIMEOUT(0x1fff);

	/* Toggle HPD circuit to trigger HPD sense */
	hdmi_write(hdmi_connector, REG_HDMI_HPD_CTRL,
			~HDMI_HPD_CTRL_ENABLE & hpd_ctrl);
	hdmi_write(hdmi_connector, REG_HDMI_HPD_CTRL,
			HDMI_HPD_CTRL_ENABLE | hpd_ctrl);

	return 0;

fail:
	return ret;
}

static int hdp_disable(struct hdmi_connector *hdmi_connector)
{
	struct drm_device *dev = hdmi_connector->base.base.dev;
	struct msm_hdmi_platform_data *pd =
			hdmi_connector->pdev->dev.platform_data;
	int ret = 0;

	/* Disable HPD interrupt */
	hdmi_write(hdmi_connector, REG_HDMI_HPD_INT_CTRL, 0);

	set_mode(hdmi_connector, false);

	if (hdmi_connector->mpp0)
		ret = regulator_disable(hdmi_connector->mpp0);
	if (!ret)
		ret = regulator_disable(hdmi_connector->mvs);
	if (ret) {
		dev_err(dev->dev, "failed to enable regulators: %d\n", ret);
		goto fail;
	}

	clk_disable_unprepare(hdmi_connector->clk);
	clk_disable_unprepare(hdmi_connector->m_pclk);
	clk_disable_unprepare(hdmi_connector->s_pclk);

	ret = pd->gpio_config(0);
	if (ret) {
		dev_err(dev->dev, "failed to unconfigure GPIOs: %d\n", ret);
		goto fail;
	}

	return 0;

fail:
	return ret;
}

static irqreturn_t hdmi_connector_irq(int irq, void *dev_id)
{
	struct hdmi_connector *hdmi_connector = dev_id;
	struct drm_connector *connector = &hdmi_connector->base.base;
	uint32_t hpd_int_status, hpd_int_ctrl;

	/* Process HPD: */
	hpd_int_status = hdmi_read(hdmi_connector, REG_HDMI_HPD_INT_STATUS);
	hpd_int_ctrl   = hdmi_read(hdmi_connector, REG_HDMI_HPD_INT_CTRL);

	if ((hpd_int_ctrl & HDMI_HPD_INT_CTRL_INT_EN) &&
			(hpd_int_status & HDMI_HPD_INT_STATUS_INT)) {
		bool detected = !!(hpd_int_status & HDMI_HPD_INT_STATUS_CABLE_DETECTED);

		DBG("status=%04x, ctrl=%04x", hpd_int_status, hpd_int_ctrl);

		/* ack the irq: */
		hdmi_write(hdmi_connector, REG_HDMI_HPD_INT_CTRL,
				hpd_int_ctrl | HDMI_HPD_INT_CTRL_INT_ACK);

		drm_helper_hpd_irq_event(connector->dev);

		/* detect disconnect if we are connected or visa versa: */
		hpd_int_ctrl = HDMI_HPD_INT_CTRL_INT_EN;
		if (!detected)
			hpd_int_ctrl |= HDMI_HPD_INT_CTRL_INT_CONNECT;
		hdmi_write(hdmi_connector, REG_HDMI_HPD_INT_CTRL, hpd_int_ctrl);

		return IRQ_HANDLED;
	}

	/* Process DDC: */
	hdmi_i2c_irq(hdmi_connector->i2c);

	/* TODO audio.. */

	return IRQ_HANDLED;
}

static enum drm_connector_status hdmi_connector_detect(
		struct drm_connector *connector, bool force)
{
	struct msm_connector *msm_connector = to_msm_connector(connector);
	struct hdmi_connector *hdmi_connector = to_hdmi_connector(msm_connector);
	uint32_t hpd_int_status;

	hpd_int_status = hdmi_read(hdmi_connector, REG_HDMI_HPD_INT_STATUS);

	return (hpd_int_status & HDMI_HPD_INT_STATUS_CABLE_DETECTED) ?
			connector_status_connected : connector_status_disconnected;
}

static void hdmi_connector_destroy(struct drm_connector *connector)
{
	struct msm_connector *msm_connector = to_msm_connector(connector);
	struct hdmi_connector *hdmi_connector = to_hdmi_connector(msm_connector);
	struct hdmi_phy *phy = hdmi_connector->phy;

	hdp_disable(hdmi_connector);

	if (phy)
		phy->funcs->destroy(phy);

	if (hdmi_connector->i2c)
		hdmi_i2c_destroy(hdmi_connector->i2c);

	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);

	put_device(&hdmi_connector->pdev->dev);

	kfree(hdmi_connector);
}

static int hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct msm_connector *msm_connector = to_msm_connector(connector);
	struct hdmi_connector *hdmi_connector = to_hdmi_connector(msm_connector);
	struct edid *edid;
	int ret = 0;

	edid = drm_get_edid(connector, hdmi_connector->i2c);

	drm_mode_connector_update_edid_property(connector, edid);

	if (edid) {
		ret = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

	return ret;
}

static int hdmi_connector_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	return 0;
}

static const struct drm_connector_funcs hdmi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = hdmi_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = hdmi_connector_destroy,
};

static const struct drm_connector_helper_funcs hdmi_connector_helper_funcs = {
	.get_modes = hdmi_connector_get_modes,
	.mode_valid = hdmi_connector_mode_valid,
	.best_encoder = msm_connector_attached_encoder,
};

static void hdmi_connector_dpms(struct msm_connector *msm_connector, int mode)
{
	struct hdmi_connector *hdmi_connector = to_hdmi_connector(msm_connector);
	struct hdmi_phy *phy = hdmi_connector->phy;
	bool enabled = (mode == DRM_MODE_DPMS_ON);

	DBG("mode=%d", mode);

	if (enabled == hdmi_connector->enabled)
		return;

	if (enabled) {
		phy->funcs->powerup(phy);
		set_mode(hdmi_connector, true);
	} else {
		set_mode(hdmi_connector, false);
		phy->funcs->powerdown(phy);
	}

	hdmi_connector->enabled = enabled;
}

static void hdmi_connector_mode_set(struct msm_connector *msm_connector,
		struct drm_display_mode *mode)
{
	struct hdmi_connector *hdmi_connector = to_hdmi_connector(msm_connector);
	int hstart, hend, vstart, vend;
	uint32_t frame_ctrl;

	hdmi_connector->hdmi = drm_match_cea_mode(mode) > 1;

	hstart = mode->htotal - mode->hsync_start;
	hend   = mode->htotal - mode->hsync_start + mode->hdisplay;

	vstart = mode->vtotal - mode->vsync_start - 1;
	vend   = mode->vtotal - mode->vsync_start + mode->vdisplay - 1;

	DBG("htotal=%d, vtotal=%d, hstart=%d, hend=%d, vstart=%d, vend=%d",
			mode->htotal, mode->vtotal, hstart, hend, vstart, vend);

	hdmi_write(hdmi_connector, REG_HDMI_TOTAL,
			HDMI_TOTAL_H_TOTAL(mode->htotal - 1) |
			HDMI_TOTAL_V_TOTAL(mode->vtotal - 1));

	hdmi_write(hdmi_connector, REG_HDMI_ACTIVE_HSYNC,
			HDMI_ACTIVE_HSYNC_START(hstart) |
			HDMI_ACTIVE_HSYNC_END(hend));
	hdmi_write(hdmi_connector, REG_HDMI_ACTIVE_VSYNC,
			HDMI_ACTIVE_VSYNC_START(vstart) |
			HDMI_ACTIVE_VSYNC_END(vend));

	if (mode->flags & DRM_MODE_FLAG_INTERLACE) {
		hdmi_write(hdmi_connector, REG_HDMI_VSYNC_TOTAL_F2,
				HDMI_VSYNC_TOTAL_F2_V_TOTAL(mode->vtotal));
		hdmi_write(hdmi_connector, REG_HDMI_VSYNC_ACTIVE_F2,
				HDMI_VSYNC_ACTIVE_F2_START(vstart + 1) |
				HDMI_VSYNC_ACTIVE_F2_END(vend + 1));
	} else {
		hdmi_write(hdmi_connector, REG_HDMI_VSYNC_TOTAL_F2,
				HDMI_VSYNC_TOTAL_F2_V_TOTAL(0));
		hdmi_write(hdmi_connector, REG_HDMI_VSYNC_ACTIVE_F2,
				HDMI_VSYNC_ACTIVE_F2_START(0) |
				HDMI_VSYNC_ACTIVE_F2_END(0));
	}

	frame_ctrl = 0;
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		frame_ctrl |= HDMI_FRAME_CTRL_HSYNC_LOW;
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		frame_ctrl |= HDMI_FRAME_CTRL_VSYNC_LOW;
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		frame_ctrl |= HDMI_FRAME_CTRL_INTERLACED_EN;
	DBG("frame_ctrl=%08x", frame_ctrl);
	hdmi_write(hdmi_connector, REG_HDMI_FRAME_CTRL, frame_ctrl);

	// TODO until we have audio, this might be safest:
	if (hdmi_connector->hdmi)
		hdmi_write(hdmi_connector, REG_HDMI_GC, HDMI_GC_MUTE);
}

static const struct msm_connector_funcs msm_connector_funcs = {
		.dpms = hdmi_connector_dpms,
		.mode_set = hdmi_connector_mode_set,
};


/* initialize connector */
struct drm_connector *hdmi_connector_init(struct drm_device *dev,
		struct drm_encoder *encoder)
{
	struct drm_connector *connector = NULL;
	struct hdmi_connector *hdmi_connector;
	struct platform_device *pdev = hdmi_pdev;
	struct resource *res;
	int ret;

	if (!pdev) {
		dev_err(dev->dev, "no hdmi device\n");
		ret = -ENXIO;
		goto fail;
	}

	get_device(&pdev->dev);

	hdmi_connector = kzalloc(sizeof(struct hdmi_connector), GFP_KERNEL);
	if (!hdmi_connector) {
		ret = -ENOMEM;
		goto fail;
	}

	connector = &hdmi_connector->base.base;

	hdmi_connector->pdev = pdev;

	msm_connector_init(&hdmi_connector->base,
			&msm_connector_funcs, encoder);

	drm_connector_init(dev, connector, &hdmi_connector_funcs,
			DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_helper_add(connector, &hdmi_connector_helper_funcs);

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	connector->interlace_allowed = 1;
	connector->doublescan_allowed = 0;

	drm_sysfs_connector_add(connector);

	/* not sure about which phy maps to which msm.. probably I miss some */
	if (cpu_is_msm8960() || cpu_is_apq8064())
		hdmi_connector->phy = hdmi_phy_8960_init(hdmi_connector);
	else if (cpu_is_msm8x60())
		hdmi_connector->phy = hdmi_phy_8x60_init(hdmi_connector);
	else
		hdmi_connector->phy = ERR_PTR(-ENXIO);

	if (IS_ERR(hdmi_connector->phy)) {
		ret = PTR_ERR(hdmi_connector->phy);
		dev_err(dev->dev, "failed to load phy: %d\n", ret);
		hdmi_connector->phy = NULL;
		goto fail;
	}

	res = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "hdmi_msm_hdmi_addr");
	if (!res) {
		dev_err(dev->dev, "failed to get memory resource\n");
		ret = -EINVAL;
		goto fail;
	}

	hdmi_connector->mmio = msm_ioremap(&pdev->dev,
			res->start, resource_size(res), "HDMI");
	if (!hdmi_connector->mmio) {
		dev_err(dev->dev, "failed to ioremap\n");
		ret = -ENOMEM;
		goto fail;
	}

	hdmi_connector->mvs = devm_regulator_get(&pdev->dev, "8901_hdmi_mvs");
	if (IS_ERR(hdmi_connector->mvs))
		hdmi_connector->mvs = devm_regulator_get(&pdev->dev, "hdmi_mvs");
	if (IS_ERR(hdmi_connector->mvs)) {
		ret = PTR_ERR(hdmi_connector->mvs);
		dev_err(dev->dev, "failed to get mvs regulator: %d\n", ret);
		goto fail;
	}

	hdmi_connector->mpp0 = devm_regulator_get(&pdev->dev, "8901_mpp0");
	if (IS_ERR(hdmi_connector->mpp0))
		hdmi_connector->mpp0 = NULL;

	hdmi_connector->clk = devm_clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(hdmi_connector->clk)) {
		ret = PTR_ERR(hdmi_connector->clk);
		dev_err(dev->dev, "failed to get 'clk': %d\n", ret);
		goto fail;
	}

	hdmi_connector->m_pclk = devm_clk_get(&pdev->dev, "master_iface_clk");
	if (IS_ERR(hdmi_connector->m_pclk)) {
		ret = PTR_ERR(hdmi_connector->m_pclk);
		dev_err(dev->dev, "failed to get 'm_pclk': %d\n", ret);
		goto fail;
	}

	hdmi_connector->s_pclk = devm_clk_get(&pdev->dev, "slave_iface_clk");
	if (IS_ERR(hdmi_connector->s_pclk)) {
		ret = PTR_ERR(hdmi_connector->s_pclk);
		dev_err(dev->dev, "failed to get 's_pclk': %d\n", ret);
		goto fail;
	}

	hdmi_connector->irq = platform_get_irq(pdev, 0);
	if (hdmi_connector->irq < 0) {
		ret = hdmi_connector->irq;
		dev_err(dev->dev, "failed to get irq: %d\n", ret);
		goto fail;
	}

	ret = devm_request_threaded_irq(&pdev->dev, hdmi_connector->irq,
			NULL, hdmi_connector_irq, IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			"hdmi_connector_isr", hdmi_connector);
	if (ret < 0) {
		dev_err(dev->dev, "failed to request IRQ%u: %d\n",
				hdmi_connector->irq, ret);
		goto fail;
	}

	hdmi_connector->i2c = hdmi_i2c_init(hdmi_connector);
	if (IS_ERR(hdmi_connector->i2c)) {
		ret = PTR_ERR(hdmi_connector->i2c);
		dev_err(dev->dev, "failed to get i2c: %d\n", ret);
		hdmi_connector->i2c = NULL;
		goto fail;
	}

	ret = hpd_enable(hdmi_connector);
	if (ret) {
		dev_err(dev->dev, "failed to enable HPD: %d\n", ret);
		goto fail;
	}

	return connector;

fail:
	if (connector)
		hdmi_connector_destroy(connector);

	return ERR_PTR(ret);
}

/*
 * The hdmi device:
 */

static int __devinit hdmi_probe(struct platform_device *pdev)
{
	hdmi_pdev = pdev;
	return 0;
}

static int __devexit hdmi_remove(struct platform_device *pdev)
{
	hdmi_pdev = NULL;
	return 0;
}

static struct platform_driver hdmi_driver = {
	.probe = hdmi_probe,
	.remove = hdmi_remove,
	.driver.name = "hdmi_msm",
};

void __init hdmi_init(void)
{
	platform_driver_register(&hdmi_driver);
}

void __exit hdmi_fini(void)
{
	platform_driver_unregister(&hdmi_driver);
}

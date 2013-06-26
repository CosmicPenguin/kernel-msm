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

#ifndef __HDMI_CONNECTOR_H__
#define __HDMI_CONNECTOR_H__

#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include "msm_connector.h"

#include "hdmi.xml.h"


struct hdmi_phy;

struct hdmi_connector {
	struct msm_connector base;

	struct platform_device *pdev;

	void __iomem *mmio;

	struct regulator *mvs;        /* HDMI_5V */
	struct regulator *mpp0;       /* External 5V */

	struct clk *clk;
	struct clk *m_pclk;
	struct clk *s_pclk;

	struct hdmi_phy *phy;
	struct i2c_adapter *i2c;

	int irq;

	bool enabled;                 /* DPMS state */
	bool hdmi;                    /* are we in hdmi mode? */
};
#define to_hdmi_connector(x) container_of(x, struct hdmi_connector, base)

static inline void hdmi_write(struct hdmi_connector *c, u32 reg, u32 data)
{
	msm_writel(data, c->mmio + reg);
}

static inline u32 hdmi_read(struct hdmi_connector *c, u32 reg)
{
	return msm_readl(c->mmio + reg);
}

/*
 * The phy appears to be different, for example between 8960 and 8x60,
 * so split the phy related functions out and load the correct one at
 * runtime:
 */

struct hdmi_phy_funcs {
	void (*destroy)(struct hdmi_phy *phy);
	void (*reset)(struct hdmi_phy *phy);
	void (*powerup)(struct hdmi_phy *phy);
	void (*powerdown)(struct hdmi_phy *phy);
};

struct hdmi_phy {
	const struct hdmi_phy_funcs *funcs;
};

struct hdmi_phy *hdmi_phy_8960_init(struct hdmi_connector *hdmi_connector);
struct hdmi_phy *hdmi_phy_8x60_init(struct hdmi_connector *hdmi_connector);

/*
 * i2c adapter for ddc:
 */

void hdmi_i2c_irq(struct i2c_adapter *i2c);
void hdmi_i2c_destroy(struct i2c_adapter *i2c);
struct i2c_adapter *hdmi_i2c_init(struct hdmi_connector *hdmi_connector);

#endif /* __HDMI_CONNECTOR_H__ */

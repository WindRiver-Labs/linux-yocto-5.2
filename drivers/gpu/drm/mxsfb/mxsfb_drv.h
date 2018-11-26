/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2016 Marek Vasut <marex@denx.de>
 *
 * i.MX23/i.MX28/i.MX6SX MXSFB LCD controller driver.
 */

#ifndef __MXSFB_DRV_H__
#define __MXSFB_DRV_H__

#define MAX_CLK_SRC 2

struct mxsfb_devdata {
	unsigned int	 transfer_count;
	unsigned int	 cur_buf;
	unsigned int	 next_buf;
	unsigned int	 debug0;
	unsigned int	 hs_wdth_mask;
	unsigned int	 hs_wdth_shift;
	unsigned int	 ipversion;
	unsigned int	 flags;
	unsigned int	 num_formats;
};

struct mode_config {
	struct clk *clk_src;
	unsigned long out_rate;
	int clock;
	int mode_clock;
	struct list_head list;
};

struct mxsfb_drm_private {
	struct device			*dev;
	const struct mxsfb_devdata	*devdata;

	void __iomem			*base;	/* registers */
	struct clk			*clk;
	struct clk			*clk_axi;
	struct clk			*clk_disp_axi;
	struct clk			*clk_src[MAX_CLK_SRC];
	struct clk			*clk_sel, *clk_pll;

	struct drm_simple_display_pipe	pipe;
	struct drm_connector		panel_connector;
	struct drm_connector		*connector;
	struct drm_panel		*panel;
	struct drm_bridge		*bridge;

	struct drm_gem_cma_object	*gem;
	bool				enabled;

	struct list_head		valid_modes;
};

int mxsfb_setup_crtc(struct drm_device *dev);
int mxsfb_create_output(struct drm_device *dev);

void mxsfb_crtc_enable(struct mxsfb_drm_private *mxsfb);
void mxsfb_crtc_disable(struct mxsfb_drm_private *mxsfb);
void mxsfb_plane_atomic_update(struct mxsfb_drm_private *mxsfb,
			       struct drm_plane_state *old_state);

#endif /* __MXSFB_DRV_H__ */

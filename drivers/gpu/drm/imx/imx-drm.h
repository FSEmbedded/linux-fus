/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _IMX_DRM_H_
#define _IMX_DRM_H_

#define MAX_CRTC	4

struct device_node;
struct drm_connector;
struct drm_device;
struct drm_encoder;

struct imx_crtc_state {
	struct drm_crtc_state			base;
	u32					bus_format;
	u32					bus_flags;
	int					di_hsync_pin;
	int					di_vsync_pin;
};

static inline struct imx_crtc_state *to_imx_crtc_state(struct drm_crtc_state *s)
{
	return container_of(s, struct imx_crtc_state, base);
}

int imx_drm_encoder_parse_of(struct drm_device *drm,
	struct drm_encoder *encoder, struct device_node *np);

#endif /* _IMX_DRM_H_ */

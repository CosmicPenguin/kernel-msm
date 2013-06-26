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

#include "drm_crtc.h"
#include "drm_crtc_helper.h"

/* per-plane info for the fb: */
struct plane {
	struct drm_gem_object *bo;
	uint32_t pitch;
	uint32_t offset;
};

struct msm_framebuffer {
	struct drm_framebuffer base;
//XXX	const struct format *format;
	struct plane planes[2];
};
#define to_msm_framebuffer(x) container_of(x, struct msm_framebuffer, base)


static int msm_framebuffer_create_handle(struct drm_framebuffer *fb,
		struct drm_file *file_priv,
		unsigned int *handle)
{
	struct msm_framebuffer *msm_fb = to_msm_framebuffer(fb);
	return drm_gem_handle_create(file_priv,
			msm_fb->planes[0].bo, handle);
}

static void msm_framebuffer_destroy(struct drm_framebuffer *fb)
{
	struct msm_framebuffer *msm_fb = to_msm_framebuffer(fb);
	int i, n = drm_format_num_planes(fb->pixel_format);

	DBG("destroy: FB ID: %d (%p)", fb->base.id, fb);

	drm_framebuffer_cleanup(fb);

	for (i = 0; i < n; i++) {
		struct plane *plane = &msm_fb->planes[i];
		if (plane->bo)
			drm_gem_object_unreference_unlocked(plane->bo);
	}

	kfree(msm_fb);
}

static int msm_framebuffer_dirty(struct drm_framebuffer *fb,
		struct drm_file *file_priv, unsigned flags, unsigned color,
		struct drm_clip_rect *clips, unsigned num_clips)
{
	return 0;
}

static const struct drm_framebuffer_funcs msm_framebuffer_funcs = {
	.create_handle = msm_framebuffer_create_handle,
	.destroy = msm_framebuffer_destroy,
	.dirty = msm_framebuffer_dirty,
};

#ifdef CONFIG_DEBUG_FS
void msm_framebuffer_describe(struct drm_framebuffer *fb, struct seq_file *m)
{
	struct msm_framebuffer *msm_fb = to_msm_framebuffer(fb);
	int i, n = drm_format_num_planes(fb->pixel_format);

	seq_printf(m, "fb: %dx%d@%4.4s\n", fb->width, fb->height,
			(char *)&fb->pixel_format);

	for (i = 0; i < n; i++) {
		struct plane *plane = &msm_fb->planes[i];
		seq_printf(m, "   %d: offset=%d pitch=%d, obj: ",
				i, plane->offset, plane->pitch);
		msm_gem_describe(plane->bo, m);
	}
}
#endif

struct drm_gem_object *msm_framebuffer_bo(struct drm_framebuffer *fb, int plane)
{
	struct msm_framebuffer *msm_fb = to_msm_framebuffer(fb);
	return msm_fb->planes[plane].bo;
}

struct drm_framebuffer *msm_framebuffer_create(struct drm_device *dev,
		struct drm_file *file, struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_gem_object *bos[4] = {0};
	struct drm_framebuffer *fb;
	int ret, i, n = drm_format_num_planes(mode_cmd->pixel_format);

	for (i = 0; i < n; i++) {
		bos[i] = drm_gem_object_lookup(dev, file,
				mode_cmd->handles[i]);
		if (!bos[i]) {
			ret = -ENXIO;
			goto out_unref;
		}
	}

	fb = msm_framebuffer_init(dev, mode_cmd, bos);
	if (IS_ERR(fb)) {
		ret = PTR_ERR(fb);
		goto out_unref;
	}

	return fb;

out_unref:
	for (i = 0; i < n; i++)
		drm_gem_object_unreference_unlocked(bos[i]);
	return ERR_PTR(ret);
}

struct drm_framebuffer *msm_framebuffer_init(struct drm_device *dev,
		struct drm_mode_fb_cmd2 *mode_cmd, struct drm_gem_object **bos)
{
	struct msm_framebuffer *msm_fb;
	struct drm_framebuffer *fb = NULL;
	int ret, i, n = drm_format_num_planes(mode_cmd->pixel_format);

	DBG("create framebuffer: dev=%p, mode_cmd=%p (%dx%d@%4.4s)",
			dev, mode_cmd, mode_cmd->width, mode_cmd->height,
			(char *)&mode_cmd->pixel_format);

/* XXX we need to get info about supported formats from the kms
 * implementation.. since potentially (probably?) different between
 * mdp4, mdss, etc..
	const struct format *format = NULL;
	for (i = 0; i < ARRAY_SIZE(formats); i++) {
		if (formats[i].pixel_format == mode_cmd->pixel_format) {
			format = &formats[i];
			break;
		}
	}

	if (!format) {
		dev_err(dev->dev, "unsupported pixel format: %4.4s\n",
				(char *)&mode_cmd->pixel_format);
		ret = -EINVAL;
		goto fail;
	}
 */

	msm_fb = kzalloc(sizeof(*msm_fb), GFP_KERNEL);
	if (!msm_fb) {
		ret = -ENOMEM;
		goto fail;
	}

	fb = &msm_fb->base;

//	msm_fb->format = format;

	for (i = 0; i < n; i++) {
		struct plane *plane = &msm_fb->planes[i];

/*
		int size, pitch = mode_cmd->pitches[i];
		if (pitch < (mode_cmd->width * format->planes[i].stride_bpp)) {
			dev_err(dev->dev, "provided buffer pitch is too small! %d < %d\n",
					pitch, mode_cmd->width * format->planes[i].stride_bpp);
			ret = -EINVAL;
			goto fail;
		}

		size = pitch * mode_cmd->height / format->planes[i].sub_y;

		if (size > (msm_gem_mmap_size(bos[i]) - mode_cmd->offsets[i])) {
			dev_err(dev->dev, "provided buffer object is too small! %d < %d\n",
					bos[i]->size - mode_cmd->offsets[i], size);
			ret = -EINVAL;
			goto fail;
		}
*/

		plane->bo     = bos[i];
		plane->offset = mode_cmd->offsets[i];
		plane->pitch  = mode_cmd->pitches[i];
	}

	drm_helper_mode_fill_fb_struct(fb, mode_cmd);

	ret = drm_framebuffer_init(dev, fb, &msm_framebuffer_funcs);
	if (ret) {
		dev_err(dev->dev, "framebuffer init failed: %d\n", ret);
		goto fail;
	}

	DBG("create: FB ID: %d (%p)", fb->base.id, fb);

	return fb;

fail:
	if (fb)
		msm_framebuffer_destroy(fb);

	return ERR_PTR(ret);
}

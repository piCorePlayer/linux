/*
 * DRM driver for ST7789V panels with flexible config
 *
 * Copyright 2019 Limor Fried
 * Copyright 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_rect.h>
#include <drm/drm_vblank.h>

#define ST77XX_MADCTL_MY  0x80
#define ST77XX_MADCTL_MX  0x40
#define ST77XX_MADCTL_MV  0x20
#define ST77XX_MADCTL_ML  0x10
#define ST77XX_MADCTL_BGR 0x08
#define ST77XX_MADCTL_RGB 0x00

static u32 col_offset = 0;
static u32 row_offset = 0;
static u8 col_hack_fix_offset = 0;
static short x_offset = 0;
static short y_offset = 0;

static void st7789v_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	bool swap = dbi->swap_bytes;
	unsigned int height = drm_rect_height(rect);
	unsigned int width = drm_rect_width(rect);
	int idx, ret = 0;
	u16 x1, x2, y1, y2;
	bool full;
	void *tr;

	if (!dbidev->enabled)
		return;

	if (!drm_dev_enter(fb->dev, &idx))
		return;

	full = width == fb->width && height == fb->height;
	
	DRM_DEBUG_KMS("Flushing [FB:%d] " DRM_RECT_FMT "\n", fb->base.id, DRM_RECT_ARG(rect));

	if (!dbi->dc || !full || swap ||
	    fb->format->format == DRM_FORMAT_XRGB8888) {
		tr = dbidev->tx_buf;
		ret = mipi_dbi_buf_copy(dbidev->tx_buf, fb, rect, swap);
		if (ret)
			goto err_msg;
	} else {
		tr = cma_obj->vaddr;
	}

	x1 = rect->x1 + x_offset;
	x2 = rect->x2 - 1 + x_offset;
	y1 = rect->y1 + y_offset;
	y2 = rect->y2 - 1 + y_offset;

	//printk(KERN_INFO "setaddrwin %d %d %d %d\n", x1, y1, x2, y2);

	mipi_dbi_command(dbi, MIPI_DCS_SET_COLUMN_ADDRESS,
			 (x1 >> 8) & 0xFF, x1 & 0xFF,
			 (x2 >> 8) & 0xFF, x2 & 0xFF);
	mipi_dbi_command(dbi, MIPI_DCS_SET_PAGE_ADDRESS,
			 (y1 >> 8) & 0xFF, y1 & 0xFF,
			 (y2 >> 8) & 0xFF, y2 & 0xFF);

	ret = mipi_dbi_command_buf(dbi, MIPI_DCS_WRITE_MEMORY_START, tr,
				(rect->x2 - rect->x1) * (rect->y2 - rect->y1) * 2);
err_msg:
	if (ret)
		dev_err_once(fb->dev->dev, "Failed to update display %d\n", ret);

	drm_dev_exit(idx);
	
}

static void st7789v_pipe_update(struct drm_simple_display_pipe *pipe,
				struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_rect rect;

	if (drm_atomic_helper_damage_merged(old_state, state, &rect))
		st7789v_fb_dirty(state->fb, &rect);

	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);
		crtc->state->event = NULL;
	}
}

static void st7789v_pipe_enable(struct drm_simple_display_pipe *pipe,
			    struct drm_crtc_state *crtc_state,
			    struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct drm_framebuffer *fb = plane_state->fb;
	struct device *dev = pipe->crtc.dev->dev;
	struct mipi_dbi *dbi = &dbidev->dbi;
	struct drm_rect rect = {
		.x1 = 0,
		.x2 = fb->width,
		.y1 = 0,
		.y2 = fb->height,
	};
	u8 addr_mode;
	int ret, idx;
	
	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		return;
	if (ret == 1)
		goto out_enable;

	ret = mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_OFF);
	if (ret) {
		DRM_DEV_ERROR(dev, "Error sending command %d\n", ret);
		goto out_exit;
	}
		
	mipi_dbi_command(dbi, MIPI_DCS_SOFT_RESET);
	msleep(150);
	mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(10);
	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, 0x55); // 16 bit color
	msleep(10);
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, 0);
	mipi_dbi_command(dbi, MIPI_DCS_SET_COLUMN_ADDRESS, 0, 0, 0, 240);
	mipi_dbi_command(dbi, MIPI_DCS_SET_PAGE_ADDRESS, 0, 0, 320>>8, 320&0xFF);
	mipi_dbi_command(dbi, MIPI_DCS_ENTER_INVERT_MODE); // odd hack, displays are inverted
	mipi_dbi_command(dbi, MIPI_DCS_ENTER_NORMAL_MODE);
	msleep(10);
	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);
	msleep(10);

out_enable:
	/* The PiTFT (ili9340) has a hardware reset circuit that
	 * resets only on power-on and not on each reboot through
	 * a gpio like the rpi-display does.
	 * As a result, we need to always apply the rotation value
	 * regardless of the display "on/off" state.
	 */
	switch (dbidev->rotation) {
	default:
		addr_mode = 0;
		x_offset = col_offset;
		y_offset = row_offset;
		break;
	case 90:
		addr_mode = ST77XX_MADCTL_MV | ST77XX_MADCTL_MX;
		x_offset = row_offset;
		y_offset = col_offset;
		break;
	case 180:
		addr_mode = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY;
		x_offset = col_offset+col_hack_fix_offset; 
		// hack tweak to account for extra pixel width to make even
		y_offset = row_offset; 
		break;
	case 270:
		addr_mode = ST77XX_MADCTL_MV | ST77XX_MADCTL_MY;
		x_offset = row_offset;
		y_offset = col_offset;
		break;
	}
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);

	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);

	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);

	backlight_enable(dbidev->backlight);
	
	dbidev->enabled = true;
	st7789v_fb_dirty(fb, &rect);

out_exit:	
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs st7789v_pipe_funcs = {
	.enable = st7789v_pipe_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = st7789v_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static struct drm_display_mode st7789v_mode = {
  DRM_SIMPLE_MODE(240, 320, 25, 15), // width, height, mm_w, mm_h
};

DEFINE_DRM_GEM_CMA_FOPS(st7789v_fops);

static struct drm_driver st7789v_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &st7789v_fops,
	.release		= mipi_dbi_release, 
	DRM_GEM_CMA_VMAP_DRIVER_OPS,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "st7789v",
	.desc			= "ST7789V Adafruit",
	.date			= "20190914",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id st7789v_of_match[] = {
	{ .compatible = "sitronix,st7789v" },
	{},
};
MODULE_DEVICE_TABLE(of, st7789v_of_match);

static const struct spi_device_id st7789v_id[] = {
	{ "st7789v", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, st7789v_id);

/*
static const struct drm_framebuffer_funcs st7789v_fb_funcs = {
	.destroy	= drm_gem_fb_destroy,
	.create_handle	= drm_gem_fb_create_handle,
	.dirty		= tinydrm_fb_dirty,
};
*/

static const uint32_t st7789v_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
};

/**
 * st7789v - MIPI DBI initialization
 * @dev: Parent device
 * @mipi: &mipi_dbi structure to initialize
 * @pipe_funcs: Display pipe functions
 * @driver: DRM driver
 * @mode: Display mode
 * @rotation: Initial rotation in degrees Counter Clock Wise
 *
 * This function initializes a &mipi_dbi structure and it's underlying
 * @tinydrm_device. It also sets up the display pipeline.
 *
 * Supported formats: Native RGB565 and emulated XRGB8888.
 *
 * Objects created by this function will be automatically freed on driver
 * detach (devres).
 *
 * Returns:
 * Zero on success, negative error code on failure.
 */
/*
int st7789v_init(struct device *dev, struct mipi_dbi *mipi,
		  const struct drm_simple_display_pipe_funcs *pipe_funcs,
		  struct drm_driver *driver,
		  const struct drm_display_mode *mode, unsigned int rotation)
{
	size_t bufsize = mode->vdisplay * mode->hdisplay * sizeof(u16);
	struct tinydrm_device *tdev = &mipi->tinydrm;
	int ret;

	if (!mipi->command)
		return -EINVAL;

	mutex_init(&mipi->cmdlock);

	mipi->tx_buf = devm_kmalloc(dev, bufsize, GFP_KERNEL);
	if (!mipi->tx_buf)
		return -ENOMEM;

	ret = devm_tinydrm_init(dev, tdev, &st7789v_fb_funcs, driver);
	if (ret)
		return ret;

	tdev->fb_dirty = st7789v_fb_dirty;

	// TODO: Maybe add DRM_MODE_CONNECTOR_SPI 
	ret = tinydrm_display_pipe_init(tdev, pipe_funcs,
					DRM_MODE_CONNECTOR_VIRTUAL,
					st7789v_formats,
					ARRAY_SIZE(st7789v_formats), mode,
					rotation);
	if (ret)
		return ret;

	tdev->drm->mode_config.preferred_depth = 16;
	mipi->rotation = rotation;

	drm_mode_config_reset(tdev->drm);

	DRM_DEBUG_KMS("preferred_depth=%u, rotation = %u\n",
		      tdev->drm->mode_config.preferred_depth, rotation);

	return 0;
}
*/



static int st7789v_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	u32 width = 240;
	u32 height = 320;
	int ret;

	dbidev = kzalloc( sizeof(*dbidev), GFP_KERNEL);
	if (!dbidev)
		return -ENOMEM;

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;
	
	ret = devm_drm_dev_init(dev, drm, &st7789v_driver);
	if (ret) {
		kfree(dbidev);
		return ret;
	}
	
	drm_mode_config_init(drm);
	
	dbi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(dbi->reset);
	}

	dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'dc'\n");
		return PTR_ERR(dc);
	}

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);
	//printk(KERN_INFO "Rotation %d\n", rotation);

	device_property_read_u32(dev, "width", &width);
	if (width % 2) {
	  width +=1;	  // odd width will cause a kernel panic
	  col_hack_fix_offset = 1;
	} else {
	  col_hack_fix_offset = 0;
	}
	//printk(KERN_INFO "Width %d\n", width);
	if ((width == 0) || (width > 240)) {
	  width = 240; // default to full framebuff;
	}
	device_property_read_u32(dev, "height", &height);
	//printk(KERN_INFO "Height %d\n", height);
	if ((height == 0) || (height > 320)) {
	  height = 320; // default to full framebuff;
	}

	st7789v_mode.hdisplay = st7789v_mode.hsync_start = 
	  st7789v_mode.hsync_end = st7789v_mode.htotal = width;
	st7789v_mode.vdisplay = st7789v_mode.vsync_start = 
	  st7789v_mode.vsync_end = st7789v_mode.vtotal = height;

	device_property_read_u32(dev, "col_offset", &col_offset);
	//printk(KERN_INFO "Column offset %d\n", col_offset);

	device_property_read_u32(dev, "row_offset", &row_offset);
	//printk(KERN_INFO "Row offset %d\n", row_offset);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	spi->mode = SPI_MODE_3;
	if (ret)
		return ret;

	/* Cannot read from this controller via SPI */
	dbi->read_commands = NULL;

	ret = mipi_dbi_dev_init( dbidev, &st7789v_pipe_funcs, &st7789v_mode, rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;
	
	spi_set_drvdata(spi, drm);

	drm_fbdev_generic_setup(drm, 0);
	
	return 0;
}

static int st7789v_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}


static void st7789v_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver st7789v_spi_driver = {
	.driver = {
		.name = "st7789v",
		.owner = THIS_MODULE,
		.of_match_table = st7789v_of_match,
	},
	.id_table = st7789v_id,
	.probe = st7789v_probe,
	.remove = st7789v_remove,
	.shutdown = st7789v_shutdown,
};
module_spi_driver(st7789v_spi_driver);

MODULE_DESCRIPTION("Sitronix ST7789V Flexible DRM driver");
MODULE_AUTHOR("Limor Fried");
MODULE_LICENSE("GPL");

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
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_rect.h>
#include <drm/drm_vblank.h>
#include <drm/drm_modeset_helper.h>

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
	struct drm_gem_dma_object *dma_obj = drm_fb_dma_get_gem_obj(fb, 0);
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
	unsigned int height = rect->y2 - rect->y1;
	unsigned int width = rect->x2 - rect->x1;
	struct mipi_dbi *dbi = &dbidev->dbi;
	bool swap = dbi->swap_bytes;
	int idx, ret = 0;
	u16 x1, x2, y1, y2;
	bool full;
	void *tr;

	if (WARN_ON(!fb))
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
		tr = dma_obj->vaddr;
	}

	x1 = rect->x1 + x_offset;
	x2 = rect->x2 - 1 + x_offset;
	y1 = rect->y1 + y_offset;
	y2 = rect->y2 - 1 + y_offset;

	//printk(KERN_INFO "setaddrwin (%d, %d) -> (%d, %d) offsets: %d & %d \n", x1, y1, x2, y2, x_offset, y_offset);

	mipi_dbi_command(dbi, MIPI_DCS_SET_COLUMN_ADDRESS,
			 (x1 >> 8) & 0xFF, x1 & 0xFF,
			 (x2 >> 8) & 0xFF, x2 & 0xFF);
	mipi_dbi_command(dbi, MIPI_DCS_SET_PAGE_ADDRESS,
			 (y1 >> 8) & 0xFF, y1 & 0xFF,
			 (y2 >> 8) & 0xFF, y2 & 0xFF);

	ret = mipi_dbi_command_buf(dbi, MIPI_DCS_WRITE_MEMORY_START, tr,
				width*height * 2);
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

static struct drm_display_mode st7789v_mode = {
  DRM_SIMPLE_MODE(240, 320, 58, 43), // width, height, mm_w, mm_h
};

DEFINE_DRM_GEM_DMA_FOPS(st7789v_fops);

static void st7789v_pipe_enable(struct drm_simple_display_pipe *pipe,
			    struct drm_crtc_state *crtc_state,
			    struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	u8 addr_mode;
	u16 width = st7789v_mode.htotal;
	u16 height = st7789v_mode.vtotal;
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out_exit;
	if (ret == 1)
		goto out_enable;

	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_OFF);

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
        x_offset = (240 - width) - col_offset + col_hack_fix_offset;
        // hack tweak to account for extra pixel width to make even
        y_offset = (320 - height) - row_offset;
		break;
	case 270:
		addr_mode = ST77XX_MADCTL_MV | ST77XX_MADCTL_MY;
		x_offset = (320 - height) - row_offset;
		y_offset = (240 - width) - col_offset;
		break;
	}
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);

	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);

	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);

	backlight_enable(dbidev->backlight);

out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs st7789v_pipe_funcs = {
	.mode_valid = mipi_dbi_pipe_mode_valid,
	.enable = st7789v_pipe_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = st7789v_pipe_update,
};


static struct drm_driver st7789v_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &st7789v_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "st7789v",
	.desc			= "ST7789V Adafruit",
	.date			= "20201218",
	.major			= 1,
	.minor			= 8,
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

static const uint32_t st7789v_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
};

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

	dbidev = devm_drm_dev_alloc(dev, &st7789v_driver,
			    struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

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

	dbidev->regulator = devm_regulator_get(dev, "power");
	if (IS_ERR(dbidev->regulator))
		return PTR_ERR(dbidev->regulator);

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);
	//printk(KERN_INFO "Rotation %d\n", rotation);

	device_property_read_u32(dev, "width", &width);
	if (width % 2) {
	  width +=1;	  // odd width will cause a kernel panic
	  col_hack_fix_offset = 3;
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

static void st7789v_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return;
}


static void st7789v_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static int __maybe_unused st7789v_pm_suspend(struct device *dev)
{
	return drm_mode_config_helper_suspend(dev_get_drvdata(dev));
}

static int __maybe_unused st7789v_pm_resume(struct device *dev)
{
	drm_mode_config_helper_resume(dev_get_drvdata(dev));

	return 0;
}

static const struct dev_pm_ops st7789v_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(st7789v_pm_suspend, st7789v_pm_resume)
};


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
MODULE_AUTHOR("Noralf Trønnes + Limor Fried");
MODULE_LICENSE("GPL");

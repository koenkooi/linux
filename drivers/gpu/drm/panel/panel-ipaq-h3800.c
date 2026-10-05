// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM panel driver for the HP iPAQ h38xx series 320x240 TFT.
 *
 * Copyright (C) 2026 Koen Kooi <koen@dominion.thruhere.net>
 *
 * The panel has no recoverable part number, so the compatible names the
 * machine it is fitted to and the mode is fixed. The panel has six
 * separately gated supply rails (five LCD rails plus the frontlight power
 * rail FL_PWR_ON), sequenced across the prepare/enable/disable/unprepare
 * split: four rails come up in prepare(), one frame elapses in enable() and
 * only then LCD_PCI is asserted. Going down, the frontlight rail and
 * LCD_PCI drop first, then the 5 V rail, VGL, VGH and LCD_ON.
 *
 * The frontlight power rail is sequenced here rather than through an
 * enable-gpios of the backlight, so the pin has exactly one owner.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <drm/drm_connector.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

struct ipaq_h3800_panel {
	struct drm_panel panel;
	enum drm_panel_orientation orientation;

	/*
	 * ASIC1 GPIO outputs: bits 5-9 are the five LCD rails and bit 10 is
	 * FL_PWR_ON, the frontlight power rail. Polarity lives in the device
	 * tree, so the driver only drives logical 1 (rail on) and 0 (rail
	 * off).
	 */
	struct gpio_desc *lcd_5v;
	struct gpio_desc *lcd_on;
	struct gpio_desc *lcd_pci;
	struct gpio_desc *vgh;
	struct gpio_desc *vgl;
	struct gpio_desc *fl_pwr;
};

/*
 * 320x240 at a pixel clock of 5722 kHz (174757 ps), both syncs active low.
 * With a 206.4 MHz CPU clock this corresponds to a pixel clock divider
 * field of round(206400 / (2 * 5722)) - 2 = 16 in the SA-1100 LCD
 * controller.
 *
 *   hsync_start = hdisplay + right_margin  = 320 + 17 = 337
 *   hsync_end   = hsync_start + hsync_len  = 337 + 3  = 340
 *   htotal      = hsync_end + left_margin  = 340 + 12 = 352
 *   vsync_start = vdisplay + lower_margin  = 240 + 1  = 241
 *   vsync_end   = vsync_start + vsync_len  = 241 + 3  = 244
 *   vtotal      = vsync_end + upper_margin = 244 + 10 = 254
 */
static const struct drm_display_mode ipaq_h3800_mode = {
	.clock = 5722,
	.hdisplay = 320,
	.hsync_start = 337,
	.hsync_end = 340,
	.htotal = 352,
	.vdisplay = 240,
	.vsync_start = 241,
	.vsync_end = 244,
	.vtotal = 254,
	.width_mm = 77,
	.height_mm = 57,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static inline struct ipaq_h3800_panel *to_ipaq_h3800_panel(struct drm_panel *panel)
{
	return container_of(panel, struct ipaq_h3800_panel, panel);
}

/*
 * Power-up rails only. The atomic helpers enable the CRTC before the bridge
 * chain, so the LCD controller is already scanning out when this runs; the
 * handhelds.org 2.6 board code raised these rails before it started the
 * controller. The delays total 90 ms.
 */
static int ipaq_h3800_panel_prepare(struct drm_panel *panel)
{
	struct ipaq_h3800_panel *p = to_ipaq_h3800_panel(panel);

	gpiod_set_value_cansleep(p->lcd_on, 1);
	msleep(30);
	gpiod_set_value_cansleep(p->vgl, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(p->vgh, 1);
	msleep(50);
	gpiod_set_value_cansleep(p->lcd_5v, 1);
	usleep_range(5000, 6000);

	return 0;
}

/*
 * Runs after prepare(), with the controller running. One frame (17 ms) has
 * to elapse before LCD_PCI is asserted.
 *
 * FL_PWR_ON is asserted here too, after LCD_PCI, so a picture is on the
 * glass before the frontlight comes up. The hardware wants it only after the
 * PWM has been fully programmed, which this ordering cannot reproduce:
 * drm_panel_enable() always runs the enable() callback to completion before
 * it calls backlight_enable(). The rail therefore comes up a few
 * milliseconds ahead of the PWM reaching its duty cycle, which is
 * acceptable for a rail that is not part of the LCD panel's own electrical
 * sequencing.
 */
static int ipaq_h3800_panel_enable(struct drm_panel *panel)
{
	struct ipaq_h3800_panel *p = to_ipaq_h3800_panel(panel);

	usleep_range(17000, 18000);
	gpiod_set_value_cansleep(p->lcd_pci, 1);
	gpiod_set_value_cansleep(p->fl_pwr, 1);

	return 0;
}

/*
 * drm_panel_disable() calls backlight_disable() before this function, so the
 * PWM has already been told to stop before FL_PWR_ON clears here, which is
 * the order the hardware wants.
 *
 * LCD_PCI clears first, then a 30 ms wait before the controller stops, as
 * drm_panel_bridge runs disable() before atomic_disable.
 */
static int ipaq_h3800_panel_disable(struct drm_panel *panel)
{
	struct ipaq_h3800_panel *p = to_ipaq_h3800_panel(panel);

	gpiod_set_value_cansleep(p->fl_pwr, 0);
	gpiod_set_value_cansleep(p->lcd_pci, 0);
	msleep(30);

	return 0;
}

/*
 * Runs from post_disable, which the atomic helpers call before they disable
 * the CRTC, so the controller is still running; the 2.6 board code stopped
 * it first. This is deliberately not prepare() run backwards: VGL drops
 * before VGH, the same order as going up in prepare(). The delays total
 * 160 ms.
 */
static int ipaq_h3800_panel_unprepare(struct drm_panel *panel)
{
	struct ipaq_h3800_panel *p = to_ipaq_h3800_panel(panel);

	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(p->lcd_5v, 0);
	msleep(50);
	gpiod_set_value_cansleep(p->vgl, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(p->vgh, 0);
	msleep(100);
	gpiod_set_value_cansleep(p->lcd_on, 0);

	return 0;
}

static int ipaq_h3800_panel_get_modes(struct drm_panel *panel,
				      struct drm_connector *connector)
{
	static const u32 bus_format = MEDIA_BUS_FMT_RGB565_1X16;
	struct drm_display_mode *mode;
	int ret;

	mode = drm_mode_duplicate(connector->dev, &ipaq_h3800_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	/*
	 * The glass is 57 x 77 mm portrait, the size given for the whole
	 * h3xxx and h5xxx family (not measured on this unit). The controller
	 * scans it as 320 x 240, so the sizes are 77 wide by 57 high in scan
	 * orientation. The connector takes the size from here, since a
	 * panel's modes carry no size into display_info on their own.
	 *
	 * DE is active high and pixel data is driven on the rising edge of
	 * the pixel clock, which are the clear state of the OEP and PCP bits
	 * of the SA-1100 LCD controller.
	 */
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bus_flags = DRM_BUS_FLAG_DE_HIGH |
					    DRM_BUS_FLAG_PIXDATA_DRIVE_POSEDGE;

	ret = drm_display_info_set_bus_formats(&connector->display_info,
					       &bus_format, 1);
	if (ret)
		return ret;

	return 1;
}

/*
 * The glass is physically portrait while the controller scans out
 * landscape. The h39xx has the same case and panel size but is not mounted
 * the same way, and only one h3870 has been checked, so the mounting
 * correction is read from the "rotation" property of the devicetree node
 * instead of being hardcoded.
 */
static enum drm_panel_orientation ipaq_h3800_panel_get_orientation(struct drm_panel *panel)
{
	struct ipaq_h3800_panel *p = to_ipaq_h3800_panel(panel);

	return p->orientation;
}

static const struct drm_panel_funcs ipaq_h3800_panel_funcs = {
	.prepare = ipaq_h3800_panel_prepare,
	.enable = ipaq_h3800_panel_enable,
	.disable = ipaq_h3800_panel_disable,
	.unprepare = ipaq_h3800_panel_unprepare,
	.get_modes = ipaq_h3800_panel_get_modes,
	.get_orientation = ipaq_h3800_panel_get_orientation,
};

static int ipaq_h3800_panel_get_rail(struct device *dev, const char *id,
				     struct gpio_desc **out)
{
	struct gpio_desc *gpiod;

	gpiod = devm_gpiod_get(dev, id, GPIOD_ASIS);
	if (IS_ERR(gpiod))
		return dev_err_probe(dev, PTR_ERR(gpiod),
				     "failed to get %s rail\n", id);
	*out = gpiod;

	return 0;
}

/*
 * The rails are requested as they are, since the boot loader may have left
 * the panel lit. Each rail is made an output at the level it already has,
 * and if any of them is on, the panel is taken down in the order of
 * disable() and unprepare() rather than by dropping all six at once.
 */
static void ipaq_h3800_panel_power_off(struct ipaq_h3800_panel *p)
{
	struct gpio_desc *rails[] = {
		p->lcd_on, p->vgl, p->vgh, p->lcd_5v, p->lcd_pci, p->fl_pwr,
	};
	bool on = false;
	int i;

	for (i = 0; i < ARRAY_SIZE(rails); i++) {
		int val = gpiod_get_value_cansleep(rails[i]);

		if (val > 0)
			on = true;
		gpiod_direction_output(rails[i], val > 0);
	}

	if (on) {
		ipaq_h3800_panel_disable(&p->panel);
		ipaq_h3800_panel_unprepare(&p->panel);
	}
}

static int ipaq_h3800_panel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ipaq_h3800_panel *p;
	int ret;

	p = devm_drm_panel_alloc(dev, struct ipaq_h3800_panel, panel,
				 &ipaq_h3800_panel_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (IS_ERR(p))
		return PTR_ERR(p);

	ret = ipaq_h3800_panel_get_rail(dev, "lcd-5v", &p->lcd_5v);
	if (ret)
		return ret;
	ret = ipaq_h3800_panel_get_rail(dev, "lcd-on", &p->lcd_on);
	if (ret)
		return ret;
	ret = ipaq_h3800_panel_get_rail(dev, "lcd-pci", &p->lcd_pci);
	if (ret)
		return ret;
	ret = ipaq_h3800_panel_get_rail(dev, "vgh", &p->vgh);
	if (ret)
		return ret;
	ret = ipaq_h3800_panel_get_rail(dev, "vgl", &p->vgl);
	if (ret)
		return ret;
	ret = ipaq_h3800_panel_get_rail(dev, "fl-pwr", &p->fl_pwr);
	if (ret)
		return ret;

	ipaq_h3800_panel_power_off(p);

	ret = drm_of_get_panel_orientation(dev->of_node, &p->orientation);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read panel orientation\n");

	/* Links panel->backlight to the "backlight" phandle, if there is one. */
	ret = drm_panel_of_backlight(&p->panel);
	if (ret)
		return ret;

	return devm_drm_panel_add(dev, &p->panel);
}

static const struct of_device_id ipaq_h3800_panel_of_match[] = {
	{ .compatible = "hp,ipaq-h3800-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, ipaq_h3800_panel_of_match);

static struct platform_driver ipaq_h3800_panel_driver = {
	.probe = ipaq_h3800_panel_probe,
	.driver = {
		.name = "panel-ipaq-h3800",
		.of_match_table = ipaq_h3800_panel_of_match,
	},
};
module_platform_driver(ipaq_h3800_panel_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("HP iPAQ h38xx TFT panel driver");
MODULE_LICENSE("GPL");

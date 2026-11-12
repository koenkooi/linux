// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM panel driver for the HP iPAQ h3900 series 320x240 TFT.
 *
 * Copyright (C) 2026 Koen Kooi <koen@dominion.thruhere.net>
 *
 * The panel is a 320x240 16bpp reflective TFT driven over the PXA25x
 * SoC's own parallel RGB (DPI) output. It has no recoverable part
 * number, so the compatible names the machine it is fitted to.
 *
 * Five separately gated supply rails come up and go down in a fixed
 * order with per-step delays, and the power-down order is not the
 * power-up order reversed. That is why this is a panel driver with its
 * own compatible rather than a panel-dpi instance: panel-dpi offers a
 * single enable-gpios and a single power-supply, which cannot express
 * five rails, and regulator-fixed cannot express an ordering between
 * independent supplies.
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

struct ipaq_h3900_panel {
	struct drm_panel panel;
	enum drm_panel_orientation orientation;

	/*
	 * ASIC3 bank-B outputs. Polarity lives in the device tree, so the
	 * driver only ever drives logical 1 (rail on) and 0 (rail off).
	 */
	struct gpio_desc *lcd_on;
	struct gpio_desc *lcd_pci;
	struct gpio_desc *lcd_5v;
	struct gpio_desc *lcd_nv;
	struct gpio_desc *lcd_9v;
};

/*
 * Decoded from the LCD controller register values a live h3900 was found
 * running under its factory OS (LCCR1 = 0x0b10093f, LCCR2 = 0x051008ef,
 * LCCR3 = 0x0430000a), through the field macros in
 * drivers/video/fbdev/pxa3xx-regs.h:
 *
 *   PPL 0x13f -> hdisplay 320       LPP 0xef  -> vdisplay 240
 *   HSW 2     -> hsync 3            VSW 2     -> vsync 3
 *   ELW 0x10  -> front porch 17     EFW 0x10  -> front porch 16
 *   BLW 0x0b  -> back porch 12      BFW 0x05  -> back porch 5
 *
 * ELW/BLW/HSW/VSW/PPL/LPP are all encoded minus one; EFW and BFW are
 * not. Totals are 352 x 264 = 92,928 pixel clocks per frame, which at
 * the 4,524,218 Hz the controller's PCD of 10 produces from a 99.5328
 * MHz LCD clock is 48.69 Hz.
 *
 * LCCR3 bit 21 (HSP) and bit 20 (VSP) are both set, i.e. both syncs are
 * active low, hence NHSYNC | NVSYNC.
 */
static const struct drm_display_mode ipaq_h3900_mode = {
	.clock = 4524,
	.hdisplay = 320,
	.hsync_start = 320 + 17,
	.hsync_end = 320 + 17 + 3,
	.htotal = 320 + 17 + 3 + 12,
	.vdisplay = 240,
	.vsync_start = 240 + 16,
	.vsync_end = 240 + 16 + 3,
	.vtotal = 240 + 16 + 3 + 5,
	.width_mm = 77,
	.height_mm = 57,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static inline struct ipaq_h3900_panel *to_ipaq_h3900_panel(struct drm_panel *panel)
{
	return container_of(panel, struct ipaq_h3900_panel, panel);
}

/*
 * Both sequences and every delay are those of h3900_lcd_set_power() in
 * the handhelds.org 2.6 board code, transcribed rather than
 * approximated. The power-down is not the power-up
 * reversed: LCD_PCI drops first rather than last, and LCD_9V_ON and
 * LCD_5V_ON drop together in one step with no delay between them.
 *
 * These run from the atomic commit tail in task context, so the waits
 * sleep rather than busy-wait the way the historic board code did.
 */
static int ipaq_h3900_panel_enable(struct drm_panel *panel)
{
	struct ipaq_h3900_panel *p = to_ipaq_h3900_panel(panel);

	gpiod_set_value_cansleep(p->lcd_on, 1);
	usleep_range(3000, 4000);
	gpiod_set_value_cansleep(p->lcd_pci, 1);
	usleep_range(7000, 8000);
	gpiod_set_value_cansleep(p->lcd_5v, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(p->lcd_nv, 1);
	usleep_range(7000, 8000);
	gpiod_set_value_cansleep(p->lcd_9v, 1);
	usleep_range(10000, 11000);

	return 0;
}

static int ipaq_h3900_panel_disable(struct drm_panel *panel)
{
	struct ipaq_h3900_panel *p = to_ipaq_h3900_panel(panel);

	gpiod_set_value_cansleep(p->lcd_pci, 0);
	usleep_range(20000, 21000);
	gpiod_set_value_cansleep(p->lcd_nv, 0);
	msleep(181);
	gpiod_set_value_cansleep(p->lcd_9v, 0);
	gpiod_set_value_cansleep(p->lcd_5v, 0);
	usleep_range(11000, 12000);
	gpiod_set_value_cansleep(p->lcd_on, 0);
	usleep_range(10000, 11000);

	return 0;
}

static int ipaq_h3900_panel_get_modes(struct drm_panel *panel,
				      struct drm_connector *connector)
{
	static const u32 bus_format = MEDIA_BUS_FMT_RGB565_1X16;
	struct drm_display_mode *mode;
	int ret;

	mode = drm_mode_duplicate(connector->dev, &ipaq_h3900_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	/*
	 * The glass is 57 x 77 mm portrait, the size given for the whole
	 * h3xxx and h5xxx family (not measured on this unit). The controller
	 * scans it as 320 x 240, so the sizes are 77 wide by 57 high in scan
	 * orientation; the panel orientation property tells userspace how
	 * the glass is mounted.
	 *
	 * The connector takes the size from here, since a panel's modes
	 * carry no size into display_info on their own.
	 *
	 * DE is active high and pixel data is driven on the rising edge of
	 * the pixel clock: the same register dump the mode comes from has
	 * LCCR3 OEP and PCP both clear.
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

static enum drm_panel_orientation ipaq_h3900_panel_get_orientation(struct drm_panel *panel)
{
	struct ipaq_h3900_panel *p = to_ipaq_h3900_panel(panel);

	return p->orientation;
}

/*
 * Only enable and disable: drm_panel_bridge runs
 * drm_panel_enable() after the CRTC is enabled and drm_panel_disable()
 * before it is disabled, which is the order in which this panel's rails
 * have always been sequenced against its controller. prepare and
 * unprepare would invert both halves.
 */
static const struct drm_panel_funcs ipaq_h3900_panel_funcs = {
	.enable = ipaq_h3900_panel_enable,
	.disable = ipaq_h3900_panel_disable,
	.get_modes = ipaq_h3900_panel_get_modes,
	.get_orientation = ipaq_h3900_panel_get_orientation,
};

static int ipaq_h3900_panel_get_rail(struct device *dev, const char *id,
				     struct gpio_desc **out)
{
	struct gpio_desc *gpiod;

	gpiod = devm_gpiod_get(dev, id, GPIOD_OUT_LOW);
	if (IS_ERR(gpiod))
		return dev_err_probe(dev, PTR_ERR(gpiod),
				     "failed to get %s rail\n", id);
	*out = gpiod;

	return 0;
}

static int ipaq_h3900_panel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ipaq_h3900_panel *p;
	int ret;

	p = devm_drm_panel_alloc(dev, struct ipaq_h3900_panel, panel,
				 &ipaq_h3900_panel_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (IS_ERR(p))
		return PTR_ERR(p);

	ret = ipaq_h3900_panel_get_rail(dev, "lcd-on", &p->lcd_on);
	if (ret)
		return ret;
	ret = ipaq_h3900_panel_get_rail(dev, "lcd-pci", &p->lcd_pci);
	if (ret)
		return ret;
	ret = ipaq_h3900_panel_get_rail(dev, "lcd-5v", &p->lcd_5v);
	if (ret)
		return ret;
	ret = ipaq_h3900_panel_get_rail(dev, "lcd-nv", &p->lcd_nv);
	if (ret)
		return ret;
	ret = ipaq_h3900_panel_get_rail(dev, "lcd-9v", &p->lcd_9v);
	if (ret)
		return ret;

	ret = drm_of_get_panel_orientation(dev->of_node, &p->orientation);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read panel orientation\n");

	ret = drm_panel_of_backlight(&p->panel);
	if (ret)
		return ret;

	return devm_drm_panel_add(dev, &p->panel);
}

static const struct of_device_id ipaq_h3900_panel_of_match[] = {
	{ .compatible = "hp,ipaq-h3900-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, ipaq_h3900_panel_of_match);

static struct platform_driver ipaq_h3900_panel_driver = {
	.probe = ipaq_h3900_panel_probe,
	.driver = {
		.name = "panel-ipaq-h3900",
		.of_match_table = ipaq_h3900_panel_of_match,
	},
};
module_platform_driver(ipaq_h3900_panel_driver);

MODULE_AUTHOR("Koen Kooi <koen@dominion.thruhere.net>");
MODULE_DESCRIPTION("HP iPAQ h3900 TFT panel driver");
MODULE_LICENSE("GPL");

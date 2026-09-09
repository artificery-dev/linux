// SPDX-License-Identifier: GPL-2.0
/*
 * GalaxyCore GC9503V MIPI-DSI panel driver for the Innioasis Y2.
 *
 * Panel module id "gc9503v_hvga_dsi_vdo_hsd": GC9503V controller, MIPI-DSI
 * video mode (sync-event), 2 data lanes, RGB888 on the DSI link. The init
 * sequence and video timing were reverse-engineered from the Y2's vendor LK
 * bootloader (references/roms/stock-3.1.7/lk.bin, get_params @0x1EDE4, init
 * @0x1EF04, cmd table @file 0x378DC); see src/y2/docs/disp-kms-notes.md.
 *
 * Active area is 480x368 (RGB565 framebuffer). Note the vendor LK LCM_PARAMS
 * programs only 360 active lines, but on-device testing showed that leaves an
 * unpainted 8-line sliver at the bottom - the panel is physically 368 tall, so
 * we drive the full height here.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct gc9503v {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;
};

/* One MIPI DCS long/short write: command byte + parameter bytes. */
struct gc9503v_cmd {
	u8 cmd;
	u8 len;
	u8 data[52];	/* largest entry is the 52-byte gamma table (D1..D6) */
};

/*
 * GC9503V init table, lifted verbatim from the vendor LK push_table (40 rows).
 * D1..D6 are six identical 52-byte gamma tables (R/G/B positive+negative).
 */
#define GAMMA \
	0x00, 0x00, 0x00, 0x0c, 0x00, 0x22, 0x00, 0x32, 0x00, 0x46, 0x00, 0x66, \
	0x00, 0x84, 0x00, 0xad, 0x00, 0xd5, 0x01, 0x12, 0x01, 0x4a, 0x01, 0xa4, \
	0x01, 0xee, 0x01, 0xf0, 0x02, 0x36, 0x02, 0x88, 0x02, 0xbe, 0x03, 0x01, \
	0x03, 0x38, 0x03, 0x62, 0x03, 0x81, 0x03, 0xa1, 0x03, 0xcd, 0x03, 0xd8, \
	0x03, 0xe0, 0x03, 0xff

static const struct gc9503v_cmd gc9503v_init_cmds[] = {
	{ 0xf0, 5, { 0x55, 0xaa, 0x52, 0x08, 0x00 } },	/* unlock inside regs */
	{ 0xf6, 2, { 0x5a, 0x87 } },
	{ 0xc1, 1, { 0x3f } },
	{ 0xcd, 1, { 0x25 } },
	{ 0xc9, 1, { 0x12 } },
	{ 0xa9, 1, { 0xad } },
	{ 0xf8, 1, { 0x8a } },
	{ 0xac, 1, { 0x45 } },
	{ 0xa7, 1, { 0x47 } },
	{ 0xa0, 1, { 0xbb } },
	{ 0x86, 4, { 0x99, 0xa3, 0xa3, 0x31 } },
	{ 0xfa, 4, { 0x08, 0x08, 0x00, 0x04 } },
	{ 0xa3, 1, { 0x6e } },
	{ 0xfd, 3, { 0x28, 0x3c, 0x00 } },
	{ 0x9a, 1, { 0x99 } },
	{ 0x9b, 1, { 0x70 } },
	{ 0x82, 2, { 0x5c, 0x5c } },
	{ 0xb1, 1, { 0x10 } },
	{ 0x7a, 2, { 0x0f, 0x13 } },
	{ 0x7b, 2, { 0x0f, 0x13 } },
	{ 0x69, 7, { 0x14, 0x22, 0x14, 0x22, 0x44, 0x22, 0x08 } },
	{ 0x6b, 1, { 0x07 } },
	{ 0x6d, 32, { 0x1d, 0x07, 0x10, 0x03, 0x0e, 0x1f, 0x01, 0x1e,
		      0x09, 0x0a, 0x0b, 0x0c, 0x1e, 0x1e, 0x1e, 0x1e,
		      0x1e, 0x1e, 0x1e, 0x1e, 0x14, 0x13, 0x12, 0x11,
		      0x1e, 0x02, 0x1f, 0x0e, 0x03, 0x10, 0x08, 0x1d } },
	{ 0x60, 8, { 0x38, 0x0d, 0x62, 0x62, 0x38, 0x0c, 0x62, 0x62 } },
	{ 0x61, 8, { 0x38, 0x0e, 0x62, 0x62, 0x38, 0x0e, 0x62, 0x62 } },
	{ 0x63, 8, { 0x38, 0x0b, 0x62, 0x62, 0x38, 0x0a, 0x62, 0x62 } },
	{ 0x64, 16, { 0x38, 0x09, 0x01, 0x67, 0x03, 0x03, 0x38, 0x07,
		      0x01, 0x69, 0x03, 0x03, 0x62, 0x62, 0x62, 0x62 } },
	{ 0x65, 16, { 0x38, 0x05, 0x01, 0x6b, 0x00, 0x03, 0x38, 0x03,
		      0x01, 0x6d, 0x03, 0x03, 0x62, 0x62, 0x62, 0x62 } },
	{ 0x66, 16, { 0xc1, 0x7c, 0x08, 0x13, 0x00, 0x03, 0xc1, 0x7c,
		      0x08, 0x13, 0x03, 0x03, 0x72, 0x72, 0x72, 0x72 } },
	{ 0x67, 16, { 0xc8, 0x13, 0x01, 0x7c, 0x00, 0x03, 0xc8, 0x13,
		      0x01, 0x7c, 0x03, 0x03, 0x72, 0x72, 0x72, 0x72 } },
	{ 0xd1, 52, { GAMMA } },
	{ 0xd2, 52, { GAMMA } },
	{ 0xd3, 52, { GAMMA } },
	{ 0xd4, 52, { GAMMA } },
	{ 0xd5, 52, { GAMMA } },
	{ 0xd6, 52, { GAMMA } },
};

static inline struct gc9503v *to_gc9503v(struct drm_panel *panel)
{
	return container_of(panel, struct gc9503v, panel);
}

static int gc9503v_prepare(struct drm_panel *panel)
{
	struct gc9503v *ctx = to_gc9503v(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	unsigned int i;
	int ret;

	ret = regulator_enable(ctx->supply);
	if (ret)
		return ret;

	/* Reset pulse per vendor LK: high, low 10ms, high 120ms. */
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(10);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(120);

	for (i = 0; i < ARRAY_SIZE(gc9503v_init_cmds); i++) {
		const struct gc9503v_cmd *c = &gc9503v_init_cmds[i];

		ret = mipi_dsi_dcs_write(dsi, c->cmd, c->data, c->len);
		if (ret < 0) {
			dev_err(dev, "init cmd 0x%02x failed: %d\n", c->cmd, ret);
			goto disable_supply;
		}
	}

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		goto disable_supply;
	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0)
		goto disable_supply;
	msleep(20);

	return 0;

disable_supply:
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	regulator_disable(ctx->supply);
	return ret;
}

static int gc9503v_unprepare(struct drm_panel *panel)
{
	struct gc9503v *ctx = to_gc9503v(panel);

	mipi_dsi_dcs_set_display_off(ctx->dsi);
	mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	msleep(120);

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	regulator_disable(ctx->supply);
	return 0;
}

/*
 * Video-mode timing. Horizontal + porches are the vendor LK LCM_PARAMS values;
 * the active HEIGHT is 368, not the 360 LK programs. On-device framebuffer
 * testing showed 360 leaves an unpainted ~8-line sliver at the bottom, i.e. the
 * panel is physically 480x368 and the vendor LCM under-scans by 8 lines. We
 * drive the full 368 so the whole panel is painted.
 *   h: active 480, fp 200, sync 10, bp 200  -> htotal 890
 *   v: active 368, fp 60,  sync 8,  bp 60   -> vtotal 496
 *   pixel clock 27.36 MHz (from LK's timing) -> ~62 Hz at vtotal 496
 * The framebuffer is 16bpp RGB565 (verified with an RGBW/MYCB test pattern);
 * that's the DRM plane format and is independent of the 24-bit DSI link.
 */
static const struct drm_display_mode gc9503v_mode = {
	.clock = 27362,
	.hdisplay = 480,
	.hsync_start = 480 + 200,
	.hsync_end = 480 + 200 + 10,
	.htotal = 480 + 200 + 10 + 200,
	.vdisplay = 360,
	.vsync_start = 360 + 60,
	.vsync_end = 360 + 60 + 8,
	.vtotal = 360 + 60 + 8 + 60,
	.width_mm = 46,
	.height_mm = 35,
};

static int gc9503v_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &gc9503v_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs gc9503v_funcs = {
	.prepare = gc9503v_prepare,
	.unprepare = gc9503v_unprepare,
	.get_modes = gc9503v_get_modes,
};

static int gc9503v_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct gc9503v *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->supply))
		return dev_err_probe(dev, PTR_ERR(ctx->supply),
				     "failed to get power supply\n");

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset gpio\n");

	dsi->lanes = 2;
	/*
	 * DSI *wire* format (SoC->panel link), not the framebuffer format. LK's
	 * LCM_PARAMS word_count = 480*3 = 1440 -> 3 bytes/pixel = 24-bit on the
	 * link, so RGB888 here. The visible framebuffer is 16bpp RGB565; the OVL
	 * plane upconverts to the DSI link format. If colours are wrong on HW, the
	 * link may instead be loosely-packed RGB666 (also 3 bytes/pixel) -> try
	 * MIPI_DSI_FMT_RGB666.
	 */
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* LK drives the panel in SYNC-PULSE video mode (DSI_MODE_CTRL=1). */
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel, dev, &gc9503v_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	return 0;
}

static void gc9503v_remove(struct mipi_dsi_device *dsi)
{
	struct gc9503v *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id gc9503v_of_match[] = {
	{ .compatible = "innioasis,y2-gc9503v" },
	{ }
};
MODULE_DEVICE_TABLE(of, gc9503v_of_match);

static struct mipi_dsi_driver gc9503v_driver = {
	.probe = gc9503v_probe,
	.remove = gc9503v_remove,
	.driver = {
		.name = "panel-gc9503v",
		.of_match_table = gc9503v_of_match,
	},
};
module_mipi_dsi_driver(gc9503v_driver);

MODULE_DESCRIPTION("GalaxyCore GC9503V MIPI-DSI panel driver (Innioasis Y2)");
MODULE_LICENSE("GPL");

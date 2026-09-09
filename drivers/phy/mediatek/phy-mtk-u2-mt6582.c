// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6582 USB 2.0 PHY driver (peripheral bring-up).
 *
 * Ported from the vendor device-mode power-on sequence in
 * drivers/misc/mediatek/usb20/mt6582/usb20_phy.c (usb_phy_recover), which
 * writes 8-bit PHY registers at USB_SIF_BASE + 0x800 + offset. The USB clocks
 * (PERI_USB0 / 48M) are already enabled by the bootloader, so we only run the
 * register init here. Registers as a generic PHY consumed by the mainline
 * musb "mediatek,mtk-musb" glue.
 */
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define U2PHY_OFF	0x800		/* PHY regs start 0x800 into USB_SIF */

struct mt6582_u2phy {
	void __iomem *base;		/* USB_SIF base (PHY regs at base+0x800) */
};

static inline void u2_set8(struct mt6582_u2phy *p, u32 off, u8 mask)
{
	void __iomem *a = p->base + U2PHY_OFF + off;

	writeb(readb(a) | mask, a);
}

static inline void u2_clr8(struct mt6582_u2phy *p, u32 off, u8 mask)
{
	void __iomem *a = p->base + U2PHY_OFF + off;

	writeb(readb(a) & ~mask, a);
}

/* usb_phy_recover(): bring the PHY up in device mode. */
static int mt6582_u2phy_power_on(struct phy *phy)
{
	struct mt6582_u2phy *p = phy_get_drvdata(phy);

	udelay(50);
	u2_clr8(p, 0x1d, 0x10);	/* PUPD_BIST_EN = 0 */
	u2_clr8(p, 0x6b, 0x04);	/* force_uart_en = 0 */
	u2_clr8(p, 0x6e, 0x01);	/* RG_UART_EN = 0 */
	u2_clr8(p, 0x6a, 0x04);	/* release force suspendm */
	u2_clr8(p, 0x68, 0x40);	/* RG_DPPULLDOWN = 0 */
	u2_clr8(p, 0x68, 0x80);	/* RG_DMPULLDOWN = 0 */
	u2_clr8(p, 0x68, 0x30);	/* RG_XCVRSEL = 0 */
	u2_clr8(p, 0x68, 0x04);	/* RG_TERMSEL = 0 */
	u2_clr8(p, 0x69, 0x3c);	/* RG_DATAIN[3:0] = 0 */
	u2_clr8(p, 0x6a, 0x10);	/* force_dp_pulldown = 0 */
	u2_clr8(p, 0x6a, 0x20);	/* force_dm_pulldown = 0 */
	u2_clr8(p, 0x6a, 0x08);	/* force_xcversel = 0 */
	u2_clr8(p, 0x6a, 0x02);	/* force_termsel = 0 */
	u2_clr8(p, 0x6a, 0x80);	/* force_datain = 0 */
	u2_clr8(p, 0x1a, 0x80);	/* RG_USB20_BC11_SW_EN = 0 */
	u2_set8(p, 0x1a, 0x10);	/* RG_USB20_OTG_VBUSSCMP_EN = 1 */
	udelay(800);
	/* force enter device mode */
	u2_clr8(p, 0x6c, 0x10);
	u2_set8(p, 0x6c, 0x2e);
	u2_set8(p, 0x6d, 0x3e);
	return 0;
}

/* usb_phy_savecurrent(): suspend the PHY. */
static int mt6582_u2phy_power_off(struct phy *phy)
{
	struct mt6582_u2phy *p = phy_get_drvdata(phy);

	u2_set8(p, 0x6a, 0x04);	/* force suspendm = 1 */
	return 0;
}

static const struct phy_ops mt6582_u2phy_ops = {
	.power_on = mt6582_u2phy_power_on,
	.power_off = mt6582_u2phy_power_off,
	.owner = THIS_MODULE,
};

static int mt6582_u2phy_probe(struct platform_device *pdev)
{
	struct mt6582_u2phy *p;
	struct phy *phy;
	struct phy_provider *provider;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);

	phy = devm_phy_create(&pdev->dev, NULL, &mt6582_u2phy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, p);

	provider = devm_of_phy_provider_register(&pdev->dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id mt6582_u2phy_of_match[] = {
	{ .compatible = "mediatek,mt6582-u2phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, mt6582_u2phy_of_match);

static struct platform_driver mt6582_u2phy_driver = {
	.probe = mt6582_u2phy_probe,
	.driver = {
		.name = "mt6582-u2phy",
		.of_match_table = mt6582_u2phy_of_match,
	},
};
module_platform_driver(mt6582_u2phy_driver);

MODULE_DESCRIPTION("MediaTek MT6582 USB2 PHY driver");
MODULE_LICENSE("GPL");

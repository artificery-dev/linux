// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6582 MFG (Mali-400 MP2 GPU) power-domain provider.
 *
 * There is no mainline MT6582 SCPSYS/clock driver, so this small genpd provider
 * runs the vendor MTCMOS power-up sequence for the MFG (3D GPU) domain and
 * ungates its clock gate, letting the lima driver drive the Mali-400. The clock
 * SOURCE (mux + parent PLL) is left exactly as the preloader/LK configured it
 * (~286-312 MHz, refcount-preset in the vendor clkmgr, i.e. always running); we
 * only toggle the MTCMOS domain and the MFG clock gate, which is all the vendor
 * clkmgr does per GPU power-cycle too.
 *
 * Sequence extracted + cross-validated from the vendor GPL kernel
 * (android-mediatek-sprout-3.4): mt_spm_mtcmos.c spm_mtcmos_ctrl_mfg(),
 * mt_clkmgr.c CG_MFG group, gpu/mt6582 platform_pmm.c power_mode_change().
 */
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>

/* SPM, mapped at 0x10006000 */
#define SPM_POWERON_CONFIG_EN	0x0000
#define  SPM_PROJECT_CODE	(0xb16 << 16)
#define  SPM_REGWR_EN		BIT(0)
#define SPM_MFG_PWR_CON		0x0214
#define SPM_PWR_STATUS		0x060c
#define SPM_PWR_STATUS_S	0x0610
/* SPM_MFG_PWR_CON fields */
#define PWR_RST_B		BIT(0)
#define PWR_ISO			BIT(1)
#define PWR_ON			BIT(2)
#define PWR_ON_2ND		BIT(3)
#define PWR_CLK_DIS		BIT(4)
#define SRAM_PDN		GENMASK(11, 8)
#define MFG_SRAM_ACK		BIT(12)
#define MFG_PWR_STA		BIT(4)		/* MFG bit in PWR_STATUS / _S */

/* MFG clock gate, mapped at 0x13000000 (G3D_CONFIG). 1-write SET/CLR. */
#define MFG_CG_SET		0x0004		/* write 1 = gate  (disable) */
#define MFG_CG_CLR		0x0008		/* write 1 = ungate (enable) */
#define MFG_CG_G3D		BIT(0)

/* DISP clock gate, mapped at 0x14000100. SMI_COMMON feeds the GPU M4U path. */
#define DISP_CG_CLR0		0x0008
#define DISP_CG_SMI_COMMON	BIT(0)

struct mt6582_mfg {
	struct generic_pm_domain genpd;
	void __iomem *spm;
	void __iomem *mfgcg;
	void __iomem *dispcg;		/* optional */
};

static int mt6582_mfg_power_on(struct generic_pm_domain *genpd)
{
	struct mt6582_mfg *mfg = container_of(genpd, struct mt6582_mfg, genpd);
	void __iomem *spm = mfg->spm;
	u32 v;
	int ret;

	/* Unlock SPM register writes (harmless if LK already unlocked it). */
	writel(SPM_PROJECT_CODE | SPM_REGWR_EN, spm + SPM_POWERON_CONFIG_EN);

	/* SMI_COMMON clock for the GPU's memory path (its power domain, SYS_DIS,
	 * is already up - the display/framebuffer is live). Ungate only. */
	if (mfg->dispcg)
		writel(DISP_CG_SMI_COMMON, mfg->dispcg + DISP_CG_CLR0);

	/* MTCMOS power-up (spm_mtcmos_ctrl_mfg, STA_POWER_ON). */
	writel(readl(spm + SPM_MFG_PWR_CON) | PWR_ON,     spm + SPM_MFG_PWR_CON);
	writel(readl(spm + SPM_MFG_PWR_CON) | PWR_ON_2ND, spm + SPM_MFG_PWR_CON);

	ret = readl_poll_timeout(spm + SPM_PWR_STATUS, v, v & MFG_PWR_STA, 10, 10000);
	if (!ret)
		ret = readl_poll_timeout(spm + SPM_PWR_STATUS_S, v,
					 v & MFG_PWR_STA, 10, 10000);
	if (ret) {
		dev_err(genpd->dev.parent, "MFG power ack timeout\n");
		return ret;
	}

	writel(readl(spm + SPM_MFG_PWR_CON) & ~PWR_CLK_DIS, spm + SPM_MFG_PWR_CON);
	writel(readl(spm + SPM_MFG_PWR_CON) & ~PWR_ISO,     spm + SPM_MFG_PWR_CON);
	writel(readl(spm + SPM_MFG_PWR_CON) |  PWR_RST_B,   spm + SPM_MFG_PWR_CON);
	writel(readl(spm + SPM_MFG_PWR_CON) & ~SRAM_PDN,    spm + SPM_MFG_PWR_CON);

	ret = readl_poll_timeout(spm + SPM_MFG_PWR_CON, v, !(v & MFG_SRAM_ACK),
				 10, 10000);
	if (ret) {
		dev_err(genpd->dev.parent, "MFG SRAM ack timeout\n");
		return ret;
	}

	/* Ungate the GPU clock. */
	writel(MFG_CG_G3D, mfg->mfgcg + MFG_CG_CLR);
	return 0;
}

static int mt6582_mfg_power_off(struct generic_pm_domain *genpd)
{
	struct mt6582_mfg *mfg = container_of(genpd, struct mt6582_mfg, genpd);

	/*
	 * Gate the GPU clock (the dominant, dynamic power cost) but leave the
	 * MTCMOS domain up. The domain-off handshake is trickier and unneeded
	 * for correctness; keeping the well-isolated MFG rail on costs only
	 * static leakage while the panel is on anyway.
	 */
	writel(MFG_CG_G3D, mfg->mfgcg + MFG_CG_SET);
	return 0;
}

static int mt6582_mfg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6582_mfg *mfg;
	int ret;

	mfg = devm_kzalloc(dev, sizeof(*mfg), GFP_KERNEL);
	if (!mfg)
		return -ENOMEM;

	mfg->spm = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mfg->spm))
		return PTR_ERR(mfg->spm);
	mfg->mfgcg = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(mfg->mfgcg))
		return PTR_ERR(mfg->mfgcg);
	mfg->dispcg = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(mfg->dispcg))
		mfg->dispcg = NULL;		/* SMI ungate is best-effort */

	mfg->genpd.name = "mfg";
	mfg->genpd.power_on = mt6582_mfg_power_on;
	mfg->genpd.power_off = mt6582_mfg_power_off;

	/* Start powered off; lima's runtime PM turns it on when it probes. */
	ret = pm_genpd_init(&mfg->genpd, NULL, true);
	if (ret)
		return ret;

	ret = of_genpd_add_provider_simple(dev->of_node, &mfg->genpd);
	if (ret) {
		pm_genpd_remove(&mfg->genpd);
		return ret;
	}

	dev_info(dev, "MT6582 MFG (Mali-400) power domain registered\n");
	return 0;
}

static const struct of_device_id mt6582_mfg_of_match[] = {
	{ .compatible = "mediatek,mt6582-mfg-power" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6582_mfg_of_match);

static struct platform_driver mt6582_mfg_driver = {
	.probe = mt6582_mfg_probe,
	.driver = {
		.name = "mt6582-mfg-power",
		.of_match_table = mt6582_mfg_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(mt6582_mfg_driver);

MODULE_DESCRIPTION("MediaTek MT6582 MFG (Mali-400 GPU) power domain");
MODULE_LICENSE("GPL");

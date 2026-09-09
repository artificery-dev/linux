// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6582 system power manager.
 *
 * This intentionally implements only the vendor kernel's boot-time SPM
 * setup and its "normal v2" PCM program.  Suspend and deep-idle need larger
 * PCM images and coordination with clocks, IRQs and the UART, so they belong
 * in later changes once the always-on path is proven.
 *
 * Sequence and PCM image are from the MT6582 vendor kernel mt_spm.c
 * (normal v2, dated 2013-04-26), checked against a stock-kernel MMIO trace.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define SPM_POWERON_CONFIG_SET		0x0000
#define  SPM_PROJECT_CODE		0x0b16
#define  SPM_REGWR_EN			BIT(0)
#define SPM_POWER_ON_VAL0		0x0010
#define SPM_POWER_ON_VAL1		0x0014

#define SPM_PCM_CON0			0x0310
#define  PCM_KICK			BIT(0)
#define  IM_KICK			BIT(1)
#define  IM_SLEEP_DVS			BIT(3)
#define  PCM_SW_RESET			BIT(15)
#define  CON0_CFG_KEY			(SPM_PROJECT_CODE << 16)
#define SPM_PCM_CON1			0x0314
#define  MIF_APBEN			BIT(3)
#define  IM_NONRP_EN			BIT(6)
#define  SPM_SRAM_SLP_B		BIT(10)
#define  SPM_SRAM_ISO_B		BIT(11)
#define  CON1_CFG_KEY			(SPM_PROJECT_CODE << 16)
#define SPM_PCM_IM_PTR			0x0318
#define SPM_PCM_IM_LEN			0x031c
#define SPM_PCM_PWR_IO_EN		0x0358
#define SPM_PCM_REG13_DATA		0x03b4
#define SPM_PCM_FSM_STA			0x03c4
#define SPM_PCM_SW_INT_CLEAR		0x03e4
#define  PCM_SW_INT0			BIT(0)
#define  PCM_SW_INT_ALL			GENMASK(3, 0)

#define SPM_CLK_CON			0x0400
#define  CXO32K_RM_EN_MD		BIT(9)
#define SPM_AP_STANBY_CON		0x0608
#define SPM_SLEEP_WAKEUP_EVENT_MASK	0x0810
#define  WAKE_SRC_THERM			BIT(21)
#define SPM_SLEEP_ISR_MASK		0x0900
#define SPM_SLEEP_ISR_STATUS		0x0904
#define  ISR_TWAM			BIT(2)
#define  ISR_PCM_RETURN			BIT(3)
#define  ISR_PCM_IRQ0			BIT(8)
#define  ISR_PCM_IRQ_AUX		GENMASK(11, 9)
#define  ISR_MASK_ALL_EXCEPT_TWAM	(ISR_PCM_IRQ_AUX | ISR_PCM_IRQ0 | \
					 ISR_PCM_RETURN)
#define  ISR_MASK_ALL			(ISR_MASK_ALL_EXCEPT_TWAM | ISR_TWAM)
#define  ISR_CLEAR_ALL			(ISR_PCM_RETURN | ISR_TWAM)
#define SPM_PCM_SRC_REQ			0x0b04

struct mt6582_spm {
	struct device *dev;
	void __iomem *base;
	spinlock_t lock;
	u32 pcm_phys;
};

/* PCM code for normal mode (v2, 2013-04-26). */
static const u32 mt6582_pcm_normal[] __aligned(4) = {
	0x1840001f, 0x00000001, 0x1b00001f, 0x00202000,
	0x1b80001f, 0x80001000, 0x8880000c, 0x00200000,
	0xd80001e2, 0x17c07c1f, 0xe8208000, 0x100063e0,
	0x00000002, 0x1b80001f, 0x00001000, 0x809c840d,
	0xd8200042, 0x17c07c1f, 0xa1d78407, 0x1890001f,
	0x10006014, 0x18c0001f, 0x10006014, 0xa0978402,
	0xe0c00002, 0x1b80001f, 0x00001000, 0xf0000000,
};

static inline u32 spm_read(struct mt6582_spm *spm, u32 reg)
{
	return readl(spm->base + reg);
}

static inline void spm_write(struct mt6582_spm *spm, u32 reg, u32 val)
{
	writel(val, spm->base + reg);
}

static void mt6582_spm_start_normal(struct mt6582_spm *spm)
{
	unsigned long flags;

	spin_lock_irqsave(&spm->lock, flags);

	/* Reset PCM, then disable instruction-memory non-replace mode. */
	spm_write(spm, SPM_PCM_CON0, CON0_CFG_KEY | PCM_SW_RESET);
	spm_write(spm, SPM_PCM_CON0, CON0_CFG_KEY);
	spm_write(spm, SPM_PCM_CON1, CON1_CFG_KEY | SPM_SRAM_ISO_B |
		  SPM_SRAM_SLP_B | MIF_APBEN);

	spm_write(spm, SPM_PCM_IM_PTR, spm->pcm_phys);
	spm_write(spm, SPM_PCM_IM_LEN, ARRAY_SIZE(mt6582_pcm_normal) - 1);
	spm_write(spm, SPM_SLEEP_WAKEUP_EVENT_MASK, ~WAKE_SRC_THERM);

	/* The falling edge of both kick bits starts the IM load and PCM. */
	spm_write(spm, SPM_PCM_CON0, CON0_CFG_KEY | IM_KICK | PCM_KICK);
	spm_write(spm, SPM_PCM_CON0, CON0_CFG_KEY);
	readl(spm->base + SPM_PCM_CON0);

	spin_unlock_irqrestore(&spm->lock, flags);
}

static irqreturn_t mt6582_spm_irq(int irq, void *data)
{
	struct mt6582_spm *spm = data;
	unsigned long flags;
	u32 isr;

	spin_lock_irqsave(&spm->lock, flags);
	isr = spm_read(spm, SPM_SLEEP_ISR_STATUS);
	spm_write(spm, SPM_SLEEP_ISR_MASK,
		  spm_read(spm, SPM_SLEEP_ISR_MASK) |
		  ISR_MASK_ALL_EXCEPT_TWAM);
	spm_write(spm, SPM_SLEEP_ISR_STATUS, isr);
	if (isr & ISR_TWAM)
		udelay(100); /* TWAM status needs three monitor-clock cycles. */
	spm_write(spm, SPM_PCM_SW_INT_CLEAR, PCM_SW_INT0);
	spin_unlock_irqrestore(&spm->lock, flags);

	dev_warn_ratelimited(spm->dev, "interrupt status %#x\n", isr);
	return IRQ_HANDLED;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mt6582_spm *spm = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += sysfs_emit_at(buf, len, "power_on_val0:        %08x\n",
			     spm_read(spm, SPM_POWER_ON_VAL0));
	len += sysfs_emit_at(buf, len, "power_on_val1:        %08x\n",
			     spm_read(spm, SPM_POWER_ON_VAL1));
	len += sysfs_emit_at(buf, len, "pcm_con0:             %08x\n",
			     spm_read(spm, SPM_PCM_CON0));
	len += sysfs_emit_at(buf, len, "pcm_con1:             %08x\n",
			     spm_read(spm, SPM_PCM_CON1));
	len += sysfs_emit_at(buf, len, "pcm_im_ptr:           %08x\n",
			     spm_read(spm, SPM_PCM_IM_PTR));
	len += sysfs_emit_at(buf, len, "pcm_im_len:           %08x\n",
			     spm_read(spm, SPM_PCM_IM_LEN));
	len += sysfs_emit_at(buf, len, "pcm_fsm_sta:          %08x\n",
			     spm_read(spm, SPM_PCM_FSM_STA));
	len += sysfs_emit_at(buf, len, "pcm_reg13_data:       %08x\n",
			     spm_read(spm, SPM_PCM_REG13_DATA));
	len += sysfs_emit_at(buf, len, "clk_con:              %08x\n",
			     spm_read(spm, SPM_CLK_CON));
	len += sysfs_emit_at(buf, len, "ap_standby_con:       %08x\n",
			     spm_read(spm, SPM_AP_STANBY_CON));
	len += sysfs_emit_at(buf, len, "wakeup_event_mask:    %08x\n",
			     spm_read(spm, SPM_SLEEP_WAKEUP_EVENT_MASK));
	len += sysfs_emit_at(buf, len, "sleep_isr_mask:       %08x\n",
			     spm_read(spm, SPM_SLEEP_ISR_MASK));
	len += sysfs_emit_at(buf, len, "sleep_isr_status:     %08x\n",
			     spm_read(spm, SPM_SLEEP_ISR_STATUS));
	len += sysfs_emit_at(buf, len, "pcm_src_req:          %08x\n",
			     spm_read(spm, SPM_PCM_SRC_REQ));

	return len;
}
static DEVICE_ATTR_RO(state);

static struct attribute *mt6582_spm_attrs[] = {
	&dev_attr_state.attr,
	NULL,
};

static const struct attribute_group mt6582_spm_group = {
	.attrs = mt6582_spm_attrs,
};

static int mt6582_spm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6582_spm *spm;
	struct resource *res;
	phys_addr_t pcm_phys;
	int irq;
	int ret;

	spm = devm_kzalloc(dev, sizeof(*spm), GFP_KERNEL);
	if (!spm)
		return -ENOMEM;
	spm->dev = dev;

	/* SPM is shared with the MFG and connectivity power-domain drivers. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	spm->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!spm->base)
		return -ENOMEM;

	spin_lock_init(&spm->lock);
	platform_set_drvdata(pdev, spm);

	pcm_phys = virt_to_phys(mt6582_pcm_normal);
	if (upper_32_bits(pcm_phys)) {
		dev_err(dev, "PCM image is above the 32-bit address window\n");
		return -ERANGE;
	}
	spm->pcm_phys = lower_32_bits(pcm_phys);

	/* Exact vendor mt_spm.c module initialization sequence. */
	spm_write(spm, SPM_POWERON_CONFIG_SET,
		  (SPM_PROJECT_CODE << 16) | SPM_REGWR_EN);
	spm_write(spm, SPM_POWER_ON_VAL0, 0);
	spm_write(spm, SPM_POWER_ON_VAL1, 0x00015820);
	spm_write(spm, SPM_PCM_PWR_IO_EN, 0);

	spm_write(spm, SPM_PCM_CON0, CON0_CFG_KEY | PCM_SW_RESET);
	spm_write(spm, SPM_PCM_CON0, CON0_CFG_KEY);
	spm_write(spm, SPM_PCM_CON0, CON0_CFG_KEY | IM_SLEEP_DVS);
	spm_write(spm, SPM_PCM_CON1, CON1_CFG_KEY | SPM_SRAM_ISO_B |
		  SPM_SRAM_SLP_B | IM_NONRP_EN | MIF_APBEN);
	spm_write(spm, SPM_PCM_IM_PTR, 0);
	spm_write(spm, SPM_PCM_IM_LEN, 0);

	spm_write(spm, SPM_CLK_CON, CXO32K_RM_EN_MD);
	spm_write(spm, SPM_PCM_SRC_REQ, BIT(1));
	spm_write(spm, SPM_SLEEP_WAKEUP_EVENT_MASK, U32_MAX);
	spm_write(spm, SPM_SLEEP_ISR_MASK, ISR_MASK_ALL);
	spm_write(spm, SPM_SLEEP_ISR_STATUS, ISR_CLEAR_ALL);
	spm_write(spm, SPM_PCM_SW_INT_CLEAR, PCM_SW_INT_ALL);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, mt6582_spm_irq,
			       IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND,
			       dev_name(dev), spm);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request SPM IRQ\n");

	mt6582_spm_start_normal(spm);

	ret = devm_device_add_group(dev, &mt6582_spm_group);
	if (ret)
		return ret;

	dev_info(dev, "normal PCM running at %#08x (%zu words)\n",
		 spm->pcm_phys, ARRAY_SIZE(mt6582_pcm_normal));
	return 0;
}

static const struct of_device_id mt6582_spm_of_match[] = {
	{ .compatible = "mediatek,mt6582-spm" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6582_spm_of_match);

static struct platform_driver mt6582_spm_driver = {
	.probe = mt6582_spm_probe,
	.driver = {
		.name = "mt6582-spm",
		.of_match_table = mt6582_spm_of_match,
		.suppress_bind_attrs = true,
	},
};

static int __init mt6582_spm_init(void)
{
	return platform_driver_register(&mt6582_spm_driver);
}
subsys_initcall(mt6582_spm_init);

static void __exit mt6582_spm_exit(void)
{
	platform_driver_unregister(&mt6582_spm_driver);
}
module_exit(mt6582_spm_exit);

MODULE_DESCRIPTION("MediaTek MT6582 system power manager");
MODULE_LICENSE("GPL");

// SPDX-License-Identifier: GPL-2.0
/*
 * MT6582 CONSYS WiFi RAM-code loader (bring-up probe).
 *
 * The stock system turned WiFi on and off at every boot, which loads the
 * WiFi RAM code into the connectivity MCU and starts it. This is the vendor
 * gen2 WLAN driver's wlanAdapterStart() download path, and nothing else:
 * take driver ownership of the WiFi host interface (the AHB "HIF" block at
 * 0x180f0000, an MT6628-style register file), push the divided firmware
 * image through the TX data port as INIT_CMD_ID_DOWNLOAD_BUF commands with
 * acknowledgements read back from the RX port, send WIFI_START and wait for
 * the ready bit. It registers with WMT as the WLAN driver, so
 * mtk_wcn_wmt_func_on(WMTDRV_TYPE_WIFI) runs it; echo 1 to the "load"
 * module parameter to trigger that. The point is to find out whether the
 * WiFi firmware's start-up leaves the shared 2.4 GHz front end in a state
 * the Bluetooth firmware alone does not reach.
 */
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "osal_typedef.h"
#include "osal.h"
#include "wmt_exp.h"
#include "consys_plat.h"
#include "mtk_wcn_consys_hw.h"

#define HIF_BASE		0x180f0000
#define HIF_LEN			0x100
#define MCR_WCIR		0x00
#define  WCIR_WLAN_READY	BIT(21)
#define MCR_WHLPCR		0x04
#define  WHLPCR_FW_OWN_REQ_CLR	BIT(9)
#define  WHLPCR_FW_OWN_REQ_SET	BIT(8)
#define MCR_WHCR		0x0c
#define MCR_WHISR		0x10
#define MCR_WHIER		0x14
#define  WHIER_DEFAULT		(BIT(0) | BIT(1) | BIT(2) | BIT(3) | GENMASK(31, 8))
#define MCR_WASR		0x18
#define MCR_WTSR0		0x20
#define MCR_WTSR1		0x24
#define MCR_WTDR0		0x28
#define MCR_WTDR1		0x2c
#define MCR_WRDR0		0x30
#define MCR_WRDR1		0x34
#define MCR_D2HRM0R		0x40
#define MCR_D2HRM1R		0x44
#define MCR_WRPLR		0x50
#define MCR_HSTCR		0x58
#define  HSTCR_BURST_4DW	(1 << 24)
#define  HSTCR_TARGET(t)	((t) << 20)
#define  HSTCR_CNT_MASK		GENMASK(19, 2)
#define  HIF_TARGET_TXD0	0
#define  HIF_TARGET_RXD0	2

#define INIT_CMD_ID_DOWNLOAD_BUF	1
#define INIT_CMD_ID_WIFI_START		2
#define INIT_EVENT_ID_CMD_RESULT	1
#define DOWNLOAD_BUF_ENCRYPTION_MODE	BIT(0)
#define DOWNLOAD_BUF_NO_CRC_CHECKING	BIT(30)
#define DOWNLOAD_BUF_ACK_OPTION		BIT(31)
#define CMD_PKT_SIZE_FOR_IMAGE		2048
#define TX_CREDITS_TC0			8

struct init_hif_tx_header {		/* INIT_HIF_TX_HEADER_T + INIT_WIFI_CMD_T */
	__le16 tx_byte_count;
	u8 ether_type_offset;
	u8 cs_flags;
	u8 cid;
	u8 seq;
	__le16 reserved;
} __packed;

struct init_cmd_download_buf {
	__le32 address;
	__le32 length;
	__le32 crc32;
	__le32 data_mode;
} __packed;

struct init_cmd_wifi_start {
	__le32 override;
	__le32 address;
} __packed;

struct init_wifi_event {
	__le16 rx_byte_count;
	u8 eid;
	u8 seq;
	u8 status;
	u8 reserved[3];
} __packed;

struct fwdl_section {
	__le32 offset;
	__le32 reserved;
	__le32 length;
	__le32 dest;
} __packed;

struct fwdl_header {
	__le32 signature;		/* 'M','T','K','W' */
	__le32 crc;			/* over everything from num_entries on */
	__le32 num_entries;
	__le32 reserved;
	struct fwdl_section section[];
} __packed;

static void __iomem *hif;
static u8 tx_credits;
static u8 cmd_seq;
static bool loaded;
static bool wifi_paldo_on;
/* The vendor builds with CFG_ENABLE_FW_DOWNLOAD_ACK and CFG_ENABLE_FW_ENCRYPTION,
 * i.e. ACK | ENCRYPTION_MODE. Tunable while this is being brought up. */
static uint data_mode = DOWNLOAD_BUF_ACK_OPTION | DOWNLOAD_BUF_ENCRYPTION_MODE;
module_param(data_mode, uint, 0644);
MODULE_PARM_DESC(data_mode, "u4DataMode of INIT_CMD_ID_DOWNLOAD_BUF");

static u32 hif_rd(u32 off) { return readl(hif + off); }
static void hif_wr(u32 off, u32 v) { writel(v, hif + off); }

static u32 mtk_crc32(const u8 *buf, u32 len)
{
	/* wlanCRC32(): the zlib CRC-32 (init 0xffffffff, final invert) */
	return crc32_le(~0U, buf, len) ^ ~0U;
}

/* The "HIF 92B" workaround the vendor does before every port transfer. */
static void hif_enhance_conf(u32 target, u32 bytes)
{
	u32 cnt = (bytes & 3) ? bytes + 4 : bytes;

	(void)hif_rd(MCR_WHIER);
	(void)hif_rd(MCR_HSTCR);
	hif_wr(MCR_HSTCR, HSTCR_BURST_4DW | HSTCR_TARGET(target) | (cnt & HSTCR_CNT_MASK));
}

static void hif_port_write(u32 port, u32 target, const u8 *buf, u32 len)
{
	u32 words = (len + 3) / 4, i;
	const u32 *w = (const u32 *)buf;

	hif_enhance_conf(target, len);
	for (i = 0; i < words; i++)
		hif_wr(port, w[i]);
}

static void hif_port_read(u32 port, u32 target, u8 *buf, u32 len)
{
	u32 words = (len + 3) / 4, i;
	u32 *w = (u32 *)buf;

	hif_enhance_conf(target, len);
	for (i = 0; i < words; i++)
		w[i] = hif_rd(port);
}

static int hif_driver_own(void)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(8192);
	u32 i = 0, v;

	while (1) {
		v = hif_rd(MCR_WHLPCR);
		if (v & WHLPCR_FW_OWN_REQ_SET)
			return 0;
		if (time_after(jiffies, deadline)) {
			pr_err("consys-wifi: driver own timeout, WHLPCR 0x%x D2HRM0R 0x%x\n",
			       v, hif_rd(MCR_D2HRM0R));
			return -ETIMEDOUT;
		}
		if ((i & 255) == 0)
			hif_wr(MCR_WHLPCR, WHLPCR_FW_OWN_REQ_CLR);
		msleep(1);
		i++;
	}
}

/* nicTxAcquireResource / nicTxPollingResource on TC0 */
static int hif_tx_credit(void)
{
	int tries = 256;

	while (tries--) {
		if (tx_credits) {
			tx_credits--;
			return 0;
		}
		{
			u32 rel[2] = { hif_rd(MCR_WTSR0), hif_rd(MCR_WTSR1) };
			u8 *cnt = (u8 *)rel;

			if (rel[0] | rel[1])
				tx_credits += cnt[0];	/* TC0 released count */
		}
		if (!tx_credits)
			msleep(50);
	}
	pr_err("consys-wifi: no TX credit\n");
	return -ETIMEDOUT;
}

/* nicRxWaitResponse on port 0 */
static int hif_rx_response(u8 *buf, u32 max, u32 *got)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(3000);
	u32 len;

	while (1) {
		len = hif_rd(MCR_WRPLR) & 0xffff;
		if (len)
			break;
		if (time_after(jiffies, deadline))
			return -ETIMEDOUT;
		usleep_range(50, 100);
	}
	if (len > max) {
		pr_err("consys-wifi: response %u bytes exceeds %u\n", len, max);
		return -E2BIG;
	}
	hif_port_read(MCR_WRDR0, HIF_TARGET_RXD0, buf, len);
	*got = len;
	return 0;
}

static int hif_send_cmd(u8 *pkt, u32 len)
{
	struct init_hif_tx_header *h = (struct init_hif_tx_header *)pkt;
	u32 total = ALIGN(len, 4);
	int ret;

	h->tx_byte_count = cpu_to_le16(total & 0xfff);
	h->ether_type_offset = 0;
	h->cs_flags = 0;
	h->seq = ++cmd_seq;
	ret = hif_tx_credit();
	if (ret)
		return ret;
	/* one zero dword terminates TX aggregation */
	*(u32 *)(pkt + total) = 0;
	hif_port_write(MCR_WTDR0, HIF_TARGET_TXD0, pkt, total);
	return 0;
}

static int download_chunk(u8 *pkt, u32 addr, const u8 *data, u32 len)
{
	struct init_hif_tx_header *h = (struct init_hif_tx_header *)pkt;
	struct init_cmd_download_buf *d = (struct init_cmd_download_buf *)(pkt + sizeof(*h));
	u8 rsp[16];
	u32 got;
	int ret;

	h->cid = INIT_CMD_ID_DOWNLOAD_BUF;
	d->address = cpu_to_le32(addr);
	d->length = cpu_to_le32(len);
	d->crc32 = cpu_to_le32(mtk_crc32(data, len));
	d->data_mode = cpu_to_le32(data_mode);
	memcpy(pkt + sizeof(*h) + sizeof(*d), data, len);
	ret = hif_send_cmd(pkt, sizeof(*h) + sizeof(*d) + len);
	if (ret)
		return ret;
	if (!(data_mode & DOWNLOAD_BUF_ACK_OPTION))
		return 0;
	ret = hif_rx_response(rsp, sizeof(rsp), &got);
	if (ret) {
		pr_err("consys-wifi: no ack for chunk at 0x%x (%d)\n", addr, ret);
		return ret;
	}
	{
		struct init_wifi_event *e = (struct init_wifi_event *)rsp;

		if (e->eid != INIT_EVENT_ID_CMD_RESULT || e->seq != cmd_seq || e->status) {
			pr_err("consys-wifi: chunk at 0x%x rejected: eid %u seq %u/%u status %u\n",
			       addr, e->eid, e->seq, cmd_seq, e->status);
			return -EIO;
		}
	}
	return 0;
}

static int consys_wifi_download(const struct firmware *fw)
{
	const struct fwdl_header *hdr = (const void *)fw->data;
	u8 *pkt;
	u32 i, n, crc;
	int ret = 0;

	if (fw->size < sizeof(*hdr) || le32_to_cpu(hdr->signature) != 0x574b544d) {
		pr_err("consys-wifi: RAM code has no MTKW header\n");
		return -EINVAL;
	}
	n = le32_to_cpu(hdr->num_entries);
	crc = mtk_crc32(fw->data + 8, fw->size - 8);
	pr_info("consys-wifi: RAM code %zu bytes, %u sections, header crc %s\n",
		fw->size, n, crc == le32_to_cpu(hdr->crc) ? "ok" : "MISMATCH");
	pkt = kmalloc(sizeof(struct init_hif_tx_header) + sizeof(struct init_cmd_download_buf) +
		      CMD_PKT_SIZE_FOR_IMAGE + 8, GFP_KERNEL);
	if (!pkt)
		return -ENOMEM;
	for (i = 0; i < n && !ret; i++) {
		u32 off = le32_to_cpu(hdr->section[i].offset);
		u32 len = le32_to_cpu(hdr->section[i].length);
		u32 dest = le32_to_cpu(hdr->section[i].dest);
		u32 j;

		pr_info("consys-wifi: section %u: %u bytes at file 0x%x -> 0x%08x\n", i, len, off, dest);
		if (off + len > fw->size) {
			ret = -EINVAL;
			break;
		}
		for (j = 0; j < len && !ret; j += CMD_PKT_SIZE_FOR_IMAGE) {
			u32 chunk = min_t(u32, CMD_PKT_SIZE_FOR_IMAGE, len - j);

			ret = download_chunk(pkt, dest + j, fw->data + off + j, chunk);
		}
	}
	if (!ret) {
		struct init_hif_tx_header *h = (struct init_hif_tx_header *)pkt;
		struct init_cmd_wifi_start *s = (struct init_cmd_wifi_start *)(pkt + sizeof(*h));

		memset(pkt, 0, sizeof(*h) + sizeof(*s));
		h->cid = INIT_CMD_ID_WIFI_START;
		s->override = 0;
		s->address = 0;
		ret = hif_send_cmd(pkt, sizeof(*h) + sizeof(*s));
		pr_info("consys-wifi: WIFI_START sent (%d)\n", ret);
	}
	kfree(pkt);
	return ret;
}

/* WMT calls this as the WLAN driver's probe when WiFi is turned on. */
static int consys_wifi_probe(void)
{
	const struct firmware *fw;
	u32 v;
	int i, ret;

	if (!consys_plat.ready)
		return -ENODEV;

	/*
	 * The vendor AHB bus probe enables VCN33_WIFI before entering
	 * wlanAdapterStart().  WMT deliberately does not own this rail for WiFi
	 * (unlike the Bluetooth PALDO), so omitting this step lets the RAM code
	 * download successfully but makes its PHY initialization assert in
	 * wifi/mgmt/mt6582/rlm_phy.c before WLAN_READY is raised.
	 */
	ret = mtk_wcn_consys_hw_wifi_paldo_ctrl(1);
	if (ret)
		return ret;
	wifi_paldo_on = true;

	hif = ioremap(HIF_BASE, HIF_LEN);
	if (!hif) {
		ret = -ENOMEM;
		goto out_power;
	}
	v = hif_rd(MCR_WCIR);
	pr_info("consys-wifi: WCIR 0x%08x (chip 0x%04x rev %u), WHLPCR 0x%x\n",
		v, v & 0xffff, (v >> 16) & 0xf, hif_rd(MCR_WHLPCR));
	ret = hif_driver_own();
	if (ret)
		goto out;
	hif_wr(MCR_WHIER, WHIER_DEFAULT);
	tx_credits = TX_CREDITS_TC0;
	cmd_seq = 0;
	ret = request_firmware(&fw, "mediatek/mt6582/WIFI_RAM_CODE_MT6582", consys_plat.dev);
	if (ret) {
		pr_err("consys-wifi: RAM code missing (%d)\n", ret);
		goto out;
	}
	ret = consys_wifi_download(fw);
	release_firmware(fw);
	if (ret)
		goto out;
	for (i = 0; i < 512; i++) {
		v = hif_rd(MCR_WCIR);
		if (v & WCIR_WLAN_READY)
			break;
		msleep(10);
	}
	if (!(v & WCIR_WLAN_READY)) {
		pr_err("consys-wifi: ready bit never set, WCIR 0x%08x mailbox 0x%08x/0x%08x\n",
		       v, hif_rd(MCR_D2HRM0R), hif_rd(MCR_D2HRM1R));
		ret = -ETIMEDOUT;
		goto out;
	}
	pr_info("consys-wifi: WiFi firmware running (WCIR 0x%08x) after %d ms\n", v, i * 10);
	loaded = true;
	return 0;
out:
	iounmap(hif);
	hif = NULL;
out_power:
	if (wifi_paldo_on) {
		mtk_wcn_consys_hw_wifi_paldo_ctrl(0);
		wifi_paldo_on = false;
	}
	return ret;
}

static int consys_wifi_remove(void)
{
	if (hif) {
		iounmap(hif);
		hif = NULL;
	}
	if (wifi_paldo_on) {
		mtk_wcn_consys_hw_wifi_paldo_ctrl(0);
		wifi_paldo_on = false;
	}
	loaded = false;
	return 0;
}

static int consys_wifi_bus_cnt(void) { return 0; }

extern INT32 mtk_wcn_wmt_wlan_reg(P_MTK_WCN_WMT_WLAN_CB_INFO pWmtWlanCbInfo);

static int load_set(const char *val, const struct kernel_param *kp)
{
	bool on;
	int ret = kstrtobool(val, &on);

	if (ret)
		return ret;
	if (on) {
		ret = mtk_wcn_wmt_func_on(WMTDRV_TYPE_WIFI) == MTK_WCN_BOOL_TRUE ? 0 : -EIO;
		pr_info("consys-wifi: WMT WiFi function on -> %d (firmware %s)\n", ret,
			loaded ? "running" : "not running");
	} else {
		ret = mtk_wcn_wmt_func_off(WMTDRV_TYPE_WIFI) == MTK_WCN_BOOL_TRUE ? 0 : -EIO;
		pr_info("consys-wifi: WMT WiFi function off -> %d\n", ret);
	}
	return ret;
}

static int load_get(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "%d\n", loaded);
}

static const struct kernel_param_ops load_ops = { .set = load_set, .get = load_get };
module_param_cb(load, &load_ops, NULL, 0644);
MODULE_PARM_DESC(load, "1: turn the WMT WiFi function on and load the RAM code; 0: off");

static int __init consys_wifi_init(void)
{
	MTK_WCN_WMT_WLAN_CB_INFO cb = {
		.wlan_probe_cb = consys_wifi_probe,
		.wlan_remove_cb = consys_wifi_remove,
		.wlan_bus_cnt_get_cb = consys_wifi_bus_cnt,
		.wlan_bus_cnt_clr_cb = consys_wifi_bus_cnt,
	};

	return mtk_wcn_wmt_wlan_reg(&cb);
}
late_initcall(consys_wifi_init);

MODULE_DESCRIPTION("MT6582 CONSYS WiFi RAM code loader (bring-up)");
MODULE_LICENSE("GPL");

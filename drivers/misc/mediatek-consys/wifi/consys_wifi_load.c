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
#include <linux/etherdevice.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
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
#define  HIF_TARGET_TXD1	1
#define  HIF_TARGET_RXD0	2
#define  HIF_TARGET_RXD1	3
#define  HIF_TARGET_WHISR	4

#define INIT_CMD_ID_DOWNLOAD_BUF	1
#define INIT_CMD_ID_WIFI_START		2
#define INIT_EVENT_ID_CMD_RESULT	1
#define DOWNLOAD_BUF_ENCRYPTION_MODE	BIT(0)
#define DOWNLOAD_BUF_NO_CRC_CHECKING	BIT(30)
#define DOWNLOAD_BUF_ACK_OPTION		BIT(31)
#define CMD_PKT_SIZE_FOR_IMAGE		2048
#define TX_CREDITS_TC0			8
#define TX_CREDITS_TC4			4
#define TC0_INDEX			0
#define TC4_INDEX			4

#define CMD_ID_SCAN_REQ_V2		0x04
#define CMD_ID_POWER_SAVE_MODE		0x06
#define CMD_ID_SET_DOMAIN_INFO		0x13
#define CMD_ID_BSS_ACTIVATE_CTRL	0x15
#define CMD_ID_SET_TX_PWR		0x28
#define CMD_ID_SET_PHY_PARAM		0x31
#define CMD_ID_SET_EDGE_TXPWR_LIMIT	0x36
#define CMD_ID_BASIC_CONFIG		0xc1
#define CMD_ID_GET_NIC_CAPABILITY	0x80

#define EVENT_ID_NIC_CAPABILITY		0x02
#define EVENT_ID_SCAN_DONE		0x15
#define EVENT_ID_BASIC_CONFIG		0x09
#define HIF_RX_PKT_TYPE_EVENT		1
#define HIF_RX_PKT_TYPE_MANAGEMENT	3
#define WIFI_SCAN_RX_MAX			4096
#define WIFI_SCAN_MAX_BSS		64

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

struct wifi_cmd {
	__le16 tx_byte_count_user_priority;
	u8 ether_type_offset;
	u8 resource_pkt_type_csflags;
	u8 cid;
	u8 set_query;
	u8 seq;
	u8 reserved;
} __packed;

struct wifi_event {
	__le16 packet_len;
	__le16 packet_type;
	u8 eid;
	u8 seq;
	u8 reserved[2];
} __packed;

struct hif_rx_header {
	__le16 packet_len;
	__le16 packet_type;
	u8 header_len_offset;
	u8 format_flags;
	__le16 seq_tid;
	u8 sta_idx;
	u8 rcpi;
	u8 hw_channel;
	u8 reserved;
} __packed;

struct cmd_basic_config {
	u8 mac[6];
	u8 native_80211;
	u8 reserved;
	__le16 rx_checksum;
	__le16 tx_checksum;
} __packed;

struct event_nic_capability {
	__le16 product_id;
	__le16 fw_version;
	__le16 driver_version;
	u8 hw_5g_disabled;
	u8 eeprom_used;
	u8 efuse_valid;
	u8 mac_valid;
	u8 rf_version;
	u8 phy_version;
	u8 rf_cal_fail;
	u8 bb_cal_fail;
	u8 reserved[2];
} __packed;

struct cmd_subband_info {
	u8 regulatory_class;
	u8 band;
	u8 channel_span;
	u8 first_channel;
	u8 channels;
	u8 reserved[3];
} __packed;

struct cmd_set_domain_info {
	__le16 country_code;
	__le16 reserved;
	struct cmd_subband_info subband[6];
	u8 bandwidth_2g4;
	u8 bandwidth_5g;
	u8 reserved2[2];
} __packed;

struct cmd_bss_activate {
	u8 network_type;
	u8 active;
	u8 reserved[2];
} __packed;

struct cmd_power_save {
	u8 network_type;
	u8 profile;
	u8 reserved[2];
} __packed;

struct cmd_ssid {
	__le32 len;
	u8 ssid[32];
} __packed;

/* CMD_SCAN_REQ_V2 up to, but not including, its variable-length IE data. */
struct cmd_scan_req_v2 {
	u8 scan_seq;
	u8 network_type;
	u8 scan_type;
	u8 ssid_type;
	struct cmd_ssid ssid[4];
	__le16 probe_delay_ms;
	__le16 channel_dwell_ms;
	u8 channel_type;
	u8 channel_count;
	u8 channel[32][2];
	__le16 ie_len;
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
static u8 tx_credits[6];
static u8 cmd_seq;
static u8 scan_seq;
static bool loaded;
static bool wifi_paldo_on;
static DEFINE_MUTEX(wifi_lock);
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

static void hif_refresh_tx_credits(void)
{
	u32 rel[2] = { hif_rd(MCR_WTSR0), hif_rd(MCR_WTSR1) };
	u8 *count = (u8 *)rel;
	int tc;

	for (tc = 0; tc < ARRAY_SIZE(tx_credits); tc++)
		tx_credits[tc] += count[tc];
}

/* nicTxAcquireResource / nicTxPollingResource */
static int hif_tx_credit(u8 tc)
{
	int tries = 256;

	while (tries--) {
		if (tx_credits[tc]) {
			tx_credits[tc]--;
			return 0;
		}
		hif_refresh_tx_credits();
		if (!tx_credits[tc])
			msleep(50);
	}
	pr_err("consys-wifi: no TX credit on TC%u\n", tc);
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
	ret = hif_tx_credit(TC0_INDEX);
	if (ret)
		return ret;
	/* one zero dword terminates TX aggregation */
	*(u32 *)(pkt + total) = 0;
	hif_port_write(MCR_WTDR0, HIF_TARGET_TXD0, pkt, total);
	return 0;
}

static int hif_send_normal_cmd(u8 cid, bool set, const void *payload,
			       u32 payload_len, u8 *sent_seq)
{
	struct wifi_cmd *h;
	u8 *pkt;
	u32 len = sizeof(*h) + payload_len;
	u32 total = ALIGN(len, 4);
	int ret;

	pkt = kzalloc(total + sizeof(u32), GFP_KERNEL);
	if (!pkt)
		return -ENOMEM;
	h = (struct wifi_cmd *)pkt;
	h->tx_byte_count_user_priority = cpu_to_le16(total & 0xfff);
	h->resource_pkt_type_csflags = (TC4_INDEX << 2) | (1 << 6);
	h->cid = cid;
	h->set_query = set;
	h->seq = ++cmd_seq;
	if (payload_len)
		memcpy(pkt + sizeof(*h), payload, payload_len);
	ret = hif_tx_credit(TC4_INDEX);
	if (!ret) {
		/* one zero dword terminates TX aggregation */
		*(u32 *)(pkt + total) = 0;
		hif_port_write(MCR_WTDR1, HIF_TARGET_TXD1, pkt, total);
		if (sent_seq)
			*sent_seq = h->seq;
	}
	kfree(pkt);
	return ret;
}

static int hif_read_rx_port(u8 port, u8 *buf, u32 max, u32 *got)
{
	u32 lengths = hif_rd(MCR_WRPLR);
	u32 len = port ? lengths >> 16 : lengths & 0xffff;

	if (!len)
		return -EAGAIN;
	if (len > max) {
		pr_err("consys-wifi: RX%u packet %u bytes exceeds %u\n",
		       port, len, max);
		return -E2BIG;
	}
	hif_port_read(port ? MCR_WRDR1 : MCR_WRDR0,
		      port ? HIF_TARGET_RXD1 : HIF_TARGET_RXD0, buf, len);
	*got = len;
	return 0;
}

static int hif_wait_event(u8 eid, u8 seq, u8 *payload, u32 payload_max,
			  u32 *payload_len)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(3000);
	u8 rsp[256];

	while (time_before(jiffies, deadline)) {
		int port;

		hif_refresh_tx_credits();
		for (port = 0; port < 2; port++) {
			struct wifi_event *event = (struct wifi_event *)rsp;
			u32 got, event_len;
			int ret = hif_read_rx_port(port, rsp, sizeof(rsp), &got);

			if (ret == -EAGAIN)
				continue;
			if (ret)
				return ret;
			if (got < sizeof(*event) ||
			    (le16_to_cpu(event->packet_type) & 3) != HIF_RX_PKT_TYPE_EVENT) {
				pr_warn("consys-wifi: unexpected RX%u packet while waiting for event 0x%02x\n",
					port, eid);
				continue;
			}
			if (event->eid != eid || event->seq != seq) {
				pr_info("consys-wifi: skipped event 0x%02x seq %u while waiting for 0x%02x seq %u\n",
					event->eid, event->seq, eid, seq);
				continue;
			}
			event_len = min_t(u32, le16_to_cpu(event->packet_len), got);
			if (event_len < sizeof(*event))
				return -EPROTO;
			*payload_len = event_len - sizeof(*event);
			if (*payload_len > payload_max)
				return -E2BIG;
			memcpy(payload, rsp + sizeof(*event), *payload_len);
			return 0;
		}
		usleep_range(500, 1000);
	}
	return -ETIMEDOUT;
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

/* Stock stWifiCfgDefault.bin values passed by wlanLoadManufactureData(). */
static const u8 stock_tx_power[40] = {
	[0] = 0x26,
	[1] = 0x26,
	[4 ... 15] = 0x20,
	[16 ... 21] = 0x1e,
};

static const u8 stock_efuse[144];

static const struct cmd_set_domain_info stock_domain = {
	/* A zero country code falls back to the vendor driver's EU domain. */
	.subband = {
		{ 81,  1, 1,   1, 13, { 0, 0, 0 } },
		{ 115, 2, 4,  36,  4, { 0, 0, 0 } },
		{ 118, 2, 4,  52,  4, { 0, 0, 0 } },
		{ 121, 2, 4, 100, 11, { 0, 0, 0 } },
		{ 125, 2, 4, 149,  7, { 0, 0, 0 } },
	},
	.bandwidth_2g4 = 1,	/* CONFIG_BW_20M */
	.bandwidth_5g = 0,	/* CONFIG_BW_20_40M */
};

static const u8 stock_edge_power[4] = { 0x26, 0x1e, 0x1a, 0 };

static int consys_wifi_post_ready(void)
{
	struct cmd_basic_config basic = { };
	struct event_nic_capability capability = { };
	struct cmd_power_save power_save = { .network_type = 0, .profile = 2 };
	/* Shipped firmware appends private capability words not present in the
	 * public vendor structure.  Only consume its stable common prefix. */
	u8 rsp[64];
	u8 seq;
	u32 got = 0, intr, wasr;
	int ret;

	/* Match wlanAdapterStart(): clear the init interrupt/release state and
	 * begin normal operation with four command buffers on TC4/TX port 1. */
	hif_port_read(MCR_WHISR, HIF_TARGET_WHISR, (u8 *)&intr, sizeof(intr));
	(void)hif_rd(MCR_WTSR0);
	(void)hif_rd(MCR_WTSR1);
	memset(tx_credits, 0, sizeof(tx_credits));
	tx_credits[TC4_INDEX] = TX_CREDITS_TC4;

	ret = hif_send_normal_cmd(CMD_ID_BASIC_CONFIG, false, &basic,
				  sizeof(basic), &seq);
	if (ret)
		return ret;
	ret = hif_wait_event(EVENT_ID_BASIC_CONFIG, seq, rsp, sizeof(rsp), &got);
	if (ret || got < sizeof(basic)) {
		pr_err("consys-wifi: permanent-address query failed (%d, %u bytes)\n",
		       ret, got);
		return ret ? ret : -EPROTO;
	}
	memcpy(&basic, rsp, sizeof(basic));
	pr_info("consys-wifi: firmware MAC %pM\n", basic.mac);

	ret = hif_send_normal_cmd(CMD_ID_GET_NIC_CAPABILITY, false, &capability,
				  sizeof(capability), &seq);
	if (ret)
		return ret;
	ret = hif_wait_event(EVENT_ID_NIC_CAPABILITY, seq, rsp, sizeof(rsp), &got);
	if (ret || got < 10) {
		pr_err("consys-wifi: capability query failed (%d, %u bytes)\n", ret, got);
		return ret ? ret : -EPROTO;
	}
	memcpy(&capability, rsp, min_t(u32, got, sizeof(capability)));
	pr_info("consys-wifi: firmware product 0x%04x version 0x%04x, PHY/RF %u/%u, calibration %u/%u\n",
		le16_to_cpu(capability.product_id), le16_to_cpu(capability.fw_version),
		capability.phy_version, capability.rf_version,
		capability.bb_cal_fail, capability.rf_cal_fail);

	/* CFG_INIT_POWER_SAVE_PROF is ENUM_PSP_FAST_SWITCH in the stock tree. */
	ret = hif_send_normal_cmd(CMD_ID_POWER_SAVE_MODE, true, &power_save,
				  sizeof(power_save), NULL);
	if (!ret)
		ret = hif_send_normal_cmd(CMD_ID_SET_TX_PWR, true, stock_tx_power,
					  sizeof(stock_tx_power), NULL);
	if (!ret)
		ret = hif_send_normal_cmd(CMD_ID_SET_PHY_PARAM, true, stock_efuse,
					  sizeof(stock_efuse), NULL);
	if (!ret)
		ret = hif_send_normal_cmd(CMD_ID_SET_DOMAIN_INFO, true, &stock_domain,
					  sizeof(stock_domain), NULL);
	if (!ret)
		ret = hif_send_normal_cmd(CMD_ID_SET_EDGE_TXPWR_LIMIT, true,
					  stock_edge_power, sizeof(stock_edge_power), NULL);
	if (ret)
		return ret;

	msleep(20);
	wasr = hif_rd(MCR_WASR);
	pr_info("consys-wifi: stock power, eFuse, domain, and band-edge data sent (WASR 0x%08x)\n",
		wasr);
	return 0;
}

static bool bssid_seen(u8 seen[WIFI_SCAN_MAX_BSS][6], int count,
		       const u8 *bssid)
{
	int i;

	for (i = 0; i < count; i++)
		if (ether_addr_equal(seen[i], bssid))
			return true;
	return false;
}

static void log_scan_management(const u8 *buf, u32 got,
				u8 seen[WIFI_SCAN_MAX_BSS][6], int *seen_count)
{
	const struct hif_rx_header *h = (const struct hif_rx_header *)buf;
	const u8 *frame, *ie, *end, *ssid = NULL;
	u8 clean_ssid[33];
	u32 packet_len, frame_len, offset, ssid_len = 0;
	u16 fc;
	int i, signal;

	if (got < sizeof(*h))
		return;
	packet_len = min_t(u32, le16_to_cpu(h->packet_len), got);
	offset = h->header_len_offset & 3;
	if (packet_len < sizeof(*h) + offset + 36)
		return;
	frame = buf + sizeof(*h) + offset;
	frame_len = packet_len - sizeof(*h) - offset;
	fc = frame[0] | frame[1] << 8;
	if ((fc & 0x00fc) != 0x0080 && (fc & 0x00fc) != 0x0050)
		return;
	if (bssid_seen(seen, *seen_count, frame + 16))
		return;
	if (*seen_count < WIFI_SCAN_MAX_BSS)
		ether_addr_copy(seen[(*seen_count)++], frame + 16);

	ie = frame + 36;
	end = frame + frame_len;
	while (ie + 2 <= end && ie + 2 + ie[1] <= end) {
		if (ie[0] == 0) {
			ssid = ie + 2;
			ssid_len = min_t(u32, ie[1], 32);
			break;
		}
		ie += 2 + ie[1];
	}
	if (ssid_len) {
		for (i = 0; i < ssid_len; i++)
			clean_ssid[i] = (ssid[i] >= 0x20 && ssid[i] < 0x7f) ? ssid[i] : '.';
		clean_ssid[ssid_len] = 0;
	} else {
		strscpy(clean_ssid, "<hidden>", sizeof(clean_ssid));
	}
	signal = h->rcpi / 2 - 110;
	pr_info("consys-wifi: scan BSS %pM ch %u %d dBm SSID \"%s\"\n",
		frame + 16, h->hw_channel, signal, clean_ssid);
}

static int consys_wifi_scan_once(void)
{
	struct cmd_bss_activate activate = { .network_type = 0, .active = 1 };
	struct cmd_scan_req_v2 scan = {
		.network_type = 0,
		.scan_type = 1,	/* SCAN_TYPE_ACTIVE_SCAN */
		.ssid_type = 1,	/* wildcard */
		.channel_type = 1,	/* SCAN_CHANNEL_2G4 */
	};
	u8 seen[WIFI_SCAN_MAX_BSS][6] = { };
	u8 *buf;
	unsigned long deadline;
	int ret, port, seen_count = 0;
	bool done = false;

	buf = kmalloc(WIFI_SCAN_RX_MAX, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	scan.scan_seq = ++scan_seq;
	ret = hif_send_normal_cmd(CMD_ID_BSS_ACTIVATE_CTRL, true, &activate,
				  sizeof(activate), NULL);
	if (ret)
		goto out;
	ret = hif_send_normal_cmd(CMD_ID_SCAN_REQ_V2, true, &scan, sizeof(scan), NULL);
	if (ret)
		goto deactivate;
	pr_info("consys-wifi: starting active wildcard 2.4 GHz scan %u\n", scan.scan_seq);

	deadline = jiffies + msecs_to_jiffies(15000);
	while (!done && time_before(jiffies, deadline)) {
		bool received = false;

		hif_refresh_tx_credits();
		for (port = 0; port < 2; port++) {
			u32 got;

			while (!(ret = hif_read_rx_port(port, buf, WIFI_SCAN_RX_MAX, &got))) {
				u16 packet_type;

				received = true;
				if (got < sizeof(__le32))
					continue;
				packet_type = (buf[2] | buf[3] << 8) & 3;
				if (packet_type == HIF_RX_PKT_TYPE_MANAGEMENT) {
					log_scan_management(buf, got, seen, &seen_count);
				} else if (packet_type == HIF_RX_PKT_TYPE_EVENT &&
					   got >= sizeof(struct wifi_event)) {
					struct wifi_event *event = (struct wifi_event *)buf;

					if (event->eid == EVENT_ID_SCAN_DONE &&
					    got >= sizeof(*event) + 4 &&
					    buf[sizeof(*event)] == scan.scan_seq) {
						done = true;
						break;
					}
					pr_info("consys-wifi: scan received event 0x%02x seq %u\n",
						event->eid, event->seq);
				}
			}
			if (done)
				break;
			if (ret != -EAGAIN)
				goto deactivate;
		}
		if (!received)
			msleep(10);
	}
	ret = done ? 0 : -ETIMEDOUT;
	if (done)
		pr_info("consys-wifi: scan %u complete, %d BSS%s received\n",
			scan.scan_seq, seen_count, seen_count == 1 ? "" : "es");
	else
		pr_err("consys-wifi: scan %u timed out after receiving %d BSS%s (WASR 0x%08x)\n",
		       scan.scan_seq, seen_count, seen_count == 1 ? "" : "es",
		       hif_rd(MCR_WASR));

deactivate:
	activate.active = 0;
	if (hif_send_normal_cmd(CMD_ID_BSS_ACTIVATE_CTRL, true, &activate,
					sizeof(activate), NULL) && !ret)
		ret = -EIO;
out:
	kfree(buf);
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
	memset(tx_credits, 0, sizeof(tx_credits));
	tx_credits[TC0_INDEX] = TX_CREDITS_TC0;
	cmd_seq = 0;
	scan_seq = 0;
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
	ret = consys_wifi_post_ready();
	if (ret) {
		pr_err("consys-wifi: post-ready initialization failed (%d)\n", ret);
		goto out;
	}
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
	mutex_lock(&wifi_lock);
	if (on) {
		ret = mtk_wcn_wmt_func_on(WMTDRV_TYPE_WIFI) == MTK_WCN_BOOL_TRUE ? 0 : -EIO;
		pr_info("consys-wifi: WMT WiFi function on -> %d (firmware %s)\n", ret,
			loaded ? "running" : "not running");
	} else {
		ret = mtk_wcn_wmt_func_off(WMTDRV_TYPE_WIFI) == MTK_WCN_BOOL_TRUE ? 0 : -EIO;
		pr_info("consys-wifi: WMT WiFi function off -> %d\n", ret);
	}
	mutex_unlock(&wifi_lock);
	return ret;
}

static int load_get(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "%d\n", loaded);
}

static const struct kernel_param_ops load_ops = { .set = load_set, .get = load_get };
module_param_cb(load, &load_ops, NULL, 0644);
MODULE_PARM_DESC(load, "1: turn the WMT WiFi function on and load the RAM code; 0: off");

static int scan_set(const char *val, const struct kernel_param *kp)
{
	bool trigger;
	int ret = kstrtobool(val, &trigger);

	if (ret || !trigger)
		return ret;
	mutex_lock(&wifi_lock);
	ret = loaded && hif ? consys_wifi_scan_once() : -ENODEV;
	mutex_unlock(&wifi_lock);
	return ret;
}

static int scan_get(char *buf, const struct kernel_param *kp)
{
	return sprintf(buf, "0\n");
}

static const struct kernel_param_ops scan_ops = { .set = scan_set, .get = scan_get };
module_param_cb(scan, &scan_ops, NULL, 0644);
MODULE_PARM_DESC(scan, "write 1 to run one active wildcard 2.4 GHz firmware scan");

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

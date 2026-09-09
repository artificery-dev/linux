// SPDX-License-Identifier: GPL-2.0
/*
 * Bluetooth HCI driver over the MediaTek CONSYS STP transport.
 *
 * The vendor Android stack exposed the STP Bluetooth channel as /dev/stpbt,
 * a char device carrying HCI H4 packets (packet-type byte + HCI packet) for
 * a userspace HAL. This driver puts the same channel behind a kernel hci_dev
 * so BlueZ sees an ordinary hci0: open = WMT "function on" for BT (which
 * powers the subsystem and downloads the patch on first use) plus the STP
 * receive-event hook; send = an H4 frame into mtk_wcn_stp_send_data(); the
 * receive side drains the STP BT queue into the H4 stream parser the other
 * MediaTek Bluetooth drivers use.
 */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include "../../../bluetooth/h4_recv.h"
#include "osal_typedef.h"
#include "osal.h"
#include "stp_exp.h"
#include "wmt_exp.h"

/* wmt_lib.h drags vendor macros in that clash with skbuff.h; declare what we use */
extern INT32 wmt_lib_ps_disable(VOID);
#include "consys_plat.h"

#define STP_HCI_RX_CHUNK	2048
#define STP_HCI_TX_RETRIES	50	/* 50 x 2 ms for STP window space */

struct stp_hci {
	struct hci_dev *hdev;
	struct work_struct rx_work;
	struct sk_buff *rx_skb;
	u8 rx_buf[STP_HCI_RX_CHUNK];
	bool open;
	bool resetting;
};

static struct stp_hci *stp_hci_dev;	/* the STP event callback has no cookie */

/* The STP Bluetooth channel has one consumer. The vendor's /dev/stpbt can be
 * built alongside this for bring-up, so tell it when hci0 holds the channel:
 * both talking at once wedges the link and takes the system with it. */
bool stp_hci_channel_busy(void)
{
	return stp_hci_dev && stp_hci_dev->open;
}
EXPORT_SYMBOL_GPL(stp_hci_channel_busy);
static bool debug;		/* dump the H4 stream (bring-up aid) */
module_param(debug, bool, 0644);
static bool psm = true;		/* STP power saving, as the vendor leaves it */
module_param(psm, bool, 0644);

static const struct h4_recv_pkt stp_hci_recv_pkts[] = {
	{ H4_RECV_ACL,   .recv = hci_recv_frame },
	{ H4_RECV_SCO,   .recv = hci_recv_frame },
	{ H4_RECV_EVENT, .recv = hci_recv_frame },
};

static void stp_hci_rx_work(struct work_struct *work)
{
	struct stp_hci *sh = container_of(work, struct stp_hci, rx_work);
	int n;

	while (sh->open) {
		n = mtk_wcn_stp_receive_data(sh->rx_buf, sizeof(sh->rx_buf), BT_TASK_INDX);
		if (n <= 0)
			break;
		if (debug)
			print_hex_dump(KERN_INFO, "stp_hci rx: ", DUMP_PREFIX_OFFSET, 16, 1,
				       sh->rx_buf, min(n, 64), false);
		sh->rx_skb = h4_recv_buf(sh->hdev, sh->rx_skb, sh->rx_buf, n,
					 stp_hci_recv_pkts, ARRAY_SIZE(stp_hci_recv_pkts));
		if (IS_ERR(sh->rx_skb)) {
			bt_dev_err(sh->hdev, "H4 frame error %ld", PTR_ERR(sh->rx_skb));
			sh->hdev->stat.err_rx++;
			sh->rx_skb = NULL;
		}
	}
}

/* STP: "the BT rx queue has data" */
static void stp_hci_event_cb(void)
{
	struct stp_hci *sh = stp_hci_dev;

	if (sh && sh->open)
		schedule_work(&sh->rx_work);
}

/* WMT: the subsystem is being reset (firmware assert / whole-chip reset) */
static void stp_hci_rst_cb(ENUM_WMTDRV_TYPE_T src, ENUM_WMTDRV_TYPE_T dst,
			   ENUM_WMTMSG_TYPE_T type, void *buf, unsigned int sz)
{
	struct stp_hci *sh = stp_hci_dev;
	ENUM_WMTRSTMSG_TYPE_T msg;

	if (!sh || sz > sizeof(msg) || src != WMTDRV_TYPE_WMT ||
	    dst != WMTDRV_TYPE_BT || type != WMTMSG_TYPE_RESET)
		return;
	memcpy(&msg, buf, sz);
	if (msg == WMTRSTMSG_RESET_START) {
		bt_dev_warn(sh->hdev, "CONSYS reset started");
		sh->resetting = true;
	} else if (msg == WMTRSTMSG_RESET_END) {
		bt_dev_warn(sh->hdev, "CONSYS reset finished");
		sh->resetting = false;
		/* the controller state is gone: let the stack re-open us */
		if (sh->open)
			hci_reset_dev(sh->hdev);
	}
}

static int stp_hci_open(struct hci_dev *hdev)
{
	struct stp_hci *sh = hci_get_drvdata(hdev);

	if (consys_wmt_autoconf()) {
		bt_dev_err(hdev, "CONSYS not configured");
		return -ENODEV;
	}
	if (mtk_wcn_wmt_func_on(WMTDRV_TYPE_BT) == MTK_WCN_BOOL_FALSE) {
		bt_dev_err(hdev, "WMT could not turn Bluetooth on");
		return -ENODEV;
	}
	if (!mtk_wcn_stp_is_ready()) {
		bt_dev_err(hdev, "STP not ready after function on");
		mtk_wcn_wmt_func_off(WMTDRV_TYPE_BT);
		return -ENODEV;
	}
	mtk_wcn_stp_set_bluez(0);	/* MTK-framed HCI over STP, like /dev/stpbt */
	if (!psm) {
		/* The stock system leaves power saving on and the link sleeps
		 * between transfers; turning it off is a bring-up aid only. */
		wmt_lib_ps_disable();
		bt_dev_info(hdev, "STP power saving disabled");
	}
	sh->resetting = false;
	sh->open = true;
	mtk_wcn_wmt_msgcb_reg(WMTDRV_TYPE_BT, stp_hci_rst_cb);
	mtk_wcn_stp_register_event_cb(BT_TASK_INDX, stp_hci_event_cb);
	/* anything already queued */
	schedule_work(&sh->rx_work);
	bt_dev_info(hdev, "Bluetooth function on over STP/BTIF");
	return 0;
}

static int stp_hci_close(struct hci_dev *hdev)
{
	struct stp_hci *sh = hci_get_drvdata(hdev);

	sh->open = false;
	mtk_wcn_stp_register_event_cb(BT_TASK_INDX, NULL);
	mtk_wcn_wmt_msgcb_unreg(WMTDRV_TYPE_BT);
	cancel_work_sync(&sh->rx_work);
	kfree_skb(sh->rx_skb);
	sh->rx_skb = NULL;
	if (mtk_wcn_wmt_func_off(WMTDRV_TYPE_BT) == MTK_WCN_BOOL_FALSE)
		bt_dev_warn(hdev, "WMT could not turn Bluetooth off");
	return 0;
}

/*
 * The controller answers Read Local Supported Commands with every LE bit
 * clear (bytes 26 and 27 are zero) even though it runs LE Set Scan Enable
 * and LE Create Connection perfectly well. hci_le_set_event_mask_sync()
 * builds the LE event mask from exactly those bits, so it programs a mask
 * with neither the advertising report nor the connection complete event:
 * scanning reports nothing and LE connections never complete. Correct the
 * bitmap and re-send the mask once the core has finished its init.
 */
static void stp_hci_radio_cal(struct hci_dev *hdev)
{
	static const struct { u16 opcode; u8 len; u8 param[7]; } cmds[] = {
		{ 0xfc79, 6, { 0x05, 0x07, 0x03, 0x40, 0x1f, 0x40 } }, /* Set_Radio */
		{ 0xfc7a, 7, { 0x1f, 0x00, 0x04, 0x80, 0x00, 0xff, 0xff } },
		{ 0xfc93, 3, { 0x00, 0x00, 0x00 } },
	};
	struct sk_buff *skb;
	int i;

	for (i = 0; i < ARRAY_SIZE(cmds); i++) {
		skb = __hci_cmd_sync(hdev, cmds[i].opcode, cmds[i].len,
				     cmds[i].param, HCI_CMD_TIMEOUT);
		if (IS_ERR(skb)) {
			bt_dev_warn(hdev, "radio cal %04x: %ld",
				    cmds[i].opcode, PTR_ERR(skb));
			continue;
		}
		kfree_skb(skb);
	}
	bt_dev_info(hdev, "NVRAM radio calibration applied");
}

static int stp_hci_post_init(struct hci_dev *hdev)
{
	u8 events[8] = { 0 };
	struct sk_buff *skb;

	stp_hci_radio_cal(hdev);

	if (hdev->commands[26] & 0x18)
		return 0;		/* a controller that tells the truth */

	hdev->commands[26] |= 0x08 |	/* LE Set Scan Enable */
			      0x10;	/* LE Create Connection */
	hdev->commands[27] |= 0x04 |	/* LE Connection Update */
			      0x20;	/* LE Read Remote Used Features */

	events[0] = 0x01 |		/* LE Connection Complete */
		    0x02 |		/* LE Advertising Report */
		    0x04 |		/* LE Connection Update Complete */
		    0x08;		/* LE Read Remote Used Features Complete */
	if (hdev->le_features[0] & HCI_LE_ENCRYPTION)
		events[0] |= 0x10;	/* LE Long Term Key Request */

	skb = __hci_cmd_sync(hdev, HCI_OP_LE_SET_EVENT_MASK, sizeof(events),
			     events, HCI_CMD_TIMEOUT);
	if (IS_ERR(skb)) {
		bt_dev_err(hdev, "LE event mask: %ld", PTR_ERR(skb));
		return PTR_ERR(skb);
	}
	kfree_skb(skb);
	bt_dev_info(hdev, "LE event mask corrected to 0x%02x", events[0]);
	return 0;
}

static int stp_hci_flush(struct hci_dev *hdev)
{
	return 0;
}

static int stp_hci_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct stp_hci *sh = hci_get_drvdata(hdev);
	int written, tries = STP_HCI_TX_RETRIES;

	if (sh->resetting) {
		kfree_skb(skb);
		return -EBUSY;
	}
	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	}
	/* H4: the packet type byte leads the packet */
	memcpy(skb_push(skb, 1), &hci_skb_pkt_type(skb), 1);

	if (debug)
		print_hex_dump(KERN_INFO, "stp_hci tx: ", DUMP_PREFIX_OFFSET, 16, 1,
			       skb->data, min_t(unsigned int, skb->len, 64), false);
	/* STP only takes a whole packet, and returns 0 when its transmit
	 * window has no room (the chip has not acked earlier frames yet). */
	do {
		written = mtk_wcn_stp_send_data(skb->data, skb->len, BT_TASK_INDX);
		if (written > 0)
			break;
		usleep_range(2000, 3000);
	} while (--tries);

	if (written != skb->len) {
		bt_dev_err(hdev, "STP send %d of %u bytes", written, skb->len);
		hdev->stat.err_tx++;
		kfree_skb(skb);
		return -EIO;
	}
	hdev->stat.byte_tx += skb->len;
	kfree_skb(skb);
	return 0;
}

static int __init stp_hci_init(void)
{
	struct stp_hci *sh;
	struct hci_dev *hdev;
	int err;

	sh = kzalloc(sizeof(*sh), GFP_KERNEL);
	if (!sh)
		return -ENOMEM;
	INIT_WORK(&sh->rx_work, stp_hci_rx_work);

	hdev = hci_alloc_dev();
	if (!hdev) {
		kfree(sh);
		return -ENOMEM;
	}
	sh->hdev = hdev;
	hci_set_drvdata(hdev, sh);
	hdev->bus = HCI_VIRTUAL;	/* an on-SoC link; no bus enum fits */
	hdev->open = stp_hci_open;
	hdev->close = stp_hci_close;
	hdev->flush = stp_hci_flush;
	hdev->post_init = stp_hci_post_init;
	hdev->send = stp_hci_send_frame;
	SET_HCIDEV_DEV(hdev, wmt_plat_get_dev());
	/* The E2 firmware advertises two extended-feature pages, then answers
	 * the page-2 read with status 0x30 (parameter out of range). */
	set_bit(HCI_QUIRK_BROKEN_LOCAL_EXT_FEATURES_PAGE_2, &hdev->quirks);
	/* The controller keeps its own address (the vendor HAL could override
	 * it from NVRAM; the stock unit had none set). */

	stp_hci_dev = sh;
	err = hci_register_dev(hdev);
	if (err) {
		stp_hci_dev = NULL;
		hci_free_dev(hdev);
		kfree(sh);
		return err;
	}
	pr_info("stp_hci: MediaTek CONSYS Bluetooth registered as %s\n", hdev->name);
	return 0;
}
late_initcall(stp_hci_init);

MODULE_DESCRIPTION("Bluetooth HCI over MediaTek CONSYS STP");
MODULE_LICENSE("GPL");

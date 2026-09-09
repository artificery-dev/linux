// SPDX-License-Identifier: GPL-2.0
/*
 * What the Android "6620_launcher" did for the WMT core, done in the kernel:
 *
 *  - before the first function-on, the STP transport mode (BTIF full mode,
 *    FM over the common interface) and the patch location are configured
 *    through the same wmt_lib entry points the /dev/stpwmt ioctls use;
 *  - when the SoC init script asks for its patches ("srh_patch"), the patch
 *    list is built from the firmware images instead of a directory scan.
 *
 * The launcher's patch search (stp_uart_launcher.c, cmd_hdr_sch_patch): for
 * each "<chip>_patch_*" file, bytes 22-23 are the patch's firmware version
 * (its low byte must match the chip's), and bytes 24-27 are the "patch
 * info": the top nibble of byte 24 is the number of patches, its low nibble
 * this patch's download sequence, and the four bytes, with byte 24 cleared,
 * are the address the WMT partial-patch command carries.
 */
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "osal_typedef.h"
#include "osal.h"
#include "wmt_core.h"
#include "wmt_lib.h"
#include "wmt_dev.h"
#include "consys_plat.h"
#include "mtk_wcn_consys_hw.h"

extern UINT8 __iomem *pEmibaseaddr;

#define CONSYS_FW_DIR	"mediatek/mt6582/"

/* The MT6582's ROMv1 patch set, in the firmware image. */
static const char * const consys_patch_files[] = {
	"mt6572_82_patch_e1_0_hdr.bin",
	"mt6572_82_patch_e1_1_hdr.bin",
};

static bool consys_wmt_configured;

int consys_wmt_autoconf(void)
{
	P_OSAL_OP pOp;
	P_WMT_HIF_CONF pHif;
	int ret;

	if (consys_wmt_configured)
		return 0;
	if (!consys_plat.ready)
		return -ENODEV;
	/* wmt_lib_init() ran at module init, before the platform device had
	 * probed (its rails come from the PMIC, which probes late): map the
	 * EMI window now if that first attempt could not. */
	if (!pEmibaseaddr) {
		ret = mtk_wcn_consys_hw_init();
		if (ret) {
			pr_err("consys: EMI setup failed (%d)\n", ret);
			return ret;
		}
	}

	wmt_lib_set_patch_name((UCHAR *)CONSYS_FW_DIR);

	/* WMT_IOCTL_SET_STP_MODE: ((fm & 0xF) << 4) | stp_mode */
	ret = wmt_lib_set_hif((WMT_FM_COMM << 4) | STP_BTIF_FULL);
	if (ret) {
		pr_err("consys: wmt_lib_set_hif failed (%d)\n", ret);
		return -EIO;
	}
	pOp = wmt_lib_get_free_op();
	if (!pOp) {
		pr_err("consys: no free WMT op for HIF_CONF\n");
		return -EBUSY;
	}
	pOp->op.opId = WMT_OPID_HIF_CONF;
	pHif = wmt_lib_get_hif();
	osal_memcpy(&pOp->op.au4OpData[0], pHif, sizeof(WMT_HIF_CONF));
	pOp->op.u4InfoBit = WMT_OP_HIF_BIT;
	pOp->signal.timeoutValue = 0;
	if (wmt_lib_put_act_op(pOp) == MTK_WCN_BOOL_FALSE) {
		pr_err("consys: WMT_OPID_HIF_CONF failed\n");
		return -EIO;
	}
	consys_wmt_configured = true;
	pr_info("consys: WMT configured for STP over BTIF\n");
	return 0;
}
EXPORT_SYMBOL(consys_wmt_autoconf);

/*
 * The "srh_patch" request from the SoC init script: build the patch table
 * from the firmware images. Returns 0 when the table was handed to the core.
 */
int consys_wmt_patch_search(void)
{
	P_WMT_PATCH_INFO info;
	const struct firmware *fw;
	char name[NAME_MAX + 1];
	unsigned int num = 0, filled = 0;
	int i, ret;

	if (!consys_plat.ready)
		return -ENODEV;
	info = kcalloc(ARRAY_SIZE(consys_patch_files), sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(consys_patch_files); i++) {
		unsigned int cnt, seq;

		snprintf(name, sizeof(name), CONSYS_FW_DIR "%s", consys_patch_files[i]);
		ret = request_firmware(&fw, name, consys_plat.dev);
		if (ret) {
			pr_err("consys: patch %s missing (%d)\n", name, ret);
			continue;
		}
		if (fw->size < 28) {
			pr_err("consys: patch %s too short\n", name);
			release_firmware(fw);
			continue;
		}
		cnt = fw->data[24] >> 4;
		seq = fw->data[24] & 0xf;
		if (!num)
			num = cnt;
		if (!seq || seq > ARRAY_SIZE(consys_patch_files) || cnt != num) {
			pr_err("consys: patch %s: odd info %02x %02x %02x %02x\n", name,
			       fw->data[24], fw->data[25], fw->data[26], fw->data[27]);
			release_firmware(fw);
			continue;
		}
		info[seq - 1].dowloadSeq = seq;
		info[seq - 1].addRess[0] = 0;
		info[seq - 1].addRess[1] = fw->data[25];
		info[seq - 1].addRess[2] = fw->data[26];
		info[seq - 1].addRess[3] = fw->data[27];
		strscpy(info[seq - 1].patchName, name, sizeof(info[seq - 1].patchName));
		pr_info("consys: patch %u/%u %s (fw ver %02x%02x, address %02x %02x %02x)\n",
			seq, cnt, consys_patch_files[i], fw->data[22], fw->data[23],
			fw->data[25], fw->data[26], fw->data[27]);
		release_firmware(fw);
		filled++;
	}
	if (!num || filled != num) {
		pr_err("consys: patch table incomplete (%u of %u)\n", filled, num);
		kfree(info);
		return -ENOENT;
	}
	wmt_lib_set_patch_num(num);
	/* the core keeps the pointer (wmt_lib_set_patch_info), as the ioctl
	 * path's kzalloc'd table did */
	wmt_lib_set_patch_info(info);
	return 0;
}

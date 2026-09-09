// SPDX-License-Identifier: GPL-2.0
/* Firmware access helpers for the MT6627 FM DSP. */

#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "fm_typedef.h"
#include "fm_dbg.h"
#include "fm_err.h"
#include "fm_patch.h"

#define MT6627_FM_FW_DIR "mediatek/mt6582/mt6627/"

static const char *fm_firmware_name(const fm_s8 *filename, char *name,
				    size_t name_len)
{
	const char *base = strrchr(filename, '/');

	base = base ? base + 1 : (const char *)filename;
	snprintf(name, name_len, MT6627_FM_FW_DIR "%s", base);
	return name;
}

fm_s32 fm_file_exist(const fm_s8 *filename)
{
	const struct firmware *fw;
	char name[96];
	int ret;

	ret = firmware_request_nowarn(&fw,
			fm_firmware_name(filename, name, sizeof(name)), NULL);
	if (ret)
		return -FM_EPATCH;

	release_firmware(fw);
	return 0;
}

fm_s32 fm_file_read(const fm_s8 *filename, fm_u8 *dst, fm_s32 len,
		    fm_s32 position)
{
	const struct firmware *fw;
	char name[96];
	size_t count;
	int ret;

	if (!dst || len < 0 || position < 0)
		return -EINVAL;

	ret = request_firmware(&fw,
			fm_firmware_name(filename, name, sizeof(name)), NULL);
	if (ret) {
		WCN_DBG(FM_ERR | CHIP, "firmware %s unavailable: %d\n", name, ret);
		return -FM_EPATCH;
	}

	if ((size_t)position >= fw->size)
		count = 0;
	else
		count = min_t(size_t, len, fw->size - position);
	memcpy(dst, fw->data + position, count);
	release_firmware(fw);

	return count;
}

fm_s32 fm_file_write(const fm_s8 *filename, fm_u8 *src, fm_s32 len,
		     fm_s32 *ppos)
{
	/* Only the optional vendor CQI text logger uses this path. */
	return -EOPNOTSUPP;
}

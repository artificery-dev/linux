// SPDX-License-Identifier: GPL-2.0
/* Mainline port: no SDIO-attached combo chip on the MT6582 (see wmt_detect_pwr.c). */
#include "wmt_detect.h"
#include "sdio_detect.h"

int sdio_detect_init(void)
{
	return 0;
}

int sdio_detect_exit(void)
{
	return 0;
}

int sdio_detect_query_chipid(int waitFlag)
{
	return -1;	/* nothing on SDIO */
}

int sdio_detect_do_autok(int chipid)
{
	return 0;
}

int hif_sdio_is_autok_support(void)
{
	return 0;
}

int hif_sdio_is_chipid_valid(int chipId)
{
	return -1;
}

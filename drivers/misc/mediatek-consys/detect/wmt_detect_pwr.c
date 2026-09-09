// SPDX-License-Identifier: GPL-2.0
/*
 * Mainline port: the vendor wmt_detect_pwr.c drives the power/reset GPIOs of
 * an EXTERNAL MT66xx combo chip so wmt_detect can probe it over SDIO. The
 * MT6582's connectivity block is on the SoC and is powered by the CONSYS
 * platform driver, so there is nothing to switch here.
 */
#include "wmt_detect.h"

int wmt_detect_chip_pwr_ctrl(int on)
{
	return 0;
}

int wmt_detect_sdio_pwr_ctrl(int on)
{
	return 0;
}

int wmt_detect_read_ext_cmb_status(void)
{
	return 0;	/* no external combo chip */
}

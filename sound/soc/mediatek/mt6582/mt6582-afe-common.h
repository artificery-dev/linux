/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6582-afe-common.h  --  MediaTek MT6582 audio front end definitions
 *
 * The MT6582 AFE is the MT8135-generation block (the same register layout the
 * MT8173 driver drives), minus the APLLs, HDMI/TDM and the 24-bit paths. The
 * offsets and bit positions here are the vendor 3.4 driver's AudDrv_Afe.h,
 * cross-checked against sound/soc/mediatek/mt8173.
 */

#ifndef _MT6582_AFE_COMMON_H_
#define _MT6582_AFE_COMMON_H_

#include <linux/clk.h>
#include <linux/regmap.h>

enum {
	MT6582_AFE_MEMIF_DL1,
	MT6582_AFE_MEMIF_NUM,
	MT6582_AFE_IO_I2S = MT6582_AFE_MEMIF_NUM,
};

enum {
	MT6582_AFE_IRQ_DL1,
	MT6582_AFE_IRQ_NUM,
};

enum {
	MT6582_CLK_INFRA_AUDIO,
	MT6582_CLK_TOP_AUDINTBUS,
	MT6582_CLK_TOP_AUDIO,
	MT6582_CLK_NUM
};

/* AFE registers (vendor AudDrv_Afe.h) */
#define AUDIO_TOP_CON0		0x0000
#define AUDIO_TOP_CON1		0x0004
#define AFE_DAC_CON0		0x0010
#define AFE_DAC_CON1		0x0014
#define AFE_I2S_CON		0x0018	/* the "2nd I2S" (in/out, master/slave) */
#define AFE_CONN0		0x0020
#define AFE_CONN1		0x0024
#define AFE_CONN2		0x0028
#define AFE_CONN3		0x002c
#define AFE_CONN4		0x0030
#define AFE_I2S_CON1		0x0034	/* I2S out */
#define AFE_I2S_CON2		0x0038	/* I2S in */
#define AFE_DL1_BASE		0x0040
#define AFE_DL1_CUR		0x0044
#define AFE_DL1_END		0x0048
#define AFE_I2S_CON3		0x004c
#define AFE_DL2_BASE		0x0050
#define AFE_DL2_CUR		0x0054
#define AFE_DL2_END		0x0058
#define AFE_AWB_BASE		0x0070
#define AFE_AWB_END		0x0078
#define AFE_AWB_CUR		0x007c
#define AFE_VUL_BASE		0x0080
#define AFE_VUL_END		0x0088
#define AFE_VUL_CUR		0x008c
#define AFE_MEMIF_MON0		0x00d0
#define AFE_ADDA_DL_SRC2_CON0	0x0108
#define AFE_ADDA_DL_SRC2_CON1	0x010c
#define AFE_ADDA_UL_SRC_CON0	0x0114
#define AFE_ADDA_UL_SRC_CON1	0x0118
#define AFE_ADDA_TOP_CON0	0x0120
#define AFE_ADDA_UL_DL_CON0	0x0124
#define AFE_SGEN_CON0		0x01f0
#define AFE_TOP_CON0		0x0200
#define AFE_IRQ_MCU_CON		0x03a0
#define AFE_IRQ_STATUS		0x03a4
#define AFE_IRQ_CLR		0x03a8
#define AFE_IRQ_CNT1		0x03ac
#define AFE_IRQ_CNT2		0x03b0
#define AFE_IRQ_MON2		0x03b8
#define AFE_MEMIF_MINLEN	0x03d0
#define AFE_MEMIF_MAXLEN	0x03d4
#define AFE_MEMIF_PBUF_SIZE	0x03d8
#define AFE_ASRC_CON0		0x0500
#define AFE_ASRC_CON21		0x0570
#define PCM_INTF_CON1		0x0530

#define MT6582_AFE_MAX_REGISTER	AFE_ASRC_CON21

/* AUDIO_TOP_CON0 */
#define AUD_TCON0_PDN_AFE		BIT(2)
#define AUD_TCON0_PDN_ADC		BIT(5)
#define AUD_TCON0_PDN_I2S		BIT(6)
#define AUD_TCON0_APB_W2T		BIT(12)
#define AUD_TCON0_APB_R2T		BIT(13)
#define AUD_TCON0_APB_SRC		BIT(14)
/*
 * The vendor driver writes AUDIO_TOP_CON0 = 0x60004000 outright when it powers
 * the block up (bits 29/30 undocumented, APB_SRC set, every PDN clear) and sets
 * PDN_AFE | PDN_I2S | APB_SRC to power it down. Reproduced as-is.
 */
#define AUD_TCON0_POWER_ON		0x60004000
#define AUD_TCON0_POWER_OFF		(AUD_TCON0_PDN_AFE | AUD_TCON0_PDN_I2S | \
					 AUD_TCON0_APB_SRC)

/* AFE_DAC_CON0 */
#define AFE_DAC_CON0_AFE_ON		BIT(0)
#define AFE_DAC_CON0_DL1_ON		BIT(1)
#define AFE_DAC_CON0_I2S_ON		BIT(5)

/*
 * AFE_I2S_CON3: the "2nd I2S out", the one whose pads leave the SoC (on the
 * Y2: GPIO 43 bit clock, 44 word clock, 46 data, muxed by the bootloader).
 * The stock HAL writes (rate << 8) | FMT_I2S | WLEN_32BIT | EN here for the
 * external DAC; AFE_I2S_CON1 (below) only feeds the on-chip ADDA path.
 */
#define AFE_I2S_CON3_EN			BIT(0)
#define AFE_I2S_CON3_WLEN_32BIT		BIT(1)
#define AFE_I2S_CON3_FORMAT_I2S		BIT(3)
#define AFE_I2S_CON3_INV_LRCK		BIT(5)
#define AFE_I2S_CON3_RATE(x)		(((x) & 0xf) << 8)

/* AFE_CONN0: DL1 (I05/I06) into the 2nd I2S out (O00/O01) */
#define AFE_CONN0_I05_O00		BIT(5)
#define AFE_CONN0_I06_O01		BIT(22)

/* FPGA_CFG1: the stock HAL sets bit 4 with the 2nd I2S out, clears it for ADDA */
#define FPGA_CFG1			0x04c4
#define FPGA_CFG1_I2S_OUT		BIT(4)

/* AFE_I2S_CON1 / AFE_I2S_CON2 */
#define AFE_I2S_CON1_EN			BIT(0)
#define AFE_I2S_CON1_WLEN_32BIT		BIT(1)
#define AFE_I2S_CON1_FORMAT_I2S		BIT(3)
#define AFE_I2S_CON1_RATE(x)		(((x) & 0xf) << 8)
#define AFE_I2S_CON1_RATE_MASK		(0xf << 8)
#define AFE_I2S_CON1_LR_SWAP		BIT(31)

/* AFE_I2S_CON (the 2nd I2S) */
#define AFE_I2S_CON_EN			BIT(0)
#define AFE_I2S_CON_WLEN_32BIT		BIT(1)
#define AFE_I2S_CON_SLAVE		BIT(2)
#define AFE_I2S_CON_FORMAT_I2S		BIT(3)
#define AFE_I2S_CON_DIR_IN		BIT(4)
#define AFE_I2S_CON_RATE(x)		(((x) & 0xf) << 8)

/* AFE_CONN1 / AFE_CONN2: DL1 (I05/I06) into the I2S out (O03/O04) */
#define AFE_CONN1_I05_O03		BIT(21)
#define AFE_CONN2_I06_O04		BIT(6)

/* AFE_ADDA_TOP_CON0 */
#define AFE_ADDA_TOP_CON0_EXT_I2S	BIT(0)

#define AFE_IRQ_STATUS_BITS		0xff
#define AFE_IRQ_CLR_ALL			BIT(6)

#endif

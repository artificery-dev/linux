// SPDX-License-Identifier: GPL-2.0
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/export.h>
#include "consys_res.h"

void __iomem *consys_io_btif, *consys_io_apdma, *consys_io_spm, *consys_io_infra,
	     *consys_io_rgu, *consys_io_conn_mcu, *consys_io_conn_top;
int consys_irq_btif, consys_irq_btif_tx_dma, consys_irq_btif_rx_dma, consys_irq_bgf;
struct clk *consys_clks[CONSYS_CLK_NR];

static atomic_t consys_clk_users[CONSYS_CLK_NR];

int consys_clk_enable(enum consys_clk id)
{
	int ret;

	if (id >= CONSYS_CLK_NR || !consys_clks[id])
		return -ENODEV;
	ret = clk_prepare_enable(consys_clks[id]);
	if (!ret)
		atomic_inc(&consys_clk_users[id]);
	return ret;
}

int consys_clk_disable(enum consys_clk id)
{
	if (id >= CONSYS_CLK_NR || !consys_clks[id])
		return -ENODEV;
	/* AP_DMA is shared with the i2c controllers, whose driver holds only a
	 * placeholder clock; gating the real gate when BTIF closes killed the
	 * audio codec's i2c. Leave that gate alone. */
	if (id == CONSYS_CLK_APDMA) {
		atomic_dec_if_positive(&consys_clk_users[id]);
		return 0;
	}
	/* The vendor code "disables by default" at init: not an error. */
	if (atomic_dec_if_positive(&consys_clk_users[id]) < 0)
		return 0;
	clk_disable_unprepare(consys_clks[id]);
	return 0;
}

/* The vendor clkmgr's clock_is_on(): "did we enable it", not the hardware bit. */
bool consys_clk_is_on(enum consys_clk id)
{
	return id < CONSYS_CLK_NR && atomic_read(&consys_clk_users[id]) > 0;
}

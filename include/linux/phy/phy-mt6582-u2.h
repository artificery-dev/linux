/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Submodes of the MT6582 USB2 PHY's PHY_MODE_USB_DEVICE, for phy_set_mode_ext().
 *
 * The PMIC's BC1.1 charger-port detector only reaches D+/D- through the
 * PHY's BC11 switch; the charger driver closes it for the length of one
 * detection and opens it again before the gadget uses the lines.
 */
#ifndef __PHY_MT6582_U2_H
#define __PHY_MT6582_U2_H

#define MT6582_U2PHY_BC11_SET	1	/* route D+/D- to the PMIC's BC1.1 detector */
#define MT6582_U2PHY_BC11_CLR	2	/* give D+/D- back to the USB core */

#endif

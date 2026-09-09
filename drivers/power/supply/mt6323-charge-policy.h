/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MT6323_CHARGE_POLICY_H
#define MT6323_CHARGE_POLICY_H
/* Hyptrace's recovery policy, isolated so its transitions can be host-tested. */
static inline unsigned int mt6323_recovery_current_code(unsigned int *checks,
                                                        unsigned int con0,
                                                        int uv)
{
    const unsigned int required = (1u << 5) | (1u << 6) | (1u << 4);
    if ((con0 & required) != required || (con0 & (1u << 7)) ||
        uv < 0 || uv >= 4150000)
        *checks = 0;
    else if (*checks < 3)
        ++*checks;
    return *checks >= 3 ? 0x6 : 0xc;
}
#endif

/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_CCT_API_H
#define _MEC_CCT_API_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "device_mec5.h"
#include "mec_defs.h"
#include "mec_retval.h"
#include "mec_cct_regs.h"
#include "mec_mmcr.h"

/* Microchip MEC5 Capture Compare timer
 * 32-bit freerun counter with divider for: 48, 24, 12, 6, 3, 1.5, 0.750, 0.375 MHz
 * 6 32-bit capture registers with capture current freerun counter on trigger event of
 * and external signal.
 *   external signal is selectable one of 16 pins.
 *   falling, risising, or both edge dection with interrupt.
 *   glitch filter where input must be stable for 3 filter clock periods.
 *   filter clock selectable: same as freerun counter.
 * 2 32-bit compare registers.
 *   sets status when freerun counter matches compare register value.
 *   can interrupt on match.
 */

/* Interfaces to any C modules */
#ifdef __cplusplus
extern "C"
{
#endif

#define MEC_CCT_CFG_FR_EN    MEC_BIT(0)
#define MEC_CCT_CFG_CMP0_EN  MEC_BIT(1)
#define MEC_CCT_CFG_CMP1_EN  MEC_BIT(2)

/* CCT clock dividers for free run counter and capture filter clock */
#define MEC_CCT_CLK_DIV_48M   0u
#define MEC_CCT_CLK_DIV_24M   1u
#define MEC_CCT_CLK_DIV_12M   2u
#define MEC_CCT_CLK_DIV_6M    3u
#define MEC_CCT_CLK_DIV_3M    4u
#define MEC_CCT_CLK_DIV_1500K 5u
#define MEC_CCT_CLK_DIV_750K  6u
#define MEC_CCT_CLK_DIV_375K  7u
#define MEC_CCT_CLK_DIV_MAX   8u

#define MEC_CCT_CAP0_ID    0u
#define MEC_CCT_CAP1_ID    1u
#define MEC_CCT_CAP2_ID    2u
#define MEC_CCT_CAP3_ID    3u
#define MEC_CCT_CAP4_ID    4u
#define MEC_CCT_CAP5_ID    5u
#define MEC_CCT_CAP_ID_MAX 6u

#define MEC_CCT_CAP_MUX_0   0
#define MEC_CCT_CAP_MUX_1   1
#define MEC_CCT_CAP_MUX_2   2
#define MEC_CCT_CAP_MUX_3   3
#define MEC_CCT_CAP_MUX_4   4
#define MEC_CCT_CAP_MUX_5   5
#define MEC_CCT_CAP_MUX_6   6
#define MEC_CCT_CAP_MUX_7   7
#define MEC_CCT_CAP_MUX_8   8
#define MEC_CCT_CAP_MUX_9   9
#define MEC_CCT_CAP_MUX_10  10
#define MEC_CCT_CAP_MUX_11  11
#define MEC_CCT_CAP_MUX_12  12
#define MEC_CCT_CAP_MUX_13  13
#define MEC_CCT_CAP_MUX_14  14
#define MEC_CCT_CAP_MUX_15  15
#define MEC_CCT_CAP_MUX_MAX 16

/* Edge interrupt detection:
 * 0 = falling edges
 * 1 = rising edges
 * 2 = both edges
 * 3 = disabled
 */
#define MEC_CCT_CAP_IDET_FE  0u
#define MEC_CCT_CAP_IDET_RE  1u
#define MEC_CCT_CAP_IDET_BE  2u
#define MEC_CCT_CAP_IDET_DIS 3u

/* Capture input filter clock selection */
#define MEC_CCT_FCLK_48M   0u
#define MEC_CCT_FCLK_24M   1u
#define MEC_CCT_FCLK_12M   2u
#define MEC_CCT_FCLK_6M    3u
#define MEC_CCT_FCLK_3M    4u
#define MEC_CCT_FCLK_1500K 5u
#define MEC_CCT_FCLK_750K  6u
#define MEC_CCT_FCLK_375K  7u
#define MEC_CCT_FCLK_DIS   8u /* filter is bypassed */

#define MEC_CCT_CAP_ID_POS   0
#define MEC_CCT_CAP_ID_MSK0  0x7u
#define MEC_CCT_CAP_ID_MSK   0x7u

#define MEC_CCT_CAP_VALID_POS 3
#define MEC_CCT_CAP_VALID     MEC_BIT(MEC_CCT_CAP_VALID_POS)

#define MEC_CCT_CAP_MUX_POS  4
#define MEC_CCT_CAP_MUX_MSK0 0xfu
#define MEC_CCT_CAP_MUX_MSK  0xf0u

#define MEC_CCT_CAP_EDGE_POS  8
#define MEC_CCT_CAP_EDGE_MSK0 0x3u
#define MEC_CCT_CAP_EDGE_MSK  0x300u

#define MEC_CCT_CAP_FCLK_POS  12
#define MEC_CCT_CAP_FCLK_MSK0 0xfu
#define MEC_CCT_CAP_FCLK_MSK  0xf000u

#define MEC_CCT_CAP_INFO(id, edge, fclk, mux) \
    (MEC_CCT_CAP_VALID | ((uint32_t)(id) & MEC_CCT_CAP_ID_MSK0) |\
     (((uint32_t)(mux) & MEC_CCT_CAP_MUX_MSK0) << MEC_CCT_CAP_MUX_POS) |\
     (((uint32_t)(edge) & MEC_CCT_CAP_EDGE_MSK0) << MEC_CCT_CAP_EDGE_POS) |\
     (((uint32_t)(fclk) & MEC_CCT_CAP_FCLK_MSK0) << MEC_CCT_CAP_FCLK_POS))

#define MEC_CCT_CAP_ID_FROM_INFO(info) (((info) & MEC_CCT_CAP_ID_MSK) >> MEC_CCT_CAP_ID_POS)

#define MEC_CCT_CAP_MUX_FROM_INFO(info) (((info) & MEC_CCT_CAP_MUX_MSK) >> MEC_CCT_CAP_MUX_POS)

#define MEC_CCT_CAP_EDGE_FROM_INFO(info) (((info) & MEC_CCT_CAP_EDGE_MSK) >> MEC_CCT_CAP_EDGE_POS)

#define MEC_CCT_CAP_FCLK_FROM_INFO(info) (((info) & MEC_CCT_CAP_FCLK_MSK) >> MEC_CCT_CAP_FCLK_POS)

enum mec_cct_irq_src_pos {
    MEC_CCT_IRQ_SRC_FR_POS = 0,
    MEC_CCT_IRQ_SRC_CAP0_POS,
    MEC_CCT_IRQ_SRC_CAP1_POS,
    MEC_CCT_IRQ_SRC_CAP2_POS,
    MEC_CCT_IRQ_SRC_CAP3_POS,
    MEC_CCT_IRQ_SRC_CAP4_POS,
    MEC_CCT_IRQ_SRC_CAP5_POS,
    MEC_CCT_IRQ_SRC_CMP0_POS,
    MEC_CCT_IRQ_SRC_CMP1_POS,
    MEC_CCT_IRQ_SRC_MAX_POS,
};

struct mec_cct_cap_config {
    uint16_t edge          : 2;
    uint16_t filter_bypass : 1;
    uint16_t rsvd          : 2;
    uint16_t filter_clk    : 3;
    uint16_t mux_sel       : 4;
    uint16_t prog_mux      : 1;
    uint16_t rsvd2         : 2;
    uint16_t enable        : 1;
};

struct mec_cct_config {
    uint16_t flags;
    uint8_t fr_clk_div;
    uint32_t comp0;
    uint32_t comp1;
    struct mec_cct_cap_config cap_cfg[MEC_CCT_CAP_ID_MAX];
};

static inline uint32_t mec_hal_cct_fr_count_get(uintptr_t cct_base)
{
    return mmcr32_rd(cct_base + MEC_CCT_FRCNT_OFS);
}

static inline uint32_t mec_hal_cct_cap_count(uintptr_t cct_base, uint8_t capid)
{
    uintptr_t raddr = (cct_base + MEC_CCT_CAP0_CNT_OFS) + (capid * 4u);

    return mmcr32_rd(raddr);
}

int mec_hal_cct_cap_cfg(uintptr_t cct_base, uint8_t capid, struct mec_cct_cap_config *capcfg);
void mec_hal_cct_cap_ctrl(uintptr_t cct_base, uint8_t capid, uint8_t edge);
void mec_hal_cct_clear_irq(uintptr_t cct_base, uint32_t imask);
void mec_hal_cct_enable_irq(uintptr_t cct_base, uint32_t ibm);
void mec_hal_cct_disable_irq(uintptr_t cct_base, uint32_t ibm);
uint8_t mec_hal_cct_fr_clkdiv_get(uintptr_t cct_base);
void mec_hal_cct_fr_enable(uintptr_t cct_base);
void mec_hal_cct_fr_disable(uintptr_t cct_base);
void mec_hal_cct_fr_reset(uintptr_t cct_base);
int mec_hal_cct_init(uintptr_t cct_base, struct mec_cct_config *cfg);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _MEC_CCT_API_H */

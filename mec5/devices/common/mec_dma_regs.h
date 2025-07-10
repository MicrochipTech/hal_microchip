/*
 * Copyright 2025 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_DMA_REGS_H
#define _MEC_DMA_REGS_H

#include "mec_defs.h"

#define MEC_DMA_BASE_ADDR       0x40002400u
#define MEC_DMA_CHAN_OFS        0x40u
#define MEC_DMA_CHAN_SIZE       0x40u

#define MEC_DMA_CHAN_BASE(ch)   \
        (MEC_DMA_BASE_ADDR + MEC_DMA_CHAN_OFS + ((uint32_t)(ch) * MEC_DMA_CHAN_SIZE))

#define MEC_DMA_GIRQ                14u
#define MEC_DMA_ZGIRQ               6u
#define MEC_DMA_CHAN_GIRQ_POS(ch)   MEC_BIT((ch) & 0x1Fu)

/* DMA Main registers */
#define MEC_DMAM_CR_OFS             0
#define MEC_DMAM_CR_ACTV_POS        0
#define MEC_DMAM_CR_SRST_POS        1

#define MEC_DMAM_DPKT_OFS           4u /* read-only last data moved by a channel */

#define MEC_DMAM_AFSM_OFS           8u /* read-only arbiter FSM state */
#define MEC_DMAM_AFSM_STATE_POS     0
#define MEC_DMAM_AFSM_STATE_MSK     MEC_GENMASK(7, 0)
#define MEC_DMAM_AFSM_STATE_GET(s)  MEC_FIELD_GET(MEC_DMAM_AFSM_STATE_MSK, (s))

/* Registers implemented by each DMA channel */
#define MEC_DMACH_ACTV_OFS          0u
#define MEC_DMACH_ACTV_EN_POS       0

#define MEC_DMACH_MSA_OFS           4u /* memory start address */
#define MEC_DMACH_MEA_OFS           8u /* memory end address */
#define MEC_DMACH_DSA_OFS           0xCu /* device start address */

#define MEC_DMACH_CR_OFS            0x10u /* channel control register */
#define MEC_DMACH_CR_HFC_RUN_POS    0 /* hardware flow control run */
#define MEC_DMACH_CR_REQ_POS        1 /* read-only */
#define MEC_DMACH_CR_DONE_POS       2 /* read-only */
#define MEC_DMACH_CR_BUSY_POS       3 /* read-only */
#define MEC_DMACH_CR_M2D_POS        8 /* memory-to-device direction */
#define MEC_DMACH_CR_HFCD_POS       9 /* HW flow control device ID */
#define MEC_DMACH_CR_HFCD_MSK       MEC_GENMASK(15, 9)
#define MEC_DMACH_CR_HFCD_SET(d)    MEC_FIELD_PREP(MEC_DMACH_CR_HFCD_MSK, (d))
#define MEC_DMACH_CR_HFCD_GET(d)    MEC_FIELD_GET(MEC_DMACH_CR_HFCD_MSK, (d))
#define MEC_DMACH_CR_INCM_POS       16
#define MEC_DMACH_CR_INCD_POS       17
#define MEC_DMACH_CR_LOCK_ARB_POS   18
#define MEC_DMACH_CR_DHFC_POS       19 /* disable HW flow control mode */
#define MEC_DMACH_CR_UNIT_POS       20
#define MEC_DMACH_CR_UNIT_MSK       MEC_GENMASK(22, 20)
#define MEC_DMACH_CR_UNIT_1         1u /* transfer unit = bytes */
#define MEC_DMACH_CR_UNIT_2         2u /* transfer unit = 2-bytes */
#define MEC_DMACH_CR_UNIT_4         4u /* transfer unit = word (4-bytes) */
#define MEC_DMACH_CR_UNIT_SET(u)    MEC_FIELD_PREP(MEC_DMACH_CR_UNIT_MSK, (u))
#define MEC_DMACH_CR_UNIT_GET(u)    MEC_FIELD_GET(MEC_DMACH_CR_UNIT_MSK, (u))
#define MEC_DMACH_CR_SFC_GO_POS     24
#define MEC_DMACH_CR_ABORT_POS      25

#define MEC_DMACH_SR_OFS            0x14u
#define MEC_DMACH_SR_BERR_POS       0
#define MEC_DMACH_SR_HW_OVFL_POS    1
#define MEC_DMACH_SR_DONE_POS       2
#define MEC_DMACH_SR_HW_TERM_POS    3

#define MEC_DMACH_IER_OFS            0x18u
#define MEC_DMACH_IER_BERR_POS       0
#define MEC_DMACH_IER_HW_OVFL_POS    1
#define MEC_DMACH_IER_DONE_POS       2
#define MEC_DMACH_IER_HW_TERM_POS    3

#define MEC_DMACH_FSM_OFS           0x1Cu /* read-only */
#define MEC_DMACH_FSM_ARBS_POS      0
#define MEC_DMACH_FSM_ARBS_MSK      MEC_GENMASK(7, 0)
#define MEC_DMACH_FSM_ARBS_GET(s)   MEC_FIELD_GET(MEC_DMACH_FSM_ARBS_MSK, (s))
#define MEC_DMACH_FSM_CTRLS_POS     8
#define MEC_DMACH_FSM_CTRLS_MSK     MEC_GENMASK(15, 8)
#define MEC_DMACH_FSM_CTRLS_GET(s)  MEC_FIELD_GET(MEC_DMACH_FSM_CTRLS_MSK, (s))

/* following channel registers are read-only 0 except for channels 0 & 1 */
#define MEC_DMACH_ALU_CR_OFS        0x20u
#define MEC_DMACH_ALU_CR_EN_POS     0
#define MEC_DMACH_ALU_CR_PT_POS     1

#define MEC_DMACH_ALU_DATA_OFS      0x24u /* 32-bit R/W */

#define MEC_DMACH_ALU_SR_OFS         0x28u /* read-only */
#define MEC_DMACH_ALU_SR_DONE_POS    0
#define MEC_DMACH_ALU_SR_BUSY_POS    1
#define MEC_DMACH_ALU_SR_PT_DONE_POS 2
#define MEC_DMACH_ALU_SR_DRDY_POS    3

#endif /* #ifndef _MEC_DMA_REGS_H */

/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_ECS_REGS_H
#define _MEC_ECS_REGS_H

#include <stddef.h>
#include <stdint.h>
#include <mec_defs.h>

/* EC Subsystem */
#ifndef MEC_ECS_BASE
#define MEC_ECS_BASE                0x4000FC00U
#endif

#define MEC_ECS_AERA_OFS            0x4u

#define MEC_ECS_AERC_OFS            0x14u
#define MEC_ECS_AERC_DIS_POS        0

#define MEC_ECS_ICR_OFS             0x18u
#define MEC_ECS_ICR_DIRECT_EN_POS   0

#define MEC_ECS_DTR_OFS             0x1Cu
#define MEC_ECS_DTR_ETM_EN_POS      0

#define MEC_ECS_DCR_OFS             0x20u
#define MEC_ECS_DCR_EN_POS          0
#define MEC_ECS_DCR_CFG_POS         1
#define MEC_ECS_DCR_CFG_JTAG        0
#define MEC_ECS_DCR_CFG_SWD_SWV     1u
#define MEC_ECS_DCR_CFG_SWD         2u
#define MEC_ECS_DCR_CFG_MSK         MEC_GENMASK(2, 1)
#define MEC_ECS_DCR_CFG_SET(n)      MEC_FIELD_PREP(MEC_ECS_DCR_CFG_MSK, (n))
#define MEC_ECS_DCR_CFG_GET(n)      MEC_FIELD_GET(MEC_ECS_DCR_CFG_MSK, (n))
#define MEC_ECS_DCR_PU_EN_POS       3
#define MEC_ECS_DCR_BSCAN_EN_POS    4
#define MEC_ECS_DCR_LOCK_EN_POS     5

#define MEC_ECS_WDT_EV_OFS          0x28u
#define MEC_ECS_WDT_EV_CNT_POS      0
#define MEC_ECS_WDT_EV_CNT_MSK      MEC_GENMASK(3, 0)
#define MEC_ECS_WDT_EV_CNT_SET(n)   MEC_FIELD_PREP(MEC_ECS_WDT_EV_CNT_MSK, (n))
#define MEC_ECS_WDT_EV_CNT_GET(n)   MEC_FIELD_GET(MEC_ECS_WDT_EV_CNT_MSK, (n))

#define MEC_ECS_PECI_DIS_OFS        0x40u
#define MEC_ECS_PECI_VREF_OFF_POS   0

#define MEC_ECS_VCI_FOVR_OFS        0x50u
#define MEC_ECS_VCI_FOVR_SSDN_POS   0

#define MEC_ECS_BROM_STS_OFS        0x54u

#define MEC_ECS_FLOCK_OFS           0x68u
#define MEC_ECS_MLOCK_OFS           0x6Cu

#define MEC_ECS_JTAG_CR_OFS         0x70u
#define MEC_ECS_JTAG_SR_OFS         0x74u
#define MEC_ECS_JTAG_TDO_OFS        0x78u
#define MEC_ECS_JTAG_TDI_OFS        0x7Cu
#define MEC_ECS_JTAG_TMS_OFS        0x80u
#define MEC_ECS_JTAG_CMD_OFS        0x84u

#define MEC_ECS_ACMP_CR_OFS         0x94u
#define MEC_ECS_ACMP_CR_MSK         0x15u
#define MEC_ECS_ACMP_CR_EN0_POS     0
#define MEC_ECS_ACMP_CR_LOCK0_POS   2
#define MEC_ECS_ACMP_CR_EN1_POS     4

#define MEC_ECS_ACMP_SLP_CR_OFS     0x98u
#define MEC_ECS_ACMP_SLP_EN0_POS    0
#define MEC_ECS_ACMP_SLP_EN1_POS    1

#define MEC_ECS_ERST_CR_OFS         0xB0u
#define MEC_ECS_ERST_CR_EN_POS      0

#define MEC_ECS_ERST_TMCR_OFS       0xB4u
#define MEC_ECS_ERST_TMV_POS        0
#define MEC_ECS_ERST_TMV_MSK        MEC_GENMASK(2, 0)
#define MEC_ECS_ERST_TMV_SET(n)     MEC_FIELD_PREP(MEC_ECS_ERST_TMV_MSK, (n))
#define MEC_ECS_ERST_TMV_GET(n)     MEC_FIELD_GET(MEC_ECS_ERST_TMV_MSK, (n))

#define MEC_ECS_ERST_SR_OFS         0xB8u
#define MEC_ECS_ERST_SR_ACTV_POS    0

#define MEC_ECS_ERST_CNTR_OFS       0xBCu
#define MEC_ECS_ERST_CNT_POS        0
#define MEC_ECS_ERST_CNT_MSK        MEC_GENMASK(18, 0)
#define MEC_ECS_ERST_CNT_GET(n)     MEC_FIELD_GET(MEC_ECS_ERST_CNT_MSK, (n))

#endif /* _MEC_ECS_REGS_H */

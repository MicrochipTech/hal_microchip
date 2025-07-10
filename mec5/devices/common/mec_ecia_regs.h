/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_ECIA_REGS_H
#define _MEC_ECIA_REGS_H

#include <stddef.h>
#include <stdint.h>

/* EC Interrupt Aggregator */
#define MEC_ECIA_BASE               0x4000E000U

#define MEC_ECIA_GIRQ_SIZE          0x14U /* 20 bytes (5 32-bit registers) */

#define MEC_ECIA_GIRQZ_MAX          19u

/* 0 <= gz <= 18 */
#define MEC_ECIA_GIRQZ_OFS(gz, regofs) \
        (((uint32_t)(gz) * MEC_ECIA_GIRQ_SIZE) + (uint32_t)(regofs))

/* b = ECIA base address, 0 <= gz <= 18 */
#define MEC_ECIA_GIRQZ_BASE(b, gz) ((uint32_t)(b) + ((uint32_t)(gz) * MEC_ECIA_GIRQ_SIZE))

/* Each GIRQ composed of 5 32-bit registers */
#define MEC_ECIA_GIRQ_STS_OFS       0u
#define MEC_ECIA_GIRQ_ENSET_OFS     0x4u /* R/W1S */
#define MEC_ECIA_GIRQ_RESULT_OFS    0x8u /* R/O = STS | ENSET */
#define MEC_ECIA_GIRQ_ENCLR_OFS     0xCu /* R/W1C. read returns ENSET */
#define MEC_ECIA_RSVD_OFS           0x10u /* R/O */

#define MEC_ECIA_AGGR_SETEN_OFS     0x200u /* R/W1S */
#define MEC_ECIA_AGGR_CLREN_OFS     0x204u /* R/W1C */
#define MEC_ECIA_AGGR_ACTIVE_OFS    0x208u /* R/O */

#endif /* _MEC_ECIA_REGS_H */

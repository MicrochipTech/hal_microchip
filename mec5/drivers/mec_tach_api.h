/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_TACH_API_H
#define _MEC_TACH_API_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mec_defs.h"
#include "mec_retval.h"

/* Interfaces to any C modules */
#ifdef __cplusplus
extern "C"
{
#endif

#define MEC5_TACH_CFG_RESET 0x01u
#define MEC5_TACH_CFG_ENABLE 0x02u
#define MEC5_TACH_CFG_FILTER_EN 0x04u
#define MEC5_TACH_CFG_CNT_INCR_CLK 0x08u
#define MEC5_TACH_CFG_OOL_INTR_EN 0x10u
#define MEC5_TACH_CFG_CNT_RDY_INTR_EN 0x20u
#define MEC5_TACH_CFG_INPUT_CHG_INTR_EN 0x40u
#define MEC5_TACH_CFG_INTERVAL_EDGES_POS 8
#define MEC5_TACH_CFG_INTERVAL_EDGES_MSK 0x300u
#define MEC5_TACH_CFG_INTERVAL_EDGES_2 0
#define MEC5_TACH_CFG_INTERVAL_EDGES_3 0x100u
#define MEC5_TACH_CFG_INTERVAL_EDGES_5 0x200u
#define MEC5_TACH_CFG_INTERVAL_EDGES_9 0x300u

#define MEC5_TACH_LIMITS(limlo, limhi) \
    (((uint32_t)(limhi) << 16) | ((uint32_t)(limlo) & 0xffffu))

enum mec5_tach_status {
    MEC5_TACH_STS_OOL = BIT(0), /* counter out of limit */
    MEC5_TACH_STS_PIN_STATE = BIT(1),
    MEC5_TACH_STS_PIN_TOGGLE = BIT(2),
    MEC5_TACH_STS_CNT_RDY = BIT(3),
};

/* forward declaration */
struct tach_regs;

int mec_tach_init(struct tach_regs *regs, uint32_t limits, uint32_t flags);
void tach_enable(struct tach_regs *regs, uint8_t enable);
uint32_t mec_tach_clock_freq(void);
uint32_t mec_tach_counter(struct tach_regs *regs);
uint32_t mec_tach_status(struct tach_regs *regs);
void mec_tach_status_clr(struct tach_regs *regs, uint32_t status);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _MEC_TACH_API_H */

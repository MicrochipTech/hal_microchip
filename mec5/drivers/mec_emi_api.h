/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_EMI_API_H
#define _MEC_EMI_API_H

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

enum mec_emi_mbox {
    MEC_EMI_EC_TO_HOST_MBOX = 0,
    MEC_EMI_HOST_TO_EC_MBOX,
};

enum mec_emi_flags {
    MEC_EMI_RESET = BIT(0),
};

enum mec_emi_swi {
    MEC_EMI_SWI_1 = BIT(1),
    MEC_EMI_SWI_2 = BIT(2),
    MEC_EMI_SWI_3 = BIT(3),
    MEC_EMI_SWI_4 = BIT(4),
    MEC_EMI_SWI_5 = BIT(5),
    MEC_EMI_SWI_6 = BIT(6),
    MEC_EMI_SWI_7 = BIT(7),
    MEC_EMI_SWI_8 = BIT(8),
    MEC_EMI_SWI_9 = BIT(9),
    MEC_EMI_SWI_10 = BIT(10),
    MEC_EMI_SWI_11 = BIT(11),
    MEC_EMI_SWI_12 = BIT(12),
    MEC_EMI_SWI_13 = BIT(13),
    MEC_EMI_SWI_14 = BIT(14),
    MEC_EMI_SWI_15 = BIT(15),
};

enum mec_emi_mem_region {
    MEC_EMI_MEM_REGION_0 = 0,
    MEC_EMI_MEM_REGION_1,
    MEC_EMI_MEM_REGION_NUM,
};

struct emi_regs; /* forward declaration */

int mec_emi_girq_ctrl(struct emi_regs *base, uint8_t enable);
int mec_emi_girq_clr(struct emi_regs *base);
uint32_t mec_emi_girq_result(struct emi_regs *base);

int mec_emi_init(struct emi_regs *regs, uint32_t flags);

#define MEC_EMI_MEMR_CFG_RW_MSK 0xfffcu
#define MEC_EMI_MEMR_CFG_RDSZ_POS 0
#define MEC_EMI_MEMR_CFG_RDSZ_MSK (MEC_EMI_MEMR_CFG_RW_MSK)
#define MEC_EMI_MEMR_CFG_WRSZ_POS 16
#define MEC_EMI_MEMR_CFG_WRSZ_MSK ((MEC_EMI_MEMR_CFG_RW_MSK) << MEC_EMI_MEMR_CFG_WRSZ_POS)

#define MEC_EMI_MEMR_CFG_SIZES(rsz, wsz) \
    ((((uint32_t)(rsz) & MEC_EMI_MEMR_CFG_RW_MSK) << MEC_EMI_MEMR_CFG_RDSZ_POS) |\
     (((uint32_t)(wsz) & MEC_EMI_MEMR_CFG_RW_MSK) << MEC_EMI_MEMR_CFG_WRSZ_POS))

/* Configure EMI memory region.
 * region is MEC_EMI_MEM_REGION_0 or MEC_EMI_MEM_REGION_1
 * mbase is memory region address in SRAM aligned >= 4 bytes
 * rwszs b[15:0] = readable sub-region size in bytes. Must be a multiple of 4 bytes.
 * rwszs b[31:16] = writeable sub-region size in bytes. Must be a multiple of 4 bytes.
 * rwszs == 0 disables the region
 *
 */
int mec_emi_mem_region_config(struct emi_regs *regs, uint8_t region,
                              uint32_t mbase, uint32_t rwszs);

int mec_emi_mbox_wr(struct emi_regs *regs, uint8_t host_to_ec, uint8_t val);
uint8_t mec_emi_mbox_rd(struct emi_regs *regs, uint8_t host_to_ec);

int mec_emi_swi_set_one(struct emi_regs *regs, uint8_t swi_pos);
int mec_emi_swi_set(struct emi_regs *regs, uint16_t swi_bit_map);
int mec_emi_swi_host_clear_enable(struct emi_regs *regs, uint16_t mask, uint16_t enable);

int mec_emi_is_appid(struct emi_regs *regs, uint8_t appid);
int mec_emi_clear_appid(struct emi_regs *regs, uint8_t appid);
int mec_emi_clear_all_appid(struct emi_regs *regs);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _MEC_EMI_API_H */
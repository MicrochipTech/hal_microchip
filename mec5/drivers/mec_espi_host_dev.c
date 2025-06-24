/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stddef.h>
#include <stdint.h>

#include <device_mec5.h>
#include "mec_defs.h"
#include "mec_espi_core.h"
#include "mec_espi_pc.h"
#include "mec_retval.h"
#include "mec_mmcr.h"
#include "mec_espi_regs.h"

/* Logical device to BAR index table. Note: not all logical devices have
 * a memory BAR. BAR indices are stored +1. A value of zero indicates no
 * BAR of that type exists for the logical device.
 */
struct ld_info {
    uint8_t ldn;
    uint8_t io_bar_idx;
    uint8_t mem_bar_idx;
    uint8_t num_sirqs;
    uint8_t sirq_idx[4];
};

const uint8_t ldn_to_xlat_tbl[MEC_ESPI_LDN_MAX] = {
    [MEC_ESPI_LDN_MBOX] = 0,         [MEC_ESPI_LDN_KBC] = 1,
    [MEC_ESPI_LDN_ACPI_EC0] = 2,     [MEC_ESPI_LDN_ACPI_EC1] = 3,
    [MEC_ESPI_LDN_ACPI_EC2] = 4,     [MEC_ESPI_LDN_ACPI_EC3] = 5,
    [MEC_ESPI_LDN_ACPI_EC4] = 6,     [MEC_ESPI_LDN_ACPI_PM1] = 7,
    [MEC_ESPI_LDN_KB_PORT92] = 8,    [MEC_ESPI_LDN_UART0] = 9,
    [MEC_ESPI_LDN_UART1] = 10,       [MEC_ESPI_LDN_UART2] = 11,
    [MEC_ESPI_LDN_UART3] = 12,       [MEC_ESPI_LDN_IOC] = 13,
    [MEC_ESPI_LDN_IOMC] = 14,        [MEC_ESPI_LDN_GLUE] = 15,
    [MEC_ESPI_LDN_EMI0] = 16,        [MEC_ESPI_LDN_EMI1] = 17,
    [MEC_ESPI_LDN_EMI2] = 18,        0xffu,
    [MEC_ESPI_LDN_RTC] = 19,         [MEC_ESPI_LDN_PP0] = 20,
    0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu,
    [MEC_ESPI_LDN_BDBG0] = 21,
    [MEC_ESPI_LDN_BDBG0_ALIAS] = 22,
    0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu,
    0xffu, 0xffu, 0xffu, 0xffu, 0xffu,
    [MEC_ESPI_LDN_TB32] = 23,
    0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu,
    0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu,
};

const struct ld_info ld_table[] = {
    { MEC_ESPI_LDN_MBOX, MEC_ESPI_IO_HBV_MBOX, MEC_ESPI_MEM_HBV_MBOX, /* 0 */
      2, {MEC_SIRQ_IDX_MBOX_E2H, MEC_SIRQ_IDX_MBOX_SMI, 0, 0} },
    { MEC_ESPI_LDN_KBC, MEC_ESPI_IO_HBV_KBC, MEC_ESPI_MEM_HBV_MAX, /* 1 */
      2, {MEC_SIRQ_IDX_KBC_KIRQ, MEC_SIRQ_IDX_KBC_MIRQ, 0, 0} },
    { MEC_ESPI_LDN_ACPI_EC0, MEC_ESPI_IO_HBV_AEC0, MEC_ESPI_MEM_HBV_AEC0, /* 2 */
      1, {MEC_SIRQ_IDX_AEC0_OBF, 0, 0, 0} },
    { MEC_ESPI_LDN_ACPI_EC1, MEC_ESPI_IO_HBV_AEC1, MEC_ESPI_MEM_HBV_AEC1, /* 3 */
      1, {MEC_SIRQ_IDX_AEC1_OBF, 0, 0, 0} },
    { MEC_ESPI_LDN_ACPI_EC2, MEC_ESPI_IO_HBV_AEC2, MEC_ESPI_MEM_HBV_AEC2, /* 4 */
      1, {MEC_SIRQ_IDX_AEC2_OBF, 0, 0, 0} },
    { MEC_ESPI_LDN_ACPI_EC3, MEC_ESPI_IO_HBV_AEC3, MEC_ESPI_MEM_HBV_AEC3, /* 5 */
      1, {MEC_SIRQ_IDX_AEC3_OBF, 0, 0, 0} },
    { MEC_ESPI_LDN_ACPI_EC4, MEC_ESPI_IO_HBV_AEC4, MEC_ESPI_MEM_HBV_AEC4, /* 6 */
      1, {MEC_SIRQ_IDX_AEC3_OBF, 0, 0, 0} },
    { MEC_ESPI_LDN_ACPI_PM1, MEC_ESPI_IO_HBV_APM1, MEC_ESPI_MEM_HBV_MAX, 0, {0, 0, 0, 0}}, /* 7 */
    { MEC_ESPI_LDN_KB_PORT92, MEC_ESPI_IO_HBV_FKB, MEC_ESPI_MEM_HBV_MAX, 0, {0, 0, 0, 0}}, /* 8 */
    { MEC_ESPI_LDN_UART0, MEC_ESPI_IO_HBV_UART0, MEC_ESPI_MEM_HBV_MAX, /* 9 */
      1, {MEC_SIRQ_IDX_UART0, 0, 0, 0} },
    { MEC_ESPI_LDN_UART1, MEC_ESPI_IO_HBV_UART1, MEC_ESPI_MEM_HBV_MAX, /* 10 */
      1, {MEC_SIRQ_IDX_UART1, 0, 0, 0}},
    { MEC_ESPI_LDN_UART2, MEC_ESPI_IO_HBV_UART2, MEC_ESPI_MEM_HBV_MAX, /* 11 */
      1, {MEC_SIRQ_IDX_UART2, 0, 0, 0}},
    { MEC_ESPI_LDN_UART3, MEC_ESPI_IO_HBV_UART3, MEC_ESPI_MEM_HBV_MAX, /* 12 */
      1, {MEC_SIRQ_IDX_UART3, 0, 0, 0}},
    { MEC_ESPI_LDN_IOC, MEC_ESPI_IO_HBV_IOC, MEC_ESPI_MEM_HBV_MAX, /* 13 */
      1, {MEC_SIRQ_IDX_EC_IRQ, 0, 0, 0}},
    { MEC_ESPI_LDN_IOMC, MEC_ESPI_IO_HBV_MC, MEC_ESPI_MEM_HBV_MAX, 0, {0, 0, 0, 0}}, /* 14 */
    { MEC_ESPI_LDN_GLUE, MEC_ESPI_IO_HBV_GL, MEC_ESPI_MEM_HBV_MAX, 0, {0, 0, 0, 0}}, /* 15 */
    { MEC_ESPI_LDN_EMI0, MEC_ESPI_IO_HBV_EMI0, MEC_ESPI_MEM_HBV_EMI0, /* 16 */
      2, {MEC_SIRQ_IDX_EMI0_HE, MEC_SIRQ_IDX_EMI0_E2H, 0, 0}},
    { MEC_ESPI_LDN_EMI1, MEC_ESPI_IO_HBV_EMI1, MEC_ESPI_MEM_HBV_EMI1, /* 17 */
      2, {MEC_SIRQ_IDX_EMI1_HE, MEC_SIRQ_IDX_EMI1_E2H, 0, 0}},
    { MEC_ESPI_LDN_EMI2, MEC_ESPI_IO_HBV_EMI2, MEC_ESPI_MEM_HBV_EMI2, /* 18 */
      2, {MEC_SIRQ_IDX_EMI2_HE, MEC_SIRQ_IDX_EMI2_E2H, 0, 0}},
    { MEC_ESPI_LDN_RTC, MEC_ESPI_IO_HBV_RTC0, MEC_ESPI_MEM_HBV_MAX, /* 19 */
      1, {MEC_SIRQ_IDX_RTC, 0, 0, 0}},
    { MEC_ESPI_LDN_PP0, MEC_ESPI_IO_HBV_PP0, MEC_ESPI_MEM_HBV_MAX, /* 20 */
      1, {MEC_SIRQ_IDX_PP0, 0, 0, 0}}, /* desktop devices */
    { MEC_ESPI_LDN_BDBG0, MEC_ESPI_IO_HBV_BDP0, MEC_ESPI_MEM_HBV_MAX, 0, {0, 0, 0, 0}}, /* 21 */
    { MEC_ESPI_LDN_BDBG0_ALIAS, MEC_ESPI_IO_HBV_BDP0A, MEC_ESPI_MEM_HBV_MAX, 0, {0, 0, 0, 0}},
    { MEC_ESPI_LDN_TB32, MEC_ESPI_IO_HBV_TB32, MEC_ESPI_MEM_HBV_TB32, 0, {0, 0, 0, 0}}, /* 23 */
};
#define LD_TBL_NUM_ENTRIES (sizeof(ld_table) / sizeof(struct ld_info))

#if 0 /* UNUSED */
static struct ld_info const *find_bar(uint8_t ldn)
{
    if (ldn >= MEC_ESPI_LDN_MAX) {
        return NULL;
    }

    for (size_t i = 0; i < LD_TBL_NUM_ENTRIES; i++) {
        if (ld_table[i].ldn == ldn) {
            return &ld_table[i];
        }
    }

    return NULL;
}
#endif /* UNUSED */

#if 0 /* UNUSED */
static inline int ldn_has_iob(uint8_t ldn)
{
    if (ldn < 32) {
        return (int)(MEC_BIT(ldn) & (uint32_t)(MEC5_ESPI_LDN_IOB_MSK_LO));
    } else {
        ldn = ldn - (uint8_t)32u;
        return (int)(MEC_BIT(ldn) & (uint32_t)(MEC5_ESPI_LDN_IOB_MSK_HI));
    }
}

static inline int ldn_has_memb(uint8_t ldn)
{
    if (ldn < 32) {
        return (int)(MEC_BIT(ldn) & (uint32_t)(MEC5_ESPI_LDN_MEMB_MSK_LO));
    } else {
        ldn = ldn - (uint8_t)32u;
        return (int)(MEC_BIT(ldn) & (uint32_t)(MEC5_ESPI_LDN_MEMB_MSK_HI));
    }
}
#endif /* 0 UNUSED */

#if 0 /* UNUSED */
static uint8_t mec_espi_sirq_get(uintptr_t regbase, uint8_t sirq_idx)
{
    if (sirq_idx >= MEC_SIRQ_IDX_MAX) {
        return MEC_ESPI_SIRQ_SLOT_DIS;
    }

    return mmcr8_rd(MEC_ESPI_SIRQ_OFS((uint32_t)sirq_idx));
}

/* Set SERIRQ slot number for specified SERIRQ index */
static void espi_sirq_set(uintptr_t regbase, uint8_t sirq_idx, uint8_t slot)
{
    if (sirq_idx >= MEC_SIRQ_IDX_MAX) {
        return;
    }

    mmcr8_wr(slot, MEC_ESPI_SIRQ_OFS((uint32_t)sirq_idx));
}
#endif /* UNUSED */
/*-----------------------------------------------------------------------*/

int mec_hal_espi_iobar_cfg(uintptr_t regbase, uint8_t ldn, uint16_t io_base, uint8_t enable)
{
    uintptr_t rb = regbase;
    uint32_t bar_val = MEC_ESPI_HBV_BASE_SET((uint32_t)io_base);
#if 0
    const struct ld_info *ldi = find_bar(ldn);
#endif
    uint8_t idx = 0, tbl_idx = 0;

    if ((rb == 0u) || (ldn >= MEC_ESPI_LDN_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];

    idx = ldi->io_bar_idx;
    if (idx >= MEC_ESPI_IO_HBV_MAX) {
        return MEC_RET_ERR_INVAL; /* Logical device does not implement an I/O BAR */
    }

    rb += MEC_ESPI_HBV_OFS(idx);

    mmcr32_clr_bit(rb, MEC_ESPI_HBV_VALID_EN_POS);
    mmcr32_update_field(rb, bar_val, MEC_ESPI_HBV_BASE_MSK);

    if (enable != 0) {
        mmcr32_set_bit(rb, MEC_ESPI_HBV_VALID_EN_POS);
    }

    return MEC_RET_OK;
}

int mec_hal_espi_iobar_enable(uintptr_t regbase, uint8_t ldn, uint8_t enable)
{
    uintptr_t rb = regbase;
    uint8_t idx = 0, tbl_idx = 0;

    if ((rb == 0) || (ldn >= MEC_ESPI_LDN_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];
    
    idx = ldi->io_bar_idx;
    if (idx >= MEC_ESPI_IO_HBV_MAX) {
        return MEC_RET_ERR_INVAL; /* Logical device does not implement an I/O BAR */
    }

    rb += MEC_ESPI_HBV_OFS(idx);

    if (enable != 0) {
        mmcr32_set_bit(rb, MEC_ESPI_HBV_VALID_EN_POS);
    } else {
        mmcr32_clr_bit(rb, MEC_ESPI_HBV_VALID_EN_POS);
    }

    return MEC_RET_OK;
}

int mec_hal_espi_iobar_is_enabled(uintptr_t regbase, uint8_t ldn)
{
    uintptr_t rb = regbase;
    uint8_t idx = 0, tbl_idx = 0;

    if ((rb == 0) || (ldn >= MEC_ESPI_LDN_MAX)) {
        return 0;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];

    idx = ldi->io_bar_idx;
    if (idx >= MEC_ESPI_IO_HBV_MAX) {
        return 0; /* Logical device does not implement an I/O BAR */
    }

    rb += MEC_ESPI_HBV_OFS(idx);

    return mmcr32_test_bit(rb, MEC_ESPI_HBV_VALID_EN_POS);
}

uint32_t mec_hal_espi_iobar_mask(uintptr_t regbase, uint8_t ldn)
{
    uintptr_t rb = regbase;
    uint32_t msk = 0;
    uint8_t idx = 0, tbl_idx = 0;

    if ((regbase == 0) || (ldn >= MEC_ESPI_LDN_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];

    idx = ldi->io_bar_idx;
    if (idx >= MEC_ESPI_IO_HBV_MAX) {
        return MEC_RET_ERR_INVAL;
    }

    rb += MEC_ESPI_ECB_OFS(idx);

    msk = mmcr32_rd(rb);
    msk = MEC_ESPI_ECB_AMSK_GET(msk);

    return msk;
}

/* Set the logical device's I/O BAR mask field.
 * NOTE1: Only ACPI_EC0 logical devices mask fiels is writable.
 * NOTE2: The mask field is only writable when the I/O BAR is not in reset
 * and the valid bit in the corresponding LDN's Host I/O BAR is not set.
 */
int mec_hal_espi_iobar_mask_set(uintptr_t regbase, uint8_t ldn, uint8_t mask)
{
    uintptr_t rb = regbase;
    uint32_t v = 0;
    uint8_t idx = 0, tbl_idx = 0;

    if ((regbase == 0) || (ldn >= MEC_ESPI_LDN_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];

    idx = ldi->io_bar_idx;
    if (idx >= MEC_ESPI_IO_HBV_MAX) { /* Logical device does not implement an I/O BAR */
        return MEC_RET_ERR_INVAL;
    }

    rb += MEC_ESPI_ECB_OFS(idx);
    
    v = MEC_ESPI_ECB_AMSK_SET((uint32_t)mask);
    mmcr32_update_field(rb, v, MEC_ESPI_ECB_AMSK_MSK);

    return MEC_RET_OK;
}

/* eSPI logical device memory BAR's are 80-bit registers packed in address space
 * resulting 16-bit alignment.
 * b[0] = 0(disabled), 1(enabled/valid)
 * b[47:16] = bits[31:0] of the Host address.
 * NOTE: If Host requires more than a 32-bit memory address, all eSPI I/O memory
 * BAR's must be located in the same 4GB region and the upper address bits are
 * specified in the MBAR Host Extended Address register.
 */
int mec_hal_espi_mbar_cfg(uintptr_t mrbase, uint8_t ldn, uint32_t mem_base, uint8_t enable)
{
    uintptr_t mrb = mrbase;
    uint32_t reg_ofs = 0;
    uint8_t idx = 0, tbl_idx = 0;

    if ((mrb == 0) || (ldn >= MEC_ESPI_LDN_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];

    idx = ldi->mem_bar_idx;
    if (idx >= MEC_ESPI_MEM_HBV_MAX) {
        return MEC_RET_ERR_INVAL; /* Logical device does not implement a memory BAR */
    }

    reg_ofs = MEC_ESPI_MC_BAR_CFG_REG_OFS(idx, MEC_ESPI_MC_BAR_CFG_VAL_HW);

    /* clear valid bit before changing */
    mmcr16_clr_bit(mrb + reg_ofs, MEC_ESPI_MC_BAR_CFG_VAL_POS);

    /* program bits[31:0] of host address as two 16-bit chunks due to register layout */
    mmcr16_wr((uint16_t)(mem_base & UINT16_MAX), mrb + reg_ofs + 2u);
    mmcr16_wr((uint16_t)(mem_base >> 16) & UINT16_MAX, mrb + reg_ofs + 4u);

    if (enable != 0) {
        mmcr16_set_bit(mrb + reg_ofs, MEC_ESPI_MC_BAR_CFG_VAL_POS);
    }

    return MEC_RET_OK;
}

/* MEC eSPI SRAM BARs have a configurable size implemented as powers of 2 from 0 to 15.
 * Hardware design requires EC SRAM memory region to be aligned to the size!
 */
int mec_hal_espi_sram_bar_ec_mem_cfg(uintptr_t mrbase, uint8_t sram_bar_id,
                                     uint32_t maddr, uint16_t size, uint8_t access,
                                     uint8_t enable)
{
    uintptr_t mrb = mrbase;
    uint32_t reg_ofs = 0;
    uint16_t vasz = 0;
    uint8_t nz = 0;

    if ((mrb == 0) || (size == 0) || (sram_bar_id >= MEC_ESPI_SRAM_BAR_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    if (((size & (size - 1u)) != 0) || (size > (32u * 1024u))) {
        return MEC_RET_ERR_INVAL; /* not power of 2 or larger than max (32KB) */
    }

    nz = 31u - __CLZ((uint32_t)size);
    if (nz > 15u) {
        nz = 15u;
    }

    reg_ofs = MEC_ESPI_MC_SBAR_OFS(sram_bar_id);

    /* disable before changing */
    mmcr16_clr_bit(mrb + reg_ofs, MEC_ESPI_MC_SBAR_VALID_POS);

    /* access and size */
    vasz = MEC_ESPI_MC_SBAR_ACC_SET((uint16_t)access);
    vasz |= MEC_ESPI_MC_SBAR_SZ_SET((uint16_t)size);
    mmcr16_update_field(mrb + reg_ofs, vasz,
                        (MEC_ESPI_MC_SBAR_ACC_MSK | MEC_ESPI_MC_SBAR_SZ_MSK));

    /* EC SRAM address */
    mmcr16_wr((uint16_t)(maddr & UINT16_MAX), mrb + reg_ofs + 2u);
    mmcr16_wr((uint16_t)(maddr >> 16) & UINT16_MAX, mrb + reg_ofs + 4u);

    if (enable != 0) {
        mmcr16_set_bit(mrb + reg_ofs, MEC_ESPI_MC_SBAR_VALID_POS);
    }

    return MEC_RET_OK;
}

int mec_hal_espi_sram_bar_cfg(uintptr_t mrbase,
                              const struct espi_mec5_sram_bar_cfg *barcfg,
                              uint8_t sram_bar_id, uint8_t enable)
{
    uintptr_t mrb = mrbase;
    uint32_t reg_ofs = 0, cfg_reg_ofs = 0;
    uint16_t v = 0;

    if ((mrb == 0) || (barcfg == 0) || (sram_bar_id >= MEC_ESPI_SRAM_BAR_MAX)
        || (barcfg->access > MEC_ESPI_SRAM_HOST_ACCESS_RW)
        || (barcfg->size > MEC_ESPI_SRAM_BAR_SIZE_32KB)) {
            return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_MC_SBAR_OFS(sram_bar_id);
    cfg_reg_ofs = MEC_ESPI_MC_SBAR_CFG_OFS(sram_bar_id);

    /* disable before modification */
    mmcr16_clr_bit(mrb + reg_ofs, MEC_ESPI_MC_SBAR_VALID_POS);

    /* region size and access attributes */
    v = MEC_ESPI_MC_SBAR_SZ_SET((uint16_t)barcfg->size);
    v |= MEC_ESPI_MC_SBAR_ACC_SET((uint16_t)barcfg->access);
    mmcr16_update_field(mrb + reg_ofs, v, (MEC_ESPI_MC_SBAR_ACC_MSK | MEC_ESPI_MC_SBAR_SZ_MSK));

    /* EC SRAM address */
    v = (uint16_t)(barcfg->maddr & UINT16_MAX);
    mmcr16_wr(v, mrb + reg_ofs + 2u);
    v = (uint16_t)(barcfg->maddr >> 16) & UINT16_MAX;
    mmcr16_wr(v, mrb + reg_ofs + 4u);

    /* Host address b[31:0] */
    v = (uint16_t)(barcfg->haddr & UINT16_MAX);
    mmcr16_wr(v, mrb + cfg_reg_ofs + 2u);
    v = (uint16_t)(barcfg->haddr >> 16) & UINT16_MAX;
    mmcr16_wr(v, mrb + cfg_reg_ofs + 4u);

    if (enable) {
        mmcr16_set_bit(mrb + reg_ofs, MEC_ESPI_MC_SBAR_VALID_POS);
    }

    return MEC_RET_OK;
}

int mec_hal_espi_sram_bar_host_addr_set(uintptr_t mrbase, uint8_t sram_bar_id, uint32_t host_addr_lsw)
{
    uintptr_t mrb = mrbase;
    uint32_t cfg_reg_ofs = 0;
    uint16_t v = 0;

    if ((mrb == 0) || (sram_bar_id >= MEC_ESPI_SRAM_BAR_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    cfg_reg_ofs = MEC_ESPI_MC_SBAR_CFG_OFS(sram_bar_id);

    v = (uint16_t)(host_addr_lsw & UINT16_MAX);
    mmcr16_wr(v, mrb + cfg_reg_ofs + 2u);
    v = (uint16_t)(host_addr_lsw >> 16) & UINT16_MAX;
    mmcr16_wr(v, mrb + cfg_reg_ofs + 4u);

    return MEC_RET_OK;
}

int mec_hal_espi_sram_bar_enable(uintptr_t mrbase, uint8_t sram_bar_id, uint8_t enable)
{
    uintptr_t mrb = mrbase;
    uint32_t reg_ofs = 0;

    if ((mrb == 0) || (sram_bar_id >= MEC_ESPI_SRAM_BAR_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_MC_SBAR_OFS(sram_bar_id);

    if (enable != 0) {
        mmcr16_set_bit(mrb + reg_ofs, MEC_ESPI_MC_SBAR_VALID_POS);
    } else {
        mmcr16_clr_bit(mrb + reg_ofs, MEC_ESPI_MC_SBAR_VALID_POS);
    }

    return MEC_RET_OK;
}

int mec_hal_espi_sram_bar_size_get(uintptr_t mrbase, uint8_t sram_bar_id, size_t *size)
{
    uintptr_t mrb = mrbase;
    uint32_t reg_ofs = 0;
    uint16_t sz = 0;

    if ((mrb == 0) || (size == NULL) || (sram_bar_id >= MEC_ESPI_SRAM_BAR_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_MC_SBAR_OFS(sram_bar_id);
    sz = mmcr16_rd(mrb + reg_ofs);
    sz = MEC_ESPI_MC_SBAR_SZ_GET(sz);

    /* size of power of 2 */
    *size = MEC_BIT(sz);

    return MEC_RET_OK;
}

int mec_hal_espi_sram_bar_access_get(uintptr_t mrbase, uint8_t sram_bar_id, int *access)
{
    uintptr_t mrb = mrbase;
    uint32_t reg_ofs = 0;
    uint16_t acc = 0;

    if ((mrb == 0) || (access == NULL) || (sram_bar_id >= MEC_ESPI_SRAM_BAR_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    reg_ofs = MEC_ESPI_MC_SBAR_OFS(sram_bar_id);
    acc = mmcr16_rd(mrb + reg_ofs);
    acc = MEC_ESPI_MC_SBAR_ACC_GET(acc);

    *access = (int)acc;

    return MEC_RET_OK;
}

/* Set host address bits[47:32] for memory BAR's
 * Each Logical device implementing a memory BAR includes Host address bits [31:0].
 * Host address bits [47:32] are the same for all memory BAR's. Therefore all memory
 * BAR's must be located in the same 4GB host address space range.
 * NOTE: this register is held in reset by chip reset, ESPI_nRESET, and nPLTRST.
 */
int mec_hal_espi_mbar_extended_addr_set(uintptr_t mrbase, uint32_t extended_addr)
{
    uintptr_t mrb = mrbase;

    if (mrb == 0) {
        return MEC_RET_ERR_INVAL;
    }

    mmcr32_wr(extended_addr, mrb + MEC_ESPI_MC_MBAR_HA_EXT);

    return MEC_RET_OK;
}

/* Program eSPI Peripheral Channel SRAM BAR extended Host address register.
 * This register contains Host memory space address bits[47:32] and applies
 * to both SRAM BARs.
 * NOTE: this register is held in reset by by chip reset, ESPI_nRESET, and nPLTRST.
 */
int mec_hal_espi_sram_bar_extended_addr_set(uintptr_t mrbase, uint32_t extended_addr)
{
    uintptr_t mrb = mrbase;

    if (mrb == 0) {
        return MEC_RET_ERR_INVAL;
    }

    mmcr32_wr(extended_addr, mrb + MEC_ESPI_MC_SBAR_HA_EXT);

    return MEC_RET_OK;
}

int mec_hal_espi_mbar_enable(uintptr_t mrbase, uint8_t ldn, uint8_t enable)
{
    uintptr_t mrb = mrbase;
    uint32_t reg_ofs = 0;
    uint8_t idx = 0, tbl_idx = 0;

    if ((mrb == 0) || (ldn >= MEC_ESPI_LDN_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];

    idx = ldi->mem_bar_idx;
    if (idx >= MEC_ESPI_MEM_HBV_MAX) {
        return MEC_RET_ERR_INVAL; /* Logical device does not implement a memory BAR */
    }

    reg_ofs = MEC_ESPI_MC_BAR_CFG_REG_OFS(idx, MEC_ESPI_MC_BAR_CFG_VAL_HW);

    if (enable) {
        mmcr16_set_bit(mrb + reg_ofs, MEC_ESPI_MC_BAR_CFG_VAL_POS);
    } else {
        mmcr16_clr_bit(mrb + reg_ofs, MEC_ESPI_MC_BAR_CFG_VAL_POS);
    }

    return MEC_RET_OK;
}

int mec_hal_espi_mbar_is_enabled(uintptr_t mrbase, uint8_t ldn)
{
    uintptr_t mrb = mrbase;
    uint32_t reg_ofs = 0;
    uint8_t idx = 0, tbl_idx = 0;
    
    if ((mrb == 0) || (ldn >= MEC_ESPI_LDN_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];

    idx = ldi->mem_bar_idx;
    if (idx >= MEC_ESPI_MEM_HBV_MAX) {
        return MEC_RET_ERR_INVAL; /* Logical device does not implement a memory BAR */
    }

    reg_ofs = MEC_ESPI_MC_BAR_CFG_REG_OFS(idx, MEC_ESPI_MC_BAR_CFG_VAL_HW);

    return mmcr16_test_bit(mrb + reg_ofs, MEC_ESPI_MC_BAR_CFG_VAL_POS);
}

/* Disable I/O and Memory BARs for a logical device */
int mec_hal_espi_bar_inhibit(uintptr_t regbase, uint8_t ldn, uint8_t inhibit)
{
    uintptr_t rb = regbase;
    uint8_t tbl_idx = 0;

    if ((rb == 0) || (ldn >= MEC_ESPI_LDN_MAX)) {
        return MEC_RET_ERR_INVAL;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];

    if ((ldi->io_bar_idx >= MEC_ESPI_IO_HBV_MAX) && (ldi->mem_bar_idx >= MEC_ESPI_MEM_HBV_MAX)) {
        return MEC_RET_ERR_INVAL; /* Logical device does not implement any BAR */
    }

    if (ldn < 32) {
        if (inhibit != 0) {
            mmcr32_set_bit(rb + MEC_ESPI_PC_BAR_INH_LSW_OFS, ldn);
        } else {
            mmcr32_clr_bit(rb + MEC_ESPI_PC_BAR_INH_LSW_OFS, ldn);
        }
    } else {
        ldn = ldn - (uint8_t)32u;
        if (inhibit != 0) {
            mmcr32_set_bit(rb + MEC_ESPI_PC_BAR_INH_MSW_OFS, ldn);
        } else {
            mmcr32_clr_bit(rb + MEC_ESPI_PC_BAR_INH_MSW_OFS, ldn);
        }
    }

    return MEC_RET_OK;
}

int mec_hal_espi_bar_inhibit_msk(uintptr_t regbase, uint8_t inhibit, uint32_t msklo, uint32_t mskhi)
{
    uintptr_t rb = regbase;

    if (rb == 0) {
        return MEC_RET_ERR_INVAL;
    }

    if (inhibit != 0) {
        mmcr32_set_bits(rb + MEC_ESPI_PC_BAR_INH_LSW_OFS, msklo);
        mmcr32_set_bits(rb + MEC_ESPI_PC_BAR_INH_MSW_OFS, mskhi);
    } else {
        mmcr32_clr_bits(rb + MEC_ESPI_PC_BAR_INH_LSW_OFS, msklo);
        mmcr32_clr_bits(rb + MEC_ESPI_PC_BAR_INH_MSW_OFS, mskhi);
    }

    return MEC_RET_OK;
}

/* ---- Logical Device Serial IRQ ---- */

/* Return the number of Serial IRQs a logical device implements */
uint8_t mec_hal_espi_ld_sirq_num(uintptr_t regbase, uint8_t ldn)
{
    uintptr_t rb = regbase;
    uint8_t tbl_idx = 0;

    if ((rb == 0) || (ldn >= MEC_ESPI_LDN_MAX)) {
        return 0;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];

    return ldi->num_sirqs;
}

/* get current slot number the specified logical device SIRQ instance is programmed to.
 * NOTE: a value of 255 (0xff) indicates this SIRQ is disabled.
 */
uint8_t mec_hal_espi_ld_sirq_get(uintptr_t regbase, uint8_t ldn, uint8_t ldn_sirq_id)
{
    uintptr_t rb = regbase;
    uint8_t tbl_idx = 0;

    if ((rb == 0) || (ldn >= MEC_ESPI_LDN_MAX) || (ldn_sirq_id > 3u)) {
        return 0;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];

    if ((ldi->num_sirqs == 0) || (ldn_sirq_id > ldi->num_sirqs)) {
        return 0xffu;
    }

    rb += ldi->sirq_idx[ldn_sirq_id];

    return mmcr8_rd(rb);
}

void mec_hal_espi_ld_sirq_set(uintptr_t regbase, uint8_t ldn, uint8_t ldn_sirq_id, uint8_t slot)
{
    uintptr_t rb = regbase;
    uint8_t tbl_idx = 0;

    if ((rb == 0) || (ldn >= MEC_ESPI_LDN_MAX) || (ldn_sirq_id > 3u)) {
        return;
    }

    tbl_idx = ldn_to_xlat_tbl[ldn];

    const struct ld_info *ldi = &ld_table[tbl_idx];

    if ((ldi->num_sirqs == 0) || (ldn_sirq_id > ldi->num_sirqs)) {
        return;
    }

    rb += ldi->sirq_idx[ldn_sirq_id];

    mmcr8_wr(slot, rb);
}

/* Generate EC_IRQ Serial IRQ to the Host using the Serial IRQ slot
 * number previously programmed by mec_espi_ld_sirq_set().
 */
int mec_hal_espi_gen_ec_sirq(uintptr_t regbase, uint8_t val)
{
    uintptr_t rb = regbase;

    if (rb == 0) {
        return MEC_RET_ERR_INVAL;
    }

    if (val != 0) {
        mmcr8_set_bit(rb + MEC_ESPI_PC_EC_SIRQ_OFS, MEC_ESPI_PC_EC_SIRQ_GEN_POS);
    } else {
        mmcr8_clr_bit(rb + MEC_ESPI_PC_EC_SIRQ_OFS, MEC_ESPI_PC_EC_SIRQ_GEN_POS);
    }

    return MEC_RET_OK;
}

/* end mec_espi_host_dev.c */

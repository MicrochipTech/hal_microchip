/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_ESPI_API_H
#define _MEC_ESPI_API_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "mec_defs.h"

/* Interfaces to any C modules */
#ifdef __cplusplus
extern "C"
{
#endif

/* ---- eSPI configuration ---- */
#define MEC_ESPI_CFG_PERIPH_CHAN_SUP_POS 0
#define MEC_ESPI_CFG_VW_CHAN_SUP_POS 1
#define MEC_ESPI_CFG_OOB_CHAN_SUP_POS 2
#define MEC_ESPI_CFG_FLASH_CHAN_SUP_POS 3
#define MEC_ESPI_CFG_FLASH_MAX_PLD_SZ_POS 4
#define MEC_ESPI_CFG_FLASH_MAX_PLD_SZ_MSK0 0x7u
#define MEC_ESPI_CFG_FLASH_MAX_PLD_SZ_MSK 0x70u
#define MEC_ESPI_CFG_MAX_SUPP_FREQ_POS 8
#define MEC_ESPI_CFG_MAX_SUPP_FREQ_MSK0 0x7u
#define MEC_ESPI_CFG_MAX_SUPP_FREQ_MSK 0x700u
#define MEC_ESPI_CFG_IO_MODE_SUPP_POS 12
#define MEC_ESPI_CFG_IO_MODE_SUPP_MSK0 0x3
#define MEC_ESPI_CFG_IO_MODE_SUPP_MSK 0x3000u
#define MEC_ESPI_CFG_ALERT_OD_SUPP_POS 14
#define MEC_ESPI_CFG_PLTRST_EXT_POS 15
#define MEC_ESPI_CFG_PC_MAX_PLD_SZ_POS 16
#define MEC_ESPI_CFG_PC_MAX_PLD_SZ_MSK0 0x7u
#define MEC_ESPI_CFG_PC_MAX_PLD_SZ_MSK 0x70000u
#define MEC_ESPI_CFG_OOB_MAX_PLD_SZ_POS 20
#define MEC_ESPI_CFG_OOB_MAX_PLD_SZ_MSK0 0x7u
#define MEC_ESPI_CFG_OOB_MAX_PLD_SZ_MSK 0x700000u
#define MEC_ESPI_CFG_VW_CNT_MAX_POS 24
#define MEC_ESPI_CFG_VW_CNT_MAX_MSK0 0x3fu
#define MEC_ESPI_CFG_VW_CNT_MAX_MSK 0x3f000000u
#define MEC_ESPI_CFG_FLASH_SHARED_MODE_POS 28
#define MEC_ESPI_CFG_FLASH_SHARED_MODE_MSK0 0x3u
#define MEC_ESPI_CFG_FLASH_SHARED_MODE_MSK 0x30000000u

#define MEC_ESPI_RESET_CHG_POS 0
#define MEC_ESPI_RESET_STATE_POS 1

#define MEC_ESPI_RESET_CHG 0x01
#define MEC_ESPI_RESET_HI 0x02

/* Each channel has an API returning a bitmap containing current state
 * of the channel enable set by the eSPI Host and if the channel enable
 * changed.
 */
#define MEC_ESPI_CHAN_ENABLED_POS 0
#define MEC_ESPI_CHAN_EN_CHG_POS 1

#define MEC_ESPI_CHAN_ENABLED 0x01
#define MEC_ESPI_CHAN_EN_CHG 0x02

#define MEC_ESPI_SIRQ_SLOT_DIS 0xffu

enum mec_espi_max_freq {
    MEC_ESPI_MAX_SUPP_FREQ_20M = 0,
    MEC_ESPI_MAX_SUPP_FREQ_25M,
    MEC_ESPI_MAX_SUPP_FREQ_33M,
    MEC_ESPI_MAX_SUPP_FREQ_50M,
    MEC_ESPI_MAX_SUPP_FREQ_66M,
};

enum mec_espi_io_mode {
    MEC_ESPI_IO_MODE_1 = 0,
    MEC_ESPI_IO_MODE_1_2,
    MEC_ESPI_IO_MODE_1_4,
    MEC_ESPI_IO_MODE_1_2_4,
};

enum mec_espi_alert_io_mode {
    MEC_ESPI_ALERT_IOM_NO_OD = 0,
    MEC_ESPI_ALERT_IOM_OD,
};

/* NOTE: OOB maximum payload size has same bit field encoding but actual size
 * is 9 bytes larger to accomodate MCTP prefix.
 */
enum mec_espi_chan_max_pld_sz {
    MEC_ESPI_CHAN_MAX_PLD_64B = 1,
};

enum mec_espi_fc_sharing {
    MEC_ESPI_FC_SHARING_MAF = 0,
    MEC_ESPI_FC_SHARING_MAF_ALT,
    MEC_ESPI_FC_SHARING_SAF,
    MEC_ESPI_FC_SHARING_MAF_SAF,
};

enum mec_espi_vw_max_count {
    MEC_ESPI_VW_MAX_COUNT_MIN = 7,
    MEC_ESPI_VW_MAX_COUNT_MAX = 0x3f,
};

enum mec_espi_global_cap {
    MEC_ESPI_CAP_MAX_FREQ = 0,
    MEC_ESPI_CAP_IO_MODE,
    MEC_ESPI_CAP_ALERT_OD,
    MEC_ESPI_CAP_PERIPH_CHAN,
    MEC_ESPI_CAP_PC_MAX_PLD_SIZE,
    MEC_ESPI_CAP_VWIRE_CHAN,
    MEC_ESPI_CAP_MAX_VW_COUNT,
    MEC_ESPI_CAP_OOB_CHAN,
    MEC_ESPI_CAP_OOB_MAX_PLD_SIZE,
    MEC_ESPI_CAP_FLASH_CHAN,
    MEC_ESPI_CAP_FC_MAX_PLD_SIZE,
    MEC_ESPI_CAP_FC_SHARING,
    MEC_ESPI_CAP_FC_MAX_RD_REQ_SIZE,
    MEC_ESPI_CAP_MAX,
};

/* ---- Logical Device I/O and Memory BAR ---- */

/* eSPI Host Logical Device numbers */
enum mec_espi_ldn {
    MEC_ESPI_LDN_MBOX = 0,
    MEC_ESPI_LDN_KBC,
    MEC_ESPI_LDN_ACPI_EC0,
    MEC_ESPI_LDN_ACPI_EC1,
    MEC_ESPI_LDN_ACPI_EC2,
    MEC_ESPI_LDN_ACPI_EC3,
    MEC_ESPI_LDN_ACPI_EC4,
    MEC_ESPI_LDN_ACPI_PM1,
    MEC_ESPI_LDN_KB_PORT92,
    MEC_ESPI_LDN_UART0,
    MEC_ESPI_LDN_UART1,
    MEC_ESPI_LDN_UART2,
    MEC_ESPI_LDN_UART3,
    MEC_ESPI_LDN_IOC,
    MEC_ESPI_LDN_IOMC,
    MEC_ESPI_LDN_GLUE,
    MEC_ESPI_LDN_EMI0,
    MEC_ESPI_LDN_EMI1,
    MEC_ESPI_LDN_EMI2,
    MEC_ESPI_LDN_RTC = 20,
    MEC_ESPI_LDN_PP0,
    MEC_ESPI_LDN_BDBG0 = 32,
    MEC_ESPI_LDN_BDBG0_ALIAS,
    MEC_ESPI_LDN_TB32 = 47,
    MEC_ESPI_LDN_EC = 66,
    MEC_ESPI_LDN_MAX,
};

/* Logical devices using Serial IRQ implement one SIRQ except:
 * Mailbox, KBC, and EMI implement two Serial IRQs each.
 * Use these enumerated types for the instance parameter of
 * sirq get/set.
 */
enum mec_espi_ld_sirq_mbox {
    MEC_ESPI_LD_MBOX_HOST_SIRQ = 0,
    MEC_ESPI_LD_MBOX_HOST_SMI,
};

enum mec_espi_ld_sirq_kbc {
    MEC_ESPI_LD_KBC_KIRQ = 0,
    MEC_ESPI_LD_KBC_MIRQ,
};

enum mec_espi_ld_sirq_emi {
    MEC_ESPI_LD_EMI_HOST_EVENT = 0,
    MEC_ESPI_LD_EMI_E2H,
};

#define MEC_ESPI_CFG_FLAG_PC_GIRQ_EN_POS 0
#define MEC_ESPI_CFG_FLAG_BM1_GIRQ_EN_POS 1
#define MEC_ESPI_CFG_FLAG_BM2_GIRQ_EN_POS 2
#define MEC_ESPI_CFG_FLAG_LTR_GIRQ_EN_POS 3
#define MEC_ESPI_CFG_FLAG_OOB_UP_GIRQ_EN_POS 4
#define MEC_ESPI_CFG_FLAG_OOB_DN_GIRQ_EN_POS 5
#define MEC_ESPI_CFG_FLAG_FC_GIRQ_EN_POS 6
#define MEC_ESPI_CFG_FLAG_ERST_GIRQ_EN_POS 7
#define MEC_ESPI_CFG_FLAG_VW_CT_GIRQ_EN_POS 8
#define MEC_ESPI_CFG_FLAG_VW_CHEN_GIRQ_EN_POS 9

struct espi_config {
    struct espi_io_regs *iobase;
    struct espi_mem_regs *mbase;
    struct espi_vw_regs *vwbase;
    uint32_t capabilities;
    uint32_t cfg_flags;
};

/* forward declarations */
struct espi_io_regs;
struct espi_mem_regs;
struct espi_vw_regs;

 /* Configure eSPI controller hardware capabilities read by the Host eSPI controller
 * soon after the Host de-asserts ESPI_RESET#.
 * NOTE eSPI is only fully reset by a full chip reset or power cycle.
 * The external ESPI_RESET# signal when asserted does hold portions of the logic
 * in reset state. Please refer to the Microchip eSPI block document.
 * This routine should be called while the Host is holding ESPI_RESET# asserted active.
 */
int mec_espi_init(struct espi_config *cfg);

int mec_espi_capability_set(struct espi_io_regs *iobase,
                            enum mec_espi_global_cap cap, uint32_t cfg);

void mec_espi_reset_change_clr(struct espi_io_regs *iobase);
void mec_espi_reset_change_intr_en(struct espi_io_regs *iobase, uint8_t enable);
void mec_espi_reset_girq_ctrl(uint8_t enable);
void mec_espi_reset_girq_status_clr(void);
uint32_t mec_espi_reset_girq_status(void);
uint32_t mec_espi_reset_girq_result(void);

/* Return bits indicating ESPI_RESET# has changed and its current state */
uint32_t mec_espi_reset_state(struct espi_io_regs *iobase);

int mec_espi_iobar_cfg(struct espi_io_regs *base, uint8_t ldn, uint16_t io_base, uint8_t enable);
int mec_espi_iobar_enable(struct espi_io_regs *base, uint8_t ldn, uint8_t enable);
int mec_espi_iobar_is_enabled(struct espi_io_regs *base, uint8_t ldn);
uint32_t mec_espi_iobar_mask(struct espi_io_regs *base, uint8_t ldn);
int mec_espi_iobar_mask_set(struct espi_io_regs *base, uint8_t ldn, uint8_t mask);

/* Inhibit both I/O and Memory BAR for a logical device or a bit map of LDNs */
int mec_espi_bar_inhibit(struct espi_io_regs *base, uint8_t ldn, uint8_t inhibit);
int mec_espi_bar_inhibit_msk(struct espi_io_regs *base, uint8_t inhibit,
                             uint32_t msklo, uint32_t mskhi);

int mec_espi_mbar_enable(struct espi_mem_regs *base, uint8_t ldn, uint8_t enable);
int mec_espi_mbar_is_enabled(struct espi_mem_regs *base, uint8_t ldn);
int mec_espi_mbar_cfg(struct espi_mem_regs *base, uint8_t ldn, uint32_t mem_base, uint8_t enable);
int mec_espi_mbar_extended_addr_set(struct espi_mem_regs *base, uint32_t extended_addr);

enum espi_mec5_sram_bar_id {
    MEC_ESPI_SRAM_BAR_0 = 0,
    MEC_ESPI_SRAM_BAR_1,
    MEC_ESPI_SRAM_BAR_MAX,
};

enum espi_mec5_sram_host_access {
    MEC_ESPI_SRAM_HOST_ACCESS_NONE = 0,
    MEC_ESPI_SRAM_HOST_ACCESS_RO,
    MEC_ESPI_SRAM_HOST_ACCESS_WO,
    MEC_ESPI_SRAM_HOST_ACCESS_RW,
};

enum espi_mec5_sram_bar_size {
    MEC_ESPI_SRAM_BAR_SIZE_1B = 0,
    MEC_ESPI_SRAM_BAR_SIZE_2B,
    MEC_ESPI_SRAM_BAR_SIZE_4B,
    MEC_ESPI_SRAM_BAR_SIZE_8B,
    MEC_ESPI_SRAM_BAR_SIZE_16B,
    MEC_ESPI_SRAM_BAR_SIZE_32B,
    MEC_ESPI_SRAM_BAR_SIZE_64B,
    MEC_ESPI_SRAM_BAR_SIZE_128B,
    MEC_ESPI_SRAM_BAR_SIZE_256B,
    MEC_ESPI_SRAM_BAR_SIZE_512B,
    MEC_ESPI_SRAM_BAR_SIZE_1KB,
    MEC_ESPI_SRAM_BAR_SIZE_2KB,
    MEC_ESPI_SRAM_BAR_SIZE_4KB,
    MEC_ESPI_SRAM_BAR_SIZE_8KB,
    MEC_ESPI_SRAM_BAR_SIZE_16KB,
    MEC_ESPI_SRAM_BAR_SIZE_32KB,
    MEC_ESPI_SRAM_BAR_SIZE_MAX,
};

struct espi_mec5_sram_bar_cfg {
    uint32_t haddr;
    uint32_t maddr;
    uint8_t size;
    uint8_t access;
};

int mec_espi_sram_bar_cfg(struct espi_mem_regs *base, const struct espi_mec5_sram_bar_cfg *barcfg,
                          uint8_t sram_bar_id, uint8_t enable);
int mec_espi_sram_bar_extended_addr_set(struct espi_mem_regs *base, uint32_t extended_addr);

/* Get/set Serial IRQ slot(interrupt) number for a logical device.
 * Some logical devices implement more than one SIRQ selected by ldn_sirq_id (zero based)
 */
uint8_t mec_espi_ld_sirq_get(struct espi_io_regs *iobase, uint8_t ldn, uint8_t ldn_sirq_id);
void mec_espi_ld_sirq_set(struct espi_io_regs *iobase, uint8_t ldn, uint8_t ldn_sirq_id, uint8_t slot);

/* Generate EC_IRQ Serial IRQ to the Host using the Serial IRQ slot
 * number previously programmed by mec_espi_ld_sirq_set().
 */
int mec_espi_gen_ec_sirq(struct espi_io_regs *iobase);

/* Enable eSPI controller after all static configuration has been performed.
 * MEC eSPI activate must be set before the Host de-asserts ESPI_RESET#.
 */
void mec_espi_activate(struct espi_io_regs *iobase, uint8_t enable);
int mec_espi_is_activated(struct espi_io_regs *iobase);

/* ---- Peripheral channel (PC) ---- */
#define MEC_ESPI_PC_BM_EN_CHG 0x1
#define MEC_ESPI_PC_BM_EN_STATE_HI 0x2
#define MEC_ESPI_PC_EN_CHG 0x1
#define MEC_ESPI_PC_EN_STATE_HI 0x2

enum mec_espi_pc_intr_pos {
    MEC_ESPI_PC_INTR_CHEN_CHG_POS = 0,
    MEC_ESPI_PC_INTR_BMEN_CHG_POS,
    MEC_ESPI_PC_INTR_BERR_POS,
    MEC_ESPI_PC_INTR_POS_MAX
};

enum mec_espi_pc_sts_pos {
    MEC_ESPI_PC_ISTS_CHEN_STATE_POS = 0,
    MEC_ESPI_PC_ISTS_CHEN_CHG_POS,
    MEC_ESPI_PC_ISTS_BMEN_STATE_POS,
    MEC_ESPI_PC_ISTS_BMEN_CHG_POS,
    MEC_ESPI_PC_ISTS_BERR_POS,
    MEC_ESPI_PC_ISTS_POS_MAX,
};

struct mec_espi_pc_last_cycle {
    uint32_t host_pc_addr_lsw;
    uint32_t host_pc_addr_msw;
    uint16_t len;
    uint8_t cycle_type;
    uint8_t tag;
};

void mec_espi_pc_ready_set(struct espi_io_regs *iobase);
/* return 1 is ready else 0 */
int mec_espi_pc_is_ready(struct espi_io_regs *iobase);

/* return bits indicating eSPI peripheral channel enable has changed and the
 * channel enable's current state.
 */
uint32_t mec_espi_pc_en_status(struct espi_io_regs *iobase);
uint32_t mec_espi_pc_bm_status(struct espi_io_regs *iobase);

/* return status bit map interpreted using mec_espi_pc_status_pos bit positions */
uint32_t mec_espi_pc_status(struct espi_io_regs *iobase);
void mec_espi_pc_status_clr(struct espi_io_regs *iobase, uint32_t bitmap);
void mec_espi_pc_status_clr_all(struct espi_io_regs *iobase);

void mec_espi_pc_intr_en(struct espi_io_regs *iobase, uint32_t bitmap);
void mec_espi_pc_intr_dis(struct espi_io_regs *iobase, uint32_t bitmap);

/* Get 64-bit address sent by Host which caused an error */
uint64_t mec_espi_pc_error_addr(struct espi_io_regs *iobase);

void mec_espi_pc_last_cycle(struct espi_io_regs *iobase,
                            struct mec_espi_pc_last_cycle *lc);

void mec_espi_pc_girq_ctrl(uint8_t enable);
void mec_espi_pc_girq_status_clr(void);
uint32_t mec_espi_pc_girq_status(void);
uint32_t mec_espi_pc_girq_result(void);

/* PC LTR */
enum mec_espi_pc_ltr_intr_pos {
    MEC_ESPI_PC_LTR_INTR_TX_DONE_POS = 0,
    MEC_ESPI_PC_LTR_INTR_START_OVR_POS = 3,
    MEC_ESPI_PC_LTR_INTR_DIS_BY_HOST_POS,
    MEC_ESPI_PC_LTR_INTR_TX_BUSY_POS = 8
};

uint32_t mec_espi_pc_ltr_status(struct espi_io_regs *iobase);
void mec_espi_pc_ltr_intr_en(struct espi_io_regs *iobase, uint32_t enmask);
void mec_espi_pc_ltr_ctrl(struct espi_io_regs *iobase, uint8_t tag, uint8_t start);
void mec_espi_pc_ltr_msg(struct espi_io_regs *iobase, uint16_t nunits, uint8_t time_unit,
                         uint8_t rsvd_bits, uint8_t max_lat);
void mec_espi_pc_ltr_girq_ctrl(uint8_t enable);
void mec_espi_pc_ltr_girq_status_clr(void);
uint32_t mec_espi_pc_ltr_girq_status(void);
uint32_t mec_espi_pc_ltr_girq_result(void);

/* ---- Virtual Wire channel (VW) ---- */

/* Configuration for a Controller-to-Target(EC) eSPI Virtual Wire group
 * host_idx is the eSPI Host Index from the eSPI specification this
 * CTVW register represents.
 * reset_src is the signal triggering CTVW hardware to load reset
 * default values into the 4 VWire source bits in this group.
 * reset_state_bm bits [3:0] are the reset state of [SRC3:SRC0]
 * src_val_bm loads each of the 4 VWire source bits based on:
 *   A 1 bit in b[3:0] causes the config API to load the value
 *   in src_val bit n+4 into the VWire SRC[n] bit. 0 <= n <= 3.
 * src_ien[4] is the interrupt enable and trigger for each VWire.
 */
#define MEC_ESPI_VW_SRC_VAL_WR_MSK 0xfu;
#define MEC_ESPI_VW_SRC_VAL_MSK 0xf0u;

enum mec_espi_vw_reset_source {
    MEC_ESPI_VW_RESET_ESPI = 0,
    MEC_ESPI_VW_RESET_SYS,
    MEC_ESPI_VW_RESET_SIO,
    MEC_ESPI_VW_RESET_PLTRST,
    MEC_ESPI_VW_RESET_MAX,
};

enum mec_espi_vw_ct_intr_en {
    MEC_ESPI_VW_CT_IEN_DIS = 0,
    MEC_ESPI_VW_CT_IEN_LL,
    MEC_ESPI_VW_CT_IEN_LH,
    MEC_ESPI_VW_CT_IEN_RE,
    MEC_ESPI_VW_CT_IEN_FE,
    MEC_ESPI_VW_CT_IEN_BE,
    MEC_ESPI_VW_CT_IEN_MAX,
};

enum mec_espi_vw_reg_idx {
    MEC_ESPI_CTVW00_REG_IDX = 0,
    MEC_ESPI_CTVW01_REG_IDX,
    MEC_ESPI_CTVW02_REG_IDX,
    MEC_ESPI_CTVW03_REG_IDX,
    MEC_ESPI_CTVW04_REG_IDX,
    MEC_ESPI_CTVW05_REG_IDX,
    MEC_ESPI_CTVW06_REG_IDX,
    MEC_ESPI_CTVW07_REG_IDX,
    MEC_ESPI_CTVW08_REG_IDX,
    MEC_ESPI_CTVW09_REG_IDX,
    MEC_ESPI_CTVW10_REG_IDX,
    MEC_ESPI_TCVW00_REG_IDX,
    MEC_ESPI_TCVW01_REG_IDX,
    MEC_ESPI_TCVW02_REG_IDX,
    MEC_ESPI_TCVW03_REG_IDX,
    MEC_ESPI_TCVW04_REG_IDX,
    MEC_ESPI_TCVW05_REG_IDX,
    MEC_ESPI_TCVW06_REG_IDX,
    MEC_ESPI_TCVW07_REG_IDX,
    MEC_ESPI_TCVW08_REG_IDX,
    MEC_ESPI_TCVW09_REG_IDX,
    MEC_ESPI_TCVW10_REG_IDX,
    MEC_ESPI_VW_MAX_REG_IDX,
};

enum mec_espi_vw_source {
    MEC_ESPI_VW_SOURCE_0 = 0,
    MEC_ESPI_VW_SOURCE_1,
    MEC_ESPI_VW_SOURCE_2,
    MEC_ESPI_VW_SOURCE_3,
    MEC_ESPI_VW_SOURCE_MAX,
};

struct espi_vw_config {
    uint8_t host_idx;
    uint8_t reset_src;
    uint8_t reset_val_bm;
    uint8_t src_val_bm;
    uint8_t src_irq_sel[4];
};

/* MEC5 CT VWires are grouped into two aggregated EC GIRQs
 * bank 0 contains CTVW00 - CTVW06 sources
 * bank 1 contains CTVW07 - CTVW10 sources
 */
enum espi_ctvw_irq_bank {
    MEC_ESPI_CTVW_IRQ_BANK_0 = 0,
    MEC_ESPI_CTVW_IRQ_BANK_1,
    MEC_ESPI_CTVW_IRQ_BANK_MAX,
};

/* mec_espi_vw_config: Update configuration of a single VWire */
#define MEC_ESPI_VW_CFG_IRQSEL_POS 0
#define MEC_ESPI_VW_CFG_IRQSEL_MSK0 0x7u
#define MEC_ESPI_VW_CFG_IRQSEL_MSK 0x7u
#define MEC_ESPI_VW_CFG_IRQSEL_DO_POS 3
#define MEC_ESPI_VW_CFG_RSTSRC_POS 4
#define MEC_ESPI_VW_CFG_RSTSRC_MSK0 0x3u
#define MEC_ESPI_VW_CFG_RSTSRC_MSK 0x30u
#define MEC_ESPI_VW_CFG_RSTSRC_DO_POS 6
#define MEC_ESPI_VW_CFG_RSTVAL_POS 8
#define MEC_ESPI_VW_CFG_RSTVAL_MSK0 0x1u
#define MEC_ESPI_VW_CFG_RSTVAL_MSK 0x100u
#define MEC_ESPI_VW_CFG_RSTVAL_DO_POS 9

/* mec_espi_vwg_config: Update configuration of all four VWires in a group */
#define MEC_ESPI_VWG_CFG_HI_POS 0
#define MEC_ESPI_VWG_CFG_RST_SRC_POS 1
#define MEC_ESPI_VWG_CFG_SRC0_RST_VAL_POS 4
#define MEC_ESPI_VWG_CFG_SRC1_RST_VAL_POS 5
#define MEC_ESPI_VWG_CFG_SRC2_RST_VAL_POS 6
#define MEC_ESPI_VWG_CFG_SRC3_RST_VAL_POS 7
#define MEC_ESPI_VWG_CFG_SRC0_VAL_POS 8
#define MEC_ESPI_VWG_CFG_SRC1_VAL_POS 9
#define MEC_ESPI_VWG_CFG_SRC2_VAL_POS 10
#define MEC_ESPI_VWG_CFG_SRC3_VAL_POS 11
#define MEC_ESPI_VWG_CFG_SRC0_IRQ_POS 12
#define MEC_ESPI_VWG_CFG_SRC1_IRQ_POS 13
#define MEC_ESPI_VWG_CFG_SRC2_IRQ_POS 14
#define MEC_ESPI_VWG_CFG_SRC3_IRQ_POS 15

#define MEC_ESPI_VWG_CFG_SRC_RST_VAL_ALL (MEC_BIT(MEC_ESPI_VWG_CFG_SRC0_RST_VAL_POS)  \
                                          | MEC_BIT(MEC_ESPI_VWG_CFG_SRC1_RST_VAL_POS) \
                                          | MEC_BIT(MEC_ESPI_VWG_CFG_SRC2_RST_VAL_POS) \
                                          | MEC_BIT(MEC_ESPI_VWG_CFG_SRC3_RST_VAL_POS))
#define MEC_ESPI_VWG_CFG_SRC_VAL_ALL (MEC_BIT(MEC_ESPI_VWG_CFG_SRC0_VAL_POS)  \
                                      | MEC_BIT(MEC_ESPI_VWG_CFG_SRC1_VAL_POS) \
                                      | MEC_BIT(MEC_ESPI_VWG_CFG_SRC2_VAL_POS) \
                                      | MEC_BIT(MEC_ESPI_VWG_CFG_SRC3_VAL_POS))
#define MEC_ESPI_VWG_CFG_SRC_IRQ_ALL (MEC_BIT(MEC_ESPI_VWG_CFG_SRC0_IRQ_POS)  \
                                      | MEC_BIT(MEC_ESPI_VWG_CFG_SRC1_IRQ_POS) \
                                      | MEC_BIT(MEC_ESPI_VWG_CFG_SRC2_IRQ_POS) \
                                      | MEC_BIT(MEC_ESPI_VWG_CFG_SRC3_IRQ_POS))
#define MEC_ESPI_VWG_CFG_ALL_PROP (MEC_BIT(MEC_ESPI_VWG_CFG_HI_POS) \
                                   | MEC_BIT(MEC_ESPI_VWG_CFG_RST_SRC_POS) \
                                   | MEC_ESPI_VWG_CFG_SRC_RST_VAL_ALL \
                                   | MEC_ESPI_VWG_CFG_SRC_VAL_ALL \
                                   | MEC_ESPI_VWG_CFG_SRC_IRQ_ALL)


/* CT VWire ECIA GIRQ functions */
int mec_espi_vw_ct_girq_ctrl(uint8_t ct_idx, uint8_t src_idx, uint8_t enable);
void mec_espi_vw_ct_girq_ctrl_all(uint8_t enable);
int mec_espi_vw_ct_girq_clr(uint8_t ct_idx, uint8_t src_idx);
int mec_espi_vw_ct_girq_clr_msk(uint8_t ct_idx, uint8_t clr_msk);
void mec_espi_vw_ct_girq_clr_all(void);
uint32_t mec_espi_vw_ct_girq_sts(uint8_t ct_idx, uint8_t src_idx);
uint32_t mec_espi_vw_ct_girq_res(uint8_t ct_idx, uint8_t src_idx);
uint32_t mec_espi_vw_ct_group_girq_sts(uint8_t ct_idx);
void mec_espi_vw_ct_group_girq_sts_clr(uint8_t ct_idx);
uint32_t mec_espi_vw_ct_group_girq_res(uint8_t ct_idx);
int mec_espi_vw_ct_group_girq_ctrl(uint8_t ct_idx, uint8_t src_msk, uint8_t enable);
uint32_t mec_espi_vw_ct_girq_bank_result(uint8_t bank);
void mec_espi_vw_ct_girq_bank_clr(uint8_t bank, uint32_t clrmsk);

void mec_espi_vw_ct_from_girq_pos(uint8_t bank, uint8_t girq_pos,
                                  uint8_t *ctidx, uint8_t *ctsrc);

int mec_espi_vw_ct_irq_sel_set(struct espi_vw_regs * const vwbase, uint8_t vw_idx,
                               uint8_t src_idx, uint8_t irq_sel);
int mec_espi_vw_ct_irq_sel_set_all(struct espi_vw_regs * const vwbase, uint8_t vw_idx,
                                   uint32_t irq_sels);

int mec_espi_vw_ct_wire_set(struct espi_vw_regs * const vwbase, uint8_t ctidx,
                            uint8_t widx, uint8_t val);
int mec_espi_vw_ct_wire_get(struct espi_vw_regs * const vwbase, uint8_t ctidx,
                            uint8_t widx, uint8_t *val);
int mec_espi_vw_ct_group_set(struct espi_vw_regs * const vwbase, uint8_t ctidx,
                             uint8_t val, uint8_t msk);
int mec_espi_vw_ct_group_get(struct espi_vw_regs * const vwbase, uint8_t ctidx,
                             uint8_t *val);

int mec_espi_vw_tc_wire_set(struct espi_vw_regs * const vwbase, uint8_t tcidx,
                            uint8_t widx, uint8_t val, uint32_t flags);
int mec_espi_vw_tc_wire_get(struct espi_vw_regs * const vwbase, uint8_t tcidx,
                            uint8_t widx, uint8_t *val);
/* Sets *val bit[0]=C2T VWire state and bit[7]=C2T VWire change status */
int mec_espi_vw_tc_wire_cs_get(struct espi_vw_regs * const vwbase, uint8_t tcidx,
                               uint8_t widx, uint8_t *val);
int mec_espi_vw_tc_group_set(struct espi_vw_regs * const vwbase, uint8_t tcidx,
                             uint8_t val, uint8_t msk, uint32_t flags);
int mec_espi_vw_tc_group_get(struct espi_vw_regs * const vwbase, uint8_t tcidx,
                             uint8_t *val);

/* Unlike the other channels where channel enable change (edge)
 * detected, the virtual channel enable state is connected
 * directly to the GIRQ status. This creates level detection
 * and is always active when VWire channel is enabled.
 */
int mec_espi_vw_is_enabled(struct espi_io_regs * const iobase);
uint32_t mec_espi_vw_en_status(struct espi_io_regs * const iobase);
void mec_espi_vw_en_status_clr(void);
void mec_espi_vw_en_ien(uint8_t enable);
uint32_t mec_espi_vw_en_result(void);

void mec_espi_vw_ready_set(struct espi_io_regs * const iobase);
int mec_espi_vw_is_ready(struct espi_io_regs * const iobase);

int mec_espi_vw_config(struct espi_vw_regs *const vwbase, uint8_t vwidx, uint8_t src_idx,
                       uint8_t host_index, uint32_t config);

int mec_espi_vwg_config(struct espi_vw_regs * const vwbase, uint8_t vwidx,
                        struct espi_vw_config *cfg, uint32_t flags);

int mec_espi_vw_ct_host_index_set(struct espi_vw_regs * const vwbase, uint8_t ctidx,
                                  uint8_t host_index);
int mec_espi_vw_ct_reset_source_get(struct espi_vw_regs * const vwbase,
                                    uint8_t ctidx, uint8_t *reset_source);
int mec_espi_vw_ct_reset_source_set(struct espi_vw_regs * const vwbase,
                                    uint8_t ctidx, uint8_t reset_source);
int mec_espi_vw_ct_reset_state_set(struct espi_vw_regs * const vwbase, uint8_t ctidx,
                                   uint8_t src_idx, uint8_t reset_state);
int mec_espi_vw_ct_irqsel_set(struct espi_vw_regs * const vwbase, uint8_t ctidx,
                              uint8_t src_idx, uint8_t irq_sel);

/* Access VWires using MEC5 VW register index and vwire source position
 * idx_src:
 *    b[7:0] = tc vw index
 *    b[11:8] = VW in group [0:3]
 * flags
 *   b[0]=1 if wire changed state wait infinite time for it to be transmitted.
 */

#define MEC_ESPI_VW_FLAG_WAIT_TC_TX_POS 0

struct mec_espi_vw {
    uint8_t vwidx; /* value from enum mec_espi_vw_reg_idx */
    uint8_t srcidx; /* VWire's source index [0 to 3] in the group */
    uint8_t val; /* VWire's value (0 or 1) */
    uint8_t msk; /* group mask, used with group API's */
};

struct mec_espi_vw_poll {
    void (*delayfp)(uint32_t);
    uint32_t delay_param;
    uint32_t nloops;
};

int mec_espi_vw_get_src(struct espi_vw_regs *const vwbase, struct mec_espi_vw *vw,
                        uint32_t flags);
int mec_espi_vw_set_src(struct espi_vw_regs *const vwbase, struct mec_espi_vw *vw,
                        uint32_t flags);
int mec_espi_vw_set_src_cs(struct espi_vw_regs *const vwbase, struct mec_espi_vw *vw,
                           const struct mec_espi_vw_poll *vwp);

int mec_espi_vw_get_src_group(struct espi_vw_regs *const vwbase, struct mec_espi_vw *vw,
                              uint32_t flags);
int mec_espi_vw_set_src_group(struct espi_vw_regs *const vwbase, struct mec_espi_vw *vw,
                              uint32_t flags);

/* Get/Set value of a eSPI Virtual Wire given
 * the VW's Host Index and source position (0-3)
 */
int mec_espi_vw_set(struct espi_vw_regs *const vwbase, uint8_t host_index,
                    uint8_t src_id, uint8_t val, uint32_t flags);

int mec_espi_vw_set_cs(struct espi_vw_regs * const vwbase, uint8_t host_index,
                       uint8_t src_id, uint8_t val, const struct mec_espi_vw_poll *vwp);

int mec_espi_vw_get(struct espi_vw_regs *const vwbase, uint8_t host_index,
                    uint8_t src_id, uint8_t *val);

/* Get/Set the group of 4 eSPI Virtual Wires for the given Host Index */
int mec_espi_vw_get_group(struct espi_vw_regs *const vwbase, uint8_t host_index,
                          uint8_t *groupval);

int mec_espi_vw_set_group(struct espi_vw_regs *const vwbase, uint8_t host_index,
                          uint8_t groupval, uint8_t groupmsk, uint32_t flags);

/* ---- Out-Of-Band channel (OOB) ---- */
enum mec_espi_oob_dir {
    MEC_ESPI_OOB_DIR_UP = 1, /* EC TX buffer to upstream Host eSPI controller */
    MEC_ESPI_OOB_DIR_DN = 2, /* Host eSPI controller to EC RX buffer */
};

enum mec_espi_oob_up_status {
    MEC_ESPI_OOB_UP_STS_DONE_POS= 0,
    MEC_ESPI_OOB_UP_STS_CHEN_CHG_POS,
    MEC_ESPI_OOB_UP_STS_BERR_POS,
    MEC_ESPI_OOB_UP_STS_OVR_POS,
    MEC_ESPI_OOB_UP_STS_BAD_REQ_POS,
};

enum mec_espi_oob_dn_status {
    MEC_ESPI_OOB_DN_STS_DONE_POS = 0,
    MEC_ESPI_OOB_DN_STS_BERR_POS,
    MEC_ESPI_OOB_DN_STS_OVR_POS,
};

enum mec_espi_oob_up_intr {
    MEC_ESPI_OOB_UP_INTR_DONE_POS = 0,
    MEC_ESPI_OOB_UP_INTR_CHEN_CHG_POS,
    MEC_ESPI_OOB_DN_INTR_DONE_POS,
};

struct mec_espi_oob_buf {
    uint32_t maddr;
    uint16_t len;
    uint16_t rx_len; /* actual received length from a RX transaction */
};

void mec_espi_oob_girq_ctrl(uint8_t enable, uint8_t msk);
void mec_espi_oob_girq_status_clr(uint8_t msk);
uint32_t mec_espi_oob_girq_status(void);
uint32_t mec_espi_oob_girq_result(void);

void mec_espi_oob_ready_set(struct espi_io_regs *iobase);
int mec_espi_oob_is_ready(struct espi_io_regs *iobase);

/* return bits indicating eSPI OOB channel enable has changed and the
 * channel enable's current state.
 */
uint32_t mec_espi_oob_en_status(struct espi_io_regs *iobase);

uint32_t mec_espi_oob_max_pkt_size(struct espi_io_regs *iobase);

int mec_espi_oob_buffer_set(struct espi_io_regs *iobase, uint8_t dir, struct mec_espi_oob_buf * buf);
void mec_espi_oob_rx_buffer_avail(struct espi_io_regs *iobase);

void mec_espi_oob_intr_ctrl(struct espi_io_regs *iobase, uint32_t msk, uint8_t en);

void mec_espi_oob_tx_start(struct espi_io_regs *iobase, uint8_t tag, uint8_t start);

/* Get the TAG in the OOB RX packet recevied from the eSPI Host */
uint8_t mec_espi_oob_rx_tag(struct espi_io_regs *iobase);

uint32_t mec_espi_oob_received_len(struct espi_io_regs *iobase);
int mec_espi_oob_tx_is_busy(struct espi_io_regs *iobase);

uint32_t mec_espi_oob_status(struct espi_io_regs *iobase, uint8_t dir);
int mec_espi_oob_is_done(uint32_t status, uint8_t dir);
int mec_espi_oob_is_error(uint32_t status, uint8_t dir);
void mec_espi_oob_status_clr_done(struct espi_io_regs *iobase, uint8_t dir);
void mec_espi_oob_status_clr_err(struct espi_io_regs *iobase, uint8_t dir);
void mec_espi_oob_status_clr_chen_change(struct espi_io_regs *iobase);
int mec_espi_oob_up_is_chan_event(uint32_t status);
void mec_espi_oob_status_clr_all(struct espi_io_regs *iobase, uint8_t dir);

/* ---- Flash Channel (FC) ---- */
enum mec_espi_fc_op {
    MEC_ESPI_FC_OP_READ = 0,
    MEC_ESPI_FC_OP_WRITE,
    MEC_ESPI_FC_OP_ERASE_S,
    MEC_ESPI_FC_OP_ERASE_L,
    MEC_ESPI_FC_OP_MAX,
};

enum mec_espi_fc_intr {
    MEC_ESPI_FC_INTR_CHEN_POS = 0,
    MEC_ESPI_FC_INTR_CHEN_CHG_POS,
    MEC_ESPI_FC_INTR_DONE_POS,
    MEC_ESPI_FC_INTR_DIS_BY_HOST_POS,
    MEC_ESPI_FC_INTR_EC_BERR_POS,
    MEC_ESPI_FC_INTR_ABORT_BY_EC_POS,
    MEC_ESPI_FC_INTR_DATA_OVRUN_POS,
    MEC_ESPI_FC_INTR_INCOMPL_POS,
    MEC_ESPI_FC_INTR_FAIL_POS,
    MEC_ESPI_FC_INTR_START_OVFL_POS,
    MEC_ESPI_FC_INTR_BAD_REQ_POS = 11,
};

struct mec_espi_fc_xfr {
    uint32_t buf_addr;
    uint32_t flash_addr;
    uint32_t byte_len;
    uint8_t operation;
    uint8_t tag;
};

#define MEC_ESPI_FC_XFR_FLAG_START_IEN_POS 0

void mec_espi_fc_ready_set(struct espi_io_regs *iobase);
int mec_espi_fc_is_ready(struct espi_io_regs *iobase);

/* return bits indicating channel enable state and enable change status */
uint32_t mec_espi_fc_en_status(struct espi_io_regs *iobase);

void mec_espi_fc_girq_ctrl(uint8_t enable);
void mec_espi_fc_girq_status_clr(void);
uint32_t mec_espi_fc_girq_status(void);
uint32_t mec_espi_fc_girq_result(void);

uint32_t mec_espi_fc_max_read_req_sz(struct espi_io_regs *iobase);
uint32_t mec_espi_fc_max_pld_sz(struct espi_io_regs *iobase);

int mec_espi_fc_is_busy(struct espi_io_regs *iobase);
void mec_espi_fc_op_start(struct espi_io_regs *iobase, uint32_t flags);
void mec_espi_fc_op_abort(struct espi_io_regs *iobase);
void mec_espi_fc_intr_ctrl(struct espi_io_regs *iobase, uint32_t msk, uint8_t en);
uint32_t mec_espi_fc_status(struct espi_io_regs *iobase);
void mec_espi_fc_status_clr(struct espi_io_regs *iobase, uint32_t msk);
int mec_espi_fc_is_error(uint32_t fc_status);

/* Return the two allowed erase block sizes in b[15:0] and b[31:16] in units
 * of KB. If only one erase size allowed both fields will be identical.
 * A return value of 0 indicates the flash channel has not been properly
 * configured during eSPI link negoitation.
 */
uint32_t mec_espi_fc_get_erase_sz(struct espi_io_regs *iobase);
int mec_espi_fc_check_erase_sz(struct espi_io_regs *iobase, uint32_t ersz_bytes);

int mec_espi_fc_xfr_start(struct espi_io_regs *iobase, struct mec_espi_fc_xfr *pxfr,
                          uint32_t flags);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _MEC_ESPI_API_H */

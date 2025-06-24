/*
 * Copyright 2025 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_ACPI_EC_REGS_H
#define _MEC_ACPI_EC_REGS_H

/* ACPI EC Controller */

/* Host visible */
#define MEC_AEC_OS_DATA0_OFS 0
#define MEC_AEC_OS_DATA1_OFS 1u
#define MEC_AEC_OS_DATA2_OFS 2u
#define MEC_AEC_OS_DATA3_OFS 3u
#define MEC_AEC_OS_CMD_OFS   4u /* WO */
#define MEC_AEC_OS_STS_OFS   4u /* RO */
#define MEC_AEC_OS_BC_OFS    5u

/* EC-only */
#define MEC_AEC_E2H_DATA0_OFS 0x100u /* 4-byte mode disabled: writes by EC set OBF */
#define MEC_AEC_E2H_DATA1_OFS 0x101u
#define MEC_AEC_E2H_DATA2_OFS 0x102u
#define MEC_AEC_E2H_DATA3_OFS 0x103u /* 4-byte mode enabled: writes by EC set OBF */
#define MEC_AEC_SR_OFS        0x104u
#define MEC_AEC_SR_OBF_POS    0 /* RO */
#define MEC_AEC_SR_IBF_POS    1 /* RO */
#define MEC_AEC_SR_UD1A_POS   2 /* RW */
#define MEC_AEC_SR_CMD_POS    3 /* RO */
#define MEC_AEC_SR_BURST_POS  4 /* RW */
#define MEC_AEC_SR_SCI_POS    5 /* RW */
#define MEC_AEC_SR_SMI_POS    6 /* RW */
#define MEC_AEC_SR_UD0A_POS   7 /* RW */
#define MEC_AEC_BC_OFS        0x105u
#define MEC_AEC_BC_4BEN_POS   0 /* enable 4-byte mode */
#define MEC_AEC_H2E_DATA0_OFS 0x108u /* 4-byte mode disabled: read by EC clears IBF */
#define MEC_AEC_H2E_DATA1_OFS 0x109u
#define MEC_AEC_H2E_DATA2_OFS 0x10au
#define MEC_AEC_H2E_DATA3_OFS 0x10bu /* 4-byte mode enabled: read by EC clears IBF */

#endif /* #ifndef _MEC_ACPI_EC_REGS_H */

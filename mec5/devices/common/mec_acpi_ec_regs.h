/*
 * Copyright (c) 2025 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_ACPI_EC_REGS_H
#define _MEC_ACPI_EC_REGS_H

#include <stddef.h>
#include <stdint.h>
#include <mec_defs.h>

/* EC Subsystem */
#ifndef MEC_ACPI_EC0_BASE
#define MEC_ACPI_EC0_BASE           0x400F0800UL
#endif

#define MEC_ACPI_EC_INST_SPACING    0x400UL

/* Runtime registers accessible by EC and Host */
#define MEC_AEC_RT_DATA0_OFS        0
#define MEC_AEC_RT_DATA1_OFS        1u
#define MEC_AEC_RT_DATA2_OFS        2u
#define MEC_AEC_RT_DATA3_OFS        3u

/* Write is command, read is status */
#define MEC_AEC_RT_CSR_OFS          4u

#define MEC_AEC_RT_BCR_OFS          5u /* byte control */

/* EC-only registers */

/* Target-to-Host Data registers */
#define MEC_AEC_T2H_DATA0_OFS       0x100u
#define MEC_AEC_T2H_DATA1_OFS       0x101u
#define MEC_AEC_T2H_DATA2_OFS       0x102u
#define MEC_AEC_T2H_DATA3_OFS       0x103u

#define MEC_AEC_SR_OFS              0x104u
#define MEC_AEC_SR_OBF_POS          0
#define MEC_AEC_SR_IBF_POS          1
#define MEC_AEC_SR_UD1A_POS         2
#define MEC_AEC_SR_CMD_POS          3
#define MEC_AEC_SR_BURST_POS        4
#define MEC_AEC_SR_SCI_EVT_POS      5
#define MEC_AEC_SR_SMI_EVT_POS      6
#define MEC_AEC_SR_UD0A_POS         7

#define MEC_AEC_SR_UD_MSK           (MEC_BIT(MEC_AEC_SR_UD0A_POS) | MEC_BIT(MEC_AEC_SR_UD1A_POS))

#define MEC_AEC_BCR_OFS             0x105u /* byte control */
#define MEC_AEC_BCR_4B_EN_POS       0 /* enable 4-byte data mode */

/* Host-to-Target Data registers */
#define MEC_AEC_H2T_DATA0_OFS       0x108u
#define MEC_AEC_H2T_DATA1_OFS       0x109u
#define MEC_AEC_H2T_DATA2_OFS       0x10Au
#define MEC_AEC_H2T_DATA3_OFS       0x10Bu

#endif /* _MEC_ACPI_EC_REGS_H */

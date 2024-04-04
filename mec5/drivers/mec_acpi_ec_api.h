/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_ACPI_EC_API_H
#define _MEC_ACPI_EC_API_H

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

/* ACPI EC Controller */

/* MEC5_ACPI_EC_INSTANCES from mec5_defs.h */
#define MEC_ACPI_EC_NUM_IRQS 2 /* number of IRQ sources per instance */

struct acpi_ec_regs;

enum mec_acpi_ec_flags {
    MEC_ACPI_EC_IBF_IRQ = BIT(0),
    MEC_ACPI_EC_OBE_IRQ = BIT(1),
    MEC_ACPI_EC_4BYTE_MODE = BIT(2),
    MEC_ACPI_EC_BURST_MODE = BIT(4),
    MEC_ACPI_EC_RESET = BIT(7),
    MEC_ACPI_EC_UD0A_SET = BIT(8),
    MEC_ACPI_EC_UD1A_SET = BIT(9),
    MEC_ACPI_EC_UD0A_ONE = BIT(16),
    MEC_ACPI_EC_UD1A_ONE = BIT(17),
};

enum mec_acpi_ec_status {
    MEC_ACPI_EC_STS_OBF = BIT(0),
    MEC_ACPI_EC_STS_IBF = BIT(1),
    MEC_ACPI_EC_STS_UD1A = BIT(2),
    MEC_ACPI_EC_STS_CMD = BIT(3),
    MEC_ACPI_EC_STS_BURST = BIT(4),
    MEC_ACPI_EC_STS_SCI = BIT(5),
    MEC_ACPI_EC_STS_SMI = BIT(6),
    MEC_ACPI_EC_STS_UD0A = BIT(7),
};

int mec_acpi_ec_init(struct acpi_ec_regs *regs, uint32_t flags);
int mec_acpi_ec_is_enabled(struct acpi_ec_regs *regs);
int mec_acpi_ec_is_4byte_mode(struct acpi_ec_regs *regs);

uint8_t mec_acpi_ec_status(struct acpi_ec_regs *regs);
void mec_acpi_ec_status_wr(struct acpi_ec_regs *regs, uint8_t val);
void mec_acpi_ec_status_set(struct acpi_ec_regs *regs, uint8_t val);
void mec_acpi_ec_status_mask(struct acpi_ec_regs *regs, uint8_t val, uint8_t msk);
uint8_t mec_acpi_ec_status_obf(struct acpi_ec_regs *regs);
uint8_t mec_acpi_ec_status_ibf(struct acpi_ec_regs *regs);

uint32_t mec_acpi_ec_host_to_ec_data_rd32(struct acpi_ec_regs *regs);
void mec_acpi_ec_host_to_ec_data_wr32(struct acpi_ec_regs *regs, uint32_t data);
uint8_t mec_acpi_ec_host_to_ec_data_rd8(struct acpi_ec_regs *regs, uint8_t offset);
void mec_acpi_ec_host_to_ec_data_wr8(struct acpi_ec_regs *regs, uint8_t offset, uint8_t data);

uint32_t mec_acpi_ec_e2h_data_rd32(struct acpi_ec_regs *regs);
void mec_acpi_ec_e2h_to_ec_data_wr32(struct acpi_ec_regs *regs, uint32_t data);
uint8_t mec_acpi_ec_e2h_data_rd8(struct acpi_ec_regs *regs, uint8_t offset);
void mec_acpi_ec_e2h_data_wr8(struct acpi_ec_regs *regs, uint8_t offset, uint8_t data);

int mec_acpi_ec_girq_en(struct acpi_ec_regs *regs, uint32_t flags);
int mec_acpi_ec_girq_dis(struct acpi_ec_regs *regs, uint32_t flags);
int mec_acpi_ec_girq_clr(struct acpi_ec_regs *regs, uint32_t flags);
uint32_t mec_acpi_ec_girq_result(struct acpi_ec_regs *regs);


#ifdef __cplusplus
}
#endif

#endif /* #ifndef _MEC_ACPI_EC_API_H */

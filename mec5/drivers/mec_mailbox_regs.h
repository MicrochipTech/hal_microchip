/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_MAILBOX_REGS_H
#define _MEC_MAILBOX_REGS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mec_defs.h"

/* Interfaces to any C modules */
#ifdef __cplusplus
extern "C"
{
#endif

#define MEC_MBOX_GIRQ     15
#define MEC_MBOX_GIRQ_POS 20

#define MEC_MBOX0_ECIA_INFO MEC_ECIA_INFO(15, 20, 7, 60)

/* Each mailbox device implements 32 8-bit mailboxes */
#define MEC_MBOX_MAX_INDEX 31

/* Mailbox registers */
#define MBOX_H2E_OFS    0x100u
#define MBOX_E2H_OFS    0x104u
#define MBOX_SMI_IS_OFS 0x108u
#define MBOX_SMI_IM_OFS 0x10cu
#define MBOX_MB_OFS(n)  (0x110u + (uint32_t)(n))

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _MEC_MAILBOX_REGS_H */

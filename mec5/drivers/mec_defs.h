/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_DEFS_H
#define _MEC_DEFS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* General Constants */
#ifndef FALSE
#define FALSE   0x00
#endif

#ifndef TRUE
#define TRUE    !FALSE
#endif

#ifndef ON
#define ON  1
#endif

#ifndef OFF
#define OFF 0
#endif

#define BIT_n_MASK(n)   (1U << (n))
#define BIT_0_MASK  (1U << 0)
#define BIT_1_MASK  (1U << 1)
#define BIT_2_MASK  (1U << 2)
#define BIT_3_MASK  (1U << 3)
#define BIT_4_MASK  (1U << 4)
#define BIT_5_MASK  (1U << 5)
#define BIT_6_MASK  (1U << 6)
#define BIT_7_MASK  (1U << 7)
#define BIT_8_MASK  (1U << 8)
#define BIT_9_MASK  (1U << 9)
#define BIT_10_MASK (1U << 10)
#define BIT_11_MASK (1U << 11)
#define BIT_12_MASK (1U << 12)
#define BIT_13_MASK (1U << 13)
#define BIT_14_MASK (1U << 14)
#define BIT_15_MASK (1U << 15)
#define BIT_16_MASK (1U << 16)
#define BIT_17_MASK (1U << 17)
#define BIT_18_MASK (1U << 18)
#define BIT_19_MASK (1U << 19)
#define BIT_20_MASK (1U << 20)
#define BIT_21_MASK (1U << 21)
#define BIT_22_MASK (1U << 22)
#define BIT_23_MASK (1U << 23)
#define BIT_24_MASK (1U << 24)
#define BIT_25_MASK (1U << 25)
#define BIT_26_MASK (1U << 26)
#define BIT_27_MASK (1U << 27)
#define BIT_28_MASK (1U << 28)
#define BIT_29_MASK (1U << 29)
#define BIT_30_MASK (1U << 30)
#define BIT_31_MASK (1U << 31)

#ifndef BIT
#define BIT(n)      (1ul << (n))
#endif

#ifndef BIT32
#define BIT32(n)    (1ul << (n))
#endif

#ifndef BIT64
#define BIT64(n) (1ULL << (n))
#endif

#ifndef BIT_SET
#define BIT_SET(d, pos) ((d) |= BIT(pos))
#endif

#ifndef BIT_CLR
#define BIT_CLR(d, pos) ((d) &= ~BIT(pos))
#endif

#ifndef LSHFT
#define LSHFT(v, s) (((uint32_t)(v)) << ((s) & 0x1Fu))
#endif

#ifndef FIELD_VAL
#define FIELD_VAL(val, pos) ((uint32_t)(val) << (pos))
#endif

#ifndef IS_PTR_ALIGNED16
#define IS_PTR_ALIGNED16(ptr) ((((uintptr_t)(ptr)) & 0x01U) == 0)
#endif

#ifndef IS_PTR_ALIGNED32
#define IS_PTR_ALIGNED32(ptr) ((((uintptr_t)(ptr)) & 0x03U) == 0)
#endif

#ifndef IS_PTR_ALIGNED64
#define IS_PTR_ALIGNED64(ptr) ((((uintptr_t)(ptr)) & 0x07U) == 0)
#endif

#ifndef IS_PTR_ALIGNED128
#define IS_PTR_ALIGNED128(ptr) ((((uintptr_t)(ptr)) & 0x0fU) == 0)
#endif

/* Align pointer in memory region. NOTE: memory region must be large
 * enough to move pointer forward to an aligned address.
 */
#ifndef PTR_ALIGN4
#define PTR_ALIGN4(ptr)     (((uintptr_t)(ptr) + 4U) & ~0x3u)
#endif

#ifndef PTR_ALIGN8
#define PTR_ALIGN8(ptr)     (((uintptr_t)(ptr) + 8U) & ~0x7u)
#endif

#ifndef PTR_ALIGN16
#define PTR_ALIGN16(ptr)    (((uintptr_t)(ptr) + 16U) & ~0xFu)
#endif

#define MMCR8(a) *(volatile uint8_t *)(a)
#define MMCR16(a) *(volatile uint16_t *)(a)
#define MMCR32(a) *(volatile uint32_t *)(a)

#define MMCR8_WR(a, b) *(volatile uint8_t *)(a) = (uint8_t)(b)
#define MMCR8_RD(a) *(volatile uint8_t *)(a)
#define MMCR16_WR(a, b) *(volatile uint16_t *)(a) = (uint16_t)(b)
#define MMCR16_RD(a) *(volatile uint16_t *)(a)
#define MMCR32_WR(a, b) *(volatile uint32_t *)(a) = (uint32_t)(b)
#define MMCR32_RD(a) *(volatile uint32_t *)(a)

struct mec_buf {
    void *data;
    size_t len;
};

struct mec_buf_set {
    struct mchp_buf *bufptr;
    size_t count;
};

struct mec_buf_link {
    void *data;
    size_t len;
    struct mec_buf_link *next;
};

#endif /* #ifndef _MEC_DEFS_H */

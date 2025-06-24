/*
 * Copyright 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC_MMCR_H
#define _MEC_MMCR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "mec_defs.h"

static ALWAYS_INLINE uint8_t mmcr8_rd(uintptr_t addr)
{
    return *(volatile uint8_t *)addr;
}

static ALWAYS_INLINE void mmcr8_wr(uint8_t data, uintptr_t addr)
{
    *(volatile uint8_t *)addr = data;
}

static ALWAYS_INLINE uint16_t mmcr16_rd(uintptr_t addr)
{
    return *(volatile uint16_t *)addr;
}

static ALWAYS_INLINE void mmcr16_wr(uint16_t data, uintptr_t addr)
{
    *(volatile uint16_t *)addr = data;
}

static ALWAYS_INLINE uint32_t mmcr32_rd(uintptr_t addr)
{
    return *(volatile uint32_t *)addr;
}

static ALWAYS_INLINE void mmcr32_wr(uint32_t data, uintptr_t addr)
{
    *(volatile uint32_t *)addr = data;
}

static ALWAYS_INLINE void mmcr8_set_bit(uintptr_t addr, uint8_t bit)
{
	uint8_t temp = *(volatile uint8_t *)addr;

	*(volatile uint8_t *)addr = temp | (1U << bit);
}

static ALWAYS_INLINE void mmcr16_set_bit(uintptr_t addr, uint8_t bit)
{
	uint16_t temp = *(volatile uint16_t *)addr;

	*(volatile uint16_t *)addr = temp | (1U << bit);
}

static ALWAYS_INLINE void mmcr32_set_bit(uintptr_t addr, uint8_t bit)
{
	uint32_t temp = *(volatile uint32_t *)addr;

	*(volatile uint32_t *)addr = temp | (1U << bit);
}

static ALWAYS_INLINE void mmcr8_clr_bit(uintptr_t addr, uint8_t bit)
{
	uint8_t temp = *(volatile uint8_t *)addr;

	*(volatile uint8_t *)addr = temp & ~(1U << bit);
}

static ALWAYS_INLINE void mmcr16_clr_bit(uintptr_t addr, uint8_t bit)
{
	uint16_t temp = *(volatile uint16_t *)addr;

	*(volatile uint16_t *)addr = temp & ~(1U << bit);
}

static ALWAYS_INLINE void mmcr32_clr_bit(uintptr_t addr, uint8_t bit)
{
	uint32_t temp = *(volatile uint32_t *)addr;

	*(volatile uint32_t *)addr = temp & ~(1U << bit);
}

static ALWAYS_INLINE int mmcr8_test_bit(uintptr_t addr, uint8_t bit)
{
	uint8_t temp = *(volatile uint8_t *)addr;

	return (int)(temp & (1U << bit));
}

static ALWAYS_INLINE int mmcr16_test_bit(uintptr_t addr, uint8_t bit)
{
	uint16_t temp = *(volatile uint16_t *)addr;

	return (int)(temp & (1U << bit));
}

static ALWAYS_INLINE int mmcr32_test_bit(uintptr_t addr, uint8_t bit)
{
	uint32_t temp = *(volatile uint32_t *)addr;

	return (int)(temp & (1U << bit));
}

static ALWAYS_INLINE void mmcr8_set_bits(uintptr_t addr, uint8_t mask)
{
	uint16_t temp = *(volatile uint8_t *)addr;

	*(volatile uint8_t *)addr = temp | mask;
}

static ALWAYS_INLINE void mmcr16_set_bits(uintptr_t addr, uint16_t mask)
{
	uint16_t temp = *(volatile uint16_t *)addr;

	*(volatile uint16_t *)addr = temp | mask;
}

static ALWAYS_INLINE void mmcr32_set_bits(uintptr_t addr, uint32_t mask)
{
	uint32_t temp = *(volatile uint32_t *)addr;

	*(volatile uint32_t *)addr = temp | mask;
}

static ALWAYS_INLINE void mmcr8_clr_bits(uintptr_t addr, uint8_t mask)
{
	uint8_t temp = *(volatile uint8_t *)addr;

	*(volatile uint8_t *)addr = temp & (uint8_t)~mask;
}

static ALWAYS_INLINE void mmcr16_clr_bits(uintptr_t addr, uint16_t mask)
{
	uint16_t temp = *(volatile uint16_t *)addr;

	*(volatile uint16_t *)addr = temp & (uint16_t)~mask;
}

static ALWAYS_INLINE void mmcr32_clr_bits(uintptr_t addr, uint32_t mask)
{
	uint32_t temp = *(volatile uint32_t *)addr;

	*(volatile uint32_t *)addr = temp & (uint32_t)~mask;
}

static ALWAYS_INLINE int mmcr8_test_and_clr_bit(uintptr_t addr, uint8_t bit)
{
	int ret;

	ret = mmcr8_test_bit(addr, bit);
	mmcr8_clr_bit(addr, bit);

	return ret;
}

static ALWAYS_INLINE int mmcr16_test_and_clr_bit(uintptr_t addr, uint8_t bit)
{
	int ret;

	ret = mmcr16_test_bit(addr, bit);
	mmcr16_clr_bit(addr, bit);

	return ret;
}

static ALWAYS_INLINE int mmcr32_test_and_clr_bit(uintptr_t addr, uint8_t bit)
{
	int ret;

	ret = mmcr32_test_bit(addr, bit);
	mmcr32_clr_bit(addr, bit);

	return ret;
}

static ALWAYS_INLINE void mmcr8_update_field(uintptr_t addr, uint8_t val, uint8_t msk)
{
    uint8_t temp = mmcr8_rd(addr);

    temp = (temp & (uint8_t)~msk) | (val & msk);
    mmcr8_wr(temp, addr);
}

static ALWAYS_INLINE void mmcr16_update_field(uintptr_t addr, uint16_t val, uint16_t msk)
{
    uint16_t temp = mmcr16_rd(addr);

    temp = (temp & (uint16_t)~msk) | (val & msk);
    mmcr16_wr(temp, addr);
}

static ALWAYS_INLINE void mmcr32_update_field(uintptr_t addr, uint32_t val, uint32_t msk)
{
    uint32_t temp = mmcr32_rd(addr);

    temp = (temp & (uint32_t)~msk) | (val & msk);
    mmcr32_wr(temp, addr);
}

#endif /* #ifndef _MEC_MMCR_H */

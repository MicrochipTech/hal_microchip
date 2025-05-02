/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MEC5_VBAT_MEM_128B_V1_H
#define _MEC5_VBAT_MEM_128B_V1_H

#include <stdint.h>

#ifndef __I
#ifdef __cplusplus
#define __I volatile
#else
#define __I volatile const
#endif
#endif

#ifndef __O
#define __O volatile
#endif

#ifndef __IO
#define __IO volatile
#endif

#ifndef __IM
#define __IM volatile const
#endif

#ifndef __OM
#define __OM volatile
#endif

#ifndef __IOM
#define __IOM volatile
#endif

/** @addtogroup Device_Peripheral_peripherals
  * @{
  */

/**
  * @brief VBAT powered 128 byte SRAM (MEC_VBATM)
  */

typedef struct mec_vbatm_regs {                 /*!< (@ 0x4000A800) MEC_VBATM Structure                                        */
  __IOM uint8_t   VBMEM[128];                   /*!< (@ 0x00000000) 8-bit access to VBAT memory                                */
} MEC_VBATM_Type;                               /*!< Size = 128 (0x80)                                                         */

/** @} */ /* End of group Device_Peripheral_peripherals */

#endif /* _MEC5_VBAT_MEM_128B_V1_H */

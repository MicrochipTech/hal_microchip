/**
 *
 * Copyright (c) 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _DEVICE_MEC5_H
#define _DEVICE_MEC5_H

#if defined(CONFIG_SOC_SERIES_MEC174X)
    #include <mec174x_specs.h>
    #include <mec174x.h>
#elif defined(CONFIG_SOC_SERIES_MEC175X)
    #include <mec175x_specs.h>
    #include <mec175x.h>
#elif defined(CONFIG_SOC_SERIES_MECH172X)
    #include <mech172x_specs.h>
    #include <mech172x.h>
#elif defined(CONFIG_SOC_SERIES_MEC165XB)
    #include <mec165xb_specs.h>
    #include <mec165xb.h>
#else
    #error "No valid SoC defined!"
#endif

#endif /* _DEVICE_MEC5_H */

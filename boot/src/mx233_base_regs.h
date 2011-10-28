//------------------------------------------------------------------------------
//
// Copyright (c) Microsoft Corporation.  All rights reserved.
//
//
// Use of this source code is subject to the terms of the Microsoft end-user
// license agreement (EULA) under which you licensed this SOFTWARE PRODUCT.
// If you did not accept the terms of the EULA, you are not authorized to use
// this source code. For a copy of the EULA, please see the LICENSE.RTF on your
// install media.
//
//------------------------------------------------------------------------------
//
//  Copyright (C) 2007-2009, Freescale Semiconductor, Inc. All Rights Reserved.
//  THIS SOURCE CODE, AND ITS USE AND DISTRIBUTION, IS SUBJECT TO THE TERMS
//  AND CONDITIONS OF THE APPLICABLE LICENSE AGREEMENT
//
//------------------------------------------------------------------------------
//
//  Header: mx233_base_regs.h
//
//  This header file defines the Physical Addresses (PA) of
//  the base registers for the System on Chip (SoC) components.
//
//  Provides memory map definitions that are specific to the MX233
//  architecture.
//
//-----------------------------------------------------------------------------

#ifndef __MX233_BASE_REGS_H
#define __MX233_BASE_REGS_H

#if __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------------
//  INFORMATION
//
//  The physical addresses for SoC registers are fixed, hence they are defined
//  in the CPU's common directory. The virtual addresses of the SoC registers
//  are defined by the OEM and are configured in the platform's configuration
//  directory by the file: .../PLATFORM/<NAME>/SRC/CONFIG/CPU_BASE_REG_CFG.H.
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//  NAMING CONVENTIONS
//
//  CPU_BASE_REG_ is the standard prefix for CPU base registers.
//
//  Memory ranges are accessed using physical, uncached, or cached addresses,
//  depending on the system state. The following abbreviations are used for
//  each addressing type:
//
//      PA - physical address
//      UA - uncached virtual address
//      CA - cached virtual address
//
//  The naming convention for CPU base registers is:
//
//      CPU_BASE_REG_<ADDRTYPE>_<SUBSYSTEM>
//
//-----------------------------------------------------------------------------

#define CSP_BASE_REG_PA_ICOLL                    (0x80000000)
#define CSP_BASE_REG_PA_APBH                     (0x80004000)
#define CSP_BASE_REG_PA_ECC8                     (0x80008000)
#define CSP_BASE_REG_PA_BCH                      (0x8000A000)
#define CSP_BASE_REG_PA_GPMI                     (0x8000C000)
#define CSP_BASE_REG_PA_SSP1                     (0x80010000)
#define CSP_BASE_REG_PA_PINCTRL                  (0x80018000)
#define CSP_BASE_REG_PA_DIGCTL                   (0x8001C000)
#define CSP_BASE_REG_PA_EMI                      (0x80020000)
#define CSP_BASE_REG_PA_APBX                     (0x80024000)
#define CSP_BASE_REG_PA_DCP                      (0x80028000)
#define CSP_BASE_REG_PA_PXP                      (0x8002A000)   
#define CSP_BASE_REG_PA_OCOTP                    (0x8002C000)
#define CSP_BASE_REG_PA_LCDIF                    (0x80030000)
#define CSP_BASE_REG_PA_SSP2                     (0x80034000)
#define CSP_BASE_REG_PA_TVENC                    (0x80038000)
#define CSP_BASE_REG_PA_CLKCTRL                  (0x80040000)
#define CSP_BASE_REG_PA_SAIF1                    (0x80042000)
#define CSP_BASE_REG_PA_POWER                    (0x80044000)
#define CSP_BASE_REG_PA_SAIF2                    (0x80046000)
#define CSP_BASE_REG_PA_AUDIOOUT                 (0x80048000)
#define CSP_BASE_REG_PA_AUDIOIN                  (0x8004C000)
#define CSP_BASE_REG_PA_LRADC                    (0x80050000)
#define CSP_BASE_REG_PA_SPDIF                    (0x80054000)
#define CSP_BASE_REG_PA_I2C                      (0x80058000)
#define CSP_BASE_REG_PA_RTC                      (0x8005C000)
#define CSP_BASE_REG_PA_PWM                      (0x80064000)
#define CSP_BASE_REG_PA_TIMROT                   (0x80068000)
#define CSP_BASE_REG_PA_UARTAPP0                 (0x8006C000)
#define CSP_BASE_REG_PA_UARTAPP1                 (0x8006E000)
#define CSP_BASE_REG_PA_UARTDBG                  (0x80070000)
#define CSP_BASE_REG_PA_DRI                      (0x80074000)
#define CSP_BASE_REG_PA_IR                       (0x80078000)
#define CSP_BASE_REG_PA_USBPHY                   (0x8007C000)
#define CSP_BASE_REG_PA_USB                      (0x80080000)
#define CSP_BASE_REG_PA_DRAM                     (0x800E0000)

#if __cplusplus
}
#endif

#endif

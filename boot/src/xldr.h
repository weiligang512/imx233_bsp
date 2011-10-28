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
//  Copyright (C) 2009 Freescale Semiconductor, Inc. All Rights Reserved.
//  THIS SOURCE CODE, AND ITS USE AND DISTRIBUTION, IS SUBJECT TO THE TERMS
//  AND CONDITIONS OF THE APPLICABLE LICENSE AGREEMENT
//
//------------------------------------------------------------------------------
//
//  Header: xldr.h
//
//  This header file defines the MX233 processor.
//
//  The MX233 is a System on Chip (SoC) part consisting of an ARM9 core.
//  This header file is comprised of component header files that define the
//  register layout of each component.
//
//-----------------------------------------------------------------------------
#ifndef __XLDR_H
#define __XLDR_H

#if __cplusplus
extern "C" {
#endif

#include "bsp_cfg.h"
#include "mx233_base_regs.h"
#include "soc_types.h"
#include "regsdigctl.h"
#include "regsemi.h"
#include "regsdram.h"
#include "regsclkctrl.h"
#include "regspower.h"
#include "regspinctrl.h"
#include "regsuartdbg.h"
#include "regsusbphy.h"
#include "regslradc.h"
#include "regsocotp.h"
#include "soc_macros.h"

#define IMAGE_WINCE_POWEROFF_IRAM_OFFSET 0x5000
#define REGS_BASE 0x80000000
#define pv_HWregDIGCTL (REGS_BASE + 0x0001C000)
#define pv_HWregCLKCTRL (REGS_BASE + 0x00040000)
#define pv_HWregPOWER (REGS_BASE + 0x00044000)
#define pv_HWregPINCTRL (REGS_BASE + 0x00018000)
#define pv_HWregDRAM (REGS_BASE + 0x000e0000)
#define pv_HWregEMI (REGS_BASE + 0x00020000)
#define pv_HWregUARTDbg (REGS_BASE + 0x00070000)
#define pv_HWregUSBPhy  (REGS_BASE + 0x0007C000)
#define pv_HWregLRADC  (REGS_BASE + 0x00050000)
#define pv_HWregOTP (REGS_BASE + 0x0002C000)

#define BATT_VOLTAGE_8_MV 8

#define ERROR_MASK                      (-268435456)
#define TRUE 1
#define FALSE 0

#define HW_GROUP                  (0x00100000) //0x80100000
#define DDI_GROUP                 (0x00200000) //0x80200000
#define OS_GROUP                  (0x00300000) //0x80300000
#define MIDDLEWARE_GROUP          (0x00400000) //0x80400000
#define ROM_GROUP                 (0x00500000) //0x80500000
#define BM_GROUP                  (0x00600000) //0x80600000
#define APPS_GROUP                (0x00700000) //0x80700000
#define UTILITY_GROUP             (0x00800000) //0x80800000
#define HW_CLOCKS_GROUP           (HW_GROUP|0x0001A000)

#define ERROR_HW_CLOCKS_GROUP           (ERROR_MASK|HW_CLOCKS_GROUP)
#define DDI_UART_DEBUG_GROUP      (DDI_GROUP|0x00000000)
#define DDI_LED_GROUP             (DDI_GROUP|0x00001000)
#define DDI_TIMER_GROUP           (DDI_GROUP|0x00002000)
#define DDI_PWM_OUTPUT_GROUP      (DDI_GROUP|0x00003000)
#define DDI_UARTAPP_GROUP         (DDI_GROUP|0x00004000)
#define DDI_ETM_GROUP             (DDI_GROUP|0x00005000)
#define DDI_SSP_GROUP             (DDI_GROUP|0x00006000)
#define DDI_I2C_GROUP             (DDI_GROUP|0x00007000)
#define DDI_LDL_GROUP             (DDI_GROUP|0x00008000)
#define DDI_USB_GROUP             (DDI_GROUP|0x0000A000)
#define DDI_LCDIF_GROUP           (DDI_GROUP|0x0000B000)
#define DDI_ADC_GROUP             (DDI_GROUP|0x0000D000)
#define DDI_RTC_GROUP             (DDI_GROUP|0x0000E000)
#define DDI_ALARM_GROUP           (DDI_GROUP|0x0000F000)
#define DDI_UART_GROUP            (DDI_GROUP|0x00010000)
#define DDI_FM_TUNER_GROUP        (DDI_GROUP|0x00011000)
#define DDI_LRADC_GROUP           (DDI_GROUP|0x00012000)
#define DDI_GPIO_GROUP            (DDI_GROUP|0x00013000)
#define DDI_DISPLAY_GROUP         (DDI_GROUP|0x00015000)
#define DDI_PSWITCH_GROUP         (DDI_GROUP|0x00016000)
#define DDI_BCM_GROUP             (DDI_GROUP|0x00017000)
#define DDI_DRI_GROUP             (DDI_GROUP|0x00018000)
#define DDI_CLOCKS_GROUP          (DDI_GROUP|0x00019000)
#define DDI_MEDIA_CACHE_GROUP     (DDI_GROUP|0x0001e000)
#define DDI_MEDIABUFMGR_GROUP     (DDI_GROUP|0x0001f000)
#define DDI_NAND_GROUP            (DDI_GROUP|0x00020000)
#define DDI_LBA_NAND_GROUP        (DDI_GROUP|0x00030000)
#define DDI_NAND_GPMI_GROUP       (DDI_GROUP|0x00031000)
#define DDI_MMC_GROUP             (DDI_GROUP|0x00021000)
#define DDI_DCP_GROUP             (DDI_GROUP|0x00022000)
#define DDI_POWER_GROUP           (DDI_GROUP|0x00023000)
#define DDI_EMI_GROUP             (DDI_GROUP|0x00024000)
#define DDI_RNG_GROUP             (DDI_GROUP|0x00025000)
#define DDI_AUDIOOUT_GROUP        (DDI_GROUP|0x00026000)
#define DDI_AUDIOIN_GROUP         (DDI_GROUP|0x00027000)

#define ERROR_DDI_UART_DEBUG_GROUP      (ERROR_MASK |DDI_UART_DEBUG_GROUP)
#define ERROR_DDI_UARTAPP_GROUP         (ERROR_MASK |DDI_UARTAPP_GROUP)
#define ERROR_DDI_LED_GROUP             (ERROR_MASK |DDI_LED_GROUP)
#define ERROR_DDI_TIMER_GROUP           (ERROR_MASK |DDI_TIMER_GROUP)
#define ERROR_DDI_PWM_OUTPUT_GROUP      (ERROR_MASK |DDI_PWM_OUTPUT_GROUP)
#define ERROR_DDI_ETM_GROUP             (ERROR_MASK |DDI_ETM_GROUP)
#define ERROR_DDI_SSP_GROUP             (ERROR_MASK |DDI_SSP_GROUP)
#define ERROR_DDI_I2C_GROUP             (ERROR_MASK |DDI_I2C_GROUP)
#define ERROR_DDI_LDL_GROUP             (ERROR_MASK |DDI_LDL_GROUP)
#define ERROR_DDI_USB_GROUP             (ERROR_MASK |DDI_USB_GROUP)
#define ERROR_DDI_LCDIF_GROUP           (ERROR_MASK |DDI_LCDIF_GROUP)
#define ERROR_DDI_ADC_GROUP             (ERROR_MASK |DDI_ADC_GROUP)
#define ERROR_DDI_RTC_GROUP             (ERROR_MASK |DDI_RTC_GROUP)
#define ERROR_DDI_ALARM_GROUP           (ERROR_MASK |DDI_ALARM_GROUP)
#define ERROR_DDI_FM_TUNER_GROUP        (ERROR_MASK |DDI_FM_TUNER_GROUP)
#define ERROR_DDI_LRADC_GROUP           (ERROR_MASK |DDI_LRADC_GROUP)
#define ERROR_DDI_GPIO_GROUP            (ERROR_MASK |DDI_GPIO_GROUP)
#define ERROR_DDI_DISPLAY_GROUP         (ERROR_MASK |DDI_DISPLAY_GROUP)
#define ERROR_DDI_PSWITCH_GROUP         (ERROR_MASK |DDI_PSWITCH_GROUP)
#define ERROR_DDI_BCM_GROUP             (ERROR_MASK |DDI_BCM_GROUP)
#define ERROR_DDI_DRI_GROUP             (ERROR_MASK |DDI_DRI_GROUP)
#define ERROR_DDI_CLOCKS_GROUP          (ERROR_MASK |DDI_CLOCKS_GROUP)
#define ERROR_DDI_MEDIA_CACHE_GROUP     (ERROR_MASK |DDI_MEDIA_CACHE_GROUP)
#define ERROR_DDI_MEDIABUFMGR_GROUP     (ERROR_MASK |DDI_MEDIABUFMGR_GROUP)
#define ERROR_DDI_NAND_GROUP            (ERROR_MASK |DDI_NAND_GROUP)
#define ERROR_DDI_LBA_NAND_GROUP        (ERROR_MASK |DDI_LBA_NAND_GROUP)
#define ERROR_DDI_NAND_GPMI_GROUP       (ERROR_MASK |DDI_NAND_GPMI_GROUP)
#define ERROR_DDI_MMC_GROUP             (ERROR_MASK |DDI_MMC_GROUP)
#define ERROR_DDI_DCP_GROUP             (ERROR_MASK |DDI_DCP_GROUP)
#define ERROR_DDI_POWER_GROUP           (ERROR_MASK |DDI_POWER_GROUP)
#define ERROR_DDI_EMI_GROUP             (ERROR_MASK |DDI_EMI_GROUP)
#define ERROR_DDI_RNG_GROUP             (ERROR_MASK |DDI_RNG_GROUP)
#define ERROR_DDI_UART_GROUP            (ERROR_MASK |DDI_UART_GROUP)
#define HW_PLL_GROUP              (HW_GROUP|0x00006000)

#define ERROR_HW_PLL_GROUP              (ERROR_MASK|HW_PLL_GROUP)

#define ERROR_DDI_EMI_SDRAM_NOT_SUPPORTED                        (ERROR_DDI_EMI_GROUP)
#define ERROR_DDI_EMI_ENTER_SELF_REFRESH_TIMEOUT                 (ERROR_DDI_EMI_GROUP + 1)
#define ERROR_DDI_EMI_EXIT_SELF_REFRESH_TIMEOUT                  (ERROR_DDI_EMI_GROUP + 2)
#define ERROR_DDI_EMI_RESYNC_TIMEOUT                             (ERROR_DDI_EMI_GROUP + 3)
#define ERROR_DDI_EMI_NO_CHIP_SELECT_WAS_SELECTED                (ERROR_DDI_EMI_GROUP + 4)
#define ERROR_DDI_EMI_ONLY_2_CHIP_SELECTS_SUPPORTED              (ERROR_DDI_EMI_GROUP + 5)
#define ERROR_DDI_EMI_64MB_PER_CHIP_SELCT_MAXIMUM                (ERROR_DDI_EMI_GROUP + 6)
#define ERROR_DDI_EMI_UNEXPECTED_OBJECT_LOCATION                 (ERROR_DDI_EMI_GROUP + 7)
#define ERROR_DDI_EMI_UNEXPECTED_OBJECT_SIZE                     (ERROR_DDI_EMI_GROUP + 8)
#define ERROR_DDI_EMI_SDRAM_TYPE_NOT_SUPPORTED                   (ERROR_DDI_EMI_GROUP + 9)
#define ERROR_HW_PLL_GENERAL                                     (ERROR_HW_PLL_GROUP)

#define ERROR_HW_CLOCKS_GENERAL                                  (ERROR_HW_CLOCKS_GROUP)
#define ERROR_HW_CLOCKS_SET_PCLK_TIMEOUT                         (ERROR_HW_CLOCKS_GROUP + 1)
#define ERROR_HW_CLOCKS_SET_HCLK_TIMEOUT                         (ERROR_HW_CLOCKS_GROUP + 2)
#define ERROR_HW_CLOCKS_SET_XCLK_TIMEOUT                         (ERROR_HW_CLOCKS_GROUP + 3)
#define ERROR_HW_CLOCKS_SET_PLL_FREQ_TIMEOUT                     (ERROR_HW_CLOCKS_GROUP + 4)
#define ERROR_HW_CLOCKS_INVALID_AUTOSLOW_DIV                     (ERROR_HW_CLOCKS_GROUP + 5)


#define ERROR_HW_CLKCTRL_INVALID_LFR_VALUE                      (ERROR_HW_CLOCKS_GROUP)
#define ERROR_HW_CLKCTRL_INVALID_CP_VALUE                       (ERROR_HW_CLOCKS_GROUP + 0x01)
#define ERROR_HW_CLKCTRL_INVALID_DIV_VALUE                      (ERROR_HW_CLOCKS_GROUP + 0x02)
#define ERROR_HW_CLKCTRL_DIV_BY_ZERO                            (ERROR_HW_CLOCKS_GROUP + 0x03)
#define ERROR_HW_CLKCTRL_CLK_DIV_BUSY                           (ERROR_HW_CLOCKS_GROUP + 0x04)
#define ERROR_HW_CLKCTRL_CLK_GATED                              (ERROR_HW_CLOCKS_GROUP + 0x05)
#define ERROR_HW_CLKCTRL_REF_CLK_GATED                          (ERROR_HW_CLOCKS_GROUP + 0x06)
#define ERROR_HW_CLKCTRL_REF_CPU_GATED                          (ERROR_HW_CLOCKS_GROUP + 0x07)
#define ERROR_HW_CLKCTRL_REF_EMI_GATED                          (ERROR_HW_CLOCKS_GROUP + 0x08)
#define ERROR_HW_CLKCTRL_REF_IO_GATED                           (ERROR_HW_CLOCKS_GROUP + 0x09)
#define ERROR_HW_CLKCTRL_REF_PIX_GATED                          (ERROR_HW_CLOCKS_GROUP + 0x0A)
#define ERROR_HW_CLKCTRL_INVALID_GATE_VALUE                     (ERROR_HW_CLOCKS_GROUP + 0x0B)
#define ERROR_HW_CLKCTRL_INVALID_PARAM                          (ERROR_HW_CLOCKS_GROUP + 0x0C)
#define ERROR_HW_CLKCTRL_UNSUPPORTED_AUTOSLOW_COMPONENT         (ERROR_HW_CLOCKS_GROUP + 0x0D)


#define SUCCESS 0
#define DDI_EMI_MAX_NUM_CHIP_SELECTS 2


// Valid stmp37xx values are 4, 8, and 12mA
// \todo The following 5 defines should be configured by the application, not the driver.
#define EMI_PIN_DRIVE_ADDRESS       PIN_DRIVE_12mA
#define EMI_PIN_DRIVE_DATA          PIN_DRIVE_12mA
#define EMI_PIN_DRIVE_CHIP_ENABLE   PIN_DRIVE_12mA
#define EMI_PIN_DRIVE_CLOCK         PIN_DRIVE_12mA
#define EMI_PIN_DRIVE_CONTROL       PIN_DRIVE_12mA

#define EXTENDED_MODE_REGISTER_VALUE 0x20

#define BUILD_DBG_SERIAL_BAUDRATE_DIVIDER(b)    (((UART_CLOCK_FREQUENCY * 4) + ((b) / 2)) / (b))

#define GET_UARTDBG_BAUD_DIVINT(b)                              ((BUILD_DBG_SERIAL_BAUDRATE_DIVIDER(b)) >> 6)
#define GET_UARTDBG_BAUD_DIVFRAC(b)                             ((BUILD_DBG_SERIAL_BAUDRATE_DIVIDER(b)) >> 0)

// Defines the frequency value in MHz for which no ddc_resync is used.
// This is to assure we don't run out of hardware delay elements for lower frequencies.
#define DDI_EMI_DDR_MAX_NO_DCCRESYNC_FREQ_MHZ 60

typedef int     bool;
typedef unsigned int DWORD;

typedef enum _hw_clkctrl_bypass_clk_t
{
    //! \brief Bit position to bypass ref_cpu and use the crystal for the CPU clock.
    BYPASS_CPU  = 0x80,
    //! \brief Bit position to bypass ref_emi and use the crystal for the EMI clock.
    BYPASS_EMI  = 0x40,
    //! \brief Bit position to bypass ref_io and use the crystal for the SSP clock.
    BYPASS_SSP  = 0x20,
    //! \brief Bit position to bypass ref_io and use the crystal for the GPMI clock.
    BYPASS_GPMI = 0x10,
    //! \brief Bit position to bypass ref_io and use the crystal for the IR clock.
    BYPASS_IR   = 0x08,
    //! \brief Bit position to bypass ref_pix and use the crystal for the display clock.
    BYPASS_PIX  = 0x02,
    //! \brief Bit position to bypass ref_pll and use the crystal for the SAIF clock.
    BYPASS_SAIF = 0x01,
} hw_clkctrl_bypass_clk_t;

typedef enum
{
    EMI_CLK_OFF = 0,
    EMI_CLK_6MHz = 6,
    EMI_CLK_24MHz = 24,
    EMI_CLK_48MHz = 48,
    EMI_CLK_60MHz = 60,
    EMI_CLK_96MHz = 96,
    EMI_CLK_120MHz = 120,
    EMI_CLK_133MHz = 133,
    EMI_CLK_151MHz = 151,
    EMI_CLK_160MHz = 160
} hw_emi_ClockState_t;

typedef enum _hw_emi_MemType_t
{
    //! \brief TBD
    EMI_DEV_SDRAM,
    //! \brief TBD
    EMI_DEV_MOBILE_SDRAM,
    //! \brief TBD
    EMI_DEV_MOBILE_DDR,
    //! \brief TBD
    EMI_DEV_NOR,
    //! \brief TBD
    EMI_DEV_DDR1    
    
} hw_emi_MemType_t;

typedef enum
{
    PIN_VOLTAGE_1pt8V = 0,
    PIN_VOLTAGE_3pt3V = 1,
} TPinVoltage;

typedef enum
{
    PIN_DRIVE_4mA  = 0,
    PIN_DRIVE_8mA  = 1,
    PIN_DRIVE_12mA = 2,
    PIN_DRIVE_16mA = 3
} TPinDrive;

typedef struct hw_emi_ClockDependentRegs
{
    unsigned int DramReg4;
    unsigned int DramReg7;
    unsigned int DramReg12;
    unsigned int DramReg13;
    unsigned int DramReg15;
    unsigned int DramReg17;
    unsigned int DramReg18;
    unsigned int DramReg19;
    unsigned int DramReg20;
    unsigned int DramReg21;
    unsigned int DramReg26;
    unsigned int DramReg32;
    unsigned int DramReg33;
    unsigned int DramReg34;
    unsigned int DramReg40;
}hw_emi_ClockDependentRegs_t;
//! \brief TBD
typedef enum
{
    //! \brief TBD
    CE0 = 1,
    CE1 = 2,
    CE2 = 4,
    CE3 = 8

} hw_emi_ChipSelectMask_t;

typedef enum
{
    EMI_2MB_DRAM = 2,
    EMI_4MB_DRAM = 4,
    EMI_8MB_DRAM = 8,
    EMI_16MB_DRAM = 16,
    EMI_32MB_DRAM = 32,
    EMI_64MB_DRAM = 64,
    EMI_128MB_DRAM = 128,
    EMI_256MB_DRAM = 256,
    EMI_512MB_DRAM = 512,

} hw_emi_TotalDramSize_t;

typedef enum _hw_power_VbusValidThresh_t
{
    //! \brief 4.40V threshold on insertion, 4.21V threshold on removal
    VBUS_VALID_THRESH_4400_4210 = 0,
    //! \brief 4.17V threshold on insertion, 4.00V threshold on removal
    VBUS_VALID_THRESH_4170_4000 = 1,
    //! \brief 2.50V threshold on insertion, 2.45V threshold on removal
    VBUS_VALID_THRESH_2500_2450 = 2,
    //! \brief 4.73V threshold on insertion, 4.48V threshold on removal
    VBUS_VALID_THRESH_4730_4480 = 3,
    //! \brief Maximum threshold value for the register setting
    VBUS_VALID_THRESH_MAX       = 3,

    //! \brief Use under normal operating conditions.
    VBUS_VALID_THRESH_NORMAL = VBUS_VALID_THRESH_4400_4210,
    //! \brief Use when a lower than normal threshold is needed.
    VBUS_VALID_THRESH_LOW    = VBUS_VALID_THRESH_4170_4000,
    //! \brief Use when a higher than normal threshold is needed.
    VBUS_VALID_THRESH_HIGH   = VBUS_VALID_THRESH_4730_4480,
    //! \brief Use only for testing, or under guidance.
    VBUS_VALID_THRESH_TEST   = VBUS_VALID_THRESH_2500_2450
} hw_power_VbusValidThresh_t;

typedef enum _hw_power_5vDetection_t
{
    //! \brief Use VBUSVALID comparator for detection
    HW_POWER_5V_VBUSVALID,
    //! \brief Use VDD5V_GT_VDDIO comparison for detection
    HW_POWER_5V_VDD5V_GT_VDDIO,
    //! \brief Uses VBUSVALID 5V detection, but does not rely on the interrupt.
    HW_POWER_5V_VBUSVALID_WITH_POLLING
} hw_power_5vDetection_t;

//#pragma ghs section bss=".ocram.bss"   /// MUST BE IN OCRAM !!!!
typedef struct ddi_emi_vars_tag
{
    hw_emi_ClockState_t EmiClkSpeedState;
    hw_emi_MemType_t MemType;
    hw_emi_ClockDependentRegs_t ClockDependentRegs;
    bool bAutoMemorySelfRefreshModeEnabled;
    bool bAutoMemoryClockGateModeEnabled;
    bool bStaticMemorySelfRefreshModeEnabled;
    bool bStaticMemoryClockGateModeEnabled;
    bool bLowPfdAllowed;
}ddi_emi_vars_t;

typedef enum _ddi_power_5vDetection_t
{
    //! \brief VBUSVALID will be used for 5V/USB detection.
    DDI_POWER_VBUSVALID,
    //! \brief VDD5V_GT_VDDIO will be used for 5V/USB detection.
    DDI_POWER_VDD5V_GT_VDDIO,
    //! \brief Uses VBUSVALID for 5V detection, but does not rely on the interrupt.
    DDI_POWER_VBUSVALID_WITH_POLLING    
} ddi_power_5vDetection_t;

typedef struct _ddi_power_InitValues_t
{
    unsigned int            u32BatterySamplingInterval;
    ddi_power_5vDetection_t e5vDetection;
    
    // Not used by 37xx,377x
    bool                    bEnable4p2;
} ddi_power_InitValues_t;

#if __cplusplus
}
#endif

#endif //__XLDR_H

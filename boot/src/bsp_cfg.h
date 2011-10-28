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
//-----------------------------------------------------------------------------
//
// Copyright (C) 2007-2009, Freescale Semiconductor, Inc. All Rights Reserved.
// THIS SOURCE CODE, AND ITS USE AND DISTRIBUTION, IS SUBJECT TO THE TERMS
// AND CONDITIONS OF THE APPLICABLE LICENSE AGREEMENT
//
//-----------------------------------------------------------------------------
//
// File: bsp_cfg.h
//
// This file contains system constant specific for imx233 board.
//
//------------------------------------------------------------------------------
#ifndef __BSP_CFG_H
#define __BSP_CFG_H

//------------------------------------------------------------------------------
//
// Define: BSP_DEVICE_PREFIX
//
// Prefix used to generate device name for bootload/KITL
//
#define BSP_DEVICE_PREFIX       "MX233"                  // Device name prefix

//------------------------------------------------------------------------------
// CPU Configuration Settings
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Clock Configuration Settings
//------------------------------------------------------------------------------
// InitOSTickTimer

#define RESCHED_PERIOD          1                                    // Reschedule ms
#define BSP_TIMER0_CLKSRC       (32000)                              // Timer0 clock source TIMER_SELECT_32KHZ_XTAL
#define BSP_MICROSEC_CLKSRC     (1000000)
#define OEM_TICKS_PER_1MS       (BSP_MICROSEC_CLKSRC/1000)           // 1 ms in ticks
#define OEM_TICKS_MARGIN        ((BSP_TIMER0_CLKSRC/100000)+3)       // Tick Margin

//------------------------------------------------------------------------------
// Audio Configuration
//------------------------------------------------------------------------------

// Set BSP_AUDIO_DMA_BUF_ADDR to static DMA buffer physical address.  Comment
// out these two definitions to force the audio DMA buffers to be dynamically
// allocated by the audio driver from external RAM.
#define BSP_AUDIO_DMA_BUF_ADDR          IMAGE_WINCE_AUDIO_IRAM_PA_START
#define BSP_AUDIO_DMA_BUF_SIZE          IMAGE_WINCE_AUDIO_IRAM_SIZE

// Set BSP_AUDIO_DMA_DESCRIPTOR_ADDR to static DMA descriptor buffer physical address. 
// Comment out these two definitions to force the audio DMA descriptor buffers to be dynamically
// allocated by the audio driver from external RAM.
#define BSP_AUDIO_DMA_DESCRIPTOR_ADDR   IMAGE_WINCE_AUDIO_DESCRIPTOR_IRAM_PA_START
#define BSP_AUDIO_DMA_DESCRIPTOR_SIZE   IMAGE_WINCE_AUDIO_DESCRIPTOR_IRAM_SIZE

//------------------------------------------------------------------------------
// LCD Panel Configuration to support IOCTL_HAL_QUERY_DISPLAYSETTINGS
//------------------------------------------------------------------------------
// DirectDraw Display Driver Support for Screen Rotation
// Note: Windows CE documentation states that DirectDraw and screen rotation
// are incompatible.  Although modifications have been made to run DirectDraw
// with screen rotation, there may be unexpected failures in the GDI CETK suite.
//------------------------------------------------------------------------------
// Video Memory memory attributes
// The video memory region may be configured as cacheable, write-through (WT)
// or as non-cacheable, bufferable (NCB). iMX233 has no ARM errata (#399234) which
// affects ARM1136 core (r0p2,r1p0,r1p1,r1p2), so use WT mode to provides 
// performance benefit.
#define BSP_VID_MEM_CACHE_WRITETHROUGH  TRUE
//------------------------------------------------------------------------------
// ADC Tearing Prevention
// When using a smart display, which requires the IPU ADC driver, tearing
// can be enabled (TRUE) or disabled (FALSE).  If tearing is enabled, each ADC
// update to the display will be synchronized to the panel VSYNC.
//------------------------------------------------------------------------------
// Frame Dropping
// Video frames can be dropped discreetly (i.e., without returning a notice to
// the calling application) by setting BSP_DROP_FRAMES_QUIETLY to TRUE.
// Windows Media Player does not respond well to Flip() calls that result in
// a dropped frame.  Setting this variable to TRUE will allow WMP to play
// smoother video.
//------------------------------------------------------------------------------
// TV OUT Configurations:
//
// For TV Out, please set BSP_DIRECTDRAW_SUPPORT_ROTATION = TRUE
//
// To support YCrCb as an input of FS45X TV Encoder, please set TV_YCRCB_INPUT = TRUE
//
// To support RGB an an input of FS45X TV Encoder, please set TV_YCRCB_INPUT = FALSE
//
// To support TV Out D1 resolution, set TV_D1 = TRUE, otherwise set TV D1 = FALSE
// for 640x480
//
// To support TV Out NTSC, set TV_NTSC = TRUE, otherwise set TV_NTSC = FALSE for PAL
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//
// Define: BSP_BASE_REG_PA_SERIALKITL
//
// Specifies physical address of serial port used for serial KITL transport.
//

//------------------------------------------------------------------------------
#define SYSINTR_USBOTG                  (SYSINTR_FIRMWARE+2)

//------------------------------------------------------------------------------
// Defines the Maximum Baudrate and reference frequency for UART
//------------------------------------------------------------------------------

#define UART_CLOCK_FREQUENCY                    (24000000)
#define DEBUG_BAUD                              (115200)

#define BUILD_DBG_SERIAL_BAUDRATE_DIVIDER(b)    (((UART_CLOCK_FREQUENCY * 4) + ((b) / 2)) / (b))

#define GET_UARTDBG_BAUD_DIVINT(b)                              ((BUILD_DBG_SERIAL_BAUDRATE_DIVIDER(b)) >> 6)
#define GET_UARTDBG_BAUD_DIVFRAC(b)                             ((BUILD_DBG_SERIAL_BAUDRATE_DIVIDER(b)) >> 0)

//Define DVFC software Interrupter
#define IRQ_DVFC         IRQ_RESERVED_95

// Board ID related defines
#define OCOTP_CUSTCAP_BOARDID_MASK      0x30000000
#define OCOTP_CUSTCAP_BOARDID_SHIFT     28
#define BOARDID_DEVBOARD             0x0
#define BOARDID_EVKBOARD_A           0x1
#define BOARDID_EVKBOARD_B           0x2

#endif

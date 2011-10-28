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

#include "xldr.h"

//-----------------------------------------------------------------------------
// Global Variables

void * volatile const pTOC = (void *)-1;
static hw_power_5vDetection_t DetectionMethod;
static ddi_emi_vars_t s_ddi_emi_vars;
bool bKeepPllPowered = 0;
static bool s_bSyncModeEnabled=FALSE;
static bool s_b5V=FALSE;
static bool s_bUsb=FALSE;
static bool s_bBattery=FALSE;

//------------------------------------------------------------------------------
// Local Functions

void hw_emi_SetMemoryClockGateAutoFlag(bool bEnable);
void hw_emi_EnterMemoryClockGateMode(bool bOnOff);
void hw_emi_EnterMemorySelfRefreshMode(bool bOnOff);
void hw_emi_SetMemorySelfRefeshAutoFlag(bool bEnable);
void hw_clkctrl_PowerPll(bool bPowerOn);
bool hw_digctl_CheckTimeOut(unsigned int StartTime, unsigned int TimeOut);
int ddi_emi_PreOsInit(hw_emi_MemType_t MemType,hw_emi_ChipSelectMask_t ChipSelectMask,hw_emi_TotalDramSize_t TotalDramSize);
void hw_emi_ConfigureEmiPins(TPinVoltage pin_voltage,TPinDrive pin_drive_addr,TPinDrive pin_drive_data,TPinDrive pin_drive_ce,TPinDrive pin_drive_clk,TPinDrive pin_drive_ctrl);
void hw_emi_DisableEmiPadKeepers(void);
void hw_dram_Init_ddr_mt46v32m16_6t_96MHz(void);
void hw_dram_Init_ddr_mt46v32m16_6t_133MHz_optimized(void);
void hw_dram_Init_mobile_ddr_mt46h32m16lf_5_regs_3_133MHz(void);
void hw_dram_Init_mobile_ddr_mt46h32m16lf_5_regs_3_160MHz(void);
void hw_clkctrl_SetPfdRefEmiGate(bool bClkGate);
void ddi_emi_PrepareForDramSelfRefreshMode(void);
void ddi_emi_ExitAutoMemoryClockGateMode(void);
bool hw_core_EnableIrqInterrupt(bool bNewState);
int ddi_emi_EnterStaticSelfRefreshMode(void);
int hw_clkctrl_SetEmiClkRefEmiDiv(unsigned int u32Div);
unsigned int hw_digctl_GetCurrentTime(void);
int ddi_emi_ExitStaticSelfRefreshMode(void);
int hw_clkctrl_SetPfdRefEmiDiv(unsigned int u32Div);
void OEMWriteDebugByte(unsigned char ch);
void InitDebugSerial();
void PrintHex(unsigned int value);
void PrintBatteryVoltage(unsigned int value);
void InitPower();
void PowerExecute5VoltsToBatteryHandoff();
void PowerStop4p2(void);
void PowerDisableAutoHardwarePowerdown(bool bDisable);
void PowerSetCharger(DWORD current);
void PowerStopCharger();
void OALStall(unsigned int microSec);
void TurnOnPLLClock();



//------------------------------------------------------------------------------
//
//  Function: hw_emi_IsControllerHalted
//
//
//------------------------------------------------------------------------------

bool hw_emi_IsControllerHalted(void)
{
    return HW_EMI_STAT.B.DRAM_HALTED;
}

//------------------------------------------------------------------------------
//
//  Function: hw_core_drain_write_buffer
//
//
//------------------------------------------------------------------------------

void hw_core_drain_write_buffer( )
{
}

//------------------------------------------------------------------------------
//
//  Function: hw_emi_IsDramSupported
//
//
//------------------------------------------------------------------------------

bool hw_emi_IsDramSupported(void)
{
    if (HW_EMI_STAT.B.DRAM_PRESENT)
        return TRUE;
    else
        return FALSE;
}

//------------------------------------------------------------------------------
//
//  Function: hw_emi_ClearReset
//
//
//------------------------------------------------------------------------------

void hw_emi_ClearReset(void)
{
    HW_EMI_CTRL_CLR(BM_EMI_CTRL_SFTRST);
    HW_EMI_CTRL_CLR(BM_EMI_CTRL_CLKGATE);
}

/////////////////////////////////////////////////////////////////////////////////
//! \brief Set EmiClk divider for the ref_xtal source
//!
//! \fntype Function
//!
//! This field controls the divider connected to the crystal reference clock,
//! ref_xtal, that drives the CLK_EMI domain when bypass IS selected.
//!
//! \param[in] u32Div - Divider in range 1 to 15
//!
//! \retval ERROR_HW_CLKCTRL_DIV_BY_ZERO - should not divide by zero
//! \retval ERROR_HW_CLKCTRL_CLK_DIV_BUSY - EmiClk is still transitioning
//! \retval ERROR_HW_CLKCTRL_CLK_GATED - clock must be ungated to change divider
//! \retval SUCCESS - EmiClk ref_xtal divider set
//!
//! \notes HW_CLKCTRL_EMI.B.DIV_XTAL is used to avoid a clear/set
//! \notes which momentarily divides by zero.
/////////////////////////////////////////////////////////////////////////////////
int hw_clkctrl_SetEmiClkRefXtalDiv(unsigned int u32Div)
{
    ////////////////////////////////////
    // Error checks
    ////////////////////////////////////

    // Return if busy with another divider change
    if(HW_CLKCTRL_EMI.B.BUSY_REF_XTAL)
        return ERROR_HW_CLKCTRL_CLK_DIV_BUSY;

    // Only change DIV_FRAC_EN and DIV when CLKGATE = 0
    if(HW_CLKCTRL_EMI.B.CLKGATE)
        return ERROR_HW_CLKCTRL_CLK_GATED;


    ////////////////////////////////////
    // Change gate and/or divider
    ////////////////////////////////////

    // Set divider for DIV_XTAL
    HW_CLKCTRL_EMI.B.DIV_XTAL = u32Div;

    return SUCCESS;

}

/////////////////////////////////////////////////////////////////////////////////
// EmiClk
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//! \brief Gate/ungate EmiClk  for the ref_xtal source
//!
//! \fntype Function
//!
//! CLK_EMI crystal divider Gate. If set to 1, the EMI_CLK divider that is
//! sourced by the crystal reference clock, ref_xtal, is gated off. If set to
//! 0, CLK_EMI crystal divider is not gated
//!
//! \param[in] bClkGate - TRUE to gate, FALSE to ungate
//!
//! \notes Only gates the ref_xtal reference clock to the XTAL divider. ref_emi
//! \notes has no gate but can be muxed.
/////////////////////////////////////////////////////////////////////////////////
void hw_clkctrl_SetEmiRefXtalClkGate(bool bClkGate)
{
    // Gate or ungate the EMI clock
    if(bClkGate)
        HW_CLKCTRL_EMI_SET(BM_CLKCTRL_EMI_CLKGATE);
    else
        HW_CLKCTRL_EMI_CLR(BM_CLKCTRL_EMI_CLKGATE);

    // Wait for the clock to settle.
    while(HW_CLKCTRL_EMI.B.BUSY_REF_XTAL);
}

/////////////////////////////////////////////////////////////////////////////////
//! \brief controls the selection of clock sources for various clock dividers.
//!
//! \fntype Function
//!
//! The PClk, EmiClk, SspClk, GpmiClk, IrClk, PixClk, and SaifClk can choose their
//! clock source from the PLL output, or bypass the PLL and use the 24MHz clock
//! signal instead.
//!
//! \param[in] - eClk - clock to bypass.  Valid inputs are: BYPASS_CPU, BYPASS_EMI,
//!              BYPASS_SSP, BYPASS_GPMI, BYPASS_IR, BYPASS_PIX, BYPASS_SAIF
//! \param[in] - bBypass - TRUE to use ref_xtal clock, FALSE to use PLL
//!
//! \retval - ERROR_HW_CLKCTRL_REF_CPU_GATED - ref_cpu is gated at PFD
//! \retval - ERROR_HW_CLKCTRL_REF_EMI_GATED - ref_emi is gated at PFD
//! \retval - ERROR_HW_CLKCTRL_REF_IO_GATED - ref_io is gated at PFD
//! \retval - ERROR_HW_CLKCTRL_REF_PIX_GATED - ref_pix is gated at PFD
//! \retval - ERROR_HW_CLKCTRL_INVALID_GATE_VALUE - gate cannot be set
//! \retval - SUCCESS
/////////////////////////////////////////////////////////////////////////////////
int hw_clkctrl_SetClkBypass(hw_clkctrl_bypass_clk_t eClk, bool bBypass)
{

    ////////////////////////////////
    // Check upstream clock gates
    ////////////////////////////////
    switch(eClk)
    {
        case BYPASS_CPU:
            if(bBypass)
            {
                // ref_xtal is ok.
            }
            else
            {
                // Don't change if ref_cpu is gated
                if(HW_CLKCTRL_FRAC.B.CLKGATECPU)
                    return ERROR_HW_CLKCTRL_REF_CPU_GATED;
            }
        break;

        case BYPASS_EMI:
            if(bBypass)
            {
                // Don't change if ref_xtal is gated
                if(HW_CLKCTRL_EMI.B.CLKGATE)
                    return ERROR_HW_CLKCTRL_REF_CLK_GATED;
            }
            else
            {
                // Don't change if ref_emi is gated
                if(HW_CLKCTRL_FRAC.B.CLKGATEEMI)
                    return ERROR_HW_CLKCTRL_REF_EMI_GATED;
            }
        break;

        case BYPASS_SSP:
        case BYPASS_GPMI:
        case BYPASS_IR:
            if(bBypass)
            {
                // ref_xtal is ok
            }
            else
            {
                // Don't change if ref_io is gated
                if(HW_CLKCTRL_FRAC.B.CLKGATEIO)
                    return ERROR_HW_CLKCTRL_REF_IO_GATED;
            }
        break;

        case BYPASS_PIX:
            if(bBypass)
            {
                // ref_xtal is ok
            }
            else
            {
                // Don't change if ref_pix is gated
                if(HW_CLKCTRL_FRAC.B.CLKGATEPIX)
                    return ERROR_HW_CLKCTRL_REF_PIX_GATED;
            }
        break;

        case BYPASS_SAIF:
            if(bBypass)
            {
                // Should never be set to 1
                return ERROR_HW_CLKCTRL_INVALID_GATE_VALUE;
            }
            else
            {
                // Should always be 0

            }
        break;

        default:
            return ERROR_HW_CLKCTRL_INVALID_PARAM;

    }  // end switch(eClk)


    ////////////////////////////////
    // Change the clock
    ////////////////////////////////
    if(bBypass)
        HW_CLKCTRL_CLKSEQ_SET(eClk);
    else
        HW_CLKCTRL_CLKSEQ_CLR(eClk);

    return SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////
//
//! \brief Initialize clock speed vars with initialized register values
//!
//! \fntype Function
//!
//! Initialize clock speed vars with initialized register values.  This must
//! be called after initialized the registers.
//!
//! \param[in] MemType          Type of memory
//! \param[in] ChipSelectMask   Map/Mask of CE pins used
//! \param[in] TotalDramSize    Size in bytes of entire SDRAM memory
//!
//! \return RtStatus, SUCCESS or error code
//
/////////////////////////////////////////////////////////////////////////////////
static void ddi_emi_InitClockSpeedDependentVars(void)
{
    s_ddi_emi_vars.ClockDependentRegs.DramReg4  = HW_DRAM_CTL04.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg12 = HW_DRAM_CTL12.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg13 = HW_DRAM_CTL13.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg15 = HW_DRAM_CTL15.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg17 = HW_DRAM_CTL17.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg18 = HW_DRAM_CTL18.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg19 = HW_DRAM_CTL19.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg20 = HW_DRAM_CTL20.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg21 = HW_DRAM_CTL21.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg26 = HW_DRAM_CTL26.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg32 = HW_DRAM_CTL32.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg33 = HW_DRAM_CTL33.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg34 = HW_DRAM_CTL34.U;

    s_ddi_emi_vars.ClockDependentRegs.DramReg7  = HW_DRAM_CTL07.U;
    s_ddi_emi_vars.ClockDependentRegs.DramReg40 = HW_DRAM_CTL40.U;
}

/////////////////////////////////////////////////////////////////////////////////
//
//! \brief Enter the automatic memory clock gate mode (based on activity)
//!
//! \fntype Function
//!
//! \return none
//
/////////////////////////////////////////////////////////////////////////////////
void ddi_emi_EnterAutoMemoryClockGateMode(void)
{
    hw_emi_SetMemoryClockGateAutoFlag(TRUE);
    hw_emi_EnterMemoryClockGateMode(TRUE);
    s_ddi_emi_vars.bAutoMemoryClockGateModeEnabled = TRUE; 
}

//------------------------------------------------------------------------------
//
//  Function: hw_emi_EnterMemoryClockGateMode
//
//
//------------------------------------------------------------------------------

void hw_emi_EnterMemoryClockGateMode(bool bOnOff)
{
    if(bOnOff)
    {
        // enter memory clock gate mode
        HW_DRAM_CTL16_SET(1<<19);
        
    }
    else
    {
        // exit memory clock gate mode
        HW_DRAM_CTL16_CLR(1<<19);
    }
}

//------------------------------------------------------------------------------
//
//  Function: hw_emi_SetMemoryClockGateAutoFlag
//
//
//------------------------------------------------------------------------------

void hw_emi_SetMemoryClockGateAutoFlag(bool bEnable)
{
    if(bEnable)
    {
        // a count valid of zero will result in instant power down
        if(HW_DRAM_CTL30.B.LOWPOWER_POWER_DOWN_CNT==0)
        {
            HW_DRAM_CTL30.B.LOWPOWER_POWER_DOWN_CNT = 32;
        }   
        
        HW_DRAM_CTL16_SET(1<<11);
    }
    else
        HW_DRAM_CTL16_CLR(1<<11);
}

/////////////////////////////////////////////////////////////////////////////////
//
//! \brief \brief Exit the automatic memory/controller self-refresh mode
//!
//! \fntype Function
//!
//! \return none
//
/////////////////////////////////////////////////////////////////////////////////
void ddi_emi_ExitAutoMemorySelfRefreshMode(void)
{
    hw_emi_EnterMemorySelfRefreshMode(FALSE);
    hw_emi_SetMemorySelfRefeshAutoFlag(FALSE);
    s_ddi_emi_vars.bAutoMemorySelfRefreshModeEnabled = FALSE;
}

/////////////////////////////////////////////////////////////////////////////////
//
//! \brief Makes CPU memory accesses higher priority in EMI controller queue
//!
//! \fntype Function
//!
//! \return none
//
/////////////////////////////////////////////////////////////////////////////////
void ddi_emi_StartHighCpuPriority(void)
{
    HW_EMI_CTRL.B.PORT_PRIORITY_ORDER = 8; //PORT1230
    HW_EMI_CTRL.B.ARB_MODE = 2;    
    
    HW_DRAM_CTL01.B.AHB1_R_PRIORITY = 0;
    HW_DRAM_CTL02.B.AHB2_R_PRIORITY = 0;
    HW_DRAM_CTL02.B.AHB2_W_PRIORITY = 0;
    
    HW_DRAM_CTL06.B.PLACEMENT_EN = 1;
    HW_DRAM_CTL07.B.PRIORITY_EN = 1;
    
}

/////////////////////////////////////////////////////////////////////////////////
//! \brief Sets flag to keep PLL power on.
//!
//! \fntype Function
//!
//! \retval EmiClk frequency in kHz
/////////////////////////////////////////////////////////////////////////////////
void ddi_clocks_KeepPllPowered(bool bPowered)
{
    //--------------------------------------------------------------------------
    // The application can choose to keep the PLL on even if current clock
    // frequencies do not require it to generate their clocks.  
    //--------------------------------------------------------------------------
    bKeepPllPowered = bPowered;

    //--------------------------------------------------------------------------
    // Turn on the PLL if it was requested on.  It the request is to turn off
    // the PLL, the clock driver must turn it off because other clocks may
    // be depending on it now.  
    //--------------------------------------------------------------------------        
    if(bPowered)
    {
        hw_clkctrl_PowerPll(TRUE);    
    }        
}

//------------------------------------------------------------------------------
//
//  Function: hw_digctl_MicrosecondWait
//
//
//------------------------------------------------------------------------------

void hw_digctl_MicrosecondWait(unsigned int u32Microseconds)
{
    unsigned int Start;

    //--------------------------------------------------------------------------
    // Get the start time.
    //--------------------------------------------------------------------------
    Start = HW_DIGCTL_MICROSECONDS_RD();

    //--------------------------------------------------------------------------
    // Loop until the timeout has elapsed.
    //--------------------------------------------------------------------------
    while(!hw_digctl_CheckTimeOut(Start,u32Microseconds));

}

////////////////////////////////////////////////////////////////////////////////
//! \brief Enable/Disable VDDMEM regulation
//!
//! Enables or disables the internal VDDMEM regulator
//!
//! \param[in] bEnable TRUE to enable the power source, FALSE to disable it. 
//!
////////////////////////////////////////////////////////////////////////////////
void hw_power_Enable2p5(bool bEnable)
{
    if(bEnable)
    {
        // clear pulldown as it isn't needed for internal linear regulator
        HW_POWER_VDDMEMCTRL_CLR(BM_POWER_VDDMEMCTRL_PULLDOWN_ACTIVE);
        HW_POWER_VDDMEMCTRL.B.TRG = 0x10; // ~2.5v
        
        HW_POWER_VDDMEMCTRL_SET(BM_POWER_VDDMEMCTRL_ENABLE_ILIMIT);
        HW_POWER_VDDMEMCTRL_SET(BM_POWER_VDDMEMCTRL_ENABLE_LINREG);
        // wait 10ms for 2p5 capacitor to fully charge
        hw_digctl_MicrosecondWait(10000);
        
        HW_POWER_VDDMEMCTRL_CLR(BM_POWER_VDDMEMCTRL_ENABLE_ILIMIT);
        
    }
    else
    {
        HW_POWER_VDDMEMCTRL_CLR(BM_POWER_VDDMEMCTRL_ENABLE_LINREG);
       
        // we could also enable the pull down here
        // but I'll leave that for a separate call if it 
        // becomes necessary on some applications.
    }
}

////////////////////////////////////////////////////////////////////////////////
//! \brief Enable/Disable VDDMEM regulation
//!
//! Enables or disables the internal VDDMEM regulator
//!
//! \param[in] bEnable TRUE to enable the power source, FALSE to disable it. 
//!
////////////////////////////////////////////////////////////////////////////////
void hw_power_Enable1p8(bool bEnable)
{
    if(bEnable)
    {
        // clear pulldown as it isn't needed for internal linear regulator
        HW_POWER_VDDMEMCTRL_CLR(BM_POWER_VDDMEMCTRL_PULLDOWN_ACTIVE);
        HW_POWER_VDDMEMCTRL.B.TRG = 0x2; // ~1.8v
        
        HW_POWER_VDDMEMCTRL_SET(BM_POWER_VDDMEMCTRL_ENABLE_ILIMIT);
        HW_POWER_VDDMEMCTRL_SET(BM_POWER_VDDMEMCTRL_ENABLE_LINREG);
        // wait 10ms for 1p8 capacitor to fully charge
        hw_digctl_MicrosecondWait(10000);
        
        HW_POWER_VDDMEMCTRL_CLR(BM_POWER_VDDMEMCTRL_ENABLE_ILIMIT);
        
    }
    else
    {
        HW_POWER_VDDMEMCTRL_CLR(BM_POWER_VDDMEMCTRL_ENABLE_LINREG);
       
        // we could also enable the pull down here
        // but I'll leave that for a separate call if it 
        // becomes necessary on some applications.
    }
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Prepares the hardware platform for booting.
//!
//! \return Return value from ddi_emi_PreOsInit().
////////////////////////////////////////////////////////////////////////////////
int start()
{
    int  iReturnValue = 0;

    hw_emi_MemType_t        type = EMI_DEV_DDR1;
 // hw_emi_ChipSelectMask_t mask = CE0 | CE1; //wei
    hw_emi_ChipSelectMask_t mask = CE0;
//  hw_emi_TotalDramSize_t  size = EMI_128MB_DRAM; //wei
    hw_emi_TotalDramSize_t  size = EMI_64MB_DRAM;

#if 0 //wei
#ifdef BSP_MDDR_MEMORY    
    type = EMI_DEV_MOBILE_DDR;
#endif
    // dev board only has 64 MB of RAM on CE0
    if ( (HW_OCOTP_CUSTCAP_RD() & OCOTP_CUSTCAP_BOARDID_MASK) == BOARDID_DEVBOARD)
    {
        size = EMI_64MB_DRAM;//64M
        mask = CE0;
    }
#endif
    InitDebugSerial();

    InitPower();

    iReturnValue = ddi_emi_PreOsInit(type, mask, size);

    //--------------------------------------------------------------------------
    // Return to the ROM.
    //--------------------------------------------------------------------------
    return(iReturnValue);
   
}

/////////////////////////////////////////////////////////////////////////////////
//
//! \brief Function to initialize the EMI controlller and SDRAM
//!
//! \fntype Function
//!
//!
//! \param[in] MemType          Type of memory
//! \param[in] ChipSelectMask   Map/Mask of CE pins used
//! \param[in] TotalDramSize    Size in bytes of entire SDRAM memory
//!
//! \return RtStatus, SUCCESS or error code
//
/////////////////////////////////////////////////////////////////////////////////
int ddi_emi_PreOsInit(hw_emi_MemType_t MemType,hw_emi_ChipSelectMask_t ChipSelectMask,hw_emi_TotalDramSize_t TotalDramSize)
{
    int Rtn;
    bool bInterruptEnableState;
    unsigned int StartTime;
    unsigned char i, NumChipSelects = 0;
    hw_emi_TotalDramSize_t TotalDieSize;  // for calculating column_size and addr_pins

    // save the memory type       
    s_ddi_emi_vars.MemType = MemType;
    
    
    // initialize these state variables
    s_ddi_emi_vars.bAutoMemorySelfRefreshModeEnabled = FALSE;
    s_ddi_emi_vars.bAutoMemoryClockGateModeEnabled = FALSE;
    s_ddi_emi_vars.bStaticMemorySelfRefreshModeEnabled = FALSE;
    s_ddi_emi_vars.bStaticMemoryClockGateModeEnabled = FALSE;
    s_ddi_emi_vars.bLowPfdAllowed = FALSE;

    
    // if using DDR1, power up the 2p5 rail and keep the PLL powered
    // as DDR minimum speeds require the PLL to always be powered.

    if(MemType==EMI_DEV_DDR1)
    {
        hw_power_Enable2p5(TRUE);
        ddi_clocks_KeepPllPowered(TRUE);
    }
    else if(MemType==EMI_DEV_MOBILE_DDR)
    {
        hw_power_Enable1p8(TRUE);
        ddi_clocks_KeepPllPowered(TRUE);
    }
    // this may be temporary.  Another direction is to have a separate function
    // to reset the peripheral if the application tries to initial it.  But
    // this requires that this function not be called while the SDRAM contents need
    // to be retained which is currently violated by the SDK.

    // make sure the HW_DRAM registers have a clock signal.  Otherwise, any
    // attempt to read a HW_DRAM register will result in a crash.
    if(
        // make sure we have a clock path.
        (HW_CLKCTRL_CLKSEQ.B.BYPASS_EMI && !HW_CLKCTRL_EMI.B.CLKGATE) ||   // xtal
        (!HW_CLKCTRL_CLKSEQ.B.BYPASS_EMI && !HW_CLKCTRL_FRAC.B.CLKGATEEMI) // pll
      )
    {
        if(HW_DRAM_CTL08.B.START == 1) // is EMI controller already on?
        {
        
            ddi_emi_StartHighCpuPriority(); // \todo - probably doesn't need to be here, since values have most likely be written if EMI controller already on.
        
            ddi_emi_ExitAutoMemorySelfRefreshMode();   // only would want for VERY low power mode.
            ddi_emi_EnterAutoMemoryClockGateMode();    // default mode to save power.
            
            ddi_emi_InitClockSpeedDependentVars();
            return SUCCESS;  // emi peripheral already initialized.
        }
    }

    // Handle the ChipSelectMask input
    if(!ChipSelectMask)
    {
        return ERROR_DDI_EMI_NO_CHIP_SELECT_WAS_SELECTED;
    }
    else
    {
        for(i = 0; i < 4; i++)
        {
            if(ChipSelectMask & (1 << i))
                NumChipSelects++;   // comes in handy for *verifying* total addressable memory.
        }
      // 
        if(NumChipSelects > DDI_EMI_MAX_NUM_CHIP_SELECTS)
            return ERROR_DDI_EMI_ONLY_2_CHIP_SELECTS_SUPPORTED;
    }

  // if( (TotalDramSize / NumChipSelects) > EMI_64MB_DRAM)  // check for overflow of addressable memory.
  //      return ERROR_DDI_EMI_64MB_PER_CHIP_SELCT_MAXIMUM;

    // initialize clocks, and force usage of Xtal.
    // Reading/writing certain DRAM's (ones that have their own DLL) would be problematic with 24Mhz.
    hw_clkctrl_SetClkBypass(0x40, TRUE);//bypass EMI
    hw_clkctrl_SetEmiRefXtalClkGate(FALSE);
    hw_clkctrl_SetEmiClkRefXtalDiv(1);

    // Enable the EMI block by clearing the Soft Reset and Clock Gate
    hw_emi_ClearReset();

    // check to see if EMI controller support SDRAM communication
    if(!hw_emi_IsDramSupported())
    {
        return ERROR_DDI_EMI_SDRAM_NOT_SUPPORTED;
    }

    // Initialize the IOs used by the EMI controller
    if(MemType == EMI_DEV_SDRAM)
    {
            // Set up the pinmux for the EMI
        hw_emi_ConfigureEmiPins(
            PIN_VOLTAGE_3pt3V, 
            EMI_PIN_DRIVE_ADDRESS,
            EMI_PIN_DRIVE_DATA,
            EMI_PIN_DRIVE_CHIP_ENABLE,
            EMI_PIN_DRIVE_CLOCK,
            EMI_PIN_DRIVE_CONTROL);
    
    }
    else if(MemType == EMI_DEV_MOBILE_SDRAM)
    {
           // Set up the pinmux for the EMI
        hw_emi_ConfigureEmiPins(
            PIN_VOLTAGE_1pt8V, 
            EMI_PIN_DRIVE_ADDRESS,
            EMI_PIN_DRIVE_DATA,
            EMI_PIN_DRIVE_CHIP_ENABLE,
            EMI_PIN_DRIVE_CLOCK,
            EMI_PIN_DRIVE_CONTROL);
        
    
    }
    else if(MemType == EMI_DEV_MOBILE_DDR)
    {
        // Set up the pinmux for the EMI
        hw_emi_ConfigureEmiPins(
            PIN_VOLTAGE_1pt8V, 
            EMI_PIN_DRIVE_ADDRESS,
            EMI_PIN_DRIVE_DATA,
            EMI_PIN_DRIVE_CHIP_ENABLE,
            EMI_PIN_DRIVE_CLOCK,
            EMI_PIN_DRIVE_CONTROL);
    
    }

    else if(MemType == EMI_DEV_DDR1)
    {
        // Set up the pinmux for the EMI
        hw_emi_ConfigureEmiPins(
            PIN_VOLTAGE_1pt8V, 
            EMI_PIN_DRIVE_ADDRESS,
            EMI_PIN_DRIVE_DATA,
            EMI_PIN_DRIVE_CHIP_ENABLE,
            EMI_PIN_DRIVE_CLOCK,
            EMI_PIN_DRIVE_CONTROL);
    
    }


    // pin keepers are disabled for pins which are also driven by the memory
    hw_emi_DisableEmiPadKeepers();

    //disable interrupts, so nothing can access SDRAM while we're doing this.
    bInterruptEnableState = hw_core_EnableIrqInterrupt(FALSE);

    // Make sure we are not in these low power 
    ddi_emi_ExitAutoMemorySelfRefreshMode(); // \todo possibly unnecessary.
    ddi_emi_ExitAutoMemoryClockGateMode();

    // flush and invalidate the data cache to ensure nothing gets written to sdram while
    // we are changing the emi clk
    ddi_emi_PrepareForDramSelfRefreshMode();

    // Enter controller self-refresh mode
    Rtn = ddi_emi_EnterStaticSelfRefreshMode();
    if(Rtn != 0)
    {
        return Rtn;
    }

    // Initialize EMI controller registers for specific memory being used  
/*    if(MemType == EMI_DEV_SDRAM)
    {
        hw_dram_Init_sdram_mt48lc32m16a2_24MHz();
    }
    else if(MemType == EMI_DEV_MOBILE_SDRAM)
    {
        hw_dram_Init_mobile_sdram_k4m56163pg_7_5_24MHz();
        //hw_dram_Init_mobile_sdram_mt48h16m16lf_7_5_24MHz();

    }
    else if(MemType == EMI_DEV_MOBILE_DDR)
    {
        // Write the Databahn SDRAM setup register values
        hw_dram_Init_mobile_ddr_mt46h16m16lf_7_5_24MHz();
    }
  
    else if(MemType==EMI_DEV_DDR1)
    {
        // DDR1 requires clock change to higher speed clock
        //ddi_clocks_SetEmiClk(&EmiKhz);
        hw_clkctrl_SetPfdRefEmiGate(TRUE); // needed as sort of a high freq. "filter" for peripherals prior to ramping up PLL
        hw_clkctrl_PowerPll(TRUE);
        hw_digctl_MicrosecondWait( 11 ); // \todo There is a bit that can tell you when its stable, but it takes FOREVER, so we use this much lower time that Mike May said was good.
        
        hw_clkctrl_SetPfdRefEmiGate(FALSE);
        hw_clkctrl_SetPfdRefEmiDiv(30);    // \todo - might be good to move this BEFORE the above line.
        hw_clkctrl_SetEmiClkRefEmiDiv(3);
        hw_clkctrl_SetClkBypass(0x40, FALSE);
        hw_dram_Init_ddr_mt46v32m16_6t_96MHz();
    }
*/
    if(MemType==EMI_DEV_DDR1)
    {
        // DDR1 requires clock change to higher speed clock
        //ddi_clocks_SetEmiClk(&EmiKhz);
        hw_clkctrl_SetPfdRefEmiGate(TRUE); // needed as sort of a high freq. "filter" for peripherals prior to ramping up PLL
        hw_clkctrl_PowerPll(TRUE);
        hw_digctl_MicrosecondWait( 11 ); // \todo There is a bit that can tell you when its stable, but it takes FOREVER, so we use this much lower time that Mike May said was good.
        
        hw_clkctrl_SetPfdRefEmiGate(FALSE);
        hw_clkctrl_SetPfdRefEmiDiv(33);    // \todo - might be good to move this BEFORE the above line.
        hw_clkctrl_SetEmiClkRefEmiDiv(2);
        hw_clkctrl_SetClkBypass(0x40, FALSE);

        hw_dram_Init_ddr_mt46v32m16_6t_96MHz();  //(30,3)
        hw_dram_Init_ddr_mt46v32m16_6t_133MHz_optimized(); //(33,2)
    }
    else if(MemType == EMI_DEV_MOBILE_DDR)
    {
    
        // DDR1 requires clock change to higher speed clock
        //ddi_clocks_SetEmiClk(&EmiKhz);
        hw_clkctrl_SetPfdRefEmiGate(TRUE); // needed as sort of a high freq. "filter" for peripherals prior to ramping up PLL
        hw_clkctrl_PowerPll(TRUE);
        hw_digctl_MicrosecondWait( 11 ); // \todo There is a bit that can tell you when its stable, but it takes FOREVER, so we use this much lower time that Mike May said was good.
        
        hw_clkctrl_SetPfdRefEmiGate(FALSE);
        hw_clkctrl_SetPfdRefEmiDiv(33);    // \todo - might be good to move this BEFORE the above line.
        hw_clkctrl_SetEmiClkRefEmiDiv(2);
        hw_clkctrl_SetClkBypass(0x40, FALSE);
        // Write the Databahn SDRAM setup register values        
        hw_dram_Init_mobile_ddr_mt46h32m16lf_5_regs_3_133MHz();
    }
    // Set the controller chip selection masked to passed in value
    HW_DRAM_CTL14.B.CS_MAP = ChipSelectMask;

    // here we calculate the TotalDieSize to get the COLUMN_SIZE and
    // ADDR_PINS per die.  Avoid long division if possible
    if(NumChipSelects==1)
        TotalDieSize = TotalDramSize;
    else
        TotalDieSize = TotalDramSize >> (NumChipSelects - 1);


    if(TotalDieSize == EMI_2MB_DRAM)
    {
        HW_DRAM_CTL11.B.COLUMN_SIZE = 4;
        HW_DRAM_CTL10.B.ADDR_PINS = 2;
    }
    else if(TotalDieSize == EMI_4MB_DRAM)
    {
        HW_DRAM_CTL11.B.COLUMN_SIZE = 4;
        HW_DRAM_CTL10.B.ADDR_PINS = 1;
    }
    else if(TotalDieSize == EMI_8MB_DRAM)
    {
        HW_DRAM_CTL11.B.COLUMN_SIZE = 4;
        HW_DRAM_CTL10.B.ADDR_PINS = 1;
    }
    else if(TotalDieSize == EMI_16MB_DRAM)
    {
        HW_DRAM_CTL11.B.COLUMN_SIZE = 3;
        HW_DRAM_CTL10.B.ADDR_PINS = 1;
    }
    else if(TotalDieSize == EMI_32MB_DRAM)
    {
        HW_DRAM_CTL11.B.COLUMN_SIZE = 3;
        HW_DRAM_CTL10.B.ADDR_PINS = 0;
    }
    else if(TotalDieSize == EMI_64MB_DRAM)
    {
        HW_DRAM_CTL11.B.COLUMN_SIZE = 2;
        HW_DRAM_CTL10.B.ADDR_PINS = 0;
    }
    else
    {
        return ERROR_DDI_EMI_SDRAM_NOT_SUPPORTED;
    }
    
    // For some reason, this must be done for ESMT mobile sdram parts
    // which must be initialized as non-mobile parts
    if( (TotalDieSize == EMI_2MB_DRAM) &&
        (s_ddi_emi_vars.MemType == EMI_DEV_MOBILE_SDRAM) )
    {
        HW_DRAM_CTL05.B.EN_LOWPOWER_MODE = 0;
    }

    // All controller and clock registers are set.  Start the EMI 
    // controller (initializes the SDRAM)
    HW_DRAM_CTL08.B.START = 1;

    StartTime = hw_digctl_GetCurrentTime();
    
    // todo:  Add timeout and error
    while (HW_CLKCTRL_EMI_RD() &
            (BM_CLKCTRL_EMI_BUSY_REF_EMI  |
             BM_CLKCTRL_EMI_BUSY_DCC_RESYNC))
             {}
    {
        if(hw_digctl_CheckTimeOut(StartTime, 10000)) // \todo this time is kind of arbitrary, also we are not checking controller.
        {   
            //return ERROR_DDI_EMI_RESYNC_TIMEOUT;
            return SUCCESS;
        }
    }
    // Exit self refresh to allow access to memory
    Rtn = ddi_emi_ExitStaticSelfRefreshMode();

    
    // Make CPU memory access the highest priority to increase
    // processing efficiency which leads to higher performance and/or
    // lower power usage.             
    ddi_emi_StartHighCpuPriority();


    // Start the automatic clock gating of memory based on activity
    // to save power
    ddi_emi_EnterAutoMemoryClockGateMode();

    ddi_emi_InitClockSpeedDependentVars();
    // We're all done. Re-enable interrupts
    hw_core_EnableIrqInterrupt(bInterruptEnableState);
    
    return Rtn;
}

//------------------------------------------------------------------------------
//
//  Function: hw_core_EnableIrqInterrupt
//
//
//------------------------------------------------------------------------------

bool hw_core_EnableIrqInterrupt(bool bNewState)
{
    return bNewState;
}

//* Function Specification *****************************************************
//!
//! \brief Initialize Pin Mux to enable all EMI pins
//!
//! This function sets each of the Pin Mux select registers to enable all the
//! EMI associated pins. In addition, it also set the voltage level and drive
//! strength as specified for each of the EMI pins. This routine only sets/clears
//! the bits necessary for those pins associated with EMI. No other Pin Mux
//! settings are changed.
//!
//! \param[in] pin_voltage    - Pin voltage assigned to each EMI pin.
//! \param[in] pin_drive_addr - Pin drive strength (mA) assigned to EMI address pins.
//! \param[in] pin_drive_data - Pin drive strength (mA) assigned to EMI data pins.
//! \param[in] pin_drive_ce   - Pin drive strength (mA) assigned to EMI chip select pins.
//! \param[in] pin_drive_ctrl - Pin drive strength (mA) assigned to EMI control pins.
//! \param[in] pin_drive_clk  - Pin drive strength (mA) assigned to EMI clock pins.
//!
//******************************************************************************
void hw_emi_ConfigureEmiPins(
        TPinVoltage pin_voltage,
        TPinDrive pin_drive_addr,
        TPinDrive pin_drive_data,
        TPinDrive pin_drive_ce,
        TPinDrive pin_drive_clk,
        TPinDrive pin_drive_ctrl)
{
    //-------------------------------------------------------------------------
    // Enable the Pinmux by clearing the Soft Reset and Clock Gate
    //-------------------------------------------------------------------------
    HW_PINCTRL_CTRL_CLR(BM_PINCTRL_CTRL_SFTRST | BM_PINCTRL_CTRL_CLKGATE);

// Bank-0 EMI pins are needed for NOR flash only. These pins conflict with
// booting from NAND flash. This function needs to be reworked to account for
// this conflict. For now, just comment out the NOR flash pins.


    //-------------------------------------------------------------------------
    // Bank-0 of the Pinmux does not contain any EMI pins
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    // Bank-1 of the Pinmux does not contain any EMI pins
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    // Bank-2 of the Pinmux contains EMI pins at 9-31.
    // First, set the voltage and drive strength of these pins as specified.
    // Second, set the pinmux value to 0x0 to enable the EMI connection.
    //-------------------------------------------------------------------------

    // EMI_A00-06
    // Configure Bank-2 Pins 9-15 voltage and drive strength
    HW_PINCTRL_DRIVE9_CLR(
    BM_PINCTRL_DRIVE9_BANK2_PIN09_V | BM_PINCTRL_DRIVE9_BANK2_PIN09_MA |
    BM_PINCTRL_DRIVE9_BANK2_PIN10_V | BM_PINCTRL_DRIVE9_BANK2_PIN10_MA |
    BM_PINCTRL_DRIVE9_BANK2_PIN11_V | BM_PINCTRL_DRIVE9_BANK2_PIN11_MA |
    BM_PINCTRL_DRIVE9_BANK2_PIN12_V | BM_PINCTRL_DRIVE9_BANK2_PIN12_MA |
    BM_PINCTRL_DRIVE9_BANK2_PIN13_V | BM_PINCTRL_DRIVE9_BANK2_PIN13_MA |
    BM_PINCTRL_DRIVE9_BANK2_PIN14_V | BM_PINCTRL_DRIVE9_BANK2_PIN14_MA |
    BM_PINCTRL_DRIVE9_BANK2_PIN15_V | BM_PINCTRL_DRIVE9_BANK2_PIN15_MA);

    HW_PINCTRL_DRIVE9_SET(
    BF_PINCTRL_DRIVE9_BANK2_PIN09_V(pin_voltage) | BF_PINCTRL_DRIVE9_BANK2_PIN09_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE9_BANK2_PIN10_V(pin_voltage) | BF_PINCTRL_DRIVE9_BANK2_PIN10_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE9_BANK2_PIN11_V(pin_voltage) | BF_PINCTRL_DRIVE9_BANK2_PIN11_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE9_BANK2_PIN12_V(pin_voltage) | BF_PINCTRL_DRIVE9_BANK2_PIN12_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE9_BANK2_PIN13_V(pin_voltage) | BF_PINCTRL_DRIVE9_BANK2_PIN13_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE9_BANK2_PIN14_V(pin_voltage) | BF_PINCTRL_DRIVE9_BANK2_PIN14_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE9_BANK2_PIN15_V(pin_voltage) | BF_PINCTRL_DRIVE9_BANK2_PIN15_MA(pin_drive_addr));

    // EMI_A07-12, EMI_BA0-1
    // Configure Bank-2 Pins 16-23 voltage and drive strength
    HW_PINCTRL_DRIVE10_CLR(
    BM_PINCTRL_DRIVE10_BANK2_PIN16_V | BM_PINCTRL_DRIVE10_BANK2_PIN16_MA |
    BM_PINCTRL_DRIVE10_BANK2_PIN17_V | BM_PINCTRL_DRIVE10_BANK2_PIN17_MA |
    BM_PINCTRL_DRIVE10_BANK2_PIN18_V | BM_PINCTRL_DRIVE10_BANK2_PIN18_MA |
    BM_PINCTRL_DRIVE10_BANK2_PIN19_V | BM_PINCTRL_DRIVE10_BANK2_PIN19_MA |
    BM_PINCTRL_DRIVE10_BANK2_PIN20_V | BM_PINCTRL_DRIVE10_BANK2_PIN20_MA |
    BM_PINCTRL_DRIVE10_BANK2_PIN21_V | BM_PINCTRL_DRIVE10_BANK2_PIN21_MA |
    BM_PINCTRL_DRIVE10_BANK2_PIN22_V | BM_PINCTRL_DRIVE10_BANK2_PIN22_MA |
    BM_PINCTRL_DRIVE10_BANK2_PIN23_V | BM_PINCTRL_DRIVE10_BANK2_PIN23_MA);

    HW_PINCTRL_DRIVE10_SET(
    BF_PINCTRL_DRIVE10_BANK2_PIN16_V(pin_voltage) | BF_PINCTRL_DRIVE10_BANK2_PIN16_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE10_BANK2_PIN17_V(pin_voltage) | BF_PINCTRL_DRIVE10_BANK2_PIN17_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE10_BANK2_PIN18_V(pin_voltage) | BF_PINCTRL_DRIVE10_BANK2_PIN18_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE10_BANK2_PIN19_V(pin_voltage) | BF_PINCTRL_DRIVE10_BANK2_PIN19_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE10_BANK2_PIN20_V(pin_voltage) | BF_PINCTRL_DRIVE10_BANK2_PIN20_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE10_BANK2_PIN21_V(pin_voltage) | BF_PINCTRL_DRIVE10_BANK2_PIN21_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE10_BANK2_PIN22_V(pin_voltage) | BF_PINCTRL_DRIVE10_BANK2_PIN22_MA(pin_drive_addr) |
    BF_PINCTRL_DRIVE10_BANK2_PIN23_V(pin_voltage) | BF_PINCTRL_DRIVE10_BANK2_PIN23_MA(pin_drive_addr));

    // EMI_CAS,RAS,CE0-2,WEN,CKE
    // Configure Bank-2 Pins 24-31 voltage and drive strength
    HW_PINCTRL_DRIVE11_CLR(
    BM_PINCTRL_DRIVE11_BANK2_PIN24_V | BM_PINCTRL_DRIVE11_BANK2_PIN24_MA |
    BM_PINCTRL_DRIVE11_BANK2_PIN25_V | BM_PINCTRL_DRIVE11_BANK2_PIN25_MA |
    BM_PINCTRL_DRIVE11_BANK2_PIN26_V | BM_PINCTRL_DRIVE11_BANK2_PIN26_MA |
    BM_PINCTRL_DRIVE11_BANK2_PIN29_V | BM_PINCTRL_DRIVE11_BANK2_PIN29_MA |
    BM_PINCTRL_DRIVE11_BANK2_PIN30_V | BM_PINCTRL_DRIVE11_BANK2_PIN30_MA |
    BM_PINCTRL_DRIVE11_BANK2_PIN31_V | BM_PINCTRL_DRIVE11_BANK2_PIN31_MA);

    HW_PINCTRL_DRIVE11_SET(
    BF_PINCTRL_DRIVE11_BANK2_PIN24_V(pin_voltage) | BF_PINCTRL_DRIVE11_BANK2_PIN24_MA(pin_drive_ctrl) |
    BF_PINCTRL_DRIVE11_BANK2_PIN25_V(pin_voltage) | BF_PINCTRL_DRIVE11_BANK2_PIN25_MA(pin_drive_ce) |
    BF_PINCTRL_DRIVE11_BANK2_PIN26_V(pin_voltage) | BF_PINCTRL_DRIVE11_BANK2_PIN26_MA(pin_drive_ce) |
    BF_PINCTRL_DRIVE11_BANK2_PIN29_V(pin_voltage) | BF_PINCTRL_DRIVE11_BANK2_PIN29_MA(pin_drive_ctrl) |
    BF_PINCTRL_DRIVE11_BANK2_PIN30_V(pin_voltage) | BF_PINCTRL_DRIVE11_BANK2_PIN30_MA(pin_drive_ctrl) |
    BF_PINCTRL_DRIVE11_BANK2_PIN31_V(pin_voltage) | BF_PINCTRL_DRIVE11_BANK2_PIN31_MA(pin_drive_ctrl));

    // Configure Bank-2 Pins 9-15 as EMI pins
    HW_PINCTRL_MUXSEL4_CLR(
    BM_PINCTRL_MUXSEL4_BANK2_PIN09 |
    BM_PINCTRL_MUXSEL4_BANK2_PIN10 |
    BM_PINCTRL_MUXSEL4_BANK2_PIN11 |
    BM_PINCTRL_MUXSEL4_BANK2_PIN12 |
    BM_PINCTRL_MUXSEL4_BANK2_PIN13 |
    BM_PINCTRL_MUXSEL4_BANK2_PIN14 |
    BM_PINCTRL_MUXSEL4_BANK2_PIN15);

    // Configure Bank-2 Pins 16-31 as EMI pins
    HW_PINCTRL_MUXSEL5_CLR(
    BM_PINCTRL_MUXSEL5_BANK2_PIN16 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN17 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN18 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN19 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN20 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN21 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN22 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN23 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN24 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN25 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN26 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN29 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN30 |
    BM_PINCTRL_MUXSEL5_BANK2_PIN31);

    // EMI CE2 and EMI CE3 are secondary (select == 1) functionality on their pins
    //-------------------------------------------------------------------------
    // Bank-3 of the Pinmux contains EMI pins at 0-21.
    // First, set the voltage and drive strength of these pins as specified.
    // Second, set the pinmux value to 0x0 to enable the EMI connection.
    //-------------------------------------------------------------------------

    // EMI_D00-07
    // Configure Bank-3 Pins 00-07 voltage and drive strength
    HW_PINCTRL_DRIVE12_CLR(
    BM_PINCTRL_DRIVE12_BANK3_PIN00_V | BM_PINCTRL_DRIVE12_BANK3_PIN00_MA |
    BM_PINCTRL_DRIVE12_BANK3_PIN01_V | BM_PINCTRL_DRIVE12_BANK3_PIN01_MA |
    BM_PINCTRL_DRIVE12_BANK3_PIN02_V | BM_PINCTRL_DRIVE12_BANK3_PIN02_MA |
    BM_PINCTRL_DRIVE12_BANK3_PIN03_V | BM_PINCTRL_DRIVE12_BANK3_PIN03_MA |
    BM_PINCTRL_DRIVE12_BANK3_PIN04_V | BM_PINCTRL_DRIVE12_BANK3_PIN04_MA |
    BM_PINCTRL_DRIVE12_BANK3_PIN05_V | BM_PINCTRL_DRIVE12_BANK3_PIN05_MA |
    BM_PINCTRL_DRIVE12_BANK3_PIN06_V | BM_PINCTRL_DRIVE12_BANK3_PIN06_MA |
    BM_PINCTRL_DRIVE12_BANK3_PIN07_V | BM_PINCTRL_DRIVE12_BANK3_PIN07_MA);

    HW_PINCTRL_DRIVE12_SET(
    BF_PINCTRL_DRIVE12_BANK3_PIN00_V(pin_voltage) | BF_PINCTRL_DRIVE12_BANK3_PIN00_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE12_BANK3_PIN01_V(pin_voltage) | BF_PINCTRL_DRIVE12_BANK3_PIN01_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE12_BANK3_PIN02_V(pin_voltage) | BF_PINCTRL_DRIVE12_BANK3_PIN02_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE12_BANK3_PIN03_V(pin_voltage) | BF_PINCTRL_DRIVE12_BANK3_PIN03_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE12_BANK3_PIN04_V(pin_voltage) | BF_PINCTRL_DRIVE12_BANK3_PIN04_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE12_BANK3_PIN05_V(pin_voltage) | BF_PINCTRL_DRIVE12_BANK3_PIN05_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE12_BANK3_PIN06_V(pin_voltage) | BF_PINCTRL_DRIVE12_BANK3_PIN06_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE12_BANK3_PIN07_V(pin_voltage) | BF_PINCTRL_DRIVE12_BANK3_PIN07_MA(pin_drive_data));

    // EMI_D08-15
    // Configure Bank-3 Pins 08-15 voltage and drive strength
    HW_PINCTRL_DRIVE13_CLR(
    BM_PINCTRL_DRIVE13_BANK3_PIN08_V | BM_PINCTRL_DRIVE13_BANK3_PIN08_MA |
    BM_PINCTRL_DRIVE13_BANK3_PIN09_V | BM_PINCTRL_DRIVE13_BANK3_PIN09_MA |
    BM_PINCTRL_DRIVE13_BANK3_PIN10_V | BM_PINCTRL_DRIVE13_BANK3_PIN10_MA |
    BM_PINCTRL_DRIVE13_BANK3_PIN11_V | BM_PINCTRL_DRIVE13_BANK3_PIN11_MA |
    BM_PINCTRL_DRIVE13_BANK3_PIN12_V | BM_PINCTRL_DRIVE13_BANK3_PIN12_MA |
    BM_PINCTRL_DRIVE13_BANK3_PIN13_V | BM_PINCTRL_DRIVE13_BANK3_PIN13_MA |
    BM_PINCTRL_DRIVE13_BANK3_PIN14_V | BM_PINCTRL_DRIVE13_BANK3_PIN14_MA |
    BM_PINCTRL_DRIVE13_BANK3_PIN15_V | BM_PINCTRL_DRIVE13_BANK3_PIN15_MA);

    HW_PINCTRL_DRIVE13_SET(
    BF_PINCTRL_DRIVE13_BANK3_PIN08_V(pin_voltage) | BF_PINCTRL_DRIVE13_BANK3_PIN08_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE13_BANK3_PIN09_V(pin_voltage) | BF_PINCTRL_DRIVE13_BANK3_PIN09_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE13_BANK3_PIN10_V(pin_voltage) | BF_PINCTRL_DRIVE13_BANK3_PIN10_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE13_BANK3_PIN11_V(pin_voltage) | BF_PINCTRL_DRIVE13_BANK3_PIN11_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE13_BANK3_PIN12_V(pin_voltage) | BF_PINCTRL_DRIVE13_BANK3_PIN12_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE13_BANK3_PIN13_V(pin_voltage) | BF_PINCTRL_DRIVE13_BANK3_PIN13_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE13_BANK3_PIN14_V(pin_voltage) | BF_PINCTRL_DRIVE13_BANK3_PIN14_MA(pin_drive_data) |
    BF_PINCTRL_DRIVE13_BANK3_PIN15_V(pin_voltage) | BF_PINCTRL_DRIVE13_BANK3_PIN15_MA(pin_drive_data));

    // EMI_DQS0-1,DQM0-1,CLK,CLKN
    // Configure Bank-3 Pins 08-15 voltage and drive strength
    HW_PINCTRL_DRIVE14_CLR(
    BM_PINCTRL_DRIVE14_BANK3_PIN16_V | BM_PINCTRL_DRIVE14_BANK3_PIN16_MA |
    BM_PINCTRL_DRIVE14_BANK3_PIN17_V | BM_PINCTRL_DRIVE14_BANK3_PIN17_MA |
    BM_PINCTRL_DRIVE14_BANK3_PIN18_V | BM_PINCTRL_DRIVE14_BANK3_PIN18_MA |
    BM_PINCTRL_DRIVE14_BANK3_PIN19_V | BM_PINCTRL_DRIVE14_BANK3_PIN19_MA |
    BM_PINCTRL_DRIVE14_BANK3_PIN20_V | BM_PINCTRL_DRIVE14_BANK3_PIN20_MA |
    BM_PINCTRL_DRIVE14_BANK3_PIN21_V | BM_PINCTRL_DRIVE14_BANK3_PIN21_MA);

    HW_PINCTRL_DRIVE14_SET(
    BF_PINCTRL_DRIVE14_BANK3_PIN16_V(pin_voltage) | BF_PINCTRL_DRIVE14_BANK3_PIN16_MA(pin_drive_clk) |
    BF_PINCTRL_DRIVE14_BANK3_PIN17_V(pin_voltage) | BF_PINCTRL_DRIVE14_BANK3_PIN17_MA(pin_drive_clk) |
    BF_PINCTRL_DRIVE14_BANK3_PIN18_V(pin_voltage) | BF_PINCTRL_DRIVE14_BANK3_PIN18_MA(pin_drive_ctrl) |
    BF_PINCTRL_DRIVE14_BANK3_PIN19_V(pin_voltage) | BF_PINCTRL_DRIVE14_BANK3_PIN19_MA(pin_drive_ctrl) |
    BF_PINCTRL_DRIVE14_BANK3_PIN20_V(pin_voltage) | BF_PINCTRL_DRIVE14_BANK3_PIN20_MA(pin_drive_ctrl) |
    BF_PINCTRL_DRIVE14_BANK3_PIN21_V(pin_voltage) | BF_PINCTRL_DRIVE14_BANK3_PIN21_MA(pin_drive_ctrl));

    // Configure Bank-3 Pins 0-15 as EMI pins
    HW_PINCTRL_MUXSEL6_CLR(
    BM_PINCTRL_MUXSEL6_BANK3_PIN00 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN01 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN02 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN03 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN04 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN05 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN06 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN07 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN08 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN09 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN10 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN11 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN12 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN13 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN14 |
    BM_PINCTRL_MUXSEL6_BANK3_PIN15);

    // Configure Bank-3 Pins 16-21 as EMI pins
    HW_PINCTRL_MUXSEL7_CLR(
    BM_PINCTRL_MUXSEL7_BANK3_PIN16 |
    BM_PINCTRL_MUXSEL7_BANK3_PIN17 |
    BM_PINCTRL_MUXSEL7_BANK3_PIN18 |
    BM_PINCTRL_MUXSEL7_BANK3_PIN19 |
    BM_PINCTRL_MUXSEL7_BANK3_PIN20 |
    BM_PINCTRL_MUXSEL7_BANK3_PIN21);

}


//* Function Specification *****************************************************
//!
//! \brief Disable Bus Keepers on EMI Pins
//!
//! This function disables the internal bus keepers on the EMI pins. This is only
//! necessary when connecting a Mobile DDR device to the DRAM controller.
//!
//******************************************************************************
void hw_emi_DisableEmiPadKeepers(void)
{

    // Enable the Pinmux by clearing the Soft Reset and Clock Gate
    HW_PINCTRL_CTRL_CLR(BM_PINCTRL_CTRL_SFTRST | BM_PINCTRL_CTRL_CLKGATE);

    // Disable the internal bus-keeper pins associated with EMI.
    HW_PINCTRL_PULL3_SET(
    BM_PINCTRL_PULL3_BANK3_PIN17 |
    BM_PINCTRL_PULL3_BANK3_PIN16 |
    BM_PINCTRL_PULL3_BANK3_PIN15 |
    BM_PINCTRL_PULL3_BANK3_PIN14 |
    BM_PINCTRL_PULL3_BANK3_PIN13 |
    BM_PINCTRL_PULL3_BANK3_PIN12 |
    BM_PINCTRL_PULL3_BANK3_PIN11 |
    BM_PINCTRL_PULL3_BANK3_PIN10 |
    BM_PINCTRL_PULL3_BANK3_PIN09 |
    BM_PINCTRL_PULL3_BANK3_PIN08 |
    BM_PINCTRL_PULL3_BANK3_PIN07 |
    BM_PINCTRL_PULL3_BANK3_PIN06 |
    BM_PINCTRL_PULL3_BANK3_PIN05 |
    BM_PINCTRL_PULL3_BANK3_PIN04 |
    BM_PINCTRL_PULL3_BANK3_PIN03 |
    BM_PINCTRL_PULL3_BANK3_PIN02 |
    BM_PINCTRL_PULL3_BANK3_PIN01 |
    BM_PINCTRL_PULL3_BANK3_PIN00);

}

/////////////////////////////////////////////////////////////////////////////////
//
//! \brief Exit the automatic memory clock gate mode
//!
//! \fntype Function
//!
//! \return none
//
/////////////////////////////////////////////////////////////////////////////////
void ddi_emi_ExitAutoMemoryClockGateMode(void)
{
    hw_emi_EnterMemoryClockGateMode(FALSE);
    hw_emi_SetMemoryClockGateAutoFlag(FALSE);
    s_ddi_emi_vars.bAutoMemoryClockGateModeEnabled = FALSE;
}

/////////////////////////////////////////////////////////////////////////////////
//
//! \brief Clean caches and write buffer in preparation for sdram self refresh 
//!
//! \fntype Function
//!
//! \return none
//
/////////////////////////////////////////////////////////////////////////////////
void ddi_emi_PrepareForDramSelfRefreshMode(void)
{

    // invalidate instruction cache not necessary in SDK.  Maybe necessary elsewhere
    //hw_core_invalidate_ICache();
    
    // clean the data cache to ensure nothing gets written to sdram while
    // we are in self-refresh mode
    // hw_core_clean_DCache();


    // make sure data written    
    hw_core_drain_write_buffer();

}

/////////////////////////////////////////////////////////////////////////////////
//
//! \brief Enter the static self refresh controller and sdram state
//!
//! \fntype Function
//!
//! \return SUCCESS or error code
//
/////////////////////////////////////////////////////////////////////////////////
int ddi_emi_EnterStaticSelfRefreshMode(void)
{
    unsigned int StartTime;

    hw_emi_EnterMemorySelfRefreshMode(TRUE);

    StartTime = hw_digctl_GetCurrentTime();

    while(!hw_emi_IsControllerHalted())
    {
        if(hw_digctl_CheckTimeOut(StartTime, 1000))
        {
            return ERROR_DDI_EMI_ENTER_SELF_REFRESH_TIMEOUT;
        }
    }
    return SUCCESS;
}

//------------------------------------------------------------------------------
//
//  Function: hw_clkctrl_SetPfdRefEmiGate
//
//
//------------------------------------------------------------------------------

void hw_clkctrl_SetPfdRefEmiGate(bool bClkGate)
{
    // Gate or ungate the ref_emi clock
    if(bClkGate)
        HW_CLKCTRL_FRAC_SET(BM_CLKCTRL_FRAC_CLKGATEEMI);
    else
        HW_CLKCTRL_FRAC_CLR(BM_CLKCTRL_FRAC_CLKGATEEMI);
}
//------------------------------------------------------------------------------
//
//  Function: hw_clkctrl_PowerPll
//
//
//------------------------------------------------------------------------------

void hw_clkctrl_PowerPll(bool bPowerOn)
{
    //--------------------------------------------------------------------------
    //  Debugging code.  Never turn off the PLL if any of the PFD gates are clear
    //--------------------------------------------------------------------------

    // Power on PLL
    if(bPowerOn)
        HW_CLKCTRL_PLLCTRL0_SET(BM_CLKCTRL_PLLCTRL0_POWER);
    else
        HW_CLKCTRL_PLLCTRL0_CLR(BM_CLKCTRL_PLLCTRL0_POWER);
    // We may also be able to wait on HW_CLKCTRL_PLLCTRL1::LOCK.  When it becomes zero?
}

//------------------------------------------------------------------------------
//
//  Function: hw_clkctrl_SetPfdRefEmiDiv
//
//
//------------------------------------------------------------------------------

int hw_clkctrl_SetPfdRefEmiDiv(unsigned int u32Div)
{
    // Don't change divider if clock is gated
    if(HW_CLKCTRL_FRAC.B.CLKGATEEMI)
        return ERROR_HW_CLKCTRL_REF_CLK_GATED;

    // Set divider for EMI clock PFD
    HW_CLKCTRL_FRAC.B.EMIFRAC = u32Div;

    return SUCCESS;
}
//------------------------------------------------------------------------------
//
//  Function: hw_clkctrl_SetEmiClkRefEmiDiv
//
//
//------------------------------------------------------------------------------

int hw_clkctrl_SetEmiClkRefEmiDiv(unsigned int u32Div)
{
    ////////////////////////////////////
    // Error checks
    ////////////////////////////////////

    // Return if busy with another divider change
    if(HW_CLKCTRL_EMI.B.BUSY_REF_EMI)
        return ERROR_HW_CLKCTRL_CLK_DIV_BUSY;

    // Check upstream reference clock gate
    if(HW_CLKCTRL_FRAC.B.CLKGATEEMI)
        return ERROR_HW_CLKCTRL_REF_CLK_GATED;


    ////////////////////////////////////
    // Change gate and/or divider
    ////////////////////////////////////

    // Set divider for DIV_EMI
    HW_CLKCTRL_EMI.B.DIV_EMI = u32Div;

    // Done
    return SUCCESS;

}
//------------------------------------------------------------------------------
//
//  Function: hw_dram_Init_ddr_mt46v32m16_6t_96MHz
//
//
//------------------------------------------------------------------------------

void hw_dram_Init_ddr_mt46v32m16_6t_96MHz(void)
{
    volatile unsigned int* DRAM_REG = (volatile unsigned int*) HW_DRAM_CTL00_ADDR;

    DRAM_REG[ 0] = 0x01010001;  /* 0000000_1 ahb0_w_priority 0000000_1 ahb0_r_priority 0000000_0 ahb0_fifo_type_reg 0000000_1 addr_cmp_en */
    DRAM_REG[ 1] = 0x00010100;  /* 0000000_0 ahb2_fifo_type_reg 0000000_1 ahb1_w_priority 0000000_1 ahb1_r_priority 0000000_0 ahb1_fifo_type_reg */
    DRAM_REG[ 2] = 0x01000101;  /* 0000000_1 ahb3_r_priority 0000000_0 ahb3_fifo_type_reg 0000000_1 ahb2_w_priority 0000000_1 ahb2_r_priority */
    DRAM_REG[ 3] = 0x00000001;  /* 0000000_0 auto_refresh_mode 0000000_0 arefresh 0000000_0 ap 0000000_1 ahb3_w_priority */
    DRAM_REG[ 4] = 0x00000101;  /* 0000000_0 dll_bypass_mode 0000000_0 dlllockreg 0000000_1 concurrentap 0000000_1 bank_split_en */
    DRAM_REG[ 5] = 0x00000000;  /* 0000000_0 intrptreada 0000000_0 intrptapburst 0000000_0 fast_write 0000000_0 en_lowpower_mode */
    DRAM_REG[ 6] = 0x00010000;  /* 0000000_0 power_down 0000000_1 placement_en 0000000_0 no_cmd_init 0000000_0 intrptwritea */
    DRAM_REG[ 7] = 0x01000001;  /* 0000000_1 rw_same_en 0000000_0 reg_dimm_enable 0000000_0 rd2rd_turn 0000000_1 priority_en */
    DRAM_REG[ 9] = 0x00000001;  /* 000000_00 out_of_range_type 000000_00 out_of_range_source_id 0000000_0 write_modereg 0000000_1 writeinterp */
    DRAM_REG[10] = 0x07000200;  /* 00000_111 age_count 00000_000 addr_pins 000000_10 temrs 000000_00 q_fullness */
    DRAM_REG[11] = 0x00070202;  /* 00000_000 max_cs_reg 00000_111 command_age_count 00000_010 column_size 00000_010 caslat */
    DRAM_REG[12] = 0x02020000;  /* 00000_010 twr_int 00000_010 trrd 0000000000000_000 tcke */
    DRAM_REG[13] = 0x04040a01;  /* 0000_0100 caslat_lin_gate 0000_0100 caslat_lin 0000_1010 aprebit 00000_001 twtr */
    DRAM_REG[14] = 0x00000203;  /* 0000_0000 max_col_reg 0000_0000 lowpower_refresh_enable 0000_0010 initaref 0000_0011 cs_map */
    DRAM_REG[15] = 0x02040000;  /* 0000_0010 trp 0000_0100 tdal 0000_0000 port_busy 0000_0000 max_row_reg */
    DRAM_REG[16] = 0x02000000;  /* 000_00010 tmrd 000_00000 lowpower_control 000_00000 lowpower_auto_enable 0000_0000 int_ack */
    DRAM_REG[17] = 0x25001506;  /* 01001110 dll_start_point 00000000 dll_lock 00010101 dll_increment 000_00110 trc */
    DRAM_REG[18] = 0x1f1f0000;  /* 0_0011111 dll_dqs_delay_1 0_0011111 dll_dqs_delay_0 000_00000 int_status 000_00000 int_mask */
    DRAM_REG[19] = 0x027f1a1a;  /* 00000010 dqs_out_shift_bypass 0_1111111 dqs_out_shift 00011010 dll_dqs_delay_bypass_1 00011010 dll_dqs_delay_bypass_0 */
    DRAM_REG[20] = 0x02051c22;  /* 00000010 trcd_int 00000101 tras_min 00011100 wr_dqs_shift_bypass 0_0100010 wr_dqs_shift */
    DRAM_REG[21] = 0x00000007;  /* 00000000000000_0000000000 out_of_range_length 00000111 trfc */
    DRAM_REG[22] = 0x00080008;  /* 00000_00000001000 ahb0_wrcnt 00000_00000001000 ahb0_rdcnt */
    DRAM_REG[23] = 0x00200020;  /* 00000_00000100000 ahb1_wrcnt 00000_00000100000 ahb1_rdcnt */
    DRAM_REG[24] = 0x00200020;  /* 00000_00000100000 ahb2_wrcnt 00000_00000100000 ahb2_rdcnt */
    DRAM_REG[25] = 0x00200020;  /* 00000_00000100000 ahb3_wrcnt 00000_00000100000 ahb3_rdcnt */
    DRAM_REG[26] = 0x000002e6;  /* 00000000000000000000_001011100110 tref */
    DRAM_REG[27] = 0x00000000;  /* 00000000000000000000000000000000 */
    DRAM_REG[28] = 0x00000000;  /* 00000000000000000000000000000000 */
    DRAM_REG[29] = 0x00000000;  /* 0000000000000000 lowpower_internal_cnt 0000000000000000 lowpower_external_cnt */
    DRAM_REG[30] = 0x00000000;  /* 0000000000000000 lowpower_refresh_hold 0000000000000000 lowpower_power_down_cnt */
    DRAM_REG[31] = 0x00c80000;  /* 0000000011001000 tdll 0000000000000000 lowpower_self_refresh_cnt */
    DRAM_REG[32] = 0x00081a3b;  /* 0000000000001000 txsnr 0001101000111011 tras_max */
    DRAM_REG[33] = 0x000000c8;  /* 0000000000000000 version 0000000011001000 txsr */
    DRAM_REG[34] = 0x00004b0d;  /* 00000000000000000100101100001101 tinit */
    DRAM_REG[35] = 0x00000000;  /* 0_0000000000000000000000000000000 out_of_range_addr */
    DRAM_REG[36] = 0x00000101;  /* 0000000_0 pwrup_srefresh_exit 0000000_0 enable_quick_srefresh 0000000_1 bus_share_enable 0000000_1 active_aging */
    DRAM_REG[37] = 0x00040001;  /* 00000000000001_0000000000 bus_share_timeout 0000000_1 tref_enable */
    DRAM_REG[38] = 0x00000000;  /* 000_0000000000000 emrs2_data_0 000_0000000000000 emrs1_data */
    DRAM_REG[39] = 0x00000000;  /* 000_0000000000000 emrs2_data_2 000_0000000000000 emrs2_data_1 */
    DRAM_REG[40] = 0x00010000;  /* 0000000000000001 tpdex 000_0000000000000 emrs2_data_3 */
    DRAM_REG[ 8] = 0x01000000;  /* 0000000_1 tras_lockout 0000000_1 start 0000000_0 srefresh 0000000_0 sdr_mode */
}

//------------------------------------------------------------------------------
//
//  Function: hw_dram_Init_ddr_mt46v32m16_6t_133MHz_optimized
//
//
//------------------------------------------------------------------------------

void hw_dram_Init_ddr_mt46v32m16_6t_133MHz_optimized(void)
{
    volatile unsigned int* DRAM_REG = (volatile unsigned int*) HW_DRAM_CTL00_ADDR;

    DRAM_REG[4] = 0x00000101;  /* 0000000_0 dll_bypass_mode 0000000_0 dlllockreg 0000000_1 concurrentap 0000000_1 bank_split_en */
    DRAM_REG[7] = 0x01000001;  /* 0000000_1 rw_same_en 0000000_0 reg_dimm_enable 0000000_0 rd2rd_turn 0000000_1 priority_en */
    DRAM_REG[11] = 0x00070204;  /* 00000_000 max_cs_reg 00000_111 command_age_count 00000_010 column_size 00000_010 caslat */
    DRAM_REG[12] = 0x02020000;  /* 00000_010 twr_int 00000_010 trrd 0000000000000_000 tcke */
    DRAM_REG[13] = 0x04040a01;  /* 0000_0100 caslat_lin_gate 0000_0100 caslat_lin 0000_1010 aprebit 00000_001 twtr */
    DRAM_REG[15] = 0x02040000;  /* 0000_0010 trp 0000_0100 tdal 0000_0000 port_busy 0000_0000 max_row_reg */
    DRAM_REG[17] = 0x19000f08;  /* 00111001 dll_start_point 00000000 dll_lock 00001111 dll_increment 000_01000 trc */
    DRAM_REG[18] = 0x0d0d0000;  /* 0_0011111 dll_dqs_delay_1 0_0011111 dll_dqs_delay_0 000_00000 int_status 000_00000 int_mask */
    DRAM_REG[19] = 0x02021313;  /* 00000010 dqs_out_shift_bypass 0_1111111 dqs_out_shift 00010011 dll_dqs_delay_bypass_1 00010011 dll_dqs_delay_bypass_0 */
    DRAM_REG[20] = 0x02061521;  /* 00000010 trcd_int 00000110 tras_min 00010101 wr_dqs_shift_bypass 0_0100011 wr_dqs_shift */
    DRAM_REG[21] = 0x0000000a;  /* 00000000000000_0000000000 out_of_range_length 00001010 trfc */
    DRAM_REG[26] = 0x000003f7;  /* 00000000000000000000_001111110111 tref */
    DRAM_REG[32] = 0x000a23cd;  /* 0000000000001010 txsnr 0010001111001101 tras_max */
    DRAM_REG[33] = 0x000000c8;  /* 0000000000000000 version 0000000011001000 txsr */
    DRAM_REG[34] = 0x00006665;  /* 00000000000000000110011001100101 tinit */
    DRAM_REG[40] = 0x00010000;  /* 0000000000000001 tpdex 000_0000000000000 emrs2_data_3 */
}
//-----------------------------------------------------------------------------
//
//  Function:  hw_dram_Init_mobile_ddr_mt46h32m16lf_5_regs_3_133MHz
//
//  This function is used to initial memory controller for mDDR,133MHz freq.
//
//  Parameters:
//          
//  Returns:
//          None.
//
//-----------------------------------------------------------------------------

void hw_dram_Init_mobile_ddr_mt46h32m16lf_5_regs_3_133MHz(void)
{
    volatile unsigned int * DRAM_REG = (volatile unsigned int *) HW_DRAM_CTL00_ADDR;

    DRAM_REG[ 0] = 0x01010001;  /* 0000000_1 ahb0_w_priority 0000000_1 ahb0_r_priority 0000000_0 ahb0_fifo_type_reg 0000000_1 addr_cmp_en */
    DRAM_REG[ 1] = 0x00010100;  /* 0000000_0 ahb2_fifo_type_reg 0000000_1 ahb1_w_priority 0000000_1 ahb1_r_priority 0000000_0 ahb1_fifo_type_reg */
    DRAM_REG[ 2] = 0x01000101;  /* 0000000_1 ahb3_r_priority 0000000_0 ahb3_fifo_type_reg 0000000_1 ahb2_w_priority 0000000_1 ahb2_r_priority */
    DRAM_REG[ 3] = 0x00000001;  /* 0000000_0 auto_refresh_mode 0000000_0 arefresh 0000000_0 ap 0000000_1 ahb3_w_priority */
    DRAM_REG[ 4] = 0x00000101;  /* 0000000_0 dll_bypass_mode 0000000_0 dlllockreg 0000000_1 concurrentap 0000000_1 bank_split_en */
    DRAM_REG[ 5] = 0x00000001;  /* 0000000_0 intrptreada 0000000_0 intrptapburst 0000000_0 fast_write 0000000_1 en_lowpower_mode */
    DRAM_REG[ 6] = 0x00010000;  /* 0000000_0 power_down 0000000_1 placement_en 0000000_0 no_cmd_init 0000000_0 intrptwritea */
    DRAM_REG[ 7] = 0x01000001;  /* 0000000_1 rw_same_en 0000000_0 reg_dimm_enable 0000000_0 rd2rd_turn 0000000_1 priority_en */
    DRAM_REG[ 8] = 0x01000000;  /* 0000000_1 tras_lockout 0000000_0 start 0000000_0 srefresh 0000000_0 sdr_mode */
    DRAM_REG[ 9] = 0x00000001;  /* 000000_00 out_of_range_type 000000_00 out_of_range_source_id 0000000_0 write_modereg 0000000_1 writeinterp */
    DRAM_REG[10] = 0x07000200;  /* 00000_111 age_count 00000_000 addr_pins 000000_10 temrs 000000_00 q_fullness */
    DRAM_REG[11] = 0x00070203;  /* 00000_000 max_cs_reg 00000_111 command_age_count 00000_010 column_size 00000_011 caslat */
    DRAM_REG[12] = 0x02020001;  /* 00000_010 twr_int 00000_010 trrd 0000000000000_001 tcke */
    DRAM_REG[13] = 0x06060a03;  /* 0000_0110 caslat_lin_gate 0000_0110 caslat_lin 0000_1010 aprebit 00000_011 twtr */
    DRAM_REG[14] = 0x00000201;  /* 0000_0000 max_col_reg 0000_0000 lowpower_refresh_enable 0000_0010 initaref 0000_1111 cs_map */
    DRAM_REG[15] = 0x02040000;  /* 0000_0010 trp 0000_0100 tdal 0000_0000 port_busy 0000_0000 max_row_reg */
    DRAM_REG[16] = 0x02000000;  /* 000_00010 tmrd 000_00000 lowpower_control 000_00000 lowpower_auto_enable 0000_0000 int_ack */
    DRAM_REG[17] = 0x39000f08;  /* 00111001 dll_start_point 00000000 dll_lock 00001111 dll_increment 000_01000 trc */
    DRAM_REG[18] = 0x20200000;  /* 0_0011111 dll_dqs_delay_1 0_0011111 dll_dqs_delay_0 000_00000 int_status 000_00000 int_mask */
    DRAM_REG[19] = 0x02021313;  /* 00000010 dqs_out_shift_bypass 0_1111111 dqs_out_shift 00010011 dll_dqs_delay_bypass_1 00010011 dll_dqs_delay_bypass_0 */
    DRAM_REG[20] = 0x0206151c;  /* 00000010 trcd_int 00000110 tras_min 00010101 wr_dqs_shift_bypass 0_0100011 wr_dqs_shift */
    DRAM_REG[21] = 0x0000000d;  /* 00000000000000_0000000000 out_of_range_length 00001101 trfc */
    DRAM_REG[22] = 0x00080008;  /* 00000_00000001000 ahb0_wrcnt 00000_00000001000 ahb0_rdcnt */
    DRAM_REG[23] = 0x00200020;  /* 00000_00000100000 ahb1_wrcnt 00000_00000100000 ahb1_rdcnt */
    DRAM_REG[24] = 0x00200020;  /* 00000_00000100000 ahb2_wrcnt 00000_00000100000 ahb2_rdcnt */
    DRAM_REG[25] = 0x00200020;  /* 00000_00000100000 ahb3_wrcnt 00000_00000100000 ahb3_rdcnt */
    DRAM_REG[26] = 0x000003f7;  /* 00000000000000000000_001111110111 tref */
    DRAM_REG[27] = 0x00000000;  /* 00000000000000000000000000000000 */
    DRAM_REG[28] = 0x00000000;  /* 00000000000000000000000000000000 */
    DRAM_REG[29] = 0x00000000;  /* 0000000000000000 lowpower_internal_cnt 0000000000000000 lowpower_external_cnt */
    DRAM_REG[30] = 0x00000000;  /* 0000000000000000 lowpower_refresh_hold 0000000000000000 lowpower_power_down_cnt */
    DRAM_REG[31] = 0x00000000;  /* 0000000000000000 tdll 0000000000000000 lowpower_self_refresh_cnt */
    DRAM_REG[32] = 0x001023cd;  /* 0000000000010000 txsnr 0010001111001101 tras_max */
    DRAM_REG[33] = 0x00000012;  /* 0000000000000000 version 0000000000010010 txsr */
    DRAM_REG[34] = 0x00006665;  /* 00000000000000000110011001100101 tinit */
    DRAM_REG[35] = 0x00000000;  /* 0_0000000000000000000000000000000 out_of_range_addr */
    DRAM_REG[36] = 0x00000101;  /* 0000000_0 pwrup_srefresh_exit 0000000_0 enable_quick_srefresh 0000000_1 bus_share_enable 0000000_1 active_aging */
    DRAM_REG[37] = 0x00040001;  /* 00000000000001_0000000000 bus_share_timeout 0000000_1 tref_enable */
    //0x00400000,  /* 000_000000000000 emrs2_data_0 000_0000000000000 emrs1_data */
    DRAM_REG[38] = 0x00400162;  /* 000_0000000000000 emrs2_data_0 000_0000000000000 emrs1_data */
    DRAM_REG[39] = 0x00400040;  /* 000_0000000000000 emrs2_data_2 000_0000000000000 emrs2_data_1 */
    DRAM_REG[40] = 0x00020040;  /* 0000000000000010 tpdex 000_0000000000000 emrs2_data_3 */
}

//-----------------------------------------------------------------------------
//
//  Function:  hw_dram_Init_mobile_ddr_mt46h32m16lf_5_regs_3_160MHz
//
//  This function is used to initial memory controller for mDDR,160MHz freq.
//
//  Parameters:
//          
//  Returns:
//          None.
//
//-----------------------------------------------------------------------------

void hw_dram_Init_mobile_ddr_mt46h32m16lf_5_regs_3_160MHz(void)
{
    volatile unsigned int * DRAM_REG = (volatile unsigned int *) HW_DRAM_CTL00_ADDR;

    DRAM_REG[ 0] = 0x01010001;  /* 0000000_1 ahb0_w_priority 0000000_1 ahb0_r_priority 0000000_0 ahb0_fifo_type_reg 0000000_1 addr_cmp_en */
    DRAM_REG[ 1] = 0x00010100;  /* 0000000_0 ahb2_fifo_type_reg 0000000_1 ahb1_w_priority 0000000_1 ahb1_r_priority 0000000_0 ahb1_fifo_type_reg */
    DRAM_REG[ 2] = 0x01000101;  /* 0000000_1 ahb3_r_priority 0000000_0 ahb3_fifo_type_reg 0000000_1 ahb2_w_priority 0000000_1 ahb2_r_priority */
    DRAM_REG[ 3] = 0x00000001;  /* 0000000_0 auto_refresh_mode 0000000_0 arefresh 0000000_0 ap 0000000_1 ahb3_w_priority */
    DRAM_REG[ 4] = 0x00000101;  /* 0000000_0 dll_bypass_mode 0000000_0 dlllockreg 0000000_1 concurrentap 0000000_1 bank_split_en */
    DRAM_REG[ 5] = 0x00000001;  /* 0000000_0 intrptreada 0000000_0 intrptapburst 0000000_0 fast_write 0000000_1 en_lowpower_mode */
    DRAM_REG[ 6] = 0x00010000;  /* 0000000_0 power_down 0000000_1 placement_en 0000000_0 no_cmd_init 0000000_0 intrptwritea */
    DRAM_REG[ 7] = 0x01000101;  /* 0000000_1 rw_same_en 0000000_0 reg_dimm_enable 0000000_1 rd2rd_turn 0000000_1 priority_en */
    DRAM_REG[ 8] = 0x01000000;  /* 0000000_1 tras_lockout 0000000_0 start 0000000_0 srefresh 0000000_0 sdr_mode */
    DRAM_REG[ 9] = 0x00000001;  /* 000000_00 out_of_range_type 000000_00 out_of_range_source_id 0000000_0 write_modereg 0000000_1 writeinterp */
    DRAM_REG[10] = 0x07000200;  /* 00000_111 age_count 00000_000 addr_pins 000000_10 temrs 000000_00 q_fullness */
    DRAM_REG[11] = 0x00070203;  /* 00000_000 max_cs_reg 00000_111 command_age_count 00000_010 column_size 00000_011 caslat */
    DRAM_REG[12] = 0x02020001;  /* 00000_010 twr_int 00000_010 trrd 0000000000000_001 tcke */
    DRAM_REG[13] = 0x06060a03;  /* 0000_0110 caslat_lin_gate 0000_0110 caslat_lin 0000_1010 aprebit 00000_011 twtr */
    DRAM_REG[14] = 0x00000201;  /* 0000_0000 max_col_reg 0000_0000 lowpower_refresh_enable 0000_0010 initaref 0000_1111 cs_map */
    DRAM_REG[15] = 0x03050000;  /* 0000_0011 trp 0000_0101 tdal 0000_0000 port_busy 0000_0000 max_row_reg */
    DRAM_REG[16] = 0x02000000;  /* 000_00010 tmrd 000_00000 lowpower_control 000_00000 lowpower_auto_enable 0000_0000 int_ack */
    DRAM_REG[17] = 0x2d000d09;  /* 00101101 dll_start_point 00000000 dll_lock 00001101 dll_increment 000_01001 trc */
    DRAM_REG[18] = 0x20200000;  /* 0_0011111 dll_dqs_delay_1 0_0011111 dll_dqs_delay_0 000_00000 int_status 000_00000 int_mask */
    DRAM_REG[19] = 0x02020f0f;  /* 00000010 dqs_out_shift_bypass 0_1111111 dqs_out_shift 00001111 dll_dqs_delay_bypass_1 00001111 dll_dqs_delay_bypass_0 */
    DRAM_REG[20] = 0x0307121c;  /* 00000011 trcd_int 00000111 tras_min 00010010 wr_dqs_shift_bypass 0_0100100 wr_dqs_shift */
    DRAM_REG[21] = 0x00000010;  /* 00000000000000_0000000000 out_of_range_length 00010000 trfc */
    DRAM_REG[22] = 0x00080008;  /* 00000_00000001000 ahb0_wrcnt 00000_00000001000 ahb0_rdcnt */
    DRAM_REG[23] = 0x00200020;  /* 00000_00000100000 ahb1_wrcnt 00000_00000100000 ahb1_rdcnt */
    DRAM_REG[24] = 0x00200020;  /* 00000_00000100000 ahb2_wrcnt 00000_00000100000 ahb2_rdcnt */
    DRAM_REG[25] = 0x00200020;  /* 00000_00000100000 ahb3_wrcnt 00000_00000100000 ahb3_rdcnt */
    DRAM_REG[26] = 0x000004da;  /* 00000000000000000000_010011011010 tref */
    DRAM_REG[27] = 0x00000000;  /* 00000000000000000000000000000000 */
    DRAM_REG[28] = 0x00000000;  /* 00000000000000000000000000000000 */
    DRAM_REG[29] = 0x00000000;  /* 0000000000000000 lowpower_internal_cnt 0000000000000000 lowpower_external_cnt */
    DRAM_REG[30] = 0x00000000;  /* 0000000000000000 lowpower_refresh_hold 0000000000000000 lowpower_power_down_cnt */
    DRAM_REG[31] = 0x00000000;  /* 0000000000000000 tdll 0000000000000000 lowpower_self_refresh_cnt */
    DRAM_REG[32] = 0x00142bb6;  /* 0000000000010100 txsnr 0010101110110110 tras_max */
    DRAM_REG[33] = 0x00000016;  /* 0000000000000000 version 0000000000010110 txsr */
    DRAM_REG[34] = 0x00007d00;  /* 00000000000000000111110100000000 tinit */
    DRAM_REG[35] = 0x00000000;  /* 0_0000000000000000000000000000000 out_of_range_addr */
    DRAM_REG[36] = 0x00000101;  /* 0000000_0 pwrup_srefresh_exit 0000000_0 enable_quick_srefresh 0000000_1 bus_share_enable 0000000_1 active_aging */
    DRAM_REG[37] = 0x00040001;  /* 00000000000001_0000000000 bus_share_timeout 0000000_1 tref_enable */
    //0x00400000,  /* 000_0000000000000 emrs2_data_0 000_0000000000000 emrs1_data */
    DRAM_REG[38] = 0x00400162;  /* 000_0000000000000 emrs2_data_0 000_0000000000000 emrs1_data */
    DRAM_REG[39] = 0x00400040;  /* 000_0000000000000 emrs2_data_2 000_0000000000000 emrs2_data_1 */
    DRAM_REG[40] = 0x00020040;  /* 0000000000000010 tpdex 000_0000000000000 emrs2_data_3 */
}

/////////////////////////////////////////////////////////////////////////////////
//! See hw_digctl.h for details
/////////////////////////////////////////////////////////////////////////////////
unsigned int hw_digctl_GetCurrentTime(void)
{
    return HW_DIGCTL_MICROSECONDS_RD();
}

int ddi_emi_ExitStaticSelfRefreshMode(void)
{
    unsigned int StartTime;

    hw_emi_EnterMemorySelfRefreshMode(FALSE);

    StartTime = hw_digctl_GetCurrentTime();

    while(hw_emi_IsControllerHalted())
    {
        if(hw_digctl_CheckTimeOut(StartTime, 1000))
        {
            return ERROR_DDI_EMI_EXIT_SELF_REFRESH_TIMEOUT;
        }
    }
    return SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////
//! See hw_digctl.h for details
/////////////////////////////////////////////////////////////////////////////////
bool hw_digctl_CheckTimeOut(unsigned int StartTime, unsigned int TimeOut)
{
    unsigned int    CurTime, EndTime;
    bool        bTimeOut;

    CurTime = HW_DIGCTL_MICROSECONDS_RD();
    EndTime = StartTime + TimeOut;

    if ( StartTime <= EndTime)
    {
        bTimeOut = ((CurTime >= StartTime) && (CurTime < EndTime))? FALSE : TRUE;
    }
    else
    {
        bTimeOut = ((CurTime >= StartTime) || (CurTime < EndTime))? FALSE : TRUE;
    }

    return bTimeOut;
}
//------------------------------------------------------------------------------
//
//  Function: hw_emi_EnterMemorySelfRefreshMode
//
//
//------------------------------------------------------------------------------

void hw_emi_EnterMemorySelfRefreshMode(bool bOnOff)
{
    if(bOnOff)
        HW_DRAM_CTL16_SET(1<<17);
    else
        HW_DRAM_CTL16_CLR(1<<17);
}
//------------------------------------------------------------------------------
//
//  Function: hw_emi_SetMemorySelfRefeshAutoFlag
//
//
//------------------------------------------------------------------------------

void hw_emi_SetMemorySelfRefeshAutoFlag(bool bEnable)
{
    if(bEnable)
    {    
        // if count is zero, we'll instantly go into self-refresh mode.
        if(HW_DRAM_CTL29.B.LOWPOWER_EXTERNAL_CNT==0)
        {
            HW_DRAM_CTL29.B.LOWPOWER_EXTERNAL_CNT = 64;
        }
            
        HW_DRAM_CTL16_SET(1<<9);
    }
    else
        HW_DRAM_CTL16_CLR(1<<9);
}
//------------------------------------------------------------------------------
//
//  Function: InitDebugSerial
//
//
//------------------------------------------------------------------------------

void InitDebugSerial()
{
    unsigned int UartReadDummy;

    // Make sure all debug UART interrupts are off
    HW_UARTDBGIMSC_WR(0x0);

    HW_PINCTRL_CTRL_CLR(BM_PINCTRL_CTRL_SFTRST | BM_PINCTRL_CTRL_CLKGATE);

    // Configure the GPIO UART pins.
    HW_PINCTRL_MUXSEL3_SET(0xF << 20);     // Switch both pins to GPIO
    HW_PINCTRL_MUXSEL3_CLR(1 << 22);       // DBG-TX (bank 1 pin 22) muxmode=10
    HW_PINCTRL_MUXSEL3_CLR(1 << 20);       // DBG-RX (bank 2 pin 20) muxmode=10

    // Set the Baud Rate
    //
    HW_UARTDBGIBRD_WR((HW_UARTDBGIBRD_RD() & BM_UARTDBGIBRD_UNAVAILABLE) | GET_UARTDBG_BAUD_DIVINT(DEBUG_BAUD));
    HW_UARTDBGFBRD_WR((HW_UARTDBGFBRD_RD() & BM_UARTDBGFBRD_UNAVAILABLE) | GET_UARTDBG_BAUD_DIVFRAC(DEBUG_BAUD));

    //Setting UART properties to 8N1
    //
    HW_UARTDBGLCR_H_WR(BF_UARTDBGLCR_H_SPS(0)   |
                       BF_UARTDBGLCR_H_WLEN(3)  |
                       BF_UARTDBGLCR_H_FEN(1)   |
                       BF_UARTDBGLCR_H_STP2(0)  |
                       BF_UARTDBGLCR_H_EPS(0)   |
                       BF_UARTDBGLCR_H_PEN(0)   |
                       BF_UARTDBGLCR_H_BRK(0));

    //Clear Tx/Rx FIFO
    //
    for(; (HW_UARTDBGFR_RD() & BM_UARTDBGFR_RXFE) == 0; )
    {
        UartReadDummy = HW_UARTDBGDR_RD();
    }

    // Clear Receive Status
    HW_UARTDBGRSR_ECR_WR((HW_UARTDBGRSR_ECR_RD() & ~BM_UARTDBGRSR_ECR_EC) | \
                         BF_UARTDBGRSR_ECR_EC(0xF));

    HW_UARTDBGIFLS_WR(0x9);

    // Enable the UART.
    //
    HW_UARTDBGCR_WR( BM_UARTDBGCR_UARTEN | BM_UARTDBGCR_RXE | BM_UARTDBGCR_TXE );


}
//------------------------------------------------------------------------------
//
//  Function: OEMWriteDebugByte
//
//  Transmits a character out the debug serial port.
//
//------------------------------------------------------------------------------
void OEMWriteDebugByte(unsigned char ch)
{
    unsigned int loop = 0;
    while ( (HW_UARTDBGFR.B.TXFF) && (loop < 0x7FFF))
    {
        loop++;
    }

    // Write a character byte to the FIFO.
    //
    if(!(HW_UARTDBGFR.B.TXFF))
        BW_UARTDBGDR_DATA(ch);
}

//------------------------------------------------------------------------------
//
//  Function: InitPower
//
//  Initial power state.
//
//------------------------------------------------------------------------------
void InitPower()
{
    unsigned int i = 0,j = 0;
    unsigned int BattVoltage = 0;
    unsigned int StartTime = 0; 
    
    s_bBattery = 0;
    s_bUsb = 0;
    s_b5V = 0;
    
    // Restore VDDA 1.800 volt
    HW_POWER_VDDACTRL_WR((HW_POWER_VDDACTRL_RD() & ~BM_POWER_VDDACTRL_TRG)|12);

    // need to wait more than 10 microsecond before the DC_OK is valid
    OALStall(15);

    // wait for DC_OK
    while (!(HW_POWER_STS_RD() & BM_POWER_STS_DC_OK));

    HW_POWER_VDDACTRL_WR((HW_POWER_VDDACTRL_RD() & ~BM_POWER_VDDACTRL_BO_OFFSET) |
                        (4 << BP_POWER_VDDACTRL_BO_OFFSET));

    // Restore the VDDIO 3.440 volt
    HW_POWER_VDDIOCTRL_WR((HW_POWER_VDDIOCTRL_RD() & ~BM_POWER_VDDIOCTRL_TRG)|20);

    // need to wait more than 10 microsecond before the DC_OK is valid
    OALStall(15);

    // wait for DC_OK
    while (!(HW_POWER_STS_RD() & BM_POWER_STS_DC_OK));

    HW_POWER_VDDIOCTRL_WR((HW_POWER_VDDIOCTRL_RD() & ~BM_POWER_VDDIOCTRL_BO_OFFSET) |\
                          (4 << BP_POWER_VDDIOCTRL_BO_OFFSET));


    //Detect USB cable status
    if ((HW_POWER_5VCTRL_RD() & BM_POWER_5VCTRL_PWRUP_VBUS_CMPS) == 0)
        HW_POWER_5VCTRL_SET(BM_POWER_5VCTRL_PWRUP_VBUS_CMPS);
    
    if ((HW_USBPHY_CTRL_RD()& BM_USBPHY_CTRL_CLKGATE) == 1)
        HW_USBPHY_CTRL_CLR(BM_USBPHY_CTRL_CLKGATE);
    if ((HW_USBPHY_CTRL_RD()& BM_USBPHY_CTRL_SFTRST) == 1)
        HW_USBPHY_CTRL_CLR(BM_USBPHY_CTRL_SFTRST);

    HW_USBPHY_CTRL_SET(BM_USBPHY_CTRL_ENDEVPLUGINDETECT);

    //Detect Battery voltage
    HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_CLKGATE);    //gate
    HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_SFTRST);    //reset
    HW_LRADC_CONVERSION_SET(0x20000);  //scale factor =2 
    HW_LRADC_CONVERSION_SET(0x100000);  //automatic
    HW_LRADC_CH7_CLR(0xFFFFFFFF);
    
    HW_LRADC_CTRL1_SET(BM_LRADC_CTRL1_LRADC7_IRQ_EN);  //enable irq

    HW_LRADC_CTRL0_SET(0x80);

    while((HW_LRADC_CTRL1_RD() & 0x80) == 0)
        OEMWriteDebugByte('.');
    
    //PrintHex(HW_USBPHY_STATUS_RD());
    if((HW_USBPHY_STATUS_RD() & BM_USBPHY_STATUS_DEVPLUGIN_STATUS)) 
        s_bUsb = TRUE;

    HW_USBPHY_CTRL_CLR(BM_USBPHY_CTRL_ENDEVPLUGINDETECT);

    s_b5V = HW_POWER_STS.B.VBUSVALID;

    BattVoltage = HW_POWER_BATTMONITOR.B.BATT_VAL * BATT_VOLTAGE_8_MV;

    //Print BATTERY voltage and power status
    OEMWriteDebugByte('\r');
    OEMWriteDebugByte('\n');
    OEMWriteDebugByte('B');
    OEMWriteDebugByte('A');
    OEMWriteDebugByte('T');
    OEMWriteDebugByte('T');
    OEMWriteDebugByte(':');
    //PrintHex(BattVoltage);
    PrintBatteryVoltage(BattVoltage);
    OEMWriteDebugByte('\r');
    OEMWriteDebugByte('\n');

    if(BattVoltage > 3400)
        s_bBattery = TRUE;

    if(s_bBattery == TRUE)
    {
        OEMWriteDebugByte('B');
        OEMWriteDebugByte('\r');
        OEMWriteDebugByte('\n');
    }

    if(s_bUsb == TRUE)
    {
        OEMWriteDebugByte('U');
        OEMWriteDebugByte('\r');
        OEMWriteDebugByte('\n');
    }

    if(s_b5V == TRUE)
    {
        OEMWriteDebugByte('5');
        OEMWriteDebugByte('\r');
        OEMWriteDebugByte('\n');        
    }

    //HW_POWER_VDDDCTRL_CLR(0x200000);
    //HW_POWER_VDDACTRL_CLR(0x20000);
    
    //printf(HW_POWER_5VCTRL_RD());
    //printf(HW_POWER_VDDDCTRL_RD());
    //printf(HW_POWER_VDDACTRL_RD());
    //printf(HW_POWER_VDDIOCTRL_RD());

    if(s_bUsb == TRUE)
    {
        HW_POWER_5VCTRL_CLR(0x37000);
        HW_POWER_5VCTRL_SET(0x8000);
    }
    //HW_POWER_5VCTRL_CLR(0x80);
    if(s_bBattery == TRUE)
    {
        PowerExecute5VoltsToBatteryHandoff();
        //Enable double FETs.
        HW_POWER_MINPWR_CLR(BM_POWER_MINPWR_HALF_FETS);
        HW_POWER_MINPWR_SET(BM_POWER_MINPWR_DOUBLE_FETS);
        //set linger step below 25mV
        HW_POWER_VDDIOCTRL_SET(0x2000);
        HW_POWER_VDDACTRL_SET(0x2000);
        HW_POWER_VDDDCTRL_SET(0x20000);
        HW_POWER_VDDDCTRL_CLR(BM_POWER_VDDDCTRL_DISABLE_FET);
        //disable 4P2
        HW_POWER_5VCTRL_SET(BM_POWER_5VCTRL_PWD_CHARGE_4P2);
        //enable DCDC
        HW_POWER_5VCTRL_SET(BM_POWER_5VCTRL_ENABLE_DCDC);
        //set DCDC_4P2 or DCDC_BATT select which is high
        //HW_POWER_DCDC4P2_SET(0x20000000);   
        //set the CHARGE_4P2_ILIMIT to 100mA
        //HW_POWER_5VCTRL_SET(0x8000);
        //HW_POWER_5VCTRL_SET(0x4);
        //HW_POWER_5VCTRL_CLR(0x3F000);//set limit current to 110mA
        //HW_POWER_5VCTRL_SET(0x3F000);//set limit current to 110mA
        TurnOnPLLClock();
    }

    //Direct boot from USB,current limited to 100mA
#ifdef BSP_USB_DIRECT_BOOT    
    if((s_bUsb == TRUE) && (s_bBattery == FALSE) && (s_b5V == TRUE))
    {
        //set linger step below 25mV

        HW_POWER_VDDIOCTRL_SET(0x2000);
        HW_POWER_VDDACTRL_SET(0x2000);
        HW_POWER_VDDDCTRL_SET(0x20000);

        PowerDisableAutoHardwarePowerdown(TRUE);

        //HW_POWER_DCDC4P2_CLR(0x1F);//set to 85% of BATT
        //HW_POWER_5VCTRL_SET(0x4000000);//HEADROOM_ADJ = 0x4

        //HW_POWER_DCDC4P2_CLR(0x70000);//TRG = 4.2V
        //HW_POWER_DCDC4P2_SET(0xE0000000);//100mV drop before stealing charging current
        
        HW_POWER_CHARGE_SET(BM_POWER_CHARGE_ENABLE_LOAD);//enable load
        HW_POWER_5VCTRL_CLR(BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT);//clear limit current
        HW_POWER_5VCTRL_SET(0x1000);//set limit current to 10mA
        HW_POWER_5VCTRL_SET(BM_POWER_5VCTRL_PWD_CHARGE_4P2);
        HW_POWER_5VCTRL_CLR(BM_POWER_5VCTRL_PWD_CHARGE_4P2);//clear PWD bit
        //HW_POWER_DCDC4P2_SET(0x400000);//enable DCDC4P2
        //HW_POWER_CTRL_CLR(0x1000000);//clear irq
        //HW_POWER_CTRL_SET(0x800000);//enable irq
        
        HW_POWER_DCDC4P2_SET(BM_POWER_DCDC4P2_ENABLE_4P2);//enable 4P2

 /*
        while(HW_POWER_STS.B.VBUSVALID && HW_POWER_CTRL.B.DCDC4P2_BO_IRQ)
        {
            BF_CLR(POWER_CTRL, DCDC4P2_BO_IRQ);
            OEMWriteDebugByte('=');
        }
*/
        //disable FET
        HW_POWER_VDDDCTRL_CLR(BM_POWER_VDDDCTRL_ENABLE_LINREG);
        HW_POWER_VDDDCTRL_CLR(BM_POWER_VDDDCTRL_DISABLE_FET);
        HW_POWER_VDDACTRL_CLR(BM_POWER_VDDACTRL_DISABLE_FET);
        HW_POWER_VDDIOCTRL_CLR(BM_POWER_VDDIOCTRL_DISABLE_FET);

        //BO offset
        HW_POWER_VDDIOCTRL_CLR(0xF00);
        HW_POWER_VDDIOCTRL_SET(0x400);
        HW_POWER_VDDDCTRL_CLR(0xF00);
        HW_POWER_VDDDCTRL_SET(0x400);
        HW_POWER_VDDACTRL_CLR(0xF00);
        HW_POWER_VDDACTRL_SET(0x400);
        

        //charge 4P2 capacity to 4.2V
        i = 1;
        while(i <= 0x3F)
        {
            HW_POWER_5VCTRL_CLR(BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT);
            HW_POWER_5VCTRL_SET(i << 12);//set limit current to 10mA
            i++;
            OEMWriteDebugByte('$');
            StartTime = HW_DIGCTL_MICROSECONDS_RD();
            while(HW_DIGCTL_MICROSECONDS_RD() - StartTime < 100);
        }

        HW_POWER_5VCTRL_CLR(0x20);//clear DCDCXFER
        HW_POWER_5VCTRL_CLR(BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT);//set limit current to 100mA
        HW_POWER_5VCTRL_SET(0x8000);//set limit current to 100mA
        BF_SET(POWER_DCDC4P2, ENABLE_DCDC);      //enable DCDC 4P2 capability
        // enables DCDC during 5V connnection (for both battery or 4p2 powered DCDC)
        BF_SET(POWER_5VCTRL, ENABLE_DCDC);       // Enable the DCDC.

        //HW_POWER_5VCTRL_SET(0x1);//enable DCDC
        StartTime = HW_DIGCTL_MICROSECONDS_RD();
        while(HW_DIGCTL_MICROSECONDS_RD() - StartTime < 1000);
        HW_POWER_5VCTRL_SET(BM_POWER_5VCTRL_ENABLE_LINREG_ILIMIT);
    }
#else
    //Charge battery until it reach 3.7V to boot up system
    if((s_bUsb == TRUE) && (s_bBattery == FALSE) && (s_b5V == TRUE))
    {
        PowerSetCharger(0x4);

        OEMWriteDebugByte('c');
        OEMWriteDebugByte('h');
        OEMWriteDebugByte('a');
        OEMWriteDebugByte('r');
        OEMWriteDebugByte('g');
        OEMWriteDebugByte('i');
        OEMWriteDebugByte('n');
        OEMWriteDebugByte('g');
        OEMWriteDebugByte('.');
        OEMWriteDebugByte('.');
        OEMWriteDebugByte('.');
        OEMWriteDebugByte(' ');
        OEMWriteDebugByte('(');
        OEMWriteDebugByte('3');
        OEMWriteDebugByte('.');
        OEMWriteDebugByte('4');
        OEMWriteDebugByte('V');
        OEMWriteDebugByte(')');  
        OEMWriteDebugByte('\r');
        OEMWriteDebugByte('\n');
        i = 0;
        while(i < 2)
        {
            while(BattVoltage < 3400)
            {
                PowerStopCharger();
                StartTime = HW_DIGCTL_MICROSECONDS_RD();
                while(HW_DIGCTL_MICROSECONDS_RD() - StartTime < 500000);
        
                HW_LRADC_CTRL0_SET(0x80);
                
                while((HW_LRADC_CTRL1_RD() & 0x80) == 0);
                BattVoltage = HW_POWER_BATTMONITOR.B.BATT_VAL * BATT_VOLTAGE_8_MV;

                OEMWriteDebugByte('\r');
                OEMWriteDebugByte('B');
                OEMWriteDebugByte('A');
                OEMWriteDebugByte('T');
                OEMWriteDebugByte('T');
                OEMWriteDebugByte(':');
                PrintBatteryVoltage(BattVoltage);
                OEMWriteDebugByte(' ');
                OEMWriteDebugByte(' ');
                OEMWriteDebugByte(' ');

                HW_LRADC_CTRL0_CLR(0x80);
                PowerSetCharger(0x4);
                //Print rotary cursor,wait for 10 seconds
                j = 0;
                while(j++ < 25)
                {
                    OEMWriteDebugByte('-');
                    OEMWriteDebugByte('\b');
                    OALStall(100000);
                    OEMWriteDebugByte('\\');
                    OEMWriteDebugByte('\b');
                    OALStall(100000);
                    OEMWriteDebugByte('|');
                    OEMWriteDebugByte('\b');
                    OALStall(100000);
                    OEMWriteDebugByte('/');
                    OEMWriteDebugByte('\b');
                    OALStall(100000);                               
                }
            }
            i++;
            BattVoltage = 0;
        }
        PowerExecute5VoltsToBatteryHandoff();
        //Enable double FETs.
        HW_POWER_MINPWR_CLR(BM_POWER_MINPWR_HALF_FETS);
        HW_POWER_MINPWR_SET(BM_POWER_MINPWR_DOUBLE_FETS);
        //set linger step below 25mV
        HW_POWER_VDDIOCTRL_SET(0x2000);
        HW_POWER_VDDACTRL_SET(0x2000);
        HW_POWER_VDDDCTRL_SET(0x20000);
        HW_POWER_VDDDCTRL_CLR(BM_POWER_VDDDCTRL_DISABLE_FET);
        //disable 4P2
        HW_POWER_5VCTRL_SET(BM_POWER_5VCTRL_PWD_CHARGE_4P2);
        //enable DCDC
        HW_POWER_5VCTRL_SET(BM_POWER_5VCTRL_ENABLE_DCDC);


        //set the CHARGE_4P2_ILIMIT to 100mA
        //HW_POWER_5VCTRL_SET(0x8000);
        //HW_POWER_5VCTRL_SET(0x4);
        //HW_POWER_5VCTRL_CLR(0x3F000);//set limit current to 110mA
        //HW_POWER_5VCTRL_SET(0x3F000);//set limit current to 110mA
        TurnOnPLLClock();   
    }
#endif        
}

//------------------------------------------------------------------------------
//
//  Function: PrintHex
//
//  Print function
//
//------------------------------------------------------------------------------
void PrintHex(unsigned int value)
{
    OEMWriteDebugByte('0');
    OEMWriteDebugByte('x');

    if((unsigned char)((value & 0xf0000000) >> 28) <= 9)
        OEMWriteDebugByte((unsigned char)((value & 0xf0000000) >> 28) + 0x30);
    else
        OEMWriteDebugByte((unsigned char)((value & 0xf0000000) >> 28) + 0x57);
    if((unsigned char)((value & 0xf000000) >> 24) <= 9)
        OEMWriteDebugByte((unsigned char)((value & 0xf000000) >> 24) + 0x30);
    else
        OEMWriteDebugByte((unsigned char)((value & 0xf000000) >> 24) + 0x57);
    if((unsigned char)((value & 0xf00000) >> 20) <= 9)
        OEMWriteDebugByte((unsigned char)((value & 0xf00000) >> 20) + 0x30);
    else
        OEMWriteDebugByte((unsigned char)((value & 0xf00000) >> 20) + 0x57);
    if((unsigned char)((value & 0xf0000) >> 16) <= 9)
        OEMWriteDebugByte((unsigned char)((value & 0xf0000) >> 16) + 0x30);
    else
        OEMWriteDebugByte((unsigned char)((value & 0xf0000) >> 16) + 0x57);
    if((unsigned char)((value & 0xf000) >> 12) <= 9)
        OEMWriteDebugByte((unsigned char)((value & 0xf000) >> 12) + 0x30);
    else
        OEMWriteDebugByte((unsigned char)((value & 0xf000) >> 12) + 0x57);
    if((unsigned char)((value & 0xf00) >> 8) <= 9)
        OEMWriteDebugByte((unsigned char)((value & 0xf00) >> 8) + 0x30);
    else
        OEMWriteDebugByte((unsigned char)((value & 0xf00) >> 8) + 0x57);
    if((unsigned char)((value & 0xf0) >> 4) <= 9)
        OEMWriteDebugByte((unsigned char)((value & 0xf0) >> 4) + 0x30);
    else
        OEMWriteDebugByte((unsigned char)((value & 0xf0) >> 4) + 0x57);    
    if((unsigned char)((value & 0xf)) <= 9)
        OEMWriteDebugByte((unsigned char)((value & 0xf)) + 0x30);
    else
        OEMWriteDebugByte((unsigned char)((value & 0xf)) + 0x57);    

    OEMWriteDebugByte('\r');
    OEMWriteDebugByte('\n');

}


//------------------------------------------------------------------------------
//
//  Function: PrintBatteryVoltage
//
//  Print function
//
//------------------------------------------------------------------------------
void PrintBatteryVoltage(unsigned int value)
{
    unsigned int num[3] = {0,0,0};
    unsigned int i = 0;
    
    num[0] = value/1000;
    num[1] = value%1000/100;
    num[2] = value%100/10;

    for(i = 0;i < 3;i++)
    {
        if(i == 1)
            OEMWriteDebugByte('.');
        switch(num[i])  
        {
            case 0:
                OEMWriteDebugByte('0');   
                break;
            case 1:
                OEMWriteDebugByte('1');   
                break;
            case 2:
                OEMWriteDebugByte('2');   
                break;   
            case 3:
                OEMWriteDebugByte('3');   
                break;
            case 4:
                OEMWriteDebugByte('4');   
                break;
            case 5:
                OEMWriteDebugByte('5');   
                break; 
            case 6:
                OEMWriteDebugByte('6');   
                break;
            case 7:
                OEMWriteDebugByte('7');   
                break;
            case 8:
                OEMWriteDebugByte('8');   
                break;   
            case 9:
                OEMWriteDebugByte('9');   
                break;
            default:   
                break;
        }
    }
    OEMWriteDebugByte('V');
}

//------------------------------------------------------------------------------
//
//  Function: PowerExecute5VoltsToBatteryHandoff
//
//------------------------------------------------------------------------------
void PowerExecute5VoltsToBatteryHandoff()
{
    // Disable DCDC from 4p2 and turn off the 4p2 rail
    PowerStop4p2();
    
    // Disable hardware power down when 5V is inserted or removed
    PowerDisableAutoHardwarePowerdown(TRUE);
    
    // Re-enable the battery brownout interrupt in case it was disabled.
    //PowerEnableBatteryBrownoutInterrupt(TRUE);
}

//-----------------------------------------------------------------------------
//
//  Function:  PowerStop4p2
//
//  This function stops 4p2 functionality, used in STMP378x
//
//  Parameters:
//          None.
//
//  Returns:
//      Returns the TRUE if successful,
//      otherwise returns FALSE.
//
//-----------------------------------------------------------------------------
void PowerStop4p2(void)
{
    BF_CLR(POWER_DCDC4P2, ENABLE_DCDC);       // Enable the DCDC.

    BF_CLR(POWER_DCDC4P2, ENABLE_4P2);      //enable DCDC 4P2 regulation circuitry

    BF_CLR(POWER_CHARGE,ENABLE_LOAD);



    //BF_SET(POWER_5VCTRL, PWRUP_VBUS_CMPS);

    BF_CLR(POWER_CTRL,ENIRQ_DCDC4P2_BO);

    BF_SET(POWER_5VCTRL, PWD_CHARGE_4P2);

}

//-----------------------------------------------------------------------------
//
//  Function:  PowerDisableAutoHardwarePowerdown
//
//  This function disables the autoo hardware power down funtion
//
//  Parameters:
//          None.
//
//  Returns:
//      Returns the TRUE if successful,
//      otherwise returns FALSE.
//
//-----------------------------------------------------------------------------
void PowerDisableAutoHardwarePowerdown(bool bDisable)
{
    if(bDisable)
    {
        BF_CLR(POWER_5VCTRL, PWDN_5VBRNOUT);
    }
    else
    {
        BF_SET(POWER_5VCTRL, PWDN_5VBRNOUT);
    }
}
//-----------------------------------------------------------------------------
//
//  Function:  PowerSetCharger
//
//  This function is used to set the charger
//
//  Parameters:
//          current
//              [in] The current value of charger
//
//  Returns:
//          None.
//
//-----------------------------------------------------------------------------
void PowerSetCharger(DWORD current)
{
    BF_CLRV(POWER_CHARGE, STOP_ILIMIT, 0xF);
    BF_SETV(POWER_CHARGE, STOP_ILIMIT, 0x3);//stop limit current = 30mA  

    //HW_POWER_CHARGE_CLR(0xF00);
    //HW_POWER_CHARGE_SET(0x400);  
    
    BF_CLRV(POWER_CHARGE, BATTCHRG_I, 0x3F);
    BF_SETV(POWER_CHARGE, BATTCHRG_I, current); 

    //HW_POWER_CHARGE_CLR(0x3F);
    //HW_POWER_CHARGE_SET(current);  

    BF_CLR(POWER_CHARGE, PWD_BATTCHRG);
    BF_CLR(POWER_5VCTRL, PWD_CHARGE_4P2); 

    //HW_POWER_CHARGE_CLR(0x10000);
    //HW_POWER_5VCTRL_CLR(0x100000);

}

//-----------------------------------------------------------------------------
//
//  Function:  PowerStopCharger
//
//  This function is used to stop the charger
//
//  Parameters:
//
//  Returns:
//          None.
//
//-----------------------------------------------------------------------------
void PowerStopCharger()
{
    BF_CLRV(POWER_CHARGE, STOP_ILIMIT, 0xF);
    BF_CLRV(POWER_CHARGE, BATTCHRG_I, 0x3F);
    BF_SET(POWER_CHARGE, PWD_BATTCHRG);
    BF_SET(POWER_5VCTRL, PWD_CHARGE_4P2);  

    //HW_POWER_CHARGE_CLR(0xF00);
    //HW_POWER_CHARGE_CLR(0x3F);
    //HW_POWER_CHARGE_SET(0x10000);
    //HW_POWER_5VCTRL_SET(0x100000);    
}

//-----------------------------------------------------------------------------
//
//  Function:  OALStall
//
//  This function is used to delay micro seconds.
//
//  Parameters:
//          microSec,time to delay.
//  Returns:
//          None.
//
//-----------------------------------------------------------------------------

void OALStall(unsigned int microSec)
{
    unsigned int expireTime, currentTime;

    currentTime = HW_DIGCTL_MICROSECONDS_RD();
    expireTime = currentTime + microSec;

    //
    // Check if we wrapped on the expireTime
    // and delay first part until wrap
    //
    if (expireTime < currentTime)
    {
        while (currentTime < HW_DIGCTL_MICROSECONDS_RD()) ;
    }
    while (HW_DIGCTL_MICROSECONDS_RD() <= expireTime) ;
}

//-----------------------------------------------------------------------------
//
//  Function:  TurnOnPLLClock
//
//  This function is used to turn on PLL,and switch P_CLK to PLL.
//
//  Parameters:
//          
//  Returns:
//          None.
//
//-----------------------------------------------------------------------------

void TurnOnPLLClock()
{
    unsigned int dwRegFrac;
    unsigned int x;

    // let CPU sink the xtal clock
    //
    HW_CLKCTRL_CLKSEQ_SET(BM_CLKCTRL_CLKSEQ_BYPASS_CPU);

    // Turn on PLL
    //
    HW_CLKCTRL_PLLCTRL0_SET(BM_CLKCTRL_PLLCTRL0_POWER);

    //Set VDDD to 1.55V
    HW_POWER_VDDDCTRL_WR(HW_POWER_VDDDCTRL_RD() | BM_POWER_VDDDCTRL_BO_OFFSET);
    HW_POWER_VDDDCTRL_WR((HW_POWER_VDDDCTRL_RD() & ~BM_POWER_VDDDCTRL_TRG) | 30);

    // need to wait more than 10 microsecond before the DC_OK is valid
    OALStall(15);

    // wait for DC_OK
    while (!(HW_POWER_STS_RD() & BM_POWER_STS_DC_OK));

    // set ref.cpu 454MHZ
    HW_CLKCTRL_FRAC_CLR(BM_CLKCTRL_FRAC_CLKGATECPU);

    HW_CLKCTRL_FRAC_WR((HW_CLKCTRL_FRAC_RD() & ~BM_CLKCTRL_FRAC_CPUFRAC) | \
                        BF_CLKCTRL_FRAC_CPUFRAC(19));

    dwRegFrac = HW_CLKCTRL_FRAC_RD() & BM_CLKCTRL_FRAC_CPU_STABLE;
    for(; (HW_CLKCTRL_FRAC_RD() ^ dwRegFrac) == 0; ) ;

    // config CLK_CPU driver for 454/1 MHz (fractional divides it down
    // to 454MHz).
    HW_CLKCTRL_CPU_WR((BF_CLKCTRL_CPU_DIV_CPU(1)             | \
                       BF_CLKCTRL_CPU_DIV_CPU_FRAC_EN(0)     | \
                       BF_CLKCTRL_CPU_INTERRUPT_WAIT(1)      | \
                       BF_CLKCTRL_CPU_DIV_XTAL(1)            | \
                       BF_CLKCTRL_CPU_DIV_XTAL_FRAC_EN(0)));

    // config CLK_HBUS as CPU/3 (151MHz)
    HW_CLKCTRL_HBUS_WR((BF_CLKCTRL_HBUS_DIV(3)                    | \
                        BF_CLKCTRL_HBUS_DIV_FRAC_EN(0)            | \
                        BF_CLKCTRL_HBUS_SLOW_DIV(0)               | \
                        BF_CLKCTRL_HBUS_AUTO_SLOW_MODE(0)         | \
                        BF_CLKCTRL_HBUS_CPU_INSTR_AS_ENABLE(0)    | \
                        BF_CLKCTRL_HBUS_CPU_DATA_AS_ENABLE(0)     | \
                        BF_CLKCTRL_HBUS_TRAFFIC_AS_ENABLE(0)      | \
                        BF_CLKCTRL_HBUS_TRAFFIC_JAM_AS_ENABLE(0)  | \
                        BF_CLKCTRL_HBUS_APBXDMA_AS_ENABLE(0)      | \
                        BF_CLKCTRL_HBUS_APBHDMA_AS_ENABLE(0)));

    HW_CLKCTRL_CLKSEQ_CLR(BM_CLKCTRL_CLKSEQ_BYPASS_CPU);

    for(x=0; x++ != 0x1000; ) ;

}


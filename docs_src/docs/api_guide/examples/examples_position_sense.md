# Position Sense Examples {#EXAMPLES_POSITION_SENSE}

[TOC]

This page lists all the examples related to position sense.
\cond SOC_AM64X || SOC_AM243X
-# \subpage EXAMPLE_MOTORCONTROL_ENDAT
-# \subpage EXAMPLE_MOTORCONTROL_HDSL
-# \subpage EXAMPLE_MOTORCONTROL_TAMAGAWA
\endcond
\cond SOC_AM64X || SOC_AM243X
-# \subpage EXAMPLE_MOTORCONTROL_ENDAT
-# \subpage EXAMPLE_MOTORCONTROL_HDSL
-# \subpage EXAMPLE_MOTORCONTROL_TAMAGAWA
-# \subpage EXAMPLE_MOTORCONTROL_BISSC
-# \subpage EXAMPLE_MOTORCONTROL_NIKON
\endcond
\cond SOC_AM263X
-# \subpage EXAMPLE_MOTORCONTROL_BISSC
-# \subpage EXAMPLE_MOTORCONTROL_ENDAT
-# \subpage EXAMPLE_MOTORCONTROL_NIKON
-# \subpage EXAMPLE_MOTORCONTROL_TAMAGAWA
-# \subpage EXAMPLE_MOTORCONTROL_TAMAGAWA_OVER_UART
\endcond
\cond SOC_AM263PX
-# \subpage EXAMPLE_MOTORCONTROL_BISSC
-# \subpage EXAMPLE_MOTORCONTROL_ENDAT
-# \subpage EXAMPLE_MOTORCONTROL_NIKON
-# \subpage EXAMPLE_MOTORCONTROL_TAMAGAWA
-# \subpage EXAMPLE_MOTORCONTROL_TAMAGAWA_OVER_UART
\endcond
\cond SOC_AM261X
-# \subpage EXAMPLE_MOTORCONTROL_BISSC
-# \subpage EXAMPLE_MOTORCONTROL_ENDAT
-# \subpage EXAMPLE_MOTORCONTROL_HDSL
-# \subpage EXAMPLE_MOTORCONTROL_NIKON
-# \subpage EXAMPLE_MOTORCONTROL_TAMAGAWA
\endcond

## Encoder Examples Debug Guide {#ENCODER_EXAMPLES_DEBUG_GUIDE}

This section provides a debugging guide for troubleshooting issues that may arise during testing or development.

### Encoder Register Configuration

The \if (SOC_AM64X || SOC_AM243X) PRU_ICSSG_CFG \else ICSSM_PR1_CFG_SLV \endif registers from offset 0xE0 to 0x11C are allocated for encoder configuration. To review and verify the encoder settings:

1. Halt the R5 core
2. Open the memory browser window
3. Enter the address of the \if (SOC_AM64X || SOC_AM243X) PRU_ICSSG_CFG \else ICSSM_PR1_CFG_SLV \endif register and view the configured values
> **Note:** Ensure you are entering the correct base address for encoder registers. Each ICSS instance has different address for the CFG register.

For detailed register descriptions, refer to:
\if (SOC_AM64X || SOC_AM243X)
Section 6.4.14.5 PRU_ICSSG_CFG Registers in the AM243x Technical Reference Manual (TRM)
\elseif (SOC_AM263X || SOC_AM263PX)
Table 4-415 of the AM263x Sitara Processors Technical Reference Manual Register Addendum
\else
Table 4-1571 of the AM261x Sitara Processors Technical Reference Manual Register Addendum
\endif

Key registers and their configurations:

1. **ICSS_PRU0_ED_RX_CFG_REG**
   - Division factor for Rx clock
   - RX clock source
   - Polarity of the RX Start Bit
   - RX Over Sample size

2. **ICSS_PRU0_ED_TX_CFG_REG**
   - Division factor for Tx clock
   - Load share mode configuration
   - TX clock source

3. **ICSS_PRU0_ED_CH0_CFG0_REG**
   - RX frame size
   - TX frame size
   - TX wire delay configuration

4. **ICSS_PRU0_ED_CH0_CFG1_REG**
   - Rx arm counter value if Rx auto arm enable
   - TST delay

5. **ICSS_GPCFG0_REG**
   - Controls the ICSS wrap mux select (should be in EnDAT mode)

\cond (SOC_AM64X || SOC_AM243X)
6. **ICSSG_SA_MX_REG**
   - Use alternative encoder pins
\endcond

\image html Encoder_debug_cfg_register_view.png "PRU-ICSS 3 channel register view"

### Hardware Setup and Pin Configuration

The RS485 interface card, control card, and encoder should be connected to the correct PRU GPIO pins mapped for the Encoder interface \ref PRUICSS_PERIPHERAL_IF_MODE. All associated pins must be selected in SysConfig. 

\cond (SOC_AM64X || SOC_AM243X)
When using alternative pin options, the 'G_MUX_EN' bit of 'ICSSG_SA_MX_REG' register must be set.
\endcond

### PRU Firmware Debug

For PRU-related issues, verify that the application is loading the PRU firmware into the correct PRU core. If firmware loading fails:

1. Check the PRU core selection
2. Verify the PRU slice selection
3. Review application-level configuration

#### Debugging Steps for PRU Core with Loaded Firmware:

1. Import the PRU firmware for the target core
2. Build the firmware
3. Open the debug window and connect to the PRU core
4. Load symbols into the connected core:
   - Click the Load button
   - Select `Load Symbols`
   - Load the firmware .out file from the firmware project
5. After symbols are loaded, use the step-into button to execute debug commands

\image html EnDAT_debug_firmware_load.png "Load EnDAT RTU firmware symbols"

### Other Configurations

Guide to debug the components used in encoder examples:

#### IEP Registers Configuration

The Industrial Ethernet Peripheral (IEP) is used for periodic continuous mode. If IEP is not configured correctly, periodic continuous mode will not work.

To verify IEP configuration:

1. Check if IEP is running by examining `IEP_COUNT_REG0/1` registers
2. Verify counter increment by monitoring count values
3. Review `IEP_CMP_CFG_REG` to ensure all compare events are properly configured for trigger mode
4. Check `IEP_CMP_STATUS_REG` to verify corresponding compare events are setting status flags correctly
5. Validate that Compare Registers are configured with correct trigger point values

> **Note:** Ensure you are entering the correct address for IEP register. Each ICSS instance has different base address for the IEP register.
\image html Encoder_debug_iep_register_view1.png "PRU-ICSS IEP register view"
\image html Encoder_debug_iep_register_view2.png "PRU-ICSS IEP CMP events register view"

For detailed register descriptions, refer to:
\if (SOC_AM64X || SOC_AM243X)
Section `6.4.14.9 PRU_IEP_IEP Registers` in the AM243x TRM
\elseif (SOC_AM263X || SOC_AM263PX)
Table 4-417 of the AM263x Sitara Processors Technical Reference Manual Register Addendum
\else
Table 4-1573 of the AM261x Sitara Processors Technical Reference Manual Register Addendum
\endif

#### Interrupt Controller Internal Signals Mapping 
If you experience missing PRU interrupts or incorrect IRQ mapping, verify the interrupt mapping between PRU and R5 in the SysConfig PRU INTC module. The Host channel number and PRU Event should match your configuration.

For example, the EnDAT example uses:
- PRU Event: `18: pr0_pru_mst_intr[2]_intr_req`
- Host Channel: 2

\image html EnDAT_debug_INTC_view.png "PRU-ICSS INTC view"

The interrupt service routine (ISR) configuration is implemented in `endat_periodic_trigger.c`. Here's a key code snippet showing the configuration:

```c
/* R5F interrupt settings for ICSSG */
#define ICSS_PRU_ENDAT_INT_NUM         ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0 )

/* Register & enable ICSSG EnDat PRU FW interrupt */
HwiP_Params_init(&hwiPrms);
hwiPrms.intNum      = ICSS_PRU_ENDAT_INT_NUM;
hwiPrms.callback    = &pruEnDatIrqHandler;
hwiPrms.args        = 0;
hwiPrms.isPulse     = FALSE;
hwiPrms.isFIQ       = FALSE;
status              = HwiP_construct(&gIcssgEncoderHwiObject0, &hwiPrms);
DebugP_assert(status == SystemP_SUCCESS);

/* PRU EnDat FW IRQ handler */
void pruEnDatIrqHandler(void *args)
{
    /* Increment PRU ENDAT IRQ count */
    gPruEnDatIrqCnt0++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent(gPruIcssXHandle, PRU_TRIGGER_HOST_ENDAT_EVT0);
}
```

The PRU event number is defined in `endat_periodic_trigger.h`:
```c
#define PRU_TRIGGER_HOST_ENDAT_EVT0   ( 2+16 ) 
```
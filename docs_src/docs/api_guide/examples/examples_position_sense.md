# Position Sense Examples {#EXAMPLES_POSITION_SENSE}

[TOC]

This page lists all the examples related to position sense.
\cond SOC_AM243X
-# \subpage EXAMPLE_MOTORCONTROL_BISSC
-# \subpage EXAMPLE_MOTORCONTROL_ENDAT
-# \subpage EXAMPLE_MOTORCONTROL_ENDAT3
-# \subpage EXAMPLE_MOTORCONTROL_HDSL
-# \subpage EXAMPLE_MOTORCONTROL_NIKON
-# \subpage EXAMPLE_MOTORCONTROL_TAMAGAWA
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
-# \subpage EXAMPLE_MOTORCONTROL_ENDAT3
-# \subpage EXAMPLE_MOTORCONTROL_NIKON
-# \subpage EXAMPLE_MOTORCONTROL_TAMAGAWA
-# \subpage EXAMPLE_MOTORCONTROL_TAMAGAWA_OVER_UART
\endcond
\cond SOC_AM261X
-# \subpage EXAMPLE_MOTORCONTROL_BISSC
-# \subpage EXAMPLE_MOTORCONTROL_ENDAT
-# \subpage EXAMPLE_MOTORCONTROL_ENDAT3
-# \subpage EXAMPLE_MOTORCONTROL_HDSL
-# \subpage EXAMPLE_MOTORCONTROL_NIKON
-# \subpage EXAMPLE_MOTORCONTROL_TAMAGAWA
\endcond

## Encoder Examples Debug Guide {#ENCODER_EXAMPLES_DEBUG_GUIDE}

This section provides a debugging guide for troubleshooting issues that may arise during testing or development.

### Encoder Register Configuration

The \if (SOC_AM64X || SOC_AM243X) PRU_ICSSG_CFG \else ICSSM_PR1_CFG_SLV \endif registers from offset 0xE0 to 0x11C are allocated for encoder configuration. To review and verify the encoder settings:

1. Halt the Arm® Cortex®-R5F core
2. Open the memory browser window
3. Enter the address of the \if (SOC_AM64X || SOC_AM243X) PRU_ICSSG_CFG \else ICSSM_PR1_CFG_SLV \endif register and view the configured values
> **Note:** Ensure you are entering the correct base address for encoder registers. Each ICSS instance has a different address for the CFG register.

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
   - Rx arm counter value if Rx auto arm is enabled
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
When using alternative pin options, the 'G_MUX_EN' bit of the 'ICSSG_SA_MX_REG' register must be set.
\endcond

\cond (SOC_AM243X)
#### LP-AM243 + BP-AM2BLDCSERVO Booster Pack Pin Multiplexing for SDK example
<table>
<tr>
   <th>Pin name
   <th>Jumper number
   <th>Function
</tr>
<tr>
   <td>PRG0_PRU1_GPO0
   <td>J2.11
   <td>Channel 0 clock
</tr>
<tr>
   <td>PRG0_PRU1_GPO1
   <td>J7.67
   <td>Channel 0 transmit
</tr>
<tr>
   <td>PRG0_PRU1_GPO2
   <td>J7.68
   <td>Channel 0 transmit enable
</tr>
<tr>
   <td>PRG0_PRU1_GPI13
   <td>J8.71
   <td>Channel 0 receive
</tr>
<tr>
   <td>GPIO Pin (GPIO1_78/C16)
   <td>J8.73
   <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
<tr>
   <td>PRG0_PRU1_GPO6
   <td>J7.69
   <td>Channel 2 clock
</tr>
<tr>
   <td>PRG0_PRU1_GPO12
   <td>J8.72
   <td>Channel 2 transmit
</tr>
<tr>
   <td>PRG0_PRU1_GPO8
   <td>J6.57
   <td>Channel 2 transmit enable
</tr>
<tr>
   <td>PRG0_PRU1_GPI11
   <td>J7.70
   <td>Channel 2 receive
</tr>
<tr>
   <td>GPIO Pin (GPIO1_77/B17)
   <td>J8.74
   <td>Enable encoder voltage in Axis 2 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
</table>
\endcond

\cond SOC_AM261X
#### LP-AM261 + BP-AM2BLDCSERVO Booster Pack Pin Multiplexing for SDK example
<table>
<tr>
   <th>Pin name
   <th>Jumper number
   <th>Function
</tr>
<tr>
   <td>PR1_PRU0_GPIO0
   <td>J2.11
   <td>PRU0 Channel 0 clock
</tr>
<tr>
   <td>PR1_PRU0_GPIO1
   <td>J7.67
   <td>PRU0 Channel 0 transmit
</tr>
<tr>
   <td>PR1_PRU0_GPIO3
   <td>J7.68
   <td>PRU0 Channel 0 transmit enable
</tr>
<tr>
   <td>PR1_PRU0_GPI9
   <td>J8.71
   <td>PRU0 Channel 0 receive
</tr>
<tr>
   <td>GPIO Pin (GPIO_21/B10)
   <td>J8.73
   <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
<tr>
   <td>PR1_PRU1_GPIO0
   <td>J7.69
   <td>PRU1 Channel 0 clock
</tr>
<tr>
   <td>PR1_PRU1_GPIO1
   <td>J8.72
   <td>PRU1 Channel 0 transmit
</tr>
<tr>
   <td>PR1_PRU1_GPIO2
   <td>J6.57
   <td>PRU1 Channel 0 transmit enable
</tr>
<tr>
   <td>PR1_PRU1_GPI9
   <td>J7.70
   <td>PRU1 Channel 0 receive
</tr>
<tr>
   <td>GPIO Pin (GPIO_22/A10)
   <td>J8.74
   <td>Enable encoder voltage in Axis 2 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
</table>
\endcond

\cond (SOC_AM263X || SOC_AM263PX)

#### @VAR_LP_BOARD_NAME + BP-AM2BLDCSERVO Booster Pack Pin Multiplexing for SDK example
<table>
<tr>
   <th>Pin name
   <th>Jumper number
   <th>Function
</tr>
<tr>
   <td>PR0_PRU0_GPIO3
   <td>J2.19
   <td>Channel 1 clock
</tr>
<tr>
   <td>PR0_PRU0_GPO4
   <td>J2.17
   <td>Channel 1 transmit
</tr>
<tr>
   <td>PR0_PRU0_GPO5
   <td>J2.13
   <td>Channel 1 transmit enable
</tr>
<tr>
   <td>PR0_PRU0_GPI10
   <td>J1.8
   <td>Channel 1 receive
</tr>
<tr>
   <td>GPIO Pin (SDFM0_D1/D13)
   <td>J8.73
   <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
</table>
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

### Multi-channel modes

#### Multi-channel with single PRU mode

\note This subsection is applicable for BiSS-C, EnDat, Nikon A-format, and Tamagawa only.

- Data transmission and reception must happen simultaneously on all channels.
- The encoder configuration and cable length should be the same on all channels.
- If encoders across channels don't respond at the same time, this mode will not work. Load share configuration should be used instead.

#### Multi-channel with load share mode

\note This subsection is not applicable for EnDAT3.

- Data transmission and reception can happen independently on all channels.
- After a command is sent, all channels wait for a response and process the response independently. However, all channels must finish processing before the next command can be triggered. (This restriction does not apply to HDSL. HDSL channels can continue operating independently.)

### Periodic Trigger Mode

\note This subsection is not applicable for HDSL.

SDK examples uses IEP compare/capture event(s) to trigger commands in periodic mode. Firmware triggers a R5F interrupt after getting a response from the encoder. The application code uses a callback function to clear the PRU interrupt, which can be modified as per the use case. CMP/CAP event number can be configured using SysConfig module of encoder. Refer the example specific page for details (see \ref NIKON_EXAMPLE_PERIODIC_MODE).

When programming the values for periodic trigger CMP/CAP mode, ensure that the command send and receive can complete within the cycle time. In multi-channel, ensure that command completion for timings for all channels are considered. Incorrect values may send PRU FW in bad state. Also, refer to encoder specifications to ensure that requirement for mimimum interval between two commands is met.

### IEP Registers Configuration

The Industrial Ethernet Peripheral (IEP) is used for periodic trigger mode. If the IEP is not configured correctly, periodic trigger mode will not work.

To verify IEP configuration:

1. Check if the IEP is running by examining `IEP_COUNT_REG0/1` registers
2. Verify counter increment by monitoring count values
3. Review `IEP_CMP_CFG_REG` to ensure all compare events are properly configured for trigger mode
4. Check `IEP_CMP_STATUS_REG` to verify that corresponding compare events are setting status flags correctly
5. Validate that Compare Registers are configured with correct trigger point values

> **Note:** Ensure you are entering the correct address for IEP registers. Each ICSS instance has a different base address for the IEP registers.
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

### Interrupt Controller Internal Signals Mapping
If you experience missing PRU interrupts or incorrect IRQ mapping, verify the interrupt mapping between PRU and R5F.

- The host interrupt numbers defined in "*_periodic_trigger.c" file(s) in SDK examples (e.g., ICSS_PRU_ENDAT_INT_NUM) must match the interrupt channel assignments in SysConfig PRU(ICSS) module -> INTC section -> INTC Host Interrupt
- Host interrupt channels route PRU events to the R5F core
- Mapping: Host Interrupt 2-9 in SysConfig = HOST_INTR_PEND_0-7 registers
- If the host interrupt assignments are changed in SysConfig, macros in "*_periodic_trigger.c" file(s) need to be updated accordingly to maintain proper interrupt delivery from PRU to R5F.
- The INTC event numbers defined "*_periodic_trigger.c" file(s) must match the corresponding definitions in the PRU firmware header file (e.g., "source/position_sense/endat/firmware/endat_icss_reg_defs.h")
- These event numbers are used for communication between the R5F and PRU firmware. Any changes to these values must be synchronized between both files (firmware and application) to ensure proper interrupt handling.

For example, the EnDAT single channel example uses:
- PRU Event: `18: pr0_pru_mst_intr[2]_intr_req`
- Channel: 2
- Host Interrupt: 2

\image html EnDAT_debug_INTC_view.png "PRU-ICSS INTC view"

The interrupt service routine (ISR) configuration is implemented in `endat_periodic_trigger.c`. Here's a key code snippet showing the configuration:

```c
   /* Register and enable PRU FW interrupt */
   HwiP_Params_init(&hwi_params);
   hwi_params.intNum   = ICSS_PRU_ENDAT_INT_NUM;
   hwi_params.callback = &endat_pru_irq_handler;
   hwi_params.args     = pruicss_handle;
   hwi_params.isPulse  = FALSE;
   hwi_params.isFIQ    = FALSE;
   status              = HwiP_construct(&gEndatHwiObject[CONFIG_ENDAT0][0], &hwi_params);
   DebugP_assert(status == SystemP_SUCCESS);

   /* PRU FW IRQ handler */
   void endat_pru_irq_handler(void *pruicss_handle)
   {
      if(pruicss_handle == NULL)
      {
         return;
      }

      /* Increment IRQ count */
   #if(CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
      /* In load share mode, index 1 is used for channel 1 connected to PRU */
      gPruEndatIrqCnt[CONFIG_ENDAT0][1]++;
   #else
      /* In single PRU mode, index 0 is used for any channel connected to PRU */
      gPruEndatIrqCnt[CONFIG_ENDAT0][0]++;
   #endif
      /* Clear interrupt at source */
      PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_ENDAT_EVT);
   }
```

The R5F interrupt number "ICSS_PRU_ENDAT_INT_NUM" and PRU event number "PRU_TRIGGER_HOST_ENDAT_EVT" and is defined in `endat_periodic_trigger.h`.

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

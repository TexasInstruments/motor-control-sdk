# Tamagawa {#TAMAGAWA}

[TOC]

## Introduction

The Tamagawa receiver firmware running on PRU-ICSS provides a well-defined interface to execute the Tamagawa encoder communication protocol. The Tamagawa diagnostic application interacts with the Tamagawa receiver firmware interface.

\note This implementation using Peripheral input/output mode of PRU-ICSS. Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details.

## Features Supported {#TAMAGAWA_FEATURES}

-  Supports full-absolute SmartAbs & SmartInc encoders compatible with Smartceiver AU5561N1
-  Supports all Data Readout, Reset and EEPROM commands
-  Channel selection via SysConfig
-  Baud rate selection: 2.5 MHz and 5 MHz
\cond SOC_AM243X || SOC_AM64X
-  Support for concurrent multi-channel support on a single PRU
    - Up to 3 identical encoders of the same frequency connected to all configured channels.
    - In this mode, data transmission and reception must happen simultaneously on all channels.
    - The encoder configuration and cable length should be the same on all channels.
    - If encoders across channels don't respond at the same time, this mode will not work. Load share configuration should be used instead.
-  Support for multi-channel with different communication modes and different numbers of encoders connected across channels under load share mode (Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details).
    - Up to 3 channels with encoders of the same frequency connected to all configured channels.
    - In this mode, data transmission and reception can happen independently on all channels.
    - After a command is sent, all channels wait for a response and process the response independently. However, all channels must finish processing before the next command can be triggered.
\endcond
\cond SOC_AM261X
-  Support for dual channel configuration
    - Two independent single-channel instances using different PRU slices
    - Each channel can operate at different frequencies
    - Independent operation without load sharing
\endcond
-  Support for 8x oversampling for RX
   \note In three channel interface of PRU-ICSS, receive (Rx) is oversampled at 8x of send (Tx). Therefore, the encoder interface frequency "f" should be such that Tx source clock value is divisible by "f" and Rx source clock value is divisible by "(8*f)".
-  Support for periodic trigger using PRU-ICSS IEP timer module with two modes:
   - **CMP Mode**: IEP compare event based triggering (CMP0-CMP15)
   - **CAP Mode**: IEP capture event based triggering (CAP0-CAP7) for external signal synchronization
   - IEP events configurable via SysConfig
   - Same clock frequency for all channels in multi channel mode within the same PRU-ICSS slice
   - Different PRU slices can simultaneously handle encoders operating at different frequencies
- Possible interface speeds with different source clock combinations
- Booster Pack Support: Enable when using BP-AM2BLDCSERVO
<table>
<tr>
    <th>Clock Source
    <th>Interface Speed
</tr>
\cond SOC_AM261X
<tr>
    <td> PRU UART Clock (160 MHz)
    <td> 2.5 MHz, 5 MHz
</tr>
<tr>
    <td>PRU Core Clock (200 MHz)
    <td> 2.5 MHz, 5 MHz
</tr>
\endcond
\cond (SOC_AM263X || SOC_AM263PX)
<tr>
    <td> PRU UART Clock (160 MHz)
    <td> 2.5 MHz, 5 MHz
</tr>
<tr>
    <td>PRU Core Clock (200 MHz)
    <td> 2.5 MHz, 5 MHz
</tr>
\endcond
\cond SOC_AM243X || SOC_AM64X
<tr>
    <td>PRU Core Clock (200 MHz)
    <td> 2.5 MHz, 5 MHz
</tr>
\endcond

</table>

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not supported, including the following:
-  Other baud rates.
-  Independent clock frequency for each channel in multi channel mode within the same PRU-ICSS slice
    - Clock frequency is a PRU-ICSS slice level configuration
    - Each channel within the same PRU-ICSS slice in multi-channel mode will have same clock frequencies

## Periodic Trigger Modes {#TAMAGAWA_PERIODIC_MODES}

The Tamagawa driver supports two types of periodic trigger modes for continuous position sampling:

### CMP Mode (Compare Event Mode)
In CMP mode, the IEP timer compare event triggers position sampling. Compare events occur when the IEP timer counter matches the configured compare value. This mode enables fixed-rate periodic sampling.

**Configuration:**
- Compare event range: CMP0-CMP15 (0-15)
- Configured via \ref tamagawa_config_periodic_trigger_cmp_mode() API
- IEP compare event number set via \ref tamagawa_config_iep_cmp_event() API
- Event selection can be done in SysConfig
\note IEP configuration and CMP event configuration should be done in application. Driver uses these APIs to inform firmware to enable CMP periodic mode and uses the configured CMP event to start sampling periodically.

### CAP Mode (Capture Event Mode)
\cond SOC_AM243X
In CAP mode, external signals trigger position sampling through IEP capture events. The capture event is triggered on the rising edge of the external input pulse, enabling event-driven position capture. Internal signals can also be mapped to IEP capture events via TIMESYNC/GPIOMUX router.
\endcond

\cond (SOC_AM261X || SOC_AM263X || SOC_AM263PX)
In CAP mode, external signals trigger position sampling through IEP capture events. The capture event is triggered on the rising edge of the external input pulse, enabling event-driven position capture. Internal signals can also be mapped to IEP capture events via XBAR.
\endcond

**Configuration:**
- Capture event range: CAP0-CAP7 (0-7)
- Configured via \ref tamagawa_config_periodic_trigger_cap_mode() API
- IEP capture event number set via \ref tamagawa_config_iep_cap_event() API
- Event selection can be done in SysConfig
\note External signal must be routed to IEP capture input (via XBAR or router configuration) in application or SysConfig

\note CAP6 and CAP7 support falling edge detection as well. In Tamagawa, rising edge is used always.

\note Both IEP event configuration APIs (tamagawa_config_iep_cmp_event() and tamagawa_config_iep_cap_event()) are automatically called during tamagawa_init() with values configured in SysConfig.

## SysConfig Features {#TAMAGAWA_SYSCONFIG_FEATURES}

@VAR_SYSCFG_USAGE_NOTE

\cond SOC_AM243X
\attention For each PRU-ICSS slice being used for Tamagawa, one module instance should be created in SysConfig. For up to 3 channels using 1 slice, only 1 instance needs to be added.
\endcond

\cond SOC_AM261X
\attention For each PRU-ICSS slice being used for Tamagawa, one module instance should be created in SysConfig. For 2 channel example using 2 PRUs, 2 instances need to be added.
\endcond

\cond (SOC_AM263X || SOC_AM263PX)
\attention For each PRU-ICSS slice being used for Tamagawa, one module instance should be created in SysConfig.
\endcond

SysConfig can be used to configure the following:
\cond (SOC_AM263X || SOC_AM263PX)
- Selecting the ICSSM PRU slice (Tested on ICSSM-PRU0)
- Configuring PINMUX and GPIO
- IEP event selection for periodic trigger mode
\endcond
\if SOC_AM261X
- Selecting the ICSSM PRU slice (Tested on ICSSM1-PRU0)
- Configuring PINMUX and GPIO
- IEP event selection for periodic trigger mode
\else
- Selecting the ICSSG instance
- Selecting the ICSSG PRU slice (Tested on ICSSG0-PRU1)
- Configuring PINMUX, GPIO and ICSS clock to 200 MHz
- Enabling SA Mux mode
- IEP instance and IEP event selection for periodic trigger mode
\endif
- Channel selection
- Baud rate selection
- Selecting RX and TX source clock

## PRU-ICSS Resource Usage

- Utilizes the Peripheral IF mode (3-channel peripheral interface mode) for Tamagawa communication. Maximum of 3 channels are available per PRU slice. (Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details)
- Each channel has 4 pins (Clock, Data out, Data in, Output enable)
- Following table contains details of memory usage, IEP usage and interrupt controller usage:

\cond SOC_AM243X || SOC_AM64X
\attention In addition to the following resources used by PRU firmware, SDK examples also configure IEPx CMP0 for IEP counter reset in periodic trigger CMP mode and IEPx CMP1 for generating SYNC OUT0 used as input to CAP in periodic trigger CAP mode.

<table>
<tr>
   <th>Configuration per slice</th>
   <th>PRU Core(s)</th>
   <th>Memory Usage</th>
   <th>IEP Usage</th>
   <th>Interrupt Controller (INTC) Usage</th>
   <th>Description</th>
</tr>
<tr>
   <td>Single channel </td>
   <td>PRUx</td>
   <td>DMEM: 211 Bytes (0x00 to 0xD3)<br>IMEM: ~ 1.6 kB</td>
   <td><b>CMP Mode:</b> IEPx CMPy for trigger (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger (IEPx and CAPy selected in SysConfig)</td>
   <td>INTC event/input number 18 or 21 (prx_pru_mst_intr[2/5]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F based on slice </td>
   <td>IEP, CMP events and INTC signal are used only in periodic continuous mode</td>
</tr>
<tr>
   <td>Multi-channel with single PRU core</td>
   <td>PRUx</td>
   <td>DMEM: 211 Bytes (0x00 to 0xD3)<br>IMEM: ~ 1.6 kB</td>
   <td><b>CMP Mode:</b> IEPx CMPy for trigger (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger (IEPx and CAPy selected in SysConfig)</td>
   <td>INTC event/input number 18 or 21 (prx_pru_mst_intr[2/5]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F based on slice </td>
   <td>IEP, CMP events and INTC signal are used only in periodic continuous mode</td>
</tr>
<tr>
   <td rowspan="3">Multi-channel load share</td>
   <td>PRUx</td>
   <td>DMEM: 211 Bytes (0x00 to 0xD3)<br>IMEM: ~ 1.6 kB</td>
   <td><b>CMP Mode:</b> IEPx CMPy for trigger per channel (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger per channel (IEPx and CAPy selected in SysConfig)</td>
   <td rowspan="3">INTC events/inputs number 18, 19, 20 or 21, 22, 23 (prx_pru_mst_intr[2/3/4/5/6/7]_intr_req) are used to trigger interrupts to Arm® Cortex®-R5F </td>
   <td rowspan="3">IEP, CMP events and INTC signal are used only in periodic continuous mode</td>
</tr>
<tr>
   <td>RTU_PRUx</td>
   <td>DMEM: 211 Bytes (0x00 to 0xD3)<br>IMEM: ~ 1.6 kB</td>
   <td><b>CMP Mode:</b> IEPx CMPy for trigger per channel (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger per channel (IEPx and CAPy selected in SysConfig)</td>
</tr>
<tr>
   <td>TX_PRUx</td>
   <td>DMEM: 211 Bytes (0x00 to 0xD3)<br>IMEM: ~ 1.6 kB</td>
   <td><b>CMP Mode:</b> IEPx CMPy for trigger per channel (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger per channel (IEPx and CAPy selected in SysConfig)</td>
</tr>
\endcond

\cond (SOC_AM261X || SOC_AM263X || SOC_AM263PX)
\attention In addition to the following resources used by PRU firmware, SDK examples also configure IEP0 CMP0 for IEP counter reset in periodic trigger CMP mode.

<table>
<tr>
   <th>Configuration per slice</th>
   <th>PRU Core(s)</th>
   <th>Memory Usage</th>
   <th>IEP Usage</th>
   <th>Interrupt Controller (INTC) Usage</th>
   <th>Description</th>
</tr>
<tr>
   <td>Single channel </td>
   <td>PRUx</td>
   <td>DMEM: 211 Bytes (0x00 to 0xD3)<br>IMEM: ~ 1.6 kB</td>
   <td><b>CMP Mode:</b> IEP0 CMPy for trigger (IEP0 and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEP0 CAPy for trigger (IEP0 and CAPy selected in SysConfig)</td>
   <td>INTC event/input number 18 or 21 (prx_pru_mst_intr[2/5]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F based on slice </td>
   <td>IEP, CMP events and INTC signal are used only in periodic continuous mode</td>
</tr>
\endcond
</table>

\note For pin usage, see \ref TAMAGAWA_PIN_USAGE section.

## Tamagawa Design

\subpage TAMAGAWA_DESIGN explains the design in detail.

## Example

- \ref EXAMPLE_MOTORCONTROL_TAMAGAWA

## API

\ref TAMAGAWA_API_MODULE

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

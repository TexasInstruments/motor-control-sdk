# BISS-C {#BISS-C}

[TOC]

## Introduction

BiSS is an open-source digital interface for sensors and actuators. BiSS stands for bidirectional serial synchronous. The BiSS interface was introduced by iC-Haus GmbH as an open-source protocol in 2002. BiSS-C mode is the continuous mode in which the BiSS-C interface receiver reads out the position data cyclically. Control communication is available for the receiver to send commands to the encoders and to read and write the encoder local registers. The BiSS interface is used in position-control applications. The interface enables a complete closed-loop position control system by providing the real-time position feedback to the receiver to control the motor.

BiSS Safety is a profile definition for BiSS that has been certified by TÜV Rheinland for safety-critical applications up to SIL3 according to IEC61508:2010. BiSS Safety uses the concept of a "Black Channel" transmission and specifies the data channel contents in order to ensure failure mode detection as defined in IEC61784-3 using redundant position words, different CRC polynomials and a sign-of-life counter. BiSS Safety is fully compatible with BiSS and all of its features including line delay compensation, processing times. BiSS Safety is implemented by assuming 2 encoders connected in daisy chain, one will send CPW and another one will send SPW. Daisy chaining is also possible on top of safety (2 encoders dedicated safety - one for CPW and another one for SPW) up to 3 encoders per channel.

\note This implementation using Peripheral input/output mode of PRU-ICSS. Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details.

## Features Supported
\cond SOC_AM243X

   -  BiSS-C interface receiver for point-to-point communication
   -  Support for single channel implementation with one encoder
   -  Receive on-the-fly CRC verification of position and control data
   -  Interface speed of 1, 2, 5, 8, and 10 MHz
   -  Support for oversampling ratio with different interface speeds
<table>
<tr>
    <th rowspan="2">Clock Source
    <th rowspan="1" colspan="5">Interface Speed
</tr>
<tr>
    <th>1 MHz
    <th>2 MHz
    <th>5 MHz
    <th>8 MHz
    <th>10 MHz
</tr>
<tr>
    <td>PRU UART Clock (192 MHz)
    <td>8x
	<td>8x
    <td>Not tested
	<td>8x
    <td>Not tested
</tr>
<tr>
    <td>PRU Core Clock (200 MHz)
    <td>Not tested
	<td>Not tested
    <td>8x
	<td>Not tested
    <td>4x
</tr>
<tr>
    <td>PRU Core Clock (300 MHz)
    <td>Not tested
	<td>Not tested
    <td>8x with fractional div
	<td>Not tested
    <td>4x with Fractional div
</tr>
</table>

   -  Two modes of operation - host trigger and periodic trigger
   -  Support for periodic trigger using PRU-ICSS IEP timer module
   -  Support for single byte register communication using control communication
   -  Support for automatic processing delay detection and compensation
   -  Support for multiple encoders connected via daisy-chain configuration (up to 3 encoders)
   -  Support for concurrent multi-channel support on a single PRU
        - Up to 3 channels with identical number of encoders of the same frequency connected to all configured channels.
        - Data transmission and reception must happen simultaneously on all channels.
        - The encoder configuration and cable length should be the same on all channels.
        - If encoders across channels don't respond at the same time, this mode will not work. Load share configuration should be used instead.
   -  Support for multi-channel with encoders of different make and different numbers of encoders connected across channels under load share mode (Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details).
        - Up to 3 channels with encoders of the same frequency connected to all configured channels.
        - Data transmission and reception can happen independently on all channels.
        - After a command is sent, all channels wait for a response and process the response independently. However, all channels must finish processing before the next command can be triggered.
   -  Same clock frequency for all channels in multi channel mode within the same PRU-ICSS slice
        - Different PRU slices can simultaneously handle encoders operating at different frequencies
   -  Support for up to 100 meter cable
   -  Readiness for BiSS Safety profile by supporting 16 bit CRC and sign-of-life counter
\endcond

\cond SOC_AM261X

   -  BiSS-C interface receiver for point-to-point communication
   -  Support for single channel implementation with one encoder
   -  Receive on-the-fly CRC verification of position and control data
   -  Interface speed of 1, 2, 5, 8, and 10 MHz
   -  Support for oversampling ratio with different interface speeds
<table>
<tr>
    <th rowspan="2">Clock Source
    <th rowspan="1" colspan="5">Interface Speed
</tr>
<tr>
    <th>1 MHz
    <th>2 MHz
    <th>5 MHz
    <th>8 MHz
    <th>10 MHz
</tr>
<tr>
    <td>PRU UART Clock (160 MHz)
    <td>8x
	<td>8x
    <td>8x
	<td>4x
    <td>8x
</tr>

</table>
   -  Two modes of operation - host trigger and periodic trigger
   -  Support for periodic trigger using PRU-ICSS IEP timer module
   -  Support for control communication
   -  Support for automatic processing delay detection and compensation
   -  Support for multiple encoders connected via daisy-chain configuration (up to 3 encoders)
   -  Support for dual channel configuration using two independent PRU cores
   -  Support for up to 100 meter cable
   -  Readiness for BiSS Safety profile by supporting 16 bit CRC and sign-of-life counter

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

   -  BiSS-C interface receiver for point-to-point communication
   -  Support for single channel implementation with one encoder
   -  Receive on-the-fly CRC verification of position and control data
   -  Interface speed of 1, 2, 5, 8, and 10 MHz
   -  Support for oversampling ratio with different interface speeds
<table>
<tr>
    <th rowspan="2">Clock Source
    <th rowspan="1" colspan="5">Interface Speed
</tr>
<tr>
    <th>1 MHz
    <th>2 MHz
    <th>5 MHz
    <th>8 MHz
    <th>10 MHz
</tr>
<tr>
    <td>PRU UART Clock (192 MHz)
    <td>8x
	<td>8x
    <td>Not tested
	<td>8x
    <td>Not tested
</tr>
<tr>
    <td>PRU Core Clock (200 MHz)
    <td>Not tested
	<td>Not tested
    <td>8x
	<td>Not tested
    <td>4x
</tr>

</table>
   -  Two modes of operation - host trigger and periodic trigger
   -  Support for periodic trigger using PRU-ICSS IEP timer module
   -  Support for control communication
   -  Support for automatic processing delay detection and compensation
   -  Support for multiple encoders connected via daisy-chain configuration (up to 3 encoders)
   -  Support for up to 100 meter cable
   -  Readiness for BiSS Safety profile by supporting 16 bit CRC and sign-of-life counter

\endcond

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported in this release, including the following:
- Control communication
    - BiSS-C Commands (Control Select bit (CTS) = 0)
    - Following features with Register Communication (Control Select bit (CTS) = 1):
        - Start bit delay
        - Sequential multi-byte access
-  BiSS Line
-  Independent clock frequency for each channel in multi channel mode within the same PRU-ICSS slice
    - Clock frequency is a PRU-ICSS slice level configuration
    - Each channel within the same PRU-ICSS slice in multi-channel mode will have same clock frequencies

## SysConfig Features {#BISSC_SYSCONFIG_FEATURES}

\cond SOC_AM243X

@VAR_SYSCFG_USAGE_NOTE

\attention For each PRU-ICSS slice being used for BiSS-C, one module instance should be created in SysConfig. For up to 3 channels using 1 slice, only 1 instance needs to be added.

SysConfig can be used to configure the following:
- Selecting the ICSSG instance (Tested on ICSSG0)
- Selecting the ICSSG PRU slice (Tested on ICSSG0-PRU1)
- Configuring PINMUX
- Frequency selection
- Channel selection
- Selecting Multi Channel with encoders of different make using load share mode
- Enabling SA Mux mode
- Selecting clock source
- IEP instance and IEP event selection for periodic trigger mode
- Booster Pack Support: Enable when using BP-AM2BLDCSERVO
\note BiSS-C firmware is tested with ICSS Core Clock running at 200 MHz/300 MHz frequency or ICSS UART Clock running at 192 MHz only. ICSS Core Clock at 225/250/333 MHz is not supported due to clock divider requirements.
\endcond

\cond SOC_AM261X

@VAR_SYSCFG_USAGE_NOTE

\attention For each PRU-ICSS slice being used for BiSS-C, one module instance should be created in SysConfig. For dual channel example using 2 PRUs, 2 instances need to be added.

SysConfig can be used to configure the following:
- Selecting the ICSSM instance (Tested on ICSSM1)
- Selecting the ICSSM PRU slice (Tested on ICSSM1-PRU0 and ICSSM1-PRU1)
- Configuring PINMUX
- Frequency selection
- Channel selection
- Selecting clock source
- IEP event selection for periodic trigger mode
- Booster Pack Support: Enable when using BP-AM2BLDCSERVO
\note BiSS-C firmware is tested with ICSS UART Clock running at 160 MHz only, when ICSS Core Clock is 225 MHz due to clock divider requirements.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

@VAR_SYSCFG_USAGE_NOTE

\attention For each PRU-ICSS slice being used for BiSS-C, one module instance should be created in SysConfig.

SysConfig can be used to configure the following:
- Selecting the ICSSM PRU slice (Tested on ICSSM-PRU0)
- Configuring PINMUX
- Frequency selection
- Channel selection
- Selecting clock source
- IEP event selection for periodic trigger mode
- Booster Pack Support: Enable when using BP-AM2BLDCSERVO
\note BiSS-C firmware is tested with ICSS Core Clock running at 200 MHz frequency or ICSS UART Clock running at 192 MHz frequency only due to clock divider requirements.

\endcond

## Periodic Trigger Modes {#BISSC_PERIODIC_MODES}

The BiSS-C driver supports two types of periodic trigger modes for continuous position sampling:

### CMP Mode (Compare Event Mode)
In CMP mode, the IEP timer compare event triggers position sampling. Compare events occur when the IEP timer counter matches the configured compare value. This mode enables fixed-rate periodic sampling.

**Configuration:**
- Compare event range: CMP0-CMP15 (0-15)
- Configured via \ref bissc_config_periodic_trigger_cmp_mode() API
- IEP compare event number set via \ref bissc_config_iep_cmp_event() API
- Event selection can be done in SysConfig
- IEP configuration and CMP event configuration should be done in application. Driver uses above APIs to inform firmware to enable CMP periodic mode and uses the configured CMP event to start sampling periodically.

### CAP Mode (Capture Event Mode)
\cond SOC_AM243X
In CAP mode, external signals trigger position sampling through IEP capture events. The capture event is triggered on the rising edge of the external input pulse, enabling event-driven position capture. Internal signals can also be mapped to IEP capture events via TIMESYNC/GPIOMUX router.
\endcond

\cond (SOC_AM261X || SOC_AM263X || SOC_AM263PX)
In CAP mode, external signals trigger position sampling through IEP capture events. The capture event is triggered on the rising edge of the external input pulse, enabling event-driven position capture. Internal signals can also be mapped to IEP capture events via XBAR.
\endcond

**Configuration:**
- Capture event range: CAP0-CAP7 (0-7)
- Configured via \ref bissc_config_periodic_trigger_cap_mode() API
- IEP capture event number set via \ref bissc_config_iep_cap_event() API
- Event selection can be done in SysConfig
- IEP configuration and CAP event configuration should be done in application. Driver uses above APIs to inform firmware to enable CAP periodic mode and uses the configured CAP event to start sampling periodically.

\note
    - External signal must be routed to IEP capture input (if needed) in application
    - CAP6 and CAP7 support falling edge detection as well. In BiSS-C, rising edge is used always.

\attention Both IEP event configuration APIs (bissc_config_iep_cmp_event() and bissc_config_iep_cap_event()) are automatically called during bissc_init() with values configured in SysConfig.

## PRU-ICSS Resource Usage

- Utilizes the Peripheral IF mode (3-channel peripheral interface mode) for BiSS-C communication. Maximum of 3 channels are available per PRU slice. (Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details)
- Each channel has 4 pins (Clock, Data out, Data in, Output enable)
    - For BiSS-C, only clock and data pins are needed
    - If RS485 is used, ensure that TX enable is always pulled low
- Following table contains details of memory usage, IEP usage and interrupt controller usage:

\cond SOC_AM243X

\attention In addition to the following resources used by PRU firmware, SDK examples also configure IEPx CMP0 for IEP counter reset in periodic trigger CMP mode and IEPx CMP1 for generating SYNC OUT0 used as input to CAP in periodic trigger CAP mode.

<table>
<tr>
    <th> Configuration per slice
    <th> PRU Core(s)
    <th> Memory Usage
    <th> IEP Usage
    <th> Interrupt Controller (INTC) Usage
    <th> Description
</tr>
<tr>
    <td> Single channel
    <td> PRUx
    <td> DMEM: (0x0 to 0x133) 308 Bytes <br>IMEM: ~ 3.13 kB
	<td> <b>CMP Mode:</b> IEPx CMPy for trigger (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger (IEPx and CAPy selected in SysConfig)
    <td> INTC event/input number 18 or 21 (prx_pru_mst_intr[2/3]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F based on slice
    <td> IEP, CMP/CAP events and INTC signals are used only in periodic trigger modes
</tr>
<tr>
    <td> Multi-channel with single PRU core
    <td> PRUx
    <td> DMEM: (0x0 to 0x133) 308 Bytes <br>IMEM: ~ 3.43 kB
	<td> <b>CMP Mode:</b> IEPx CMPy for trigger (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger (IEPx and CAPy selected in SysConfig)
    <td> INTC event/input number 18 or 21 (prx_pru_mst_intr[2/3]_intr_req) is used to trigger interrupt to R5F based on slice
    <td> IEP, CMP/CAP events and INTC signals are used only in periodic trigger modes
</tr>
<tr>
    <td rowspan="3"> Multi-channel with load share across 3 PRU cores
    <td> PRUx
    <td rowspan="3"> DMEM: (0x0 to 0x133) 308 Bytes <br>IMEM (per core): ~ 3.74 kB
	<td rowspan="3"> <b>CMP Mode:</b> IEPx CMPy for trigger per channel (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger per channel (IEPx and CAPy selected in SysConfig)
    <td rowspan="3"> INTC events/inputs number 18, 19, 20 or 21, 22, 23 (prx_pru_mst_intr[2/3/4/5/6/7]_intr_req) are used to trigger interrupts to R5F
    <td rowspan="3"> IEP, CMP/CAP events and INTC signals are used only in periodic trigger modes</td>
</tr>
<tr>
    <td> RTU_PRUx
</tr>
<tr>
    <td> TX_PRUx
</tr>
</table>

\note For pin usage, see \ref BISSC_PIN_USAGE section.

\endcond

\cond (SOC_AM261X || SOC_AM263X || SOC_AM263PX)

\attention In addition to the following resources used by PRU firmware, SDK examples also configure IEP0 CMP0 for IEP counter reset in periodic trigger CMP mode.

<table>
<tr>
    <th> Configuration per slice
    <th> PRU Core
    <th> Memory Usage
    <th> IEP Usage
    <th> Interrupt Controller (INTC) Usage
    <th> Description
</tr>
<tr>
    <td> Single channel
    <td> PRUx
    <td> DMEM: (0x0 to 0x133) 308 Bytes <br>IMEM: ~ 3.13 kB
	<td> <b>CMP Mode:</b> IEP0 CMPy for trigger (IEP0 CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEP0 CAPy for trigger (IEP0 CAPy selected in SysConfig)
    <td> INTC event/input number 18 or 21 (prx_pru_mst_intr[2/3]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F based on slice
    <td> IEP, CMP/CAP events and INTC signals are used only in periodic trigger modes
</tr>
</table>
\note For pin usage, see \ref BISSC_PIN_USAGE section.

\endcond

## BISS-C Design

\subpage BISSC_DESIGN explains the design in detail.

## Example
\ref EXAMPLE_MOTORCONTROL_BISSC

## API
\ref BISSC_API_MODULE

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

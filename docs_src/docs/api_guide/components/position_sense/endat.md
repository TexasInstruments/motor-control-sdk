# EnDat {#ENDAT}

[TOC]

## Introduction

EnDat is a bidirectional interface for position encoders. During EnDat operation, the EnDat receiver receives position information from the EnDat position encoder.

\note This implementation using Peripheral input/output mode of PRU-ICSS. Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details.

## Features Supported

   -  EnDat 2.2 command set
   -  EnDat 2.1 command set
   -  Interrupted and continuous clock mode
   -  Cable length up to 100m @8MHz
   -  Propagation delay compensation  \if (SOC_AM243X || SOC_AM64X) (capable of handling different
      propagation delays of different channels in concurrent multi-channel configuration) \endif
   -  Automatic estimation of propagation delay
   -  Receive on-the-fly CRC verification of position, parameters and additional information
   -  Two modes of operation - host trigger and periodic trigger
   -  Channel select
   -  Same clock frequency for all channels in multi channel mode within the same PRU-ICSS slice
        - Different PRU slices can simultaneously handle encoders operating at different frequencies
   -  Support for periodic trigger using PRU-ICSS IEP timer module
\cond SOC_AM243X || SOC_AM64X
   -  Concurrent multi-channel support (up to 3 encoders with identical part numbers @ 8MHz maximum)
      - Data transmission and reception must happen simultaneously on all channels.
      - The encoder configuration and cable length should be the same on all channels.
      - If encoders across channels don't respond at the same time, this mode will not work. Load share configuration should be used instead.
   -  "Multi Channel with encoders of different make" using load share mode (Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details)
      - Data transmission and reception can happen independently on all channels.
      - After a command is sent, all channels wait for a response and process the response independently. However, all channels must finish processing before the next command can be triggered.
\endcond
\cond SOC_AM261X
   -  Dual channel support using two independent PRU cores (PRU0 and PRU1)
      - Each PRU core handles one encoder channel independently
      - Different PRU slices can operate encoders at different frequencies
      - Each channel requires a separate EnDat instance in SysConfig
\endcond
   -  Safety Readiness: Recovery time
   -  Clock up to 16MHz with single channel \if (SOC_AM243X || SOC_AM64X) and load share mode (multi-channel) \endif
      \note In three channel interface of PRU-ICSS, receive (Rx) is oversampled at 8x of send (Tx). Therefore, the encoder interface frequency "f" should be such that Tx source clock value is divisible by "f" and Rx source clock value is divisible by "8*f".
   - Possible interface speeds with different source clock combinations:
<table>
<tr>
   <th>Clock Source
   <th>Interface Speed
</tr>
\cond SOC_AM261X
<tr>
   <td> PRU UART Clock (160 MHz)
   <td> 1MHz, 2MHz, 4MHz, 5MHz, 10MHz
</tr>
<tr>
   <td>PRU Core Clock (225 MHz)
   <td> 1MHz, 5MHz
</tr>
\endcond
\cond (SOC_AM263X || SOC_AM263PX)
<tr>
   <td> PRU UART Clock (192 MHz)
   <td> 1MHz, 2MHz, 4MHz, 6MHz, 8MHz, 12MHz, 16MHz
</tr>
<tr>
   <td>PRU Core Clock (200 MHz)
   <td> 1MHz, 5MHz
</tr>
\endcond
\cond SOC_AM243X || SOC_AM64X
<tr>
   <td> PRU UART Clock (192 MHz)
   <td> 1MHz, 2MHz, 4MHz, 6MHz, 8MHz, 12MHz, 16MHz
</tr>
<tr>
   <td>PRU Core Clock (200 MHz)
   <td> 1MHz, 5MHz
</tr>
\endcond

</table>

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported in this release, including the below:
-  Independent clock frequency for each channel in multi channel mode within the same PRU-ICSS slice
    - Clock frequency is a PRU-ICSS slice level configuration
    - Each channel within the same PRU-ICSS slice in multi-channel mode will have same clock frequencies
\cond SOC_AM243X || SOC_AM64X
-  Continuous clock mode in Multi-channel single PRU mode
\endcond

### Limitations
\cond SOC_AM243X || SOC_AM64X
This section describes known limitations of the current implementation in multi-channel single PRU mode.
- Clock above 8 MHz: it is not possible to oversample, downsample and store one bit for all three channels in one clock cycle time.
- Reset command CRC failure: The encoder which takes more time in reset operation will show CRC failure because the reset time is not the same for each encoder so the acknowledgment will not arrive at the same time for all encoders at the EnDat receiver end.
\endcond

## SysConfig Features {#ENDAT_SYSCONFIG_FEATURES}

@VAR_SYSCFG_USAGE_NOTE

\cond SOC_AM243X || SOC_AM64X
\attention For each PRU-ICSS slice being used for EnDat, one module instance should be created in SysConfig. For up to 3 channels using 1 slice, only 1 instance needs to be added.
\endcond

\cond SOC_AM261X
\attention For each PRU-ICSS slice being used for EnDat, one module instance should be created in SysConfig. For 2 channel example using 2 PRUs, 2 instances need to be added.
\endcond

\cond (SOC_AM263X || SOC_AM263PX)
\attention For each PRU-ICSS slice being used for EnDat, one module instance should be created in SysConfig.
\endcond

SysConfig can be used to configure the following:
- Selecting the ICSS instance
\cond (SOC_AM263X || SOC_AM263PX)
- Selecting the ICSSM PRU slice (Tested on ICSSM-PRU0)
\endcond
\cond SOC_AM261X
- Selecting the ICSSM instance (Tested on ICSSM1)
- Selecting the ICSSM PRU slice (Tested on ICSSM1-PRU0 and ICSSM1-PRU1)
\endcond
\cond SOC_AM243X || SOC_AM64X
- Selecting the ICSSG instance (Tested on ICSSG0)
- Selecting the ICSSG PRU slice (Tested on ICSSG0-PRU1)
\endcond
- Configuring PINMUX
- Channel selection
\cond SOC_AM243X || SOC_AM64X
- Selecting Multi Channel with encoders of different make using load share mode
- Enabling SA Mux mode
- IEP instance and IEP event selection for periodic trigger mode
\endcond
\cond SOC_AM261X
- IEP event selection for periodic trigger mode
\endcond
\cond (SOC_AM263X || SOC_AM263PX)
- IEP event selection for periodic trigger mode
\endcond
- Selecting RX and TX source clock
- Booster Pack Support: Enable when using BP-AM2BLDCSERVO

\cond SOC_AM243X || SOC_AM64X
\note EnDat firmware is tested with ICSS Core Clock running at 200 MHz/300 MHz frequency or ICSS UART Clock running at 192 MHz only.
\endcond

\cond SOC_AM261X
\note EnDat firmware is tested with ICSS Core Clock running at 225 MHz frequency or ICSS UART Clock running at 160 MHz only.
\endcond

\cond (SOC_AM263X || SOC_AM263PX)
\note EnDat firmware is tested with ICSS Core Clock running at 200 MHz frequency or ICSS UART Clock running at 192 MHz only.
\endcond

## Periodic Trigger Modes {#ENDAT_PERIODIC_MODES}

The EnDat driver supports two types of periodic trigger modes for continuous position sampling:

### CMP Mode (Compare Event Mode)
In CMP mode, the IEP timer compare event triggers position sampling. Compare events occur when the IEP timer counter matches the configured compare value. This mode enables fixed-rate periodic sampling.

**Configuration:**
- Compare event range: CMP0-CMP15 (0-15)
- Configured via \ref endat_config_periodic_trigger_cmp_mode() API
- IEP compare event number set via \ref endat_config_iep_cmp_event() API
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
- Configured via \ref endat_config_periodic_trigger_cap_mode() API
- IEP capture event number set via \ref endat_config_iep_cap_event() API
- Event selection can be done in SysConfig
- IEP configuration and CAP event configuration should be done in application. Driver uses above APIs to inform firmware to enable CAP periodic mode and uses the configured CAP event to start sampling periodically.

\note
    - External signal must be routed to IEP capture input (if needed) in application
    - CAP6 and CAP7 support falling edge detection as well. In EnDat, rising edge is used always.

\attention Both IEP event configuration APIs (endat_config_iep_cmp_event() and endat_config_iep_cap_event()) are automatically called during endat_init() with values configured in SysConfig.

## PRU-ICSS Resource Usage

- Utilizes the Peripheral IF mode (3-channel peripheral interface mode) for EnDat communication. Maximum of 3 channels are available per PRU slice. (Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details)
- Each channel has 4 pins (Clock, Data out, Data in, Output enable)
- Following table contains details of memory usage, IEP usage and interrupt controller usage:

\cond SOC_AM243X || SOC_AM64X
\attention In addition to the following resources used by PRU firmware, SDK examples also configure IEPx CMP0 for IEP counter reset in periodic trigger CMP mode and IEPx CMP1 for generating SYNC OUT0 used as input to CAP in periodic trigger CAP mode.
\endcond

\cond (SOC_AM261X || SOC_AM263X || SOC_AM263PX)
\attention In addition to the following resources used by PRU firmware, SDK examples also configure IEP0 CMP0 for IEP counter reset in periodic trigger CMP mode.
\endcond

<table>
<tr>
   <th>Configuration per slice</th>
   <th>PRU Core(s)</th>
   <th>Memory Usage</th>
   <th>IEP Usage</th>
   <th>Interrupt Controller (INTC) Usage</th>
   <th>Description</th>
</tr>
\cond SOC_AM243X || SOC_AM64X
<tr>
   <td>Single channel</td>
   <td>PRUx</td>
   <td>DMEM:188 Bytes, from offset <code>0x00</code> to <code>0xBB</code> offset <br>IMEM: ~ 5.57 kB  <br>TCMB0: 40 Bytes, 40 Bytes of memory can be located anywhere within the offset range 0x00 to 0x78, depending on the selected channel.</td>
   <td><b>CMP Mode:</b> IEPx CMPy for trigger (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger (IEPx and CAPy selected in SysConfig)</td>
   <td>INTC event/input number 18 or 21 (prx_pru_mst_intr[2/5]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F based on slice</td>
   <td>IEP, CMP/CAP events and INTC signal are used only in periodic trigger modes</td>
</tr>
\endcond
\cond (SOC_AM261X || SOC_AM263PX)
<tr>
   <td>Single channel</td>
   <td>PRUx</td>
   <td>DMEM:188 Bytes, from offset <code>0x00</code> to <code>0xBB</code> offset <br>IMEM: ~ 5.57 kB  <br>TCMB0: 40 Bytes, 40 Bytes of memory can be located anywhere within the offset range 0x00 to 0x78, depending on the selected channel.</td>
   <td><b>CMP Mode:</b> IEP0 CMPy for trigger (CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEP0 CAPy for trigger (CAPy selected in SysConfig)</td>
   <td>INTC event/input number 18 or 21 (pr0_pru_mst_intr[2/5]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F based on slice</td>
   <td>IEP, CMP/CAP events and INTC signal are used only in periodic trigger modes</td>
</tr>
\endcond
\cond SOC_AM243X || SOC_AM64X
<tr>
   <td>Multi-channel with single PRU core</td>
   <td>PRUx</td>
   <td>DMEM: 188 Bytes, from offset <code>0x00</code> to <code>0xBB</code> offset  <br>IMEM: ~ 6.71 KB <br>TCMB0:120 Bytes, from offset <code>0x00</code> to <code>0x78</code> offset </td>
   <td><b>CMP Mode:</b> IEPx CMPy for trigger (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger (IEPx and CAPy selected in SysConfig)</td>
   <td>INTC event/input number 18 or 21 (prx_pru_mst_intr[2/5]_intr_req) is used to trigger interrupt to R5F based on slice</td>
   <td>IEP, CMP/CAP events and INTC signal are used only in periodic trigger modes</td>
</tr>
<tr>
    <td rowspan="3">Multi-channel with load share across 3 PRU cores</td>
    <td>PRUx</td>
    <td rowspan="3">DMEM: 188 Bytes, from offset <code>0x00</code> to <code>0xBB</code> offset  <br>IMEM (per core): ~ 4.1 KB <br>TCMB0:120 Bytes, from offset <code>0x00</code> to <code>0x78</code> offset</td>
    <td rowspan="3"><b>CMP Mode:</b> IEPx CMPy for trigger (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger (IEPx and CAPy selected in SysConfig)</td>
    <td rowspan="3">INTC events/inputs number 18, 19, 20 or 21, 22, 23 (prx_pru_mst_intr[2/3/4/5/6/7]_intr_req) are used to trigger interrupts to R5F</td>
    <td rowspan="3">IEP, CMP/CAP events and INTC signals are used only in periodic trigger modes</td>
</tr>
<tr>
    <td>RTU_PRUx</td>
</tr>
<tr>
    <td>TX_PRUx</td>
</tr>
\endcond
</table>

\attention TCMB0 memory is a part of R5F, not PRU-ICSS.

\note For pin usage, see \ref ENDAT_PIN_USAGE section.

## ENDAT Design

\subpage ENDAT_DESIGN explains the design in detail.

## Example
\ref EXAMPLE_MOTORCONTROL_ENDAT

## API
\ref ENDAT_API_MODULE

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

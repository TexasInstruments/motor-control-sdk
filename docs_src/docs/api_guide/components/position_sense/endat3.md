# EnDat3 {#ENDAT3}

[TOC]

## Introduction

EnDat3 is the latest generation of the bidirectional serial interface for position encoders, providing enhanced safety features and higher data rates compared to EnDat 2.2. During EnDat3 operation, the EnDat receiver (subsequent electronics) communicates with EnDat3 position encoders to obtain position information and additional diagnostic data.

EnDat3 introduces several key improvements over previous versions:
- Higher data transfer rates (up to 25 Mbps)
- Enhanced safety features with frame-based CRC protection
- Improved error detection and recovery mechanisms
- Support for both host-triggered and periodic trigger modes
- Background and foreground communication channels for concurrent operations

\note This implementation uses the PRU-ICSS peripheral interface mode. Refer to \ref PRUICSS_PERIPHERAL_IF_MODE for more details.

## Features Supported

- EnDat3 protocol specification compliance
- Data transfer rates: 12.5 Mbps (25 Mbps not currently supported)
- Single channel operation
- Three modes of operation:
   - Host trigger mode: Commands initiated by host controller
   - Periodic CMP trigger mode: Automatic position updates triggered by IEP compare events
   - Periodic CAP trigger mode: Automatic position updates triggered by IEP capture events
- Frame-based protocol with three frame types:
   - High Priority Frame (HPF): Position data and critical status
   - Low Priority Header (LPH): Information about following LPF frames
   - Low Priority Frame (LPF): Additional data and diagnostics
- Foreground communication commands:
   - DATA0-DATA7: Activate LPF send lists
   - DATA: General data with background data
   - DATANOP: Data without background data
   - RESET: Encoder reset
   - CLEAR: Reset encoder states
   - ECHO: Propagation time measurement
   - RATE: Set data transfer rate
   - HELLO: Switch to EnDat3 mode
   - FORCE: Forced dynamic sampling
   - BUSBC: Bus broadcast command
   - BUSP2P: Bus point-to-point command
   - BUSINIT: Bus initialization
- Background communication commands:
   - NOP: No operation
   - READ: Read from encoder memory
   - WRITE: Write to encoder memory
   - RECONFIGURE: Reconfigure parameters
   - AUTH: Authentication with user levels
   - PROTECT: Set memory protection
   - SETPASS: Set password for user level
   - LOCATE: Encoder location function
- Continuous position fetch mode for real-time applications
- Automatic CRC verification for all frame types
- Error detection and reporting with detailed error codes
- Manchester encoding support for robust data transmission
- Flexible channel configuration

\cond SOC_AM243X
   - Supported on PRU-ICSSG0
   - PRU0 or PRU1 configuration via SysConfig
\endcond

\cond SOC_AM261X
   - Supported on PRU-ICSSM1
   - PRU0 or PRU1 configuration via SysConfig
\endcond

\cond SOC_AM263PX
   - Supported on PRU-ICSSM
   - PRU0 or PRU1 configuration via SysConfig
\endcond

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported in this release, including the below:
- **25 Mbps Data Rate:** Currently only 12.5 Mbps is supported and tested
- **Multi-channel concurrent operation:** Only single channel is supported
- **Daisy Chain Topology:** Daisy chaining multiple encoders is not supported
- Cable length up to 100m

### Limitations

- Cable delays must be compensated for frequencies above 10 MHz
- Recovery time settings must match encoder specifications
- HELLO command may take up to 300ms to complete encoder initialization
- RESET command may take up to 300ms for encoder restart
- RATE command switching occurs after 2ms of no communication and may take up to 300ms

## SysConfig Features {#ENDAT3_SYSCONFIG_FEATURES}

@VAR_SYSCFG_USAGE_NOTE

\attention One module instance should be created in SysConfig per PRU-ICSS slice being used for EnDat3.

\cond SOC_AM243X
SysConfig can be used to configure the following:
- Selecting the ICSSG instance (Tested on ICSSG0)
- Selecting the ICSSG PRU slice (Tested on ICSSG0-PRU1)
- Configuring PINMUX
- Enabling SA Mux mode
- Channel selection (Channel 0, Channel 1, or Channel 2)
- Selecting baud rate (12.5 Mbps)
- IEP instance and IEP event selection for periodic trigger mode
- Booster Pack Support: Enable when using BP-AM2BLDCSERVO

\note EnDAT3 firmware is tested with ICSS Core Clock running at 200 MHz/300 MHz frequency only. ICSS Core Clock at 225/250/333 MHz is not supported due to clock divider requirements.

\endcond

\cond SOC_AM261X
SysConfig can be used to configure the following:
- Selecting the ICSSM instance (Tested on ICSSM1)
- Selecting the ICSSM PRU slice (Tested on ICSSM1-PRU0)
- Configuring PINMUX
- Channel selection (Channel 0, Channel 1, or Channel 2)
- Selecting baud rate (12.5 Mbps)
- IEP event selection for periodic trigger mode
- Booster Pack Support: Enable when using BP-AM2BLDCSERVO

\note EnDAT3 firmware is tested with ICSS Core Clock running at 200 MHz only (R5F Core Clock has to be 400 MHz) due to clock divider requirements.

\endcond

\cond SOC_AM263PX
SysConfig can be used to configure the following:
- Selecting the ICSSM PRU slice (Tested on ICSSM-PRU0)
- Configuring PINMUX
- Channel selection (Channel 0, Channel 1, or Channel 2)
- Selecting baud rate (12.5 Mbps)
- IEP event selection for periodic trigger mode
- Booster Pack Support: Enable when using BP-AM2BLDCSERVO

\note EnDAT3 firmware is tested with ICSS Core Clock running at 200 MHz frequency only due to clock divider requirements.
\endcond

## PRU-ICSS Resource Usage

- Utilizes the Peripheral IF mode (3-channel peripheral interface mode) for EnDAT3 communication. Maximum of 3 channels are available per PRU slice. (Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details)
- Each channel has 4 pins (Clock, Data out, Data in, Output enable)
- Following table contains details of memory usage, IEP usage and interrupt controller usage:

\cond SOC_AM243X

\attention In addition to the following resources used by PRU firmware, SDK examples also configure IEPx CMP0 for IEP counter reset in periodic trigger CMP mode and IEPx CMP1 for generating SYNC OUT0 used as input to CAP in periodic trigger CAP mode.

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
    <td> DMEM: 508 Bytes (0x0000 to 0x01FB) <br> IMEM: ~ 3.55 kB
    <td> <b>CMP Mode:</b> IEPx CMPy for trigger (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger (IEPx and CAPy selected in SysConfig)
    <td> INTC event/input number 18 or 21 (prx_pru_mst_intr[2/5]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F based on slice
    <td> IEP, CMP/CAP events and INTC signals are used only in periodic trigger modes
</tr>
</table>

\note For pin usage, see \ref ENDAT3_PIN_USAGE section.

\endcond

\cond (SOC_AM261X || SOC_AM263PX)

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
    <td> DMEM: 508 Bytes (0x0000 to 0x01FB) <br> IMEM: ~ 3.55 kB
    <td> <b>CMP Mode:</b> IEP0 CMPy for trigger (IEP0 CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEP0 CAPy for trigger (IEP0 CAPy selected in SysConfig)
    <td> INTC event/input number 18 or 21 (prx_pru_mst_intr[2/5]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F based on slice
    <td> IEP, CMP/CAP events and INTC signals are used only in periodic trigger modes
</tr>
</table>

\note For pin usage, see \ref ENDAT3_PIN_USAGE section.

\endcond

## Interface Clock Frequencies

The EnDat3 interface supports two standard data transfer rates:

<table>
<tr>
   <th>Data Rate</th>
   <th>TX Clock Frequency</th>
   <th>RX Clock Frequency</th>
   <th>Note</th>
</tr>
<tr>
   <td>12.5 Mbps</td>
   <td>25 MHz</td>
   <td>100 MHz (8x oversampling)</td>
   <td>Supported in this release</td>
</tr>
<tr>
   <td>25 Mbps</td>
   <td>50 MHz</td>
   <td>200 MHz (8x oversampling)</td>
   <td>Not supported in this release</td>
</tr>
</table>

### PRU Core Clock Requirements

The PRU core clock must be configured to support the required TX and RX clock frequencies:

\cond SOC_AM261X
<table>
<tr>
   <th>PRU Core Clock</th>
   <th>Supported Data Rates</th>
</tr>
<tr>
   <td>200 MHz</td>
   <td>12.5 Mbps</td>
</tr>
</table>
\endcond

\cond (SOC_AM263PX)
<table>
<tr>
   <th>PRU Core Clock</th>
   <th>Supported Data Rates</th>
</tr>
<tr>
   <td>200 MHz</td>
   <td>12.5 Mbps</td>
</tr>
</table>
\endcond

\cond SOC_AM243X
<table>
<tr>
   <th>PRU Core Clock</th>
   <th>Supported Data Rates</th>
</tr>
<tr>
   <td>300 MHz</td>
   <td>12.5 Mbps</td>
</tr>
</table>
\endcond

## EnDat3 Protocol Overview

### Frame Structure

EnDat3 uses a frame-based protocol with three types of frames:

1. **High Priority Frame (HPF)**
   - Contains position data (48 bits)
   - Status byte with error/warning flags
   - CRC-8 checksum
   - Transmitted in every communication cycle

2. **Low Priority Header (LPH)**
   - Indicates number of following LPF frames
   - Communication status information
   - CRC-8 checksum

3. **Low Priority Frame (LPF)**
   - Additional data and diagnostics (48 bits per frame)
   - Frame ID (FID) in status byte
   - CRC-8 checksum
   - Up to 8 frames can follow HPF and LPH

### Communication Flow

1. Host sends command via foreground channel
2. Encoder responds with HPF (position + status)
3. If background data requested, encoder sends LPH
4. Encoder sends configured number of LPF frames
5. All frames are CRC-verified by receiver

### Operating Modes {#ENDAT3_OPERATING_MODES}

#### Host Trigger Mode

- Application controls command timing
- Suitable for R5F driven position updates

#### Periodic CMP Trigger Mode

- IEP timer compare event triggers position sampling
- Compare events occur when the IEP timer counter matches the configured compare value
- Enables fixed-rate periodic sampling

**Configuration:**
- Compare event range: CMP0-CMP15 (0-15)
- Mode configured via \ref endat3_set_operating_mode() API
- IEP compare event number set via \ref endat3_config_iep_cmp_event() API
- Event selection can be done in SysConfig
- IEP configuration and CMP event configuration should be done in application. Driver uses above APIs to inform firmware to enable CMP periodic mode and uses the configured CMP event to start sampling periodically.

#### Periodic CAP Trigger Mode

\cond SOC_AM243X
- External signals trigger position sampling through IEP capture events
- The capture event is triggered on the rising edge of the external input pulse, enabling event-driven position capture
- Internal signals can also be mapped to IEP capture events via TIMESYNC/GPIOMUX router
\endcond

\cond (SOC_AM261X || SOC_AM263X || SOC_AM263PX)
- External signals trigger position sampling through IEP capture events
- The capture event is triggered on the rising edge of the external input pulse, enabling event-driven position capture
- Internal signals can also be mapped to IEP capture events via XBAR
\endcond

**Configuration:**
- Capture event range: CAP0-CAP7 (0-7)
- Mode configured via \ref endat3_set_operating_mode() API
- IEP capture event number set via \ref endat3_config_iep_cap_event() API
- Event selection can be done in SysConfig
- IEP configuration and CAP event configuration should be done in application. Driver uses above APIs to inform firmware to enable CAP periodic mode and uses the configured CAP event to start sampling periodically.

\note
    - External signal must be routed to IEP capture input (if needed) in application
    - CAP6 and CAP7 support falling edge detection as well. In EnDAT3, rising edge is used always.

\attention Both IEP event configuration APIs (endat3_config_iep_cmp_event() and endat3_config_iep_cap_event()) are automatically called during endat3_init() with values configured in SysConfig.

## EnDAT3 Design

\subpage ENDAT3_DESIGN explains the design in detail.

## Example

\ref EXAMPLE_MOTORCONTROL_ENDAT3

## API

\ref ENDAT3_API_MODULE

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

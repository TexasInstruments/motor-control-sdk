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
   - Two modes of operation:
     - Host trigger mode: Commands initiated by host controller
     - Periodic trigger mode: Automatic position updates triggered by IEP timer
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

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSS instance
\cond SOC_AM243X
- Selecting the ICSSG PRU slice (Tested on ICSSG0-PRU1)
\endcond
\cond SOC_AM261X
- Selecting the ICSSM PRU slice (Tested on ICSSM1-PRU0)
\endcond
\cond SOC_AM263PX
- Selecting the ICSSM PRU slice (Tested on ICSSM-PRU0)
\endcond
- Configuring PINMUX
- Channel selection (Channel 0, Channel 1, or Channel 2)
- Selecting baud rate (12.5 Mbps - 25 Mbps not currently supported)
- Selecting operating mode (Single channel only)
- Core clock frequency configuration

## ICSS PRU Resource Usage

<table>
<tr>
   <th>Configuration</th>
   <th>PRU Core</th>
   <th>Memory Usage</th>
   <th>IEP Usage</th>
   <th>Other Peripheral Usage</th>
   <th>Description</th>
</tr>
<tr>
   <td>Single channel</td>
   <td>PRUx</td>
   <td>DMEM: 495 Bytes, from offset <code>0x00</code> to <code>0x1EE</code><br>IMEM: 3.56 KB</td>
   <td>IEP0: CMP0 and CMP3</td>
   <td>INTC event/input number 18 (pr[0/1]_pru_mst_intr[2]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F</td>
   <td>IEP, CMP events and INTC signal are used only in periodic continuous mode. Manchester encoding/decoding performed in firmware.</td>
</tr>
</table>

\note For pin usage, see \ref ENDAT3_PIN_USAGE section.

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
   <td>supported in this release</th>
</tr>
<tr>
   <td>25 Mbps</td>
   <td>50 MHz</td>
   <td>200 MHz (8x oversampling)</td>
   <td>not-supported in this release</th>
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
   <td>12.5 Mbps </td>
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
   <td>12.5 Mbps </td>
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
   <td>12.5 Mbps </td>
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

### Operating Modes

**Host Trigger Mode:**
- Application controls command timing
- Suitable for event-driven position updates
- Lower CPU overhead when position updates not needed

**Periodic Trigger Mode:**
- IEP timer automatically triggers position requests
- Deterministic update rate
- Ideal for real-time control loops
- Configurable update period via CMP registers

## EnDAT3 Design

\subpage ENDAT3_DESIGN explains the design in detail.

## Example

\ref EXAMPLE_MOTORCONTROL_ENDAT3

## API

\ref ENDAT3_API_MODULE

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

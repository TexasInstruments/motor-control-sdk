# HDSL {#HDSL}

[TOC]

## Introduction

The HDSL firmware running on ICSS-PRU provides a well-defined interface to execute the HDSL protocol.

\note This implementation using Peripheral input/output mode of PRU-ICSS. Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details.

## Features Supported

- Safe position
- Fast position, speed
- Communication status
- External pulse synchronization using IEP CAP (capture) event
    - 1 to 10 frames per cycle
    - 8 kHz to 50 kHz cycle frequency
    - Uses IEP CAP6 for PRU slice 1 and CAP7 for PRU slice 0 (NOTE: CAP6 and CAP7 support falling edge detection as well. In HDSL, rising edge is used always.)
- Register interface to be compatible with SICK HDSL FPGA IP Core (apart from the differences listed in \ref HDSL_EXCEPTIONS_LIST)
- Parameter channel communication
    - Short message
    - Long message
- Safety
- Pipeline Channel Data
\cond SOC_AM243X
- Single channel support (with PRU Core Clock frequency of 225 MHz) per PRU slice
    - Cores used: PRU0/PRU1
- Multi-channel support (with PRU Core Clock frequency of 300 MHz) per PRU slice
    - Support for multi-channel encoders of different make under load share mode (Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details)
    - Cores used: RTU_PRU0/RTU_PRU1 for channel 0, PRU0/PRU1 for channel 1 and TX_PRU0/TX_PRU1 for channel 2
    - Three channel example on TMDS243EVM and two channel example on LP-AM243 (tested on ICSSG0 instance and PRU1 slice)
\endcond
\cond SOC_AM261X
- Single channel support (with PRU Core Clock frequency of 225 MHz) per PRU
    - Cores used: PRU0/PRU1
    - Two channel example and one channel example on LP-AM261 (tested on ICSSM1 instance and PRU0/PRU1)
\endcond
- Tested with three different encoder makes (EDM35, EKS36, EKM36)

\cond SOC_AM243X
\attention Channel 2 can be enabled only if channel 0 is enabled because of the code overlay scheme needed in TX-PRU. See \ref HDSL_DESIGN_TXPRU_OVERLAY for more details.
\endcond

## Features Not Supported

In general, peripherals or features not mentioned as part of the "Features Supported" section are not
supported, including the below:
 - 100m cable
 - Pipeline Channel Status

## SysConfig Features {#HDSL_SYSCONFIG_FEATURES}

@VAR_SYSCFG_USAGE_NOTE

\cond SOC_AM243X
\attention For each PRU-ICSS slice being used for HDSL, one module instance should be created in SysConfig. For up to 3 channels using 1 slice, only 1 instance needs to be added.

SysConfig can be used to configure the following:
- Selecting the ICSSG instance (Tested on ICSSG0)
- Selecting the ICSSG PRU slice (Tested on ICSSG0-PRU1)
- Configuring PINMUX
- Channel selection
- Selecting multi Channel with encoders of different make using load share mode
- Enabling SA Mux mode
- Mode Selection (Free Run / Sync mode)
- Booster Pack Support: Enable when using BP-AM2BLDCSERVO

\note HDSL firmware supports operation with PRU-ICSS Core Clock running at 225/300 MHz only due to clock divider requirements.

\endcond

\cond SOC_AM261X
\attention For each PRU-ICSS slice being used for HDSL, one module instance should be created in SysConfig. For dual channel example using 2 PRUs, 2 instances need to be added.

SysConfig can be used to configure the following:
- Selecting the ICSSM instance (Tested on ICSSM1)
- Selecting the ICSSM PRU slice (Tested on ICSSM1-PRU0 and ICSSM1-PRU1)
- Configuring PINMUX
- Channel selection
- Mode Selection (Free Run / Sync mode)
- Booster Pack Support: Enable when using BP-AM2BLDCSERVO

\note HDSL firmware supports operation with PRU-ICSS Core Clock running at 225 MHz only due to clock divider requirements.

\endcond

## PRU-ICSS Resource Usage

- Utilizes the Peripheral IF mode (3-channel peripheral interface mode) for HDSL communication. Maximum of 3 channels are available per PRU slice. (Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details)
- Each channel has 4 pins (Clock, Data out, Data in, Output enable)
- Following table contains details of memory usage, IEP usage and interrupt controller usage:

\cond SOC_AM243X

\attention In addition to the following resources used by PRU firmware, SDK examples also configure ICSSG0 IEP1 CMP1 for generating SYNC OUT0 used as sync pulse input.

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
    <td> DMEM: 1773 Bytes: 256B for HDSL Registers (0x0 to 0xFF) + 1517B for LUTs (0x100 to 0x6EC) <br> IMEM: ~ 7.6 kB (For Sync mode. For Free Run mode, less IMEM is used)
    <td> IEP1: CAP7 (slice 0) or CAP6 (slice 1)
    <td> INTC events/inputs numbers 16, 18, 19, 20, 21, 22 (pr[0/1]_pru_mst_intr[0/2/3/4/5/6]_intr_req) are used to trigger interrupts to Arm® Cortex®-R5F for EVENT, V-frame, H-frame, EVENT_S and H-frame respectively </td>
    <td> IEP CAP event is used for external pulse synchronization in Sync mode only
</tr>
<tr>
    <td rowspan="3"> Multi-channel with load share across 3 PRU cores (Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details)
    <td> PRUx
    <td rowspan="3"> DMEM: 5101 Bytes: 3 * 256B for HDSL Registers per channel (0x0 to 0xFF, 0x700 to 0x7FF and 0xE00 to 0xEFF) + 1517B for LUTs (0x100 to 0x6EC) + 2816 Bytes for instructions storage related to TXPRU dynamic overlay (0x1500 to 0x1FFF) <br> IMEM: ~ 7.44 kB (For Sync mode. For Free Run mode, less IMEM is used)
    <td rowspan="3"> IEP1: CAP7 (slice 0) or CAP6 (slice 1) </td>
    <td rowspan="3"> INTC events/inputs numbers 16, 18, 19, 20, 21, 22 (pr[0/1]_pru_mst_intr[0/2/3/4/5/6]_intr_req) are used to trigger interrupts to Arm® Cortex®-R5F for EVENT, V-frame, H-frame, EVENT_S and H-frame respectively </td>
    <td rowspan="3"> IEP CAP event is used for external pulse synchronization in Sync mode only</td>
</tr>
<tr>
    <td> RTU_PRUx
</tr>
<tr>
    <td> TX_PRUx
</tr>
</table>

\note For pin usage see \ref HDSL_PIN_USAGE page.

\endcond

\cond SOC_AM261X

\attention In addition to the following resources used by PRU firmware, SDK examples also configure ICSSM0 IEP0 CMP1 for generating SYNC OUT0 used as sync pulse input.

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
    <td> DMEM: 1773 Bytes: 256B for HDSL Registers (0x0 to 0xFF) + 1517B for LUTs (0x100 to 0x6EC) <br> IMEM: ~ 7.6 kB (For Sync mode. For Free Run mode, less IMEM is used)
    <td> IEP0: CAP7 (slice 0) or CAP6 (slice 1)
    <td> INTC events/inputs numbers 16, 18, 19, 20, 21, 22 (pr[0/1]_pru_mst_intr[0/2/3/4/5/6]_intr_req) are used to trigger interrupts to Arm® Cortex®-R5F for EVENT, V-frame, H-frame, EVENT_S and H-frame respectively </td>
    <td> IEP CAP event is used for external pulse synchronization in Sync mode only
</tr>
</table>
\note For pin usage, see \ref HDSL_PIN_USAGE section.

\endcond
## HDSL Design

\subpage HDSL_DESIGN explains the design in detail.

## Register List

\subpage HDSL_REGISTER_LIST contains the description of registers in TI's HDSL implementation.

## Exceptions

\subpage HDSL_EXCEPTIONS_LIST lists the exceptions in TI's HDSL implementation when compared with SICK HDSL FPGA IP Core. Please note that not all the corresponding register fields are implemented, and see the description of register for more details.

## Datasheet

### Synchronization Pulse Jitter

- Synchronization Pulse Jitter is under 100ns. Please refer to the image below for jitter calculation waveforms.

\image html hdsl_sync_mode_waveforms.png "HDSL Sync mode waveforms for 2 channels"
\image html hdsl_sync_mode_jitter.jpg "HDSL Sync mode jitter analysis"

### Protocol Package Lengths with different ES and Sync Pulse Frequency values

NOTE: Images below show TX_EN signal in "Red" and RX signal in "Yellow".

<table>
<tr>
    <th> ES Value
    <th> Cycle Time (in us)
    <th> Cycle Frequency (in kHz)
    <th> Observed Protocol Package Length (in us)
</tr>
<tr>
    <td> 1
    <td> 25
    <td> 40
    <td> 25.06
</tr>
<tr>
    <td> 1
    <td> 20
    <td> 50
    <td> 19.942
</tr>
<tr>
    <td> 2
    <td> 25
    <td> 40
    <td> Between 12.26 and 12.80
</tr>
<tr>
    <td> 5
    <td> 62.5
    <td> 16
    <td> Between 11.94 and 12.60
</tr>
<tr>
    <td> 10
    <td> 125
    <td> 8
    <td> Between 11.94 and 12.90
</tr>
</table>

## Example

\ref EXAMPLE_MOTORCONTROL_HDSL

## API
\ref HDSL_API_MODULE

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

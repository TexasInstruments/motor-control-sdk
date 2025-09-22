# HDSL {#HDSL}

[TOC]

## Introduction

The HDSL firmware running on ICSS-PRU provides a well-defined interface to execute the HDSL protocol.

\note This implementation using Peripheral input/output mode of PRU-ICSS. Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details.

## Features Supported

- Safe position
- Fast position, speed
- Communication status
- External pulse synchronization
    - 1 to 10 frames per cycle
    - 8 kHz to 50 kHz cycle frequency
- Register interface to be compatible with SICK HDSL FPGA IP Core (apart from the differences listed in \ref HDSL_EXCEPTIONS_LIST)
- Parameter channel communication
    - Short message
    - Long message
- Safety
- Pipeline Channel Data
\cond SOC_AM243X
-  Support for multi-channel encoders of different make under load share mode (Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details)
    - Three channel support on TMDS243EVM and 2 channel support on LP-AM243 (tested on ICSSG0 instance and PRU1 slice).
\endcond
\cond SOC_AM261X
- Single channel support on LP-AM261 (tested on ICSSM1 instance and PRU0 slice).
\endcond
- Tested with three different encoder makes (EDM35, EKS36, EKM36)
\cond SOC_AM243X
\note Channel 2 can be enabled only if channel 0 is enabled because of the code overlay scheme needed in TX-PRU. See \ref HDSL_DESIGN_TXPRU_OVERLAY for more details.
\endcond

## Features Not Supported

In general, peripherals or features not mentioned as part of the "Features Supported" section are not
supported, including the below:
 - 100m cable
 - Pipeline Channel Status

 ## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure the following:
- Selecting the ICSS PRU slice instance (Tested on ICSSG0-PRU1 for AM243x (EVM, LP) and ICSSM1-PRU0 for LP-AM261)
- Configuring PINMUX
- Channel selection
- Mode Selection (Free run/Sync mode)
- Hardware selection (Booster Pack for LP-AM243 and LP-AM261)

## ICSS PRU Resource Usage
\cond SOC_AM243X
<table>
<tr>
    <th> Configuration
    <th> PRU Core
    <th> Memory Usage
    <th> IEP Usage
    <th> Other Peripheral Usage
    <th> Description
</tr>
<tr>
    <td> Single channel
    <td> PRUx
    <td> DMEM: 1773 Bytes: 256B for HDSL Registers (0x0 to 0xFF) + 1517B for LUTs (0x100 to 0x6ED) <br>  IMEM: 7284 Bytes
	<td> IEP1: CMP1
    <td> INTC events/inputs numbers 16, 18, 19, 20, 21, 22 (pr[0/1]_pru_mst_intr[0/2/3/4/5/6]_intr_req) are used to trigger interrupts to Arm® Cortex®-R5F for EVENT, V-frame, H-frame, EVENT_S and H-frame respectively </td>
    <td> IEP, CMP events and INTC signal are used only in periodic continuous mode.
</tr>
<tr>
    <td rowspan="3"> Multi-channel with load share across 3 PRU cores (Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details)
    <td> PRUx
    <td rowspan="3"> DMEM:5101 Bytes: 3 * 256B for HDSL Registers per channel (0x0 to 0xFF, 0x700 to 0x7FF and 0xE00 to 0xEFF) + 1517B for LUTs (0x100 to 0x6ED) + 2816 Bytes for instructions storage related to TXPRU dynamic overlay (0x1500 to 0x1FFF)<br>  IMEM: 7428 Bytes
	<td rowspan="3"> IEP1: CMP1 </td>
    <td rowspan="3"> INTC events/inputs numbers 16, 18, 19, 20, 21, 22 (pr[0/1]_pru_mst_intr[0/2/3/4/5/6]_intr_req) are used to trigger interrupts to R5F for EVENT, V-frame, H-frame, EVENT_S and H-frame respectively </td>
    <td rowspan="3"> IEP, CMP events and INTC signal are used only in periodic continuous mode.</td>
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
<table>
<tr>
    <th> Configuration
    <th> PRU Core
    <th> Memory Usage
    <th> IEP Usage
    <th> Other Peripheral Usage
    <th> Description
</tr>
<tr>
    <td> Single channel
    <td> PRUx
    <td> DMEM: 1773 Bytes: 256B for HDSL Registers (0x0 to 0xFF) + 1517B for LUTs (0x100 to 0x6ED) <br>  IMEM: ~7.4 kB (Sync Mode), ~6.9 kB (Free Run Mode)
	<td> IEP0: CMP1
    <td> INTC event/input numbers 16, 18, 19, 20, 21, 22 (pr[0/1]_pru_mst_intr[0/2/3/4/5/6]_intr_req) are used to trigger interrupt to R5F for EVENT, V-frame, H-frame, EVENT_S and H-frame respectively </td>
    <td> IEP, CMP events and INTC signal are used only in periodic continuous mode.
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

# HDSL {#HDSL}

[TOC]

## Introduction

The HDSL firmware running on ICSS-PRU provides a defined well interface to execute the HDSL protocol.

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
- Three channel support using single PRU-ICSSG slice
	- Three channel support on am243x-evm
	- Two channel support on am243x-lp
- Tested with three different encoder makes (EDM35, EKS36, EKM36)

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported, including the below
 - 100m cable
 - Pipeline Channel Status

 ## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSSG0PRUx instance.(Tested on ICSSG0-PRU1)
- Configuring PINMUX
- Channel selection
- Mode Selection (Free run/Sync mode)
- Hardware selection (Booster Pack for am243x-lp)

## HDSL Design

\subpage HDSL_DESIGN explains the design in detail.

## Register List

\subpage HDSL_REGISTER_LIST contains the description of registers in TI's HDSL implementation. Please note that all the corresponding register fields are not implemented.

## Exceptions

\subpage HDSL_EXCEPTIONS_LIST lists the exceptions TI's HDSL implementation when compared with SICK HDSL FPGA IP Core. Please note that all the corresponding register fields are not implemented.

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
# Tamagawa {#TAMAGAWA}

[TOC]

## Introduction

The Tamagawa receiver firmware running on PRU-ICSS provides a well-defined interface to execute the Tamagawa encoder communication protocol. The Tamagawa diagnostic application interacts with the Tamagawa receiver firmware interface.

\note This implementation using Peripheral input/output mode of PRU-ICSS. Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details.

## Features Supported

-  Supports full-absolute SmartAbs & SmartInc encoders compatible with Smartceiver AU5561N1
-  Channel selection
-  Baud rate selection
-  Supports all Data Readout, Reset and EEPROM commands
\cond SOC_AM243X || SOC_AM64X
-  Support for concurrent multi-channel support on a single PRU (up to 3 identical encoders)
    - In this mode, data transmission and reception must happen simultaneously on all channels.
    - The encoder configuration and cable length should be the same on all channels.
    - If encoders across channels don't respond at the same time, this mode will not work.
\endcond
-  2.5 Mbps and 5 Mbps encoder support
   \note In three channel interface of PRU-ICSS, receive (Rx) is oversampled at 8x of send (Tx). Therefore, the encoder interface frequency "f" should be such that Tx source clock value is divisible by "f" and Rx source clock value is divisible by "(8*f)".
- Possible interface speeds with different source clock combinations.
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

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure the following:
\cond (SOC_AM263X || SOC_AM263PX)
- Selecting the ICSSM PRU slice (Tested on ICSSM-PRU0)
- Configuring PINMUX and GPIO
\endcond
\if SOC_AM261X
- Selecting the ICSSM PRU slice (Tested on ICSSM1-PRU0)
- Configuring PINMUX and GPIO
\else
- Selecting the ICSSG instance
- Selecting the ICSSG PRU slice (Tested on ICSSG0-PRU1)
- Configuring PINMUX, GPIO and ICSS clock to 200 MHz
- Enabling SA Mux mode
\endif
- Channel selection
- Baud rate selection
- Selecting RX and TX source clock

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
   <td>Single channel </td>
   <td>PRUx</td>
   <td>DMEM: 220 Bytes, from offset <code>0x00</code> to <code>0xDC</code> offset <br> IMEM: 1.8 KB</td>
   <td>IEP0: CMP0 and CMP3 </td>
   <td>INTC event/input number 18 (pr[0/1]_pru_mst_intr[2]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F </td>
   <td>IEP, CMP events and INTC signal are used only in periodic continuous mode</td>
</tr>
\cond SOC_AM243X || SOC_AM64X
<tr>
   <td>Multi-channel with single PRU core</td>
   <td>PRUx</td>
   <td>DMEM: 220 Bytes, from offset <code>0x00</code> to <code>0xDC</code> offset <br> IMEM: 1.5 KB</td>
   <td>IEP0: CMP0 and CMP3 </td>
   <td>INTC event/input number 18 (pr[0/1]_pru_mst_intr[2]_intr_req) is used to trigger interrupt to R5F </td>
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

# Tamagawa {#TAMAGAWA}

[TOC]

## Introduction

The Tamagawa receiver firmware running on PRU-ICSS provides a defined well interface to execute the Tamagawa protocol. The Tamagawa diagnostic application interacts with the Tamagawa receiver firmware interface.

\note
Tamagawa firmware and examples are based on 3 Channel Peripheral interface from \if ( SOC_AM263X || SOC_AM261X)  PRU-ICSSM \else PRU-ICSSG \endif.

## Features Supported

-  Supports full-absolute SmartAbs & SmartInc encoders compatible with Smartceiver AU5561N1
-  Channel selection
-  Baud rate selection
-  Supports all Data Readout, Reset and EEPROM commands
-  2.5 Mbps and 5 Mbps encoder support
   \note Receive (Rx) is oversampled at 8x of send(Tx). Therefore, the encoder interface frequency "f" should such that Tx source clock value is divisible by "f" and Rx source clock value is divisible by "(8*f)".
- Possible interface speeds with different source clock combinations.
<table>
<tr>
    <th>Clock Source
    <th>Interface Speed
</tr>
\cond SOC_AM261X
<tr>
    <td> PRU UART Clock (160 MHz)
    <td> 2.5MHz, 5MHz
</tr>
<tr>
    <td>PRU Core Clock (200 MHz)
    <td> 2.5MHz, 5MHz
</tr>
\endcond
\cond (SOC_AM263X || SOC_AM263PX)
<tr>
    <td> PRU UART Clock (160 MHz)
    <td> 2.5MHz, 5MHz
</tr>
<tr>
    <td>PRU Core Clock (200 MHz)
    <td> 2.5MHz, 5MHz
</tr>
\endcond
\cond SOC_AM243X || SOC_AM64X
<tr>
    <td>PRU Core Clock (200 MHz)
    <td> 2.5MHz, 5MHz
</tr>
\endcond

</table>

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not supported, including the below
-  Other baud rates.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
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
- Configuring PINMUX, GPIO and ICSS clock to 200MHz
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
   <td>Single Channel Configuration</td>
   <td>PRUx</td>
   <td>DMEM: 220 Bytes <br> IMEM: 1.8 KB</td>
   <td>IEP0: CMP0 and CMP3 </td>
   <td>INTC Signal No. 18 is used to trigger a R5 interrupt </td>
   <td>IEP, CMP events and INTC signal are used only in periodic continuous mode</td>
</tr>
\cond SOC_AM243X || SOC_AM64X
<tr>
   <td>Multi Channel Configuration with single PRU core</td>
   <td>PRUx</td>
   <td>DMEM: 220 Bytes <br> IMEM: 1.5 KB</td>
   <td>IEP0: CMP0 and CMP3 </td>
   <td>INTC Signal No. 18 is used to trigger a R5 interrupt</td>
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

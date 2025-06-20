# Nikon A-Format {#NIKON}

[TOC]

\note A-Format® is a registered trademark of the Nikon Corporation.

## Introduction

Nikon A-Format absolute encoder receiver implementation on the TI PRU-ICSS interfaces with the Nikon A-Format encoders either via point-to-point communication or up to 8 Nikon A-Format encoders connected on a bus. Nikon A-Format encoders use a proprietary asynchronous serial bi-directional half-duplex communication protocol compliant with the RS-485. The encoder can generate either single- or multi-turn absolute position data and can operate at a baud rate of up to 16 MHz. Besides position data, the encoder can also send status and diagnostic information well. Reliability is ensured by using a 3-bit CRC by the receiver and an 8-bit CRC by the encoder. Another interesting feature of the Nikon-A encoders is the optional battery backup circuit, which can be used in case of power failure. Nikon A-Format encoders are widely used in industrial machinery, including industrial robots in automobile production lines and machine tools, and are highly valued as the next-generation standard for sensors that can detect the absolute rotational displacement values of robot arms.

## Features Supported {#NIKON_FEATURES}
   -  Support for Nikon version 2.1 and Nikon version 3.0
   -  Support for point-to-point and bus communication (up to 8 encoders).
   -  Support for baud rates from 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz.
\cond SOC_AM243X
   -  Support for oversampling ratio with different baud rates.
<table>
<tr>
    <th rowspan="2">Clock Source
    <th rowspan="1" colspan="5">Interface Speed
</tr>
<tr>
    <th>2.5 MHz
    <th>4 MHz
    <th>6.67 MHz
    <th>8 MHz
    <th>16 MHz
</tr>
<tr>
    <td>PRU UART Clock (192 MHz)
    <td>Not tested
	<td>8x
    <td>Not tested
	<td>8x
    <td>8x with fractional div
</tr>
<tr>
    <td>PRU Core Clock (200 MHz)
    <td>8x
	<td>Not tested
    <td>6x
	<td>Not tested
    <td>Not tested
</tr>
<tr>
    <td>PRU Core Clock (300 MHz)
    <td>8x
	<td>Not tested
    <td>6x with fractional div
	<td>Not tested
    <td>Not tested
</tr>
</table>
   -  Support for concurrent multi-channel support on a single PRU (up-to 3 channels with identical number of encoders of same frequency connected to all configured channels).
   -  Support for multi-channel with different communication modes and different number of encoders connected across channels under load share model (each of PRU, RTU-PRU, and TX-PRU from one PRU-ICSSG slice handles all 3 channels).
\endcond

\cond SOC_AM261X
   -  Support for oversampling ratio with different baud rates.
<table>
<tr>
    <th rowspan="2">Clock Source
    <th rowspan="1" colspan="5">Interface Speed
</tr>
<tr>
    <th>2.5 MHz
    <th>4 MHz
    <th>6.67 MHz
    <th>8 MHz
    <th>16 MHz
</tr>
<tr>
    <td>PRU UART Clock (160 MHz)
    <td>8x
	<td>8x
    <td>8x
	<td>4x
    <td>5x
</tr>
</table>

\endcond

\cond (SOC_AM263X || SOC_AM263PX)
   -  Support for oversampling ratio with different baud rates.
<table>
<tr>
    <th rowspan="2">Clock Source
    <th rowspan="1" colspan="5">Interface Speed
</tr>
<tr>
    <th>2.5 MHz
    <th>4 MHz
    <th>6.67 MHz
    <th>8 MHz
    <th>16 MHz
</tr>
<tr>
    <td>PRU UART Clock (192 MHz)
    <td>Not tested
	<td>8x
    <td>Not tested
	<td>8x
    <td>8x with fractional div
</tr>
<tr>
    <td>PRU Core Clock (200 MHz)
    <td>8x
	<td>Not tested
    <td>6x
	<td>Not tested
    <td>Not tested
</tr>
</table>
   -  Support for up to 40-bit absolute position (single turn + multi turn) data with additional information.
   -  Support for position preset, temperature information and alarms.
   -  Support for non-volatile (EEPROM) read and write.
   -  Support for identification code read and write.
   -  Support for encoder address setting.
   -  Support for individual and multiple transmission mode with encoder addresses ranging between ENC1-ENC8.
   -  Nikon A-Format version 3.0 specific supported features:
        -  Support for velocity and acceleration data.
        -  Support for velocity coefficient read and write.
        -  Support for non-volatile (EEPROM) read and write access with bank.
        -  Support for factory setting modes which change the command response behavior:
            - CMD1/CMD5 for reading ABS 40-bit data and velocity.
            - CMD8/CMD9/CMD10/CMD11/CMD12 for reading lower ABS 24-bit data.
\endcond

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported in this release, including the below
-  Independent clocks on multi channel mode.

### Known Limitations

\cond SOC_AM243X
- PRU Firmware gets stuck if encoder does not respond with number of bytes expected by driver as per PINDSW-9179 in \ref RELEASE_NOTES_11_00_00_PAGE
\endcond

\cond SOC_AM261X
- PRU Firmware gets stuck if encoder does not respond with number of bytes expected by driver as per PINDSW-9179 in \ref RELEASE_NOTES_10_02_00_PAGE
\endcond

 - Example cases when firmware gets stuck
    1. Any command is sent with encoder address not matching to that of encoder connected with the device
    2. In bus mode, if encoders with address 0, 1, 2 are connected and MT command is sent with encoder address 3 or more
    3. Command 20 is sent with ID not matching to encoder connected with the device

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

\cond SOC_AM243X

SysConfig can be used to configure things mentioned below:
- Selecting the ICSSG instance. (Tested on ICSSG0)
- Selecting the ICSSG0PRUx instance.(Tested on ICSSG0-PRU1)
- Configuring PINMUX.
- Frequency selection.
- Channel selection.
- Selecting Multi Channel with Encoders connected in Different Configurations (bus or one-to-one) using load share mode.
- Enabling SA Mux mode.
- Selecting source clock.
- Selecting Nikon protocol version (2.1 or 3.0)

\endcond

\cond SOC_AM261X

SysConfig can be used to configure things mentioned below:
- Selecting the ICSS instance. (Tested on ICSSM1)
- Selecting the ICSS PRU slice.(Tested on ICSSM1-PRU0)
- Configuring PINMUX.
- Frequency selection.
- Channel selection.
- Selecting source clock.
- Selecting Nikon protocol version (2.1 or 3.0)

\note Nikon firmware will only run with ICSS UART Clock running at 160 MHz(when ICSS Core Clock is 225 MHz).

\endcond

\cond  (SOC_AM263X || SOC_AM263PX)

SysConfig can be used to configure things mentioned below:
- Selecting the ICSS instance. (Tested on ICSSM)
- Selecting the ICSS PRU slice.(Tested on ICSSM-PRU0)
- Configuring PINMUX.
- Frequency selection.
- Channel selection.
- Selecting source clock.
- Selecting Nikon protocol version (2.1 or 3.0)

\endcond

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
    <td> Single Channel Mode
    <td> PRUx
    <td> DMEM: 552 Bytes <br>  IMEM: ~ 1.4 kB
	<td> IEP0: CMP0 and CMP3
    <td> INTC Signal host interrupt 2 is used to trigger a R5 interrupt
    <td> IEP, CMP events and INTC signal are used only in periodic continuous mode.
</tr>
<tr>
    <td> Multi Channel Single PRU Mode
    <td> PRUx
    <td> DMEM: 552 Bytes <br>  IMEM: 1700 Bytes
	<td> IEP0: CMP0 and CMP3
    <td> INTC Signal host interrupt 2 is used to trigger a R5 interrupt
    <td> IEP, CMP events and INTC signal are used only in periodic continuous mode.
</tr>
<tr>
    <td rowspan="3"> Multi Channel Load Share Mode
    <td> PRUx
    <td rowspan="3"> DMEM: 552 Bytes <br>  IMEM: 1604 Bytes
	<td rowspan="3">IEP0: CMP0, CMP3, CMP5 and CMP6 </td>
    <td rowspan="3">INTC Signal host interrupt 2,3 & 4 is used to trigger a R5 interrupt</td>
    <td rowspan="3">IEP, CMP events and INTC signals are used only in periodic continuous mode.</td>
</tr>
<tr>
    <td> RTU_PRUx
</tr>
<tr>
    <td> TX_PRUx
</tr>
</table>

\note For pin usage see \ref NIKON_PIN_USAGE  page.

\endcond

\cond (SOC_AM261X || SOC_AM263X || SOC_AM263PX)
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
    <td> Single Channel Mode
    <td> PRUx
    <td> DMEM: 552 Bytes <br>  IMEM: ~ 1.4 kB
	<td> IEP0: CMP0 and CMP3
    <td> INTC Signal host interrupt 2 is used to trigger a R5 interrupt
    <td> IEP, CMP events and INTC signal are used only in periodic continuous mode.
</tr>
</table>
\note For pin usage, see \ref NIKON_PIN_USAGE section.

\endcond

## NIKON Design

\subpage NIKON_DESIGN explains the design in detail.

## Example
\ref EXAMPLE_MOTORCONTROL_NIKON

## API
\ref NIKON_API_MODULE


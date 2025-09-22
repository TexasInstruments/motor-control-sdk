# Nikon A-format {#NIKON}

[TOC]

\note A-format® is a registered trademark of the Nikon Corporation.

## Introduction

Nikon A-format absolute encoder receiver implementation on the TI PRU-ICSS interfaces with the Nikon A-format encoders either via point-to-point communication or up to 8 Nikon A-format encoders connected on a bus. Nikon A-format encoders use a proprietary asynchronous serial bi-directional half-duplex communication protocol compliant with RS-485. The encoder can generate either single- or multi-turn absolute position data and can operate at a baud rate of up to 16 MHz. Besides position data, the encoder can also send status and diagnostic information as well. Reliability is ensured by using a 3-bit CRC by the receiver and an 8-bit CRC by the encoder. Another interesting feature of the Nikon-A encoders is the optional battery backup circuit, which can be used in case of power failure. Nikon A-format encoders are widely used in industrial machinery, including industrial robots in automobile production lines and machine tools, and are highly valued as the next-generation standard for sensors that can detect the absolute rotational displacement values of robot arms.

\note This implementation using Peripheral input/output mode of PRU-ICSS. Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details.

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
   -  Support for concurrent multi-channel support on a single PRU (up to 3 channels with identical number of encoders of the same frequency connected to all configured channels).
       - In this mode, data transmission and reception must happen simultaneously on all channels.
       - The encoder configuration and cable length should be the same on all channels.
       - If encoders across channels don't respond at the same time, this mode will not work. Load share configuration should be used instead.
   -  Support for multi-channel with different communication modes and different numbers of encoders connected across channels under load share mode (Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details).
       - In this mode, data transmission and reception can happen independently on all channels.
       - After a command is sent, all channels wait for a response and process the response independently. However, all channels must finish processing before the next command can be triggered.
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
   -  Nikon A-format version 3.0 specific supported features:
        -  Support for velocity and acceleration data.
        -  Support for velocity coefficient read and write.
        -  Support for non-volatile (EEPROM) read and write access with bank.
        -  Support for factory setting modes which change the command response behavior:
            - CMD1/CMD5 for reading ABS 40-bit data and velocity.
            - CMD8/CMD9/CMD10/CMD11/CMD12 for reading lower ABS 24-bit data.
\endcond

## Features Not Supported

In general, peripherals or features not mentioned as part of the "Features Supported" section are not
supported in this release, including the following:
-  Independent clocks on multi channel mode.

### Known Limitations

-    Limitation with <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a> boosterpack with TI LaunchPad
    - Hardware Limitation with BP-AM2BLDCSERVO
        - Only up to 7 encoders in bus connection have been tested with BP-AM2BLDCSERVO
        - This limitation is due to insufficient voltage when attempting to power 8 encoders
        - Attempting to connect 8 encoders may result in unreliable operation

    - Software Support
        - The example code and PRU firmware are designed to handle 8 encoders

\cond SOC_AM243X
- PRU Firmware gets stuck if encoder does not respond with the number of bytes expected by the driver as per PINDSW-9179 in \ref RELEASE_NOTES_11_00_00_PAGE
 - Example cases when firmware gets stuck
    1. Any command is sent with encoder address not matching that of the encoder connected with the device
    2. In bus mode, if encoders with addresses 0, 1, 2 are connected and MT command is sent with encoder address 3 or more
    3. Command 20 is sent with ID not matching the encoder connected with the device
\endcond

\cond (SOC_AM263PX || SOC_AM261X)
- PRU Firmware gets stuck if encoder does not respond with the number of bytes expected by the driver as per PINDSW-9179 in \ref RELEASE_NOTES_10_02_00_PAGE
 - Example cases when firmware gets stuck
    1. Any command is sent with encoder address not matching that of the encoder connected with the device
    2. In bus mode, if encoders with addresses 0, 1, 2 are connected and MT command is sent with encoder address 3 or more
    3. Command 20 is sent with ID not matching the encoder connected with the device
\endcond

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

\cond SOC_AM243X

SysConfig can be used to configure the following:
- Selecting the ICSSG instance. (Tested on ICSSG0)
- Selecting the ICSSG0PRUx instance. (Tested on ICSSG0-PRU1)
- Configuring PINMUX.
- Frequency selection.
- Channel selection.
- Selecting Multi Channel with encoders connected in Different Configurations (bus or one-to-one) using load share mode.
- Enabling SA Mux mode.
- Selecting source clock.
- Selecting Nikon protocol version (2.1 or 3.0)

\note Nikon firmware supports operation with ICSS Core Clock running at 200 MHz/300 MHz frequency or ICSS UART Clock running at 192 MHz only. ICSS Core Clock at 225/250/333 MHz is not supported due to clock divider requirements.

\endcond

\cond SOC_AM261X

SysConfig can be used to configure the following:
- Selecting the ICSS instance. (Tested on ICSSM1)
- Selecting the ICSS PRU slice. (Tested on ICSSM1-PRU0)
- Configuring PINMUX.
- Frequency selection.
- Channel selection.
- Selecting source clock.
- Selecting Nikon protocol version (2.1 or 3.0)

\note Nikon firmware supports operation with ICSS UART Clock running at 160 MHz only, when ICSS Core Clock is 225 MHz due to clock divider requirements.

\endcond

\cond  (SOC_AM263X || SOC_AM263PX)

SysConfig can be used to configure the following:
- Selecting the ICSS instance. (Tested on ICSSM)
- Selecting the ICSS PRU slice. (Tested on ICSSM-PRU0)
- Configuring PINMUX.
- Frequency selection.
- Channel selection.
- Selecting source clock.
- Selecting Nikon protocol version (2.1 or 3.0)

\note Nikon firmware supports operation with ICSS Core Clock running at 200 MHz frequency or ICSS UART Clock running at 192 MHz frequency only.

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
    <td> Single channel
    <td> PRUx
    <td> DMEM: 549 Bytes (0x0 to 0x225) <br>  IMEM: ~ 1.4 kB
	<td> IEP0: CMP0 and CMP3
    <td> INTC event/input number 18 (pr[0/1]_pru_mst_intr[2]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F
    <td> IEP, CMP events and INTC signal are used only in periodic continuous mode.
</tr>
<tr>
    <td> Multi-channel with single PRU core
    <td> PRUx
    <td> DMEM: 549 Bytes (0x0 to 0x225) <br>  IMEM: 1700 Bytes
	<td> IEP0: CMP0 and CMP3
    <td> INTC event/input number 18 (pr[0/1]_pru_mst_intr[2]_intr_req) is used to trigger interrupt to R5F
    <td> IEP, CMP events and INTC signal are used only in periodic continuous mode.
</tr>
<tr>
    <td rowspan="3"> Multi-channel with load share across 3 PRU cores
    <td> PRUx
    <td rowspan="3"> DMEM: 549 Bytes (0x0 to 0x225) <br>  IMEM: 1604 Bytes
	<td rowspan="3"> IEP0: CMP0, CMP3, CMP5 and CMP6 </td>
    <td rowspan="3"> INTC events/inputs number 18, 19 and 20 (pr[0/1]_pru_mst_intr[2/3/4]_intr_req) are used to trigger interrupts to R5F
    <td rowspan="3"> IEP, CMP events and INTC signals are used only in periodic continuous mode.</td>
</tr>
<tr>
    <td> RTU_PRUx
</tr>
<tr>
    <td> TX_PRUx
</tr>
</table>

\note For pin usage, see \ref NIKON_PIN_USAGE page.

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
    <td> Single channel
    <td> PRUx
    <td> DMEM: 549 Bytes (0x0 to 0x225) <br>  IMEM: ~ 1.4 kB
	<td> IEP0: CMP0 and CMP3
    <td> INTC event/input number 18 (pr[0/1]_pru_mst_intr[2]_intr_req) is used to trigger interrupt to R5F
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

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

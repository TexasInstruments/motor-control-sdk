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
   -  Support for concurrent multi-channel support on a single PRU
       - Up to 3 channels with identical number of encoders of the same frequency connected to all configured channels.
       - In this mode, data transmission and reception must happen simultaneously on all channels.
       - The encoder configuration and cable length should be the same on all channels.
       - If encoders across channels don't respond at the same time, this mode will not work. Load share configuration should be used instead.
   -  Support for multi-channel with encoders of different make and different numbers of encoders connected across channels under load share mode (Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details).
       - Up to 3 channels with encoders of the same frequency connected to all configured channels.
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
\endcond

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
   - Support for periodic trigger using PRU-ICSS IEP timer module
   - Same clock frequency for all channels in multi channel mode within the same PRU-ICSS slice
        - Different PRU slices can simultaneously handle encoders operating at different frequencies

## Features Not Supported

In general, peripherals or features not mentioned as part of the "Features Supported" section are not
supported in this release, including the following:
-  Independent clock frequency for each channel in multi channel mode within the same PRU-ICSS slice
    - Clock frequency is a PRU-ICSS slice level configuration
    - Each channel within the same PRU-ICSS slice in multi-channel mode will have same clock frequencies

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

## SysConfig Features {#NIKON_SYSCONFIG_FEATURES}

@VAR_SYSCFG_USAGE_NOTE

\cond SOC_AM243X

\attention For each PRU-ICSS slice being used for Nikon, one module instance should be created in SysConfig. For up to 3 channels using 1 slice, only 1 instance needs to be added.

SysConfig can be used to configure the following:
- Selecting the ICSSG instance. (Tested on ICSSG0)
- Selecting the ICSSG0PRUx instance. (Tested on ICSSG0-PRU1)
- Configuring PINMUX
- Frequency selection
- Channel selection
- Selecting Multi Channel with encoders connected in Different Configurations (bus or one-to-one) using load share mode.
- Enabling SA Mux mode.
- Selecting source clock.
- Selecting Nikon protocol version (2.1 or 3.0)
- IEP instance and IEP event selection for periodic mode

\note Nikon firmware supports operation with ICSS Core Clock running at 200 MHz/300 MHz frequency or ICSS UART Clock running at 192 MHz only. ICSS Core Clock at 225/250/333 MHz is not supported due to clock divider requirements.

\endcond

\cond SOC_AM261X

\attention For each PRU-ICSS slice being used for Nikon, one module instance should be created in SysConfig. For 2 channel example using 2 PRUs, 2 instances need to be added.

SysConfig can be used to configure the following:
- Selecting the ICSS instance. (Tested on ICSSM1)
- Selecting the ICSS PRU slice. (Tested on ICSSM1-PRU0)
- Configuring PINMUX
- Frequency selection
- Channel selection
- Selecting source clock.
- Selecting Nikon protocol version (2.1 or 3.0)
- IEP event selection for periodic mode

\note Nikon firmware supports operation with ICSS UART Clock running at 160 MHz only, when ICSS Core Clock is 225 MHz due to clock divider requirements.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

\attention For each PRU-ICSS slice being used for Nikon, one module instance should be created in SysConfig.

SysConfig can be used to configure the following:
- Selecting the ICSS instance. (Tested on ICSSM)
- Selecting the ICSS PRU slice. (Tested on ICSSM-PRU0)
- Configuring PINMUX
- Frequency selection
- Channel selection
- Selecting source clock.
- Selecting Nikon protocol version (2.1 or 3.0)
- IEP event selection for periodic mode

\note Nikon firmware supports operation with ICSS Core Clock running at 200 MHz frequency or ICSS UART Clock running at 192 MHz frequency only.

\endcond

## Periodic Trigger Modes {#NIKON_PERIODIC_MODES}

The Nikon driver supports two types of periodic trigger modes for continuous position sampling:

### CMP Mode (Compare Event Mode)
In CMP mode, IEP timer compare events trigger position sampling based on compare events. Compare event occurs when IEP timer hits the configured compare value. This is useful for applications requiring position sampling at regular intervals.

### CAP Mode (Capture Event Mode)
\cond SOC_AM243X
In CAP mode, external signals trigger position sampling based on IEP capture events. Capture event is triggered on rising edge of the input pulse. This is useful for synchronizing position capture with external inputs. Internal signals can also be mapped to IEP capture events via TIMESYNC/GPIOMUX router.
\endcond

\cond (SOC_AM261X || SOC_AM263X || SOC_AM263PX)
In CAP mode, external signals trigger position sampling based on IEP capture events. Capture event is triggered on rising edge of the input pulse. This is useful for synchronizing position capture with external inputs. Internal signals can also be mapped to IEP capture events via XBAR.
\endcond

\note CAP6 and CAP7 support falling edge detection as well. In Nikon, rising edge is used always.

## PRU-ICSS Resource Usage

- Utilizes the Peripheral IF mode (3-channel peripheral interface mode) for Nikon communication. Maximum of 3 channels are available per PRU slice. (Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details)
- Each channel has 4 pins (Clock, Data out, Data in, Output enable)
- Following table contains details of memory usage, IEP usage and interrupt controller usage:

\cond SOC_AM243X

\attention In addition to following resources used by PRU firmwares, SDK examples also configure IEPx CMP0 for IEP counter reset in periodic trigger CMP mode and IEPx CMP1 for generating SYNC OUT0 used as input to CAP in periodic trigger CAP mode.

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
    <td> DMEM: 580 Bytes (0x0 to 0x243) <br>IMEM: ~ 1.4 kB
	<td> <b>CMP Mode:</b> IEPx CMPy for trigger (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger (IEPx and CAPy selected in SysConfig)
    <td> INTC event/input number 18 or 21 (prx_pru_mst_intr[2/3]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F based on slice
    <td> IEP events and INTC signals are used only in periodic trigger modes
</tr>
<tr>
    <td> Multi-channel with single PRU core
    <td> PRUx
    <td> DMEM: 580 Bytes (0x0 to 0x243) <br>IMEM: ~ 1.73 kB
	<td> <b>CMP Mode:</b> IEPx CMPy for trigger (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger (IEPx and CAPy selected in SysConfig)
    <td> INTC event/input number 18 or 21 (prx_pru_mst_intr[2/3]_intr_req) is used to trigger interrupt to R5F based on slice
    <td> IEP events and INTC signals are used only in periodic trigger modes
</tr>
<tr>
    <td rowspan="3"> Multi-channel with load share across 3 PRU cores
    <td> PRUx
    <td rowspan="3"> DMEM: 580 Bytes (0x0 to 0x243) <br>IMEM: ~ 1.6 kB
	<td rowspan="3"> <b>CMP Mode:</b> IEPx CMPy for trigger per channel (IEPx and CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEPx CAPy for trigger per channel (IEPx and CAPy selected in SysConfig)
    <td rowspan="3"> INTC events/inputs number 18, 19, 20 or 21, 22, 23(prx_pru_mst_intr[2/3/4/5/6/7]_intr_req) are used to trigger interrupts to R5F
    <td rowspan="3"> IEP events and INTC signals are used only in periodic trigger modes</td>
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

\attention In addition to following resources used by PRU firmwares, SDK examples also configure IEP0 CMP0 for IEP counter reset in periodic trigger CMP mode.

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
    <td> DMEM: 580 Bytes (0x0 to 0x243) <br>IMEM: ~ 1.4 kB
	<td> <b>CMP Mode:</b> IEP0 CMPy for trigger (IEP0 CMPy selected in SysConfig)<br><b>CAP Mode:</b> IEP0 CAPy for trigger (IEP0 CAPy selected in SysConfig)
    <td> INTC event/input number 18 or 21 (prx_pru_mst_intr[2/3]_intr_req) is used to trigger interrupt to Arm® Cortex®-R5F based on slice
    <td> IEP events and INTC signals are used only in periodic trigger modes
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

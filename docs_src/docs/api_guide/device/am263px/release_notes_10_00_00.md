# Release Notes 10.00.00 {#RELEASE_NOTES_10_00_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention For release notes of Industrial Communications SDK and MCU+ SDK, please refer to <a href="@VAR_IC_SDK_DOCS_PATH/RELEASE_NOTES_10_00_00_PAGE.html" target="_blank"> @VAR_SOC_NAME Industrial Communications SDK Release Notes 10.00.00</a> and <a href="@VAR_MCU_SDK_DOCS_PATH/RELEASE_NOTES_10_00_00_PAGE.html" target="_blank"> @VAR_SOC_NAME MCU+ SDK Release Notes 10.00.00</a> respectively.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
Tamagawa over PRU 3-channel interface                                                           | Position Sense Tamagawa
Tamagawa over SoC UART                                                                          | Position Sense Tamagawa
Multiple baud rate support : 2.5 Mbps, and 5 Mbps                                               | Position Sense Tamagawa
EnDat 2.2                                                                                       | Position Sense EnDat
Nikon A-Format Single Channel                                                                   | Position Sense Nikon A-Format
Multiple baud rate support : 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz                        | Position Sense Nikon A-Format
BiSS-C Single Channel                                                                           | Position Sense BiSS-C
Multiple baud rate support : 1 MHz, 2 MHz, 5 MHz, 8 MHz, and 10 MHz                             | Position Sense BiSS-C
Digital Control Library and Examples                                                            | Real Time Libraries
Transforms Library and Example                                                                  | Real Time Libraries
SFRA Library and Example                                                                        | Real Time Libraries
Datalog Library and Example                                                                     | Real Time Libraries
Control Library                                                                                 | Real Time Libraries
Observer Library                                                                                | Real Time Libraries


## Device and Validation Information

SOC    | Supported CPUs  | EVM                                                                          | Host PC
-------|-----------------|------------------------------------------------------------------------------|-----------------------------------------
AM263Px| R5F             | AM263Px ControlCard E2 Rev     (referred to as am263px-cc in code). \n       | Windows 10 64b or Ubuntu 18.04 64b
AM263Px| R5F             | AM263Px LaunchPad              (referred to as am263px-lp in code). \n       | Windows 10 64b or Ubuntu 18.04 64b

## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | 12.8.0
SysConfig               | R5F            | 1.21.0 build, build 3721
TI ARM CLANG            | R5F            | 4.0.0.LTS
FreeRTOS Kernel         | R5F            | 10.4.3
LwIP                    | R5F            | STABLE-2_2_0_RELEASE
Mbed-TLS                | R5F            | mbedtls-3.0.0

## Key Features

### Position Sense

<table>
<tr>
    <th> Module
    <th> Supported CPUs
    <th> SysConfig Support
    <th> OS Support
    <th> Key features tested
    <th> Key features not tested
</tr>
<tr>
    <td> BiSS-C
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, point-to-point connection,control communication, automatic processing delay detection and compensation, Interface speed of 1, 2, 5, 8, and 10 MHz, Boosterpack with LP-AM261
    <td> Multi Transmission Mode, Long cable (upto 100 meters)
</tr>
<tr>
    <td> EnDat
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Continuous mode, Recovery Time for 2.2 command set, Long cable (upto 100 meters), Boosterpack with LP-AM263P
    <td> Encoder receive communication command
</tr>
<tr>
    <td> Nikon A-Format
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, point-to-point connection, baud rates from 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz, up to 40-bit absolute position (single turn + multi turn) data with additional information, Boosterpack with LP-AM261
    <td> Daisy Chain Testing, Long cable (upto 100 meters)
</tr>
<tr>
    <td> Tamagawa over PRU 3-channel interface 
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Absolute position, Encoder ID, Reset, EEPROM Read, EEPROM Write, 2.5 Mbps Encoder, Boosterpack with LP-AM263P
    <td> 5 Mbps Encoder
</tr>
<tr>
    <td> Tamagawa over SOC UART
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Absolute position, Encoder ID, Reset, EEPROM Read, EEPROM Write, 2.5 Mbps Encoder
    <td> 5 Mbps Encoder
</tr>
</table>

### Real Time Libraries

<table>
<tr>
    <th> Module
    <th> Supported CPUs
    <th> SysConfig Support
    <th> OS Support
    <th> Key features tested
    <th> Key features not tested
</tr>
<tr>
    <td> Control
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Field Weakening Control, Maximum Torque Per Ampere, Strator voltage frequency generator support
    <td> -
</tr>
<tr>
    <td> Digital Control Library (DCL)
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Linear PI, Linear PID, Linear PI with double integrator (PI2), Direct Form 1 (first order), Direct Form 1 (second order), Direct Form 1 (third order), Direct Form 2 (second order), Direct Form 2 (third order), Non-linear PID controller
    <td> -
</tr>
<tr>
    <td> Observer
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Sensored eQEP-based encoder, Hall sensor, Sensorless Enhanced Sliding Mode Observer, both speed measurement for sensored (speedcalc) and sensorless (speedfr)
    <td> -
</tr>
<tr>
    <td> SFRA
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Software Frequency Response Analyzer support
    <td> -
</tr>
<tr>
    <td> Transforms
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Clarke transformation, Park transformation, Inverse Park transformation, Space Vector Generation (SVGEN), Common-mode subtraction approach, DPWM Generation (Part of SVGEN), Maximum Modulation, Minimum Modulation, SVGEN current reconstruction for single-shunt (SVGENCURRENT), Phase voltage reconstruction in overmodulation (VOLTS_RECON)
    <td> -
</tr>
<tr>
    <td> Utilities
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Angle Compensation Generator, Step Response, Datalog, Trapezoid generator
    <td> -
</tr>
</table>

<!-- ## Fixed Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Applicable Releases
    <th> Resolution/Comments
</tr>

</table> -->

## Known Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Applicable Releases
    <th> Workaround
</tr>
<tr>
    <td> PINDSW-8358
    <td> BiSS-C/Nikon/EnDat/Tamagawa: Exiting Periodic Trigger mode UART option does not work
    <td> Position Sense BiSS-C, Position Sense EnDat, Position Sense Nikon A-Format, Position Sense Tamagawa
    <td> 10.00.00 onwards
    <td> -
</tr>
</table>

<!-- ## Errata
<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> SDK Status
</tr>
<tr>
    <td> i2311
    <td> USART: Spurious DMA Interrupts
    <td> UART
    <td> Implemented
</tr>
<tr>
    <td> i2313
    <td> GPMC: Sub-32-bit read issue with NAND and FPGA/FIFO
    <td> GPMC
    <td> Implemented
</tr>
<tr>
    <td> i2331
    <td> CPSW: Device lockup when reading CPSW registers
    <td> CPSW, SBL
    <td> Implemented
</tr>
<tr>
    <td> i2345
    <td> CPSW: Ethernet Packet corruption occurs if CPDMA fetches a packet which spans across memory banks
    <td> CPSW
    <td> Implemented
</tr>
<tr>
    <td> i2326
    <td> PCIe: MAIN_PLLx operating in fractional mode, which is required for enabling SSC, is not compliant with PCIe Refclk jitter limits
    <td> PCIe
    <td> Open
</tr>
<tr>
    <td> i2312
    <td> MMCSD: HS200 and SDR104 Command Timeout Window Too Small
    <td> MMCSD
    <td> Open
</tr>
<tr>
    <td> i2310
    <td> USART: Erroneous clear/trigger of timeout interrupt
    <td> UART
    <td> Open
</tr>
<tr>
    <td> i2279
    <td> MCAN: Specification Update for dedicated Tx Buffers and Tx Queues configured with same Message ID
    <td> MCAN
    <td> Open
</tr>
<tr>
    <td> i2278
    <td> MCAN: Message Transmit order not guaranteed from dedicated Tx Buffers configured with same Message ID
    <td> MCAN
    <td> Open
</tr>
</table> -->

<!-- ## Limitations
<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Reported in Release
    <th> Applicable Devices
    <th> Workaround
</tr>

</table> -->

<!-- ## Upgrade and Compatibility Information for Motor Control SDK 10.00.00 {#UPGRADE_AND_COMPATIBILITY_INFORMATION_10_0_0}

<!-- This section lists changes which could affect user applications developed using older SDK versions.
Read this carefully to see if you need to do any changes in your existing application when migrating to this SDK version relative to
previous SDK version. Also refer to older SDK version release notes to see changes in
earlier SDKs. -->

<!-- ### Compiler Options

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td>
    <td>
    <td>
    <td>
</tr>
</table> -->

<!-- ### Examples

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
</table>

### Drivers

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
</table> -->


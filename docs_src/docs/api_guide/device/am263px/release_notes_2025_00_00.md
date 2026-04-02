# Release Notes 2025.00.00 {#RELEASE_NOTES_2025_00_00_PAGE}

[TOC]

\attention
    1. Please refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.
    2. Motor Control SDK 10.x included the Industrial Communications SDK and MCU+ SDK in bundled ind_comms_sdk and mcu_plus_sdk folders. Starting with Motor Control SDK 2025.00.00, these folders are no longer included and must be downloaded separately. Motor Control SDK installer provides an option to install the required Industrial Communications SDK and MCU+ SDK.
    3. For release notes of Industrial Communications SDK and MCU+ SDK, please refer to <a href="@VAR_IC_SDK_DOCS_PATH/RELEASE_NOTES_2025_00_00_PAGE.html" target="_blank"> @VAR_SOC_NAME Industrial Communications SDK Release Notes 2025.00.00</a> and <a href="@VAR_MCU_SDK_DOCS_PATH/RELEASE_NOTES_11_00_00_PAGE.html" target="_blank"> @VAR_SOC_NAME MCU+ SDK Release Notes 11.00.00</a> respectively.

\note
    These examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
    Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
    Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
    Unless noted otherwise, the SW modules would work on all supported EVMs \n

## New in this Release

<table>
<tr>
    <th>Feature
    <th>Module
</tr>
<tr>
    <td>EnDat3 protocol support with 12.5 Mbps data rate, Manchester encoding, frame-based communication (HPF/LPH/LPF), foreground and background communication channels, host trigger and periodic trigger modes
    <td>Position Sense EnDat3
</tr>
<tr>
    <td>Handle-based API architecture, periodic trigger modes (CMP and CAP), SysConfig-based initialization, per-channel encoder timeout configuration
    <td>Position Sense BiSS-C
</tr>
<tr>
    <td>Handle-based API architecture, periodic trigger modes (CMP and CAP), SysConfig-based initialization
    <td>Position Sense Nikon A-format
</tr>
<tr>
    <td>Handle-based API architecture, periodic trigger modes (CMP and CAP), SysConfig-based initialization
    <td>Position Sense EnDat
</tr>
<tr>
    <td>Handle-based API architecture, Periodic trigger modes (CMP and CAP), SysConfig-based initialization
    <td>Position Sense Tamagawa
</tr>
</table>

SOC    | Supported CPUs  | Boards                                                                 | Host PC
-------|-----------------|------------------------------------------------------------------------|-----------------------------------------------------
AM263Px| R5F             | AM263Px ControlCard Rev B (referred to as am263px-cc in code).         | Windows 10 64b or Ubuntu 18.04 64b or MacOS
AM263Px| R5F             | AM263Px LaunchPad Rev A (referred to as am263px-lp in code).           | Windows 10 64b or Ubuntu 18.04 64b or MacOS

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module                     | Supported CPUs | Version
--------------------------------------|----------------|-----------------------
AM263Px Industrial Communications SDK | R5F            | 2025.00.00.08
AM263Px MCU+ SDK                      | R5F, M4F       | 11.00.00.19
Code Composer Studio                  | R5F            | 20.3.0
SysConfig                             | R5F            | 1.25.0, build 4268
TI ARM CLANG                          | R5F            | 4.0.3.LTS
FreeRTOS Kernel                       | R5F            | 11.1.0
LwIP                                  | R5F            | STABLE-2_2_1_RELEASE
Mbed-TLS                              | R5F            | 2.13.1
Uniflash                              | R5F            | 9.3.0

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
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode), point-to-point connection, control communication, automatic processing delay detection and compensation, interface speed of 1, 2, 5, 8, and 10 MHz, long cable (upto 100 meters), host trigger mode, periodic trigger modes (CMP and CAP), daisy chaining, safety mode (safety CRC and sign-of-life counter), BP-AM2BLDCSERVO Boosterpack with LP-AM263P
    <td> -
</tr>
<tr>
    <td> EnDat
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode), recovery time for 2.2 command set, interface speed of 4, 8 and 16 MHz, long cable (upto 100 meters), periodic trigger modes (CMP and CAP), continuous mode, BP-AM2BLDCSERVO Boosterpack with LP-AM263P
    <td> Encoder receive communication command
</tr>
<tr>
    <td> EnDat3
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Manchester encoding, data transfer rate of 12.5 Mbps, frame-based protocol (HPF/LPH/LPF), foreground communication commands (DATA0-DATA7, DATA, DATANOP, RESET, CLEAR, ECHO, RATE, HELLO), background communication commands (NOP, READ, WRITE, RECONFIGURE, AUTH, PROTECT, SETPASS, LOCATE), host trigger mode, periodic trigger modes (CMP and CAP), automatic CRC verification, BP-AM2BLDCSERVO Boosterpack with LP-AM263P
    <td> 25 Mbps data rate, Multi-channel concurrent operation, Daisy chain topology, Long cable (upto 100 meters)
</tr>
<tr>
    <td> Nikon A-format
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Nikon A-format version 2.1 and version 3.0, Single channel, Multi channel using single PRU core and three PRU cores (load share mode), point-to-point connection, bus connection up to 7 encoders, individual and multiple transmission mode with encoder addresses ranging between ENC1-ENC8, baud rates from 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz, up to 40-bit absolute position (single turn + multi turn) data with additional information, long cable (upto 100 meters), host trigger mode, periodic trigger modes (CMP and CAP), BP-AM2BLDCSERVO Boosterpack with LP-AM263P
    <td> Bus connection with 8 encoders
</tr>
<tr>
    <td> Tamagawa
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using single PRU core, absolute position, encoder ID, reset, EEPROM read, EEPROM write, 2.5 Mbps Encoder, periodic trigger modes (CMP and CAP), BP-AM2BLDCSERVO Boosterpack with LP-AM263P
    <td> 5 Mbps encoder
</tr>
<tr>
    <td> Tamagawa over SOC UART
    <td> R5F
    <td> YES
    <td> FreeRTOS
    <td> Single channel, absolute position, encoder ID, reset, EEPROM read, EEPROM write, 2.5 Mbps Encoder, CRC verification
    <td> 5 Mbps encoder, long cable length
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

## Fixed Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Applicable Releases
    <th> Resolution/Comments
</tr>
<tr>
    <td> PINDSW-9371
    <td> SysConfig allows multiple channels to be selected simultaneously from 1 PRU
    <td> Position Sense BiSS-C, Position Sense EnDat, Position Sense Nikon A-format, Position Sense Tamagawa
    <td> 10.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9605
    <td> Unable to open example.syscfg directly from SysConfig tool
    <td> All examples
    <td> 10.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9784
    <td> Position Sense: PR0_PRU0_GPIO7 pin is not available in SoC, but shown in SysConfig modules
    <td> Position Sense BiSS-C, Position Sense EnDat, Position Sense Nikon, Position Sense Tamagawa
    <td> 10.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9788
    <td> BiSS-C/Nikon: SysConfig does not enable the required PRU GPIO pins
    <td> Position Sense BiSS-C, Position Sense Nikon A-format
    <td> 10.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-10365
    <td> Shared memory region located in TCM marked as Cached in SDK examples
    <td> Position Sense EnDat
    <td> 10.0 onwards
    <td> In SDK examples, PRU firmware for EnDat store the data into R5F TCM memory.
</tr>
<tr>
    <td> PINDSW-10389
    <td> Tamagawa: Periodic command does not work with a lower cycle period
    <td> Position Sense Tamagawa
    <td> 10.0 onwards
    <td> Due to maximum value of RX frame size being configured always for all commands, the periodic mode does not work for lower periods for certain commands.
</tr>
</table>

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
    <td> PINDSW-9179
    <td> Nikon: PRU Firmware gets stuck if encoder does not respond with number of bytes expected by driver
    <td> Position Sense Nikon A-format
    <td> 10.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-10641
    <td> Unable to create a new .syscfg with MCSDK directly from SysConfig tool
    <td> SysConfig
    <td> 2025.0 onwards
    <td> Copy an example.syscfg file from SDK example and use it
</tr>
</table>

## Limitations

\attention Please refer to individual module pages for more details on known limitations and unsupported features.

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Reported in Release
    <th> Applicable Devices
    <th> Workaround
</tr>
<tr>
    <td> MCUSDK-208
    <td> gmake with -j can sometimes lock up Windows command prompt
    <td> Build
    <td> 10.0 onwards
    <td> AM64x, AM243x
    <td> Use bash for windows as part of git for windows or don't use -j option
</tr>
<tr>
    <td> PINDSW-9370
    <td> Nikon: Only 7 encoders can be tested in bus connection with BP-AM2BLDCSERVO
    <td> Position Sense Nikon
    <td> 10.0 onwards
    <td> Unable to test 8 encoders because of voltage drop when using BP-AM2BLDCSERVO is used
</tr>
</table>

## Upgrade and Compatibility Information for Motor Control SDK 2025.0.0 {#UPGRADE_AND_COMPATIBILITY_INFORMATION_2025_0_0}

\attention When migrating from MCU+ SDK, see \ref MIGRATION_GUIDES for more details.

This section lists changes which could affect user applications developed using older SDK versions.
Read this carefully to see if you need to do any changes in your existing application when migrating to this SDK version relative to
previous SDK version. Also refer to older SDK version release notes to see changes in earlier SDKs.

SDK drivers for following modules underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes, improved SysConfig integration, etc. For module specific migration details, refer to links from \ref MIGRATION_SECTION_2025_00.

- Position Sense
    - BiSS-C
    - EnDAT 2
    - Nikon A-format
    - Tamagawa

### Examples
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
    <th> Affected API or structure
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> Tamagawa over SOC UART
    <td> tamagawa_init()
    <td> Return type changed from void to int32_t (`SystemP_SUCCESS` / `SystemP_FAILURE`). Added NULL pointer validation.
    <td> -
</tr>
<tr>
    <td> Tamagawa over SOC UART
    <td> tamagawa_command_process()
    <td> Return values standardized to `SystemP_SUCCESS` / `SystemP_FAILURE`
    <td> -
</tr>
</table>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

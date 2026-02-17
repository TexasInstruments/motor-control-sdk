# Release Notes 2025.0.0 {#RELEASE_NOTES_2025_0_0_PAGE}

[TOC]

\attention Please refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention For release notes of Industrial Communications SDK and MCU+ SDK, please refer to <a href="@VAR_IC_SDK_DOCS_PATH/RELEASE_NOTES_2025_0_0_PAGE.html" target="_blank"> @VAR_SOC_NAME Industrial Communications SDK Release Notes 2025.0.0</a> and <a href="@VAR_MCU_SDK_DOCS_PATH/RELEASE_NOTES_2025_0_0_PAGE.html" target="_blank"> @VAR_SOC_NAME MCU+ SDK Release Notes 2025.0.0</a> respectively.

\note
        1. In Motor Control SDK 2025.0.0 (production package), using the EtherCAT example from Industrial Communications SDK or EtherCAT example for TIDEP-01032 EtherCAT-Connected Single-Chip Dual-Servo Motor Drive Reference Design requires downloading the Beckhoff SSC stack from the ETG website and rebuilding the library as described in the following documentation:
            - <a href="../../ind_comms_sdk/docs/am243x/ethercat_subdevice/beckhoff__s_c_c.html" target="_blank">Beckhoff SSC</a>
            - <a href="@VAR_IC_SDK_DOCS_PATH/EXAMPLES_INDUSTRIAL_COMMS_ETHERCAT_SLAVE_BECKHOFF_SSC_DEMO.html#STEPS_TO_RUN_ECAT_BECKHOFF_SSC_DEMO" target="_blank"> Steps to Run the Example</a>
            - For EtherCAT example evaluation without access to Beckhoff SSC, please use the evaluation package.

        2. These examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
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
</table>

SOC    | Supported CPUs  | Boards                                                                                                      | Host PC
-------|-----------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------
AM263Px| R5F             | AM263Px ControlCard Rev A    (referred to as am263px-cc in code).
         | Windows 10 64b or Ubuntu 18.04 64b or MacOS or MacOS
AM263Px| R5F             | AM263Px LaunchPad  Rev A    (referred to as am263px-lp in code).
         | Windows 10 64b or Ubuntu 18.04 64b or MacOS or MacOS

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module                    | Supported CPUs | Version
-------------------------------------|----------------|-----------------------
AM243x Industrial Communications SDK | R5F            | 11.00.00.13
AM243x MCU+ SDK                      | R5F, M4F       | 11.00.00.18
Code Composer Studio                 | R5F, M4F       | 12.8.1
SysConfig                            | R5F, M4F       | 1.22.0, build 3893
TI ARM CLANG                         | R5F, M4F       | 4.1.0.LTS
FreeRTOS Kernel                      | R5F, M4F       | 11.1.0
Tiny USB                             | R5F            | 0.14.0
LwIP                                 | R5F            | STABLE-2_2_0_RELEASE
Mbed-TLS                             | R5F            | mbedtls-2.13.1

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
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode), point-to-point connection, control communication, automatic processing delay detection and compensation, interface speed of 1, 2, 5, 8, and 10 MHz, long cable (upto 100 meters), periodic trigger modes (CMP and CAP), daisy chaining, safety mode (safety CRC and sign-of-life counter), BP-AM2BLDCSERVO Boosterpack with LP-AM263P
    <td> -
</tr>
<tr>
    <td> EnDat
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode), recovery time for 2.2 command set, interface speed of 4, 8 and 16 MHz, long cable (upto 100 meters), continuous mode, BP-AM2BLDCSERVO Boosterpack with LP-AM263P
    <td> Encoder receive communication command
</tr>
<tr>
    <td> EnDat3
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Manchester encoding, data transfer rate of 12.5 Mbps, frame-based protocol (HPF/LPH/LPF), foreground communication commands (DATA0-DATA7, DATA, DATANOP, RESET, CLEAR, ECHO, RATE, HELLO), background communication commands (NOP, READ, WRITE, RECONFIGURE, AUTH, PROTECT, SETPASS, LOCATE), host trigger mode, periodic trigger mode, continuous mode, automatic CRC verification, BP-AM2BLDCSERVO Boosterpack with LP-AM263P
    <td> 25 Mbps data rate, Multi-channel concurrent operation, Daisy chain topology, Long cable (upto 100 meters)
</tr>
<tr>
    <td> HDSL
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using three PRU cores (load share mode), Free Run mode, Sync mode, Short Message Read and Write, Long Message Read and Write, Pipeline Channel Data, Long cable (upto 100 meters) with single channel Free Run mode, BP-AM2BLDCSERVO Boosterpack with LP-AM263P, 225 MHz PRU firmware
    <td> Long cable (upto 100 meters) with sync mode
</tr>
<tr>
    <td> Nikon A-format
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Nikon A-format version 2.1 and version 3.0, Single channel, Multi channel using single PRU core and three PRU cores (load share mode), point-to-point connection, bus connection up to 8 encoders, individual and multiple transmission mode with encoder addresses ranging between ENC1-ENC8, baud rates from 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz, up to 40-bit absolute position (single turn + multi turn) data with additional information, long cable (upto 100 meters), BP-AM2BLDCSERVO Boosterpack with LP-AM263P
    <td> -
</tr>
<tr>
    <td> Tamagawa
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using single PRU core, absolute position, encoder ID, reset, EEPROM read, EEPROM write, 2.5 Mbps Encoder, continuous mode, BP-AM2BLDCSERVO Boosterpack with LP-AM263P
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
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Use bash for windows as part of git for windows or don't use -j option
</tr>
</table>

## Upgrade and Compatibility Information for Motor Control SDK 2025.0.0 {#UPGRADE_AND_COMPATIBILITY_INFORMATION_2025_0_0}

\attention When migrating from MCU+ SDK, see \ref MIGRATION_GUIDES for more details.

This section lists changes which could affect user applications developed using older SDK versions.
Read this carefully to see if you need to do any changes in your existing application when migrating to this SDK version relative to
previous SDK version. Also refer to older SDK version release notes to see changes in earlier SDKs.

SDK drivers underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes, improved SysConfig integration, etc. For module specific migration details, refer to links from \ref MIGRATION_SECTION_2025_00.

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

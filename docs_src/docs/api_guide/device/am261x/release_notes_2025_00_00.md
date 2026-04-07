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
    <td>Dual channel support using two independent PRU cores, handle-based API architecture, periodic trigger modes (CMP and CAP), SysConfig-based initialization, per-channel encoder timeout configuration
    <td>Position Sense BiSS-C
</tr>
<tr>
    <td>Dual channel support using two independent PRU cores, improved sampling for RX data, handle-based API architecture, different IEP CAP signal per slice, SysConfig-based initialization
    <td>Position Sense HDSL
</tr>
<tr>
    <td>Dual channel support using two independent PRU cores, handle-based API architecture, periodic trigger modes (CMP and CAP), SysConfig-based initialization
    <td>Position Sense Nikon A-format
</tr>
<tr>
    <td>Dual channel support using two independent PRU cores, handle-based API architecture, periodic trigger modes (CMP and CAP), SysConfig-based initialization
    <td>Position Sense EnDat
</tr>
<tr>
    <td>Dual channel support using two independent PRU cores, Handle-based API architecture, Periodic trigger modes (CMP and CAP), SysConfig-based initialization
    <td>Position Sense Tamagawa
</tr>
</table>

## Device and Validation Information

SOC    | Supported CPUs  | EVM                                                                          | Host PC
-------|-----------------|------------------------------------------------------------------------------|-----------------------------------------------
AM261x | R5F             | AM261x LaunchPad Revision A  (referred to as am261x-lp in code). \n          | Windows 10 64b or Ubuntu 18.04 64b or MacOS

## Dependent Tools, Compiler and Other Open Source SW Module Information

Tools                                               | Supported CPUs | Version
----------------------------------------------------|----------------|-----------------------
AM261x Industrial Communications SDK                | R5F            | 2025.00.00.08
AM261x MCU+ SDK                                     | R5F, M4F       | 11.00.00.29
Code Composer Studio                                | R5F            | 20.3.0
SysConfig                                           | R5F            | 1.25.0, build 4268
TI ARM CLANG                                        | R5F            | 4.0.3.LTS
FreeRTOS Kernel (included in AM261x MCU+ SDK)       | R5F            | 11.1.0
LwIP (included in AM261x MCU+ SDK)                  | R5F            | STABLE-2_2_1_RELEASE
Mbed-TLS (included in AM261x MCU+ SDK)              | R5F            | 2.13.1
Uniflash (included in AM261x MCU+ SDK)              | R5F            | 9.3.0

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
    <td> Single channel, dual channel using two independent PRU cores, point-to-point connection, control communication, automatic processing delay detection and compensation, interface speed of 1, 2, 5, 8, and 10 MHz, host trigger mode, periodic trigger modes (CMP and CAP), daisy chaining, safety mode (safety CRC and sign-of-life counter), BP-AM2BLDCSERVO Boosterpack with LP-AM261
    <td> -
</tr>
<tr>
    <td> EnDat
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, dual channel using two independent PRU cores, Recovery Time for 2.2 command set, Interface speed of 5 and 10 MHz, periodic trigger modes (CMP and CAP), continuous mode, BP-AM2BLDCSERVO Boosterpack with LP-AM261
    <td> Encoder receive communication command
</tr>
<tr>
    <td> EnDat3
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Manchester encoding, data transfer rate of 12.5 Mbps, frame-based protocol (HPF/LPH/LPF), foreground communication commands (DATA0-DATA7, DATA, DATANOP, RESET, CLEAR, ECHO, RATE, HELLO), background communication commands (NOP, READ, WRITE, RECONFIGURE, AUTH, PROTECT, SETPASS, LOCATE), host trigger mode, periodic trigger modes (CMP and CAP), automatic CRC verification, BP-AM2BLDCSERVO Boosterpack with LP-AM261
    <td> 25 Mbps data rate, Multi-channel concurrent operation, Daisy chain topology, Long cable (upto 100 meters)
</tr>
<tr>
    <td> HDSL
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Dual channel using two independent PRU cores, Free Run mode, Sync mode, Short Message Read and Write, Long Message Read and Write, Pipeline Channel Data, Cable length upto 10 meters, BP-AM2BLDCSERVO Boosterpack with LP-AM261
    <td> Long cable upto 100 meters, Trace feature for logging registers
</tr>
<tr>
    <td> Nikon A-format
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Nikon A-format version 2.1 and version 3.0, Single channel, Multi channel using single PRU core, point-to-point connection, bus connection up to 7 encoders, Individual and multiple transmission mode with encoder addresses ranging between ENC1-ENC8, baud rates from 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz, up to 40-bit absolute position (single turn + multi turn) data with additional information, long cable (upto 100 meters), host trigger mode, periodic trigger modes (CMP and CAP), BP-AM2BLDCSERVO Boosterpack with LP-AM261
    <td> Bus connection with 8 encoders
</tr>
<tr>
    <td> Tamagawa
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Absolute position, Encoder ID, Reset, EEPROM Read, EEPROM Write, 2.5 Mbps Encoder, periodic trigger modes (CMP and CAP), BP-AM2BLDCSERVO Boosterpack with LP-AM261
    <td> 5 Mbps encoder
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
    <td> PINDSW-5690
    <td> HDSL: EDGE register is not updated
    <td> Position Sense HDSL
    <td> 10.0.1 onwards
    <td> NOTE: This register is not implemented in TI HDSL solution. It is documented as a known difference in \ref HDSL_EXCEPTIONS_LIST.
</tr>
<tr>
    <td> PINDSW-8296
    <td> HDSL: Incorrect SAFE_SUM value is seen
    <td> Position Sense HDSL
    <td> 10.0.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9308
    <td> Nikon: nikon_calc_clock API does not handle baud rate correctly
    <td> Position Sense Nikon A-format
    <td> 10.0.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9312
    <td> Nikon: Data type of velocity and acceleration is unsigned integer
    <td> Position Sense Nikon A-format
    <td> 10.0.1 onwards
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13295, EXT_EP-13295}, PINDSW-9317
    <td> BiSS-C: bissc_update_data_len does not set number of encoders correctly
    <td> Position Sense BiSS-C
    <td> 10.0.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9371
    <td> SysConfig allows multiple channels to be selected simultaneously from 1 PRU
    <td> Position Sense BiSS-C, Position Sense EnDat, Position Sense HDSL, Position Sense Nikon A-format, Position Sense Tamagawa
    <td> 10.0.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9372
    <td> BiSS-C/Nikon: SysConfig shows option for load share mode
    <td> Position Sense BiSS-C, Position Sense Nikon A-format
    <td> 10.0.1 onwards
    <td> This option is not relevant for AM261x
</tr>
<tr>
    <td> PINDSW-9385
    <td> EnDat: Recovery time does not work with EnDat 2.2 supplementary commands for certain EnDat frequencies
    <td> Position Sense EnDat
    <td> 10.0.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9605
    <td> Unable to open example.syscfg directly from SysConfig tool
    <td> All examples
    <td> 10.0.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9788
    <td> BiSS-C/Nikon: SysConfig does not enable the required PRU GPIO pins
    <td> Position Sense BiSS-C, Position Sense Nikon A-format
    <td> 10.0.0 onwards
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13298, EXT_EP-13298}, PINDSW-10244
    <td> EnDat: Variations seen in propagation delay measurement with different PRU Clock frequencies
    <td> Position Sense EnDat
    <td> 10.0.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-10275
    <td> HDSL: Sync pulse alignment does not work
    <td> Position Sense HDSL
    <td> 10.0.1 onwards
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13300, EXT_EP-13300}, PINDSW-10365
    <td> Shared memory region located in TCM marked as Cached in SDK examples
    <td> Position Sense EnDat
    <td> 10.0.0 onwards
    <td> In SDK examples, PRU firmware for EnDat store the data into R5F TCM memory.
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13301, EXT_EP-13301}, PINDSW-10389
    <td> Tamagawa: Periodic command does not work with a lower cycle period
    <td> Position Sense Tamagawa
    <td> 10.0.0 onwards
    <td> Due to maximum value of RX frame size being configured always for all commands, the periodic mode does not work for lower periods for certain commands.
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13294, EXT_EP-13294}, PINDSW-10670
    <td> HDSL: Communication drops seen in SYNC mode
    <td> Position Sense HDSL
    <td> 10.0.1 onwards
    <td> -
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
    <td> PINDSW-6486
    <td> HDSL: RSSI register shows higher values than expected for a non-noisy setup
    <td> Position Sense HDSL
    <td> 10.0.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7130
    <td> HDSL: Few protocol resets seen during initialization with Free Run mode on LP-AM243 and LP-AM261
    <td> Position Sense HDSL
    <td> 10.0.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7163
    <td> HDSL: Trailer data contains 4 zeros instead of 5
    <td> Position Sense HDSL
    <td> 10.0.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7474
    <td> HDSL: Sync mode does not work with cable length > 10 meters
    <td> Position Sense HDSL
    <td> 10.0.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9179
    <td> Nikon: PRU Firmware gets stuck if encoder does not respond with number of bytes expected by driver
    <td> Position Sense Nikon A-format
    <td> 10.0.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-10391
    <td> HDSL: Free run mode does not work with cable length > 10 meters when PRU runs at 225 MHz
    <td> Position Sense HDSL
    <td> 10.0.1 onwards
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
    <td> 10.0.0 onwards
    <td> AM64x, AM243x
    <td> Use bash for windows as part of git for windows or don't use -j option
</tr>
<tr>
    <td> PINDSW-9370
    <td> Nikon: Only 7 encoders can be tested in bus connection with BP-AM2BLDCSERVO
    <td> Position Sense Nikon
    <td> 10.0.1 onwards
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
    - HDSL
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
</table>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

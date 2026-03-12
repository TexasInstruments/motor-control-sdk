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

        3. Tamagawa over SoC UART example is not supported for AM243x.

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
    <td>Multi-channel with different communication modes and different numbers of encoders connected across channels under load share mode, Handle-based API architecture, Periodic trigger modes (CMP and CAP), SysConfig-based initialization
    <td>Position Sense Tamagawa
</tr>
<tr>
    <td>Handle-based API architecture, periodic trigger modes (CMP and CAP), SysConfig-based initialization
    <td>Position Sense EnDat
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
    <td>Handle-based API architecture, SysConfig-based initialization, improved error handling with return codes, support for 9 channels on a single PRU core
    <td>Current Sense %SDFM
</tr>
</table>

SOC    | Supported CPUs  | Boards                                                                                                      | Host PC
-------|-----------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------
AM243x | R5F             | AM243x GP EVM (referred to as am243x-evm in code), \n AM243x LAUNCHPAD (referred to as am243x-lp in code)   | Windows 10 64b or Ubuntu 18.04 64b

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

<!-- ### Features not supported in release -->

### Current Sense

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
    <td> ICSS %SDFM
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Three %SDFM channels using single PRU core, Nine %SDFM channels using three PRU cores (load share mode), %SDFM Sync with EPWM, Single/Double Normal Current Sampling per EPWM cycle, Continuous Normal Current Sampling, High and Low Threshold Comparator (Over-current detction), Fast Detect, Phase Compensation, Zero Cross Detection, Trip Generation using PRU-ICSS TripZone, Tested with %SDFM clock from ECAP/IEP, Tested with 5MHz Clock from EPWM
    <td> -
</tr>
</table>

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
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode), point-to-point connection, control communication, automatic processing delay detection and compensation, interface speed of 1, 2, 5, 8, and 10 MHz, long cable (upto 100 meters), host trigger mode, periodic trigger modes (CMP and CAP), daisy chaining, safety mode (safety CRC and sign-of-life counter), BP-AM2BLDCSERVO Boosterpack with LP-AM243
    <td> -
</tr>
<tr>
    <td> EnDat
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode), recovery time for 2.2 command set, interface speed of 4, 8 and 16 MHz, long cable (upto 100 meters), periodic trigger modes (CMP and CAP), continuous mode, BP-AM2BLDCSERVO Boosterpack with LP-AM243
    <td> Encoder receive communication command
</tr>
<tr>
    <td> EnDat3
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Manchester encoding, data transfer rate of 12.5 Mbps, frame-based protocol (HPF/LPH/LPF), foreground communication commands (DATA0-DATA7, DATA, DATANOP, RESET, CLEAR, ECHO, RATE, HELLO), background communication commands (NOP, READ, WRITE, RECONFIGURE, AUTH, PROTECT, SETPASS, LOCATE), host trigger mode, periodic trigger modes (CMP and CAP) automatic CRC verification, BP-AM2BLDCSERVO Boosterpack with LP-AM243
    <td> 25 Mbps data rate, Multi-channel concurrent operation, Daisy chain topology, Long cable (upto 100 meters)
</tr>
<tr>
    <td> HDSL
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using three PRU cores (load share mode), Free Run mode, Sync mode, Short Message Read and Write, Long Message Read and Write, Pipeline Channel Data, Long cable (upto 100 meters) with single channel Free Run mode, BP-AM2BLDCSERVO Boosterpack with LP-AM243, 225 MHz PRU firmware
    <td> Long cable (upto 100 meters) with sync mode
</tr>
<tr>
    <td> Nikon A-format
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Nikon A-format version 2.1 and version 3.0, Single channel, Multi channel using single PRU core and three PRU cores (load share mode), point-to-point connection, bus connection up to 7 encoders, individual and multiple transmission mode with encoder addresses ranging between ENC1-ENC8, baud rates from 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz, up to 40-bit absolute position (single turn + multi turn) data with additional information, long cable (upto 100 meters), host trigger mode, periodic trigger modes (CMP and CAP), BP-AM2BLDCSERVO Boosterpack with LP-AM243
    <td> Bus connection with 8 encoders
</tr>
<tr>
    <td> Tamagawa
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using single PRU core, Multi channel using three PRU cores (load share mode), absolute position, encoder ID, reset, EEPROM read, EEPROM write, 2.5 Mbps Encoder, periodic trigger modes (CMP and CAP), BP-AM2BLDCSERVO Boosterpack with LP-AM243
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

### PRU-ICSS PWM

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
    <td> PRU-ICSS PWM
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> PWM Duty cycle, PWM Phase shift, PWM Dead band, PWM Period
    <td> -
</tr>
</table>

### TIDEP-01032 EtherCAT-Connected Single-Chip Dual-Servo Motor Drive Reference Design

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
    <td> TIDEP-01032 EtherCAT-Connected Single-Chip Dual-Servo Motor Drive Reference Design
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> All build levels, speed control for open loop and closed loop, closed loop with EtherCAT Distributed Clock (DC) mode
    <td> -
</tr>
</table>

### Timesync

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
    <td> Timesync
    <td> R5F
    <td> YES
    <td> FreeRTOS
    <td> Tested with two board setup: board 1 running time_sync_transmitter_receiver example and board 2 running time_receiver example at 100us cycle time
    <td> Not tested time_receiver example in release mode
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
<tr>
    <td> PRUICSS PWM
    <td> App_epwmSync0Irq
    <td> Remove calling \ref PRUICSS_PWM_changePwmSetToIntialState API call
    <td> PWM is not changed to init state when it is being generated
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> SDFM_init(), SDFM_enable(), and all SDFM APIs
    <td> Handle type changed from sdfm_handle to \ref SDFM_Handle, initialization uses SysConfig index and \ref SDFM_Params
    <td> Refer \ref SDFM_MIGRATION_GUIDE_2025_00
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
    <td> PRUICSS PWM
    <td> \ref PRUICSS_PWM_config
    <td> Change dutyCycle, riseEdgeDelay, fallEdgeDelay data type to float
    <td> -
</tr>
<tr>
    <td> PRUICSS PWM
    <td> \ref PRUICSS_PWM_iepConfig
    <td> Initialize IEP counter value with zero and compare values with \ref PRUICSS_IEP_COUNT_REG_MAX
    <td> -
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> SDFM_init()
    <td> Signature changed from (PRUICSS_Handle, uint8_t, uint8_t) to (uint32_t index, \ref SDFM_Params *params). Returns \ref SDFM_Handle instead of sdfm_handle.
    <td> Refer \ref SDFM_MIGRATION_GUIDE_2025_00
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> All SDFM APIs
    <td> Handle type changed from sdfm_handle to \ref SDFM_Handle. Most APIs now return int32_t (SystemP_SUCCESS/FAILURE/TIMEOUT) instead of void.
    <td> Refer \ref SDFM_MIGRATION_GUIDE_2025_00
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> SDFM_setCompFilterThresholds(), SDFM_configFastDetect()
    <td> Parameter types changed from raw pointers (uint32_t*, uint8_t*) to \ref SDFM_ThresholdConfig and \ref SDFM_FastDetectConfig structures
    <td> Refer \ref SDFM_MIGRATION_GUIDE_2025_00
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> SDFM_enableContinuousNormalCurrent(), SDFM_enableLoadShareMode()
    <td> APIs removed. Use SDFM_enableTriggerModeForNormalCurrent() and SysConfig configuration respectively.
    <td> Refer \ref SDFM_MIGRATION_GUIDE_2025_00
</tr>
</table>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

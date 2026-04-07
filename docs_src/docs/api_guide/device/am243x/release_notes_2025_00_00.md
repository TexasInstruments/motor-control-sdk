# Release Notes 2025.00.00 {#RELEASE_NOTES_2025_00_00_PAGE}

[TOC]

\attention
    1. Please refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.
    2. Motor Control SDK 9.x/11.x included the Industrial Communications SDK and MCU+ SDK in bundled ind_comms_sdk and mcu_plus_sdk folders. Starting with Motor Control SDK 2025.00.00, these folders are no longer included and must be downloaded separately. Motor Control SDK installer provides an option to install the required Industrial Communications SDK and MCU+ SDK.
    3. For release notes of Industrial Communications SDK and MCU+ SDK, please refer to <a href="@VAR_IC_SDK_DOCS_PATH/RELEASE_NOTES_2025_00_00_PAGE.html" target="_blank"> @VAR_SOC_NAME Industrial Communications SDK Release Notes 2025.00.00</a> and <a href="@VAR_MCU_SDK_DOCS_PATH/RELEASE_NOTES_11_01_00_PAGE.html" target="_blank"> @VAR_SOC_NAME MCU+ SDK Release Notes 11.01.00</a> respectively.

\note
    1. These examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
        Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
        Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
        Unless noted otherwise, the SW modules would work on all supported EVMs \n

    2. Tamagawa over SoC UART example is not supported for AM243x.

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
    <td>Improved sampling for RX data, Handle-based API architecture, different IEP CAP signal per slice, SysConfig-based initialization
    <td>Position Sense HDSL
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
-------|-----------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------------------
AM243x | R5F             | AM243x GP EVM (referred to as am243x-evm in code), \n AM243x LAUNCHPAD (referred to as am243x-lp in code)   |Windows 10 64b or Ubuntu 18.04 64b or MacOS

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module                    | Supported CPUs | Version
-------------------------------------|----------------|-----------------------
AM243x Industrial Communications SDK | R5F            | 2025.00.00.08
AM243x MCU+ SDK                      | R5F, M4F       | 11.01.00.19
Code Composer Studio                 | R5F, M4F       | 20.2.0
SysConfig                            | R5F, M4F       | 1.23.0, build 4000
TI ARM CLANG                         | R5F, M4F       | 4.0.1.LTS
FreeRTOS Kernel                      | R5F, M4F       | 11.1.0
Tiny USB                             | R5F            | 0.14.0
LwIP                                 | R5F            | STABLE-2_2_0_RELEASE
Mbed-TLS                             | R5F            | 2.13.1

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
    <td> Three %SDFM channels using single PRU core, Nine %SDFM channels using three PRU cores (load share mode), %SDFM Sync with EPWM, Single/Double Normal Current Sampling per EPWM cycle, Continuous Normal Current Sampling, High and Low Threshold Comparator (Over-current detection), Fast Detect, Phase Compensation, Zero Cross Detection, Trip Generation using PRU-ICSS TripZone, Tested with %SDFM clock from ECAP/IEP, Tested with 5MHz Clock from EPWM
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
    <td> Single channel, Manchester encoding, data transfer rate of 12.5 Mbps, frame-based protocol (HPF/LPH/LPF), foreground communication commands (DATA0-DATA7, DATA, DATANOP, RESET, CLEAR, ECHO, RATE, HELLO), background communication commands (NOP, READ, WRITE, RECONFIGURE, AUTH, PROTECT, SETPASS, LOCATE), host trigger mode, periodic trigger modes (CMP and CAP), automatic CRC verification, BP-AM2BLDCSERVO Boosterpack with LP-AM243
    <td> 25 Mbps data rate, Multi-channel concurrent operation, Daisy chain topology, Long cable (upto 100 meters)
</tr>
<tr>
    <td> HDSL
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using three PRU cores (load share mode), Free Run mode, Sync mode, Short Message Read and Write, Long Message Read and Write, Pipeline Channel Data, Cable length upto 10 meters, Long cable upto 100 meters with single channel Free Run mode and PRU core running at 300 MHz, BP-AM2BLDCSERVO Boosterpack with LP-AM243
    <td> Long cable upto 100 meters (except single channel Free Run mode and PRU core running at 300 MHz)
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
<tr>
    <td> PINDSW-5690
    <td> HDSL: EDGE register is not updated
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> NOTE: This register is not implemented in TI HDSL solution. It is documented as a known difference in \ref HDSL_EXCEPTIONS_LIST.
</tr>
<tr>
    <td> PINDSW-9605
    <td> Unable to open example.syscfg directly from SysConfig tool
    <td> All examples
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9707
    <td> %SDFM: Changing OSR values at runtime does not have any effect on sample output
    <td> Current Sense %SDFM
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-10139
    <td> EnDat: Supplement 2_2 commands do not work with periodic mode when channels 0 and 1 both are enabled
    <td> Position Sense EnDat
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-10244
    <td> EnDat: Variations seen in propagation delay measurement with different PRU Clock frequencies
    <td> Position Sense EnDat
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-10284
    <td> %SDFM: Trigger mode requires all three channels to be connected
    <td> Current Sense %SDFM
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-10322
    <td> TIDEP-01032: USER_M1_IS_OFFSET_CMPSS macro type mismatch causes data abort at -O1 optimization
    <td> TIDEP-01032 Reference Design
    <td> 9.1 onwards
    <td> The USER_M1_IS_OFFSET_CMPSS macro for the BP_AM2BLDCSERVO board was casting SDFM_HALF_SCALE (131072.0f) to uint16_t, which exceeds the uint16_t range (max 65535). This float-to-integer undefined behavior caused the compiler to eliminate the calcMotorOverCurrentThreshold function body at -O1 optimization. Fixed by setting the macro to 0 for SDFM-based boards where ADC CMPSS is not applicable.
</tr>
<tr>
    <td> PINDSW-10364
    <td> TIDEP-01032: Incorrect constant table configuration for PRU-ICSSG1
    <td> TIDEP-01032 Reference Design
    <td> 11.0
    <td> -
</tr>
<tr>
    <td> PINDSW-10365
    <td> Shared memory region located in TCM marked as Cached in SDK examples
    <td> TIDEP-01032 Reference Design, Position Sense EnDat, Current Sense %SDFM
    <td> 9.0 onwards
    <td> In SDK examples, PRU firmwares for %SDFM and EnDat store the data into R5F TCM memory.
</tr>
<tr>
    <td> PINDSW-10389
    <td> Tamagawa: Periodic command does not work with a lower cycle period
    <td> Position Sense Tamagawa
    <td> 9.0 onwards
    <td> Due to maximum value of RX frame size being configured always for all commands, the periodic mode does not work for lower periods for certain commands.
</tr>
<tr>
    <td> PINDSW-10435
    <td> PRUICSS PWM: PRUICSS_PWM_enableIEP1Slave always uses 1 instead of using enable parameter
    <td> PRUICSS PWM
    <td> 9.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-10647
    <td> PRUICSS PWM: PRUICSS_PWM_iepConfig uses incorrect variable for IEP shadow mode check
    <td> PRUICSS PWM
    <td> 9.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-10670
    <td> HDSL: Communication drops seen in SYNC mode
    <td> Position Sense HDSL
    <td> 9.0 onwards
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
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7130
    <td> HDSL: Few protocol resets seen during initialization with Free Run mode on LP-AM243
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7163
    <td> HDSL: Trailer data contains 4 zeros instead of 5
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7474
    <td> HDSL: Sync mode does not work with cable length > 10 meters
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9179
    <td> Nikon: PRU Firmware gets stuck if encoder does not respond with number of bytes expected by driver
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-10391
    <td> HDSL: Free run mode does not work with cable length > 10 meters when PRU runs at 225 MHz
    <td> Position Sense HDSL
    <td> 9.0 onwards
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
    <td> 9.0 onwards
    <td> AM64x, AM243x
    <td> Use bash for windows as part of git for windows or don't use -j option
</tr>
<tr>
    <td> PINDSW-9370
    <td> Nikon: Only 7 encoders can be tested in bus connection with BP-AM2BLDCSERVO
    <td> Position Sense Nikon
    <td> 9.2 onwards
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
- Current Sense

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
    <td> TIDEP-01032 Reference Design
    <td> HAL_setupSDFM, HAL_readMtrSdfmData, HAL_setupEncoder, HAL_getMtrEncoderPosition in hal.c
    <td> The %SDFM and EnDat driver code in hal.c has been updated to use the new handle-based API architecture. The %SDFM initialization now uses SDFM_init with params-based configuration, and data reads use SDFM_getFilterData with handle. The EnDat initialization now uses endat_init with params-based configuration, and position reads use endat_command_build/endat_command_process with handle. These changes are required due to SDK driver migration to handle-based APIs.
    <td> For more information on %SDFM and EnDat driver migration, refer the migration guide of EnDat and %SDFM.
</tr>
<tr>
    <td> TIDEP-01032 Reference Design
    <td> PRUICSS INTC mapping in SysCfg, MOTOR1_PRU_TRIGGER_HOST_SDFM_EVT_NUMBER and MOTOR2_PRU_TRIGGER_HOST_SDFM_EVT_NUMBER in hal.h
    <td> The PRU INTC event-to-channel mapping for %SDFM interrupts has been updated. MOTOR1 now uses PRU event 21 (channel 5) and MOTOR2 uses PRU event 24 (channel 6). The SysCfg INTC mapping configuration has been updated accordingly. Additionally, SDFM_selectIepCmpEvent() API call has been added to configure the IEP compare event for %SDFM sampling trigger.
    <td> Update the SysCfg INTC mapping and hal.h PRU event number defines to match the new assignment.
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

</table>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

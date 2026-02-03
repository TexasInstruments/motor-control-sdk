# Release Notes 10.02.00 {#RELEASE_NOTES_10_02_00_PAGE}

[TOC]

\note Motor Control SDK version 10.02.00 supports LP-AM261 Rev. E2 only. The upcoming LP-AM261 board revision will feature integrated DP83869 Ethernet PHYs onboard, replacing the DP83826E daughter card approach used in LP-AM261 Rev. E2.

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention For release notes of Industrial Communications SDK and MCU+ SDK, please refer to <a href="@VAR_IC_SDK_DOCS_PATH/RELEASE_NOTES_10_02_00_PAGE.html" target="_blank"> @VAR_SOC_NAME Industrial Communications SDK Release Notes 10.02.00</a> and <a href="@VAR_MCU_SDK_DOCS_PATH/RELEASE_NOTES_10_02_00_PAGE.html" target="_blank"> @VAR_SOC_NAME MCU+ SDK Release Notes 10.02.00</a> respectively.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
Nikon A-format version 3.0                                                                      | Position Sense Nikon A-format
Support for up to 8 encoders in bus connection                                                  | Position Sense Nikon A-format
Control algorithms                                                                              | Real Time Libraries
Datalog                                                                                         | Real Time Libraries
Digital Control Library (DCL)                                                                   | Real Time Libraries
Observer algorithms                                                                             | Real Time Libraries
Software Frequency Response Analyzer                                                            | Real Time Libraries
Transformation Algorithm                                                                        | Real Time Libraries

## Device and Validation Information

SOC    | Supported CPUs  | EVM                                                                          | Host PC
-------|-----------------|------------------------------------------------------------------------------|-----------------------------------------------
AM261x | R5F             | AM261x Launchpad Rev E2    (referred to as am261x-lp in code). \n            | Windows 10 64b or Ubuntu 18.04 64b or MacOS

## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | 12.8.1
SysConfig               | R5F            | 1.23.0 build, build 4000
TI ARM CLANG            | R5F            | 4.0.1.LTS
FreeRTOS Kernel         | R5F            | 11.1.0
LwIP                    | R5F            | STABLE-2_2_0_RELEASE
Mbed-TLS                | R5F            | 2.13.1

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
    <td> Single channel, point-to-point connection, control communication, automatic processing delay detection and compensation, Interface speed of 1, 2, 5, 8, and 10 MHz, Boosterpack with LP-AM261
    <td> Daisy chaining, Multi Transmission Mode
</tr>
<tr>
    <td> EnDat
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Continuous mode, Recovery Time for 2.2 command set, Interface speed of 5 and 10 MHz, Boosterpack with LP-AM261
    <td> Encoder receive communication command
</tr>
<tr>
    <td> HDSL
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Free Run mode, Sync mode, Short Message Read and Write, Long Message Read and Write, Pipeline Channel Data, Long cable (upto 100 meters) with single channel Free Run mode, Boosterpack with LP-AM261, 225 MHz PRU firmware
    <td> Long cable (upto 100 meters) with sync mode, Trace feature for logging registers
</tr>
<tr>
    <td> Nikon A-format
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Nikon A-format version 2.1 and version 3.0, Single channel, Multi channel using single PRU core, point-to-point connection, bus connection up to 8 encoders, Individual and multiple transmission mode with encoder addresses ranging between ENC1-ENC8, baud rates from 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz, up to 40-bit absolute position (single turn + multi turn) data with additional information, long cable (upto 100 meters), Boosterpack with LP-AM261
    <td> -
</tr>
<tr>
    <td> Tamagawa
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Absolute position, Encoder ID, Reset, EEPROM Read, EEPROM Write, 2.5 Mbps Encoder, Boosterpack with LP-AM261
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
    <td> PINDSW-8087
    <td> Tamagawa: UART clock source is used for TX fifo
    <td> Position Sense Tamagawa
    <td> 10.0.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-8353
    <td> AM261x: HDSL Long message UART Commands (8, 9 and 10) not working
    <td> Position Sense HDSL
    <td> 10.0.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-8358
    <td> BiSS-C/Nikon/EnDat/Tamagawa: Exiting Periodic Trigger mode UART option does not work
    <td> Position Sense BiSS-C, Position Sense EnDat, Position Sense Nikon A-format, Position Sense Tamagawa
    <td> 10.0.1
    <td> -
</tr>
<tr>
    <td> PINDSW-8399
    <td> EnDat: Implementation of Recovery time is not as per specification (RT counter is missing)
    <td> Position Sense EnDat
    <td> 10.0.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9123
    <td> Nikon: Data reversal not done correctly for EEPROM and ID commands
    <td> Position Sense Nikon A-format
    <td> 10.0.1
    <td> Fixes done in \ref nikon_update_eeprom_addr, \ref nikon_update_eeprom_data, \ref nikon_update_id_code and \ref nikon_get_pos
</tr>
<tr>
    <td> PINDSW-9124
    <td> Nikon: Data reversal not done correctly for encoder status received in response
    <td> Position Sense Nikon A-format
    <td> 10.0.1
    <td> Fixes done in \ref nikon_get_pos
</tr>
<tr>
    <td> PINDSW-9127
    <td> Nikon: For commands 8 to 12, 9 requests are sent instead of 8
    <td> Position Sense Nikon A-format
    <td> 10.0.1
    <td> Fixes done in \ref nikon_get_pos
</tr>
<tr>
    <td> PINDSW-9128
    <td> Nikon: For EEPROM commands, request is sent 2 times from firmware
    <td> Position Sense Nikon A-format
    <td> 10.0.1
    <td> Fixes done in \ref nikon_get_pos, and updates in the application code for EEPROM command handling
</tr>
<tr>
    <td> PINDSW-9131
    <td> Nikon: EEPROM read for temperature does not use 10 bit data
    <td> Position Sense Nikon A-format
    <td> 10.0.1
    <td> Fixes done in \ref nikon_get_pos, and updated the application code for EEPROM command handling
</tr>
<tr>
    <td> PINDSW-9144
    <td> Nikon: For EEPROM/ID commands, same data is used for all channels in multi-channel mode
    <td> Position Sense Nikon A-format
    <td> 10.0.1
    <td> Added provision for different address/data per channel
</tr>
<tr>
    <td> PINDSW-9180
    <td> Nikon: 10 us delay between CDF-MDF and MDF-MDF is used for commands needing MDF
    <td> Position Sense Nikon A-format
    <td> 10.0.1
    <td> -
</tr>
<tr>
    <td> PINDSW-9238
    <td> Nikon: IEP compare value is not set correctly for continuous mode
    <td> Position Sense Nikon A-format
    <td> 10.0.1
    <td> Change the units to IEP clock cycle count
</tr>
<tr>
    <td> PINDSW-9239
    <td> Nikon: nikon_config_periodic_trigger assumes fixed receive size of response
    <td> Position Sense Nikon A-format
    <td> 10.0.1
    <td> Remove receive size hard-coding in driver
</tr>
<tr>
    <td> PINDSW-9248
    <td> EnDat: RX arm delay and TST delay settings are hard-coded for a 200MHz core clock
    <td> Position Sense EnDat
    <td> 10.0.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9255
    <td> Nikon: If Debug log/scan is removed and encoder resolution is fixed in application code, initialization fails
    <td> Position Sense Nikon A-format
    <td> 10.0.1
    <td> Add 0.5 seconds delay in application after powering up the encoder
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
    <td> PINDSW-5690
    <td> HDSL: EDGE register is not updated
    <td> Position Sense HDSL
    <td> 10.0.1 onwards
    <td> -
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
    <td> HDSL: Sync mode does not work 100m long cable
    <td> Position Sense HDSL
    <td> 10.0.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-8296
    <td> HDSL: Incorrect SAFE_SUM value is seen
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
</table>

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
<tr>
    <td> MCUSDK-208
    <td> gmake with -j can sometimes lock up Windows command prompt
    <td> Build
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Use bash for windows as part of git for windows or don't use -j option
</tr>
</table> -->

## Upgrade and Compatibility Information for Motor Control SDK 10.02.00 {#UPGRADE_AND_COMPATIBILITY_INFORMATION_10_02_00}

This section lists changes which could affect user applications developed using older SDK versions.
Read this carefully to see if you need to do any changes in your existing application when migrating to this SDK version relative to
previous SDK version. Also refer to older SDK version release notes to see changes in earlier SDKs.

### Examples

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> Multiple variables
    <td> Refactor code to avoid unsigned to signed and signed to unsigned conversion
    <td> -
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
<tr>
    <td> Position Sense Endat
    <td> `Endat_ChRxInfo`
    <td> Added new variables: `Endat_ChRTInfo` and Removed variables: `resvdInt2`, `resvdInt3" and `recoveryTime`
    <td> -
</tr>
<tr>
    <td> Position Sense Endat
    <td> struct `pruss_xchg`
    <td> Added new variables: `enableRTM`
    <td> -
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure `pos_data_info`
    <td> Added variables `raw_data4`, `raw_data5`, `velocity` and `acc`
    <td> Needed for Nikon A-format version 3.0
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure `alm_bits`
    <td> Added variables `ov_spd_s`, `st_err_s`, `ps_err_s`, `busy_s` and `inc_err_s`
    <td> Needed for Nikon A-format version 3.0
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure `alm_bits`
    <td> Updated variable `inc_err`'s name to `inc_err`
    <td> Updated as per Nikon A-format version 3.0
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure \ref nikon_priv
    <td> Added parameters `tx_rx_clock_source` and `protocol_version`
    <td> Needed for configuring clock source selection and specifying protocol version respectively
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure \ref nikon_priv
    <td> Updated parameter `tx_mdf`
    <td> Updated the variable type from `uint32_t` to 2D array of type `uint32_t` to store different `tx_mdf` data for each channel and each MDF
</tr>

<tr>
    <td> Position Sense Nikon A-format
    <td> structure \ref nikon_priv
    <td> Updated parameter `mem_data`
    <td> Updated the variable type from 1D array of type `uint32_t` to 2D array of type `uint32_t` to store different `mem_data` data for each channel and each MDF
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure \ref nikon_priv
    <td> Added variable `velocity_coefficient`, `pm_alm_field`, and `bank_error`
    <td> Needed for Nikon A-format version 3.0
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> API \ref nikon_init
    <td> Added input parameters `tx_rx_clock_source` and `protocol_version`
    <td> Needed for configuring clock source selection and specifying protocol version respectively
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure \ref nikon_pruicss_xchg
    <td> Updated parameter `mdf_frame`
    <td> Updated the variable type from 1D array of type `uint32_t` to 2D array of type `uint32_t` to store different `mdf_frame` data for each channel and each MDF
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure \ref nikon_pruicss_xchg
    <td> Add parameter `num_mdf`
    <td> It stores the number of MDFs to be sent
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure \ref nikon_pruicss_xchg
    <td> Removed `delay_300us`, `delay_30ms`, `delay_10us` and added `delay_1us`
    <td> 300 us, 30 ms and 10 us delay values are not needed in firmware. 1 us is needed for delay between CDF and MDFs.
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure \ref nikon_pruicss_xchg
    <td> Renamed variable `icssg_clk` to `icss_clk`
    <td> -
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> API \ref nikon_get_pos
    <td> CRC is not removed while copying data into last raw data byte for all commands (`priv->pos_data_info[ch].raw_data<x>[enc_num]`)
    <td> -
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> APIs \ref nikon_get_pos, \ref nikon_init, \ref nikon_generate_cdf, \ref nikon_reverse_bits, \ref nikon_config_load_share
    <td> Refactor code to avoid unsigned to signed and signed to unsigned conversion. Update the data type of function arguments.
    <td> -
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> APIs \ref nikon_update_eeprom_addr, \ref nikon_update_eeprom_data, \ref nikon_update_id_code
    <td> Add channel as function argument, and update the data type based on bit width needed for address, data, and ID code
    <td> -
</tr>
</table>


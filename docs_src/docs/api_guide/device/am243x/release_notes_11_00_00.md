# Release Notes 11.00.00 {#RELEASE_NOTES_11_00_00_PAGE}

[TOC]

\attention Please refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention For release notes of Industrial Communications SDK and MCU+ SDK, please refer to <a href="@VAR_IC_SDK_DOCS_PATH/RELEASE_NOTES_11_00_00_PAGE.html" target="_blank"> @VAR_SOC_NAME Industrial Communications SDK Release Notes 11.00.00</a> and <a href="@VAR_MCU_SDK_DOCS_PATH/RELEASE_NOTES_11_00_00_PAGE.html" target="_blank"> @VAR_SOC_NAME MCU+ SDK Release Notes 11.00.00</a> respectively.

\note
        1. In Motor Control SDK 11.00.00, using the EtherCAT example from Industrial Communications SDK 11.00.00.13 requires downloading the Beckhoff SSC stack from the ETG website and rebuilding the library as described in the following documentation:
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
    <td>Single R5 core example for dual-servo motor drive reference design based on Universal Motor Control similar to other Sitara™ and C2000™ devices.
    <td>TIDEP-01032 EtherCAT-Connected Single-Chip Dual-Servo Motor Drive Reference Design
</tr>
<tr>
    <td>Nikon A-format version 3.0
    <td>Position Sense Nikon A-format
</tr>
<tr>
    <td>Support for up to 8 encoders in bus connection
    <td>Position Sense Nikon A-format
</tr>
<tr>
    <td>Refactor the examples, renamed the old example to snoop mode based example and added new examples for shadow mode which does not use snoop mode
    <td>Current Sense %SDFM
</tr>
<tr>
    <td>Phase Shift Example (PRU Based PWM Control)
    <td>PRU-ICSS PWM
</tr>
<tr>
    <td>Enabled configuration of IEP1 independently
    <td>PRU-ICSS PWM
</tr>
<tr>
    <td>Time Transmitter Receiver Example
    <td>Time Sync
</tr>
<tr>
    <td>Time Receiver Example (Supported only in debug mode)
    <td>Time Sync
</tr>
<tr>
    <td>SFRA Library
    <td>Real Time Libraries
</tr>
<tr>
    <td>Datalog Library and Example
    <td>Real Time Libraries
</tr>
<tr>
    <td>Independent IEP events for each axis in load share mode
    <td>Position Sense BiSS-C
</tr>
<tr>
    <td>Independent IEP events for each axis in load share mode
    <td>Position Sense Nikon A-format
</tr>
<tr>
    <td>SAMUX mode configuration from SysConfig
    <td>Position Sense
</tr>
<tr>
    <td>TX/RX clock source configuration from SysConfig
    <td>Position Sense
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
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode),point-to-point connection, control communication, automatic processing delay detection and compensation, interface speed of 1, 2, 5, 8, and 10 MHz, long cable (upto 100 meters), continuous mode, BP-AM2BLDCSERVO Boosterpack with LP-AM243
    <td> Daisy chaining, safety mode (safety CRC and sign-of-life counter)
</tr>
<tr>
    <td> EnDat
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode), recovery time for 2.2 command set, interface speed of 5 and 10 MHz, long cable (upto 100 meters), continuous mode, BP-AM2BLDCSERVO Boosterpack with LP-AM243
    <td> Encoder receive communication command
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
    <td> Nikon A-format version 2.1 and version 3.0, Single channel, Multi channel using single PRU core and three PRU cores (load share mode), point-to-point connection, bus connection up to 8 encoders, individual and multiple transmission mode with encoder addresses ranging between ENC1-ENC8, baud rates from 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz, up to 40-bit absolute position (single turn + multi turn) data with additional information, long cable (upto 100 meters), continuous mode, BP-AM2BLDCSERVO Boosterpack with LP-AM243
    <td> -
</tr>
<tr>
    <td> Tamagawa
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using single PRU core, absolute position, encoder ID, reset, EEPROM read, EEPROM write, 2.5 Mbps Encoder, continuous mode, BP-AM2BLDCSERVO Boosterpack with LP-AM243
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
    <td> PINDSW-5537
    <td> HDSL not working with 225 MHz PRU-ICSSG Core Clock Frequency
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-6544
    <td> SDFM: Incorrect samples seen intermittently with EPWM as SDFM clock
    <td> Current Sense %SDFM
    <td> 9.0 onwards
    <td> Use new example with shadow register based normal current sampling
</tr>
<tr>
    <td> PINDSW-7976
    <td> PRUICSS PWM : validation of number of pwm channels and pwm trip zone blocks are not correct in sysconfig
    <td> PRUICSS PWM
    <td> 9.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-8042
    <td> ReferenceDesign: Firmware binary path included multiple times
    <td> Reference Design
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-8087
    <td> Tamagawa: UART clock source is used for TX fifo
    <td> Position Sense Tamagawa
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-8220
    <td> Endat: Initialization clock is not 200 kHz when the clock source does not run at 192 MHz
    <td> Position Sense EnDat
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-8399
    <td> EnDat: Implement the recovery time as specified in section 3.3 of "Heidenhain Document D1128897-03-A-02.8: Safety with EnDat 2.2 and Non-Safe EnDat Master"
    <td> Position Sense EnDat
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-8564
    <td> EnDat: endat_recvd_process function does not handle when different types of encoders are connected
    <td> Position Sense EnDat
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9123
    <td> Nikon: Data reversal not done correctly for EEPROM and ID commands
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> Fixes done in \ref nikon_update_eeprom_addr, \ref nikon_update_eeprom_data, \ref nikon_update_id_code and \ref nikon_get_pos
</tr>
<tr>
    <td> PINDSW-9124
    <td> Nikon: Data reversal not done correctly for encoder status received in response
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> Fixes done in \ref nikon_get_pos
</tr>
<tr>
    <td> PINDSW-9127
    <td> Nikon: For commands 8 to 12, 9 requests are sent instead of 8
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> Fixes done in \ref nikon_get_pos
</tr>
<tr>
    <td> PINDSW-9128
    <td> Nikon: For EEPROM commands, request is sent 2 times from firmware
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> Fixes done in \ref nikon_get_pos, and updates in the application code for EEPROM command handling
</tr>
<tr>
    <td> PINDSW-9131
    <td> Nikon: EEPROM read for temperature does not use 10 bit data
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> Fixes done in \ref nikon_get_pos, and updated the application code for EEPROM command handling
</tr>
<tr>
    <td> PINDSW-9144
    <td> Nikon: For EEPROM/ID commands, same data is used for all channels in multi-channel mode
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> Added provision for different address/data per channel
</tr>
<tr>
    <td> PINDSW-9154
    <td> Nikon: CRC error seen with 16 Mbps in Single PRU Multi-channel mode
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9180
    <td> Nikon: 10 us delay between CDF-MDF and MDF-MDF is used for commands needing MDF
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9238
    <td> Nikon: IEP compare value is not set correctly for continuous mode
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> Change the units to IEP clock cycle count
</tr>
<tr>
    <td> PINDSW-9239
    <td> Nikon: nikon_config_periodic_trigger assumes fixed receive size of response
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> Remove receive size hard-coding in driver
</tr>
<tr>
    <td> PINDSW-9248
    <td> EnDat: RX arm delay and TST delay settings are hard-coded for a 200 MHz core clock
    <td> Position Sense EnDat
    <td> 9.2 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9255
    <td> Nikon: If Debug log/scan is removed and encoder resolution is fixed in application code, initialization fails
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> Add 0.5 seconds delay in application after powering up the encoder
</tr>
<tr>
    <td> PINDSW-9292
    <td> SDFM: Fast Detect only works if the Fast Detect option is enabled for channel 0
    <td> Current Sense %SDFM
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9312
    <td> Nikon: Data type of velocity and acceleration is unsigned integer
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9317
    <td> BiSS-C: bissc_update_data_len does not set number of encoders correctly
    <td> Position Sense BiSS-C
    <td> 9.2 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9385
    <td> EnDat: Recovery time does not work with EnDat 2.2 supplementary commands for certain EnDat frequencies
    <td> Position Sense EnDat
    <td> 9.2 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9406
    <td> EnDat/Nikon: Channel 2 does not work on PRU Slice 0 in load share mode
    <td> Position Sense EnDat, Position Sense Nikon
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9501
    <td> BiSS-C/Nikon: Synchronization in load share fails in certain cases due to race condition
    <td> Position Sense BiSS-C, Position Sense Nikon
    <td> 9.1 onwards
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
    <td> HDSL: Sync mode does not work 100m long cable
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-8296
    <td> HDSL: Incorrect SAFE_SUM value is seen
    <td> Position Sense HDSL
    <td> 11.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9179
    <td> Nikon: PRU Firmware gets stuck if encoder does not respond with number of bytes expected by driver
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
    <td> -
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

## Upgrade and Compatibility Information for Motor Control SDK 11.00.00 {#UPGRADE_AND_COMPATIBILITY_INFORMATION_11_0_0}

\attention When migrating from MCU+ SDK, see \ref MIGRATION_GUIDES for more details.

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
    <td rowspan="8"> Current Sense %SDFM
    <td> `SdfmPrms_s`
    <td> Added variable: `snoopModeEnable`
    <td> -
</tr>
<tr>
    <td> Example `icss_sdfm_nine_channel_load_share_snoop_mode`
    <td> Renamed the example name from `icss_sdfm_nine_channel_load_share_mode` to `icss_sdfm_nine_channel_load_share_snoop_mode`, Changed the INTC mapping between host channels and PRU events
    <td> This change identifies the sampling mode between Normal mode and Snoop mode. INTC mapping info is available at \ref SDFM_INTC_MAPPING
</tr>
<tr>
    <td> Example `icss_sdfm_three_channel_single_pru_snoop_mode`
    <td> Renamed the example name from `icss_sdfm_three_channel_single_pru_mode` to `icss_sdfm_three_channel_single_pru_snoop_mode`, Changed the INTC mapping between host channel and PRU event
    <td> This change identifies the sampling mode between Normal mode and Snoop mode. INTC mapping info is available at \ref SDFM_INTC_MAPPING
</tr>
<tr>
    <td> Example `icss_sdfm_nine_channel_load_share_mode`
    <td> Normal mode for sampling is used
    <td> To demonstrate shadow mode-based sampling, it uses Normal mode for sampling.
</tr>
<tr>
    <td> Example `icss_sdfm_nine_channel_with_continuous_mode`
    <td> Made Normal mode the default mode for sampling, and individual interrupt is used for all channels
    <td> Updated the callback function to read sample data. INTC mapping info is available at \ref SDFM_INTC_MAPPING
</tr>
<tr>
    <td> Example `icss_sdfm_three_channel_single_pru_mode`
    <td> Normal mode for sampling is used
    <td> To demonstrate shadow mode-based sampling, it uses Normal mode for sampling.
</tr>
<tr>
    <td> Example `icss_sdfm_three_channel_with_continuous_mode`
    <td> Made Normal mode the default mode for sampling, and individual interrupt is used for all channels
    <td> Updated the callback function to read sample data. INTC mapping info is available at \ref SDFM_INTC_MAPPING
</tr>
<tr>
    <td> Example `icss_sdfm_three_channel_with_phase_compensation`
    <td> Made Normal mode the default mode for sampling, Changed the INTC mapping between host channel and PRU event
    <td> INTC mapping info is available at \ref SDFM_INTC_MAPPING
</tr>
<tr>
    <td rowspan="7"> Position Sense BiSS-C
    <td> bissc_process_periodic_command()
    <td> Updated input parameters for the API cmp0 -> iep_reset_count, cmp3 -> ch0_trigger_count and added new parameters ch1_trigger_count, ch2_trigger_count
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> bissc_periodic_interface_init()
    <td> Updated input parameters for the API cmp0 -> iep_reset_count, cmp3 -> ch0_trigger_count and added new parameters ch1_trigger_count, ch2_trigger_count
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> structure bissc_periodic_interface
    <td> updated parameters for the structure cmp0 -> iep_reset_count, cmp3 -> ch0_trigger_count and added new parameters ch1_trigger_count, ch2_trigger_count
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> pruBisscIrqHandler()
    <td> Updated API name
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> txpruBisscIrqHandler()
    <td> Added new API Method for handling interrupt from txpru
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> rtuBisscIrqHandler()
    <td> Added new API Method for handling interrupt from rtu
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> PRU_TRIGGER_HOST_EVT,RTU_TRIGGER_HOST_EVT,TXPRU_TRIGGER_HOST_EVT
    <td> Updated PRU_TRIGGER_HOST_BISSC_EVT0,PRU_TRIGGER_HOST_BISSC_EVT1,PRU_TRIGGER_HOST_BISSC_EVT2 host event macro names
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td rowspan="2"> Position Sense Endat
    <td> structure `endat_periodic_interface`
    <td> Renamed the cmp event variables
    <td> Renamed variables: `cmp0` to `cmp0_count`, `cmp3` to `ch0_trigger_count`, `cmp5` to `ch1_trigger_count`, and `cmp6` to `ch2_trigger_count`
</tr>
<tr>
    <td> Firmware binary files
    <td> Renamed all binary files
    <td> Used encoder_receiver inclusive term
</tr>
<tr>
    <td rowspan="8"> Position Sense Nikon A-format
    <td> nikon_process_periodic_command()
    <td> Updated input parameters for the API cmp0 -> iep_reset_count, cmp3 -> ch0_trigger_count & added new parameters ch1_trigger_count & ch2_trigger_count
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> nikon_periodic_interface_init()
    <td>  Updated input parameters for the API cmp0 -> iep_reset_count, cmp3 -> ch0_trigger_count & added new parameters ch1_trigger_count & ch2_trigger_count
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> structure nikon_periodic_interface
    <td> updated parameters for the structure cmp0 -> iep_reset_count, cmp3 -> ch0_trigger_count & added new parameters ch1_trigger_count & ch2_trigger_count
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> pru_nikon_irq_handler()
    <td> Updated API name
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> txpru_nikon_irq_handler()
    <td> Added new API Method for handling interrupt from txpru
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> rtu_nikon_irq_handler()
    <td> Added new API Method for handling interrupt from rtu
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> PRU_TRIGGER_HOST_EVT,RTU_TRIGGER_HOST_EVT,TXPRU_TRIGGER_HOST_EVT
    <td> Updated PRU_TRIGGER_HOST_NIKON_EVT0,PRU_TRIGGER_HOST_NIKON_EVT1,PRU_TRIGGER_HOST_NIKON_EVT2 host event macro names
    <td> Change affects multi channel load share example
</tr>
<tr>
    <td> Multiple variables
    <td> Refactor code to avoid unsigned to signed and signed to unsigned conversion
    <td> -
</tr>
<tr>
    <td rowspan="2"> Position Sense Tamagawa
    <td> Firmware binary files
    <td> Renamed all binary files
    <td> Used encoder_receiver inclusive term
</tr>
<tr>
    <td> structure `tamagawa_periodic_interface`
    <td> Renamed the cmp event variables
    <td> Renamed variables: `cmp0` to `iep_reset_count`, `cmp3` to `periodic_trigger_count`
</tr>
<tr>
    <td> Real Time Libraries
    <td> -
    <td> Moved examples/dcl, examples/sfra, examples/transforms, examples/utilities to examples/rtlibs folder
    <td> All examples using Real Time Libraries are updated to use new path
</tr>
<tr>
    <td> Reference Design
    <td> Project for TIDEP-01032 EtherCAT-Connected Single-Chip Dual-Servo Motor Drive Reference Design
    <td> Refactored the software and FOC implementation based on universal motor control, similar to other Sitara™ and C2000™ devices.
    <td> -
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
    <td rowspan="2"> Current Sense %SDFM
    <td> \ref SDFM_Ctrl
    <td> Added variable: `sdfm_en_snoop_nc`
    <td> -
</tr>
<tr>
    <td>\ref SDFM_setFilterOverSamplingRatio
    <td> Added input parameter: `channel`
    <td> Take current channel number as input parameter
</tr>
<tr>
    <td rowspan="2"> Position Sense BiSS-C
    <td> structure \ref bissc_priv
    <td> Added variable `tx_rx_clock_source`
    <td> Needed for configuring clock source selection
</tr>
<tr>
    <td> API \ref bissc_init
    <td> Added input parameter `tx_rx_clock_source`
    <td> Needed for configuring clock source selection
</tr>
<tr>
    <td rowspan="6"> Position Sense Endat
    <td> structure \ref endat_priv
    <td> Added new variables: `pru_uart_clock`, `pru_clock`, `rx_clock_source` and `tx_clock_source`
    <td> -
</tr>
<tr>
    <td> structure \ref endat_init
    <td> Added new parameter: \ref endat_clock_config
    <td> -
</tr>
<tr>
    <td> structure \ref endat_priv
    <td> Renamed variables: `cmp0` to `iep_reset_count`, `cmp3` to `ch0_trigger_count`, `cmp5` to `ch1_trigger_count`, and `cmp6` to `ch2_trigger_count`
    <td> -
</tr>
<tr>
    <td> structure \ref cmd_supplement
    <td> Renamed variables: `cmp0` to `iep_reset_count`, `cmp3` to `ch0_trigger_count`, `cmp5` to `ch1_trigger_count`, and `cmp6` to `ch2_trigger_count`
    <td> -
</tr>
<tr>
    <td> structure `Endat_ChRxInfo`
    <td> Added new variables: struct `Endat_ChRTInfo` and Removed variables: `resvdInt2`, `resvdInt3` and `recoveryTime`
    <td> -
</tr>
<tr>
    <td> structure `pruss_xchg`
    <td> Added new variables: `enableRTM`
    <td> -
</tr>
<tr>
    <td rowspan="15"> Position Sense Nikon A-format
    <td> structure \ref pos_data_info
    <td> Added variables `raw_data4`, `raw_data5`, `velocity` and `acc`
    <td> Needed for Nikon A-format version 3.0
</tr>
<tr>
    <td> structure \ref alm_bits
    <td> Added variables `ov_spd_s`, `st_err_s`, `ps_err_s`, `busy_s` and `inc_err_s`
    <td> Needed for Nikon A-format version 3.0
</tr>
<tr>
    <td> structure \ref alm_bits
    <td> Updated variable `inc_err`'s name to `inc_err`
    <td> Updated as per Nikon A-format version 3.0
</tr>
<tr>
    <td> structure \ref nikon_priv
    <td> Added parameters `tx_rx_clock_source` and `protocol_version`
    <td> Needed for configuring clock source selection and specifying protocol version respectively
</tr>
<tr>
    <td> structure \ref nikon_priv
    <td> Updated parameter `tx_mdf`
    <td> Updated the variable type from `uint32_t` to 2D array of type `uint32_t` to store different `tx_mdf` data for each channel and each MDF
</tr>

<tr>
    <td> structure \ref nikon_priv
    <td> Updated parameter `mem_data`
    <td> Updated the variable type from 1D array of type `uint32_t` to 2D array of type `uint32_t` to store different `mem_data` data for each channel and each MDF
</tr>
<tr>
    <td> structure \ref nikon_priv
    <td> Added variable `velocity_coefficient`, `pm_alm_field`, and `bank_error`
    <td> Needed for Nikon A-format version 3.0
</tr>
<tr>
    <td> API \ref nikon_init
    <td> Added input parameters `tx_rx_clock_source` and `protocol_version`
    <td> Needed for configuring clock source selection and specifying protocol version respectively
</tr>
<tr>
    <td> structure \ref nikon_pruicss_xchg
    <td> Updated parameter `mdf_frame`
    <td> Updated the variable type from 1D array of type `uint32_t` to 2D array of type `uint32_t` to store different `mdf_frame` data for each channel and each MDF
</tr>
<tr>
    <td> structure \ref nikon_pruicss_xchg
    <td> Added parameter `num_mdf`
    <td> It stores the number of MDFs to be sent
</tr>
<tr>
    <td> structure \ref nikon_pruicss_xchg
    <td> Removed `delay_300us`, `delay_30ms`, `delay_10us` and added `delay_1us`
    <td> 300 us, 30 ms and 10 us delay values are not needed in firmware. 1 us is needed for delay between CDF and MDFs.
</tr>
<tr>
    <td> structure \ref nikon_pruicss_xchg
    <td> Renamed variable `icssg_clk` to `icss_clk`
    <td> -
</tr>
<tr>
    <td> API \ref nikon_get_pos
    <td> CRC is not removed while copying data into last raw data byte for all commands (`priv->pos_data_info[ch].raw_data<x>[enc_num]`)
    <td> -
</tr>
<tr>
    <td> APIs \ref nikon_get_pos, \ref nikon_init, \ref nikon_generate_cdf, \ref nikon_reverse_bits, \ref nikon_config_load_share
    <td> Refactor code to avoid unsigned to signed and signed to unsigned conversion. Update the data type of function arguments.
    <td> -
</tr>
<tr>
    <td> APIs \ref nikon_update_eeprom_addr, \ref nikon_update_eeprom_data, \ref nikon_update_id_code
    <td> Add channel as function argument, and update the data type based on bit width needed for address, data, and ID code
    <td> -
</tr>
<tr>
    <td rowspan="4"> Position Sense Tamagawa
    <td> \ref tamagawa_priv
    <td> Added new variable: `pru_clock`
    <td> -
</tr>
<tr>
    <td> \ref tamagawa_clk_cfg
    <td> Added new variables: `rx_clk_source`, `tx_clk_source` and `rx_os_rate`
    <td> -
</tr>
<tr>
    <td> \ref tamagawa_priv
    <td> Added new variables: `pru_uart_clock`, `rx_clock_source" and `tx_clock_source`
    <td> -
</tr>

<tr>
    <td> \ref tamagawa_priv
    <td> Renamed variables: `cmp0` to `iep_reset_count`, `cmp3` to `periodic_trigger_count`
    <td> -
</tr>
<tr>
    <td rowspan="2"> PRUICSS PWM
    <td> structure `PRUICSS_PWM_IEP_Attrs_s`
    <td> Added variables : `iep1IncrementValue`, `enableIep1`, `enableIep1ResetOnEpwm0_Sync`, `enableIep1ResetOnEpwm3_Sync`, `enableIep1ResetOnCompare0`, `enableIEP1ShadowMode`
    <td> -
</tr>
<tr>
    <td> API \ref PRUICSS_PWM_iepConfig
    <td> Updated definition enabling configuration of IEP1 independently
    <td> -
</tr>
<tr>
    <td> Real Time Libraries
    <td> -
    <td> Moved source/control, source/dcl, source/observers, source/sfra, source/transforms, source/utilities to source/rtlibs folder
    <td> All examples using Real Time Libraries are updated to use new path
</tr>
</table>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

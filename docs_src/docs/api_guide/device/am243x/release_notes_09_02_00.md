# Release Notes 09.02.00 {#RELEASE_NOTES_09_02_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention For release notes of Industrial Communications SDK and MCU+ SDK, please refer to <a href="@VAR_IC_SDK_DOCS_PATH/RELEASE_NOTES_09_02_00_PAGE.html" target="_blank"> @VAR_SOC_NAME Industrial Communications SDK Release Notes 09.02.00</a> and <a href="@VAR_MCU_SDK_DOCS_PATH/RELEASE_NOTES_09_02_00_PAGE.html" target="_blank"> @VAR_SOC_NAME MCU+ SDK Release Notes 09.02.00</a> respectively.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

\note Tamagawa over SoC UART example is not supported for AM243x

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
Three channel Nikon A-Format with one PRU-ICSSG Slice                                           | Position Sense Nikon A-Format
Point-to-point and bus connection support for Nikon A-Format                                    | Position Sense Nikon A-Format
Multiple baud rate support : 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz                        | Position Sense Nikon A-Format
Long cable (upto 100 meters)                                                                    | Position Sense Nikon A-Format
Updated HDSL position realignment algorithm                                                     | Position Sense HDSL
Readiness for BiSS Safety profile by supporting 16 bit CRC and sign-of-life counter             | Position Sense BiSS-C
Compare event based command trigger support                                                     | Position Sense BiSS-C
SysConfig Enhancements                                                                          | Current Sense %SDFM
SINC1 and SINC2 Filtering                                                                       | Current Sense %SDFM
Deadband Configuration                                                                          | PRU-ICSS PWM
Control Library                                                                                 | Real Time Libraries
Observer Library                                                                                | Real Time Libraries

## Device and Validation Information

SOC    | Supported CPUs  | Boards                                                                                                      | Host PC
-------|-----------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------
AM243x | R5F             | AM243x EVM (referred to as am243x-evm in code), \n AM243x LAUNCHPAD (referred to as am243x-lp in code)      | Windows 10 64b or Ubuntu 18.04 64b

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, M4F       | 12.7.0
SysConfig               | R5F, M4F       | 1.20.0, build 3587
TI ARM CLANG            | R5F, M4F       | 3.2.2.LTS
FreeRTOS Kernel         | R5F, M4F       | 10.4.3
Tiny USB                | R5F            | 0.14.0
LwIP                    | R5F            | STABLE-2_2_0_RELEASE
Mbed-TLS                | R5F            | mbedtls-2.13.1

## Key Features

<!-- ### Experimental Features

\attention Features listed below are early versions and should be considered as "experimental".
\attention Users can evaluate the feature, however the feature is not fully tested at TI side.
\attention TI would not support these feature on public e2e.
\attention Experimental features will be enabled with limited examples and SW modules.


Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
                                                                    |  -->

<!-- ### Features not supported in release -->


<!-- ### AM243X LAUNCHPAD not tested/not supported features

Below features are not support on AM243X LAUNCHPAD due to SOC or board constraints, -->


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
    <td> Three %SDFM channels on single PRU core, Nine %SDFM channels on three PRU cores (load share mode), %SDFM Sync with EPWM, Single/Double Normal Current Sampling per PWM cycle, Continuous Normal Current Sampling, High and Low Threshold Comparator (Over-current detction), Fast Detect, Phase Compensation, Zero Cross Detection, Trip Generation using PRU-ICSS TripZone, Tested with %SDFM clock from ECAP, Tested with 5MHz Clock from EPWM
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
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode), point-to-point connection, daisy chaining, control communication, automatic processing delay detection and compensation, interface speed of 1, 2, 5, 8, and 10 MHz, Long cable (upto 100 meters), safety CRC, sign-of-life counter, Boosterpack with AM243x-LP
    <td> -
</tr>
<tr>
    <td> EnDat
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode), Continuous mode, Recovery Time for 2.2 command set, Long cable (upto 100 meters), Boosterpack with AM243x-LP
    <td> Encoder receive communication command
</tr>
<tr>
    <td> HDSL
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using three PRU cores (load share mode), Free Run mode, Sync mode, Short Message Read and Write, Long Message Read and Write, Pipeline Channel Data, Long cable (upto 100 meters) with single channel Free Run mode, Boosterpack with AM243x-LP
    <td> 225 MHz PRU-ICSSG Core Clock based firmware, Multi-channel with long cables(100m length), Long cable (upto 100 meters) with sync mode
</tr>
<tr>
    <td> Nikon A-Format
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode), point-to-point connection, bus connection, ndividual and multiple transmission mode with encoder addresses ranging between ENC1-ENC8, baud rates from 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz, up to 40-bit absolute position (single turn + multi turn) data with additional information, long cable (upto 100 meters), Boosterpack with AM243x-LP
    <td> -
</tr>
<tr>
    <td> Tamagawa
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Absolute position, Encoder ID, Reset, EEPROM Read, EEPROM Write, 2.5 Mbps and 5 Mbps Encoder Support, Boosterpack with AM243x-LP
    <td> -
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
    <td> Transforms
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Clarke transformation, Park transformation, Inverse Park transformation, Space Vector Generation (SVGEN), Common-mode subtraction approach, DPWM Generation (Part of SVGEN), Maximum Modulation, Minimum Modulation, SVGEN current reconstruction for single-shunt (SVGENCURRENT), Phase voltage reconstruction in overmodulation (VOLTS_RECON)
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
    <td> PINDSW-6630
    <td> HDSL: POS bit is not set during initial fast position alignment
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7158
    <td> HDSL: Reset Value of SSUM bit is not correct
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7480
    <td> HDSL: Continuous short/long message requests cause PRU to get stuck
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7498
    <td> ROV is not working for the Industrial Communication SDK Examples which are part of Motor Control SDK
    <td> Examples
    <td> 9.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7560
    <td> PRU firmware build in Motor Control SDK is failing in Linux environment
    <td> Examples
    <td> 9.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7567
    <td> PRUICSS PWM : Mapping of pwm signals PWMm_0_NEG to PWMm_A1, PWMm_1_POS to PWMm_A2, PWMm_1_NEG to PWMm_B0, PWMm_2_POS to PWMm_B1 is not correct
    <td> PRUICSS PWM
    <td> 9.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7595
    <td> %SDFM: \ref SDFM_init API does not have support for ICSSG1
    <td> Current Sense %SDFM
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7642
    <td> Docs: The tool versions of older releases in Release Notes are not correct
    <td> Documentation
    <td> 9.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7696
    <td> Unable to use SBL OSPI boot mode for all the demo applications of Motor Control SDK
    <td> Examples
    <td> 9.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7756
    <td> Tamagawa: After stopping periodic trigger mode, firmware gets stuck
    <td> Position Sense Tamagawa
    <td> 9.1 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7758
    <td> BiSS-C: Firmware does not work when changing ICSSG Core Clock Frequency to 300MHz
    <td> Position Sense BiSS-C
    <td> 9.1 onwards
    <td> Clock dividers for Three Channel Peripheral Interface were not being set correctly for 300MHz
</tr>
<tr>
    <td> PINDSW-7940
    <td> Transforms: UART Log does not work in transforms_test example
    <td> Real Time Libraries
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
    <td> PINDSW-5537
    <td> HDSL not working with 225 MHz PRU-ICSSG Core Clock Frequency
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> Use 300 MHz frequency for PRU-ICSSG Core Clock
</tr>
<tr>
    <td> PINDSW-5690
    <td> HDSL: EDGE register is not updated
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-6486
    <td> HDSL: RSSI register shows higher values than expected for a non-noisy setup
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-6544
    <td> %SDFM: Incorrect samples seen intermittently with EPWM as %SDFM clock
    <td> Current Sense %SDFM
    <td> 9.0 onwards
    <td> Use 5MHz %SDFM clock from EPWM1 (tested with 5MHz clock from EPWM) or use PRU-ICSSG ECAP as %SDFM clock source
</tr>
<tr>
    <td> PINDSW-7130
    <td> HDSL: Few protocol resets seen during initialization with Free Run mode on AM243x-LP
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
    <td> HDSL : 100 meter cable length does not work for sync mode in 300MHz PRU clock
    <td> Position Sense HDSL
    <td> 9.0 onwards
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

## Limitations
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
    <td> 7.3.0
    <td> AM64x, AM243x
    <td> Use bash for windows as part of git for windows or don't use -j option
</tr>
</table>

## Upgrade and Compatibility Information for Motor Control SDK 09.02.00 {#UPGRADE_AND_COMPATIBILITY_INFORMATION_9_2_0}

\attention When migrating from MCU+ SDK, see \ref MIGRATION_GUIDES for more details.

This section lists changes which could affect user applications developed using Motor Control SDK 09.01.00. Read this carefully to see if you need to do any changes in your existing application when migrating to this SDK version relative to previous SDK version. Also refer to older SDK version release notes to see changes in earlier SDKs.

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

### Examples

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td>  Current Sense %SDFM
    <td>  Structure `SdfmPrms_s`
    <td>  Added a variable: `icssgInsId`
    <td>  -
</tr>
<tr>
    <td>  Position Sense BiSS-C
    <td>  \ref bissc_update_data_len
    <td>  Changed API parameter: `pru_num` to `ch_num`
    <td>  -
</tr>
<tr>
    <td>  Position Sense BiSS-C
    <td>  \ref bissc_generate_ctrl_cmd
    <td>  Added an API: bissc_generate_ctrl_cmd
    <td>  Needed to generate Hex equivalent command
</tr>
<tr>
    <td>  Position Sense BiSS-C
    <td>  Structure `bissc_periodic_interface`
    <td>  Removed `pruicss_cfg` and `pruicss_dmem` variables, and added variable `cmp0`
    <td>  Needed for time trigger
</tr>
<tr>
    <td>  Current Sense %SDFM
    <td>  Structure `SdfmPrms_s`
    <td>  Added sub structures for sdfm parameters: `SdfmClkPrms_s`, `SdfmCompFilterPrms_s`, `SdfmChannelPrms_s`
    <td>  Split SDFM parameters into multiple structures
</tr>
<tr>
    <td>  Position Sense EnDat
    <td>  Structure `endat_periodic_interface`
    <td>  Removed `pruicss_cfg` variable and added variable `cmp0`
    <td>  -
</tr>
<tr>
    <td>  Position Sense Tamagawa
    <td>  Structure `tamagawa_periodic_interface`
    <td>  Removed `pruicss_cfg` variable and added variable `cmp0`
    <td>  -
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
    <td> PRUICSS PWM
    <td> \ref PRUICSS_PWM_actionOnOutputCfgPwmSignalB0
    <td> Updated definition to configure action of PWMn_m_0_NEG signal in active, intial, trigger states
    <td> -
</tr>
<tr>
    <td> PRUICSS PWM
    <td> \ref PRUICSS_PWM_actionOnOutputCfgPwmSignalA1
    <td> Updated definition to configure action of PWMn_m_1_POS signal in active, intial, trigger states
    <td> -
</tr>
<tr>
    <td> PRUICSS PWM
    <td> \ref PRUICSS_PWM_actionOnOutputCfgPwmSignalB1
    <td> Updated definition to configure action of PWMn_m_1_NEG signal in active, intial, trigger states
    <td> -
</tr>
<tr>
    <td> PRUICSS PWM
    <td> \ref PRUICSS_PWM_actionOnOutputCfgPwmSignalA2
    <td> Updated definition to configure action of PWMn_m_2_POS signal in active, intial, trigger states
    <td> -
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> \ref SDFM_init
    <td> Added new parameter: PRU-ICSSG handle.
    <td> -
</tr>
<tr>
    <td> Position Sense BiSS-C
    <td> Structure `bissc_priv`
    <td> Added variables: `is_continuous_mode`, `ctrl_write_status[NUM_ED_CH_MAX]`, `ctrl_reg_address[NUM_ED_CH_MAX]`, `ctrl_reg_data[NUM_ED_CH_MAX]`, `ctrl_enc_id[NUM_ED_CH_MAX]`.
    <td> Required to generate Hex equivalent control command and continuous mode
</tr>
<tr>
    <td> Position Sense BiSS-C
    <td> \ref bissc_generate_ctrl_cmd
    <td> Added API: bissc_generate_ctrl_cmd.
    <td> Required to generate Hex equivalent control command
</tr>
<tr>
    <td> Position Sense BiSS-C
    <td> \ref bissc_get_totalchannels, \ref bissc_get_current_channel, \ref bissc_clear_data_len
    <td> Added API: bissc_get_totalchannels, bissc_get_current_channel, bissc_clear_data_len.
    <td> Required to offload priv variable assignments from examples
</tr>
<tr>
    <td> Position Sense BiSS-C
    <td> \ref bissc_config_host_trigger, \ref bissc_config_periodic_trigger
    <td> Updating priv->is_continuous_mode
    <td> Required for continuous mode
</tr>
<tr>
    <td> Position Sense BiSS-C
    <td> \ref bissc_command_wait
    <td> Checking for priv->is_continuous_mode
    <td> Required for continuous mode
</tr>
<tr>
    <td> Position Sense BiSS-C
    <td> \ref bissc_set_default_initialization
    <td> Updating Structure variables: `is_continuous_mode`, `ctrl_reg_address`, `ctrl_enc_id`
    <td> Required for continuous mode and control command generation
</tr>
<tr>
    <td>  Current Sense %SDFM
    <td>  Structure `SDFM_s`
    <td>  Added a variable: `gPruPwmHandle`
    <td>
</tr>
<tr>
    <td>  Current Sense %SDFM
    <td>  Structure `SDFM_s`
    <td>  Added a void pointer: `prussIep`
    <td>
</tr>
<tr>
    <td>  Current Sense %SDFM
    <td>  \ref SDFM_setCompFilterThresholds
    <td>  Updated parameter type
    <td>
</tr>
<tr>
    <td>  Current Sense %SDFM
    <td>  \ref SDFM_selectClockSource
    <td>  Updated parameter name and type
    <td>
</tr>
<tr>
    <td>  Position Sense EnDat
    <td>  Structure `cmd_supplement`
    <td>  Added a variable: cmp0
    <td>
</tr>
<tr>
    <td>  Position Sense EnDat
    <td>  Structure `endat_priv`
    <td>  Added a variable: cmp0
    <td>
</tr>
<tr>
    <td>  Position Sense Tamagawa
    <td>  Structure `tamagawa_priv`
    <td>  Added a variable: cmp0
    <td>
</tr>
<tr>
    <td> PRUICSS PWM
    <td> \ref PRUICSS_PWM_iepConfig
    <td> \ref PRUICSS_PWM_configureIepCompareEnable API call is removed from the \ref PRUICSS_PWM_iepConfig function
    <td> \ref PRUICSS_PWM_configureIepCompareEnable needs to be called separately to enable IEP cmp events
</tr>
</table>

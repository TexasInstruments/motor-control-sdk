# Release Notes 09.01.00 {#RELEASE_NOTES_09_01_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention For release notes of Industrial Communications SDK and MCU+ SDK, please refer to <a href="@VAR_IC_SDK_DOCS_PATH/RELEASE_NOTES_09_01_00_PAGE.html" target="_blank"> @VAR_SOC_NAME Industrial Communications SDK Release Notes 09.01.00</a> and <a href="@VAR_MCU_SDK_DOCS_PATH/RELEASE_NOTES_09_01_00_PAGE.html" target="_blank"> @VAR_SOC_NAME MCU+ SDK Release Notes 09.01.00</a> respectively.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

\note Tamagawa over SoC UART example is not supported for AM243x

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
EtherCAT-Connected, Single-Chip, Dual-Servo Motor Drive Reference Design (TIDEP-01032)          | Reference Design
Three channel BiSS-C with one PRU-ICSSG Slice                                                   | Position Sense BiSS-C
Point-to-point and daisy chaining connection support for BiSS-C                                 | Position Sense BiSS-C
Multi channel BiSS-C with daisy chaining                                                        | Position Sense BiSS-C
Multiple interface speed support                                                                | Position Sense BiSS-C
Long cable (upto 100 meters)                                                                    | Position Sense BiSS-C
Nine Channel ICSS %SDFM Examples                                                                | Current Sense %SDFM
ICSS %SDFM Example with Continuous Normal Current Sampling                                      | Current Sense %SDFM
ICSS %SDFM Example with Phase Compensation                                                      | Current Sense %SDFM
Fast Detect                                                                                     | Current Sense %SDFM
Trip Generation using PRU-ICSS TripZone                                                         | Current Sense %SDFM
Zero Crossing Detection                                                                         | Current Sense %SDFM
Three channel support with one PRU-ICSSG Slice                                                  | Position Sense HDSL
SYNC Mode support for 1 to 10 frames per cycle and 8 kHz to 50 kHz cycle frequency              | Position Sense HDSL
API support for Parameter Channel Long Message Read and Write                                   | Position Sense HDSL
Add support for PIPE_D register for SensorHub Channel                                           | Position Sense HDSL
Add support for PIPE_D register for SensorHub Channel                                           | Position Sense HDSL
Long cable (upto 100 meters)                                                                    | Position Sense HDSL
Two channel example with LP-AM243 + BP-AM2BLDCSERVO                                             | Position Sense HDSL
Compare event based command trigger support                                                     | Position Sense Tamagawa
Two channel example with LP-AM243 + BP-AM2BLDCSERVO                                             | Position Sense Tamagawa
Two channel example with LP-AM243 + BP-AM2BLDCSERVO                                             | Position Sense EnDAT
Three channel PRU-ICSS PWM Example                                                              | PRU-ICSS PWM
Transforms Library and Example                                                                  | Real Time Libraries

## Device and Validation Information

SOC    | Supported CPUs  | Boards                                                                                                      | Host PC
-------|-----------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------
AM243x | R5F             | AM243x EVM (referred to as am243x-evm in code), \n AM243x LAUNCHPAD (referred to as am243x-lp in code)      | Windows 10 64b or Ubuntu 18.04 64b

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, M4F, A53  | @VAR_CCS_VERSION
SysConfig               | R5F, M4F, A53  | @VAR_SYSCFG_VERSION, build @VAR_SYSCFG_BUILD
TI ARM CLANG            | R5F, M4F       | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | R5F, M4F, A53  | @VAR_FREERTOS_KERNEL_VERSION
FreeRTOS SMP Kernel     | A53            | @VAR_FREERTOS_SMP_KERNEL_VERSION

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
    <td> Single channel, Multi channel using three PRU cores (load share mode), Free Run mode, Sync mode, Short Message Read and Write, Long Message Read and Write, Pipeline Channel Data, Long cable (upto 100 meters), Boosterpack with AM243x-LP
    <td> 225 MHz PRU-ICSSG Core Clock based firmware
</tr>
<tr>
    <td> Tamagawa
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Absolute position, Encoder ID, Reset, EEPROM Read, EEPROM Write, 2.5 Mbps and 5 Mbps Encoder Support, Boosterpack with AM243x-LP
    <td> -
</tr>
<tr>
    <td> BiSS-C 
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using single PRU core and three PRU cores (load share mode), point-to-point connection, daisy chaining, control communication, automatic processing delay detection and compensation, interface speed of 1, 2, 5, 8, and 10 MHz, Long cable (upto 100 meters)
    <td> 
</tr>
</table>

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
    <td> PINDSW-6628
    <td> HDSL: Reset value of PRST bit is not correct
    <td> Position Sense HDSL
    <td> 9.0
    <td> -
</tr>
<tr>
    <td> PINDSW-6629
    <td> HDSL: SSUM bit in EVENT_S is not set when SUMMARY is non-zero
    <td> Position Sense HDSL
    <td> 9.0
    <td> SSUM bit in EVENT_S was masked with MASK_SUM which was incorrect.
</tr>
<tr>
    <td> PINDSW-6931
    <td> Tamagawa: Firmware build failing
    <td> Position Sense Tamagawa
    <td> 9.0
    <td> -
</tr>
<tr>
    <td> PINDSW-6944
    <td> HDSL: QM increment is not correct
    <td> Position Sense HDSL
    <td> 9.0
    <td> QM updates for Safe Channel 2 were incorrect.
</tr>
<tr>
    <td> PINDSW-7048
    <td> HDSL: SCE bit in ONLINE STATUS 1 gets cleared before seeing a correct CRC in safe channel
    <td> Position Sense HDSL
    <td> 9.0
    <td> -
</tr>
<tr>
    <td> PINDSW-7126
    <td> HDSL: Protocol reset is seen multiple times with certain encoders
    <td> Position Sense HDSL
    <td> 9.0
    <td> -
</tr>
<tr>
    <td> PINDSW-7129
    <td> HDSL: Stuffing Data in Learn state is incorrect with Free Run Mode
    <td> Position Sense HDSL
    <td> 9.0
    <td> -
</tr>
<tr>
    <td> PINDSW-7157
    <td> HDSL : Sync mode does not work for ES value greater than 1
    <td> Position Sense HDSL
    <td> 9.0
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
    <td> PINDSW-6630
    <td> HDSL: POS bit is not set during initial fast position alignment
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7130
    <td> HDSL: Few protocol resets seen during initialization with Free Run mode on AM243x-LP
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
    <td> PINDSW-7163
    <td> HDSL: Trailer data contains 4 zeros instead of 5
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

## Upgrade and Compatibility Information {#UPGRADE_AND_COMPATIBILITY_INFORMATION_9_1_0}

<!-- \attention When migrating from MCU+ SDK, see \ref MIGRATION_GUIDES for more details -->

This section lists changes which could affect user applications developed using older SDK versions.
Read this carefully to see if you need to do any changes in your existing application when migrating to this SDK version relative to
previous SDK version. Also refer to older SDK version release notes to see changes in
earlier SDKs.

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
    <td>
    <td>
    <td>
    <td>
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
    <td> Position Sense HDSL
    <td> \ref HDSL_write_pc_buffer
    <td> Updated parameters to take only buffer offset and data, instead of data for all buffers.
    <td> -
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> \ref SDFM_init
    <td> Added new parameter for PRU core Id.
    <td> -
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> \ref SDFM_configComparatorGpioPins
    <td> Removed parameter: `threshold_type`.
    <td> -
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> Structure: \ref SDFM_CfgTrigger
    <td> Added new variable: `en_continuous_mode`.
    <td> -
</tr>

<tr>
    <td> Current Sense %SDFM
    <td> Structure: \ref SDFM_ChCtrl
    <td> Added new variables: `en_fast_detect`, `en_phase_delay`, `clock_phase_delay` & `clock_edge`.
    <td> -
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> Structure: \ref SDFM_ThresholdParms 
    <td> Added new variables: `zeroCrossTh`, `zeroCrossThstatus`, `zeroCrossEn`, `lowThStatus` and `highThStatus`.
    <td> -
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> Structure: \ref SDFM_Cfg
    <td> Added new variables: `fd_window`, `fd_zero_max`, `fd_zero_min`, `fd_one_max` and `fd_one_min`.
    <td> -
</tr>
<tr>
    <td> Current Sense %SDFM
    <td> Structure: \ref SDFM_Interface
    <td> Added new variables: `sampleBufferBaseAdd` and `firmwareVersion`.
    <td> -
</tr>
</table>

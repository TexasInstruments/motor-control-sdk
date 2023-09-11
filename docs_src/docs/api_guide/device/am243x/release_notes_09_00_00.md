# Release Notes 09.00.00 {#RELEASE_NOTES_09_00_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention For release notes of Industrial Communications SDK and MCU+ SDK, please refer to \htmllink{@VAR_IC_SDK_DOCS_PATH/RELEASE_NOTES_09_00_00_PAGE.html, @VAR_SOC_NAME Industrial Communications SDK Release Notes 09.00.00} and \htmllink{@VAR_MCU_SDK_DOCS_PATH/RELEASE_NOTES_09_00_00_PAGE.html, @VAR_SOC_NAME MCU+ SDK Release Notes 09.00.00} respectively.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
SYNC Mode Support with 2 channels                                                               | Position Sense HDSL
Trigger based normal current sampling                                                           | Current Sense %SDFM
Double sampling per PWM cycle                                                                   | Current Sense %SDFM

## Device and Validation Information

SOC    | Supported CPUs  | Boards                                                                                                      | Host PC
-------|-----------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------
AM243x | R5F             | AM243x GP EVM (referred to as am243x-evm in code), \n AM243x LAUNCHPAD (referred to as am243x-lp in code)   | Windows 10 64b or Ubuntu 18.04 64b

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, M4F, A53  | @VAR_CCS_VERSION
SysConfig               | R5F, M4F, A53  | @VAR_SYSCFG_VERSION, build @VAR_SYSCFG_BUILD
TI ARM CLANG            | R5F, M4F       | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | R5F, M4F, A53  | @VAR_FREERTOS_KERNEL_VERSION
FreeRTOS SMP Kernel     | A53            | @VAR_FREERTOS_SMP_KERNEL_VERSION

\attention TI ARM CLANG @VAR_TI_ARM_CLANG_VERSION is not part of CCS by default, Follow steps at \htmllink{@VAR_MCU_SDK_DOCS_PATH/SDK_DOWNLOAD_PAGE.html#INSTALL_TIARMCLANG, TI CLANG Compiler Toolchain} to install the compiler.

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

Module       | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                                                                                            | Key features not tested
-------------|----------------|-------------------|-------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------
EnDat        | R5F            | YES               | FreeRTOS, NORTOS  | Single channel, Multi channel, Continuous mode for single channel, Load share mode, Recovery Time for 2.2 command set, Boosterpack with AM243x-LP              |  16 MHz Baud Rate Different cable lengths, Continuous clock mode for multi channel
HDSL         | R5F            | YES               | FreeRTOS, NORTOS  | Freerun mode(300MHz,225MHz), Sync mode(225MHz), Short Message Read & Write, Long Message Read & Write, Boosterpack with AM243x-LP                              |  Long cables
Tamagawa     | R5F            | YES               | FreeRTOS, NORTOS  | Absolute position, Encoder ID, Reset, EEPROM Read, EEPROM Write, 2.5 Mbps and 5 Mbps Encoder Support, Boosterpack with AM243x-LP                               |  -

### Current Sense


Module       | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                                                                                            | Key features not tested
-------------|----------------|-------------------|-------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------
%SDFM        | R5F            | YES               | FreeRTOS, NORTOS  |                                                                                                                                                                | -


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
    <td> PINDSW-5651
    <td> HDSL: Multi-turn bits of fast position do not contain correct data
    <td> Position Sense HDSL
    <td> -
    <td> -
</tr>
<tr>
    <td> PINDSW-5689
    <td> HDSL: High deviation in fast position when encoder shaft is fixed
    <td> Position Sense HDSL
    <td> -
    <td> -
</tr>
<tr>
    <td> PINDSW-6487
    <td> HDSL: FIX bits in ONLINE STATUS 1 register are losing the expected fix value
    <td> Position Sense HDSL
    <td> -
    <td> -
</tr>
<tr>
    <td> PINDSW-6488
    <td> HDSL: SUM/SSUM bit not working in ONLINE STATUS registers
    <td> Position Sense HDSL
    <td> -
    <td> -
</tr>
<tr>
    <td> PINDSW-6492
    <td> HDSL: Protocol reset is not working
    <td> Position Sense HDSL
    <td> -
    <td> -
</tr>
<tr>
    <td> PINDSW-6530
    <td> HDSL: QMLW bit not working in ONLINE STATUS registers
    <td> Position Sense HDSL
    <td> -
    <td> -
</tr>
<tr>
    <td> PINDSW-5538
    <td> HDSL: Long message not working with multi-channel application
    <td> Position Sense HDSL
    <td> -
    <td> -
</tr>
<tr>
    <td> PINDSW-6489
    <td> HDSL: Offsets for ONLINE STATUS registers in C structure are not correct
    <td> Position Sense HDSL
    <td> -
    <td> -
</tr>
<tr>
    <td> PINDSW-6607
    <td> %SDFM: NULL pointer dereferenced in \ref SDFM_getFilterData
    <td> Current Sense %SDFM
    <td> -
    <td> -
</tr>
<tr>
    <td> PINDSW-6526
    <td> HDSL: FREL/FRES bits in EVENT/EVENT_S registers are not sticky
    <td> Position Sense HDSL
    <td> -
    <td> -
</tr>
<tr>
    <td> PINDSW-5681
    <td> EnDat: Recovery Time not correct for 2.1 commands
    <td> Position Sense EnDat
    <td> -
    <td> -
</tr>
<tr>
    <td> PINDSW-5681
    <td> %SDFM: Sampling does not work with EPWM as %SDFM clock
    <td> Current Sense %SDFM
    <td> -
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
    <td> PINDSW-6628
    <td> HDSL: Reset value of PRST bit is not correct
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-6608
    <td> %SDFM: Incorrect samples seen intermittently with EPWM as %SDFM clock
    <td> Current Sense %SDFM
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-6630
    <td> HDSL: POS bit is not set during initial fast position alignment
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-6629
    <td> HDSL: SSUM bit in EVENT_S is not set when SUMMARY is non-zero
    <td> Position Sense HDSL
    <td> 9.0 onwards
    <td> -
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

## Upgrade and Compatibility Information {#UPGRADE_AND_COMPATIBILITY_INFORMATION_9_0_0}

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
    <td> Position Sense EnDat
    <td> \ref endat_init
    <td> Added api parameter
    <td> void* pruss_iep
</tr>
<tr>
    <td> Position Sense EnDat
    <td> Structure: endat_priv
    <td> Added variables: pruss_iep, cmp3, cmp5 and cmp6
    <td> 
</tr>
<tr>
    <td> Position Sense EnDat
    <td> Structure: cmd_supplement
    <td> Added variables: cmp3, cmp5 and cmp6
    <td> 
</tr>
</table>

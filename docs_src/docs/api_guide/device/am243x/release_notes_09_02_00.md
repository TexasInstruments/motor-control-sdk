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

## Device and Validation Information

SOC    | Supported CPUs  | Boards                                                                                                      | Host PC
-------|-----------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------
AM243x | R5F             | AM243x EVM (referred to as am243x-evm in code), \n AM243x LAUNCHPAD (referred to as am243x-lp in code)      | Windows 10 64b or Ubuntu 18.04 64b

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, M4F       | 12.6.0
SysConfig               | R5F, M4F       | 1.19.0, build 3426
TI ARM CLANG            | R5F, M4F       | 3.2.1.LTS
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
    <td> PINDSW-7567
    <td> PRUICSS PWM : Mapping of pwm signals PWMm_0_NEG to PWMm_A1, PWMm_1_POS to PWMm_A2, PWMm_1_NEG to PWMm_B0, PWMm_2_POS to PWMm_B1 is not correct
    <td> PRUICSS PWM 
    <td> 9.1
    <td> -
</tr>
<tr>
    <td> PINDSW-7595
    <td> %SDFM: \ref SDFM_init API does not have support for icssg1
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
</table>

## Upgrade and Compatibility Information {#UPGRADE_AND_COMPATIBILITY_INFORMATION_9_2_0}

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
    <td>  Current Sense %SDFM
    <td>  Structure `SdfmPrms_s`
    <td>  Added a variable: `icssgInsId`
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
</table>

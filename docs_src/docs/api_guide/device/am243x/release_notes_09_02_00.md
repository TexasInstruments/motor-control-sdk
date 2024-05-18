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
    <td> %SDFM: \ref SDFM_init API does not have support for ICSSG1
    <td> Current Sense %SDFM
    <td> 9.0 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7756
    <td> Tamagawa: After stopping periodic trigger mode, firmware gets stuck
    <td> Position Sense Tamagawa
    <td> 9.1
    <td> -
</tr>
<tr>
    <td> PINDSW-7758
    <td> BiSS-C: Firmware does not work when changing ICSSG Core Clock Frequency to 300 MHz
    <td> Position Sense BiSS-C
    <td> 9.1
    <td> Clock dividers for Three Channel Peripheral Interface were not being set correctly for 300 MHz
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

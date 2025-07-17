# Release Notes 11.00.00 {#RELEASE_NOTES_11_00_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention For release notes of Industrial Communications SDK and MCU+ SDK, please refer to <a href="@VAR_IC_SDK_DOCS_PATH/RELEASE_NOTES_11_00_00_PAGE.html" target="_blank"> @VAR_SOC_NAME Industrial Communications SDK Release Notes 11.00.00</a> and <a href="@VAR_MCU_SDK_DOCS_PATH/RELEASE_NOTES_11_00_00_PAGE.html" target="_blank"> @VAR_SOC_NAME MCU+ SDK Release Notes 11.00.00</a> respectively.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

\note Tamagawa over SoC UART example is not supported for AM243x

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
Nikon A-format version 3.0                                                                      | Position Sense Nikon A-format
Support for up to 8 encoders in bus connection                                                  | Position Sense Nikon A-format
Enabled configuration of IEP1 independently                                                     | PRU-ICSS PWM

\cond SOC_AM243X
SOC    | Supported CPUs  | Boards                                                                                                      | Host PC
-------|-----------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------
AM243x | R5F             | AM243x GP EVM (referred to as am243x-evm in code), \n AM243x LAUNCHPAD (referred to as am243x-lp in code)   | Windows 10 64b or Ubuntu 18.04 64b
\endcond

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, M4F, A53  | 12.8.1
SysConfig               | R5F, M4F, A53  | 1.22.0, build 3893
TI ARM CLANG            | R5F, M4F       | 4.1.0.LTS
GCC AARCH64             | A53            | 9.2-2019.12
GCC ARM                 | R5F            | 7-2017-q4-major (AM64x only)
FreeRTOS Kernel         | R5F, M4F, A53  | 11.1.0
FreeRTOS SMP Kernel     | A53            | 202110.00-SMP
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
    <td> HDSL
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Single channel, Multi channel using three PRU cores (load share mode), Free Run mode, Sync mode, Short Message Read and Write, Long Message Read and Write, Pipeline Channel Data, Long cable (upto 100 meters) with single channel Free Run mode, Boosterpack with LP-AM243, 225/300 MHz PRU firmware
    <td> Multi-channel with long cables(100m length), Long cable (upto 100 meters) with sync mode
</tr>
<tr>
    <td> Nikon A-format
    <td> R5F
    <td> YES
    <td> FreeRTOS, NORTOS
    <td> Nikon A-format version 2.1 and version3.0, Single channel, Multi channel using single PRU core and three PRU cores (load share mode), point-to-point connection, bus connection up to 7 encoders, Individual and multiple transmission mode with encoder addresses ranging between ENC1-ENC8, baud rates from 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz, up to 40-bit absolute position (single turn + multi turn) data with additional information, long cable (upto 100 meters), Boosterpack with LP-AM243
    <td> Bus connected with 8 encoders (Tested up to 7 encoders)
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
    <td> PINDSW-7976
    <td> PRUICSS PWM : validation of number of pwm channels and pwm trip zone blocks are not correct in sysconfig
    <td> PRUICSS PWM
    <td> 9.1 onwards
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
    <td> PINDSW-8399
    <td> EnDat: Implement the recovery time as specified in section 3.3 of "Heidenhain Document D1128897-03-A-02.8: Safety with EnDat 2.2 and Non-Safe EnDat Master"
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
    <td> EnDat: RX arm delay and TST delay settings are hard-coded for a 200MHz core clock
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
    <td> PINDSW-9308
    <td> Nikon: nikon_calc_clock API does not handle baud rate correctly
    <td> Position Sense Nikon A-format
    <td> 9.2 onwards
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
    <td> HDSL : 100 meter cable length does not work for sync mode in 300MHz PRU clock
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
</table>


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
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Use bash for windows as part of git for windows or don't use -j option
</tr>
</table>

## Upgrade and Compatibility Information for Motor Control SDK 11.00.00 {#UPGRADE_AND_COMPATIBILITY_INFORMATION_11_00_00}

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
    <td> Position Sense Endat
    <td> Firmware binary files
    <td> Renamed the all binary files
    <td> Used encoder_receiver inclusive term
</tr>
<tr>
    <td> Position Sense Tamagawa
    <td> Firmware binary files
    <td> Renamed the all binary files
    <td> Used encoder_receiver inclusive term
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> Multiple variables
    <td> Refactor code to avoid unsigned to signed and signed to unsigned conversion
    <td> -
</tr>
<tr>
    <td> Real Time Libraries
    <td> -
    <td> Moved examples/dcl, examples/sfra, examples/transforms, examples/utilities to examples/rtlibs folder
    <td> All examples using Real Time Libraries are updated to use new path
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
    <td> structure `PRUICSS_PWM_IEP_Attrs_s`
    <td> Added variables : `iep1IncrementValue`, `enableIep1`, `enableIep1ResetOnEpwm0_Sync`, `enableIep1ResetOnEpwm3_Sync`, `enableIep1ResetOnCompare0`, `enableIEP1ShadowMode`
    <td> -
</tr>
<tr>
    <td> PRUICSS PWM
    <td> API \ref PRUICSS_PWM_iepConfig
    <td> Updated definition enabling configuration of IEP1 independently
    <td> -
</tr>
<tr>
    <td> Position Sense BiSS-C
    <td> structure `bissc_priv`
    <td> Added variable `tx_rx_clock_source`
    <td> Needed for configuring clock source selection
</tr>
<tr>
    <td> Position Sense BiSS-C
    <td> API \ref bissc_init
    <td> Added input parameter `tx_rx_clock_source`
    <td> Needed for configuring clock source selection
</tr>
<tr>
    <td> Position Sense Endat
    <td> structure \ref endat_priv
    <td> Added new variables: `pru_uart_clock`, `pru_clock`, `rx_clock_source" and `tx_clock_source`
    <td> -
</tr>
<tr>
    <td> Position Sense Endat
    <td> structure \ref endat_init
    <td> Added new parameter: \ref endat_clock_config
    <td> -
</tr>
<tr>
    <td> Position Sense Endat
    <td> structure `Endat_ChRxInfo`
    <td> Added new variables: struct `Endat_ChRTInfo` and Removed variables: `resvdInt2`, `resvdInt3" and `recoveryTime`
    <td> -
</tr>
<tr>
    <td> Position Sense Endat
    <td> structure `pruss_xchg`
    <td> Added new variables: `enableRTM`
    <td> -
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure \ref pos_data_info
    <td> Added variables `raw_data4`, `raw_data5`, `velocity` and `acc`
    <td> Needed for Nikon A-format version 3.0
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure \ref alm_bits
    <td> Added variables `ov_spd_s`, `st_err_s`, `ps_err_s`, `busy_s` and `inc_err_s`
    <td> Needed for Nikon A-format version 3.0
</tr>
<tr>
    <td> Position Sense Nikon A-format
    <td> structure \ref alm_bits
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
<tr>
    <td> Position Sense Tamagawa
    <td> \ref tamagawa_priv
    <td> Added new variable: `pru_clock`
    <td> -
</tr>
<tr>
    <td> Position Sense Tamagawa
    <td> \ref tamagawa_clk_cfg
    <td> Added new variables: `rx_clk_source`, `tx_clk_source` and `rx_os_rate`
    <td> -
</tr>
<tr>
    <td> Position Sense Tamagawa
    <td> \ref tamagawa_priv
    <td> Added new variables: `pru_uart_clock`, `rx_clock_source" and `tx_clock_source`
    <td> -
</tr>
<tr>
    <td> Real Time Libraries
    <td> -
    <td> Moved source/control, source/dcl, source/observers, source/sfra, source/transforms, source/utilities to source/rtlibs folder
    <td> All examples using Real Time Libraries are updated to use new path
</tr>
</table>

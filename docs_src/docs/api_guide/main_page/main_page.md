# Introduction {#mainpage}

[TOC]

\cond SOC_AM64X || SOC_AM243X

Welcome to **@VAR_SDK_NAME for @VAR_SOC_NAME**. This SDK contains examples, libraries and tools to develop **RTOS and no-RTOS** based applications enabling real-time communication for position and current sense from motors, and real-time control libraries for **ARM R5F CPU** and related peripherals.


Real-time communication with encoders and current sensing is typically handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implement the low level firmware. The PRU-ICSS frees up the main ARM cores in the device for other functions, such as control and data processing.

Applications and PRU-ICSS firmwares for position sense and current sense are provided in the SDK.

\endcond

\cond SOC_AM263X

Welcome to **@VAR_SDK_NAME for @VAR_SOC_NAME**. This SDK contains examples, libraries and tools to develop **RTOS and no-RTOS** based applications enabling real-time communication for position sense from motors, and real-time control libraries for **ARM R5F CPU** and related peripherals.

\endcond

\attention This SDK also includes <a href="@VAR_IC_SDK_DOCS_PATH/index.html" target="_blank">@VAR_SOC_NAME Industrial Communications SDK</a> and <a href="@VAR_MCU_SDK_DOCS_PATH/index.html" target="_blank">@VAR_SOC_NAME MCU+ SDK</a>.

## Getting Started

To get started, see <a href="@VAR_MCU_SDK_DOCS_PATH/GETTING_STARTED.html" target="_blank"> GETTING STARTED </a> page.

\note To build examples from Industrial Communications SDK and MCU+ SDK using CCS projects, user has to add <b>${SDK_INSTALL_PATH}/ind_comms_sdk</b> path and <b>${SDK_INSTALL_PATH}/mcu_plus_sdk</b> path to "Product discovery path" respectively in CCS (from Window->Preferences->Code Composer Studio->Products)

## Block Diagram

Given below is a block diagram of the SW modules in this SDK

\image html block_diagram.png "Software Block Diagram"

The main software components in the block diagram specific to motor control are described below

\cond SOC_AM64X

<table>
<tr>
    <th>Software Components
    <th>Documentation Page
    <th>Description
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Position Sense Encoders**</td></tr>
<tr>
    <td> EnDat
    <td>\ref ENDAT
    <td> Driver and PRU-ICSS firmware for EnDat encoder interface
</tr>
<tr>
    <td> HDSL
    <td>\ref HDSL
    <td> Driver and PRU-ICSS firmware for Hiperface DSL encoder interface
</tr>
<tr>
    <td> Tamagawa
    <td>\ref TAMAGAWA
    <td> Driver and PRU-ICSS firmware for Tamagawa encoder interface
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Current Sense**</td></tr>
<tr>
    <td> %SDFM
    <td>\ref SDFM
    <td> Driver and firmware for current sense using Sigma-Delta Filtering Module
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Examples and Demos**</td></tr>
<tr>
    <td>Examples and Demos
    <td>\ref EXAMPLES
    <td>Examples and demos showing usage of different SW libraries and APIs for motor control
</tr>
</table>

\endcond

\cond SOC_AM243X

<table>
<tr>
    <th>Software Components
    <th>Documentation Page
    <th>Description
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Position Sense Encoders**</td></tr>
<tr>
    <td> EnDat
    <td>\ref ENDAT
    <td> Driver and PRU-ICSS firmware for EnDat encoder interface
</tr>
<tr>
    <td> HDSL
    <td>\ref HDSL
    <td> Driver and PRU-ICSS firmware for Hiperface DSL encoder interface
</tr>
<tr>
    <td> Tamagawa
    <td>\ref TAMAGAWA
    <td> Driver and PRU-ICSS firmware for Tamagawa encoder interface
</tr>
<tr>
    <td> BISS-C
    <td>\ref BISS-C
    <td> Driver and PRU-ICSS firmware for BISS-C encoder interface
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Current Sense**</td></tr>
<tr>
    <td> %SDFM
    <td>\ref SDFM
    <td> Driver and firmware for current sense using Sigma-Delta Filtering Module
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Real Time Libraries**</td></tr>
<tr>
    <td> Digital Control Library (DCL)
    <td>\ref DCL
    <td> Header-only library for digital control applications
</tr>
<tr>
    <td> Transforms
    <td>\ref Transforms
    <td> Transformation including Clarke, Park, Space Vector Generation used in motor control applications.
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Examples and Demos**</td></tr>
<tr>
    <td>Examples and Demos
    <td>\ref EXAMPLES
    <td>Examples and demos showing usage of different SW libraries and APIs for motor control
</tr>
</table>

\endcond

\cond SOC_AM263X
<table>
<tr>
    <th>Software Components
    <th>Documentation Page
    <th>Description
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Position Sense Encoders**</td></tr>
<tr>
    <td> Tamagawa
    <td>\ref TAMAGAWA_OVER_UART
    <td> Driver for Tamagawa encoder interface
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Real Time Libraries**</td></tr>
<tr>
    <td> Digital Control Library (DCL)
    <td>\ref DCL
    <td> Header-only library for digital control applications
</tr>
<tr>
    <td> Transforms
    <td>\ref Transforms
    <td> Transformation including Clarke, Park, Space Vector Generation used in motor control applications.
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Examples and Demos**</td></tr>
<tr>
    <td>Examples and Demos
    <td>\ref EXAMPLES
    <td>Examples and demos showing usage of different SW libraries and APIs for motor control
</tr>
</table>

\endcond

For details on software components of Industrial Communications SDK and MCU+ SDK, please refer to <a href="@VAR_IC_SDK_DOCS_PATH/index.html" target="_blank">@VAR_SOC_NAME Industrial Communications SDK</a> and <a href="@VAR_MCU_SDK_DOCS_PATH/index.html" target="_blank">@VAR_SOC_NAME MCU+ SDK</a> respectively.

## Directory Structure

Given below is a overview of the directory structure to help you navigate the SDK and related tools.

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/</td></tr>
<tr>
    <td>README_FIRST_@VAR_SOC_NAME.html
    <td>Open this file in a web browser to reach this user guide</td>
</tr>
<tr>
    <td>makefile
    <td>Top level makefile to build the whole SDK using "make"</td>
</tr>
<tr>
    <td>imports.mak
    <td>Top level makefile to list paths to dependent tools</td>
</tr>
<tr>
    <td>docs/
    <td>Offline browseable HTML documentation</td>
</tr>
<tr>
    <td>examples/
    <td>Example applications for @VAR_SOC_NAME, across multiple boards </td>
</tr>
<tr>
    <td>source/
    <td>Device drivers, middleware libraries and APIs</td>
</tr>
<tr>
    <td>ind_comms_sdk/
    <td>Industrial Communications SDK</td>
</tr>
<tr>
    <td>mcu_plus_sdk/
    <td>MCU+ SDK</td>
</tr>
</table>

Given below are the paths where the different tools needed outside the SDK, like CCS, SysConfig are installed by default in Windows.
In Linux, the tools are installed by default in ${HOME}/ti.

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr>
    <td>C:/ti/ccs@VAR_CCS_FOLDER_VERSION
    <td>Code composer studio</td>
</tr>
<tr>
    <td>C:/ti/sysconfig_@VAR_SYSCFG_VERSION
    <td>SysConfig. **NOTE**, SysConfig is also installed as part of CCS at ${CCS_INSTALL_PATH}/ccs/utils/sysconfig_x.x.x</td>
</tr>
<tr>
    <td>C:/ti/ti-cgt-armllvm_@VAR_TI_ARM_CLANG_VERSION
    <td>TI ARM CLANG compiler tool chain</td>
</tr>
</table>

## Licenses

The licensing information of this SDK, as well as any third-party components included which are made available under a number of other open-source licenses are enumerated as part of the manifest.
A complete manifest along with export control information is detailed here [\htmllink{../../docs/@VAR_SOC_MANIFEST,LINK}] and the SDK Software License Agreement (SLA) is here [\htmllink{../../license.txt,LINK}]

## Help and Support

For additional help and support, see https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum

## Documentation Credits

This user guide is generated using doxygen, v1.8.20. See https://www.doxygen.nl/index.html

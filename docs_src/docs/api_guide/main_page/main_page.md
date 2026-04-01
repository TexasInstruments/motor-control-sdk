# Introduction {#mainpage}

[TOC]

\cond SOC_AM243X

Welcome to **@VAR_SDK_NAME for @VAR_SOC_NAME**. This SDK contains examples, libraries and tools to develop **RTOS and no-RTOS** based applications enabling real-time communication for position and current sense from motors, and real-time control libraries for **Arm® Cortex®-R5F CPU** and related peripherals.

Real-time communication with encoders and current sensing is typically handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implement the low level firmware. The PRU-ICSS frees up the main Arm-based cores in the device for other functions, such as control and data processing.

Applications and PRU-ICSS firmwares for position sense and current sense are provided in the SDK.

\attention Motor Control SDK 9.x/11.x included the Industrial Communications SDK and MCU+ SDK in bundled ind_comms_sdk and mcu_plus_sdk folders. Starting with Motor Control SDK 2025.00.00, these folders are no longer included and must be downloaded separately. Motor Control SDK installer provides an option to install the required Industrial Communications SDK and MCU+ SDK.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

Welcome to **@VAR_SDK_NAME for @VAR_SOC_NAME**. This SDK contains examples, libraries and tools to develop **RTOS and no-RTOS** based applications enabling real-time communication for position sense from motors, and real-time control libraries for **Arm® Cortex®-R5F CPU** and related peripherals.

Real-time communication with encoders is typically handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implement the low level firmware. The PRU-ICSS frees up the main Arm-based cores in the device for other functions, such as control and data processing.

Applications and PRU-ICSS firmwares for position sense are provided in the SDK.

\attention Motor Control SDK 10.x included the Industrial Communications SDK and MCU+ SDK in bundled ind_comms_sdk and mcu_plus_sdk folders. Starting with Motor Control SDK 2025.00.00, these folders are no longer included and must be downloaded separately. Motor Control SDK installer provides an option to install the required Industrial Communications SDK and MCU+ SDK.

\endcond

\cond SOC_AM261X

Welcome to **@VAR_SDK_NAME for @VAR_SOC_NAME**. This SDK contains examples, libraries and tools to develop **RTOS and no-RTOS** based applications enabling real-time communication for position sense from motors for **Arm® Cortex®-R5F CPU** and related peripherals.

Real-time communication with encoders is typically handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implement the low level firmware. The PRU-ICSS frees up the main Arm-based cores in the device for other functions, such as control and data processing.

Applications and PRU-ICSS firmwares for position sense are provided in the SDK.

\attention Motor Control SDK 10.x included the Industrial Communications SDK and MCU+ SDK in bundled ind_comms_sdk and mcu_plus_sdk folders. Starting with Motor Control SDK 2025.00.00, these folders are no longer included and must be downloaded separately. Motor Control SDK installer provides an option to install the required Industrial Communications SDK and MCU+ SDK.

\endcond

## Getting Started

To get started, see <a href="@VAR_MCU_SDK_DOCS_PATH/GETTING_STARTED.html" target="_blank"> GETTING STARTED </a> page.

## Block Diagram

Given below is a block diagram of the software modules in this SDK

\image html block_diagram.png "Software Block Diagram"

The main software components in the block diagram specific to motor control are described below

\cond SOC_AM243X

<table>
<tr>
    <th>Software Components
    <th>Documentation Page
    <th>Description
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Position Sense Encoders**</td></tr>
<tr>
    <td> BISS-C
    <td>\ref BISS-C
    <td> Driver and PRU-ICSS firmware for BISS-C encoder interface
</tr>
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
    <td> Nikon A-format
    <td>\ref NIKON
    <td> Driver and PRU-ICSS firmware for Nikon A-format encoder interface
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
<tr><td colspan="5" bgcolor=#F0F0F0>**Real Time Libraries**</td></tr>
<tr>
    <td> Control algorithms
    <td> \ref CONTROL
    <td> Library for control algorithms and techniques used in motors such as PMSM and BLDC
</tr>
<tr>
    <td> Datalog
    <td>\ref DATALOG
    <td> Library for storing the real time values of user selectable variables
</tr>
<tr>
    <td> Digital Control Library (DCL)
    <td>\ref DCL
    <td> Header-only library for digital control applications
</tr>
<tr>
    <td> Observer algorithms
    <td>\ref OBSERVERS
    <td> Header-only library for encoder/observer algorithms used in both sensored/sensorless field oriented control (FOC)
</tr>
<tr>
    <td> Software Frequency Response Analyzer
    <td>\ref SFRA
    <td> Library that enables developers to quickly measure the frequency response of their digital power converter.
</tr>
<tr>
    <td> Transforms
    <td>\ref TRANSFORMS
    <td> Transformation including Clarke, Park, Space Vector Generation used in motor control applications.
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**PRU-ICSS PWM**</td></tr>
<tr>
    <td> PRU-ICSS PWM
    <td>\ref PRUICSS_PWM
    <td> Driver for using PRU-ICSS PWM Peripheral
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**TIDEP-01032 EtherCAT-Connected Single-Chip Dual-Servo Motor Drive Reference Design**</td></tr>
<tr>
    <td> Reference design
    <td>\ref REFERENCE_DESIGN
    <td>Reference design showcasing the ability of the AM243x device to support a fully integrated real-time servo motor drive control and industrial communication.
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**PRU-ICSS TIME SYNC**</td></tr>
<tr>
    <td>Timesync
    <td>\ref EXAMPLE_PRUICSS_TIME_SYNC
    <td>Example showing clock synchronization between multiple devices using PRU-ICSS.
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Examples and Demos**</td></tr>
<tr>
    <td>Examples and Demos
    <td>\ref EXAMPLES
    <td>Examples and demos showing usage of different SW libraries and APIs for position sense, current sense, PRU-ICSS PWM and real-time libraries.
</tr>
</table>

\endcond

\cond (SOC_AM263X || SOC_AM263PX)
<table>
<tr>
    <th>Software Components
    <th>Documentation Page
    <th>Description
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Position Sense Encoders**</td></tr>
<tr>
    <td> BISS-C
    <td>\ref BISS-C
    <td> Driver and PRU-ICSS firmware for BISS-C encoder interface
</tr>
<tr>
    <td> EnDat
    <td>\ref ENDAT
    <td> Driver and PRU-ICSS firmware for EnDat encoder interface
</tr>
<tr>
    <td> Nikon A-format
    <td>\ref NIKON
    <td> Driver and PRU-ICSS firmware for Nikon A-format encoder interface
</tr>
<tr>
    <td> Tamagawa
    <td>\ref TAMAGAWA
    <td> Driver and PRU-ICSS firmware for Tamagawa encoder interface
</tr>
<tr>
    <td> Tamagawa (Over UART)
    <td>\ref TAMAGAWA_OVER_UART
    <td> Driver for Tamagawa encoder interface using SoC UART
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Real Time Libraries**</td></tr>
<tr>
    <td> Control algorithms
    <td> \ref CONTROL
    <td> Library for control algorithms and techniques used in motors such as PMSM and BLDC
</tr>
<tr>
    <td> Datalog
    <td>\ref DATALOG
    <td> Library for storing the real time values of user selectable variables
</tr>
<tr>
    <td> Digital Control Library (DCL)
    <td>\ref DCL
    <td> Header-only library for digital control applications
</tr>
<tr>
    <td> Observer algorithms
    <td>\ref OBSERVERS
    <td> Header-only library for encoder/observer algorithms used in both sensored/sensorless field oriented control (FOC)
</tr>
<tr>
    <td> Software Frequency Response Analyzer
    <td>\ref SFRA
    <td> Library that enables developers to quickly measure the frequency response of their digital power converter.
</tr>
<tr>
    <td> Transforms
    <td>\ref TRANSFORMS
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

\cond SOC_AM261X
<table>
<tr>
    <th>Software Components
    <th>Documentation Page
    <th>Description
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Position Sense Encoders**</td></tr>
<tr>
    <td> BISS-C
    <td>\ref BISS-C
    <td> Driver and PRU-ICSS firmware for BISS-C encoder interface
</tr>
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
    <td> Nikon A-format
    <td>\ref NIKON
    <td> Driver and PRU-ICSS firmware for Nikon A-format encoder interface
</tr>
<tr>
    <td> Tamagawa
    <td>\ref TAMAGAWA
    <td> Driver and PRU-ICSS firmware for Tamagawa encoder interface
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Real Time Libraries**</td></tr>
<tr>
    <td> Control algorithms
    <td> \ref CONTROL
    <td> Library for control algorithms and techniques used in motors such as PMSM and BLDC
</tr>
<tr>
    <td> Datalog
    <td>\ref DATALOG
    <td> Library for storing the real time values of user selectable variables
</tr>
<tr>
    <td> Digital Control Library (DCL)
    <td>\ref DCL
    <td> Header-only library for digital control applications
</tr>
<tr>
    <td> Observer algorithms
    <td>\ref OBSERVERS
    <td> Header-only library for encoder/observer algorithms used in both sensored/sensorless field oriented control (FOC)
</tr>
<tr>
    <td> Software Frequency Response Analyzer
    <td>\ref SFRA
    <td> Library that enables developers to quickly measure the frequency response of their digital power converter.
</tr>
<tr>
    <td> Transforms
    <td>\ref TRANSFORMS
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

Given below is an overview of the directory structure to help you navigate the SDK and related tools.

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
</table>

Given below are the paths where the different tools needed outside the SDK, like CCS, SysConfig are installed by default in Windows.
In Linux, the tools are installed by default in ${HOME}/ti.

\cond SOC_AM243X
<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr>
    <td>C:/ti/ccs@VAR_CCS_VERSION_AM243X
    <td>Code composer studio</td>
</tr>
<tr>
    <td>C:/ti/sysconfig_@VAR_SYSCFG_VERSION_AM243X
    <td>SysConfig. **NOTE**, SysConfig is also installed as part of CCS at ${CCS_INSTALL_PATH}/ccs/utils/sysconfig_x.x.x</td>
</tr>
<tr>
    <td>C:/ti/ti-cgt-armllvm_@VAR_TI_ARM_CLANG_VERSION_AM243X
    <td>TI ARM CLANG compiler tool chain</td>
</tr>
</table>
\endcond

\cond SOC_AM263X
<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr>
    <td>C:/ti/ccs@VAR_CCS_VERSION_AM263X
    <td>Code Composer Studio</td>
</tr>
<tr>
    <td>C:/ti/sysconfig_@VAR_SYSCFG_VERSION_AM263X
    <td>SysConfig. **NOTE**, SysConfig is also installed as part of CCS at ${CCS_INSTALL_PATH}/ccs/utils/sysconfig_x.x.x</td>
</tr>
<tr>
    <td>C:/ti/ti-cgt-armllvm_@VAR_TI_ARM_CLANG_VERSION_AM263X
    <td>TI ARM CLANG compiler tool chain</td>
</tr>
</table>
\endcond

\cond SOC_AM263PX
<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr>
    <td>C:/ti/ccs@VAR_CCS_VERSION_AM263PX
    <td>Code Composer Studio</td>
</tr>
<tr>
    <td>C:/ti/sysconfig_@VAR_SYSCFG_VERSION_AM263PX
    <td>SysConfig. **NOTE**, SysConfig is also installed as part of CCS at ${CCS_INSTALL_PATH}/ccs/utils/sysconfig_x.x.x</td>
</tr>
<tr>
    <td>C:/ti/ti-cgt-armllvm_@VAR_TI_ARM_CLANG_VERSION_AM263PX
    <td>TI ARM CLANG compiler tool chain</td>
</tr>
</table>
\endcond

\cond SOC_AM261X
<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr>
    <td>C:/ti/ccs@VAR_CCS_VERSION_AM261X
    <td>Code Composer Studio</td>
</tr>
<tr>
    <td>C:/ti/sysconfig_@VAR_SYSCFG_VERSION_AM261X
    <td>SysConfig. **NOTE**, SysConfig is also installed as part of CCS at ${CCS_INSTALL_PATH}/ccs/utils/sysconfig_x.x.x</td>
</tr>
<tr>
    <td>C:/ti/ti-cgt-armllvm_@VAR_TI_ARM_CLANG_VERSION_AM261X
    <td>TI ARM CLANG compiler tool chain</td>
</tr>
</table>
\endcond

## Licenses

The licensing information of this SDK, as well as any third-party components included which are made available under a number of other open-source licenses are enumerated as part of the manifest.

A complete manifest along with export control information is detailed in "${SDK_INSTALL_PATH}/docs/manifest.html" file.

## Help and Support

For additional help and support, see https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum

## Documentation Credits

This user guide is generated using Doxygen, v1.8.20. See https://www.doxygen.nl/index.html

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

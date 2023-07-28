# Introduction {#mainpage}

[TOC]

\if SOC_AM64X
Welcome to **@VAR_SDK_NAME for @VAR_SOC_NAME**. This SDK contains examples, libraries and tools to develop **RTOS and no-RTOS** based applications for **ARM R5F, ARM M4F, ARM A53 (single core and SMP on both cores) CPUs** and related peripherals.
\else
Welcome to **@VAR_SDK_NAME for @VAR_SOC_NAME**. This SDK contains examples, libraries and tools to develop **RTOS and no-RTOS** based applications for **ARM R5F, ARM M4F CPUs** and related peripherals.
\endif

\cond SOC_AM64X
This SDK also contains examples to interface these ARM R5F, ARM M4F applications with **Processor SDK Linux** based Cortex-A applications.
\endcond

## Getting Started

To get started, see \ref GETTING_STARTED

## Block Diagram

Given below is a block diagram of the SW modules in this SDK,

\if SOC_AM64X
\imageStyle{am64x/block_diagram.png,width:70%}
\image html am64x/block_diagram.png "Software Block Diagram"
\else
\imageStyle{block_diagram.png,width:70%}
\image html block_diagram.png "Software Block Diagram"
\endif

The main software components in the block diagram are described below


<table>
<tr>
    <th>Software Components
    <th>Documentation Page
    <th>Description
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Protocol Stacks and Middleware**</td></tr>
<tr>
    <td>Motor Control
    <td>\ref EXAMPLES_MOTORCONTROL
    <td>Drive protocols for EnDAT, HDSL.
</tr>
<tr><td colspan="3" bgcolor=#F0F0F0>**Examples and Demos**</td></tr>
<tr>
    <td>Examples and Demos
    <td>\ref EXAMPLES
    <td>Examples and demos showing usage of different SW libraries and APIs
</tr>
</table>

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
    <td>Top level makefile to list paths to dependant tools</td>
</tr>
<tr>
    <td>docs/
    <td>Offline browseable HTML documentation</td>
</tr>
<tr>
    <td>examples/
    <td>Example applications for @VAR_SOC_NAME, across multiple boards, CPUs, NO-RTOS, RTOS</td>
</tr>
<tr>
    <td>source/
    <td>Device drivers, middleware libraries and APIs</td>
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

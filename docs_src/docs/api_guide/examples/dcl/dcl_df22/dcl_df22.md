#  DCL DF22 Test {#EXAMPLES_DCL_DF22}

[TOC]

Simple DCL DF22 controller demonstration

This example leverages a direct form 2 2nd order (DF22) compensator from
the digital control library (DCL).

It demonstrates how to setup and run the DF22 controller. The example first
initializes the datalog DCL_FDLOG to the test vector, run the controller
based on the test input and compare the output with the expected value.


## Files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${MOTOR_CONTROL_SDK_PATH}/examples/dcl/dcl_df22/</td></tr>
<tr>
    <td>df22_test.c</td>
    <td>Main function for DF22 testing</td>
</tr>
<tr>
    <td>df22_test.h</td>
    <td>Contains all initialization of needed strctures</td>
</tr>
<tr>
    <td>data/</td>
    <td>Contains pre-generated data files result compares with</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${MOTOR_CONTROL_SDK_PATH}/source/dcl</td></tr>
<tr>
    <td>dcl/</td>
    <td>Folder containing DCL library source</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_DCL_DF22_COMBO}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 ^              | r5fss0-0 freertos
 ^              | m4fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/dcl/dcl_df22

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 ^              | r5fss0-0 freertos
 ^              | m4fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/dcl/dcl_df22

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/dcl/dcl_df22

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination and build it using the CCS project menu (see \htmllink{@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html, Using SDK with CCS Projects}).
- **When using makefiles to build**, note the required combination and build using make command (see \htmllink{@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html, Using SDK with Makefiles}).
- Launch a CCS debug session and run the executable, see \htmllink{@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html, CCS Launch\, Load and Run}


### Sample Output

Shown below is a sample output when the application is run:

\code
DF22 test produced 0 error
\endcode

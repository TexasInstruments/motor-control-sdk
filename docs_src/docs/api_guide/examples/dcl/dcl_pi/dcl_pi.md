#  DCL PI Test {#EXAMPLES_DCL_PI}

[TOC]

Simple DCL PI controller demonstration

This example leverages a PI controller from the digital control library(DCL).
It demonstrates how to setup and run the PI controller. The example first
initializes the datalog DCL_FDLOG to the test vector, run the controller
based on the test input and compare the output with the expected value.


## Files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/dcl/dcl_pi/</td></tr>
<tr>
    <td>pi_test.c</td>
    <td>Main function for PI testing</td>
</tr>
<tr>
    <td>pi_test.h</td>
    <td>Contains all initialization of needed strctures</td>
</tr>
<tr>
    <td>data/</td>
    <td>Contains pre-generated data files result compares with</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/dcl</td></tr>
<tr>
    <td>dcl/</td>
    <td>Folder containing DCL library source</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_DCL_PI_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 ^              | r5fss0-0 freertos
 ^              | m4fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/dcl/dcl_pi

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
 Example folder | examples/dcl/dcl_pi

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/dcl/dcl_pi

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>).
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>



### Sample Output

Shown below is a sample output when the application is run:

\code
PI test produced 0 error
\endcode

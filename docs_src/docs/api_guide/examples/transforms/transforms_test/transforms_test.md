#  CLAKRE-PARK-ICLARKE-SVGEN Test {#EXAMPLES_TRANSFORMS_TEST}

[TOC]

Simple data test incorporating foc-related libraries

This example runs verifies various transforms libraries used in motor-control application,
specifically as a building block of Field of Control (FOC) motor design.

It demonstrates how to run clarke, park, inverse park transform and 
space-vector generation (svgen). This test trys to mimic a conventional
workflow of passing current values into clarke transform, then park, then
inverse park (PI controllers are neglected here but should exist for both open and close-loop design)
and finally passing to svgen to generate the three-phase ABC value. 

The test used an aribitary phasor angle that feeds into park/ipark, 
as opposed to an angle generation module required by an actual FOC design.

The test vector then compares the PARK, SVGEN results to ensure the integraty of the library.



## Files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/transforms/transforms_test</td></tr>
<tr>
    <td>transforms_test.c</td>
    <td>Main function for transforms library testing</td>
</tr>
<tr>
    <td>transforms_test.h</td>
    <td>Contains all initialization of test strctures and variables</td>
</tr>

<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/transforms</td></tr>
<tr>
    <td>clarke/</td>
    <td>Folder containing CLARKE library source</td>
</tr>
<tr>
    <td>park/</td>
    <td>Folder containing PARK library source</td>
</tr>
<tr>
    <td>ipark/</td>
    <td>Folder containing IPARK library source</td>
</tr>
<tr>
    <td>svgen/</td>
    <td>Folder containing SVGEN library source</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_TRANSFORMS_TEST_COMBO}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/transforms/transforms_test

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/transforms/transforms_test

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/transforms/transforms_test

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>).
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>

### Sample Output

Shown below is a sample output when the application is run, assuming when all test passes:

\code
Transforms test produced 0 error
\endcode

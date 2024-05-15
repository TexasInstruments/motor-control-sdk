#  Software frequency response analyzer Test {#EXAMPLES_SFRA_TEST}

[TOC]

This example demonstrates usage of SFRA library. PI controller is designed for plant of constant value for testing the functionality of SFRA. When "sfra1.start" variable provided with value "1" at run time will initiate the SFRA functionality for configured frequency vectors and calculates plant /open loop/ closed loop magnitude and phase vectors. These data points can be exported and used for boded plots generation.

## Files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr>
    <td>sfra_main.c</td>
    <td>Main function for SFRA Library testing</td>
</tr>
<tr>
    <td>sfra_main.h</td>
    <td>Main function for defining control loop functionality with SFRA</td>
</tr>
<tr>
    <td>sfra_examples_hal.c and sfra_examples_hal.h</td>
    <td>files for setting up interrupt and gpio profiling</td>
</tr>
<tr>
    <td>sfra_examples_settings.h</td>
    <td>User configurable SFRA defines</td>
</tr>
<tr>
    <td>sfra_f32.h</td>
    <td>SFRA library related header file</td>
</tr>
<tr>
    <td>dcl/</td>
    <td>Folder with DCL library related files</td>
</tr>
<tr>
    <td>sfra.syscfg</td>
    <td>EPWM Configuration as per testing requirement</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_SFRA_TEST_COMBO}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sfra

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>).
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>

## Sample Output

Shown below is a sample output when the application is run:

Magnitude and phase vector data values after SFRA initiated by providing "1" to sfra1.start is shown in figure-

\imageStyle{sfra_test.png,width:50%}
\image html sfra_test.png "Data values of calculated magnitude and phase vector in expression window"

\code
Test passed
\endcode


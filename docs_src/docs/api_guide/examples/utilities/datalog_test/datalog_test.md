#  Datalog Test {#EXAMPLES_DATALOG_TEST}

[TOC]

Simple data logging test incorporating datalog library with user configurable as follows:

<table>
<tr>
    <th>Parameters
    <th>Values
</tr>
<tr>
    <td>Number of data log channels</td>
    <td>4</td> 
</tr>
<tr>
    <td>Buffer size</td>
    <td>400</td> 
</tr>
<tr>
    <td>Data type</td>
    <td>32bit floating point</td> 
</tr>
<tr>
    <td>Trigger typ</td>
    <td>Auto trigger</td> 
</tr>
<tr>
    <td>Trigger Value</td>
    <td>5.0</td> 
</tr>
<tr>
    <td>Prescalar/size</td>
    <td>1</td> 
</tr>
</table>

This example verifies data logging capability and showcase datalog API usage. In this one float variable(i) will be incremented with "1" in every loop starting with "0" and with trigger value as "5". When *iptr[0] value will be monitered at auto trigger type and if it crosses trigger value, data logging is initiated and values provided to *iptr[] will be logged for every prescalar loop.

In datalog_input.h, user can use following defines as per data type:

<table>
<tr>
    <th>Data type
    <th>Defines
</tr>
<tr>
    <td>32 bit signed integer</td>
    <td>u_int32_t</td> 
</tr>
<tr>
    <td>32 bit unsigned integer</td>
    <td>u_uint32_t</td> 
</tr>
<tr>
    <td>64 bit signed integer</td>
    <td>u_int64_t</td> 
</tr>
<tr>
    <td>64 bit floating point</td>
    <td>u_float64_t</td> 
</tr>
<tr>
    <td>32 bit floating point</td>
    <td>u_float32_t</td> 
</tr>
<tr>
    <td>16 bit signed integer</td>
    <td>u_int16_t</td> 
</tr>
<tr>
    <td>16 bit unsigned integer</td>
    <td>u_uint16_t</td> 
</tr>
<tr>
    <td>8 bit signed integer</td>
    <td>u_int8_t</td> 
</tr>
<tr>
    <td>8 bit unsigned integer</td>
    <td>u_uint8_t</td> 
</tr>
</table>


## Files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr>
    <td>main.c</td>
    <td>Main function for datalog library testing</td>
</tr>
<tr>
    <td>datalog_test.c</td>
    <td>Contains all initialization of test strctures and variables with datalog functionality</td>
</tr>
<tr>
    <td>datalog.c</td>
    <td>datalog library source file</td>
</tr>
<tr>
    <td>datalog.h</td>
    <td>datalog library header file at "${SDK_INSTALL_PATH}/source/utilities/datalog/include"</td>
</tr>
<tr>
    <td>datalog_input.h</td>
    <td>User configurable datalog header file need to be changed at "${SDK_INSTALL_PATH}/source/utilities/datalog/include"</td>
</tr>
<tr>
    <td>datalog.syscfg</td>
    <td>syscfg file for application</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_DATALOG_TEST_COMBO}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/utilities/datalog

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>).
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>

## Sample Output

Shown below is a sample output when the application is run and datalog output observed in datalogBuff[][]:

\imageStyle{datalog_test.png,width:50%}
\image html datalog_test.png "Datalog Buffer values in expression window of CCS"

\code
- Datalog initiated with manual trigger!!
- Data-log buffers are updated !!
- Test passed !!
\endcode

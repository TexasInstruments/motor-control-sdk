#  EnDAT Diagnostic {#EXAMPLE_MOTORCONTROL_ENDAT}

[TOC]

The EnDat diagnostic application, described here,
demonstrates the EnDat receiver operation.

The EnDat driver provides a well defined set of APIs to expose EnDat
receiver interface.



The diagnostic invokes these APIs to
- initialize EnDat,
- select one configuration among concurrent multi channel with Encoders of Same make, multi channel with Encoders of Different Make and single channel configuration based on SysConfig.
- select the channel (channels in the case of concurrent multi channel with encoders of same make or multi channel with Encoders of Different Make),
- configure the host trigger mode,
- and run the firmware.
- If using "Multi Channel with Encoders of Different Make" configuration is selected" :
    - enable load share mode.
    - select primary core for global configuration.
    - configuration of synchronization bits.


Once these steps are executed,
- the driver waits for the EnDat to be initialized.
- It then sets clock frequency to 200KHz (as propagation delay is not yet compensated)
- and obtains the encoder details including serial number, position resolution etc, and displays on the console/UART.
- Based on the whether encoder is 2.2 or 2.1 type, it sets clock to either 8MHz or 1MHz respectively.
- While configuring clock, propagation delay is taken care using the automatically estimated propagation delay (user can override it too).
- In the case of concurrent Multi Channel with Encoders of Same Make or Multi Channel with Encoders of Different Make, if propagation delay between various channels are different, that too is automatically taken care.

Once initial setup is over,
- the diagnostic provides the user with a self explanatory menu.
- Two types of menu options are presented. One type (1-14) will send an EnDat command as per EnDat 2.2 specification.
- The other type (100-108) allows the user to configure clock frequency, various timing parameters, simulate motor control loop using 2.1 command as well as 2.2 command with safety (redundant position information), switch to continuous clock mode and monitor raw data.
- Concurrent multi channel with Encoder of Same Make configuration can work simultaneously for up-to three encoders with identical part number, all variants of 2.2 position commands as well as the 2.1 position command is supported and an additional option (109) to configure wire delay (useful when propagation delay in each channel is different) is available.
- Single PRU core handles enabled channels in single channel and Multi Channel with Encoders of Same Make configuration.
- Application by default, handles wire delay as required, the menu option provides a way to override it.

After the user selects an EnDat command,
- the diagnostic asks for more details to frame the command and performs a basic sanity check on the user entered values.
- Then the EnDat APIs are invoked to process the command set, set the host trigger bit and waiting until the host trigger bit cleared, If multi-channel with Encoders of Different make is used, these operations are done for each channel".
- The received EnDat is processed & validated using the defined APIs. The result is then presented to the user.

### Channel Selection In Sysconfig

\image html EnDat_channel_selection_In_sysconfig.PNG      "Channel Selection In Sysconfig"


\image html Endat_channel_selection_configuration.png     "EnDAT configuration seletion between Single/Multi channel "

### Endat Example Implementation

Following section describes the Example implementation of EnDat on ARM(R5F).
\image html Endat_Example_Implementation.png "Endat Example"

## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/position_sense/endat_diagnostic</td></tr>
<tr>
    <td>endat_diagnostic.c</td>
    <td>EnDAT diagnostic application</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/position_sense/endat</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing EnDAT firmware sources.</td>
</tr>
<tr>
    <td>driver/</td>
    <td>EnDAT diagnostic driver.</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_MOTORCONTROL_ENDAT_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1, TXPRU1 and RTUPRU1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/motorcontrol/endat_example

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1, TXPRU1 and RTUPRU1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/motorcontrol/endat_example

\endcond

# Steps to Run the Example

## Hardware Prerequisites

Other than the basic EVM setup mentioned in \htmllink{@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html, EVM Setup}, below additional HW is required to run this demo
- EnDAT encoder
- TIDA-00179 Universal Digital Interface to Absolute Position Encoders, http://www.ti.com/tool/TIDA-00179
- TIDEP-01015 3 Axis board
- Interface card connecting EVM and TIDEP-01015 3 Axis board

\cond SOC_AM243X
### Hardware Prerequisities for Booster Pack

- EnDat encoder
- AM243x-LP board 
- BP-AM2BLDCSERVO, https://www.ti.com/tool/BP-AM2BLDCSERVO
\endcond

## Hardware Setup

\imageStyle{EnDAT_Connections.png,width:40%}
\image html EnDAT_Connections.png "Hardware Setup"

\cond SOC_AM243X
## Hardware Setup(Using Booster Pack & AM243x-LP)
\imageStyle{EnDat_Booster_Pack.png,width:40%}
\image html EnDat_Booster_Pack.png  "Hardware Setup of Booster Pack + LP for EnDat"

#### Booster Pack Jumper Configuration
<table>
<tr>
    <th>Designator</th>
    <th>ON/OFF</th>
    <th>Description</th>
</tr>
<tr>
    <td>J11</td>
    <td>OFF</td>
    <td>VSENSE/ISENSE select</td>
</tr>
<tr>
    <td>J13</td>
    <td>OFF</td>
    <td>VSENSE/ISENSE select</td>
</tr>
<tr>
    <td>J17</td>
    <td>Pin 1-2 Connected</td>
    <td>%SDFM Clock Feedback Select</td>
</tr>
<tr>
    <td>J18/J19</td>
    <td>J18 OFF & J19 ON</td>
    <td>Axis 1: Encoder/Resolver Voltage Select</td>
</tr>
<tr>
    <td>J20/J21</td>
    <td>J20 ON & J21 OFF</td>
    <td>Axis 2: Encoder/Resolver Voltage Select</td>
</tr>
<tr>
    <td>J22</td>
    <td>OFF</td>
    <td>Axis 1: Manchester Encoding Select</td>
</tr>
<tr>
    <td>J23</td>
    <td>OFF</td>
    <td>Axis 2: Manchester Encoding Select</td>
</tr>
<tr>
    <td>J24</td>
    <td>OFF</td>
    <td>Axis 1: RS485/DSL MUX</td>
</tr>
<tr>
    <td>J25</td>
    <td>OFF</td>
    <td>Axis 2: RS485/DSL MUX</td>
</tr>
<tr>
    <td>J26</td>
    <td>OFF</td>
    <td>VSENSE/ISENSE Select</td>
</tr>
<tr>
    <td>J27</td>
    <td>ON</td>
    <td>3WIRE/%SDFM MUX</td>
</tr>
<tr>
    <td>J28</td>
    <td>OFF</td>
    <td>3WIRE MUX</td>
</tr>
</table>
\endcond
## Build, load and run

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see \htmllink{@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html, Using SDK with CCS Projects}).
- **When using makefiles to build**, note the required combination and build using
  make command (see \htmllink{@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html, Using SDK with Makefiles})
- Launch a CCS debug session and run the executable, see \htmllink{@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html, CCS Launch\, Load and Run}
- Refer to UART terminal for user interface menu options.

### Sample Output

Shown below is a sample output when the application is run:

\imageStyle{EnDAT_Initialization_UART_PRINT.png,width:60%}
\image html EnDAT_Initialization_UART_PRINT.png "EnDAT Usage"

### Test Case Description

<table>
    <tr>
        <th style="width:4%">S.No</th>
        <th>Test detail
        <th>Steps
        <th>Pass/fail crieteria
    </tr>
    <tr>
        <td style="text-align: center">1.</td>
        <td style="text-align: center">To check position value</td>
        <td>1. Enter 1 to select "Encoder send position values"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="4" style="text-align: center">2.</td>
        <td rowspan="4" style="text-align: center">To receive encoder's operating parameters (Error Message)</td>
        <td>1. Enter 2 to select "Selection of memory area"</td>
        <td></td>
    </tr>
    <tr>
        <td>2. Enter "B9"  in MRS code to select "Operating parameters"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 4 to select "Encoder to send parameter"</td>
        <td></td>
    </tr>
    <tr>
        <td>4. Enter 00 in "parameter address" for selecting "Error message"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="4" style="text-align: center">3.</td>
        <td rowspan="4" style="text-align: center">To receive encoder's operating parameters (Warning message)</td>
        <td>1. Enter 2 to select "Selection of memory area"</td>
        <td></td>
    </tr>
    <tr>
        <td>2. Enter "B9"  in MRS code to select "Operating parameters"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 4 to select "Encoder to send parameter"</td>
        <td></td>
    </tr>
    <tr>
        <td>4. Enter 01 in "parameter address" for selecting "Error message"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="4" style="text-align: center">4.</td>
        <td rowspan="4" style="text-align: center">To receive encoder's manufacture parameters for Endat 2.2</td>
        <td>1. Enter 2 to select "Selection of memory area"</td>
        <td></td>
    </tr>
    <tr>
        <td>2. Enter "BD"  in MRS code to select "Parameters of encoder manufacturer for Endat 2.2"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 4 to select "Encoder to send parameter"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>4. Enter 0 in "parameter address" for selecting "Status of additional info 1"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="5" style="text-align: center">5.</td>
        <td rowspan="5" style="text-align: center">To set values to encoder's operating parameters (Clear error message)</td>
        <td>1. Enter 2 to select "Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "B9"  in MRS code to select "Operating parameters"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 3 to select "Encoder to receive parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td>4. Enter 0 in "parameter address" for selecting "Error message"</td>
        <td></td>
    </tr>
    <tr>
        <td>5. Enter 0 in "parameter value"  for seting value in " Error message"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="5" style="text-align: center">6.</td>
        <td rowspan="5" style="text-align: center">To set values to encoder's operating parameters (Clear warning message)</td>
        <td>1. Enter 2 to select "Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "B9"  in MRS code to select "Operating parameters"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 3 to select "Encoder to receive parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td>4. Enter 01 in "parameter address" for selecting "Error message"</td>
        <td></td>
    </tr>
    <tr>
        <td>5. Enter 0 in "parameter value"  for seting value in " Error message"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="8" style="text-align: center">7.</td>
        <td rowspan="8" style="text-align: center">To set values to encoder's manufacturing parameters for Endat 2.2(Status of additional info)</td>
        <td>1. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td></td>
    </tr>
    <tr>
        <td>2. Enter "45"  in MRS code to select "Parameters of encoder manufacturer for Endat 2.2"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>4. Enter "BD"  in MRS code to select "Memory parameter (LSB)" of Additional Information 1</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>5. Enter 10 to select "Encoder send position values + Additional Information(s) and receive parameter"</td>
        <td></td>
    </tr>
    <tr>
        <td>6. Enter 0 in "parameter address" for selecting "Status of additional info"</td>
        <td></td>
    </tr>
    <tr>
        <td>7. Enter 1235 (or any 2 byte value) in "parameter value"  for seting value in " Status of additional info"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>8. Enter 8 to select "Encoder send position values + Additional Information(s)"<br>&emsp; <b>Note: Write is not permenant. When read again using Command 11, encoder will return the default value</b></td>
        <td style="text-align: center">Values followed by 0x45 represents last byte of the data received by encoder<br> Crc success </td>
    </tr>
    <tr>
        <td style="text-align: center">8.</td>
        <td style="text-align: center">To reset encoder</td>
        <td>1. Enter 5 to select "Encoder receive reset"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="3" style="text-align: center">9.</td>
        <td rowspan="3" style="text-align: center">To receive test values from encoder with port address "0"</td>
        <td>1. Enter 7 to select "Encoder receive test command"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter 0 in "enter port address" </td>
        <td> </td>
    </tr>
    <tr>
        <td>3. Enter 6 to select "Encoder send test values"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="3" style="text-align: center">10.</td>
        <td rowspan="3" style="text-align: center">To receive test values from encoder with port address "E" </td>
        <td>1. Enter 7 to select "Encoder receive test command"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "E" in "enter port address" </td>
        <td> </td>
    </tr>
    <tr>
        <td>3. Enter 6 to select "Encoder send test values"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="5" style="text-align: center">11.</td>
        <td rowspan="5" style="text-align: center">To check position value with aditional info.</td>
        <td>1. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>2. Enter "47" in MRS code to select "Acknowledge MRS code" of Additional Information 1</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>4. Enter "56" in MRS code to select "Asynchronous Position value word 1 LSB" of Additional Information 2</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>5. Enter 8 to select "Encoder send position values + Additional Information(s)"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="7" style="text-align: center">12.</td>
        <td rowspan="7" style="text-align: center">To receive encoder's operating parameters(error messege)
		+receive position value with  additional info
		</td>
        <td>1. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "45"  in MRS code to select "Memory parameter (LSB)" of Additional Information 1</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>4. Enter "B9"  in MRS code to select "Operating parameters"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>5. Enter 11 to select "Encoder send position values + Additional Information(s) and  send parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td>6. Enter 0 in "parameter address" for selecting "Error message"</td>
        <td style="text-align: center">CRC success</td>
    </tr>
    <tr>
        <td>7. Enter 8 to select "Encoder send position values + Additional Information(s)"</td>
        <td style="text-align: center">CRC success</td>
    </tr>
    <tr>
        <td rowspan="7" style="text-align: center">13.</td>
        <td rowspan="7" style="text-align: center">To receive encoder's manufacture parameters
		for Endat 2.2 +receive position value with  additional info
        <td>1. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "45"  in MRS code to select "Memory parameter (LSB)" of Additional Information 1</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>4. Enter "BD"  in MRS code to select "Parameters of encoder manufacturer for Endat 2.2"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>5. Enter 11 to select "Encoder send position values + Additional Information(s) and send parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td>6. Enter 0 in "parameter address" for selecting "Status of additional info 1"</td>
        <td> </td>
    </tr>
    <tr>
        <td>7. Enter 8 to select "Encoder send position values + Additional Information(s)"</td>
        <td style="text-align: center">CRC success</td>
    </tr>
    <tr>
        <td rowspan="5" style="text-align: center">14.</td>
        <td rowspan="5" style="text-align: center">To acknowledge MRS code for Endat 2.2 
        <td>1. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "47"  in MRS code to select "Acknowledge MRS code" of Additional Information 1</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area</td>
        <td> </td>
    </tr>
    <tr>
        <td>4. Enter "BD" or any other valid MRS code </td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>5. Enter 8 to select "Encoder send position values + Additional Information(s)"</td>
        <td style="text-align: center">Additional Information 1:0x47bd00<br>&emsp;&emsp;&emsp;&emsp;(for MRS code = BD) </td>
    </tr>
    <tr>
        <td rowspan="5" style="text-align: center">15.</td>
        <td rowspan="5" style="text-align: center">To set values to encoder's operating parameters (error message)
		+receive position value with additional info
        <td>1. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "B9"  in MRS code to select "Operating parameters"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 10 to select "Encoder send position values + Additional Information(s) and receive parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td>4. Enter 0 in "parameter address" for selecting "Error message"</td>
        <td> </td>
    </tr>
    <tr>
        <td>5. Enter 0 in "parameter value"  for seting value in "Error message"</td>
        <td style="text-align: center">CRC success</td>
    </tr>
    <tr>
        <td rowspan="5" style="text-align: center">16.</td>
        <td rowspan="5" style="text-align: center">To set values to encoder's manufacturing parameters for Endat 2.2(Status of additional info)
		+receive position value with additional info
		</td>
        <td>1. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "BD"  in MRS code to select "Parameters of encoder manufacturer for Endat 2.2"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 10 to select "Encoder send position values + Additional Information(s) and receive parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td>4. Enter 0 in "parameter address" for selecting "Status of additional info"</td>
        <td> </td>
    </tr>
    <tr>
        <td>5. Enter 0 in "parameter value"  for seting value in " Status of additional info"</td>
        <td style="text-align: center">CRC success</td>
    </tr>
    <tr>
        <td rowspan="7" style="text-align: center">17.</td>
        <td rowspan="7" style="text-align: center">To receive encoder's OEM (Original Equipment Manufacturer) data </td>
        <td>1. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "45"  in MRS code to select "Memory parameter (LSB)" of Additional Information 1</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>4. Enter A9 (or AB or AD)  in MRS code to select the OEM memory</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>5. Enter 11 to select "Encoder send position values + Additional Information(s) and send parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td>6. Enter 40 in "parameter address" </td>
        <td> </td>
    </tr>
    <tr>
        <td>7. Enter 8 to select "Encoder send position values + Additional Information(s)"</td>
        <td style="text-align: center">CRC success</td>
    </tr>
    <tr>
        <td rowspan="5" style="text-align: center">18.</td>
        <td rowspan="5" style="text-align: center">To set values in OEM memory area </td>
        <td>1. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "A9"  in MRS code to select "Operating parameters"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 10 to select "Encoder send position values + Additional Information(s) and receive parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td>4. Enter 40 in "parameter address" </td>
        <td> </td>
    </tr>
    <tr>
        <td>5. Enter E9 in "parameter value"</td>
        <td style="text-align: center">CRC success</td>
    </tr>
    <tr>
        <td rowspan="7" style="text-align: center">19.</td>
        <td rowspan="7" style="text-align: center">To reset encoder +receive position value with additional info</td>
        <td>1. Enter 12 to select "Encoder send position values + Additional Information(s) and receive error reset"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>1. Enter 14 to select "Encoder receive communication command"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>2. Enter ______in "enter encoder address" </td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter _____ in  "instruction hex value"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>1. Enter 14 to select "Encoder receive communication command"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>2. Enter ______in "enter encoder address" </td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter _____ in  "instruction hex value"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="2" style="text-align: center">20.</td>
        <td rowspan="2" style="text-align: center">Configure Clock </td>
        <td>1. Enter 100 to select "configure clock"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter ___ for clock frequency(in Hz)</td>
        <td style="text-align: center">CRC success(Tested up to 8MHz)</td>
    </tr>
    <tr>
        <td rowspan="3" style="text-align: center">21.</td>
        <td rowspan="3" style="text-align: center">Simulate motor control 2.1 position loop</td>
        <td>1. Enter 101 to select "Simulate motor control 2.1 position loop"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter 10000 to select "clock frequency"</td>
        <td> </td>
    </tr>
    <tr>
        <td>3. Rotate the rotor of motor and see the changes in Position value on UART</td>
        <td style="text-align: center">Position Values are changing when rotor moves </td>
    </tr>
    <tr>
        <td rowspan="2" style="text-align: center">22.</td>
        <td rowspan="2" style="text-align: center">Toggle raw data display</td>
        <td>1. Enter 102 to select "Toggle raw data display"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter 1 to select "Encoder send position value"</td>
        <td style="text-align: center">raw data can be displayed </td>
    </tr>
    <tr>
        <td style="text-align: center">23.</td>
        <td style="text-align: center">Configure TST delay</td>
        <td></td>
        <td> </td>
    </tr>
    <tr>
        <td rowspan="2" style="text-align: center">24.</td>
        <td rowspan="2" style="text-align: center">Start continuous mode</td>
        <td>1. Enter 104 to select "Start continuous mode"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Rotate the rotor of motor and see the changes in Position value on UART</td>
        <td style="text-align: center">Position Values are changing when rotor moves </td>
    </tr>
    <tr>
        <td rowspan="3" style="text-align: center">25.</td>
        <td rowspan="3" style="text-align: center">Configure rx arm counter</td>
        <td>1. Enter 105 to select "Configure rx arm counter" </td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter 0 to select channel 0(only for multi channel configuration) </td>
        <td> </td>
    </tr>
    <tr>
        <td>3. Enter ___ to "select time in ns"</td>
        <td> </td>
    </tr>
    <tr>
        <td rowspan="3" style="text-align: center">26.</td>
        <td rowspan="3" style="text-align: center">configure rx clock disable time</td>
        <td>1. Enter 106 to select "configure rx clock disable time"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter 0 to select channel 0(only for multi channel configuration)</td>
        <td> </td>
    </tr>
    <tr>
        <td>3. Enter ___ to "select time in ns"</td>
        <td> </td>
    </tr>
    <tr>
        <td rowspan="3" style="text-align: center">27.</td>
        <td rowspan="3" style="text-align: center">Simulate motor control 2.2 position loop(safety)</td>
        <td>1. Enter 107 to select "Simulate motor control 2.2 position loop"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter 10000 to select "clock frequency"</td>
        <td> </td>
    </tr>
    <tr>
        <td>3. Rotate the rotor of motor and see the changes in Position value on UART</td>
        <td style="text-align: center">Position Values are changing when rotor moves </td>
    </tr>
    <tr>
        <td rowspan="2" style="text-align: center">28.</td>
        <td rowspan="2" style="text-align: center">Configure propogation delay(td)</td>
        <td>1. Enter 108 to select configure propagation delay </td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter 0 to select channel 0(only for multi channel configuration)</td>
        <td> </td>
    </tr>
    <tr>
        <td rowspan="5" style="text-align: center">29.</td>
        <td rowspan="5" style="text-align: center">Configure Recovery time (tr) using EnDat 2.2 mode transmission</td>
        <td>1. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area" </td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "B9" in MRS code to select "Operating parameters"</td>
        <td> </td>
    </tr>
    <tr>
        <td>3. Enter 10 to select "Encoder send position values + Additional Information(s) and receive parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td>4. Enter 03 in "parameter address" for selecting "Initializing the functions"</td>
        <td> </td>
    </tr>
    <tr>
        <td>5. Enter 01 in "parameter value" for selecting low recovery time<br>&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; or <br>&nbsp; &nbsp;Enter 02 in "parameter value" for selecting high recovery time</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="2" style="text-align: center">30.</td>
        <td rowspan="2" style="text-align: center">To read Recovery Time</td>
        <td>1. Enter 8 to select "Encoder send position values + Additional Information(s)" </td>
        <td style="text-align: center">CRC Success </td>
    </tr>
    <tr>
        <td>2. Enter 110 for read recovery time from DMEM </td>
        <td style="text-align: center">Recovery Time is set to 1.25 us <= RT <= 3.75us or  10 us <= RT <= 30 us</td>
    </tr>
    <tr>
        <td rowspan="2" style="text-align: center">31.</td>
        <td rowspan="2" style="text-align: center">To test periodic continuous mode</td>
        <td>1. Enter 200 to enable periodic mode </td>
        <td style="text-align: center"> </td>
    </tr>
    <tr>
        <td>2. Enter 5000 (in ns) for position read time </td>
        <td style="text-align: center">Position Values are changing when rotor moves </td>
    </tr>
    <tr>
        <td rowspan="2" style="text-align: center">32.</td>
        <td rowspan="2" style="text-align: center">Long term test</td>
        <td>1. Enter 111 to enable Long time continuous mode  </td>
        <td></td>
    </tr>
    <tr>
        <td>2. press enter to stop the long term test</td>
        <td style="text-align: center">The result shows the number of position command sent and the number of CRC failures received</td>
    </tr>
</table>
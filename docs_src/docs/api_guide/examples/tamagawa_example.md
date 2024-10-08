# Tamagawa Diagnostic {#EXAMPLE_MOTORCONTROL_TAMAGAWA}
[TOC]
\note
Starting with MCU+ SDK version 08.05.00, the Tamagawa firmware and examples are based on EnDAT hardware interface from PRU-ICSSG.
## Introduction

Tamagawa diagnostic application does below,

- Configures pinmux, GPIO, UART, ICSS clock to 200MHz
- Initializes ICSS0-PRU1
- Loads the initialization section of PRU firmware & executes it

This application is controlled with a terminal interface using a serial over USB connection between the PC host and the EVM.
Please connect a USB cable between the PC and the EVM/LP.
A serial terminal application (like teraterm/ hyperterminal/ minicom) is then run on the host.
To configure, select the serial port corresponding to the port emulated over USB by the EVM.
The host serial port should be configured to 115200 baud, no parity, 1 stop bit and no flow control.

The Tamagawa receiver firmware running on ICSS0-PRU1 provides a defined interface. The Tamagawa diagnostic application interacts with the Tamagawa receiver firmware interface. It then presents the user with menu options to select Data ID code (as defined by Tamagawa) to be sent to the encoder. The application collects the data entered by the user and configures the relevant interface. Then via the Tamagawa receiver interface, the command is triggered. Once the command completion is indicated by the interface, the status of the transaction is checked. If the Status indicates success, the result is presented to the user.

## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/position_sense/tamagawa_diagnostic</td></tr>
<tr>
    <td>tamagawa_diagnostic.c</td>
    <td>Tamagawa diagnostic application</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/position_sense/tamagawa</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing TAMAGAWA PRU firmware sources.</td>
</tr>
<tr>
    <td>driver/</td>
    <td>Tamagawa diagnostic driver.</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_MOTORCONTROL_TAMAGAWA_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/position_sense/tamagawa_diagnostic

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER (3 channel and 1 channel examples), @VAR_LP_BOARD_NAME_LOWER (2 channel and 1 channel examples)
 Example folder | examples/position_sense/tamagawa_diagnostic

\endcond

# Steps to Run the Example

## Hardware Prerequisites
Other than the basic EVM setup mentioned in <a href="@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html" target="_blank"> EVM Setup </a>, additional hardware required to run this demo is mentioned below

- Tamagawa Encoder(s)
- <a href="http://www.ti.com/tool/TIDA-00179" target="_blank"> TIDA-00179 Universal Digital Interface to Absolute Position Encoders </a>
- <a href="../TIDEP-01015Rev E1.1(001)_Sch.pdf" target="_blank"> TIDEP-01015 3 Axis Board </a>
- <a href="../MS_TI_EVM_3-AXIS_INTERFACE_BOARD_SCH_REV_E1.pdf" target="_blank"> Interface card connecting EVM and TIDEP-01015 3 Axis </a>

\note For more design details of the TIDEP-01015 3 Axis Board, or Interface card connecting EVM and TIDEP-01015 3 Axis, please contact TI via E2E/FAE.

\cond SOC_AM243X
### Hardware Prerequisities for Booster Pack

- Tamagawa Encoder(s)
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> AM243x-LP Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
\endcond


## Hardware Setup

\imageStyle{Tamagawa_setup.jpg,width:60%}
\image html Tamagawa_setup.jpg "Hardware Setup for 3 channels on EVM"

\imageStyle{Tamagawa_connections.JPG,width:60%}
\image html Tamagawa_connections.JPG "Tamagawa Encoder Hardware Setup for 3 channels"

\cond SOC_AM243X
## Hardware Setup(Using Booster Pack & AM243x-LP)
\imageStyle{Tamagawa_Booster_Pack.png,width:40%}
\image html Tamagawa_Booster_Pack.png  "Hardware Setup of Booster Pack + LP for Tamagawa"

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
    <td>J19 installed: sets VSENSOR1 to 5.0V</td>
    <td>Axis 1: Encoder/Resolver Voltage Select</td>
</tr>
<tr>
    <td>J20/J21</td>
    <td>J21 installed: sets VSENSOR2 to 5.0V</td>
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

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using
  make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>
- Refer to UART terminal for user interface menu options.

### Sample Output

Shown below is a sample output when the application is run:

\imageStyle{Tamagawa_SampleOutput.JPG,width:60%}
\image html Tamagawa_SampleOutput.JPG "Tamagawa Sample Output"

### Test Case Description

<table>
    <tr>
        <th>Data_ID
        <th>Name
        <th>Description
        <th>Pass/fail Criteria
    </tr>
    <tr>
        <td>Data ID 0</td>
        <td>Data readout (absolute position data)</td>
        <td>In this command we will receive:
		Absolute rotor position value in field name ABS..
		Errors and warnings in field name SF(status field)
		</td>
        <td>CRC success with ABS, SF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 1</td>
        <td>Data readout (multi-turn data)</td>
        <td>In this command we will receive data about:
		No. of rotor turns in field name ABM.
		Errors and warnings in field name SF(status field).
		</td>
        <td>CRC success with ABM, SF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 2</td>
        <td>Endoder-ID</td>
        <td>In this command we will receive data about :
		Tamagawa encoder make-ID in ENID field .
		Errors and warnings in field name SF(status field)
		</td>
        <td>CRC success with ENID, SF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 3</td>
        <td>Data readout(absolute+multiturn+encoder-ID)</td>
        <td>In this command we will receive :
		Absolute rotor position value in field name ABS.
		No. of rotor turns in field name ABM.
		Tamagawa encoder make-ID in ENID field .
		Errors and warnings in field name SF(status field)
		Other warnings in field name ALMC
		</td>
        <td>CRC success with ABS, ENID, ABM, ALMC, SF, CF and CRC values printed in the terminal.</td>
    </tr>
    <tr>
        <td>Data ID 6</td>
        <td>Writing to EEPROM</td>
        <td>In this command you provide :
        Proper address of the EEPROM where you want to write
		Proper data that you want to write.<br>
        As a response you recieve:
        Control Field for EEPROM Write command
        EEPROM address that you want to write to
        Data that you want to write to the EEPROM
        CRC value
		</td>
        <td>CRC success with EDF, ADF, CF and CRC values printed in the terminal.</td>
    </tr>
    <tr>
        <td>Data ID D</td>
        <td>Readout from EEPROM</td>
        <td>In this command you provide :
        Proper address of the EEPROM that you want to read.<br>
		As a response you recieve:
        Control Field for EEPROM Write command
        EEPROM address that you want to write to
        Data that you want to write to the EEPROM
        CRC value
		</td>
        <td>CRC success with EDF, ADF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 7</td>
        <td>Reset-Error</td>
        <td>This command used to reset errors. </td>
        <td>CRC success with ABS, SF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 8</td>
        <td>Reset - absolute</td>
        <td>This command is used to reset absolute position data(ABS). In order to reset the ABS value, send this command 10 times and send Data ID 0. </td>
        <td>CRC success with ABS value set to 0 along with SF, CF and CRC values printed in the terminal.</td>
    </tr>    <tr>
        <td>Data ID C</td>
        <td>Reset - multiturn</td>
        <td>This command is used to reset multi-turn data(ABM). In order to reset the ABM value, send this command 10 times and send Data ID 1. </td>
        <td>CRC success with ABM value set to 0 along with SF, CF and CRC values printed in the terminal.</td>
    </tr>
</table>
# NIKON Diagnostic {#EXAMPLE_MOTORCONTROL_NIKON}
[TOC]

Nikon diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 200MHz,
- Initializes ICSS0-PRU1,
- Initializes defalt parameters, loads the PRU firmware & executes it.

This application is controlled with a terminal interface using a serial over USB connection between the PC host and the EVM.
Please connect a USB cable between the PC and the EVM/LP.
A serial terminal application (like teraterm/ hyperterminal/ minicom) is then run on the host.
To configure, select the serial port corresponding to the port emulated over USB by the EVM.
The host serial port should be configured to 115200 baud, no parity, 1 stop bit and no flow control.

The Nikon receiver firmware running on ICSS0-PRU1 provides a defined interface. The Nikon diagnostic application interacts with the Nikon receiver firmware interface. It then presents the user with menu options to select different commands. The application collects the data entered by the user and configures the relevant interface. Then via the Nikon receiver interface, the command is triggered. Once the command completion is indicated by the interface, the status of the transaction is checked. If the Status indicates success, the result is presented to the user.

## Channel Selection In Sysconfig

\image html nikon_syscfg_ch_sel.png      "Channel Selection In Sysconfig"

\image html Endat_channel_selection_configuration.png     "Nikon configuration seletion between Single/Multi channel "


## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/position_sense/nikon_diagnostic</td></tr>
<tr>
    <td>nikon_diagnostic.c</td>
    <td>Nikon diagnostic application</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/position_sense/nikon</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing Nikon PRU firmware sources.</td>
</tr>
<tr>
    <td>driver/</td>
    <td>Nikon diagnostic driver.</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_MOTORCONTROL_NIKON_COMBOS}

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1 (single channel, multi channel using single PRU)
 ^              | PRU1, RTU-PRU1, TXPRU1 (multi channel using three PRUs - load share mode)
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER (2 channel and 1 channel examples)
 Example folder | examples/position_sense/nikon_diagnostic

\endcond

# Steps to Run the Example

## Hardware Prerequisites

- Nikon Encoders
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> AM243x-LP Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

## Hardware Setup

\cond SOC_AM243X
### Hardware Setup(Using Booster Pack & AM243x-LP)
\imageStyle{AM243x_lp_bp_nikon_encoder_setup.png,width:40%}
\image html AM243x_lp_bp_nikon_encoder_setup.png  "Hardware Setup of Booster Pack + LP for Nikon"

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
    <td>SDFM Clock Feedback Select</td>
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
    <td>3WIRE/SDFM MUX</td>
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

\imageStyle{nikon_sample_output.png,width:60%}
\image html nikon_sample_output.png "Nikon Sample Output"

### Test Case Description

<table>
    <tr>
        <th>CMD_ID
        <th>CMD 
        <th>Description
        <th>Pass/fail Criteria
    </tr>
    <tr>
        <td>0</td>
        <td>ABS full 40 bit data request</td>
        <td>In this command we will receive:
		Absolute 40 bit data for Single encoder with status information.
		</td>
        <td>CRC success with Rotor angle, num rotations and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>1</td>
        <td>ABS lower 24bit data request</td>
        <td>In this command we will receive:
        Absolute lower 24 bit data for Single encoder with status information.
		</td>
        <td>CRC success with Rotor angle, num rotations and CRC values printed in the terminal.	
        </td>
    </tr>
    <tr>
        <td>2</td>
        <td>ABS upper 24bit data request</td>
        <td>In this command we will receive:
		Absolute upper 24 bit data for Single encoder with status information. 
		</td>
        <td>CRC success with Rotor angle, num rotations and CRC values printed in the terminal.	
        </td>
    </tr>
    <tr>
        <td>3</td>
        <td>Encoder status Request</td>
        <td>In this command we will receive:
		Encoder will send status information, Alarm bits and additional information. 
		</td>
        <td>Alarm bits, Encoder status bits printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>4</td>
        <td>ABS full 40 bit data request(MT)</td>
        <td>In this command we will receive:
		Absolute 40 bit data for Multiple encoders connected in bus with status information.
		</td>
        <td>CRC success with Rotor angle, num rotations and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>5</td>
        <td>ABS lower 24bit data request(MT)</td>
        <td>In this command we will receive:
        Absolute lower 24 bit data for Multiple encoders connected in bus with status information.
		</td>
        <td>CRC success with Rotor angle, num rotations and CRC values printed in the terminal.	
        </td>
    </tr>
    <tr>
        <td>6</td>
        <td>ABS upper 24bit data request(MT)</td>
        <td>In this command we will receive:
		Absolute upper 24 bit data for Multiple encoders connected in bus with status information.
		</td>
        <td>CRC success with Rotor angle, num rotations and CRC values printed in the terminal.	
        </td>
    </tr>
    <tr>
        <td>7</td>
        <td>Encoder status Request(MT)</td>
        <td>In this command we will receive:
		Encoder will send status information, Alarm bits and additional information for all encoders connected in bus. 
		</td>
        <td>Alarm bits, Encoder status bits printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>8</td>
        <td>Status flag clear request</td>
        <td>In this command we will receive:
		Encoder will send status information, Alarm bits and additional information after clearing the status flags. 
		</td>
        <td>Alarm bits, Encoder status bits printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>9</td>
        <td>Multiple turn data clear request</td>
        <td>In this command we will receive:
		Encoder will send status information, Alarm bits and additional information after clearing the Multiple turn data bits. 
		</td>
        <td>Alarm bits, Encoder status bits printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>10</td>
        <td>Status+ Multiple turn data clear request</td>
        <td>In this command we will receive:
		Encoder will send status information, Alarm bits and additional information after clearing the Status and Multiple turn data bits. 
		</td>
        <td>Alarm bits, Encoder status bits printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>11</td>
        <td>Encoder address setting I (one-to-one connection)</td>
        <td>In this command we will receive:
		Encoder address setting will be performed and status will be returned with ALM bits. 
		</td>
        <td>Alarm bits, Encoder status bits printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>12</td>
        <td>Single turn data zero preset</td>
        <td>In this command we will receive:
		Encoder will set single turn data bits to zero and returns status bits along with ALM bits. 
		</td>
        <td>Alarm bits, Encoder status bits printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>13</td>
        <td>EEPROM read request</td>
        <td>In this command we will receive:
		Encoder will send EEPROM register data along with the requested address information. 
		</td>
        <td>Received data CRC should match with calculated CRC and CRC success printed in the terminal.	
        </td>
    </tr>
    <tr>
        <td>14</td>
        <td>EEPROM write request</td>
        <td>In this command we will receive:
		Encoder will perform EEPROM register write the data specified by user in the specified address. 
		</td>
        <td>Received data CRC should match with calculated CRC and CRC success printed in the terminal.	
        </td>
    </tr>
    <tr>
        <td>15</td>
        <td>Temperature data request</td>
        <td>In this command we will receive:
		Encoder will send temperature information along with status. 
		</td>
        <td>Encoder Temperature and status bits printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>16</td>
        <td>Identification code read I</td>
        <td>In this command we will receive:
		Encoder will send identification code. 
		</td>
        <td>Encoder identification code will be printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>17</td>
        <td>Identification code read II(one-to-one connection)</td>
        <td>In this command we will receive:
		Encoder will send identification code irrespective of encoder address. 
		</td>
        <td>Encoder identification code will be printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>18</td>
        <td>Identification code write I</td>
        <td>In this command we will receive:
		Encoder will update identification code provide by user in its local register. 
		</td>
        <td>Encoder identification code will be printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>19</td>
        <td>Identification code write II(one-to-one connection)</td>
        <td>In this command we will receive:
		Encoder will update identification code provide by user in its local register irrespective of encoder address.
		</td>
        <td>Encoder identification code will be printed in the terminal along with CRC success.	
        </td>
    </tr>
    <tr>
        <td>20</td>
        <td>Encoder address setting II</td>
        <td>In this command we will receive:
		Encoder will update its own address field based on identification code provide by user.
		</td>
        <td>Encoder identification code and updated address will be printed in the terminal along with CRC success.	
        </td>
    </tr>
	<tr>
        <td>21</td>
        <td>ABS lower 17bit data request</td>
        <td>In this command we will receive:
        Absolute lower 17 bit data for Single encoder with status information.
		</td>
        <td>CRC success with Rotor angle, num rotations and CRC values printed in the terminal.	
        </td>
    </tr>
	<tr>
        <td>22</td>
        <td>ABS lower 17bit data request(MT)</td>
        <td>In this command we will receive:
        Absolute lower 17 bit data for encoders connected in bus with status information.
		</td>
        <td>CRC success with Rotor angle, num rotations and CRC values printed in the terminal.	
        </td>
    </tr>
	<tr>
        <td>27</td>
        <td>ABS lower 24bit + status request</td>
        <td>In this command we will receive:
        Absolute lower 24 bit data for Single encoder with status and alarm bits information.
		</td>
        <td>CRC success with Rotor angle, num rotations, alm and CRC values printed in the terminal.	
        </td>
    </tr>
	<tr>
        <td>28</td>
        <td>ABS lower 24bit + status request(MT)</td>
        <td>In this command we will receive:
        Absolute lower 24 bit data for encoders connected in bus with status and alarm bits information.
		</td>
        <td>CRC success with Rotor angle, num rotations, alm and CRC values printed in the terminal.	
        </td>
    </tr>
	<tr>
        <td>29</td>
        <td>ABS lower 24bit + Temperature data request</td>
        <td>In this command we will receive:
        Absolute lower 24 bit data for Single encoder with Temperature information.
		</td>
        <td>CRC success with Rotor angle, num rotations, Temperature and CRC values printed in the terminal.	
        </td>
    </tr>
	<tr>
        <td>30</td>
        <td>ABS lower 24bit + Temperature data request(MT)</td>
        <td>In this command we will receive:
        Absolute lower 24 bit data for encoders connected in bus with Temperature information.
		</td>
        <td>CRC success with Rotor angle, num rotations, Temperature and CRC values printed in the terminal.	
        </td>
    </tr>
	<tr>
        <td>32</td>
        <td>Start Continuous Mode</td>
        <td>In this command we will receive:
        Absolute lower 40 bit data for encoders connected in point to point / bus.
		</td>
        <td>CRC success with Rotor angle, num rotations and CRC stats printed in the terminal.	
        </td>
    </tr>
</table>

# NIKON Diagnostic {#EXAMPLE_MOTORCONTROL_NIKON}
[TOC]

\note A-Format® is a registered trademark of the Nikon Corporation.

\cond SOC_AM243X

Nikon diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 200MHz,
- Initializes ICSS0-PRU1,
- Initializes default parameters, loads the PRU firmware & executes it.

\note Nikon firmware will only run with ICSS Core Clock running at 200 MHz/300 MHz frequency or ICSS UART Clock running at 192 MHz. 225/250/333 MHz values are not supported for interface clock generation due to clock divider requirements.
\endcond

\cond SOC_AM261X

Nikon diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 225MHz,
- Initializes ICSS0-PRU1,
- Initializes default parameters, loads the PRU firmware & executes it.

\note Nikon firmware will only run with ICSS UART Clock running at 160 MHz frequency. ICSS Core Clock is configured at 225 MHz, and is not supported for interface clock generation due to clock divider requirements.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

Nikon diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 200MHz,
- Initializes ICSS-PRU0,
- Initializes default parameters, loads the PRU firmware & executes it.

\note Nikon firmware will only run with ICSS Core Clock running at 200 MHz frequency or ICSS UART Clock running at 192 MHz frequency.

\endcond

This application is controlled with a terminal interface using a serial over USB connection between the PC host and the EVM.
Please connect a USB cable between the PC and the EVM/LP.
A serial terminal application (like teraterm/ hyperterminal/ minicom) is then run on the host.
To configure, select the serial port corresponding to the port emulated over USB by the EVM.
The host serial port should be configured to 115200 baud, no parity, 1 stop bit and no flow control.

The Nikon receiver firmware running on ICSS-PRU provides a defined interface. The Nikon diagnostic application interacts with the Nikon receiver firmware interface. It then presents the user with menu options to select different commands. The application collects the data entered by the user and configures the relevant interface. Then via the Nikon receiver interface, the command is triggered. Once the command completion is indicated by the interface, the status of the transaction is checked. If the Status indicates success, the result is presented to the user.

\cond SOC_AM243X

## Channel Selection In Sysconfig

\image html nikon_syscfg_ch_sel.png      "Channel Selection In Sysconfig"

\image html Endat_channel_selection_configuration.png     "Nikon configuration selection between Single/Multi channel "

\endcond

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

\cond (SOC_AM263X || SOC_AM263PX)

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSS           | ICSSM
 PRU            | PRU0
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER (Single channel example)
 Example folder | examples/position_sense/nikon_diagnostic

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSS           | ICSSM1
 PRU            | PRU0 (single channel)
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/position_sense/nikon_diagnostic

\endcond

# Steps to Run the Example

## Hardware Prerequisites
\cond SOC_AM243X

- Nikon A-Format encoders
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> LP-AM243 </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

\cond SOC_AM261X

- Nikon A-Format encoders
- <a href="https://www.ti.com/tool/LP-AM261" target="_blank"> LP-AM261 </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

\cond SOC_AM263X

- Nikon A-Format encoders
- <a href="https://www.ti.com/tool/LP-AM263" target="_blank"> LP-AM263 </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

\cond SOC_AM263PX

- Nikon A-Format encoders
- <a href="https://www.ti.com/tool/LP-AM263P" target="_blank"> LP-AM263P </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond
## Hardware Setup

\cond SOC_AM243X
### Hardware Setup(Using Booster Pack & LP-AM243)
\imageStyle{AM243x_lp_bp_nikon_encoder_setup.png,width:40%}
\image html AM243x_lp_bp_nikon_encoder_setup.png  "Hardware Setup of Booster Pack + LP for Nikon"

\note
    - The PROC109A version of LP supports two channels
    - To enable the second channel on LP, SW6 needs to be turn OFF

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
    <td>Encoder and %SDFM Paths Select</td>
</tr>
<tr>
    <td>J28</td>
    <td>OFF</td>
    <td>AM243/AM263 Mode</td>
</tr>
</table>

\endcond

\cond SOC_AM261X
### Hardware Setup(Using Booster Pack & LP-AM261)
\imageStyle{AM261x_lp_bp_nikon_encoder_setup.png,width:40%}
\image html AM261x_lp_bp_nikon_encoder_setup.png  "Hardware Setup of Booster Pack + LP for Nikon"

#### LP-AM261 Jumper Configuration
<table>
<tr>
    <th>Designator</th>
    <th>ON/OFF</th>
    <th>Description</th>
</tr>
<tr>
    <td>J13</td>
    <td>Pin 1-2 Connected</td>
    <td>3V3 Supply to Booster Pack</td>
</tr>
<tr>
    <td>J26</td>
    <td>Pin 1-2 Connected</td>
    <td>5V0 Supply to Booster Pack</td>
</tr>
</table>

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
    <td>Encoder and %SDFM Paths Select</td>
</tr>
<tr>
    <td>J28</td>
    <td>OFF</td>
    <td>AM243/AM263 Mode</td>
</tr>
</table>

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

\cond SOC_AM263X

### Hardware Setup(Using Booster Pack & LP-AM263)
\imageStyle{AM263x_lp_bp_nikon_encoder_setup.png,width:40%}
\image html AM263x_lp_bp_nikon_encoder_setup.png  "Hardware Setup of Booster Pack + LP for Nikon"

#### LP-AM263 Jumper Configuration

\endcond

\cond SOC_AM263PX
### Hardware Setup(Using Booster Pack & LP-AM263P)
\imageStyle{AM263Px_lp_bp_nikon_encoder_setup.png,width:40%}
\image html AM263Px_lp_bp_nikon_encoder_setup.png  "Hardware Setup of Booster Pack + LP for Nikon"

#### LP-AM263P Jumper Configuration

\endcond

<table>
<tr>
    <th>Designator</th>
    <th>ON/OFF</th>
    <th>Description</th>
</tr>
<tr>
    <td>J13</td>
    <td>Pin 1-2 Connected</td>
    <td>3V3 Supply to Booster Pack</td>
</tr>
<tr>
    <td>J14</td>
    <td>Pin 1-2 Connected</td>
    <td>5V0 Supply to Booster Pack</td>
</tr>
</table>

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
    <td>OFF</td>
    <td>Encoder and %SDFM Paths Select</td>
</tr>
<tr>
    <td>J28</td>
    <td>OFF</td>
    <td>AM243/AM263 Mode</td>
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
        <td>In this command, encoder sends absolute 40 bit data for single encoder with status information.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>1</td>
        <td>ABS lower 24bit data request</td>
        <td>In this command, encoder sends absolute lower 24 bit data for single encoder with status information.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>1 (Nikon A-Format version 3.0 only, based on factory setting)</td>
        <td>ABS full 40bit data + velocity data request </td>
        <td>In this command, encoder sends absolute 40 bit data for single encoder with status and velocity information.
		</td>
        <td>CRC success with rotor angle, number of rotations, velocity and CRC values printed in the terminal.
        </td>
    </tr>
    <tr>
        <td>2</td>
        <td>ABS upper 24bit data request</td>
        <td>In this command, encoder sends absolute upper 24 bit data for single encoder with status information.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC values printed in the terminal.
        </td>
    </tr>
    <tr>
        <td>3</td>
        <td>Encoder status Request</td>
        <td>In this command, encoder sends status information, alarm bits and additional information.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-Format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>4</td>
        <td>ABS full 40 bit data request(MT)</td>
        <td>In this command, encoder sends absolute 40 bit data for multiple encoders connected in bus with status information.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>5</td>
        <td>ABS lower 24bit data request(MT)</td>
        <td>In this command, encoder sends absolute lower 24 bit data for multiple encoders connected in bus with status information.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>5 (Nikon A-Format version 3.0 only, based on factory setting)</td>
        <td>ABS full 40bit data + velocity data request </td>
        <td>In this command, encoder sends absolute 40 bit data for multiple encoders connected in bus with status and velocity information.
		</td>
        <td>CRC success with rotor angle, number of rotations, velocity and CRC values printed in the terminal.
        </td>
    </tr>
    <tr>
        <td>6</td>
        <td>ABS upper 24bit data request(MT)</td>
        <td>In this command, encoder sends absolute upper 24 bit data for multiple encoders connected in bus with status information.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC values printed in the terminal.
        </td>
    </tr>
    <tr>
        <td>7</td>
        <td>Encoder status Request(MT)</td>
        <td>In this command, encoder sends status information, alarm bits and additional information for all encoders connected in bus.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-Format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>8</td>
        <td>Status flag clear request</td>
        <td>In this command, encoder sends status information, alarm bits and additional information after clearing the status flags.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-Format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>9</td>
        <td>Multiple turn data clear request</td>
        <td>In this command, encoder sends status information, alarm bits and additional information after clearing the multiple turn data bits.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-Format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>10</td>
        <td>Status+ Multiple turn data clear request</td>
        <td>In this command, encoder sends status information, alarm bits and additional information after clearing the status and multiple turn data bits.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-Format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>11</td>
        <td>Encoder address setting I (one-to-one connection)</td>
        <td>In this command, encoder address setting will be performed and status will be returned with ALM bits.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-Format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>12</td>
        <td>Single turn data zero preset</td>
        <td>In this command, encoder sets single turn data bits to zero and returns status bits along with ALM bits.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-Format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
        <tr>
        <td> 8 to 12 (Nikon A-Format version 3.0 only, based on factory setting)</td>
        <td>ABS lower 24bit data request</td>
        <td>In this command, encoder sends absolute lower 24 bit data for single encoder with status information.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC values printed in the terminal.
        </td>
    </tr>
    <tr>
        <td>13</td>
        <td>EEPROM read request</td>
        <td>In this command, encoder sends EEPROM register data along with the requested address information.
		</td>
        <td>CRC success with matching address (received address should be the same as sent by user)
        </td>
    </tr>
    <tr>
        <td>13 (Nikon A-Format version 3.0 only) </td>
        <td>EEPROM read request with bank</td>
        <td>In this command, encoder sends EEPROM register data along with the requested address and bank information.
		</td>
        <td>CRC success with matching address and bank (received address and bank should be the same as sent by user)
        </td>
    </tr>
    <tr>
        <td>14</td>
        <td>EEPROM write request</td>
        <td>In this command, encoder performs write and acknowledges the data and address specified by user.
		</td>
        <td>CRC success with matching data and address (received data and address should be the same as sent by user)
        </td>
    </tr>
    <tr>
        <td>14 (Nikon A-Format version 3.0 only) </td>
        <td>EEPROM write request with bank</td>
        <td>In this command, encoder performs write and acknowledges the data, address and bank specified by user.
		</td>
        <td>CRC success with matching data, address and bank (received data, address and bank should be the same as sent by user)
        </td>
    </tr>
    <tr>
        <td>15</td>
        <td>Temperature data request</td>
        <td>In this command, encoder sends temperature information along with status.
		</td>
        <td>Encoder temperature and status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>16</td>
        <td>Identification code read I</td>
        <td>In this command, encoder sends identification code.
		</td>
        <td>Encoder identification code will be printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>16 (Nikon A-Format version 3.0 only) </td>
        <td>Velocity coefficient read</td>
        <td>In this command, encoder sends velocity coefficient.
		</td>
        <td>Encoder velocity coefficient will be printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>17</td>
        <td>Identification code read II(one-to-one connection)</td>
        <td>In this command, encoder sends identification code irrespective of encoder address.
		</td>
        <td>Encoder identification code will be printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>18</td>
        <td>Identification code write I</td>
        <td>In this command, encoder writes and acknowledges identification code provide by user in its local register.
		</td>
        <td>CRC success with matching identification code (received identification code should be the same as sent by user)
        </td>
    </tr>
        <tr>
        <td>18 (Nikon A-Format version 3.0 only) </td>
        <td>Velocity coefficient write</td>
        <td>In this command, encoder writes and acknowledges velocity coefficient.
		</td>
        <td>CRC success with matching velocity coefficient (received velocity coefficient should be the same as sent by user)
        </td>
    </tr>
    <tr>
        <td>19</td>
        <td>Identification code write II(one-to-one connection)</td>
        <td>In this command, encoder writes and acknowledges identification code provide by user in its local register irrespective of encoder address.
		</td>
        <td>CRC success with matching identification code (received identification code should be the same as sent by user)
        </td>
    </tr>
    <tr>
        <td>20</td>
        <td>Encoder address setting II</td>
        <td>In this command, encoder address is set based on identification code provided by user.
		</td>
        <td>CRC success with matching identification code (received identification code should be the same as sent by user)
        </td>
    </tr>
	<tr>
        <td>21</td>
        <td>ABS lower 17bit data request</td>
        <td>In this command, encoder sends absolute lower 17 bit data for single encoder with status information.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>22</td>
        <td>ABS lower 17bit data request(MT)</td>
        <td>In this command, encoder sends absolute lower 17 bit data for encoders connected in bus with status information.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>23 (Nikon A-Format version 3.0 only)</td>
        <td>ABS lower 24bit + velocity request</td>
        <td>In this command, encoder sends absolute lower 24 bit data for single encoder with velocity information.
		</td>
        <td>CRC success with rotor angle, number of rotations, velocity and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>24 (Nikon A-Format version 3.0 only)</td>
        <td>ABS lower 24bit + velocity request(MT)</td>
        <td>In this command, encoder sends absolute lower 24 bit data for encoders connected in bus with velocity information.
		</td>
        <td>CRC success with rotor angle, number of rotations, velocity and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>25 (Nikon A-Format version 3.0 only)</td>
        <td>ABS lower 24bit + velocity + acceleration request</td>
        <td>In this command, encoder sends absolute lower 24 bit data for single encoder with velocity and acceleration information.
		</td>
        <td>CRC success with rotor angle, number of rotations, velocity, acceleration and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>26 (Nikon A-Format version 3.0 only)</td>
        <td>ABS lower 24bit + velocity + acceleration request(MT)</td>
        <td>In this command, encoder sends absolute lower 24 bit data for encoders connected in bus with velocity and acceleration information.
		</td>
        <td>CRC success with rotor angle, number of rotations, velocity, acceleration and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>27</td>
        <td>ABS lower 24bit + status request</td>
        <td>In this command, encoder sends absolute lower 24 bit data for single encoder with status and alarm bits information.
		</td>
        <td>CRC success with rotor angle, number of rotations, alarm bits and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>28</td>
        <td>ABS lower 24bit + status request(MT)</td>
        <td>In this command, encoder sends absolute lower 24 bit data for encoders connected in bus with status and alarm bits information.
		</td>
        <td>CRC success with rotor angle, number of rotations, alarm bits and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>29</td>
        <td>ABS lower 24bit + temperature data request</td>
        <td>In this command, encoder sends absolute lower 24 bit data for single encoder with temperature information.
		</td>
        <td>CRC success with rotor angle, number of rotations, temperature and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>30</td>
        <td>ABS lower 24bit + temperature data request(MT)</td>
        <td>In this command, encoder sends absolute lower 24 bit data for encoders connected in bus with temperature information.
		</td>
        <td>CRC success with rotor angle, number of rotations, temperature and CRC values printed in the terminal.
        </td>
    </tr>
    <tr>
        <td>32 (Note: This is not a command with ID 32, it is UART option number 32)</td>
        <td>Start Continuous Mode</td>
        <td>In this command, encoder sends absolute lower 40 bit data for encoders connected in point to point / bus.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC stats printed in the terminal.
        </td>
    </tr>
</table>

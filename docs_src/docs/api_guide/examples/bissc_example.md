# BISS-C Diagnostic {#EXAMPLE_MOTORCONTROL_BISSC}
[TOC]

BISS-C diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 200MHz,
- Initializes ICSS0-PRU1,
- Initializes defalt parameters, loads the PRU firmware & executes it.

This application is controlled with a terminal interface using a serial over USB connection between the PC host and the EVM.
Please connect a USB cable between the PC and the EVM/LP.
A serial terminal application (like teraterm/ hyperterminal/ minicom) is then run on the host.
To configure, select the serial port corresponding to the port emulated over USB by the EVM.
The host serial port should be configured to 115200 baud, no parity, 1 stop bit and no flow control.

Connect the BISS-C encoder(s) via TIDA-00179 cards on a 3 axis board (TIDEP-01015) to the EVM.
On AM243x LP, a BISS-C encoder can be directly connected using a TIDA-00179 card.

The connections between AM243x LP and TIDA-00179 for Channel 0 are:

BISSC_CHANNEL0_TX -> PRG0_PRU1_GPO1,
BISSC_CHANNEL0_TX_ENABLE -> PRG0_PRU1_GPO2,
BISSC_CHANNEL0_RX -> PRG0_PRU1_GPO13,
BISSC_CHANNEL0_CLK -> PRG0_PRU1_GPO0.

The BISS-C receiver firmware running on ICSS0-PRU1 provides a defined interface. The BISS-C diagnostic application interacts with the BISS-C receiver firmware interface. It then presents the user with menu options to select Data ID code. The application collects the data entered by the user and configures the relevant interface. Then via the BISS-C receiver interface, the command is triggered. Once the command completion is indicated by the interface, the status of the transaction is checked. If the Status indicates success, the result is presented to the user.

## Channel Selection In Sysconfig

\image html bissc_syscfg_ch_sel.png      "Channel Selection In Sysconfig"


\image html Endat_channel_selection_configuration.png     "BiSS-C configuration seletion between Single/Multi channel "


## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/motor_control/bissc_diagnostic</td></tr>
<tr>
    <td>bissc_diagnostic.c</td>
    <td>BISS-C diagnostic application</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/motor_control/position_sense/bissc</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing BISS-C PRU firmware sources.</td>
</tr>
<tr>
    <td>driver/</td>
    <td>BISS-C diagnostic driver.</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_MOTORCONTROL_BISSC_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/motor_control/bissc_diagnostic

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER (E3 Revision)
 Example folder | examples/motor_control/bissc_diagnostic

\endcond

# Steps to Run the Example

## Hardware Prerequisites
Other than the basic EVM setup mentioned in \htmllink{@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html, EVM Setup}, additional hardware required to run this demo is mentioned below
-  BISS-C Encoders
-  TIDA-00179 Universal Digital Interface to Absolute Position Encoders, http://www.ti.com/tool/TIDA-00179
-  TIDEP-01015 3 Axis board
-  Interface card connecting EVM and TIDEP-01015 3 Axis board

\cond SOC_AM243X
### Hardware Prerequisities for Booster Pack

- BISS-C encoder
- AM243x-LP board
- BP-AM2BLDCSERVO
\endcond


## Hardware Setup

\cond SOC_AM243X
### Hardware Setup(Using Booster Pack & AM243x-LP)
\imageStyle{AM243x_lp_bp_bissc_encoder_setup.png,width:40%}
\image html AM243x_lp_bp_bissc_encoder_setup.png  "Hardware Setup of Booster Pack + LP for BISS-C"

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

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see \htmllink{@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html, Using SDK with CCS Projects}).
- **When using makefiles to build**, note the required combination and build using
  make command (see \htmllink{@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html, Using SDK with Makefiles})
- Launch a CCS debug session and run the executable, see \htmllink{@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html, CCS Launch\, Load and Run}
- Refer to UART terminal for user interface menu options.


### Sample Output

Shown below is a sample output when the application is run:

\imageStyle{bissc_sample_output.png,width:60%}
\image html bissc_sample_output.png "BISS-C Sample Output"

### Test Case Description

<table>
    <tr>
        <th>#
        <th>Name
        <th>Description
        <th>Pass/fail Criteria
    </tr>
    <tr>
        <td>1</td>
        <td>Data readout (absolute position data)</td>
        <td>In this command we will receive:
		Absolute rotor position value,
		errors, and warnings
		</td>
        <td>CRC success with ABS, E, W and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>2</td>
        <td>Control Communication</td>
        <td>In this command we will receive:
		Absolute rotor position value,
		errors, and warnings, along with the result of the control
        communication command.
		</td>
        <td>CRC success with ABS, E, W and CRC values of position data along with the control communication result printed in the terminal.</td>
    </tr>
</table>

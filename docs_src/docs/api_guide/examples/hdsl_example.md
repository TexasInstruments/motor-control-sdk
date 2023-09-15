#  HDSL Diagnostic {#EXAMPLE_MOTORCONTROL_HDSL}
[TOC]

## Introduction
The HDSL diagnostic application described here interacts with the firmware interface.

HDSL diagnostic application does below,
- Configures pinmux, GPIO, ICSS clock to 300MHz,
- Initializes ICSS0-PRU1, ICSS0-IEP0 and IEP1(for SYNC mode support.Timesync router is used to latch the loopback.),
- Loads lookup table for encoding/decoding of Hiperface data
- Loads the initialization section of PRU firmware & executes it.

Firmware is split to three sections, initialization, datalink and transport.
At startup, the application displays details about encoder and status.
It then presents the user with menu options, based on the option selected, application communicates with HDSL interface and the result is presented to the user.

This example also allows the capability to save the HDSL register data into memory for the defined duration.

\cond SOC_AM243X
- For @VAR_BOARD_NAME_LOWER example, the data is stored in DDR.
- For @VAR_LP_BOARD_NAME_LOWER example, the data is stored in MSRAM.
\endcond

\note The HDSL register trace option is only available with debug mode builds for single channel examples.

## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/position_sense/hdsl_diagnostic</td></tr>
<tr>
    <td>hdsl_diagnostic.c
    hdsl_diagnostic.h</td>
	<td> Source and Header files </td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/position_sense/hdsl</td></tr>
<tr>
    <td>driver/</td>
    <td>Folder containing HDSL PRU driver sources.</td>
</tr>
<tr>
    <td>include/</td>
    <td>Folder containing HDSL PRU header sources.</td>
</tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing HDSL PRU firmware sources.</td>
</tr>

</table>

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/position_sense/hdsl_diagnostic

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER (2 channel and 1 channel examples), @VAR_LP_BOARD_NAME_LOWER (1 channel example)
 Example folder | examples/position_sense/hdsl_diagnostic

\endcond

# Steps to Run the Example

## Hardware Prerequisites

Other than the basic EVM setup mentioned in \htmllink{@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html, EVM Setup}, below additional HW is required to run this demo
- HDSL encoder
- Below are two options to connect encoder to AM64x/AM243x EVM.
    - **Option 1**
        - TIDA-00179 Universal Digital Interface to Absolute Position Encoders, http://www.ti.com/tool/TIDA-00179
        - TIDEP-01015 3 Axis board
        - Interface card connecting EVM and TIDEP-01015 3 Axis board
        - Connect the Hiperface DSL encoder to HDSL+/-(Pin number 6 and 7) signals available on header J7 or Sub-D15 connector of the "Universal Digital Interface to Absolute Position Encoders" board.
    - **Option 2**
        - HDSL AM64xE1 Transceiver. If application is using this card, define the macro HDSL_AM64xE1_TRANSCEIVER in the CCS project/make file.
        - Connect the Hiperface DSL encoder to J10.
		- HDSL AM64xE1 Transceiver supports two channels that can be used to support HDSL safety, multi axis servo drives.
		- Schematics are shared in the MCU+SDK package. For more design details of the transceiver card, please contact TI via E2E/FAE.
		- \htmllink{../am64x_am243x/HDSL_AM64xE1_Schematics.pdf, HDSL Transceiver Card Schematics} document.

\cond SOC_AM243X
### Hardware Prerequisities for Booster Pack

- HDSL encoder
- AM243x-LP board
- BP-AM2BLDCSERVO, https://www.ti.com/tool/BP-AM2BLDCSERVO
\endcond


## Hardware Setup(Using TIDA-00179, TIDEP-01015 and Interface board)

\imageStyle{HDSL_Connections.png,width:40%}
\image html HDSL_Connections.png "Hardware Setup"

## Hardware Setup(Using HDSL AM64xE1 Transceiver)

\imageStyle{HDSL_AM64xE1.png,width:60%}
\image html HDSL_AM64xE1.png "Hardware Setup"

\cond SOC_AM243X
## Hardware Setup(Using Booster Pack & AM243x-LP)
\imageStyle{HDSL_Booster_Pack.png,width:40%}
\image html HDSL_Booster_Pack.png  "Hardware Setup of Booster Pack + LP for HDSL"

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
    <td>ON</td>
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

# Mode, Channel(s) and Board Selection from sysconfig:

- Select Mode from sysconfig menu (Freerun/sync mode).
- Select Channel 0/channel 1 from sysconfig menu for channel selection.
- Select Boosterpack option from sysconfig for running application on AM243x-LP.
\imageStyle{hdsl_sysconfig_menu.png,width:60%}
\image html hdsl_sysconfig_menu.png "HDSL SYSCONFIG Menu"

# Sample Output

Shown below is a sample output when the application is run

- Freerun mode
\image html hdsl_freerun_menu.png "HDSL Freerun mode Menu"
\image html hdsl_positional_commands_menu.png "HDSL Freerun mode Menu"

- Sync Mode
This is a test feature. In real application, PWM syncout will be connected to Latch input instead of IEP1 sync.
Enter 6000 as period in UART menu after loading application. Refer \ref HDSL_DESIGN_SYNC for more details about sync mode.

\image html hdsl_sync_mode_menu1.png "HDSL Sync mode Menu"
\image html hdsl_sync_mode_menu2.png "HDSL Sync mode Menu"
\image html hdsl_positional_commands_menu.png "HDSL Sync mode Menu"
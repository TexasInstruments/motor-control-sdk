#  HDSL Diagnostic {#EXAMPLE_MOTORCONTROL_HDSL}

[TOC]

\cond SOC_AM243X

## Introduction

The HDSL diagnostic application described here interacts with the firmware interface.

The HDSL diagnostic application does the following:
- Configures pinmux, GPIO, ICSS clock to 300MHz,
- Initializes ICSSG0-PRU1, ICSSG0-IEP0 and IEP1 (for SYNC mode support. Timesync router is used to latch the loopback.),
- Loads lookup table for encoding/decoding of Hiperface data,
- Loads the initialization section of PRU firmware & executes it.

The firmware is split into three sections: initialization, datalink and transport.
At startup, the application displays details about the encoder and status.
It then presents the user with menu options. Based on the option selected, the application communicates with the HDSL interface and the result is presented to the user.

This example also allows the capability to save the HDSL register data into memory for the defined duration.

- For @VAR_BOARD_NAME_LOWER example, the data is stored in DDR.
- For @VAR_LP_BOARD_NAME_LOWER example, the data is stored in MSRAM.

\note Channel 2 can be enabled only if channel 0 is enabled because of the code overlay scheme needed in TX-PRU. See \ref HDSL_DESIGN_TXPRU_OVERLAY for more details.

\note The HDSL register trace option is only available with debug mode builds for single channel examples.
\endcond

\cond SOC_AM261X

## Introduction

The HDSL diagnostic application described here interacts with the firmware interface.

The HDSL diagnostic application does the following:
- Configures pinmux, GPIO, ICSS clock to 225MHz,
- Initializes ICSSM1-PRU0, ICSSM0-IEP0 and ICSSM1-IEP0 (for SYNC mode support. Timesync router is used to latch the loopback.),
- Loads lookup table for encoding/decoding of Hiperface data,
- Loads the initialization section of PRU firmware & executes it.

The firmware is split into three sections: initialization, datalink and transport.
At startup, the application displays details about the encoder and status.
It then presents the user with menu options. Based on the option selected, the application communicates with the HDSL interface and the result is presented to the user.

This example also allows the capability to save the HDSL register data into memory for the defined duration.

- For @VAR_LP_BOARD_NAME_LOWER example, the data is stored in MSRAM.

\endcond

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
 ICSS Instance  | ICSSG0
 PRU            | PRU1 (single channel)
 ^              | PRU1, RTU-PRU1, TXPRU1 (multi channel using three PRUs - load share mode)
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/position_sense/hdsl_diagnostic

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSS Instance  |ICSSG0
 PRU            | PRU1 (single channel)
 ^              | PRU1, RTU-PRU1, TXPRU1 (multi channel using three PRUs - load share mode)
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER (2 channel and 1 channel examples), @VAR_LP_BOARD_NAME_LOWER (2 channel and 1 channel examples)
 Example folder | examples/position_sense/hdsl_diagnostic

\endcond

\cond SOC_AM261X

Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSS Instance  | ICSSM1
 PRU            | PRU0 (single channel)
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER (1 channel example)
 Example folder | examples/position_sense/hdsl_diagnostic

\endcond


\cond SOC_AM243X

# Steps to Run the Example
## Hardware Prerequisites for TMDS243EVM
Other than the basic EVM setup mentioned in <a href="@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html" target="_blank"> EVM Setup </a>, the following additional hardware is required to run this demo:
- HDSL Encoder(s)
- <a href="https://www.ti.com/tool/TMDS243EVM" target="_blank"> TMDS243EVM Board </a>
- Below are two options to connect encoders to AM64x/AM243x EVM:
    - **Option 1**
        - <a href="http://www.ti.com/tool/TIDA-00179" target="_blank"> TIDA-00179 Universal Digital Interface to Absolute Position Encoders </a>
        - <a href="../TIDEP-01015RevE1.1(001)_Sch.pdf" target="_blank"> TIDEP-01015 3 Axis Board </a>
        - <a href="../MS_TI_EVM_3-AXIS_INTERFACE_BOARD_SCH_REV_E1.pdf" target="_blank"> Interface card connecting EVM and TIDEP-01015 3 Axis </a>
        - Connect the Hiperface DSL encoder to HDSL+/- (Pin number 6 and 7) signals available on header J7 or Sub-D15 connector of the "Universal Digital Interface to Absolute Position Encoders" board.
    - **Option 2**
        - <a href="../HDSL_Transceiver_E1_Schematics.pdf" target="_blank"> HDSL AM64xE1 Transceiver</a> (If the application is using this card, define the macro HDSL_AM64xE1_TRANSCEIVER in the CCS project/make file)
        - Connect the Hiperface DSL encoder to J10.
		- The HDSL AM64xE1 Transceiver supports two channels that can be used to support HDSL safety and multi-axis servo drives.

\note For more design details of the TIDEP-01015 3 Axis Board, Interface card connecting EVM and TIDEP-01015 3 Axis, or HDSL AM64xE1 Transceiver card, please contact TI via E2E/FAE.


### Hardware Prerequisites for Booster Pack & LP-AM243

- HDSL Encoder(s)
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> LP-AM243 Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
## Hardware Setup (Using TIDA-00179, TIDEP-01015 and Interface board)

\imageStyle{HDSL_Connections.png,width:40%}
\image html HDSL_Connections.png "Hardware Setup"

## Hardware Setup (Using HDSL AM64xE1 Transceiver)

\imageStyle{HDSL_AM64xE1.png,width:60%}
\image html HDSL_AM64xE1.png "Hardware Setup"
## Hardware Setup (Using Booster Pack & LP-AM243)
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
    <td>J18 installed: sets VSENSOR1 to 12 V</td>
    <td>Axis 1: Encoder/Resolver Voltage Select</td>
</tr>
<tr>
    <td>J20/J21</td>
    <td>J20 installed: sets VSENSOR2 to 12V</td>
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

\cond SOC_AM261X
# Steps to Run the Example

## Hardware Setup (Using Booster Pack & LP-AM261)

\imageStyle{HDSL_AM261xLP_SETUP.jpg,width:40%}
\image html HDSL_AM261xLP_SETUP.jpg  "Hardware Setup of Booster Pack + LP for HDSL"

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
    <td>J18 installed: sets VSENSOR1 to 12 V</td>
    <td>Axis 1: Encoder/Resolver Voltage Select</td>
</tr>
<tr>
    <td>J20/J21</td>
    <td>J20 installed: sets VSENSOR2 to 12V</td>
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
\cond SOC_AM261X
# Steps to Run the Example
## Hardware Prerequisites for Booster Pack & LP-AM261

- HDSL Encoder(s)
- <a href="https://www.ti.com/tool/LP-AM261" target="_blank"> LP-AM261 Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>


\endcond

## Build, load and run

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using
  the make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>
- Refer to the UART terminal for user interface menu options.

# Mode, Channel(s) and Board Selection from sysconfig:
\cond SOC_AM243X

- Select Mode from the sysconfig menu (Freerun/sync mode).
- Select Channel 0/channel 1 from the sysconfig menu for channel selection.
- Select Boosterpack option from sysconfig for running the application on LP-AM243.
\imageStyle{hdsl_sysconfig_menu.png,width:60%}
\image html hdsl_sysconfig_menu.png "HDSL SYSCONFIG Menu"

\endcond

\cond SOC_AM261X

- Select Mode from the sysconfig menu (Freerun/sync mode).
- Select Channel 0 from the sysconfig menu for channel selection.
- Select Boosterpack option from sysconfig for running the application on LP-AM261.
\imageStyle{LP-AM261_HDSL_SYSCONFIG.png,width:60%}
\image html LP-AM261_HDSL_SYSCONFIG.png "HDSL SYSCONFIG Menu"

\endcond

# Sample Output

Shown below is a sample output when the application is run:

- Freerun mode
\image html hdsl_freerun_menu.png "HDSL Freerun mode Menu"
\image html hdsl_positional_commands_menu.png "HDSL Freerun mode Menu"

- Sync Mode
This is a test feature. In a real application, PWM syncout will be connected to Latch input instead of IEP1 sync. Select ES value from 1 to 10.
Enter the period (which can be calculated with the formula = Cycle Time (in us) * PRU Core frequency (MHz)) in the UART menu after loading the application. Refer to \ref HDSL_DESIGN_SYNC for more details about sync mode.

\image html hdsl_sync_mode_menu1.png "HDSL Sync mode Menu"
\image html hdsl_sync_mode_menu2.png "HDSL Sync mode Menu"
\image html hdsl_positional_commands_menu.png "HDSL Sync mode Menu"

## HDSL Debug Guide {#HDSL_DEBUG_GUIDE}
This section describes how to debug the HDSL application, including a guide to debugging the HDSL example and firmware. It mainly focuses on verifying the configuration of all components and encoder registers as described in \ref ENCODER_EXAMPLES_DEBUG_GUIDE.

If the HDSL interface is not initializing correctly, the steps mentioned below can help identify the root cause. Additionally, ensure that the hardware connections and software configurations are properly set up before proceeding with debugging.

### Initialization Failures

In case of initialization failure, perform the following steps to identify the root cause:

1. Probe the four pins of the connected channel
2. Load the example and capture signals during the initialization sequence
3. Compare with the expected initialization communication shown below:
\image html HDSL_debug_init_responses.png "HDSL Initialization Responses"

Verify that the TX and RX signals show meaningful data exchanges.
If the initialization communication is incorrect, possible causes include:

1. Firmware not loaded into the correct PRU core
2. Firmware stuck due to:
   - No response or incorrect response from encoder
   - Incorrect channel selection or incorrect clock configuration

Troubleshooting steps:
1. Verify hardware connections
2. Confirm the pin settings and the clock configuration
3. If the above are correct, debug the PRU firmware by connecting to the appropriate core as described in the \ref ENCODER_EXAMPLES_DEBUG_GUIDE

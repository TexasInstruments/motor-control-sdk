# Tamagawa Diagnostic {#EXAMPLE_MOTORCONTROL_TAMAGAWA}
[TOC]
\note
\if (SOC_AM243X || SOC_AM64X)
Starting with MCU+ SDK version 08.05.00, the Tamagawa firmware and examples are based on EnDAT hardware interface from PRU-ICSSG.
\endif

\cond SOC_AM261X
\note ICSSM1 PRU Core clock is set to 225 MHz.
\endcond

## Introduction

The Tamagawa diagnostic application does the following:
\cond (SOC_AM243X || SOC_AM64X)
- Configures pinmux, GPIO, UART, ICSS clock to 200MHz
- Initializes ICSSG0-PRU1
\endcond
\cond (SOC_AM263X || SOC_AM263PX)
- Configures pinmux, GPIO, UART, ICSSM
- Initializes ICSSM-PRU0
\endcond
\cond (SOC_AM261X)
- Configures pinmux, GPIO, UART, ICSSM
- Initializes ICSSM1-PRU0
\endcond
- Loads the initialization section of PRU firmware and executes it

This application is controlled with a terminal interface using a serial over USB connection between the PC host and the EVM.
Please connect a USB cable between the PC and the EVM/LP.
A serial terminal application (like teraterm/ hyperterminal/ minicom) is then run on the host.
To configure, select the serial port corresponding to the port emulated over USB by the EVM.
The host serial port should be configured to 115200 baud, no parity, 1 stop bit and no flow control.

\if (SOC_AM243X || SOC_AM64X)
The Tamagawa receiver firmware running on ICSSG0-PRU1 provides a defined interface.
\endif

\if (SOC_AM263X || SOC_AM263PX)
The Tamagawa receiver firmware running on ICSSM-PRU0 provides a defined interface.
\endif

\if (SOC_AM261X)
The Tamagawa receiver firmware running on ICSSM1-PRU0 provides a defined interface.
\endif

The Tamagawa diagnostic application interacts with the Tamagawa receiver firmware interface. It then presents the user with menu options to select Data ID code (as defined by Tamagawa) to be sent to the encoder. The application collects the data entered by the user and configures the relevant interface. Then via the Tamagawa receiver interface, the command is triggered. Once the command completion is indicated by the interface, the status of the transaction is checked. If the Status indicates success, the result is presented to the user.

## Periodic Trigger Modes {#TAMAGAWA_EXAMPLE_PERIODIC_MODE}

The Tamagawa diagnostic application supports two types of periodic trigger modes for continuous position sampling as described in \ref TAMAGAWA_PERIODIC_MODES.

### CMP Mode (Compare Event Mode)
- Implementation: Command 9 demonstrates this mode using position command 1
- Configuration: Uses a user-defined compare value to trigger sampling events
- IEP Counter Reset: Uses CMP0 by default (skip if reset is handled differently)
- Notification: Firmware triggers an Arm® Cortex®-R5F interrupt after receiving encoder response

### CAP Mode (Capture Event Mode)
- Implementation: Command 10 demonstrates this mode using position command 1
\cond SOC_AM243X
- Router Configuration for CAP6/CAP7 (LATCH_IN0/LATCH_IN1) via TIMESYNC router and CAP0 via GPIOMUX router (requires external GPIO connection)
    - This example configures the TIMESYNC/GPIOMUX router to use IEP SYNC OUT0 as an input signal for the CAP6/CAP7/CAP0 events. This configuration includes:
        - CMP1: Generates SYNC OUT0 signal (skip if not using SYNC OUT0)
    - NOTE: All router configuration is optional if this signal path isn't needed

\endcond
\cond (SOC_AM263PX || SOC_AM261X)
- XBAR Configuration for CAP6/CAP7 (LATCH_IN0/LATCH_IN1)
    - This example configures the XBAR for routing EPWM SYNC OUT as input to CAP using SysConfig
    - Customization: XBAR settings can be modified for alternative inputs
    - NOTE: XBAR routing configuration is optional if not needed
\endcond
- NOTE: When using different CAP events instead of the ones used in SDK example, ensure all related configurations (source selection, signal routing, etc.) are properly done.
- Notification: Firmware triggers an R5F interrupt after receiving encoder response

### Important Notes for Periodic Mode

1. Initialization: Call \ref tamagawa_command_process() once in host trigger mode before switching to periodic mode
2. Automatic Behavior: In periodic mode, \ref tamagawa_command_process() skips sending operations (PRU firmware handles triggering via IEP events)
3. CMP Resource Allocation
    - Avoid using CMP0 if it's already used to IEP counter reset
    - Avoid using CMP1/CMP2 if they're used to SYNC OUT generation
    - Avoid sharing CMP events across different channels or instances of Tamagawa or other encoders. Each CMP event must be assigned exclusively to a single encoder channel. 
4. Modifying Commands in Periodic Mode
    - Default: cmd 1 is used by default
    - To use a different command:
        - First modify the `tamagawa_process_periodic_command()` function in example code
        - Before switching to periodic mode, send this command once using appropriate API in host trigger mode
        - Ensure all prerequisite APIs are called before the command API to properly set up command data. Refer the `tamagawa_handle_command()` function in the example code to identify all required API calls for specific command

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
    <td>Folder containing Tamagawa PRU firmware sources</td>
</tr>
<tr>
    <td>driver/</td>
    <td>Tamagawa diagnostic driver</td>
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
 PRU            | PRU1 (single channel, multi channel using single PRU)
 ^              | PRU1, RTU-PRU1, TXPRU1 (multi channel using three PRUs - load share mode)
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER (single channel, multi channel using single PRU), @VAR_LP_BOARD_NAME_LOWER (single channel, multi channel using single PRU, multi channel using three PRUs - load share mode)
 Example folder | examples/position_sense/tamagawa_diagnostic/single_channel
 ^              | examples/position_sense/tamagawa_diagnostic/multi_channel_single_pru 
 ^              | examples/position_sense/tamagawa_diagnostic/multi_channel_load_share

 ## Single Channel with Single PRU Example
This example supports one Tamagawa channel using one PRU. In this example:
- 1 Tamagawa driver instance and corresponding SysConfig Tamagawa module instance is used.

## Multi Channel with Single PRU Example
This example supports up to three Tamagawa channels using one PRU. In this example:
- Encoders of the same frequency must be connected to all configured channels.
- Data reception must happen simultaneously on all channels.
- The encoder configuration and cable length should be the same on all channels.
- If encoders across channels don't respond at the same time, this example will not work. Load share configuration should be used instead.
- 1 Tamagawa driver instance and corresponding SysConfig Tamagawa module instance is used for all channels.

## Multi Channel with Multiple PRUs (Load Share) Example
This example supports up to three Tamagawa channels using three PRUs from same PRU-ICSSG slice. In this example:
- Load share mode is used. Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details.
- Encoders of different make and different numbers of encoders connected across channels can be connected.
- Encoders of the same frequency must be connected to all configured channels.
- In this mode, data reception can start independently on all channels.
- After clock transmission, all channels wait for a response and process the response independently. However, all channels must finish processing before the next command can be triggered.
- 1 Tamagawa driver instance and corresponding SysConfig Tamagawa module instance is used for all channels.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSS           | ICSSM
 PRU            | PRU0
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER (Single channel example)
 Example folder | examples/position_sense/tamagawa_diagnostic/single_channel

## Single Channel with Single PRU Example
This example supports one Tamagawa channel using one PRU. In this example:
- 1 Tamagawa driver instance and corresponding SysConfig Tamagawa module instance is used.

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSM          | ICSSM1
 PRU            | PRU0 (single channel)
 ^              | PRU0, PRU1 (dual channel)
 Toolchain      | ti-arm-clang
 Board          |  @VAR_LP_BOARD_NAME_LOWER (Single channel and dual channel examples)
 Example folder | examples/position_sense/tamagawa_diagnostic/single_channel
 ^              | examples/position_sense/tamagawa_diagnostic/dual_channel

## Single Channel with Single PRU Example
This example supports one Tamagawa channel using one PRU. In this example:
- 1 Tamagawa driver instance and corresponding SysConfig Tamagawa module instance is used.

## Dual Channel with Two PRUs Example
This example supports two Tamagawa channels using two PRUs from same PRU-ICSSM. In this example:
- Two independent Tamagawa driver instances run simultaneously. Each driver instance has a corresponding SysConfig Tamagawa module instance.
- Each instance operates independently on a different PRU slice (PRU0 or PRU1).
- Both instances share common PRU-ICSS level resources.
- Different PRUs can handle encoders with different frequencies simultaneously. For example, you can configure 4 MHz encoder on PRU0 channel, while configuring 8 MHz on PRU1 channel.

\endcond

# Steps to Run the Example

Other than the basic EVM setup mentioned in <a href="@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html" target="_blank"> EVM Setup </a>, additional hardware required to run this demo is mentioned below

## Hardware Prerequisites

\cond SOC_AM243X

## Hardware Prerequisites with TMDS243EVM
- Tamagawa Encoder(s)
- <a href="https://www.ti.com/tool/TMDS243EVM" target="_blank"> TMDS243EVM Board </a>
- <a href="http://www.ti.com/tool/TIDA-00179" target="_blank"> TIDA-00179 Universal Digital Interface to Absolute Position Encoders </a>
- <a href="../TIDEP-01015RevE1.1(001)_Sch.pdf" target="_blank"> TIDEP-01015 3 Axis Board </a>
- <a href="../MS_TI_EVM_3-AXIS_INTERFACE_BOARD_SCH_REV_E1.pdf" target="_blank"> Interface card connecting EVM and TIDEP-01015 3 Axis </a>

\note For more design details of the TIDEP-01015 3 Axis Board, or Interface card connecting EVM and TIDEP-01015 3 Axis, please contact TI via E2E/FAE.

## Hardware Prerequisites with LP-AM243
- Tamagawa Encoder(s)
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> LP-AM243 Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

\cond SOC_AM263X

## Hardware Prerequisites with LP-AM263
- Tamagawa Encoder(s)
- <a href="https://www.ti.com/tool/LP-AM263" target="_blank"> LP-AM263 Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
\endcond

\cond SOC_AM263PX
## Hardware Prerequisites with LP-AM263P
- Tamagawa Encoder(s)
- <a href="https://www.ti.com/tool/LP-AM263P" target="_blank"> LP-AM263P Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
\endcond

\cond SOC_AM261X
## Hardware Prerequisites with LP-AM261
- Tamagawa Encoder(s)
- <a href="https://www.ti.com/tool/LP-AM261" target="_blank"> LP-AM261 Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
\endcond

\cond SOC_AM243X

## Hardware Setup with TMDS243EVM
\imageStyle{Tamagawa_setup.jpg,width:60%}
\image html Tamagawa_setup.jpg "Hardware Setup for 3 channels on EVM"

\imageStyle{Tamagawa_connections.JPG,width:60%}
\image html Tamagawa_connections.JPG "Tamagawa Encoder Hardware Setup for 3 channels"

## Hardware Setup(Using BP-AM2BLDCSERVO Booster Pack & LP-AM243)
\imageStyle{Tamagawa_Booster_Pack.png,width:40%}
\image html Tamagawa_Booster_Pack.png  "Hardware Setup with LP-AM243"
\note
    - The PROC109A version of LP-AM243 with BP-AM2BLDCSERVO Booster Pack supports two channels
    - To enable the second channel on LP, SW6 needs to be turned OFF
    - To enable VSENSOR1/VSENSOR2, BoosterPack pins J8.73/J8.74 must be set high (In this example, this pin is configured in GPIO mode and pulled high)

#### BP-AM2BLDCSERVO Booster Pack Jumper Configuration
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

\cond (SOC_AM263X)
## Hardware Setup(Using BP-AM2BLDCSERVO Booster Pack & LP-AM263)
\imageStyle{Tamagawa_am263x_hw_Setup.jpeg,width:60%}
\image html Tamagawa_am263x_hw_Setup.jpeg "Hardware Setup for single channel on LP-AM263 + BP"
\endcond

\cond (SOC_AM263PX)
## Hardware Setup(Using BP-AM2BLDCSERVO Booster Pack & LP-AM263P)
\imageStyle{Tamagawa_am263px_hw_Setup.jpeg,width:60%}
\image html Tamagawa_am263px_hw_Setup.jpeg "Hardware Setup for single channel on LP-AM263P + BP"
\endcond

#### LaunchPad Jumper Configuration
\note
    - Connect the jumpers J13 and J26 for providing 3.3V and 5V to boosterpack.
    - To enable VSENSOR1, BoosterPack pin J8.73 must be set high (In this example, this pin is configured in GPIO mode and pulled high)


#### BP-AM2BLDCSERVO Booster Pack Jumper Configuration
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
    <td>OFF</td>
    <td>Encoder and %SDFM Paths Select</td>
</tr>
<tr>
    <td>J28</td>
    <td>ON</td>
    <td>AM243/AM263 Mode</td>
</tr>
</table>

\endcond

\cond SOC_AM261X

## Hardware Setup(Using BP-AM2BLDCSERVO Booster Pack & LP-AM261)

\imageStyle{Tamagawa_am261x_hw_Setup.jpeg,width:60%}
\image html Tamagawa_am261x_hw_Setup.jpeg "Hardware Setup with LP-AM261"

#### LaunchPad Jumper Configuration
\note
    - Connect the jumpers J13 and J26 for providing 3.3V and 5V to boosterpack.
    - The Rev. A version of LP-AM261 with BP-AM2BLDCSERVO Booster Pack supports two channels
    - To enable VSENSOR1/VSENSOR2, BoosterPack pins J8.73/J8.74 must be set high (In this example, this pin is configured in GPIO mode and pulled high)

#### BP-AM2BLDCSERVO Booster Pack Jumper Configuration
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

\imageStyle{Tamagawa_SampleOutput.png,width:60%}
\image html Tamagawa_SampleOutput.png "Tamagawa Sample Output"

## Tamagawa Debug Guide {#TAMAGAWA_DEBUG_GUIDE}

This section describes how to debug the Tamagawa application, including a guide to debugging the Tamagawa example and firmware. Several common debugging steps on verifying the configuration of key registers, hardware details for probing pins, debugging firmware, common issues with multi-channel or continuous mode, etc. are described in \ref ENCODER_EXAMPLES_DEBUG_GUIDE.

If the Tamagawa interface is not initializing correctly, the steps mentioned below can help identify the root cause. Additionally, ensure that the hardware connections and software configurations are properly set up before proceeding with debugging.

### Initial Command Failures

In the case of getting a CRC failure, firmware getting stuck, or getting `0xffff` data with CRC success when the command is run for the first time through the UART terminal, possible root causes can be:
- Incorrect clock configuration
- Incorrect hardware configuration

The following things can be tried for finding the root cause:

1. Probe the four pins of the connected channel.
2. Load the example and capture signals for command `0`.
3. Compare with the expected communication shown below:

\image html Tamagawa_debug_init_responses.png "Tamagawa cmd 0 Response"

Verify that the clock frequency is set to `2.5 Mbps` or `5 Mbps`. The TX and RX signals should show meaningful data exchange for command `0`.

If nothing is appearing on the signals, possible causes are:

1. Firmware not loaded into the correct PRU core
2. Firmware getting stuck due to:
   - Incorrect channel selection or incorrect configuration

This issue can be troubleshot by debugging the firmware and examining register values, as described in the \ref ENCODER_EXAMPLES_DEBUG_GUIDE.

## Test Case Description

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
        <td>Receive following data:
		<br>Absolute rotor position value in field name ABS.
		<br>Errors and warnings in field name SF(status field)
		</td>
        <td>CRC success with ABS, SF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 1</td>
        <td>Data readout (multi-turn data)</td>
        <td>Receive following data:
		<br>No. of rotor turns in field name ABM.
		<br>Errors and warnings in field name SF(status field).
		</td>
        <td>CRC success with ABM, SF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 2</td>
        <td>Encoder-ID</td>
        <td>Receive following data:
    	<br>Tamagawa encoder make-ID in ENID field.
		<br>Errors and warnings in field name SF(status field)
		</td>
        <td>CRC success with ENID, SF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 3</td>
        <td>Data readout(absolute+multiturn+encoder-ID)</td>
        <td>Receive following data:
		Absolute rotor position value in field name ABS.
		No. of rotor turns in field name ABM.
		Tamagawa encoder make-ID in ENID field.
		Errors and warnings in field name SF(status field)
		Other warnings in field name ALMC
		</td>
        <td>CRC success with ABS, ENID, ABM, ALMC, SF, CF and CRC values printed in the terminal.</td>
    </tr>
    <tr>
        <td>Data ID 6</td>
        <td>Writing to EEPROM</td>
        <td>Transmit following data:
        <br>Proper address of the EEPROM where you want to write
		<br>Proper data that you want to write.<br>
        <br>Receive following data:
        <br>Control Field for EEPROM Write command
        <br>EEPROM address that you want to write to
        <br>Data that you want to write to the EEPROM
        <br>CRC value
		</td>
        <td>CRC success with EDF, ADF, CF and CRC values printed in the terminal.</td>
    </tr>
    <tr>
        <td>Data ID D</td>
        <td>Readout from EEPROM</td>
        <td>Transmit following data:
        <br>Proper address of the EEPROM that you want to read.<br>
        <br>Receive following data:
        <br>Control Field for EEPROM Read command
        <br>EEPROM address that was read from
        <br>Data read from the EEPROM
        <br>CRC value
        </td>
        <td>CRC success with EDF, ADF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 7</td>
        <td>Reset-Error</td>
        <td>This command is used to reset errors. </td>
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
    <tr>
        <td>9 (NOTE: This is not a command with Data ID 9, it is UART option number 9)</td>
        <td>Start Periodic CMP Mode</td>
        <td>In this command, encoder sends absolute position data based on IEP CMP event.
		</td>
        <td>CRC success with ABS, SF, CF and CRC stats printed in the terminal.
        </td>
    </tr>
    <tr>
        <td>10 (NOTE: This is not a command with Data ID 10, it is UART option number 10)</td>
        <td>Start Periodic CAP Mode</td>
        <td>In this command, encoder sends absolute position data based on IEP CAP event.
		</td>
        <td>CRC success with ABS, SF, CF and CRC stats printed in the terminal.
        </td>
    </tr>
</table>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

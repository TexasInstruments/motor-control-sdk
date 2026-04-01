# BISS-C Diagnostic {#EXAMPLE_MOTORCONTROL_BISSC}
[TOC]
\cond SOC_AM243X

BISS-C diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 200MHz (default is 200 MHz, 300 MHz can also be used)
- Initializes PRU-ICSS
- Initializes default parameters, loads the PRU firmware and executes it.

\note BiSS-C firmware is tested with ICSS Core Clock running at 200 MHz/300 MHz frequency or ICSS UART Clock running at 192 MHz only. ICSS Core Clock at 225/250/333 MHz is not supported due to clock divider requirements.

\endcond

\cond SOC_AM261X

BISS-C diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 225MHz
- Initializes PRU-ICSS
- Initializes default parameters, loads the PRU firmware and executes it.

\note BiSS-C firmware is tested with ICSS UART Clock running at 160 MHz only, when ICSS Core Clock is 225 MHz due to clock divider requirements.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

BISS-C diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 200MHz
- Initializes PRU-ICSS
- Initializes default parameters, loads the PRU firmware and executes it.

\note BiSS-C firmware is tested with ICSS Core Clock running at 200 MHz frequency or ICSS UART Clock running at 192 MHz frequency only.

\endcond

This application is controlled with a terminal interface using a serial over USB connection between the PC host and the EVM.
Please connect a USB cable between the PC and the EVM/LP.
A serial terminal application (like teraterm/ hyperterminal/ minicom) is then run on the host.
To configure, select the serial port corresponding to the port emulated over USB by the EVM.
The host serial port should be configured to 115200 baud, no parity, 1 stop bit and no flow control.

The BISS-C receiver firmware running on PRU provides a defined interface. The BISS-C diagnostic application interacts with the BISS-C receiver firmware interface. It then presents the user with menu options to select Data ID code. The application collects the data entered by the user and configures the relevant interface. Then via the BISS-C receiver interface, the command is triggered. Once the command completion is indicated by the interface, the status of the transaction is checked. If the status indicates success, the result is presented to the user.

\cond SOC_AM243X
## Channel Selection In Sysconfig

\image html bissc_syscfg_ch_sel.png      "Channel Selection In Sysconfig"

\image html Endat_channel_selection_configuration.png     "BiSS-C configuration selection between Single/Multi channel"

\endcond

## Periodic Trigger Modes {#BISSC_EXAMPLE_PERIODIC_MODE}

The BiSS-C diagnostic application supports two types of periodic trigger modes for continuous position sampling as described in \ref BISSC_PERIODIC_MODES.

### CMP Mode (Compare Event Mode)
- Implementation: Command 6 demonstrates this mode
- Configuration: Uses a user-defined compare value to trigger sampling events
- IEP Counter Reset: Uses CMP0 by default (skip if reset is handled differently)
- Notification: Firmware triggers an Arm® Cortex®-R5F interrupt after receiving encoder response

### CAP Mode (Capture Event Mode)
- Implementation: Command 7 demonstrates this mode
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

1. Automatic Behavior: In periodic mode, bissc_command_process() skips sending operations (PRU firmware handles triggering via IEP events)
2. CMP Resource Allocation
    - Avoid using CMP0 if it's already used to IEP counter reset
    - Avoid using CMP1/CMP2 if they're used to SYNC OUT generation
    - Avoid sharing CMP events across different channels or instances of BiSS-C or other encoders. Each CMP event must be assigned exclusively to a single encoder channel.

## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/position_sense/bissc_diagnostic</td></tr>
<tr>
    <td>bissc_diagnostic.c</td>
    <td>BISS-C diagnostic application</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/position_sense/bissc</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing BISS-C PRU firmware sources</td>
</tr>
<tr>
    <td>driver/</td>
    <td>BISS-C diagnostic driver</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_MOTORCONTROL_BISSC_COMBOS}

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1 (single channel, multi channel using single PRU)
 ^              | PRU1, RTU-PRU1, TXPRU1 (multi channel using three PRUs - load share mode)
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/position_sense/bissc_diagnostic/single_channel
 ^              | examples/position_sense/bissc_diagnostic/multi_channel_single_pru
 ^              | examples/position_sense/bissc_diagnostic/multi_channel_load_share

## Single Channel with Single PRU Example
This example supports one BiSS-C channel using one PRU. In this example:
- 1 BiSS-C driver instance and corresponding SysConfig BiSS-C module instance is used.

## Multi Channel with Single PRU Example
This example supports up to three BiSS-C channels using one PRU. In this example:
- Encoders of the same frequency must be connected to all configured channels.
- Data reception must happen simultaneously on all channels.
- The encoder configuration and cable length should be the same on all channels.
- If encoders across channels don't respond at the same time, this example will not work. Load share configuration should be used instead.
- 1 BiSS-C driver instance and corresponding SysConfig BiSS-C module instance is used for all channels.

## Multi Channel with Multiple PRUs (Load Share) Example
This example supports up to three BiSS-C channels using three PRUs from same PRU-ICSSG slice. In this example:
- Load share mode is used. Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details.
- Encoders of different make and different numbers of encoders connected across channels can be connected.
- Encoders of the same frequency must be connected to all configured channels.
- Data reception can start independently on all channels.
- After clock transmission, all channels wait for a response and process the response independently. However, all channels must finish processing before the next command can be triggered.
- 1 BiSS-C driver instance and corresponding SysConfig BiSS-C module instance is used for all channels.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSM          | ICSSM0
 PRU            | PRU0
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/position_sense/bissc_diagnostic/single_channel

## Single Channel with Single PRU Example
This example supports one BiSS-C channel using one PRU. In this example:
- 1 BiSS-C driver instance and corresponding SysConfig BiSS-C module instance is used.

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSM1
 PRU            | PRU0 (single channel)
 ^              | PRU0, PRU1 (dual channel)
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/position_sense/bissc_diagnostic/single_channel
 ^              | examples/position_sense/bissc_diagnostic/dual_channel

## Single Channel with Single PRU Example
This example supports one BiSS-C channel using one PRU. In this example:
- 1 BiSS-C driver instance and corresponding SysConfig BiSS-C module instance is used.

## Dual Channel with Two PRUs Example
This example supports two BiSS-C channels using two PRUs from same PRU-ICSSM. In this example:
- Two independent BiSS-C driver instances run simultaneously. Each driver instance has a corresponding SysConfig BiSS-C module instance.
- Each instance operates independently on a different PRU slice (PRU0 or PRU1).
- Both instances share common PRU-ICSS level resources.
- Different PRUs can handle encoders with different frequencies simultaneously. For example, you can configure 4 MHz encoder on PRU0 channel, while configuring 8 MHz on PRU1 channel.
- For dual channel example testing, the application takes UART command input from user, then does the operation one by one for each channel.
- When using two instances example, avoid selecting the same CMP event or CAP event for both instances. Each instance must use a different IEP event number to prevent conflicts.

\endcond

# Steps to Run the Example

## Hardware Prerequisites

\cond SOC_A243X

- BISS-C Encoders
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> LP-AM243</a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

\cond SOC_AM261X

- BISS-C Encoders
- <a href="https://www.ti.com/tool/LP-AM261" target="_blank"> LP-AM261 </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

\cond SOC_AM263X

- BISS-C Encoders
- <a href="https://www.ti.com/tool/LP-AM263" target="_blank"> LP-AM263 </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

\cond SOC_AM263PX

- BISS-C Encoders
- <a href="https://www.ti.com/tool/LP-AM263P" target="_blank"> LP-AM263P </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

## Hardware Setup

\cond SOC_AM243X

### Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack and LP-AM243)
\imageStyle{AM243x_lp_bp_bissc_encoder_setup.png,width:40%}
\image html AM243x_lp_bp_bissc_encoder_setup.png "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for BISS-C"

\note
    - The PROC109A version of LP-AM243 with BP-AM2BLDCSERVO Booster Pack supports two channels
    - To enable the second channel on LP, SW6 needs to be turn OFF
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

### Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack and LP-AM261)
\imageStyle{AM261x_lp_bp_bissc_encoder_setup.png,width:40%}
\image html AM261x_lp_bp_bissc_encoder_setup.png "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for BISS-C"

\note
    - The Rev. A version of LP-AM261 with BP-AM2BLDCSERVO Booster Pack supports two channels
    - To enable VSENSOR1/VSENSOR2, BoosterPack pins J8.73/J8.74 must be set high (In this example, this pin is configured in GPIO mode and pulled high)

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

### Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack and LP-AM263)
\imageStyle{AM263x_lp_bp_bissc_encoder_setup.png,width:40%}
\image html AM263x_lp_bp_bissc_encoder_setup.png "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for BISS-C"

\note
    - To enable VSENSOR1, BoosterPack pin J8.73 must be set high (In this example, this pin is configured in GPIO mode and pulled high)

#### LP-AM263 Jumper Configuration

\endcond

\cond SOC_AM263PX

### Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack and LP-AM263P)
\imageStyle{AM263Px_lp_bp_bissc_encoder_setup.png,width:40%}
\image html AM263Px_lp_bp_bissc_encoder_setup.png "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for BISS-C"

\note
    - To enable VSENSOR1, BoosterPack pin J8.73 must be set high (In this example, this pin is configured in GPIO mode and pulled high)

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
    <td>ON</td>
    <td>3V3 Supply to Booster Pack</td>
</tr>
<tr>
    <td>J14</td>
    <td>ON</td>
    <td>5V0 Supply to Booster Pack</td>
</tr>
</table>

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
    <td>ON</td>
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

\imageStyle{bissc_sample_output.png,width:60%}
\image html bissc_sample_output.png "BISS-C Sample Output"

## BiSS-C Debug Guide {#BISSC_DEBUG_GUIDE}
This section describes how to debug the BiSS-C application, including a guide to debugging the BiSS-C example and firmware. Several common debugging steps on verifying the configuration of key registers, hardware details for probing pins, debugging firmware, common issues with multi-channel or continuous mode, etc. are described in \ref ENCODER_EXAMPLES_DEBUG_GUIDE.

If the BiSS-C interface is not initializing correctly, the steps mentioned below can help identify the root cause. Additionally, ensure that the hardware connections and software configurations are properly set up before proceeding with debugging.

### Initialization Failures

In case of initialization failure, perform the following steps to identify the root cause:

1. Probe the RX and clock pins of the connected channel
2. Load the example and capture signals during the initialization sequence. Check \ref BISSC_DESIGN for details on initialization.
3. Compare with the expected initialization sequence below:
\imageStyle{bissc_initialization_response.png,width:60%}
\image html bissc_initialization_response.png "BiSS-C Initialization Response"

4. Probe the response and verify it with the image below
\imageStyle{bissc_response.png,width:60%}
\image html bissc_response.png "BiSS-C Response"

\note The initialization sequence for BiSS-C will be repeated 8 times and will have an extended clock signal for process delay measurement.

Verify that the clock and RX signals show meaningful data exchanges.
If the initialization communication is incorrect, possible causes include:

1. Firmware not loaded into the correct PRU core
2. Firmware stuck due to:
   - No response or incorrect response from encoder
   - Incorrect channel selection or incorrect clock configuration

Troubleshooting steps:
1. Verify hardware connections
2. Confirm the pin settings and the clock configuration
3. If the above are correct, debug the PRU firmware by connecting to the appropriate core as described in the \ref ENCODER_EXAMPLES_DEBUG_GUIDE

### Test Case Description

<table>
    <tr>
        <th>UART Option Number
        <th>Name
        <th>Description
        <th>Pass/fail Criteria
    </tr>
    <tr>
        <td>3</td>
        <td>Data readout (absolute position data)</td>
        <td>Absolute rotor position value, errors, and warnings are received.
		</td>
        <td>CRC success with ABS, E, W and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>4</td>
        <td>Control Communication</td>
        <td>Absolute rotor position value, errors, and warnings, along with the result of the control communication command are received.
		</td>
        <td>CRC success with ABS position value, E, W and CRC values of position data along with the control communication result printed in the terminal.</td>
    </tr>
    <tr>
        <td>6</td>
        <td>Start periodic CMP mode</td>
        <td>Absolute rotor position value, errors, and warnings are received periodically using IEP CMP event. Rotate the rotor of motor and see the changes in position value on UART.
		</td>
        <td>0 CRC errors with ABS position value, E, W and CRC values printed in the terminal.</td>
    </tr>
    <tr>
        <td>7</td>
        <td>Start periodic CAP mode</td>
        <td>Absolute rotor position value, errors, and warnings are received periodically using IEP CAP event. Rotate the rotor of motor and see the changes in position value on UART.
		</td>
        <td>0 CRC errors with ABS position value, E, W and CRC values printed in the terminal.</td>
    </tr>
    <tr>
        <td>8</td>
        <td>Enable/disable safety mode</td>
        <td>Enable/disable safety mode which toggles between 6/16 bit CRC and enables/disables Sign of Life counter </td>
        <td>Safety should be enabled and CRC and Sign of Life counter will be displayed from next position data request</td>
    </tr>
</table>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

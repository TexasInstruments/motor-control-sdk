# NIKON Diagnostic {#EXAMPLE_MOTORCONTROL_NIKON}
[TOC]

\note A-format® is a registered trademark of the Nikon Corporation.

\cond SOC_AM243X

Nikon diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 200MHz,
- Initializes ICSS0-PRU1,
- Initializes default parameters, loads the PRU firmware & executes it.

\note Nikon firmware supports operation with ICSS Core Clock running at 200 MHz/300 MHz frequency or ICSS UART Clock running at 192 MHz only. ICSS Core Clock at 225/250/333 MHz is not supported due to clock divider requirements.
\endcond

\cond SOC_AM261X

Nikon diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 225MHz,
- Initializes ICSS0-PRU1,
- Initializes default parameters, loads the PRU firmware & executes it.

\note Nikon firmware supports operation with ICSS UART Clock running at 160 MHz only, when ICSS Core Clock is 225 MHz due to clock divider requirements.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

Nikon diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 200MHz,
- Initializes ICSS-PRU0,
- Initializes default parameters, loads the PRU firmware & executes it.

\note Nikon firmware supports operation with ICSS Core Clock running at 200 MHz frequency or ICSS UART Clock running at 192 MHz frequency only.

\endcond

This application is controlled with a terminal interface using a serial over USB connection between the PC host and the EVM.
Please connect a USB cable between the PC and the EVM/LP.
A serial terminal application (like teraterm/ hyperterminal/ minicom) is then run on the host.
To configure, select the serial port corresponding to the port emulated over USB by the EVM.
The host serial port should be configured to 115200 baud, no parity, 1 stop bit and no flow control.

The Nikon receiver firmware running on ICSS-PRU provides a defined interface. The Nikon diagnostic application interacts with the Nikon receiver firmware interface. It then presents the user with menu options to select different commands. The application collects the data entered by the user and configures the relevant interface. Then via the Nikon receiver interface, the command is triggered. Once the command completion is indicated by the interface, the status of the transaction is checked. If the Status indicates success, the result is presented to the user.

\note
    Limitation with <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a> boosterpack with TI LaunchPad

\note
    - Hardware Limitation with BP-AM2BLDCSERVO
        - Only up to 7 encoders in bus connection have been tested with BP-AM2BLDCSERVO
        - This limitation is due to insufficient voltage when attempting to power 8 encoders
        - Attempting to connect 8 encoders may result in unreliable operation

    - Software Support
        - The example code and PRU firmware are designed to handle 8 encoders

\cond SOC_AM243X

## Channel Selection In Sysconfig

\image html nikon_syscfg_ch_sel.png      "Channel Selection In Sysconfig"

\image html Endat_channel_selection_configuration.png     "Nikon configuration selection between Single/Multi channel "

\endcond

## Periodic Trigger Modes {#NIKON_EXAMPLE_PERIODIC_MODE}

The Nikon diagnostic application supports two types of periodic trigger modes for continuous position sampling as described in \ref NIKON_PERIODIC_MODES.

### CMP Mode (Compare Event Mode)
- Implementation: Command 32 demonstrates this mode using position command CMD_4
- Configuration: Uses a user-defined compare value to trigger sampling events
- IEP Counter Reset: Uses CMP0 by default (skip if reset is handled differently)
- Notification: Firmware triggers an Arm® Cortex®-R5F interrupt after receiving encoder response

### CAP Mode (Capture Event Mode)
- Implementation: Command 33 demonstrates this mode using position command CMD_4
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

1. Initialization: Call nikon_command_process() once in host trigger mode before switching to periodic mode
2. Automatic Behavior: In periodic mode, nikon_command_process() skips sending operations (PRU firmware handles triggering via IEP events)
3. CMP Resource Allocation
    - Avoid using CMP0 if it's already used to IEP counter reset
    - Avoid using CMP1/CMP2 if they're used to SYNC OUT generation
    - Avoid sharing CMP events across different channels or instances of Nikon or other encoders. Each CMP event must be assigned exclusively to a single encoder channel.
4. Modifying Commands in Periodic Mode
    - Default: Position Command 4 (CMD_4) is used by default
    - To use a different command:
        - First modify the `nikon_process_periodic_command()` function in example code
        - Before switching to periodic mode, send this command once using `nikon_get_pos()` in host trigger mode
        - Ensure all prerequisite APIs are called before `nikon_get_pos()` to properly set up command data (e.g., `nikon_generate_cdf()`). Refer the `nikon_handle_command()` function in the example code to identify all required API calls for specific command

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
    <td>Folder containing Nikon PRU firmware sources</td>
</tr>
<tr>
    <td>driver/</td>
    <td>Nikon diagnostic driver</td>
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
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/position_sense/nikon_diagnostic/single_channel
 ^              | examples/position_sense/nikon_diagnostic/multi_channel_single_pru
 ^              | examples/position_sense/nikon_diagnostic/multi_channel_load_share

## Single Channel with Single PRU Example
This example supports one Nikon channel using one PRU. In this example:
- 1 Nikon driver instance and corresponding SysConfig Nikon module instance is used.

## Multi Channel with Single PRU Example
This example supports up to three Nikon channels using one PRU. In this example:
- Encoders of the same frequency must be connected to all configured channels.
- Data transmission and reception must happen simultaneously on all channels.
- The encoder configuration and cable length should be the same on all channels.
- If encoders across channels don't respond at the same time, this example will not work. Load share configuration should be used instead.
- 1 Nikon driver instance and corresponding SysConfig Nikon module instance is used for all channels.

## Multi Channel with Multiple PRUs (Load Share) Example
This example supports up to three Nikon channels using three PRUs from same PRU-ICSSG slice. In this example:
- Load share mode is used. Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details.
- Encoders of different make and different numbers of encoders connected across channels can be connected.
- Encoders of the same frequency must be connected to all configured channels.
- In this mode, data transmission and reception can start independently on all channels.
- After a command is sent, all channels wait for a response and process the response independently. However, all channels must finish processing before the next command can be triggered.
- 1 Nikon driver instance and corresponding SysConfig Nikon module instance is used for all channels.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSM          | ICSSM0
 PRU            | PRU0
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/position_sense/nikon_diagnostic/single_channel

## Single Channel with Single PRU Example
This example supports one Nikon channel using one PRU. In this example:
- 1 Nikon driver instance and corresponding SysConfig Nikon module instance is used.

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSM          | ICSSM1
 PRU            | PRU0 (single channel)
 ^              | PRU0, PRU1 (dual channel)
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/position_sense/nikon_diagnostic/single_channel
 ^              | examples/position_sense/nikon_diagnostic/dual_channel

## Single Channel with Single PRU Example
This example supports one Nikon channel using one PRU. In this example:
- 1 Nikon driver instance and corresponding SysConfig Nikon module instance is used.

## Dual Channel with Two PRUs Example
This example supports two Nikon channels using two PRUs from same PRU-ICSSM. In this example:
- Two independent Nikon driver instances run simultaneously. Each driver instance has a corresponding SysConfig Nikon module instance.
- Each instance operates independently on a different PRU slice (PRU0 or PRU1).
- Both instances share common PRU-ICSS level resources.
- Different PRUs can handle encoders with different frequencies simultaneously. For example, you can connect a 4 MHz encoder to a PRU0 channel, while connecting an 8 MHz encoder to a PRU1 channel.
\endcond

# Steps to Run the Example

## Hardware Prerequisites
\cond SOC_AM243X

- Nikon A-format encoders
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> LP-AM243 </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

\cond SOC_AM261X

- Nikon A-format encoders
- <a href="https://www.ti.com/tool/LP-AM261" target="_blank"> LP-AM261 </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

\cond SOC_AM263X

- Nikon A-format encoders
- <a href="https://www.ti.com/tool/LP-AM263" target="_blank"> LP-AM263 </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

\cond SOC_AM263PX

- Nikon A-format encoders
- <a href="https://www.ti.com/tool/LP-AM263P" target="_blank"> LP-AM263P </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>

\endcond

## Hardware Setup

\cond SOC_AM243X
### Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack & LP-AM243)
\imageStyle{AM243x_lp_bp_nikon_encoder_setup.png,width:40%}
\image html AM243x_lp_bp_nikon_encoder_setup.png  "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for Nikon"

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

### Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack & LP-AM261)

\imageStyle{AM261x_lp_bp_nikon_encoder_setup.png,width:40%}
\image html AM261x_lp_bp_nikon_encoder_setup.png  "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for Nikon"

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

### Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack & LP-AM263)
\imageStyle{AM263x_lp_bp_nikon_encoder_setup.png,width:40%}
\image html AM263x_lp_bp_nikon_encoder_setup.png  "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for Nikon"

#### LP-AM263 Jumper Configuration

\endcond

\cond SOC_AM263PX

### Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack & LP-AM263P)

\imageStyle{AM263Px_lp_bp_nikon_encoder_setup.png,width:40%}
\image html AM263Px_lp_bp_nikon_encoder_setup.png  "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for Nikon"

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
    <td>Pin 1-2 Connected</td>
    <td>3V3 Supply to Booster Pack</td>
</tr>
<tr>
    <td>J14</td>
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

\imageStyle{nikon_sample_output.png,width:60%}
\image html nikon_sample_output.png "Nikon Sample Output"

## Nikon A-format Debug Guide {#NIKON_DEBUG_GUIDE}

This section describes how to debug the Nikon A-format application, including a guide to debugging the Nikon A-format example and firmware. It mainly focuses on verifying the configuration of all components and encoder registers, as described in the \ref ENCODER_EXAMPLES_DEBUG_GUIDE.

If the Nikon A-format interface is not initializing correctly, the steps mentioned below can help identify the root cause. Additionally, ensure that the hardware connections and software configurations are properly set up before proceeding with debugging, as described in the \ref ENCODER_EXAMPLES_DEBUG_GUIDE.

### Initialization Failures

In case of initialization failure, perform the following steps to identify the root cause:

1. Probe the TX, RX, TX_EN and clock pins of the connected channel (as per \ref ENCODER_EXAMPLES_DEBUG_GUIDE)
2. Run the example and capture signals during the initialization sequence. CMD_4 which is a multi-transmission command, is sent during initialization.
3. Compare with the expected initialization sequence below:

\image html nikon_initialization_response.png "Nikon A-format Initialization Response"

4. Verify that the TX and RX signals show meaningful data exchanges.

If the initialization communication is incorrect, possible causes include:
1. Firmware not loaded into the correct PRU core
2. Firmware stuck due to:
   - No response or incorrect response from encoder
   - Incorrect channel selection or incorrect clock configuration

Troubleshooting steps:
1. Verify hardware connections
2. Confirm the pin settings and the clock configuration
3. If the above are correct, debug the PRU firmware by connecting to the appropriate core as described in the \ref ENCODER_EXAMPLES_DEBUG_GUIDE

### Command Failures

- If the encoder ID is not set correctly for encoders, it may cause problems with PRU firmware or R5F driver. Correct encoder ID should be set using UART menu option 31 before sending commands or by calling \ref nikon_update_enc_addr API appropriately.
- In single PRU multi-channel mode, the encoder ID should be same on all channels.

\cond SOC_AM243X
- PRU Firmware gets stuck if encoder does not respond with the number of bytes expected by the driver as per PINDSW-9179 in \ref RELEASE_NOTES_11_00_00_PAGE
- Example cases when firmware gets stuck
    1. Any command is sent with encoder address not matching that of the encoder connected with the device
    2. In bus mode, if encoders with addresses 0, 1, 2 are connected and MT command is sent with encoder address 3 or more
    3. Command 20 is sent with ID not matching the encoder connected with the device
\endcond

\cond (SOC_AM263PX || SOC_AM261X)
- PRU Firmware gets stuck if encoder does not respond with the number of bytes expected by the driver as per PINDSW-9179 in \ref RELEASE_NOTES_10_02_00_PAGE
- Example cases when firmware gets stuck
    1. Any command is sent with encoder address not matching that of the encoder connected with the device
    2. In bus mode, if encoders with addresses 0, 1, 2 are connected and MT command is sent with encoder address 3 or more
    3. Command 20 is sent with ID not matching the encoder connected with the device
\endcond

## Test Case Description

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
        <td>1 (Nikon A-format version 3.0 only, based on factory setting)</td>
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
        <td>Alarm bits (PM Alarm bits shown for Nikon A-format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
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
        <td>5 (Nikon A-format version 3.0 only, based on factory setting)</td>
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
        <td>Alarm bits (PM Alarm bits shown for Nikon A-format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>8</td>
        <td>Status flag clear request</td>
        <td>In this command, encoder sends status information, alarm bits and additional information after clearing the status flags.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>9</td>
        <td>Multiple turn data clear request</td>
        <td>In this command, encoder sends status information, alarm bits and additional information after clearing the multiple turn data bits.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>10</td>
        <td>Status+ Multiple turn data clear request</td>
        <td>In this command, encoder sends status information, alarm bits and additional information after clearing the status and multiple turn data bits.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>11</td>
        <td>Encoder address setting I (one-to-one connection)</td>
        <td>In this command, encoder address setting will be performed and status will be returned with ALM bits.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
    <tr>
        <td>12</td>
        <td>Single turn data zero preset</td>
        <td>In this command, encoder sets single turn data bits to zero and returns status bits along with ALM bits.
		</td>
        <td>Alarm bits (PM Alarm bits shown for Nikon A-format version 3.0 only), encoder status bits printed in the terminal along with CRC success.
        </td>
    </tr>
        <tr>
        <td> 8 to 12 (Nikon A-format version 3.0 only, based on factory setting)</td>
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
        <td>13 (Nikon A-format version 3.0 only) </td>
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
        <td>14 (Nikon A-format version 3.0 only) </td>
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
        <td>16 (Nikon A-format version 3.0 only) </td>
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
        <td>18 (Nikon A-format version 3.0 only) </td>
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
        <td>23 (Nikon A-format version 3.0 only)</td>
        <td>ABS lower 24bit + velocity request</td>
        <td>In this command, encoder sends absolute lower 24 bit data for single encoder with velocity information.
		</td>
        <td>CRC success with rotor angle, number of rotations, velocity and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>24 (Nikon A-format version 3.0 only)</td>
        <td>ABS lower 24bit + velocity request(MT)</td>
        <td>In this command, encoder sends absolute lower 24 bit data for encoders connected in bus with velocity information.
		</td>
        <td>CRC success with rotor angle, number of rotations, velocity and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>25 (Nikon A-format version 3.0 only)</td>
        <td>ABS lower 24bit + velocity + acceleration request</td>
        <td>In this command, encoder sends absolute lower 24 bit data for single encoder with velocity and acceleration information.
		</td>
        <td>CRC success with rotor angle, number of rotations, velocity, acceleration and CRC values printed in the terminal.
        </td>
    </tr>
	<tr>
        <td>26 (Nikon A-format version 3.0 only)</td>
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
        <td>32 (NOTE: This is not a command with ID 32, it is UART option number 32)</td>
        <td>Start Periodic CMP Mode</td>
        <td>In this command, encoder sends absolute lower 40 bit data for encoders connected in point to point / bus based on IEP CMP event.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC stats printed in the terminal.
        </td>
    </tr>
    <tr>
        <td>33 (NOTE: This is not a command with ID 33, it is UART option number 33)</td>
        <td>Start Periodic CAP Mode</td>
        <td>In this command, encoder sends absolute lower 40 bit data for encoders connected in point to point / bus based on IEP CAP event.
		</td>
        <td>CRC success with rotor angle, number of rotations and CRC stats printed in the terminal.
        </td>
    </tr>
</table>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

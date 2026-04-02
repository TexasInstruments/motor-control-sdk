#  EnDat Diagnostic {#EXAMPLE_MOTORCONTROL_ENDAT}

[TOC]

The EnDat diagnostic application described here
demonstrates the EnDat receiver operation.

The EnDat driver provides a well-defined set of APIs to expose the EnDat
receiver interface.

\cond (SOC_AM261X)
\note ICSSM UART clock set to 160 MHz is used to drive the EnDat interface. In three channel interface of PRU-ICSS, receive (Rx) is oversampled at 8x of send (Tx). Therefore, the encoder interface frequency "f" should be such that 160 MHz is divisible by "f" and "8 times f".
\endcond

\cond SOC_AM243X || SOC_AM64X
\note EnDat firmware is tested with ICSS Core Clock running at 200 MHz/300 MHz frequency or ICSS UART Clock running at 192 MHz only.
\endcond

\cond SOC_AM261X
\note EnDat firmware is tested with ICSS Core Clock running at 225 MHz frequency or ICSS UART Clock running at 160 MHz only.
\endcond

\cond (SOC_AM263X || SOC_AM263PX)
\note EnDat firmware is tested with ICSS Core Clock running at 200 MHz frequency or ICSS UART Clock running at 192 MHz only.
\endcond

The diagnostic invokes these APIs to:
- initialize EnDat,
\cond (SOC_AM243X || SOC_AM64X)
- select one configuration among concurrent multi channel with encoders of same make, multi channel with encoders of different make, and single channel configuration based on SysConfig,
- select the channel (channels in the case of concurrent multi channel with encoders of same make or multi channel with encoders of different make),
\endcond
- configure the host trigger mode,
- and run the firmware.
\cond (SOC_AM243X || SOC_AM64X)
- If "Multi-Channel with encoders of different make" configuration is selected:
    - enable load share mode,
    - select primary core for global configuration,
    - configure synchronization bits.
\endcond

Once these steps are executed:
- the driver waits for the EnDat to be initialized.
- It then sets clock frequency to 200KHz (as propagation delay is not yet compensated)
- and obtains the encoder details including serial number, position resolution etc., and displays them on the console/UART.
- Based on whether the encoder is 2.2 or 2.1 type, it sets the clock to either \if (SOC_AM261X) 5MHz \else 8MHz \endif or 1MHz respectively.
- While configuring the clock, propagation delay is taken care of using the automatically estimated propagation delay (user can override it too).
\cond (SOC_AM243X || SOC_AM64X)
- In the case of concurrent Multi-Channel with encoders of same make or Multi-Channel with encoders of different make, if propagation delays between various channels are different, that too is automatically taken care of.
\endcond

Once the initial setup is over:
- the diagnostic provides the user with a self-explanatory menu.
- Two types of menu options are presented. One type (1-14) will send an EnDat command as per EnDat 2.2 specification.
- The other type (100-108) allows the user to configure clock frequency, various timing parameters, simulate motor control loop using 2.1 command as well as 2.2 command with safety (redundant position information), switch to continuous clock mode, and monitor raw data.
\cond (SOC_AM243X || SOC_AM64X)
- Concurrent multi channel with encoder of same make configuration can work simultaneously for up to three encoders with identical part numbers. All variants of 2.2 position commands as well as the 2.1 position command are supported, and an additional option (109) to configure wire delay (useful when propagation delay in each channel is different) is available.
- Single PRU core handles enabled channels in single channel and Multi-Channel with encoders of same make configuration.
\endcond
- Application by default handles wire delay as required. The menu option provides a way to override it.

After the user selects an EnDat command:
- the diagnostic asks for more details to frame the command and performs a basic sanity check on the user-entered values.
- Then the EnDat APIs are invoked to process the command set, set the host trigger bit, and wait until the host trigger bit is cleared. \if (SOC_AM243X || SOC_AM64X) If multi channel with encoders of different make is used, these operations are done for each channel.\endif
- The received EnDat data is processed & validated using the defined APIs. The result is then presented to the user.

### Channel Selection In SysConfig

\cond SOC_AM243X || SOC_AM64X
\image html EnDat_channel_selection_In_sysconfig.PNG "Channel Selection In SysConfig"
\endcond

\cond SOC_AM261X
\image html EnDat_channel_selection_In_sysconfig_for_am261x.PNG "Channel Selection In SysConfig"
\endcond

\cond (SOC_AM263X || SOC_AM263PX )
\image html EnDat_channel_selection_In_sysconfig_for_am263x.PNG "Channel Selection In SysConfig"
\endcond

\cond (SOC_AM243X || SOC_AM64X)
\image html Endat_channel_selection_configuration.png "Mode selection based on number of channels and encoder type"
\endcond

## Periodic Trigger Modes {#ENDAT_EXAMPLE_PERIODIC_MODE}

The EnDat diagnostic application supports two types of periodic trigger modes for continuous position sampling as described in \ref ENDAT_PERIODIC_MODES.

### CMP Mode (Compare Event Mode)
- Implementation: Command 200 demonstrates this mode, user can select any position command
- Configuration: Uses a user-defined compare value to trigger sampling events
- IEP Counter Reset: Uses CMP0 by default (skip if reset is handled differently)
- Notification: Firmware triggers an Arm® Cortex®-R5F interrupt after receiving encoder response

### CAP Mode (Capture Event Mode)
- Implementation: Command 201 demonstrates this mode, user can select any position command
\cond SOC_AM243X || SOC_AM64X
- Router Configuration for CAP6/CAP7 (LATCH_IN0/LATCH_IN1) via TIMESYNC router and CAP0 via GPIOMUX router (requires external GPIO connection)
    - This example configures the TIMESYNC/GPIOMUX router to use IEP SYNC OUT0 as an input signal for the CAP6/CAP7/CAP0 events. This configuration includes:
        - CMP1: Generates SYNC OUT0 signal (skip if not using SYNC OUT0)
    - NOTE: All router configuration is optional if this signal path isn't needed

\endcond
\cond (SOC_AM263PX || SOC_AM261X || SOC_AM263X)
- XBAR Configuration for CAP6/CAP7 (LATCH_IN0/LATCH_IN1)
    - This example configures the XBAR for routing EPWM SYNC OUT as input to CAP using SysConfig
    - Customization: XBAR settings can be modified for alternative inputs
    - NOTE: XBAR routing configuration is optional if not needed
\endcond
- NOTE: When using different CAP events instead of the ones used in SDK example, ensure all related configurations (source selection, signal routing, etc.) are properly done.
- Notification: Firmware triggers an R5F interrupt after receiving encoder response

### Important Notes for Periodic Mode

1. Initialization: Call \ref endat_command_process() once in host trigger mode before switching to periodic mode
2. Automatic Behavior: In periodic mode, \ref endat_command_process() skips sending operations (PRU firmware handles triggering via IEP events)
3. CMP Resource Allocation
    - Avoid using CMP0 if it's already used to IEP counter reset
    - Avoid using CMP1/CMP2 if they're used to SYNC OUT generation
    - Avoid sharing CMP events across different channels or instances of EnDat or other encoders. Each CMP event must be assigned exclusively to a single encoder channel.

### Endat Example Implementation

The following section describes the Example implementation of EnDat on Arm®-based core.
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
    <td>EnDat diagnostic application</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/position_sense/endat</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing EnDat firmware sources</td>
</tr>
<tr>
    <td>driver/</td>
    <td>EnDat diagnostic driver</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_MOTORCONTROL_ENDAT_COMBOS}

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1 (single channel, multi channel using single PRU)
 ^              | PRU1, RTU-PRU1, TXPRU1 (multi channel using three PRUs - load share mode)
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/position_sense/endat_diagnostic/single_channel
 ^              | examples/position_sense/endat_diagnostic/multi_channel_single_pru
 ^              | examples/position_sense/endat_diagnostic/multi_channel_load_share


## Single Channel with Single PRU Example
This example supports one EnDat channel using one PRU. In this example:
- 1 EnDat driver instance and corresponding SysConfig EnDat module instance is used.

##  Multi Channel with Single PRU Example
This example supports up to three EnDat channels using one PRU. In this example:
- Encoders of the same frequency must be connected to all configured channels.
- Data reception must happen simultaneously on all channels.
- The encoder configuration and cable length should be the same on all channels.
- If encoders across channels don't respond at the same time, this example will not work. Load share configuration should be used instead.
- 1 EnDat driver instance and corresponding SysConfig EnDat module instance is used for all channels.

## Multi Channel with Multiple PRUs (Load Share) Example
This example supports up to three EnDat channels using three PRUs from same PRU-ICSSG slice. In this example:
- Load share mode is used. Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details.
- Encoders of different make and different numbers of encoders connected across channels can be connected.
- Encoders of the same frequency must be connected to all configured channels.
- Data reception can start independently on all channels.
- Each channel can have different memory areas, MRS codes, or parameters (command type remains the same).
- After a command is sent, all channels wait for a response and process the response independently. However, all channels must finish processing before the next command can be triggered.
- 1 EnDat driver instance and corresponding SysConfig EnDat module instance is used for all channels.

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
 Example folder | examples/position_sense/endat_diagnostic/single_channel
 ^              | examples/position_sense/endat_diagnostic/dual_channel

## Single Channel with Single PRU Example
This example supports one EnDat channel using one PRU. In this example:
- 1 EnDat driver instance and corresponding SysConfig EnDat module instance is used.

## Dual Channel with Two PRUs Example
This example supports two EnDat channels using two PRUs from same PRU-ICSSM. In this example:
- Two independent EnDat driver instances run simultaneously. Each driver instance has a corresponding SysConfig EnDat module instance.
- Each instance operates independently on a different PRU slice (PRU0 or PRU1).
- Both instances share common PRU-ICSS level resources.
- Different PRUs can handle encoders with different frequencies simultaneously. For example, you can connect a 4 MHz encoder to a PRU0 channel, while connecting an 8 MHz encoder to a PRU1 channel.
- For dual channel example testing, the application takes commands for both instances from user, then sends the commands one by one for each channel. So the user needs to ensure that the entered command for both instances is correct to perform the correct operation. For continuous/periodic modes, always enter the same command and parameters for both instances, as it uses the same command for both.
- When using two instances example, avoid selecting the same CMP event or CAP event for both instances. Each instance must use a different IEP event number to prevent conflicts.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSM          | ICSSM0
 PRU            | PRU0
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/position_sense/endat_diagnostic/single_channel

## Single Channel with Single PRU Example
This example supports one EnDat channel using one PRU. In this example:
- 1 EnDat driver instance and corresponding SysConfig EnDat module instance is used.
\endcond

# Steps to Run the Example

Other than the basic EVM setup mentioned in <a href="@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html" target="_blank"> EVM Setup </a>, the following additional hardware is required to run this demo:


\cond SOC_AM243X

## Hardware Prerequisites with TMDS243EVM
- EnDat Encoder(s)
- <a href="https://www.ti.com/tool/TMDS243EVM" target="_blank"> TMDS243EVM Board </a>
- <a href="http://www.ti.com/tool/TIDA-00179" target="_blank"> TIDA-00179 Universal Digital Interface to Absolute Position Encoders </a>
- <a href="../TIDEP-01015RevE1.1(001)_Sch.pdf" target="_blank"> TIDEP-01015 3 Axis Board </a>
- <a href="../MS_TI_EVM_3-AXIS_INTERFACE_BOARD_SCH_REV_E1.pdf" target="_blank"> Interface card connecting EVM and TIDEP-01015 3 Axis </a>

\note For more design details of the TIDEP-01015 3 Axis Board, or Interface card connecting EVM and TIDEP-01015 3 Axis, please contact TI via E2E/FAE.

## Hardware Prerequisites with LP-AM243

- EnDat Encoder(s)
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> LP-AM243 Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
\endcond

\cond SOC_AM261X

## Hardware Prerequisites with LP-AM261

- EnDat Encoder(s)
- <a href="https://www.ti.com/tool/LP-AM261" target="_blank"> LP-AM261 Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
\endcond

\cond SOC_AM263X

## Hardware Prerequisites with LP-AM263

- EnDat Encoder(s)
- <a href="https://www.ti.com/tool/LP-AM263" target="_blank"> LP-AM263 Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
\endcond

\cond SOC_AM263PX

## Hardware Prerequisites with LP-AM263P

- EnDat Encoder(s)
- <a href="https://www.ti.com/tool/LP-AM263P" target="_blank"> LP-AM263P Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
\endcond

\cond SOC_AM243X

## Hardware Setup (Using TMDS243EVM, TIDA-00179, TIDEP-01015 and Interface board)
\imageStyle{EnDAT_Connections.png,width:40%}
\image html EnDAT_Connections.png "Hardware Setup using TMDS243EVM, TIDA-00179, TIDEP-01015 and Interface board for EnDat"

## Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack and LP-AM243)
\imageStyle{EnDat_Booster_Pack.png,width:40%}
\image html EnDat_Booster_Pack.png "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for EnDat"
\note
    - The PROC109A version of LP-AM243 with BP-AM2BLDCSERVO Booster Pack supports two channels
    - To enable the second channel on LP, SW6 needs to be turned OFF
    - To enable VSENSOR1/VSENSOR2, BoosterPack pins J8.73/J8.74 must be set high (In this example, this pin is configured in GPIO mode and pulled high)

### BP-AM2BLDCSERVO Booster Pack Jumper Configuration
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

\if SOC_AM263X
## Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack and LP-AM263)
\imageStyle{EnDat_am263x_hw_Setup.jpeg,width:60%}
\image html EnDat_am263x_hw_Setup.jpeg "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for EnDat"
\else
## Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack and LP-AM263P)
\imageStyle{EnDat_am263px_hw_Setup.jpeg,width:60%}
\image html EnDat_am263px_hw_Setup.jpeg "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for EnDat"
\endif

\note
    - To enable VSENSOR1, BoosterPack pin J8.73 must be set high (In this example, this pin is configured in GPIO mode and pulled high)

\cond (SOC_AM263X)
### LP-AM263 Jumper Configuration
\endcond
\cond (SOC_AM263PX)
### LP-AM263P Jumper Configuration
\endcond

Connect the jumpers J13 and J26 for providing 3.3V and 5V to boosterpack.

### BP-AM2BLDCSERVO Booster Pack Jumper Configuration
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

## Hardware Setup (Using BP-AM2BLDCSERVO Booster Pack and LP-AM261)
\note
    - The Rev. A version of LP-AM261 with BP-AM2BLDCSERVO Booster Pack supports two channels
    - To enable VSENSOR1/VSENSOR2, BoosterPack pins J8.73/J8.74 must be set high (In this example, this pin is configured in GPIO mode and pulled high)

\imageStyle{EnDat_am261x_hw_Setup.jpeg,width:40%}
\image html EnDat_am261x_hw_Setup.jpeg "Hardware Setup of BP-AM2BLDCSERVO Booster Pack + LP for EnDat"

### LP-AM261 Jumper Configuration

Connect the jumpers J13 and J26 for providing 3.3V and 5V to boosterpack.

### BP-AM2BLDCSERVO Booster Pack Jumper Configuration
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
  the make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>
- Refer to the UART terminal for user interface menu options.

### Sample Output

Shown below is a sample output when the application is run:

\imageStyle{EnDAT_Initialization_UART_PRINT.png,width:60%}
\image html EnDAT_Initialization_UART_PRINT.png "EnDat Usage"

## EnDat Debug Guide {#ENDAT_DEBUG_GUIDE}
This section describes how to debug the EnDat application, including a guide to debugging the EnDat example and firmware. Several common debugging steps on verifying the configuration of key registers, hardware details for probing pins, debugging firmware, common issues with multi channel or continuous mode, etc. are described in \ref ENCODER_EXAMPLES_DEBUG_GUIDE.

If the EnDat interface is not initializing correctly, the steps mentioned below can help identify the root cause. Additionally, ensure that the hardware connections and software configurations are properly set up before proceeding with debugging.

### Initialization Failures

In case of initialization failure, perform the following steps to identify the root cause:

1. Probe the four pins of the connected channel
2. Load the example and capture signals during the initialization sequence
3. Compare with the expected initialization communication shown below:
\image html EnDat_debug_init_responses.png "EnDat Initialization Responses"

Verify that the clock frequency is set to 200KHz. The TX and RX signals should show meaningful data exchanges for all initialization commands.

If the initialization communication is incorrect, possible causes include:

1. Firmware not loaded into the correct PRU core
2. Firmware stuck due to:
   - No response or incorrect response from encoder
   - Incorrect channel selection or incorrect clock configuration

Troubleshooting steps:
1. Verify hardware connections
2. Confirm the pin settings and the clock configuration
3. If the above are correct, debug the PRU firmware by connecting to the appropriate core as described in the \ref ENCODER_EXAMPLES_DEBUG_GUIDE

> **Note:** For issues occurring after encoder initialization, debug by examining register values and capturing TX, RX, and clock signals.

## Test Case Description

<table>
    <tr>
        <th style="width:4%">S.No</th>
        <th>Test Details
        <th>Steps
        <th>Pass/Fail Criteria
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
        <td>5. Enter 0 in "parameter value"  for setting value in "Error message"</td>
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
        <td>5. Enter 0 in "parameter value" for setting value in "Error message"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="8" style="text-align: center">7.</td>
        <td rowspan="8" style="text-align: center">To set values to encoder's manufacturing parameters for Endat 2.2 (Status of additional info)</td>
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
        <td>7. Enter 1235 (or any 2 byte value) in "parameter value" for setting value in "Status of additional info"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>8. Enter 8 to select "Encoder send position values + Additional Information(s)"<br>&emsp; <b>Note: </b>Write is not permanent. When read again using Command 11, encoder will return the default value</td>
        <td style="text-align: center">Values followed by 0x45 represent the last byte of the data received by encoder<br> CRC success </td>
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
        <td rowspan="5" style="text-align: center">To check position value with additional info.</td>
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
        <td rowspan="7" style="text-align: center">To receive encoder's operating parameters (error message)
		+ receive position value with additional info
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
		for Endat 2.2 + receive position value with additional info
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
        <td rowspan="5" style="text-align: center">To acknowledge MRS code for Endat 2.2</td>
        <td>1. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter "47"  in MRS code to select "Acknowledge MRS code" of Additional Information 1</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter 9 to select "Encoder send position values + Additional Information(s) and Selection of memory area"</td>
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
		+ receive position value with additional info</td>
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
        <td>5. Enter 0 in "parameter value" for setting value in "Error message"</td>
        <td style="text-align: center">CRC success</td>
    </tr>
    <tr>
        <td rowspan="5" style="text-align: center">16.</td>
        <td rowspan="5" style="text-align: center">To set values to encoder's manufacturing parameters for Endat 2.2 (Status of additional info)
		+ receive position value with additional info
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
        <td>5. Enter 0 in "parameter value" for setting value in "Status of additional info"</td>
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
        <td rowspan="7" style="text-align: center">To reset encoder + receive position value with additional info</td>
        <td>1. Enter 12 to select "Encoder send position values + Additional Information(s) and receive error reset"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>1. Enter 14 to select "Encoder receive communication command"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>2. Enter ______ in "enter encoder address" </td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter _____ in "instruction hex value"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>1. Enter 14 to select "Encoder receive communication command"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>2. Enter ______ in "enter encoder address" </td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td>3. Enter _____ in "instruction hex value"</td>
        <td style="text-align: center">CRC success </td>
    </tr>
    <tr>
        <td rowspan="2" style="text-align: center">20.</td>
        <td rowspan="2" style="text-align: center">Configure Clock </td>
        <td>1. Enter 100 to select "configure clock"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter ___ for clock frequency (in Hz)</td>
        <td style="text-align: center">CRC success (Tested up to 8MHz)</td>
    </tr>
    <tr>
        <td rowspan="3" style="text-align: center">21.</td>
        <td rowspan="3" style="text-align: center">Simulate motor control 2.1 position loop</td>
        <td>1. Enter 101 to select "Simulate motor control 2.1 position loop"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter 1000 to select "clock frequency"</td>
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
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td>3. Enter ___ to "select time in ns"</td>
        <td> </td>
    </tr>
    <tr>
        <td rowspan="3" style="text-align: center">26.</td>
        <td rowspan="3" style="text-align: center">Configure rx clock disable time</td>
        <td>1. Enter 106 to select "configure rx clock disable time"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter 0 to select channel 0</td>
        <td> </td>
    </tr>
    <tr>
        <td>3. Enter ___ to "select time in ns"</td>
        <td> </td>
    </tr>
    <tr>
        <td rowspan="3" style="text-align: center">27.</td>
        <td rowspan="3" style="text-align: center">Simulate motor control 2.2 position loop (safety)</td>
        <td>1. Enter 107 to select "Simulate motor control 2.2 position loop"</td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter 1000 to select "clock frequency"</td>
        <td> </td>
    </tr>
    <tr>
        <td>3. Rotate the rotor of motor and see the changes in Position value on UART</td>
        <td style="text-align: center">Position Values are changing when rotor moves </td>
    </tr>
    <tr>
        <td rowspan="2" style="text-align: center">28.</td>
        <td rowspan="2" style="text-align: center">Configure propagation delay (td)</td>
        <td>1. Enter 108 to select configure propagation delay </td>
        <td> </td>
    </tr>
    <tr>
        <td>2. Enter 0 to select channel 0</td>
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
        <td>2. Enter 110 for read recovery time information </td>
        <td style="text-align: center">Recovery Time is set to 1.25 us <= RT <= 3.75us or  10 us <= RT <= 30 us</td>
    </tr>
    <tr>
        <td rowspan="5" style="text-align: center">31.</td>
        <td rowspan="5" style="text-align: center">To test periodic CMP mode</td>
        <td>1. Enter 200 to enable periodic CMP mode </td>
        <td style="text-align: center"> </td>
    </tr>
    <tr>
        <td>2. Enter IEP reset count (in IEP timer count) - the periodic cycle time </td>
        <td style="text-align: center"> </td>
    </tr>
    <tr>
        <td>3. Enter channel trigger count (in IEP timer count) - when to trigger position read within the cycle </td>
        <td style="text-align: center"> </td>
    </tr>
    <tr>
        <td>4. Select position command (Valid commands: 1, 8, 9, 10, 11, 13) </td>
        <td style="text-align: center">Position Values are changing when rotor moves </td>
    </tr>
    <tr>
        <td><b>Note:</b> If the selected command requires supplements (e.g., command 9 needs MRS code), the system will prompt for those parameters before the periodic mode starts.</td>
        <td style="text-align: center"> </td>
    </tr>
    <tr>
        <td rowspan="5" style="text-align: center">32.</td>
        <td rowspan="5" style="text-align: center">To test periodic CAP mode</td>
        <td>1. Enter 201 to enable periodic CAP mode </td>
        <td style="text-align: center"> </td>
    </tr>
    <tr>
        <td>2. Enter IEP SYNC0 period (in IEP time count) - the CAP event generation period </td>
        <td style="text-align: center"> </td>
    </tr>
    <tr>
        <td><b>Note:</b> Step 2. valid for AM243x examples, For AM26x it is configured in SysConfig </td>
        <td style="text-align: center"> </td>
    </tr>
    <tr>
        <td>3. Select position command (Valid commands: 1, 8, 9, 10, 11, 13) </td>
        <td style="text-align: center">Position Values are changing when rotor moves </td>
    </tr>
    <tr>
        <td><b>Note:</b> If the selected command requires supplements (e.g., command 9 needs MRS code), the system will prompt for those parameters before the periodic mode starts.</td>
        <td style="text-align: center"> </td>
    </tr>
    <tr>
        <td rowspan="2" style="text-align: center">33.</td>
        <td rowspan="2" style="text-align: center">Long term test</td>
        <td>1. Enter 111 to enable Long time continuous mode  </td>
        <td></td>
    </tr>
    <tr>
        <td>2. Press enter to stop the long term test</td>
        <td style="text-align: center">The result shows the number of position commands sent and the number of CRC failures received</td>
    </tr>
</table>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.
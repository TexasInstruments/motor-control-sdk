# BISS-C Diagnostic {#EXAMPLE_MOTORCONTROL_BISSC}
[TOC]
\cond SOC_AM243X

BISS-C diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 200MHz (default is 200 MHz, 300 MHz can also be used)
- Initializes ICSS0-PRU1
- Initializes default parameters, loads the PRU firmware & executes it.

\note BiSS-C firmware supports operation with ICSS Core Clock running at 200 MHz/300 MHz frequency or ICSS UART Clock running at 192 MHz only. ICSS Core Clock at 225/250/333 MHz is not supported due to clock divider requirements.

\endcond

\cond SOC_AM261X

BISS-C diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 225MHz
- Initializes ICSS0-PRU0
- Initializes default parameters, loads the PRU firmware & executes it.

\note BiSS-C firmware supports operation with ICSS UART Clock running at 160 MHz only, when ICSS Core Clock is 225 MHz due to clock divider requirements.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

BISS-C diagnostic application does the following:

- Configures pinmux, GPIO, UART, ICSS clock to 200MHz
- Initializes ICSS-PRU0
- Initializes default parameters, loads the PRU firmware & executes it.

\note BiSS-C firmware supports operation with ICSS Core Clock running at 200 MHz frequency or ICSS UART Clock running at 192 MHz frequency only.

\endcond

This application is controlled with a terminal interface using a serial over USB connection between the PC host and the EVM.
Please connect a USB cable between the PC and the EVM/LP.
A serial terminal application (like teraterm/ hyperterminal/ minicom) is then run on the host.
To configure, select the serial port corresponding to the port emulated over USB by the EVM.
The host serial port should be configured to 115200 baud, no parity, 1 stop bit and no flow control.

The BISS-C receiver firmware running on ICSS0-PRU1 provides a defined interface. The BISS-C diagnostic application interacts with the BISS-C receiver firmware interface. It then presents the user with menu options to select Data ID code. The application collects the data entered by the user and configures the relevant interface. Then via the BISS-C receiver interface, the command is triggered. Once the command completion is indicated by the interface, the status of the transaction is checked. If the status indicates success, the result is presented to the user.

\cond SOC_AM243X
## Channel Selection In Sysconfig

\image html bissc_syscfg_ch_sel.png      "Channel Selection In Sysconfig"

\image html Endat_channel_selection_configuration.png     "BiSS-C configuration selection between Single/Multi channel"

\endcond

### Periodic Continuous Mode
Current SDK example uses IEP CMP event to trigger periodic mode. CMP0 is used to get periodic CMP events by resetting the IEP counter continuously. Firmware triggers a R5 interrupt after getting a response from the encoder. The application code uses a callback function to clear the PRU interrupt, which can be modified as per the use case. Currently, command 6 is used to demonstrate the periodic mode, which informs the firmware to use position cmd 3. It prints the response written by the firmware on the UART terminal.
CMP3 is used for single channel and multi-channel PRU mode, and for load-share mode CMP5 is used for RTU core and CMP6 is used for TX PRU core channel. To use changes in CMP event, the following macros need to be updated in the application `bissc_periodic_trigger.h` file and source file `bissc_params.h`:
```c
#define IEP_CH0_CMP_EVNT ( 3 )
#define IEP_CH1_CMP_EVNT ( 5 )
#define IEP_CH2_CMP_EVNT ( 6 )
```

> **Note:** To disable IEP counter rest by CMP0 event, the following code needs to be disabled in `bissc_periodic_trigger.c`:
```c
event |= IEP_CMP0_ENABLE;
event |= IEP_RST_CNT_EN;
```

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
    <td>Folder containing BISS-C PRU firmware sources.</td>
</tr>
<tr>
    <td>driver/</td>
    <td>BISS-C diagnostic driver.</td>
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
 Board          | @VAR_LP_BOARD_NAME_LOWER (2 channel and 1 channel examples)
 Example folder | examples/position_sense/bissc_diagnostic

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSM          | ICSSM
 PRU            | PRU0
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER (Single channel example)
 Example folder | examples/position_sense/bissc_diagnostic

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSM1
 PRU            | PRU0
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/position_sense/bissc_diagnostic

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

### Hardware Setup (Using Booster Pack & LP-AM243)
\imageStyle{AM243x_lp_bp_bissc_encoder_setup.png,width:40%}
\image html AM243x_lp_bp_bissc_encoder_setup.png  "Hardware Setup of Booster Pack + LP for BISS-C"

\note
    - The PROC109A version of LP supports two channels
    - To enable the second channel on LP, SW6 needs to be turned OFF

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

### Hardware Setup (Using Booster Pack & LP-AM261)
\imageStyle{AM261x_lp_bp_bissc_encoder_setup.png,width:40%}
\image html AM261x_lp_bp_bissc_encoder_setup.png  "Hardware Setup of Booster Pack + LP for BISS-C"

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

### Hardware Setup (Using Booster Pack & LP-AM263)
\imageStyle{AM263x_lp_bp_bissc_encoder_setup.png,width:40%}
\image html AM263x_lp_bp_bissc_encoder_setup.png  "Hardware Setup of Booster Pack + LP for BISS-C"

#### LP-AM263 Jumper Configuration

\endcond

\cond SOC_AM263PX

### Hardware Setup (Using Booster Pack & LP-AM263P)
\imageStyle{AM263Px_lp_bp_bissc_encoder_setup.png,width:40%}
\image html AM263Px_lp_bp_bissc_encoder_setup.png  "Hardware Setup of Booster Pack + LP for BISS-C"

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

Shown below is a sample output to enable safety and the safety encoder results:
\imageStyle{bissc_safety_sample_output.png,width:60%}
\image html bissc_safety_sample_output.png "BISS-C Sample Output when safety is enabled"


## BiSS-C Debug Guide {#BISSC_DEBUG_GUIDE}
This section describes how to debug the BiSS-C application, including a guide to debugging the BiSS-C example and firmware. It mainly focuses on verifying the configuration of all components and encoder registers as described in \ref ENCODER_EXAMPLES_DEBUG_GUIDE.

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
        <th>#
        <th>Name
        <th>Description
        <th>Pass/fail Criteria
    </tr>
    <tr>
        <td>1</td>
        <td>Data readout (absolute position data)</td>
        <td>Absolute rotor position value, errors, and warnings are received.
		</td>
        <td>CRC success with ABS, E, W and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>2</td>
        <td>Control Communication</td>
        <td>Absolute rotor position value, errors, and warnings, along with the result of the control communication command are received.
		</td>
        <td>CRC success with ABS position value, E, W and CRC values of position data along with the control communication result printed in the terminal.</td>
    </tr>
    <tr>
        <td>3</td>
        <td>Start periodic continuous mode</td>
        <td>Absolute rotor position value, errors, and warnings are received periodically. Rotate the rotor of motor and see the changes in position value on UART.
		</td>
        <td>0 CRC errors with ABS position value, E, W and CRC values printed in the terminal.</td>
    </tr>
    <tr>
        <td>4</td>
        <td>Enable safety mode</td>
        <td>Enable safety mode by using control communication</td>
        <td>Safety should be enabled and CRC and Sign of Life counters will be displayed from next position data request</td>
    </tr>
</table>

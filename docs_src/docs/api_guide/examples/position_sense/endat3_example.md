# EnDAT3 Diagnostic {#EXAMPLE_MOTORCONTROL_ENDAT3}

[TOC]

The EnDAT3 diagnostic application demonstrates the EnDAT3 receiver operation with comprehensive command support for both foreground and background communication channels.

EnDAT3 is the next generation bidirectional interface for position encoders, offering significant improvements over EnDAT 2.2. During EnDAT3 operation, the EnDAT3 receiver communicates with the EnDAT3 position encoder using Manchester encoding with data rates up to 25 Mbps.

\note This implementation uses Peripheral input/output mode of PRU-ICSS. Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details.

The EnDAT3 driver provides a well-defined set of APIs to expose the EnDAT3 receiver interface with enhanced safety features and higher data rates compared to EnDAT 2.2.

The diagnostic invokes these APIs to:
- Initialize EnDAT3 interface
- Select channel configuration based on SysConfig
- Configure host trigger mode or periodic trigger mode
- Run the firmware on the selected PRU core

Once these steps are executed:
- The driver initializes communication with the encoder using the HELLO command
- The encoder wakes up and establishes EnDAT3 communication (may take up to 300ms)
- The diagnostic presents a user-friendly menu interface
- Users can select from foreground commands, background commands, continuous mode, or periodic mode

## Key Improvements Over EnDAT 2.2

EnDAT3 provides several significant enhancements:

- **Higher Data Rates:** Up to 25 Mbps (current implementation supports 12.5 Mbps, compared to EnDAT 2.2's maximum of ~16 MHz with legacy encoding)
- **Manchester Encoding:** Self-clocking data transmission for improved reliability
- **Enhanced Safety:** Additional safety features and error detection mechanisms
- **Bidirectional Background Communication:** Parallel data exchange alongside position data
- **Low Priority Frames (LPF):** Flexible send lists for transmitting additional encoder information
- **Frame Structure:** Organized into High Priority Frame (HPF), Low Priority Header (LPH), and Low Priority Frame (LPF)

## EnDAT3 Protocol Overview

### System Architecture

The EnDAT3 system consists of multiple layers working together to provide position encoder communication:

\imageStyle{Endat3_system_design_dfd.jpg,width:80%}
\image html Endat3_system_design_dfd.jpg "EnDAT3 System Design - Data Flow Diagram"

### Communication Frame Structure

EnDAT3 communication uses a structured frame format consisting of three main components:

1. **High Priority Frame (HPF)**
   - Contains position data (up to 48 bits)
   - Status byte with F (Error), W (Warning), and HPFV (HPF Valid) flags
   - 8-bit CRC for data integrity
   - Always transmitted in every cycle

2. **Low Priority Header (LPH)**
   - 8-bit status field indicating communication state (IDLE, RX_START, RX_LAST, BUSY)
   - Number of LPF frames following (num_lpf field)
   - 8-bit CRC for header integrity
   - Manages background communication flow

3. **Low Priority Frame (LPF)**
   - Contains additional encoder data (48 bits per frame)
   - Configurable via send lists (DATA0-DATA7 commands)
   - 8-bit Frame ID (FID) for identification
   - Separate 8-bit CRC per frame
   - Multiple LPF frames can be transmitted per cycle

### Manchester Encoding

EnDAT3 uses Manchester encoding for data transmission:
- Self-clocking: Clock information embedded in data signal
- Each bit period divided into two half-bits
- Logic '1': High-to-Low transition in middle of bit period
- Logic '0': Low-to-High transition in middle of bit period
- Provides robust noise immunity and DC-free transmission
- Requires 8x oversampling for reliable decoding

### Command Preamble and Postamble

Each EnDAT3 command transmission includes:
- **Preamble:** Minimum 25 halfbits of alternating pattern for synchronization
- **Command Data:** Manchester-encoded foreground and background operation codes
- **Postamble:** 4 halfbits to complete the transmission

### Data Rates

EnDAT3 protocol specification defines two primary data rates:
- **12.5 Mbps:** 25 MHz clock (2 halfbits per clock cycle) - **Supported**
- **25 Mbps:** 50 MHz clock (2 halfbits per clock cycle) - **Not currently supported**

\note Current implementation supports only 12.5 Mbps data rate. The RATE command is available in the diagnostic menu but changing to 25 Mbps is not functional in this release.

## Features Demonstrated

The diagnostic application demonstrates:
- **Foreground Communication:**
  - DATA0-DATA7: Activate different LPF send lists (configurable encoder data)
  - DATA: General data with background data
  - DATANOP: Data without background data
  - RESET: Encoder reset functionality (hard reset or soft reset)
  - CLEAR: Reset encoder states (F, W, REF flags)
  - ECHO: Measure propagation time for cable delay compensation
  - RATE: Data rate configuration command (note: only 12.5 Mbps is currently supported)
  - HELLO: Initialization sequence to wake up encoder on power-up and establish EnDAT3 communication

- **Background Communication:**
  - NOP: No operation
  - READ: Read from encoder memory
  - WRITE: Write to encoder memory
  - RECONFIGURE: Reconfigure encoder parameters
  - AUTH: Authenticate with user level and password
  - PROTECT: Set memory protection levels
  - SETPASS: Set password for user level
  - LOCATE: Encoder location function

- **Operating Modes:**
  - Host trigger mode: Command-driven communication
  - Periodic trigger mode: Automatic position updates via IEP timer
  - Continuous mode: Non-stop position fetching

- **Data Display:**
  - High Priority Frame (HPF): Position data and status
  - Low Priority Header (LPH): Frame count and communication status
  - Low Priority Frame (LPF): Additional data with Frame ID
  - CRC verification for all frame types
  - Error and warning flag interpretation

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Demonstrated" section are not supported in this release, including the below:

- **25 Mbps Data Rate:** Currently only 12.5 Mbps is supported and tested
- **Multi-Channel Configuration:** Only single channel operation is supported
- **Daisy Chain Topology:** Daisy chaining multiple encoders on a single interface is not supported

### Limitations

This section describes known limitations of the current implementation:

- The 25 Mbps data rate requires higher clock frequencies and tighter timing constraints. Current implementation is validated for 12.5 Mbps operation only.
- Multi-channel support would require additional PRU cores or time-multiplexing, which is not implemented in the current firmware architecture.
- Daisy chain configuration requires special handling of multiple encoder responses in series, which is not supported in this release.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure the following settings for EnDAT3:

- **ICSS Instance Selection:**
\cond SOC_AM243X
  - Selecting the ICSSG instance (Tested on ICSSG0-PRU1)
\endcond
\cond SOC_AM261X
  - Selecting the ICSSM instance (Tested on ICSSM1-PRU0)
\endcond
\cond SOC_AM263PX
  - Selecting the ICSSM instance (Tested on ICSSM-PRU0)
\endcond

- **PRU Core Selection:** Configure which PRU core (PRU0 or PRU1) runs the EnDAT3 firmware
- **PINMUX Configuration:** Automatic pin configuration for:
  - Clock signal (TX_CLK)
  - Data output signal (TX)
  - Data input signal (RX)
  - Transmit enable signal (TX_EN)
- **Channel Selection:** Enable and configure encoder channels
- **Booster Pack Support:** Enable GPIO-based channel control when using BP-AM2BLDCSERVO
- **Clock Source Selection:** Choose between different TX/RX clock source options:
\cond SOC_AM243X
  - PRU Core Clock (300 MHz) - configured for 12.5 Mbps data rate
\endcond
\cond SOC_AM261X
  - PRU UART Clock (160 MHz) or PRU Core Clock (200 MHz) - configured for 12.5 Mbps data rate
\endcond
\cond SOC_AM263PX
  - PRU UART Clock (192 MHz) or PRU Core Clock (200 MHz) - configured for 12.5 Mbps data rate
\endcond

### Channel Selection In SysConfig

The SysConfig GUI allows easy configuration of the EnDAT3 interface channels. Below are examples showing channel selection for different platforms:

\cond SOC_AM243X

\imageStyle{Endat3_sysconfig1_am243x.png,width:70%}
\image html Endat3_sysconfig1_am243x.png "EnDAT3 SysConfig - Channel Selection for AM243x"

\imageStyle{Endat3_sysconfig2_am243x.png,width:70%}
\image html Endat3_sysconfig2_am243x.png "EnDAT3 SysConfig - Booster pack pin selection for AM243x"

For AM243x, channels can be enabled through GPIO control when using the booster pack configuration. The example uses GPIO to enable Channel 0 or Channel 2 via the ENC0_EN and ENC2_EN GPIO pins.
\endcond

\cond SOC_AM261X

\imageStyle{Endat3_sysconfig1_am261x.png,width:70%}
\image html Endat3_sysconfig1_am261x.png "EnDAT3 SysConfig - Channel Selection for AM261x"

\imageStyle{Endat3_sysconfig2_am261x.png,width:70%}
\image html Endat3_sysconfig2_am261x.png "EnDAT3 SysConfig - IOEXP configuration for AM261x"

For AM261x, the channel is selected via the IOEXP (I/O Expander) configuration in SysConfig. The TCA6408 I/O expander on the LaunchPad controls the booster pack multiplexer settings to route signals to the appropriate encoder channel.
\endcond

\cond SOC_AM263PX

\imageStyle{Endat3_sysconfig1_am263px.png,width:70%}
\image html Endat3_sysconfig1_am263px.png "EnDAT3 SysConfig - Channel Selection for AM263Px"

\imageStyle{Endat3_sysconfig2_am263px.png,width:70%}
\image html Endat3_sysconfig2_am263px.png "EnDAT3 SysConfig - IOEXP configuration for AM263Px"

For AM263Px, similar to AM261x, the channel selection is controlled via the TCA6416 I/O expander configuration in SysConfig, which manages the booster pack MUX selection.
\endcond

## Important Files and Directory Structure

<table>
<tr>
    <th>Folder/Files</th>
    <th>Description</th>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/position_sense/endat3_diagnostic</td></tr>
<tr>
    <td>endat3_diagnostic.c</td>
    <td>EnDAT3 diagnostic application</td>
</tr>
<tr>
    <td>endat3_periodic_trigger.h</td>
    <td>Periodic trigger mode header</td>
</tr>
<tr>
    <td>endat3_periodic_trigger.c</td>
    <td>IEP timer configuration for periodic mode</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/position_sense/endat3</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing EnDAT3 firmware sources</td>
</tr>
<tr>
    <td>driver/</td>
    <td>EnDAT3 driver implementation</td>
</tr>
<tr>
    <td>include/</td>
    <td>EnDAT3 API header files</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_MOTORCONTROL_ENDAT3_COMBOS}

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU0 or PRU1 (configurable via SysConfig)
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER (Single channel)
 Example folder | examples/position_sense/endat3_diagnostic

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSM          | ICSSM1
 PRU            | PRU0 or PRU1 (configurable via SysConfig)
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER (Single channel)
 Example folder | examples/position_sense/endat3_diagnostic

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSM          | ICSSM
 PRU            | PRU0 or PRU1 (configurable via SysConfig)
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER (Single channel)
 Example folder | examples/position_sense/endat3_diagnostic

\endcond

# Steps to Run the Example

Other than the basic EVM setup mentioned in <a href="@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html" target="_blank"> EVM Setup </a>, the following additional hardware is required to run this demo:

\cond SOC_AM243X

## Hardware Prerequisites with LP-AM243

- EnDAT3 Encoder
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> LP-AM243 Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
\endcond

\cond SOC_AM261X

## Hardware Prerequisites with LP-AM261

- EnDAT3 Encoder
- <a href="https://www.ti.com/tool/LP-AM261" target="_blank"> LP-AM261 Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
\endcond

\cond SOC_AM263PX

## Hardware Prerequisites with LP-AM263P

- EnDAT3 Encoder
- <a href="https://www.ti.com/tool/LP-AM263P" target="_blank"> LP-AM263P Board </a>
- <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank"> BP-AM2BLDCSERVO </a>
\endcond

\cond SOC_AM243X

## Hardware Setup with LP-AM243

\note
    - The PROC109A version of LP supports single channel EnDAT3
    - Enable channel via GPIO as configured in SysConfig

\imageStyle{Endat3_am243x_hw_setup.PNG,width:70%}
\image html Endat3_am243x_hw_setup.PNG "EnDAT3 Hardware Setup with LP-AM243 and BP-AM2BLDCSERVO"

#### EnDAT3 Connector Pinout

<table>
<tr>
    <th>S.No</th>
    <th>Booster Pack Axis I Pins</th>
    <th>EnDAT3 Connector No. (15 pin)</th>
</tr>
<tr>
    <td>1</td>
    <td>VENCODER</td>
    <td>4, 12</td>
</tr>
<tr>
    <td>2</td>
    <td>GND</td>
    <td>2, 10</td>
</tr>
<tr>
    <td>3</td>
    <td>DATP1 (data+)</td>
    <td>8</td>
</tr>
<tr>
    <td>4</td>
    <td>DATM1 (data-)</td>
    <td>15</td>
</tr>
</table>

#### Pin Configuration (AM243x)

The EnDAT3 interface on AM243x uses ICSSG0-PRU1 with the following pin mapping:

<table>
<tr>
    <th>Signal</th>
    <th>PRU GPIO</th>
    <th>LaunchPad Pin</th>
    <th>Description</th>
</tr>
<tr>
    <td>TX (Data Out)</td>
    <td>PRG0_PRU1_GPO0</td>
    <td>Configured via SysConfig</td>
    <td>Encoder command data output</td>
</tr>
<tr>
    <td>TX_CLK (Clock Out)</td>
    <td>PRG0_PRU1_GPO1</td>
    <td>Configured via SysConfig</td>
    <td>Clock signal to encoder</td>
</tr>
<tr>
    <td>TX_EN (Transmit Enable)</td>
    <td>PRG0_PRU1_GPO2</td>
    <td>Configured via SysConfig</td>
    <td>RS485 driver enable</td>
</tr>
<tr>
    <td>RX (Data In)</td>
    <td>PRG0_PRU1_GPI13</td>
    <td>Configured via SysConfig</td>
    <td>Encoder response data input</td>
</tr>
<tr>
    <td>ENC0_EN (Channel Enable)</td>
    <td>GPIO (MMC1_SDWP)</td>
    <td>Controlled by R5F</td>
    <td>Enable Channel 0 encoder path</td>
</tr>
<tr>
    <td>ENC2_EN (Channel Enable)</td>
    <td>GPIO (MMC1_SDCD)</td>
    <td>Controlled by R5F</td>
    <td>Enable Channel 1 encoder path</td>
</tr>
</table>

\note The PRU core clock is configured to 300 MHz to support EnDAT3 data rates (12.5 Mbps and 25 Mbps).

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
    <td>J18 installed: sets VSENSOR1 to 12.0V</td>
    <td>Axis 1: Encoder/Resolver Voltage Select</td>
</tr>
<tr>
    <td>J20/J21</td>
    <td>J20 installed: sets VSENSOR2 to 12.0V</td>
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

\cond (SOC_AM263PX)

## Hardware Setup with LP-AM263P

\imageStyle{Endat3_am263px_hw_setup.png,width:70%}
\image html Endat3_am263px_hw_setup.png "EnDAT3 Hardware Setup with LP-AM263P and BP-AM2BLDCSERVO"

#### EnDAT3 Connector Pinout

<table>
<tr>
    <th>S.No</th>
    <th>Booster Pack Axis I Pins</th>
    <th>EnDAT3 Connector No. (15 pin)</th>
</tr>
<tr>
    <td>1</td>
    <td>VENCODER</td>
    <td>4, 12</td>
</tr>
<tr>
    <td>2</td>
    <td>GND</td>
    <td>2, 10</td>
</tr>
<tr>
    <td>3</td>
    <td>DATP1 (data+)</td>
    <td>8</td>
</tr>
<tr>
    <td>4</td>
    <td>DATM1 (data-)</td>
    <td>15</td>
</tr>
</table>

#### LaunchPad Jumper Configuration

Connect the jumpers J13 and J26 for providing 3.3V and 5V to boosterpack.

#### Pin Configuration (AM263Px)

The EnDAT3 interface on AM263Px uses ICSSM-PRU0 with the following pin mapping:

<table>
<tr>
    <th>Signal</th>
    <th>PRU GPIO</th>
    <th>LaunchPad Pin</th>
    <th>Description</th>
</tr>
<tr>
    <td>TX (Data Out)</td>
    <td>PR0_PRU0_GPIO3</td>
    <td>Configured via SysConfig</td>
    <td>Encoder command data output</td>
</tr>
<tr>
    <td>TX_CLK (Clock Out)</td>
    <td>PR0_PRU0_GPIO4</td>
    <td>Configured via SysConfig</td>
    <td>Clock signal to encoder</td>
</tr>
<tr>
    <td>TX_EN (Transmit Enable)</td>
    <td>PR0_PRU0_GPIO5</td>
    <td>Configured via SysConfig</td>
    <td>RS485 driver enable</td>
</tr>
<tr>
    <td>RX (Data In)</td>
    <td>PR0_PRU0_GPIO10</td>
    <td>Configured via SysConfig</td>
    <td>Encoder response data input</td>
</tr>
<tr>
    <td>ENC1_EN (Channel Enable)</td>
    <td>GPIO (SDFM0_D1)</td>
    <td>Controlled by R5F</td>
    <td>Enable encoder channel 1 via GPIO</td>
</tr>
</table>

\note
- The PRU core clock and IEP clock are configured to 200 MHz
- I/O Expander (TCA6416) controls booster pack MUX configuration via I2C1
- Default example configuration uses Channel 1 (Channel 0 disabled)

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
    <td>J18 installed: sets VSENSOR1 to 12.0V</td>
    <td>Axis 1: Encoder/Resolver Voltage Select</td>
</tr>
<tr>
    <td>J20/J21</td>
    <td>J20 installed: sets VSENSOR2 to 12.0V</td>
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

## Hardware Setup with LP-AM261

\imageStyle{Endat3_am261x_hw_setup.png,width:70%}
\image html Endat3_am261x_hw_setup.png "EnDAT3 Hardware Setup with LP-AM261 and BP-AM2BLDCSERVO"

#### EnDAT3 Connector Pinout

<table>
<tr>
    <th>S.No</th>
    <th>Booster Pack Axis I Pins</th>
    <th>EnDAT3 Connector Pin No. (15 pins)</th>
</tr>
<tr>
    <td>1</td>
    <td>VENCODER</td>
    <td>4, 12</td>
</tr>
<tr>
    <td>2</td>
    <td>GND</td>
    <td>2, 10</td>
</tr>
<tr>
    <td>3</td>
    <td>DATP1 (data+)</td>
    <td>8</td>
</tr>
<tr>
    <td>4</td>
    <td>DATM1 (data-)</td>
    <td>15</td>
</tr>
</table>

#### LaunchPad Jumper Configuration

Connect the jumpers J13 and J26 for providing 3.3V and 5V to boosterpack.

#### Pin Configuration (AM261x)

The EnDAT3 interface on AM261x uses ICSSM1-PRU0 with the following pin mapping:

<table>
<tr>
    <th>Signal</th>
    <th>PRU GPIO</th>
    <th>LaunchPad Pin</th>
    <th>Description</th>
</tr>
<tr>
    <td>TX (Data Out)</td>
    <td>PR1_PRU0_GPIO0</td>
    <td>GPIO81</td>
    <td>Encoder command data output</td>
</tr>
<tr>
    <td>TX_CLK (Clock Out)</td>
    <td>PR1_PRU0_GPIO1</td>
    <td>GPIO82</td>
    <td>Clock signal to encoder</td>
</tr>
<tr>
    <td>TX_EN (Transmit Enable)</td>
    <td>PR1_PRU0_GPIO2</td>
    <td>GPIO83</td>
    <td>RS485 driver enable</td>
</tr>
<tr>
    <td>RX (Data In)</td>
    <td>PR1_PRU0_GPIO9</td>
    <td>GPIO80</td>
    <td>Encoder response data input</td>
</tr>
<tr>
    <td>ENC0_EN (Channel Enable)</td>
    <td>GPIO21</td>
    <td>Controlled by R5F</td>
    <td>Enable encoder channel via GPIO</td>
</tr>
</table>

\note
- The PRU core clock and IEP clock are configured to 200 MHz
- I/O Expander (TCA6408) controls booster pack MUX configuration via I2C0

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
    <td>J18 installed: sets VSENSOR1 to 12.0V</td>
    <td>Axis 1: Encoder/Resolver Voltage Select</td>
</tr>
<tr>
    <td>J20/J21</td>
    <td>J20 installed: sets VSENSOR2 to 12.0V</td>
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

## Build, Load and Run

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using
  the make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>
- Refer to the UART terminal for user interface menu options.

## Application Flow

### Application Flowchart

The following flowchart shows the complete application flow from initialization to command execution:

\imageStyle{Endat3_r5f_application_flowchart.jpg,width:70%}
\image html Endat3_r5f_application_flowchart.jpg "EnDAT3 R5F Application Flowchart"

### Initialization Sequence

1. **Hardware Initialization**
   - Open drivers (UART for console)
   - Initialize PRU-ICSS subsystem
\cond SOC_AM261X || SOC_AM263PX
   - Initialize I2C driver for I/O expander communication
   - Configure I/O expander (TCA6408/TCA6416) for booster pack MUX control
\endcond
   - Enable GPIO for booster pack channel selection (if applicable)
   - Configure GPIO pins for encoder channel enable signals

2. **EnDAT3 Interface Initialization**
   - Open EnDAT3 interface with `endat3_open()`
   - Load PRU firmware binary to selected PRU core
   - Configure PRU-ICSS INTC (Interrupt Controller) for event mapping
   - Initialize shared memory interface between R5F and PRU
   - Run PRU firmware
   - Set to host trigger mode by default (opmode = 1)
   - Clear any previous trigger states
   - Wait for encoder to be ready (up to 1 second timeout)

3. **EnDAT3 Communication Establishment**
   - Send HELLO command to encoder with background data 0x2222
   - Encoder wakes up and establishes EnDAT3 communication
   - Wait for initialization to complete (up to 300ms)
   - Verify successful initialization by checking encoder response
   - Configure initial data rate (typically starts at 12.5 Mbps)

4. **Main Menu Loop**
   - Display command type selection menu
   - Wait for user input via UART
   - Execute selected command type
   - Process and display results

### Command Type Selection

The application presents four command type options via the UART terminal interface:

\imageStyle{Endat3_uart_main_menu.png,width:70%}
\image html Endat3_uart_main_menu.png "EnDAT3 Main Menu - Command Type Selection"

The application presents four command type options:

1. **Foreground Communication (0)**
   - Single command execution
   - Position data with optional background data
   - Various encoder control commands

2. **Background Communication (1)**
   - Multi-cycle operation
   - Memory read/write operations
   - Parameter configuration
   - Authentication and protection

3. **Continuous Position Fetch (2)**
   - Non-stop position updates
   - Real-time data display
   - User-initiated stop

4. **Periodic Trigger Mode (3)**
   - IEP timer-driven updates
   - Deterministic cycle time
   - Automatic position requests

### Foreground Communication Commands

After selecting foreground communication, the menu presents:

\imageStyle{Endat3_uart_menu_foreground_comm.png,width:70%}
\image html Endat3_uart_menu_foreground_comm.png "EnDAT3 Foreground Communication Menu"

The foreground menu presents:

```
Select command for endat3 foreground communication:
 1: DATA0 (Activate LPF send list 0)
 2: DATA1 (Activate LPF send list 1)
 3: DATA2 (Activate LPF send list 2)
 4: DATA3 (Activate LPF send list 3)
 5: DATA4 (Activate LPF send list 4)
 6: DATA5 (Activate LPF send list 5)
 7: DATA6 (Activate LPF send list 6)
 8: DATA7 (Activate LPF send list 7)
 9: DATA (General data with BGD)
10: DATANOP (Data without BGD)
11: RESET (Encoder reset)
12: CLEAR (Resetting of states)
13: ECHO (Echo for measuring propagation time)
14: RATE (Set data transfer rate)
15: HELLO (Switch to EnDat 3 mode)
```

**Execution Flow:**
1. Parse user selection to command code
2. Set foreground operation code
3. Set background operation code to NOP
4. Prompt for additional parameters (if needed)
5. Set background data (for RESET, CLEAR, ECHO, RATE, HELLO)
6. Send command and wait for response
7. Display results with error checking

**Example - HELLO Command:**
```
User selects: 15
Application sets:
  - foreground_op_code = ENDAT3_REQ_HELLO
  - background_data[0] = 0x2222 (fixed value)
  - expected_tx_frames = 1
Sends command and receives response
Displays: "Successfully established EnDat3 communication"
Note:
  - HELLO must be sent after encoder power-up to wake up the encoder
  - HELLO must be sent after each baud rate change via RATE command
  - May take up to 300ms to complete
  - Establishes EnDAT3 communication and synchronization
```

**Example - RESET Command:**
```
User selects: 11
Menu prompts: "Select reset type:"
  1: Hard Reset (0xBBBB)
  2: Other (Enter custom value)
User selects: 1
Application sets:
  - foreground_op_code = ENDAT3_REQ_RESET
  - background_data[0] = 0xBBBB
Sends command and receives response
Waits additional 302ms for encoder restart
```

### Background Communication Commands

After selecting background communication, the menu presents:

\imageStyle{Endat3_uart_menu_background_comm.png,width:70%}
\image html Endat3_uart_menu_background_comm.png "EnDAT3 Background Communication Menu"

The background menu presents:

```
Select background request operation:
 1: No Operation (NOP)
 2: Read from encoder memory
 3: Write to encoder memory
 4: Reconfigure parameters
 5: Authentication
 6: Set protection
 7: Set password
 8: Locate function
```

**Multi-Cycle Execution:**

Background communication requires multiple command cycles depending on LPH status:

| LPH Status | State | Cycles Required | Behavior |
|------------|-------|-----------------|----------|
| 0 | IDLE | 4 | Send request in 4 cycles, response in 4th cycle |
| 1 | RX_START | 6 | Send request for 2 NOP cycles, then send request acc. to IDLE state|
| 2 | RX_LAST | 5 | Send request for 1 NOP cycle, then send request acc. to IDLE state |
| 3 | BUSY | N/A | Wait, cannot send request |

**Example - READ Operation:**
```
User selects: 2 (Read)
Menu prompts:
  "Enter number of words to read:" → 1
  "Enter address (MSB byte):" → 0xA1
  "Enter address (LSB 2 bytes):" → 0x0000

Application checks LPH status:
  If IDLE (0):
    endat3_handle_background_command_request(ch, 0, 4, READ, 0xA1, 0x0000, 1)
    Sends 4 command cycles with proper timing

Response processing:
  HPF: Position data
  LPH: Indicates 1 LPF frame following
  LPF[0]: Contains read data at address 0xA10000
```

**Example - AUTH Operation:**
```
User selects: 5 (Authentication)
Menu prompts:
  "Enter user level (0-255):" → 1 (OEM2)
  "Enter password (hex, up to 32 bits):" → 0x12345678

Application formats:
  addr_msb = 1 (user level)
  addr_lsb = (0x12345678 >> 16) & 0xFFFF = 0x1234
  data = 0x12345678 & 0xFFFF = 0x5678

Sends AUTH command with multi-cycle protocol
Response indicates authentication success/failure
```

### Continuous Position Fetch Mode

When user selects continuous mode (option 2):

1. **Setup:**
   - Create background task to monitor Enter key
   - Set position loop status to START
   - Display instructions to user

2. **Main Loop:**
   ```
   while (loop_status == START):
     Send DATA0 command
     Wait for response
     If HPF data valid:
       Extract position (30-bit value)
       Calculate angle = position * 360.0 / 2^30
       Display: "Position: 0x%x, Angle: %f degrees"
     Delay 1ms
     Check if user pressed Enter to stop
   ```

3. **Stop:**
   - User presses Enter
   - Background task sets loop_status to STOP
   - Main loop exits
   - Return to main menu

### Periodic Trigger Mode

When user selects periodic mode (option 3):

1. **Configuration:**
   ```
   Menu prompts:
     "Enter IEP reset cycle count (in IEP cycles):" → cmp0_val
     "Enter IEP trigger time (in IEP cycles):" → cmp3_val

   Validation: cmp3_val ≤ cmp0_val
   ```

2. **IEP Setup:**
   - Configure IEP CMP0 for counter reset (defines period)
   - Configure IEP CMP3 for trigger event (defines trigger point)
   - Enable IEP counter
   - Set firmware to periodic mode (opmode = 0)

3. **Firmware Operation:**
   - Pre-configure command (DATA0)
   - Set expected TX frame count to 1
   - Release start trigger to firmware
   - IEP CMP3 events automatically trigger commands

4. **Data Display Loop:**
   ```
   while (loop_status == START):
     Wait for firmware busy flag to clear
     Receive response
     If successful and HPF valid:
       Extract position and angle
       Display values
     Delay 10ms
     Check if user pressed Enter to stop
   ```

5. **Stop and Cleanup:**
   - Stop IEP timer
   - Set firmware back to host trigger mode (opmode = 1)
   - Wait for mode switch to complete
   - Return to main menu

## Response Display

### Position Information Display

For successful DATA commands, the application displays:

```
Position Information:
 HPF Status: 0x20
 - HPF data valid (HPFV bit set)
 Position Data: 0x12345678
 CRC: 0xA5
 Additional Information:
 LPH Status (communication Status): 0x2
 LPF Status: 0x10
 LPF Data: 0x01 0x02 0x03 0x04 0x05 0x06
 LPF CRC: 0x5A
```

**HPF Status Flags:**
- Bit 0 (F): Error flag
- Bit 1 (W): Warning flag
- Bit 5 (HPFV): HPF data valid
- Bit 6 (RM): Absolute value available

### Error Status Display

When errors are detected:

```
HPF and LPH Status:
 HPF Status: 0x01
 LPH Status: 0x00
 Error Code: 0x1234
 Description: Voltage supply too low
 Recommended action: Check encoder power supply
```

The application uses `endat3_getErrorCode()`, `endat3_getErrorDescription()`, and `endat3_getErrorAction()` to provide detailed error information.

## Periodic Trigger Mode Configuration

### IEP Timer Configuration

The periodic mode uses IEP (Industrial Ethernet Peripheral) timer events:

- **CMP0 Event:** Resets IEP counter to create periodic behavior
- **CMP3 Event:** Triggers command transmission to encoder

**Example Configuration:**
```
PRU Core Clock: 300 MHz
Desired Position Update Rate: 1 kHz (1ms period)

IEP counter runs at PRU core clock frequency
CMP0 value = 300 MHz / 1 kHz = 300,000 cycles
CMP3 value = 10,000 cycles (trigger 10,000 cycles after reset)

User enters:
  IEP reset cycle count: 300000
  IEP trigger time: 10000
```

**Timing Diagram:**
```
Counter: 0 ──────────> 10000 ───────────────────> 300000 ─> 0
         ↑             ↑                           ↑          ↑
         Reset         Trigger                    Reset      Trigger
                       (CMP3)                     (CMP0)     (CMP3)
                       Command Sent               Command Sent
```

### IRQ Callback

In periodic mode, firmware generates R5F interrupt after encoder response:
```c
void endat3_iep_isr_callback(void *args) {
    // Called by firmware after receiving encoder response
    // Application can clear interrupt and process new position data
    // Currently prints position to UART in diagnostic
}
```

## Firmware Flow

The EnDAT3 firmware implements the low-level protocol handling on the PRU cores. The firmware consists of transmit and receive processing paths:

### Firmware Transmit Flow

\imageStyle{endat3_firmware_tx_flow.png,width:70%}
\image html endat3_firmware_tx_flow.png "EnDAT3 Firmware Transmit Processing Flow"

### Firmware Receive Flow

\imageStyle{endat3_firmware_rx_flow.jpg,width:70%}
\image html endat3_firmware_rx_flow.jpg "EnDAT3 Firmware Receive Processing Flow"

The firmware handles Manchester encoding/decoding, frame assembly/parsing, CRC calculation, and timing control at the PRU level.

## EnDat3 Debug Guide {#ENDAT3_DEBUG_GUIDE}

This section describes how to debug the EnDAT3 application, including a guide to debugging the EnDAT3 example and firmware.

### Common Debugging Steps

Several common debugging procedures apply to EnDAT3:
- Verify register configuration
- Check hardware connections
- Debug firmware execution
- Analyze protocol timing

Refer to \ref ENCODER_EXAMPLES_DEBUG_GUIDE for detailed encoder debugging procedures.

### EnDAT3-Specific Debugging

If the EnDAT3 interface is not initializing correctly, follow these steps:

#### Initialization Failures

1. **Probe the encoder signals:**
   - Clock (CLK)
   - Data Out (TX)
   - Data In (RX)
   - Transmit Enable (TX_EN)

2. **Capture HELLO command:**
   - Should see clock at 25 MHz (for 12.5 Mbps mode)
   - TX should show Manchester-encoded HELLO command
   - RX should show encoder response after ~300ms

3. **Verify timing:**
   - Preamble: Minimum 25 halfbits
   - Command: Manchester encoded
   - Postamble: 4 halfbits

4. **Common issues:**
   - Firmware not loaded to correct PRU core
   - Incorrect channel selection
   - Encoder not powered or not connected
   - Cable delay too large for configured timing

#### Frame Reception Issues

If frames are received but CRC fails:

1. **Check Manchester decoding:**
   - Verify LUT values in driver
   - Check oversample rate (should be 8x)
   - Verify clock alignment

2. **Analyze frame structure:**
   - HPF: 48-bit data + 8-bit status + 8-bit CRC
   - LPH: 8-bit status + 8-bit num_lpf + 16-bit reserved + 8-bit CRC
   - LPF: 8-bit status + 48-bit data + 8-bit CRC

3. **Debug with raw data:**
   - Read RX buffer directly from PRU memory
   - Manually decode Manchester encoding
   - Calculate expected CRC

#### Background Communication Issues

If background commands fail:

1. **Check LPH status:**
   - Must be IDLE (0) or proper state for request
   - Cannot send request when BUSY (3)

2. **Verify cycle count:**
   - Different states require different cycle counts
   - Application must handle multi-cycle protocol

3. **Examine LPF responses:**
   - Check Frame ID (FID) in LPF status byte
   - Verify LPF data corresponds to request

#### Periodic Mode Issues

If periodic mode doesn't trigger:

1. **Verify IEP configuration:**
   - CMP0 and CMP3 values properly set
   - IEP counter enabled and running
   - CMP events properly configured

2. **Check firmware mode:**
   - Operating mode must be set to 0 (periodic)
   - Start trigger must be released
   - Command parameters pre-configured

3. **Monitor IRQ generation:**
   - Firmware should generate interrupt after response
   - R5F ISR should be called
   - Check INTC configuration

> **Note:** For detailed PRU firmware debugging, connect to the PRU core using CCS and set breakpoints in the firmware code as described in \ref ENCODER_EXAMPLES_DEBUG_GUIDE.

## Test Case Summary

The EnDAT3 diagnostic application provides comprehensive UART menu-based test coverage for all protocol features. The test suite covers 33 functional test cases accessible through the interactive UART terminal interface, organized into the following categories:

### 1. Foreground Communication Tests (13 Test Cases)

Foreground commands are accessed via menu option `0` and provide single-cycle position and control operations:

**Basic Position Commands:**
- **DATA0-DATA7** (Menu options 1-8): Activate Low Priority Frame (LPF) send lists 0-7 for retrieving different encoder data sets configured in the encoder
- **DATA** (Menu option 9): General data command with background data support
- **DATANOP** (Menu option 10): Position data request without background data (uses fixed value 0x0000)

**Control Commands:**
- **RESET** (Menu option 11): Encoder reset functionality with two sub-options:
  - Hard Reset (0xBBBB): Full device restart, takes up to 300ms
  - Custom Reset: User-defined reset codes for specific reset types
- **CLEAR** (Menu option 12): Reset encoder states (F, W, REF flags) with three selective prompts for flag clearing
- **ECHO** (Menu option 13): Measure propagation time for cable delay compensation with user-defined echo data input
- **RATE** (Menu option 14): Data rate configuration (12.5 Mbps supported; 25 Mbps command available but not functional in current release)
- **HELLO** (Menu option 15): Initialize EnDAT3 communication (uses fixed value 0x2222, takes up to 300ms to establish communication)

### 2. Background Communication Tests (8 Test Cases)

Background commands are accessed via menu option `1` and require multi-cycle operation based on LPH state machine:

**Memory Operations:**
- **NOP** (Menu option 1): No operation background command for testing background communication without data transfer
- **READ** (Menu option 2): Read from encoder memory with prompts for:
  - Number of words to read
  - Address MSB byte (hex)
  - Address LSB 2 bytes (hex)
- **WRITE** (Menu option 3): Write to encoder memory (subject to protection levels) with prompts for address and data word
- **RECONFIGURE** (Menu option 4): Reconfigure encoder parameters, causes encoder to enter BUSY state during reconfiguration

**Security Operations:**
- **AUTH** (Menu option 5): Authenticate with user level and password, with prompts for:
  - User level: 0=USER, 1=OEM2, 2=OEM1, 3=MANUFACTURER
  - Password (hex, up to 32 bits)
- **PROTECT** (Menu option 6): Set memory protection with three modes:
  - QUERY (0x01): Query current read/write access levels for specified address
  - SET_READ (0x02): Set read access protection level for address range
  - SET_WRITE (0x03): Set write access protection level for address range
- **SETPASS** (Menu option 7): Set password for user authentication levels with prompts for user level and new password
- **LOCATE** (Menu option 8): Encoder location function for physical identification with control value input

### 3. Continuous Position Fetch Tests (2 Test Cases)

Continuous mode is accessed via menu option `2`:

- **Continuous Mode Start/Stop**: Non-stop position updates displaying "Position: 0x________, Angle: ___.___ degrees" continuously at ~1000 Hz until Enter key pressed
- **Position Value Verification**: Verify 30-bit position values and angle calculations (360° = 2^30 counts = 1073741824) while rotating encoder

### 4. Periodic Trigger Mode Tests (1 Test Case with 3 Scenarios)

Periodic mode is accessed via menu option `3` with IEP timer configuration:

- **Invalid Configuration Handling**: Test CMP3 > CMP0 validation with error message "ERROR: Trigger time (CMP3=X) must be <= Reset cycle (CMP0=Y)"
- **Normal Frequency Operation**: Test periodic updates at configurable rates (e.g., 5 Hz with CMP0=20000000, CMP3=10000000 at 100 MHz IEP clock)
- **High Frequency Operation**: Test 1 kHz position updates (CMP0=100000, CMP3=50000) for real-time performance validation

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.


# %SDFM Example With Phase Compensation {#BASIC_SDFM_EXAMPLE_WITH_PHASE_DELAY}

[TOC]

# ICSS SDFM three channel with phase compensation
This example demonstrates phase compensation implemented for channel 0. It uses PRU-ICSSG IEP SYNC0 and SYNC1 to generate phase delayed clock for %SDFM interface and modulator.

It measures the phase delay for %SDFM channel 0 in PRU GPIO mode during initialization. Normal current Over-sampling Ratio (OSR), Over current OSR and Normal current trigger time can be configured by the user.

> **Note:** The current firmware supports measuring the phase delay using the SD0_DATA pin and the SD8_CLK pin for a single-axis use case. A similar measurement can be applied and implemented for multi-axis or individual channels as described in \ref PHASE_COMPENSATION.

This example demonstrates a three-channel single PRU configuration that performs the following tasks:
- Configures ICSSG0 IEP0 to generate a clock for the %SDFM interface and modulator.
- Enables phase compensation measurement.
- Configures the SYNC1 delay register based on the measured phase delay.
- Sets up %SDFM channels: Channel 0, Channel 1 and Channel 2.
- Configures the normal current sample trigger time, oversampling ratio (OSR), and sinc filter type.

# Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr>
    <td>${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_three_channel_with_phase_compensation</td>
    <td> Application specific sources for ICSS %SDFM with phase compensation </td>
</tr>
<tr>
    <td>${SDK_INSTALL_PATH}/examples/current_sense</td>
    <td> Common source for ICSS %SDFM applications </td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/current_sense/sdfm</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing ICSS %SDFM firmware sources</td>
</tr>
<tr>
    <td>driver/</td>
    <td>ICSS %SDFM driver source</td>
</tr>
<tr>
    <td>include/</td>
    <td>Folder containing ICSS %SDFM structures and APIs declarations</td>
</tr>
</table>

# Supported Combinations

\cond SOC_AM243X

 Parameter       | Value
 ----------------|-----------
 CPU + OS        | r5fss0-0 freertos
 ICSSG           | ICSSG0
 PRU             | PRU0
 Toolchain       | ti-arm-clang
 Board           | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Examples folder | examples/current_sense

\endcond

# Steps to Run the Example

## Hardware Prerequisites for EVM
Other than the basic EVM setup mentioned in <a href="@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html" target="_blank"> EVM Setup </a>, the additional hardware listed below is required to run this demo
- TMDS64DC01EVM IO Link/Breakout Board
- AMC1035EVM
- TMDS243EVM
- Signal generator


### EVM Hardware Setup 
\imageStyle{SDFM_EVMHw_SETUP_image.jpeg,width:40%}
\image html SDFM_EVMHw_SETUP_image.jpeg "Hardware Setup SDFM"
\image html SDFM_EVM_SETUP_FOR_PHASE_DELAY.png "SDFM: EVM and IO breakout board setup view"
\cond SOC_AM243X
## Hardware Prerequisites for LP
- AMC1035EVM
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> LP-AM243 Board </a>
- Signal generator

### LP Hardware Setup
\imageStyle{SDFM_LPHw_SETUP_FOR_PHASE_DELAY.jpeg,width:40%}
\image html SDFM_LPHw_SETUP_FOR_PHASE_DELAY.jpeg "LP Hardware setup"
\image html SDFM_LP_HWSETUP_FOR_PHASE_DELAY.png "SDFM: LP setup view"
\endcond
## Build, load and run

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using
  make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>

### Test Case Description
<table>
<tr>
        <th>Test Details
        <th>Steps
        <th>Pass/Fail Criteria
</tr>
<tr>
        <td>1. To check Phase Compensation</td>
        <td>1. Run the example on supported board </td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>2. Probe ch0 SD0_D pin and SD8_CLK pin </td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>3. Build and run example</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>4. Take timestamp of any rising edge of SD0_D pin and upcoming active SD8_CLK edge</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>5. Compare this time with measured delay (stored in DMEM at offset 0x18)</td>
        <td>Both values should be the same or have a maximum variation of 1 PRU cycle</td>
</tr>
</table>
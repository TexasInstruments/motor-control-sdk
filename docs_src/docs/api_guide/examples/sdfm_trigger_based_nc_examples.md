#  Basic ICSS %SDFM Examples {#BASIC_SDFM_EXAMPLES}

[TOC]

This example does trigger based normal current sampling. Normal current Over-samping Ratio (OSR), Over current OSR and Normal current trigger time can be configured by the user. There are two different examples based on number of %SDFM channels.

# Three Channels

Only one core - PRU is used for this example.

The example does the below:
- Set %SDFM channels: Channel 0 - Channel 2
- Configure normal current sample trigger time (time for read sample) and OSR

# Nine Channels

Load share mode of PRU-ICSSG is enabled for this example and three cores - RTU-PRU, PRU and TX-PRU are used for this example.

The example does the below
 - Enable load share mode
 - Set %SDFM channels: Channel 0 - Channel 8
 - Configure normal current sample trigger time (time for read sample) and OSR


# Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr>
    <td> ${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_nine_channel_load_share_mode</td>
    <td> Application specific sources for ICSS %SDFM for trigger based normal current sampling for nine channels </td>
</tr>
<tr>
    <td> ${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_three_channel_single_pru_mode</td>
    <td> Application specific sources for ICSS %SDFM for trigger based normal current sampling for three channels </td>
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
 PRU             | PRU0 (single channel)
 ^               | PRU0, RTU-PRU0, TXPRU0 (multi channel using three PRUs - load share mode)
 Toolchain       | ti-arm-clang
 Board           | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Examples folder | examples/current_sense

\endcond

# Steps to Run the Example

## Hardware Prerequisites
Other than the basic EVM setup mentioned in <a href="@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html" target="_blank"> EVM Setup </a>, below additional hardware is required to run this demo
- <a href="../TIDEP-01015Rev E1.1(001)_Sch.pdf" target="_blank"> TIDEP-01015 3 Axis Board </a>
- <a href="../MS_TI_AM64x_EVM_3-AXIS_INTERFACE_BOARD_SCH_REV_E1.pdf" target="_blank"> Interface card connecting EVM and TIDEP-01015 3 Axis </a>
- Signal generator

\note For more design details of the TIDEP-01015 3 Axis Board, or Interface card connecting EVM and TIDEP-01015 3 Axis, please contact TI via E2E/FAE.

### Hardware Setup
\image html SDFM_HwSetup_image.PNG  "Hardware Setup SDFM"
\image html SDFM_EVM_HW_setup.png  "SDFM: EVM and 3axis board setup view"
\cond SOC_AM243X
### Hardware Prerequisities for LP
- AMC1035EVM
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> AM243x-LP Board </a>
- Signal generator

#### LP Hardware Setup
\image html SDFM_LpHwSetup_image.png  "LP Hardware setup"
\image html SDFM_LpHwSetup.png  "SDFM: LP setup view"
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
        <td>1. Normal current sample data</td>
        <td>1. Run example on supported board</td>
        <td>The drawn graph and raw data should look like the attached image</td>
</tr>
<tr>
        <td></td>
        <td>2. Draw the graph of sdfm_ch0_samples, sdfm_ch1_samples and sdfm_ch2_samples arrays</td>
        <td>\image html SDFM_sample_output.PNG "NC sample data"</td>
 </tr>

<tr>
        <td>2. To check raw data for Single Update (64 Normal Current (NC) OSR)</td>
        <td>1. Set NC OSR to 64</td>
        <td>The drawn graph and raw data should look like the attached image</td>
</tr>
<tr>
        <td></td>
        <td>2. Set single update trigger time to half of EPWM cycle time </td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>3. Disable double update</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>3. Build and run example </td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>4. Draw graph for Raw data</td>
        <td>\image html SDFM_Single_update_64OSR.PNG "Single Update Raw data"</td>
</tr>

<tr>
        <td>3. To check Raw data for Double Update</td>
        <td>1. Set NC OSR to 64</td>
        <td>The drawn graphs and raw data should look like attached image</td>
</tr>
<tr>
        <td></td>
        <td>2. Enable double update</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>3. Set single update trigger time to 1/4 of EPWM cycle time</td>
        <td></td>
 </tr>
<tr>
        <td></td>
        <td>4. Set double update trigger time to 3/4 of EPWM cycle time</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>5. Build and run example</td>
        <td></td>
 </tr>
<tr>
        <td></td>
        <td>6. Draw graph for Raw data</td>
        <td>\image html SDFM_Double_update_64OSR.PNG "Double Update Raw data"</td>
</tr>
<tr>
        <td></td>
        <td></td>
        <td>The pattern of the graph should be different from the single update graph. It takes 2 samples in one EPWM cycle so the graph pattern should look more like a sine wave compare to single update graph</td>
</tr>

<tr>
        <td>4. To check Threshold comparator and Over current</td>
        <td>1. Set High Threshold to 3500 and low threshold to 2500</td>
        <td> Trip status bit must be set for the respective pwm trip zone block and TZ_OUT pin must be high</td>
</tr>
<tr>
        <td></td>
        <td>2. Set Over current OSR to 16</td>
        <td>High Low Threshold status bits must be constantly unset and set</td>
</tr>
<tr>
        <td></td>
        <td>3. Probe PWMm_TZ_OUT pin </td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>4. Build and run example</td>
        <td></td>
 </tr>
<tr>
        <td></td>
        <td>5. Capture signal in Logic analyzer</td>
        <td> </td>
 </tr>
<tr>
        <td>5. To check NC Samples with Different NC OSR Values</td>
        <td>1. Set NC OSR values between 16 to 255 </td>
        <td>Raw data should have different resolution for different OSR values </td>
</tr>
<tr>
        <td></td>
        <td>2. Build and run example</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>3. Observe resolution of raw data</td>
        <td></td>
</tr>
<tr>
        <td>6. To check NC samples with different sdfm clock values</td>
        <td>1. Set NC OSR to 64</td>
        <td> Raw data should have different resolution for different sdfm clock values  </td>
</tr>
<tr>
        <td></td>
        <td>2. Set ecap_divider variable in sdfm.c file for different sd clock generation</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>3. Set Sigma delta clock equal to ecap generated clock</td>
        <td></td>
 </tr>
<tr>
        <td></td>
        <td>4. Build and run example</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>5. Observe resolution of raw data</td>
        <td></td>
</tr>
<tr>
        <td>7. To check Fast detect</td>
        <td>1. Set NC OSR to 64</td>
        <td> Trip must be triggered for the respective pwm trip zone block </td>
</tr>
<tr>
        <td></td>
        <td>2. Enable Fast detect</td>
        <td></td>
 </tr>
<tr>
        <td></td>
        <td>3. Set Fast Detect fields with these values { window size = 4, Zero max = 18, Zero min = 2}</td>
        <td>Zero max/min Threshold hit bits must be constantly unset and set </td>
 </tr>

<tr>
        <td></td>
        <td>4. Build and run example</td>
        <td>One max/min threshold hit bits must be unset</td>
</tr>
<tr>
        <td></td>
        <td>5. 1) Observe TZ_OUT PIN.
               2) Check zero/one count max & zero/one count min threshold hit bits in memory map</td>
        <td></td>
</tr>
<tr>
        <td>8. To check Zero Cross</td>
        <td>1. Set Overcurrent (OC) OSR to 16</td>
        <td> Logic analyzer capture should match with this capture </td>
</tr>
<tr>
        <td></td>
        <td>2. Enable Zero cross detection</td>
        <td></td>
 </tr>
<tr>
        <td></td>
        <td>3. Set zero cross threshold vales to 1700 {value should be between max sampled value and min sampled value for 16 OSR}</td>
        <td>\image html SDFM_Zero_cross_GPIO_output.png "Zero cross GPIO behaviour" </td>
 </tr>
<tr>
        <td></td>
        <td>4. probe ch0 zero cross GPIO pins and input SD analog signal</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>5. Build and run example</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>6. Capture signals in logic analyzer</td>
        <td></td>
</tr>
\cond SOC_AM243X
<tr>
        <td>9.Testing with sdfm clock from EPWM </td>
        <td>1. Make hardware set up like attached image </td>
        <td>All test cases results should match with ECAP test case results</td>
 </tr>
<tr>
        <td></td>
        <td>2. \image html SDFM_EPWM1_HW_Setup.png "SDFM: Hw set for clock from EPWM"</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>3. Enable "APP_EPWM1_ENABLE" macro in app_sdfm.c file</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>4. Set EPWM1 out put frequency to 12.5MHz or 5MHz in app_sdfm.c file</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>5. Set Sigma delta clock equal to EPWM1 output frequency</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>6. Build and run example</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>7. Test all tese cases from 1 to 5 with EPWM clock</td>
        <td></td>
 </tr>
 \endcond
</table>
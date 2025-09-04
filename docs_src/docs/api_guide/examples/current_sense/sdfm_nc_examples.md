#  Basic ICSS %SDFM Examples {#BASIC_SDFM_EXAMPLES}

[TOC]

These examples demonstrate both trigger-based and continuous normal current sampling. The examples support Normal Current, Over Current, and Fast Detect configurations.

There are four different examples based on the number of %SDFM channels and the mode of Normal Current sampling.

> **Note:** Normal Current trigger mode examples do not support Over Current. 

## Three Channels

Only one core, PRU, is used for these examples.

1. **Continuous Mode Example**  
        - Continuous Normal Current sampling.  
        - Three channels: Channel 0, Channel 1, and Channel 2.  
        - Each channel has an individual interrupt to trigger an R5 event.  
        - ICSS PWM trip-based Fast Detect. ICSS PWM0 instance is used to generate the PWM trip.  
        - ICSS PWM trip-based Over Current detection. ICSS PWM0 instance is used to generate the PWM trip.  
                - **Note:** Over Current OSR should be equal to Normal Current OSR.  

2. **Trigger Mode Example**  
        - Three channels: Channel 0, Channel 1, and Channel 2.  
        - Trigger-based Normal Current sampling synchronized with EPWM.  
        - A common interrupt is used for all three channels.  
        - ICSS PWM trip-based Fast Detect. ICSS PWM0 instance is used to generate the PWM trip.  

## Nine Channels

The load share mode of PRU-ICSSG is enabled for these examples. Three cores—RTU-PRU, PRU, and TX-PRU—are used.

\note Channels 6 to 8 Fast Detect is not mapped with any ICSS PWM trip zone block. This is a hardware limitation. A software-based solution can be used as described in \ref OC_FD_TRIP.

1. **Continuous Mode Example**  
        - Continuous Normal Current sampling.  
        - Nine channels with load share mode.  
        - ICSS PWM trip-based Fast Detect for channels 0 to 5.  
        - ICSS PWM trip-based Over Current detection for all nine channels.  
                - **Note:** Over Current OSR should be equal to Normal Current OSR.  
        - Each channel has an individual interrupt.  
                - **Note:** Due to the unavailability of host interrupts, only the Channel 0 interrupt is configured for the TX PRU (Channels 6 to 8) in the application.  

2. **Trigger Mode Example**  
        - Trigger-based Normal Current sampling synchronized with EPWM.  
        - A common interrupt is used for every three channels.  
        - Nine channels with load share mode.  
        - ICSS PWM trip-based Fast Detect for channels 0 to 5.  

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
    <td> ${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_nine_channel_with_continuous_mode</td>
    <td> Application specific sources for ICSS %SDFM for continuous normal current sampling for nine channels </td>
</tr>
<tr>
    <td> ${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_three_channel_with_continuous_mode</td>
    <td> Application specific sources for ICSS %SDFM for continuous normal current sampling for three channels </td>
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

## Hardware Prerequisites for EVM
Other than the basic EVM setup mentioned in <a href="@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html" target="_blank"> EVM Setup </a>, the additional hardware listed below is required to run this demo
- <a href="../TIDEP-01015RevE1.1(001)_Sch.pdf" target="_blank"> TIDEP-01015 3 Axis Board </a>
- <a href="../MS_TI_EVM_3-AXIS_INTERFACE_BOARD_SCH_REV_E1.pdf" target="_blank"> Interface card connecting EVM and TIDEP-01015 3 Axis </a>
- Signal generator

\note For more design details of the TIDEP-01015 3 Axis Board, or Interface card connecting EVM and TIDEP-01015 3 Axis, please contact TI via E2E/FAE.

### EVM Hardware Setup
\image html SDFM_HwSetup_image.PNG  "Hardware Setup SDFM"
\image html SDFM_EVM_HW_setup.png  "SDFM: EVM and 3axis board setup view"
\cond SOC_AM243X
## Hardware Prerequisites for LP
- AMC1035EVM
- <a href="https://www.ti.com/tool/LP-AM243" target="_blank"> LP-AM243 Board </a>
- Signal generator

### LP Hardware Setup
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
        <td>1. Normal current sample data for trigger mode </td>
        <td>1. Run example on supported board</td>
        <td>The drawn graph and raw data should look like the attached image</td>
</tr>
<tr>
        <td></td>
        <td>2. Draw the graph of sdfm_ch_samples array</td>
        <td>\image html SDFM_sample_output.PNG "NC sample data"</td>
 </tr>

 <tr>
        <td>2. Normal current sample data for continuous mode</td>
        <td>1. Run example on supported board</td>
        <td>The drawn graph and raw data should look like the attached image</td>
</tr>
<tr>
        <td></td>
        <td>2. Draw the graph of raw the graph of sdfm_ch_samples array</td>
        <td>\image html SDFM_Continuous_mode_sample.PNG "NC sample data"</td>
 </tr>

<tr>
        <td>3. To check raw data for Single Update (64 Normal Current (NC) OSR)</td>
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
        <td>3. Build and run example </td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>4. Draw graph for Raw data</td>
        <td>\image html SDFM_Single_update_64OSR.PNG "Single Update Raw data"</td>
</tr>

<tr>
        <td>4. To check Raw data for Double Update</td>
        <td>1. Set NC OSR to 64</td>
        <td>The drawn graphs and raw data should look like the attached image</td>
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
        <td>The pattern of the graph should be different from the single update graph. It takes 2 samples in one EPWM cycle so the graph pattern should look more like a sine wave compared to the single update graph</td>
</tr>

<tr>
        <td>5. To test SINC1/SINC2/SINC3 filter</td>
        <td>1. Set NC OSR to 64 </td>
        <td> Raw data should have different resolution for different SINC filter </td>
</tr>
<tr>
        <td></td>
        <td>2. Set accumulator source to SINC1/SINC2/SINC3 </td>
        <td> SINC3 raw data resolution = OSR *(SINC2 raw data resolution) = OSR*OSR*(SINC1 raw data resolution)</td>
</tr>
<tr>
        <td></td>
        <td>3. Build and run example </td>
        <td></td>
</tr>


<tr>
        <td>6. To check Threshold comparator and Over current for continuous mode example </td>
        <td>1. Enable Comparator filter  </td>
        <td> Trip status bit must be set for the respective pwm trip zone block and TZ_OUT pin must be high</td>
</tr>
<tr>
        <td></td>
        <td>2. Set High Threshold to 3500 and low threshold to 2500 (low and high threshold values should be configured based on raw data resolution for 16 OSR) </td>
        <td></td>
</tr>

<tr>
        <td></td>
        <td>3. Set Over current and Normal current OSR to 16</td>
        <td>High Low Threshold status bits must be constantly unset and set</td>
</tr>
<tr>
        <td></td>
        <td>4. Probe PWMm_TZ_OUT pin </td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>5. Build and run example</td>
        <td></td>
 </tr>
<tr>
        <td></td>
        <td>6. Capture signal in Logic analyzer</td>
        <td> Trip must be triggered for the respective pwm trip zone block</td>
 </tr>

<tr>
        <td>7. To check NC Samples with Different NC OSR Values</td>
        <td>1. Set NC OSR values between 8 to 256 </td>
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
        <td>8. To check NC samples with different sdfm clock values</td>
        <td>1. Set NC OSR to 64</td>
        <td> Raw data range should not exceed the OSR limits   </td>
</tr>
<tr>
        <td></td>
        <td>2. Set ecap_divider variable in sdfm_example.c file for different sd clock generation</td>
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
        <td>9. To check Fast detect</td>
        <td>1. Set NC OSR to 64</td>
        <td> Trip must be triggered for the respective pwm trip zone block </td>
</tr>
<tr>
        <td></td>
        <td>2. Enable Fast detect and disable Comparator </td>
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

\cond SOC_AM243X

<tr>
        <td>10.Testing with sdfm clock from EPWM </td>
        <td>1. Make hardware setup like attached image </td>
        <td>All test case results should match with ECAP test case results</td>
 </tr>
<tr>
        <td></td>
        <td>2. \image html SDFM_EPWM1_HW_Setup.png "SDFM: HW set for clock from EPWM"</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>3. Enable "APP_EPWM1_ENABLE" macro in app_sdfm.c file</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>4. Set EPWM1 output frequency to 12.5MHz or 5MHz in app_sdfm.c file</td>
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
        <td>7. Test all test cases from 1 to 5 with EPWM clock</td>
        <td></td>
 </tr>
 \endcond

 <tr>
        <td>11. To test EPWM Synchronization source</td>
        <td>1. Set synchronization source to EPWM3 </td>
        <td> All test cases should work properly </td>
</tr>
<tr>
        <td></td>
        <td>2. Set NC OSR to 64</td>
        <td>The drawn graph and raw data should look like the attached image</td>
</tr>
<tr>
        <td></td>
        <td>3. Set single update trigger time to half of EPWM cycle time </td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>4. Add and configure EPWM3 instance in SysConfig</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>5. Build and run example </td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>6. Draw graph for Raw data</td>
        <td>\image html SDFM_Single_update_64OSR.PNG "Single Update Raw data"</td>
</tr>

<tr>
        <td>12. To test %SDFM clock generation from eCAP</td>
        <td>1. Set SDCLK Generation From to eCAP </td>
        <td> </td>
</tr>
<tr>
        <td></td>
        <td>2. %SDFM Clock to 20MHz </td>
        <td> Generated clock should come out on PRGx_ECAP0_IN_APWM_OUT signal </td>
</tr>
<tr>
        <td></td>
        <td>3. Configure PRGx_ECAP0_IN_APWM_OUT pin inside PRU ICSSG ECAP SysConfig module </td>
        <td> </td>
</tr>
<tr>
        <td></td>
        <td>4. Build and run example </td>
        <td></td>
</tr>

<tr>
        <td>13. To test %SDFM clock generation from ICSSG PRU GPO1</td>
        <td>1. Set `SDCLK Generation From` to PRU-ICSSG (PRG<k>_PRU1/0_GPI1) </td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>2. %SDFM Clock to 20MHz </td>
        <td> Generated clock should come out on PRG<k>_PRU1/0_GPI1 signal </td>
</tr>
<tr>
        <td></td>
        <td>3. Build and run example </td>
        <td></td>
</tr>

<tr>
        <td>14. To test %SDFM clock generation from IEP</td>
        <td>1. Set `SDCLK Generation From` to IEP </td>
        <td> </td>
</tr>
<tr>
        <td></td>
        <td>2. %SDFM Clock to 20MHz </td>
        <td> Generated clock should come out on two pins corresponding to SYNC0 and SYNC1</td>
</tr>
<tr>
        <td></td>
        <td>3. Configure SYNC_OUT0 and SYNC_OUT0 pins inside PRU ICSSG IEP SysConfig module </td>
        <td> </td>
</tr>
<tr>
        <td></td>
        <td>4. Build and run example </td>
        <td></td>
</tr>
</table>
#  %SDFM {#EXAMPLE_MOTORCONTROL_SDFM}

[TOC]



The ICSS %SDFM driver provides a well defined set of APIs to expose sigma delta interface.

The ICSS %SDFM example invokes these APIs to
- Set %SDFM channels
- Set ACC source, NC OSR, OC OSR, Clock source & Clock inversion
- Enable & disable threshold comparators
- Set high and low threshold values
- configure normal current sample trigger time (time for read sample)
- Enable & disable double update
- Inform firmware to enable %SDFM mode
- Configure GPIO pins for high and low threshold


Once these steps are executed
- ICSS %SDFM example waits for a interrupt (trigger by %SDFM firmware) to read sample data
- when interrupt occurs, example reads sample data from DMEM and again comes back to waiting loop

### ICSS SDFM Example Implementation
Following section describes the Example implementation of ICSS %SDFM on ARM(R5F).
\image html SDFM_EXAMPLE_FLOWCHART.png "ICSS SDFM Example"

## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm</td></tr>
<tr>
    <td>app_sdfm.c & sdfm.c</td>
    <td>ICSS %SDFM application</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/current_sense/sdfm</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing %SDFM firmware sources</td>
</tr>
<tr>
    <td>driver/</td>
    <td>ICSS %SDFM driver</td>
</tr>
<tr>
    <td>include/</td>
    <td>Folder containing ICSS %SDFM structures & APIs sources</td>
</tr>
</table>


# Supported Combinations {#EXAMPLES_MOTORCONTROL_SDFM_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU0
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/current_sense/icss_sdfm

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU0
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/current_sense/icss_sdfm

\endcond

# Steps to Run the Example

## Hardware Prerequisites
Other than the basic EVM setup mentioned in <a href="@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html" target="_blank"> EVM Setup </a>, below additional HW is required to run this demo
- TIDEP-01015 3 Axis board
- Interface card connecting EVM and TIDEP-01015 3 Axis board
- Signal generator


### Hardware Setup
\image html SDFM_HwSetup_image.PNG  "Hardware Setup SDFM"
\image html SDFM_EVM_HW_setup.png  "SDFM: EVM and 3axis board setup view"
\cond SOC_AM243X
### Hardware Prerequisities for LP
- AMC1035EVM 
- AM243x-LP board
- Signal generator

#### LP Hardware set up
\image html SDFM_LpHwSetup_image.png  "LP Hardware setup"
\image html SDFM_LpHwSetup.png  "SDFM: LP setup view"
\endcond
## Build, load and run

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using
  make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>
- Refer to UART terminal for user interface menu options.



### Test Case Description
<table>
<tr>
        <th>Test detail
        <th>Steps
        <th>Pass/fail crieteria
</tr>
<tr>
        <td>1. Normal current sample data</td>
        <td>1. Run icss sdfm example on am64x/am243x board</td>
        <td>he drawn graph and raw data should look like the attached image</td>
</tr>
<tr>
        <td></td>
        <td>2. Draw the graph of sdfm_ch0_samples, sdfm_ch1_samples and sdfm_ch2_samples arrays</td>
        <td>\image html SDFM_sample_output.PNG "NC sample data"</td>
 </tr>

<tr>
        <td>2. To check raw data for Single Update (64 NC OSR)</td>
        <td>1. Set NC OSR to 64</td>
        <td>The drawn graph and raw data should look like the attached image</td>
</tr>
<tr>
        <td></td>
        <td>2. Set single update trigger time to half of epwm cycle time </td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>3. Disable double update</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>3. Build and run icss sdfm example </td>
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
        <td>drawn Graphs and raw data should look like attached image</td>
</tr>
<tr>
        <td></td>
        <td>2. Enable double update</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>3. Set single update trigger time to 1/4 of epwm cycle time</td>
        <td></td>
 </tr>
<tr>
        <td></td>
        <td>4. Set double update trigger time to 3/4 of epwm cycle time</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>5. Build and run icss sdfm example</td>
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
        <td>Logic analyzer capture for High & Low Thresholds  </td>
</tr>
<tr>
        <td></td>
        <td>2. Set Over current OSR to 32</td>
        <td>\image html SDFM_threshold_comparator_salea_capture.png  "Logic analyzer Capture"</td>
</tr>
<tr>
        <td></td>
        <td>3. Probe Ch0 high, low threshold GPIO pins & input signal </td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>4. Build and run icss sdfm example</td>
        <td></td>
 </tr>
<tr>
        <td></td>
        <td>5. Capture signal in Logic analyzer</td>
        <td></td>
 </tr>
<tr>
        <td>5. To check NC Samples with Different NC OSR Values</td>
        <td>1. Set NC OSR values between 16 to 255 </td>
        <td>Raw data should have different resolution for different OSR values </td>
</tr>
<tr>
        <td></td>
        <td>2. Build and run icss sdfm example</td>
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
\cond SOC_AM243X
<tr>
        <td>7.Testing with sdfm clock from EPWM </td>
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
        <td>6. Build and run icss sdfm example</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>7. Test all tese cases from 1 to 5 with EPWM clock</td>
        <td></td>
 </tr>
 \endcond
</table>
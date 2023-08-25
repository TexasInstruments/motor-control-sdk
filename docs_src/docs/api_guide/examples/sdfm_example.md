#  SDFM {#EXAMPLE_MOTORCONTROL_SDFM}

[TOC]



The ICSS SDFM driver provides a well defined set of APIs to expose sigma delta interface.

The ICSS SDFM example invokes these APIs to
- Set SDFM channels
- Set ACC source, NC OSR, OC OSR, Clock source & Clock inversion
- Enable & disable threshold comparators
- Set high, low & zero cross threshold values
- configure sample trigger time (time for read sample)
- Inform firmware to enable SDFM mode
- Configure GPIO pins for high, low & zero cross thresholds


Once these steps are executed
- ICSS SDFM example waits for a interrupt (trigger by SDFM firmware) to read sample data
- when interrupt occurs, example reads sample data from DMEM and again comes back to waiting loop

### ICSS SDFM Example Implementation
Following section describes the Example implementation of ICSS SDFM on ARM(R5F).
\image html SDFM_EXAMPLE_FLOWCHART.png "ICSS SDFM Example"

## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/motor_control/icss_sdfm</td></tr>
<tr>
    <td>app_sddf.c & sddf.c</td>
    <td>ICSS SDFM application</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/current_sense/sdfm</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing SDFM firmware sources.</td>
</tr>
<tr>
    <td>driver/</td>
    <td>ICSS SDFM driver.</td>
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
 Example folder | examples/motorcontrol/sddf

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU0
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/motorcontrol/sddf

\endcond

# Steps to Run the Example

## Hardware Prerequisites
Other than the basic EVM setup mentioned in \ref EVM_SETUP_PAGE, below additional HW is required to run this demo
- TIDEP-01015 3 Axis board
- Interface card connecting EVM and TIDEP-01015 3 Axis board
- Signal generator

## Hardware Setup
\image html SDFM_HwSetup_image.PNG  "Hardware Setup SDFM"

## Build, load and run

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Refer to UART terminal for user interface menu options.



### Test Case Description
<table>
<tr>
        <th>Test detail
        <th>Steps
        <th>Pass/fail crieteria
</tr>
<tr>
        <td>To check raw data for 64 NC OSR</td>
        <td>1. Set NC OSR to 64</td>
        <td>Resolution of sampled data should be 13.9 bits </td>
</tr>
<tr>
        <td></td>
        <td>2. Set data read time half of epwm cycle</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>3. Set epwm output frequency 20Khz</td>
        <td></td>
</tr>
<tr>
        <td></td>
        <td>4. Draw graph for Raw data</td>
        <td>\image html SDFM_sample_output.PNG "NC sample data"</td>
</tr>
<tr>
        <td>To check raw data for 32 NC OSR</td>
        <td>1. Set NC OSR to 32</td>
        <td>Resolution of sampled data should be 11.4 bits </td>
</tr>
<tr>
        <td></td>
        <td>2. Set data read time half of epwm cycle</td>
        <td></td>
 </tr>
 <tr>
        <td></td>
        <td>3. Set epwm output frequency 20Khz</td>
        <td></td>
</tr>
<tr>
        <td>To check Threshold comparator</td>
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
        <td>4. Capture signal in Logic analyzer</td>
        <td></td>
 </tr>
</table>
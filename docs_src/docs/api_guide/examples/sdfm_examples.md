# ICSS %SDFM Examples {#EXAMPLES_MOTORCONTROL_SDFM}

[TOC]

This page lists all the examples of ICSSG %SDFM availble in this SDK. Following sections describe the features available in each of the examples.

The ICSS %SDFM driver provides a well defined set of APIs to expose sigma delta interface.

The ICSS %SDFM examples invoke these APIs to
- Set %SDFM channels
- Set Accumulator (ACC) source, Normal Current (NC) Over-samping Ratio (OSR), Over-current (OC) OSR, Clock source and Clock inversion
- Enable/disable threshold comparators
- Set high and low threshold values
- Enable Zero Cross & set Zero cross threshold value
- configure normal current sample trigger time (time for read sample)
- Enable & disable double update
- Inform firmware to enable %SDFM mode
- Configure and fast detect block
- Enable PRU load share mode
- Enable Phase Compensation

Once these steps are executed
- ICSS %SDFM example waits for a interrupt (trigger by %SDFM firmware) to read sample data
- when interrupt occurs, example reads sample data from DMEM and again comes back to waiting loop

# ICSS SDFM Examples Implementation
Following section describes the flow of the examples.

\image html SDFM_EXAMPLE_FLOWCHART.png "ICSS SDFM Examples"

# Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr>
    <td>${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_nine_channel_with_continuous_mode</td>
    <td> Application specific sources for ICSS %SDFM for continuous normal current sampling for nine channels </td>
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
    <td>${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_three_channel_with_continuous_mode</td>
    <td> Application specific sources for ICSS %SDFM for continuous normal current sampling for three channels </td>
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

# Supported Combinations {#EXAMPLES_MOTORCONTROL_SDFM_COMBOS}

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

# ICSS SDFM Examples Description

Following are different examples for ICSS %SDFM:

<table>
<tr>
        <th>Example
        <th>Enabled Features
        <th>Tested/Supported Features
</tr>
<tr>
        <td>\subpage BASIC_SDFM_EXAMPLES </td>
        <td>Trigger based Normal current, Over current detection</td>
        <td>Zero cross detection, Fast detect, Double Update</td>
 </tr>
 <tr>
        <td>\subpage BASIC_SDFM_EXAMPLES_WITH_CONTINUOUS_NC</td>
        <td>Continuous normal current sampling</td>
        <td>Fast detect</td>
 </tr>
 <tr>
        <td>\subpage BASIC_SDFM_EXAMPLE_WITH_PHASE_DELAY</td>
        <td>Trigger based Normal current, Phase compensation, Over current detection </td>
        <td>Fast detect, Double Update, Zero cross detection</td>

 </tr>
</table>


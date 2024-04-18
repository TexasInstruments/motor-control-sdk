# ICSS %SDFM Examples {#EXAMPLES_MOTORCONTROL_SDFM}

[TOC]

This page lists all the examples of ICSSG %SDFM available in this SDK. Following sections describe the features available in each of the examples.

The ICSS %SDFM driver provides a well defined set of APIs to expose sigma delta interface.

The ICSS %SDFM examples invoke these APIs to
- Set %SDFM channels
- Set Accumulator (ACC) source, Normal Current (NC) Over-sampling Ratio (OSR), Over-current (OC) OSR, Clock source and Clock inversion
- Enable/disable threshold comparators
- Set high and low threshold values
- Enable Zero Cross and set Zero cross threshold value
- configure normal current sample trigger time (time for read sample)
- Enable and disable double update
- Inform firmware to enable %SDFM mode
- Configure and fast detect block
- Enable PRU load share mode
- Enable Phase Compensation

Once these steps are executed
- ICSS %SDFM example waits for a interrupt (trigger by %SDFM firmware) to read sample data
- when interrupt occurs, example reads sample data from DMEM and again comes back to waiting loop

# SDFM SysConfig {#SDFM_SYSCFG}
%SDFM has SysConfig support to initialize %SDFM parameters and configure %SDFM pins.
\image html SDFM_syscfg_view.PNG "%SDFM SysConfig view"

SysConfig is used to configure things mentioned below:
- Selecting the ICSSG instance. (Tested on ICSSG0)
- Selecting the ICSSG PRU slice.(Tested on ICSSG0-PRU0)
- Configuring PINMUX.
- Channel selection.
- %SDFM Channel parameters initialization 
    \image html SDFM_Channel_params_config.PNG "SDFM Channel parameters SysConfig view" 
    - Clock parameters Configuration 
        \image html  SDFM_Channel_clk_params_configuration.PNG "SDFM Channel clk parameters SysConfig view"
    - Normal Current Configuration
        \image html  SDFM_Channel_NC_syscfg.PNG "SDFM Channel Normal Current parameters"  
    - Comparator configuration 
        \image html  SDFM_Channel_OC_params_syscfg.PNG "SDFM Channel Over Current parameters"  
    - Fast Detect Configuration 
        \image html SDFM_Channel_FD_params_syscfg.PNG "SDFM Channel Fast Detect parameters"  
    - SD modulator Settings 
        \image html  SDFM_Channel_SD_modulator_params_syscfg.PNG "SDFM Channel SD modulator parameters"  

## SDFM SysConfig Features
## SD Clock Options {#SDCLK_GEN_FROM}

ICSSG %SDFM driver supports three methods to generate clock which can be fed externally to modulator and SD_CLK pins. All %SDFM sdk examples are using %SDFM clock generated from eCAP, other two options has some trade-off as you will lose SD channel due to pin conflict.
- Clock form ICSSG PRU GPO1
- Clock from ICSSG eCAP 
- Clock from ICSSG IEP

Note: SysConfig only provides the clock selection option. Dividers have to be configured via API calls.  

### ICSSG PRU GPO1
The PRG<k>_PRU1/0_GPI1 signal (muxed with SD0_D) can be used as SD_CLKOUT when PRU ICSSG generates clock. This is a trade-off as PRU application will lose one SD channel. Based on dividers value API \ref SDFM_configClockFromGPO1 does configuration of PRU registers and enables pru to generate clock.
- Generated clock comes out on PRG<k>_PRU1/0_GPI1 signal that can be fed externally to modulator and SD_CLK pins 
- Dividers value settings for given PRU core clock and SD clock values 
 <table>
<tr>
    <th>SD clock (MHz)
    <th>PRU core clock (MHz)
	<th>First divider(div0)
    <th>Second divider(div1)
    <th>Description
</tr>
<tr>
    <td>20
    <td>300
	<td>15 (0x1C)
    <td>1 (0x00)
    <td>20 = 300/(15*1)
</tr>
<tr>
    <td>20
    <td>300
	<td>7.5 (0x0E)
    <td>2 (0x01)
    <td>20 = 300/(7.5*1)
</tr>
<tr>
    <td>10
    <td>300
	<td>15 (0x1C)
    <td>2 (0x01)
    <td>10 = 300/(15*2) 
</tr>
<tr>
    <td>5
    <td>200
	<td>10 (0x)
    <td>4 (0x01)
    <td>5 = 200/(10*4)
</tr>
</table> 

### ICSSG eCAP 
ICSSG eCAP can be used for %SDFM clock generation. Based on divider value API \ref SDFM_configEcap does configuration of eCAP registers and enables eCAP to generate output clock on PRGx_ECAP0_IN_APWM_OUT pin.
- Generated clock comes out on PRG<k>_PRU1/0_GPI1 signal that can be fed externally to modulator and SD_CLK pins 
- Divider value settings for given PRU core clock and SD clock values
<table>
<tr>
    <th>SD clock (MHz)
    <th>PRU core clock (MHz)
	<th>divider
</tr>
<tr>
    <td>20
    <td>300
	<td>15 
</tr>
<tr>
    <td>15
    <td>300
    <td>20 
</tr>
<tr>
    <td>10
    <td>200
	<td>20 
</tr>
</table> 


### ICSSG IEP
ICSSG IEP has Sync0/Sync1 cyclic generation mode to generate clock which can be used for %SDFM clock. The generated clock comes out on two pin corresponding SYNC0 and SYNC1. IEP also has SYNC1 delay feature which can be used to define delay between SYNC1 and SYNC0 output. 
-  APIs related to IEP SYNC0 and SYNC1 configuration 
<table>
<tr>
    <th>API
    <th>Description 
	<th>Parameters 
</tr>
<tr>
    <td> \ref SDFM_configIepSyncMode
    <td> Configures IEP SYNC0 and SYNC1 registers to generate free running clock
	<td> highPulseWidth: Number of clock cycles SYNC0/1 will be high, periodTime: Period between the rising edges of output clock, syncStartTime: starting time of free running clock
</tr>
<tr>
    <td> \ref SDFM_enableIep
    <td> Enables IEP timer
    <td>20 
</tr>
<tr>
    <td> \ref SDFM_configSync1Delay
    <td>  Defines clock cycles from the start of SYNC0 to the start of SYNC1
	<td>  delay: Delay before the start of SYNC1
</tr>
</table> 

-  \ref SDFM_configIepSyncMode params value configuration for given PRU core clock and SD clock values
<table>
<tr>
    <th>SD clock (MHz)
    <th>IEP clock (MHz)
	<th>`highPulseWidth` 
    <th>`periodTime`
    <th>`syncStartTime`
    <th>Description
</tr>
<tr>
    <td>20
    <td>300
	<td> 6  (7-1)
    <td> 14 (15-1)
    <td> Any unsigned integer value
    <td> IEP clock 300MHz, SD clk = 20Mhz, Div = 300/20 = 15, one period time = 15 IEP cycles, high plus time = 7 IEP cycles 
</tr>
<tr>
    <td>10
    <td>200
	<td> 9  (10-1)
    <td> 19 (20-1)
    <td> Any unsigned integer value
    <td> IEP clock 300MHz, SD clk = 10Mhz, Div = 200/10 = 20, one period time = 20 IEP cycles, high plus time = 10 IEP cycles (50% duty cycle)
</tr>
</table> 


It is a better clock option to handle phase compensation. SDK has an example `icss_sdfm_three_channel_with_phase_compensation` for showing phase compensation.

Note : There is pin conflict between %SDFM channel 8 data PIN and IEP0 SYNC_OUT1


## SDFM Channel Clock Configuration 
Three parameters needs to configure for each %SDFM channel.
- Clock source: Option to source clock on %sdfm channel
- Clock value: %SDFM input clock value
- Clock inversion: It is a board specific feature that is used to configure the accumulator input clock polarity. Enable this if the input %sdfm clock is reverse otherwise keep it inactive

## SDFM Channel SINC filter 
PRU ICSSG %SDFM interface supports three SINC filters to filter SD bit streams. 
- SINC1/SINC2/SINC3  
- Current %SDFM firmware uses common SINC filter for comparator as well as data filter.(Different sinc filters for data filter and comparator filter are not supported)
## SDFM Channel Normal Current 
Normal current is used for %SDFM data filter. It has features mentioned below
- OSR
- Trigger points: Sampling points in each EPWM cycle 
    - First sample point: One time sampling in each EPWM cycle 
    - Second sample point: If double update is enabled then %SDFM firmware does two times %SDFM sampling in each EPWM cycle
- Continuous mode: %SDFM firmware does continuous sampling of SD data once it started. 
- EPWM Synchronization: Synchronization between %SDFM and EPWM.

Note: 
    - All SDK examples have EPWM synchronization enabled
    - The current PRU firmware imposes certain restrictions, which effectively means that certain normal current features are specific to one axis, so they have to be same for all three SD channels of one axis.
        - Normal current OSR
        - Trigger points
        - EPWM source

## SDFM Channel Comparator
Over current is used for comparator filter to detect low threshold, high threshold and zero cross and generate PWM trips. It has features mentioned below
- OSR 
- High threshold
- Low threshold
- Zero Cross detection

## SDFM Channel Fast Detect
The Fast Detect is used for fast over current detection and trip generation. It has features mentioned below
- Fast Detect window size
- Zero count maximum limit in fast detect window
- Zero count minimum limit in fast detect window

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


# Current Sense Examples{#EXAMPLES_CURRENT_SENSE}
[TOC]

This page lists all the examples of ICSSG %SDFM available in this SDK.
-# \subpage BASIC_SDFM_EXAMPLES
-# \subpage BASIC_SDFM_EXAMPLES_WITH_SNOOP_MODE_NC
-# \subpage BASIC_SDFM_EXAMPLE_WITH_PHASE_DELAY

Following sections describe the features available in each of the examples.

The ICSS %SDFM driver provides a well-defined set of APIs to expose sigma delta interface.

The ICSS %SDFM examples invoke these APIs to:
- Set %SDFM channels
- Set accumulator (ACC) source, normal current (NC) over-sampling ratio (OSR), over-current (OC) OSR, clock source and clock inversion
- Enable/disable threshold comparators
- Set high and low threshold values
- Enable zero cross detection and set zero cross threshold value
- Configure normal current sample trigger time
- Enable and disable double update
- Enable and disable snoop based normal current sampling (if applicable)
- Inform firmware to enable %SDFM mode
- Configure the fast detect block
- Enable PRU load share mode
- Enable phase compensation

Once these steps are executed:
- ICSS %SDFM example waits for an interrupt (triggered by %SDFM firmware) to read sample data
- When interrupt occurs, example reads sample data from DMEM and again comes back to waiting loop

# SDFM SysConfig {#SDFM_SYSCFG}
%SDFM has SysConfig support to initialize %SDFM parameters and configure %SDFM pins.
\image html SDFM_syscfg_view.PNG "%SDFM SysConfig view"

SysConfig is used to configure things mentioned below:
- Selecting the ICSSG instance. (Tested on ICSSG0)
- Selecting the ICSSG PRU slice. (Tested on ICSSG0-PRU0)
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

ICSSG %SDFM driver supports three methods to generate clock which can be fed externally to modulator and SD_CLK pins. All %SDFM SDK examples are using %SDFM clock generated from eCAP, other two options have some trade-offs as you will lose SD channel due to pin conflict.
- Clock from ICSSG PRU GPO1
- Clock from ICSSG eCAP
- Clock from ICSSG IEP

Note: SysConfig only provides the clock selection option. Dividers have to be configured via API calls.

### ICSSG PRU GPO1
The PRG<k>_PRU1/0_GPI1 signal (muxed with SD0_D) can be used as SD_CLKOUT when PRU ICSSG generates clock. This is a trade-off as PRU application will lose one SD channel. Based on dividers value, API \ref SDFM_configClockFromGPO1 does configuration of PRU registers and enables PRU to generate clock.
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
	<td>10 (0x13)
    <td>4 (0x03)
    <td>5 = 200/(10*4)
</tr>
</table>

### ICSSG eCAP
ICSSG eCAP can be used for %SDFM clock generation. Based on divider value, API \ref SDFM_configEcap does configuration of eCAP registers and enables eCAP to generate output clock on PRGx_ECAP0_IN_APWM_OUT pin.
- Generated clock comes out on PRGx_ECAP0_IN_APWM_OUT signal that can be fed externally to modulator and SD_CLK pins
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
ICSSG IEP has Sync0/Sync1 cyclic generation mode to generate clock which can be used for %SDFM clock. The generated clock comes out on two pins corresponding to SYNC0 and SYNC1. IEP also has SYNC1 delay feature which can be used to define delay between SYNC1 and SYNC0 output.
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
    <td> None
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
    <td> IEP clock 300 MHz, SD clk = 20 MHz, Div = 300/20 = 15, one period time = 15 IEP cycles, high plus time = 7 IEP cycles
</tr>
<tr>
    <td>10
    <td>200
	<td> 9  (10-1)
    <td> 19 (20-1)
    <td> Any unsigned integer value
    <td> IEP clock 200 MHz, SD clk = 10 MHz, Div = 200/10 = 20, one period time = 20 IEP cycles, high pulse time = 10 IEP cycles (50% duty cycle)
</tr>
</table>


It is a better clock option to handle phase compensation. SDK has an example `icss_sdfm_three_channel_with_phase_compensation` for showing phase compensation.

Note: There is pin conflict between %SDFM channel 8 data PIN and IEP0 SYNC_OUT1


## SDFM Channel Clock Configuration
Three parameters need to be configured for each %SDFM channel.
- Clock source: Option to source clock on %SDFM channel
- Clock value: %SDFM input clock value
- Clock inversion: It is a board specific feature that is used to configure the accumulator input clock polarity. Enable this if the input %SDFM clock is reverse, otherwise keep it inactive

## SDFM Channel SINC filter
PRU ICSSG %SDFM interface supports three SINC filters to filter SD bit streams.
- SINC1/SINC2/SINC3
- Current %SDFM firmware uses common SINC filter for comparator as well as data filter. (Different sinc filters for data filter and comparator filter are not supported)
## SDFM Channel Normal Current
Normal current is used for %SDFM data filter. It has features mentioned below:
- OSR
- Trigger points: Sampling points in each EPWM cycle
    - First sample point: One time sampling in each EPWM cycle
    - Second sample point: If double update is enabled then %SDFM firmware does two times %SDFM sampling in each EPWM cycle
- Continuous mode: %SDFM firmware does continuous sampling of SD data once it started.
- EPWM Synchronization: Synchronization between %SDFM and EPWM.

Note:
    - All SDK examples have EPWM synchronization enabled
    - The current PRU firmware imposes certain restrictions, which effectively means that certain normal current features are specific to one axis, so they have to be the same for all three SD channels of one axis.
        - Normal current OSR
        - Trigger points
        - EPWM source

## SDFM Channel Comparator
Over current is used for comparator filter to detect low threshold, high threshold and zero cross and generate PWM trips. It has features mentioned below:
- OSR
- High threshold
- Low threshold
- Zero Cross detection

## SDFM Channel Fast Detect
The Fast Detect is used for fast over current detection and trip generation. It has features mentioned below:
- Fast Detect window size
- Zero count maximum limit in fast detect window
- Zero count minimum limit in fast detect window

## SDFM IEP CMP Event Configuration {#SDFM_IEP_CMP_CONFIG}
The Industrial Ethernet Peripheral (IEP) compare events are used to trigger %SDFM normal current sampling in trigger mode. %SDFM supports user-selectable IEP CMP events via SysConfig, providing flexibility for different system configurations.

### IEP CMP Event Selection via SysConfig
Users can select IEP compare events (CMP0-CMP15) and IEP instance via SysConfig to trigger %SDFM sampling. The selection depends on the system configuration and resource availability.

### CMP Event Conflict Warnings

\attention The following CMP events require special attention to avoid conflicts:

- **CMP0 - IEP Counter Reset**: CMP0 is used to reset the IEP counter to trigger periodic sampling when EPWM synchronization is not used. The cycle period of the IEP counter depends on the configured `IEP Reset Frequency (Hz)` in SysConfig.

- **CMP1 and CMP2 - SYNC0/SYNC1 Outputs**: When using IEP SYNC mode for %SDFM clock generation:
  - CMP1 is used to start clock on IEP SYNC0 output pin
  - CMP2 is used to start clock on IEP SYNC1 output pin
  - If your application uses IEP for clock generation, select CMP3-CMP15 for sampling triggers to avoid conflicts

### Default CMP Event Configurations for Trigger Mode

The table below shows the default IEP compare events used by each PRU core in trigger mode. These defaults are configured via SysConfig and can be changed as needed.

<table>
<tr>
    <th>Configuration</th>
    <th>Default CMP Events</th>
    <th>Notes</th>
</tr>
<tr>
    <td>Single PRU trigger mode</td>
    <td>CMP3</td>
    <td>Configurable via SysConfig under PRU core settings.</td>
</tr>
<tr>
    <td>Load-Share Mode (9 channels, 3 PRU cores)</td>
    <td>RTU: CMP4, PRU: CMP3, TXPRU: CMP5</td>
    <td>Each PRU core uses its dedicated CMP event. Configurable via SysConfig. SysConfig validates that different cores use different CMP events.</td>
</tr>
</table>

\note For detailed interrupt and event mappings per example, refer to the INTC Mapping table below.

## SDFM INTC Mapping {#SDFM_INTC_MAPPING}

The following table shows the PRU event to R5F host interrupt mapping for all SDFM examples:

<table>
  <tr>
    <th>Example</th>
    <th>PRU Core</th>
    <th>Channel(s)</th>
    <th>PRU Event Number</th>
    <th>PRU Event Name</th>
    <th>Host Interrupt</th>
    <th>IEP CMP Event</th>
  </tr>

  <!-- Nine Channel Single PRU Mode (Trigger) -->
  <tr>
    <td rowspan="1">${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_nine_channel_single_pru_mode</td>
    <td>PRU</td>
    <td>Ch0-Ch8</td>
    <td>21</td>
    <td>pr[0/1]_pru_mst_intr[5]_intr_req</td>
    <td>HOST_INTR_PEND_0</td>
    <td>CMP3 (default)</td>
  </tr>

  <!-- Three Channel Single PRU Continuous Mode -->
  <tr>
    <td rowspan="9">${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_three_channel_single_pru_continuous_mode</td>
    <td>PRU</td>
    <td>Ch0</td>
    <td>21</td>
    <td>pr[0/1]_pru_mst_intr[5]_intr_req</td>
    <td>HOST_INTR_PEND_0</td>
    <td>N/A (continuous)</td>
  </tr>
  <tr>
    <td>PRU</td>
    <td>Ch1</td>
    <td>22</td>
    <td>pr[0/1]_pru_mst_intr[6]_intr_req</td>
    <td>HOST_INTR_PEND_1</td>
    <td>N/A (continuous)</td>
  </tr>
  <tr>
    <td>PRU</td>
    <td>Ch2</td>
    <td>23</td>
    <td>pr[0/1]_pru_mst_intr[7]_intr_req</td>
    <td>HOST_INTR_PEND_2</td>
    <td>N/A (continuous)</td>
  </tr>
   <tr>
    <td>PRU</td>
    <td>Ch3</td>
    <td>24</td>
    <td>pr[0/1]_pru_mst_intr[8]_intr_req</td>
    <td>HOST_INTR_PEND_3</td>
    <td>N/A (continuous)</td>
  </tr>
  <tr>
    <td>PRU</td>
    <td>Ch4</td>
    <td>25</td>
    <td>pr[0/1]_pru_mst_intr[9]_intr_req</td>
    <td>HOST_INTR_PEND_4</td>
    <td>N/A (continuous)</td>
  </tr>
  <tr>
    <td>PRU</td>
    <td>Ch5</td>
    <td>26</td>
    <td>pr[0/1]_pru_mst_intr[10]_intr_req</td>
    <td>HOST_INTR_PEND_5</td>
    <td>N/A (continuous)</td>
  </tr>
   <tr>
    <td>PRU</td>
    <td>Ch6</td>
    <td>27</td>
    <td>pr[0/1]_pru_mst_intr[11]_intr_req</td>
    <td>HOST_INTR_PEND_6</td>
    <td>N/A (continuous)</td>
  </tr>
   <tr>
    <td>PRU</td>
    <td>Ch7</td>
    <td>28</td>
    <td>pr[0/1]_pru_mst_intr[12]_intr_req</td>
    <td>HOST_INTR_PEND_7</td>
    <td>N/A (continuous)</td>
  </tr>
   <tr>
    <td>PRU</td>
    <td>Ch8</td>
    <td>29</td>
    <td>pr[0/1]_pru_mst_intr[13]_intr_req</td>
    <td>HOST_INTR_PEND_0</td>
    <td>N/A (continuous)</td>
  </tr>

  <!-- Nine Channel Load Share Snoop Mode (Trigger) -->
  <tr>
    <td rowspan="3">${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_nine_channel_load_share_snoop_mode</td>
    <td>RTU-PRU</td>
    <td>Ch0-Ch2</td>
    <td>21</td>
    <td>pr[0/1]_pru_mst_intr[5]_intr_req</td>
    <td>HOST_INTR_PEND_0</td>
    <td>CMP4 (default)</td>
  </tr>
  <tr>
    <td>PRU</td>
    <td>Ch3-Ch5</td>
    <td>24</td>
    <td>pr[0/1]_pru_mst_intr[8]_intr_req</td>
    <td>HOST_INTR_PEND_3</td>
    <td>CMP3 (default)</td>
  </tr>
  <tr>
    <td>TX-PRU</td>
    <td>Ch6-Ch8</td>
    <td>27</td>
    <td>pr[0/1]_pru_mst_intr[11]_intr_req</td>
    <td>HOST_INTR_PEND_6</td>
    <td>CMP5 (default)</td>
  </tr>

  <!-- Nine Channel Load Share Continuous Mode (Common IRQ) -->
  <tr>
    <td rowspan="3">${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_nine_channel_load_share_continuous_mode<br/>(Common IRQ Mode)</td>
    <td>RTU-PRU</td>
    <td>Ch0-Ch2</td>
    <td>21, 22, 23</td>
    <td>pr[0/1]_pru_mst_intr[5,6,7]_intr_req</td>
    <td>HOST_INTR_PEND_0 (shared)</td>
    <td>N/A (continuous)</td>
  </tr>
  <tr>
    <td>PRU</td>
    <td>Ch3-Ch5</td>
    <td>24, 25, 26</td>
    <td>pr[0/1]_pru_mst_intr[8,9,10]_intr_req</td>
    <td>HOST_INTR_PEND_3 (shared)</td>
    <td>N/A (continuous)</td>
  </tr>
  <tr>
    <td>TX-PRU</td>
    <td>Ch6-Ch8</td>
    <td>27, 28, 29</td>
    <td>pr[0/1]_pru_mst_intr[11,12,13]_intr_req</td>
    <td>HOST_INTR_PEND_6 (shared)</td>
    <td>N/A (continuous)</td>
  </tr>

  <!-- Three Channel Single PRU Snoop Mode -->
  <tr>
    <td rowspan="1">${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_three_channel_single_pru_snoop_mode</td>
    <td>PRU</td>
    <td>Ch0-Ch2</td>
    <td>21</td>
    <td>pr[0/1]_pru_mst_intr[5]_intr_req</td>
    <td>HOST_INTR_PEND_0</td>
    <td>CMP3 </td>
  </tr>

  <!-- Three Channel with Phase Compensation -->
  <tr>
    <td rowspan="1">${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_three_channel_with_phase_compensation</td>
    <td>PRU</td>
    <td>Ch0-Ch2</td>
    <td>21</td>
    <td>pr[0/1]_pru_mst_intr[5]_intr_req</td>
    <td>HOST_INTR_PEND_0</td>
    <td>CMP3 </td>
  </tr>
</table>

**Notes:**
- PRU Event Numbers are defined in `icssg_sdfm.h` as ICSS_SDFM_TRIGGER_EVNT_CHx (range: 21-29)
- Common IRQ mode (load share continuous): All channels per PRU core map to one shared host interrupt
- IEP CMP events are configurable via SysConfig; defaults shown here

## Load Share Mode Common IRQ Handler {#SDFM_LOAD_SHARE_COMMON_IRQ}

The load share continuous mode examples support an optimized interrupt handling approach using common IRQ handlers per PRU core, controlled by the `SDFM_LOAD_SHARE_COMMON_IRQ_ENABLE` macro in `app_sdfm.c`.

### Common IRQ Mode (Default)
When enabled (`SDFM_LOAD_SHARE_COMMON_IRQ_ENABLE = 1`):
- Each PRU core uses one shared interrupt handler for all its channels:
  - RTU PRU: `sdfmCommonIrqHandlerRTU()` handles channels 0-2
  - PRU: `sdfmCommonIrqHandlerPRU()` handles channels 3-5
  - TX PRU: `sdfmCommonIrqHandlerTXPRU()` handles channels 6-8
- All three channels per PRU core map to the first channel's host event:
  - Channels 0-2: CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0
  - Channels 3-5: CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_3
  - Channels 6-8: CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_6


### Individual IRQ Mode 
When disabled (`SDFM_LOAD_SHARE_COMMON_IRQ_ENABLE = 0`):
- Each channel has its own dedicated interrupt handler (`sdfmIrqHandlerCh0` through `sdfmIrqHandlerCh8`)
- Requires individual SysConfig INTC mapping for each channel
- Uses more R5F interrupt resources

\note Due to host interrupt limitations, channel 8 shares HOST_INTR_PEND_0 with channel 0 in individual IRQ mode. If both channels are enabled together in continuous mode, remap channel 8 to an available host event via SysConfig to avoid conflicts.

\note Due to high interrupt overload when multiple channels are enabled and individual interrupts are used with low normal current OSR, there is a high risk of R5F overload. The R5F may not be able to process all interrupts in this scenario. For such use cases, common IRQ mode is suitable to avoid interrupt overload.  

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
    <td>${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_nine_channel_load_share_continuous_mode</td>
    <td>Application specific sources for ICSS %SDFM for continuous normal current sampling for nine channels using load share mode (3 PRU cores). Uses common IRQ mode.</td>
</tr>
<tr>
    <td> ${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_nine_channel_load_share_snoop_mode</td>
    <td>Application specific sources for ICSS %SDFM for trigger based normal current sampling for nine channels using load share mode and ICSS %SDFM snoop mode.</td>
</tr>
<tr>
    <td> ${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_nine_channel_single_pru_mode</td>
    <td>Application specific sources for ICSS %SDFM for trigger based normal current sampling for nine channels using single PRU core.</td>
</tr>
<tr>
    <td>${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_three_channel_single_pru_continuous_mode</td>
    <td><td>Application specific sources for ICSS %SDFM for continuous normal current sampling using single PRU core (supports up to nine channels). NOTE: It is not recommended to use more than three channels together when OSR is below 128 to avoid interrupt overload.</td>
</tr>
<tr>
    <td> ${SDK_INSTALL_PATH}/examples/current_sense/icss_sdfm_three_channel_single_pru_snoop_mode</td>
    <td> Application specific sources for ICSS %SDFM for trigger based normal current sampling for three channels using ICSS %SDFM snoop mode</td>
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

# ICSS SDFM Examples Description {#SDFM_EXAMPLES_DESCRIPTION}

Following are different examples for ICSS %SDFM:

<table>
<tr>
    <th>Example</th>
    <th>Tested/Supported Features</th>
    <th>Limitations</th>
</tr>
<tr>
    <td>\subpage BASIC_SDFM_EXAMPLES</td>
    <td>
        - Continuous normal current<br>
        - PRU-ICSS PWM trip-based over current<br>
        - Trigger-based normal current synchronized with EPWM<br>
        - PRU-ICSS PWM trip-based fast detect<br>
        - Double Update<br>
        - Up to 9-channel support using single PRU mode
        - Up to 9-channel support using load share mode <br>
        - OSR 8 - 256 <br>
        - SINC1, SINC2 and SINC3 filter
        - Supported clock sources:
            - Independent clock source for each channel
            - Shared clock source for three channels
            - Common clock source for all nine channels
    </td>
    <td>
        - OSR values must be identical for both normal current and over current<br>
        - Normal current trigger mode does not support over current<br>
        - Zero cross detection is not supported
    </td>
</tr>
<tr>
    <td>\subpage BASIC_SDFM_EXAMPLES_WITH_SNOOP_MODE_NC</td>
    <td>
        - Continuous Normal Current<br>
        - PRU-ICSS PWM trip-based over current<br>
        - Trigger-based normal current synchronized with EPWM<br>
        - PRU-ICSS PWM trip-based fast detect<br>
        - Double Update<br>
        - Up to 9-channel support using load share mode<br>
        - Zero Cross detection<br>
        - OSR 16 - 256 <br>
        - SINC1, SINC2 and SINC3 filter
        - Supported clock sources:
            - Shared clock source for three channels
            - Common clock source for all nine channels
    </td>
    <td>
        These examples use the IEP CMP event for %SDFM snoop mode. However, they may not be reliable in certain situations:<br>
        - When the SD clock is unstable with jitter and variations<br>
        - When the IEP gets reset during normal current sampling
        - Independent clock source for each channel
    </td>
</tr>
<tr>
    <td>\subpage BASIC_SDFM_EXAMPLE_WITH_PHASE_DELAY</td>
    <td>
        - Phase compensation for Channel0
    </td>
    <td>
        - This example is based on the %SDFM basic examples, which does not use the snoop mode.
    </td>
</tr>
</table>

# ICSS SDFM Debug Guide {#SDFM_EXAMPLES_DEBUG_GUIDE}
This section provides a comprehensive debugging guide for troubleshooting issues that may arise during %SDFM testing or development. Follow these steps to identify and resolve potential problems.

\note For initial debugging, use the `icss_sdfm_three_channel_single_pru_continuous_mode` example. This basic example has minimal dependencies and uses the internal eCAP clock source. Refer to \ref BASIC_SDFM_EXAMPLES for setup instructions. This example helps determine whether an issue is hardware or software related.

## SDFM Register Configuration
The PRU_ICSSG_CFG registers from offset 0x44 to 0xD8 are allocated for %SDFM configuration. To review and verify the %SDFM settings:

1. Halt the R5F core
2. Open the memory browser window
3. Enter the base address of the PRU_ICSSG_CFG registers and view the configured values

\note Ensure you are entering the correct address for %SDFM registers. Each ICSS instance has a different address for the CFG registers.

For detailed register descriptions, refer to section 6.4.14.5 PRU_ICSSG_CFG Registers in the AM243x Technical Reference Manual (TRM).

Key registers and their configurations:

1. **ICSSG_PRU0_SD_CFG_REG**
   - Load share mode configuration
   - Controls multi-PRU operation mode
2. **ICSSG_PRU0_SD_CLK_SEL_REG0**
   - Fast Detect Zero Count configuration
   - SINC filter type selection
   - Clock source selection
   - Clock inversion settings
3. **ICSSG_PRU0_SD_SAMPLE_SIZE_REG0**
   - Fast Detect Window Size configuration
   - Fast Detect One Count configuration
   - Over Sample Rate (OSR) configuration
     - When snoop mode is enabled: Used for overcurrent
     - When snoop mode is disabled: Used for normal current and overcurrent both
\image html SDFM_debug_cfg_registers_view.png "PRU-ICSS SDFM register view"

## IEP Registers Configuration
The Industrial Ethernet Peripheral (IEP) is critical for triggering normal current sampling. If the IEP is not configured correctly, normal current tasks will fail to execute.

To verify IEP configuration:
1. Check if the IEP is running by examining `IEP_COUNT_REG0/1` registers
2. Verify counter increment by monitoring count values
3. Review `IEP_CMP_CFG_REG` to ensure all compare events are properly configured for trigger mode
4. Check `IEP_CMP_STATUS_REG` to verify that corresponding compare events are setting status flags correctly
5. Validate that Compare Registers are configured with correct trigger point values

\note Ensure you are entering the correct address for IEP registers. Each ICSS instance has a different address for the IEP registers.

\image html SDFM_debug_IEP_registers_view1.png "PRU-ICSS IEP register view"
\image html SDFM_debug_IEP_registers_view2.png "PRU-ICSS IEP CMP events register view"

For detailed register descriptions, refer to section `6.4.14.9` PRU_IEP_IEP Registers in the AM243x TRM.

## Interrupt Controller Internal Signals Mapping
If you experience missing PRU interrupts or incorrect IRQ mapping, verify the interrupt mapping between PRU and R5F in the SysConfig PRU INTC module. The Host channel number and PRU Event should match your configuration.

For example, a basic %SDFM example uses:
- PRU Event: `21: pr[0/1]_pru_mst_intr[5]_intr_req`
- Host Interrupt: HOST_INTR_PEND_0 (Channel 0)

\image html SDFM_debug_INTC_module.png "PRU-ICSS INTC view"

The interrupt service routine (ISR) configuration is implemented in `app_sdfm.c`. Here's a key code snippet showing the configuration:

```c
/* R5F interrupt numbers for ICSSG SDFM - Continuous mode */
#define ICSSG_SDFM_HOST_INTR_NUM_CH0    (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
```

The PRU event numbers are defined in `icssg_sdfm.h`:
```c
#define ICSS_SDFM_TRIGGER_EVNT_CH0  (3+18)   /* PRU Event 21 */
/* ... continuing through CH8 (PRU Event 29) */
```

## PRU Debug
For PRU-related issues, verify that the application is loading the PRU firmware into the correct PRU core. If firmware loading fails:
1. Check the PRU core selection
2. Verify the PRU slice selection
3. Review application-level configuration

### Debugging Steps for PRU Core with Loaded Firmware:
1. Import the PRU firmware for the target core
2. Build the firmware
3. Open the debug window and connect to the PRU core
4. Load symbols into the connected core:
   - Click the Load button
   - Select `Load Symbols`
   - Load the firmware .out file from your project
   \image html SDFM_debug_firmware_load.png "Loading PRU firmware"

### PRU Execution Analysis
After loading the firmware, you can analyze the PRU execution flow:

- **Without Overcurrent Enabled:**
  - PRU waits for IEP compare event
  - Normal current execution starts after event trigger

- **With Overcurrent Enabled:**
  - PRU continuously executes overcurrent sampling

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

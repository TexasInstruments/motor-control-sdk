\cond SOC_AM243X || SOC_AM64X
# EnDat Encoder Migration Guide (v11.00.00 to v2025.00.00) {#ENDAT_MIGRATION_GUIDE_2025_00}
\endcond

\cond (SOC_AM263PX || SOC_AM261X || SOC_AM263X)
# EnDat Encoder Migration Guide (v10.02.00 to v2025.00.00) {#ENDAT_MIGRATION_GUIDE_2025_00}
\endcond

[TOC]

## Introduction

\cond SOC_AM243X || SOC_AM64X
This guide helps developers migrate EnDat encoder applications from Motor Control SDK v11.00.00 to v2025.00.00 and later versions. The driver underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes, and improved SysConfig integration.
\endcond
\cond (SOC_AM263PX || SOC_AM261X || SOC_AM263X)
This guide helps developers migrate EnDat encoder applications from Motor Control SDK v10.02.00 to v2025.00.00 and later versions. The driver underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes, and improved SysConfig integration.
\endcond

## Major Architectural Changes

### 1. Handle-Based API Architecture

The entire EnDat driver API was refactored from pointer-based to an improved handle-based architecture.

<table>
<tr>
    <th>Old API</th>
    <th>New API</th>
    <th>Impact</th>
</tr>
<tr>
    <td>struct \ref endat_priv *priv</td>
    <td>\ref endat_handle handle</td>
    <td>All APIs use opaque handle instead of direct structure pointer</td>
</tr>
</table>

### 2. SysConfig-Based Initialization

The initialization process was completely redesigned to use SysConfig-generated parameters for compile-time variables and endat_params structure for run-time variables.

<table>
<tr>
    <th>Old Initialization</th>
    <th>New Initialization</th>
</tr>
<tr>
    <td>
    <pre>
    struct endat_priv *priv = endat_init(
        pruDmemBaseAddress,     // PRU dmem base address
        &gEndatChInfo,          // Pointer of TCM
        gEndatChInfoGlobalAddr, // Global address of TCM
        pruicss_cfg,            // CFG base address
        pruicss_iep,            // IEP base address
        PRUICSS_SLICEx,         // PRU Slice
        &endat_clk_config       // Clock configuration
    );
    </pre>
    </td>
    <td>
    <pre>
    endat_params params;
    endat_params_init(&params);
    // Update params
    endat_handle handle = endat_init(CONFIG_ENDAT0, &params);
    </pre>
    </td>
</tr>
</table>

## API Changes

Driver APIs use following validation approach now:
- **Public API validation**: All public APIs validate the handle parameter for NULL and perform array bounds checking for index parameters
- **Internal function validation**: Internal static functions assume valid parameters. The caller is responsible for ensuring parameters are valid before calling internal functions
- **Internal structure validation**: Each API validates the internal structure pointers it accesses (e.g., attrs, priv, pruicss_xchg, pruicss_handle) for NULL before dereferencing
 - **Error state handling**: Error in \ref endat_recvd_process may leave internal state partially modified. Subsequent calls will overwrite these values. Caller is responsible for explicit state cleanup if needed.

### New APIs Added

<table>
<tr>
    <th>New API</th>
    <th>Description</th>
    <th>Usage</th>
</tr>
<tr>
    <td>endat_params_init()</td>
    <td>Initialize params structure with defaults</td>
    <td>Use before endat_init()</td>
</tr>
<tr>
    <td>endat_deinit()</td>
    <td>De-initialize EnDat interface</td>
    <td>Cleanup when done using driver</td>
</tr>
<tr>
    <td>endat_get_attrs()</td>
    <td>Get pointer to compile-time attributes</td>
    <td>Access SysConfig-generated configuration</td>
</tr>
<tr>
    <td>endat_get_priv()</td>
    <td>Get pointer to runtime private data</td>
    <td>Access runtime state information</td>
</tr>
<tr>
    <td>endat_config_propagation_delay()</td>
    <td>Configure propagation delay</td>
    <td>Set propagation delay for encoder communication</td>
</tr>
<tr>
    <td>endat_config_periodic_trigger_cmp_mode()</td>
    <td>Configure IEP CMP event based periodic triggering mode</td>
    <td>Replaces endat_config_periodic_trigger()</td>
</tr>
<tr>
    <td>endat_config_periodic_trigger_cap_mode()</td>
    <td>Configure IEP CAP event based periodic triggering mode</td>
    <td>-</td>
</tr>
<tr>
    <td>endat_config_iep_cmp_event()</td>
    <td>Configure IEP CMP event number in firmware DMEM</td>
    <td>Set CMP event (0-15) for each channel</td>
</tr>
<tr>
    <td>endat_config_iep_cap_event()</td>
    <td>Configure IEP CAP event number in firmware DMEM</td>
    <td>Set CAP event (0-7) for each channel</td>
</tr>
</table>

### APIs Modified

\note Most APIs that previously returned <code>void</code> now return <code>int32_t</code> status codes (<code>SystemP_SUCCESS</code>, <code>SystemP_FAILURE</code>, or <code>SystemP_TIMEOUT</code>) to enable proper error handling.

<table>
<tr>
    <th>API</th>
    <th>Key Changes</th>
    <th>Additional Details</th>
</tr>
<tr>
    <td>endat_init()</td>
    <td>- Complete signature change<br>- 7 parameters to 2 parameters<br>- Returns <code>endat_handle</code> instead of <code>priv*</code></td>
    <td>Uses SysConfig-generated index and params structure as arguments</td>
</tr>
<tr>
    <td>endat_command_build()<br>endat_command_process()<br>endat_addinfo_track()</td>
    <td>- <code>priv</code> to <code>handle</code><br>- <code>struct cmd_supplement</code> to <code>endat_cmd_supplement</code></td>
    <td>Type definition change for supplement parameter</td>
</tr>
<tr>
    <td>endat_recvd_process()<br>endat_recvd_validate()</td>
    <td>- <code>priv</code> to <code>handle</code><br>- <code>union endat_format_data</code> to <code>endat_format_data</code></td>
    <td>Type definition change for format data parameter</td>
</tr>
<tr>
    <td>endat_config_clock()</td>
    <td>- <code>priv</code> to <code>handle</code><br>- <code>struct endat_clk_cfg*</code> to <code>uint32_t freq</code><br>- <code>void</code> to <code>int32_t</code> return</td>
    <td>Simplified clock configuration with frequency value</td>
</tr>
<tr>
    <td>endat_config_rx_arm_cnt()<br>endat_config_wire_delay()<br>endat_config_rx_clock_disable()<br>endat_config_tst_delay()<br>endat_config_host_trigger()<br>endat_config_channel()<br>endat_config_multi_channel_mask()<br>endat_start_continuous_mode() <br>endat_multi_channel_set_cur()<br>endat_multi_channel_detected()<br>endat_stop_continuous_mode()<br>endat_wait_initialization()<br>endat_init_rt_measurement()<br>endat_enable_rt_measurement()<br>endat_disable_rt_measurement() <br>endat_get_encoder_info()<br>endat_command_send()<br>endat_command_wait() </td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>Returns status code</td>
</tr>
<tr>
    <td>endat_get_recovery_time()<br>endat_get_prop_delay()<br>endat_check_rt_error()<br>endat_status_rt_measurement()</td>
    <td>- Returns via output parameter<br>- <code>priv</code> to <code>handle</code><br>- Added output pointer param</td>
    <td>Returns status, provides value via pointer</td>
</tr>
</table>

### APIs Removed

<table>
<tr>
    <th>Removed API</th>
    <th>Replacement</th>
    <th>Migration Action</th>
</tr>
<tr>
    <td>endat_config_periodic_trigger()</td>
    <td>endat_config_periodic_trigger_cmp_mode() or endat_config_periodic_trigger_cap_mode()</td>
    <td>Choose appropriate mode based on trigger source</td>
</tr>
<tr>
    <td>endat_config_syn_bits()<br>endat_config_primary_core_mask()<br>endat_enable_load_share_mode()</td>
    <td>Made internal (static)</td>
    <td>No longer accessible from application</td>
</tr>
<tr>
    <td>endat_get_2_2_angle()</td>
    <td>None</td>
    <td>Removed from driver</td>
</tr>
</table>

## Structure and Type Changes

### endat_priv Structure

#### Added Members

<table>
<tr>
    <th>Member</th>
    <th>Type</th>
    <th>Description</th>
</tr>
<tr>
    <td>is_open</td>
    <td>uint8_t</td>
    <td>Initialization state flag (0 = closed, 1 = open)</td>
</tr>
<tr>
    <td>pruicss_handle</td>
    <td>PRUICSS_Handle</td>
    <td>PRU-ICSS driver handle from params</td>
</tr>
<tr>
    <td>channel_rx_info</td>
    <td>endat_ch_rx_info_array*</td>
    <td>Pointer to channel RX info array (replaces endatChRxInfo)</td>
</tr>
<tr>
    <td>cmd_process_delay_us</td>
    <td>uint32_t</td>
    <td>Delay in microseconds for command processing</td>
</tr>
<tr>
    <td>fw_wait_delay_us</td>
    <td>uint32_t</td>
    <td>Delay between firmware status checks</td>
</tr>
<tr>
    <td>max_wait_loop_count</td>
    <td>uint32_t</td>
    <td>Maximum wait loop count for timeout</td>
</tr>
<tr>
    <td>endat_freq</td>
    <td>uint32_t</td>
    <td>Configured EnDAT communication clock frequency</td>
</tr>
</table>

#### Removed Members

<table>
<tr>
    <th>Member</th>
    <th>Old Type</th>
    <th>Migration Action</th>
</tr>
<tr>
    <td>pruicss_slicex</td>
    <td>int32_t</td>
    <td>Moved to endat_attrs->pruicss_slice (via SysConfig)</td>
</tr>
<tr>
    <td>load_share</td>
    <td>int32_t</td>
    <td>Moved to endat_attrs->load_share_enabled (via SysConfig)</td>
</tr>
<tr>
    <td>pruss_cfg</td>
    <td>void*</td>
    <td> Can be obtained from the `pruicss_handle` member of the `endat_priv`.</td>
</tr>
<tr>
    <td>pruss_iep</td>
    <td>void*</td>
    <td>Can be obtained from the `iep_base_addr` member of the `endat_attrs`</td>
</tr>
\cond (SOC_AM263PX || SOC_AM261X || SOC_AM263X)
<tr>
    <td>cmp0, cmp3, cmp5, cmp6</td>
    <td>uint64_t</td>
    <td>Moved to endat_cmd_supplement</td>
</tr>
\endcond
\cond SOC_AM243X || SOC_AM64X
<tr>
    <td>iep_reset_count</td>
    <td>uint64_t</td>
    <td>Moved to endat_cmd_supplement</td>
</tr>
<tr>
    <td>ch0_trigger_count<br>ch1_trigger_count<br>ch2_trigger_count</td>
    <td>uint64_t</td>
    <td>Moved to endat_cmd_supplement->ch_trigger_count[]</td>
</tr>
\endcond
<tr>
    <td>pru_clock</td>
    <td>uint64_t</td>
    <td>Now in endat_attrs via SysConfig</td>
</tr>
<tr>
    <td>pru_uart_clock</td>
    <td>uint64_t</td>
    <td>Now in endat_attrs via SysConfig</td>
</tr>
<tr>
    <td>rx_clock_source</td>
    <td>uint8_t</td>
    <td>Now in endat_attrs via SysConfig</td>
</tr>
<tr>
    <td>tx_clock_source</td>
    <td>uint8_t</td>
    <td>Now in endat_attrs via SysConfig</td>
</tr>
<tr>
    <td>endatChRxInfo</td>
    <td>struct endatChRxInfo*</td>
    <td>Replaced by channel_rx_info</td>
</tr>
</table>

#### Modified Members

<table>
<tr>
    <th>Member</th>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Notes</th>
</tr>
<tr>
    <td>flags</td>
    <td>struct flags</td>
    <td>endat_flags[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
    <td>Changed to array per channel with renamed type</td>
</tr>
<tr>
    <td>id</td>
    <td>struct id</td>
    <td>endat_id</td>
    <td>Renamed type with endat_ prefix</td>
</tr>
<tr>
    <td>sn</td>
    <td>struct sn</td>
    <td>endat_sn</td>
    <td>Renamed type with endat_ prefix</td>
</tr>
<tr>
    <td>pruicss_xchg</td>
    <td>struct endat_pruss_xchg*</td>
    <td>endat_pruicss_xchg*</td>
    <td>Renamed type with typedef</td>
</tr>
</table>

### PRU-ICSS Interface Structures (endat_interface.h)

The following structures in endat_interface.h have been renamed to follow consistent naming conventions (CamelCase to snake_case with typedef pattern):

#### Structure Type Changes

<table>
<tr>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Description</th>
</tr>
<tr>
    <td>Endat_CrcInfo</td>
    <td>endat_crc_info</td>
    <td>CRC error tracking information</td>
</tr>
<tr>
    <td>Endat_ChInfo</td>
    <td>endat_ch_info</td>
    <td>Channel configuration and status</td>
</tr>
<tr>
    <td>Endat_ChRTInfo</td>
    <td>endat_ch_rt_info</td>
    <td>Recovery time parameters</td>
</tr>
<tr>
    <td>Endat_ChRxInfo</td>
    <td>endat_ch_rx_info</td>
    <td>Channel received data structure</td>
</tr>
<tr>
    <td>struct endat_pruss_cmd</td>
    <td>endat_pruicss_cmd</td>
    <td>Command interface (typedef added, pruss→pruicss)</td>
</tr>
<tr>
    <td>struct endat_pruss_config</td>
    <td>endat_pruicss_config</td>
    <td>Configuration interface (typedef added, pruss→pruicss)</td>
</tr>
<tr>
    <td>struct endat_pruss_xchg</td>
    <td>endat_pruicss_xchg</td>
    <td>PRU-ICSS exchange interface (typedef added, pruss→pruicss)</td>
</tr>
<tr>
    <td>struct endatChRxInfo</td>
    <td>endat_ch_rx_info_array</td>
    <td>Channel RX info array structure</td>
</tr>
</table>

#### Member Name Changes (endat_crc_info)

<table>
<tr>
    <th>Old Member</th>
    <th>New Member</th>
</tr>
<tr>
    <td>errCntData</td>
    <td>err_cnt_data</td>
</tr>
<tr>
    <td>errCntAddinfox</td>
    <td>err_cnt_addinfox</td>
</tr>
<tr>
    <td>errCntAddinfo1</td>
    <td>err_cnt_addinfo1</td>
</tr>
<tr>
    <td>resvdInt1</td>
    <td>resvd_int1</td>
</tr>
</table>

#### Member Name Changes (endat_ch_info)

<table>
<tr>
    <th>Old Member</th>
    <th>New Member</th>
</tr>
<tr>
    <td>numClkPulse</td>
    <td>num_clk_pulse</td>
</tr>
<tr>
    <td>endat22Stat</td>
    <td>endat22_stat</td>
</tr>
<tr>
    <td>rxClkLess</td>
    <td>rx_clk_less</td>
</tr>
<tr>
    <td>propDelay</td>
    <td>prop_delay</td>
</tr>
<tr>
    <td>enableRTM</td>
    <td>enable_rtm</td>
</tr>
</table>

#### Member Name Changes (endat_ch_rt_info)

<table>
<tr>
    <th>Old Member</th>
    <th>New Member</th>
</tr>
<tr>
    <td>recoveryTime</td>
    <td>recovery_time</td>
</tr>
<tr>
    <td>currentCounterValue</td>
    <td>current_counter_value</td>
</tr>
<tr>
    <td>lastCounterValue</td>
    <td>last_counter_value</td>
</tr>
<tr>
    <td>startingValue</td>
    <td>starting_value</td>
</tr>
<tr>
    <td>isCounterStuck</td>
    <td>is_counter_stuck</td>
</tr>
</table>

#### Member Name Changes (endat_ch_rx_info)

<table>
<tr>
    <th>Old Member</th>
    <th>New Member</th>
</tr>
<tr>
    <td>posWord0</td>
    <td>pos_word0</td>
</tr>
<tr>
    <td>posWord1</td>
    <td>pos_word1</td>
</tr>
<tr>
    <td>posWord2</td>
    <td>pos_word2</td>
</tr>
<tr>
    <td>posWord3</td>
    <td>pos_word3</td>
</tr>
<tr>
    <td>crcStatus</td>
    <td>crc_status</td>
</tr>
<tr>
    <td>recoveryTimeParms</td>
    <td>recovery_time_parms</td>
</tr>
</table>

### endat_pruicss_xchg Structure

#### Added Members

<table>
<tr>
    <th>Member</th>
    <th>Type</th>
    <th>Description</th>
</tr>
<tr>
    <td>endat_iep_base_addr</td>
    <td>uint32_t</td>
    <td>IEP timer base address for periodic trigger mode</td>
</tr>
<tr>
    <td>trigger_params</td>
    <td>endat_periodic_trigger_cfg[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
    <td>Periodic trigger configuration for each channel</td>
</tr>
<tr>
    <td>reserved</td>
    <td>uint64_t</td>
    <td>Reserved for alignment</td>
</tr>
</table>

#### Removed Members

<table>
<tr>
    <th>Member</th>
    <th>Old Type</th>
    <th>Migration Action</th>
</tr>
<tr>
    <td>endat_rx_clk_config</td>
    <td>uint16_t</td>
    <td>Configured internally by driver</td>
</tr>
<tr>
    <td>endat_tx_clk_config</td>
    <td>uint16_t</td>
    <td>Configured internally by driver</td>
</tr>
<tr>
    <td>endat_rx_clk_cnten</td>
    <td>uint32_t</td>
    <td>Configured internally by driver</td>
</tr>
</table>

#### Modified Members

<table>
<tr>
    <th>Member</th>
    <th>Old Name/Type</th>
    <th>New Name/Type</th>
    <th>Notes</th>
</tr>
<tr>
    <td>Channel info memory address</td>
    <td>endatChInfoMemoryAdd</td>
    <td>ch_info_memory_add</td>
    <td>Renamed to snake_case</td>
</tr>
<tr>
    <td>ICSS clock</td>
    <td>icssg_clk</td>
    <td>icss_clk</td>
    <td>Renamed (icssg→icss)</td>
</tr>
<tr>
    <td>config array</td>
    <td>struct endat_pruss_config config[3]</td>
    <td>endat_pruicss_config config[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
    <td>Type renamed, uses macro for array size</td>
</tr>
<tr>
    <td>cmd array</td>
    <td>struct endat_pruss_cmd cmd[3]</td>
    <td>endat_pruicss_cmd cmd[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
    <td>Type renamed, uses macro for array size</td>
</tr>
<tr>
    <td>ch array</td>
    <td>Endat_ChInfo ch[3]</td>
    <td>endat_ch_info ch[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
    <td>Type renamed, uses macro for array size</td>
</tr>
</table>

### New Structures (endat_interface.h)

<table>
<tr>
    <th>Structure</th>
    <th>Purpose</th>
</tr>
<tr>
    <td>endat_pruicss_cmd</td>
    <td>PRU-ICSS command interface (typedef version of struct endat_pruss_cmd)</td>
</tr>
<tr>
    <td>endat_pruicss_config</td>
    <td>PRU-ICSS configuration interface (typedef version of struct endat_pruss_config)</td>
</tr>
<tr>
    <td>endat_periodic_trigger_cfg</td>
    <td>IEP event configuration for periodic mode (cmp_event, cap_event, iep_capture_reg)</td>
</tr>
<tr>
    <td>endat_ch_rx_info_array</td>
    <td>Array structure for channel RX info (replaces struct endatChRxInfo)</td>
</tr>
</table>

### Removed Structures

<table>
<tr>
    <th>Removed Structure</th>
    <th>Previous Members</th>
    <th>Migration Action</th>
</tr>
<tr>
    <td>endat_clock_config</td>
    <td>rx_clock_source, tx_clock_source, pru_clock, pru_uart_clock</td>
    <td>Clock configuration now handled via SysConfig and endat_attrs structure. Configure clock sources in SysConfig instead.</td>
</tr>
</table>

### New Structures

<table>
<tr>
    <th>Structure</th>
    <th>Purpose</th>
    <th>Key Members</th>
</tr>
<tr>
    <td>endat_params</td>
    <td>Initialization parameters</td>
    <td>pruicss_handle, channel_rx_info, ch_info_global_addr, cmd_process_delay_us, fw_wait_delay_us, max_wait_loop_count</td>
</tr>
<tr>
    <td>endat_attrs</td>
    <td>Compile-time attributes from SysConfig</td>
    <td>PRU-ICSS attributes, channel configuration, clock settings, IEP event configuration</td>
</tr>
<tr>
    <td>endat_config</td>
    <td>Internal configuration structure</td>
    <td>priv (pointer to endat_priv), attrs (pointer to endat_attrs)</td>
</tr>
<tr>
    <td>endat_periodic_trigger_cfg</td>
    <td>IEP event configuration for periodic trigger</td>
    <td>iep_cmp_event, iep_cap_event, iep_capture_reg</td>
</tr>
</table>

### New Types

<table>
<tr>
    <th>New Type</th>
    <th>Description</th>
</tr>
<tr>
    <td>typedef struct endat_params_s ... endat_params</td>
    <td>Initialization parameters structure</td>
</tr>
<tr>
    <td>typedef struct endat_attrs_s ... endat_attrs</td>
    <td>Compile-time attributes structure from SysConfig</td>
</tr>
<tr>
    <td>typedef struct endat_config_s ... endat_config</td>
    <td>Internal configuration structure</td>
</tr>
<tr>
    <td>typedef endat_config *endat_handle</td>
    <td>Opaque handle type for all APIs</td>
</tr>
<tr>
    <td>typedef struct endat_periodic_trigger_cfg_s ... endat_periodic_trigger_cfg</td>
    <td>IEP event configuration structure for periodic trigger</td>
</tr>
</table>

### Type Changes

<table>
<tr>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Description</th>
</tr>
<tr>
    <td>struct endat_clk_cfg<br>struct endat_priv</td>
    <td>typedef struct endat_clk_cfg_s ... endat_clk_cfg<br>typedef struct endat_priv_s ... endat_priv</td>
    <td>Converted to typedef</td>
</tr>
<tr>
    <td>struct endat_data<br>struct endat_position<br>struct endat_addinfo<br>struct endat_position_addinfo<br>struct endat_addr_params<br>struct endat_test_values</td>
    <td>typedef struct endat_data_s ... endat_data<br>typedef struct endat_position_s ... endat_position<br>typedef struct endat_addinfo_s ... endat_addinfo<br>typedef struct endat_position_addinfo_s ... endat_position_addinfo<br>typedef struct endat_addr_params_s ... endat_addr_params<br>typedef struct endat_test_values_s ... endat_test_values</td>
    <td>Converted to typedef</td>
</tr>
<tr>
    <td>union endat_format_data</td>
    <td>typedef union endat_format_data_u ... endat_format_data</td>
    <td>Converted to typedef</td>
</tr>
<tr>
    <td>struct flags<br>struct id<br>struct sn<br>struct cmd_supplement</td>
    <td>typedef struct endat_flags_s ... endat_flags<br>typedef struct endat_id_s ... endat_id<br>typedef struct endat_sn_s ... endat_sn<br>typedef struct endat_cmd_supplement_s ... endat_cmd_supplement</td>
    <td>Added endat_ prefix to type name and converted to typedef</td>
</tr>
<tr>
    <td>enum { linear, rotary }</td>
    <td>enum endat_encoder_type_e { ENDAT_ENCODER_TYPE_LINEAR, ENDAT_ENCODER_TYPE_ROTARY }</td>
    <td>Named enum with ENDAT_ prefixed values</td>
</tr>
</table>

## Macro and Constant Changes

### New Macros

<table>
<tr>
    <th>New Macro</th>
    <th>Description</th>
</tr>
<tr>
    <td>ENDAT_CONFIG_PERIODIC_TRIGGER_CAP_MODE</td>
    <td>CAP-based periodic triggering</td>
</tr>
<tr>
    <td>ENDAT_RX_FRAC_DIV</td>
    <td>RX fractional divider enable</td>
</tr>
<tr>
    <td>ENDAT_OPMODE_CMP_PERIODIC</td>
    <td>CMP periodic mode constant</td>
</tr>
<tr>
    <td>ENDAT_OPMODE_HOST_TRIGGER</td>
    <td>Host trigger mode constant</td>
</tr>
<tr>
    <td>ENDAT_OPMODE_CAP_PERIODIC</td>
    <td>CAP periodic mode constant</td>
</tr>
<tr>
    <td>ENDAT_IEP_CMP_EVENT_MAX</td>
    <td>Maximum CMP event number (16)</td>
</tr>
<tr>
    <td>ENDAT_IEP_CAP_EVENT_MAX</td>
    <td>Maximum CAP event number (8)</td>
</tr>
<tr>
    <td>ENDAT_CHANNEL_MASK</td>
    <td>Channel mask value</td>
</tr>
<tr>
    <td>ENDAT_DEFAULT_CMD_PROCESS_DELAY_US</td>
    <td>Default command delay (1000us)</td>
</tr>
<tr>
    <td>ENDAT_DEFAULT_FW_WAIT_DELAY_US</td>
    <td>Default FW wait delay (1000us)</td>
</tr>
<tr>
    <td>ENDAT_DEFAULT_MAX_WAIT_LOOP_COUNT</td>
    <td>Default timeout loop count (1000)</td>
</tr>
<tr>
    <td>ENDAT_DELAY_COUNTER_INCREMENT</td>
    <td>Delay counter increment value</td>
</tr>
<tr>
    <td>ENDAT_CMD_PROCESS_DELAY_12MS_US</td>
    <td>12ms command delay</td>
</tr>
<tr>
    <td>ENDAT_PARAM_READ_DELAY_2MS_US</td>
    <td>2ms parameter read delay</td>
</tr>
<tr>
    <td>ENDAT_RT_COUNTERS_STARTING_DIFFERENCE</td>
    <td>RT counter starting difference</td>
</tr>
</table>

### Renamed Macros

<table>
<tr>
    <th>Old Name</th>
    <th>New Name</th>
</tr>
<tr>
    <td>NUM_ENCODERS_MAX</td>
    <td>ENDAT_NUM_CH_PER_SLICE_MAX</td>
</tr>
<tr>
    <td>MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0</td>
    <td>ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0</td>
</tr>
<tr>
    <td>MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1</td>
    <td>ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1</td>
</tr>
<tr>
    <td>MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE2</td>
    <td>ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE2</td>
</tr>
<tr>
    <td>SHORT_RECOVERY_TIME_MIN</td>
    <td>ENDAT_SHORT_RECOVERY_TIME_MIN</td>
</tr>
<tr>
    <td>SHORT_RECOVERY_TIME_MAX</td>
    <td>ENDAT_SHORT_RECOVERY_TIME_MAX</td>
</tr>
<tr>
    <td>LONG_RECOVERY_TIME_MIN</td>
    <td>ENDAT_LONG_RECOVERY_TIME_MIN</td>
</tr>
<tr>
    <td>LONG_RECOVERY_TIME_MAX</td>
    <td>ENDAT_LONG_RECOVERY_TIME_MAX</td>
</tr>
<tr>
    <td>MAX_RT_COUNTER_VALUE</td>
    <td>ENDAT_MAX_RT_COUNTER_VALUE</td>
</tr>
<tr>
    <td>RT_OUT_OF_RANGE_ERROR</td>
    <td>ENDAT_RT_OUT_OF_RANGE_ERROR</td>
</tr>
<tr>
    <td>RT_COUNTER_STUCK_ERROR</td>
    <td>ENDAT_RT_COUNTER_STUCK_ERROR</td>
</tr>
<tr>
    <td>RT_NO_ERROR</td>
    <td>ENDAT_RT_NO_ERROR</td>
</tr>
<tr>
    <td>RT_COUNTER_STARTING_VALUE</td>
    <td>ENDAT_RT_COUNTER_STARTING_VALUE</td>
</tr>
<tr>
    <td>ENDAT_CONFIG_PERIODIC_TRIGGER_MODE</td>
    <td>ENDAT_CONFIG_PERIODIC_TRIGGER_CMP_MODE</td>
</tr>
</table>

### Removed Macros

<table>
<tr>
    <th>Removed Macro</th>
    <th>Notes</th>
</tr>
<tr>
    <td>EINVAL</td>
    <td>Removed - use SystemP return codes instead</td>
</tr>
</table>

## Return Value Changes

Many APIs that previously returned void return int32_t for proper error handling:

<table>
<tr>
    <th>Return Value</th>
    <th>Meaning</th>
    <th>When Returned</th>
</tr>
<tr>
    <td>SystemP_SUCCESS</td>
    <td>Operation successful</td>
    <td>Normal completion</td>
</tr>
<tr>
    <td>SystemP_FAILURE</td>
    <td>Operation failed</td>
    <td>Validation failure, NULL pointer, invalid parameters</td>
</tr>
<tr>
    <td>SystemP_TIMEOUT</td>
    <td>Communication timeout</td>
    <td>Encoder not responding within timeout period</td>
</tr>
</table>

## New Features

### 1. Periodic CAP Mode

In addition to the existing CMP (compare) mode, a new CAP (capture) mode was added for periodic triggering:

- **CMP Mode**: Triggers based on IEP timer compare events
- **CAP Mode**: Triggers based on IEP capture events

IEP event configuration can be done using endat_config_iep_cmp_event() or endat_config_iep_cap_event() APIs. These APIs are called in endat_init() with the values configured in SysConfig.

### 2. Enhanced Timeout Handling

Configurable timeout parameters with clear timeout detection for APIs waiting for firmware to signal completion.

## Migration Examples

### Example 1: Basic Initialization

**Old Code:**
```c
struct endat_priv *priv;
priv = endat_init(pruDmemBaseAddress,
                  &gEndatChInfo,
                  gEndatChInfoGlobalAddr,
                  pruicss_cfg,
                  pruicss_iep,
                  PRUICSS_SLICEx,
                  &endat_clk_config);

if (priv == NULL)
{
    /* Handle error */
}
```

**New Code:**
```c
endat_handle handle;
endat_params params;

/* Initialize params */
endat_params_init(&params);
params.pruicss_handle = pruicssHandle;
/* update other params */

/* Initialize EnDat (CONFIG_ENDAT0 is generated by SysConfig) */
handle = endat_init(CONFIG_ENDAT0, &params);
if (handle == NULL)
{
    /* Handle error */
}
```

### Example 2: Command Processing

**Old Code:**
```c
endat_command_process(priv, cmd, cmd_supplement, &val);
endat_position_update(priv);
position = endat_get_position(priv);
```

**New Code:**
```c
int32_t ret = endat_command_process(handle, cmd, cmd_supplement, &val);
if (ret == SystemP_TIMEOUT)
{
    /* Handle timeout */
}
else if (ret != SystemP_SUCCESS)
{
    /* Handle other errors */
}
endat_position_update(handle);
position = endat_get_position(handle);
```

### Example 3: Configuring Periodic Mode

**Old Code:**
```c
endat_config_periodic_trigger(priv);
```

**New Code:**
```c
/* For CMP mode */
ret = endat_config_periodic_trigger_cmp_mode(handle);
if (ret != SystemP_SUCCESS)
{
    /* Handle error */
}

/* OR for CAP mode */
ret = endat_config_periodic_trigger_cap_mode(handle);

if (ret != SystemP_SUCCESS)
{
    /* Handle error */
}
```

### Example 4: Multi-Channel Configuration

**Old Code:**
```c
endat_multi_channel_set_cur(priv, channel);
```

**New Code:**
```c
endat_multi_channel_set_cur(handle, channel);
```

## SysConfig Migration

### Adding EnDat Module in SysConfig

1. Open your project's `.syscfg` file
2. Add the EnDat module under Position Sense, if not added already
3. Configure the parameters as per the requirement. Refer \ref ENDAT_SYSCONFIG_FEATURES for more details.

### Generated Code

SysConfig will generate:
- `endat_attrs` structures with compile-time configuration
- `gEndatHandle` array with endat_config entries
- Code is generated in `ti_drivers_config.c` and `ti_drivers_config.h`

## Common Migration Issues

1. **Compilation Errors with priv pointer**
   - Replace all `struct endat_priv *priv` with `endat_handle handle`
   - Use `endat_get_priv()` when you need access to priv structure

2. **Channel access**
   - Use `endat_get_attrs()` to access total_channels instead

3. **Timeout Errors**
   - Timeout detection is added in certain APIs waiting for firmware.
   - Adjust timeout parameters if needed via endat_params before calling endat_init().

5. **SysConfig Errors**
   - Ensure EnDat module is added and configured in `.syscfg` file
   - Review the configured parameters

6. **Periodic Mode Not Working**
   - Explicitly choose between CMP and CAP modes
   - Use `endat_config_periodic_trigger_cmp_mode()` for timer-based
   - Use `endat_config_periodic_trigger_cap_mode()` for event-based
   - Configure IEP events properly for each channel

5. **Missing Configuration Parameters**
   - Configuration was hardcoded in old implementation
   - Move configuration to SysConfig

## Additional Resources

- \ref ENDAT - EnDat Driver Documentation
- \ref ENDAT_PERIODIC_MODES - Periodic Trigger Modes Details
- \ref EXAMPLE_MOTORCONTROL_ENDAT - EnDat Example Application
- \ref ENDAT_API_MODULE - Complete API Reference

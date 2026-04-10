\cond SOC_AM243X
# EnDat Encoder Migration Guide (v11.00.00 to v2025.00.00) {#ENDAT_MIGRATION_GUIDE_2025_00}
\endcond

\cond (SOC_AM263PX || SOC_AM261X)
# EnDat Encoder Migration Guide (v10.02.00 to v2025.00.00) {#ENDAT_MIGRATION_GUIDE_2025_00}
\endcond

[TOC]

## Introduction

\cond SOC_AM243X
This guide helps developers migrate EnDat encoder applications from Motor Control SDK v11.00.00 to v2025.00.00. The driver underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes, and improved SysConfig integration.
\endcond
\cond (SOC_AM263PX || SOC_AM261X)
This guide helps developers migrate EnDat encoder applications from Motor Control SDK v10.02.00 to v2025.00.00. The driver underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes, and improved SysConfig integration.
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
- **Handle validation**: All public APIs validate the handle parameter for NULL
- **Array bounds checking**: APIs with array parameters or index parameters perform bounds validation
- **Targeted internal structure validation**: Each function validates the pointers it dereferences

\note These changes are not mentioned in the "Additional Details" column of the \ref ENDAT_MIGRATION_GUIDE_2025_00_APIS_MODIFIED section below. The \ref ENDAT_MIGRATION_GUIDE_2025_00_APIS_MODIFIED section describes changes in addition to the points mentioned above.

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

### APIs Modified {#ENDAT_MIGRATION_GUIDE_2025_00_APIS_MODIFIED}

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
    <td>- Uses SysConfig-generated index and params structure as arguments<br>- Validates parameter limits</td>
</tr>
<tr>
    <td>endat_command_build()</td>
    <td>- <code>priv</code> to <code>handle</code><br>- <code>struct cmd_supplement</code> to <code>endat_cmd_supplement</code></td>
    <td>- Type definition change for supplement parameter</td>
</tr>
<tr>
    <td>endat_command_process()</td>
    <td>- <code>priv</code> to <code>handle</code><br>- <code>struct cmd_supplement</code> to <code>endat_cmd_supplement</code></td>
    <td>- Type definition change for supplement parameter<br>- Can return <code>SystemP_TIMEOUT</code> if firmware does not respond within timeout</td>
</tr>
<tr>
    <td>endat_addinfo_track()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code><br>- <code>struct cmd_supplement</code> to <code>endat_cmd_supplement</code></td>
    <td>- Returns status code<br>- Type definition change for supplement parameter</td>
</tr>
<tr>
    <td>endat_recvd_process()<br>endat_recvd_validate()</td>
    <td>- <code>priv</code> to <code>handle</code><br>- <code>union endat_format_data</code> to <code>endat_format_data</code></td>
    <td>- Type definition change for format data parameter</td>
</tr>
<tr>
    <td>endat_config_clock()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code><br>- <code>struct endat_clk_cfg*</code> to <code>uint32_t freq</code></td>
    <td>- Simplified clock configuration with frequency value</td>
</tr>
<tr>
    <td>endat_config_rx_arm_cnt()<br>endat_config_wire_delay()<br>endat_config_rx_clock_disable()<br>endat_config_tst_delay()<br>endat_config_host_trigger()<br>endat_config_multi_channel_mask()<br>endat_init_rt_measurement()<br>endat_enable_rt_measurement()<br>endat_disable_rt_measurement()<br>endat_command_send()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>- Returns status code</td>
</tr>
<tr>
    <td>endat_config_channel()<br>endat_multi_channel_set_cur()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code><br>- <code>int32_t ch</code> to <code>uint32_t ch</code></td>
    <td>- Returns status code<br>- Channel parameter type changed to unsigned</td>
</tr>
<tr>
    <td>endat_command_wait()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>- Returns status code<br>- Can return <code>SystemP_TIMEOUT</code> if firmware does not complete transaction within configured timeout</td>
</tr>
<tr>
    <td>endat_stop_continuous_mode()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>- Returns status code<br>- Can return <code>SystemP_TIMEOUT</code> if firmware does not respond within timeout</td>
</tr>
<tr>
    <td>endat_start_continuous_mode()</td>
    <td>- <code>priv</code> to <code>handle</code></td>
    <td>- Return type unchanged (was already <code>int32_t</code>)</td>
</tr>
<tr>
    <td>endat_wait_initialization()</td>
    <td>- <code>priv</code> to <code>handle</code></td>
    <td>- Return type unchanged (was already <code>int32_t</code>)<br>- Can return <code>SystemP_TIMEOUT</code> if initialization does not complete within timeout period</td>
</tr>
<tr>
    <td>endat_get_encoder_info()</td>
    <td>- <code>priv</code> to <code>handle</code></td>
    <td>- Return type unchanged (was already <code>int32_t</code>)<br>- Can return <code>SystemP_TIMEOUT</code> if firmware does not respond within timeout (propagated from <code>endat_command_process()</code>)</td>
</tr>
<tr>
    <td>endat_multi_channel_detected()</td>
    <td>- <code>priv</code> to <code>handle</code></td>
    <td>- Return type unchanged (<code>uint8_t</code>)</td>
</tr>
<tr>
    <td>endat_get_recovery_time()<br>endat_get_prop_delay()<br>endat_status_rt_measurement()</td>
    <td>- <code>uint32_t</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code><br>- Added output pointer param</td>
    <td>- Return value moved to output pointer; returns status code</td>
</tr>
<tr>
    <td>endat_check_rt_error()</td>
    <td>- <code>int8_t</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code><br>- Added <code>int8_t *error_code</code> output param</td>
    <td>- Error code moved to output pointer; returns status code</td>
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

<table>
<tr>
    <th>Change Type</th>
    <th>Old Member</th>
    <th>New Member</th>
    <th>Notes</th>
</tr>
<tr>
    <td rowspan="6">Added</td>
    <td rowspan="6">-</td>
    <td>uint8_t is_open</td>
    <td>Initialization state flag (0 = closed, 1 = open)</td>
</tr>
<tr>
    <td>PRUICSS_Handle pruicss_handle</td>
    <td>PRU-ICSS driver handle from params</td>
</tr>
<tr>
    <td>uint32_t cmd_process_delay_us</td>
    <td>Delay in microseconds for command processing</td>
</tr>
<tr>
    <td>uint32_t fw_wait_delay_us</td>
    <td>Delay between firmware status checks</td>
</tr>
<tr>
    <td>uint32_t max_wait_loop_count</td>
    <td>Maximum wait loop count for timeout</td>
</tr>
<tr>
    <td>uint32_t endat_freq</td>
    <td>Configured EnDAT communication clock frequency</td>
</tr>
<tr>
    <td rowspan="4">Removed</td>
    <td>void* pruss_cfg</td>
    <td>-</td>
    <td>Can be obtained from the `pruicss_handle` member of endat_priv</td>
</tr>
<tr>
    <td>void* pruss_iep</td>
    <td>-</td>
    <td>Can be obtained from the `iep_base_addr` member of endat_attrs</td>
</tr>
\cond (SOC_AM263PX || SOC_AM261X)
<tr>
    <td>uint64_t cmp0<br>uint64_t cmp3<br>uint64_t cmp5<br>uint64_t cmp6</td>
    <td>-</td>
    <td>-</td>
</tr>
\endcond
\cond SOC_AM243X
<tr>
    <td>uint64_t iep_reset_count<br>uint64_t ch0_trigger_count<br>uint64_t ch1_trigger_count<br>uint64_t ch2_trigger_count</td>
    <td>-</td>
    <td>-</td>
</tr>
\endcond
<tr>
    <td>int32_t pruicss_slicex<br>int32_t load_share<br>uint64_t pru_clock<br>uint64_t pru_uart_clock<br>uint8_t rx_clock_source<br>uint8_t tx_clock_source</td>
    <td>-</td>
    <td>Moved to endat_attrs</td>
</tr>
<tr>
    <td rowspan="2">Modified</td>
    <td>struct flags flags</td>
    <td>endat_flags flags[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
    <td>Changed to per-channel array</td>
</tr>
<tr>
    <td>struct endatChRxInfo* endatChRxInfo</td>
    <td>endat_ch_rx_info_array* channel_rx_info</td>
    <td>Renamed</td>
</tr>
</table>

### endat_cmd_supplement Structure

The `struct cmd_supplement` was renamed to `endat_cmd_supplement` with typedef. In addition to the type rename, the following member changes were made:

<table>
<tr>
    <th>Change Type</th>
    <th>Member</th>
    <th>Old</th>
    <th>New</th>
    <th>Notes</th>
</tr>
\cond (SOC_AM263PX || SOC_AM261X)
<tr>
    <td rowspan="4">Modified</td>
    <td>cmp0</td>
    <td>uint64_t cmp0</td>
    <td>uint64_t iep_reset_count</td>
    <td>Renamed</td>
</tr>
<tr>
    <td>cmp3</td>
    <td>uint64_t cmp3</td>
    <td>uint64_t ch_trigger_count[0]</td>
    <td rowspan="3">Renamed and consolidated into per-channel array</td>
</tr>
<tr>
    <td>cmp5</td>
    <td>uint64_t cmp5</td>
    <td>uint64_t ch_trigger_count[1]</td>
</tr>
<tr>
    <td>cmp6</td>
    <td>uint64_t cmp6</td>
    <td>uint64_t ch_trigger_count[2]</td>
</tr>
\endcond
\cond SOC_AM243X
<tr>
    <td rowspan="3">Modified</td>
    <td>ch0_trigger_count</td>
    <td>uint64_t ch0_trigger_count</td>
    <td>uint64_t ch_trigger_count[0]</td>
    <td rowspan="3">Consolidated into per-channel array</td>
</tr>
<tr>
    <td>ch1_trigger_count</td>
    <td>uint64_t ch1_trigger_count</td>
    <td>uint64_t ch_trigger_count[1]</td>
</tr>
<tr>
    <td>ch2_trigger_count</td>
    <td>uint64_t ch2_trigger_count</td>
    <td>uint64_t ch_trigger_count[2]</td>
</tr>
\endcond
<tr>
    <td rowspan="4">Type Changed</td>
    <td>address</td>
    <td>uint32_t</td>
    <td>uint32_t address[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
    <td rowspan="4">Per-channel member for multi-channel support using load share mode (on AM243x only)</td>
</tr>
<tr>
    <td>data</td>
    <td>uint32_t</td>
    <td>uint32_t data[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
</tr>
<tr>
    <td>block</td>
    <td>uint32_t</td>
    <td>uint32_t block[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
</tr>
<tr>
    <td>has_block_address</td>
    <td>uint8_t</td>
    <td>uint8_t has_block_address[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
</tr>
<tr>
    <td rowspan="6">Added</td>
    <td>cmd_type</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>Command type identifier</td>
</tr>
<tr>
    <td>delay</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>User-specified delay parameter</td>
</tr>
<tr>
    <td>enable_rt</td>
    <td>-</td>
    <td>uint8_t</td>
    <td>Enable/disable recovery time counter</td>
</tr>
<tr>
    <td>selected_channel</td>
    <td>-</td>
    <td>uint8_t</td>
    <td>Selected channel for multi-channel operations</td>
</tr>
<tr>
    <td>periodic_mode_cmd</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>Command for periodic mode configuration</td>
</tr>
<tr>
    <td>iep_sync0_period</td>
    <td>-</td>
    <td>uint64_t</td>
    <td>IEP SYNC OUT0 period</td>
</tr>
</table>

### endat_pruicss_xchg Structure

<table>
<tr>
    <th>Change Type</th>
    <th>Old Member</th>
    <th>New Member</th>
    <th>Notes</th>
</tr>
<tr>
    <td rowspan="3">Added</td>
    <td rowspan="3">-</td>
    <td>uint32_t endat_iep_base_addr</td>
    <td>IEP timer base address for periodic trigger mode</td>
</tr>
<tr>
    <td>endat_periodic_trigger_cfg trigger_params[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
    <td>Periodic trigger configuration for each channel</td>
</tr>
<tr>
    <td>uint64_t reserved</td>
    <td>Reserved for alignment</td>
</tr>
<tr>
    <td rowspan="3">Removed</td>
    <td>uint16_t endat_rx_clk_config</td>
    <td rowspan="3">-</td>
    <td rowspan="3">Configured internally by driver</td>
</tr>
<tr>
    <td>uint16_t endat_tx_clk_config</td>
</tr>
<tr>
    <td>uint32_t endat_rx_clk_cnten</td>
</tr>
<tr>
    <td rowspan="5">Modified</td>
    <td>endatChInfoMemoryAdd</td>
    <td>ch_info_memory_add</td>
    <td rowspan="2">Renamed</td>
</tr>
<tr>
    <td>icssg_clk</td>
    <td>icss_clk</td>
</tr>
<tr>
    <td>struct endat_pruss_config config[3]</td>
    <td>endat_pruicss_config config[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
    <td rowspan="3">Use macro for array size</td>
</tr>
<tr>
    <td>struct endat_pruss_cmd cmd[3]</td>
    <td>endat_pruicss_cmd cmd[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
</tr>
<tr>
    <td>Endat_ChInfo ch[3]</td>
    <td>endat_ch_info ch[ENDAT_NUM_CH_PER_SLICE_MAX]</td>
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
    <td>Clock configuration now handled via endat_attrs structure</td>
</tr>
</table>

### New Structures

<table>
<tr>
    <th>Structure</th>
    <th>Typedef</th>
    <th>Purpose</th>
    <th>Key Members</th>
</tr>
<tr>
    <td>endat_params</td>
    <td><code>typedef struct endat_params_s endat_params</code></td>
    <td>Initialization parameters</td>
    <td>pruicss_handle, channel_rx_info, ch_info_global_addr, cmd_process_delay_us, fw_wait_delay_us, max_wait_loop_count</td>
</tr>
<tr>
    <td>endat_attrs</td>
    <td><code>typedef struct endat_attrs_s endat_attrs</code></td>
    <td>Compile-time attributes from SysConfig</td>
    <td>PRU-ICSS attributes, channel configuration, clock settings, IEP event configuration</td>
</tr>
<tr>
    <td>endat_config</td>
    <td><code>typedef struct endat_config_s endat_config</code><br><code>typedef endat_config *endat_handle</code></td>
    <td>Internal configuration structure</td>
    <td>priv (pointer to endat_priv), attrs (pointer to endat_attrs)</td>
</tr>
<tr>
    <td>\ref endat_periodic_trigger_cfg</td>
    <td><code>typedef struct endat_periodic_trigger_cfg_s endat_periodic_trigger_cfg</code></td>
    <td>IEP event configuration for periodic trigger</td>
    <td>iep_cmp_event, iep_cap_event, iep_capture_reg</td>
</tr>
</table>


### Type and Name Changes

<table>
<tr>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Description</th>
</tr>
<tr>
    <td>struct endat_clk_cfg<br>struct endat_priv<br>struct endat_data<br>struct endat_position<br>struct endat_addinfo<br>struct endat_position_addinfo<br>struct endat_addr_params<br>struct endat_test_values<br>union endat_format_data</td>
    <td>typedef struct endat_clk_cfg_s ... endat_clk_cfg<br>typedef struct endat_priv_s ... endat_priv<br>typedef struct endat_data_s ... endat_data<br>typedef struct endat_position_s ... endat_position<br>typedef struct endat_addinfo_s ... endat_addinfo<br>typedef struct endat_position_addinfo_s ... endat_position_addinfo<br>typedef struct endat_addr_params_s ... endat_addr_params<br>typedef struct endat_test_values_s ... endat_test_values<br>typedef union endat_format_data_u ... endat_format_data</td>
    <td>Converted to typedef</td>
</tr>
<tr>
    <td>struct flags<br>struct id<br>struct sn<br>struct cmd_supplement</td>
    <td>typedef struct endat_flags_s ... endat_flags<br>typedef struct endat_id_s ... endat_id<br>typedef struct endat_sn_s ... endat_sn<br>typedef struct endat_cmd_supplement_s ... endat_cmd_supplement</td>
    <td>Added endat_ prefix to type name and converted to typedef</td>
</tr>
<tr>
    <td>Endat_CrcInfo<br>Endat_ChInfo<br>Endat_ChRTInfo<br>Endat_ChRxInfo</td>
    <td>endat_crc_info<br>endat_ch_info<br>endat_ch_rt_info<br>endat_ch_rx_info</td>
    <td>Renamed</td>
</tr>
<tr>
    <td>struct endat_pruss_cmd<br>struct endat_pruss_config<br>struct endat_pruss_xchg<br>struct endatChRxInfo</td>
    <td>typedef endat_pruicss_cmd_s ... endat_pruicss_cmd<br>typedef endat_pruicss_config_s ... endat_pruicss_config<br>typedef endat_pruicss_xchg_s ... endat_pruicss_xchg<br>typedef endat_ch_rx_info_array_s ... endat_ch_rx_info_array</td>
    <td>Renamed with typedef</td>
</tr>
<tr>
    <td>enum { linear, rotary }</td>
    <td>enum endat_encoder_type_e <br>{ ENDAT_ENCODER_TYPE_LINEAR, ENDAT_ENCODER_TYPE_ROTARY} <br> endat_encoder_type</td>
    <td>Named enum with updated values containing ENDAT_ENCODER_TYPE_ prefix</td>
</tr>
</table>

### Member Name Changes

<table>
<tr>
    <th>Structure</th>
    <th>Old Member</th>
    <th>New Member</th>
</tr>
<tr>
    <td rowspan="4">endat_crc_info</td>
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
<tr>
    <td rowspan="5">endat_ch_info</td>
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
<tr>
    <td rowspan="5">endat_ch_rt_info</td>
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
<tr>
    <td rowspan="6">endat_ch_rx_info</td>
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

## Macro and Constant Changes

\note Only important macro changes are listed below.

### New Macros

<table>
<tr>
    <th>New Macro</th>
    <th>Description</th>
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
</table>

### Modified Macros

<table>
<tr>
    <th>Macro</th>
    <th>Key Changes</th>
    <th>Additional Details</th>
</tr>
<tr>
    <td>ENDAT_GET_POS_MULTI_TURN(pos, handle)<br>ENDAT_GET_POS_SINGLE_TURN(pos, handle)</td>
    <td>- Parameter renamed: <code>priv</code> to <code>handle</code><br>- Type cast changed: <code>unsigned long long</code> to <code>uint64_t</code></td>
    <td>- Update all call sites to pass <code>handle</code> instead of <code>priv</code></td>
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
    <td>Removed, replaced with SystemP return codes</td>
</tr>
</table>

## Return Value Changes

Many APIs that previously returned void return int32_t for proper error handling.

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

2. **Timeout Errors**
   - Timeout detection is added in certain APIs waiting for firmware.
   - Adjust timeout parameters if needed via endat_params before calling endat_init().

3. **SysConfig Errors**
   - Ensure EnDat module is added and configured in `.syscfg` file
   - Review the configured parameters

4. **Periodic Mode Not Working**
   - Explicitly choose between CMP and CAP modes
   - Use `endat_config_periodic_trigger_cmp_mode()` for IEP compare event based trigger
   - Use `endat_config_periodic_trigger_cap_mode()` for IEP capture event based trigger
   - Configure IEP events properly for each channel

## Additional Resources

- \ref EXAMPLE_MOTORCONTROL_ENDAT
- \ref ENDAT_API_MODULE
- \ref ENDAT
- \ref ENDAT_DESIGN

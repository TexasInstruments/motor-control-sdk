\cond SOC_AM243X
# Tamagawa Encoder Migration Guide (v11.00.00 to v2025.00.00) {#TAMAGAWA_MIGRATION_GUIDE_2025_00}
\endcond

\cond (SOC_AM263PX || SOC_AM261X)
# Tamagawa Encoder Migration Guide (v10.02.00 to v2025.00.00) {#TAMAGAWA_MIGRATION_GUIDE_2025_00}
\endcond

[TOC]

## Introduction

\cond SOC_AM243X
This guide helps developers migrate Tamagawa encoder applications from Motor Control SDK v11.00.00 to v2025.00.00. The driver underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes, and improved SysConfig integration.
\endcond
\cond (SOC_AM263PX || SOC_AM261X)
This guide helps developers migrate Tamagawa encoder applications from Motor Control SDK v10.02.00 to v2025.00.00. The driver underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes, and improved SysConfig integration.
\endcond

## Major Architectural Changes

### 1. Handle-Based API Architecture

The entire Tamagawa driver API was refactored from pointer-based to an improved handle-based architecture.

<table>
<tr>
    <th>Old API</th>
    <th>New API</th>
    <th>Impact</th>
</tr>
<tr>
    <td>struct \ref tamagawa_priv *priv</td>
    <td>\ref tamagawa_handle handle</td>
    <td>All APIs use opaque handle instead of direct structure pointer</td>
</tr>
</table>

### 2. SysConfig-Based Initialization

The initialization process was completely redesigned to use SysConfig-generated parameters for compile-time variables and tamagawa_params structure for run-time variables.

<table>
<tr>
    <th>Old Initialization</th>
    <th>New Initialization</th>
</tr>
<tr>
    <td>
    <pre>
    struct tamagawa_priv *priv;
    struct tamagawa_xchg *xchg;
    void *pruss_cfg, *pruss_iep;

    /* Manual base address setup */
    xchg = (struct tamagawa_xchg *)pru_dram_base;
    pruss_cfg = (void *)ICSS_CFG_BASE;
    pruss_iep = (void *)ICSS_IEP_BASE;

    /* Initialize */
    priv = tamagawa_init(xchg, pruss_cfg,
                         pruss_iep, slice);
    </pre>
    </td>
    <td>
    <pre>
    tamagawa_params params;
    tamagawa_params_init(&params);
    params.pruicss_handle = pruHandle;

    /* SysConfig provides compile-time configuration */
    tamagawa_handle handle = tamagawa_init(
        CONFIG_TAMAGAWA0, &params);
    </pre>
    </td>
</tr>
</table>

## API Changes

Driver APIs use following validation approach now:
 - **Handle validation**: All public APIs validate the handle parameter for NULL
 - **Array bounds checking**: APIs with array parameters or index parameters perform bounds validation
 - **Internal structure validation**: All APIs validate internal structure pointers (attrs, priv, tamagawa_xchg, pruicss_handle, etc.) for NULL before dereferencing to provide protection against NULL pointer dereferences

\note These changes are not mentioned in the "Additional Details" column of the \ref TAMAGAWA_MIGRATION_GUIDE_2025_00_APIS_MODIFIED section below. The \ref TAMAGAWA_MIGRATION_GUIDE_2025_00_APIS_MODIFIED section describes changes in addition to the points mentioned above.

### New APIs Added

<table>
<tr>
    <th>New API</th>
    <th>Description</th>
    <th>Usage</th>
</tr>
<tr>
    <td>tamagawa_params_init()</td>
    <td>Initialize params structure with defaults</td>
    <td>Use before tamagawa_init()</td>
</tr>
<tr>
    <td>tamagawa_get_attrs()</td>
    <td>Get pointer to compile-time attributes</td>
    <td>Access SysConfig-generated configuration</td>
</tr>
<tr>
    <td>tamagawa_get_priv()</td>
    <td>Get pointer to runtime private data</td>
    <td>Access runtime state information</td>
</tr>
<tr>
    <td>tamagawa_deinit()</td>
    <td>De-initialize Tamagawa interface</td>
    <td>Cleanup when done using driver</td>
</tr>
<tr>
    <td>tamagawa_config_periodic_trigger_cmp_mode()</td>
    <td>Configure IEP CMP event based periodic triggering</td>
    <td>Replaces tamagawa_config_periodic_trigger() for CMP mode</td>
</tr>
<tr>
    <td>tamagawa_config_periodic_trigger_cap_mode()</td>
    <td>Configure IEP CAP event based periodic triggering</td>
    <td>-</td>
</tr>
<tr>
    <td>tamagawa_config_iep_cmp_event()</td>
    <td>Configure IEP CMP event number in firmware DMEM</td>
    <td>Set CMP event (0-15) for each channel</td>
</tr>
<tr>
    <td>tamagawa_config_iep_cap_event()</td>
    <td>Configure IEP CAP event number in firmware DMEM</td>
    <td>Set CAP event (0-7) for each channel</td>
</tr>
<tr>
    <td>tamagawa_config_global_rx_arm_cnt()</td>
    <td>Configure global RX auto arm counter for all channels</td>
    <td>-</td>
</tr>
</table>

### APIs Modified {#TAMAGAWA_MIGRATION_GUIDE_2025_00_APIS_MODIFIED}

<table>
<tr>
    <th>API</th>
    <th>Key Changes</th>
    <th>Additional Details</th>
</tr>
<tr>
    <td>tamagawa_init()</td>
    <td>- Complete signature change<br>- 4 parameters to 2 parameters<br>- Returns <code>tamagawa_handle</code> instead of <code>tamagawa_priv priv *</code></td>
    <td>- Complete signature change<br>- Use SysConfig instance ID and params structure<br>- Validates parameter limits</td>
</tr>
<tr>
    <td>tamagawa_command_process()</td>
    <td>- <code>priv</code> to <code>handle</code><br>- Removed gTamagawa_multi_ch_mask parameter</td>
    <td>- Channel mask now determined from attrs<br>- Returns SystemP_TIMEOUT on timeout</td>
</tr>
<tr>
    <td>tamagawa_command_build()</td>
    <td>- <code>priv</code> to <code>handle</code><br>- Removed gTamagawa_multi_ch_mask parameter</td>
    <td>- Channel mask now determined from attrs<br>- Validates parameter limits</td>
</tr>
<tr>
    <td>tamagawa_command_send()<br>tamagawa_config_clock()<br>tamagawa_config_host_trigger()<br>tamagawa_update_data_id() </td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>- Returns status code</td>
</tr>
<tr>
    <td>tamagawa_command_wait()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>- Returns status code<br>- Returns SystemP_TIMEOUT on timeout</td>
</tr>
<tr>
    <td>tamagawa_config_channel()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code><br>- Parameter changed from uint32_t ch to uint8_t mask</td>
    <td>- Returns status code<br>- Validates parameter limits</td>
</tr>
<tr>
    <td>tamagawa_multi_channel_set_cur()<br>tamagawa_update_adf()<br>tamagawa_update_edf()<br>tamagawa_update_crc()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code><br>- Parameter ch changed from uint32_t to uint8_t</td>
    <td>- Update parameters type<br>- Returns status code<br>- Validates parameter limits</td>
</tr>
<tr>
    <td>tamagawa_parse()</td>
    <td>- <code>priv</code> to <code>handle</code><br>- Parameter order changed (handle first, cmd second)</td>
    <td>-</td>
</tr>
<tr>
    <td>tamagawa_crc_verify()</td>
    <td>- <code>priv</code> to <code>handle</code></td>
    <td>- Returns SystemP_SUCCESS/SystemP_FAILURE instead of 1/-1</td>
</tr>
<tr>
    <td>tamagawa_set_baudrate()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code><br>- Parameter renamed from baudrate to baud_rate</td>
    <td>- Returns status code<br>- Divider configuration and oversampling rate configuration moved to driver from PRU firmware</td>
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
    <td>tamagawa_config_periodic_trigger()</td>
    <td>tamagawa_config_periodic_trigger_cmp_mode() or tamagawa_config_periodic_trigger_cap_mode()</td>
    <td>Choose appropriate mode based on trigger source</td>
</tr>
<tr>
    <td>tamagawa_config_multi_channel_mask()</td>
    <td>tamagawa_config_channel()</td>
    <td>Use tamagawa_config_channel() with mask parameter</td>
</tr>
<tr>
    <td>tamagawa_multi_channel_detected()</td>
    <td>-</td>
    <td>-</td>
</tr>
<tr>
    <td>tamagawa_eeprom_crc_reinit()<br>tamagawa_reverse_bits()<br>tamagawa_prepare_eeprom_tx_data()<br>tamagawa_prepare_eeprom_command() </td>
    <td>Made internal (static)</td>
    <td>No longer accessible from application</td>
</tr>

</table>

## Structure and Type Changes

### Structures Removed

<table>
<tr>
    <th>Structure</th>
    <th>Migration Action</th>
</tr>
<tr>
    <td>struct register_offsets</td>
    <td>Register offset management is now internal to the driver. No replacement needed.</td>
</tr>
</table>

### tamagawa_priv Structure

<table>
<tr>
    <th>Change Type</th>
    <th>Member</th>
    <th>Old Type/Location</th>
    <th>New Type/Location</th>
    <th>Notes</th>
</tr>
<tr>
    <td rowspan="5">Added</td>
    <td>is_open</td>
    <td>-</td>
    <td>uint8_t</td>
    <td>Initialization state flag (0 = closed, 1 = open)</td>
</tr>
<tr>
    <td>clk_cfg</td>
    <td>-</td>
    <td>tamagawa_clk_cfg</td>
    <td>Clock configuration structure</td>
</tr>
<tr>
    <td>pruicss_handle</td>
    <td>-</td>
    <td>PRUICSS_Handle</td>
    <td>PRU-ICSS handle from params</td>
</tr>
<tr>
    <td>cmd_wait_delay_us<br>max_wait_loop_count</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>Timeout and delay configuration parameters</td>
</tr>
<tr>
    <td>tamagawa_interface[3]</td>
    <td>-</td>
    <td>tamagawa_interface array</td>
    <td>Interface configuration per channel (moved from tamagawa_xchg)</td>
</tr>
<tr>
    <td rowspan="6">Removed</td>
    <td>register_offset_val</td>
    <td>struct register_offsets</td>
    <td>-</td>
    <td>Register offset management now internal to driver</td>
</tr>
<tr>
    <td>slice_value</td>
    <td>int32_t</td>
    <td>attrs->pruicss_slice</td>
    <td>PRU slice value moved to tamagawa_attrs</td>
</tr>
<tr>
    <td>rx_en_cnt</td>
    <td>uint16_t</td>
    <td>clk_cfg.rx_en_cnt</td>
    <td>Moved to tamagawa_clk_cfg clk_cfg within tamagawa_priv</td>
</tr>
<tr>
    <td>pruss_cfg</td>
    <td>void *</td>
    <td>-</td>
    <td>PRU-ICSS CFG registers base offset (now internal)</td>
</tr>
<tr>
    <td>pruss_iep</td>
    <td>void *</td>
    <td>attrs->iep_base_addr</td>
    <td>ICSS IEP base address moved to tamagawa_attrs</td>
</tr>
\cond SOC_AM243X
<tr>
    <td>iep_reset_count<br>periodic_trigger_count<br>pru_clock<br>pru_uart_clock<br>rx_clock_source<br>tx_clock_source</td>
    <td>uint64_t / uint8_t</td>
    <td>-</td>
    <td>IEP configuration and clock configuration moved to tamagawa_attrs and tamagawa_periodic_trigger_cfg</td>
</tr>
\endcond
\cond (SOC_AM263PX || SOC_AM261X)
<tr>
    <td>cmp0<br>cmp3<br>pru_clock<br>pru_uart_clock<br>rx_clock_source<br>tx_clock_source</td>
    <td>uint64_t / uint8_t</td>
    <td>-</td>
    <td>IEP configuration and clock configuration moved to tamagawa_attrs and tamagawa_periodic_trigger_cfg</td>
</tr>
\endcond
<tr>
    <td>Type Changed</td>
    <td>channel</td>
    <td>int32_t</td>
    <td>uint8_t</td>
    <td>Changed to unsigned 8-bit</td>
</tr>
</table>

### tamagawa_xchg Structure

<table>
<tr>
    <th>Change Type</th>
    <th>Member</th>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Notes</th>
</tr>
<tr>
    <td rowspan="4">Added</td>
    <td>execution_state</td>
    <td>-</td>
    <td>uint8_t[TAMAGAWA_MAX_CHANNELS_PER_SLICE]</td>
    <td>Execution state per channel for load-share mode</td>
</tr>
<tr>
    <td>primary_core_mask</td>
    <td>-</td>
    <td>uint8_t</td>
    <td>Primary core mask for load-share mode</td>
</tr>
<tr>
    <td>iep_base_addr</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>IEP register base address for periodic trigger mode</td>
</tr>
<tr>
    <td>trigger_params</td>
    <td>-</td>
    <td>tamagawa_periodic_trigger_cfg[TAMAGAWA_MAX_CHANNELS_PER_SLICE]</td>
    <td>Periodic trigger configuration per channel</td>
</tr>
<tr>
    <td rowspan="2">Changed to Array</td>
    <td>config</td>
    <td>tamagawa_fw_config</td>
    <td>tamagawa_fw_config[TAMAGAWA_MAX_CHANNELS_PER_SLICE]</td>
    <td>Config per channel for load-share support</td>
</tr>
<tr>
    <td>cmd</td>
    <td>tamagawa_cmd</td>
    <td>tamagawa_cmd[TAMAGAWA_MAX_CHANNELS_PER_SLICE]</td>
    <td>Command per channel for load-share support</td>
</tr>
<tr>
    <td>Removed</td>
    <td>tamagawa_interface</td>
    <td>tamagawa_interface</td>
    <td>-</td>
    <td>Moved to tamagawa_priv as array</td>
</tr>
</table>

### tamagawa_interface Structure

<table>
<tr>
    <th>Change Type</th>
    <th>Member</th>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Notes</th>
</tr>
<tr>
    <td rowspan="4">Removed</td>
    <td>ch_mask</td>
    <td>uint8_t</td>
    <td>-</td>
    <td>Moved to attrs->channel_mask</td>
</tr>
<tr>
    <td>rx_div_factor</td>
    <td>uint16_t</td>
    <td>-</td>
    <td>Computed internally</td>
</tr>
<tr>
    <td>tx_div_factor</td>
    <td>uint16_t</td>
    <td>-</td>
    <td>Computed internally</td>
</tr>
<tr>
    <td>oversample_rate</td>
    <td>uint8_t</td>
    <td>-</td>
    <td>Computed internally</td>
</tr>
</table>

### tamagawa_clk_cfg Structure

<table>
<tr>
    <th>Change Type</th>
    <th>Member</th>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Notes</th>
</tr>
<tr>
    <td>Added</td>
    <td>rx_en_cnt</td>
    <td>-</td>
    <td>uint16_t</td>
    <td>RX enable count field</td>
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
    <td>tamagawa_params</td>
    <td><code>typedef struct tamagawa_params_s tamagawa_params</code></td>
    <td>Initialization parameters</td>
    <td>pruicss_handle, cmd_wait_delay_us, max_wait_loop_count</td>
</tr>
<tr>
    <td>tamagawa_attrs</td>
    <td><code>typedef struct tamagawa_attrs_s tamagawa_attrs</code></td>
    <td>Compile-time attributes from SysConfig</td>
    <td>instance, mode, PRU-ICSS configuration, channel_mask, baud_rate, clock frequencies, IEP event arrays</td>
</tr>
<tr>
    <td>tamagawa_periodic_trigger_cfg</td>
    <td><code>typedef struct tamagawa_periodic_trigger_cfg_s tamagawa_periodic_trigger_cfg</code></td>
    <td>IEP event configuration for periodic trigger</td>
    <td>iep_cmp_event, iep_cap_event, iep_capture_reg</td>
</tr>
<tr>
    <td>tamagawa_config</td>
    <td><code>typedef struct tamagawa_config_s tamagawa_config</code><br><code>typedef tamagawa_config *tamagawa_handle</code></td>
    <td>Internal configuration structure (handle)</td>
    <td>priv, attrs pointers</td>
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
    <td>struct tamagawa_clk_cfg<br>struct tamagawa_priv<br>struct tamagawa_xchg<br>struct tamagawa_interface<br>struct tamagawa_ch_info<br>struct tamagawa_cmd<br>struct tamagawa_eeprom_interface</td>
    <td>typedef struct tamagawa_clk_cfg_s ... tamagawa_clk_cfg<br>typedef struct tamagawa_priv_s ... tamagawa_priv<br>typedef struct tamagawa_xchg_s ... tamagawa_xchg<br>typedef struct tamagawa_interface_s ... tamagawa_interface<br>typedef struct tamagawa_ch_info_s ... tamagawa_ch_info<br>typedef struct tamagawa_cmd_s ... tamagawa_cmd<br>typedef struct tamagawa_eeprom_interface_s ... tamagawa_eeprom_interface</td>
    <td>Converted to typedef pattern</td>
</tr>
<tr>
    <td>struct `%tamagawa_config`</td>
    <td>typedef struct tamagawa_fw_config_s ... tamagawa_fw_config</td>
    <td>Renamed to tamagawa_fw_config to avoid confusion with new tamagawa_config handle structure and converted to typedef</td>
</tr>
<tr>
    <td>struct `rx_frames_received`</td>
    <td>typedef struct tamagawa_rx_frames_s ... tamagawa_rx_frames</td>
    <td>Renamed with tamagawa_ prefix and converted to typedef</td>
</tr><tr>
    <td>struct `config`</td>
    <td>typedef struct tamagawa_channel_config_s ... tamagawa_channel_config</td>
    <td>Renamed to tamagawa_channel_config to avoid confusion with new tamagawa_config handle structure and converted to typedef</td>
</tr>
<tr>
    <td>enum `data_id`</td>
    <td>typedef enum tamagawa_data_id_e ... tamagawa_data_id</td>
    <td>Named enum with typedef. PERIODIC_TRIGGER_CMD renamed to PERIODIC_TRIGGER_CMP_CMD, added PERIODIC_TRIGGER_CAP_CMD</td>
</tr>
</table>

## Macro and Constant Changes

\note Only important macro changes are listed below.

### New Macros

<table>
<tr>
    <th>Macro</th>
    <th>Value</th>
    <th>Description</th>
</tr>
<tr>
    <td>TAMAGAWA_MODE_SINGLE_CHANNEL_SINGLE_PRU</td>
    <td>0U</td>
    <td>Configuration mode for single channel, single PRU</td>
</tr>
<tr>
    <td>TAMAGAWA_MODE_MULTI_CHANNEL_SINGLE_PRU</td>
    <td>1U</td>
    <td>Configuration mode for multi-channel, single PRU</td>
</tr>
<tr>
    <td>TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU</td>
    <td>2U</td>
    <td>Configuration mode for multi-channel with load-share across multiple PRUs</td>
</tr>
<tr>
    <td>TAMAGAWA_OPMODE_PERIODIC_CMP</td>
    <td>0x0U</td>
    <td>Operation mode: periodic trigger using IEP compare event</td>
</tr>
<tr>
    <td>TAMAGAWA_OPMODE_HOST_TRIGGER</td>
    <td>0x1U</td>
    <td>Operation mode: host trigger</td>
</tr>
<tr>
    <td>TAMAGAWA_OPMODE_PERIODIC_CAP</td>
    <td>0x2U</td>
    <td>Operation mode: periodic trigger using IEP capture event</td>
</tr>
<tr>
    <td>TAMAGAWA_ENABLE_CYCLE_TRIGGER</td>
    <td>0x1</td>
    <td>Enable cycle trigger for firmware</td>
</tr>
<tr>
    <td>TAMAGAWA_DISABLE_CYCLE_TRIGGER</td>
    <td>0x0</td>
    <td>Disable cycle trigger for firmware</td>
</tr>
<tr>
    <td>TAMAGAWA_FREQ_2_5_MHZ</td>
    <td>2500000U</td>
    <td>Allowed Tamagawa communication frequency (2.5 MHz)</td>
</tr>
<tr>
    <td>TAMAGAWA_FREQ_5_MHZ</td>
    <td>5000000U</td>
    <td>Allowed Tamagawa communication frequency (5 MHz)</td>
</tr>
<tr>
    <td>TAMAGAWA_DEFAULT_CMD_WAIT_DELAY_US</td>
    <td>100</td>
    <td>Default command wait delay in microseconds</td>
</tr>
<tr>
    <td>TAMAGAWA_DEFAULT_MAX_WAIT_LOOP_COUNT</td>
    <td>50U</td>
    <td>Default maximum wait loop count. Actual timeout = max_wait_loop_count × cmd_wait_delay_us (default: 5000 us)</td>
</tr>
<tr>
    <td>TAMAGAWA_IEP_MAX_CAP_EVENT</td>
    <td>0x8U</td>
    <td>Maximum IEP CAP event number</td>
</tr>
<tr>
    <td>TAMAGAWA_IEP_MAX_CMP_EVENT</td>
    <td>0x10U</td>
    <td>Maximum IEP CMP event number</td>
</tr>
</table>

### Renamed Macros

<table>
<tr>
    <th>Old Name</th>
    <th>New Name</th>
</tr>
<tr>
    <td>MAX_CHANNELS</td>
    <td>TAMAGAWA_MAX_CHANNELS_PER_SLICE</td>
</tr>
<tr>
    <td>MAX_EEPROM_ADDRESS</td>
    <td>TAMAGAWA_MAX_EEPROM_ADDRESS</td>
</tr>
<tr>
    <td>MAX_EEPROM_WRITE_DATA</td>
    <td>TAMAGAWA_MAX_EEPROM_WRITE_DATA</td>
</tr>
</table>

### Changed Macros

<table>
<tr>
    <th>Macro</th>
    <th>Old Value</th>
    <th>New Value</th>
    <th>Notes</th>
</tr>
<tr>
    <td>TAMAGAWA_RX_OVERSAMPLING_RATE</td>
    <td>8</td>
    <td>7</td>
    <td>Set to 7 to configure 8x oversampling (hardware expects a 0-based value)</td>
</tr>
</table>

### Removed Macros

<table>
<tr>
    <th>Macro
    <th>Old Value
    <th>Replacement
</tr>
<tr>
    <td>PRU_UART_CLOCK_SOURCE
    <td>0
    <td>Logic now internal to driver, use is_core_clk in attrs
</tr>
<tr>
    <td>PRU_CORE_CLOCK_SOURCE
    <td>1
    <td>Logic now internal to driver, use is_core_clk in attrs
</tr>
<tr>
    <td>TAMAGAWA_MULTI_CH0
    <td>(1 << 0)
    <td>Use channel mask directly (bit 0 = CH0)
</tr>
<tr>
    <td>TAMAGAWA_MULTI_CH1
    <td>(1 << 1)
    <td>Use channel mask directly (bit 1 = CH1)
</tr>
<tr>
    <td>TAMAGAWA_MULTI_CH2
    <td>(1 << 2)
    <td>Use channel mask directly (bit 2 = CH2)
</tr>
</table>

## Return Value Changes

Many APIs that previously returned void now return int32_t for proper error handling:

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

IEP event configuration can be done using tamagawa_config_iep_cmp_event() or tamagawa_config_iep_cap_event() APIs. These APIs are called in tamagawa_init() with the values configured in SysConfig.

### 2. Enhanced Timeout Handling

Configurable timeout parameters with clear timeout detection for APIs waiting for firmware to signal completion.

### 3. Load-Share Mode Support

Multi-PRU load-share mode allows distributing channels across multiple PRU cores:
- Mode controlled by attrs->mode and attrs->load_share_enabled
- Each channel can use separate PRU core
- Global reinit operations synchronized across PRUs

## Migration Examples

### Example 1: Basic Initialization

**Old Code:**
```c
struct tamagawa_priv *priv;
struct tamagawa_xchg *tamagawa_xchg;
void *pruss_cfg;
void *pruss_iep;

/* Manual setup of base addresses */
tamagawa_xchg = (struct tamagawa_xchg *)pru_dram_base;
pruss_cfg = (void *)ICSS_CFG_BASE;
pruss_iep = (void *)ICSS_IEP_BASE;

/* Initialize */
priv = tamagawa_init(tamagawa_xchg, pruss_cfg, pruss_iep, slice);

if (priv == NULL)
{
    /* Handle error */
}
```

**New Code:**
```c
tamagawa_handle handle;
tamagawa_params params;

/* Initialize params with defaults */
tamagawa_params_init(&params);
params.pruicss_handle = pruicssHandle;
/* update other params */

/* Initialize Tamagawa (CONFIG_TAMAGAWA0 is generated by SysConfig) */
handle = tamagawa_init(CONFIG_TAMAGAWA0, &params);
if (handle == NULL)
{
    /* Handle error */
}
```

### Example 2: Configuring Periodic Mode

**Old Code:**
```c
tamagawa_config_periodic_trigger(priv);
```

**New Code:**
```c
/* For CMP mode */
ret = tamagawa_config_periodic_trigger_cmp_mode(handle);
if (ret != SystemP_SUCCESS)
{
    /* Handle error */
}

/* OR for CAP mode */
ret = tamagawa_config_periodic_trigger_cap_mode(handle);

if (ret != SystemP_SUCCESS)
{
    /* Handle error */
}
```

### Example 3: Accessing Configuration

**Old Code:**
```c
/* Direct access to priv structure */
uint8_t channel = priv->channel;
```

**New Code:**
```c
const tamagawa_attrs *attrs = tamagawa_get_attrs(handle);
uint8_t channel_mask = attrs->channel_mask;

/* Access priv via getter */
tamagawa_priv *priv = tamagawa_get_priv(handle);
uint8_t channel = priv->channel;
```

## SysConfig Migration

### Adding Tamagawa Module in SysConfig

1. Open your project's `.syscfg` file
2. Add the Tamagawa module under Position Sense, if not added already
3. Configure the parameters as per the requirement. Refer \ref TAMAGAWA_SYSCONFIG_FEATURES for more details.

### Generated Code

SysConfig will generate:
- `tamagawa_attrs` structures with compile-time configuration
- `gTamagawaHandle` array with tamagawa_config entries
- Code is generated in `ti_drivers_config.c` and `ti_drivers_config.h`

## Common Migration Issues

1. **Compilation Errors with priv pointer**
   - Replace all `struct tamagawa_priv *priv` with `tamagawa_handle handle`
   - Use `tamagawa_get_priv()` when you need access to priv structure

2. **Missing channel mask parameter**
   - Channel mask is now configured in SysConfig, not passed to APIs
   - Remove channel mask parameters from command APIs
   - Access channel_mask via tamagawa_get_attrs()

3. **Timeout Errors**
   - Timeout detection is added in certain APIs waiting for firmware.
   - Adjust timeout parameters if needed via tamagawa_params before calling tamagawa_init().

4. **SysConfig Errors**
   - Ensure Tamagawa module is added and configured in `.syscfg` file
   - Review the configured parameters

5. **Periodic Mode Not Working**
   - Explicitly choose between CMP and CAP modes
   - Configure IEP events properly for each channel

## Additional Resources

- \ref EXAMPLE_MOTORCONTROL_TAMAGAWA
- \ref TAMAGAWA_API_MODULE
- \ref TAMAGAWA
- \ref TAMAGAWA_DESIGN

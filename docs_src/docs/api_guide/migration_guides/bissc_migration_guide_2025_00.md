\cond SOC_AM243X
# BiSS-C Encoder Driver Migration Guide (v11.00.00 to v2025.00.00) {#BISSC_MIGRATION_GUIDE_2025_00}
\endcond

\cond (SOC_AM263PX || SOC_AM261X)
# BiSS-C Encoder Driver Migration Guide (v10.02.00 to v2025.00.00) {#BISSC_MIGRATION_GUIDE_2025_00}
\endcond

[TOC]

## Introduction

\cond SOC_AM243X
This guide helps developers migrate BiSS-C encoder driver applications from Motor Control SDK v11.00.00 to v2025.00.00 and later versions. The driver underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes with CAP support, per-channel timeout configuration, and improved SysConfig integration.
\endcond
\cond (SOC_AM263PX || SOC_AM261X)
This guide helps developers migrate BiSS-C encoder driver applications from Motor Control SDK v10.02.00 to v2025.00.00 and later versions. The driver underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes with CAP support, per-channel timeout configuration, and improved SysConfig integration.
\endcond

## Major Architectural Changes

### 1. Handle-Based API Architecture

The entire BiSS-C driver API was refactored from pointer-based to an improved handle-based architecture.

<table>
<tr>
    <th>Old API</th>
    <th>New API</th>
    <th>Impact</th>
</tr>
<tr>
    <td>struct \ref bissc_priv *priv</td>
    <td>\ref bissc_handle handle</td>
    <td>All APIs use opaque handle instead of direct structure pointer</td>
</tr>
</table>

### 2. SysConfig-Based Initialization

The initialization process was completely redesigned to use SysConfig-generated parameters for compile-time variables and bissc_params structure for run-time variables.

<table>
<tr>
    <th>Old Initialization</th>
    <th>New Initialization</th>
</tr>
<tr>
    <td>
    <pre>
    struct bissc_priv *priv = bissc_init(
        pruHandle,     // PRUICSS handle
        slice,         // PRU slice
        frequency,     // Frequency
        core_clk,      // Core clock
        uart_clk,      // UART clock
        clk_src        // Clock source
    );
    </pre>
    </td>
    <td>
    <pre>
    bissc_params params;
    bissc_params_init(&params);
    // Update params
    bissc_handle handle = bissc_init(CONFIG_BISSC0, &params);
    </pre>
    </td>
</tr>
</table>

### 3. Per-Channel Encoder Timeout

The driver supports per-channel encoder timeout configuration instead of a single global timeout.

<table>
<tr>
    <th>Old Timeout</th>
    <th>New Timeout</th>
</tr>
<tr>
    <td>Single global delay_40us in firmware</td>
    <td>Per-channel configurable timeout via bissc_set_encoder_timeout()</td>
</tr>
</table>

## API Changes

Driver APIs use following validation approach now:
- **Handle validation**: All public APIs validate the handle parameter for NULL
- **Array bounds checking**: APIs with array parameters or index parameters perform bounds validation
- **Internal structure validation**: Each API validates the internal structure pointers it accesses (e.g., attrs, priv, pruicss_xchg, pruicss_handle) for NULL before dereferencing

### New APIs Added

<table>
<tr>
    <th>New API</th>
    <th>Description</th>
    <th>Usage</th>
</tr>
<tr>
    <td>bissc_params_init()</td>
    <td>Initialize params structure with defaults</td>
    <td>Use before bissc_init()</td>
</tr>
<tr>
    <td>bissc_get_attrs()</td>
    <td>Get pointer to compile-time attributes</td>
    <td>Access SysConfig-generated configuration</td>
</tr>
<tr>
    <td>bissc_get_priv()</td>
    <td>Get pointer to runtime private data</td>
    <td>Access runtime state information</td>
</tr>
<tr>
    <td>bissc_deinit()</td>
    <td>De-initialize BiSS-C interface</td>
    <td>Cleanup when done using driver</td>
</tr>
<tr>
    <td>bissc_set_encoder_timeout()</td>
    <td>Set per-channel encoder timeout in PRU cycles</td>
    <td>Configure timeout for each channel independently</td>
</tr>
<tr>
    <td>bissc_get_encoder_timeout()</td>
    <td>Get per-channel encoder timeout</td>
    <td>Read current timeout configuration</td>
</tr>
<tr>
    <td>bissc_config_periodic_trigger_cmp_mode()</td>
    <td>Configure IEP CMP event based periodic triggering mode</td>
    <td>Replaces bissc_config_periodic_trigger()</td>
</tr>
<tr>
    <td>bissc_config_periodic_trigger_cap_mode()</td>
    <td>Configure IEP CAP event based periodic triggering mode </td>
    <td>-</td>
</tr>
<tr>
    <td>bissc_config_iep_cap_event()</td>
    <td>Configure IEP CAP event number in firmware DMEM</td>
    <td>Set CAP event (0-7) for each channel</td>
</tr>
<tr>
    <td>bissc_config_iep_cmp_event()</td>
    <td>Configure IEP CMP event number in firmware DMEM</td>
    <td>Set CMP event (0-15) for each channel</td>
</tr>
<tr>
    <td>bissc_clock_config()</td>
    <td>This function updates the clock frequency, recalculates clock divisors, reinitializes the hardware with new clock settings, and waits for the firmware to complete encoder processing delay measurement.</td>
    <td>-</td>
</tr>
</table>

### APIs Modified

<table>
<tr>
    <th>API</th>
    <th>Key Changes</th>
    <th>Additional Details</th>
</tr>
<tr>
    <td>bissc_init()</td>
    <td>- Complete signature change<br>- 6 parameters to 2 parameters<br>- Returns <code>bissc_handle</code> instead of <code>priv*</code></td>
    <td>Uses SysConfig-generated index and params structure as arguments</td>
</tr>
<tr>
    <td>bissc_command_process()<br>bissc_command_wait()<br>bissc_get_pos()</td>
    <td>- <code>priv</code> to <code>handle</code></td>
    <td>Returns SystemP_TIMEOUT on timeout</td>
</tr>
<tr>
    <td>bissc_set_ctrl_cmd_and_process()</td>
    <td>- <code>priv</code> to <code>handle</code><br></td>
    <td>Returns SystemP_TIMEOUT on timeout and SystemP_FAILURE on validation failure </td>
</tr>
<tr>
    <td>bissc_command_send()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>Returns status code</td>
</tr>
<tr>
    <td>bissc_config_clock()<br>bissc_hw_init()<br>bissc_update_max_proc_delay()<br>bissc_get_enc_proc_delay()<br>bissc_config_host_trigger()<br>bissc_clear_data_len()<br>bissc_update_clock_freq()<br>bissc_enable_safety()<br>bissc_disable_safety()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>Returns status code with validation</td>
</tr>
<tr>
    <td>bissc_config_channel()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code><br>- <code>mask</code> type: <code>int32_t</code> to <code>uint8_t</code><br>- <code>totalch</code> renamed to <code>total_channels</code> with type: <code>int32_t</code> to <code>uint8_t</code></td>
    <td>Returns status code with validation</td>
</tr>
<tr>
    <td>bissc_wait_for_fw_initialization()</td>
    <td>- <code>priv</code> to <code>handle</code><br>- Removed <code>mask</code> parameter<br>- <code>timeout</code> renamed to <code>loop_count</code></td>
    <td>Channel mask used from attrs, and returns SystemP_TIMEOUT on timeout</td>
</tr>
<tr>
    <td>bissc_wait_measure_proc_delay()</td>
    <td>- <code>priv</code> to <code>handle</code><br>- <code>timeout</code> renamed to <code>loop_count</code></td>
    <td>More explicit parameter naming, and returns SystemP_TIMEOUT on timeout</td>
</tr>
<tr>
    <td>bissc_set_default_initialization()</td>
    <td>- Removed <code>icssgclk</code> parameter<br>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>Clock info used from attrs</td>
</tr>
<tr>
    <td>bissc_update_data_len()</td>
    <td>- <code>ch_num</code> type: <code>int32_t</code> to <code>uint32_t</code><br>- Added frame size validation<br>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>Validates total frame size <= 64 bits. Updates the data in PRU data memory only if all validations pass (no partial configuration on failure).</td>
</tr>
<tr>
    <td>bissc_calc_clock()</td>
    <td>- <code>priv</code> to <code>handle</code></td>
    <td>Parameter type change only</td>
</tr>
<tr>
    <td>bissc_generate_ctrl_cmd()</td>
    <td>- <code>ls_ch</code> type: <code>int8_t</code> to <code>uint8_t</code><br>- <code>ctrl_write_status</code> type: <code>uint32_t</code> to <code>uint8_t</code><br>- <code>priv</code> to <code>handle</code></td>
    <td>More appropriate data types, added parameter validation, and returns 0 on validation failure</td>
</tr>
<tr>
    <td>bissc_get_current_channel()</td>
    <td>- <code>priv</code> to <code>handle</code></td>
    <td>Returns 0 on validation error</td>
</tr>
<tr>
    <td>bissc_get_total_channels()</td>
    <td>- Renamed from <code>bissc_get_totalchannels()</code><br>- <code>priv</code> to <code>handle</code></td>
    <td>Function name changed to use underscore between "total" and "channels", and returns 0 on validation error</td>
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
    <td>bissc_config_periodic_trigger()</td>
    <td>bissc_config_periodic_trigger_cmp_mode() or bissc_config_periodic_trigger_cap_mode()</td>
    <td>Choose appropriate mode based on trigger source</td>
</tr>
<tr>
    <td>bissc_config_load_share()</td>
    <td rowspan="3">Integrated into bissc_init()</td>
    <td rowspan="3">Automatically configured based on SysConfig</td>
</tr>
<tr>
    <td>bissc_enable_load_share_mode()</td>
</tr>
<tr>
    <td>bissc_config_primary_core_mask()</td>
</tr>
<tr>
    <td>bissc_config_endat_mode()</td>
    <td rowspan="2">Made internal (static)</td>
    <td rowspan="2">No longer accessible from application</td>
</tr>
<tr>
    <td>bissc_config_clr_cfg0()</td>
</tr>
</table>

## Structure and Type Changes

### bissc_priv Structure

<table>
<tr>
    <th>Change Type</th>
    <th>Member</th>
    <th>Old Type/Location</th>
    <th>New Type/Location</th>
    <th>Notes</th>
</tr>
<tr>
    <td rowspan="3">Added</td>
    <td>is_open</td>
    <td>-</td>
    <td>uint8_t</td>
    <td>Track initialization state</td>
</tr>
<tr>
    <td>cmd_process_delay_us<br>fw_wait_delay_us<br>max_wait_loop_count</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>Timeout and delay configuration parameters</td>
</tr>
<tr>
    <td>pruicss_handle</td>
    <td>-</td>
    <td>PRUICSS_Handle</td>
    <td>Store PRUICSS handle</td>
</tr>
<tr>
    <td rowspan="2">Removed</td>
    <td>pruicss_slicex<br>load_share<br>totalchannels</td>
    <td>int32_t</td>
    <td>Moved to attrs</td>
    <td>Compile-time configuration</td>
</tr>
<tr>
    <td>pruicss_cfg<br>pruicss_iep<br>cmp3</td>
    <td>void*</td>
    <td>-</td>
    <td>Not needed</td>
</tr>
<tr>
    <td rowspan="1">Moved to attrs</td>
    <td>tx_rx_clock_source<br>core_clk_freq<br>uart_clk_freq</td>
    <td>In bissc_priv</td>
    <td>bissc_attrs->is_core_clk<br>bissc_attrs->core_clk_freq<br>bissc_attrs->uart_clk_freq<br>bissc_attrs->iep_base_addr<br>Removed (replaced by trigger_params)</td>
    <td>Compile-time configuration</td>
</tr>
<tr>
    <td rowspan="6">Type Changed</td>
    <td>data_len<br>single_turn_len<br>multi_turn_len<br>channel<br>num_encoders<br>pd_crc_err_cnt<br>ctrl_crc_err_cnt</td>
    <td>int32_t</td>
    <td>uint32_t</td>
    <td>More appropriate data type</td>
</tr>
<tr>
    <td>has_safety<br>sign_of_life_cnt<br>is_continuous_mode</td>
    <td>int32_t</td>
    <td>uint8_t</td>
    <td>More appropriate data type</td>
</tr>
<tr>
    <td>rcv_safety_crc<br>calc_safety_crc</td>
    <td>int32_t</td>
    <td>uint16_t</td>
    <td>16-bit CRC, uint16_t is sufficient</td>
</tr>
<tr>
    <td>raw_data</td>
    <td>int64_t</td>
    <td>uint64_t</td>
    <td>More appropriate data type</td>
</tr>
<tr>
    <td>ctrl_write_status</td>
    <td>int8_t</td>
    <td>uint8_t</td>
    <td>More appropriate data type</td>
</tr>
<tr>
    <td>baud_rate<br>core_clk_freq<br>uart_clk_freq</td>
    <td>In bissc_priv</td>
    <td>bissc_attrs (and copied to bissc_priv)</td>
    <td>Clock parameters also available in attrs</td>
</tr>
</table>

### bissc_pruicss_xchg Structure

<table>
<tr>
    <th>Change Type</th>
    <th>Member</th>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Notes</th>
</tr>
<tr>
    <td>Replaced</td>
    <td>delay_40us</td>
    <td>Single uint32_t</td>
    <td>encoder_timeout[BISSC_NUM_CH_PER_SLICE_MAX]</td>
    <td>Per-channel timeout configuration</td>
</tr>
<tr>
    <td>Renamed</td>
    <td>icssg_clk</td>
    <td>icssg_clk</td>
    <td>icss_clk</td>
    <td>More generic naming</td>
</tr>
<tr>
    <td rowspan="2">Added</td>
    <td>iep_base_address</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>IEP base for periodic mode</td>
</tr>
<tr>
    <td>trigger_params</td>
    <td>-</td>
    <td>bissc_periodic_trigger_cfg[BISSC_NUM_CH_PER_SLICE_MAX]</td>
    <td>Periodic trigger configuration parameters for each channel</td>
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
    <td>bissc_params</td>
    <td>Initialization parameters</td>
    <td>pruicss_handle, cmd_process_delay_us, fw_wait_delay_us, max_wait_loop_count</td>
</tr>
<tr>
    <td>bissc_attrs</td>
    <td>Compile-time attributes from SysConfig</td>
    <td>instance, mode, pruicss_instance, pruicss_slice, load_share_enabled, channel_mask, channel0/1/2_enabled, total_channels, baud_rate, core_clk_freq, uart_clk_freq, iep_clk_freq, is_core_clk, iep_instance, iep_cmp_event[BISSC_NUM_CH_PER_SLICE_MAX], iep_cap_event[BISSC_NUM_CH_PER_SLICE_MAX], iep_base_addr</td>
</tr>
<tr>
    <td>bissc_config</td>
    <td>Internal configuration structure</td>
    <td>priv (pointer to bissc_priv), attrs (pointer to bissc_attrs)</td>
</tr>
<tr>
    <td>bissc_periodic_trigger_cfg</td>
    <td>IEP event configuration for periodic trigger</td>
    <td>iep_cmp_event, iep_cap_event, iep_capture_reg</td>
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
    <td>struct bissc_clk_cfg<br>struct bissc_position_info<br>struct bissc_control_info<br>struct bissc_priv<br>struct bissc_pruicss_xchg</td>
    <td>typedef struct bissc_clk_cfg_s ... bissc_clk_cfg<br>typedef struct bissc_position_info_s ... bissc_position_info<br>typedef struct bissc_control_info_s ... bissc_control_info<br>typedef struct bissc_priv_s ... bissc_priv<br>typedef struct bissc_pruicss_xchg_s ... bissc_pruicss_xchg</td>
    <td>Converted to typedef</td>
</tr>
<tr>
    <td>struct raw_data<br>struct enc_len<br>struct pos_data_res<br>struct ctrl_res</td>
    <td>typedef struct bissc_raw_data_s ... bissc_raw_data<br>typedef struct bissc_enc_len_s ... bissc_enc_len<br>typedef struct bissc_pos_data_res_s ... bissc_pos_data_res<br>typedef struct bissc_ctrl_res_s ... bissc_ctrl_res</td>
    <td>Added bissc_ prefix to type name and converted to typedef</td>
</tr>
</table>

### New Types

<table>
<tr>
    <th>New Type</th>
    <th>Description</th>
</tr>
<tr>
    <td>typedef struct bissc_params_s ... bissc_params</td>
    <td>Initialization parameters structure</td>
</tr>
<tr>
    <td>typedef struct bissc_attrs_s ... bissc_attrs</td>
    <td>Compile-time attributes structure from SysConfig</td>
</tr>
<tr>
    <td>typedef struct bissc_config_s ... bissc_config</td>
    <td>Internal configuration structure</td>
</tr>
<tr>
    <td>typedef bissc_config *bissc_handle</td>
    <td>Opaque handle type for all APIs</td>
</tr>
<tr>
    <td>typedef struct bissc_periodic_trigger_cfg_s ... bissc_periodic_trigger_cfg</td>
    <td>IEP event configuration structure for periodic trigger</td>
</tr>
</table>

\attention **Additional Note on Interface Structures**: All hardcoded array sizes `[3]` in interface structures (bissc_pruicss_xchg, bissc_enc_len, bissc_pos_data_res, etc.) have been replaced with named constants `[BISSC_NUM_CH_PER_SLICE_MAX]` or `[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX]` for better code maintainability. This change is functionally equivalent (both evaluate to 3) but improves code readability and maintainability.

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
    <td>BISSC_DEFAULT_CMD_PROCESS_DELAY_US</td>
    <td>1000U</td>
    <td>Default command process delay</td>
</tr>
<tr>
    <td>BISSC_DEFAULT_FW_WAIT_DELAY_US</td>
    <td>1000U</td>
    <td>Default firmware wait delay</td>
</tr>
<tr>
    <td>BISSC_DEFAULT_ENCODER_TIMEOUT_US</td>
    <td>40U</td>
    <td>Default encoder timeout (microseconds)</td>
</tr>
<tr>
    <td>BISSC_OPMODE_PERIODIC_CMP</td>
    <td>0x0U</td>
    <td>IEP CMP-based periodic mode</td>
</tr>
<tr>
    <td>BISSC_OPMODE_HOST_TRIGGER</td>
    <td>0x1U</td>
    <td>Host-triggered mode</td>
</tr>
<tr>
    <td>BISSC_OPMODE_PERIODIC_CAP</td>
    <td>0x2U</td>
    <td>IEP CAP-based periodic mode</td>
</tr>
<tr>
    <td>BISSC_MAX_FRAME_SIZE</td>
    <td>64</td>
    <td>Maximum frame size in bits</td>
</tr>
<tr>
    <td>BISSC_IEP_MAX_CAP_EVENT</td>
    <td>0x8U</td>
    <td>Maximum IEP CAP event number</td>
</tr>
<tr>
    <td>BISSC_IEP_MAX_CMP_EVENT</td>
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
    <td>NUM_ED_CH_MAX</td>
    <td>BISSC_NUM_CH_PER_SLICE_MAX</td>
</tr>
<tr>
    <td>NUM_ENCODERS_MAX</td>
    <td>BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX</td>
</tr>
<tr>
    <td>BISSC_MAX_CYCLE_TIMEOUT</td>
    <td>BISSC_DEFAULT_MAX_WAIT_LOOP_COUNT</td>
</tr>
<tr>
    <td>PRU_CORE_CLK_FREQ_200MHZ</td>
    <td>BISSC_PRU_CORE_CLK_FREQ_200MHZ</td>
</tr>
<tr>
    <td>PRU_CORE_CLK_FREQ_300MHZ</td>
    <td>BISSC_PRU_CORE_CLK_FREQ_300MHZ</td>
</tr>
<tr>
    <td>PRU_UART_CLK_FREQ_160MHZ</td>
    <td>BISSC_PRU_UART_CLK_FREQ_160MHZ</td>
</tr>
<tr>
    <td>PRU_UART_CLK_FREQ_192MHZ</td>
    <td>BISSC_PRU_UART_CLK_FREQ_192MHZ</td>
</tr>
</table>

## Return Value Changes

Many APIs that previously returned void, return int32_t for proper error handling:

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
    <td>Validation failure, NULL pointer, invalid parameters, frame size exceeded</td>
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

IEP event configuration can be done using bissc_config_iep_cmp_event() or bissc_config_iep_cap_event() APIs. These APIs are called in bissc_init() with the values configured in SysConfig.

### 2. Per-Channel Encoder Timeout

Timeout can be configured individually for each encoder channel.

### 3. Enhanced Timeout Handling

Configurable timeout parameters with clear timeout detection for APIs waiting for firmware to signal completion.

### 4. Encoder Resolution Validation

The driver validates total frame size to ensure it doesn't exceed 64 bits:

- **Without Safety**: single_turn + multi_turn + 8 bits <= 64 bits
- **With Safety**: single_turn + multi_turn + 24 bits <= 64 bits

## Migration Examples

### Example 1: Basic Initialization

**Old Code:**
```c
struct bissc_priv *priv;
priv = bissc_init(pruicssHandle,
                  PRUICSS_PRU1,
                  5,    /* 5 MHz */
                  200,  /* Core clock 200 MHz */
                  192,  /* UART clock 192 MHz */
                  1);   /* Use core clock */

if (priv == NULL) {
    /* Handle error */
}

/* Configure channels and load share separately */
bissc_config_channel(priv, 0x7, 3);  /* All 3 channels */
bissc_config_load_share(priv, ...);
```

**New Code:**
```c
bissc_handle handle;
bissc_params params;

/* Initialize params with defaults */
bissc_params_init(&params);
params.pruicss_handle = pruicssHandle;
params.cmd_process_delay_us = 1000;
params.fw_wait_delay_us = 1000;
params.max_wait_loop_count = 5;

/* Initialize BiSS-C (CONFIG_BISSC0 is generated by SysConfig) */
handle = bissc_init(CONFIG_BISSC0, &params);
if (handle == NULL) {
    /* Handle error */
}
/* Channels and load share should be configured via SysConfig */
```

### Example 2: Getting Position Data

**Old Code:**
```c
bissc_command_send(priv);
int32_t ret = bissc_command_wait(priv);
if (ret != 0) {
    /* Handle error */
}
/* Access data directly from priv */
uint64_t position = priv->enc0_pos_data[0];
```

**New Code:**
```c
int32_t ret = bissc_command_send(handle);
if (ret != SystemP_SUCCESS) {
    /* Handle error */
}

ret = bissc_command_wait(handle);
if (ret == SystemP_TIMEOUT) {
    /* Handle timeout */
} else if (ret != SystemP_SUCCESS) {
    /* Handle other errors */
}

/* Access data via priv pointer */
bissc_priv *priv = bissc_get_priv(handle);
uint64_t position = priv->enc0_pos_data[0];
```

### Example 3: Configuring Periodic Mode

**Old Code:**
```c
bissc_config_periodic_trigger(priv);
```

**New Code:**
```c
/* For CMP mode (timer-based) */
ret = bissc_config_periodic_trigger_cmp_mode(handle);

if (ret != SystemP_SUCCESS) {
    /* Handle error */
}

/* OR for CAP mode (external signal) */
ret = bissc_config_periodic_trigger_cap_mode(handle);

if (ret != SystemP_SUCCESS) {
    /* Handle error */
}
```

### Example 4: Setting Per-Channel Timeout

**New Code:**
```c
/* Set different timeouts for each channel */
/* Timeout in PRU cycles (200MHz PRU clock = 5ns per cycle) */
ret = bissc_set_encoder_timeout(handle, 0, 8000);   /* Ch0: 40us */
ret = bissc_set_encoder_timeout(handle, 1, 12000);  /* Ch1: 60us */
ret = bissc_set_encoder_timeout(handle, 2, 10000);  /* Ch2: 50us */

if (ret != SystemP_SUCCESS) {
    /* Handle error */
}

/* Verify configuration */
uint32_t timeout_ch1 = bissc_get_encoder_timeout(handle, 1);
```

### Example 5: Accessing Configuration

**Old Code:**
```c
uint32_t total_channels = priv->totalchannels;
uint32_t channel = bissc_get_current_channel(priv, 0);
```

**New Code:**
```c
const bissc_attrs *attrs = bissc_get_attrs(handle);
uint32_t total_channels = attrs->total_channels;

uint32_t channel = bissc_get_current_channel(handle, 0);
```

## SysConfig Migration

### Adding BiSS-C Module in SysConfig

1. Open your project's `.syscfg` file
2. Add the BiSS-C module under Position Sense, if not added already
3. Configure the parameters as per the requirement. Refer \ref BISSC_SYSCONFIG_FEATURES for more details.

### Generated Code

SysConfig will generate:
- `bissc_attrs` structures with compile-time configuration
- `gBisscHandle` array with bissc_config entries
- Code is generated in `ti_drivers_config.c` and `ti_drivers_config.h`

## Common Migration Issues

1. **Compilation Errors with priv pointer**
- Replace all `struct bissc_priv *priv` with `bissc_handle handle`
- Use `bissc_get_priv()` when you need access to priv structure

2. **Missing bissc_get_totalchannels()**
- Replace `bissc_get_totalchannels()` with `bissc_get_total_channels()` (function renamed)

3. **Missing Load Share Configuration APIs**
- Load share configuration should be done via SysConfig
- No need to call bissc_config_load_share() or bissc_enable_load_share_mode() as bissc_init() will call it automatically

4. **Timeout Configuration**
- Use bissc_set_encoder_timeout() for per-channel timeout. Convert microseconds to PRU cycles based on PRU clock frequency.
- Also, timeout detection is added in certain APIs waiting for firmware. Adjust timeout parameters if needed via bissc_params before calling bissc_init().

5. **SysConfig Errors**
   - Ensure BiSS-C module is added and configured in `.syscfg` file
   - Review the configured parameters

6. **Periodic Mode Not Working**
- Explicitly choose between CMP and CAP modes
- Configure IEP events properly for each channel
- For CAP mode, ensure external trigger signal is properly routed

## Additional Resources

- \ref EXAMPLE_MOTORCONTROL_BISSC
- \ref BISSC_API_MODULE
- \ref BISS-C
- \ref BISSC_DESIGN
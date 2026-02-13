\cond SOC_AM243X
# Nikon Encoder Migration Guide (v11.00.00 to v2025.00.00) {#NIKON_MIGRATION_GUIDE_2025_00}
\endcond

\cond (SOC_AM263PX || SOC_AM261X)
# Nikon Encoder Migration Guide (v10.02.00 to v2025.00.00) {#NIKON_MIGRATION_GUIDE_2025_00}
\endcond

[TOC]

## Introduction

\cond SOC_AM243X
This guide helps developers migrate Nikon encoder applications from Motor Control SDK v11.00.00 to v2025.00.00 and later versions. The driver underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes, and improved SysConfig integration.
\endcond
\cond (SOC_AM263PX || SOC_AM261X)
This guide helps developers migrate Nikon encoder applications from Motor Control SDK v10.02.00 to v2025.00.00 and later versions. The driver underwent significant architectural changes including a move to handle-based APIs, enhanced periodic trigger modes, and improved SysConfig integration.
\endcond

## Major Architectural Changes

### 1. Handle-Based API Architecture

The entire Nikon driver API was refactored from pointer-based to an improved handle-based architecture.

<table>
<tr>
    <th>Old API</th>
    <th>New API</th>
    <th>Impact</th>
</tr>
<tr>
    <td>struct \ref nikon_priv *priv</td>
    <td>\ref nikon_handle handle</td>
    <td>All APIs use opaque handle instead of direct structure pointer</td>
</tr>
</table>

### 2. SysConfig-Based Initialization

The initialization process was completely redesigned to use SysConfig-generated parameters for compile-time variables and nikon_params structure for run-time variables.

<table>
<tr>
    <th>Old Initialization</th>
    <th>New Initialization</th>
</tr>
<tr>
    <td>
    <pre>
    struct nikon_priv *priv = nikon_init(
        pruHandle,     // PRUICSS handle
        slice,         // PRU slice
        freq,          // Frequency
        core_clk,      // Core clock
        uart_clk,      // UART clock
        clk_src,       // Clock source
        mask,          // Channel mask
        totalch,       // Total channels
        protocol       // Protocol version
    );
    </pre>
    </td>
    <td>
    <pre>
    nikon_params params;
    nikon_params_init(&params);
    params.pruicss_handle = pruHandle;

    nikon_handle handle = nikon_init(CONFIG_NIKON0, &params);
    </pre>
    </td>
</tr>
</table>

## API Changes

### New APIs Added

<table>
<tr>
    <th>New API</th>
    <th>Description</th>
    <th>Usage</th>
</tr>
<tr>
    <td>nikon_params_init()</td>
    <td>Initialize params structure with defaults</td>
    <td>Use before nikon_init()</td>
</tr>
<tr>
    <td>nikon_get_attrs()</td>
    <td>Get pointer to compile-time attributes</td>
    <td>Access SysConfig-generated configuration</td>
</tr>
<tr>
    <td>nikon_get_priv()</td>
    <td>Get pointer to runtime private data</td>
    <td>Access runtime state information</td>
</tr>
<tr>
    <td>nikon_deinit()</td>
    <td>De-initialize Nikon interface</td>
    <td>Cleanup when done using driver</td>
</tr>
<tr>
    <td>nikon_command_send()</td>
    <td>API to trigger Nikon command to PRU</td>
    <td>-</td>
</tr>
<tr>
    <td>nikon_command_process()</td>
    <td>Send command and wait for firmware acknowledgment</td>
    <td>Combines send and wait operations</td>
</tr>
<tr>
    <td>nikon_config_periodic_trigger_cap_mode()</td>
    <td>Configure IEP CAP event based periodic triggering</td>
    <td>Replaces nikon_config_periodic_trigger() for CAP mode</td>
</tr>
<tr>
    <td>nikon_config_periodic_trigger_cmp_mode()</td>
    <td>Configure IEP CMP event based periodic triggering</td>
    <td>Replaces nikon_config_periodic_trigger() for CMP mode</td>
</tr>
<tr>
    <td>nikon_config_iep_cap_event()</td>
    <td>Configure IEP CAP event number in firmware DMEM</td>
    <td>Set CAP event (0-7) for each channel</td>
</tr>
<tr>
    <td>nikon_config_iep_cmp_event()</td>
    <td>Configure IEP CMP event number in firmware DMEM</td>
    <td>Set CMP event (0-15) for each channel</td>
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
    <td>nikon_init()</td>
    <td>- Complete signature change<br>- 9 parameters to 2 parameters<br>- Returns <code>nikon_handle</code> instead of <code>priv*</code></td>
    <td>Uses SysConfig-generated index and params structure as arguments</td>
</tr>
<tr>
    <td>nikon_get_current_channel()</td>
    <td>- Returns via output parameter<br>- <code>priv</code> to <code>handle</code><br>- Added <code>uint32_t *channel</code> param</td>
    <td>Returns status, provides channel value via pointer</td>
</tr>
<tr>
    <td>nikon_command_wait()<br>nikon_get_pos()<br>nikon_wait_for_encoder_detection()</td>
    <td>- <code>priv</code> to <code>handle</code></td>
    <td>Returns SystemP_TIMEOUT on timeout</td>
</tr>
<tr>
    <td>nikon_calc_clock()</td>
    <td>- <code>priv</code> to <code>handle</code></td>
    <td>Parameter type change only</td>
</tr>
<tr>
    <td>nikon_command_send()<br>nikon_generate_cdf()<br>nikon_config_load_share()<br>nikon_config_host_trigger()<br>nikon_update_eeprom_addr()<br>nikon_update_eeprom_data()<br>nikon_update_eeprom_bank()<br>nikon_update_clock_freq()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>Returns status code</td>
</tr>
<tr>
    <td>nikon_update_enc_addr()<br>nikon_update_id_code()<br>nikon_update_velocity_coefficient()<br>nikon_update_enc_len()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- <code>priv</code> to <code>handle</code></td>
    <td>Validates parameter limits and returns status code</td>
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
    <td>nikon_config_periodic_trigger()</td>
    <td>nikon_config_periodic_trigger_cmp_mode() or nikon_config_periodic_trigger_cap_mode()</td>
    <td>Choose appropriate mode based on trigger source</td>
</tr>
<tr>
    <td>nikon_get_totalchannels()</td>
    <td>Use handle->attrs->total_channels</td>
    <td>Access via nikon_get_attrs()->total_channels</td>
</tr>
<tr>
    <td>nikon_config_clock()<br>nikon_calc_3bitcrc()<br>nikon_config_clr_cfg0()</td>
    <td>Made internal (static)</td>
    <td>No longer accessible from application</td>
</tr>
</table>

## Structure Changes

### Type Changes

<table>
<tr>
    <th>Type of Change</th>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Description</th>
</tr>
<tr>
    <td rowspan="3">Renamed Types</td>
    <td>enum cmd_code</td>
    <td>typedef enum nikon_cmd_e ... nikon_cmd</td>
    <td>- Renamed with nikon_ prefix and typedef<br>- Added START_CONTINUOUS_CAP_MODE at index 33<br>- Renamed START_CONTINUOUS_MODE to START_CONTINUOUS_CMP_MODE<br>- Removed explicit index assignment for CMD_27<br>
</tr>
<tr>
    <td>struct nikon_clk_cfg<br>struct nikon_priv<br>struct pos_data_res<br>struct nikon_pruicss_xchg</td>
    <td>typedef struct nikon_clk_cfg_s ... nikon_clk_cfg<br>typedef struct nikon_priv_s ... nikon_priv<br>typedef struct nikon_pos_data_res_s ... nikon_pos_data_res<br>typedef struct nikon_pruicss_xchg_s ... nikon_pruicss_xchg</td>
    <td>Converted to typedef (usage remains same, implementation changed to typedef pattern)</td>
</tr>
<tr>
    <td>struct raw_data<br>struct crc</td>
    <td>typedef struct nikon_raw_data_s ... nikon_raw_data<br>typedef struct nikon_crc_s ... nikon_crc</td>
    <td>Added nikon_ prefix to type name and changed to typedef</td>
</tr>
<tr>
    <td>struct pos_data_info<br>struct enc_info<br>struct pm_alm_bits<br>struct alm_bits</td>
    <td>typedef struct nikon_position_info_s ... nikon_position_info<br>typedef struct nikon_encoder_info_s ... nikon_encoder_info<br>typedef struct nikon_pm_alarm_bits_s ... nikon_pm_alarm_bits<br>typedef struct nikon_alarm_bits_s ... nikon_alarm_bits</td>
    <td>Type name completely changed and converted to typedef</td>
</tr>
</table>

### nikon_priv Structure

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
    <td>Initialization state flag (0 = closed, 1 = open)</td>
</tr>
<tr>
    <td>pruicss_handle</td>
    <td>-</td>
    <td>PRUICSS_Handle</td>
    <td>PRU-ICSS handle from params</td>
</tr>
<tr>
    <td>cmd_process_delay_us<br>fw_wait_delay_us<br>max_wait_loop_count</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>Timeout and delay configuration parameters</td>
</tr>
<tr>
    <td rowspan="3">Removed</td>
    <td>pruicss_cfg</td>
    <td>void *</td>
    <td>-</td>
    <td>PRU-ICSS CFG registers base offset</td>
</tr>
<tr>
    <td>pruicss_iep</td>
    <td>void *</td>
    <td>attrs->iep_base_addr</td>
    <td>ICSS IEP base address moved to attrs</td>
</tr>
<tr>
    <td>cmp3</td>
    <td>uint64_t</td>
    <td>-</td>
    <td>IEP CMP3 reg used in periodic trigger mode (replaced by trigger_params)</td>
</tr>
<tr>
    <td rowspan="1">Moved to attrs</td>
    <td>pruicss_slicex<br>load_share<br>totalchannels<br>protocol_version<br>tx_rx_clock_source<br>core_clk_freq<br>uart_clk_freq</td>
    <td>In nikon_priv</td>
    <td>nikon_attrs->pruicss_slice<br>nikon_attrs->load_share_enabled<br>nikon_attrs->total_channels<br>nikon_attrs->protocol_version<br>nikon_attrs->is_core_clk<br>nikon_attrs->core_clk_freq<br>nikon_attrs->uart_clk_freq</td>
    <td>Compile-time configuration</td>
</tr>
</table>

\cond (SOC_AM261X)
### nikon_position_info Structure

<table>
<tr>
    <th>Change Type</th>
    <th>Member</th>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Notes</th>
</tr>
<tr>
    <td rowspan="2">Type Changed</td>
    <td>velocity</td>
    <td>uint32_t</td>
    <td>int32_t</td>
    <td>Changed to signed</td>
</tr>
<tr>
    <td>acc</td>
    <td>uint16_t</td>
    <td>int16_t</td>
    <td>Changed to signed</td>
</tr>
</table>

\endcond

### nikon_pruicss_xchg Structure

<table>
<tr>
    <th>Change Type</th>
    <th>Member</th>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Notes</th>
</tr>
<tr>
    <td rowspan="2">Added</td>
    <td>iep_base_address</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>IEP register base address for periodic trigger mode</td>
</tr>
<tr>
    <td>trigger_params</td>
    <td>-</td>
    <td>nikon_periodic_trigger_cfg[NIKON_NUM_CH_PER_SLICE_MAX]</td>
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
    <td>nikon_params</td>
    <td>Initialization parameters</td>
    <td>pruicss_handle, cmd_process_delay_us, fw_wait_delay_us, max_wait_loop_count</td>
</tr>
<tr>
    <td>nikon_attrs</td>
    <td>Compile-time attributes from SysConfig</td>
    <td>protocol_version, PRU-ICSS attributes, channel configuration, clock settings, IEP event configuration</td>
</tr>
<tr>
    <td>nikon_periodic_trigger_cfg</td>
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
    <td>typedef struct nikon_params_s ... nikon_params</td>
    <td>Initialization parameters structure</td>
</tr>
<tr>
    <td>typedef struct nikon_attrs_s ... nikon_attrs</td>
    <td>Compile-time attributes structure from SysConfig</td>
</tr>
<tr>
    <td>typedef struct nikon_config_s ... nikon_config</td>
    <td>Internal configuration structure</td>
</tr>
<tr>
    <td>typedef nikon_config *nikon_handle</td>
    <td>Opaque handle type for all APIs</td>
</tr>
</table>

## Macro Changes

\note Only important macro changes are listed below.

<table>
<tr>
    <th>Old Macro</th>
    <th>New Macro</th>
    <th>Description</th>
</tr>
<tr>
    <td>NUM_ED_CH_MAX</td>
    <td>NIKON_NUM_CH_PER_SLICE_MAX</td>
    <td>Renamed for consistency with nikon_ prefix naming convention. Value remains the same (3). This affects array dimensions in multiple structures including nikon_raw_data, nikon_crc, nikon_pruicss_xchg, and nikon_priv.</td>
</tr>
<tr>
    <td>NIKON_CONFIG_PERIODIC_TRIGGER_MODE</td>
    <td>NIKON_CONFIG_PERIODIC_TRIGGER_CMP_MODE</td>
    <td>Renamed to explicitly indicate CMP mode</td>
</tr>
<tr>
    <td>-</td>
    <td>NIKON_CONFIG_PERIODIC_TRIGGER_CAP_MODE</td>
    <td>Added new mode for CAP-based triggering</td>
</tr>
<tr>
    <td>NIKON_30_MILLI_SEC_DELAY</td>
    <td>NIKON_EEPROM_WRITE_WAIT_US</td>
    <td>Renamed and value remains 30000us</td>
</tr>
<tr>
    <td>NIKON_MAX_CYCLE_TIMEOUT</td>
    <td>Removed</td>
    <td>This macro (value: 35) has been removed. Timeout configuration now done via nikon_params structure members (cmd_process_delay_us, max_wait_loop_count)</td>
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

### 1. Periodic Trigger Mode Updates

In addition to the existing CMP (compare) mode, a new CAP (capture) mode was added for periodic triggering:

- **CMP Mode**: Triggers based on IEP timer compare events
- **CAP Mode**: Triggers based on external signal capture events

```c
/* Old API (only one periodic mode) */
nikon_config_periodic_trigger(priv);

/* New API (choose appropriate mode) */
nikon_config_periodic_trigger_cmp_mode(handle);
/* OR */
nikon_config_periodic_trigger_cap_mode(handle);
```

IEP event configuration can be done using nikon_config_iep_cmp_event() or nikon_config_iep_cap_event() APIs. These APIs are called in nikon_init() with the values configured in SysConfig.

### 2. Enhanced Timeout Handling

Configurable timeout parameters with clear timeout detection:

```c
nikon_params params;
nikon_params_init(&params);
params.cmd_process_delay_us = 1000;    /* 1ms between polls */
params.max_wait_loop_count = 35;       /* 35ms total timeout */
params.pruicss_handle = pruHandle;

nikon_handle handle = nikon_init(CONFIG_NIKON0, &params);

/* Check for timeout */
int32_t ret = nikon_command_wait(handle);
if (ret == SystemP_TIMEOUT)
{
    /* Handle timeout condition */
}
```

## Migration Examples

### Example 1: Basic Initialization

**Old Code:**
```c
struct nikon_priv *priv;
priv = nikon_init(pruicssHandle,
                  0,
                  NIKON_FREQ_4MHZ,
                  PRU_CORE_CLK_FREQ_200MHZ,
                  PRU_UART_CLK_FREQ_192MHZ,
                  1,  /* Use core clock */
                  0x7, /* All channels */
                  3,   /* Total channels */
                  NIKON_PROTOCOL_V2_1);

if (priv == NULL)
{
    /* Handle error */
}
```

**New Code:**
```c
nikon_handle handle;
nikon_params params;

/* Initialize params with defaults */
nikon_params_init(&params);
params.pruicss_handle = pruicssHandle;

/* Initialize Nikon (CONFIG_NIKON0 is generated by SysConfig) */
handle = nikon_init(CONFIG_NIKON0, &params);
if (handle == NULL)
{
    /* Handle error */
}
```

### Example 2: Getting Position Data

**Old Code:**
```c
int32_t ret = nikon_get_pos(priv, CMD_0);
if (ret != 0)
{
    /* Handle error */
}
/* Access data directly from priv */
uint64_t position = priv->pos_data_info[0].abs[0];
```

**New Code:**
```c
int32_t ret = nikon_get_pos(handle, CMD_0);
if (ret == SystemP_TIMEOUT)
{
    /* Handle timeout */
}
else if (ret != SystemP_SUCCESS)
{
    /* Handle other errors */
}
/* Access data via priv pointer */
nikon_priv *priv = nikon_get_priv(handle);
uint64_t position = priv->pos_data_info[0].abs[0];
```

### Example 3: Configuring Periodic Mode

**Old Code:**
```c
nikon_config_periodic_trigger(priv);
```

**New Code:**
```c
/* For CMP mode */
ret = nikon_config_periodic_trigger_cmp_mode(handle);

/* OR for CAP mode */
ret = nikon_config_periodic_trigger_cap_mode(handle);

if (ret != SystemP_SUCCESS)
{
    /* Handle error */
}
```

### Example 4: Accessing Configuration

**Old Code:**
```c
uint32_t total_channels = nikon_get_totalchannels(priv);
uint32_t current_channel = nikon_get_current_channel(priv, 0);
```

**New Code:**
```c
const nikon_attrs *attrs = nikon_get_attrs(handle);
uint32_t total_channels = attrs->total_channels;

uint32_t current_channel;
ret = nikon_get_current_channel(handle, 0, &current_channel);
if (ret != SystemP_SUCCESS)
{
    /* Handle error */
}
```

## SysConfig Migration

### Adding Nikon Module in SysConfig

1. Open your project's `.syscfg` file
2. Add the Nikon module under Position Sense, if not added already
3. Configure the parameters as per the requirement

### Generated Code

SysConfig will generate:
- `nikon_attrs` structures with compile-time configuration
- `gNikonHandle` array with nikon_config entries
- Initialization code in `ti_drivers_config.c`

## Common Migration Issues

1. **Compilation Errors with priv pointer**
   - Replace all `struct nikon_priv *priv` with `nikon_handle handle`
   - Use `nikon_get_priv()` when you need access to priv structure

2. **Missing nikon_get_totalchannels()**
   - Use `nikon_get_attrs()` to access total_channels instead

3. **Timeout Errors**
   - Timeout detection is added in certain APIs
   - Adjust timeout parameters if needed via nikon_params

4. **SysConfig Errors**
   - Ensure Nikon module is added and configured in `.syscfg` file
   - Review the configured parameters

5. **Periodic Mode Not Working**
   - Explicitly choose between CMP and CAP modes
   - Configure IEP events properly for each channel

## Additional Resources

- \ref EXAMPLE_MOTORCONTROL_NIKON
- \ref NIKON_API_MODULE
- \ref NIKON
- \ref NIKON_DESIGN

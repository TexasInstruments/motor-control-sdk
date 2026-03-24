\cond SOC_AM243X
# HDSL Encoder Migration Guide (v11.00.00 to v2025.00.00) {#HDSL_MIGRATION_GUIDE_2025_00}
\endcond

\cond SOC_AM261X
# HDSL Encoder Migration Guide (v10.02.00 to v2025.00.00) {#HDSL_MIGRATION_GUIDE_2025_00}
\endcond

[TOC]

## Introduction

\cond SOC_AM243X
This guide helps developers migrate HDSL (Hiperface DSL) encoder applications from Motor Control SDK v11.00.00 to v2025.00.00 and later versions. The driver underwent significant architectural changes including a move to handle-based APIs, SysConfig-based initialization, and standardized return values with input validation.
\endcond
\cond SOC_AM261X
This guide helps developers migrate HDSL (Hiperface DSL) encoder applications from Motor Control SDK v10.02.00 to v2025.00.00 and later versions. The driver underwent significant architectural changes including a move to handle-based APIs, SysConfig-based initialization, and standardized return values with input validation.
\endcond

## Major Architectural Changes

### 1. Handle-Based API Architecture

The HDSL driver API was refactored from a monolithic configuration structure to a handle-based architecture with separation of compile-time and runtime data.

<table>
<tr>
    <th>Old API</th>
    <th>New API</th>
    <th>Impact</th>
</tr>
<tr>
    <td>HDSL_Config *hdslHandle</td>
    <td>\ref HDSL_Handle handle (HDSL_Object *)</td>
    <td>All APIs use handle with separate priv (runtime) and attrs (compile-time) structures</td>
</tr>
</table>

### 2. SysConfig-Based Initialization

The initialization process was completely redesigned to use SysConfig-generated parameters for compile-time variables and \ref HDSL_Params structure for run-time variables.

<table>
<tr>
    <th>Old Initialization</th>
    <th>New Initialization</th>
</tr>
<tr>
    <td>
    <pre>
    HDSL_Handle hdslHandle;
    hdslHandle = HDSL_open(
        pruicssHandle, /* PRUICSS handle */
        icssCore,      /* PRU core */
        pruMode        /* Load share mode */
    );
    </pre>
    </td>
    <td>
    <pre>
    HDSL_Params params;
    HDSL_params_init(&params);
    params.pruicss_handle = pruicssHandle;
    params.channel = 0;

    HDSL_Handle handle = HDSL_open(
        CONFIG_HDSL0, &params);
    </pre>
    </td>
</tr>
</table>

## API Changes

Driver APIs use following validation approach now:

**Handle Parameter Validation:**
- All public APIs validate the handle parameter for NULL
- Returns appropriate error value if handle is invalid:
  - Functions returning int32_t status: SystemP_FAILURE
  - Functions returning pointers: NULL
  - Functions returning void: early return
- This catches programming errors where uninitialized handles are used

**Array Bounds and Index Validation:**
- APIs with array parameters or index parameters perform bounds checking
- Examples: buff_off (0-7), byte (0-2), position_id (0-2)
- Prevents buffer overruns and out-of-bounds memory access

**Internal Structure Validation:**
- All APIs validate internal structure pointers before dereferencing them
- Each function validates only the pointers it uses
- Provides protection against NULL pointer dereferences

**Pointer Parameter Validation:**
- Output pointer parameters (position, data, copy_table) are checked for NULL
- Ensures safe dereferencing before writing output data

### New APIs Added

<table>
<tr>
    <th>New API</th>
    <th>Description</th>
    <th>Usage</th>
</tr>
<tr>
    <td>HDSL_params_init()</td>
    <td>Initialize params structure with defaults</td>
    <td>Use before HDSL_open()</td>
</tr>
<tr>
    <td>HDSL_close()</td>
    <td>Close HDSL instance</td>
    <td>Cleanup when done using driver</td>
</tr>
<tr>
    <td>HDSL_get_attrs()</td>
    <td>Get pointer to compile-time attributes</td>
    <td>Access SysConfig-generated configuration</td>
</tr>
<tr>
    <td>HDSL_get_priv()</td>
    <td>Get pointer to runtime private data</td>
    <td>Access runtime state information</td>
</tr>
<tr>
    <td>HDSL_hw_init()</td>
    <td>Initialize hardware (clock divider, GP MUX, load share)</td>
    <td>Call after HDSL_open(), replaces manual hardware setup in application and hdsl_enable_load_share_mode()</td>
</tr>
<tr>
    <td>HDSL_get_pc_long_msg_error()</td>
    <td>Check encoder error from long message operation</td>
    <td>Call after HDSL_write_pc_long_msg() or HDSL_read_pc_long_msg()</td>
</tr>
<tr>
    <td>HDSL_set_res()<br>HDSL_get_res()</td>
    <td>Set/Get single-turn resolution</td>
    <td>Replaces direct access to HDSL_Config->res</td>
</tr>
<tr>
    <td>HDSL_set_multi_turn()<br>HDSL_get_multi_turn()</td>
    <td>Set/Get multi-turn resolution</td>
    <td>Replaces direct access to HDSL_Config->multi_turn</td>
</tr>
<tr>
    <td>HDSL_set_mask()<br>HDSL_get_mask()</td>
    <td>Set/Get position data mask</td>
    <td>Replaces direct access to HDSL_Config->mask</td>
</tr>
<tr>
    <td>HDSL_set_pc_addr()</td>
    <td>Set parameters channel address registers</td>
    <td>Configure PC_ADD_L, PC_ADD_H, PC_OFF_L and PC_OFF_H registers</td>
</tr>
<tr>
    <td>HDSL_set_pc_ctrl()</td>
    <td>Set parameters channel control register</td>
    <td>Configure PC_CTRL register</td>
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
    <td>HDSL_open()</td>
    <td>- Complete signature change<br>- 3 parameters to 2 parameters<br>- Old: <code>HDSL_open(icssHandle, icssCore, pruMode)</code><br>- New: <code>HDSL_open(instance, params)</code></td>
    <td>Uses SysConfig-generated instance index and HDSL_Params structure. Hardware configuration (clock divider, GP MUX) moved to HDSL_hw_init().</td>
</tr>
<tr>
    <td>HDSL_get_pos()</td>
    <td>- Return type: <code>uint64_t</code> to <code>int32_t</code><br>- Added <code>uint64_t *position</code> output param<br>- <code>int position_id</code> to <code>uint32_t position_id</code></td>
    <td>Returns status code, position value via output pointer</td>
</tr>
<tr>
    <td>HDSL_get_qm()<br>HDSL_get_safe_events()<br>HDSL_get_sum()<br>HDSL_get_acc_err_cnt()<br>HDSL_get_rssi()<br>HDSL_get_sync_ctrl()<br>HDSL_get_master_qm()<br>HDSL_get_edges()<br>HDSL_get_delay()</td>
    <td>- Return type: <code>uint8_t</code> to <code>int32_t</code><br>- Added <code>uint8_t *</code> output param</td>
    <td>Returns status code, data value via output pointer</td>
</tr>
<tr>
    <td>HDSL_get_events()<br>HDSL_get_online_status_d()<br>HDSL_get_online_status_1()<br>HDSL_get_online_status_2()</td>
    <td>- Return type: <code>uint16_t</code> to <code>int32_t</code><br>- Added <code>uint16_t *</code> output param</td>
    <td>Returns status code, data value via output pointer</td>
</tr>
<tr>
    <td>HDSL_get_enc_id()</td>
    <td>- Return type: <code>uint8_t</code> to <code>int32_t</code><br>- Added <code>uint8_t *enc_id</code> output param<br>- <code>int byte</code> to <code>uint32_t byte</code></td>
    <td>Returns status code, encoder ID byte via output pointer</td>
</tr>
<tr>
    <td>HDSL_write_pc_buffer()</td>
    <td>- Return type: <code>void</code> to <code>int32_t</code></td>
    <td>- Validates buffer offset range <br>- Returns SystemP_SUCCESS or SystemP_FAILURE</td>
</tr>
<tr>
    <td>HDSL_read_pc_buffer()</td>
    <td>- Return type: <code>uint8_t</code> to <code>int32_t</code><br>- Added <code>uint8_t *data</code> output param</td>
    <td>- Validates buffer offset range <br>- Returns status code, buffer data via output pointer</td>
</tr>
<tr>
    <td>HDSL_set_sync_ctrl()<br>HDSL_generate_memory_image()</td>
    <td>- Return type: <code>void</code> to <code>int32_t</code></td>
    <td>Returns SystemP_SUCCESS or SystemP_FAILURE</td>
</tr>
<tr>
    <td>HDSL_write_pc_short_msg()<br>HDSL_read_pc_short_msg()</td>
    <td>- Added input validation<br>- Returns SystemP_FAILURE for invalid params</td>
    <td>Validates address range (0x00-0x7F)</td>
</tr>
<tr>
    <td>HDSL_write_pc_long_msg()<br>HDSL_read_pc_long_msg()</td>
    <td>- Parameter names: camelCase to snake_case<br>- <code>offsetEnable</code> to <code>offset_enable</code><br>- <code>addrType</code> to <code>addr_type</code><br>- Added input validation<br>- Returns SystemP_FAILURE for invalid params</td>
    <td>- Validates all parameter ranges<br>- Call HDSL_get_pc_long_msg_error() after HDSL_write_pc_long_msg() or HDSL_read_pc_long_msg() to check encoder error from long message operation</td>
</tr>
<tr>
    <td>HDSL_get_src_loc()</td>
    <td>- Return type: <code>void*</code> to <code>int32_t</code><br>- Added <code>void **src_loc</code> output param</td>
    <td>Returns status code, memory address via output pointer</td>
</tr>
<tr>
    <td>HDSL_get_length()</td>
    <td>- Return type: <code>uint32_t</code> to <code>int32_t</code><br>- Added <code>uint32_t *length</code> output param</td>
    <td>Returns status code, length via output pointer</td>
</tr>
<tr>
    <td>HDSL_config_copy_table()</td>
    <td>- <code>HDSL_CopyTable *</code> to <code>const HDSL_CopyTable *</code><br>- Structure members renamed (camelCase to snake_case)</td>
    <td>Added const qualifier and input validation for 16-bit address/size limits</td>
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
    <td>hdsl_enable_load_share_mode()</td>
    <td>HDSL_hw_init()</td>
    <td>Load share mode is now configured via SysConfig and enabled automatically in HDSL_hw_init()</td>
</tr>
<tr>
    <td>HDSL_config_channel_mask()</td>
    <td>Configured via SysConfig</td>
    <td>Channel mask is now set in SysConfig and applied automatically in HDSL_open()</td>
</tr>
</table>

## Structure and Type Changes

### Type Changes

<table>
<tr>
    <th>Type of Change</th>
    <th>Old Type</th>
    <th>New Type</th>
    <th>Description</th>
</tr>
<tr>
    <td>Replaced</td>
    <td>typedef struct HDSL_Config_s *HDSL_Handle</td>
    <td>typedef struct HDSL_Object_s *HDSL_Handle</td>
    <td>Handle now points to HDSL_Object instead of HDSL_Config</td>
</tr>
<tr>
    <td>Renamed</td>
    <td>enum { MENU_SAFE_POSITION, ... }</td>
    <td>typedef enum HDSL_MenuOption_e { ... } HDSL_MenuOption</td>
    <td>Converted to typedef with HDSL_ prefix</td>
</tr>
<tr>
    <td>Tagged</td>
    <td>typedef struct { ... } HDSL_Interface</td>
    <td>typedef struct HDSL_Interface_s { ... } HDSL_Interface</td>
    <td>Struct tag added to allow forward declaration using <code>struct HDSL_Interface_s</code></td>
</tr>
</table>

### HDSL_Config Structure (Removed)

The monolithic HDSL_Config structure has been replaced by the HDSL_Object, HDSL_Priv, and HDSL_Attrs structures.

<table>
<tr>
    <th>Old HDSL_Config Member</th>
    <th>New Location</th>
    <th>Notes</th>
</tr>
<tr>
    <td>icssHandle</td>
    <td>HDSL_Priv->pruicss_handle</td>
    <td>Passed via HDSL_Params and stored in priv during HDSL_open()</td>
</tr>
<tr>
    <td>icssCore</td>
    <td>HDSL_Attrs->pruicss_slice</td>
    <td>Configured via SysConfig</td>
</tr>
<tr>
    <td>baseMemAddr</td>
    <td>HDSL_Priv->base_mem_addr</td>
    <td>Calculated internally by HDSL_open()</td>
</tr>
<tr>
    <td>hdslInterface</td>
    <td>HDSL_Priv->hdsl_interface</td>
    <td>Set internally by HDSL_open()</td>
</tr>
<tr>
    <td>multi_turn</td>
    <td>HDSL_Priv->multi_turn</td>
    <td>Access via HDSL_set_multi_turn() / HDSL_get_multi_turn()</td>
</tr>
<tr>
    <td>res</td>
    <td>HDSL_Priv->res</td>
    <td>Access via HDSL_set_res() / HDSL_get_res()</td>
</tr>
<tr>
    <td>mask</td>
    <td>HDSL_Priv->mask</td>
    <td>Access via HDSL_set_mask() / HDSL_get_mask()</td>
</tr>
</table>

### HDSL_CopyTable Structure (Member Renames)

<table>
<tr>
    <th>Old Member Name</th>
    <th>New Member Name</th>
</tr>
<tr>
    <td>loadAddr1</td>
    <td>load_addr1</td>
</tr>
<tr>
    <td>runAddr1</td>
    <td>run_addr1</td>
</tr>
<tr>
    <td>loadAddr2</td>
    <td>load_addr2</td>
</tr>
<tr>
    <td>runAddr2</td>
    <td>run_addr2</td>
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
    <td>HDSL_Params</td>
    <td>Initialization parameters</td>
    <td>pruicss_handle, channel</td>
</tr>
<tr>
    <td>HDSL_Priv</td>
    <td>Per-channel runtime state</td>
    <td>is_open, base_mem_addr, hdsl_interface, multi_turn, res, mask, pruicss_handle</td>
</tr>
<tr>
    <td>HDSL_Attrs</td>
    <td>Compile-time attributes from SysConfig</td>
    <td>instance, pruicss_instance, pruicss_type, pruicss_slice, mode, load_share_enabled, channel_mask, channel0/1/2_enabled, total_channels, core_clk_freq, iep_clk_freq</td>
</tr>
<tr>
    <td>HDSL_Object</td>
    <td>Per-channel handle combining priv and attrs</td>
    <td>priv (HDSL_Priv *), attrs (const HDSL_Attrs *)</td>
</tr>
</table>

## Macro and Constant Changes

<table>
<tr>
    <th>Old Macro</th>
    <th>New Macro</th>
    <th>Description</th>
</tr>
<tr>
    <td>HDSL_MAX_CHANNELS</td>
    <td>HDSL_NUM_CH_PER_SLICE_MAX</td>
    <td>Renamed. Value remains the same (3).</td>
</tr>
<tr>
    <td>CHANNEL_0_ENABLED<br>CHANNEL_1_ENABLED<br>CHANNEL_2_ENABLED</td>
    <td>Removed</td>
    <td>Channel enable flags are now in HDSL_Attrs structure (channel0_enabled, channel1_enabled, channel2_enabled) and configured via SysConfig.</td>
</tr>
<tr>
    <td>MAX_WAIT</td>
    <td>Removed</td>
    <td>-</td>
</tr>
<tr>
    <td>SYNCEVENT_INTRTR_IN_27<br>SYNCEVT_RTR_SYNC28_EVT<br>SYNCEVT_RTR_SYNC29_EVT<br>SYNCEVT_RTR_SYNC30_EVT<br>SYNCEVT_RTR_SYNC31_EVT<br>SYNCEVT_RTR_SYNC10_EVT</td>
    <td>Removed</td>
    <td>Sync event routing macros moved to application</td>
</tr>
<tr>
    <td>-</td>
    <td>HDSL_OPERATIONAL_MODE_FREE_RUN<br>HDSL_OPERATIONAL_MODE_SYNC</td>
    <td>New macros for operational mode selection.</td>
</tr>
<tr>
    <td>-</td>
    <td>HDSL_PRU_ICSSG<br>HDSL_PRU_ICSSM</td>
    <td>New macros for PRU-ICSS type identification.</td>
</tr>
</table>

## PRU-ICSS Interrupt Controller (INTC) related changes

The file `source/position_sense/hdsl/include/pruss_intc_mapping.h` has been deleted. Applications that previously included this file must remove the `#include` directive.

INTC configuration is now handled via SysConfig.

## Return Value Changes

Many APIs that previously returned data values or void now return int32_t status codes:

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
    <td>Encoder not responding within timeout period (short/long message APIs)</td>
</tr>
</table>

## New Features

### 1. Hardware Initialization API

The new HDSL_hw_init() API consolidates hardware configuration:
- Clock divider configuration for HDSL
- GP MUX configuration
- Load share mode enablement
- Replaces manual hardware setup and hdsl_enable_load_share_mode()

### 2. Improved Input Validation

All public APIs now include comprehensive input validation:
- Handle validation (NULL check)
- Parameter range validation (address ranges, byte indices, position IDs)
- Internal structure pointer validation
- Early failure detection with SystemP_FAILURE return code

### 3. Long Message Error Reporting

New HDSL_get_pc_long_msg_error() API provides error information after long message operations.

## Migration Examples

### Example 1: Basic Initialization

**Old Code:**
```c
HDSL_Handle hdslHandle;
hdslHandle = HDSL_open(pruicssHandle, PRUICSS_PRU1, 0);
if (hdslHandle == NULL)
{
    /* Handle error */
}
HDSL_generate_memory_image(hdslHandle);
```

**New Code:**
```c
HDSL_Handle handle;
HDSL_Params params;

/* Initialize params with defaults */
HDSL_params_init(&params);
params.pruicss_handle = pruicssHandle;
params.channel = 0;

/* Initialize HDSL (CONFIG_HDSL0 is generated by SysConfig) */
handle = HDSL_open(CONFIG_HDSL0, &params);
if (handle == NULL)
{
    /* Handle error */
}

/* Initialize hardware (clock divider, GP MUX, load share) */
int32_t status = HDSL_hw_init(handle);
if (status != SystemP_SUCCESS)
{
    /* Handle error */
}

/* Generate lookup tables */
status = HDSL_generate_memory_image(handle);
if (status != SystemP_SUCCESS)
{
    /* Handle error */
}
```

### Example 2: Reading Position Data

**Old Code:**
```c
uint64_t position;
position = HDSL_get_pos(hdslHandle, 0);
/* No error checking available */
```

**New Code:**
```c
uint64_t position;
int32_t status = HDSL_get_pos(handle, 0, &position);
if (status != SystemP_SUCCESS)
{
    /* Handle error */
}
```

### Example 3: Accessing Configuration

**Old Code:**
```c
/* Direct structure member access */
hdslHandle->multi_turn = 4;
hdslHandle->res = 13;
hdslHandle->mask = (uint64_t)(1 << (hdslHandle->multi_turn + hdslHandle->res)) - 1;
```

**New Code:**
```c
/* Use getter/setter APIs */
HDSL_set_multi_turn(handle, 4);
HDSL_set_res(handle, 13);

uint32_t multi_turn, res;
HDSL_get_multi_turn(handle, &multi_turn);
HDSL_get_res(handle, &res);
HDSL_set_mask(handle, (uint64_t)(1 << (multi_turn + res)) - 1);
```

### Example 4: Load Share Mode Setup

**Old Code:**
```c
/* Manual load share enable */
hdsl_enable_load_share_mode(pruCfg, pruSlice);
HDSL_config_channel_mask(hdslHandle, channelMask);
```

**New Code:**
```c
/* Load share is configured in SysConfig and enabled in HDSL_hw_init() */
/* No manual calls needed - just call HDSL_hw_init() after HDSL_open() */
HDSL_hw_init(handle);
```

## SysConfig Migration

### Adding HDSL Module in SysConfig

1. Open your project's `.syscfg` file
2. Add the HDSL module under Position Sense, if not added already
3. Configure the parameters as per the requirement. Refer \ref HDSL_SYSCONFIG_FEATURES for more details.

### Generated Code

SysConfig will generate:
- `HDSL_Attrs gHdslAttrs[]` structures with compile-time configuration
- `HDSL_Priv gHdslPriv[][]` array for per-channel runtime state
- `HDSL_Object gHdslHandle[][]` array with per-channel handles
- Initialization code in `ti_drivers_config.c`

## Common Migration Issues

1. **Compilation errors with HDSL_Config**
   - The HDSL_Config structure has been removed
   - Replace all `HDSL_Config *` with `HDSL_Handle`
   - Use HDSL_get_priv() for runtime state access
   - Use HDSL_get_attrs() for compile-time configuration access

2. **Direct member access no longer works**
   - Use HDSL_set_res(), HDSL_set_multi_turn(), HDSL_set_mask() instead of direct writes
   - Use HDSL_get_res(), HDSL_get_multi_turn(), HDSL_get_mask() instead of direct reads

3. **Return value changes**
   - APIs now return status codes instead of data values
   - Update all call sites to use output parameters and check return status

4. **Load share mode configuration**
   - hdsl_enable_load_share_mode() and HDSL_config_channel_mask() are removed
   - Configure load share in SysConfig and call HDSL_hw_init() instead

5. **SysConfig errors**
   - Ensure HDSL module is added and configured in `.syscfg` file
\cond SOC_AM243X
   - 225 MHz supports single channel only
   - 300 MHz requires load share mode to be enabled
   - 300 MHz is needed when multiple channels are enabled
   - Channel 2 requires channel 0 to be enabled (TX_PRU overlay dependency)
\endcond

6. <b><code>\#include "pruss_intc_mapping.h"</code> causes compilation error</b>
   - The file `pruss_intc_mapping.h` has been deleted
   - Remove the `#include` directive from your application
   - INTC configuration is now handled via SysConfig

## Additional Resources

- \ref EXAMPLE_MOTORCONTROL_HDSL
- \ref HDSL_API_MODULE
- \ref HDSL
- \ref HDSL_DESIGN

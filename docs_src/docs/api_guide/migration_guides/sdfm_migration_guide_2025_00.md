\cond SOC_AM243X
# SDFM Current Sense Migration Guide (v11.00.00 to v2025.00.00) {#SDFM_MIGRATION_GUIDE_2025_00}
\endcond

[TOC]

## Introduction

This guide helps developers migrate SDFM current sense applications from Motor Control SDK v11.00.00 to v2025.00.00. The driver underwent significant architectural changes including a move to handle-based APIs, SysConfig-based initialization, error return codes, new configuration structures, and per-PRU-core trigger mode support.

## Major Architectural Changes

### 1. Handle-Based API Architecture

The SDFM driver API was refactored from pointer-based to an improved handle-based architecture.

<table>
<tr>
    <th>Old API</th>
    <th>New API</th>
    <th>Impact</th>
</tr>
<tr>
    <td>sdfm_handle (typedef SDFM *)</td>
    <td>\ref SDFM_Handle (typedef SDFM_Config *)</td>
    <td>All APIs use opaque handle instead of direct structure pointer</td>
</tr>
</table>

### 2. SysConfig-Based Initialization

The initialization process was completely redesigned to use SysConfig-generated parameters for compile-time variables and \ref SDFM_Params structure for run-time variables.

<table>
<tr>
    <th>Old Initialization</th>
    <th>New Initialization</th>
</tr>
<tr>
    <td>
    <pre>
    sdfm_handle hSdfm = SDFM_init(
        pruIcssHandle,  /* PRUICSS handle */
        pruId,          /* PRU slice ID */
        coreId          /* PRU core ID */
    );
    </pre>
    </td>
    <td>
    <pre>
    SDFM_Params params;
    SDFM_paramsInit(&params);
    params.pruicss_handle = pruIcssHandle;
    /* update other params */
    SDFM_Handle hSdfm = SDFM_init(
        CONFIG_SDFM0,   /* SysConfig index */
        &params          /* Initialization params */
    );
    </pre>
    </td>
</tr>
</table>

## API Changes

Driver APIs use following validation approach now:
- **Public API validation**: All public APIs validate the handle parameter for NULL and perform array bounds checking for index parameters (channel, pru_core)
- **Internal structure validation**: Each API validates the internal structure pointers it accesses (e.g., attrs, priv, sdfm_interface) for NULL before dereferencing

\note These changes are not mentioned in the "Additional Details" column of the \ref SDFM_MIGRATION_GUIDE_2025_00_APIS_MODIFIED section below. The \ref SDFM_MIGRATION_GUIDE_2025_00_APIS_MODIFIED section describes changes in addition to the points mentioned above.

### New APIs Added

<table>
<tr>
    <th>New API</th>
    <th>Description</th>
    <th>Usage</th>
</tr>
<tr>
    <td>SDFM_paramsInit()</td>
    <td>Initialize params structure with defaults</td>
    <td>Use before SDFM_init()</td>
</tr>
<tr>
    <td>SDFM_deinit()</td>
    <td>De-initialize SDFM instance</td>
    <td>Cleanup when done using driver</td>
</tr>
<tr>
    <td>SDFM_getAttrs()</td>
    <td>Get pointer to compile-time attributes</td>
    <td>Access SysConfig-generated configuration</td>
</tr>
<tr>
    <td>SDFM_getPriv()</td>
    <td>Get pointer to runtime private data</td>
    <td>Access runtime state information</td>
</tr>
<tr>
    <td>SDFM_enableTriggerModeForNormalCurrent()</td>
    <td>Enable IEP-based triggered sampling for a PRU core</td>
    <td>Replaces SDFM_enableContinuousNormalCurrent()</td>
</tr>
<tr>
    <td>SDFM_setSampleOutputInterfaceGlobalAddr()</td>
    <td>Set sample output buffer base address</td>
    <td>Configure output sample buffer location</td>
</tr>
<tr>
    <td>SDFM_selectIepCmpEvent()</td>
    <td>Select IEP comparator event for a PRU core</td>
    <td>Configure IEP CMP event (0-15) per core</td>
</tr>
<tr>
    <td>SDFM_configIepCmp0ToResetIep()</td>
    <td>Configure IEP CMP0 to reset IEP counter</td>
    <td>Synchronize IEP counter with EPWM cycle</td>
</tr>
</table>

### APIs Modified {#SDFM_MIGRATION_GUIDE_2025_00_APIS_MODIFIED}

<table>
<tr>
    <th>API</th>
    <th>Key Changes</th>
    <th>Additional Details</th>
</tr>
<tr>
    <td>SDFM_init()</td>
    <td>- Complete signature change<br>- 3 parameters to 2 parameters<br>- Returns \ref SDFM_Handle instead of sdfm_handle</td>
    <td>- Uses SysConfig-generated index and \ref SDFM_Params structure as arguments<br>- Validates parameter limits</td>
</tr>
<tr>
    <td>SDFM_enable()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- Added <code>pru_core</code> parameter</td>
    <td>- Validates parameter limits<br>- Returns status code<br>- Added pru_core (0=PRU, 1=RTU, 2=TXPRU) for per-core configuration<br>- Returns SystemP_TIMEOUT on firmware acknowledgment timeout</td>
</tr>
<tr>
    <td>SDFM_setSampleTriggerTime()<br>SDFM_enableDoubleSampling()<br>SDFM_disableDoubleSampling()<br>SDFM_enableSnoopBasedNC()<br>SDFM_disableSnoopBasedNC()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- Added <code>pru_core</code> parameter</td>
    <td>- Validates parameter limits<br>- Returns status code<br>- Added pru_core (0=PRU, 1=RTU, 2=TXPRU) for per-core configuration <br>- SDFM_enableSnoopBasedNC() and SDFM_disableSnoopBasedNC() were also renamed from SDFM_enableShadowRegBasedNC() and SDFM_disableShadowRegBasedNC() respectively</td>
</tr>
<tr>
    <td>SDFM_configEcap()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle</td>
    <td>- Validates parameter limits<br>- Returns status code<br>- ECAP register configuration is moved to driver from PRU firmware</td>
</tr>
<tr>
    <td>SDFM_setEnableChannel()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle</td>
    <td>- Validates parameter limits<br>- Returns status code</td>
</tr>
<tr>
    <td>SDFM_configIepCount()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- Parameter renamed: <code>epwm_out_freq</code> to <code>iep_reset_freq</code></td>
    <td>- Validates parameter limits<br>- Returns status code</td>
</tr>
<tr>
    <td>SDFM_setCompFilterOverSamplingRatio()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- Parameter renamed: <code>ch_id</code> to <code>channel</code></td>
    <td>- Validates parameter limits<br>- Returns status code<br>- Should only be called when snoop mode is enabled (via SDFM_enableSnoopBasedNC()) <br>- When snoop mode is disabled, the over-current filter OSR is set through SDFM_setFilterOverSamplingRatio()</td>
</tr>
<tr>
    <td>SDFM_configDataFilter()<br>SDFM_configComparatorGpioPins()<br>SDFM_disableZeroCrossDetection()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- Parameter renamed: <code>ch_id</code>/<code>ch</code>/<code>chNum</code> to <code>channel</code></td>
    <td>- Validates parameter limits<br>- Returns status code</td>
</tr>
<tr>
    <td>SDFM_selectClockSource()<br>SDFM_setClockInversion()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- Parameter renamed: <code>ch_id</code>/<code>ch</code>/<code>chNum</code> to <code>channel</code></td>
    <td>- Validates parameter limits<br>- Returns status code<br>- SDFM register configuration is moved to driver from PRU firmware</td>
</tr>
<tr>
    <td>SDFM_enableComparator()<br>SDFM_disableComparator()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- Parameter renamed: <code>ch_id</code>/<code>ch</code>/<code>chNum</code> to <code>channel</code></td>
    <td>- Validates parameter limits<br>- Returns status code<br>- PWM Trip configuration is moved to driver from PRU firmware</td>
</tr>
<tr>
    <td>SDFM_setFilterOverSamplingRatio()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- New <code>channel</code> parameter added (was not present in old API)</td>
    <td>- Previously had no channel parameter <br>- Now configures OSR per-channel <br>- Behavior depends on snoop mode <br>- When snoop mode is enabled (via SDFM_enableSnoopBasedNC()), configures IEP count for normal current sampling <br>- When snoop mode is disabled, configures SD HW OSR directly</td>
</tr>
<tr>
    <td>SDFM_enableZeroCrossDetection()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- Parameter renamed:<code>chNum</code> to <code>channel</code><br>- Parameter renamed: <code>zcThr</code> to <code>zc_thr</code></td>
    <td>- Validates parameter limits<br>- Returns status code</td>
</tr>
<tr>
    <td>SDFM_setCompFilterThresholds()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- Parameter <code>uint32_t *thresholdParms</code> changed to \ref SDFM_ThresholdConfig <br>- Parameter renamed: <code>ch_id</code> to <code>channel</code></td>
    <td>- Validates parameter limits<br>- Returns status code<br>- Uses structured threshold configuration instead of raw pointer</td>
</tr>
<tr>
    <td>SDFM_configFastDetect()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- Parameter <code>uint8_t *fdParms</code> changed to \ref SDFM_FastDetectConfig <br>- Parameter renamed: <code>ch</code> to <code>channel</code></td>
    <td>- Validates parameter limits<br>- Returns status code<br>- Uses structured fast detect configuration instead of raw pointer</td>
</tr>
<tr>
    <td>SDFM_measureClockPhaseDelay()</td>
    <td>- <code>void</code> to <code>int32_t</code> return<br>- sdfm_handle to \ref SDFM_Handle <br>- Added <code>channel</code> parameter<br>- Parameter renamed: <code>clkEd</code> to <code>clk_edg</code></td>
    <td>- Validates parameter limits<br>- Returns SystemP_TIMEOUT on firmware acknowledgment timeout</td>
</tr>
<tr>
    <td>SDFM_getClockPhaseDelay()</td>
    <td>- sdfm_handle to \ref SDFM_Handle <br>- Added <code>channel</code> parameter</td>
    <td>- Validates parameter limits</td>
</tr>
<tr>
    <td>SDFM_getHighThresholdStatus()<br>SDFM_getLowThresholdStatus()<br>SDFM_getZeroCrossThresholdStatus()</td>
    <td>- Return type <code>uint8_t</code> changed to <code>int32_t</code><br>- sdfm_handle to \ref SDFM_Handle <br>- Parameter renamed: <code>chNum</code> to <code>channel</code></td>
    <td>- Returns SystemP_FAILURE on invalid parameters</td>
</tr>
<tr>
    <td>SDFM_configIepSyncMode()</td>
    <td>- sdfm_handle to \ref SDFM_Handle <br>- Parameter names: camelCase to snake_case</td>
    <td>-</td>
</tr>
<tr>
    <td>SDFM_enableEpwmSync()<br>SDFM_disableEpwmSync()</td>
    <td>- sdfm_handle to \ref SDFM_Handle <br>- Parameter name: epwmIns to epwm_ins</td>
    <td>-</td>
</tr>
<tr>
    <td>SDFM_getFilterData()</td>
    <td>- sdfm_handle to \ref SDFM_Handle <br>- Parameter renamed: <code>ch</code> to <code>channel</code></td>
    <td>- Validates parameter limits</td>
</tr>
<tr>
    <td>SDFM_getFastDetectErrorStatus()<br>SDFM_clearPwmTripStatus()<br>SDFM_clearOverCurrentError()</td>
    <td>- sdfm_handle to \ref SDFM_Handle <br>- Parameter renamed: <code>chNum</code> to <code>channel</code></td>
    <td>-</td>
</tr>
<tr>
    <td>SDFM_getFirmwareVersion()<br>SDFM_enableIep()<br>SDFM_configSync1Delay()<br>SDFM_configClockFromGPO1()</td>
    <td>- sdfm_handle to \ref SDFM_Handle</td>
    <td>-</td>
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
    <td>SDFM_enableContinuousNormalCurrent()</td>
    <td>SDFM_enableTriggerModeForNormalCurrent()</td>
    <td>Use new API with pru_core parameter for per-core trigger mode configuration</td>
</tr>
<tr>
    <td>SDFM_enableLoadShareMode()</td>
    <td>SysConfig configuration</td>
    <td>Load share mode is now configured through SysConfig (load_share_enabled in \ref SDFM_Attrs)</td>
</tr>
<tr>
    <td>SDFM_enableShadowRegBasedNC()</td>
    <td>SDFM_enableSnoopBasedNC()</td>
    <td>Renamed to reflect snoop-mode terminology; also added pru_core parameter</td>
</tr>
<tr>
    <td>SDFM_disableShadowRegBasedNC()</td>
    <td>SDFM_disableSnoopBasedNC()</td>
    <td>Renamed to reflect snoop-mode terminology; also added pru_core parameter</td>
</tr>
</table>

## Structure and Type Changes

### Removed Structures {#SDFM_MIGRATION_GUIDE_2025_00_REMOVED_STRUCTURES}

<table>
<tr>
    <th>Old Structure</th>
    <th>Old Typedef</th>
    <th>Replacement Structure</th>
    <th>Notes</th>
</tr>
<tr>
    <td>SDFM (main driver struct)</td>
    <td><code>typedef struct SDFM_s SDFM</code><br><code>typedef SDFM *sdfm_handle</code></td>
    <td>\ref SDFM_Config (\ref SDFM_Attrs + \ref SDFM_Priv)</td>
    <td>Split into compile-time attributes from SysConfig and private data structures. <code>iepInc</code>, <code>pruicssCfg</code>, <code>pruicssIep</code> and <code>pruicssEcap</code> fields are removed. <code>sdfm_handle</code> renamed to \ref SDFM_Handle.</td>
</tr>
<tr>
    <td>SDFM_Cfg</td>
    <td><code>typedef struct SDFM_Cfg_s SDFM_Cfg</code></td>
    <td>Fields distributed into \ref SDFM_ChannelConfig and \ref SDFM_ChannelAttrs</td>
    <td>Reorganized per-channel configuration</td>
</tr>
<tr>
    <td>SDFM_Ctrl</td>
    <td><code>typedef struct SDFM_Ctrl_s SDFM_Ctrl</code></td>
    <td>\ref SDFM_Control</td>
    <td>Per-PRU-core control with array of 3 entries. Fields are renamed.</td>
</tr>
<tr>
    <td>SDFM_ChCtrl</td>
    <td><code>typedef struct SDFM_ChCtrl_s SDFM_ChCtrl</code></td>
    <td>Fields distributed into \ref SDFM_ChannelConfig, \ref SDFM_ChannelAttrs and \ref SDFM_Attrs</td>
    <td>Reorganized per-channel configuration</td>
</tr>
<tr>
    <td>SDFM_CfgSdClk</td>
    <td><code>typedef struct SDFM_CfgSdClk_s SDFM_CfgSdClk</code></td>
    <td>Fields moved to \ref SDFM_ChannelConfig and \ref SDFM_ChannelAttrs</td>
    <td>Reorganized per-channel clock configuration. <code>sd_prd_clocks</code> field is removed.</td>
</tr>
<tr>
    <td>SDFM_CfgIep</td>
    <td><code>typedef struct SDFM_CfgIep_s SDFM_CfgIep</code></td>
    <td>Fields distributed into \ref SDFM_CfgTrigger</td>
    <td>IEP configuration merged with trigger configuration. <code>iep_inc_value</code> field is removed.</td>
</tr>
<tr>
    <td>SDFM_ClkSourceParms</td>
    <td><code>typedef struct SDFM_ClkSourceParms_s SDFM_ClkSourceParms</code></td>
    <td>Fields moved to \ref SDFM_ChannelConfig and \ref SDFM_ChannelAttrs</td>
    <td>Reorganized per-channel configuration</td>
</tr>
</table>

### New Structures

<table>
<tr>
    <th>New Structure</th>
    <th>Typedef</th>
    <th>Purpose</th>
    <th>Key Members</th>
</tr>
<tr>
    <td>\ref SDFM_Config</td>
    <td><code>typedef struct sdfm_config_s SDFM_Config</code><br><code>typedef SDFM_Config *SDFM_Handle</code></td>
    <td>Driver configuration handle</td>
    <td>priv (pointer to SDFM_Priv), attrs (pointer to SDFM_Attrs)</td>
</tr>
<tr>
    <td>\ref SDFM_Priv</td>
    <td><code>typedef struct SDFM_Priv_s SDFM_Priv</code></td>
    <td>Runtime private data</td>
    <td>is_open, sdfm_interface, sampleOutputInterface, pruicss_handle, pwm_handle</td>
</tr>
<tr>
    <td>\ref SDFM_Attrs</td>
    <td><code>typedef struct SDFM_Attrs_s SDFM_Attrs</code></td>
    <td>Compile-time attributes from SysConfig</td>
    <td>PRU instance, channel configuration, clock frequencies, operation mode settings</td>
</tr>
<tr>
    <td>\ref SDFM_ChannelConfig</td>
    <td><code>typedef struct SDFM_ChannelConfig_s SDFM_ChannelConfig</code></td>
    <td>Per-channel runtime configuration in DMEM</td>
    <td>ch_id, enabled, filter_type, OSR, threshold, GPIO, clock settings</td>
</tr>
<tr>
    <td>\ref SDFM_ChannelAttrs</td>
    <td><code>typedef struct SDFM_ChannelAttrs_s SDFM_ChannelAttrs</code></td>
    <td>Per-channel compile-time attributes</td>
    <td>ch_id, enabled, filter_type, OSR, thresholds, clock settings</td>
</tr>
<tr>
    <td>\ref SDFM_PruCoreAttrs</td>
    <td><code>typedef struct SDFM_PruCoreAttrs_s SDFM_PruCoreAttrs</code></td>
    <td>Per-PRU-core compile-time attributes</td>
    <td>enable_trigger_mode, en_double_nc_sampling, trigger times, iep_cmp_event</td>
</tr>
<tr>
    <td>\ref SDFM_Control</td>
    <td><code>typedef struct SDFM_Control_s SDFM_Control</code></td>
    <td>Per-PRU-core control settings</td>
    <td>enable, enable_ack, enable_snoop_nc</td>
</tr>
<tr>
    <td>\ref SDFM_Params</td>
    <td><code>typedef struct SDFM_Params_s SDFM_Params</code></td>
    <td>Initialization parameters</td>
    <td>pruicss_handle, pwm_handle, sample_base_addr</td>
</tr>
<tr>
    <td>\ref SDFM_ThresholdConfig</td>
    <td><code>typedef struct SDFM_ThresholdConfig_s SDFM_ThresholdConfig</code></td>
    <td>Threshold configuration for SDFM_setCompFilterThresholds()</td>
    <td>high_threshold, low_threshold</td>
</tr>
<tr>
    <td>\ref SDFM_FastDetectConfig</td>
    <td><code>typedef struct SDFM_FastDetectConfig_s SDFM_FastDetectConfig</code></td>
    <td>Fast detect configuration for SDFM_configFastDetect()</td>
    <td>fd_enable, fd_window_size, fd_zero_max, fd_zero_min</td>
</tr>
</table>

### New Enumerations

<table>
<tr>
    <th>Enumeration</th>
    <th>Purpose</th>
    <th>Values</th>
</tr>
<tr>
    <td>\ref SDFM_ClockSource</td>
    <td>Clock source enumeration</td>
    <td>SDFM_CLOCK_SOURCE_IEP, SDFM_CLOCK_SOURCE_ECAP, SDFM_CLOCK_SOURCE_PRU_GPIO1, SDFM_EXTERNAL_CLOCK_SRC</td>
</tr>
</table>

### SDFM_CfgTrigger Structure Changes

<table>
<tr>
    <th>Change Type</th>
    <th>Member</th>
    <th>Old</th>
    <th>New</th>
    <th>Notes</th>
</tr>
<tr>
    <td>Renamed</td>
    <td>en_continuous_mode</td>
    <td>uint8_t en_continuous_mode</td>
    <td>uint8_t enable_trigger_mode</td>
    <td>Reflects new trigger mode semantics</td>
</tr>
<tr>
    <td>Type Changed</td>
    <td>nc_prd_iep_cnt</td>
    <td>uint32_t</td>
    <td>uint64_t</td>
    <td>Extended range for IEP count</td>
</tr>
<tr>
    <td rowspan="6">Added</td>
    <td>max_iep_cnt_per_epwm_prd</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>Max IEP counts in one EPWM period</td>
</tr>
<tr>
    <td>iep_cmp_event</td>
    <td>-</td>
    <td>uint8_t</td>
    <td>CMP event number for trigger</td>
</tr>
<tr>
    <td>iep_cmp_event_reg</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>CMP event register address</td>
</tr>
<tr>
    <td>iep_cmp_status_reg</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>IEP CMP status register address</td>
</tr>
<tr>
    <td>sample_buff_base_addr</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>Host output sample buffer base address</td>
</tr>
<tr>
    <td>reserved</td>
    <td>-</td>
    <td>uint32_t</td>
    <td>Reserved field</td>
</tr>
</table>

### SDFM_Interface Structure Changes

The \ref SDFM_Interface structure is retained but significantly reorganized to support per-PRU-core control and all 9 channels.

<table>
<tr>
    <th>Change Type</th>
    <th>Old Member</th>
    <th>New Member</th>
    <th>Notes</th>
</tr>
<tr>
    <td>Replaced</td>
    <td>SDFM_Ctrl sdfm_ctrl</td>
    <td>SDFM_Control control[3]</td>
    <td>Single control struct replaced by array of 3 (one per PRU core: PRU, RTU-PRU, TX-PRU). Fields renamed: sdfm_en→enable, sdfm_en_ack→enable_ack, sdfm_en_shadow_nc→enable_snoop_nc. Field sdfm_pru_id is removed.</td>
</tr>
<tr>
    <td rowspan="4">Removed</td>
    <td>SDFM_CfgIep sdfm_cfg_iep_ptr</td>
    <td rowspan="4">-</td>
    <td rowspan="4">See \ref SDFM_MIGRATION_GUIDE_2025_00_REMOVED_STRUCTURES</td>
</tr>
<tr>
    <td>SDFM_CfgSdClk sd_clk</td>
</tr>
<tr>
    <td>SDFM_ChCtrl sdfm_ch_ctrl</td>
</tr>
<tr>
    <td>SDFM_Cfg sdfm_cfg_ptr[3]</td>
</tr>
<tr>
    <td>Replaced</td>
    <td>SDFM_CfgTrigger sdfm_cfg_trigger</td>
    <td>SDFM_CfgTrigger trigger_config[3]</td>
    <td>Single trigger configuration replaced by array of 3 (one per PRU core) for independent per-core trigger configuration</td>
</tr>
<tr>
    <td>Removed</td>
    <td>volatile uint32_t sampleBufferBaseAdd</td>
    <td>-</td>
    <td>Moved into SDFM_CfgTrigger.sample_buff_base_addr (per PRU core)</td>
</tr>
<tr>
    <td rowspan="2">Added</td>
    <td rowspan="2">-</td>
    <td>volatile uint16_t active_channels_mask</td>
    <td>Bitmask of active/enabled channels</td>
</tr>
<tr>
    <td>SDFM_ChannelConfig channels[9]</td>
    <td>Per-channel runtime configuration</td>
</tr>
</table>

### SDFM_ThresholdParms Field Renames

The \ref SDFM_ThresholdParms structure field names changed from camelCase to snake_case.

<table>
<tr>
    <th>Old Field Name</th>
    <th>New Field Name</th>
</tr>
<tr>
    <td>highThStatus</td>
    <td>high_th_status</td>
</tr>
<tr>
    <td>lowThStatus</td>
    <td>low_th_status</td>
</tr>
<tr>
    <td>zeroCrossEn</td>
    <td>en_zero_cross</td>
</tr>
<tr>
    <td>zeroCrossThstatus</td>
    <td>zero_cross_th_status</td>
</tr>
<tr>
    <td>zeroCrossTh</td>
    <td>zero_cross_threshold</td>
</tr>
</table>

## Macro and Constant Changes

\note Only backward-compatibility-breaking macro changes are listed below.

### New Macros

<table>
<tr>
    <th>Macro</th>
    <th>Value</th>
    <th>Description</th>
</tr>
<tr>
    <td>SDFM_NUM_OF_CH_PER_PRU_SLICE</td>
    <td>9</td>
    <td>Total number of SDFM channels per PRU slice</td>
</tr>
<tr>
    <td>NUM_OF_PRU_CORE_PER_PRU_SLICE</td>
    <td>3</td>
    <td>Number of PRU cores per slice (PRU, RTU, TX)</td>
</tr>
<tr>
    <td>SDFM_PRU_CORE_INDEX</td>
    <td>0U</td>
    <td>PRU core index</td>
</tr>
<tr>
    <td>SDFM_RTUPRU_CORE_INDEX</td>
    <td>1U</td>
    <td>RTU PRU core index</td>
</tr>
<tr>
    <td>SDFM_TXPRU_CORE_INDEX</td>
    <td>2U</td>
    <td>TX PRU core index</td>
</tr>
<tr>
    <td>SDFM_OSR_MIN</td>
    <td>4U</td>
    <td>Minimum user-facing OSR value</td>
</tr>
<tr>
    <td>SDFM_OSR_MAX</td>
    <td>256U</td>
    <td>Maximum user-facing OSR value</td>
</tr>
<tr>
    <td>SDFM_IEP_CMP_EVENT_MAX</td>
    <td>15U</td>
    <td>Maximum IEP compare event number</td>
</tr>
<tr>
    <td>SDFM_DEFAULT_MAX_WAIT_LOOP_COUNT</td>
    <td>5U</td>
    <td>Maximum wait loop count for firmware acknowledgment timeout</td>
</tr>
<tr>
    <td>SDFM_DEFAULT_FW_WAIT_DELAY_US</td>
    <td>1000U</td>
    <td>Delay in microseconds between firmware acknowledgment checks</td>
</tr>
</table>

### Removed Macros

<table>
<tr>
    <th>Name</th>
    <th>Details</th>
</tr>
<tr>
    <td>NUM_PRU</td>
    <td>Use NUM_OF_PRU_CORE_PER_PRU_SLICE instead</td>
</tr>
<tr>
    <td>PRU_ID_0, PRU_ID_1</td>
    <td>-</td>
</tr>
<tr>
    <td>NUM_SD_CH</td>
    <td>Use SDFM_NUM_OF_CH_PER_PRU_SLICE instead</td>
</tr>
<tr>
    <td>SDFM_EVT</td>
    <td>Event configuration handled internally by driver</td>
</tr>
<tr>
    <td>PRUx_DMEM_BASE_ADD<br>RTUx_DMEM_BASE_ADD<br>TXPRUx_DMEM_BASE_ADD</td>
    <td>DMEM base addresses are managed internally by driver</td>
</tr>
<tr>
    <td>SDFM_CH_CTRL_CH_EN_* (all channel enable masks)</td>
    <td>Channel enable is now per-channel via SDFM_setEnableChannel()</td>
</tr>
<tr>
    <td>SDFM_RECFG_* (all reconfiguration flags)</td>
    <td>Reconfiguration is handled through individual API calls</td>
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
    <td>Firmware not responding within timeout period (SDFM_enable, SDFM_measureClockPhaseDelay)</td>
</tr>
</table>

## Migration Examples

### Example 1: Basic Initialization

**Old Code:**
```c
sdfm_handle hSdfm;

hSdfm = SDFM_init(pruIcssHandle, pruSliceId, coreId);
if (hSdfm == NULL)
{
    /* Handle error */
}
```

**New Code:**
```c
SDFM_Handle hSdfm;
SDFM_Params params;

/* Initialize params with defaults */
SDFM_paramsInit(&params);
params.pruicss_handle = pruIcssHandle;
params.pwm_handle = pruPwmHandle;
params.sample_base_addr = sampleBaseAddr;

/* Initialize SDFM (CONFIG_SDFM0 is generated by SysConfig) */
hSdfm = SDFM_init(CONFIG_SDFM0, &params);
if (hSdfm == NULL)
{
    /* Handle error */
}
```

### Example 2: Enabling SDFM and Reading Data

**Old Code:**
```c
SDFM_enable(hSdfm);

/* Read sample data */
uint32_t sample = SDFM_getFilterData(hSdfm, SDFM_CHANNEL0);
```

**New Code:**
```c
int32_t status;

status = SDFM_enable(hSdfm, SDFM_PRU_CORE_INDEX);
if (status == SystemP_TIMEOUT)
{
    /* Handle firmware timeout */
}
else if (status != SystemP_SUCCESS)
{
    /* Handle other errors */
}

/* Read sample data */
uint32_t sample = SDFM_getFilterData(hSdfm, SDFM_CHANNEL0);
```

### Example 3: Configuring Thresholds

**Old Code:**
```c
uint32_t thresholdParms[2];
thresholdParms[0] = highThreshold;
thresholdParms[1] = lowThreshold;

SDFM_setCompFilterThresholds(hSdfm, SDFM_CHANNEL0, thresholdParms);
```

**New Code:**
```c
SDFM_ThresholdConfig thresholdConfig;
thresholdConfig.high_threshold = highThreshold;
thresholdConfig.low_threshold = lowThreshold;

int32_t status = SDFM_setCompFilterThresholds(hSdfm, SDFM_CHANNEL0, thresholdConfig);
if (status != SystemP_SUCCESS)
{
    /* Handle error */
}
```

### Example 4: Configuring Fast Detect

**Old Code:**
```c
uint8_t fdParms[5];
fdParms[0] = windowSize;
fdParms[1] = zeroMaxCount;
fdParms[2] = zeroMinCount;
fdParms[3] = oneMaxCount;
fdParms[4] = oneMinCount;

SDFM_configFastDetect(hSdfm, SDFM_CHANNEL0, fdParms);
```

**New Code:**
```c
SDFM_FastDetectConfig fdConfig;
fdConfig.fd_enable = 1;
fdConfig.fd_window_size = windowSize;
fdConfig.fd_zero_max = zeroMaxCount;
fdConfig.fd_zero_min = zeroMinCount;

int32_t status = SDFM_configFastDetect(hSdfm, SDFM_CHANNEL0, fdConfig);
if (status != SystemP_SUCCESS)
{
    /* Handle error */
}
```

### Example 5: Accessing Configuration

**Old Code:**
```c
/* Access fields directly from SDFM structure */
uint32_t sdfmClock = hSdfm->sdfmClock;
uint32_t iepClock = hSdfm->iepClock;
```

**New Code:**
```c
const SDFM_Attrs *attrs = SDFM_getAttrs(hSdfm);
if (attrs != NULL)
{
    uint32_t coreClkFreq = attrs->core_clk_freq;
    uint32_t iepClkFreq = attrs->iep_clk_freq;
}

SDFM_Priv *priv = SDFM_getPriv(hSdfm);
if (priv != NULL)
{
    /* Access runtime state */
}
```

### Example 6: Phase Delay Measurement

**Old Code:**
```c
SDFM_measureClockPhaseDelay(hSdfm, clockEdge);
float phaseDelay = SDFM_getClockPhaseDelay(hSdfm);
```

**New Code:**
```c
int32_t status = SDFM_measureClockPhaseDelay(hSdfm, clockEdge, SDFM_CHANNEL0);
if (status == SystemP_TIMEOUT)
{
    /* Handle firmware timeout */
}
else if (status == SystemP_SUCCESS)
{
    float phaseDelay = SDFM_getClockPhaseDelay(hSdfm, SDFM_CHANNEL0);
}
```

## SysConfig Migration

### Adding SDFM Module in SysConfig

1. Open your project's `.syscfg` file
2. Add the SDFM module under Current Sense, if not added already
3. Configure the parameters as per the requirement

### Generated Code

SysConfig will generate:
- \ref SDFM_Attrs structures with compile-time configuration
- `gSdfmConfig` array with \ref SDFM_Config entries
- Code is generated in `ti_drivers_config.c` and `ti_drivers_config.h`

## Common Migration Issues

1. **Compilation Errors with sdfm_handle**
   - Replace all `sdfm_handle` with `SDFM_Handle`
   - Use `SDFM_getPriv()` when you need access to internal runtime state

2. **Missing SDFM_enableContinuousNormalCurrent()**
   - Use `SDFM_enableTriggerModeForNormalCurrent()` with pru_core parameter

3. **Missing SDFM_enableLoadShareMode()**
   - Load share mode is now configured through SysConfig (load_share_enabled in \ref SDFM_Attrs)

4. **Missing NUM_SD_CH macro**
   - Use SDFM_NUM_OF_CH_PER_PRU_SLICE instead

5. **SDFM_setCompFilterThresholds() parameter change**
   - Replace `uint32_t*` array with \ref SDFM_ThresholdConfig structure

6. **SDFM_configFastDetect() parameter change**
   - Replace `uint8_t*` array with \ref SDFM_FastDetectConfig structure

7. **SysConfig Errors**
   - Ensure SDFM module is added and configured in `.syscfg` file
   - Review the configured parameters

8. **Missing SDFM_enableShadowRegBasedNC() / SDFM_disableShadowRegBasedNC()**
   - These APIs were renamed. Use `SDFM_enableSnoopBasedNC()` and `SDFM_disableSnoopBasedNC()` with the new `pru_core` parameter

9. **SDFM_setFilterOverSamplingRatio() requires a new channel parameter**
    - The old API had no channel parameter: `SDFM_setFilterOverSamplingRatio(h_sdfm, nc_osr)`
    - The new API requires a `channel` argument (0-8): `SDFM_setFilterOverSamplingRatio(handle, channel, nc_osr)`
    - Update all call sites to supply the target channel number

## Additional Resources

- \ref EXAMPLES_CURRENT_SENSE
- \ref SDFM_API_MODULE
- \ref CURRENT_SENSE
- \ref SDFM_DESIGN

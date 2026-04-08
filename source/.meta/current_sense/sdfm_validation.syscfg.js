let inst_name;

function onValidate(inst, validation) {
    for (let channel = 0; channel < 9; channel++) {
        /* Only validate enabled channels */
        if (!inst["Enable_Channel_" + channel]) {
            continue;
        }

        inst_name = "Ch" + channel.toString() + "_NC_OSR";
        let nosr = inst[inst_name];

        if (nosr < 4 || nosr > 256) {
            validation.logError(
                "NC OSR should be between 4 and 256 (inclusive)! - Check Channel_" + channel + " NC OSR = " + nosr,
                inst, inst_name);
        }

        /* Comparator-specific validation — only when comparator is enabled */
        if (inst["Ch" + channel.toString() + "_ComparatorEnable"]) {
            inst_name = "Ch" + channel.toString() + "_OC_OSR";
            let cosr = inst[inst_name];

            if (cosr < 4 || cosr > 256) {
                validation.logError(
                    "OC OSR should be between 4 and 256 (inclusive)! - Check Channel_" + channel + " OC OSR = " + cosr,
                    inst, inst_name);
            }

            inst_name = "Ch" + channel.toString() + "_HLT";
            let HLT = inst[inst_name];

            if (HLT < 0 || HLT > 16777216) {
                validation.logError(
                    "HLT should be between 0 and 16777216 (inclusive)! - Check Channel_" + channel + " HLT = " + HLT,
                    inst, inst_name);
            }

            inst_name = "Ch" + channel.toString() + "_LLT";
            let LLT = inst[inst_name];

            if (LLT < 0 || LLT > 16777216) {
                validation.logError(
                    "LLT should be between 0 and 16777216 (inclusive)! - Check Channel_" + channel + " LLT = " + LLT,
                    inst, inst_name);
            }

            if (HLT < LLT) {
                inst_name = "Ch" + channel.toString() + "_HLT";
                validation.logError(
                    "High Level Threshold should be >= Low Level Threshold for Channel_" + channel,
                    inst, inst_name);
            }

            /* Zero cross threshold — only when zero cross detection is enabled */
            if (inst["Ch" + channel.toString() + "_ZeroCross_Enable"]) {
                inst_name = "Ch" + channel.toString() + "_ZCT";
                let ZCT = inst[inst_name];

                if (ZCT < 0 || ZCT > 16777216) {
                    validation.logError(
                        "Zero Cross Threshold should be between 0 and 16777216 (inclusive)! - Check Channel_" + channel + " ZCT = " + ZCT,
                        inst, inst_name);
                }
            }
        }

        /* Fast detect threshold validation — only when fast detect is enabled */
        if (inst["Ch" + channel.toString() + "_FastDetect"]) {
            inst_name = "Ch" + channel.toString() + "_ZeroMinTh";
            let ZeroMinTh = inst[inst_name];

            inst_name = "Ch" + channel.toString() + "_ZeroMaxTh";
            let ZeroMaxTh = inst[inst_name];

            if (ZeroMinTh > ZeroMaxTh) {
                inst_name = "Ch" + channel.toString() + "_ZeroMinTh";
                validation.logError(
                    "Zero Count Min Threshold should be <= Max Threshold for Channel_" + channel,
                    inst, inst_name);
            }
        }
    }

    inst_name = "Enable_Phase_Compensation";
    let Phase_Delay = inst[inst_name];
    inst_name = "Enable_Load_Share";
    let Load_Share = inst[inst_name];

    if (Load_Share && Phase_Delay) {
        validation.logWarning(
            "Load share mode examples in SDK do not support phase compensation",
            inst, inst_name);
    }

    inst_name = "SDFM_CLK_GEN";
    let clock_gen = inst[inst_name];

    if ((clock_gen != "0") && Phase_Delay) {
        validation.logError(
            "Select SDFM clock generation from IEP for phase compensation",
            inst, inst_name);
    }

    /* Get PRU/RTU/TXPRU trigger and snoop mode settings */
    inst_name = "PRU_EnableSnoopNC";
    let pru_snoop_mode = inst[inst_name];
    inst_name = "RTU_EnableSnoopNC";
    let rtu_snoop_mode = inst[inst_name];
    inst_name = "TXPRU_EnableSnoopNC";
    let txpru_snoop_mode = inst[inst_name];

    inst_name = "PRU_EnableTriggerMode";
    let pru_trig_mode = inst[inst_name];
    inst_name = "RTU_EnableTriggerMode";
    let rtu_trig_mode = inst[inst_name];
    inst_name = "TXPRU_EnableTriggerMode";
    let txpru_trig_mode = inst[inst_name];

    /* Check that channels 3-8 are disabled when PRU snoop mode is active without load share */
    if(pru_snoop_mode && !Load_Share) {
        let hasEnabledInvalidChannels = false;
        for (let ch = 3; ch < 9; ch++) {
            if (inst["Enable_Channel_" + ch]) {
                hasEnabledInvalidChannels = true;
                break;
            }
        }
        if (hasEnabledInvalidChannels) {
            validation.logError(
                "Channels 3-8 are enabled but not supported in PRU snoop mode without load share. Disable PRU snoop mode or enable load share to access and disable those channels.",
                inst, "PRU_EnableSnoopNC");
        }
    }

    /* Check that if snoop mode is enabled, trigger mode must be enabled too */
    if(pru_snoop_mode && !pru_trig_mode) {
        inst_name = "PRU_EnableTriggerMode";
        validation.logError(
            "Enable trigger mode for SDFM PRU channels",
            inst, inst_name);
    }

    if(rtu_snoop_mode && !rtu_trig_mode) {
        inst_name = "RTU_EnableTriggerMode";
        validation.logError(
            "Enable trigger mode for SDFM RTU channels",
            inst, inst_name);
    }

    if(txpru_snoop_mode && !txpru_trig_mode) {
        inst_name = "TXPRU_EnableTriggerMode";
        validation.logError(
            "Enable trigger mode for SDFM TXPRU channels",
            inst, inst_name);
    }

    /* Check that RTU/TXPRU snoop and trigger modes can only be enabled in load share mode.
     * In non-load-share mode, only PRU core is available for snoop/trigger functionality. */
    if (!Load_Share && rtu_snoop_mode) {
        inst_name = "RTU_EnableSnoopNC";
        validation.logError(
            "RTU snoop mode is only available in load share mode",
            inst, inst_name);
    }

    if (!Load_Share && txpru_snoop_mode) {
        inst_name = "TXPRU_EnableSnoopNC";
        validation.logError(
            "TXPRU snoop mode is only available in load share mode",
            inst, inst_name);
    }

    if (!Load_Share && rtu_trig_mode) {
        inst_name = "RTU_EnableTriggerMode";
        validation.logError(
            "RTU trigger mode is only available in load share mode",
            inst, inst_name);
    }

    if (!Load_Share && txpru_trig_mode) {
        inst_name = "TXPRU_EnableTriggerMode";
        validation.logError(
            "TXPRU trigger mode is only available in load share mode",
            inst, inst_name);
    }

    if (!Load_Share && inst.RTU_EnableDoubleUpdate) {
        inst_name = "RTU_EnableDoubleUpdate";
        validation.logError(
            "RTU double update is only available in load share mode",
            inst, inst_name);
    }

    if (!Load_Share && inst.TXPRU_EnableDoubleUpdate) {
        inst_name = "TXPRU_EnableDoubleUpdate";
        validation.logError(
            "TXPRU double update is only available in load share mode",
            inst, inst_name);
    }

    /* Double update is only valid when corresponding trigger mode is enabled */
    if (inst.PRU_EnableDoubleUpdate && !inst.PRU_EnableTriggerMode) {
        inst_name = "PRU_EnableDoubleUpdate";
        validation.logWarning(
            "PRU double update requires PRU trigger mode enabled",
            inst, inst_name);
    }

    if (Load_Share && inst.RTU_EnableDoubleUpdate && !inst.RTU_EnableTriggerMode) {
        inst_name = "RTU_EnableDoubleUpdate";
        validation.logWarning(
            "RTU double update requires RTU trigger mode enabled",
            inst, inst_name);
    }

    if (Load_Share && inst.TXPRU_EnableDoubleUpdate && !inst.TXPRU_EnableTriggerMode) {
        inst_name = "TXPRU_EnableDoubleUpdate";
        validation.logWarning(
            "TXPRU double update requires TXPRU trigger mode enabled",
            inst, inst_name);
    }

    /* Get all channel settings */
    let channel_enabled = [];
    let channel_acc_source = [];
    let channel_nc_osr = [];
    let channel_clock = [];

    for(let i = 0; i < 9; i++) {
        channel_enabled[i] = inst["Enable_Channel_" + i];
        channel_acc_source[i] = inst["Ch" + i + "_AccSource"];
        channel_nc_osr[i] = inst["Ch" + i + "_NC_OSR"];
        channel_clock[i] = inst["Ch" + i + "_SDFM_Clock"];
    }
    /* Validate IEP compare event selection - each PRU core must use different compare event */
    validateIepCompareEvents(inst, validation, pru_trig_mode, rtu_trig_mode, txpru_trig_mode, Load_Share);

    /* VALIDATION FOR LOAD SHARE MODE */
    if(Load_Share) {
        /* RTU manages channels 0-2 */
        if(rtu_trig_mode ) {
            validateChannelsConsistency(inst, validation, 0, 2, channel_enabled, channel_acc_source, channel_nc_osr, channel_clock);
        }

        /* PRU manages channels 3-5 */
        if(pru_trig_mode ) {
            validateChannelsConsistency(inst, validation, 3, 5, channel_enabled, channel_acc_source, channel_nc_osr, channel_clock);
        }

        /* TXPRU manages channels 6-8 */
        if(txpru_trig_mode ) {
            validateChannelsConsistency(inst, validation, 6, 8, channel_enabled, channel_acc_source, channel_nc_osr, channel_clock);
        }
    }
    /* VALIDATION FOR NON-LOAD SHARE MODE */
    else {
        /* In non-load share mode with trigger mode enabled, all channels must have consistent settings */
        if(pru_trig_mode ) {
            /* Check all channels (0-8) for consistency */
            validateAllChannelsConsistency(inst, validation, channel_enabled, channel_acc_source, channel_nc_osr, channel_clock);
        }
    }

    /* Validate trigger point relationships when trigger mode is enabled */
    if(pru_trig_mode) {
        /* FirstTriggerPoint must be less than SecondTriggerPoint when double update is enabled */
        if(inst.PRU_EnableDoubleUpdate && inst.PRU_FirstTriggerPoint >= inst.PRU_SecondTriggerPoint) {
            inst_name = "PRU_FirstTriggerPoint";
            validation.logError(
                "PRU first trigger point must be less than second trigger point when double update is enabled",
                inst, inst_name);
        }
    }

    if(Load_Share && rtu_trig_mode) {
        if(inst.RTU_EnableDoubleUpdate && inst.RTU_FirstTriggerPoint >= inst.RTU_SecondTriggerPoint) {
            inst_name = "RTU_FirstTriggerPoint";
            validation.logError(
                "RTU first trigger point must be less than second trigger point when double update is enabled",
                inst, inst_name);
        }
    }

    if(Load_Share && txpru_trig_mode) {
        if(inst.TXPRU_EnableDoubleUpdate && inst.TXPRU_FirstTriggerPoint >= inst.TXPRU_SecondTriggerPoint) {
            inst_name = "TXPRU_FirstTriggerPoint";
            validation.logError(
                "TXPRU first trigger point must be less than second trigger point when double update is enabled",
                inst, inst_name);
        }
    }
}

function validateChannelsConsistency(inst, validation, startCh, endCh, enabled, acc_source, nc_osr, clock) {
    let enabled_channels = [];

    /* Find enabled channels in the range */
    for(let i = startCh; i <= endCh; i++) {
        if(enabled[i]) {
            enabled_channels.push(i);
        }
    }

    /* Skip if less than 2 channels are enabled */
    if(enabled_channels.length < 2) {
        return;
    }

    /* Get reference values from first enabled channel */
    let ref_ch = enabled_channels[0];
    let ref_acc_source = acc_source[ref_ch];
    let ref_nc_osr = nc_osr[ref_ch];
    let ref_clock = clock[ref_ch];

    /* Check all enabled channels have consistent settings */
    for(let i = 1; i < enabled_channels.length; i++) {
        let ch = enabled_channels[i];

        /* Check accumulator source */
        if(acc_source[ch] != ref_acc_source) {
            inst_name = "Ch" + ref_ch + "_AccSource";
            validation.logError(
                "Accumulator source should be same for Channels " + startCh + " through " + endCh + " in trigger mode",
                inst, inst_name);
            break;
        }

        /* Check NC OSR */
        if(nc_osr[ch] != ref_nc_osr) {
            inst_name = "Ch" + ref_ch + "_NC_OSR";
            validation.logError(
                "NC OSR value should be same for Channels " + startCh + " through " + endCh + " in trigger mode",
                inst, inst_name);
            break;
        }

        /* Check clock */
        if(clock[ch] != ref_clock) {
            inst_name = "Ch" + ref_ch + "_SDFM_Clock";
            validation.logError(
                "SDFM clock value should be same for Channels " + startCh + " through " + endCh + " in trigger mode",
                inst, inst_name);
            break;
        }
    }
}

function validateAllChannelsConsistency(inst, validation, enabled, acc_source, nc_osr, clock) {
    let enabled_channels = [];

    /* Find all enabled channels */
    for(let i = 0; i < 9; i++) {
        if(enabled[i]) {
            enabled_channels.push(i);
        }
    }

    /* Skip if less than 2 channels are enabled */
    if(enabled_channels.length < 2) {
        return;
    }

    /* Get reference values from first enabled channel */
    let ref_ch = enabled_channels[0];
    let ref_acc_source = acc_source[ref_ch];
    let ref_nc_osr = nc_osr[ref_ch];
    let ref_clock = clock[ref_ch];

    /* Check all enabled channels have consistent settings */
    for(let i = 1; i < enabled_channels.length; i++) {
        let ch = enabled_channels[i];

        /* Check accumulator source */
        if(acc_source[ch] != ref_acc_source) {
            inst_name = "Ch" + ref_ch + "_AccSource";
            validation.logError(
                "Accumulator source should be same for all channels in trigger mode",
                inst, inst_name);
            break;
        }

        /* Check NC OSR */
        if(nc_osr[ch] != ref_nc_osr) {
            inst_name = "Ch" + ref_ch + "_NC_OSR";
            validation.logError(
                "NC OSR value should be same for all channels in trigger mode",
                inst, inst_name);
            break;
        }

        /* Check clock */
        if(clock[ch] != ref_clock) {
            inst_name = "Ch" + ref_ch + "_SDFM_Clock";
            validation.logError(
                "SDFM clock value should be same for all channels in trigger mode",
                inst, inst_name);
            break;
        }
    }
}

function validateIepCompareEvents(inst, validation, pru_trig, rtu_trig, txpru_trig, load_share) {
    /* Only validate if at least one trigger mode is enabled */
    if(!pru_trig && !rtu_trig && !txpru_trig) {
        return;
    }

    let pru_cmp = inst["PRU_SelectIepCmpEvent"];
    let rtu_cmp = inst["RTU_SelectIepCmpEvent"];
    let txpru_cmp = inst["TXPRU_SelectIepCmpEvent"];

    /* In load share mode, validate between enabled PRU cores */
    if(load_share) {
        /* Check PRU vs RTU */
        if(pru_trig && rtu_trig && pru_cmp === rtu_cmp) {
            validation.logError(
                "PRU and RTU cannot use the same IEP compare event. PRU uses CMP" + pru_cmp + ", RTU uses CMP" + rtu_cmp,
                inst, "PRU_SelectIepCmpEvent");
        }

        /* Check PRU vs TXPRU */
        if(pru_trig && txpru_trig && pru_cmp === txpru_cmp) {
            validation.logError(
                "PRU and TXPRU cannot use the same IEP compare event. PRU uses CMP" + pru_cmp + ", TXPRU uses CMP" + txpru_cmp,
                inst, "PRU_SelectIepCmpEvent");
        }

        /* Check RTU vs TXPRU */
        if(rtu_trig && txpru_trig && rtu_cmp === txpru_cmp) {
            validation.logError(
                "RTU and TXPRU cannot use the same IEP compare event. RTU uses CMP" + rtu_cmp + ", TXPRU uses CMP" + txpru_cmp,
                inst, "RTU_SelectIepCmpEvent");
        }
    }
}

exports = {
    onValidate: onValidate,
};
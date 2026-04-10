let common = system.getScript("/common");
let sdfm_pins = system.getScript("/current_sense/sdfm_pins.js");
let sdfm_clk_config = system.getScript("/current_sense/sdfm_clockConfiguration.syscfg.js");
let sdfm_fd_config = system.getScript("/current_sense/sdfm_channelConfiguration.syscfg.js");
let sd_validate = system.getScript("/current_sense/sdfm_validation.syscfg.js");

let device = common.getDeviceName();

let sdfm_module_name = "/current_sense/sdfm";

let config = [];
let SDFM_IepCmpEvnt = [
	{ name: "0", displayName: "CMP0" },
    { name: "1", displayName: "CMP1" },
    { name: "2", displayName: "CMP2" },
    { name: "3", displayName: "CMP3" },
    { name: "4", displayName: "CMP4" },
    { name: "5", displayName: "CMP5" },
    { name: "6", displayName: "CMP6" },
    { name: "7", displayName: "CMP7" },
    { name: "8", displayName: "CMP8" },
    { name: "9", displayName: "CMP9" },
    { name: "10", displayName: "CMP10" },
    { name: "11", displayName: "CMP11" },
    { name: "12", displayName: "CMP12" },
    { name: "13", displayName: "CMP13" },
    { name: "14", displayName: "CMP14" },
    { name: "15", displayName: "CMP15" },
]

let total_channel = 9;
config = config.concat([
    {
        name: "instance",
        displayName: "Instance",
        default: "ICSSG0",
        options: [
            {
                name: "ICSSG0",
            },
            {
                name: "ICSSG1",
            }
        ],
    },
    {
        name: "G_MUX_EN",
        displayName: "Enable G Mux",
        longDescription : `The G_MUX_EN bit (bit 7) in the ICSSG_SA_MX_REG register is a multiplexer control bit that enables alternative pin configurations for the SDFM mode.
        This bit allows remapping of Data and Clock pins to support different hardware configurations and use cases.

#### Pin Multiplexing Behavior

| G_MUX_EN Value | SD_CHANNEL4_DATA  | SD5_CLK             | Description                    |
|----------------|-------------------|---------------------|--------------------------------|
| 0 (Default)    | GPI9              | GPI10               | Default pinmux configuration   |
| 1 (Enabled)    | GPI18             | GPI19               | Alternate pinmux configuration |`,
        default: false,
        hidden:  (device == "am243x-lp" ||device == "am243x-evm" ||device == "am64x-evm") ? false : true,
    },
    {
        name: "Enable_Load_Share",
        displayName: "Enable Load Share",
        description: "Enable load sharing across multiple PRU cores (PRU, RTU_PRU, TX_PRU). RTU_PRU handles Ch0-2, PRU handles Ch3-5, TX_PRU handles Ch6-8.",
        default: false,
        hidden: false,
        onChange: addOtherPru,
    },
    {
        name: "Enable_Phase_Compensation",
        displayName: "Enable Phase Compensation",
        description: "Enable Phase Compensation ",
        default: false,
        hidden: false,
    },
    {
        name: "Enable_Epwm_Sync",
        displayName: "Enable Epwm Sync",
        description: "Enable SDFM synchronization with ePWM module.",
        default: false,
        hidden: false,
        onChange: addEpwmSource,
    },
    {
        name: "Epwm_Source",
        displayName : "Source of SD SYNC Event",
		description : 'Source of SD SYNC Event',
		hidden      : true,
		default     : "0",
		options     :
		[
			{
				name: "0",
				displayName: "SDFM SYNC source is EPWM0 SYNC out event",
			},
			{
				name:"3",
				displayName: "SDFM SYNC source is EPWM3 SYNC out event",
			},
		]
    },
    {
        name: "IEP_Instance",
        displayName : "PRU ICSS IEP Instance",
		description : 'Select IEP timer instance (IEP0 or IEP1).',
		hidden      : true,
		default     : "0",
		options     :
		[
			{
				name: "0",
				displayName: "PRU ICSS IEP0",
			},
			{
				name:"1",
				displayName: "PRU ICSS IEP1",
			},
		]
    },
    {
        name: "IEP_Reset_Freq",
        displayName : "IEP Reset Frequency (Hz)",
		description : 'IEP counter reset frequency in Hz. Typically matches PWM frequency',
		hidden      : true,
		default     : 8000,
        range       : [1, 0xFFFFFFFF],
    },
    {
        name: "SDFM_CLK_GEN",
        displayName: "SDCLK Generation From",
        description: "SDCLK Generation From",
		hidden	: false,
        default: "1",
        options: [
                {
                    name: "3",
                    displayName: "None (External Clock)"
                },
                {
                    name: "2",
                    displayName: "PRU-ICSSG (PR<k>_PRU0_GPO1)"
                },
                {
                    name: "1",
                    displayName: "eCAP"
                },
                {
                    name: "0",
                    displayName: "IEP"
                },
        ],
        onChange: updateClockSourceVisibility,
    },
    {
        name: "SDFM_Clock_Value",
        displayName: "SDFM Frequency (Hz)",
        description: "SDFM clock frequency in Hz. For internal clock sources (IEP, ECAP, GPIO1), this is the generated clock frequency. For external clock source (None), specify the externally provided clock frequency. This value is used by the driver for timing calculations. Note: Clock divider values need to be set manually in application initialization based on this frequency.",
        hidden: false,
        default: 20000000,
        range: [1, 0xFFFFFFFF],
        onChange: propagateClockToChannels,
    },
    {
        name: "GROUP_pruSettings",
		displayName : "SDFM PRU Channels Configuration",
        config      : [
            {
                name: "PRU_EnableSnoopNC",
                displayName: "Enable NC Snoop Mode",
                description: "Enable NC Snoop Mode",
                hidden: false,
                default: false,
                onChange: updateChannelVisibilityOnSnoopMode,
            },
            {
                name        : "PRU_EnableTriggerMode",
                displayName : "Enable NC Trigger Mode",
                description : 'Enable NC trigger Mode',
                hidden      : false,
                default     : false,
                onChange	: configNCsamplingMode,
            },
            {
                name        : "PRU_SelectIepCmpEvent",
                displayName : "Select IEP Compare Event for NC Trigger Mode",
                description : "IEP compare event (CMP0-CMP15) that triggers normal current sampling. Note: Avoid using CMP0-CMP2, if CMP0 is used for IEP reset or CMP1/CMP2 are used for clock generation.",
                hidden      : true,
                default: SDFM_IepCmpEvnt[3].name,
                options: SDFM_IepCmpEvnt
            },
            {
                name        : "PRU_FirstTriggerPoint",
                displayName : "First Trigger Point (us)",
                description : 'First trigger time point in microseconds within IEP reset period. Normal current samples are captured at this time',
                hidden      : true,
                default     : 15,
            },
            {
                name        : "PRU_EnableDoubleUpdate",
                displayName : "Enable Double Update",
                description : 'Enable double update mode to capture two normal current samples per PWM cycle',
                hidden      : true,
                default     : false,
                onChange    : doubleUpdateConfig,
            },
            {
                name        : "PRU_SecondTriggerPoint",
                displayName : "Second Trigger Point (us)",
                description : 'Second trigger time point in microseconds within IEP reset period. Only used when double update mode is enabled',
                hidden      : true,
                default     : 30,
            },
        ]

    },
    {
        name: "GROUP_rtuSettings",
        displayName : "SDFM RTU Channels Configuration",
        config      : [
            {
                name: "RTU_EnableSnoopNC",
                displayName: "Enable NC Snoop Mode",
                description: "Enable NC Snoop Mode",
                hidden: true,
                default: false,
            },
            {
                name        : "RTU_EnableTriggerMode",
                displayName : "Enable NC Trigger Mode",
                description : 'Enable NC trigger Mode',
                hidden      : true,
                default     : false,
                onChange	: configNCsamplingMode,
            },
            {
                name        : "RTU_SelectIepCmpEvent",
                displayName : "Select IEP Compare Event for NC Trigger Mode",
                description : "IEP compare event (CMP0-CMP15) that triggers normal current sampling. Note: Avoid using CMP0-CMP2, if CMP0 is used for IEP reset or CMP1/CMP2 are used for clock generation.",
                hidden      : true,
                default: SDFM_IepCmpEvnt[4].name,
                options: SDFM_IepCmpEvnt
            },
            {
                name        : "RTU_EnableDoubleUpdate",
                displayName : "Enable Double Update",
                description : 'Enable Double Update',
                hidden      : true,
                default     : false,
                onChange	: doubleUpdateConfig,
            },
            {
                name        : "RTU_FirstTriggerPoint",
                displayName : "First Trigger Point (us)",
                description : 'First Trigger Point (us)',
                hidden      : true,
                default     : 15,
            },
            {
                name        : "RTU_SecondTriggerPoint",
                displayName : "Second Trigger Point (us)",
                description : 'Second Trigger Point (us)',
                hidden      : true,
                default     : 30,
            }

        ]


    },
    {
        name: "GROUP_txpruSettings",
        displayName : "SDFM TXPRU Channels Configuration",
        config      : [
            {
                name: "TXPRU_EnableSnoopNC",
                displayName: "Enable NC Snoop Mode",
                description: "Enable NC Snoop Mode",
                hidden: true,
                default: false,
            },
            {
                name        : "TXPRU_EnableTriggerMode",
                displayName : "Enable NC Trigger Mode",
                description : 'Enable NC trigger Mode',
                hidden      : true,
                default     : false,
                onChange	: configNCsamplingMode,
            },
            {
                name        : "TXPRU_SelectIepCmpEvent",
                displayName : "Select IEP Compare Event for NC Trigger Mode",
                description : "IEP compare event (CMP0-CMP15) that triggers normal current sampling. Note: Avoid using CMP0-CMP2, if CMP0 is used for IEP reset or CMP1/CMP2 are used for clock generation.",
                hidden      : true,
                default: SDFM_IepCmpEvnt[5].name,
                options: SDFM_IepCmpEvnt
            },
            {
                name        : "TXPRU_EnableDoubleUpdate",
                displayName : "Enable Double Update",
                description : 'Enable Double Update',
                hidden      : true,
                default     : false,
                onChange	: doubleUpdateConfig,
            },
            {
                name        : "TXPRU_FirstTriggerPoint",
                displayName : "First Trigger Point (us)",
                description : 'First Trigger Point (us)',
                hidden      : true,
                default     : 15,
            },
            {
                name        : "TXPRU_SecondTriggerPoint",
                displayName : "Second Trigger Point (us)",
                description : 'Second Trigger Point (us)',
                hidden      : true,
                default     : 30,
            }

        ]
    }

])

for(let ch = 0; ch < total_channel; ch++)
{
    config = config.concat([
        {
            name: "Enable_Channel_" + ch.toString(),
            displayName: "Enable Channel " + ch.toString(),
            description: 'Enable / Disable SDFM Channel',
            hidden: false,
            default: false,
            onChange: onChangeEnableChannel,
        },
    ])
}

let submodulesComponents = [
    {
        moduleName: "/current_sense/sdfm_clockConfiguration.syscfg.js",
        name: "sdfmClockConfiguration",
        displayName:"SDFM Clock Configuration",
        description:"SDFM Clock Configuration",
    },
    {
        moduleName: "/current_sense/sdfm_channelConfiguration.syscfg.js",
        name: "sdfmChannelConfig",
        displayName:"SDFM Channel Configuration",
        description:"SDFM Channel Configuration",
    },
]

for (let submoduleComponent of submodulesComponents)
{
    let submodule = system.getScript(submoduleComponent.moduleName)
    config = config.concat([
        {
            name: "GROUP_" + submodule.defaultInstanceName,
            displayName: submodule.displayName,
            longDescription: submodule.description,
            description: "",
            config: submodule.config
        },
    ])
}

function doubleUpdateConfig(inst, ui)
{
    if(inst.PRU_EnableDoubleUpdate == true)
    {
        ui.PRU_SecondTriggerPoint.hidden = false;
    }
    else
    {
        ui.PRU_SecondTriggerPoint.hidden = true;
    }

    if(inst.RTU_EnableDoubleUpdate == true)
    {
        ui.RTU_SecondTriggerPoint.hidden = false;
    }
    else
    {
        ui.RTU_SecondTriggerPoint.hidden = true;
    }
    if(inst.TXPRU_EnableDoubleUpdate == true)
    {
        ui.TXPRU_SecondTriggerPoint.hidden = false;
    }
    else
    {
        ui.TXPRU_SecondTriggerPoint.hidden = true;
    }

}
function configNCsamplingMode(inst, ui)
{
    if(inst.PRU_EnableTriggerMode == true)
    {
        ui.PRU_FirstTriggerPoint.hidden = false;
        ui.PRU_SelectIepCmpEvent.hidden = false;
        ui.PRU_EnableDoubleUpdate.hidden = false;
    }
    else
    {
        ui.PRU_FirstTriggerPoint.hidden = true;
        ui.PRU_SelectIepCmpEvent.hidden = true;
        ui.PRU_EnableDoubleUpdate.hidden = true;
    }

    if(inst.RTU_EnableTriggerMode == true)
    {
        ui.RTU_FirstTriggerPoint.hidden = false;
        ui.RTU_SelectIepCmpEvent.hidden = false;
        ui.RTU_EnableDoubleUpdate.hidden = false;
    }
    else
    {
        ui.RTU_FirstTriggerPoint.hidden = true;
        ui.RTU_SelectIepCmpEvent.hidden = true;
        ui.RTU_EnableDoubleUpdate.hidden = true;
    }

    if(inst.TXPRU_EnableTriggerMode == true)
    {
        ui.TXPRU_FirstTriggerPoint.hidden = false;
        ui.TXPRU_SelectIepCmpEvent.hidden = false;
        ui.TXPRU_EnableDoubleUpdate.hidden = false;
    }
    else
    {
        ui.TXPRU_FirstTriggerPoint.hidden = true;
        ui.TXPRU_SelectIepCmpEvent.hidden = true;
        ui.TXPRU_EnableDoubleUpdate.hidden = true;
    }
    if(inst.PRU_EnableTriggerMode == true || inst.RTU_EnableTriggerMode == true || inst.TXPRU_EnableTriggerMode == true)
    {
        ui.IEP_Instance.hidden = false;
        ui.IEP_Reset_Freq.hidden = false;
    }
    else
    {
        ui.IEP_Instance.hidden = true;
        ui.IEP_Reset_Freq.hidden = true;
    }

}

function addEpwmSource(inst, ui)
{
    let hideEpwmSource = true;
    if (inst.Enable_Epwm_Sync == true)
    {
        hideEpwmSource = false;
    }
    ui.Epwm_Source.hidden = hideEpwmSource;
}

function addOtherPru(inst, ui)
{
    let hideConfigs = true;
    if (inst.Enable_Load_Share == true)
    {
        hideConfigs = false;
    }
    ui.RTU_EnableTriggerMode.hidden = hideConfigs;
    ui.TXPRU_EnableTriggerMode.hidden = hideConfigs;
    ui.RTU_EnableSnoopNC.hidden = hideConfigs;
    ui.TXPRU_EnableSnoopNC.hidden = hideConfigs;

    /* When load share is disabled, reset RTU/TXPRU snoop and trigger modes to false
     * to prevent invalid configuration states from persisting. */
    if (hideConfigs)
    {
        inst.RTU_EnableSnoopNC = false;
        inst.TXPRU_EnableSnoopNC = false;
        inst.RTU_EnableTriggerMode = false;
        inst.TXPRU_EnableTriggerMode = false;
        inst.RTU_EnableDoubleUpdate = false;
        inst.TXPRU_EnableDoubleUpdate = false;
    }

    configNCsamplingMode(inst, ui);
    doubleUpdateConfig(inst, ui);

    /* Update channel visibility when load share mode changes */
    updateChannelVisibilityOnSnoopMode(inst, ui);
}

function updateClockSourceVisibility(inst, ui)
{
    /* SDFM_Clock_Value is always visible: for internal clock sources it drives the
     * generated frequency; for external clock it must be set to the external clock
     * frequency so the driver can compute snoop-mode IEP counts correctly. */
    ui.SDFM_Clock_Value.hidden = false;

    /* Update read-only status for all channel clock fields based on clock source.
     * Internal clock sources: Ch*_SDFM_Clock is readOnly, synced from SDFM_Clock_Value
     * External clock source: Ch*_SDFM_Clock is writable per-channel */
    let isReadOnly = (inst.SDFM_CLK_GEN != "3");
    for (let channel = 0; channel < 9; channel++)
    {
        ui["Ch" + channel.toString() + "_SDFM_Clock"].readOnly = isReadOnly;
    }

    propagateClockToChannels(inst, ui);
}

function propagateClockToChannels(inst, ui)
{
    /* Synchronize all channel clock values with SDFM_Clock_Value for internal clock sources.
     * All channels (enabled and disabled) are updated so that re-enabling a channel always
     * shows the current clock value rather than a stale one. */
    if (inst.SDFM_CLK_GEN != "3")
    {
        let clockValue = inst.SDFM_Clock_Value;
        for (let channel = 0; channel < 9; channel++)
        {
            inst["Ch" + channel.toString() + "_SDFM_Clock"] = clockValue;
        }
    }
}

function updateChannelVisibilityOnSnoopMode(inst, ui)
{
    /* Hide channels 3-8 when PRU snoop mode is enabled and load share is not enabled
     * Only channels 0-2 should be visible in this mode */
    let hideChannels = inst.PRU_EnableSnoopNC && !inst.Enable_Load_Share;

    for (let ch = 3; ch < 9; ch++)
    {
        ui["Enable_Channel_" + ch.toString()].hidden = hideChannels;
        if (hideChannels)
        {
            inst["Enable_Channel_" + ch.toString()] = false;
            /* Clear sub-configuration state for hidden channels to prevent orphaned settings
             * from persisting in the instance object. */
            inst["Ch" + ch.toString() + "_ComparatorEnable"] = false;
            inst["Ch" + ch.toString() + "_FastDetect"] = false;
            inst["Ch" + ch.toString() + "_AccSource"] = "0";
        }
    }

    /* When channels 3-8 are force-disabled, sync their sub-config visibility so
     * settings for disabled channels are not left visible on screen. */
    if (hideChannels)
    {
        onChangeEnableChannel(inst, ui);
    }
}

function onChangeEnableChannel(inst, ui)
{
    for (let channel = 0; channel < total_channel; channel++)
	{
		let status = inst["Enable_Channel_" + channel.toString()];

		ui["Ch" + channel.toString() + "_SDCLKSEL"].hidden = !status;
        ui["Ch" + channel.toString() + "_CLKINV"].hidden = !status;
        ui["Ch" + channel.toString() + "_SDFM_Clock"].hidden = !status;
        if (status)
        {
            /* Re-apply readOnly when making a channel visible so it matches the
             * current clock source, regardless of when updateClockSourceVisibility
             * last ran relative to this call. */
            ui["Ch" + channel.toString() + "_SDFM_Clock"].readOnly = (inst.SDFM_CLK_GEN != "3");
            if (inst.SDFM_CLK_GEN != "3")
            {
                inst["Ch" + channel.toString() + "_SDFM_Clock"] = inst.SDFM_Clock_Value;
            }
        }

        ui["Ch" + channel.toString() + "_AccSource"].hidden = !status;
        ui["Ch" + channel.toString() + "_ComparatorEnable"].hidden = !status;
        ui["Ch" + channel.toString() + "_NC_OSR"].hidden = !status;
        ui["Ch" + channel.toString() + "_FastDetect"].hidden = !status;
        ui["Ch" + channel.toString() + "_Datarate_DF"].hidden = !status;
        ui["Ch" + channel.toString() + "_Latency_DF"].hidden = !status;
        ui["Ch" + channel.toString() + "_Min_FilterOutput"].hidden = !status;
        ui["Ch" + channel.toString() + "_Max_FilterOutput"].hidden = !status;
        ui["Ch" + channel.toString() + "_Vclipping"].hidden = !status;
        ui["Ch" + channel.toString() + "_DC_Input"].hidden = !status;
        ui["Ch" + channel.toString() + "_bitstream_1s_density"].hidden = !status;
        ui["Ch" + channel.toString() + "_SD_modulatorFrequency"].hidden = !status;
        ui["Ch" + channel.toString() + "_Theoritical_DataFilterOutput"].hidden = !status;
        ui["Ch" + channel.toString() + "_Theoritical_ComparatorFilterOutput"].hidden = !status;
	}

}

function sharedModuleInstances(instance) {
    let modInstances = new Array();

    modInstances.push({
        name: "pru",
        displayName: "PRU ICSS Configuration",
        moduleName: '/drivers/pruicss/pruicss',
        requiredArgs: {
            instance: instance.instance,

        },
    });
    return (modInstances);
}

let sdfm_module = {
    displayName: "SDFM",
    description: "Sigma Delta Filter",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config:"/.meta/current_sense/templates/sdfm_template.c.xdt",
            moduleName: sdfm_module_name,
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config:"/.meta/current_sense/templates/sdfm_template.h.xdt",
            moduleName: sdfm_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: sdfm_module_name,
        },
    },
    defaultInstanceName: "CONFIG_SDFM",
    config: config,
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    validate : sd_validate.onValidate,
    pinmuxRequirements: sdfm_pins.pinmuxRequirements,
    getInterfaceName: sdfm_pins.getInterfaceName,
    getPeripheralPinNames: sdfm_pins.getPeripheralPinNames,
    sharedModuleInstances: sharedModuleInstances,

};
exports = sdfm_module;
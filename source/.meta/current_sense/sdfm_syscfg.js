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
        description: "Enable G Mux in ICSSG_SA_MX_REG Register, Few SD pins get remapped to enable different usecase",
        default: false,
        hidden:  (device == "am243x-lp" ||device == "am243x-evm" ||device == "am64x-evm") ? false : true,
    },
    {
        name: "Enable_Load_Share",
        displayName: "Enable Load Share",
        description: "Enable Load Share",
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
        description: "Enable Sdfm synchronization with ePWM module",
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
		description : 'PRU ICSS IEP Instance',
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
		description : 'IEP Reset Frequency (Hz)',
		hidden      : true,
		default     : "8000",
    },
    {
        name: "SDFM_CLK_GEN",
        displayName: "SDCLK Generation From",
        description: "SDCLK Generation From",
		hidden	: false,
        default: "2",
        options: [
                {
                    name: "0",
                    displayName: "None"
                },
                {
                    name: "1",
                    displayName: "PRU-ICSSG (PR<k>_PRU0_GPO1)"
                },
                {
                    name: "2",
                    displayName: "eCAP"
                },
                {
                    name: "3",
                    displayName: "IEP"
                },
        ],
        onChange: updateClockSourceVisibility,
    },
    {
        name: "SDFM_Clock_Value",
        displayName: "SDFM Frequency (Hz)",
        description: "Generated sdfm frequency from the selected clock source (IEP, ECAP, or GPIO1), Note: divider values need to be set manually in sdfm_example.c file.",
        hidden: true,
        default: 20000000,
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
                description : 'Select IEP Compare Event for NC Trigger Mode',
                hidden      : true,
                default: SDFM_IepCmpEvnt[0].name,
                options: SDFM_IepCmpEvnt       
            },
            {
                name        : "PRU_FirstTriggerPoint",
                displayName : "First Trigger Point (us)",
                description : 'First Trigger Point (us)',
                hidden      : true,
                default     : 15,
            },
            {
                name        : "PRU_EnableDoubleUpdate",
                displayName : "Enable Double Update",
                description : 'Enable Double Update',
                hidden      : true,
                default     : false,
                onChange    : doubleUpdateConfig,
            },
            {
                name        : "PRU_SecondTriggerPoint",
                displayName : "Second Trigger Point (us)",
                description : 'Second Trigger Point (us)',
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
                description : 'Select IEP Compare Event for NC Trigger Mode',
                hidden      : true,
                default: SDFM_IepCmpEvnt[0].name,
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
                description : 'Select IEP Compare Event for NC Trigger Mode',
                hidden      : true,
                default: SDFM_IepCmpEvnt[0].name,
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
}

function updateClockSourceVisibility(inst, ui)
{
    if (inst.SDFM_CLK_GEN == "0")
    {
        ui.SDFM_Clock_Value.hidden = true;
    }
    else
    {
        ui.SDFM_Clock_Value.hidden = false;
    }

    // Update read-only status for all channel clock fields
    for (let channel = 0; channel < 9; channel++)
    {
        if (inst["Enable_Channel_" + channel.toString()])
        {
            ui["Ch" + channel.toString() + "_SDFM_Clock"].readOnly = (inst.SDFM_CLK_GEN != "0");
        }
    }

    propagateClockToChannels(inst, ui);
}

function propagateClockToChannels(inst, ui)
{
    if (inst.SDFM_CLK_GEN != "0")
    {
        let clockValue = inst.SDFM_Clock_Value;
        for (let channel = 0; channel < 9; channel++)
        {
            if (inst["Enable_Channel_" + channel.toString()] == true)
            {
                inst["Ch" + channel.toString() + "_SDFM_Clock"] = clockValue;
            }
        }
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
        ui["Ch" + channel.toString() + "_SDFM_Clock"].readOnly = (inst.SDFM_CLK_GEN != "0");

        if (status && inst.SDFM_CLK_GEN != "0")
        {
            inst["Ch" + channel.toString() + "_SDFM_Clock"] = inst.SDFM_Clock_Value;
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
let common = system.getScript("/common");
let sdfm_pins = system.getScript("/current_sense/sdfm_pins.js");
let sdfm_clk_config = system.getScript("/current_sense/sdfm_clockConfiguration.syscfg.js");
let sdfm_fd_config = system.getScript("/current_sense/sdfm_channelConfiguration.syscfg.js");
let sd_validate = system.getScript("/current_sense/sdfm_validation.syscfg.js");

let device = common.getDeviceName();

let sdfm_module_name = "/current_sense/sdfm";

let config = [];

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
        name: "Enable_Load_Share",
        displayName: "Enable Load Share",
        description: "Enable Load Share",
        default: false,
        hidden: false,
        onChange: addSixChannel,
    },
    {
        name: "Enable_Phase_Compensation",
        displayName: "Enable Phase Compensation",
        description: "Enable Phase Compensation ",
        default: false,
        hidden: false,
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
        ]
        },

])

for(let ch = 0; ch < 3; ch++)
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

for(let ch = 3; ch < total_channel; ch++)
{
    config = config.concat([
        {
            name: "Enable_Channel_" + ch.toString(),
            displayName: "Enable Channel " + ch.toString(),
            description: 'Enable / Disable SDFM Channel',
            hidden: true,
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

function addSixChannel(inst, ui)
{
    let hideConfigs = true;
    if (inst.Enable_Load_Share == true)
    {
        hideConfigs = false;
    }
    for(let ch = 3; ch < total_channel; ch++)
    {
        ui["Enable_Channel_" + ch.toString()].hidden = hideConfigs;
    }
}

function onChangeEnableChannel(inst, ui)
{
    for (let channel = 0; channel < total_channel; channel++)
	{
		let status = inst["Enable_Channel_" + channel.toString()];

		ui["Ch" + channel.toString() + "_SDCLKSEL"].hidden = !status;
        ui["Ch" + channel.toString() + "_CLKINV"].hidden = !status;
        ui["Ch" + channel.toString() + "_SDFM Clock"].hidden = !status;
        ui["Ch" + channel.toString() + "_AccSource"].hidden = !status;
        ui["Ch" + channel.toString() + "_ComparatorEnable"].hidden = !status;
        ui["Ch" + channel.toString() + "_EnableDoubleUpdate"].hidden = !status;
        ui["Ch" + channel.toString() + "_NC_OSR"].hidden = !status;
        ui["Ch" + channel.toString() + "_EnableContinuousMode"].hidden = !status;
        ui["Ch" + channel.toString() + "_FirstTriggerPoint"].hidden = !status;
        ui["Ch" + channel.toString() + "_FastDetect"].hidden = !status;
        ui["Ch" + channel.toString() + "_Datarate_DF"].hidden = !status;
        ui["Ch" + channel.toString() + "_Latency_DF"].hidden = !status;
        ui["Ch" + channel.toString() + "_Min_FilterOutput"].hidden = !status;
        ui["Ch" + channel.toString() + "_Max_FilterOutput"].hidden = !status;
        ui["Ch" + channel.toString() + "_EPWM_SYNC"].hidden = !status;
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
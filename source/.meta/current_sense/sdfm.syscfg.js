let common = system.getScript("/common");
let sdfm_pins = system.getScript("/current_sense/sdfm_pins.js");
let device = common.getDeviceName();
let is_am243x_lp_device = (device === "am243x-lp") ? true : false;

let sdfm_module_name = "/current_sense/sdfm";

/*function onValidate(inst, validation) {
    for (let instance_index in inst.$module.$instances)
    {
       let instance = inst.$module.$instances[instance_index];
        //if ((!instance.channel_0)&&(!instance.channel_1)&&(!instance.channel_2)&&(!instance.channel_3)&&(!instance.channel_4)&&(!instance.channel_5)&&(!instance.channel_6)&&(!instance.channel_7)&&(!instance.channel_8))
        //    validation.logError("Select atleast one channel",inst,"channel_0"
        //)
    }
}*/

let sdfm_module = {
    displayName: "SDFM",
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
    config: [
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
            name: "Channel_0",
            displayName: "Enable Channel 0",
            description: "Channel 0 Selection",
            default: true,
        },
        {
            name: "Channel_1",
            displayName: "Enable Channel 1",
            description: "Channel 1 Selection ",
            default: true,
        },
        {
            name: "Channel_2",
            displayName: "Enable Channel 2",
            description: "Channel 2 Selection ",
            default: true,
        },

        {
            name: "Channel_3",
            displayName: "Enable Channel 3",
            description: "Channel 3 Selection",
            default: false,
        },
        {
            name: "Channel_4",
            displayName: "Enable Channel 4",
            description: "Channel 4 Selection ",
            default: false,
        },
        {
            name: "Channel_5",
            displayName: "Enable Channel 5",
            description: "Channel 5 Selection ",
            default: false,
        },
        {
            name: "Channel_6",
            displayName: "Enable Channel 6",
            description: "Channel 6 Selection",
            default: false,
        },
        {
            name: "Channel_7",
            displayName: "Enable Channel 7",
            description: "Channel 7 Selection ",
            default: false,
        },
        {
            name: "Channel_8",
            displayName: "Enable Channel 8",
            description: "Channel 8 Selection ",
            default: false,
        },

    ],
    moduleStatic: {
        modules: function(inst) {
            return [{
                name: "system_common",
                moduleName: "/system_common",
            }]
        },
    },
    pinmuxRequirements: sdfm_pins.pinmuxRequirements,
    getInterfaceName: sdfm_pins.getInterfaceName,
    getPeripheralPinNames: sdfm_pins.getPeripheralPinNames,
    sharedModuleInstances: sharedModuleInstances,
    /*validate: onValidate,*/

};

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

exports = sdfm_module;
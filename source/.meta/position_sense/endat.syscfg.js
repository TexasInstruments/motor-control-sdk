
let common = system.getScript("/common");
let hdsl_endat_pins = system.getScript("/position_sense/hdsl_endat_pins.js");
let endat_module_name = "/position_sense/endat";
let device = common.getDeviceName();
function onValidate(inst, validation) {

    for (let instance_index in inst.$module.$instances)
    {
        let instance = inst.$module.$instances[instance_index];
        /* select atleast one cahnnel */
        if ((!instance.Channel_0)&&(!instance.Channel_2)&&(!instance.Channel_1))
            validation.logError("Select atleast one channel",inst,"Channel_0");

        /* channel 0 and channel 2 are supported on am243x-lp*/
        if((device=="am243x-lp") && (instance.Channel_1))
        {
            validation.logError("Channel 1 is not supported on am243x-lp",inst,"Channel_1");
        }

        /* validation for booster pack */
        if((device!="am243x-lp")&&(instance.Booster_Pack))
        {
            validation.logError("Select only when using Booster Pack with LP",inst,"Booster_Pack");
        }
    }

}



let endat_module = {
    displayName: "EnDat Position Encoder",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config:"/.meta/position_sense/templates/endat_templates.h.xdt",
            moduleName: endat_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: endat_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ENDAT",
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
            default: false,
        },
        {
            name: "Channel_2",
            displayName: "Enable Channel 2",
            description: "Channel 2 Selection ",
            default: false,
        },

        {
            name: "Multi_Channel_Load_Share",
            displayName: "Multi Channel Load Share",
            description: "Selected Channels have different make",
            default: false,
        },
        {
            name: "Booster_Pack",
            displayName: "Booster Pack",
            description: "Only for Booster Pack",
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
    pinmuxRequirements: hdsl_endat_pins.pinmuxRequirements,
    getInterfaceName: hdsl_endat_pins.getInterfaceName,
    getPeripheralPinNames: hdsl_endat_pins.getPeripheralPinNames,
    sharedModuleInstances: sharedModuleInstances,
    moduleInstances: moduleInstances,
    validate: onValidate,

};

function moduleInstances(instance){
    let modInstances = new Array();
    if(device == "am243x-lp")
    {
       modInstances.push({
            name: "ENC1_EN",
            displayName: "Booster Pack Ch0 Enable Pin",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT",
                
            },
        });
        modInstances.push({
            name: "ENC2_EN",
            displayName: "Booster Pack Ch2 Enable Pin",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT",
            },
        });
    }
    return (modInstances);
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

exports = endat_module;


let common = system.getScript("/common");
let tamagawa_pins = system.getScript("/position_sense/tamagawa_pins.js");
let device = common.getDeviceName();
let is_am243x_lp_device = (device === "am243x-lp") ? true : false;

let tamagawa_module_name = "/position_sense/tamagawa";

function onValidate(inst, validation) {
    for (let instance_index in inst.$module.$instances)
    {
       let instance = inst.$module.$instances[instance_index];
        if ((!instance.channel_0)&&(!instance.channel_1)&&(!instance.channel_2))
            validation.logError(
               "Select atleast one channel",inst,"channel_0"
        );
            
        /* channel 0 and channel 2 are supported on am243x-lp*/
        if((device=="am243x-lp") && (instance.channel_1))
        {
            validation.logError("Channel 1 is not supported on am243x-lp",inst,"channel_1");
        }
        
        /* validation for booster pack */
        if((device!="am243x-lp")&&(instance.Booster_Pack))
        {
            validation.logError("Select only when using Booster Pack with LP",inst,"Booster_Pack");
        }
    }
}

let tamagawa_module = {

    displayName: "Tamagawa Position Encoder",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/position_sense/tamagawa/tamagawa.h.xdt",
            moduleName: tamagawa_module_name,
        },

        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: tamagawa_module_name,
        },
    },
    defaultInstanceName: "CONFIG_TAMAGAWA",
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
            name: "channel_0",
            displayName: "Select Channel 0",
            description: "Channel 0 Selection ",
            default: true,

        },
        {
            name: "channel_1",
            displayName: "Select Channel 1",
            description: "Channel 1 Selection ",
            default: false,

        },
        {
            name: "channel_2",
            displayName: "Select Channel 2",
            description: "Channel 2 Selection ",
            default: false,

        },
        {
            name: "baudrate",
            displayName: "Select Baud Rate(in Mbps)",
            description: "Data Speed Selection ",
            default: 2.5,
            options: [
                {
                    name: 2.5,
                },
                {
                    name: 5,
                },
            ],
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
    pinmuxRequirements: tamagawa_pins.pinmuxRequirements,
    getInterfaceName: tamagawa_pins.getInterfaceName,
    getPeripheralPinNames: tamagawa_pins.getPeripheralPinNames,
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
            coreClk: 200*1000000,
        },
    });
    return (modInstances);
}

exports = tamagawa_module;

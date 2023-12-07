
let common = system.getScript("/common");
let bissc_pins = system.getScript("/position_sense/bissc_pins.js");
let device = common.getDeviceName();
let is_am243x_lp_device = (device === "am243x-lp") ? true : false;

let bissc_module_name = "/position_sense/bissc";

function onValidate(inst, validation) {
    for (let instance_index in inst.$module.$instances)
    {
       let instance = inst.$module.$instances[instance_index];
        if ((!instance.channel_0)&&(!instance.channel_1)&&(!instance.channel_2))
            validation.logError("Select atleast one channel",inst,"channel_0"
        );

        /* validation for booster pack */
        if((device!="am243x-lp")&&(instance.Booster_Pack))
        {
            validation.logError("Select only when using Booster Pack with LP",inst,"Booster_Pack");
        }
    }
}

let bissc_module = {

    displayName: "BiSSC Position Encoder",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/position_sense/bissc/bissc.h.xdt",
            moduleName: bissc_module_name,
        },

        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: bissc_module_name,
        },
    },
    defaultInstanceName: "CONFIG_BISSC",
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
            default: 1,
            options: [
                {
                    name: 1,
                },
                {
                    name: 2,
                },
                {
                    name: 5,
                },
                {
                    name: 8,
                },
                {
                    name: 10,
                },
            ],
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
    pinmuxRequirements: bissc_pins.pinmuxRequirements,
    getInterfaceName: bissc_pins.getInterfaceName,
    getPeripheralPinNames: bissc_pins.getPeripheralPinNames,
    sharedModuleInstances: sharedModuleInstances,
    validate: onValidate,
};

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
    if(device == "am243x-lp")
    {
       modInstances.push({
            name: "ENC0_EN",
            displayName: "Booster Pack Ch0 Enable Pin",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT"
            },
        });
        modInstances.push({
            name: "ENC1_EN",
            displayName: "Booster Pack Ch1 Enable Pin",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT"
            },
        });
        modInstances.push({
            name: "ENC2_EN",
            displayName: "Booster Pack Ch2 Enable Pin",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT"
            },
        });
        modInstances.push({
            name: "BISSC_CH0_OUT_EN",
            displayName: "ENDAT Ch0 TX Enable Pin",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT"
            },
        });
        modInstances.push({
            name: "BISSC_CH1_OUT_EN",
            displayName: "ENDAT Ch1 TX Enable Pin",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT"
            },
        });
        modInstances.push({
            name: "BISSC_CH2_OUT_EN",
            displayName: "ENDAT Ch2 TX Enable Pin",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT"
            },
        });
    }
    return (modInstances);
}

exports = bissc_module;

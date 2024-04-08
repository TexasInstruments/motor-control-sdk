
let common = system.getScript("/common");
let nikon_pins = system.getScript("/position_sense/nikon_pins.js");
let device = common.getDeviceName();
let nikon_module_name = "/position_sense/nikon";

function onValidate(inst, validation)
{
    for (let instance_index in inst.$module.$instances)
    {
       let instance = inst.$module.$instances[instance_index];
        if ((!instance.channel_0)&&(!instance.channel_1)&&(!instance.channel_2))
        {
            validation.logError("Select atleast one channel",inst,"channel_0");
        }
        if((device=="am243x-lp") && (instance.channel_1 ))
        {
            validation.logError("On AM243x-LP, Channel 1 is not supported",inst,"channel_1");
        }
        /* validation for booster pack */
        if((device!="am243x-lp")&&(instance.Booster_Pack))
        {
            validation.logError("Select only when using Booster Pack with LP",inst,"Booster_Pack");
        }
    }
}

let nikon_module = {

    displayName: "Nikon Position Encoder",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/position_sense/nikon/nikon.h.xdt",
            moduleName: nikon_module_name,
        },

        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: nikon_module_name,
        },
    },
    defaultInstanceName: "CONFIG_NIKON",
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
            default: 4,
            options: [
                {
                    name: 2.5,
                },
                {
                    name: 4,
                },
                {
                    name: 20/3,
                },
                {
                    name: 8,
                },
                {
                    name: 16,
                },
            ],
        },
        {
            name: "coreClk",
            displayName: "PRU-ICSS Core Clk (Hz)",
            default: 200*1000000,
            options: [
                {
                    name: 200*1000000,
                },

                {
                    name: 300*1000000,
                },

                {
                    name: 333.333333*1000000,
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
    pinmuxRequirements: nikon_pins.pinmuxRequirements,
    getInterfaceName: nikon_pins.getInterfaceName,
    getPeripheralPinNames: nikon_pins.getPeripheralPinNames,
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
            coreClk: instance.coreClk,
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
            name: "ENC2_EN",
            displayName: "Booster Pack Ch2 Enable Pin",
            moduleName: "/drivers/gpio/gpio",
            requiredArgs: {
                pinDir: "OUTPUT"
            },
        });
    }
    return (modInstances);
}

exports = nikon_module;


let common = system.getScript("/common");
let device = common.getDeviceName();
let is_am243x_lp_device = (device === "am243x-lp") ? true : false;
let is_am26x_soc = (device === "am263x-cc" || device === "am261x-lp") ? true : false;
let is_am263x_soc = (device === "am263x-cc") ? true : false;
let is_am261x_soc = (device === "am261x-lp") ? true : false;
let tamagawa_pins = (is_am26x_soc) ? system.getScript("/position_sense/tamagawa/am26x_pins.js") : system.getScript("/position_sense/tamagawa/am24x_pins.js");

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
        if((device!="am243x-lp" && device != "am263x-cc" &&  device != "am261x-lp")&&(instance.Booster_Pack))
        {
            validation.logError("Select only when using Booster Pack with LP",inst,"Booster_Pack");
        }

        if(is_am26x_soc)
        {
            if(is_am263x_soc)
            {
                if(instance.PRU_Slice == "PRU0" && instance.channel_2)
                {
                    validation.logWarning("Channel2 TX EN signal is not pinned out at the device level", inst, "channel_2");
                }
                
                if((instance.channel_2 || instance.channel_0)&&(instance.Booster_Pack))
                {
                    validation.logError("Channel0 and Channel2 are not supported on Booster Pack",inst,"Booster_Pack");
                }
            }
            if(is_am261x_soc)
            {
                
                if((instance.channel_2 || instance.channel_1)&&(instance.Booster_Pack))
                {
                    validation.logError("Channel1 and Channel2 are not supported on Booster Pack",inst,"Booster_Pack");
                }
            }
            
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
            default: (is_am261x_soc) ? "ICSSM1" : ((is_am263x_soc) ? "ICSSM" : "ICSSG0"),
            options: (is_am261x_soc) ?
                        [
                            {
                                name: "ICSSM0",
                            },
                            {
                                name: "ICSSM1",
                            }
                        ]
                        :
                        ((is_am263x_soc) ?
                        [
                            {
                            name: "ICSSM",
                            displayName:"ICSSM0"
                            }
                        ]
                        :
                        [
                            {
                                name: "ICSSG0",
                            },
                            {
                                name: "ICSSG1",
                            }
                        ])
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
            default: 2500000,
            options: [
                {
                    name: 2500000,
                },
                {
                    name: 5000000,
                },
            ],
        },
        {
            name: "Booster_Pack",
            displayName: "Booster Pack",
            description: "Only for Booster Pack",
            default: false,
        },

        {
            name: "PRU_Slice",
            displayName: "Select PRU Slice",
            description: "ICSSM PRU Slice",
            hidden :(is_am26x_soc) ? false : true,
            default: "PRU1",
            options: [
                {
                    name: "PRU0",
                },
                {
                    name: "PRU1",
                },
            ],
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
    let BoosterPack = instance["Booster_Pack"];

    if(device == "am243x-lp" || is_am26x_soc)
    {
        if(BoosterPack)
        {
            modInstances.push({
                name: "ENC1_EN",
                displayName: "Booster Pack Axis1 Power Enable Pin",
                moduleName: "/drivers/gpio/gpio",
                requiredArgs: {
                    pinDir: "OUTPUT",
                    defaultValue: "1",
                },
            });
            if(device == "am243x-lp")
            {
                modInstances.push({
                    name: "ENC2_EN",
                    displayName: "Booster Pack Axis2 Power Enable Pin",
                    moduleName: "/drivers/gpio/gpio",
                    requiredArgs: {
                        pinDir: "OUTPUT",
                        defaultValue: "1",
                    },
                });
            }
        }
        if(is_am263x_soc)
        {
            modInstances.push({
                name: "PRU_MUX_SEL_GPIO64",
                displayName: "Select line for PRU MUX",
                moduleName: "/drivers/gpio/gpio",
                requiredArgs: {
                    pinDir: "OUTPUT",
                    defaultValue: "1",

                },
            });
            modInstances.push({
                name: "MUX_EN_GPIO58",
                displayName: "MUX Enable Pin",
                moduleName: "/drivers/gpio/gpio",
                requiredArgs: {
                    pinDir: "OUTPUT",
                    defaultValue: "1",
                },
            });
        }

    }

    return (modInstances);
}

function sharedModuleInstances(instance) {
    let modInstances = new Array();
    let requiredArgs = (is_am263x_soc) ? {instance:`${instance.instance}0`} : {instance: instance.instance};

    modInstances.push({
        name: "pru",
        displayName: "PRU ICSS Configuration",
        moduleName: '/drivers/pruicss/pruicss',
        requiredArgs
    });
    return (modInstances);
}

exports = tamagawa_module;

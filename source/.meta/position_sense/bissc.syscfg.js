let common = system.getScript("/common");
let device = common.getDeviceName();
let bissc_module_name = "/position_sense/bissc";
let is_am243x_lp_device = (device === "am243x-lp") ? true : false;
let is_am26x_soc = (device === "am261x-lp" || device === "am263x-cc" || device === "am263px-cc") ? true : false;
let is_am263x_soc = (device === "am263x-cc") ? true : false;
let is_am261x_soc = (device === "am261x-lp") ? true : false;
let is_am263px_soc = (device === "am263px-cc") ? true : false;
let bissc_pins = (is_am26x_soc) ? system.getScript("/position_sense/bissc/am26x_pins.js") : system.getScript("/position_sense/bissc_pins.js");

function onValidate(inst, validation) {
    for (let instance_index in inst.$module.$instances)
    {
       let instance = inst.$module.$instances[instance_index];
        if ((!instance.channel_0)&&(!instance.channel_1)&&(!instance.channel_2))
        {
            validation.logError("Select atleast one channel",inst,"channel_0");
        }
        if((device === "am243x-lp") && (instance.channel_1 ))
        {
            validation.logError( "On AM243x-LP, Channel 1 is not supported",inst,"channel_1" );
        }
        /* validation for booster pack */
        if((device!="am243x-lp" && device!= "am263x-cc" &&  device!= "am261x-lp" && device != "am263px-cc" )&&(instance.Booster_Pack))
        {
            validation.logError("Select only when using Booster Pack with LP",inst,"Booster_Pack");
        }
        if(is_am26x_soc)
            {
                if(is_am263x_soc || is_am263px_soc)
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
            default: (is_am261x_soc) ? "ICSSM1" : ((is_am263x_soc || is_am263px_soc) ? "ICSSM" : "ICSSG0"),
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
                        ((is_am263x_soc || is_am263px_soc) ?
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
            name: "Tx_Rx_Clk_Source",
            displayName: "TX RX FIFO Clock Source",
            description: "TX RX FIFO Clock Source Options",
            default: "0",
            options: [
                {
                    name: "0",
                    displayName: "ICSS UART Clock",
                },
                {
                    name: "1",
                    displayName: "ICSS Core Clock",
                },
            ],
        },
        {
            name: "G_MUX_EN",
            displayName: "Enable G MUX ",
        description: "Enable G mux in ICSSG_SA_MX_REG Register, Few 3 channel Peripheral pins get remapped to enable different usecase",
            default: false,
            hidden:  (device == "am243x-lp" ||device == "am243x-evm" ||device == "am64x-evm") ? false : true,
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
            hidden :(is_am26x_soc) ? true : false,
            default: false,
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
    pinmuxRequirements: bissc_pins.pinmuxRequirements,
    getInterfaceName: bissc_pins.getInterfaceName,
    getPeripheralPinNames: bissc_pins.getPeripheralPinNames,
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
                name: "ENC0_EN",
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
            modInstances.push({
                name: "BISSC_CH0_OUT_EN",
                displayName: "BISSC Ch0 TX Enable Pin",
                moduleName: "/drivers/gpio/gpio",
                requiredArgs: {
                    pinDir: "OUTPUT"
                },
            });
            if(device == "am243x-lp")
            {
            modInstances.push({
                name: "BISSC_CH2_OUT_EN",
                displayName: "BISSC Ch2 TX Enable Pin",
                moduleName: "/drivers/gpio/gpio",
                requiredArgs: {
                    pinDir: "OUTPUT"
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
    let requiredArgs = (is_am263x_soc || is_am263px_soc) ? {instance:`${instance.instance}0`} : (is_am261x_soc) ? {instance:`${instance.instance}`} : {instance: instance.instance};

    modInstances.push({
        name: "pru",
        displayName: "PRU ICSS Configuration",
        moduleName: '/drivers/pruicss/pruicss',
        requiredArgs
    });
    return (modInstances);
}

exports = bissc_module;

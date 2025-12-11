
let common = system.getScript("/common");
let endat3_module_name = "/position_sense/endat3";
let device = common.getDeviceName();
let is_am26x_soc = (device === "am263x-cc" || device === "am261x-lp" || device === "am263px-cc") ? true : false;
let is_am263x_soc = (device === "am263x-cc") ? true : false;
let is_am261x_soc = (device === "am261x-lp") ? true : false;
let is_am263px_soc = (device === "am263px-cc") ? true : false;
let hdsl_endat3_pins = (is_am26x_soc) ? system.getScript("/position_sense/endat3/am26x_pins.js") : system.getScript("/position_sense/hdsl_endat_pins.js");

function onValidate(inst, validation)
{
    for (let instance_index in inst.$module.$instances)
    {
       let instance = inst.$module.$instances[instance_index];

        /* Validate that at least one channel is selected */
        if ((!instance.channel_0)&&(!instance.channel_1)&&(!instance.channel_2))
        {
            validation.logError("Select atleast one channel",inst,"channel_0");
        }

        /* Calculate total channels for validation */
        let total_channels = (instance.channel_0 ? 1 : 0) + (instance.channel_1 ? 1 : 0) + (instance.channel_2 ? 1 : 0);

        /* Only single channel supported */
        if(total_channels > 1)
        {
            validation.logError("Only single channel operation per PRU core/slice", inst, "channel_0");
        }

        if((device === "am243x-lp") && (instance.channel_1) && (instance.Booster_Pack))
        {
            validation.logError("Channel 1 is not supported with BP-AM2BLDCSERVO BoosterPack", inst, "Booster_Pack");
        }

        /* validation for BP-AM2BLDCSERVO BoosterPack */
        if((device!="am243x-lp" && device!= "am263x-cc" &&  device!= "am261x-lp" && device != "am263px-cc" )&&(instance.Booster_Pack))
        {
            validation.logError("Select only when using BP-AM2BLDCSERVO BoosterPack with LP", inst, "Booster_Pack");
        }

        if(is_am26x_soc)
        {
            if(is_am263x_soc || is_am263px_soc)
            {
                if(instance.PRU_Slice == "PRU0" && instance.channel_2)
                {
                    validation.logError("Channel 2 TX signal is not pinned out at the device level", inst, "channel_2");
                }

                if((instance.channel_2 || instance.channel_0)&&(instance.Booster_Pack))
                {
                    validation.logError("Channel 0 and Channel 2 are not supported with BP-AM2BLDCSERVO BoosterPack",inst,"Booster_Pack");
                }
            }
            if(is_am261x_soc)
            {

                if((instance.channel_2 || instance.channel_1)&&(instance.Booster_Pack))
                {
                    validation.logError("Channel 1 and Channel 2 are not supported with BP-AM2BLDCSERVO BoosterPack",inst,"Booster_Pack");
                }
            }
        }
    }
}

let endat3_module = {
    displayName: "EnDat3 Position Encoder",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/position_sense/endat3/endat3.c.xdt",
            moduleName: endat3_module_name,
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/position_sense/endat3/endat3.h.xdt",
            moduleName: endat3_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: endat3_module_name,
        },
    },
    defaultInstanceName: "CONFIG_ENDAT3",
    config: [
        {
            name: "instance",
            displayName: "Instance",
            default: (is_am261x_soc) ? "ICSSM1" : ((is_am263x_soc || is_am263px_soc ) ? "ICSSM" : "ICSSG0"),
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
            name: "G_MUX_EN",
            displayName: "Enable G MUX ",
            description: "Enable G mux in ICSSG_SA_MX_REG Register, Few 3 channel Peripheral pins get remapped to enable different usecase",
            default: false,
            hidden:  (device == "am243x-lp" || device == "am243x-evm" || device == "am64x-evm") ? false : true,
        },
        {
            name: "Tx_Rx_Clk_Source",
            displayName: "TX RX FIFO Clock Source",
            description: "TX RX FIFO Clock Source Options. See module specific page in SDK documentation for more details and known limitations.",
            default: "1",
            options: [
                {
                    name: "1",
                    displayName: "ICSS Core Clock",
                },
            ],
        },
        {
            name: "baudrate",
            displayName: "Select Baud Rate(in Mbps)",
            description: "EnDAT3 Communication Baud Rate",
            default: 12,
            options: [
                {
                    name: 12,
                    displayName: "12.5 Mbps",
                },
            ],
        },
        {
            name: "channel_0",
            displayName: "Enable Channel 0",
            description: "Channel 0 Selection",
            default: true,
        },
        {
            name: "channel_1",
            displayName: "Enable Channel 1",
            description: "Channel 1 Selection ",
            default: false,
        },
        {
            name: "channel_2",
            displayName: "Enable Channel 2",
            description: "Channel 2 Selection ",
            default: false,
        },

        {
            name: "Booster_Pack",
            displayName: "Using BP-AM2BLDCSERVO BoosterPack",
            description: "Only for BP-AM2BLDCSERVO BoosterPack",
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
    pinmuxRequirements: hdsl_endat3_pins.pinmuxRequirements,
    getInterfaceName: hdsl_endat3_pins.getInterfaceName,
    getPeripheralPinNames: hdsl_endat3_pins.getPeripheralPinNames,
    sharedModuleInstances: sharedModuleInstances,
    moduleInstances: moduleInstances,
    validate: onValidate,

};

function moduleInstances(instance){
    let modInstances = new Array();
    let BoosterPack = instance["Booster_Pack"];
    if((device == "am243x-lp") || is_am26x_soc)
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
    let requiredArgs = (is_am263x_soc || is_am263px_soc) ? {instance:`${instance.instance}0`} : {instance: instance.instance};
    modInstances.push({
        name: "pru",
        displayName: "PRU ICSS Configuration",
        moduleName: '/drivers/pruicss/pruicss',
        requiredArgs
    });
    return (modInstances);
}

exports = endat3_module;


let common = system.getScript("/common");
let device = common.getDeviceName();
let nikon_module_name = "/position_sense/nikon";
let is_am243x_lp_device = (device === "am243x-lp") ? true : false;
let is_am26x_soc = (device === "am261x-lp" || device === "am263x-cc" || device === "am263px-cc" ) ? true : false;
let is_am263x_soc = (device === "am263x-cc") ? true : false;
let is_am261x_soc = (device === "am261x-lp") ? true : false;
let is_am263px_soc = (device === "am263px-cc") ? true : false;
let nikon_pins = (is_am26x_soc) ? system.getScript("/position_sense/nikon/am26x_pins.js") : system.getScript("/position_sense/nikon_pins.js");

function onValidate(inst, validation)
{
    for (let instance_index in inst.$module.$instances)
    {
       let instance = inst.$module.$instances[instance_index];
        if ((!instance.channel_0)&&(!instance.channel_1)&&(!instance.channel_2))
        {
            validation.logError("Select atleast one channel", inst, "channel_0");
        }
        if((device === "am243x-lp") && (instance.channel_1) && (instance.Booster_Pack))
        {
            validation.logError("Channel 1 is not supported on BP-AM2BLDCSERVO BoosterPack due to pinout limitations", inst, "Booster_Pack");
        }
        /* validation for BP-AM2BLDCSERVO BoosterPack */
        if(( device!="am243x-lp" && device!= "am263x-cc" &&  device!= "am261x-lp" && device != "am263px-cc")&&(instance.Booster_Pack))
        {
            validation.logError("Select only when using BP-AM2BLDCSERVO BoosterPack with LP", inst, "Booster_Pack");
        }

        if(is_am26x_soc)
            {
                if(is_am263x_soc || is_am263px_soc)
                {
                    if(instance.PRU_Slice == "PRU0" && instance.channel_2)
                    {
                        validation.logError("Channel 2 signal is not pinned out at the device level", inst, "channel_2");
                    }

                    if((instance.channel_2 || instance.channel_0)&&(instance.Booster_Pack))
                    {
                        validation.logError("Channel 0 and Channel 2 are not supported on BP-AM2BLDCSERVO BoosterPack due to pinout limitations", inst, "Booster_Pack");
                    }
                }
                if(is_am261x_soc)
                {

                    if((instance.channel_2 || instance.channel_1)&&(instance.Booster_Pack))
                    {
                        validation.logError("Channel 1 and Channel 2 are not supported on BP-AM2BLDCSERVO BoosterPack due to pinout limitations", inst, "Booster_Pack");
                    }
                }

            }
    }
}

let nikon_module = {

    displayName: "Nikon Position Encoder",
    templates: {
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/position_sense/nikon/nikon.c.xdt",
            moduleName: nikon_module_name,
        },
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
            description: "TX RX FIFO Clock Source Options. See module specific page in SDK documentation for more details and known limitations.",
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
            displayName: "Enable G Mux",
            description: "Enable G Mux in ICSSG_SA_MX_REG Register, Few 3 channel Peripheral pins get remapped to enable different usecase",
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
            name: "protocolVersion",
            displayName: "Select Nikon Protocol Version",
            description: "Nikon Protocol Version",
            default: 2.1,
            options: [
                {
                    name: 2.1,
                },
                {
                    name: 3.0,
                },
            ],
        },
        (is_am243x_lp_device) ?
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
        }
        :
        {
            name: "coreClk",
            displayName: "PRU-ICSS Core Clk (Hz)",
            default: 200*1000000,
            options: [
                {
                    name: 200*1000000,
                }
            ],
            hidden: true,
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
    pinmuxRequirements: nikon_pins.pinmuxRequirements,
    getInterfaceName: nikon_pins.getInterfaceName,
    getPeripheralPinNames: nikon_pins.getPeripheralPinNames,
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
                displayName: "BP-AM2BLDCSERVO BoosterPack Axis1 Power Enable Pin",
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
                    displayName: "BP-AM2BLDCSERVO BoosterPack Axis2 Power Enable Pin",
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
    let requiredArgs = (is_am263x_soc || is_am263px_soc) ? {instance:`${instance.instance}0`} : (is_am261x_soc) ? {instance:`${instance.instance}`} : {instance: instance.instance ,coreClk:instance.coreClk};

    modInstances.push({
        name: "pru",
        displayName: "PRU ICSS Configuration",
        moduleName: '/drivers/pruicss/pruicss',
        requiredArgs
    });

    return (modInstances);
}

exports = nikon_module;

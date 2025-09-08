
let common = system.getScript("/common");
let hdsl_module_name = "/position_sense/hdsl";
let device = common.getDeviceName();
let is_am261x_soc = (device === "am261x-lp") ? true : false;
let hdsl_endat_pins = (is_am261x_soc) ? system.getScript("/position_sense/hdsl/am26x_pins.js") : system.getScript("/position_sense/hdsl_endat_pins.js");


function onValidate(inst, validation) {

    for (let instance_index in inst.$module.$instances)
    {
        let instance = inst.$module.$instances[instance_index];
        /* select atleast one cahnnel */
        if ((!instance.channel_0)&&(!instance.channel_2)&&(!instance.channel_1))
            validation.logError("Select atleast one channel",inst,"channel_0");

        if((device === "am243x-lp") && (instance.channel_1) && (instance.Booster_Pack))
        {
            validation.logError("Channel 1 is not supported with BP-AM2BLDCSERVO BoosterPack",inst,"Booster_Pack");
        }

        /* validation for BP-AM2BLDCSERVO BoosterPack */
        if((device!="am243x-lp" && device!="am261x-lp")&&(instance.Booster_Pack))
        {
            validation.logError("Select only when using BP-AM2BLDCSERVO BoosterPack with LP",inst,"Booster_Pack");
        }
        if(is_am261x_soc)
        {

            if((instance.Channel_2 || instance.Channel_1)&&(instance.Booster_Pack))
            {
                validation.logError("Channel 1 and Channel 2 are not supported with BP-AM2BLDCSERVO BoosterPack",inst,"Booster_Pack");
            }
        }
    }

}

let hdsl_module = {

    displayName: "HDSL Position Encoder",
    templates: {
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/position_sense/hdsl/hdsl_templates.h.xdt",
            moduleName: hdsl_module_name,
        },
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: hdsl_module_name,
        },
    },
    defaultInstanceName: "CONFIG_HDSL",
    config: [
        {
            name: "instance",
            displayName: "Instance",
            default: (is_am261x_soc) ? "ICSSM1"  : "ICSSG0",
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
                        [
                            {
                                name: "ICSSG0",
                            },
                            {
                                name: "ICSSG1",
                            }
                        ]
        },
        {
            name: "G_MUX_EN",
            displayName: "Enable G MUX ",
            description: "Enable G mux in ICSSG_SA_MX_REG Register, Few 3 channel Peripheral pins get remapped to enable different usecase",
            default: true,
            hidden:  (device == "am243x-lp" ||device == "am243x-evm" ||device == "am64x-evm") ? false : true,
        },
        {
            name: "coreClk",
            displayName: "PRU-ICSS Core Clk (Hz)",
            default: 225*1000000,
            options: [
                {
                    name: 225*1000000,
                },

                {
                    name: 300*1000000,
                },

            ],
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
            hidden :(is_am261x_soc) ? false : true,
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
        {
            name: "Mode",
            displayName: "Select Mode",
            description: "Mode selection",
            default: "FREE_RUN",
            options: [
                {
                    name: "FREE_RUN",
                },
                {
                    name: "SYNC",
                },
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
    let BoosterPack = instance["Booster_Pack"];
    if(device == "am243x-lp" || is_am261x_soc)
    {
        if(BoosterPack)
        {
            modInstances.push({
                name: "ENC1_EN",
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
    }
    return (modInstances);
}

function sharedModuleInstances(instance) {
    let modInstances = new Array();
    let requiredArgs =  (is_am261x_soc) ? {instance:`${instance.instance}`} : {instance: instance.instance ,coreClk:instance.coreClk};

    modInstances.push({
        name: "pru",
        displayName: "PRU ICSS Configuration",
        moduleName: '/drivers/pruicss/pruicss',
        requiredArgs
    });
    return (modInstances);
}

exports = hdsl_module;

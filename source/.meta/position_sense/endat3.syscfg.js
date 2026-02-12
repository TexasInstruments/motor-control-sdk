
let common = system.getScript("/common");
let endat3_module_name = "/position_sense/endat3";
let device = common.getDeviceName();
let is_am26x_soc = (device === "am263x-cc" || device === "am261x-lp" || device === "am263px-cc") ? true : false;
let is_am263x_soc = (device === "am263x-cc") ? true : false;
let is_am261x_soc = (device === "am261x-lp") ? true : false;
let is_am263px_soc = (device === "am263px-cc") ? true : false;
let hdsl_endat3_pins = (is_am26x_soc) ? system.getScript("/position_sense/endat3/am26x_pins.js") : system.getScript("/position_sense/hdsl_endat_pins.js");

function validateCmpEventWarnings(instance, inst, validation) {
    /* Helper function to check and warn about CMP0 and CMP1 usage */
    const checkCmpEvent = (cmpEvent, fieldName) => {
        if (cmpEvent === 0) {
            validation.logWarning(
                "CMP0 is used for IEP counter reset in periodic CMP mode. Using CMP0 may cause conflicts.",
                inst,
                fieldName
            );
        } else if (cmpEvent === 1) {
            validation.logWarning(
                "CMP1 is used for SYNC OUT0 generation in periodic CAP mode. Using CMP1 may cause conflicts if CAP mode is used.",
                inst,
                fieldName
            );
        }
    };

    /* Single-channel mode - check global CMP event */
    checkCmpEvent(instance.CMP_Event_Num, "CMP_Event_Num");
}

function onValidate(inst, validation)
{
    for (let instance_index in inst.$module.$instances)
    {
       let instance = inst.$module.$instances[instance_index];

        /* Validate CMP0 and CMP1 usage warnings */
        validateCmpEventWarnings(instance, inst, validation);

        /* Validate that at least one channel is selected */
        if ((!instance.channel_0)&&(!instance.channel_1)&&(!instance.channel_2))
        {
            validation.logError("Select atleast one channel", inst, "channel_0");
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
            validation.logError("Channel 1 is not supported on BP-AM2BLDCSERVO BoosterPack due to pinout limitations", inst, "Booster_Pack");
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
            longDescription : `The G_MUX_EN bit (bit 7) in the ICSSG_SA_MX_REG register is a multiplexer control bit that enables alternative pin configurations for the PRUICSS Peripheral Interface mode. 
                               This bit allows remapping of receive pins to support different hardware configurations and use cases.
                                
#### Pin Multiplexing Behavior

| G_MUX_EN Value | Channel 0 Receive | Channel 1 Receive | Channel 2 Transmit | Description                    |
|----------------|-------------------|-------------------|--------------------|--------------------------------|
| 0 (Default)    | GPI9              | GPI10             | GPO7               | Default pinmux configuration   |
| 1 (Enabled)    | GPI13             | GPI14             | GPO12              | Alternate pinmux configuration |`,
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
        {
            name: "Periodic_Trigger_Mode_Config",
            displayName: "Periodic Trigger Mode Configuration",
            description: "Configure periodic trigger mode using IEP CMP and CAP events.",
            config: [
                {
                    name: "IEP_Instance",
                    displayName: "Select IEP Instance",
                    description: "Select IEP instance for periodic trigger",
                    default: "IEP0",
                    options: (is_am26x_soc) ?
                        [
                            {
                                name: "IEP0",
                            },
                        ]
                        :
                        [
                            {
                                name: "IEP0",
                            },
                            {
                                name: "IEP1",
                            },
                        ],
                },
                {
                    name: "CMP_Event_Num",
                    displayName: "IEP CMP Event Number",
                    description: "CMP event number (0-15) for periodic CMP trigger mode",
                    default: 2,
                    options: [
                        { name: 0, displayName: "CMP0" },
                        { name: 1, displayName: "CMP1" },
                        { name: 2, displayName: "CMP2" },
                        { name: 3, displayName: "CMP3" },
                        { name: 4, displayName: "CMP4" },
                        { name: 5, displayName: "CMP5" },
                        { name: 6, displayName: "CMP6" },
                        { name: 7, displayName: "CMP7" },
                        { name: 8, displayName: "CMP8" },
                        { name: 9, displayName: "CMP9" },
                        { name: 10, displayName: "CMP10" },
                        { name: 11, displayName: "CMP11" },
                        { name: 12, displayName: "CMP12" },
                        { name: 13, displayName: "CMP13" },
                        { name: 14, displayName: "CMP14" },
                        { name: 15, displayName: "CMP15" },
                    ],
                },
                {
                    name: "CAP_Event_Num",
                    displayName: "IEP CAP Event Number",
                    description: "CAP event number (0-7) for periodic CAP trigger mode. Note: Routing (SYNC out to CAP) is done in application code (endat3_config_iep_cap_for_sync). Correct router signal should be configured for selected CAP event if needed.",
                    default: 6,
                    options: [
                        { name: 0, displayName: "CAP0" },
                        { name: 1, displayName: "CAP1" },
                        { name: 2, displayName: "CAP2" },
                        { name: 3, displayName: "CAP3" },
                        { name: 4, displayName: "CAP4" },
                        { name: 5, displayName: "CAP5" },
                        { name: 6, displayName: "CAP6 (LATCH_IN0)" },
                        { name: 7, displayName: "CAP7 (LATCH_IN1)" },
                    ],
                },
            ],
            collapsed: false,
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
    let total_channels = (instance["channel_0"] ? 1 : 0) + (instance["channel_1"] ? 1 : 0) + (instance["channel_2"] ? 1 : 0);

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

            if((device == "am243x-lp") && (total_channels > 1))
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

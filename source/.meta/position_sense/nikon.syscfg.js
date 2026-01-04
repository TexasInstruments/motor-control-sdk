
let common = system.getScript("/common");
let device = common.getDeviceName();
let nikon_module_name = "/position_sense/nikon";
let is_am243x_lp_device = (device === "am243x-lp") ? true : false;
let is_am26x_soc = (device === "am261x-lp" || device === "am263x-cc" || device === "am263px-cc" ) ? true : false;
let is_am263x_soc = (device === "am263x-cc") ? true : false;
let is_am261x_soc = (device === "am261x-lp") ? true : false;
let is_am263px_soc = (device === "am263px-cc") ? true : false;
let nikon_pins = (is_am26x_soc) ? system.getScript("/position_sense/nikon/am26x_pins.js") : system.getScript("/position_sense/nikon_pins.js");

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

    if (!instance.Multi_Channel_Load_Share) {
        /* Non-load share mode - check global CMP event */
        checkCmpEvent(instance.CMP_Event_Num, "CMP_Event_Num");
    } else {
        /* Load share mode - check per-channel CMP events */
        if (instance.channel_0) {
            checkCmpEvent(instance.CMP_Event_Num_CH0, "CMP_Event_Num_CH0");
        }
        if (instance.channel_1) {
            checkCmpEvent(instance.CMP_Event_Num_CH1, "CMP_Event_Num_CH1");
        }
        if (instance.channel_2) {
            checkCmpEvent(instance.CMP_Event_Num_CH2, "CMP_Event_Num_CH2");
        }
    }
}

function onValidate(inst, validation)
{
    for (let instance_index in inst.$module.$instances)
    {
       let instance = inst.$module.$instances[instance_index];

        /* Validate CMP0 and CMP1 usage warnings */
        validateCmpEventWarnings(instance, inst, validation);

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

        /* Validate CMP and CAP event numbers don't conflict across channels in load share mode */
        if (instance.Multi_Channel_Load_Share) {
            let enabled_channels = [];

            /* Collect enabled channels and their event numbers */
            if (instance.channel_0) {
                enabled_channels.push({
                    name: "Channel 0",
                    cmp_event: instance.CMP_Event_Num_CH0,
                    cap_event: instance.CAP_Event_Num_CH0,
                    cmp_field: "CMP_Event_Num_CH0",
                    cap_field: "CAP_Event_Num_CH0"
                });
            }

            if (instance.channel_1) {
                enabled_channels.push({
                    name: "Channel 1",
                    cmp_event: instance.CMP_Event_Num_CH1,
                    cap_event: instance.CAP_Event_Num_CH1,
                    cmp_field: "CMP_Event_Num_CH1",
                    cap_field: "CAP_Event_Num_CH1"
                });
            }

            if (instance.channel_2) {
                enabled_channels.push({
                    name: "Channel 2",
                    cmp_event: instance.CMP_Event_Num_CH2,
                    cap_event: instance.CAP_Event_Num_CH2,
                    cmp_field: "CMP_Event_Num_CH2",
                    cap_field: "CAP_Event_Num_CH2"
                });
            }

            /* Check for CMP event conflicts between channels */
            for (let i = 0; i < enabled_channels.length; i++) {
                for (let j = i + 1; j < enabled_channels.length; j++) {
                    if (enabled_channels[i].cmp_event === enabled_channels[j].cmp_event) {
                        validation.logError(
                            enabled_channels[i].name + " and " + enabled_channels[j].name +
                            " cannot use the same IEP CMP event number (CMP" + enabled_channels[i].cmp_event + ")",
                            inst,
                            enabled_channels[j].cmp_field
                        );
                    }
                }
            }

            /* Check for CAP event conflicts between channels */
            for (let i = 0; i < enabled_channels.length; i++) {
                for (let j = i + 1; j < enabled_channels.length; j++) {
                    if (enabled_channels[i].cap_event === enabled_channels[j].cap_event) {
                        validation.logError(
                            enabled_channels[i].name + " and " + enabled_channels[j].name +
                            " cannot use the same IEP CAP event number (CAP" + enabled_channels[i].cap_event + ")",
                            inst,
                            enabled_channels[j].cap_field
                        );
                    }
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
            onChange: function(inst, ui) {
                if (inst.Multi_Channel_Load_Share) {
                    ui.CMP_Event_Num_CH0.hidden = !inst.channel_0;
                    ui.CAP_Event_Num_CH0.hidden = !inst.channel_0;
                }
            },
        },
        {
            name: "channel_1",
            displayName: "Select Channel 1",
            description: "Channel 1 Selection ",
            default: false,
            onChange: function(inst, ui) {
                if (inst.Multi_Channel_Load_Share) {
                    ui.CMP_Event_Num_CH1.hidden = !inst.channel_1;
                    ui.CAP_Event_Num_CH1.hidden = !inst.channel_1;
                }
            },
        },
        {
            name: "channel_2",
            displayName: "Select Channel 2",
            description: "Channel 2 Selection ",
            default: false,
            onChange: function(inst, ui) {
                if (inst.Multi_Channel_Load_Share) {
                    ui.CMP_Event_Num_CH2.hidden = !inst.channel_2;
                    ui.CAP_Event_Num_CH2.hidden = !inst.channel_2;
                }
            },
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
            onChange: function(inst, ui) {
                /* When load share is toggled, update visibility of event selectors */
                if (inst.Multi_Channel_Load_Share) {
                    /* Hide global CMP and CAP event selectors */
                    ui.CMP_Event_Num.hidden = true;
                    ui.CAP_Event_Num.hidden = true;
                    /* Show per-channel CMP and CAP selectors based on enabled channels */
                    ui.CMP_Event_Num_CH0.hidden = !inst.channel_0;
                    ui.CAP_Event_Num_CH0.hidden = !inst.channel_0;
                    ui.CMP_Event_Num_CH1.hidden = !inst.channel_1;
                    ui.CAP_Event_Num_CH1.hidden = !inst.channel_1;
                    ui.CMP_Event_Num_CH2.hidden = !inst.channel_2;
                    ui.CAP_Event_Num_CH2.hidden = !inst.channel_2;
                } else {
                    /* Show global CMP and CAP event selectors */
                    ui.CMP_Event_Num.hidden = false;
                    ui.CAP_Event_Num.hidden = false;
                    /* Hide all per-channel selectors */
                    ui.CMP_Event_Num_CH0.hidden = true;
                    ui.CAP_Event_Num_CH0.hidden = true;
                    ui.CMP_Event_Num_CH1.hidden = true;
                    ui.CAP_Event_Num_CH1.hidden = true;
                    ui.CMP_Event_Num_CH2.hidden = true;
                    ui.CAP_Event_Num_CH2.hidden = true;
                }
            },
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
                    hidden: false,
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
                    description: "CAP event number (0-7) for periodic CAP trigger mode. Note: Routing (SYNC out to CAP) is done in application code (nikon_config_iep_cap_for_sync). Correct router signal should be configured for selected CAP event if needed",
                    default: 6,
                    hidden: false,
                    options: [
                        { name: 0, displayName: "CAP0" },
                        { name: 1, displayName: "CAP1" },
                        { name: 2, displayName: "CAP2" },
                        { name: 3, displayName: "CAP3" },
                        { name: 4, displayName: "CAP4" },
                        { name: 5, displayName: "CAP5" },
                        { name: 6, displayName: "CAP6" },
                        { name: 7, displayName: "CAP7" },
                    ],
                },
                /* Channel event numbers (only shown when load share is enabled) */
                /* CMP Event Numbers for each channel */
                {
                    name: "CMP_Event_Num_CH0",
                    displayName: "Channel 0 - CMP Event Number",
                    description: "CMP event number (0-15) for Channel 0 periodic CMP trigger mode",
                    default: 2,
                    hidden: true,
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
                    name: "CAP_Event_Num_CH0",
                    displayName: "Channel 0 - CAP Event Number",
                    description: "CAP event number (0-7) for Channel 0 periodic CAP trigger mode. Note: Routing (SYNC out to CAP) is done in application code (nikon_config_iep_cap_for_sync). Correct router signal should be configured for selected CAP event if needed",
                    default: 6,
                    hidden: true,
                    options: [
                        { name: 0, displayName: "CAP0" },
                        { name: 1, displayName: "CAP1" },
                        { name: 2, displayName: "CAP2" },
                        { name: 3, displayName: "CAP3" },
                        { name: 4, displayName: "CAP4" },
                        { name: 5, displayName: "CAP5" },
                        { name: 6, displayName: "CAP6" },
                        { name: 7, displayName: "CAP7" },
                    ],
                },
                {
                    name: "CMP_Event_Num_CH1",
                    displayName: "Channel 1 - CMP Event Number",
                    description: "CMP event number (0-15) for Channel 1 periodic CMP trigger mode",
                    default: 3,
                    hidden: true,
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
                    name: "CAP_Event_Num_CH1",
                    displayName: "Channel 1 - CAP Event Number",
                    description: "CAP event number (0-7) for Channel 1 periodic CAP trigger mode. Note: Routing (SYNC out to CAP) is done in application code (nikon_config_iep_cap_for_sync). Correct router signal should be configured for selected CAP event if needed",
                    default: 0,
                    hidden: true,
                    options: [
                        { name: 0, displayName: "CAP0" },
                        { name: 1, displayName: "CAP1" },
                        { name: 2, displayName: "CAP2" },
                        { name: 3, displayName: "CAP3" },
                        { name: 4, displayName: "CAP4" },
                        { name: 5, displayName: "CAP5" },
                        { name: 6, displayName: "CAP6" },
                        { name: 7, displayName: "CAP7" },
                    ],
                },
                {
                    name: "CMP_Event_Num_CH2",
                    displayName: "Channel 2 - CMP Event Number",
                    description: "CMP event number (0-15) for Channel 2 periodic CMP trigger mode",
                    default: 4,
                    hidden: true,
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
                    name: "CAP_Event_Num_CH2",
                    displayName: "Channel 2 - CAP Event Number",
                    description: "CAP event number (0-7) for Channel 2 periodic CAP trigger mode. Note: Routing (SYNC out to CAP) is done in application code (nikon_config_iep_cap_for_sync). Correct router signal should be configured for selected CAP event if needed",
                    default: 7,
                    hidden: true,
                    options: [
                        { name: 0, displayName: "CAP0" },
                        { name: 1, displayName: "CAP1" },
                        { name: 2, displayName: "CAP2" },
                        { name: 3, displayName: "CAP3" },
                        { name: 4, displayName: "CAP4" },
                        { name: 5, displayName: "CAP5" },
                        { name: 6, displayName: "CAP6" },
                        { name: 7, displayName: "CAP7" },
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

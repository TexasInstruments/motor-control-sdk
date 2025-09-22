let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/pinmux/pinmux_${common.getSocName()}`);

function getInterfaceName(inst, peripheralName)
{
    return `PRU_${inst.instance}_${peripheralName}`;
}

function getPwmPinRequirements(interfaceName, pinName)
{    
    let pinConfig = soc.getPinConfigurables(interfaceName, pinName); 

    let interfacePins = Object.keys(system.deviceData.interfaces[interfaceName].interfacePins);

    // remove 'TZ_IN', 'TZ_OUT'

    interfacePins.pop();

    interfacePins.pop();
    
    let pin = {
        name           : "pwmPin",
        displayName    : "ICSS_PWM_CHANNEL pin",
        interfaceNames : interfacePins,
        config         : pinConfig,
    };

    return pin;
}

function getPeripheralRequirements(inst, peripheralName)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let resources = [];

    let pinResource = getPwmPinRequirements(interfaceName, "A0");

    pinmux.setConfigurableDefault(pinResource, 'rx', false);

    pinResource.used = true;

    resources.push(pinResource);

    let peripheralRequirements = {
        name: interfaceName,
        displayName: interfaceName,
        interfaceName: interfaceName,
        resources: resources,
        canShareWith: "/pruicss_g_v0_pwm/pruicss_g_v0_pwm"
    };

    return peripheralRequirements;
}

function pinmuxRequirements(inst) {

    let pwm = getPeripheralRequirements(inst, "PWM");
    return [pwm];
}

function getInterfaceNameList(inst) {

    return [
        getInterfaceName(inst, "PWM"),
    ];
}

function getPeripheralPinNames(inst)
{
    return ["pwmPin"];
}

let pruicss_top_module_name = "/pruicss_g_v0_pwm/pruicss_g_v0_pwm_channel";

let pruicss_top_module = {
    displayName: "PRU ICSS PWM CHANNEL",

    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: pruicss_top_module_name,
        },
    },

    defaultInstanceName: "CONFIG_PRUICSS_PWM_CHANNEL",
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
                name: "duty_cycle",
                displayName : "Duty Cycle Of Pwm",
                longDescription: "This determines duty cycle of selected pwm channal",
                default: 25,
            },

            {
                name: "rise_edge_delay",
                displayName: "Rise Edge Delay Of Pwm(nsecs)",
                longDescription: "This determines rise edge delay of selected pwm channel, value can be either positive or negative",
                default: 0
            },

            {
                name: "fall_edge_delay",
                displayName: "Fall Edge Delay Of Pwm(nsecs)",
                longDescription: "This determines fall edge delay of selected pwm channel, value can be either positive or negative",
                default: 0,
            },

    ],
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
    validate: onValidate,
};

function onValidate(inst, report) {
    let icssg0Channels = 0;
    let icssg1Channels = 0;
    for (let instance_index in inst.$module.$instances)
    {
        let moduleInstance = inst.$module.$instances[instance_index];
        if(moduleInstance.instance == "ICSSG0")
        {
            if(icssg0Channels < 24)
            {
                icssg0Channels = icssg0Channels + 1;    
            }
            else
            {
                report.logError("Maximum number of PWM channels that can be added are 24 (4 Sets, Each set has 6 PWM channels)",inst);
            }
        }
        else if(moduleInstance.instance == "ICSSG1")
        {
            if(icssg1Channels < 24)
            {
                icssg1Channels = icssg1Channels + 1;    
            }
            else
            {
                report.logError("Maximum number of PWM channels that can be added are 24 (4 Sets, Each set has 6 PWM channels)",inst);
            }
        }
    }
}

exports = pruicss_top_module;
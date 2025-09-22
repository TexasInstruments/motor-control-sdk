let common = system.getScript("/common");
let pinmux = system.getScript("/drivers/pinmux/pinmux");
let soc = system.getScript(`/drivers/pinmux/pinmux_${common.getSocName()}`);

function getInterfaceName(inst, peripheralName)
{
    return `PRU_${inst.instance}_${peripheralName}`;
}

function getPeripheralRequirements(inst, peripheralName)
{
    let interfaceName = getInterfaceName(inst, peripheralName);
    let resources = [];

    // add TZ_IN as pin resource
    let pinResource = pinmux.getPinRequirements(interfaceName, "TZ_IN");

    pinmux.setConfigurableDefault(pinResource, 'rx', false);

    pinResource.used = false;

    resources.push(pinResource);

    // add TZ_OUT as pin resource
    pinResource = pinmux.getPinRequirements(interfaceName, "TZ_OUT");

    pinmux.setConfigurableDefault(pinResource, 'rx', false);

    pinResource.used = false;

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
    return ["TZ_IN", "TZ_OUT"];
}

let pruicss_top_module_name =  "/pruicss_g_v0_pwm/pruicss_g_v0_pwm_tripzone";

let pruicss_top_module = {
    displayName: "PRU ICSS PWM TRIPZONE",

    templates: {
        "/drivers/pinmux/pinmux_config.c.xdt": {
            moduleName: pruicss_top_module_name,
        },
    },

    defaultInstanceName: "CONFIG_PRUICSS_PWM_TRIPZONE",
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
            // TODO : add icss pwm tripzone module configurations
    ],
    pinmuxRequirements,
    getInterfaceNameList,
    getPeripheralPinNames,
    validate: onValidate,
};

function onValidate(inst, report) {
    let icssg0 = 0;
    let icssg1 = 0;
    for (let instance_index in inst.$module.$instances)
    {
        let moduleInstance = inst.$module.$instances[instance_index];
        if(moduleInstance.instance == "ICSSG0")
        {
            if(icssg0 < 4)
            {
                icssg0 = icssg0 + 1;    
            }
            else
            {
                report.logError("Maximum number of PWM Trip zone blocks that can be added are 4 (4 Sets, Each set has 1 Trip zone block)",inst);
            }
        }
        else if(moduleInstance.instance == "ICSSG1")
        {
            if(icssg1 < 4)
            {
                icssg1 = icssg1 + 1;    
            }
            else
            {
                report.logError("Maximum number of PWM Trip zone blocks that can be added are 4 (4 Sets, Each set has 1 Trip zone block)",inst);
            }
        }
    }
}

exports = pruicss_top_module;
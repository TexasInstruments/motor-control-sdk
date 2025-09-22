let common = system.getScript("/common");
let pruicss_top_module_name = "/pruicss_g_v0_pwm/pruicss_g_v0_pwm";

let pruicss_top_module = {
    templates :{
        "/drivers/system/system_config.c.xdt": {
            driver_config: "/pruicss_g_v0_pwm/templates/pruicss_g_v0_pwm_config.c.xdt",
            moduleName : pruicss_top_module_name
        },
        "/drivers/system/system_config.h.xdt": {
            driver_config: "/pruicss_g_v0_pwm/templates/pruicss_g_v0_pwm.h.xdt",
            moduleName : pruicss_top_module_name
        }
    },
    displayName: "PRU ICSS PWM",
    defaultInstanceName: "CONFIG_PRUICSS_PWM",
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
                name: "pwmFrequency",
                displayName: "Pwm Frequency(Hz)",
                longDescription: "Pwm channels of all sets will be generated with same frequency",
                default: 16000
           }
    ],
    moduleInstances: (inst) => {
        return[
            {
                name: "PRUICSS_PWM_IEP",
                displayName: "PRU ICSS PWM IEP",
                moduleName:  "/pruicss_g_v0_pwm/pruicss_g_v0_pwm_iep",
                collapsed: true,
                requiredArgs: {
                    instance: inst["instance"],
                },
            },
            {
                name: "PRUICSS_PWM_CHANNEL",
                displayName: "PRU ICSS PWM CHANNEL",
                moduleName:  "/pruicss_g_v0_pwm/pruicss_g_v0_pwm_channel",
                useArray: true,
                defaultInstanceCount: 1,
                // 4 sets, each with 6 instances
                // maxInstanceCount: 24,
                collapsed: true,
                requiredArgs: {
                    instance: inst["instance"],
                },
            },
            {
                name: "PRUICSS_PWM_TRIPZONE",
                displayName: "PRU ICSS PWM TRIPZONE",
                moduleName:  "/pruicss_g_v0_pwm/pruicss_g_v0_pwm_tripzone",
                useArray: true,
                defaultInstanceCount: 1,
                // 4 sets, each with 1 tripzone block
                // maxInstanceCount: 4,
                collapsed: true,
                requiredArgs: {
                    instance: inst["instance"],
                },
            }, 
        ]
    },

    sharedModuleInstances: (inst) => {
        return[
            {
                name: "PRUICSS",
                displayName: "PRU ICSS",
                moduleName: '/drivers/pruicss/pruicss',
                requiredArgs: {
                    instance: inst["instance"],
                },
            }
        ]
    },
    validate: onValidate,
};

function onValidate(inst, report) {
    common.validate.checkSameInstanceName(inst, report);
}



exports = pruicss_top_module
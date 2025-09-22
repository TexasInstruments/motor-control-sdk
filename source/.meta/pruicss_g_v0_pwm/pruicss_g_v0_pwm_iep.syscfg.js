let common = system.getScript("/common");
let pruicss_top_module_name =  "/pruicss_g_v0_pwm/pruicss_g_v0_pwm_iep";

let pruicss_top_module = {

    displayName: "PRU ICSS PWM IEP",

    defaultInstanceName: "CONFIG_PRUICSS_PWM_IEP",
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
                name: "enableIep0",
                displayName: "Enable Iep0",
                default: false      
            },
            {
                name: "iep0IncrementValue",
                displayName: "Iep0 Increment Value",
                default: 5
            },
            {
                name: "enableIEP0ShadowMode",
                displayName: "Enable Iep0 Shadow Mode",
                longDescription: "When Iep0 shadow mode & Iep1 slave mode are enabled" +
                                    "Refer Section 6.4.13 of Technical Reference manual to understand iep shadow mode",
                default: false
            },
            {
                name: "enableIep0ResetOnEpwm0_Sync",
                displayName: "Enable Iep0 Reset On Epwm0 Syncout",
                longDescription: "When enabled epwm0 syncout signal resets IEP0 counter",
                default: false
            },
            {
                name: "enableIep0ResetOnEpwm3_Sync",
                displayName: "Enable Iep0 Reset On Epwm3 Sync",
                longDescription: "When enabled epwm3 syncout signal resets IEP0 counter",
                default: false
            },
            {
                name: "enableIep0ResetOnCompare0",
                displayName: "Enable Iep0 Reset On Compare0",
                longDescription: "When enabled iep0 compare0 resets IEP0 counter",
                default: false
            },

            {
                name: "enableIep1",
                displayName: "Enable Iep1",
                default: false,
                longDescription: "When IEP0 slave mode is enabled, it is not required to enable IEP1",
                
            },
            {
                name: "iep1IncrementValue",
                displayName: "Iep1 Increment Value",
                default: 5,
                longDescription: "When IEP0 slave mode is enabled, it is not required to configure IEP1 increment value",
            },
            {
                name: "enableIEP1ShadowMode",
                displayName: "Enable Iep1 Shadow Mode",
                longDescription: "IEP1 shadow mode is configurable irrespective of IEP1 slave mode configuration",
                default: false
            },
            {
                name: "enableIep1ResetOnEpwm0_Sync",
                displayName: "Enable Iep1 Reset On Epwm0 Syncout",
                longDescription: "When enabled epwm0 syncout signal resets IEP1 counter", 
                default: false
            },
            {
                name: "enableIep1ResetOnEpwm3_Sync",
                displayName: "Enable Iep1 Reset On Epwm3 Sync",
                longDescription: "When enabled epwm3 syncout signal resets IEP1 counter",
                default: false
            },
            {
                name: "enableIep1ResetOnCompare0",
                displayName: "Enable Iep1 Reset On Compare0",
                longDescription: "When enabled iep1 compare0 resets IEP1 counter ",
                default: false
            },

            {
                name: "enableIep1SlaveMode",
                displayName: "Enable Iep1 Slave Mode",
                longDescription: "When IEP1 slave mode is enabled, Iep1 follows Iep0 counter" +
                                  " It can be simplified as Iep0 and Iep1 shows same value in every iep clock cycle"+
                                  "& Iep1 compare events, Iep0 compare events are controlled by Iep0 counter" + 
                                  " Note : enableIep1, iep1IncrementValue  becomes invalid ",
                default: false,
                onChange: (inst, ui) => {
                    if(inst.enableIep1SlaveMode)
                    {
                        ui.enableIep1.hidden = true,
                        ui.iep1IncrementValue.hidden = true
                    }
                    else
                    {
                        ui.enableIep1.hidden = false,
                        ui.iep1IncrementValue.hidden = false
                    }
                },
            },

            {
                name: "enableAutoClearCompareStatus",
                displayName: "Enable Auto Clear Compare Status",
                longDescription: "When enabled compare status is autocleared in next iep clock cycle",
                default: false
            }   
    ],
    validate: onValidate,
};

function onValidate(inst, report) {
    common.validate.checkSameInstanceName(inst, report);
}

exports = pruicss_top_module;
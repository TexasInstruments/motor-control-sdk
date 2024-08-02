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
                default: false,
                longDescription: "When IEP0 is enabled, it is not required enable IEP1 counter, as IEP1 slave mode will be enabled by sysconfig",
                onChange: (inst, ui) => {
                    if(inst.enableIep0)
                    {
                        inst["enableIep1SlaveMode"] = true;
                    }
                    else
                    {
                        inst["enableIep1SlaveMode"] = false;
                    }
                },
            },
            {
                name: "iep0IncrementValue",
                displayName: "Iep0 Increment Value",
                default: 5
            },
            {
                name: "enableIEP0ShadowMode",
                displayName: "Enable Iep0 Shadow Mode",
                longDescription: "When Iep0 shadow mode & Iep1 slave mode are enabled, Iep1 shadow mode is enabled by driver" +
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
                name: "enableIep1SlaveMode",
                displayName: "Enable Iep1 Slave Mode",
                longDescription: "When IEP1 slave mode is enabled, Iep1 follows Iep0 counter" +
                                  "It can be simplified as Iep0 and Iep1 shows same value in every iep clock cycle"+
                                  "& Iep1 compare events, Iep0 compare events are controlled by Iep0 counter",
                default: false,
                readOnly: true
            },
            {
                name: "enableAutoClearCompareStatus",
                displayName: "Enable Auto Clear Compare Status",
                longDescription: "When enabled compare status is autocleared in next iep clock cycle",
                default: false
            }
           
    ],
};

function validate(inst, report) {
    common.validate.checkSameInstanceName(inst, report);
}

exports = pruicss_top_module;
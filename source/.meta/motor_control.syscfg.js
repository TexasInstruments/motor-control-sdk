
let common = system.getScript("/common");
let soc = system.getScript(`/soc/motor_control_${common.getSocName()}`);

exports = {
    displayName: "TI Motor Control Protocols",
    topModules: soc.getTopModules(),
};

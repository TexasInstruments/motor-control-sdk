let common = system.getScript("/common");
let device = common.getDeviceName();

function getModule() {

    let SD_Module = system.getScript("/current_sense/sdfm_syscfg.js");

    return SD_Module;
}

exports = getModule();


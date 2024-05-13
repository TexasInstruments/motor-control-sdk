
let common = system.getScript("/common");

const topModules_main = [
    "/current_sense/sdfm",
    "/position_sense/bissc",
    "/position_sense/endat",
    "/position_sense/hdsl",
    "/position_sense/nikon",
    "/position_sense/tamagawa",
    "/pruicss_g_v0_pwm/pruicss_g_v0_pwm",
];

const topModules_mcu = [

];

exports = {
    getTopModules: function() {

        let topModules = topModules_main;

        if(common.getSelfSysCfgCoreName().includes("m4f")) {
            topModules = topModules_mcu;
        }

        return topModules;
    },
};

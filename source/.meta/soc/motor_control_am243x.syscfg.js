
let common = system.getScript("/common");

const topModules_main = [
    "/position_sense/endat",
    "/position_sense/hdsl",
    "/position_sense/tamagawa",
    "/position_sense/bissc",
    "/current_sense/sdfm",

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

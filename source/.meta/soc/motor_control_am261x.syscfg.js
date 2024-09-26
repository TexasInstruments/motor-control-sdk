let common = system.getScript("/common");

const topModules_main = [
    "/position_sense/tamagawa",
    "/position_sense/endat",
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
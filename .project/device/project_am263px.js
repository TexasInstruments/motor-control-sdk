const common = require("../common.js");

const component_file_list = [
    "source/position_sense/bissc/.project/project.js",
    "source/position_sense/endat/.project/project.js",
    "source/position_sense/nikon/.project/project.js",
    "source/position_sense/tamagawa/.project/project.js",
    "source/position_sense/tamagawa_over_soc_uart/.project/project.js",
];

const device_defines = {
    common: [
        "SOC_AM263PX",
    ],
};

const example_file_list = [
    "examples/dcl/dcl_df22/.project/mcsdk_project.js",
    "examples/dcl/dcl_pi/.project/mcsdk_project.js",   
    "examples/position_sense/bissc_diagnostic/single_channel/.project/project.js",
    "examples/position_sense/endat_diagnostic/single_channel/.project/project.js",
    "examples/position_sense/nikon_diagnostic/single_channel/.project/project.js",
    "examples/position_sense/tamagawa_diagnostic/single_channel/.project/project.js",
    "examples/position_sense/tamagawa_diagnostic_over_soc_uart/.project/project.js",
    "examples/sfra/.project/mcsdk_project.js",
    "examples/transforms/transforms_test/.project/mcsdk_project.js",
    "examples/utilities/datalog/.project/mcsdk_project.js",
    "source/position_sense/bissc/firmware/single_channel/.project/project.js",
    "source/position_sense/endat/firmware/single_channel/.project/project.js",
    "source/position_sense/nikon/firmware/single_channel/.project/project.js",
    "source/position_sense/tamagawa/firmware/single_channel/.project/project.js",
];

function getProjectSpecCpu(cpu) {
    let projectSpecCpu =
    {
        "r5fss0-0": "Cortex_R5_0",
        "r5fss0-1": "Cortex_R5_1",
        "r5fss1-0": "Cortex_R5_2",
        "r5fss1-1": "Cortex_R5_3",
        "icss_m0_pru0": "ICSSM_PRU_0",
        "icss_m0_pru1": "ICSSM_PRU_1",
    }

    return projectSpecCpu[cpu];
}

function getComponentList() {
    return component_file_list;
}

function getExampleList() {
    return example_file_list;
}

function getSysCfgDevice(board) {
    switch (board) {
        case "am263px-lp":
            return "AM263Px";
        default:
        case "am263px-cc":
            return "AM263Px";
        case "am263px-cc-addon-ind":
            return "AM263Px";
        case "am263px-cc-addon-auto":
            return "AM263Px";
    }
}

function getProjectSpecDevice(board) {
    switch (board) {
        case "am263px":
            return "AM263Px";
        case "am263px-lp":
            return "AM263Px";
        default:
        case "am263px-cc":
            return "AM263Px";
        case "am263px-cc-addon-ind":
            return "AM263Px";
        case "am263px-cc-addon-auto":
            return "AM263Px";
    }
}

function getSysCfgCpu(cpu) {
    return cpu;
}

function getSysCfgPkg(board) {
    switch (board) {
        case "am263px-lp":
            return "ZCZ_C";
        default:
        case "am263px-cc":
            return "ZCZ_S";
        case "am263px-cc-addon-ind":
            return "ZCZ_S";
        case "am263px-cc-addon-auto":
            return "ZCZ_S";
    }
}

function getSysCfgPart(board) {
    switch (board) {
        case "am263px-lp":
            return "AM263P4";
        default:
        case "am263px-cc":
            return "AM263P4";
        case "am263px-cc-addon-ind":
            return "AM263P4";
        case "am263px-cc-addon-auto":
            return "AM263P4";
    }
}

function getDevToolTirex(board) {
    switch (board) {
        case "am263px-lp":
            return "LP-AM263P";
        default:
        case "am263px-cc":
            return "TMDSCNCD263P";
        case "am263px-cc-addon-ind":
            return "TMDSCNCD263P";
        case "am263px-cc-addon-auto":
            return "TMDSCNCD263P";
    }
}

function getProperty() {
    let property = {};

    property.defines = device_defines;

    return property;
}

function getLinuxFwName(cpu) {

    switch(cpu) {
        case "r5fss0-0":
            return "am263p-main-r5f0_0-fw";
        case "r5fss0-1":
            return "am263p-main-r5f0_1-fw";
        case "r5fss1-0":
            return "am263p-main-r5f1_0-fw";
        case "r5fss1-1":
            return "am263p-main-r5f1_1-fw";
    }
    return undefined;
}

function getProductNameProjectSpec() {
    return "MOTOR_CONTROL_SDK_AM263PX";
}

function getFlashAddr() {
    return 0x60000000;
}

module.exports = {
    getComponentList,
    getExampleList,
    getSysCfgDevice,
    getSysCfgCpu,
    getSysCfgPkg,
    getSysCfgPart,
    getProjectSpecDevice,
    getProjectSpecCpu,
    getDevToolTirex,
    getProperty,
    getLinuxFwName,
    getProductNameProjectSpec,
    getFlashAddr,
};
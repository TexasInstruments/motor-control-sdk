const common = require("../common.js");

const component_file_list = [
    "source/motor_control/current_sense/sdfm/.project/project.js",
    "source/motor_control/position_sense/endat/.project/project.js",
    "source/motor_control/position_sense/hdsl/.project/project.js",
    "source/motor_control/position_sense/tamagawa/.project/project.js",
];

const device_defines = {
    common: [
        "SOC_AM243X",
    ],
};

const example_file_list = [
    "examples/motor_control/endat_diagnostic/single_channel/.project/project.js",
    "examples/motor_control/endat_diagnostic/multi_channel_load_share/.project/project.js",
    "examples/motor_control/endat_diagnostic/multi_channel_single_pru/.project/project.js",
    "examples/motor_control/hdsl_diagnostic/multi_channel/.project/project.js",
    "examples/motor_control/hdsl_diagnostic/single_channel/.project/project.js",
    "examples/motor_control/hdsl_diagnostic_with_traces/.project/project.js",
    "examples/motor_control/tamagawa_diagnostic/multi_channel/.project/project.js",
    "examples/motor_control/tamagawa_diagnostic/single_channel/.project/project.js",
    "examples/motor_control/icss_sdfm/.project/project.js",
    "source/motor_control/current_sense/sdfm/firmware/.project/project.js",
    "source/motor_control/position_sense/endat/firmware/multi_channel_load_share/.project/project.js",
    "source/motor_control/position_sense/endat/firmware/single_channel/.project/project.js",
    "source/motor_control/position_sense/endat/firmware/multi_channel_single_pru/.project/project.js",
    "source/motor_control/position_sense/hdsl/firmware/freerun_225_mhz/.project/project.js",
    "source/motor_control/position_sense/hdsl/firmware/freerun_300_mhz/.project/project.js",
    "source/motor_control/position_sense/hdsl/firmware/multichannel_ch0/.project/project.js",
    "source/motor_control/position_sense/hdsl/firmware/multichannel_ch1/.project/project.js",
    "source/motor_control/position_sense/hdsl/firmware/sync_225_mhz/.project/project.js",
    "source/motor_control/position_sense/tamagawa/firmware/multi_channel/.project/project.js",
    "source/motor_control/position_sense/tamagawa/firmware/single_channel/.project/project.js",
];

function getProjectSpecCpu(cpu) {
    let projectSpecCpu =
    {
        "r5fss0-0": "MAIN_PULSAR_Cortex_R5_0_0",
        "r5fss0-1": "MAIN_PULSAR_Cortex_R5_0_1",
        "r5fss1-0": "MAIN_PULSAR_Cortex_R5_1_0",
        "r5fss1-1": "MAIN_PULSAR_Cortex_R5_1_1",
        "m4fss0-0": "Cortex_M4F_0",
        "icssg0-pru0": "ICSS_G0_PRU_0",
        "icssg0-pru1": "ICSS_G0_PRU_1",
        "icssg0-rtupru0": "ICSS_G0_RTU_PRU_0",
        "icssg0-rtupru1": "ICSS_G0_RTU_PRU_1",
        "icssg0-txpru0": "ICSS_G0_TX_PRU_0",
        "icssg0-txpru1": "ICSS_G0_TX_PRU_1",
        "icssg1-pru0": "ICSS_G1_PRU_0",
        "icssg1-pru1": "ICSS_G1_PRU_1",
        "icssg1-rtupru0": "ICSS_G1_RTU_PRU_0",
        "icssg1-rtupru1": "ICSS_G1_RTU_PRU_1",
        "icssg1-txpru0": "ICSS_G1_TX_PRU_0",
        "icssg1-txpru1": "ICSS_G1_TX_PRU_1",
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
        case "am243x-lp":
            return "AM243x_ALX_beta";
        default:
        case "am243x-evm":
            return "AM243x_ALV_beta";
    }
}

function getProjectSpecDevice(board) {
    switch (board) {
        case "am243x":
            return "AM243x";
        case "am243x-lp":
            return "AM2434_ALX";
        default:
        case "am243x-evm":
            return "AM2434_ALV";
    }
}

function getSysCfgCpu(cpu) {
    return cpu;
}

function getSysCfgPkg(board) {
    switch (board) {
        case "am243x-lp":
            return "ALX";
        default:
        case "am243x-evm":
            return "ALV";
    }
}

function getSysCfgPart(board) {
    switch (board) {
        case "am243x-lp":
            return "ALX";
        default:
        case "am243x-evm":
            return "ALV";
    }
}

function getDevToolTirex(board) {
    switch (board) {
        case "am243x-lp":
            return "AM243x_LAUNCHPAD";
        default:
        case "am243x-evm":
            return "AM243x_GP_EVM";
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
            return "am243-main-r5f0_0-fw";
        case "r5fss0-1":
            return "am243-main-r5f0_1-fw";
        case "r5fss1-0":
            return "am243-main-r5f1_0-fw";
        case "r5fss1-1":
            return "am243-main-r5f1_1-fw";
        case "m4fss0-0":
            return "am243-mcu-m4f0_0-fw";
    }
    return undefined;
}

function getProductNameProjectSpec() {
    return "MOTOR_CONTROL_SDK_AM243X";
}

function getDependentProductNameProjectSpec() {
    return "MCU_PLUS_SDK_AM243X";
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
    getDependentProductNameProjectSpec,
    getFlashAddr,
};
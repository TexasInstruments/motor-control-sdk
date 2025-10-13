const common = require("../common.js");

const component_file_list = [
];

const device_defines = {
    common: [
        "SOC_AM64X",
    ],
};

const example_file_list = [
];

function getProjectSpecCpu(cpu) {
    let projectSpecCpu =
    {
        "r5fss0-0": "MAIN_PULSAR_Cortex_R5_0_0",
        "r5fss0-1": "MAIN_PULSAR_Cortex_R5_0_1",
        "r5fss1-0": "MAIN_PULSAR_Cortex_R5_1_0",
        "r5fss1-1": "MAIN_PULSAR_Cortex_R5_1_1",
        "m4fss0-0": "Cortex_M4F_0",
        "a53ss0-0": "CortexA53_0",
        "a53ss0-1": "CortexA53_1",
        "icss_g0_pru0": "ICSS_G0_PRU_0",
        "icss_g0_pru1": "ICSS_G0_PRU_1",
        "icss_g0_rtu_pru0": "ICSS_G0_RTU_PRU_0",
        "icss_g0_rtu_pru1": "ICSS_G0_RTU_PRU_1",
        "icss_g0_tx_pru0": "ICSS_G0_TX_PRU_0",
        "icss_g0_tx_pru1": "ICSS_G0_TX_PRU_1",
        "icss_g1_pru0": "ICSS_G1_PRU_0",
        "icss_g1_pru1": "ICSS_G1_PRU_1",
        "icss_g1_rtu_pru0": "ICSS_G1_RTU_PRU_0",
        "icss_g1_rtu_pru1": "ICSS_G1_RTU_PRU_1",
        "icss_g1_tx_pru0": "ICSS_G1_TX_PRU_0",
        "icss_g1_tx_pru1": "ICSS_G1_TX_PRU_1",
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
    return "AM64x";
}

function getProjectSpecDevice(board) {
    return "AM64x";
}

function getSysCfgCpu(cpu) {
    return cpu;
}

function getSysCfgPkg(board) {
    return "ALV";
}

function getSysCfgPart(board) {
    return "Default";
}

function getDevToolTirex(board) {
    switch (board) {
        case "am64x-sk":
            return "SK-AM64B";
        default:
        case "am64x-evm":
            return "TMDS64EVM";
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
            return "am64-main-r5f0_0-fw";
        case "r5fss0-1":
            return "am64-main-r5f0_1-fw";
        case "r5fss1-0":
            return "am64-main-r5f1_0-fw";
        case "r5fss1-1":
            return "am64-main-r5f1_1-fw";
        case "m4fss0-0":
            return "am64-mcu-m4f0_0-fw";
    }
    return undefined;
}

function getProductNameProjectSpec() {
    return "MOTOR_CONTROL_SDK_AM64X";
}

function getFlashAddr() {
    return 0x60000000;
}

function getEnableGccBuild() {
    const IsGccBuildEnabled = 0;
    return IsGccBuildEnabled;
}
function getOsList(cpu) {
    switch(cpu) {
        default:
            return ["nortos", "freertos"];
        case "a53":
        case "a53-smp":
            return ["nortos", "freertos", "freertos-smp"];
    }
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
    getEnableGccBuild,
    getOsList,
};

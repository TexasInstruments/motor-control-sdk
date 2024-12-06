const common = require("../common.js");

const component_file_list = [
    "source/position_sense/bissc/.project/project.js",
    "source/position_sense/endat/.project/project.js",
    "source/position_sense/hdsl/.project/project.js",
    "source/position_sense/nikon/.project/project.js",
    "source/position_sense/tamagawa/.project/project.js",
];

const device_defines = {
    common: [
        "SOC_AM261X",
    ],
};

const example_file_list = [
    "examples/position_sense/bissc_diagnostic/single_channel/.project/project.js",
    "examples/position_sense/endat_diagnostic/single_channel/.project/project.js",
    "examples/position_sense/hdsl_diagnostic/single_channel/.project/project.js",
    "examples/position_sense/nikon_diagnostic/single_channel/.project/project.js",
    "examples/position_sense/tamagawa_diagnostic/single_channel/.project/project.js",
    "source/position_sense/bissc/firmware/single_channel/.project/project.js",
    "source/position_sense/endat/firmware/single_channel/.project/project.js",
    "source/position_sense/hdsl/firmware/freerun_225_mhz/.project/project.js",
    "source/position_sense/hdsl/firmware/sync_225_mhz/.project/project.js",
    "source/position_sense/nikon/firmware/single_channel/.project/project.js",
    "source/position_sense/tamagawa/firmware/single_channel/.project/project.js",
   
];

function getProjectSpecCpu(cpu) {
    let projectSpecCpu =
    {
        "r5fss0-0": "Cortex_R5_0",
        "r5fss0-1": "Cortex_R5_1",
		"icss_m0_pru0": "ICSS_M0_PRU_0",
        "icss_m0_pru1": "ICSS_M0_PRU_1",
		"icss_m1_pru0": "ICSS_M1_PRU_0",
        "icss_m1_pru1": "ICSS_M1_PRU_1",
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
        case "am261x-lp":
            return "AM261x_ZFG";
        case "am261x-lp (dp83826 phy)":
            return "AM261x_ZFG";
        case "am261x-lp (dp83tg720 phy)":
            return "AM261x_ZFG";
        case "am261x-som":
	default:
            return "AM261x_ZCZ";
    }
}

function getProjectSpecDevice(board) {
    switch (board) {
        case "am261x":
            return "AM261x";
		case "am261x-lp":
			return "AM261x";
        case "am261x-lp (dp83826 phy)":
            return "AM261x";
        case "am261x-lp (dp83tg720 phy)":
            return "AM261x";
        case "am261x-som":
        default:
            return "AM261x";
    }
}

function getSysCfgCpu(cpu) {
    return cpu;
}

function getSysCfgPkg(board) {
    switch (board) {
        case "am261x-lp":
            return "ZFG";
        case "am261x-lp (dp83826 phy)":
            return "ZFG";
        case "am261x-lp (dp83tg720 phy)":
            return "ZFG";
        case "am261x-som":
        default:
            return "ZCZ";
    }
}

function getSysCfgPart(board) {
    switch (board) {
        case "am261x-lp":
            return "AM2612";
        case "am261x-lp (dp83826 phy)":
            return "AM2612";
        case "am261x-lp (dp83tg720 phy)":
            return "AM2612";
        case "am261x-som":
        default:
            return "AM2611";
    }
}

function getDevToolTirex(board) {
	switch (board) {
		case "am261x-lp":
			return "LP-AM263P";
		case "am261x-lp (dp83826 phy)":
			return "LP-AM263P";
		case "am261x-lp (dp83tg720 phy)":
			return "LP-AM263P";
		case "am261x-som":
		default:
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
            return "am261-main-r5f0_0-fw";
        case "r5fss0-1":
            return "am261-main-r5f0_1-fw";
        case "r5fss1-0":
            return "am261-main-r5f1_0-fw";
        case "r5fss1-1":
            return "am261-main-r5f1_1-fw";
    }
    return undefined;
}

function getProductNameProjectSpec() {
    return "MOTOR_CONTROL_SDK_AM261X";
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

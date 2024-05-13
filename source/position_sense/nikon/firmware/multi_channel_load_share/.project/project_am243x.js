let path = require('path');

let device = "am243x";

const files = {
    common: [
        "nikon_main.asm",
        "nikon_diagnostic.cmd",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../..", /* Example base */
        "../../../..",
    ],
};

const includes = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/nikon/firmware",
    ],
};

const defines_rtu = {
    common: [
        "ENABLE_MULTI_MAKE_RTU",

    ],

};
const defines_pru = {
    common: [

        "ENABLE_MULTI_MAKE_PRU",

    ],

};
const defines_txpru = {
    common: [
        "ENABLE_MULTI_MAKE_TXPRU",
    ],

};



const readmeDoxygenPageTag = "NIKON_DESIGN";

const cflags = {
    common: [
        "-v4"
    ],
};

const lflags = {
    common: [
        "--diag_suppress=10063-D", /* Added to suppress entry_point related warning */
        "--entry_point=NIKON_INIT",
        "--disable_auto_rts",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icssg0-rtupru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icssg0-txpru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
];

function getmakefilePruPostBuildSteps(cpu, board)
{
    let postBuildSteps

    switch(cpu)
    {
        case "icssg0-pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=NikonFirmwareMultiMakePRU -o nikon_receiver_multi_PRU_bin.h  nikon_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h nikon_receiver_multi_PRU_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/nikon/firmware/nikon_receiver_multi_PRU_bin.h;"+ 
            "$(RM)  nikon_receiver_multi_PRU_bin.h;"]
            break;
        case "icssg0-rtupru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=NikonFirmwareMultiMakeRTU -o nikon_receiver_multi_RTU_bin.h  nikon_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h nikon_receiver_multi_RTU_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/nikon/firmware/nikon_receiver_multi_RTU_bin.h;"+ 
            "$(RM)  nikon_receiver_multi_RTU_bin.h;"]
            break;
        case "icssg0-txpru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=NikonFirmwareMultiMakeTXPRU -o nikon_receiver_multi_TXPRU_bin.h  nikon_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h nikon_receiver_multi_TXPRU_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/nikon/firmware/nikon_receiver_multi_TXPRU_bin.h;"+ 
            "$(RM)  nikon_receiver_multi_TXPRU_bin.h;"]
            break;
    }

    return postBuildSteps
}

function getccsPruPostBuildSteps(cpu, board)
{
    let postBuildSteps

    switch(cpu)
    {
        case "icssg0-pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=NikonFirmwareMultiMakePRU -o nikon_receiver_multi_PRU_bin.h  nikon_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h nikon_receiver_multi_PRU_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/nikon/firmware/nikon_receiver_multi_PRU_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm nikon_receiver_multi_PRU_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h nikon_receiver_multi_PRU_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/nikon/firmware/nikon_receiver_multi_PRU_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm nikon_receiver_multi_PRU_bin.h;"]
            break;
        case "icssg0-rtupru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=NikonFirmwareMultiMakeRTU -o nikon_receiver_multi_RTU_bin.h  nikon_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h nikon_receiver_multi_RTU_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/nikon/firmware/nikon_receiver_multi_RTU_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm nikon_receiver_multi_RTU_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h nikon_receiver_multi_RTU_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/nikon/firmware/nikon_receiver_multi_RTU_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm nikon_receiver_multi_RTU_bin.h;"]
            break;
        case "icssg0-txpru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=NikonFirmwareMultiMakeTXPRU -o nikon_receiver_multi_TXPRU_bin.h  nikon_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h nikon_receiver_multi_TXPRU_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/nikon/firmware/nikon_receiver_multi_TXPRU_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm nikon_receiver_multi_TXPRU_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h nikon_receiver_multi_TXPRU_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/nikon/firmware/nikon_receiver_multi_TXPRU_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm nikon_receiver_multi_TXPRU_bin.h;"]
            break;
    }
    return postBuildSteps
}

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "nikon_peripheral_interface_multi_ch_load_share";
    property.isInternal = false;
    property.description = "NIKON Multi channel Interface for Different Make Encoders"
    property.buildOptionCombos = buildOptionCombos;
    property.pru_main_file = "main";
    property.pru_linker_file = "linker";
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;
    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    if(buildOption.cpu.match("icssg0-pru1"))
    {
        build_property.defines = defines_pru;
    }
    if(buildOption.cpu.match("icssg0-rtupru1"))
    {
        build_property.defines = defines_rtu;
    }
    if(buildOption.cpu.match("icssg0-txpru1"))
    {
        build_property.defines = defines_txpru;
    }
    build_property.ccsPruPostBuildSteps = getccsPruPostBuildSteps(buildOption.cpu, buildOption.board);
    build_property.makefilePruPostBuildSteps = getmakefilePruPostBuildSteps(buildOption.cpu, buildOption.board);
    build_property.cflags = cflags;
    build_property.lflags = lflags;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projecspecFileAction = "copy";
    build_property.skipMakefileCcsBootimageGen = true;
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};

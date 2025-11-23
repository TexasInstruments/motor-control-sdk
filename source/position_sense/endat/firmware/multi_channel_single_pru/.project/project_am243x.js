let path = require('path');

let device = "am243x";

const files = {
    common: [
        "endat_main.asm",
        "endat_diagnostic.cmd",
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
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware",
    ],
};

const defines = {
    common: [
        "ENABLE_MULTI_CHANNEL",
    ],
};



const readmeDoxygenPageTag = "ENDAT_DESIGN";

const cflags = {
    common: [
        "-v4"
    ],
};

const lflags = {
    common: [
        "--diag_suppress=10063-D", /* Added to suppress entry_point related warning */
        "--entry_point=ENDAT_INIT",
        "--disable_auto_rts",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "icss_g0_pru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icss_g0_pru0", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
];

function getmakefilePruPostBuildSteps(cpu, board)
{
    let postBuildSteps
    switch(cpu)
    {
        case "icss_g0_pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=EnDatFirmwareMultiPru1 -o endat_receiver_multi_pru1_bin.h endat_peripheral_interface_multi_ch_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h endat_receiver_multi_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware/multi_channel_single_pru/endat_receiver_multi_pru1_bin.h;"+ 
            "$(RM) endat_receiver_multi_pru1_bin.h;"]
            break;
        case "icss_g0_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=EnDatFirmwareMultiPru0 -o endat_receiver_multi_pru0_bin.h endat_peripheral_interface_multi_ch_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h endat_receiver_multi_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware/multi_channel_single_pru/endat_receiver_multi_pru0_bin.h;"+ 
            "$(RM) endat_receiver_multi_pru0_bin.h;"]        
    }

    return postBuildSteps;
}

function getccsPruPostBuildSteps(cpu, board)
{
    let postBuildSteps
    switch(cpu)
    {
        case "icss_g0_pru1":
            postBuildSteps = [ "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=EnDatFirmwareMultiPru1 -o endat_receiver_multi_pru1_bin.h endat_peripheral_interface_multi_ch_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h endat_receiver_multi_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware/multi_channel_single_pru/endat_receiver_multi_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm endat_receiver_multi_pru1_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h endat_receiver_multi_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware/multi_channel_single_pru/endat_receiver_multi_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm endat_receiver_multi_pru1_bin.h;"]
            break;
        case "icss_g0_pru0":
            postBuildSteps = [ "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=EnDatFirmwareMultiPru0 -o endat_receiver_multi_pru0_bin.h endat_peripheral_interface_multi_ch_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h endat_receiver_multi_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware/multi_channel_single_pru/endat_receiver_multi_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm endat_receiver_multi_pru0_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h endat_receiver_multi_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware/multi_channel_single_pru/endat_receiver_multi_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm endat_receiver_multi_pru0_bin.h;"]
    }
    return postBuildSteps;
}


function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "endat_peripheral_interface_multi_ch";
    property.isInternal = false;
    property.description = "Endat Multi Channel Interface for Same make encoders"
    property.buildOptionCombos = buildOptionCombos;
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;
    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.defines = defines;
    build_property.cflags = cflags;
    build_property.lflags = lflags;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projecspecFileAction = "copy";
    build_property.skipMakefileCcsBootimageGen = true;
    build_property.ccsPruPostBuildSteps = getccsPruPostBuildSteps(buildOption.cpu, buildOption.board);
    build_property.makefilePruPostBuildSteps = getmakefilePruPostBuildSteps(buildOption.cpu, buildOption.board);

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};

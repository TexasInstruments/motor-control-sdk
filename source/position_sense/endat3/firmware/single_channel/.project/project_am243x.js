let path = require('path');

let device = "am243x";

const files = {
    common: [
        "endat3_main.asm",
        "endat3_diagnostic.cmd"
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../..",
    ],
};

const includes = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat3/firmware",
    ],
};

const defines = {
    common: [

    ],
};

const readmeDoxygenPageTag = "ENDAT3_DESIGN";

const cflags = {
    common: [
        "-v4"
    ],
};

const lflags = {
    common: [
        "--diag_suppress=10063-D", /* Added to suppress entry_point related warning */
        "--entry_point=endat3_init",
        "--disable_auto_rts",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "icss_g0_pru1", cgt: "ti-pru-cgt", board: "am243x-lp", os: "fw"},
    { device: device, cpu: "icss_g0_pru0", cgt: "ti-pru-cgt", board: "am243x-lp", os: "fw"},
];

function getmakefilePruPostBuildSteps(cpu, board)
{
    let postBuildSteps
    switch(cpu)
    {
        case "icss_g0_pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=EnDat3FirmwarePru1 -o endat3_receiver_pru1_bin.h endat3_peripheral_interface_single_channel_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h endat3_receiver_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat3/firmware/single_channel/endat3_receiver_pru1_bin.h;"+
            "$(RM) endat3_receiver_pru1_bin.h;"]
            break;
        case "icss_g0_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=EnDat3FirmwarePru0 -o endat3_receiver_pru0_bin.h endat3_peripheral_interface_single_channel_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h endat3_receiver_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat3/firmware/single_channel/endat3_receiver_pru0_bin.h;"+
            "$(RM) endat3_receiver_pru0_bin.h;"]
            break;
    }
    return postBuildSteps;
}

function getccsPruPostBuildSteps(cpu, board)
{
    let postBuildSteps
    switch(cpu)
    {
        case "icss_g0_pru1":
            postBuildSteps =["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=EnDat3FirmwarePru1 -o endat3_receiver_pru1_bin.h endat3_peripheral_interface_single_channel_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h endat3_receiver_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat3/firmware/single_channel/endat3_receiver_pru1_bin.h;"+
            "if ${CCS_HOST_OS} == linux rm endat3_receiver_pru1_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h endat3_receiver_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat3/firmware/single_channel/endat3_receiver_pru1_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm endat3_receiver_pru1_bin.h;"]
            break;
        case "icss_g0_pru0":
            postBuildSteps =["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=EnDat3FirmwarePru0 -o endat3_receiver_pru0_bin.h endat3_peripheral_interface_single_channel_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h endat3_receiver_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat3/firmware/single_channel/endat3_receiver_pru0_bin.h;"+
            "if ${CCS_HOST_OS} == linux rm endat3_receiver_pru0_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h endat3_receiver_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat3/firmware/single_channel/endat3_receiver_pru0_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm endat3_receiver_pru0_bin.h;"]
            break;
    }
    return postBuildSteps;
}

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "endat3_peripheral_interface_single_channel";
    property.isInternal = false;
    property.description = "EnDAT3 Peripheral Interface"
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

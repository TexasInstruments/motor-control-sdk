let path = require('path');

let device = "am263x";

const files = {
    common: [
        "tamagawa_main.asm",
        "tamagawa_diagnostic.cmd"
    ],
};

const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../../..", /* Example base */
    ],
};

const includes = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware",
    ],
};

const readmeDoxygenPageTag = "TAMAGAWA_DESIGN";

const cflags = {
    common: [
        "-v4"
    ],
};

const lflags = {
    common: [
        "--diag_suppress=10063-D", /* Added to suppress entry_point related warning */
        "--entry_point=TAMAGAWA_INIT",
        "--disable_auto_rts",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "icss_m0_pru0", cgt: "ti-pru-cgt", board: "am263x-lp", os: "fw"},
    { device: device, cpu: "icss_m0_pru1", cgt: "ti-pru-cgt", board: "am263x-lp", os: "fw"},
];


function getmakefilePruPostBuildSteps(cpu, board)
{
    let postBuildSteps

    switch(cpu)
    {
        case "icss_m0_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwarePru0 -o tamagawa_receiver_single_channel_pru0_bin.h tamagawa_single_channel_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h tamagawa_receiver_single_channel_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/single_channel/tamagawa_receiver_single_channel_pru0_bin.h;"+ 
            "$(RM) tamagawa_receiver_single_channel_pru0_bin.h;"]
            break;
        case "icss_m0_pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwarePru1 -o tamagawa_receiver_single_channel_pru1_bin.h tamagawa_single_channel_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h tamagawa_receiver_single_channel_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/single_channel/tamagawa_receiver_single_channel_pru1_bin.h;"+ 
            "$(RM) tamagawa_receiver_single_channel_pru1_bin.h;"]
            break;
    }
    return postBuildSteps
}

function getccsPruPostBuildSteps(cpu, board)
{
    let postBuildSteps

    switch(cpu)
    {
        case "icss_m0_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwarePru0 -o tamagawa_receiver_single_channel_pru0_bin.h tamagawa_single_channel_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h tamagawa_receiver_single_channel_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/single_channel/tamagawa_receiver_single_channel_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm tamagawa_receiver_single_channel_pru0_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h tamagawa_receiver_single_channel_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/single_channel/tamagawa_receiver_single_channel_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm tamagawa_receiver_single_channel_pru0_bin.h;"]
            break;
        case "icss_m0_pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwarePru1 -o tamagawa_receiver_single_channel_pru1_bin.h tamagawa_single_channel_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h tamagawa_receiver_single_channel_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/single_channel/tamagawa_receiver_single_channel_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm tamagawa_receiver_single_channel_pru1_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/pru_load_bin_copyright.h tamagawa_receiver_single_channel_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/single_channel/tamagawa_receiver_single_channel_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm tamagawa_receiver_single_channel_pru1_bin.h;"]
            break;
    }
    return postBuildSteps
}

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "tamagawa_single_channel";
    property.isInternal = false;
    property.description = "Tamagawa Peripheral Interface"
    property.buildOptionCombos = buildOptionCombos;
    property.pru_main_file = "tamagawa_main";
    property.pru_linker_file = "tamagawa_diagnostic";
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
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

let path = require('path');

let device = "am64x";

const files = {
    common: [
        "tamagawa_main.asm",
        "tamagawa_diagnostic.cmd",
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
    { device: device, cpu: "icssg0-pru1", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
];

function getmakefilePruPostBuildSteps(cpu, board)
{
    return  [
        "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmware -o tamagawa_master_single_channel_bin.h tamagawa_single_channel_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
        "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h tamagawa_master_single_channel_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/tamagawa_master_single_channel_bin.h;"+ 
        "$(RM) tamagawa_master_single_channel_bin.h;"
    ];
}

function getccsPruPostBuildSteps(cpu, board)
{
    return  [
        "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmware -o tamagawa_master_single_channel_bin.h tamagawa_single_channel_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
        "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h tamagawa_master_single_channel_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/tamagawa_master_single_channel_bin.h;"+ 
        "if ${CCS_HOST_OS} == linux rm tamagawa_master_single_channel_bin.h;"+
        "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h tamagawa_master_single_channel_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/tamagawa_master_single_channel_bin.h;"+ 
        "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm tamagawa_master_single_channel_bin.h;"
    ];
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

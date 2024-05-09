let path = require('path');

let device = "am64x";

const files = {
    common: [
        "main.asm",
        "datalink.asm",
        "datalink_init.asm",
        "transport.asm",
        "utils.asm",
        "hdsl_master_icssg.cmd",
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
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/hdsl/firmware",
    ],
};

const defines = {
    common: [
        "PRU1",
        "CHANNEL_0",
        "EXT_SYNC_ENABLE",
        "ICSS_G_V_1_0",
    ],
};

const lflags = {
    common: [
        "--disable_auto_rts",
        "--diag_suppress=10063-D", /* Added to suppress entry_point related warning */
        "--entry_point=main",
    ],
};

function getmakefilePruPostBuildSteps(cpu, board)
{
    return  [
        "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=Hiperface_DSL_SYNC2_0_RTU -o hdsl_master_icssg_sync_225_mhz_bin.h  hdsl_master_sync_225_mhz_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
        "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h hdsl_master_icssg_sync_225_mhz_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/hdsl/firmware/hdsl_master_icssg_sync_225_mhz_bin.h;"+ 
        "$(RM) hdsl_master_icssg_sync_225_mhz_bin.h;"
    ];
}

function getccsPruPostBuildSteps(cpu, board)
{
    return  [
        "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=Hiperface_DSL_SYNC2_0_RTU -o hdsl_master_icssg_sync_225_mhz_bin.h  hdsl_master_sync_225_mhz_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
        "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h hdsl_master_icssg_sync_225_mhz_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/ hdsl/firmware/hdsl_master_icssg_sync_225_mhz_bin.h;"+ 
        "if ${CCS_HOST_OS} == linux rm hdsl_master_icssg_sync_225_mhz_bin.h;"+
        "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h hdsl_master_icssg_sync_225_mhz_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/hdsl/firmware/hdsl_master_icssg_sync_225_mhz_bin.h;"+ 
        "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm hdsl_master_icssg_sync_225_mhz_bin.h;"
    ];
}

const readmeDoxygenPageTag = "HDSL_DESIGN";

const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru1", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "hdsl_master_sync_225_mhz";
    property.description = "HDSL Master Sync Mode Firmware for PRU-ICSS running at 225 MHz";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;
    property.pru_main_file = "main";
    property.pru_linker_file = "hdsl_master_icssg";
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
    build_property.lflags = lflags;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.ccsPruPostBuildSteps = getccsPruPostBuildSteps(buildOption.cpu, buildOption.board);
    build_property.makefilePruPostBuildSteps = getmakefilePruPostBuildSteps(buildOption.cpu, buildOption.board);
    build_property.projecspecFileAction = "copy";
    build_property.skipMakefileCcsBootimageGen = true;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};

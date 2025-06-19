let path = require('path');

let device = "am243x";

const files = {
    common: [
        "sdfm.asm",
        "icssg_pru.cmd",
    ],
};

const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../..",
        "../../../..", /* Example base */

    ],
};

const includes = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware",
    ],
};

const readmeDoxygenPageTag = "SDFM_DESIGN";

const cflags_pru = {
    common: [
        "-v4",
        "-DSDFM_PRU_CORE",
        "-o2",
        "--display_error_number",
        "--hardware_mac=on",
    ],
};

const lflags_pru = {
    common: [
        "--disable_auto_rts",
        "--diag_suppress=10063-D", /* Added to suppress entry_point related warning */
        "--entry_point=SDFM_ENTRY",
        "--define=SDFM_PRU_CORE=1",
        "--warn_sections",
        "--zero_init=off",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "icss_g0_pru0", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icss_g0_pru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},

];

function getmakefilePruPostBuildSteps(cpu, board)
{
    let postBuildSteps
    switch(cpu)
    {
        case "icss_g0_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=SDFM_PRU0_image -o sdfm_pru0_bin.h sdfm_firmware_single_axis_single_pru_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/single_axis_single_pru/sdfm_pru0_bin.h;"+ 
            "$(RM) sdfm_pru0_bin.h;"]
            break;
        case "icss_g0_pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=SDFM_PRU1_image -o sdfm_pru1_bin.h sdfm_firmware_single_axis_single_pru_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/single_axis_single_pru/sdfm_pru1_bin.h;"+ 
            "$(RM) sdfm_pru1_bin.h;"]
            break;
    }
    return postBuildSteps;
}

function getccsPruPostBuildSteps(cpu, board)
{
    let postBuildSteps
    switch(cpu)
    {
        case "icss_g0_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=SDFM_PRU0_image -o sdfm_pru0_bin.h sdfm_firmware_single_axis_single_pru_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/single_axis_single_pru/sdfm_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm sdfm_pru0_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/single_axis_single_pru/sdfm_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm sdfm_pru0_bin.h;"]
            break;
        case "icss_g0_pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=SDFM_PRU0_image -o sdfm_pru1_bin.h sdfm_firmware_single_axis_single_pru_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/single_axis_single_pru/sdfm_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm sdfm_pru1_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/single_axis_single_pru/sdfm_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm sdfm_pru1_bin.h;"]
            break;
    }    
    return postBuildSteps;
}

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "sdfm_firmware_single_axis_single_pru";
    property.isInternal = false;
    property.description = "ICSS SDFM"
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
    build_property.cflags = cflags_pru;
    build_property.lflags = lflags_pru;
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
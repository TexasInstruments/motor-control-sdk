let path = require('path');

let device = "am243x";

const files = {
    common: [
        "sdfm.asm",
        "icssg_pru.cmd",
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
        "${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware",
    ],
};


const defines_rtu = {
    common: [
        "SDFM_RTU_CORE",
        "SDFM_LOAD_SHARE_MODE",

    ],

};
const defines_pru = {
    common: [

        "SDFM_PRU_CORE",
        "SDFM_LOAD_SHARE_MODE",

    ],

};
const defines_txpru = {
    common: [
        "SDFM_TXPRU_CORE",
        "SDFM_LOAD_SHARE_MODE",
    ],

};

const readmeDoxygenPageTag = "SDFM_DESIGN";

const cflags = {
    common: [
        "-v4",
        "-o2",
        "--display_error_number",
        "--hardware_mac=on",
    ],
};

const lflags = {
    common: [
        "--warn_sections",
        "--entry_point=SDFM_ENTRY",
        "--zero_init=off",
        "--disable_auto_rts",
        "--define=SDFM_LOAD_SHARE_MODE=1",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru0", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icssg0-rtupru0", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icssg0-txpru0", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
];

function getmakefilePruPostBuildSteps(cpu, board)
{
    let postBuildSteps

    switch(cpu)
    {
        case "icssg0-pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=pru_SDFM_PRU0_image -o sdfm_pru_bin.h  sdfm_firmware_multi_axis_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_pru_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/sdfm_pru_bin.h;"+ 
            "$(RM)  sdfm_pru_bin.h;"]
            break;
        case "icssg0-rtupru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=pru_SDFM_RTU0_image -o sdfm_rtu_bin.h  sdfm_firmware_multi_axis_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_rtu_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/sdfm_rtu_bin.h;"+ 
            "$(RM)  sdfm_rtu_bin.h;"]
            break;
        case "icssg0-txpru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=pru_SDFM_TXPRU0_image -o sdfm_txpru_bin.h  sdfm_firmware_multi_axis_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_txpru_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/sdfm_txpru_bin.h;"+ 
            "$(RM)  sdfm_txpru_bin.h;"]
            break;
    }

    return postBuildSteps
}

function getccsPruPostBuildSteps(cpu, board)
{
    let postBuildSteps

    switch(cpu)
    {
        case "icssg0-pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=pru_SDFM_PRU0_image -o sdfm_pru_bin.h  sdfm_firmware_multi_axis_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_pru_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/sdfm_pru_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm sdfm_pru_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_pru_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/sdfm_pru_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm sdfm_pru_bin.h;"]
            break;
        case "icssg0-rtupru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=pru_SDFM_RTU0_image -o sdfm_rtu_bin.h  sdfm_firmware_multi_axis_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_rtu_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/sdfm_rtu_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm sdfm_rtu_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_rtu_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/sdfm_rtu_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm sdfm_rtu_bin.h;"]
            break;
        case "icssg0-txpru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=pru_SDFM_TXPRU0_image -o sdfm_txpru_bin.h  sdfm_firmware_multi_axis_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_txpru_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/sdfm_txpru_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm sdfm_txpru_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h sdfm_txpru_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/sdfm_txpru_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm sdfm_txpru_bin.h;"]
            break;
    }
    return postBuildSteps
}

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "sdfm_firmware_multi_axis_load_share";
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
    if(buildOption.cpu.match("icssg0-pru0"))
    {
        build_property.defines = defines_pru;
    }
    if(buildOption.cpu.match("icssg0-rtupru0"))
    {
        build_property.defines = defines_rtu;
    }
    if(buildOption.cpu.match("icssg0-txpru0"))
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
let path = require('path');

let device = "am243x";

const files = {
    common: [
        "tamagawa_main.asm",
        "tamagawa_diagnostic.cmd"
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
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware",
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
    { device: device, cpu: "icss_g0_pru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icss_g0_rtu_pru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icss_g0_tx_pru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icss_g0_pru0", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icss_g0_rtu_pru0", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icss_g0_tx_pru0", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
];

function getmakefilePruPostBuildSteps(cpu, board)
{
    let postBuildSteps

    switch(cpu)
    {
        case "icss_g0_pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakePru1 -o tamagawa_receiver_multi_pru1_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_pru1_bin.h;"+ 
            "$(RM)  tamagawa_receiver_multi_pru1_bin.h;"]
            break;
        case "icss_g0_rtu_pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakeRtuPru1 -o tamagawa_receiver_multi_rtu_pru1_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_rtu_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_rtu_pru1_bin.h;"+ 
            "$(RM)  tamagawa_receiver_multi_rtu_pru1_bin.h;"]
            break;
        case "icss_g0_tx_pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakeTxPru1 -o tamagawa_receiver_multi_tx_pru1_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_tx_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_tx_pru1_bin.h;"+ 
            "$(RM)  tamagawa_receiver_multi_tx_pru1_bin.h;"]
            break;
        case "icss_g0_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakePru0 -o tamagawa_receiver_multi_pru0_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_pru0_bin.h;"+ 
            "$(RM)  tamagawa_receiver_multi_pru0_bin.h;"]
            break;
        case "icss_g0_rtu_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakeRtuPru0 -o tamagawa_receiver_multi_rtu_pru0_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_rtu_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_rtu_pru0_bin.h;"+ 
            "$(RM)  tamagawa_receiver_multi_rtu_pru0_bin.h;"]
            break;
        case "icss_g0_tx_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakeTxPru0 -o tamagawa_receiver_multi_tx_pru0_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_tx_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_tx_pru0_bin.h;"+ 
            "$(RM)  tamagawa_receiver_multi_tx_pru0_bin.h;"]
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
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakePru1 -o tamagawa_receiver_multi_pru1_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm tamagawa_receiver_multi_pru1_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm tamagawa_receiver_multi_pru1_bin.h;"]
            break;
        case "icss_g0_rtu_pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakeRtuPru1 -o tamagawa_receiver_multi_rtu_pru1_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_rtu_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_rtu_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm tamagawa_receiver_multi_rtu_pru1_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_rtu_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_rtu_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm tamagawa_receiver_multi_rtu_pru1_bin.h;"]
            break;
        case "icss_g0_tx_pru1":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakeTxPru1 -o tamagawa_receiver_multi_tx_pru1_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_tx_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_tx_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm tamagawa_receiver_multi_tx_pru1_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_tx_pru1_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_tx_pru1_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm tamagawa_receiver_multi_tx_pru1_bin.h;"]
            break;
        case "icss_g0_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakePru0 -o tamagawa_receiver_multi_pru0_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm tamagawa_receiver_multi_pru0_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm tamagawa_receiver_multi_pru0_bin.h;"]
            break;
        case "icss_g0_rtu_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakeRtuPru0 -o tamagawa_receiver_multi_rtu_pru0_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_rtu_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_rtu_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm tamagawa_receiver_multi_rtu_pru0_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_rtu_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_rtu_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm tamagawa_receiver_multi_rtu_pru0_bin.h;"]
            break;
        case "icss_g0_tx_pru0":
            postBuildSteps = ["$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=TamagawaFirmwareMultiMakeTxPru0 -o tamagawa_receiver_multi_tx_pru0_bin.h  tamagawa_peripheral_interface_multi_ch_load_share_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
            "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_tx_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_tx_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == linux rm tamagawa_receiver_multi_tx_pru0_bin.h;"+
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h tamagawa_receiver_multi_tx_pru0_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/multi_channel_load_share/tamagawa_receiver_multi_tx_pru0_bin.h;"+ 
            "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm tamagawa_receiver_multi_tx_pru0_bin.h;"]
            break;
    }
    return postBuildSteps;
}

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "tamagawa_peripheral_interface_multi_ch_load_share";
    property.isInternal = false;
    property.description = "Tamagawa Multi channel Interface for Different Make Encoders"
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
    if(buildOption.cpu.match("icss_g0_pru1") || buildOption.cpu.match("icss_g0_pru0") )
    {
        build_property.defines = defines_pru;
    }
    if(buildOption.cpu.match("icss_g0_rtu_pru1") || buildOption.cpu.match("icss_g0_rtu_pru0"))
    {
        build_property.defines = defines_rtu;
    }
    if(buildOption.cpu.match("icss_g0_tx_pru1") || buildOption.cpu.match("icss_g0_tx_pru0"))
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

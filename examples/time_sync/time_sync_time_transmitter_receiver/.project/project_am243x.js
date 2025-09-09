let path = require('path');

let device = "am243x";

const files = {
    common: [
        "main.c",
        "time_sync_main.c",
        "time_sync.c",
        "time_sync.h"
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../.."
    ],
};

const defines = {
    common: [
        "am243x_evm",
        "TIME_TRANSMITTER_RECEIVER",
        "OS_FREERTOS"
    ],
};

const libdirs_freertos = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/lib",
    ],
};

const includes_freertos_r5f_am243x_evm = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/config/am243x/r5f",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/driver",
        "${MOTOR_CONTROL_SDK_PATH}/source/pruicss_iep_sync_out_generation/am243x-evm",
    ],
};

const includes_freertos_r5f_am243x_lp = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/config/am243x/r5f",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/driver",
        "${MOTOR_CONTROL_SDK_PATH}/source/pruicss_iep_sync_out_generation/am243x-evm",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg"

const readmeDoxygenPageTag = "TIME_SYNC_TRANSMITTER_RECEIVER";

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "pru_icss_with_time_sync_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
];


function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "time_sync_transmitter_receiver";
    property.isInternal = false;
    property.description = "time sync transmitter receiver R5F Project"
    property.buildOptionCombos = buildOptionCombos;
    property.isSkipTopLevelBuild = false;
    property.skipUpdatingTirex = false;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    
    if(buildOption.board=="am243x-evm"){
        build_property.includes = includes_freertos_r5f_am243x_evm;
        build_property.defines = defines;
    }else{
        build_property.includes = includes_freertos_r5f_am243x_lp;
    }

    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    
    build_property.libdirs = libdirs_freertos;
    build_property.libs = libs_freertos_r5f;
    build_property.templates = templates_freertos_r5f;
    
    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};

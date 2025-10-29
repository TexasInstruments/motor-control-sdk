let path = require('path');

let device = "am261x";

const files = {
    common: [
        "bissc_diagnostic.c",
        "bissc_periodic_trigger.c",
        "main.c",
    ],
};

const projectspec_files = {
    common: [
        "../../../../bissc_periodic_trigger.h",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../../..", /* Example base */
    ],
};

const libdirs_freertos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/bissc/lib",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am261x/r5f",
        "${MOTOR_CONTROL_SDK_PATH}/examples/position_sense/bissc_diagnostic",
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/bissc/firmware/single_channel",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "motorcontrol_bissc.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const defines_r5f = {
    common: [
        "SOC_AM261X",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLE_MOTORCONTROL_BISSC";

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am261x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "bissc_main",
            taskPri: 5,
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "freertos"},

];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "bissc_diagnostic_single_channel";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;
    property.isSkipTopLevelBuild = false;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.projectspec_files = projectspec_files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_r5f;
            build_property.libdirs = libdirs_freertos;
            build_property.libs = libs_freertos_r5f;
            build_property.templates = templates_freertos_r5f;
            build_property.defines = defines_r5f;
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};

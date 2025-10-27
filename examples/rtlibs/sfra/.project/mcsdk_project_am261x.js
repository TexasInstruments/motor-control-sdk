let path = require('path');

let device = "am261x";

const files = {
    common: [
        "sfra_main.c",
        "sfra_examples_hal.c",
    ],
};

const projectspec_files = {
    common: [
        "../../../sfra_main.h",
        "../../../sfra_examples_hal.h",
        "../../../sfra_examples_settings.h",
        "../../../sfra_expected.h",
        "${MOTOR_CONTROL_SDK_PATH}/source/rtlibs/sfra/sfra_f32.h"
    ]
}

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "../../..",
    ],
};

const libdirs_nortos = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib",
        "${MCU_PLUS_SDK_PATH}/source/drivers/lib",
        "${MCU_PLUS_SDK_PATH}/source/board/lib",
        "${MOTOR_CONTROL_SDK_PATH}/source/rtlibs/sfra/lib",

    ],
};

const includes_nortos_r5f = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/rtlibs/dcl",
        "${MOTOR_CONTROL_SDK_PATH}/source/rtlibs/sfra",
        "${MOTOR_CONTROL_SDK_PATH}/examples/rtlibs/sfra",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
        "sfra.am261x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};



const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../sfra.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_SFRA_TEST";

const templates_nortos_r5f =
[

];


const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am261x-lp", os: "nortos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "sfralib";
    property.isInternal = false;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "Verification of SFRA Library for PI Controller. "
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs_nortos;
    build_property.lnkfiles = lnkfiles;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projectspec_files = projectspec_files;

    if(buildOption.cpu.match(/r5f*/)) {
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_r5f;
            build_property.libdirs = libdirs_freertos;
            build_property.libs = libs_freertos_r5f;
            build_property.templates = templates_freertos_r5f;
        }
        else
        {
            build_property.includes = includes_nortos_r5f;
            build_property.libs = libs_nortos_r5f;
            build_property.templates = templates_nortos_r5f;
        }
    }


    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};

let path = require('path');

let device = "am263x";

const files = {
    common: [
        /* Project Files */
        "transforms_test.c", 
        "main.c",
        /* Dataset */
        /* Libraries */
    ],
};

const projectspec_files = {
    common: [
        "../../../transforms_test.h",
    ]
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
    ],
};

const libdirs_nortos = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/nortos/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/mathlib/lib",
    ],
};

const libdirs_freertos = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/mathlib/lib",
    ],
};

const includes_nortos_r5f = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/clarke",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/park",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/ipark",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/svgen",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/mathlib/trig",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/config/am263x/r5f",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/clarke",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/park",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/ipark",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/svgen",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/mathlib/trig",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "mathlib.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
        "mathlib.am263x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};


const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLES_TRANSFORMS_TEST";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am263x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "transforms_test_main",
        },
    }
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am263x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "transforms_test_main",
        },
    }
];


const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263x-cc", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263x-cc", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263x-lp", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am263x-lp", os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "transforms_test";
    property.isInternal = false;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "A simple test vector of ClARKE-PARK-IPARK-SVGEN data verification"
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

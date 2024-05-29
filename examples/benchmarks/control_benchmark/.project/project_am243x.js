let path = require('path');

let device = "am243x";

const files = {
    common: [
        /* Project Files */
        "benchmark_routine.c",
        "main.c",
        "fwc.c",
        "mtpa.c",
        "vs_freq.c",
    ],
};

const projectspec_files = {
    common: [
        "../../../user.h",
    ]
}

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../../../../source/control/fwc/source",
        "../../../../../../source/control/mtpa/source",
        "../../../../../../source/control/vs_freq/source",
    ],
};

const libdirs_nortos = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/nortos/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib",
    ],
};

const libdirs_freertos = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib",
    ],
};

const includes_nortos_r5f = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/dcl",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/fwc/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/mtpa/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/vs_freq/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/utilities",
        "${MOTOR_CONTROL_SDK_PATH}/examples/benchmarks/control_benchmark",
    ],
};

const includes_freertos_r5f = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/config/am243x/r5f",
        "${MOTOR_CONTROL_SDK_PATH}/source/dcl",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/fwc/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/mtpa/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/vs_freq/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/utilities",
        "${MOTOR_CONTROL_SDK_PATH}/examples/benchmarks/control_benchmark",
    ],
};

const includes_nortos_m4f = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/dcl",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/fwc/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/mtpa/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/vs_freq/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/utilities",
        "${MOTOR_CONTROL_SDK_PATH}/examples/benchmarks/control_benchmark",
    ],
};

const includes_freertos_m4f = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CM4F",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/config/am243x/m4f",
        "${MOTOR_CONTROL_SDK_PATH}/source/dcl",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/fwc/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/mtpa/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/control/vs_freq/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/utilities",
        "${MOTOR_CONTROL_SDK_PATH}/examples/benchmarks/control_benchmark",
    ],
};

const libs_nortos_r5f = {
    common: [
        "nortos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_r5f = {
    common: [
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_nortos_m4f = {
    common: [
        "nortos.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const libs_freertos_m4f = {
    common: [
        "freertos.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.m4f.ti-arm-clang.${ConfigName}.lib",
    ],
};

const lnkfiles_r5f = {
    common: [
        "linker.cmd",
    ]
};

const lnkfiles_m4f = {
    common: [
        "linker.cmd",
    ]
};

const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "BENCHMARKS_CONTROL";

const templates_nortos_r5f =
[
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "benchmark_main",
        },
    }
];

const templates_freertos_r5f =
[
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "benchmark_main",
        },
    }
];


const templates_nortos_m4f =
[
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "benchmark_main",
        },
    }
];

const templates_freertos_m4f =
[
    {
        input: ".project/templates/am243x/freertos/main_freertos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "benchmark_main",
        },
    }
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-evm", os: "freertos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp",  os: "nortos"},
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp",  os: "freertos"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "benchmark_control";
    property.isInternal = false;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "Benchmark for the controls library"
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    /* The following build property has been modified for dcl usage,
     specifically - added different includes vars distinguishing no/free rtos
                    added different lnkfiles vars distinguishing cpu-specific lnkfiles
    */

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.libdirs = libdirs_nortos;
    build_property.syscfgfile = syscfgfile;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.projectspec_files = projectspec_files;

    if(buildOption.cpu.match(/r5f*/)) {
        build_property.lnkfiles = lnkfiles_r5f;
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
    if(buildOption.cpu.match(/m4f*/)) {
        build_property.lnkfiles = lnkfiles_m4f;
        if(buildOption.os.match(/freertos*/) )
        {
            build_property.includes = includes_freertos_m4f;
            build_property.libdirs = libdirs_freertos;
            build_property.libs = libs_freertos_m4f;
            build_property.templates = templates_freertos_m4f;
        }
        else
        {
            build_property.includes = includes_nortos_m4f;
            build_property.libs = libs_nortos_m4f;
            build_property.templates = templates_nortos_m4f;
        }
    }

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
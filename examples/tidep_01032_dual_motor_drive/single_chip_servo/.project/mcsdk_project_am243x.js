let path = require('path');

let device = "am243x";

const files_r5f_0_0 = {
    common: [
        /* Project Files */
        "endat_periodic_trigger.c",
        "epwm_drv_aux.c",
        "mclk_iep_sync.c",
        "pwm.c",
        "sddf.c",
        "ti_r5fmath_trig.c",
        "single_chip_servo.c",
        "main.c",
    ],
};

const files_r5f_0_1 = {
    common: [
        /* Project Files */
        "epwm_drv_aux.c",
        "pwm.c",
        "ti_r5fmath_trig.c",
        "single_chip_servo.c",
        "main.c",
    ],
};

const files_r5f_1_0 = {
    common: [
        /* Project Files */
        "ESL_version.c",
        "ESL_OS_os.c",
        "ESL_eeprom_tidep_01032.c",
        "ESL_fileHandling.c",
        "ESL_foeDemo.c",
        "ESL_soeDemo.c",
        "ESL_gpioHelper.c",
        "CUST_PHY_base.c",
        "CUST_PHY_dp83869.c",
        "ecSlvCiA402.c",
        "ESL_cia402Obd.c",
        "EtherCAT_Slave_CiA402_tidep_01032.c",
        "ESL_cia402Demo_tidep_01032.c",
        "ESL_BOARD_OS_config_tidep_01032.c",
        "nvm.c",
        "nvm_drv_eeprom_tidep_01032.c",
        "nvm_drv_flash.c",
    ],
};

const projectspec_files_r5f_0_0 = {
    common: [
        "../DRV8350_defs.h",
        "../endat_periodic_trigger.h",
        "../epwm_drv_aux.h",
        "../mclk_iep_sync.h",
        "../pwm.h",
        "../sddf.h",
        "../tisddf_pruss_intc_mapping.h",
        "../settings.h",
        "../ti_r5fmath_trig.h",
    ],
};

const projectspec_files_r5f_0_1 = {
    common: [
        "../epwm_drv_aux.h",
        "../pwm.h",
        "../tisddf_pruss_intc_mapping.h",
        "../settings.h",
        "../ti_r5fmath_trig.h",
    ],
};

const projectspec_files_r5f_1_0 = {
    common: [
        "../project.h",
        "../version.h",
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs_r5f_0_0 = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
    ]
};

const filedirs_r5f_0_1 = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
    ],
};

const filedirs_r5f_1_0 = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../../../..", /* Motor Control SDK base */
        "../../../../../../ind_comms_sdk/examples/industrial_comms/ethercat_slave_demo/common",
        "../../../../../../ind_comms_sdk/examples/industrial_comms/ethercat_slave_demo/common/os/freertos",
        "../../../../../../ind_comms_sdk/examples/industrial_comms/custom_phy/src",
        "../../../../../../ind_comms_sdk/examples/industrial_comms/ethercat_slave_demo/device_profiles/402_cia",
        "../../../../../../ind_comms_sdk/examples/industrial_comms/nvm",
        "../../../../../../ind_comms_sdk/examples/industrial_comms/nvm/app/src",
        "../../../../../../ind_comms_sdk/examples/industrial_comms/nvm/app/inc",
        "../../../../../../ind_comms_sdk/examples/industrial_comms/nvm/drv/src",
        "../../../../../../ind_comms_sdk/examples/industrial_comms/nvm/drv/inc",
    ],
};

const libdirs_r5f_0_0 = {
    common: [
        "${CG_TOOL_ROOT}/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/nortos/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib",
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/lib",
        "${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/cmsis/lib",
    ],
};

const libdirs_r5f_0_1 = {
    common: [
        "${CG_TOOL_ROOT}/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/nortos/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib",
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/lib",
        "${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/cmsis/lib",
    ],
};

const libdirs_r5f_1_0 = {
    common: [
        "${CG_TOOL_ROOT}/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/source/industrial_comms/ethercat_slave/stack/lib",
    ],
};

const includes_r5f_0_0 = {
    common: [
        "${CG_TOOL_ROOT}/include/c",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/cmsis/DSP/Include",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/cmsis/Core/Include",
        "${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware",
        "${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware",
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/clarke",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/ipark",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/park",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/svgen",
        "${MOTOR_CONTROL_SDK_PATH}/source/dcl",
    ],
};

const includes_r5f_0_1 = {
    common: [
        "${CG_TOOL_ROOT}/include/c",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/cmsis/DSP/Include",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/cmsis/Core/Include",
        "${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware",
        "${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware",
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/clarke",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/ipark",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/park",
        "${MOTOR_CONTROL_SDK_PATH}/source/transforms/svgen",
        "${MOTOR_CONTROL_SDK_PATH}/source/dcl",
    ],
};

const includes_r5f_1_0 = {
    common: [
        "${CG_TOOL_ROOT}/include/c",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/source",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/FreeRTOS-Kernel/include",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F",
        "${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/freertos/config/am243x/r5f",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/examples/industrial_comms/ethercat_slave_demo/device_profiles/402_cia",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/examples/industrial_comms/ethercat_slave_demo/common",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/examples/industrial_comms/ethercat_slave_demo/common/os",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/examples/industrial_comms/ethercat_slave_demo/common/os/freertos",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/examples/industrial_comms/ethercat_slave_demo/common/board/am243lp",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/examples/industrial_comms/ethercat_slave_demo/common/board/am243lp/freertos",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/examples/industrial_comms/custom_phy/inc",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/examples/industrial_comms/nvm/app/inc",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/examples/industrial_comms/nvm/drv/inc",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/source/industrial_comms/ethercat_slave/stack/inc",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/source/industrial_comms/ethercat_slave/stack/inc/defines",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/source/industrial_comms/ethercat_slave/stack/inc/ext",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/source/industrial_comms/ethercat_slave/stack/inc/profiles",
        "${MOTOR_CONTROL_SDK_PATH}/ind_comms_sdk/source/industrial_comms/common/inc",
    ],
};

const libs_r5f_0_0 = {
    common: [
        "libc.a",
        "nortos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "motorcontrol_sdfm.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "motorcontrol_endat.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "cmsis.am243x.r5f.ti-arm-clang.release.lib",
    ],
};

const libs_r5f_0_1 = {
    common: [
        "libc.a",
        "nortos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "motorcontrol_sdfm.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "motorcontrol_endat.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "cmsis.am243x.r5f.ti-arm-clang.release.lib",
    ],
};

const libs_r5f_1_0 = {
    common: [
        "libc.a",
        "libsysbm.a",
        "freertos.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "board.am243x.r5f.ti-arm-clang.${ConfigName}.lib",
        "ethercat_slave.am243x_lp.r5f.ti-arm-clang.release.lib",
        "ethercat_slave_bkhf_ssc.am243x_lp.r5f.ti-arm-clang.release.lib",
    ],
};

const defines_r5f_1_0 = {
    common: [

        "SOC_AM243X=1",
        "OSAL_FREERTOS=1",
        "core0",
        "am243x",
        "am243x_lp",
        "SSC_CHECKTIMER=1",
        "USE_ECAT_TIMER=1",

    ],
};

const cflags_r5f_1_0 = {
    common: [

        "-Wno-unused-but-set-variable",

    ],
    debug: [

        "-O0",

    ],
};

const lflags_r5f_1_0 = {
    common: [

        "--use_memcpy=fast",
        "--use_memset=fast",

    ],
};
const lnkfiles_r5f = {
    common: [
        "linker.cmd",
    ],
};


const syscfgfile = "../example.syscfg";

const readmeDoxygenPageTag = "EXAMPLE_TIDEP_01032_REFERENCE_DESIGN";

const templates_r5f_0_0 =
[
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "single_chip_servo_main",
        },
    }
];

const templates_r5f_0_1 =
[
    {
        input: ".project/templates/am243x/nortos/main_nortos.c.xdt",
        output: "../main.c",
        options: {
            entryFunction: "single_chip_servo_main",
        },
    }
];

const templates_r5f_1_0 =
[
];

const buildOptionCombos = [
    { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp",  os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-lp",  os: "nortos", isPartOfSystemProject: true},
    { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-lp",  os: "freertos", isPartOfSystemProject: true},
];

const systemProjects = [
    {
        name: "single_chip_servo",
        tag: "freertos_nortos",
        skipProjectSpec: false,
        readmeDoxygenPageTag: readmeDoxygenPageTag,
        board: "am243x-lp",
        projects: [
            { device: device, cpu: "r5fss0-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos"},
            { device: device, cpu: "r5fss0-1", cgt: "ti-arm-clang", board: "am243x-lp", os: "nortos"},
            { device: device, cpu: "r5fss1-0", cgt: "ti-arm-clang", board: "am243x-lp", os: "freertos"},
        ],
    },
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.name = "single_chip_servo";
    property.isInternal = false;
    property.tirexResourceSubClass = [ "example.gettingstarted" ];
    property.description = "EtherCAT connected dual servo motor drive example"
    property.buildOptionCombos = buildOptionCombos;

    return property;
};

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    /* The following build property has been modified for motor drive usage,
     specifically - added different includes vars distinguishing no/free rtos
                    added different lnkfiles vars distinguishing cpu-specific lnkfiles
    */
    build_property.syscfgfile = syscfgfile;
    build_property.lnkfiles = lnkfiles_r5f;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;

    if(buildOption.cpu.match(/r5fss0-0/)) {
        build_property.files = files_r5f_0_0;
        build_property.filedirs = filedirs_r5f_0_0;
        build_property.projectspec_files = projectspec_files_r5f_0_0;
        build_property.includes = includes_r5f_0_0;
        build_property.libs = libs_r5f_0_0;
        build_property.libdirs = libdirs_r5f_0_0;
        build_property.templates = templates_r5f_0_0;
    }

    if(buildOption.cpu.match(/r5fss0-1/)) {
        build_property.files = files_r5f_0_1;
        build_property.filedirs = filedirs_r5f_0_1;
        build_property.projectspec_files = projectspec_files_r5f_0_1;
        build_property.includes = includes_r5f_0_1;
        build_property.libs = libs_r5f_0_1;
        build_property.libdirs = libdirs_r5f_0_1;
        build_property.templates = templates_r5f_0_1;
    }

    if(buildOption.cpu.match(/r5fss1-0/)) {
        build_property.files = files_r5f_1_0;
        build_property.filedirs = filedirs_r5f_1_0;
        build_property.projectspec_files = projectspec_files_r5f_1_0;
        build_property.includes = includes_r5f_1_0;
        build_property.libs = libs_r5f_1_0;
        build_property.libdirs = libdirs_r5f_1_0;
        build_property.templates = templates_r5f_1_0;
        build_property.cflags = cflags_r5f_1_0;
        build_property.lflags = lflags_r5f_1_0;
        build_property.defines = defines_r5f_1_0;
    }

    return build_property;
};

function getSystemProjects(device)
{
    return systemProjects;
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
    getSystemProjects,
};

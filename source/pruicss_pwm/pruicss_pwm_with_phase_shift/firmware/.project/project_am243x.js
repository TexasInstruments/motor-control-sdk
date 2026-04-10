let path = require('path');

let device = "am243x";

const files = {
    common: [
        "main.asm",
        "linker.cmd"
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <source_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* source base */
        "."
    ],
};

const includes = {
    common: [
        "${MCU_PLUS_SDK_PATH}/source/pru_io/firmware/common",
    ],
};

const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};

const lflags = {
    common: [
        "--entry_point=main",
        "--diag_suppress=10063-D", /* Added to suppress entry_point related warning */
    ],
};

const readmeDoxygenPageTag = "EXAMPLE_PRUICSS_PWM_WITH_PHASE_SHIFT";


const templates_pru =
[
    {
        input: ".project/templates/am243x/common/pru/linker_pru0.cmd.xdt",
        output: "linker.cmd",
    }
];

const buildOptionCombos = [
    { device: device, cpu: "icss_g0_pru0", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icss_g1_pru0", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
];

function getmakefilePruPostBuildSteps(cpu, board)
{
    let core = "PRU0"

    switch(cpu)
    {
        case "icss_g0_pru0":
            core = "PRU0_G0"
            break;
        case "icss_g1_pru0":
            core = "PRU0_G1"
    }
    return  [
        "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix="+ core + "_Firmware  -o "+ core.toLocaleLowerCase() + "_load_bin.h " + "pruicss_pwm_with_phase_shift_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
        "$(CAT) ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h "+ core.toLocaleLowerCase() + "_load_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/pruicss_pwm/pruicss_pwm_with_phase_shift/firmware/"+ board + "/" +core.toLocaleLowerCase() + "_load_bin.h ;"+ 
        "$(RM) "+ core.toLocaleLowerCase() + "_load_bin.h;"
    ];
}

function getccsPruPostBuildSteps(cpu, board)
{
    let core = "PRU0"

    switch(cpu)
    {
        case "icss_g0_pru0":
            core = "PRU0_G0"
            break;
        case "icss_g1_pru0":
            core = "PRU0_G1"
    }
    return  [
        "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix="+ core + "_Firmware  -o "+ core.toLocaleLowerCase() + "_load_bin.h " + "pruicss_pwm_with_phase_shift_" + board + "_" + cpu + "_fw_ti-pru-cgt.out;"+ 
        "if ${CCS_HOST_OS} == linux cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h "+ core.toLocaleLowerCase() + "_load_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/pruicss_pwm/pruicss_pwm_with_phase_shift/firmware/"+ board + "/" +core.toLocaleLowerCase() + "_load_bin.h ;"+ 
        "if ${CCS_HOST_OS} == linux rm "+ core.toLocaleLowerCase() + "_load_bin.h;"+
        "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/cat ${MOTOR_CONTROL_SDK_PATH}/source/pru_ti_text_file_license_copyright.h "+ core.toLocaleLowerCase() + "_load_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/pruicss_pwm/pruicss_pwm_with_phase_shift/firmware/"+ board + "/" +core.toLocaleLowerCase() + "_load_bin.h ;"+ 
        "if ${CCS_HOST_OS} == win32  $(CCS_INSTALL_DIR)/utils/cygwin/rm "+ core.toLocaleLowerCase() + "_load_bin.h;"
    ];
}

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "pruicss_pwm_with_phase_shift";
    property.isInternal = false;
    property.description = "PRU ICSS PWM with phase shift PRU Project"
    property.buildOptionCombos = buildOptionCombos;
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.lnkfiles = lnkfiles;
    build_property.includes = includes;
    build_property.lflags = lflags;
    build_property.templates = templates_pru;
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

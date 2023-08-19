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
        "../../..", /* Example base */
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
        "--define=SDFM_PRU_CORE=1",
        "--warn_sections",
        "--entry_point=SDFM_ENTRY",
        "--zero_init=off",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru0", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},

];



let postBuildStepsPru = [
   "$(CG_TOOL_ROOT)/bin/hexpru.exe --diag_wrap=off --array --array:name_prefix=pru_SDFM_PRU0_image -o sdfm_firmware_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.h sdfm_firmware_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.out;  move sdfm_firmware_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.h ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/sdfm_bin.h;"
];



function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "sdfm_firmware";
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
    build_property.postBuildSteps = postBuildStepsPru;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
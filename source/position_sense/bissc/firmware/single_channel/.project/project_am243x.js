let path = require('path');

let device = "am243x";

const files = {
    common: [
        "bissc_main.asm",
        "bissc_diagnostic.cmd",
        "bissc_master_hexpru.cmd"
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../..", /* Example base */
        "../../../..",
    ],
};

const includes = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/bissc/firmware",
    ],
};

const defines = {
    common: [

    ],
};
/*const lnkfiles = {
    common: [
        "linker.cmd",
    ]
};*/


const readmeDoxygenPageTag = "BISSC_DESIGN";

const cflags = {
    common: [
        "-v4"
    ],
};

const lflags = {
    common: [
        "--entry_point=BISSC_INIT",
        "--disable_auto_rts",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
];

let postBuildSteps = [
    "$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=BiSSFirmware -o bissc_master_bin.h bissc_peripheral_interface_single_ch_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.out; move bissc_master_bin.h ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/bissc/firmware/bissc_master_bin.h;"

];
function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "bissc_peripheral_interface_single_ch";
    property.isInternal = false;
    property.description = "BiSSC Peripheral Interface"
    property.buildOptionCombos = buildOptionCombos;
    property.pru_main_file = "main";
    property.pru_linker_file = "linker";
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;
    property.postBuildSteps = postBuildSteps;
    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.defines = defines;
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

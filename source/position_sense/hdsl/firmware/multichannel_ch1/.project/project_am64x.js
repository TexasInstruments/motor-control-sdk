let path = require('path');

let device = "am64x";

const files = {
    common: [
        "main.asm",
        "datalink.asm",
        "datalink_init.asm",
        "transport.asm",
        "utils.asm",
        "hdsl_master_icssg.cmd",
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
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/hdsl/firmware",
    ],
};

const defines = {
    common: [
        "PRU1",
        "CHANNEL_1",
        "ICSS_G_V_1_0",
        "FREERUN_300_MHZ",
        "HDSL_MULTICHANNEL",
    ],
};

const lflags = {
    common: [
        "--disable_auto_rts",
        "--entry_point=main",
    ],
};

let postBuildSteps = [
    "$(CG_TOOL_ROOT)/bin/hexpru.exe --diag_wrap=off --array --array:name_prefix=Hiperface_DSL2_0_PRU -o hdsl_master_icssg_multichannel_ch1_bin.h hdsl_master_multichannel_ch1_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out;  $(COPY)  hdsl_master_icssg_multichannel_ch1_bin.h  ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/hdsl/firmware/hdsl_master_icssg_multichannel_ch1_bin.h"
];


const readmeDoxygenPageTag = "HDSL_DESIGN";

const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru1", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "hdsl_master_multichannel_ch1";
    property.description = "HDSL Master Free Run Mode Firmware for PRU-ICSS running at 300 MHz";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;
    property.pru_main_file = "main";
    property.pru_linker_file = "hdsl_master_icssg";
    property.isSkipTopLevelBuild = true;
    property.skipUpdatingTirex = true;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.defines = defines;
    build_property.lflags = lflags;
    build_property.readmeDoxygenPageTag = readmeDoxygenPageTag;
    build_property.postBuildSteps = postBuildSteps;
    build_property.projecspecFileAction = "copy";
    build_property.skipMakefileCcsBootimageGen = true;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};

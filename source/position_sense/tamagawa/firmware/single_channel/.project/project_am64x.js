let path = require('path');

let device = "am64x";

const files = {
    common: [
        "tamagawa_main.asm",
        "tamagawa_diagnostic.cmd",
        "tamagawa_master_hexpru.cmd",
    ],
};

const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../../../..", /* Example base */
    ],
};

const includes = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware",
    ],
};

const readmeDoxygenPageTag = "TAMAGAWA_DESIGN";

const cflags = {
    common: [
        "-v4"
    ],
};

const lflags = {
    common: [
        "--entry_point=TAMAGAWA_INIT",
        "--disable_auto_rts",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru1", cgt: "ti-pru-cgt", board: "am64x-evm", os: "fw"},
];

let postBuildSteps = [
    "$(CG_TOOL_ROOT)/bin/hexpru.exe ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/tamagawa_master_hexpru.cmd tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out; ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/tools/bin2header/bin2header.exe tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.b00 tamagawa_master_single_channel_bin.h TamagawaFirmware 4; $(COPY) tamagawa_master_single_channel_bin.h ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/tamagawa_master_single_channel_bin.h ;"
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "tamagawa_single_channel";
    property.isInternal = false;
    property.description = "Tamagawa Peripheral Interface"
    property.buildOptionCombos = buildOptionCombos;
    property.pru_main_file = "tamagawa_main";
    property.pru_linker_file = "tamagawa_diagnostic";
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

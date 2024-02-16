let path = require('path');

let device = "am243x";

const files = {
    common: [
        "endat_main.asm",
        "endat_diagnostic.cmd",
        "endat_master_hexpru.cmd"
    ],
};

/* Relative to where the makefile will be generated
 * Typically at <example_folder>/<BOARD>/<core_os_combo>/<compiler>
 */
const filedirs = {
    common: [
        "..",       /* core_os_combo base */
        "../..", /* Example base */
        "../../../..",
    ],
};

const includes = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware",
    ],
};

const defines_rtu = {
    common: [
        "ENABLE_MULTI_MAKE_RTU",

    ],

};
const defines_pru = {
    common: [

        "ENABLE_MULTI_MAKE_PRU",

    ],

};
const defines_txpru = {
    common: [
        "ENABLE_MULTI_MAKE_TXPRU",
    ],

};



const readmeDoxygenPageTag = "ENDAT_DESIGN";

const cflags = {
    common: [
        "-v4"
    ],
};

const lflags = {
    common: [
        "--entry_point=ENDAT_INIT",
        "--disable_auto_rts",
    ],
};


const buildOptionCombos = [
    { device: device, cpu: "icssg0-pru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icssg0-rtupru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
    { device: device, cpu: "icssg0-txpru1", cgt: "ti-pru-cgt", board: "am243x-evm", os: "fw"},
];

let postBuildStepsPru1 = [
    "$(CG_TOOL_ROOT)/bin/hexpru.exe --diag_wrap=off --array --array:name_prefix=EnDatFirmwareMultiMakePRU -o endat_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.h endat_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.out;  $(COPY) endat_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.h ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware/endat_master_multi_PRU_bin.h"

];
let postBuildStepsRtupru1 = [
   "$(CG_TOOL_ROOT)/bin/hexpru.exe --diag_wrap=off --array --array:name_prefix=EnDatFirmwareMultiMakeRTU -o endat_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.h endat_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.out;  $(COPY) endat_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.h ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware/endat_master_multi_RTU_bin.h"

];
let postBuildStepsTxpru1 = [
    "$(CG_TOOL_ROOT)/bin/hexpru.exe --diag_wrap=off --array --array:name_prefix=EnDatFirmwareMultiMakeTXPRU -o endat_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-txpru1_fw_ti-pru-cgt.h endat_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-txpru1_fw_ti-pru-cgt.out;  $(COPY) endat_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-txpru1_fw_ti-pru-cgt.h ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware/endat_master_multi_TXPRU_bin.h"

];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "executable";
    property.makefile = "pru";
    property.name = "endat_peripheral_interface_multi_ch_load_share";
    property.isInternal = false;
    property.description = "Endat Multi channel Interface for Different Make Encoders"
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
    if(buildOption.cpu.match("icssg0-pru1"))
    {
        build_property.defines = defines_pru;
        build_property.postBuildSteps = postBuildStepsPru1;
    }
    if(buildOption.cpu.match("icssg0-rtupru1"))
    {
        build_property.defines = defines_rtu;
        build_property.postBuildSteps = postBuildStepsRtupru1;
    }
    if(buildOption.cpu.match("icssg0-txpru1"))
    {
        build_property.defines = defines_txpru;
        build_property.postBuildSteps = postBuildStepsTxpru1;
    }

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

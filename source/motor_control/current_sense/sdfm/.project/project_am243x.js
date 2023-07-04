let path = require('path');

let device = "am243x";

const files = {
    common: [
        "sdfm_drv.c",
    ],
};

const filedirs = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/motor_control/current_sense/sdfm/driver",
    ],
};

const includes = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/motor_control/current_sense/sdfm/include",
        "${MOTOR_CONTROL_SDK_PATH}/source/motor_control/current_sense/sdfm/firmware",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "motorcontrol_sdfm";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};

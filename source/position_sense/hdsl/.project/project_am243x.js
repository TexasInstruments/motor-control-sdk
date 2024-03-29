let path = require('path');

let device = "am243x";

const files = {
    common: [
        "hdsl_drv.c",
        "hdsl_lut.c",
    ],
};

const filedirs = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/hdsl/driver",
    ],
};

const includes = {
    common: [  
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/hdsl/include",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "motorcontrol_hdsl";
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

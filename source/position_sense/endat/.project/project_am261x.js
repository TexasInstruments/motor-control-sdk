let path = require('path');

let device = "am261x";

const files = {
    common: [
        "endat_drv.c",
    ],
};

const filedirs = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/driver",
    ],
};

const includes = {
    common: [
        "${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/include",
    ],
};

const buildOptionCombos = [
    { device: device, cpu: "r5f", cgt: "ti-arm-clang"},
];

function getComponentProperty() {
    let property = {};

    property.dirPath = path.resolve(__dirname, "..");
    property.type = "library";
    property.name = "motorcontrol_endat";
    property.isInternal = false;
    property.buildOptionCombos = buildOptionCombos;

    return property;
}

function getComponentBuildProperty(buildOption) {
    let build_property = {};

    build_property.files = files;
    build_property.filedirs = filedirs;
    build_property.includes = includes;
    build_property.projecspecFileAction = "link";

    return build_property;
}

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};

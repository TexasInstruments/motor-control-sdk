function getComponentProperty(device)
{
    return require(`./mcsdk_project_${device}`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./mcsdk_project_${buildOption.device}`).getComponentBuildProperty(buildOption);
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};

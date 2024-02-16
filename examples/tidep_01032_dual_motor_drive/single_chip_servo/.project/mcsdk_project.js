function getComponentProperty(device)
{
    return require(`./mcsdk_project_${device}`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./mcsdk_project_${buildOption.device}`).getComponentBuildProperty(buildOption);
};

function getSystemProjects(device)
{
    return require(`./mcsdk_project_${device}`).getSystemProjects(device);
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
    getSystemProjects,
};

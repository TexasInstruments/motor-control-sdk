const common = require(`./common.js`);
const path = require(`path`);
const _ = require('lodash');
const fs = require('fs');
const versions = require('./toolChainVersions.js');

const utils = {

    capitalize: (s) => {
        if (typeof s !== 'string') return ''
        return s.charAt(0).toUpperCase() + s.slice(1)
    },
    replace: (s, a, b) => {
        return s.replace(a, b);
    },
    getDeviceFamilyProjectSpec: (cpu) => {
        let cpuFamily = 'ARM';

        if (cpu.includes('r5f') == true) {
            cpuFamily = 'ARM';
        }
        if (cpu.includes('c66') == true) {
            cpuFamily = 'C6000';
        }
        if (cpu.includes('pru') == true) {
            cpuFamily = 'PRU';
        }
        return cpuFamily;
    },
    getOutputTypeProjectSpec: (type) => {
        let outputType = 'executable';

        if(type == "executable")
            outputType = 'executable';
        if(type == "library")
            outputType = 'staticLibrary';
        return outputType;
    },
    getSysCfgDeviceProjectSpec: (device, board) => {
        return require(`./device/project_${device}`).getSysCfgDevice(board);
    },
    getSysCfgCpuProjectSpec: (cpu) => {
        return require(`./device/project_${device}`).getSysCfgCpu(cpu);
    },
    getProjectSpecDevice: (device) => {
        return require(`./device/project_${device}`).getProjectSpecDevice(device);
    },
    getProjectSpecCpu: (device, cpu) => {
        return require(`./device/project_${device}`).getProjectSpecCpu(cpu);
    },
    getDeviceIdProjectSpec: (device, cpu, board) => {
        let deviceName = '';
        let cpuName = '';

        deviceName = require(`./device/project_${device}`).getProjectSpecDevice(board);

        if (cpu.includes('r5f') == true) {
            cpuName = 'Cortex R';
        }

        if (cpu.includes('a53') == true) {
            cpuName = 'Cortex A';
        }

        if (cpu.includes('m4f') == true) {
            cpuName = 'Cortex M';
        }

        if (cpu.includes('c66') == true) {
            cpuName = 'TMS320C66XX';
        }

        if (cpu.includes('pru') == true) {
            if (board == "am64x-evm") {
                return "AM64x_GP_EVM";      // check for other devices
            } else if (board == "am243x-evm") {
                return "AM243x_GP_EVM";
            } else if (board == "am243x-lp") {
                return "AM243x_LAUNCHPAD";
            } else if (board == "am263x-cc") {
                return "AM263x_CC";
            } else if (board == "am263x-lp") {
                return "AM263x_LAUNCHPAD";
            }else if (board == "am263px-cc") {
                return "AM263px";
            } else if (board == "am263px-lp") {
                return "AM263px";
            } else if (board == "am261x-lp"){
                return "AM261x";
            }else if (board == "am261x-som"){
                return "AM261x";
            }
        }

        return cpuName + '.' + deviceName;
    },
    getFileListProjectSpec: (files, filedirs, projectabspath) => {

        let filelist = [];

        for (prop in files) {
            if (files.hasOwnProperty(prop) && Array.isArray(files[prop])) {
                for (let file of files[prop]) {
                    let foundFile = false;
                    for (filedir of filedirs[prop]) {
                        let checkPath = path.normalize(projectabspath + '/' + filedir + '/' + file);
                        if (fs.existsSync(checkPath) == true) {
                            filelist.push(filedir + '/' + file);
                            foundFile = true;
                        }
                    }
                    if(foundFile == false) {
                        console.log(`ERROR : Couldn't find ${file} in given source directories for ${projectabspath} ...`)
                    }
                }
            }
        }
        return filelist;
    },

    getToolChainProjectSpec: (cgt) => {
        let toolchain = ''

        switch(cgt) {
            case 'ti-arm-clang':
                toolchain = 'TICLANG'
                break;
            case 'ti-arm-cgt':
                toolchain = 'TI'
                break;
            case 'gcc-armv7':
                toolchain = 'GNU'
                break;
            case 'gcc-aarch64':
                toolchain = 'GNU'
                break;
            case 'ti-pru-cgt':
                toolchain = 'TI'
                break;
            default:
                toolchain = 'TI'
        }

        return toolchain;
    },

    getProductNameProjectSpec: (device) => {

        if(common.isDevelopmentMode())
            return "MOTOR_CONTROL_SDK_AMXXX";

        return require(`./device/project_${device}`).getProductNameProjectSpec();
    },

    getIcsdkProductNameProjectSpec: (device) => {

        if(common.isDevelopmentMode())
            return "INDUSTRIAL_COMMUNICATIONS_SDK_AMXXX";

        return require(`./device/project_${device}`).getIcsdkProductNameProjectSpec();
    },

    getMcusdkProductNameProjectSpec: (device) => {

        if(common.isDevelopmentMode())
            return "MCU_PLUS_SDK_AMXXX";

        return require(`./device/project_${device}`).getMcusdkProductNameProjectSpec();
    },

    getSdkVersionProjectSpec: (device) => {

        if(common.isDevelopmentMode())
            return versions.sdkVersions.default.version;

        return versions.sdkVersions[device].version || versions.sdkVersions.default.version;
    },

    getIcsdkVersionProjectSpec: (device) => {

        if(common.isDevelopmentMode())
            return versions.icsdkVersions.default.version;

        return versions.icsdkVersions[device].version || versions.icsdkVersions.default.version;
    },

    getMcusdkVersionProjectSpec: (device) => {

        if(common.isDevelopmentMode())
                    return versions.mcusdkVersions.default.version;

        return versions.mcusdkVersions[device].version || versions.mcusdkVersions.default.version;
    },

    /* default action for files in project spec, i.e copy or link */
    getDefaultActionProjectSpec: () => {

        if(common.isDevelopmentMode())
            return "copy"; /* use copy for development mode as well */

        return "copy";
    },
    /**
     * Get the toolchain version based on compiler type and SOC
     *
     * @param {string} cgt - Compiler/toolchain identifier (e.g., 'ti-arm-clang', 'gcc-aarch64')
     * @param {string} device - Device identifier (e.g., 'am64x', 'am243x')
     * @returns {string} Version of the specified toolchain for the given device, falls back to default if not specified
     */
    getToolChainVersionProjectSpec: (cgt, device) => {
        const deviceVersions = versions.toolchain[device] || versions.toolchain.default;
        return deviceVersions[cgt] || '';
    },

    /**
     * Get SysConfig version for specified device
     *
     * @param {string} device - Device identifier (e.g., 'am64x', 'am243x')
     * @returns {string} SysConfig version for the device, falls back to default if not specified
     */
    getSysCfgVersionProjectSpec: (device) => {
        return versions.sysConfig[device] || versions.sysConfig.default;
    },

    /**
     * Get Code Composer Studio (CCS) version for specified device
     *
     * @param {string} device - Device identifier (e.g., 'am64x', 'am243x')
     * @returns {string} CCS version for the device, falls back to default if not specified
     */
    getCCSVersionProjectSpec: (device) => {
        return versions.ccs[device] || versions.ccs.default;
    },

    /**
     * Get TI Clang compiler version for specified device
     *
     * @param {string} device - Device identifier (e.g., 'am64x', 'am243x')
     * @returns {string} TI Clang version for the device, falls back to default if not specified
     */
    getTiClangVersionProjectSpec: (device) => {
        return versions.tiClang[device] || versions.tiClang.default;
    },

    /**
     * Get GCC AArch64 compiler version for specified device
     *
     * @param {string} device - Device identifier (e.g., 'am64x', 'am243x')
     * @returns {string} GCC AArch64 version for the device, falls back to default if not specified
     */
    getGCCAarch64NoneVersionProjectSpec: (device) => {
        return versions.gccAarch64[device] || versions.gccAarch64.default;
    },

    /**
     * Get GCC ARMv7 compiler version for specified device
     *
     * @param {string} device - Device identifier (e.g., 'am64x', 'am243x')
     * @returns {string} GCC ARMv7 version for the device, falls back to default if not specified
     */
    getGCCArmv7NoneVersionProjectSpec: (device) => {
        return versions.gccArmv7[device] || versions.gccArmv7.default;
    },

    getTitleProjectSpec: (name) => {
        let title = name.replace(/_/g, ' ');

        // Title Case : Converts "hello world" to "Hello World" using regex
        return title.replace(
            /\w\S*/g,
            function (s) {
                return s.charAt(0).toUpperCase() + s.substr(1).toLowerCase();
            }
        );
    }
}

function genProjectSpecExample(device) {
    let example_file_list = require(`./device/project_${device}`).getExampleList();

    for(example of example_file_list) {
        let property = require(`../${example}`).getComponentProperty(device);

        if(property.skipProjectSpec)
            continue;

        for(buildOption of property.buildOptionCombos) {
            let commonCgtOptions = require(`./cgt/cgt_${buildOption.cgt}`).getCgtOptions(buildOption.cpu, device);
            let common_build_property = require(`./device/project_${device}`).getProperty();
            let project = [];
            let projectSpecOutPath = common.path.makeExampleOutPath(property.dirPath, buildOption);

            fs.mkdirSync(projectSpecOutPath, { recursive: true });

            buildOption.isProjectSpecBuild = true;
            build_property = require(`../${example}`).getComponentBuildProperty(buildOption);

            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), project.dirPath);
            project = _.merge({}, project, buildOption);
            project = _.merge({}, project, build_property);
            project = common.updateLibsWithOs(project, buildOption.os);
            project = common.mergeCgtOptions(project, commonCgtOptions);
            project = common.mergeCgtOptions(project, common_build_property);
            project = common.addOsDefine(project, buildOption.os);
            project = common.addOsIncludes(project, buildOption.os, buildOption);
            project.dirPath = projectSpecOutPath;

            let args = {
                sdkName: "MOTOR_CONTROL_SDK_PATH",
                dependentIcsdkName: "IND_COMMS_SDK_PATH",
                dependentMcusdkName: "MCU_PLUS_SDK_PATH",
                sdkPath: common.path.relative(projectSpecOutPath, path.normalize(__dirname + "/..")),
                relPath: common.path.relative(project.dirPath, "."),
                project: project,
                utils: utils,
                cgtOptions: require(`./cgt/cgt_${project.cgt}`).getCgtOptions(buildOption.cpu, device),
                linuxFwName: require(`./device/project_${device}`).getLinuxFwName(buildOption.cpu),
                syscfg: {
                    device: require(`./device/project_${device}.js`).getSysCfgDevice(buildOption.board),
                    cpu: require(`./device/project_${device}.js`).getSysCfgCpu(buildOption.cpu),
                    pkg: require(`./device/project_${device}.js`).getSysCfgPkg(buildOption.board),
                    part: require(`./device/project_${device}.js`).getSysCfgPart(buildOption.board),
                },
                flashAddr: require(`./device/project_${device}.js`).getFlashAddr(),
            };

            common.convertTemplateToFile(
                    `.project/templates/projectspec_${project.type}.xdt`,
                    `${project.dirPath}/example.projectspec`,
                    args);
            if("syscfgfile" in args.project) {
                common.convertTemplateToFile(
                        `.project/templates/syscfg_c.rov.xs.xdt`,
                        `${project.dirPath}/syscfg_c.rov.xs`,
                        args);
            }
            if(args.project.skipMakefileCcsBootimageGen) {
                // skip makefile_ccs_bootimage_gen
            }
            else{
                common.convertTemplateToFile(
                    `.project/templates/makefile_ccs_bootimage_gen.xdt`,
                    `${project.dirPath}/makefile_ccs_bootimage_gen`,
                    args);
            }
        }
    }
}

function genProjectSpecsDevice(device) {
    genProjectSpecExample(device);
}

function cleanProjectSpecsDevice(device) {
    let example_file_list = require(`./device/project_${device}`).getExampleList();
    for(example of example_file_list) {
        let property = require(`../${example}`).getComponentProperty(device);

        if(property.skipProjectSpec)
            continue;

        for(buildOption of property.buildOptionCombos) {
            let project = [];
            let projectSpecOutPath = common.path.makeExampleOutPath(property.dirPath, buildOption);

            fs.mkdirSync(projectSpecOutPath, { recursive: true });

            build_property = require(`../${example}`).getComponentBuildProperty(buildOption);

            project = _.merge({}, project, property);
            project.relpath = common.path.relative(path.normalize(__dirname + "/.."), project.dirPath);
            project = _.merge({}, project, buildOption);
            project = _.merge({}, project, build_property);
            project.dirPath = projectSpecOutPath;

            common.deleteFile(`${project.dirPath}/example.projectspec`);
            common.deleteFile(`${project.dirPath}/makefile_ccs_bootimage_gen`);
            common.deleteFile(`${project.dirPath}/syscfg_c.rov.xs`);
        }
    }
}

module.exports = {
    genProjectSpecsDevice,
    cleanProjectSpecsDevice,
    utils,
}

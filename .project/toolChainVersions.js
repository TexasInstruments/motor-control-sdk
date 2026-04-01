/**
 * This file contains version information for various tools and SDKs used in the project.
 * All version configurations are centralized here for easier maintenance.
 */

const versions = {
    /**
     * SysConfig versions for different devices
     * These versions are used for system configuration tools
     *
     * Refer to the genProjectSpec.js file in the Motor Control SDK for the version
     */
    sysConfig: {
        default: "1.23.0",
        am64x: "1.23.0",
        am243x: "1.23.0",
        am263px: "1.25.0",
        am263x: "1.25.0",
        am261x: "1.25.0"
    },

    /**
     * Code Composer Studio (CCS) versions for different devices
     * Specifies the CCS version to be used for each device
     *
     * Refer to the genProjectSpec.js file in the Motor Control SDK for the version
     */
    ccs: {
        default: "2020",
        am64x: "2020",
        am243x: "2020",
        am263px: "2030",
        am263x: "2030",
        am261x: "2030"
    },

    /**
     * TI ARM Clang compiler versions for different devices
     * Device-specific compiler versions for TI Clang
     *
     * Refer to the genProjectSpec.js file in the Motor Control SDK for the version
     */
    tiClang: {
        default: "4.0.1",
        am64x: "4.0.1",
        am243x: "4.0.1",
        am263px: "4.0.3",
        am263x: "4.0.3",
        am261x: "4.0.3"
    },

    /**
     * GCC AArch64 compiler versions for different devices
     * Used for 64-bit ARM compilation
     *
     * Refer to the genProjectSpec.js file in the Motor Control SDK for the version
     */
    gccAarch64: {
        default: "9.2.1",
        am64x: "9.2.1",
        am243x: "9.2.1",
        am263px: "9.2.1",
        am263x: "9.2.1",
        am261x: "9.2.1"
    },

    /**
     * GCC ARMv7 compiler versions for different devices
     * Used for 32-bit ARM compilation
     *
     * Refer to the genProjectSpec.js file in the Motor Control SDK for the version
     */
    gccArmv7: {
        default: "10",
        am64x: "10",
        am243x: "10",
        am263px: "10",
        am263x: "10",
        am261x: "10"
    },

    /**
     * Generic toolchain versions
     * These versions are used in the getToolChainVersionProjectSpec function
     * Key is the toolchain identifier and value is its version
     *
     * Refer to the genProjectSpec.js file in the Motor Control SDK for the version
     */
    toolchain: {
        default: {
            'ti-arm-clang': "4.0.1",   // TI ARM Clang Compiler
            'gcc-aarch64': "9.2",      // GCC for AArch64
            'gcc-armv7': "7.2",        // GCC for ARMv7
            'ti-c6000': "8.3.12",      // TI C6000 Compiler
            'ti-pru-cgt': "2.3.3"      // TI PRU-CGT Compiler
        },
        am64x: {
            'ti-arm-clang': "4.0.1",   // TI ARM Clang Compiler
            'gcc-aarch64': "9.2",      // GCC for AArch64
            'gcc-armv7': "7.2",        // GCC for ARMv7
            'ti-c6000': "8.3.12",      // TI C6000 Compiler
            'ti-pru-cgt': "2.3.3"      // TI PRU-CGT Compiler
        },
        am243x: {
            'ti-arm-clang': "4.0.1",   // TI ARM Clang Compiler
            'gcc-aarch64': "9.2",      // GCC for AArch64
            'gcc-armv7': "7.2",        // GCC for ARMv7
            'ti-c6000': "8.3.12",      // TI C6000 Compiler
            'ti-pru-cgt': "2.3.3"      // TI PRU-CGT Compiler
        },
        am263px: {
            'ti-arm-clang': "4.0.3",   // TI ARM Clang Compiler
            'gcc-aarch64': "9.2",      // GCC for AArch64
            'gcc-armv7': "7.2",        // GCC for ARMv7
            'ti-c6000': "8.3.12",      // TI C6000 Compiler
            'ti-pru-cgt': "2.3.3"      // TI PRU-CGT Compiler
        },
        am263x: {
            'ti-arm-clang': "4.0.3",   // TI ARM Clang Compiler
            'gcc-aarch64': "9.2",      // GCC for AArch64
            'gcc-armv7': "7.2",        // GCC for ARMv7
            'ti-c6000': "8.3.12",      // TI C6000 Compiler
            'ti-pru-cgt': "2.3.3"      // TI PRU-CGT Compiler
        },
        am261x: {
            'ti-arm-clang': "4.0.3",   // TI ARM Clang Compiler
            'gcc-aarch64': "9.2",      // GCC for AArch64
            'gcc-armv7': "7.2",        // GCC for ARMv7
            'ti-c6000': "8.3.12",      // TI C6000 Compiler
            'ti-pru-cgt': "2.3.3"      // TI PRU-CGT Compiler
        }
    },

    /**
     * Tool versions and paths configuration specific to each device
     * These versions are used in imports.mak.xdt
     * Contains compiler versions, paths and development tools
     *
     * Refer to the genProjectSpec.js file in the Motor Control SDK for the version
     */
    toolVersionsForImportsMakefile: {
        default: {
            ccsVersion: "ccs2020",
            icsdkVersion: "2025_00_00_01",
            mcusdkVersion: "07_03_00_01",
            cgtPruVersion: "ti-cgt-pru_2.3.3",
            cgtArmClangVersion: "ti-cgt-armllvm_4.0.1.LTS",
            cgtC6000Version: "ti-cgt-c6000_8.3.12",
            dsplibVersion: "dsplib_c66x_3_4_0_0",
            sysconfigVersion: "sysconfig_1.23.0",
            gccAarch64WinVersion: "gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf",
            gccArmWinVersion: "gcc-arm-none-eabi-9-2019-q4-major-win32",
            gccAarch64LinuxVersion: "gcc-arm-9.2-2019.12-x86_64-aarch64-none-elf",
            gccArmLinuxVersion: "gcc-arm-none-eabi-9-2019-q4-major"
        },
        am64x: {
            ccsVersion: "ccs2020",
            icsdkVersion: "2025_00_00_08",
            mcusdkVersion: "11_01_00_17",
            cgtPruVersion: "ti-cgt-pru_2.3.3",
            cgtArmClangVersion: "ti-cgt-armllvm_4.0.1.LTS",
            cgtC6000Version: "ti-cgt-c6000_8.3.12",
            dsplibVersion: "dsplib_c66x_3_4_0_0",
            sysconfigVersion: "sysconfig_1.23.0",
            gccAarch64WinVersion: "gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf",
            gccArmWinVersion: "gcc-arm-none-eabi-7-2017-q4-major-win32",
            gccAarch64LinuxVersion: "gcc-arm-9.2-2019.12-x86_64-aarch64-none-elf",
            gccArmLinuxVersion: "gcc-arm-none-eabi-7-2017-q4-major"
        },
        am243x: {
            ccsVersion: "ccs2020",
            icsdkVersion: "2025_00_00_08",
            mcusdkVersion: "11_01_00_19",
            cgtPruVersion: "ti-cgt-pru_2.3.3",
            cgtArmClangVersion: "ti-cgt-armllvm_4.0.1.LTS",
            cgtC6000Version: "ti-cgt-c6000_8.3.12",
            dsplibVersion: "dsplib_c66x_3_4_0_0",
            sysconfigVersion: "sysconfig_1.23.0",
            gccAarch64WinVersion: "gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf",
            gccArmWinVersion: "gcc-arm-none-eabi-7-2017-q4-major-win32",
            gccAarch64LinuxVersion: "gcc-arm-9.2-2019.12-x86_64-aarch64-none-elf",
            gccArmLinuxVersion: "gcc-arm-none-eabi-7-2017-q4-major"
        },
        am263px: {
            ccsVersion: "ccs2030",
            icsdkVersion: "2025_00_00_08",
            mcusdkVersion: "11_00_00_19",
            cgtPruVersion: "ti-cgt-pru_2.3.3",
            cgtArmClangVersion: "ti-cgt-armllvm_4.0.3.LTS",
            cgtC6000Version: "ti-cgt-c6000_8.3.12",
            dsplibVersion: "dsplib_c66x_3_4_0_0",
            sysconfigVersion: "sysconfig_1.25.0",
            gccAarch64WinVersion: "gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf",
            gccArmWinVersion: "gcc-arm-none-eabi-9-2019-q4-major-win32",
            gccAarch64LinuxVersion: "gcc-arm-9.2-2019.12-x86_64-aarch64-none-elf",
            gccArmLinuxVersion: "gcc-arm-none-eabi-9-2019-q4-major"
        },
        am263x: {
            ccsVersion: "ccs2030",
            icsdkVersion: "2025_00_00_08",
            mcusdkVersion: "11_00_00_19",
            cgtPruVersion: "ti-cgt-pru_2.3.3",
            cgtArmClangVersion: "ti-cgt-armllvm_4.0.3.LTS",
            cgtC6000Version: "ti-cgt-c6000_8.3.12",
            dsplibVersion: "dsplib_c66x_3_4_0_0",
            sysconfigVersion: "sysconfig_1.25.0",
            gccAarch64WinVersion: "gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf",
            gccArmWinVersion: "gcc-arm-none-eabi-9-2019-q4-major-win32",
            gccAarch64LinuxVersion: "gcc-arm-9.2-2019.12-x86_64-aarch64-none-elf",
            gccArmLinuxVersion: "gcc-arm-none-eabi-9-2019-q4-major"
        },
        am261x: {
            ccsVersion: "ccs2030",
            icsdkVersion: "2025_00_00_08",
            mcusdkVersion: "11_00_00_29",
            cgtPruVersion: "ti-cgt-pru_2.3.3",
            cgtArmClangVersion: "ti-cgt-armllvm_4.0.3.LTS",
            cgtC6000Version: "ti-cgt-c6000_8.3.12",
            dsplibVersion: "dsplib_c66x_3_4_0_0",
            sysconfigVersion: "sysconfig_1.25.0",
            gccAarch64WinVersion: "gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf",
            gccArmWinVersion: "gcc-arm-none-eabi-9-2019-q4-major-win32",
            gccAarch64LinuxVersion: "gcc-arm-9.2-2019.12-x86_64-aarch64-none-elf",
            gccArmLinuxVersion: "gcc-arm-none-eabi-9-2019-q4-major"
        }
    },

    /**
     * SDK versions for different devices
     * These versions are referenced in package.tirex.json.xdt and product.json.xdt
     */
    sdkVersions: {
        default: {
            version: "2025.00.00.01"
        },
        am64x: {
            version: "2025.00.00.06"
        },
        am243x: {
            version: "2025.00.00.06"
        },
        am263px: {
            version: "2025.00.00.06"
        },
        am263x: {
            version: "2025.00.00.06"
        },
        am261x: {
            version: "2025.00.00.06"
        }
    },

     /**
     * MCU+ SDK versions for different devices
     */
    mcusdkVersions: {
        default: {
            version: "07.03.00.01"
        },
        am64x: {
            version: "11.01.00.17"
        },
        am243x: {
            version: "11.01.00.19"
        },
        am263px: {
            version: "11.00.00.19"
        },
        am263x: {
            version: "11.00.00.19"
        },
        am261x: {
            version: "11.00.00.29"
        }
    },

    /**
     * ICSDK versions for different devices
     */
    icsdkVersions: {
        default: {
            version: "2025.00.00.01"
        },
        am64x: {
            version: "2025.00.00.08"
        },
        am243x: {
            version: "2025.00.00.08"
        },
        am263px: {
            version: "2025.00.00.08"
        },
        am263x: {
            version: "2025.00.00.08"
        },
        am261x: {
            version: "2025.00.00.08"
        }
    },
};

/**
 * Export the versions object for use in other modules
 * This centralized approach ensures consistency across the project
 */
module.exports = versions;

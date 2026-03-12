# Migration Guide {#MIGRATION_GUIDES}

## Migration Details for Motor Control SDK 2025.00.00 {#MIGRATION_SECTION_2025_00}

Following pages contain details on changes which can affect migration of applications based on Motor Control SDK 9.x/10.x/11.x to Motor Control SDK 2025.00.00.

- \subpage BISSC_MIGRATION_GUIDE_2025_00
- \subpage NIKON_MIGRATION_GUIDE_2025_00
- \subpage ENDAT_MIGRATION_GUIDE_2025_00
- \subpage TAMAGAWA_MIGRATION_GUIDE_2025_00
- \subpage SDFM_MIGRATION_GUIDE_2025_00

## Migration Details for Motor Control SDK 9.x/10.x/11.x
\cond SOC_AM64X || SOC_AM243X

- \ref UPGRADE_AND_COMPATIBILITY_INFORMATION_11_0_0 has details on changes which can affect migration of applications based on Motor Control SDK 09.02.00 to Motor Control SDK 11.00.00.
- \ref UPGRADE_AND_COMPATIBILITY_INFORMATION_9_2_0 has details on changes which can affect migration of applications based on Motor Control SDK 09.01.00 to Motor Control SDK 09.02.00.
- \ref UPGRADE_AND_COMPATIBILITY_INFORMATION_9_1_0 has details on changes which can affect migration of applications based on Motor Control SDK 09.00.00 to Motor Control SDK 09.01.00.
- \ref UPGRADE_AND_COMPATIBILITY_INFORMATION_9_0_0 has details on changes which can affect migration of applications based on MCU+ SDK 08.06.00 to Motor Control SDK 09.00.00.

\endcond

\cond SOC_AM263PX

- \ref UPGRADE_AND_COMPATIBILITY_INFORMATION_10_02_00 has details on changes which can affect migration of applications based on Motor Control SDK 10.00.00 to Motor Control SDK 10.02.00.

\endcond

\cond SOC_AM261X

- \ref UPGRADE_AND_COMPATIBILITY_INFORMATION_10_02_00 has details on changes which can affect migration of applications based on Motor Control SDK 10.00.01 to Motor Control SDK 10.02.00.

\endcond

## Migration from MCU+ SDK 

Motor Control SDK 9.x/10.x/11.x includes <a href="@VAR_IC_SDK_DOCS_PATH/index.html" target="_blank">@VAR_SOC_NAME Industrial Communications SDK</a> under the `ind_comms_sdk` folder and <a href="@VAR_MCU_SDK_DOCS_PATH/index.html" target="_blank">@VAR_SOC_NAME MCU+ SDK</a> under the `mcu_plus_sdk` folder.

\cond (SOC_AM64X || SOC_AM243X)
Components under Motor Control SDK were available in MCU+ SDK 7.x and 8.x releases.
\endcond

\cond (SOC_AM263X)
Components under Motor Control SDK were available in MCU+ SDK 8.x releases.
\endcond

If you are a user of MCU+ SDK, the items listed on this page will assist you in migrating to Motor Control SDK.

- To build examples from Industrial Communications SDK and MCU+ SDK using CCS projects, users have to add <b>${SDK_INSTALL_PATH}/ind_comms_sdk</b> path and <b>${SDK_INSTALL_PATH}/mcu_plus_sdk</b> path to "Product discovery path" respectively in CCS (from Window->Preferences->Code Composer Studio->Products).

\cond SOC_AM64X || SOC_AM243X

- The examples, drivers and PRU-ICSS firmwares for position sense encoders and current sense %SDFM (using PRU-ICSS) are moved from MCU+ SDK to Motor Control SDK. Folder location changes are shown below.

   Module                      | MCU+ SDK Folder Location                                                                       | Motor Control SDK Folder Location
   ----------------------------|------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------
   Position Sense EnDat        | `examples/motor_control/endat_diagnostic`                                                      | `examples/position_sense/endat_diagnostic`
   ^                           | `source/motor_control/position_sense/endat`                                                    | `source/position_sense/endat`
   Position Sense HDSL         | `examples/motor_control/hdsl_diagnostic`                                                       | `examples/position_sense/hdsl_diagnostic`
   ^                           | `examples/motor_control/hdsl_diagnostic_with_traces`                                           | Merged with `hdsl_diagnostic` example
   ^                           | `source/motor_control/position_sense/hdsl`                                                     | `source/position_sense/hdsl`
   Position Sense Tamagawa     | `examples/motor_control/tamagawa_diagnostic`                                                   | `examples/position_sense/tamagawa_diagnostic`
   ^                           | `source/motor_control/position_sense/tamagawa`                                                 | `source/position_sense/tamagawa`
   Current Sense %SDFM         | `examples/motor_control/icss_sdfm`                                                             | `examples/current_sense/icss_sdfm`
   ^                           | `source/motor_control/current_sense/sdfm`                                                      | `source/current_sense/sdfm`

\endcond

\cond (SOC_AM263X)

- The example and driver for position sense encoder are moved from MCU+ SDK to Motor Control SDK. Folder location changes are shown below.

   Module                      | MCU+ SDK Folder Location                                                                    | Motor Control SDK Folder Location
   ----------------------------|---------------------------------------------------------------------------------------------|-----------------------------------------------------------
   Position Sense Tamagawa     | `examples/motor_control/tamagawa_diagnostic_over_soc_uart`                                  | `examples/position_sense/tamagawa_diagnostic_over_soc_uart`
   ^                           | `source/motor_control/position_sense/tamagawa_over_soc_uart`                                | `source/position_sense/tamagawa_over_soc_uart`

\endcond
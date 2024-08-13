# Migration Guide {#MIGRATION_GUIDES}

\cond SOC_AM64X || SOC_AM243X
Components under Motor Control SDK were available in MCU+ SDK 7.x and 8.x releases.
\endcond

\cond SOC_AM263X
Components under Motor Control SDK were available in MCU+ SDK 8.x releases.
\endcond

If you are a user of MCU+ SDK, then items listed on this page that will assist you in migration to Motor Control SDK.


- To build examples from Industrial Communications SDK and MCU+ SDK using CCS projects, user has to add <b>${SDK_INSTALL_PATH}/ind_comms_sdk</b> path and <b>${SDK_INSTALL_PATH}/mcu_plus_sdk</b> path to "Product discovery path" respectively in CCS (from Window->Preferences->Code Composer Studio->Products). 

\cond SOC_AM64X || SOC_AM243X

- The examples, drivers and PRU-ICSS firmwares for position sense encoders and current sense %SDFM (using PRU-ICSS) are moved from MCU+ SDK to Motor Control SDK. Folder location changes are also shown below.

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

\cond SOC_AM263X

- The example and driver for position sense encoder is moved from MCU+ SDK to Motor Control SDK. Folder location changes are also shown below.

   Module                      | MCU+ SDK Folder Location                                                                    | Motor Control SDK Folder Location
   ----------------------------|---------------------------------------------------------------------------------------------|-----------------------------------------------------------
   Position Sense Tamagawa     | `examples/motor_control/tamagawa_diagnostic_over_soc_uart`                                  | `examples/position_sense/tamagawa_diagnostic_over_soc_uart`
   ^                           | `source/motor_control/position_sense/tamagawa_over_soc_uart`                                | `source/position_sense/tamagawa_over_soc_uart`

\endcond

- Motor Control SDK also includes <a href="@VAR_IC_SDK_DOCS_PATH/index.html" target="_blank">@VAR_SOC_NAME Industrial Communications SDK</a> under `ind_comms_sdk` folder and <a href="@VAR_MCU_SDK_DOCS_PATH/index.html" target="_blank">@VAR_SOC_NAME MCU+ SDK</a> under `mcu_plus_sdk` folder. 

\cond !SOC_AM261X
- \ref UPGRADE_AND_COMPATIBILITY_INFORMATION_10_0_0 has details on changes which can affect migration of applications based on Motor Control SDK 09.02.00 to Motor Control SDK 10.00.00.
- \ref UPGRADE_AND_COMPATIBILITY_INFORMATION_9_2_0 has details on changes which can affect migration of applications based on Motor Control SDK 09.01.00 to Motor Control SDK 09.02.00.
- \ref UPGRADE_AND_COMPATIBILITY_INFORMATION_9_1_0 has details on changes which can affect migration of applications based on Motor Control SDK 09.00.00 to Motor Control SDK 09.01.00.
- \ref UPGRADE_AND_COMPATIBILITY_INFORMATION_9_0_0 has details on changes which can affect migration of applications based on MCU+ SDK 08.06.00 to Motor Control SDK 09.00.00.
\endcond

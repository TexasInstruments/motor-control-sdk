# PRU-ICSS TIME SYNC {#EXAMPLE_PRUICSS_TIME_SYNC}

[TOC]

## Introduction

The Time Synchronization System provides precise clock synchronization between multiple devices using Texas Instruments' PRU-ICSS (Programmable Real-time Unit Industrial Communication Subsystem) hardware. The system implements a trasmitter-receiver synchronization protocol where one device acts as a time trasmitter generating periodic sync signals, while other devices synchronize their local clocks to match the trasmitter's timing.

### System Configurations

#### Time Transmitter_Receiver Configuration
- Dual ICSS instances (ICSS0 and ICSS1)
- ICSS0 generates trasmitter sync signals
- ICSS1 receives and synchronizes to sync signal routed internally

#### Time Receiver Configuration
- Single ICSS instance (ICSS0)
- Receives and synchronizes to external sync signal

### Hardware connections
#### TMDS243EVM
<a href="https://www.ti.com/tool/TMDS64DC01EVM" target="_blank"> An IO Breakout Board </a> is required to probe below outputs
- PRG0_IEP0_EDC_SYNC_OUT0 is available on J4.3
- PRG0_IEP0_EDC_LATCH_IN0 is available on J4.1
- connect PRG0_IEP0_EDC_SYNC_OUT0 of time_sync_time_transmitter_receiver(Borad1) to PRG0_IEP0_EDC_LATCH_IN0 of time_sync_time_receiver(Board2)

\imageStyle{time_sync_hardware_setup.png, width:80%}
\image html time_sync_hardware_setup.png "Hardware connections between Board1 and Board2"

### Supported Combinations

\cond SOC_AM243X

 Parameter            | Value
 -------------------- |-----------
 CPU + OS             | r5fss0-0 freertos
 ^                    | r5fss0-0 nortos
 Toolchain            | ti-arm-clang
 Boards               | @VAR_BOARD_NAME_LOWER
 R5F project (Board1) | examples/time_sync/time_sync_time_transmitter_receiver
 R5F project (Board2) | examples/time_sync/time_sync_time_receiver
 PRU Project folder   | source/pruicss_iep_sync_out_generation

\endcond

### Steps to Run the Example

- **When using CCS projects to build**, import the CCS project from the above mentioned Example folder path for R5F and PRU. After this, `main.asm`, `linker.cmd` files get copied to ccs workspace of PRU project. The `main.asm` contains code to configure compare1 and generate sync out

- Build the PRU project using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
     - Build Flow: Once you click on build in PRU project, firmware header file which is generated in release or debug folder of ccs workspace, is moved to  `<sdk-install-dir/source/pruicss_iep_sync_out_generation/firmware/device/>`

- Build the R5F project using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
     - Firmware header file path is included in R5F project include options by default. Instructions in Firmware header file can be written into PRU IRAM memory using PRUICSS_loadFirmware API call
     - Build Flow: Once you click on build in R5F project, SysConfig files are generated. Finally the R5F project will be generated using both the generated SysConfig and PRU project binaries.

    \note
    Prerequisite: [PRU-CGT-2-3](https://www.ti.com/tool/PRU-CGT) (ti-pru-cgt) should be installed at: `C:/ti/`

- **When using makefiles to build**, note the required combination and build PRU project then R5F project using make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)

- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

### Results
#### Probe below signals after running time_sync_time_transmitter_receiver on Board1 and time_sync_time_receiver on Board2
##### Board 1
- PRG0_IEP0_EDC_SYNC_OUT0 can be probed from IO Breakout board on J4.3
- DEBUG_GPIO(PRG0_PRU0_GPO1) can be probed from IO Breakout board on J2.3
- PRG1_IEP0_EDC_SYNC_OUT0 can be probed from TMDS243EVM on J18.1

##### Board 2
- PRG0_IEP0_EDC_SYNC_OUT0 can be probed from IO Breakout board on J4.3
- DEBUG_GPIO(PRG0_PRU0_GPO1) can be probed from IO Breakout board on J2.3

\imageStyle{time_sync_transmitter_and_receiver.png, width:90%}
\image html time_sync_transmitter_and_receiver.png "Delta of 20ns can be observed from DEVICE1 and DEVICE2 sync output signals"

For more details refer below:
- \subpage TIME_SYNC_DESIGN
- <sdk-install-dir/source/pruicss_iep_sync_out_generation/main.asm>
- <sdk-install-dir/examples/time_sync/time_sync_main.c>
- <sdk-install-dir/examples/time_sync/time_sync.c>
- <sdk-install-dir/examples/time_sync/time_sync.h>
- section 6.4.13 of Technical Reference Manual

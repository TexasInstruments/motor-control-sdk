# PRU-ICSS PWM WITH PHASE SHIFT {#EXAMPLE_PRUICSS_PWM_WITH_PHASE_SHIFT}

[TOC]

# Introduction

This example uses the PRU to control generation of PRUICSS PWM signals of 3 axes with 6 PWM signals per axis (PRUICSS PWM set 0, PRUICSS PWM set 1, PRUICSS PWM set 2) with a specified duty cycle and deadband at rise edge and fall edge of PWM, maintaining phase shift between axes.

The phase shift is achieved by delaying the rising edge of the PWM signals of the next axis by a fixed value. This creates a delay between the rising edges of the PWM signals of two adjacent axes.

Below parameters are fixed and configured during initialization:
- Frequency           : 16kHz
- Phase Shift         : 6944ns

Below parameters are configurable every PWM period and Arm® Cortex®-R5F writes these parameters in PRUICSS DMEM, PRU reads and controls PWM signals of every axis:
- PWMn_x_POS is configured with duty cycle of 25%, rise edge delay as 0ns and fall edge delay as 0ns (n = 0 to 2, x = 0 to 2)
- PWMn_x_NEG is configured with duty cycle of 25%, rise edge delay as 3000ns and fall edge delay as -3000ns (n = 0 to 2, x = 0 to 2)
- Number of PWM signals per each axis: 6
- Number of axes: 3 (Note: Each ICSS Instance can support up to 4 axes with 6 PWM signals per axis)

All these parameters are configurable.

In this example, PRUICSS IEP is configured in free run mode then Arm Cortex-R5F writes Compare Up, Compare down values, Scheduler Initial value and Incremental values in PRUICSS DMEM then PRU uses these values to generate PWM signals as shown below.

\imageStyle{pruicss_pwm_with_phase_shift.png, width:90%}
\image html pruicss_pwm_with_phase_shift.png "Programming Flow of PRU to update PRU-ICSS0 PWM0_A0(AXIS 0), PWM1_A0(AXIS 1), PWM2_A0(AXIS 2) compare values"

For more details refer below:
- section 6.4.10 of Technical Reference Manual
- <sdk-install-dir/source/pruicss_pwm_with_phase_shift/firmware/main.asm>
- <sdk-install-dir/examples/pruicss_pwm_with_phase_shift/pruicss_pwm_with_phase_shift_example.c>

#### TMDS243EVM
<a href="https://www.ti.com/tool/TMDS64DC01EVM" target="_blank"> An IO Breakout Board </a> is required to probe the PWM outputs
- PRG0_PWM0_A0 can be probed on J3.1
- PRG0_PWM0_B0 can be probed on J3.3
- PRG0_PWM0_A1 can be probed on J3.5
- PRG0_PWM0_B1 can be probed on J3.7
- PRG0_PWM0_A2 can be probed on J3.9
- PRG0_PWM0_B2 can be probed on J3.11
- PRG0_PWM1_A0 can be probed on J5.3
- PRG0_PWM1_B0 can be probed on J5.5
- PRG0_PWM1_A1 can be probed on J5.7
- PRG0_PWM1_B1 can be probed on J5.11
- PRG0_PWM1_A2 can be probed on J5.13
- PRG0_PWM2_A0 can be probed on J6.7
- PRG0_PWM2_B0 can be probed on J6.9
- PRG0_PWM2_A1 can be probed on J6.11
- PRG0_PWM2_B1 can be probed on J6.13
- PRG0_PWM2_A2 can be probed on J6.17
- PRG0_PWM2_B2 can be probed on J6.19


#### Probe Output

\imageStyle{pruicss_pwm_with_phase_shift_probe_output.png, width:90%}
\image html pruicss_pwm_with_phase_shift_probe_output.png "Phase shift between PRU-ICSS0 PWM0_A0, PWM1_A0, PWM2_A0"

- step 1: Here PWM signals are off
- step 2: Generate PWM signals for 500usecs with 12.5%, 25%, 50%, 75% duty cycle
- step 3: Fix PRGx_PWMy_Az and PRGx_PWMy_Bz to ~(initial_state), this can be altered to Fix PRGx_PWMy_Az to and PRGx_PWMy_Bz to initial_state by changing FIX_PRGX_PWMY_AZ_PRGX_PWMY_BZ_TO_INITIAL_STATE_COMPLIMENT to 0 in main.asm of PRU project and rebuilding it
(When FIX_PRGX_PWMY_AZ_PRGX_PWMY_BZ_TO_INITIAL_STATE_COMPLIMENT is set to 1, generating or stopping of PWM signal is decided while configuring second compare event and AXIS_X_SKIP_CMP_UP_AND_DOWN_INIT_VAL can be used to generate or stop PWM signal during firmware initialization)
- step 4: Regenerate PWM signals for 500usecs with 75% duty cycle

Above four steps are repeated in an infinite loop.

# Supported Combinations

\cond SOC_AM243X

 Parameter          | Value
 -------------------|-----------
 CPU + OS           | r5fss0-0 freertos
 ^                  | r5fss0-0 nortos
 Toolchain          | ti-arm-clang
 Boards             | @VAR_BOARD_NAME_LOWER
 R5F project folder | examples/pruicss_pwm/pruicss_pwm_with_phase_shift
 PRU Project folder | source/pruicss_pwm_with_phase_shift

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project from the above mentioned Example folder path for R5F and PRU. After this, `main.asm`, `linker.cmd` files get copied to ccs workspace of PRU project. The `main.asm` contains code to configure compare up and compare down values of PWM signals every PWM period.

- Build the PRU project using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
     - Build Flow: Once you click on build in PRU project, firmware header file which is generated in release or debug folder of ccs workspace, is moved to  `<sdk-install-dir/source/pruicss_pwm_with_phase_shift/firmware/device/>`

- Build the R5F project using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
     - Firmware header file path is included in R5F project include options by default. Instructions in Firmware header file can be written into PRU IRAM memory using PRUICSS_loadFirmware API call
     - Build Flow: Once you click on build in R5F project, SysConfig files are generated. Finally the R5F project will be generated using both the generated SysConfig and PRU project binaries.

    \note
    Prerequisite: [PRU-CGT-2-3](https://www.ti.com/tool/PRU-CGT) (ti-pru-cgt) should be installed at: `C:/ti/`

- **When using makefiles to build**, note the required combination and build PRU project then R5F project using make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)

- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

# PRUICSS PWM Phase Shift Example Debug Guide {#PRUICSS_PWM_PHASE_SHIFT_EXAMPLE_DEBUG_GUIDE}

### No PRUICSS PWM Output

- Verify pin configurations in SysConfig.
   - Open `example.syscfg` file for correct PWM pin assignments.

- Verify IEP counter is running.
   - Check IEP0_GLOBAL_CFG bit 0 (CNT_ENABLE) = 1.
   - Monitor IEP0_COUNT_REG0/1 to verify incrementing.

- Verify PWM active state configuration.
   - Using CCS memory browser, check CSL_ICSSCFG_PWM0_0 register.
   - Verify CSL_ICSSCFG_PWM0_0_PWM0_0_POS_ACT field = 0 (TOGGLE).
   - Check same field for all PWM channels in use.

- Verify firmware loading.
   - Connect to PRU core in CCS and open disassembly window.
   - Verify instructions are loaded in PRU instruction memory.
   - Verify firmware version is printed on console.
   - Check PRU_DMEM at 0x600 for updated PWM parameters.

### Incorrect PWM Output

- Verify frequency and phase shift values.
   - Check `PWM_OUTPUT_FREQ` value.
   - Verify `PHASE_SHIFT_BETWEEN_AXES` value.
   - Measure actual frequency and phase shift with oscilloscope.

- Check duty cycle and deadband settings.
   - Add `gpruIcssPwmDutyCycleDeadBandValues` to CCS expression window.
   - Verify duty cycle and deadband values.

\note
   - use AM243x Technical Reference Manual to find register address mentioned above

# PRUICSS PWM EPWM SYNC {#EXAMPLE_PRUICSS_PWM_EPWM_SYNC}

[TOC]

# Introduction

This example generates a signal for a specified period and duty cycle using
PRUICSS PWM and SOC EPWM. The period and duty cycle can be configured by the user.

\cond SOC_AM243X

## AM243X-LP
The example Uses PRUICSSG0 PWM module and does below

- Configures EPWM0_CHANNEL_A to generate a 1KHz signal with 25% duty cycle
- Configures EPWM0_SYNC_OUT to output high when SOC EPWM PERIOD VAL reaches zero
- Configures IEP counter reset on EPWM0_SYNC_OUT event
- Configures PWM0_0_POS(alias signal PWM0_A0) to generate a 1KHz signal with 50% duty cycle 
- Configures PWM3_2_NEG(alias signal PWM3_B2) to generate a 1KHz signal with 75% duty cycle 
- PWM0_0_POS(alias signal PWM0_A0) uses IEP0 CMP1  EVENT to control Duty cycle and EPWM0 SYNC OUT to control output Frequency
- PWM3_2_NEG(alias signal PWM3_B2) uses IEP1 CMP12 EVENT to control Duty cycle and EPWM0 SYNC OUT to control output Frequency
- Configures IEP0 CMP6 value with PWM0_0_POS(alias signal PWM0_A0) output duty cycle value
- Configures IEP1 CMP12 value with PWM3_2_NEG(alias signal PWM3_B2) output duty cycle value
- Configures IEP0 CMP0 value with zero to make state transition to intial on IEP counter reset
- PWM0_0_POS(alias signal PWM0_A0) and PWM3_2_NEG(alias signal PWM3_B2) and EPWM0_CHANNEL_A will be in sync with respect to each other as EPWM0_SYNC_OUT is used to  control output period of these signals
- PRG0_PWM0_A0 can be probed on J1.5
- PRG0_PWM3_B2 can be probed on J2.8
- EPWM0_CHANNEL_A can be probed on J4.1

#### AM243X-LP Probe Output 
\imageStyle{am243x_lp_soc_epwm_pruicss_pwm_sync_probe_output.png,width:85%}
\image html am243x_lp_soc_epwm_pruicss_pwm_sync_probe_output.png "PRUICSS PWM EPWM SYNC PROBE OUTPUT"

\endcond

# Supported Combinations

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/pruicss_pwm/pruicss_pwm_epwm_sync

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using
  make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>
- To probe the PRUICSS PWM output please refer setup details as mentioned above in Introduction section

# See Also

\ref PRUICSS_PWM_API



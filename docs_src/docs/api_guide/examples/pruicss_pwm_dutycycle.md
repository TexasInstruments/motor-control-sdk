# PRUICSS PWM Duty Cycle {#EXAMPLE_PRUICSS_PWM_DUTY_CYCLE}

[TOC]

# Introduction

This example generates a signal for a specified period and duty cycle using
PRUICSS PWM. The period and duty cycle can be configured by the user.

\cond SOC_AM243X

## AM243X-EVM
The example Uses PRUICSSG1 PWM module and does below

- This example uses PRUICSSG1 PWM module as probing PRUICSSG0 PWM signals requires IO breakout board
- Configures PWM0_2_NEG(alias signal PWM0_B2) to generate a 1KHz signal with 25% duty cycle 
- PWM0_2_NEG(alias signal PWM0_B2) uses IEP0 CMP6 EVENT to control Duty cycle and IEP0 CMP0 to control output Frequency
- Configures IEP0 CMP0 value with PWM0_2_NEG(alias signal PWM0_B2) output period value
- Configures IEP0 CMP6 value with PWM0_2_NEG(alias signal PWM0_B2) output duty cycle value
- Configures IEP counter reset on CMP0 event
- PRG1_PWM0_B2 can be probed on J16 PIN1

#### AM243X-EVM Probe Output 
\imageStyle{am64x_am243x_evm_duty_example_probe_output.png,width:70%}
\image html am64x_am243x_evm_duty_example_probe_output.png "PRUICSS PWM DUTY CYCLE PROBE OUTPUT"

## AM243X-LP
The example Uses PRUICSSG0 PWM module and does below

- Configures PWM0_0_POS(alias signal PWM0_A0) to generate a 1KHz signal with 25% duty cycle 
- Configures PWM3_2_NEG(alias signal PWM3_B2) to generate a 1KHz signal with 25% duty cycle 
- PWM0_0_POS(alias signal PWM0_A0) uses IEP0 CMP1 EVENT to control Duty cycle and IEP0 CMP0 to control output Frequency
- PWM3_2_NEG(alias signal PWM3_B2) uses IEP1 CMP12 EVENT to control Duty cycle and IEP0 CMP0 to control output Frequency
- Configures IEP0 CMP0 value with PWM0_0_POS(alias signal PWM0_A0) and PWM3_2_NEG(alias signal PWM3_B2) output period value
- Configures IEP counter reset on CMP0 event
- Configures IEP0 CMP6 value with PWM0_0_POS(alias signal PWM0_A0) output duty cycle value
- Configures IEP1 CMP12 value with PWM3_2_NEG(alias signal PWM3_B2) output duty cycle value
- PWM0_0_POS(alias signal PWM0_A0) and PWM3_2_NEG(alias signal PWM3_B2) will be in sync with respect to each other as IEP0 CMP0 value is used to control output period of these signals
- PRG0_PWM0_A0 can be probed on J1.5
- PRG0_PWM3_B2 can be probed on J2.8

#### AM243X-LP Probe Output 
\image html am243x_lp_duty_example_probe_output.png "PRUICSS PWM DUTY CYCLE PROBE OUTPUT"

\endcond

\cond SOC_AM64X

## AM64X-EVM
The example Uses PRUICSSG1 PWM module and does below

- This example uses PRUICSSG1 PWM module as probing PRUICSSG0 PWM signals requires IO breakout board
- Configures PWM0_2_NEG(alias signal PWM0_B2) to generate a 1KHz signal with 25% duty cycle 
- PWM0_2_NEG(alias signal PWM0_B2) uses IEP0 CMP6 EVENT to control Duty cycle and IEP0 CMP0 to control output Frequency
- Configures IEP0 CMP0 value with PWM0_2_NEG(alias signal PWM0_B2) output period value
- Configures IEP0 CMP6 value with PWM0_2_NEG(alias signal PWM0_B2) output duty cycle value
- Configures IEP counter reset on CMP0 event
- PRG1_PWM0_B2 can be probed on J16 PIN1

#### AM64X-EVM Probe Output 
\imageStyle{am64x_am243x_evm_duty_example_probe_output.png,width:70%}
\image html am64x_am243x_evm_duty_example_probe_output.png "PRUICSS PWM DUTY CYCLE PROBE OUTPUT"

\endcond


# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/pruicss_pwm/pruicss_pwm_dutycycle

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/pruicss_pwm/pruicss_pwm_dutycycle

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



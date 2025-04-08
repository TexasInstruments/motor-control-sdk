# PRU-ICSS PWM(R5F Based and PRU Based Control) {#PRUICSS_PWM}

[TOC]

# Features Supported in R5F based and PRU based PRU-ICSS PWM control

- Configuration of IEP compare events to produce the PWM outputs
- Enable Efficiency mode to auto clear compare status on state transition
- Configuration of  duty cycle to each of the PWM outputs
- Configuration of  PWM outputs behaviour in Intial state
- Configuration of  PWM outputs behaviour in Active state
- Configuration of  PWM outputs behaviour in Trip   state
- Configuration of  PWM Glitch Filter with Debounce Value
- Configuration of  PWM Deadband
- Configuration of  Tripzone output block to mask trip errors inputs

# PRU-ICSS PWM control using R5F

The PRU-ICSS PWM driver provides API to program Intial, Trip, Active states of PWM outputs.
Above are the high level features supported by the driver.

## SysConfig Features supported for R5F based PRU-ICSS PWM control

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:

- Configuration of  IEP counter
- Configuration of  PWM frequency
- Configuration of  Duty cycle to each of the PWM outputs
- Configuration of  Fall edge and Rise edge delay which inserts deadband

## Features NOT Supported in R5F based PRU-ICSS PWM control

- Generate PWM outputs with distinct time period, which means all PRUICSS PWM outputs are generated with same frequency controlled by IEP compare 0 event.
- Phase shift greater than half of the PWM period.

## Example Usage for R5F based PRU-ICSS PWM control

- \ref EXAMPLE_PRUICSS_PWM_DEADBAND_EPWM_SYNC

## API Usage for R5F based PRU-ICSS PWM control

\ref PRUICSS_PWM_API

# PRU-ICSS PWM control using PRU

Major difference between R5F driver based PRUICSS PWM control and PRU based PRUICSS PWM control is below:

- In PRU based PWM control PRUICSS IEP timer is configured free running and Compare event is used as schedular in order to update compare events of each axis PWM signals

## Additional features supported in PRU based PRU-ICSS PWM control
    - Each PWM output can have distinct PWM period (Note : this is not supported currently in SDK example)
    - Configurable Phase shift between each axis

## SysConfig Features supported for PRU based PRU-ICSS PWM control
    - Pinmuxing can be done from Additonal Settings of PRU(ICSS) sysconfig module
 
## Example Usage

- \ref EXAMPLE_PRUICSS_PWM_WITH_PHASE_SHIFT

## Important Note

PRU-ICSS has one PWM module, which has four PWM sets (PWM0, PWM1, PWM2, PWM3)
Each Set has six signals (A0,A1,A2,B0,B1,B2) With Reference to Technical Reference Manual, Pwm six signals(A0,A1,A2,B0,B1,B2) Naming convention is is slightly different as mentioned in \ref PRUICSS_PWM_API

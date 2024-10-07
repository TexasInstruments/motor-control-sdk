# PRU-ICSS PWM {#PRUICSS_PWM}

[TOC]

The PRU-ICSS PWM driver provides API to program Intial, Trip, Active states of PWM outputs.
Below are the high level features supported by the driver.

## Features Supported

- Configuration of IEP compare events to produce the PWM outputs
- Enable Efficiency mode to auto clear compare status on state transition
- Configuration of  duty cycle to each of the PWM outputs
- Configuration of  PWM outputs behaviour in Intial state
- Configuration of  PWM outputs behaviour in Active state
- Configuration of  PWM outputs behaviour in Trip   state
- Configuration of  PWM Glitch Filter with Debounce Value
- Configuration of  PWM Deadband
- Configuration of  Tripzone output block to mask trip errors inputs

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:

- Configuration of  IEP counter
- Configuration of  PWM frequency
- Configuration of  Duty cycle to each of the PWM outputs
- Configuration of  Fall edge and Rise edge delay which inserts deadband

## Features NOT Supported

- Generate PWM outputs with distinct time period, which means all PRUICSS PWM outputs are generated with same frequency controlled by IEP compare 0 event.

## Important Note

PRU-ICSS has one PWM module, which has four PWM sets (PWM0, PWM1, PWM2, PWM3)
Each Set has six signals (A0,A1,A2,B0,B1,B2) With Reference to Technical Reference Manual, Pwm six signals(A0,A1,A2,B0,B1,B2) Naming convention is is slightly different as mentioned in \ref PRUICSS_PWM_API

## Example Usage

- \ref EXAMPLE_PRUICSS_PWM_DEADBAND_EPWM_SYNC

## API

\ref PRUICSS_PWM_API

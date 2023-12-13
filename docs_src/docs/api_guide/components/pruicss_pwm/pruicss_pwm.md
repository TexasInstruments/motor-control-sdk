# PRUICSS PWM {#PRUICSS_PWM}

[TOC]

The PRUICSS PWM driver provides API to program Intial, Trip, Active states of PWM outputs. 
Below are the high level features supported by the driver.

## Features Supported

- Configuration of IEP compare events to produce the PWM outputs
- Enable Efficiency mode to auto clear compare status on state transition
- Configuration of  duty cycle to each of the PWM outputs
- Configuration of  PWM outputs behaviour in Intial state
- Configuration of  PWM outputs behaviour in Active state
- Configuration of  PWM outputs behaviour in Trip   state
- Configuration of  PWM Glitch Filter with Debounce Value
- Configuration of  Tripzone output block to mask trip errors inputs

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:

- Syscfg based customization will be supported in future releases.

## Features NOT Supported

Generate PWM outputs with distinct time period, which means all the PWM outputs will be in sync with respected to each other, without re-programming.

## Important Note

PRUICSS has one pwm module, which has four pwm sets (PWM0, PWM1, PWM2, PWM3)
Each Set has six signals (A0,A1,A2,B0,B1,B2) With Reference to Technical Reference Manual, Pwm six signals(A0,A1,A2,B0,B1,B2) Naming convention is is slightly different as mentioned in \ref PRUICSS_PWM_API          
   
## Example Usage

\ref EXAMPLE_PRUICSS_PWM_DUTY_CYCLE

## API

\ref PRUICSS_PWM_API
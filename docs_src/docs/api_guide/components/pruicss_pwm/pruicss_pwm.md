# PRU-ICSS PWM {#PRUICSS_PWM}

[TOC]

# Important Notes

- PRU-ICSS has one PWM module, which has four PWM sets (PWM0, PWM1, PWM2, PWM3)
- Each Set has six signals (A0, A1, A2, B0, B1, B2) With Reference to Technical Reference Manual, PWM six signals(A0, A1, A2, B0, B1, B2) naming convention is slightly different as mentioned in \ref PRUICSS_PWM_API
- Major difference between Arm® Cortex®-R5F based PRUICSS PWM control and PRU based PRUICSS PWM control is below:
    - In PRU based PWM control PRUICSS IEP timer is configured free running and Compare event is used as scheduler in order to update compare events of each axis PWM signals

# Features Supported in Arm Cortex-R5F based example

- Configuration of IEP compare events to produce the PWM outputs
- Enable Efficiency mode to auto clear compare status on state transition
- Configuration of duty cycle to each of the PWM outputs
- Configuration of PWM outputs behavior in Initial state
- Configuration of PWM outputs behavior in Active state
- Configuration of PWM outputs behavior in Trip state
- Configuration of PWM Glitch Filter with Debounce Value
- Configuration of PWM Deadband
- Configuration of Tripzone output block to mask trip errors inputs
- Driver APIs to program Initial, Trip, Active states of PWM outputs

## ICSS PRU Resource Usage
<table>
<tr>
   <th>PRU Core</th>
   <th>ICSS Memory Usage</th>
   <th>IEP Usage</th>
   <th>Other Peripheral Usage</th>
   <th>Description</th>
</tr>
<tr>
    <td>Un-used</td>
    <td>Un-used</td>
    <td>IEP CMP events are used. Refer Table 6-441 PWM to IEP Compare mapping of AM243x Technical Reference Manual</td>
    <td>Un-used</td>
    <td>This example demonstrates using deadband feature of PRUICSS PWM and synchronizing it with EPWM sync out. <br> Based on required PRUICSS PWM frequency IEP compare event configuration to generate PRUICSS PWM can be split across multiple R5F cores</td>
</tr>
</table>

## SysConfig Features supported

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:

- Configuration of IEP counter
- Configuration of PWM frequency
- Configuration of Duty cycle to each of the PWM outputs
- Configuration of Fall edge and Rise edge delay which inserts deadband

## Features NOT Supported

- Generate PWM outputs with distinct time period, which means all PRUICSS PWM outputs are generated with same frequency controlled by IEP compare 0 event.
- Phase shift greater than half of the PWM period.

## Example Usage

- \ref EXAMPLE_PRUICSS_PWM_DEADBAND_EPWM_SYNC

## API Usage

\ref PRUICSS_PWM_API

# Features supported in PRU based example

- Configuration of IEP compare events to produce the PWM outputs
- Enable Efficiency mode to auto clear compare status on state transition
- Configuration of duty cycle to each of the PWM outputs
- Configuration of PWM outputs behavior in Initial state
- Configuration of PWM outputs behavior in Active state
- Configuration of PWM outputs behavior in Trip state
- Configuration of PWM Glitch Filter with Debounce Value
- Configuration of PWM Deadband
- Configuration of Tripzone output block to mask trip errors inputs
- Configurable phase shift between each axis
- Each PWM output can have distinct PWM period (NOTE: Current SDK examples uses same period, but it can be modified)

## SysConfig Features supported
    - Pinmuxing can be done from Additional Settings of PRU(ICSS) sysconfig module

## Example Usage

- \ref EXAMPLE_PRUICSS_PWM_WITH_PHASE_SHIFT

## ICSS PRU Resource Usage
<table>
<tr>
   <th>PRU Core</th>
   <th>ICSS Memory Usage</th>
   <th>IEP Usage</th>
   <th>Other Peripheral Usage</th>
   <th>Description</th>
</tr>
<tr>
    <td>PRUx <br> Note : Any PRU can be used</td>
    <td>DMEM0 : 168bytes <br> Default offset : 0x600</td>
    <td>IEP CMP events are used. Refer Table 6-441 PWM to IEP Compare mapping of AM243x Technical Reference Manual</td>
    <td>Un-used</td>
    <td>This example demonstrates PRUICSS PWM generation with phase shifting capabilities. <br> Based on required PRUICSS PWM frequency, IEP compare event configuration to generate PRUICSS PWM can be split across multiple PRU cores</td>
</tr>
</table>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

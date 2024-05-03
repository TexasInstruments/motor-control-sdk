# Control algorithms {#CONTROL}

[TOC]

## Introduction

This section goes over a collection of control algorithms and techniques used in motors such as PMSM and BLDC. The library source code can be found in motor_control_sdk/source/control.

## Features Supported

Supported algorithms:

- Field weakening control (FWC)
- Maximum torque per ampere (MTPA)
- Strator voltage frequency generator (VS_FREQ)
- Phase voltage reconstruction (VOLREC)
  

## Field Weakening Control 

Field weakening control (FWC) is a control methodology used in iPMSM motors to expand operating limits and enable higher than rated speed while allowing optimal control across speed and voltage range.

## Maximum Torque Per Ampere 

Maximum Torque Per Ampere (MTPA) technique is used for iPMSM to optimize torque generation in the constant torque region. As iPMSM motor's total torque has a non-linear relationship with respect to the rotor angle.

## Stator Voltage Frequency Generator

Stator Voltage Frequency Generator (VS_FREQ) can be used in scalar volts/hertz motor control scheme. It generates an output command voltage based on the specified volts/hertz profile and the inputted command frequency.

## Benchmark Results

A benchmark on R5F core has been conducted to observe the following results when running controller arithmetic:

<table>
<tr>
    <th>Functions
    <th>Cpu Cycles
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Single-shunt DC-Link </td></tr>
<tr>
    <td>DCLINK_SS_runCurrentReconstruction</td>
    <td>14</td>
</tr>
<tr>
    <td>DCLINK_SS_runFastCurrentReconstruction</td>
    <td>5</td>
</tr>
<tr>
    <td>DCLINK_SS_runPWMCompensation</td>
    <td>201</td>
</tr>
<tr>
    <td>DCLINK_SS_runFastPWMCompensation</td>
    <td>107</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Field weakening control  </td></tr>
<tr>
    <td>FWC_computeCurrentAngle</td>
    <td>35</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Maximum torque per ampere </td></tr>
<tr>
    <td>MTPA_computeCurrentReference</td>
    <td>101</td>
</tr>
<tr>
    <td>MTPA_computeCurrentAngle</td>
    <td>85</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Strator voltage frequency generator </td></tr>
<tr>
    <td>VS_FREQ_run</td>
    <td>29</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Phase voltage reconstruction </td></tr>
<tr>
    <td>VOLREC_calcVolSF</td>
    <td>30</td>
</tr>
<tr>
    <td>VOLREC_run</td>
    <td>90</td>
</tr>
</table>

- Ran with TI Clang Compiler v3.2.0.LTS, with -Os flag, obtained the average result from 600 consecutive reading of running the controller with DPL CycleCountP minus the overhead, simulating a control loop scenario.
- Actual result may vary depending on provided datasets and memory configuration. For R5F, it is recommended for users to map control loops to TCM for the best performance.
  
## Provided Examples 



## Additional References

N/A

## API

\ref CONTROLS_MODULE

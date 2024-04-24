# Observer algorithms {#OBSERVERS}

[TOC]

## Introduction

This section covers the provided encoder/observer algorithms used in both sensored/sensorless field oriented control (FOC). The library source code can be found in motor_control_sdk/source/observer.

## Features Supported

Sensored:
- Encoder (eQEP based)
- Hall sensor

Sensorless:
- Enhanced Sliding Mode Observer (eSMO)

Speed measurement:
- speedcalc: Encoder speed calculation (SPDCALC)
- speedfr: Frequency estimation for eSMO (SPDFR)

## Encoder

The encoder (ENC) algorithm leverages the Enhanced Quadrature Encoder Pulse (eQEP) feature on the device for a sensored-based FOC. It calculates rotor angle based on eQEP encoder.

## Enhanced Sliding Mode Observer

Enhanced Sliding Mode Observer (eSMO) estimates rotor angle to achieve a sensorless FOC.

## Hall

The Hall algorithm uses an external Hall effect sensor to calculate speed and rotor angle for a sensored FOC design.

## Speed measurement

Speed measurement is a supplement algorithm that measures speed based on the rotor angle. It consists of two different implementations, speedfr (SPDFR) calculates speed from an eQEP encoder signal whereas speedcalc (SPDCALC) estimates frequency of the eSMO observer.

## Benchmark Results

A benchmark on R5F core has been conducted to observe the following results when calling each functions:

<table>
<tr>
    <th>Functions
    <th>Cpu Cycles
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Encoder </td></tr>
<tr>
    <td>ENC_run (w/o calibration)</td>
    <td>16</td>
</tr>
<tr>
    <td>ENC_run (w/ calibration)</td>
    <td>30</td>
</tr>
<tr>
    <td>ENC_runHall</td>
    <td>16</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Enhanced Sliding Mode Observer </td></tr>
<tr>
    <td>EMSO_run</td>
    <td>307</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Hall </td></tr>
<tr>
    <td>HALL_run</td>
    <td>73</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Sensorless speed measurement </td></tr>
<tr>
    <td>SPDCALC_run</td>
    <td>53</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Sensored speed measurement </td></tr>
<tr>
    <td>SPDFR_run</td>
    <td>32</td>
</tr>

</table>

- Ran with TI Clang Compiler v3.2.0.LTS, with -Os flag, obtained the average result from 600 consecutive reading of running the controller with DPL CycleCountP minus the overhead, simulating a control loop scenario.
- Actual result may vary depending on provided datasets and memory configuration. For R5F, it is recommended for users to map control loops to TCM for the best performance.
  
## Provided Examples 


## Additional References

N/A

## API

\ref OBSERVERS_MODULE

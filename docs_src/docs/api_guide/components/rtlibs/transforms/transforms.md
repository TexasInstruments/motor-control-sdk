# Transformation Algorithm {#TRANSFORMS}

[TOC]

## Introduction

The Transformation module consists of transformation commonly found in motor-control applications.

## Features Supported

Transformation module includes:

- Clarke transformation
- Park transformation
- Inverse Park transformation
- Space Vector Generation (SVGEN)
  - Common-mode subtraction approach
- DPWM Generation (Part of SVGEN)
  - Maximum Modulation
  - Minimum Modulation
- SVGEN current reconstruction for single-shunt (SVGENCURRENT)
- Phase voltage reconstruction in overmodulation (VOLTS_RECON)
  
## Features Not Supported

N/A

## Benchmark Results

A benchmark on R5F core has been conducted to observe the following results when running the following functions:

<table>
<tr>
    <th>Transform Function
    <th>Cpu Cycles
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Clarke Transform </td></tr>
<tr>
    <td>CLARKE_run_twoInput</td>
    <td>16</td>
</tr>
<tr>
    <td>CLARKE_run_threeInput</td>
    <td>18</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Park Transform </td></tr>
<tr>
    <td>PARK_run</td>
    <td>19*</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Inverse Park Transform </td></tr>
<tr>
    <td>IPARK_run</td>
    <td>19*</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Space Vector Generation </td></tr>
<tr>
    <td>SVGEN_runCom</td>
    <td>41</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> DPWM Generation </td></tr>
<tr>
    <td>SVGEN_runMax</td>
    <td>36</td>
</tr>
<tr>
    <td>SVGEN_runMin</td>
    <td>36</td>
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

  \note both PARK and IPARK does not include trigonometric calculations of phasor. For R5F core, it's recommened to leverage the fast ti_arm_sincos/FastRTS_sincos function found in mathlib of MCU+ SDK.
- Ran with TI Clang Compiler v3.2.0.LTS, with -Os flag and the benchmarking function placed in TCM memory. Obtained the average result from 600 consecutive loops of running transform functions with DPL CycleCountP, mimicking the usage of a control loop.
- Actual result may vary depending on provided datasets and memory configuration. For R5F, it is recommended for users to map control loops to TCM for the best performance.

## Provided Examples 

The following examples has been provided to demonstrate transformation module:

- \ref EXAMPLES_TRANSFORMS_TEST

## Additional References {#TRANSFORMS_ADDITIONAL_REFERENCES}

N/A

## API

\ref TRANSFORMS_API_MODULE

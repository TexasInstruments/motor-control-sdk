# Software Frequency Response Analyzer {#SFRA}

[TOC]

## Introduction

The Software Frequency Response Analyzer (SFRA) is a software library that enables developers to
quickly measure the frequency response of their digital power converter. The SFRA library contains
software functions that inject a frequency into the control loop and measure the response of the system
using the Sitara MCUs’ on-chip analog to digital converter (ADC). This process provides the plant
frequency response characteristics, the open loop gain frequency response and the closed loop frequency
response of the closed loop system. 

This library contains header file (sfra_f32.h) and ".lib" file based on release/debug configuration.

## Features Supported

SFRA module includes calculation and storage of following:

- User configurable sweep start frequency, frequency step size, injection amplitude
- Plant Magnitude and Phase vector corresponding to frequency points configured by user
- Open loop Magnitude and Phase vector corresponding to frequency points configured by user
- Closed loop Magnitude and Phase vector corresponding to frequency points configured by user

SFRA contains Matlab scripts used for following:

- Extract the data from excel sheet (which needs to be in same format as "SFRA.xlsx") and plot the boded plots
- Compare boded plots with laplace transfer function provided in script (user can change transfer function as per use case)
- Script uses sisotool to design the compensation based on boded plots (User need access for Matlab Control System Toolbox)
## Features Not Supported

GUI is not supported

## Benchmark Results

A benchmark on R5F core has been conducted to observe the following results when running the following functions:

<table>
<tr>
    <th>SFRA Function
    <th>Cpu Cycles
<tr>
    <td>SFRA_F32_reset</td>
    <td>15</td>
</tr>
<tr>
    <td>SFRA_F32_config</td>
    <td>15</td>
</tr>
<tr>
    <td>SFRA_F32_inject</td>
    <td>66</td>
</tr>
<tr>
    <td>SFRA_F32_collect</td>
    <td>37</td>
</tr>
<tr>
    <td>SFRA_F32_runBackgroundTask</td>
    <td>89</td>
</tr>
</table>

<table>
<tr>
    <th>SFRA Obj files
    <th>Code Size(Bytes/8bits)
<tr>
    <td>SFRA_F32_reset + SFRA_F32_config</td>
    <td>424</td>
</tr>
<tr>
    <td>SFRA_F32_inject + SFRA_F32_collect + SFRA_F32_runBackgroundTask</td>
    <td>1522</td>
</tr>
</table>

- Ran with TI Clang Compiler v3.2.1.LTS, with -Os flag and functions force-inlined onto the benchmarking function placed in TCM memory. Obtained the average result from 500 consecutive loops of running sfra functions with DPL CycleCountP, mimicking the usage of a control loop and overhead (~10 cycles) is removed.
- Actual result may vary depending on provided datasets and memory configuration. For R5F, it is recommended for users to map control loops to TCM for the best performance.
## Provided Examples 

The following examples has been provided to demonstrate sfra module:

- \ref EXAMPLES_SFRA_TEST

## Additional References {#SFRA_ADDITIONAL_REFERENCES}

N/A

## API

\ref SFRA_API_MODULE

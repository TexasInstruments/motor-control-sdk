# Datalog {#DATALOG}

[TOC]

## Introduction

The Datalog library module commonly found in both digital control and motor-control applications. This module stores the real time values of user selectable variables. User configurable, "n" number of variables are selected by configuring "n" module inputs, iptr[0], iptr[1], ... iptr[n-1], point to the address of "n" variables. Data values are stored in datalog buffers, datalogBuff[0], datalogBuff[1], ... datalogBuff[n-1].

Number of datalog channels, buffer size and data type are configurable by user with "${SDK_INSTALL_PATH}/source/utilities/datalog/include/datalog_input.h" file update.

## Features Supported

Datalog library module includes user configurable:

- Number of datalog channels (n)
- Datalog buffer size
- Data type to be stored
- Trigger type and trigger value
- Prescalar

## Features Not Supported

N/A

## Benchmark Results

A benchmark on R5F core has been conducted to observe the following results when running the following functions:

<table>
<tr>
    <th>DATALOG Function
    <th>Cpu Cycles 
<tr><td colspan="2" bgcolor=#F0F0F0> Auto Trigger </td></tr>
<tr>
    <td>DATALOG_update</td>
    <td>68</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> Manual Trigger</td></tr>
<tr>
    <td>DATALOG_update</td>
    <td>73</td>
</tr>
</table>

- Ran with TI Clang Compiler v3.2.1.LTS, with -Os flag and functions force-inlined onto the benchmarking function placed in TCM memory. Obtained the average result from 500 consecutive loops of running datalog functions with DPL CycleCountP, mimicking the usage of a control loop and overhead (~10 cycles) is removed.
- Actual result may vary depending on provided datasets and memory configuration. For R5F, it is recommended for users to map control loops to TCM for the best performance.
## Provided Examples 

The following examples has been provided to demonstrate datalog module:

- \ref EXAMPLES_DATALOG_TEST

## Additional References {#DATALOG_ADDITIONAL_REFERENCES}

N/A

## API

\ref DATALOG_API_MODULE

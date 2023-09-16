# Digital Control Library (DCL) {#DCL}

[TOC]

## Introduction

The Sitara™ Digital Control Library (DCL) provides a suite of robust software functions for developers of digital
control applications using the Texas Instruments Sitara MCU. DCL is a header-only library, and all functions in
the library are provided in the form of C source-code.

The DCL contains PI,PID and "Direct Form" controller types. The former are typically used to tune properties of
a transient response, while the latter are typically used to shape the open loop frequency response. 

In addintion, DCL contains functions to convert controller parameters from one type to the other. As well as
functions to parameterize the controller given a transfer function.

Several utility modules such as error handling, data logging are also included as a supporting module.

## Features Supported

Supported controller modules (floating-point)

- Linear PI
- Linear PID
- Linear PI with double integrator
- Direct Form 1 (first order)
- Direct Form 1 (third order)
- Direct Form 2 (second order)
- Direct Form 2 (third order)

Other utility modules:
  
- Error Handling 
- Testpoints
- Data Logging

## Features Not Supported

(Compared with C2000Ware's DCL)
- Fix-point controller modules
- Non-linear PID controller
- Reference Generator and performance index
- Multi-channel data logs

## Benchmark Results

A benchmark on R5F core has been conducted to observe the following results when running controller arithmetic:

<table>
<tr>
    <th>Controller Function
    <th>Cpu Cycles
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> PI Controller </td></tr>
<tr>
    <td>DCL_runPISeries</td>
    <td>49</td>
</tr>
<tr>
    <td>DCL_runPIParallel</td>
    <td>50</td>
</tr>
<tr>
    <td>DCL_runPISeriesTustin</td>
    <td>56</td>
</tr>
<tr>
    <td>DCL_runPIParallelEnhanced</td>
    <td>62</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> PI2 Controller </td></tr>
<tr>
    <td>DCL_runPI2Series</td>
    <td>74</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> PID Controller </td></tr>
<tr>
    <td>DCL_runPIDSeries</td>
    <td>65</td>
</tr>
<tr>
    <td>DCL_runPIDParallel</td>
    <td>65</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> DF11 Controller </td></tr>
<tr>
    <td>DCL_runDF11</td>
    <td>24</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> DF13 Controller </td></tr>
<tr>
    <td>DCL_runDF13</td>
    <td>43</td>
</tr>
<tr>
    <td>DCL_runDF13Clamp</td>
    <td>53</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> DF22 Controller </td></tr>
<tr>
    <td>DCL_runDF22</td>
    <td>27</td>
</tr>
<tr>
    <td>DCL_runDF22Clamp</td>
    <td>41</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> DF23 Controller </td></tr>
<tr>
    <td>DCL_runDF23</td>
    <td>29</td>
</tr>
<tr>
    <td>DCL_runDF23Clamp</td>
    <td>45</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> PID 64bit Controller </td></tr>
<tr>
    <td>DCL_runPIDF64Series</td>
    <td>185</td>
</tr>
<tr>
    <td>DCL_runPIDF64Parallel</td>
    <td>174</td>
</tr>
</table>

- Ran with TI Clang Compiler v2.1.3.LTS, with -Os flag and all DCL functions inlined, obtained the average result from 600 consecutive reading of running the controller with DPL CycleCountP.
- Simulated inputs are based on arbitrary sinusoidal waves and saturation condition that roughly clamps ~50% of the time. For functions with clamp (PI,PI2,PID and DF Clamps), clock cycle varies depending on clamping condition and provided inputs.
- Actual result may vary depending on provided datasets and memory configuration. For R5F, it is recommended for users to map control loops to TCM for the best performance.
  
## Provided Examples 

The following examples has been provided to demonstrate the DCL library:

- \ref EXAMPLES_DCL_PI
- \ref EXAMPLES_DCL_DF22

## Additional References {#DCL_ADDITIONAL_REFERENCES}

N/A

## API

\ref DCL_API_MODULE

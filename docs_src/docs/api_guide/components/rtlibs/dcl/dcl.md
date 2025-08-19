# Digital Control Library (DCL) {#DCL}

[TOC]

## Introduction

The Sitara™ & C2000™ Digital Control Library (DCL) provides a suite of robust software functions for developers of digital control applications using the Texas Instruments Sitara™ or C2000™ MCUs.

DCL is a header-only library, and all functions in the library are provided in the form of C source-code.

The DCL contains PI, PID and "Direct Form" controller types. The former are typically used to tune properties of a transient response, while the latter are typically used to shape the open loop frequency response.

In addition, DCL contains functions to convert controller parameters from one type to the other, as well as functions to parameterize the controller given a transfer function.

Several utility modules are also included as supporting modules. Refer below for the list of supported modules.

## Getting Started

To use the DCL library, simply include the top-level file dcl.h and make sure the compiler search path includes the source path.
```
#include "dcl.h"
```

## Features Supported

Supported controller modules:
(In 32-bit floating-point, unless specified)

- Linear PI
  -  Serial PI
     +  DCL_runPISeries
  -  Serial PI with Tustin Integrator
     +  DCL_runPISeriesTustin
  - Serial PI with Double Integrator (PI2)
    + DCL_runPI2Series
  -  Parallel PI
     +  DCL_runPIParallel
  -  Parallel PI with Enhanced Anti-windup Logic
     +  DCL_runPIParallelEnhanced
     +  Note: It incorporates an additional integrator clamp, which makes use of the Imin and Imax attributes.
- Linear PID
  - Serial PID
    + DCL_runPIDSeries
  - 64bit Serial PID (PIDF64)
    + DCL_runPIDF64Series
  - Parallel PID
    + DCL_runPIDParallel
  - 64bit Parallel PID (PIDF64)
    + DCL_runPIDF64Parallel
- Non-linear PID (NLPID)
  - Serial PID
    + DCL_runNLPIDSeries
  - Parallel PID
    + DCL_runNLPIDParallel
- Non-linear Controls
  - Simple non-linear control law
    - DCL_runNLF

- Direct Form 1 1st order (DF11)
  - w/o saturation
    + DCL_runDF11
- Direct Form 1 2nd order (DF22)
  - w/o saturation
    + DCL_runDF12
  - w/ saturation
    + DCL_runDF12Clamp
- Direct Form 1 3rd order (DF13)
  - w/o saturation
    + DCL_runDF13
  - w/ saturation
    + DCL_runDF13Clamp
- Direct Form 2 2nd order (DF22)
  - w/o saturation
    + DCL_runDF22
  - w/ saturation
    + DCL_runDF22Clamp
- Direct Form 2 3rd order (DF23)
  - w/o saturation
    + DCL_runDF23
  - w/ saturation
    + DCL_runDF23Clamp

Other utility modules include:

- Data Logging (FDLOG)
- Multi-channel data logs (MLOG)
- Reference Generator (REFGEN)
- Transient Capture Module and performance index (TCM)
- Gain Scheduler Module (GSM)

Note: Controller modules include error handling and test points which are disabled by default. These features could be enabled by macros defined in "dcl.h"
Note: DCL is also backwards compatible with C2000™'s C28 (v3) API call. For a proper mapping between the two APIs, refer to "misc/dcl_c28_compatibility.h"

## Benchmark Results

A benchmark on Arm® Cortex®-R5F core has been conducted to observe the following results when running controller arithmetic:

<table>
<tr>
    <th>Controller Function
    <th>CPU Cycles
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

- Ran with TI Clang Compiler v2.1.3.LTS, with -Os flag and all DCL functions inlined, obtained the average result from 600 consecutive readings of running the controller with DPL CycleCountP.
- Simulated inputs are based on arbitrary sinusoidal waves and saturation condition that roughly clamps ~50% of the time. For functions with clamp (PI, PI2, PID and DF Clamps), clock cycle varies depending on clamping condition and provided inputs.
- Actual result may vary depending on provided datasets and memory configuration. For Arm Cortex-R5F, it is recommended for users to map control loops to TCM for the best performance.

## Release Notes

Shown below lists all the past software revision history for this software:

<table>
<tr>
    <th> Software Revision
    <th> Changelog
</tr>
<tr>
    <td> v4.03.00
    <td> Changes:
        - Merge C29 & Sitara™ DCL to be a common library across the two platforms
        - Ported Gain Scheduler Module (GSM)
        - Fix incorrect NLPID default variables
        - Reduce PID/NLPID control arithmetic
        - C28 API mapping are now included by default
    </td>
</tr>
<tr>
    <td> v4.02.00
    <td> More ported modules:
        - 64bit PID controller
        - Non-linear PID controller (NLPID)
        - Reference Generator (REFGEN)
        - Multi-channel datalogs (MLOG)
        - Transient Capture Module and performance index (TCM)
    </td>
</tr>
<tr>
    <td> v4.01.00
    <td> Initial release version as the continuation of C2000™ DCL. The porting includes (Backwards compatible with C2000™ DCL API):
        - Linear PI
        - Linear PID
        - Linear PI with double integrator (PI2)
        - Direct Form 1 (first order)
        - Direct Form 1 (second order)
        - Direct Form 1 (third order)
        - Direct Form 2 (second order)
        - Direct Form 2 (third order)
        - Data Logging (FDLOG)
    </td>
</tr>
</table>

## Provided Examples

The following examples have been provided to demonstrate the DCL library:

- \ref EXAMPLES_DCL_PI
- \ref EXAMPLES_DCL_DF22

## API

\ref DCL_API_MODULE

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

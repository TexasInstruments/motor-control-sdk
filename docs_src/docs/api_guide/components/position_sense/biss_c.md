# BISS-C {#BISS-C}

[TOC]

## Introduction

BiSS is an open-source digital interface for sensors and actuators. BiSS stands for bidirectional serial synchronous. The BiSS interface was introduced by iC-Haus GmbH as an open-source protocol in 2002. BiSS-C mode is the continuous mode in which the BiSS-C interface master reads out the position data cyclically. Control communication is available for the master to send commands to the encoders and to read and write the encoder local registers. The BiSS interface is used in position-control applications. The interface enables a complete closed-loop position control system by providing the real-time position feedback to the master to control the motor.

BiSS Safety is a profile definition for BiSS that has been certified by TÜV Rheinland for safety-critical applications up to SIL3 according to IEC61508:2010. BiSS Safety uses the concept of a "Black Channel" transmission and specifies the data channel contents in order to ensure failure mode detection as defined in IEC61784-3 using redundant position words, different CRC polynomials and a sign-of-life counter. BiSS Safety is fully compatible with BiSS and all of its features including line delay compensation, processing times. BiSS Safety is implemented by assuming 2 encoders connected in daisy chain one will send CPW and another one will send SPW. Daisy chaining is also possible on top of safety (2 encoders dedicated safety - one for CPW and another one for SPW) up to 3 encoders per channel.

## Features Supported

   -  BiSS-C Interface Master for point-to-point communication
   -  Support for single channel implementation with one encoder
   -  Receive on-the-fly CRC verification of position and control data
   -  Interface speed of 1, 2, 5, 8, and 10 MHz
   -  Two modes of operation - host trigger and periodic trigger
   -  Support for control communication
   -  Support for automatic processing delay detection and compensation
   -  Support for multiple encoders connected via daisy-chain configuration (up-to 3 encoders)
   -  Support for concurrent multi-channel support on a single PRU (up-to 3 identical encoders)
   -  Support for multi-channel encoders of different make under load share model (each of PRU, RTU-PRU, and TX-PRU from one PRU-ICSSG slice handles one channel)
   -  Support for up to 100 meter cable
   -  Readiness for BiSS Safety profile by supporting 16 bit CRC and sign-of-life counter

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported in this release, including the below
-  BISS Line
-  Independent clocks on multi channel mode.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSSG instance. (Tested on ICSSG0)
- Selecting the ICSSG PRU slice. (Tested on ICSSG0-PRU1)
- Configuring PINMUX.
- Frequency selection.
- Channel selection.
- Selecting Multi Channel with Encoders of Different Make using load share mode.

\note BiSS-C firmware will only run with ICSSG Core Clock running at 200 MHz or 300 MHz frequency. 225/250/333 MHz values are not supported due to clock divider requirements.

## BISS-C Design

\subpage BISSC_DESIGN explains the design in detail.

## Example
\ref EXAMPLE_MOTORCONTROL_BISSC

## API
\ref BISSC_API_MODULE


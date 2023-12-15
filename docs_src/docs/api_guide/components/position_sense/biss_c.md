# BISS-C {#BISS-C}

[TOC]

## Introduction

BiSS is an open-source digital interface for sensors and actuators. BiSS stands for bidirectional serial synchronous. The BiSS interface was introduced by iC-Haus GmbH as an open-source protocol in 2002. BiSS-C mode is the continuous mode in which the BiSS-C interface master reads out the position data cyclically. Control communication is available for the master to send commands to the encoders and to read and write the encoder local registers. The BiSS interface is used in position-control applications. The interface enables a complete closed-loop position control system by providing the real-time position feedback to the master to control the motor.

## Features Supported

   -  BiSS-C Interface Master for point-to-point communication
   -  Support for single channel implementation with one encoder
   -  Receive on-the-fly CRC verification of position and control data
   -  Interface speed of 1, 2, 5, 8, and 10 MHz
   -  Support for control communication
   -  Support for automatic processing delay detection and compensation
   -  Support for multiple encoders connected via daisy-chain configuration (up-to 3 encoders)
   -  Support for concurrent multi-channel support on a single PRU (up-to 3 identical encoders)
   -  Support for multi-channel encoders of different make under load share model (each of PRU, RTU-PRU, and TX-PRU from one PRU-ICSSG slice handles one channel)
   -  Support for up to 100 mtr cable

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported in this release, including the below
-  Safety
-  BISS Line
-  Independent clocks on multi channel mode.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSSG instance. (Tested on ICSSG0)
- Selecting the ICSSG0PRUx instance.(Tested on ICSSG0-PRU1)
- Configuring PINMUX.
- Frequency selection.
- Channel selection.
- Selecting Multi Channel with Encoders of Different Make using load share mode.


## BISS-C Design

\subpage BISSC_DESIGN explains the design in detail.

## Example
\ref EXAMPLE_MOTORCONTROL_BISSC

## API
\ref BISSC_API_MODULE


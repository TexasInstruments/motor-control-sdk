# Nikon A-Format {#NIKON}

[TOC]

## Introduction

Nikon A-Format absolute encoder receiver implementation on the TI PRU-ICSS interfaces with the Nikon encoders either via point-to-point communication or up to 3 Nikon encoders connected on a bus. Nikon encoders use a proprietary asynchronous serial bi-directional half-duplex communication protocol compliant with the RS-485. The encoder can generate either single- or multi-turn absolute position data and can operate at a baud rate of up to 16 MHz. Besides position data, the encoder can also send status and diagnostic information well. Reliability is ensured by using a 3-bit CRC by the receiver and an 8-bit CRC by the encoder. Another interesting feature of the Nikon A-Format encoders is the optional battery backup circuit, which can be used in case of power failure. Nikon encoders are widely used in industrial machinery, including industrial robots in automobile production lines and machine tools, and are highly valued as the next-generation standard for sensors that can detect the absolute rotational displacement values of robot arms.

## Features Supported

   -  Support for point-to-point and bus communication.
   -  Support for baud rates from 2.5 MHz, 4 MHz, 6.67 MHz, 8 MHz, and 16 MHz.
   -  Support for up to 40-bit absolute position (single turn + multi turn) data with additional information.
   -  Support for position preset, temperature information and alarms.
   -  Support for non-volatile (EEPROM) read and write access.
   -  Support for identification code read and write process.
   -  Support for encoder address setting.
   -  Support for individual and multiple transmission mode with encoder addresses ranging between ENC1-ENC8.
   -  Support for concurrent multi-channel support on a single PRU (up-to 3 channels with identical number of encoders of same frequency connected to all configured channels).
   -  Support for multi-channel with different communication modes and different number of encoders connected accross channels under load share model (each of PRU, RTU-PRU, and TX-PRU from one PRU-ICSSG slice handles all 3 channels).

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported in this release, including the below
-  Independent clocks on multi channel mode.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSSG instance. (Tested on ICSSG0)
- Selecting the ICSSG0PRUx instance.(Tested on ICSSG0-PRU1)
- Configuring PINMUX.
- Frequency selection.
- Channel selection.
- Selecting Multi Channel with Encoders connected in Different Configurations (bus or one-to-one) using load share mode.


## NIKON Design

\subpage NIKON_DESIGN explains the design in detail.

## Example
\ref EXAMPLE_MOTORCONTROL_NIKON

## API
\ref NIKON_API_MODULE


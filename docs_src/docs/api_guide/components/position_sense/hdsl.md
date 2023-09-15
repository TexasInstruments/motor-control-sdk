# HDSL {#HDSL}

[TOC]

## Introduction

The HDSL firmware running on ICSS-PRU provides a defined well interface to execute the HDSL protocol.

## Features Supported

-  Safe position
-  Fast position, speed
-  Communication status
-  External pulse synchronization
-  Register interface to be compatible with SICK HDSL FPGA IP Core (apart from the differences listed in \ref HDSL_EXCEPTIONS_LIST)
-  Parameter channel communication
	- Short message
	- Long message
- 	Safety
-	Two channels support on am243x-evm
-	Single channel support on am243x-lp
-	Tested with three different encoder makes (EDM35, EKS36, EKM36)

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported, including the below
 -  100m cable
 -  Three channel support using single PRU-ICSSG slice
 -  Pipeline Channel
 ## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSSG0PRUx instance.(Tested on ICSSG0-PRU1)
- Configuring PINMUX
- Channel selection
- Mode Selection (Free run/Sync mode)
- Hardware selection (Booster Pack for am243x-lp)

## HDSL Design

\subpage HDSL_DESIGN explains the design in detail.

## Register List

\subpage HDSL_REGISTER_LIST contains the description of registers in TI's HDSL implementation. Please note that all the corresponding register fields are not implemented.

## Exceptions

\subpage HDSL_EXCEPTIONS_LIST lists the exceptions TI's HDSL implementation when compared with SICK HDSL FPGA IP Core. Please note that all the corresponding register fields are not implemented.

## Example

\ref EXAMPLE_MOTORCONTROL_HDSL

## API
\ref HDSL_API_MODULE
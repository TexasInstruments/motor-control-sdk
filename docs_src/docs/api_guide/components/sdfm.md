# SDFM {#SDFM}

[TOC]

## Introduction
ICSS SDFM is a sigma delta interface for phase current measurement in high performance motor and servo drives. During Sigma delta decimation filtering (SDDF) the PRU hardware provides hardware integrators that do the accumulation part of Sinc filtering, while the ICSS SDFM firmware does differentiation part.

## Features Supported
 - 3 SDFM channels on single PRU core
 - Overcurrent (OC) for comparator: free running SINC3 filter with OSR 32-256
 - Normal current (NC) for data read: free running SINC3 filter with OSR 32-256, always multiple of OC OSR (NC OSR = K*OC OSR :: K ∈ Z)
 - Event generation(ARM interrupt for data read from DMEM, GPIO toggle for high and low thresholds)
 - High and Low threshold comparator

## Features Not Supported
- Zero cross comparator
- Below 32 OSR


## ICSS SDFM Design
\subpage SDFM_DESIGN explains the design in detail.

## Example
\ref EXAMPLE_MOTORCONTROL_SDFM

## API
\ref SDFM_API_MODULE

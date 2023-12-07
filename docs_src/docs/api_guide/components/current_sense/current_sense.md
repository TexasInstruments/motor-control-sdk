# Current Sense {#CURRENT_SENSE}

[TOC]

Current sensing is handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implements the low level firmware. The PRU-ICSS frees up the main ARM cores in the device for other functions, such as control and data processing.

## SDFM {#SDFM}

ICSS %SDFM is a sigma delta interface for phase current measurement in high performance motor and servo drives. During Sigma delta decimation filtering (SDDF) the PRU hardware provides hardware integrators that do the accumulation part of Sinc filtering, while the ICSS %SDFM firmware does differentiation part.

## Features Supported
 - 3 %SDFM channels on single PRU core
 - Normal current (NC) for data read:  SINC3 filter with OSR 16 to 256
 - Overcurrent (OC) for comparator: free running SINC3 filter with OSR 16 to 256
 - Event generation(ARM interrupt for data read from DMEM, GPIO toggle for high and low thresholds)
 - Single level High and Low threshold comparator
 - Trigger based normal current sampling
 - Double update: Double normal current sampling per EPWM cycle 
 - %SDFM Sync with EPWM 
 - Fast detect 
 - PWM Trip generation for overcurrent 

## Features Not Supported
- Zero cross comparator
- Clock phase compensation 
- Multi-level threshold 

## System design considerations
### Over Sample Ratio 
-  OSR Below 16 at SD_CLK greater than 20MHz. The normal current task takes 300ns to 400ns to complete and its execution is based on CMP event and task manager. When we configure OSR below 16 for greater than 20 MHz SD clock, the NC will not be able to complete its processing until the next sample is ready, this will cause the NC samples to be inaccurate.
     

## ICSS SDFM Design
\subpage SDFM_DESIGN explains the design in detail.

## Example
\ref EXAMPLE_MOTORCONTROL_SDFM

## API
\ref SDFM_API_MODULE
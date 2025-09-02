# BISS-C {#BISS-C}

[TOC]

## Introduction

BiSS is an open-source digital interface for sensors and actuators. BiSS stands for bidirectional serial synchronous. The BiSS interface was introduced by iC-Haus GmbH as an open-source protocol in 2002. BiSS-C mode is the continuous mode in which the BiSS-C interface receiver reads out the position data cyclically. Control communication is available for the receiver to send commands to the encoders and to read and write the encoder local registers. The BiSS interface is used in position-control applications. The interface enables a complete closed-loop position control system by providing the real-time position feedback to the receiver to control the motor.

BiSS Safety is a profile definition for BiSS that has been certified by TÜV Rheinland for safety-critical applications up to SIL3 according to IEC61508:2010. BiSS Safety uses the concept of a "Black Channel" transmission and specifies the data channel contents in order to ensure failure mode detection as defined in IEC61784-3 using redundant position words, different CRC polynomials and a sign-of-life counter. BiSS Safety is fully compatible with BiSS and all of its features including line delay compensation, processing times. BiSS Safety is implemented by assuming 2 encoders connected in daisy chain, one will send CPW and another one will send SPW. Daisy chaining is also possible on top of safety (2 encoders dedicated safety - one for CPW and another one for SPW) up to 3 encoders per channel.

\note This implementation using Peripheral input/output mode of PRU-ICSS. Refer \ref PRUICSS_PERIPHERAL_IF_MODE for more details.

## Features Supported
\cond SOC_AM243X

   -  BiSS-C interface receiver for point-to-point communication
   -  Support for single channel implementation with one encoder
   -  Receive on-the-fly CRC verification of position and control data
   -  Interface speed of 1, 2, 5, 8, and 10 MHz
   -  Support for oversampling ratio with different interface speeds
<table>
<tr>
    <th rowspan="2">Clock Source
    <th rowspan="1" colspan="5">Interface Speed
</tr>
<tr>
    <th>1 MHz
    <th>2 MHz
    <th>5 MHz
    <th>8 MHz
    <th>10 MHz
</tr>
<tr>
    <td>PRU UART Clock (192 MHz)
    <td>8x
	<td>8x
    <td>Not tested
	<td>8x
    <td>Not tested
</tr>
<tr>
    <td>PRU Core Clock (200 MHz)
    <td>Not tested
	<td>Not tested
    <td>8x
	<td>Not tested
    <td>4x
</tr>
<tr>
    <td>PRU Core Clock (300 MHz)
    <td>Not tested
	<td>Not tested
    <td>8x with fractional div
	<td>Not tested
    <td>4x with Fractional div
</tr>
</table>

   -  Two modes of operation - host trigger and periodic trigger
   -  Support for control communication
   -  Support for automatic processing delay detection and compensation
   -  Support for multiple encoders connected via daisy-chain configuration (up to 3 encoders)
   -  Support for concurrent multi-channel support on a single PRU (up to 3 identical encoders)
   -  Support for multi-channel encoders of different make under load share mode (Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details)
   -  Support for up to 100 meter cable
   -  Readiness for BiSS Safety profile by supporting 16 bit CRC and sign-of-life counter

\endcond

\cond SOC_AM261X

   -  BiSS-C interface receiver for point-to-point communication
   -  Support for single channel implementation with one encoder
   -  Receive on-the-fly CRC verification of position and control data
   -  Interface speed of 1, 2, 5, 8, and 10 MHz
   -  Support for oversampling ratio with different interface speeds
<table>
<tr>
    <th rowspan="2">Clock Source
    <th rowspan="1" colspan="5">Interface Speed
</tr>
<tr>
    <th>1 MHz
    <th>2 MHz
    <th>5 MHz
    <th>8 MHz
    <th>10 MHz
</tr>
<tr>
    <td>PRU UART Clock (160 MHz)
    <td>8x
	<td>8x
    <td>8x
	<td>4x
    <td>8x
</tr>

</table>
   -  Two modes of operation - host trigger and periodic trigger
   -  Support for control communication
   -  Support for automatic processing delay detection and compensation
   -  Support for multiple encoders connected via daisy-chain configuration (up to 3 encoders)
   -  Support for concurrent multi-channel support on a single PRU (up to 3 identical encoders)
   -  Support for up to 100 meter cable
   -  Readiness for BiSS Safety profile by supporting 16 bit CRC and sign-of-life counter

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

   -  BiSS-C interface receiver for point-to-point communication
   -  Support for single channel implementation with one encoder
   -  Receive on-the-fly CRC verification of position and control data
   -  Interface speed of 1, 2, 5, 8, and 10 MHz
   -  Support for oversampling ratio with different interface speeds
<table>
<tr>
    <th rowspan="2">Clock Source
    <th rowspan="1" colspan="5">Interface Speed
</tr>
<tr>
    <th>1 MHz
    <th>2 MHz
    <th>5 MHz
    <th>8 MHz
    <th>10 MHz
</tr>
<tr>
    <td>PRU UART Clock (192 MHz)
    <td>8x
	<td>8x
    <td>Not tested
	<td>8x
    <td>Not tested
</tr>
<tr>
    <td>PRU Core Clock (200 MHz)
    <td>Not tested
	<td>Not tested
    <td>8x
	<td>Not tested
    <td>4x
</tr>

</table>
   -  Two modes of operation - host trigger and periodic trigger
   -  Support for control communication
   -  Support for automatic processing delay detection and compensation
   -  Support for multiple encoders connected via daisy-chain configuration (up to 3 encoders)
   -  Support for concurrent multi-channel support on a single PRU (up to 3 identical encoders)
   -  Support for up to 100 meter cable
   -  Readiness for BiSS Safety profile by supporting 16 bit CRC and sign-of-life counter

\endcond

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported in this release, including the below:
-  BISS Line
-  Independent clocks on multi channel mode.

## SysConfig Features

\cond SOC_AM243X

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSSG instance. (Tested on ICSSG0)
- Selecting the ICSSG PRU slice. (Tested on ICSSG0-PRU1)
- Configuring PINMUX.
- Frequency selection.
- Channel selection.
- Selecting Multi Channel with encoders of different make using load share mode.
- Enabling SA Mux mode
- Selecting clock source

\note BiSS-C firmware supports operation with ICSS Core Clock running at 200 MHz/300 MHz frequency or ICSS UART Clock running at 192 MHz only. ICSS Core Clock at 225/250/333 MHz is not supported due to clock divider requirements.
\endcond

\cond SOC_AM261X

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSS instance. (Tested on ICSSM1)
- Selecting the ICSS PRU slice. (Tested on ICSSM1-PRU0)
- Configuring PINMUX.
- Frequency selection.
- Selecting clock source

\note BiSS-C firmware supports operation with ICSS UART Clock running at 160 MHz only, when ICSS Core Clock is 225 MHz due to clock divider requirements.

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSS instance. (Tested on ICSSM)
- Selecting the ICSS PRU slice. (Tested on ICSSM-PRU0)
- Configuring PINMUX.
- Frequency selection.
- Selecting clock source

\note BiSS-C firmware supports operation with ICSS Core Clock running at 200 MHz frequency or ICSS UART Clock running at 192 MHz frequency only.

\endcond

## ICSS PRU Resource Usage
\cond SOC_AM243X
<table>
<tr>
    <th> Configuration
    <th> PRU Core
    <th> Memory Usage
    <th> IEP Usage
    <th> Other Peripheral Usage
    <th> Description
</tr>
<tr>
    <td> Single channel
    <td> PRUx
    <td> DMEM: 272 Bytes <br>  IMEM: 3080 Bytes
	<td> IEP0: CMP0 and CMP3
    <td> INTC event/input number 18 (pr[0/1]_pru_mst_intr[2]_intr_req) is used to trigger a R5 interrupt </td>
    <td> IEP, CMP events and INTC signal are used only in periodic continuous mode.
</tr>
<tr>
    <td> Multi-channel with single PRU core
    <td> PRUx
    <td> DMEM: 272 Bytes <br>  IMEM: 3380 Bytes
	<td> IEP0: CMP0 and CMP3
    <td> INTC event/input number 18 (pr[0/1]_pru_mst_intr[2]_intr_req) is used to trigger a R5 interrupt </td>
    <td> IEP, CMP events and INTC signal are used only in periodic continuous mode.
</tr>
<tr>
    <td rowspan="3"> Multi-channel with load share across 3 PRU cores
    <td> PRUx
    <td rowspan="3"> DMEM: 272 Bytes <br>  IMEM: 3484 Bytes
	<td rowspan="3"> IEP0: CMP0, CMP3, CMP5 and CMP6 </td>
    <td rowspan="3"> INTC event/input number 18, 19 and 20 (pr[0/1]_pru_mst_intr[2/3/4]_intr_req) is used to trigger a R5 interrupt </td>
    <td rowspan="3"> IEP, CMP events and INTC signals are used only in periodic continuous mode.</td>
</tr>
<tr>
    <td> RTU_PRUx
</tr>
<tr>
    <td> TX_PRUx
</tr>
</table>

\note For pin usage, see \ref BISSC_PIN_USAGE section.

\endcond

\cond (SOC_AM261X || SOC_AM263X || SOC_AM263PX)
<table>
<tr>
    <th> Configuration
    <th> PRU Core
    <th> Memory Usage
    <th> IEP Usage
    <th> Other Peripheral Usage
    <th> Description
</tr>
<tr>
    <td> Single channel
    <td> PRUx
    <td> DMEM: 272 Bytes <br>  IMEM: 3080 Bytes
	<td> IEP0: CMP0 and CMP3
    <td> INTC event/input number 18 (pr[0/1]_pru_mst_intr[2]_intr_req) is used to trigger a R5 interrupt </td>
    <td> IEP, CMP events and INTC signal are used only in periodic continuous mode.
</tr>
</table>
\note For pin usage, see \ref BISSC_PIN_USAGE section.

\endcond

## BISS-C Design

\subpage BISSC_DESIGN explains the design in detail.

## Example
\ref EXAMPLE_MOTORCONTROL_BISSC

## API
\ref BISSC_API_MODULE
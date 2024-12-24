# EnDat {#ENDAT}

[TOC]

## Introduction

EnDat is a bidirectional interface for position encoders. During EnDat operation the EnDat receiver receives position information from the EnDat position encoder.

## Features Supported

   -  EnDat 2.2 command set
   -  EnDat 2.1 command set
   -  Interrupted and continuous clock mode
   -  Cable length up to 100m @8MHz
   -  Propagation delay compensation  \if (SOC_AM243X ||SOC_AM64X) (capable of handling different propagation delay of different
      propagation delay of different channels in concurrent multi channel configuration) \endif
   -  Automatic estimation of propagation delay
   -  Receive on-the-fly CRC verification of position, parameters and additional information
   -  Two modes of operation - host trigger and periodic trigger
   -  Channel select
\cond SOC_AM243X || SOC_AM64X
   -  Concurrent multi channel support (up-to 3 encoders with identical part number @ 8MHz maximum)
   -  "Multi Channel with Encoders of Different Make" using load share mode (Each of PRU, RTU-PRU, and TX-PRU from one PRU-ICSSG slice handles one channel)
\endcond
   -  Safety Readiness: Recovery time
   -  Clock up to 16MHz with single channel \if (SOC_AM243X || SOC_AM64X) and load share mode (multi channel) \endif
      \note Receive (Rx) is oversampled at 8x of send(Tx). Therefore, the encoder interface frequency "f" should such that Tx source clock value is divisible by "f" and Rx source clock value is divisible by "8*f".
   - Possible interface speeds with different source clock combinations.
<table>
<tr>
   <th>Clock Source
   <th>Interface Speed
</tr>
\cond SOC_AM261X
<tr>
   <td> PRU UART Clock (160 MHz)
   <td> 1MHz, 2MHz, 4MHz, 5MHz, 10MHz
</tr>
<tr>
   <td>PRU Core Clock (200 MHz)
   <td> 1MHz, 5MHz
</tr>
\endcond
\cond (SOC_AM263X || SOC_AM263PX)
<tr>
   <td> PRU UART Clock (160 MHz)
   <td> 1MHz, 2MHz, 4MHz, 5MHz, 10MHz
</tr>
<tr>
   <td>PRU Core Clock (200 MHz)
   <td> 1MHz, 5MHz
</tr>
\endcond
\cond SOC_AM243X || SOC_AM64X
<tr>
   <td> PRU UART Clock (192 MHz)
   <td> 1MHz, 2MHz, 4MHz, 6MHz, 8MHz, 12MHz, 16MHz
</tr>
<tr>
   <td>PRU Core Clock (200 MHz)
   <td> 1MHz, 5MHz
</tr>
\endcond

</table>

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported in this release, including the below
-  Independent clocks on multi channel mode.
\cond SOC_AM243X || SOC_AM64X
-  Continuous clock mode in Multi-channel single PRU mode
\endcond
### Limitations
\cond SOC_AM243X || SOC_AM64X
This section describes known limitations of the current implementation in multi-channel single PRU mode.
- Clock above 8 MHz: it is not possible to over sample, downsample and store one bit for all three channels in one clock cycle time.
- Reset command CRC failure: The encoder which takes more time in reset operation will show CRC failure because the reset time is not the same for each encoder so the acknowledgment will not arrive on same time for all encoders at the master end.
\endcond
## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSS instance.
\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X)
- Selecting the ICSSM PRU slice. (\if (SOC_AM263X || SOC_AM263PX) Tested on ICSSM-PRU0 \else Tested on ICSSM1-PRU0 \endif)
\else
- Selecting the ICSSG PRU slice. (Tested on ICSSG0-PRU1)
\endif
- Configuring PINMUX.
- Channel selection.
\cond SOC_AM243X || SOC_AM64X
- Selecting Multi Channel with Encoders of Different Make" using load share mode.
- Enabling SA Mux mode
\endcond
- Selecting RX and TX source clock


## ICSS PRU Resource Usage
<table>
<tr>
   <th>Configuration</th>
   <th>PRU Core</th>
   <th>Memory Usage</th>
   <th>IEP Usage</th>
   <th>Other Peripheral Usage</th>
   <th>Description</th>
</tr>
<tr>
   <td>Single Channel Configuration</td>
   <td>PRUx</td>
   <td>DMEM: 152 Bytes <br>  IMEM: 5 KB </td>
   <td>IEP0: CMP0 and CMP3 </td>
   <td>INTC Signal No. 18 is used to trigger a R5 interrupt </td>
   <td>IEP, CMP events and INTC signal are used only in periodic continuous mode.</td>
</tr>
\cond SOC_AM243X || SOC_AM64X
<tr>
   <td>Multi Channel Configuration with single PRU core</td>
   <td>PRUx</td>
   <td>DMEM: 152 Bytes <br>  IMEM: 6.2 KB </td>
   <td>IEP0: CMP0 and CMP3 </td>
   <td>INTC Signal No. 18 is used to trigger a R5 interrupt</td>
   <td>IEP, CMP events and INTC signal are used only in periodic continuous mode <br> \note Multi-Channel single PRU firmware requires more than 6KB IMEM, so it cannot run on TX PRU
    </td>
</tr>
<tr>
   <td>Multi Channel Configuration with load share</td>
   <td>PRUx, TXPRUx, RTUPRUx</td>
   <td>DMEM: 152 Bytes <br>  PRU_IMEM: 4 KB <br>  RTU_IMEM: 4 KB <br>  TX_IMEM: 4 KB</td>
   <td>IEP0: CMP0, CMP3, CMP5 and CMP6 </td>
   <td>INTC Signal No. 18, 19 and 20 are used to trigger R5 interrupts.</td>
   <td>IEP, CMP events and INTC signals are used only in periodic continuous mode.</td>
</tr>
\endcond
</table>

\note For pin usage, see \ref ENDAT_PIN_USAGE section.

## ENDAT Design

\subpage ENDAT_DESIGN explains the design in detail.

## Example
\ref EXAMPLE_MOTORCONTROL_ENDAT

## API
\ref ENDAT_API_MODULE


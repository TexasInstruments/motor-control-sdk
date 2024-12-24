# HDSL Protocol Design {#HDSL_DESIGN}

[TOC]

## Introduction
This document presents the firmware implementation details of the Hiperface DSL protocol (SICK STEGMANN, 2010).
\cond SOC_AM243X
## System Overview

### Sitara™ AM64x/AM243x Processor

Refer TRM for details

#### PRU-ICSS

Refer PRU-ICSS chapter of AM64x/AM243x Technical Reference Manual
\endcond

\cond SOC_AM261X
## System Overview

### Sitara™ AM261x Processor

Refer TRM for details

#### PRU-ICSS

Refer PRU-ICSS chapter of AM261x Technical Reference Manual
\endcond

## Software Architecture

The Hiperface DSL function is implemented on one instance of PRU-ICSS and leave the other PRU-ICSS instance for Industrial Ethernet functions.

The firmware consists of two layers

1. Datalink Layer : It is responsible for establishing a communication link to the encoder, monitoring the connection quality and preparing the data. It assembles the information from the different channels and puts the data symbol by symbol to the channel buffers. The channel buffers are large enough to carry the data of a whole vertical frame for each channel.
2. Transport Layer :  It processes the data and determines what information is sent over the parameter channel. It controls the data sent over the parameter channel by setting the symbol to send for the next horizontal frame in the parameter channel buffer. This buffer can carry only one symbol.

Both layers have direct access to the register interface that is provided to the higher layers.

Figure "Layer Model" illustrates the relationship between the two layers.


\image html hdsl_layer_model.png "Layer Model"
\cond SOC_AM243X

### Overlay Scheme for TX-PRU {#HDSL_DESIGN_TXPRU_OVERLAY}

Each PRU-ICSS instance has two slices, and each slice has three cores : PRU, RTU-PRU and TX-PRU. The instruction memory for PRU, RTU-PRU and TX-PRU coreS is 12 kB, 8 kB and 6 kB respctively. Multi-channel implementation of Hiperface DSL is achieved by enabling load share mode of PRU-ICSS where one core is responsible for one channel. One PRU-ICSS slice supports three peripheral interfaces for HDSL. Mapping is fixed to channel 0 with RTU-PRU, channel 1 with TX-PRU. To implement an equivalent data link layer and transport layer as the reference IP-core for the Hiperface DSL on FPGA, the instruction memory for TX-PRU is not enough. Hence a code overlay scheme is required only for TX-PRU core, which is only needed if channel 2 is enabled.

For PRU and RTU-PRU, the firmware for Hiperface DSL fully fits into instruction memory. The firmware for TX-PRU is split into following three code sections based on initialization and normal operation:

1. Initialization specific code
2. Normal operation code
3. Common code needed during initialization and normal operation

Part 3 is loaded directly into instruction memory (IMEM) of TX-PRU by ARM core as it will be needed in all states. Part 1 and Part 2 of firmware for TX-PRU are stored in PRU-ICSS Data Memory (DMEM) by ARM core. During initialization (LOADFW1 state shown in next section), part 1 is copied into instruction memory (IMEM) of TX-PRU from Data Memory (DMEM) by RTU-PRU core. After initialization is complete (LOADFW2 state shown in next section), part 2 is copied into instruction memory (IMEM) of TX-PRU from Data Memory (DMEM) by RTU-PRU core.

\endcond
### State Machine

Hiperface DSL specifies a state machine for the Receiver. This implementation features two additional states for loading firmware to the TX-PRU from RTU-PRU. Figure "State Machine" depicts the modified state machine.

\image html hdsl_state_machine.png "State Machine"

### Datalink Layer

The datalink layer is responsible for handling the communication link to the encoder. This includes the sampling, cable delay compensation, DC line balancing, encoding and decoding of data and the monitoring of the connection quality.

#### Sampling
During the reception of the salve answer, the SCU oversamples the data by factor 8. This allows the firmware to compensate signal deficits, such as delay. During the LEARN state the receiver calculates the sample edge based on the first received bit.
Assuming the oversampled data is exactly aligned with one bit, the best position for the sample edge would be either bit 3 or bit 4. An unalignment of the oversampled data with the actual bit results in a shift of the sample edge. The unalignment can be measured by counting the number of ‘1’ in the data, whereas a count of 4 equals the worst alignment and a count of 0 or 8 equals perfect alignment. The number of ’1’ (n) in the oversampled data is determined using a LUT and the following calculation provides the position for the sample edge (E):

E=(4+n)%8

\image html hdsl_sampling.png "Sampling"

#### Delay Measurement and Compensation
During the LEARN state the encoder sends a test pattern to the receiver. This is used to determine the cable delay. While the test pattern is sent, the receiver records all incoming bits and searches for the beginning of the test pattern. The offset, where the test pattern starts, is the cable delay in units of bits.

\image html hdsl_test_pattern.png "Test Pattern"

After the cable delay is measured, the receiver uses this knowledge to compensate the cable delay in subsequent states. This is performed by waiting for the calculated amount of bits as soon as the encoder answer window starts. The next bit on the line is the first bit of the actual encoder answer.

#### Data Encoding and Decoding
The datalink layer has the responsibility to decode and encode the data according to an 8b/10b scheme (Franaszek, 1983). The 8b/10b encoding/decoding is split into two parts, 3b/4b and 5b/6b encoding/decoding. Each of the encoding and decoding processes is performed by using a LUT. Hiperface DSL assumes a transmission with LSB first. Therefore, in the encoding procedure, the index of the LUT is in MSB first order, while the LUT entries are in LSB first order (and vice versa when decoding data).  This way, the firmware does not need to handle the reversing of the bit order. When encoding the data, the firmware handles the sending of the correct polarity of the sub-blocks using the measured line disparity.
During receive, the firmware detects byte errors and special characters by checking the received encoded data according to the paper (Franaszek, 1983)

#### Received Signal Strength Indication (RSSI)

The RSSI is calculated by determining the number of samples between two edges during a bit period. The samples that form the longest sequence between two edges represent the stable bit period, which is used to calculate the RSSI. Instead of calculating the stable period in the firmware, a pre-calculated LUT is utilized to speed up the process. First, the edges in a bit period are determined, which is performed by a XOR operation (Figure: hdsl_rssi)]. The searched RSSI value is looked-up in the table by using the result of the XOR operation as the index.

\image html hdsl_rssi.png "Test Pattern"


#### Cyclic Redundancy Check Algorithm

A 16bit CRC verification of the data is used on multiple occasions. It is used for the vertical channel, secondary channel and messages. In order to distribute the computation load equally over all H-Frames, the firmware calculates a running CRC for those data (except for short messages). The algorithm uses a LUT with 256 entries and 2 bytes per entry, whereas each entry is the 16bit CRC for the corresponding LUT index. The basic approach for the calculation of the 16bit CRC is shown as C code in the following:

	uint16_t calc_crc(uint8_t *data, uint32_t size)
	{
		uint16_t crc = 0;
		uint32_t i;
		for(i = 0; i < size; ++i)
		{
			crc = ((*data) << 8) ^ crc;
			crc = lut[crc>>8] ^ (crc << 8);
		}
		return (crc ^ 0xff);
	}

### Transport Layer

The transport layer processes the channel information which was prepared by the datalink layer. This includes the calculation of the fast position as well as the handling of messages.

#### ∆∆Position Estimation
During normal workflow, it can occur that the received ∆∆Position data cannot be used for calculations. This is the case on either a transmission error or an internal encoder error. In order to check for a transmission error, the transport layer checks if the datalink layer detected a byte error and verifies the CRC in the acceleration channel. If no transmission error occurred, the transport layer searches for the occurrence of two K29.7 to recognize an internal encoder error. In case one of the verification of the data fails, the estimation algorithm shown in Figure

\image html hdsl_delta_pos.png "Estimation Algorithm for ∆∆Position"


## Messages
The transport layer handles the messaging. Since it is possible that the higher layers send a long and a short message at the same time, the transport layer has to decide which message to send first. In this implementation short messages are always favored over long messages.


### Short Message
Remote (DSL motor feedback system) registers that indicate interface information are mirrored in the DSL Receiver under register addresses 40h to 7Fh. These remote registers are addressed in the same way as DSL Receiver registers. As the values of remote registers are transmitted via the Parameters Channel and hence via the DSL cables, the delay between polling and answer for "short message" transactions depends on the connection cables of the systems in question. There is no delay, as this information is stored directly in the IP-Core S_PC_DATA.. The Parameters Channel can only transmit one "short message" at a time. Several remote registers can only be polled in sequence, i.e. after the previous answer has been received.

Note: It should be noted that a "short message" can be triggered during a running "long message" transaction.

## Synchronization with External Pulse {#HDSL_DESIGN_SYNC}
According to the Hiperface DSL specification, the falling edge inside the EXTRA window should coincide with the external synchronization pulse.
At the beginning of the startup phase, the firmware measures the time interval of the external pulse and calculates the required number of bits for the H-Frame.
Based on this number the stuffing length and EXTRA window size is derived.
Afterwards, the PRU waits to match its timing with the timing of the external synchronization pulse and starts the transmission.
Since it is possible to use time intervals for the external pulse that are not multiples of the bit duration, the firmware needs to adjust the H-Frame size on the fly.
Furthermore, during the EXTRA window the PRU transmits the data (sample edge) with a granularity of 13.3ns to increase the synchronization accuracy. Figure "Synchronization of External Pulse with Sample Edge in EXTRA Window" and "Illustration of Synchronization Algorithm" depict the concept.
The EXTRA_TIME_WINDOW is a fixed value that is calculated at startup to match the external pulse frequency. The TIME_REST value gives the number of overclocked ‘1’ that needs to be sent during the last bit of the EXTRA window.

\imageStyle{hdsl_external_sync.png,width:40%}
\image html hdsl_external_sync.png "Synchronization of External Pulse with Sample Edge in EXTRA Window"

In other words, the TIME_REST value represents the sample edge in a fine granularity dimension (13.3ns). While the sample edge can be send with a finer granularity, the granularity of the size of the EXTRA window is still in whole bit durations (106.67ns).
Consequently, there is an overhead, if the external pulse period is not a multiple of the bit duration. This overhead is compensated in the next H-Frame by changing the size of the EXTRA window. As a result, the size of the H-Frame is varying over time.
It is possible that these calculations lead to the excess of the maximum or minimum EXTRA window size. Therefore, the number of bits for the stuffing and EXTRA window is readjusted on a violation.

\imageStyle{hdsl_sync_algo.png,width:40%}
\image html hdsl_sync_algo.png "Illustration of Synchronization Algorithm"

The algorithm is given as C code in the following:

	/* EXTRA_SIZE equals the number of bits for the EXTRA window minus 1 */
	if(EXTRA_EDGE == 0)
	{
		TIME_REST += 8;
	}

	short b = (EXTRA_SIZE << 3) + TIME_REST;
	short overhead = (EXTRA_SIZE << 3) + 8 - TIME_EXTRA_WINDOW;
	EXTRA_SIZE = (b - overhead) >> 3;
	TIME_REST = (b - overhead) - (EXTRA_SIZE << 3);

	if(EXTRA_SIZE < 3)
	{
		EXTRA_SIZE += 6;
		NUM_STUFFING -= 1;
		TIME_EXTRA_WINDOW += (8*6);
	}
	if(EXTRA_SIZE > 8)
	{
		EXTRA_SIZE -= 6;
		NUM_STUFFING += 1;
		TIME_EXTRA_WINDOW -= (8*6);
	}


EXTRA_EDGE represents the TIME_REST value in a format that can be pushed to the TX FIFO for transmission. For instance, if TIME_REST is 4, EXTRA_EDGE is 0xf0. The edge would be in the middle of the bit duration. The value NUM_STUFFING gives the number of stuffing blocks (each block consist of 6 bits).


For further improvement of the synchronization, the time difference (∆t) between the external pulse and the sample edge we transmit is measured (Figure "Time difference between External Pulse and Sample Edge").

\imageStyle{hdsl_external_sync_sample_edge.png,width:40%}
\image html hdsl_external_sync_sample_edge.png "Time difference between External Pulse and Sample Edge"

## Pin Multiplexing {#HDSL_PIN_USAGE}

\note
    - k = 0,1 (PRU-ICSS Instance) for AM243x/AM261x/AM64x
    - n = 0,1 (PRU-ICSS Slice)

<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPO0 \else PRG<%k>_PRU<n>_GPO0 \endif
    <td>pru<n>_hdsl0_clk
	<td>Channel 0 clock
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPO1 \else PRG<%k>_PRU<n>_GPO1 \endif
    <td>pru<n>_hdsl0_out
	<td>Channel 0 transmit
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPO2 \else PRG<%k>_PRU<n>_GPO2 \endif
    <td>pru<n>_hdsl0_outen
	<td>Channel 0 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPI9 \else PRG<%k>_PRU<n>_GPI13/PRG<%k>_PRU<n>_GPI9 \endif
    <td>pru<n>_hdsl0_in
	<td>Channel 0 receive
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPO3 \else PRG<%k>_PRU<n>_GPO3 \endif
    <td>pru<n>_hdsl1_clk
	<td>Channel 1 clock
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPO4 \else PRG<%k>_PRU<n>_GPO4 \endif
    <td>pru<n>_hdsl1_out
	<td>Channel 1 transmit
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPO5 \else PRG<%k>_PRU<n>_GPO5 \endif
    <td>pru<n>_hdsl1_outen
	<td>Channel 1 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPI10 \else PRG<%k>_PRU<n>_GPI14/PRG<%k>_PRU<n>_GPI10 \endif
    <td>pru<n>_hdsl1_in
	<td>Channel 1 receive
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPO6 \else PRG<%k>_PRU<n>_GPO6 \endif
    <td>pru<n>_hdsl2_clk
	<td>Channel 2 clock
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPO7 \else PRG<%k>_PRU<n>_GPO12/PRG<%k>_PRU<n>_GPO7 \endif
    <td>pru<n>_hdsl2_out
	<td>Channel 2 transmit
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPO8 \else PRG<%k>_PRU<n>_GPO8 \endif
    <td>pru<n>_hdsl2_outen
	<td>Channel 2 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM261X) PR<%k>_PRU<n>_GPI11 \else PRG<%k>_PRU<n>_GPI11 \endif
    <td>pru<n>_hdsl2_in
	<td>Channel 2 receive
</tr>
</table>

\cond SOC_AM243X
### LP-AM243 Booster Pack Pin Multiplexing
<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>PRG0_PRU1_GPO0
    <td>pru1_hdsl0_clk
	<td>Channel 0 clock
</tr>
<tr>
    <td>PRG0_PRU1_GPO1
    <td>pru1_hdsl0_out
	<td>Channel 0 transmit
</tr>
<tr>
    <td>PRG0_PRU1_GPO2
    <td>pru1_hdsl0_out_en
	<td>Channel 0 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI9
    <td>pru1_hdsl0_in
	<td>Channel 0 receive (if(G_MUX_EN==0))
</tr>
<tr>
    <td>PRG0_PRU1_GPI13
    <td>pru1_hdsl0_in
	<td>Channel 0 receive (if(G_MUX_EN==1))
</tr>
<tr>
    <td>PRG0_PRU1_GPO6
    <td>pru1_hdsl2_clk
	<td>Channel 2 clock
</tr>
<tr>
    <td>PRG0_PRU1_GPO12
    <td>pru1_hdsl2_out
	<td>Channel 2 transmit
</tr>
<tr>
    <td>PRG0_PRU1_GPO8
    <td>pru1_hdsl2_out_en
	<td>Channel 2 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI11
    <td>pru1_hdsl2_in
	<td>Channel 2 receive
</tr>
<tr>
    <td>GPIO Pin(GPIO1_78)
    <td>ENC0_EN
    <td>Enable 3 channel peripheral interface mode in Axis 1 of BP (C16 GPIO pin)
</tr>
<tr>
    <td>GPIO Pin(GPIO1_77)
    <td>ENC2_EN
    <td>Enable 3 channel peripheral interface mode in Axis 2 of BP (B17 GPIO pin)
</tr>
</table>

\endcond

\cond  SOC_AM261X
### LP-AM261 Booster Pack Pin Multiplexing
<table>
<tr>
    <th>Pin name
    <th>Signal name
    <th>Function
</tr>
<tr>
    <td>PR1_PRU0_GPIO0
    <td>pru1_hdsl0_clk
    <td>Channel 0 clock
</tr>
<tr>
    <td>PR1_PRU0_GPIO1
    <td>pru1_hdsl0_out
    <td>Channel 0 transmit
</tr>
<tr>
    <td>PR1_PRU0_GPIO2
    <td>pru1_hdsl0_out_en
    <td>Channel 0 transmit enable
</tr>
<tr>
    <td>PR1_PRU0_GPI9
    <td>pru1_hdsl0_in
    <td>Channel 0 receive
</tr>
<tr>
    <td>GPIO Pin (GPIO_21)
    <td>ENC0_EN
    <td>Enable 3 channel peripheral interface in Axis 1 of BP (B10 GPIO pin)
</tr>
</table>
\endcond
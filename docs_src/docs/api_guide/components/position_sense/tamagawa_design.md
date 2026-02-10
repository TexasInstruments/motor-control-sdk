# Tamagawa Protocol Design {#TAMAGAWA_DESIGN}

[TOC]

## Introduction

This document presents the firmware implementation details of the Tamagawa receiver protocol.

## Tamagawa encoder receiver

It is an encoder technology used for obtaining high-precision position information in machine tools, robotics, and so forth. Tamagawa rotary encoders consist broadly of two types: incremental or absolute. Incremental encoders provide a train of pulses, while the absolute-type provides digital values. The absolute encoder group contains the single-turn types that provide outputs which can be open collector or emitter follower. The absolute encoder types include the pure digital encoder types, which provide a digital word output through a line driver such as an RS485, or a semi-absolute encoder, which provides both digital word and pulse train outputs. Of the RS485 line-driver output absolute encoders that provide only digital output, another classification is the full absolute encoder. A full absolute encoder provides multi-turn digital data, which is known as SmartAbs, and is compatible with the Tamagawa Smartceiver AU5561N1. Another type of encoders, known as SmartInc, provide single-turn information in digital format with an RS485 line driver output. The @VAR_SOC_NAME Tamagawa receiver implementation is equivalent to the Smartceiver AU5561N1, which can communicate with Tamagawa SmartAbs as well as SmartInc encoders.

The @VAR_SOC_NAME Tamagawa receiver communicates with Tamagawa SmartAbs and SmartInc encoders and provides drive control with digital information to and from the encoder. Tamagawa communication is broadly classified into three types: data readout, reset, and EEPROM transactions. Four data readout transactions occur: absolute data in one revolution, multi-turn data, encoder ID, and a combination of all of these along with the encoder error status. The reset transaction always returns the absolute data in one revolution while performing different types of resets. Three types of reset are available: reset of absolute data in one revolution, reset of multi-turn data, and error reset. The EEPROM transaction allows the system to read and write to the EEPROM in the encoder. Each transaction has a unique data ID and consists of different fields, namely control, status, data, cyclic redundancy check (CRC), EEPROM address, and EEPROM data depending on the type of transaction, that is, data ID.

Each field is 10-bits long, beginning with a start bit and ending with a delimiter. The 8 bits between these start bits and delimiters depend on the field type. The control field contains the data ID information. Data, status, and CRC fields similarly contain data, status, and CRC in those 8 bits. The receiver initially sends the control field to start the communication. This action indicates the type of transaction to the encoder and the encoder returns this information based on the data ID, as the previous paragraph explains. The encoder always returns the control field back to the receiver. In the case of data readout and reset transactions, the encoder returns the control field followed by the status, data, and ending with the CRC field at the end. In the case of an EEPROM read or write, the receiver, in addition to the control field, sends the EEPROM address field (and EEPROM data field for write) followed by the CRC. The encoder returns the control field, followed by the EEPROM address, EEPROM data, and CRC fields. The physical layer communication is RS422/RS485 based.

## System Overview

\cond SOC_AM243X

### Sitara™ AM64x/AM243x Processor

Refer to TRM for details

\endcond

\cond SOC_AM261X

### Sitara™ AM261x Microcontroller

Refer to TRM for details

\endcond

\cond SOC_AM263X

### Sitara™ AM263x Microcontroller

Refer to TRM for details

\endcond

\cond SOC_AM263PX

### Sitara™ AM263Px Microcontroller

Refer to TRM for details

\endcond

#### PRU-ICSS

Refer to PRU-ICSS chapter of @VAR_SOC_NAME Technical Reference Manual

## Software Description

\if (SOC_AM243X || SOC_AM64X)
At start-up, the application running on the Arm®-based core initializes the module clocks and configures the pinmux. The PRU is initialized and the PRU firmware is loaded on PRU slice of choice for a chosen ICSS instance (tested on PRU1 on ICSSG0).
\endif

\if (SOC_AM263X || SOC_AM263PX)
At start-up, the application running on the Arm®-based core initializes the module clocks and configures the pinmux. The PRU is initialized and the PRU firmware is loaded on PRU slice of choice for a chosen ICSS instance (tested on PRU0 on ICSSM).
\endif

\if (SOC_AM261X)
At start-up, the application running on the Arm®-based core initializes the module clocks and configures the pinmux. The PRU is initialized and the PRU firmware is loaded on PRU slice of choice for a chosen ICSS instance (tested on PRU0 on ICSSM1).
\endif

After the PRU starts executing, the Tamagawa interface is operational and the application can use it to communicate with an encoder. Use the Tamagawa diagnostic example to learn more about initialization and communication with the Tamagawa interface. This Tamagawa diagnostic example also provides an easy way to validate the Tamagawa transactions. The diagnostic example provides menu options on the host PC in a serial terminal application, where the user can select the data ID code to be sent. Based on the data ID code, the application updates the Tamagawa interface with the data ID code and triggers transaction. The application then waits until it receives an indication of complete transaction by the firmware through the interface before displaying the result.

### Firmware Architecture

\cond SOC_AM243X

Deterministic behavior of the 32 bit RISC core running up to 333 MHz provides resolution on sampling external signals and generating external signals. It makes use of 3 channel peripheral interface support in PRU for data transmission/reception.

The PRU-ICSS firmware supports the following configurations:
1. Single Channel per PRU slice
2. Multi Channel with encoders of same make per PRU slice
3. Multi Channel with encoders of different make under load share mode per PRU slice

\endcond

\cond SOC_AM261X

Deterministic behavior of the 32 bit RISC core running up to 225 MHz provides resolution on sampling external signals and generating external signals. It makes use of 3 channel peripheral interface support in PRU for data transmission/reception.

The PRU-ICSS firmware supports the following configuration:
1. Single Channel per PRU slice

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

Deterministic behavior of the 32 bit RISC core running up to 200 MHz provides resolution on sampling external signals and generating external signals. It makes use of 3 channel peripheral interface support in PRU for data transmission/reception.

The PRU-ICSS firmware supports the following configuration:
1. Single Channel per PRU slice

\endcond

#### Tamagawa Firmware Flow {#TAMAGAWA_DESIGN_FLOW}

The firmware first initializes the PRU hardware. Then it checks the operation mode: host trigger mode, periodic CMP mode, or periodic CAP mode.

**Host Trigger Mode:** The firmware waits until a command has been triggered through the interface by the host application.

**Periodic CMP/CAP Mode:** The firmware monitors the configured IEP compare/capture event and sets the host trigger bit when the event occurs, automatically initiating Tamagawa transactions at regular intervals.

Upon triggering (from any mode), the transmit data is set up based on the data ID code and the data is transmitted. The data ID code then waits until receiving all the data that depends on the data ID. The parsing over the received data then commences, which is again based on the data ID, and the interface is updated with the result. The CRC verification occurs next and the interface indicates command completion. The firmware then waits for the next command trigger from the interface or IEP compare/capture event.

\image html Tamagawa_flowchart.JPG "Overview Flow Chart"

### Initialization {#TAMAGAWA_DESIGN_INITIALIZATION}

PRU is set to 3 channel peripheral interface first. The entire EnDat configuration MMRs are cleared (CFG registers). Tx global reinit bit in R31 is set to put all channels in default mode. The clock source is selected. In Tx mode, the output data is read from the Tx FIFO at this 1x clock rate. In Rx mode, the input data is sampled at the Oversampling (OS) clock rate. Hence, Tx clock (1x clock) and Rx clock (Oversampling (OS) clock) are set up by selecting oversampling factor (x8). At the end of the initialization, status is updated and waits until trigger from user occurs for Tamagawa commands.

\image html Tamagawa_initialization_flow_chart.JPG "Initialization Flow Chart"

### Setup Transmit Data {#TAMAGAWA_DESIGN_TX}
The transmit and receive sizes are determined based on the data ID in the interface.

\image html Tamagawa_setup_tx_data.png "Setup Transmit Data Flow Chart"

### Transmit and Receive {#TAMAGAWA_DESIGN_TX_RX}

In the current implementation, the Transmit data is loaded into the Tx FIFO byte-wise. For data readout and reset commands, the requirement is to send 1 frame of 10 bits. So, 2 bytes of data are first loaded into the Tx FIFO and Tx frame size is set to 10 bits to send right data to encoder. Similarly, for EEPROM Read command, the requirement is to send 3 frames of 10 bits each, so 30 bits in total. For this, 4 bytes of data are first loaded into the Tx FIFO and then Tx frame size is set to 30 bits to send right data to encoder. This is done by using the Tx - Single Shot mode.

\image html Tamagawa_tx_flow_chart.png "Transmit Flow Chart for data readout, reset and EEPROM Read commands"

In case of EEPROM Write command, the requirement is to send 4 frames of 10 bits each - 40 bits in total. For this, 4 bytes of data are first loaded into the Tx FIFO and then transmission is started in Tx - Continuous FIFO loading mode. FIFO byte level is constantly monitored and the FIFO is reloaded with the last byte when the FIFO level reaches 3 bytes.

\image html Tamagawa_eeprom_write_flow_chart.png "Transmit Flow Chart for EEPROM Write command"

Once the transmission is complete, the encoder starts sending the data and the firmware copies the receive FIFO contents onto the receive buffer, individually, until all the data has been received.

\image html Tamagawa_rx_flow_chart.png "Receive Flow Chart"

### Receive Data Parse {#TAMAGAWA_DESIGN_RX}
Depending on the data ID used for initiating the transfer, the firmware parses the received data and copies it onto relevant fields in the interface, accordingly.

\image html Tamagawa_parse_data.png "Receive Data Parse Flow Chart"

### Verify CRC
The CRC is the last byte of the received data. The firmware then calculates the CRC of the received data excluding the last byte, compares it with the received CRC value, and updates the CRC status in the interface.

\image html Tamagawa_verify_crc.png "Verify CRC Flow Chart"

#### Pin Multiplexing {#TAMAGAWA_PIN_USAGE}

\attention \ref PRUICSS_PERIPHERAL_IF_MODE_SIGNAL_CONFIGURATION section has details on PRU pin functions in Peripheral IF mode

\note
    - k = 0,1 (PRU-ICSS Instance) for AM243x/AM261x/AM64x and k = 0 for AM263x/AM263Px
    - n = 0,1 (PRU-ICSS Slice)

<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO0 \else PRG<%k>_PRU<n>_GPO0 \endif
    <td>pru<n>_endat0_clk
	<td>Channel 0 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO1 \else PRG<%k>_PRU<n>_GPO1 \endif
    <td>pru<n>_endat0_out
	<td>Channel 0 transmit
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO2 \else PRG<%k>_PRU<n>_GPO2 \endif
    <td>pru<n>_endat0_outen
	<td>Channel 0 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI9 \else PRG<%k>_PRU<n>_GPI13/PRG<%k>_PRU<n>_GPI9 \endif
    <td>pru<n>_endat0_in
	<td>Channel 0 receive
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO3 \else PRG<%k>_PRU<n>_GPO3 \endif
    <td>pru<n>_endat1_clk
	<td>Channel 1 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO4 \else PRG<%k>_PRU<n>_GPO4 \endif
    <td>pru<n>_endat1_out
	<td>Channel 1 transmit
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO5 \else PRG<%k>_PRU<n>_GPO5 \endif
    <td>pru<n>_endat1_outen
	<td>Channel 1 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI10 \else PRG<%k>_PRU<n>_GPI14/PRG<%k>_PRU<n>_GPI10 \endif
    <td>pru<n>_endat1_in
	<td>Channel 1 receive
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO6 \else PRG<%k>_PRU<n>_GPO6 \endif
    <td>pru<n>_endat2_clk
	<td>Channel 2 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO7 \else PRG<%k>_PRU<n>_GPO12/PRG<%k>_PRU<n>_GPO7 \endif
    <td>pru<n>_endat2_out
	<td>Channel 2 transmit
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO8 \else PRG<%k>_PRU<n>_GPO8 \endif
    <td>pru<n>_endat2_outen
	<td>Channel 2 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI11 \else PRG<%k>_PRU<n>_GPI11 \endif
    <td>pru<n>_endat2_in
	<td>Channel 2 receive
</tr>
</table>
\cond SOC_AM243X

##### LP-AM243 + BP-AM2BLDCSERVO Booster Pack Pin Multiplexing for SDK example
<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>PRG0_PRU1_GPO0
    <td>pru1_endat0_clk
	<td>PRU1 Channel 0 clock
</tr>
<tr>
    <td>PRG0_PRU1_GPO1
    <td>pru1_endat0_out
	<td>PRU1 Channel 0 transmit
</tr>
<tr>
    <td>PRG0_PRU1_GPO2
    <td>pru1_endat0_outen
	<td>PRU1 Channel 0 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI13
    <td>pru1_endat0_in
	<td>PRU1 Channel 0 receive when SA mux selection is enabled (ICSSG_SA_MX_REG[7] G_MUX_EN = 1)
</tr>
<tr>
    <td>GPIO Pin (GPIO1_78/C16)
    <td>ENC0_EN
    <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
<tr>
    <td>PRG0_PRU1_GPO6
    <td>pru1_endat2_clk
	<td>PRU1 Channel 2 clock
</tr>
<tr>
    <td>PRG0_PRU1_GPO12
    <td>pru1_endat2_out
	<td>PRU1 Channel 2 transmit when SA mux selection is enabled (ICSSG_SA_MX_REG[7] G_MUX_EN = 1)
</tr>
<tr>
    <td>PRG0_PRU1_GPO8
    <td>pru1_endat2_outen
	<td>PRU1 Channel 2 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI11
    <td>pru1_endat2_in
	<td>PRU1 Channel 2 receive
</tr>
<tr>
    <td>GPIO Pin (GPIO1_77/B17)
    <td>ENC2_EN
    <td>Enable encoder voltage in Axis 2 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
</table>
\endcond

\cond  SOC_AM261X
##### LP-AM261 + BP-AM2BLDCSERVO Booster Pack Pin Multiplexing for SDK example
<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>PR1_PRU0_GPIO0
    <td>pru0_endat0_clk
	<td>PRU0 Channel 0 clock
</tr>
<tr>
    <td>PR1_PRU0_GPIO1
    <td>pru0_endat0_out
	<td>PRU0 Channel 0 transmit
</tr>
<tr>
    <td>PR1_PRU0_GPIO2
    <td>pru0_endat0_outen
	<td>PRU0 Channel 0 transmit enable
</tr>
<tr>
    <td>PR1_PRU0_GPI9
    <td>pru0_endat0_in
	<td>PRU0 Channel 0 receive
</tr>
<tr>
    <td>GPIO Pin (GPIO_21/B10)
    <td>ENC0_EN (PRU0)
    <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
<tr>
    <td>PR1_PRU1_GPIO0
    <td>pru1_endat0_clk
	<td>PRU1 Channel 0 clock
</tr>
<tr>
    <td>PR1_PRU1_GPIO1
    <td>pru1_endat0_out
	<td>PRU1 Channel 0 transmit
</tr>
<tr>
    <td>PR1_PRU1_GPIO2
    <td>pru1_endat0_outen
	<td>PRU1 Channel 0 transmit enable
</tr>
<tr>
    <td>PR1_PRU1_GPI9
    <td>pru1_endat0_in
	<td>PRU1 Channel 0 receive
</tr>
<tr>
    <td>GPIO Pin (GPIO_22/A10)
    <td>ENC0_EN (PRU1)
    <td>Enable encoder voltage in Axis 2 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
</table>
\endcond

\cond (SOC_AM263X || SOC_AM263PX)

##### @VAR_LP_BOARD_NAME + BP-AM2BLDCSERVO Booster Pack Pin Multiplexing for SDK example
<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>PR0_PRU0_GPIO3
    <td>pru0_endat1_clk
	<td>PRU0 Channel 1 clock
</tr>
<tr>
    <td>PR0_PRU0_GPO4
    <td>pru0_endat1_out
	<td>PRU0 Channel 1 transmit
</tr>
<tr>
    <td>PR0_PRU0_GPO5
    <td>pru0_endat1_outen
	<td>PRU0 Channel 1 transmit enable
</tr>
<tr>
    <td>PR0_PRU0_GPI10
    <td>pru0_endat1_in
	<td>PRU0 Channel 1 receive
</tr>
<tr>
    <td>GPIO Pin (SDFM0_D1/D13)
    <td>ENC1_EN
    <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
</table>
\endcond

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

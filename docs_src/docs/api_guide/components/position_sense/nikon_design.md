# Nikon Protocol Design {#NIKON_DESIGN}

[TOC]

\note A-format® is a registered trademark of the Nikon Corporation.

## Introduction

This design implements Nikon A-format Receiver using the 3 channel peripheral interface of PRU-ICSS available on the TI Sitara™ processors/microcontrollers. The 3 channel peripheral interface is a digital bidirectional serial interface for position encoders.
Transfer between receiver and encoder at the physical layer is in accordance with RS485, with transceivers at both ends.

## Nikon A-format encoder receiver

The Nikon A-format encoder receiver communicates with Nikon A-format encoders and provides drive control with digital information to and from the encoder. Nikon communication is broadly classified into five types: data readout, resets, encoders ID code/velocity coefficient, address assignment and EEPROM transactions. Five types of data readout transactions occur: absolute data in one revolution, multi-turn data, velocity, acceleration, encoder's temperature, encoder's status, encoder's ID and a combination of some of these. The reset transaction always returns the alarm field data while performing different types of resets. These commands need to be executed 8 times to perform the intended operation. Four types of reset are available: reset of single turn data in one revolution, reset of multi-turn data, encoder's status reset and changing encoder's address to a specified value. ID code assignment: These commands return the 24-bit ID code and there are three types of commands, to change encoder's address connected in bus by specifying 24-bit ID code of the encoder, to Read encoder's ID code and to write encoder's ID code. Similar to ID code, velocity coefficient can be read or written. The EEPROM transaction allows the system to read and write to the EEPROM in the encoder. Each transaction has a unique command code and consists of different fields, namely encoder address, status, data, cyclic redundancy check (CRC), ALM field, Identification code, encoder's temperature, EEPROM address, and EEPROM data depending on the type of transaction, that is, command code. Certain features are available in Nikon A-format version 3.0 only. Please refer to \ref NIKON_FEATURES for more details.

Each field is 18-bits long, beginning with a start bit and ending with a delimiter. The 16 bits between these start bits and delimiters depends on the field type. The Information field contains the encoder's information, such as command received by the encoder, encoder's address and status bits. The data field consists of multiple frames each starting with its own start bit and delimiter, contains various types of data based on the command code provided and the last data field frame contains 8 bit CRC field which contains CRC of all bits from all frames except start bits and delimiters. The receiver initially sends the command data field to start the communication. This action indicates the type of transaction to the encoder and the encoder returns this information based on the command code, as the previous paragraph explains. The encoder always returns the information field with encoder's status back to the receiver. In the case of data readout and reset transactions, the encoder returns the information field followed by the ALM or ABS or temperature or ID code or velocity or acceleration or velocity coefficient or combination of these, and ending with the CRC field at the end. In the case of an EEPROM read or write, the receiver, in addition to the Command Data Frame (CDF) field, sends Memory Data Frames (MDF) including the EEPROM address, data (for write only) and bank (optional). The encoder returns the info field, and EEPROM specific data which was sent.

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

#### 3 Channel Peripheral Interface PRU hardware interface

Refer to TRM for details

#### PRU-ICSS

\cond SOC_AM243X

Refer to PRU-ICSS chapter of AM64x/AM243x Technical Reference Manual.

\endcond

\cond SOC_AM261X

Refer to PRU-ICSS chapter of AM261x Technical Reference Manual.

\endcond

\cond SOC_AM263X

Refer to PRU-ICSS chapter of AM263x Technical Reference Manual.

\endcond

\cond SOC_AM263PX

Refer to PRU-ICSS chapter of AM263Px Technical Reference Manual.

\endcond
## Software Description

\cond SOC_AM243X
At start-up, the application running on the Arm®-based core initializes the module clocks and configures the pinmux. The PRU is initialized and the PRU firmware is loaded on PRU slice of choice for a chosen ICSS instance (tested on PRU1 on ICSSG0).
\endcond

\cond SOC_AM261X
At start-up, the application running on the Arm®-based core initializes the module clocks and configures the pinmux. The PRU is initialized and the PRU firmware is loaded on PRU slice of choice for a chosen ICSS instance (tested on PRU0 on ICSSM1).
\endcond

\cond (SOC_AM263X || SOC_AM263PX)
At start-up, the application running on the Arm®-based core initializes the module clocks and configures the pinmux. The PRU is initialized and the PRU firmware is loaded on PRU slice of choice for a chosen ICSS instance (tested on PRU0 on ICSSM).
\endcond

After the PRU starts executing, the Nikon interface is operational and the application can use it to communicate with an encoder. Use the Nikon diagnostic example to learn more about initialization and communication with the Nikon interface. This Nikon diagnostic example also provides an easy way to validate all the Nikon commands. The diagnostic example provides menu options on the host PC in a serial terminal application, where the user can select the command code and additional data (if needed) to be sent. Based on the command code, the application updates the Nikon interface with the CDF and MDF (in case of EEPROM access) and triggers transaction. The application then waits until it receives an indication of complete transaction by the firmware through the interface before displaying the result.

### Firmware Architecture {#NIKON_DESIGN_FLOW}

The firmware first initializes the local variables. Then it checks whether it is host trigger mode or periodic trigger mode. In host trigger mode, it waits until a command has been triggered through the interface. In periodic trigger mode, the firmware sets host trigger bit based on IEP compare 3 event configured. Upon triggering, the transmit data is set up based on the command code and the data is transmitted. The application then waits until receiving all the data that depends on the command code. The on-the-fly CRC over the received data then commences, and the interface is updated with the result. The CRC verification occurs next and the interface indicates command completion. The firmware then waits for the next command trigger from the interface.

\image html nikon_firmware_flow.png "Overview Flow Chart"

### Initialization {#NIKON_DESIGN_INITIALIZATION}

The 3 channel peripheral interface configuration MMRs are set as per protocol needs. Tx global reinit bit in R31 is set to put all channels in default mode. The clock source is selected. In Tx singleshot mode or continuous mode (in case of EEPROM Access or Identification Code Write or Velocity Coefficient Write), the output command data is loaded into Tx FIFO at 1x clock rate. In Rx mode, the input data is oversampled based on the selected baud rate. Hence, Tx clock (1x clock) and Rx clock (Oversampling (OS/UART) clock) are setup by selecting oversampling factor (x8 or x6 or x4) from application. At the end of the initialization, status is updated and wait until trigger from user occurs for Nikon commands.

\image html nikon_initialization.png "Initialization Flow Chart"


### Transmit and Receive {#NIKON_DESIGN_TX_RX}

In the current implementation, the Transmit data is loaded into the Tx FIFO byte-wise. For data ABS, ALM, temperature and ID code read commands, the requirement is to send 1 frame of 18 bits along with extra 1's at head as well as tail to compensate the timing requirements specified as per Nikon A-format specification. So, 4 bytes of data are first loaded into the Tx FIFO and Tx frame size is set to 32 bits to send data to encoder.

In case of EEPROM read, EEPROM write, ID code write, Velocity coefficient write and encoder address setting (command 20), 1 or more MDFs need to be sent after CDF as per Nikon A-format specification. There should be delay between CDF and MDF(s). For this, Tx frame size is set to 0 (continuous mode). 4 bytes of CDF are first loaded into the Tx FIFO and then FIFO byte level is constantly monitored and the FIFO is reloaded with the 1's when the FIFO level reaches 1 byte and it is repeated until PRU Cycle counter exceeds 1 microseconds equivalent value, then MDF with appropriate data is sent byte-wise when Tx FIFO level reaches 1 byte fill level. This process of sending MDF is repeated if there are more than one MDFs.

\image html nikon_tx_send.png "Transmit Flow Chart"

### Receive Data Parse {#NIKON_DESIGN_RX}

Once the transmission is complete, the encoder starts sending the data into multiple data fields, frames after information field frame with each frame being 18 bits beginning with start bit and ending with a delimiter. On-the-fly CRC is performed on all received bits except start bits and delimiters and the firmware copies the receive FIFO contents onto the receive buffer, individually, until all the data has been received except for 16 Mbps.

\image html nikon_receive_and_otf_crc.png "Receive Flow Chart"

In case of 16 Mbps, the Data is received and downsampled without On-the-fly CRC being calculated. After all Rx frames from all channels and all encoders are received, XOR with Polynomial based CRC calculation is performed in the post-processing section. Later calculated CRC is compared with received CRC.

\image html nikon_receive_16mhz_data.png "16 MHz Rx frames receive Flow Chart"
\image html nikon_post_processing.png "16 MHz post-processing Flow Chart"

### Continuous mode

\image html nikon_continuous_mode.png "Continuous mode"

Nikon receiver application has the support for continuous mode in which periodically MT Command is transmitted to encoder and its position data is read and the CRC is computed.
User can stop continuous mode by hitting any key in UART console.
Input cycle time should be greater than or equal to the Nikon cycle time by considering the maximum encoder address and timeouts.

### Receive CRC

The CRC is the last byte of the last received data frame. The firmware then stores the On-the-fly CRC performed on received data excluding the last byte, compares it with the received CRC value, and updates the CRC status and error counter in the interface.

\image html nikon_verify_crc.png "Verify CRC Flow Chart"

#### Pin Multiplexing {#NIKON_PIN_USAGE}
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
    <td>pru<n>_nikon0_clk
	<td>Channel 0 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO1 \else PRG<%k>_PRU<n>_GPO1 \endif
    <td>pru<n>_nikon0_out
	<td>Channel 0 transmit
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO2 \else PRG<%k>_PRU<n>_GPO2 \endif
    <td>pru<n>_nikon0_outen
	<td>Channel 0 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI9 \else PRG<%k>_PRU<n>_GPI13/PRG<%k>_PRU<n>_GPI9 \endif
    <td>pru<n>_nikon0_in
	<td>Channel 0 receive
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO3 \else PRG<%k>_PRU<n>_GPO3 \endif
    <td>pru<n>_nikon1_clk
	<td>Channel 1 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO4 \else PRG<%k>_PRU<n>_GPO4 \endif
    <td>pru<n>_nikon1_out
	<td>Channel 1 transmit
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO5 \else PRG<%k>_PRU<n>_GPO5 \endif
    <td>pru<n>_nikon1_outen
	<td>Channel 1 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI10 \else PRG<%k>_PRU<n>_GPI14/PRG<%k>_PRU<n>_GPI10 \endif
    <td>pru<n>_nikon1_in
	<td>Channel 1 receive
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO6 \else PRG<%k>_PRU<n>_GPO6 \endif
    <td>pru<n>_nikon2_clk
	<td>Channel 2 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO7 \else PRG<%k>_PRU<n>_GPO12/PRG<%k>_PRU<n>_GPO7 \endif
    <td>pru<n>_nikon2_out
	<td>Channel 2 transmit
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO8 \else PRG<%k>_PRU<n>_GPO8 \endif
    <td>pru<n>_nikon2_outen
	<td>Channel 2 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI11 \else PRG<%k>_PRU<n>_GPI11 \endif
    <td>pru<n>_nikon2_in
	<td>Channel 2 receive
</tr>
</table>

\cond SOC_AM243X
##### LP-AM243 Booster Pack Pin Multiplexing
<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>PRG0_PRU1_GPO0
    <td>pru1_nikon0_clk
	<td>Channel 0 clock
</tr>
<tr>
    <td>PRG0_PRU1_GPO1
    <td>pru1_nikon0_out
	<td>Channel 0 transmit
</tr>
<tr>
    <td>PRG0_PRU1_GPO2
    <td>pru1_nikon0_out_en
	<td>Channel 0 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI9
    <td>pru1_nikon0_in
	<td>Channel 0 receive (if(G_MUX_EN==0))
</tr>
<tr>
    <td>PRG0_PRU1_GPI13
    <td>pru1_nikon0_in
	<td>Channel 0 receive (if(G_MUX_EN==1))
</tr>
<tr>
    <td>PRG0_PRU1_GPO6
    <td>pru1_nikon2_clk
	<td>Channel 2 clock
</tr>
<tr>
    <td>PRG0_PRU1_GPO12
    <td>pru1_nikon2_out
	<td>Channel 2 transmit
</tr>
<tr>
    <td>PRG0_PRU1_GPO8
    <td>pru1_nikon2_out_en
	<td>Channel 2 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI11
    <td>pru1_nikon2_in
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
##### LP-AM261 Booster Pack Pin Multiplexing
<table>
<tr>
    <th>Pin name
    <th>Signal name
    <th>Function
</tr>
<tr>
    <td>PR1_PRU0_GPIO0
    <td>pru1_nikon0_clk
    <td>Channel 0 clock
</tr>
<tr>
    <td>PR1_PRU0_GPIO1
    <td>pru1_nikon0_out
    <td>Channel 0 transmit
</tr>
<tr>
    <td>PR1_PRU0_GPIO2
    <td>pru1_nikon0_out_en
    <td>Channel 0 transmit enable
</tr>
<tr>
    <td>PR1_PRU0_GPI9
    <td>pru1_nikon0_in
    <td>Channel 0 receive
</tr>
<tr>
    <td>GPIO Pin (GPIO_21)
    <td>ENC0_EN
    <td>Enable 3 channel peripheral interface in Axis 1 of BP (B10 GPIO pin)
</tr>
</table>
\endcond

\cond (SOC_AM263X || SOC_AM263PX)

##### @VAR_LP_BOARD_NAME Booster Pack Pin Multiplexing
<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>PR0_PRU0_GPIO3
    <td>pru1_endat1_clk
	<td>Channel 1 clock
</tr>
<tr>
    <td>PR0_PRU0_GPO4
    <td>pru1_endat1_out
	<td>Channel 1 transmit
</tr>
<tr>
    <td>PR0_PRU0_GPO5
    <td>pru1_endat1_outen
	<td>Channel 1 transmit enable
</tr>
<tr>
    <td>PR0_PRU0_GPI10
    <td>pru1_endat1_in
	<td>Channel 1 receive
</tr>
<tr>
    <td>SDFM0_D1 Pin (J8.73)
    <td>ENC1_EN
    <td>Enable 3 channel peripheral interface in Axis 1 of BP (D13 GPIO pin)
</tr>
</table>
\endcond

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

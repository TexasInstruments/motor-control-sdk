# Nikon Protocol Design {#NIKON_DESIGN}

[TOC]

## Introduction

This design implements Nikon Receiver (a.k.a subsequent electronics) using the 3 channel peripheral interface available on the TI Sitara™ AM64x/AM243x EVM. The 3 channel peripheral interface is a digital bidirectional serial interface for position encoders.
Transfer between receiver and encoder at the physical layer is in accordance with RS485, with transceiver at both ends.

## Nikon A-Format encoder receiver

The Nikon A-Format encoder receiver communicates with Nikon A-Format encoders and provides drive control with digital information to and from the encoder. Nikon communication is broadly classified into five types: data readout, resets, Encoder's ID code, address assignment and EEPROM transactions. Five types of data readout transactions occur: absolute data in one revolution, multi-turn data, encoder's temperature, encoder's status, encoder's ID and a combination of some of these. The reset transaction always returns the ALM field data while performing different types of resets. These commands needs to be executed 8 times to perform intended operation, and a 9th time for the changes to occur on Nikon interface. Four types of reset are available: reset of single turn data in one revolution, reset of multi-turn data, encoder's status reset and changing encoder's address to a specified value. ID code assignment: These commands returns the 24-bit ID code and there are three types of commands, to change encoder's address connected in bus by specifying 24-bit ID code of the encoder, to Read encoder's ID code and to write encoder's ID code. The EEPROM transaction allows the system to read and write to the EEPROM in the encoder. Each transaction has a unique command code and consists of different fields, namely encoder address, status, data, cyclic redundancy check (CRC), ALM field, Identification code, Encoder's temperature, EEPROM address, and EEPROM data depending on the type of transaction, that is, command code.

Each field is 18-bits long, beginning with a start bit and ending with a delimiter. The 16 bits between these start bits and delimiters depends on the field type. The Information field contains the encoder's information,such as command received by the encoder, encoder's address and status bits. The Data field is of multiple frames each starting with its own start bit and delimiter, contains various types of data based on the command code provided and last Data field frame contains 8 bit CRC field which contains CRC of all bits from all frames except start bits and delimiters. The receiver initially sends the command data field to start the communication. This action indicates the type of transaction to the encoder and the encoder returns this information based on the command code, as the previous paragraph explains. The encoder always returns the information field with encoder's status back to the receiver. In the case of data readout and reset transactions, the encoder returns the information field followed by the ALM or ABS or Temperature or ID code, and ending with the CRC field at the end. In the case of an EEPROM read or write, the receiver, in addition to the CDF field, sends the EEPROM address field MDF2 and EEPROM data field in case of write (MDF0 and MDF1). The encoder returns the info field, followed by the EEPROM address, EEPROM data, and CRC fields.

## System Overview

### Sitara™ AM64x/AM243x Processor

Refer TRM for details

#### 3 Channel Peripheral Interface PRU hardware interface

Refer TRM for details

#### PRU-ICSS

Refer PRU-ICSS chapter of AM64x/AM243x Technical Reference Manual.

## Software Description

At start-up, the application running on the ARM Cortex-R5 initializes the module clocks and configures the pinmux. The PRU is initialized and the PRU firmware is loaded on PRU slice of choice for a chosen ICSS instance (tested on PRU1 on ICSSG0). After the PRU1 starts executing, the Nikon interface is operational and the application can use it to communicate with an encoder. Use the Nikon diagnostic example to learn more about initialization and communication with the Nikon interface. This Nikon diagnostic example, also provides an easy way to validate the Nikon transactions. The diagnostic example provides menu options on the host PC in a serial terminal application, where the user can select the Command code to be sent. User can assign a 24-bit Identification code to the encoder and can change Encoder's Address.  User can also enter EEPROM address(incase of EEPROM Read) and EEPROM data(incase of EEPROM Write). Based on the command code, the application updates the Nikon interface with the Command data frame and Memory data frame (incase of EEPROM access) and trigger transaction. The application then waits until it receives an indication of complete transaction by the firmware through the interface before displaying the result.

### Firmware Architecture {#NIKON_DESIGN_FLOW}

The firmware first initializes the local variables. Then it checks whether it is host trigger mode or periodic trigger mode. In host trigger mode, it waits until a command has been triggered through the interface. In periodic trigger mode, the firmware sets host trigger bit based on IEP compare 3 event configured. Upon triggering the transmit data is set up based on the command code and the data is transmitted. The application then waits until receiving all the data that depends on the command code. The on-the-fly CRC over the received data then commences, and the interface is updated with the result. The CRC verification occurs next and the interface indicates command completion. The firmware then waits for the next command trigger from the interface.

\image html nikon_firmware_flow.png "Overview Flow Chart"

### Initialization {#NIKON_DESIGN_INITIALIZATION}

The 3 channel peripheral interface configuration MMRs are set as per protocol needs. Tx global reinit bit in R31 is set to put all channels in default mode. The clock source is selected (ICSSG clock is selected with 200MHZ frequency). In Tx Singleshot mode or Continuous mode (incase of EEPROM Access or Identification code Write), the output command data is loaded into Tx FIFO at 1x clock rate. In Rx mode, the input data is oversampled based on the selected baud rate. Hence, Tx clock(1x clock) and Rx clock(Oversampling (OS/UART) clock) are setup by selecting oversampling factor(x8 or x6 or x4) from application. At the end of the initialization status is updated and wait until trigger from user occurs for Nikon commands.

\image html nikon_initialization.png "Initialization Flow Chart"


### Transmit and Receive {#NIKON_DESIGN_TX_RX}

In the current implementation, the Transmit data is loaded into the Tx FIFO in byte wise. For data ABS, ALM, Temperature and ID code read commands, the requirement is to send 1 frame of 18 bits along with extra 1's at head as well as tail to compensate the timing requirements specified as per Nikon A-Format specification. So, 4 bytes of data is first loaded into the Tx FIFO and Tx frame size is set to 32 bits to send data to Encoder.

Similarly, for EEPROM Read command, the requirement is to send 2 frames of 32 bits each, so 64 bits in total and 10usec delay to be compensated with equivalent 1's in between each Tx frame. For this,Tx frame size is set to 0(continuous mode). 4 bytes of CDF is first loaded into the Tx FIFO and then FIFO byte level is constantly monitored and the FIFO is reloaded with the 1's when the FIFO level reaches 1 byte and it is repeated until PRU Cycle counter exceeds 10usec equivalent value, then Memory data frame(MDF2) with EEPROM address is sent byte wise when Tx FIFO level reaches 1 byte fill level. After certain delay Tx global reinit is triggered, and this process is repeated after 300usec delay to read the data from second Nikon cycle, because the data from first cycle is undetermined and encoder will be in MemBusy state for 300usec(max timeout for EEPROM read).

In case of EEPROM Write command, the requirement is to send 4 frames of 32 bits each - 128 bits in total. For this, 4 bytes of CDF is first loaded into the Tx FIFO and then transmission is started in Tx - Continuous FIFO loading mode. FIFO byte level is constantly monitored and the FIFO is reloaded with 1's until PRU cycle counter exceeds 10usec equivalent value, then MDF[0-2](Memory data frames) are loaded byte wise when Tx FIFO fill level reaches 1 byte, by maintaining 10usec delay in between them as mentioned before. After certain delay Tx global reinit is triggered, and this process is repeated after 30 millisec delay to read Rx data from second Nikon cycle, because the data from first cycle is undetermined and encoder will be in MemBusy state for 30 millisec (max timeout for EEPROM write or reset). All the delay numbers mentioned above are considered from Nikon A-Format specification.

\image html nikon_tx_send.png "Transmit Flow Chart"


### Receive Data Parse {#NIKON_DESIGN_RX}

Once the transmission is complete, the encoder starts sending the data into multiple data field, frames after information field frame with each frame being 18 bits beginning with start bit and ending with a delimiter. On-the-fly CRC is performed on all received bits except start bits and delimiters and the firmware copies the receive FIFO contents onto the receive buffer, individually, until all the data has been received except for 16 Mbps.

\image html nikon_receive_and_otf_crc.png "Receive Flow Chart"

In case of 16 Mbps, the Data is received and downsampled with out On-the-fly CRC being calculated. After all Rx frames from all channels and all encoders are received, XOR with Polynomial based CRC calculation is performed in the Post processing section. Later calculated CRC is compared with received CRC.

\image html nikon_receive_16mhz_data.png "16Mhz Rx frames receive Flow Chart"
\image html nikon_post_processing.png "16Mhz Post Processing Flow Chart"

### Continuous mode

\image html nikon_continuous_mode.png "Continuous Mode"

Nikon receiver application has the support for continuous mode in which periodically MT Command is transmitted to encoder and its position data is read and computed the CRC.
User can stop continuous mode by hitting any key in UART console.
Input cycle time should be greater than or equal to the Nikon cycle time by considering the maximum encoder address and and timeouts.

### Receive CRC

The CRC is the last byte of the last received data frame. The firmware then stores the On-the-fly performed on received data excluding the last byte, compares it with the received CRC value, and updates the CRC status and error counter in the interface.

\image html nikon_verify_crc.png "Verify CRC Flow Chart"


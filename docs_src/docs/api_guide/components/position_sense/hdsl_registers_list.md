# TI HDSL Register List {#HDSL_REGISTER_LIST}

TI HDSL Solution's register map is compatible with SICK HDSL MASTER IP Core release version 1.07, with few exceptions marked with "Not available in TI HDSL Solution", or "Different implementation from SICK HDSL MASTER IP Core", or "Register address is different from SICK HDSL MASTER IP Core" in below table.

<table>
<tr>
    <th> Register name
    <th> Register offset
    <th> Bit/s
    <th> Description
</tr>
<tr>
    <td> SYS_CTRL
    <td> 0x00
    <td>
    <td> System Control
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> PRST: Protocol reset
         - 0 = Normal protocol action
         - 1 = A forced reset of the protocol status will be initiated. If the bit is deleted, a restart of the connection is triggered.
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> MRST: Messages reset
         - 0 = Normal Parameters Channel action
         - 1 = The Parameters Channel is reset. Current short and long messages are discarded.
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> FRST: Pipeline FIFO, reset <br/> **NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> LOOP: Test drive interface <br/> **NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> PRDY: POS_READY mode <br/> **NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> SPPE: SPI-PIPE activation <br/> **NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> SPOL: Polarity of the synchronization pulse <br/> **NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> OEN: Activation of the output <br/> **NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td> SYNC_CTRL
    <td> 0x01
    <td>
    <td> Synchronization Control
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> ES: External synchronization
         - 0 = Position sampling during free running at the shortest cycle time.
         - All other values = Position sampling with the sync signal synchronized. The value from ES determines the number of position samplings carried out in one sync cycle.
</tr>
<tr>
    <td> MASTER_QM
    <td> 0x03
    <td>
    <td> Quality Monitoring
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> LINK: DSL protocol connection status
         - 0 = No connection present or connection error due to a communications error.
         - 1 = Protocol connection between DSL Master and Slave was established.
</tr>
<tr>
    <td>
    <td>
    <td> 6:4
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 3:0
    <td> Quality monitoring value
         - Quality monitoring is initiated with the value "8".
         - The maximum quality monitoring value is "15". This is the standard value during operation.
         - Higher values indicate a better connection.
</tr>
<tr>
    <td> EVENT_H
    <td> 0x04
    <td>
    <td> Events (High Byte)
         - It contains the messaging bits for warning and error modes of the DSL system.
         - All messaging bits are set by the DSL Master if a corresponding status is determined.
         - An event bit that has been set is not reset by the DSL Master.
         - It should be noted that all event register bits are also transferred to ONLINE_STATUS_D register. The event bits are not static there and contain the actual status of each individual event.
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> INT: Interrupt status <br/> **NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> SUM: Remote event monitoring
         - 0 = All DSL Slave events are deleted.
         - 1 = The DSL Slave has signaled an event and the summary mask is set accordingly (see registers MASK_SUM and SUMMARY).

         When the SUM bit is set, an error or a warning has been transmitted from the DSL Slave. The application must check the SUMMARY register to obtain a detailed description.
</tr>
<tr>
    <td>
    <td>
    <td> 5:4
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> POS: Estimator turned on
         - 0 = The data for the fast position was correctly transmitted.
         - 1 = Fast position data consistency error. The fast position read through drive interface is supplied by the estimator.
         This error usually indicates a transmission error on the DSL connection. If this error occurs frequently, the wiring of the DSL connection should be checked. If this error occurs continuously, there is probably an error in the motor feedback system.
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> DTE: Estimator Deviation Threshold Error
         - 0 = Current value of deviation smaller than the specified maximum.
         - 1 = Current value of deviation greater than the specified maximum.
        <br/> **NOTE : Not available in TI HDSL Solution.**
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> PRST: Protocol reset warning
         - 0 = Normal protocol action
         - 1 = The forced protocol reset was triggered.
</tr>
<tr>
    <td> EVENT_L
    <td> 0x05
    <td>
    <td> Events (Low Byte)
         - It contains the messaging bits for warning and error modes of the DSL system.
         - All messaging bits are set by the DSL Master if a corresponding status is determined.
         - An event bit that has been set is not reset by the DSL Master.
         - It should be noted that all event register bits are also transferred to ONLINE_STATUS_D register. The event bits are not static there and contain the actual status of each individual event.
</tr>
<tr>
    <td>
    <td>
    <td> 7:6
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> MIN: Message initialization
         - 0 = No acknowledgment for the initialization received.
         - 1 = An acknowledgment was received from the Slave for the initialization of a message.

         When this warning is displayed, the Parameters Channel is still in the initialization status and no "short message" or "long message" can be triggered.
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> ANS: Erroneous answer to "long message"
         - 0 = The last answers to "long messages" were error free.
         - 1 = An error occurred during the answer to a long message. The effectiveness of the previous transaction is not known.

         This error indicates that the transmission of an answer from the DSL Slave to the last "long message" failed. The application must send the "long message" again.
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> QMLW: Quality monitoring low value warning
         - 0 = Quality monitoring value greater than or equal to "14"
         - 1 = Quality monitoring value (see register 03h) below "14"
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> FREL: Channel free for "long message"
         - 1 = A "long message" can be sent on the Parameters Channel.
         - 0 = No "long message" can be sent.

         If the bit is set, the application can trigger a "long message". Provided no answer has been received from the DSL Slave, this bit remains deleted. As the processing duration of a "long message" in the motor feedback system is not specified, a user time limit condition should be installed via the application. When a time limit is exceeded, the MRST bit in the SYS_CTRL register is set, which causes the Parameters Channel to be reset.
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td> MASK_H
    <td> 0x06
    <td>
    <td> Event mask (High Byte)
         - In the event mask registers MASK_H/MASK_L, the events are set with which the event interrupt is set.
         - Several events can be masked to trigger an event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> MSUM: Mask for remote event monitoring
         - 0 = DSL Slave events that are masked in the SUMMARY register do not set the event interrupt.
         - 1 = DSL Slave events that are masked in the SUMMARY register set the event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 5:4
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> MPOS: Mask for fast position error
         - 0 = An error in the fast position does not set the event interrupt.
         - 1 = An error in the fast position sets the event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> MDTE: Mask for estimator deviation threshold error warning
         - 0 = A high deviation threshold error value does not set the event interrupt.
         - 1 = A high estimator deviation threshold error sets the event interrupt.
        <br/> **NOTE : Not available in TI HDSL Solution.**
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> MPRST: Mask for protocol reset warning
         - 0 = A protocol reset does not set the event interrupt.
         - 1 = A protocol reset sets the event interrupt.
</tr>
<tr>
    <td> MASK_L
    <td> 0x07
    <td>
    <td> Event mask (Low Byte)
         - In the event mask registers MASK_H/MASK_L, the events are set with which the event interrupt is set.
         - Several events can be masked to trigger an event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 7:6
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> MMIN: Mask for message initialization confirmation
         - 0 = The acknowledgment for the initialization of a DSL Slave message does not set the event interrupt.
         - 1 = The acknowledgment for the initialization of a DSL Slave message sets the event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> MANS: Mask for erroneous answer to long message
         - 0 = A transmission error during the answer to a long message does not set the event interrupt.
         - 1 = A transmission error during the answer to a long message sets the event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> MQMLW: Mask for low quality monitoring value warning
         - 0 = A low quality monitoring value does not set the event interrupt.
         - 1 = A low quality monitoring value (see registers MASTER_QM and EVENT_L) sets the event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> MFREL: Mask for "channel free for "long message"
         - 0 = If a "long message" can be sent on the Parameters Channel, the event interrupt is not set.
         - 1 = If a "long message" can be sent on the Parameters Channel, the event interrupt is set.
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td> MASK_SUM
    <td> 0x08
    <td>
    <td> Summary mask
         - In this register, the DSL Slave collective events are determined with which the SUM event monitoring in the event register as well as the signal to the interrupt pin are set (interrupt).
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> MSUM7:MSUM0: Mask for status summary bits
         - 0 = In the set status, the corresponding status summary bit does not set the SUM event monitoring and the signal at the interrupt pin.
         - 1 = In the set status, the corresponding status summary bit sets the SUM event monitoring and the signal at the interrupt pin.
</tr>
<tr>
    <td> EDGES
    <td> 0x09
    <td>
    <td> Edges
         - This register contains the time control for the DSL cable bit sampling and can be used to monitor the connection quality.
         - Each individual edge register bit is set if, at system start-up, an edge of the test signal is detected during the time period of the corresponding bit. An edge is defined as a change in cable value between successive detections.
         - The sampling is carried out eight times as fast as the cable bit rate.
         - Clean cable signals mean that only a few bits are set in the edge register, whilst noisy cable signals set a large number of bits.
         <br/> **NOTE : Not available in TI HDSL Solution in this release**
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Bit sampling pattern: Identification of edges in the cable signal
         - 0 = No edge was detected in the time period of the corresponding bit.
         - 1 = An edge was detected in the time period of the corresponding bit.
</tr>
<tr>
    <td> DELAY
    <td> 0x0A
    <td>
    <td> Run time delay / RSSI
</tr>
<tr>
    <td>
    <td>
    <td> 7:4
    <td> Cable delay
         - 4 bit value for cable delay, which gives the cable signal round trip delay of cable and transceivers in bits.
         - This value enables a rough estimate of cable length to be made.
         - The value for Line Delay does not change after the start-up phase. A fresh value for Line Delay is only measured after a forced reset of the protocol.
<table>
<tr>
    <th> Cable Delay
    <th> DSL Connection Cable Delay (ns)
    <th> DSL Connection Cable Length (m)
</tr>
<tr>
    <td> 0
    <td> &lt; 100
    <td> &lt; 10
</tr>
<tr>
    <td> 1
    <td> 100-200
    <td> 10-20
</tr>
<tr>
    <td> 2
    <td> 200-300
    <td> 20-30
</tr>
<tr>
    <td> 3
    <td> 300-400
    <td> 30-40
</tr>
<tr>
    <td> 4
    <td> 400-500
    <td> 40-50
</tr>
<tr>
    <td> 5
    <td> 500-600
    <td> 50-60
</tr>
<tr>
    <td> 6
    <td> 600-700
    <td> 60-70
</tr>
<tr>
    <td> 7
    <td> 700-800
    <td> 70-80
</tr>
<tr>
    <td> 8
    <td> 800-900
    <td> 80-90
</tr>
<tr>
    <td> 9
    <td> 900-1000
    <td> 90-100
</tr>
</table>
</tr>
<tr>
    <td>
    <td>
    <td> 3:0
    <td> RSSI: Indication of the received signal strength
         - 4 bit value for the cable signal strength, from "0" to "12".
         - Higher values indicate better connection quality.
         - RSSI is continuously updated during operation and used for signal monitoring during run time.
</tr>
<tr>
    <td> VERSION
    <td> 0x0B
    <td>
    <td> Version
         <br/> **NOTE : Different implementation from SICK HDSL MASTER IP Core**
</tr>
<tr>
    <td>
    <td>
    <td> 7:4
    <td> Major Release Number
</tr>
<tr>
    <td>
    <td>
    <td> 3:0
    <td> Minor Release Number
</tr>
<tr>
    <td> RELEASE
    <td> 0x0C
    <td>
    <td> Release Date
         <br/>**NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td> ENC_ID2
    <td> 0x0D
    <td>
    <td> Encoder ID (Byte 2) <br/>
         The ENC_ID registers (ENC_ID2, ENC_ID1 and ENC_ID0) contain the designation code of the motor feedback system connected to the DSL Master. In the current protocol specification, the designation code is 20 bits long.
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 6:4
    <td> SCI: Indication of special characters
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> Continue
         - 1 = ENC_ID is longer than 20 bits (for future use).
</tr>
<tr>
    <td>
    <td>
    <td> 2:0
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td> ENC_ID1
    <td> 0x0E
    <td>
    <td> Encoder ID (Byte 1)
</tr>
<tr>
    <td>
    <td>
    <td> 7:4
    <td> User defined encoder index
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> Sign
         - 0 = Position value is signed.
         - 1 = Position value is not signed.
</tr>
<tr>
    <td>
    <td>
    <td> 1:0
    <td> Higher 2 bits of length of position information minus length of the acceleration value transmitted.
</tr>
<tr>
    <td> ENC_ID0
    <td> 0x0F
    <td>
    <td> Encoder ID (Byte 0)
</tr>
<tr>
    <td>
    <td>
    <td> 7:4
    <td> Lower 4 bits of length of position information minus length of the acceleration value transmitted.
</tr>
<tr>
    <td>
    <td>
    <td> 3:0
    <td> Length of the acceleration value transmitted minus 8.
</tr>
<tr>
    <td> POS4
    <td> 0x10
    <td>
    <td> Fast Position (Byte 4)  <br/>
         - The POS registers for the fast position contain the value of the motor feedback system connected.
         - This position is generated incrementally from the safe position at start-up and is updated with every protocol frame.
         - After every eight protocol frames, the fast position is checked against the safe position (see registers 0x18 to 0x1C).
         - The position sampling point is determined by the ES value of the synchronization control register.
         - Only those POS bits are activated that lie within the range that the motor feedback system has actually measured. All other higher value bits are read as "0".
            - The number of measurable bits can be taken from ENC_ID bits 9 to 0 in the ENC_ID0 and ENC_ID1 registers.
            - If Sign is set in the ENC_ID1 register, the value of the fast position is given signed in the two's complement.
            - The units of the position value are (steps).

        **NOTE** : The fast position must not be used for safety functions.
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 4 of fast position value of the motor feedback system (length: 40 bits), incrementally generated
</tr>
<tr>
    <td> POS3
    <td> 0x11
    <td>
    <td> Fast Position (Byte 3)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 3 of fast position value of the motor feedback system (length: 40 bits), incrementally generated
</tr>
<tr>
    <td> POS2
    <td> 0x12
    <td>
    <td> Fast Position (Byte 2)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 2 of fast position value of the motor feedback system (length: 40 bits), incrementally generated
</tr>
<tr>
    <td> POS1
    <td> 0x13
    <td>
    <td> Fast Position (Byte 1)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 1 of fast position value of the motor feedback system (length: 40 bits), incrementally generated
</tr>
<tr>
    <td> POS0
    <td> 0x14
    <td>
    <td> Fast Position (Byte 0)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 0 of fast position value of the motor feedback system (length: 40 bits), incrementally generated
</tr>
<tr>
    <td> VEL2
    <td> 0x15
    <td>
    <td> Speed (Byte 2) <br/>
         - The VEL speed registers contain the speed values of the connected motor feedback system.
         - This value is calculated as a delta position from the acceleration value (delta-delta position) transmitted on the process data channel and the currently updated protocol frame.
         - The speed sampling point is determined by the ES value of the SYNC_CTRL register.
         - The units of the speed value are (steps/frame cycle time).

         **NOTE** : The speed value must not be used for safety functions.
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 2 of speed of the motor feedback system (length: 24 bits)
</tr>
<tr>
    <td> VEL1
    <td> 0x16
    <td>
    <td> Speed (Byte 1)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 1 of speed of the motor feedback system (length: 24 bits)
</tr>
<tr>
    <td> VEL0
    <td> 0x17
    <td>
    <td> Speed (Byte 0)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 0 of speed of the motor feedback system (length: 24 bits)
</tr>
<tr>
    <td> MIR_SUM
    <td> 0x18
    <td>
    <td> Mirror Summary
         <br/> **NOTE : Not available in TI HDSL Solution. Please see SAFE_SUM (0x36) for getting summary information**
</tr>
<tr>
    <td> VPOS4
    <td> 0x19
    <td>
    <td> Safe Position, Channel 1 (Byte 4)
         - The VPOS registers for the safe position contain the position value from the primary channel of the motor feedback system connected.
         - This safe position is transmitted in every eighth protocol frame.
         - Only those VPOS bits are activated that lie within the range that the motor feedback system has actually measured. All other higher value bits are read as "0".
            - The number of measurable bits can be taken from ENC_ID bits 9 to 0 in the ENC_ID0 and ENC_ID1 registers.
            - If Sign is set in the ENC_ID1 register, the value of the fast position is given signed in the two's complement.
            - The units of the position value are (steps).
         - The safe position will have the same data format as the fast position.
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 4 of position value transmitted through Safe Channel 1 (length: 40 bits), absolute value.
</tr>
<tr>
    <td> VPOS3
    <td> 0x1A
    <td>
    <td> Safe Position, Channel 1 (Byte 3)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 3 of position value transmitted through Safe Channel 1 (length: 40 bits), absolute value.
</tr>
<tr>
    <td> VPOS2
    <td> 0x1B
    <td>
    <td> Safe Position, Channel 1 (Byte 2)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 2 of position value transmitted through Safe Channel 1 (length: 40 bits), absolute value.
</tr>
<tr>
    <td> VPOS1
    <td> 0x1C
    <td>
    <td> Safe Position, Channel 1 (Byte 1)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 1 of position value transmitted through Safe Channel 1 (length: 40 bits), absolute value.
</tr>
<tr>
    <td> VPOS0
    <td> 0x1D
    <td>
    <td> Safe Position, Channel 1 (Byte 0)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 0 of position value transmitted through Safe Channel 1 (length: 40 bits), absolute value.
</tr>
<tr>
    <td> VPOSCRC_H
    <td> 0x1E
    <td>
    <td> Position checksum, Channel 1 (High Byte)
         - The VPOSCRC registers for the position checksum contain the CRC checksum of the safe position VPOS and the SUMMARY status.
         - The CRC is checked in the DSL Master IP Core.
         - In order to guarantee, in a safety related application, that the CRC machine in the IP Core is functioning, these registers can be checked with an external cross check in the diagnostics test interval.
         - The CRC is generated with the following CRC parameters:
            - CRC Sequence : 16 Bit
            - CRC Polynomial : 0xC86C (x<sup>16</sup> + x<sup>15</sup> + x<sup>12</sup> + x<sup>7</sup> + x<sup>6</sup> + x<sup>4</sup> + x<sup>3</sup> + 1), Normal representation: 0x90D9
            - Starting Value : 0x0000
            - Closing XOR Value : 0x00FF
            - Reverse Data Bytes : No
            - Reverse CRC before closing XOR : No
            - Sequence of the bytes for calculation : SAFE_SUM, VPOS4, VPOS3, VPOS2, VPOS1, VPOS0
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 1 of 16 bit CRC checksum (CRC 16) of the safe position and status summary in Safe Channel 1.
</tr>
<tr>
    <td> VPOSCRC_L
    <td> 0x1F
    <td>
    <td> Position checksum, Channel 1 (Low Byte)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 0 of 16 bit CRC checksum (CRC 16) of the safe position and status summary in Safe Channel 1.
</tr>
<tr>
    <td> PC_BUFFER0
    <td> 0x20
    <td>
    <td> Parameters Channel Buffer (Byte 0)
         - The eight PC_BUFFER registers of the Parameters Channel buffer contain the answer to the last "long message" request or the data for a "long message" write operation.
         - Depending on the length of the "long message" answer, the registers are used as follows:
<table>
<tr>
    <th> Length of the "long message"
    <th> Registers used
</tr>
<tr>
    <td> 8 bytes
    <td> 0x20 to 0x27
</tr>
<tr>
    <td> 4 bytes
    <td> 0x20 to 0x23
</tr>
<tr>
    <td> 2 bytes
    <td> 0x20 to 0x21
</tr>
<tr>
    <td> 0 bytes
    <td> None
</tr>
</table>
         - These registers are also for the reporting of error conditions arising from a "long message" operation. If, when accessing a resource, an error due to a "long message" arises (e.g. invalid data, error in the A/D conversion), after the answering message has been received the LOFF bit in the PC_ADD_H register (28h) is set. In this case the Parameters Channel buffer bytes 0 and 1 contain an error code.
         - The meaning of the error code depends on the particular encoder.
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 0 of 8 bytes for the answer to a long message (read operation) or for a "long message" write operation; or <br/>
         Byte 0 of 2 bytes for reports about errors in encoder resources arising from the previous "long message" operation.
</tr>
<tr>
    <td> PC_BUFFER1
    <td> 0x21
    <td>
    <td> Parameters Channel Buffer (Byte 1)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 1 of 8 bytes for the answer to a long message (read operation) or for a "long message" write operation; or <br/>
         Byte 1 of 2 bytes for reports about errors in encoder resources arising from the previous "long message" operation.
</tr>
<tr>
    <td> PC_BUFFER2
    <td> 0x22
    <td>
    <td> Parameters Channel Buffer (Byte 2)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 2 of 8 bytes for the answer to a long message (read operation) or for a "long message" write operation.
</tr>
<tr>
    <td> PC_BUFFER3
    <td> 0x23
    <td>
    <td> Parameters Channel Buffer (Byte 3)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 3 of 8 bytes for the answer to a long message (read operation) or for a "long message" write operation.
</tr>
<tr>
    <td> PC_BUFFER4
    <td> 0x24
    <td>
    <td> Parameters Channel Buffer (Byte 4)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 4 of 8 bytes for the answer to a long message (read operation) or for a "long message" write operation.
</tr>
<tr>
    <td> PC_BUFFER5
    <td> 0x25
    <td>
    <td> Parameters Channel Buffer (Byte 5)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 5 of 8 bytes for the answer to a long message (read operation) or for a "long message" write operation.
</tr>
<tr>
    <td> PC_BUFFER6
    <td> 0x26
    <td>
    <td> Parameters Channel Buffer (Byte 6)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 6 of 8 bytes for the answer to a long message (read operation) or for a "long message" write operation.
</tr>
<tr>
    <td> PC_BUFFER7
    <td> 0x27
    <td>
    <td> Parameters Channel Buffer (Byte 7)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 7 of 8 bytes for the answer to a long message (read operation) or for a "long message" write operation.
</tr>
<tr>
    <td> PC_ADD_H
    <td> 0x28
    <td>
    <td> Long message address (High Byte)
         - The addresses and the addressing mode for "long messages" sent over the Parameters Channel are determined in the PC_ADD_H/PC_ADD_L long message address registers.
         - In addition, the long message address register 0x28 (PC_ADD_H) contains indications of errors arising from "long message" operations. For this sort of error, the Parameters Channel buffer contains the error code in bytes 0 and 1 associated with this status.
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> LRW: Long message, read/write mode
         - 0 = "Long message" write operation
         - 1 = "Long message" read operation
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> LOFF: Long message addressing mode/long message error
         - Write Access
            - 0 = Addressing of "long messages" without offset. The offset value from the PC_OFF_H/PC_OFF_Lregisters is not used.
            - 1 = Offset addressing of "long messages". The offset value from the PC_OFF_H/PC_OFF_L registers is used in the resource of the selected database entry as a sub-address.
         - Read Access
            - 0 = The last "long message" was correctly processed.
            - 1 = The last "long message" caused an error.
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> LIND: Indirect addressing of long messages
         - 0 = Direct addressing of "long messages". The operation affects the database entry given in the current address.
         - 1 = Indirect addressing of "long messages". During this operation, the stored address content in the given database entry is evaluated.
</tr>
<tr>
    <td>
    <td>
    <td> 3:2
    <td> LLEN: Data length of the "long message"
         - 00 = No data bytes
         - 01 = 2 data bytes
         - 10 = 4 data bytes
         - 11 = 8 data bytes
</tr>
<tr>
    <td>
    <td>
    <td> 1:0
    <td> Bits 9:8 of 10 bit address for a "long message" operation
</tr>
<tr>
    <td> PC_ADD_L
    <td> 0x29
    <td>
    <td> Long message address (Low Byte)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Bits 7:0 of 10 bit address for a "long message" operation
</tr>
<tr>
    <td> PC_OFF_H
    <td> 0x2A
    <td>
    <td> Long message address offset (High Byte)
         - The PC_OFF_H/PC_OFF_L address offset registers for long messages are used in "long message" operations, if LOFF is set in the register 0x28.
         - In this case the LOFFADD value from these registers is used to communicate with the sub-address of a multiple byte encoder resource.
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> LID: Long message identification.
         The value must be "1".
</tr>
<tr>
    <td>
    <td>
    <td> 6:0
    <td> LOFFADD (14:8) : Long message offset value
         Bits 14:8 of the 15 bit offset value of the "long message" address offset is stored in these bits.
</tr>
<tr>
    <td> PC_OFF_L
    <td> 0x2B
    <td>
    <td> Long message address offset (Low Byte)
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> LOFFADD (7:0) : Long message offset value
         Bits 7:0 of the 15 bit offset value of the "long message" address offset is stored in these bits.
</tr>
<tr>
    <td> PC_CTRL
    <td> 0x2C
    <td>
    <td> Parameters Channel Control
         - This register for the Parameters Channel handles the start of "long message" transactions. After setting all "long message" registers (registers PC_BUFFER0 to 7, PC_ADD_H/PC_ADD_L and PC_OFF_H/L), the "long message" is transmitted to the DSL Slave by setting the LSTA bit.
</tr>
<tr>
    <td>
    <td>
    <td> 7:1
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> LSTA: Control of the long message start
         - 0 = No effect.
         - 1 = A "long message" transaction is started with the values currently stored in the "long message" registers.
</tr>
<tr>
    <td> PIPE_S
    <td> 0x2D
    <td>
    <td> SensorHub Channel Status
         <br/> **NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td> PIPE_D
    <td> 0x2E
    <td>
    <td> SensorHub Channel Data
         <br/> **NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td> PC_DATA
    <td> 0x2F
    <td>
    <td> "Short message" Mirror Register
         <br/> **NOTE : Not available in TI HDSL Solution. Please see S_PC_DATA (0x37) for “short message” transactions.**
</tr>
<tr>
    <td> RESERVED
    <td> 0x34, 0x33, 0x32, 0x31, 0x30
    <td>
    <td>  **NOTE** : Reserved for future use
</tr>
<tr>
    <td> SAFE_CTRL
    <td> 0x35
    <td>
    <td> Safe System Control
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> PRST: Protocol reset
         - 0 = Normal protocol action.
         - 1 = A forced reset of the protocol status will be initiated. If the bit is deleted, a restart of the connection is triggered.
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> MRST: Messages reset
         - 0 = Normal Parameters Channel action.
         - 1 = The Parameters Channel is reset. Current short and long messages are discarded.
</tr>
<tr>
    <td>
    <td>
    <td> 5:0
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td> SAFE_SUM
    <td> 0x36
    <td>
    <td> Safe Summary
         - This register contains the summarized DSL Slave status information for the safety related application.
         - It is based on the encoder status ENC_ST7:0.
         - Each status summary bit contains the summarized information from 8 error, warning and event modes of the DSL Slave.
</tr>
<tr>
    <td>
    <td>
    <td> 7:1
    <td> SSUM7:SSUM1: Status Summary bit (external resource)
         - 0 = The corresponding error, warning or event is not active.
         - 1 = An error, a warning or an event associated with DSL Slave external resources was triggered.
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> SSUM0: Status summary bit (interface)
         - 0 = The DSL Slave protocol has not triggered an error, a warning or event.
         - 1 = An error, a warning or an event associated with the DSL Slave protocol interface was triggered.
</tr>
<tr>
    <td> S_PC_DATA
    <td> 0x37
    <td>
    <td> "Short message" Parameters Channel Data
         - This register for the Parameters Channel short message contains the results of “short message” transactions.
         - “Short message” transactions are generated if operations are carried out with remote registers (DSL Slave).
         - Generally, FRES (in the EVENT_S register) must be set after a transaction is started. Only then will S_PC_DATA contain valid information.
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> 8 bit value of the requested remote register.
</tr>
<tr>
    <td> ACC_ERR_CNT
    <td> 0x38
    <td>
    <td> Fast Position Error Counter
         <br/>**NOTE : Different implementation from SICK HDSL MASTER IP Core**
         - This register gives the count of transmitted fast position values with consecutive transmission errors.
         - Writing to this register does not set any threshold for setting an error signal.
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> 8 bit value of count of transmitted fast position values with consecutive transmission errors.
</tr>
<tr>
    <td> MAXACC
    <td> 0x39
    <td>
    <td> Fast Position Acceleration Boundary
         <br/> **NOTE : Not available in TI HDSL Solution.**
</tr>
<tr>
    <td> MAXDEV
    <td> 0x3A, 0x3B
    <td>
    <td> Fast Position Estimator Deviation
         <br/> **NOTE : Not available in TI HDSL Solution.**
</tr>
<tr>
    <td> RESERVED
    <td> 0x3C
    <td>
    <td>  **NOTE** : Reserved for future use
</tr>
<tr>
    <td> EVENT_S
    <td> 0x3D
    <td>
    <td> Safe Events
         - It contains the messaging bits for warning and error modes of the DSL system.
         - All messaging bits are set by the DSL Master if a corresponding status is determined.
         - An event bit that has been set is not reset by the DSL Master.
         - The safety related application must delete bits that have been set.
         - It should be noted that all event register bits are also transferred to Online Status 1. The event bits are not static there and contain the actual status of each individual event.
         - The following bit description lists the effects of warning and error conditions as well as the reactions to errors that must be installed in the safety related application.
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> SINT: Safe Interrupt status
         <br/>**NOTE** : Not available in TI HDSL Solution.
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> SSUM: Remote event monitoring
         - 0 = All DSL Slave events are deleted.
         - 1 = The DSL Slave has signaled an event.

         When the bit is set, an error or a warning has been transmitted from the DSL Slave. The safety related application must check the SAFE_SUM register to obtain a detailed description.
<tr>
    <td>
    <td>
    <td> 5
    <td> SCE: Error on the Safe Channel
         - 0 = Safe Channel data was correctly transmitted.
         - 1 = Data consistency error on the Safe Channel.

         This error usually indicates a transmission error on the DSL connection. If this error occurs frequently, the wiring of the DSL connection should be checked. If this error occurs continuously, there is probably an error in the motor feedback system. This error affects quality monitoring and produces the QMLW warning or a protocol reset.
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> VPOS: Safe position error
         - 0 = The safe position is correct.
         - 1 = Sensor error.

         This error usually indicates an encoder sensor error. If this error occurs continuously, there is probably an error in the motor feedback system.
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> QMLW: Quality monitoring low value warning
         - 0 = Quality monitoring value greater than or equal to “14”
         - 1 = Quality monitoring value (see register 0x03) below “14”

         This warning indicates that a transmission error occurred. If this error occurs frequently, the wiring of the DSL connection should be checked.
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> PRST: Protocol reset warning
         - 0 = Normal protocol action.
         - 1 = The forced protocol reset was triggered.

         This error message indicates that the protocol connection to the DSL Slave has been re-initialized. This error message can be caused by a frequency inverter application request (PRST bit in SYS_CTRL), a safety related application request (PRST bit in SAFE_CTRL), or generated by the DSL Master itself. The DSL Master causes a protocol reset if too many transmission errors indicate a connection problem. A protocol reset causes a re-synchronization with the DSL Slave that can improve the connection quality.
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> MIN: Message init
         - 0 = No acknowledgment for the initialization received.
         - 1 = An acknowledgment was received from the Slave for the initialization of a message.

         When this warning is displayed, the Parameters Channel is still in the initialization status and no “short message” or “long message” can be triggered.
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> FRES: Channel free for “short message”
         - 0 = No “short message” can be sent.
         - 1 = A “short message” can be sent on the Parameters Channel.

         If the bit is set, the frequency inverter application can trigger a “short message”. Provided no answer has been received from the DSL Slave, this bit remains deleted. As the processing duration of a “short message” in the motor feedback system is not specified, a time limit condition is installed in the DSL Master. If the time limit is exceeded, attempts are made again automatically.
</tr>
<tr>
    <td> MASK_S
    <td> 0x3E
    <td>
    <td> Safe Event Mask
         - In the safe event mask register, the events are set with which the safe event interrupt is set.
         - Several events can be masked to trigger an safe event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> MSSUM: Mask for remote event monitoring
         - 0 = DSL Slave events that are set in the SAFE_SUM register do not set the safe event interrupt.
         - 1 = DSL Slave events that are set in the SAFE_SUM register set the safe event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> MSCE: Mask for transmission errors on the Safe Channel
         - 0 = A transmission error on the Safe Channel does not set the safe event interrupt.
         - 1 = A transmission error on the Safe Channel sets the safe event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> MVPOS: Mask for safe position error
         - 0 = An error in the safe position does not set the safe event interrupt.
         - 1 = An error in the safe position sets the safe event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> MQMLW: Mask for low quality monitoring value warning
         - 0 = A low quality monitoring value does not set the safe event interrupt.
         - 1 = A low quality monitoring value (see registers 03h and 05h) sets the safe event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> MPRST: Mask for protocol reset warning
         - 0 = A protocol reset does not set the safe event interrupt.
         - 1 = A protocol reset sets the safe event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> MMIN: Mask for message initialization confirmation
         - 0 = The acknowledgment for the initialization of a DSL Slave message does not set the safe event interrupt.
         - 1 = The acknowledgment for the initialization of a DSL Slave message sets the safe event interrupt.
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> MFRES: Mask for channel free for “short message”
         - 0 = If a “short message” can be sent on the Parameters Channel, the safe event interrupt is not set.
         - 1 = If a “short message” can be sent on the Parameters Channel, the safe event interrupt is set.
</tr>
<tr>
    <td> RESERVED
    <td> 0x3F
    <td>
    <td>  **NOTE** : Reserved for future use
</tr>
<tr>
    <td> SLAVE_REG_CTRL
    <td> 0x40
    <td>
    <td> Short Message Control
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> Short message, read/write mode
         - 0 = "Short message" write operation
         - 1 = "Short message" read operation
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 5:0
    <td> 6 bit address for a “short message” operation
</tr>
<tr>
    <td> ACC_ERR_CNT_TRESH
    <td> 0x41
    <td>
    <td> Fast Position Error Counter Threshold
         <br/> **NOTE : Different implementation from SICK HDSL MASTER IP Core**
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> 8 bit threshold value for triggering a protocol reset when ACC_ERR_CNT crosses this threshold.
</tr>
<tr>
    <td> RESERVED
    <td> 0x43, 0x42
    <td>
    <td>  **NOTE** : Reserved for future use
</tr>
<tr>
    <td> VERSION2
    <td> 0x44
    <td>
    <td> Version in Safe Channel 2 (Identical to VERSION register)
         <br/> **NOTE : Different implementation from SICK HDSL MASTER IP Core**
         <br/> **NOTE : Register address is different from SICK HDSL MASTER IP Core (TI implementation uses 0x44 instead of 0x0B)**
</tr>
<tr>
    <td>
    <td>
    <td> 7:4
    <td> Major Release Number
</tr>
<tr>
    <td>
    <td>
    <td> 3:0
    <td> Minor Release Number
</tr>
<tr>
    <td> ENC2_ID
    <td> 0x45
    <td>
    <td> Encoder ID in Safe Channel 2
         <br/> **NOTE : Not available in TI HDSL Solution**
         <br/> **NOTE : Register address is different from SICK HDSL MASTER IP Core (TI implementation uses 0x45 instead of 0x0F)**
</tr>
<tr>
    <td> STATUS2
    <td> 0x46
    <td>
    <td> Safe Channel 2 Status
         - This register contains the status information for Safe Channel 2 of the HDSL motor feedback system.
         - A summary of the contents is also available in the SUM2 bit of Online Status 2.
         <br/> **NOTE : Register address is different from SICK HDSL MASTER IP Core (TI implementation uses 0x46 instead of 0x18)**
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> TOG2: Safe Channel 2 toggle bit
         For successive position transmissions on Safe Channel 2, TOG2 must always toggle between “0” and “1”. The starting value for TOG2 is “0”. If the toggle bit does not change its value, it is probable that a transmission error occurred and the transmitted absolute value for Safe Channel 2 is invalid. Suitable measures must be installed in the user application.
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> TEST2: Safe Channel 2 has just been tested
         TEST2 is set if a test is carried out during the currently available Safe Channel 2 status and position values.TEST2 can only be valid if the user application has previously requested a test. Corresponding error indications for TEST2 are either the ERR2 bit or a discrepancy between the position and the CRC of Safe Channel 2.
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> ERR2: Safe Channel 2, position error
         - 0 = The last safe position received in Safe Channel 2 is correct.
         - 1 = The last safe position received in Safe Channel 2 is invalid. Suitable measures must be installed in the user application.
</tr>
<tr>
    <td>
    <td>
    <td> 4:0
    <td> FIX2: Safe Channel 2, fixed bit pattern
         The standard value of the fixed bit pattern is “11100”. All other values indicate an error on Safe Channel 2 of the DSL system. Suitable measures must be installed in the user application.
</tr>
<tr>
    <td> VPOS24
    <td> 0x47
    <td>
    <td> Safe Position, Channel 2 (Byte 4)
         - The VPOS2 registers for the safe position contain the position value from the secondary channel of the motor feedback system connected.
         - This safe position is transmitted in every eighth protocol frame if the validity of the data transfer has been checked.
         - Only those VPOS2 bits are relevant that lie within the range that the motor feedback system has actually measured.
         - Also, typically channel 2 has a lower resolution than channel 1.
         - The units of the position value are (steps).
         <br/> **NOTE : Register address is different from SICK HDSL MASTER IP Core (TI implementation uses 0x47 instead of 0x19)**
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 4 of position value at motor feedback system Safe Channel 2 (length: 40 bits), as an absolute value complement.
</tr>
<tr>
    <td> VPOS23
    <td> 0x48
    <td>
    <td> Safe Position, Channel 2 (Byte 3)
         <br/> **NOTE : Register address is different from SICK HDSL MASTER IP Core (TI implementation uses 0x48 instead of 0x1A)**
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 3 of position value at motor feedback system Safe Channel 2 (length: 40 bits), as an absolute value complement.
</tr>
<tr>
    <td> VPOS22
    <td> 0x49
    <td>
    <td> Safe Position, Channel 2 (Byte 2)
         <br/> **NOTE : Register address is different from SICK HDSL MASTER IP Core (TI implementation uses 0x49 instead of 0x1B)**
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 2 of position value at motor feedback system Safe Channel 2 (length: 40 bits), as an absolute value complement.
</tr>
<tr>
    <td> VPOS21
    <td> 0x4A
    <td>
    <td> Safe Position, Channel 2 (Byte 1)
         <br/> **NOTE : Register address is different from SICK HDSL MASTER IP Core (TI implementation uses 0x4A instead of 0x1C)**
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 1 of position value at motor feedback system Safe Channel 2 (length: 40 bits), as an absolute value complement.
</tr>
<tr>
    <td> VPOS20
    <td> 0x4B
    <td>
    <td> Safe Position, Channel 2 (Byte 0)
         <br/> **NOTE : Register address is different from SICK HDSL MASTER IP Core (TI implementation uses 0x4B instead of 0x1D)**
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 0 of position value at motor feedback system Safe Channel 2 (length: 40 bits), as an absolute value complement.
</tr>
<tr>
    <td> VPOSCRC2_H
    <td> 0x4C
    <td>
    <td> Position checksum, Channel 2 (High Byte)
         - The VPOSCRC2 registers for the position checksum contain the CRC checksum of the safe position VPOS2 and STATUS2.
         - The CRC is checked in the DSL Master IP Core.
         - In order to guarantee, in a safety related application, that the CRC machine in the IP Core is functioning, these registers can be checked with an external cross check in the diagnostics test interval.
         - The CRC is generated with the following CRC parameters:
            - CRC Sequence : 16 Bit
            - CRC Polynomial : 0xC86C (x<sup>16</sup> + x<sup>15</sup> + x<sup>12</sup> + x<sup>7</sup> + x<sup>6</sup> + x<sup>4</sup> + x<sup>3</sup> + 1), Normal representation: 0x90D9
            - Starting Value : 0x0000
            - Closing XOR Value : 0x00FF
            - Reverse Data Bytes : No
            - Reverse CRC before closing XOR : No
            - Sequence of the bytes for calculation : STATUS2, VPOS24, VPOS23, VPOS22, VPOS21, VPOS20
         <br/> **NOTE : Register address is different from SICK HDSL MASTER IP Core (TI implementation uses 0x4C instead of 0x1E)**
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 1 of 16 bit CRC checksum (CRC 16)of the safe position and status summary in Safe Channel 2.
</tr>
<tr>
    <td> VPOSCRC2_L
    <td> 0x4D
    <td>
    <td> Position checksum, Channel 2 (Low Byte)
         <br/> **NOTE : Register address is different from SICK HDSL MASTER IP Core (TI implementation uses 0x4D instead of 0x1F)**
</tr>
<tr>
    <td>
    <td>
    <td> 7:0
    <td> Byte 0 of 16 bit CRC checksum (CRC 16) of the safe position and status summary in Safe Channel 2.
</tr>
<tr>
    <td> POSTX
    <td> 0x4E
    <td>
    <td> Position Transmission Status
         <br/> **NOTE : Different implementation from SICK HDSL MASTER IP Core**
</tr>
<tr>
    <td>
    <td>
    <td> 7:2
    <td> **NOTE** : Reserved (Read as \"0\")
</tr>
<tr>
    <td>
    <td>
    <td> 1:0
    <td> - 0: Position request is transmitted to the DSL encoder
         - 1: Reserved
         - 2: Fast position was received or position newly updated by estimator
         - 3: Safe position 1 and 2 were received
</tr>
<tr>
    <td> RESERVED
    <td> 0x4F
    <td>
    <td>  **NOTE** : Reserved for future use
</tr>
<tr>
    <td> ONLINE_STATUS_D_H
    <td> 0x50
    <td>
    <td> Online Status D (High Byte)
         - The Online Status D is a non-storing copy of registers EVENT_H and EVENT_L. The static information in these event registers must be deleted by the user after the read process, by writing the value "0" to the corresponding bit in the register, whilst the Online Status D only shows the current status without storing previous indications.
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> INT: Status of the Interrupt output
         <br/> **NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> SUM: Summary byte
         - 0 = The last valid value from SAFE_SUM was zero.
         - 1 = The last valid value from SAFE_SUM was not zero. The importance of this flag depends on the particular error source that leads to a set SAFE_SUM.
         <br/> **NOTE : Different implementation from SICK HDSL MASTER IP Core**
         <br/> SAFE_SUM is used instead of MIR_SUM.
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> FIX0: This bit always gives a “0”.
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> FIX1: This bit always gives a “1”.
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> POS: Estimator turned on
         - 0 = No fast position error.
         - 1 = A source of an error in the fast position was identified or an alignment procedure is currently being carried out. It is probable that the last fast position is invalid.
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> FIX0: This bit always gives a “0”.
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> DTE: Estimator Deviation Threshold Error
         - 0 = Current value of deviation smaller than the specified maximum.
         - 1 = Current value of deviation greater than the specified maximum.
        <br/> **NOTE : Not available in TI HDSL Solution.**
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> PRST: Protocol reset
         - 0 = Normal protocol action
         - 1 = The forced protocol reset was triggered
</tr>
<tr>
    <td> ONLINE_STATUS_D_L
    <td> 0x51
    <td>
    <td> Online Status D (Low Byte)
</tr>
<tr>
    <td>
    <td>
    <td> 7:6
    <td> FIX0: This bit always gives a “0”.
         <br/> **NOTE : Different implementation from SICK HDSL MASTER IP Core**
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> MIN: Acknowledgment of message initialization
         - 0 = Parameter Channel not functioning.
         - 1 = The DSL encoder sends a figure by which the initialization of the Parameter Channel is acknowledged.
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> ANS: Incorrect answer detected.
         - 0 = No error detected in the last answer to a long message.
         - 1 = The last answer to a long message was damaged.
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> FIX0: This bit always gives a “0”.
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> QMLW: Quality monitoring at Low level
         - 0 = Current value of quality monitoring greater than or equal to 14.
         - 1 = Current value of quality monitoring less than 14.
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> FREL: Channel status for “long message”.
         - 0 = The channel for the “long message” is in use.
         - 1 = The channel for the “long message” is free.
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> FIX0: This bit always gives a “0”.
</tr>
<tr>
    <td> ONLINE_STATUS_1_H
    <td> 0x52
    <td>
    <td> Online Status 1 (High Byte)
         - The Online Status D is a non-storing copy of registers EVENT_S. The static information in the event register must be deleted by the user after the read process, by writing the value "0" to the corresponding bit in the register, whilst the Online Status 1 only shows the current status without storing previous indications.
         - All fault indications in Online Status 1 are potentially critical and safety-related. Suitable measures must be installed in the user application.
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> SINT: Status of the Interrupt output
         <br/> **NOTE : Not available in TI HDSL Solution**
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> SSUM: Safe Summary bit
         - 0 = The last valid value from SAFE_SUM was zero.
         - 1 = The last valid value from SAFE_SUM was not zero. The importance of this flag depends on the particular error source that leads to a set SAFE_SUM.
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> SCE: CRC error on the Safe Channel
         - 0 = The last Safe Channel 1 CRC received was correct.
         - 1 = The last Safe Channel 1 CRC received was wrong. It is expected that the last safe position 1 transmitted is invalid.
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> FIX1: This bit always gives a “1”.
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> FIX0: This bit always gives a “0”.
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> VPOS: Safe position invalid
         - 0 = The last safe position received was correct.
         - 1 = An error in the safe position was identified. It is expected that the safe position transmitted from the encoder is invalid.
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> FIX0: This bit always gives a “0”.
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> PRST: Protocol reset
         - 0 = Normal protocol action
         - 1 = The forced protocol reset was triggered
</tr>
<tr>
    <td> ONLINE_STATUS_1_L
    <td> 0x53
    <td>
    <td> Online Status 1 (Low Byte)
</tr>
<tr>
    <td>
    <td>
    <td> 7:6
    <td> FIX0: This bit always gives a “0”.
         <br/> **NOTE : Different implementation from SICK HDSL MASTER IP Core**
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> MIN: Acknowledgment of message initialization
         - 0 = Parameter Channel not functioning.
         - 1 = The DSL encoder sends a figure by which the initialization of the Parameter Channel is acknowledged.
</tr>
<tr>
    <td>
    <td>
    <td> 4:3
    <td> FIX0: This bit always gives a “0”.
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> QMLW: Quality monitoring at Low level
         - 0 = Current value of quality monitoring greater than or equal to 14.
         - 1 = Current value of quality monitoring less than 14.
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> FIX0: This bit always gives a “0”.
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> FRES: Channel status for the “short message”.
         - 0 = The channel for the “short message” is in use.
         - 1 = The channel for the “short message” is free.
</tr>
<tr>
    <td> ONLINE_STATUS_2_H
    <td> 0x54
    <td>
    <td> Online Status 2 (High Byte)
         - Online Status 2 provides information about Safe Channel 2 of the DSL encoder.
         - The data always indicate the current status, with previous indications not being stored.
         - All fault indications in Online Status 2 are potentially critical and safety-related. Suitable measures must be installed in the user application.
</tr>
<tr>
    <td>
    <td>
    <td> 7
    <td> FIX0: This bit always gives a “0”.
</tr>
<tr>
    <td>
    <td>
    <td> 6
    <td> SUM2: Summary byte Channel 2
         - 0 = Neither TEST2 nor ERR2 is set.
         - 1 = One of the indications TEST2 or ERR2 is set. The error reaction to this flag depends on the meaning of the bit they are based on.
</tr>
<tr>
    <td>
    <td>
    <td> 5
    <td> SCE2: Transmission error Channel 2
         - 0 = The last data received in Channel 2 was correct.
         - 1 = The last Safe Channel 2 CRC received was wrong. It is expected that the last safe position 2 transmitted is invalid. Suitable measures must be installed in the user application.
</tr>
<tr>
    <td>
    <td>
    <td> 4
    <td> FIX1: This bit always gives a “1”.
</tr>
<tr>
    <td>
    <td>
    <td> 3
    <td> FIX0: This bit always gives a “0”.
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> VPOS2: Safe position Channel 2 invalid
         - 0 = The last safe position received in Channel 2 was correct.
         - 1 = A source of an error in the safe position in Channel 2 was identified. It is probable that the safe position transmitted from Channel 2 is invalid. Suitable measures must be installed in the user application.
</tr>
<tr>
    <td>
    <td>
    <td> 1
    <td> FIX0: This bit always gives a “0”.
</tr>
<tr>
    <td>
    <td>
    <td> 0
    <td> PRST: Protocol reset
         - 0 = Normal protocol action
         - 1 = The forced protocol reset was triggered
</tr>
<tr>
    <td> ONLINE_STATUS_2_L
    <td> 0x55
    <td>
    <td> Online Status 2 (Low Byte)
</tr>
<tr>
    <td>
    <td>
    <td> 7:3
    <td> FIX0: These bits always gives a “0”.
         <br/> **NOTE : Different implementation from SICK HDSL MASTER IP Core**
</tr>
<tr>
    <td>
    <td>
    <td> 2
    <td> QMLW: Quality monitoring at Low level
         - 0 = Current value of quality monitoring greater than or equal to 14.
         - 1 = Current value of quality monitoring less than 14.
</tr>
<tr>
    <td>
    <td>
    <td> 1:0
    <td> FIX0: These bits always gives a “0”.
</tr>
</table>

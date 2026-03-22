# EnDat Protocol Design {#ENDAT_DESIGN}

[TOC]

## Introduction

This design implements EnDat receiver (a.k.a subsequent electronics) on TI Sitara™ processors/microcontrollers.
EnDat is a digital bidirectional serial interface for position encoders, also suited for safety-related applications.
Only four signal lines are required, differential pairs each for clock and data.
Clock is provided by the receiver and data is bidirectional. Data is transmitted in synchronism with the clock.
Transfer between receiver and encoder at the physical layer is in accordance with RS485, with transceivers at both ends.

## System Overview

Position feedback system consists of a position encoder attached to a motor, up to 100 meters of cable which provides power and serial communication, and the receiver interface for the position encoder.
In the case of the Sitara™ processor/microcontroller, the receiver interface for the position encoder is just one function of a connected drive controller.
The AM64x/AM243x/AM26x provides, in addition to the resources for Industrial Ethernet and motor control application, including on-chip ADCs, Delta Sigma demodulator for current measurement.
EnDat receiver on Sitara™ processor/microcontroller uses one PRU-ICSS slice.
Clock, data transmit, data receive and receive enable signals from PRU of ICSS are available in Sitara™ processors/microcontrollers.

## Implementation

The EnDat receiver function is implemented on TI Sitara™ processors/microcontrollers.

Design is split into three parts:
    1. EnDat hardware support in PRU using three channel peripheral interface
    2. Firmware running in PRU
    3. Driver running in Arm®-based core

Application is supposed to use the EnDat driver APIs to leverage EnDat functionality.

Default SDK examples three channel peripheral interface in \if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PRU0 of PRU-ICSSM \else Slice 1 (either 1 core or 3 cores based on the configuration) of PRU-ICSSG0 \endif.

###  Specifications

<table>
<tr>
    <th>Parameter
    <th>Value
    <th>Details
</tr>
<tr>
    <td>Maximum Cable Length
    <td>100m
    <td>Supports up to 8MHz with delay compensation
</tr>
<tr>
    <td>Maximum Frequency
    <td>16 MHz
    <td>Supports up to 20m cable \if (SOC_AM243X || SOC_AM64X) \note Tested up to 8MHz in multi-channel single PRU mode \endif
</tr>
<tr>
    <td>Startup/Initialization Frequency
    <td>200 KHz
    <td>After power on or reset
</tr>
<tr>
    <td>CRC
    <td>6 bits
    <td>Position/data verification
</tr>
<tr>
    <td>Receive oversample ratio
    <td>8
    <td>
</tr>
</table>

### EnDat PRU hardware

Refer to TRM for details

### EnDat Firmware Implementation

Following section describes the firmware implementation of EnDat receiver on PRU-ICSS.

\cond SOC_AM243X || SOC_AM64X

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

#### Implementation for Single PRU
Single core of PRU-ICSS slice is used in this configuration.

\image html endat_module_integration.png "Arm-based core, PRU, EnDat module Integration for Single PRU configuration"

\if (SOC_AM243X || SOC_AM64X)
#### Implementation for Multi Channel with encoders of different make
Each of PRU, TX-PRU and RTU-PRU handle one channel in this configuration. Load share mode is enabled in case of multi make encoders.

\image html Endat_load_share_mode.png "PRU, EnDat module Integration for "Multi Channel with encoders of different make" configuration"

\endif
#### EnDat Firmware Flow {#ENDAT_DESIGN_FLOW}

\image html endat_overall_block_diagram.png "Overall Block Diagram"

Firmware first does initialization of PRU-ICSS's Three Channel Peripheral Interface and EnDat encoder.
Then it checks the operation mode: host trigger mode, periodic CMP mode, or periodic CAP mode.

**Host Trigger Mode:** The firmware waits until a command has been triggered through the interface by the host application.

**Periodic CMP Mode (Compare Event Mode):** In CMP mode, IEP timer compare event triggers position sampling. The firmware monitors the configured IEP compare event and automatically initiates EnDat transactions when the IEP timer counter matches the compare value. This enables fixed-rate periodic sampling.

**Periodic CAP Mode (Capture Event Mode):** In CAP mode, external signals trigger position sampling through IEP capture events. The capture event is triggered on the rising edge of the external input pulse, enabling event-driven position capture. \if (SOC_AM243X || SOC_AM64X) Internal signals can also be mapped to IEP capture events via TIMESYNC/GPIOMUX router. \else Internal signals can also be mapped to IEP capture events via XBAR. \endif

The following is the operation flow for the periodic mode:

1. The firmware polls the IEP CMP/CAP status register and clears the status after the event is detected.
2. When the event is detected, the firmware initiates an EnDat transaction.
3. The position data is automatically updated in the shared memory.
4. After the transaction is complete, the firmware triggers a R5F interrupt.
5. The firmware checks the current trigger mode. If still in periodic mode, it returns to step 1 to wait for the next IEP CMP/CAP event. If the mode has been switched to host trigger mode, the firmware stops periodic operation.

\cond SOC_AM243X
\note In load share mode, each channel can have independent IEP CMP/CAP event configuration.
\endcond

\attention Input cycle time (CMP mode) or external trigger period (CAP mode) should be greater than or equal to the EnDat communication cycle time.

If it is a normal command, it reads command, its attributes like transmit bits, receive bits etc., then it transmits the data and collects the data sent by the encoder stored onto a buffer with one byte representing a bit (since oversample ratio of 8 is used).
Next it checks whether there is 2.2 command supplement to be transmitted based on attributes, if so it transmits it.
The received data is now downsampled to extract bit from oversampled 8 bits and the result written to the defined PRU RAM locations.

If command requested is continuous mode, 2.1 position command will be transmitted. During receive, it is different from the normal mode that downsampling is done on-the-fly, i.e. downsampling is done as soon as each bit is received. This is done due to the timing constraints with continuous mode, as data is continuously being received.

At the end of transaction as requested by the user, trigger bit that is set by the user is unset.
User can wait on this bit to know that the command has been completed.
EnDat driver provides API to achieve this.

#####	 Initialization
######  Initialization for "Single Channel" \if (SOC_AM243X || SOC_AM64X)  and "Multi Channel with encoders of same make" \endif configurations
\image html endat_initialization.png "Initialization for Single PRU mode"

\if (SOC_AM243X || SOC_AM64X)
###### Initialization for "Single Channel" and "Multi Channel with encoders of different make" configuration
\image html endat_load_share_mode_initialization.png "Initialization for Load share mode"

\endif
Before executing the firmware, the Arm-based core needs to enable 3 channel peripheral interface in PRU-ICSS first, then configure the clock to 200KHz, with oversample ratio of 8 (hence receive clock would be 200 * 8 KHz).
The entire EnDat configuration MMRs are cleared. Through the defined interface (PRU RAM location), user requested channel is determined in Single PRU configuration.
Then power-on-init as per specification is implemented, after which encoder is reset by sending reset command.
Firmware sets up the command and its attribute for all the commands that are sent during initialization. Alarms, errors and warning are cleared.
Firmware then determines number of clock pulses for position and whether encoder supports EnDat 2.2. Propagation delay is then estimated.
\note The propagation delay is measured by the PRU firmware using the PRU cycle counter during initialization. The delay is averaged over 8 samples. The calculated propagation delay can vary by a maximum of -/+ 2 PRU cycles between measurements.

If user has required for clock to be configured, it is obeyed, else it defaults to 8MHz. At the end of the initialization status is updated.

\if (SOC_AM243X || SOC_AM64X)
###### Synchronization among PRU cores for "Multi Channel with encoders of different make" configuration

If using "Multi Channel with encoders of different make" configuration where load share mode is enabled, one of the cores among enabled cores will be set as the primary core for performing global configurations of PRU-ICSS's EnDat interface. These global configurations include clock frequency configuration and TX global re-initialization.

There needs to be a synchronization between PRUs before changing any global configuration. For this purpose, each active PRU core sets synchronization bit before any operation needing synchronization and clears the synchronization bit when it is ready. The assigned primary core will wait for all active channel's synchronization bits to be cleared and then perform the global configuration.
\endif

#####	Send and Receive

\image html endat_send_receive.png "Send And Receive"


If requested command attribute indicates 2.2 command supplement, clock is configured to free run stop low mode, else to free run stop high.
Command is written to the transmit FIFO and send routine followed by receive is invoked.

######	Send

\image html endat_send.png "Send"

Transmit and receive frame sizes are configured in PRU EnDat hardware.
With long cables, it may be required to configure receive frame size lesser than receive bits so that extra clocks are not sent to the encoder.
If transmit was going on, it will wait till it has finished and then transmit GO bit is set, which would start the new transmission.

######	Receive

\image html endat_receive.png "Receive"


Receive bits obtained via command attribute are stored as header (initial 2 bytes) in the receive buffer.
Then it waits till receive valid flag has been set. Once set, 1 byte corresponding to 1 bit (because of oversampling of 8) is read and stored in receive buffer and the flags are cleared.
Receive buffer pointer is incremented and receive bit count decremented. This continues till count is zero, once zero, it extracts receive data one more time to take care of SB (receive count excludes SB).
If 2.2 command supplement is not present, transmit re-init is done.
If using "Multi Channel with encoders of different make" configuration where load share mode is enabled, the primary core waits for synchronization bits for active channels to be cleared before performing TX Global Init.

\image html Endat_Load_share_receive.png "Receive In load share mode"

###### EnDat 2.2 Command Supplement Send

\image html endat_2_2_supplement_send.png "EnDAT 2.2 command supplement send"

Clock mode is configured to stop low after transmit. 2.2 command supplement to be transmitted is written to FIFO preceded by 7 dummy bits and SB.
Transmission is configured to transmit till end of the FIFO. Transmission is started after making sure that transmit module is not busy.

###### Receive Downsample

\image html endat_receive_downsampling.png "Downsampling"

This is the most complex portion of the firmware. Received data is exposed through PRU interface in four bytes.
First two words (word = 4 bytes) holds the position data, third holds additional information 2 (if only second additional info is present or both present) or 1 (if only first additional info is present) and the last additional information 1 (if both present).
The order is as mentioned in EnDat 2.2 specification. Splitting the received data on word boundaries when additional info's are present causes the complexity here.

If command is neither 2.2 nor position request or if no additional info is present, handling is easy – just copy the received data into initial 2 words in the order it is received.
If command is 2.2 position command and depending on the number of additional info's, markers (used in the downsampling loop) are set to write additional info's to next word boundaries.
If only one addinfo is present, marker "rx pos bits" stores clocks required to receive position (inclusive of CRC, F1 and F2).
If both addinfo's are present another marker is set to 2 words (first 2 words holds the position) plus 30 bit to account for addinfo.
If markers are not required, then their values are set so that they never match the counting receive bits, hence the value "0xff".

After updating the marker, number of received bits is retrieved from the receive buffer header. SB is skipped for downsampling and the result registers are cleared.
Next, each byte (8 bit, oversample of 8) is read from the receive buffer, 4th bit of each decides the actual received bit. This is continued till the end of receive buffer.
If during the loop, receive bit count matches any of the markers, bit count is updated appropriately. This helps in naturally bringing the received data as per the word format specified by the interface.
In the loop, as the number of bits reaches word boundary, it will start saving received data to next word. At the end of the loop, the last word is copied to the result register.

###### Continuous mode

\image html endat_continuous_mode.png "Continuous Mode"

2.1 position command as well as its attribute that has been set up by the user is read first. Clock is configured for free run mode. Position command is written to FIFO and send routine is invoked.
Then receive is done along with on-the-fly downsampling. This is required as time between receipt of successive position data is less than the time that downsampling routine (mentioned earlier) takes.
Once data is read and downsampled on-the-fly, command trigger interface is read to see if user wants to stop continuous mode; if so, do transmit re-init, disable receive and wait till the end of re-init.

###### Receive and On-The-Fly Downsample

\image html endat_on_fly_downsampling.png "Endat on the fly Downsampling"

Two registers (a word each) that hold the result are cleared initially. Upon receiving the first receive valid, it discards it and proceeds to wait for the next one as the first one is SB.
Thereafter for every valid flag set, 4th bit in the received byte is checked to find the actual received bit and it is stored, word crossing is also taken care of.
After all the bits for a position command are received, receive is disabled and is activated only after 2T clock cycles – this is to prevent falsely detecting SB immediately (upon calling this routine back-to-back as mentioned in previous section) after encoder has finished sending data as it can pull data line high for 2T more clock cycles.

####  Recovery Time Measurement
The recovery time is defined as the high period of the EnDat data signal at the end of every transmission. This high period is a key metric because it is related to the encoder's internal clock frequency and a parameter stored within the encoder.
The factory default settings for the Recovery Time is programmed to 10us <= recovery time <= 30us. It can only be changed to 1.25us <= recovery time <=3.75us for type 2.2 mode commands. For clock pulse frequency <= 1MHz, recovery time must be set to 10us <= recovery time <= 30us.
The User can set the function parameters in word 3 at "0xB9" memory area for recovery time range. If bit 0th is unset and 1st bit is set of word3 then recovery time will belong to large range(10us-30us) and if 0th bit is set and 1st bit is unset of word3 then recovery time will belong to short range(1.25us to 3.75us).

##### Counter for Measuring Recovery time

This is a free-run counter, clocked by the PRU cycle counter. If the counted value deviates from an expected tolerance range, it signals an issue with the encoder's clock frequency.
This section outlines how the counter measures recovery time.
Measurement Process: Recovery time is measured by calculating the time difference between two key events in the EnDat protocol:
- Start Trigger: The rising edge of the EnDat clock (positive signal).
- Stop Trigger: The falling edge of the EnDat data signal (negative signal).

Counter Operation: The counter value is stored in a 32-bit recovery time register in memory. This value is dynamic and changes during normal operation to detect any "stuck-at" errors.
The expected recovery time is derived from the difference between the last and current counter values.

Multi-Channel Load Share Mode: In this mode, counters for different axes are initialized with different starting values.
To enhance fault detection, especially in systems with multiple EnDat receivers, the counters for different axes are initialized with distinct starting values. The difference between these starting values is kept greater than twice the expected tolerance range for the recovery time.

##### Method for measuring the recovery time for position command
\image html Endat_Recovery_Time_For_Position.png "Endat Recovery time for Endat 2.2 position command "
\image html Endat_RT_FlowChart_for_position.png "Endat Recovery time flow-chart for Endat 2.2 position command"
1. After the CRC bits are received, there is a wait for rising clock edge.
2. Start the measurement of Recovery Time using PRU cycle counter (The cycle counter is set to zero).
3. Wait for falling edge of the data from encoder (RX).
4. Read the PRU cycle counter which gives the value of Recovery Time in PRU Clock Cycle units
5. Update recovery time counters


##### Method for measuring the recovery time for supplement command
\image html Endat_Recovery_Time_For_Supplement.PNG "Endat Recovery time for Endat 2.2 supplement command "
\image html Endat_RT_FlowChart_for_supplement.png "Endat Recovery time flow-chart for Endat 2.2 supplement command"
1. After TX_GO bit is set which starts the TX, wait for TX FIFO level to reach 0
2. In case of Single Channel or Multi Channel with encoders of different make mode, wait for RX enable. But In case of Multi Channel with encoders of same make mode, wait for TX complete.
3. After the CRC bits are received, there is a wait for rising clock edge.
4. Start the measurement of Recovery Time using PRU cycle counter (The cycle counter is set to zero).
5. Wait for falling edge of the data from encoder (RX).
6. Read the PRU cycle counter which gives the value of Recovery Time in PRU Clock Cycle units
7. Update the recovery time counters

\if (SOC_AM243X || SOC_AM64X)
##### NOTE for Multi-channel Single PRU Mode
 As the same PRU has to poll for three channels in this mode, sequential polling is done for each channel. Accuracy of recovery time measurement will be less than single channel mode or multi channel load share mode.

1. Wait for rising edge in clock for all connected channels
2. Start the measurement of Recovery Time using PRU cycle counter (The cycle counter is set to zero).
3. Wait for RX completion on all connected channels, and check completion for all connected channels one by one. Whenever completion is detected for a channel, save the PRU cycle counter value and continue for remaining channels.
\endif

### EnDat Hardware interface

The physical data transmission in EnDat is done using RS-485 standard. The data is transmitted as differential signals using the RS485 between the EnDat receiver and the encoder.

The receiver sends the clock to the EnDat encoder, data transmission in either direction (one at a time) occurs in synchronism with the clock. The design uses two differential signals for each of the lines (clock and data).

EnDat receiver and the encoder are connected using the RS-485 transceiver. Data is transmitted differentially over RS-485. It has the advantages of high noise immunity and long distance transmission capabilities.

#### Pin Multiplexing {#ENDAT_PIN_USAGE}

\attention \ref PRUICSS_PERIPHERAL_IF_MODE_SIGNAL_CONFIGURATION section has details on PRU pin functions in Peripheral IF mode

\note
    - k = 0,1 (PRU-ICSS Instance) for AM243x/AM261x and k = 0 for AM263Px
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
    <td>pru<n>_endat0_out_en
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
    <td>pru<n>_endat1_out_en
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
    <td>pru<n>_endat2_out_en
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
    <td>pru1_endat0_out_en
    <td>PRU1 Channel 0 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI13
    <td>pru1_endat0_in
    <td>PRU1 Channel 0 receive when SA mux selection is enabled (ICSSG_SA_MX_REG[7] G_MUX_EN = 1)
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
    <td>pru1_endat2_out_en
    <td>PRU1 Channel 2 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI11
    <td>pru1_endat2_in
    <td>PRU1 Channel 2 receive
</tr>
<tr>
    <td>GPIO Pin (GPIO1_78/C16)
    <td>ENC0_EN
    <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
<tr>
    <td>GPIO Pin (GPIO1_77/B17)
    <td>ENC2_EN
    <td>Enable encoder voltage in Axis 2 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
</table>

\endcond

\cond SOC_AM261X
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
    <td>pru0_endat0_out_en
    <td>PRU0 Channel 0 transmit enable
</tr>
<tr>
    <td>PR1_PRU0_GPI9
    <td>pru0_endat0_in
    <td>PRU0 Channel 0 receive
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
    <td>pru1_endat0_out_en
    <td>PRU1 Channel 0 transmit enable
</tr>
<tr>
    <td>PR1_PRU1_GPI9
    <td>pru1_endat0_in
    <td>PRU1 Channel 0 receive
</tr>
<tr>
    <td>GPIO Pin (GPIO_21/B10)
    <td>ENC0_EN (PRU0)
    <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
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
    <td>pru0_endat1_out_en
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

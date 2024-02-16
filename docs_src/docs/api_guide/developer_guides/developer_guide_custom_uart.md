# How to modify Tamagawa software and firmware for Custom UART based Protocols ? {#DEVELOPER_GUIDE_CUSTOM_UART}

[TOC]

Various industries like Robotics, Industrial Automation, Manufacturing processes, CNC Machining, and Medical Devices require accurate and precise position control. Multiple UART-based encoder protocols in the market help to effectively measure position, speed, and direction of motion in various systems. It is a challenge to interface the different UART based encoders with a host processor in a multi-channel fashion due to factors like different data sizes and different baud rates used in various protocols.

The aim of this document is to showcase the capabilities of the Three Channel Peripheral Interface in PRU-ICSSG on Sitara Processors and to show steps for achieving a solution for interfacing different UART based encoders in the market and also provide flexibility in selection of baud rates for the encoder communication. The same solution can also be used for general purpose UART like debugging, logging, etc. For more details on Three Channel Peripheral Interface of PRU-ICSSG, please see section "6.4.5.2.2.3.6 Three Channel Peripheral Interface" of <a href="https://www.ti.com/lit/ug/spruim2h/spruim2h.pdf" target="_blank">AM243x Technical Reference Manual</a>.

There are three main reasons when it comes to creating a programmable software based solution for a custom multi-channel UART encoder interface:

1. Different sized data frames
2. Multiple channels
3. Multiple baud rates

This SDK contains software (which runs on ARM core) and firmware (which runs on PRU-ICSSG) which implement the interface for Tamagawa encoders. More details on Tamagawa implementation can be found in \ref TAMAGAWA and \ref TAMAGAWA_DESIGN pages.

There are three aspects of this firmware + software solution :
1. PRU firmware running on a PRU-ICSSG core
2. Driver running on ARM core
3. Application running on ARM core

## PRU firmware running on a PRU-ICSSG core

Firmware sources for Tamagawa are available in "${SDK_INSTALL_PATH}/source/position_sense/tamagawa/firmware" folder.

Following are different aspects of firmware which can be tweaked in firmware based on the need of custom UART implementation.

Firstly, initialization is done as shown in \ref TAMAGAWA_DESIGN_INITIALIZATION. Then the configuration for send and receive is done as shown in \ref TAMAGAWA_DESIGN_TX_RX.

### PRU-ICSS Internal Pinmuxing

PRU-ICSSG supports an internal wrapper multiplexing that expands the device top-level multiplexing. This wrapper multiplexing is controlled by the GPCFGx_REG register (where x = 0 or
1) in the PRU-ICSSG CFG register space and allows MII_RT, 3 channel Peripheral Interface, and Sigma Delta functionality to be muxed with the PRU GPI/O device signals. For this use-case, 3 channel Peripheral Interface should be configured.

#### Relevant Code Sections in Tamagawa

- Code under `TAMAGAWA_INIT` in "${SDK_INSTALL_PATH}/source/position_sense/tamagawa/firmware/tamagawa_main.asm"

### Shared Memory Definition

Communication is needed between ARM core and PRUs. For this purpose, data memory of PRU-ICSSG is typically used. The memory map of this shared memory region for Tamagawa is defined in one file. Based on the need, this memory map can be updated to add or remove data variables as per the need.

#### Relevant Code Sections in Tamagawa

- "${SDK_INSTALL_PATH}/source/position_sense/tamagawa/firmware/tamagawa_interface.h" file

### Clock Configuration for Interface Speed

The Peripheral Interface module has two source clock options, ICSSGn_UART_CLK (default) and ICSSGn_ICLK. There are two independent clock dividers (div16) for the 1x and oversampling (OS) clocks, and each clock divider is configurable by two cascading dividers in ICSSG_PRUx_ED_RX_CFG_REG and ICSSG_PRU0_ED_TX_CFG_REG registers.

The 1x clock is output on the clock signal. In TX mode, the output data is read from the TX FIFO at this 1x clock rate. In RX mode, the input data is sampled at the OS clock rate.Multiple options are available for start and stop conditions for the clock.

For more details on clocking capabilities and configuration, please see section "6.4.5.2.2.3.6.3 Clock Generation" of <a href="https://www.ti.com/lit/ug/spruim2h/spruim2h.pdf" target="_blank">AM243x Technical Reference Manual</a>.

#### Relevant Code Sections in Tamagawa

- Code under `TAMAGAWA_SET_CLOCK`, and `FN_SET_TX_CLK` in "${SDK_INSTALL_PATH}/source/position_sense/tamagawa/firmware/tamagawa_main.asm"

For Tamagawa, this configuration is done for 2.5 MHz or 5 MHz based on the encoder being used.

### Trigger Mode

After initialization, the firmware checks whether it is host trigger mode or periodic trigger mode. In host trigger mode, it waits until a command has been triggered through the share memory interface from ARM core. In periodic trigger mode, the firmware sets host trigger bit based on PRU-ICSS IEP compare event configured. Upon triggering the transmit data is set up and transmitted.

#### Relevant Code Sections in Tamagawa

- Code under `CHECK_OPERATING_MODE`, `HANDLE_PERIODIC_TRIGGER_MODE`, and `HANDLE_HOST_TRIGGER_MODE` in "${SDK_INSTALL_PATH}/source/position_sense/tamagawa/firmware/tamagawa_main.asm"

### Send and Receive

Now the configuration for sending and receiving data over the interface needs to be done.

- Send (TX) : For Tamagawa, the typical size of one transfer is 10 or 30 or 40 bits. `FN_SEND` configures the size of TX in ICSS_CFG_PRUx_ED_CHx_CFG0 register. This code can be modified to change the size to any number. The TX FIFO size is 32 bits. If more than 32 bits need to be sent, then continuous FIFO loading mode has to be used. In Tamagawa, for EEPROM Write Command this mode is used. `FN_SEND` configures the size to 10 bits for normal commands, 30 bits for EEPROM Read command and 0 bit for EEPROM Write command (which means continuous mode). For continuous mode, FIFO level should be polled and data should be continuously pushed based on free space. Data is loaded into FIFO and then TX GO is asserted which starts the TX. The flow for transmit in Tamagawa is explained in \ref TAMAGAWA_DESIGN_TX.

- Receive (RX) : After TX completion, RX mode is enabled in peripheral interface. `RECEIVE_FRAMES_S` and `RECEIVE_FRAMES_M` contain the code for receive. Start bit polarity of RX can be configured in ICSSG_PRUx_ED_RX_CFG_REG. Once this bit is seen on RX pin, the RX FIFO starts filling up. The size of RX needs to configured based on the protocol requirement. The data needs to be fetched from FIFO to ensure that overflow of RX FIFO does not occur. The clock will be stopped based on the clock mode configured before the start of the RX operation. The flow for receive in Tamagawa is explained in \ref TAMAGAWA_DESIGN_TX_RX.

For more details on the programming sequence for TX, RX and clock configuration, please see section "6.4.5.2.2.3.6.4 Three Peripheral Mode Basic Programming Model" of <a href="https://www.ti.com/lit/ug/spruim2h/spruim2h.pdf" target="_blank">AM243x Technical Reference Manual</a>.

#### PRU Instructions Estimation for TX and RX

When doing TX or RX, FIFO over-run and under-run should be avoided to ensure correct operation.

##### TX Timing Considerations

- If the size if less than or equal to 32 bits, then one-shot mode should be used.
    - All data bits can be loaded in one go before starting TX.
- If the size if more than 32 bits, then continuous mode should be used.
    - First load of FIFO has to of size less than our equal to 32 bits. Then once the TX GO is asserted, the bits are sent out on the wire and FIFO starts draining.
    - FIFO Fill Level should be monitored and data should be sent before FIFO Fill Level becomes zero, else it will lead to over-run. Also, if size of data pushed into FIFO is more than than free space, then it will lead to over-run.
    - `EEPROM_WRITE_MACRO` in Tamagawa firmware uses continuous mode for TX. It loads 32 bits (4 bytes) initially and enables TX. Then once the FIFO Fill Level is below 3 bytes, it pushes one more byte.
    - The calculation of time needed for 1 bit to be sent can be done based on the configured clock size. For example, 5 MHz clock is configured. Then 1 bit will take 200 ns time. So if we have pushed 4 bytes, then it will take (32 * 200) = 6400 ns for the FIFO to drain completely. The code for TX needs to ensure that before this under-run, new data is pushed. FIFO Fill Level can be monitored using register R31.

The time available can be converted into PRU cycles based on the PRU Core Clock Frequency. If PRU Core Clock Frequency is 200 MHz, then one PRU cycle will be 5 ns. Non read and write instructions take exactly one PRU clock cycle to execute. For read and write instructions, there are specific rules for how long a read or a write instruction will take. This is explained in an FAQ <a href="https://e2e.ti.com/support/processors-group/processors/f/processors-forum/1096933/faq-pru-how-do-i-calculate-read-and-write-latencies/4065864#4065864
" target="_blank">"PRU: How do I calculate read and write latencies? "</a>

##### RX Timing Considerations

- Typically the RX is done at 4x/8x oversampled rate compared to TX. So the rate of RX will be 40 MHz if TX was at 5 MHz. For each oversampled data byte (Byte if it is 8x oversampling), the firmware needs to wait for valid flag in register R31, then read the data from R31 and clear the valid flag before next data will arrive.
- `RECEIVE_FRAMES_S` in Tamagawa firmware performs the RX operation for single channel mode.
- The calculation of time needed for 1 bit to be sent can be done based on the configured clock size. For example, 5 MHz clock is configured. Then 1 actual bit/8 oversampled bits (for 8x oversampling) will take 200 ns time. The code for RX needs to ensure that after getting valid flag and reading data, we are ready for next valid flag and data within 200 ns to avoid overflow.

#### Relevant Code Sections in Tamagawa

- Code under `FN_SEND_RECEIVE_TAMAGAWA` and `FN_SEND` in "${SDK_INSTALL_PATH}/source/position_sense/tamagawa/firmware/tamagawa_main.asm"
- Code under `TAMAGAWA_SEND_MACRO` in "${SDK_INSTALL_PATH}/source/position_sense/tamagawa/firmware/tamagawa_send.h"
- Code under `RECEIVE_FRAMES_S` in "${SDK_INSTALL_PATH}/source/position_sense/tamagawa/firmware/single_ch_receive_frames.h" for single channel receive
- Code under `RECEIVE_FRAMES_M` in "${SDK_INSTALL_PATH}/source/position_sense/tamagawa/firmware/multi_ch_receive_frames.h" for multi channel receive

### Cyclic Redundancy Check (CRC) Computation

On the data received, CRC needs to be computed. The RX code does on-the-fly CRC computation as it is receiving data bits continuously. The computed CRC can then be compared with the CRC from the received data.

For Tamagawa, the CRC polynomial is (x<sup>8</sup> + 1).

This code for on-the-fly CRC computation can be modified for any other polynomial as well. The PRU instruction cycle budget requirement will vary based on the polynomial. RX Loop timing should not be broken, else it will lead to RX FIFO overflow (if data is not read fast enough). If on-the-fly CRC computation is not viable, then there are two options. One option is to do CRC computation as a part of post-processing after RX is complete. Second is to check if HW CRC16/32 Module from PRU-ICSSG can be used. The CRC16/32 module directly connects with the PRU internal registers R25-R29 through use of the PRU broadside interface and XFR instructions. It supports three different polynomials. For more details, see section "6.4.6.2.2 PRU CRC16/32 Module" of <a href="https://www.ti.com/lit/ug/spruim2h/spruim2h.pdf" target="_blank">AM243x Technical Reference Manual</a>.

#### Relevant Code Sections in Tamagawa

- Code under `RECEIVE_FRAMES_S` in "${SDK_INSTALL_PATH}/source/position_sense/tamagawa/firmware/single_ch_receive_frames.h" for single channel receive
- Code under `RECEIVE_FRAMES_M` in "${SDK_INSTALL_PATH}/source/position_sense/tamagawa/firmware/multi_ch_receive_frames.h" for multi channel receive

## Driver running on ARM core

Driver layer does communication with PRU core(s) via shared memory. New driver APIs can be added or existing APIs can be updated based on the changes need in this communication.


The APIs for Tamagawa are described in \ref TAMAGAWA_API_MODULE.

## Application running on ARM core

Tamagawa application does below configures pinmux, UART, PRU-ICSSG clock, and loads the the PRU firmware. This application is controlled with a terminal interface using a serial over USB connection between the PC host and the EVM, using which the data transfer can be triggered. The application collects the data entered by the user, configures the relevant interface and sends the command. Once the command completion is indicated by the interface, the status of the transaction is checked. If the Status indicates success, the result is presented to the user.

For a new custom UART application, Tamagawa application is a good starting point as most of the configuration like pinmux, PRU-ICSSG initialization, etc. will be same as in Tamagawa. Based on the changes in driver APIs and features implemented in firmware, the API calls can be updated in the application.

## References {#DEVELOPER_GUIDE_CUSTOM_UART_ADDITIONAL_REFERENCES}

Please refer to following documents to understand more about certain topics discussed in this document.

<table>
<tr>
    <th> Document
    <th> Description
</tr>
<tr>
    <td> \ref TAMAGAWA <br/>
         \ref TAMAGAWA_DESIGN
    <td> SDK Documentation for Tamagawa features and design
</tr>
<tr>
    <td rowspan="4"><a href="https://www.ti.com/lit/ug/spruim2h/spruim2h.pdf" target="_blank">AM243x Technical Reference Manual</a>
    <td> Section "6.4.5.2.2.3.6 Three Channel Peripheral Interface"
</tr>
<tr>
    <td> Section "6.4.5.2.2.3.6.3 Clock Generation"
</tr>
<tr>
    <td> Section 6.4.5.2.2.3.6.4 Three Peripheral Mode Basic Programming Model
</tr>
<tr>
    <td> Section "6.4.6.2.2 PRU CRC16/32 Module"
</tr>
<tr>
    <td> <a href="https://e2e.ti.com/support/processors-group/processors/f/processors-forum/1096933/faq-pru-how-do-i-calculate-read-and-write-latencies/4065864#4065864
" target="_blank">PRU: How do I calculate read and write latencies?</a>
    <td> FAQ on read and write latencies for PRU Instructions
</tr>
<tr>
    <td> \ref TAMAGAWA_API_MODULE
    <td> SDK Documentation for Tamagawa Driver APIs
</tr>
</table>


# EnDat3 Protocol Design {#ENDAT3_DESIGN}

[TOC]

## Introduction

This design implements an EnDat3 receiver (subsequent electronics) on TI Sitara™ processors/microcontrollers.
EnDat3 is the latest generation of the digital bidirectional serial interface for position encoders, offering enhanced safety features and higher data rates compared to EnDat 2.2.

Key improvements in EnDat3:
- Frame-based protocol with dedicated High Priority Frames (HPF), Low Priority Header (LPH), and Low Priority Frames (LPF)
- Higher data transfer rates: 12.5 Mbps and 25 Mbps
- Enhanced CRC protection on all frame types
- Support for both foreground and background communication channels
- Improved error detection and status reporting
- Flexible operating modes (host trigger, periodic CMP trigger, and periodic CAP trigger)

Only two differential signal pairs are required: bidirectional data.
Clock is not needed here and data is bidirectional and asynchronous. Transfer between receiver and encoder at the physical layer is in accordance with RS485, with transceivers at both ends.

## System Overview

The position feedback system consists of a position encoder attached to a motor, up to 100 meters of cable which provides power and serial communication, and the receiver interface for the position encoder.

In the case of the Sitara™ processor/microcontroller, the receiver interface for the position encoder is just one function of a connected drive controller.
The AM243x/AM261x/AM263Px provides, in addition to the resources for Industrial Ethernet and motor control applications, including on-chip ADCs, Delta Sigma demodulator for current measurement.

EnDat3 receiver on Sitara™ processor/microcontroller uses one PRU-ICSS slice.
Data transmit and data receive signals from PRU of ICSS are available in Sitara™ processors/microcontrollers.

## Implementation

The EnDat3 receiver function is implemented on TI Sitara™ processors/microcontrollers.

Design is split into three parts:
1. EnDat3 hardware support in PRU using peripheral interface
2. Firmware running in PRU
3. Driver running in Arm®-based core

Application is supposed to use the EnDat3 driver APIs to leverage EnDat3 functionality.

\cond SOC_AM243X
Default SDK example uses PRU-ICSSG0 PRU0 or PRU1 based on SysConfig selection.
\endcond

\cond SOC_AM261X
Default SDK example uses PRU-ICSSM1 PRU0 or PRU1 based on SysConfig selection.
\endcond

\cond SOC_AM263PX
Default SDK example uses PRU-ICSSM PRU0 or PRU1 based on SysConfig selection.
\endcond

### Specifications

<table>
<tr>
    <th>Parameter</th>
    <th>Value</th>
    <th>Details</th>
</tr>
<tr>
    <td>Maximum Cable Length</td>
    <td>100m</td>
    <td>Supported at 12.5 Mbps with proper cable quality</td>
</tr>
<tr>
    <td>Data Transfer Rates</td>
    <td>12.5 Mbps, 25 Mbps</td>
    <td>Configurable via RATE command</td>
</tr>
<tr>
    <td>Initialization Rate</td>
    <td>12.5 Mbps</td>
    <td>Default rate after HELLO command</td>
</tr>
<tr>
    <td>Frame Types</td>
    <td>HPF, LPH, LPF</td>
    <td>Three frame types with individual CRC protection</td>
</tr>
<tr>
    <td>CRC Algorithm</td>
    <td>Custom</td>
    <td>Polynomial: 0x1A7</td>
</tr>
<tr>
    <td>Encoding</td>
    <td>Manchester</td>
    <td>Self-clocking encoding with lookup table decoding</td>
</tr>
<tr>
    <td>Oversample Ratio</td>
    <td>8x</td>
    <td>8x oversampling for robust data reception</td>
</tr>
</table>

### EnDat3 PRU Hardware

The PRU-ICSS provides hardware support for EnDat3 communication through its peripheral interface.

Refer to the device TRM for details on PRU-ICSS peripheral interface configuration.

### EnDat3 Firmware Implementation

The following section describes the firmware implementation of EnDat3 receiver on PRU-ICSS. The deterministic behavior of the 32-bit RISC core provides precise control for sampling external signals and generating timing-critical outputs. It makes use of 3 channel peripheral interface support in PRU for data transmission/reception.

#### Firmware Architecture

The PRU-ICSS firmware supports the following configuration:
1. Single Channel per PRU slice

The firmware first initializes the local variables and configures the PRU-ICSS peripheral interface. Then it checks the operating mode: host trigger mode, periodic CMP mode, or periodic CAP mode.

The firmware operates in a state machine architecture with the following main phases:

1. **Initialization Phase**
   - Configure PRU-ICSS peripheral interface
   - Initialize Manchester encoding/decoding lookup tables
   - Set initial communication parameters
   - Configure operating mode (host trigger, periodic CMP, or periodic CAP)

2. **Command Phase**
   - Wait for command trigger (from R5F application, IEP compare event, or IEP capture event)
   - Parse command type (foreground/background)
   - Set up transmission parameters
   - Configure expected frame counts

3. **Transmission Phase**
   - Generate clock signal
   - Transmit command data with Manchester encoding
   - Handle preamble and postamble sequences
   - Send background data if applicable

4. **Reception Phase**
   - Receive encoder response with oversampling
   - Decode Manchester-encoded data
   - Validate frame structure (preamble detection)
   - Store received data in frame buffers

5. **Processing Phase**
   - Verify CRC for all received frames
   - Extract position data from HPF
   - Parse LPH for LPF frame information
   - Process background response data from LPF
   - Update status flags

##### Initialization

Before executing the firmware, the Arm-based core needs to:
1. Enable peripheral interface in PRU-ICSS
2. Configure initial clock frequency (12.5 Mbps)
3. Set operating mode (host trigger mode by default)
4. Clear any previous trigger states

The firmware initialization sequence:
1. Initialize Manchester encoding/decoding lookup table
2. Configure PRU peripheral interface registers
3. Set up frame structure parameters
4. Initialize communication state machine
5. Configure delay cycles for timing-critical operations
6. Wait for first command trigger

After initialization, the encoder must be initialized with the HELLO command:
- Switches encoder from EnDat 2.2 mode to EnDat3 mode
- May take up to 300ms to complete
- Application should verify HPF data valid flag after HELLO

##### Manchester Encoding and Decoding

EnDat3 uses Manchester encoding for robust data transmission:

**Encoding (TX):**
- Logical '0' = Low-to-High transition in bit period
- Logical '1' = High-to-Low transition in bit period
- Self-clocking property aids receiver synchronization

**Decoding (RX):**
- Implemented using pre-computed lookup table (LUT)
- LUT maps 8-bit oversampled pattern to decoded bit
- Handles edge detection and sampling point determination
- 64-entry LUT (256 bytes) stored in PRU memory

The LUT is generated based on:
```
For each 8-bit oversampled pattern:
  - Detect transition location
  - Determine bit value from transition direction
  - Account for oversampling phase alignment
```

##### Send and Receive

###### Command Transmission

Command transmission sequence:
1. Set up foreground and background operation codes
2. Configure background data (if applicable)
3. Generate preamble (minimum 25 halfbits)
4. Transmit start bit with configured polarity
5. Transmit command with Manchester encoding
6. Transmit background data (if requested)
7. Generate postamble (4 halfbits)
8. Transition to receive mode

###### Frame Reception

Reception sequence:
1. **Preamble Detection**
   - Wait for preamble pattern from encoder
   - Synchronize to encoder clock
   - Validate minimum preamble length

2. **HPF Reception**
   - Receive 48-bit position data
   - Receive 8-bit HPF status
   - Receive 8-bit HPF CRC
   - Decode Manchester encoding using LUT
   - Verify CRC-8 checksum

3. **LPH Reception** (if background communication active)
   - Receive 8-bit LPH status
   - Receive 8-bit num_lpf field
   - Receive 16-bit reserved field
   - Receive 8-bit LPH CRC
   - Verify CRC-8 checksum

4. **LPF Reception** (num_lpf times)
   - Receive 8-bit LPF status with FID
   - Receive 48-bit LPF data
   - Receive 8-bit LPF CRC
   - Verify CRC-8 checksum

###### Preamble and Postamble

**Preamble:**
- Minimum 25 halfbits of alternating pattern
- Allows receiver clock synchronization
- Transmitted before each command

**Postamble:**
- 4 halfbits to complete Manchester encoding
- Ensures clean signal transition
- Required by protocol specification

##### Operating Modes

The firmware supports three operating modes: host trigger mode, periodic CMP mode, and periodic CAP mode.

Upon triggering (from any mode), the transmit data is set up based on the command code and the data is transmitted. The firmware then waits until receiving all frames from the encoder. CRC verification is performed on all received frames and the interface is updated with the result. The firmware then waits for the next command trigger from the interface or IEP compare/capture event.

**Host Trigger Mode:** The firmware waits until a command has been triggered through the interface by the host application.

Following is the operation flow for host trigger mode:
1. Application sets up command parameters
2. Application triggers command via start trigger flag
3. Firmware executes complete command/response cycle
4. Firmware clears trigger flag upon completion
5. Application reads response data

**Periodic CMP Mode (Compare Event Mode):** In CMP mode, IEP timer compare event triggers position sampling. The firmware monitors the configured IEP compare event and automatically initiates EnDat3 transactions when the IEP timer counter matches the compare value. This enables fixed-rate periodic sampling.

**Periodic CAP Mode (Capture Event Mode):** In CAP mode, external signals trigger position sampling through IEP capture events. The capture event is triggered on the rising edge of the external input pulse, enabling event-driven position capture. \if (SOC_AM243X || SOC_AM64X) Internal signals can also be mapped to IEP capture events via TIMESYNC/GPIOMUX router. \else Internal signals can also be mapped to IEP capture events via XBAR. \endif

Following is the operation flow for periodic mode:
1. Firmware polls IEP CMP/CAP status register and clears status after event is detected
2. On event detection, firmware initiates EnDat3 transaction
3. Position data is automatically updated in shared memory
4. R5F interrupt notifies application of new data
5. The firmware checks the current trigger mode. If still in periodic mode, it returns to step 1 to wait for the next IEP CMP/CAP event. If the mode has been switched to host trigger mode, the firmware stops periodic operation.

\attention Input cycle time (CMP mode) or external trigger period (CAP mode) should be greater than or equal to the EnDat3 communication cycle time.

##### Background Communication State Machine

Background communication uses a multi-cycle protocol:

**LPH Status States:**
- IDLE (0): No background communication active
- RX_START (1): Background request initiated
- RX_LAST (2): Last frame of background response
- BUSY (3): Background operation in progress

**Command Cycle Requirements:**

Depending on LPH status, different numbers of command cycles are required:

| LPH Status | State | Cycles Required | Behavior |
|------------|-------|-----------------|----------|
| 0 | IDLE | 4 | Send request in 4 cycles, response in 4th cycle |
| 1 | RX_START | 6 | Send request for 2 NOP cycles, then send request acc. to IDLE state|
| 2 | RX_LAST | 5 | Send request for 1 NOP cycle, then send request acc. to IDLE state |
| 3 | BUSY | N/A | Wait, cannot send request |

##### Delay Cycle Configuration

The firmware uses precise delay cycles for timing-critical operations:

| Delay Parameter | Purpose | Typical Value |
|----------------|---------|---------------|
| delay_tx_start_1 | Delay before TX preamble | Calculated based on PRU freq |
| delay_tx_start_2 | Delay before command transmission | Calculated based on PRU freq |
| delay_rx_start | Delay before RX enable | Calculated based on PRU freq |
| delay_rx_to_tx | Turnaround time RX to TX | Calculated based on PRU freq |
| delay_timeout | Timeout for encoder response | Calculated based on PRU freq |

All delay values are computed by R5F driver based on actual PRU frequency to ensure timing accuracy across different clock configurations.

### EnDat3 Hardware Interface

The physical data transmission in EnDat3 is done using RS-485 standard. The data is transmitted as differential signals using RS485 between the EnDat3 receiver and the encoder. The design uses differential signal pairs for bidirectional data.

EnDat3 receiver and encoder are connected using RS-485 transceivers. Data is transmitted differentially over RS-485 with advantages of high noise immunity and long-distance transmission capabilities.

#### Pin Multiplexing {#ENDAT3_PIN_USAGE}

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
    <td>pru<n>_endat3_0_clk
    <td>Channel 0 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO1 \else PRG<%k>_PRU<n>_GPO1 \endif
    <td>pru<n>_endat3_0_out
    <td>Channel 0 transmit
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO2 \else PRG<%k>_PRU<n>_GPO2 \endif
    <td>pru<n>_endat3_0_out_en
    <td>Channel 0 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI9 \else PRG<%k>_PRU<n>_GPI13/PRG<%k>_PRU<n>_GPI9 \endif
    <td>pru<n>_endat3_0_in
    <td>Channel 0 receive
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO3 \else PRG<%k>_PRU<n>_GPO3 \endif
    <td>pru<n>_endat3_1_clk
    <td>Channel 1 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO4 \else PRG<%k>_PRU<n>_GPO4 \endif
    <td>pru<n>_endat3_1_out
    <td>Channel 1 transmit
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO5 \else PRG<%k>_PRU<n>_GPO5 \endif
    <td>pru<n>_endat3_1_out_en
    <td>Channel 1 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI10 \else PRG<%k>_PRU<n>_GPI14/PRG<%k>_PRU<n>_GPI10 \endif
    <td>pru<n>_endat3_1_in
    <td>Channel 1 receive
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO6 \else PRG<%k>_PRU<n>_GPO6 \endif
    <td>pru<n>_endat3_2_clk
    <td>Channel 2 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO7 \else PRG<%k>_PRU<n>_GPO12/PRG<%k>_PRU<n>_GPO7 \endif
    <td>pru<n>_endat3_2_out
    <td>Channel 2 transmit
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO8 \else PRG<%k>_PRU<n>_GPO8 \endif
    <td>pru<n>_endat3_2_out_en
    <td>Channel 2 transmit enable
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI11 \else PRG<%k>_PRU<n>_GPI11 \endif
    <td>pru<n>_endat3_2_in
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
    <td>pru1_endat3_0_clk
    <td>PRU1 Channel 0 clock
</tr>
<tr>
    <td>PRG0_PRU1_GPO1
    <td>pru1_endat3_0_out
    <td>PRU1 Channel 0 transmit
</tr>
<tr>
    <td>PRG0_PRU1_GPO2
    <td>pru1_endat3_0_out_en
    <td>PRU1 Channel 0 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI13
    <td>pru1_endat3_0_in
    <td>PRU1 Channel 0 receive when SA mux selection is enabled (ICSSG_SA_MX_REG[7] G_MUX_EN = 1)
</tr>
<tr>
    <td>GPIO Pin (GPIO1_78/C16)
    <td>ENC0_EN
    <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
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
    <td>pru0_endat3_0_clk
    <td>PRU0 Channel 0 clock
</tr>
<tr>
    <td>PR1_PRU0_GPIO1
    <td>pru0_endat3_0_out
    <td>PRU0 Channel 0 transmit
</tr>
<tr>
    <td>PR1_PRU0_GPIO2
    <td>pru0_endat3_0_out_en
    <td>PRU0 Channel 0 transmit enable
</tr>
<tr>
    <td>PR1_PRU0_GPI9
    <td>pru0_endat3_0_in
    <td>PRU0 Channel 0 receive
</tr>
<tr>
    <td>GPIO Pin (GPIO_21/B10)
    <td>ENC0_EN (PRU0)
    <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
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
    <td>pru0_endat3_1_clk
    <td>PRU0 Channel 1 clock
</tr>
<tr>
    <td>PR0_PRU0_GPO4
    <td>pru0_endat3_1_out
    <td>PRU0 Channel 1 transmit
</tr>
<tr>
    <td>PR0_PRU0_GPO5
    <td>pru0_endat3_1_out_en
    <td>PRU0 Channel 1 transmit enable
</tr>
<tr>
    <td>PR0_PRU0_GPI10
    <td>pru0_endat3_1_in
    <td>PRU0 Channel 1 receive
</tr>
<tr>
    <td>GPIO Pin (SDFM0_D1/D13)
    <td>ENC1_EN
    <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
</table>
\endcond

## Software Driver Architecture

### Driver Layers

The EnDat3 software is organized into three layers:

1. **Application Layer**
   - User application code
   - Uses EnDat3 driver APIs
   - Handles encoder data and control logic

2. **Driver Layer** (endat3_drv.c)
   - API implementation
   - Frame parsing and CRC verification
   - Error handling and status reporting
   - Configuration management

3. **Firmware Layer** (endat3_main.asm)
   - PRU assembly code
   - Protocol timing implementation
   - Manchester encoding/decoding
   - Hardware interface control

### Key Driver APIs

The driver provides comprehensive APIs for EnDat3 communication:

**Initialization APIs:**
- `endat3_init()` - Initialize EnDat3 interface
- `endat3_set_operating_mode()` - Configure operating mode (host trigger, periodic CMP, or periodic CAP)
- `endat3_release_start_trigger()` - Enable communication
- `endat3_clear_start_trigger()` - Disable communication

**Command APIs:**
- `endat3_set_foreground_op_code()` - Set foreground command
- `endat3_set_background_op_code()` - Set background command
- `endat3_set_expected_tx_frame_count()` - Configure frame count
- `endat3_set_bg_data()` - Set background data
- `endat3_send_command()` - Transmit command
- `endat3_receive_response()` - Receive and parse response

**Data Access APIs:**
- `endat3_get_hpf_data()` - Get position data from HPF
- `endat3_get_hpf_status()` - Get HPF status
- `endat3_get_hpf_crc()` - Get HPF CRC
- `endat3_get_lph_status()` - Get LPH status
- `endat3_get_lph_state()` - Get background communication state
- `endat3_get_lpf_status()` - Get LPF status for frame N
- `endat3_get_lpf_data()` - Get LPF data for frame N
- `endat3_get_lpf_crc()` - Get LPF CRC for frame N

**Status Check APIs:**
- `endat3_has_hpf_error()` - Check HPF error flag
- `endat3_has_hpf_warning()` - Check HPF warning flag
- `endat3_is_hpf_data_valid()` - Check HPF data validity
- `endat3_has_absolute_value()` - Check absolute value availability
- `endat3_is_busy()` - Check if transfer in progress
- `endat3_get_error_code()` - Get detailed error code

**Periodic Trigger APIs:**
- `endat3_config_iep_cmp_event()` - Configure IEP CMP event number for periodic CMP mode
- `endat3_config_iep_cap_event()` - Configure IEP CAP event number for periodic CAP mode
- `endat3_config_periodic_mode()` - Configure and start periodic trigger mode (example code)
- `endat3_stop_periodic_mode()` - Stop periodic trigger mode (example code)

### Error Handling

The driver provides comprehensive error detection and reporting:

**Driver Error Codes:**
- `ENDAT3_ERR_INVALID_HANDLE` - Invalid handle or interface
- `ENDAT3_ERR_CHANNEL_CONFIG` - Channel configuration failed
- `ENDAT3_ERR_DELAY_CONFIG` - Delay configuration failed
- `ENDAT3_ERR_INVALID_CORE` - Invalid PRU core specified
- `ENDAT3_ERR_CLOCK_CONFIG` - Clock configuration failed
- `ENDAT3_ERR_SAMPLING_ERROR` - Sampling error detected
- `ENDAT3_ERR_LPF_CRC_FAIL` - LPF CRC verification failed
- `ENDAT3_ERR_HPF_CRC_FAIL` - HPF CRC verification failed
- `ENDAT3_ERR_LPH_CRC_FAIL` - LPH CRC verification failed
- `ENDAT3_ERR_TIMEOUT` - Communication timeout
- `ENDAT3_ERR_RX_FAIL` - Reception failed
- `ENDAT3_ERR_INVALID_PARAM` - Invalid parameter

**Protocol Error Codes:**
Extracted from encoder response, with descriptive strings and recommended actions.

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

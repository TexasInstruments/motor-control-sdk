# PRU-ICSSM Input/Output Modes {#DEVELOPER_GUIDE_PRUICSSM_IO_MODES}

[TOC]

## PRU-ICSSM overview
One PRU-ICSSM has 2 programmable real-time unit (PRU) cores.

\image html pruicssm_block_diagram.png "PRU-ICSSM Block Diagram"

\cond SOC_AM261X
For more details on PRU-ICSSM, please see section "7.3 Programmable Real-Time Unit Subsystem (PRU-ICSS)" of <a href="https://www.ti.com/lit/ug/sprujb6b/sprujb6b.pdf" target="_blank">AM261x Technical Reference Manual</a>.
\endcond
\cond SOC_AM263PX
For more details on PRU-ICSSM, please see section "7.3 Programmable Real-Time Unit Subsystem (PRU-ICSS)" of <a href="https://www.ti.com/lit/ug/spruj55d/spruj55d.pdf" target="_blank">AM263Px Technical Reference Manual</a>.
\endcond

## Input/Output modes {#PRUICSSM_IO_MODES}

Each PRU has 20 PRU GPIO pins, which are multiplexed with other functional signals at the device level. There is also an internal wrapper multiplexing that expands the device top-level multiplexing. This wrapper multiplexing is controlled by the GPCFGx_REG register (where x = 0 or 1 for PRU) in the PRU-ICSS CFG register space and allows direct GPIO, MII_RT, 3-channel Peripheral Interface, and Sigma Delta functionality to be muxed with the PRU GPIO device signals, as shown in the figure below. Note that the device top-level muxing has higher priority over the internal muxing.

\cond SOC_AM261X
For more details, please see section "7.3.2.1 PRU-ICSS Internal Pinmux" of <a href="https://www.ti.com/lit/ug/sprujb6b/sprujb6b.pdf" target="_blank">AM261x Technical Reference Manual</a>.
\endcond
\cond SOC_AM263PX
For more details, please see section "7.3.2.1 PRU-ICSS Internal Pinmux" of <a href="https://www.ti.com/lit/ug/spruj55d/spruj55d.pdf" target="_blank">AM263Px Technical Reference Manual</a>.

\endcond

\image html pruicssm_pin_modes.png "PRU-ICSSM Pin Modes"

The following are different input/output modes based on different values of the GPCFGx_REG register:

1. GPIO mode (0)
    - PRU EGPIs Direct Input
    - PRU EGPIs 16-Bit Parallel Capture
    - PRU EGPIs 28-Bit Shift In
    - PRU EGPOs Direct Output
    - PRU EGPO Shift Out
2. Peripheral IF mode (1)
    - Three-Channel Peripheral Interface used for absolute encoder interfaces like BiSS-C, EnDat, Hiperface DSL, Nikon A-format, Tamagawa, custom UART, etc.
3. MII mode (2)
    - MII/RGMII mode used for industrial communication protocol and standard ethernet
4. SD mode (3)
    - Sigma Delta (SD) Decimation Filtering used for current sense interface


\attention **Pin mode selection is done per PRU. If Peripheral IF mode is selected for a PRU, all pins from that PRU are selected for Peripheral IF mode and cannot be used for other modes like GPIO or SD or MII.**

\note
    - Some devices may not pin out all 29 bits of R31 and all 32 bits of R30. For which pins are available on a specific device, see the device-specific datasheet for device pin mapping.

\cond SOC_AM261X
For more details, please see section "7.3.5.2.2.3 General-Purpose Inputs (R31): Enhanced PRU GP Module" of <a href="https://www.ti.com/lit/ug/sprujb6b/sprujb6b.pdf" target="_blank">AM261x Technical Reference Manual</a>.
\endcond
\cond SOC_AM263PX
For more details, please see section "7.3.5.2.2.3 General-Purpose Inputs (R31): Enhanced PRU GP Module" of <a href="https://www.ti.com/lit/ug/spruj55d/spruj55d.pdf" target="_blank">AM263Px Technical Reference Manual</a>.
\endcond

The following sections contain a brief overview of different modes.

### GPIO mode

\note
    - Some devices may not pin out all 29 bits of R31 and all 32 bits of R30. For which pins are available on a specific device, see the device-specific datasheet for device pin mapping.

#### PRU EGPIs Direct Input

\image html gpi_direct_input_mode.png "PRU R31 (EGPI) Direct Input Mode"

- PRU GPI[19:0] pin data feeds directly into the PRU R31[19:0] bits.


#### PRU EGPIs 16-Bit Parallel Capture

\image html pruicssm_16b_parallel_capture.png "PRU R31 (EGPI) 16-Bit Parallel Capture Mode"

- DATAIN[0:15] is captured by the positive edge or negative edge of CLOCKIN.
- CLOCKIN edge is selected by the PRU_ICSSM_GPCFGx register.


#### PRU EGPIs 28-Bit Shift In

\image html pruicssm_28b_shift_mode.png "PRU R31 (EGPI) 28-Bit Shift Mode"

- DATAIN is sampled and shifted into a 28-bit shift register.
- Shift Counter (Cnt_16) feature is mapped to pru<n>_r31_status[28]. Cnt_16 is self-clearing and is connected to the PRU Interrupt Controller (INTC).
- SB (Start Bit detection) feature is mapped to pru<n>_r31_status[29]. Start Bit (SB) is cleared by the PRU_ICSSM_GPECFGx register. Start Bit value (0h or 1h) is selected by the PRU_ICSSM_GPECFGx register.

#### PRU EGPOs Direct Output

\image html gpi_direct_output_mode.png "PRU R30 (EGPO) Direct Output Mode"

- PRU R30[19:0] bits feed directly to GPO[19:0].

#### PRU EGPO Shift Out

\image html pruicssm_shift_out_mode.png "PRU R30 (GPO) Shift Out Mode"

- pru<n>_r30[0] is shifted out on DATAOUT on every rising edge of pru<n>_r30[1] (CLOCKOUT).
- Free Running Clock or Fixed Clock Count Mode is selected by the PRU_ICSSM_GPECFGx register.
- LOAD_GPO_SH0 (Load Shadow Register 0) is mapped to pru<n>_r30[29]. LOAD_GPO_SH1 (Load Shadow Register 1) is mapped to pru<n>_r30[30].
- ENABLE_SHIFT is mapped to pru<n>_r30[31].

### Peripheral IF mode

\image html pruicssm_peripheral_if_mode.png "Peripheral IF Mode"

- The Three-Channel Peripheral Interface is used for absolute encoder interfaces, custom UART, etc.
    - Implementation of EnDat 2.1/2.2, BiSS-C, Hiperface DSL, Tamagawa (T-format), Nikon A-format 2.1/3.0 is available in Motor Control SDK.
    - Can also implement custom UART.
- 3 channels per PRU with baud range from 100 kHz to 32 MHz (based on SoC PLL) per PRU
- ICSS_UART_CLK (default) or ICSS_CORE_CLK clock is an input to independent clock dividers to produce a normal clock and an oversampling clock
- Half-duplex (TX and RX are not supported concurrently)
- TX/RX FIFO size of 32 bits
- Configurable oversampling on RX
- Optional RX frame size auto start/shut off
- Individual TX channel start trigger or simultaneous TX start trigger for all channels
- Optional SW direct snoop of data input
- RX Start Bit of '1' or '0'
- Flexible HW-assisted clock output generation to allow free running, stop high and stop low (after last RX data), or stop high (after last TX data) operation with optional software clock override feature

#### Signal Configuration

<table>
<tr>
    <th> Pad Names at Device Level
    <th> Peripheral IF Mode
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI9
    <td> PERIF0_IN
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI10
    <td> PERIF1_IN
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI11
    <td> PERIF2_IN
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPO0
    <td> PERIF0_CLK
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPO1
    <td> PERIF0_OUT
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPO2
    <td> PERIF0_OUT_EN
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPO3
    <td> PERIF1_CLK
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPO4
    <td> PERIF1_OUT
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPO5
    <td> PERIF1_OUT_EN
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPO6
    <td> PERIF2_CLK
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPO7
    <td> PERIF2_OUT
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPO8
    <td> PERIF2_OUT_EN
</tr>
</table>

\note
    - These signals are shared with the GPIO, MII, and SD modes. To configure for Peripheral IF, PRU_ICSS_GPCFGx[29-26] PR1_PRUy_GP_MUX_SEL needs to be set to 1h.
    - Some devices may not pin out all 29 bits of R31 and all 32 bits of R30. For which pins are available on a specific device, see the device-specific datasheet for device pin mapping.

### SD mode

\image html pruicssm_sd_mode.png "Sigma Delta Mode"

- Sigma Delta (SD) Decimation Filtering mode is used for current sense interface.
- Sigma Delta Filter Module (SDFM) is achieved by a combination of PRU hardware and firmware.
- Hardware integrators perform the accumulation part of Sinc filtering and the differentiation part is done in firmware (Sinc2/Sinc3)
- Up to 9 channels of concurrent counting per PRU
- Software can read all 3 stages of accumulators
- Clock source option of per channel or one per 3 channels or one per 9 channels
- Fast detection for overcurrent detection (via sliding window size max: 28 clocks)
- sinc1 snoop for overcurrent detection
- Manchester decode mode per channel removes the need for CLK
- Synchronized conversion is possible using IEP comparator registers

The following images show the block diagram of the Sigma Delta hardware integrators and integration with the PRU R30 / R31 interface for a single channel. The three accumulators (acc1-acc3) for each channel are simple 28-bit adders. The input for acc1 is 1-bit, while the inputs for acc2 and acc3 are 28-bits. On each positive edge of CLK_OUT, all three 28-bit counters (acc1-acc3) increment.

\image html pruicssm_sd_mode_snoop_0.png "Sigma Delta Hardware Integrators Block Diagram (snoop = 0)"
\image html pruicssm_sd_mode_snoop_1.png "Sigma Delta Hardware Integrators Block Diagram (snoop = 1)"

#### Signal Configuration

<table>
<tr>
    <th> Pad Names at Device Level
    <th> Sigma Delta (SD) Mode
    <th> Function
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI0
    <td> SD0_CLK
    <td> SD demodulator clock channel 0
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI1
    <td> SD0_D
    <td> SD demodulator data channel 0
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI2
    <td> SD1_CLK
    <td> SD demodulator clock channel 1
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI3
    <td> SD1_D
    <td> SD demodulator data channel 1
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI4
    <td> SD2_CLK
    <td> SD demodulator clock channel 2
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI5
    <td> SD2_D
    <td> SD demodulator data channel 2
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI6
    <td> SD3_CLK
    <td> SD demodulator clock channel 3
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI7
    <td> SD3_D
    <td> SD demodulator data channel 3
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI8
    <td> SD4_CLK
    <td> SD demodulator clock channel 4
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI9
    <td> SD4_D
    <td> SD demodulator data channel 4
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI10
    <td> SD5_CLK
    <td> SD demodulator clock channel 5
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI11
    <td> SD5_D
    <td> SD demodulator data channel 5
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI12
    <td> SD6_CLK
    <td> SD demodulator clock channel 6
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI13
    <td> SD6_D
    <td> SD demodulator data channel 6
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI14
    <td> SD7_CLK
    <td> SD demodulator clock channel 7
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI15
    <td> SD7_D
    <td> SD demodulator data channel 7
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI16
    <td> SD8_CLK
    <td> SD demodulator clock channel 8
</tr>
<tr>
    <td> PR<k>_PRU<n>_GPI17
    <td> SD8_D
    <td> SD demodulator data channel 8
</tr>
</table>


\cond SOC_AM261X
\note
    - These signals are shared with the GPIO, MII, and Peripheral IF modes. To configure for SD, PRU_ICSS_GPCFGx[29-26] PR1_PRUy_GP_MUX_SEL needs to be set to 3h.
    - Some devices may not pin out all 29 bits of R31 and all 32 bits of R30. For which pins are available on a specific device, see the device-specific datasheet for device pin mapping.
    - The PR<k>_PRU0_GPI1 signal (muxed with SD0_D) can be used as SD_CLKOUT when PRU-ICSS generates the clock. This is a trade-off as the PRU application will lose one SD channel. SD_CLKOUT needs to go through a clock generator chip if driving multiple sigma delta modulators and also be looped back into PRU-ICSS as SD_CLKIN, typically pru_gpi16. For more details, please see section "7.3.5.2.2.3.5.1 Sigma Delta Block Diagram and Signals" of <a href="https://www.ti.com/lit/ug/sprujb6b/sprujb6b.pdf" target="_blank">AM261x Technical Reference Manual</a>.
\endcond
\cond SOC_AM263PX
\note
    - These signals are shared with the GPIO, MII, and Peripheral IF modes. To configure for SD, PRU_ICSS_GPCFGx[29-26] PR1_PRUy_GP_MUX_SEL needs to be set to 3h.
    - Some devices may not pin out all 29 bits of R31 and all 32 bits of R30. For which pins are available on a specific device, see the device-specific datasheet for device pin mapping.
    - The PR<k>_PRU0_GPI1 signal (muxed with SD0_D) can be used as SD_CLKOUT when PRU-ICSS generates the clock. This is a trade-off as the PRU application will lose one SD channel. SD_CLKOUT needs to go through a clock generator chip if driving multiple sigma delta modulators and also be looped back into PRU-ICSS as SD_CLKIN, typically pru_gpi16. For more details, please see section "7.3.5.2.2.3.5.1 Sigma Delta Block Diagram and Signals" of <a href="https://www.ti.com/lit/ug/spruj55d/spruj55d.pdf" target="_blank">AM263Px Technical Reference Manual</a>.
\endcond
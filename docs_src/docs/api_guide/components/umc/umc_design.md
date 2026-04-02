# TIDEP-01032 EtherCAT-Connected Single-Chip Dual-Servo Motor Drive Reference Design {#REFERENCE_DESIGN}

[TOC]

This reference design is based on <a href="https://www.ti.com/tool/TIDEP-01032" target="_blank">UMC framework</a> for TI's AM243x Arm® based MCUs.

The design demonstrates the use of AM243x MCUs for implementing Field-Oriented Control (FOC) motor control techniques. It also provides instructions on how to change %SDFM settings, EPWM settings, and enabling support for different encoder.

The system setup for the TIDEP-01032 system is shown in the image below:

\imageStyle{umc_system_set_up_for_tidep_01032.png,width:60%}
\image html umc_system_set_up_for_tidep_01032.png "TIDEP-01032 System Setup"

## Resources

<table>
<tr>
    <th>Resource</th>
    <th>Description</th>
</tr>
<tr>
    <td><a href="https://www.ti.com/tool/TIDEP-01032" target="_blank">TIDEP-01032 Design</a></td>
    <td>Design Folder</td>
</tr>
<tr>
    <td><a href="https://www.ti.com/tool/LP-AM243" target="_blank">AM243x LaunchPad™</a></td>
    <td>Tool Folder</td>
</tr>
<tr>
    <td><a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank">BP-AM2BLDCSERVO BoosterPack</a></td>
    <td>Tool Folder</td>
</tr>
<tr>
    <td><a href="https://dev.ti.com/tirex/explore/node?node=A__AAjt1vUw09B6n8RKzVMpuA__AM24X-ACADEMY__ZPSnq-h__LATEST" target="_blank">AM243x Academy</a></td>
    <td>Academy Training Materials</td>
</tr>
<tr>
    <td><a href="https://e2e.ti.com/support/applications/ti_designs/" target="_blank">TI E2E Support</a></td>
    <td>Support Forums</td>
</tr>
</table>

## Key Features

- EtherCAT CiA402 device profile support for motor velocity control
- Single-chip, dual-servo motor control
- TI BoosterPack™ Plug-in Module design
    - 80 digital and analog I/O compatible with AM2x LaunchPad™ Development Kits
- Two axes of 3-phase BLDC motor drive with DRV8316R
    - 24V, 8A monolithic gate drive and amplifier bridges
- Dual-axis current feedback
    - 6 channels of 3-phase current feedback via AMC1035D Sigma-Delta modulator
    - INA241A current sense path
- Dual-axis RS-485 based absolute encoder feedback
    - Supports multiple industrial encoder standards

## Terminology

| Term/Abbreviation | Full Name/Description |
|------------------|----------------------|
| EtherCAT | Ethernet for Control Automation Technology |
| CiA402 | An EtherCAT Profile for Drives and Motion Control |
| CMP | Event Comparator |
| EnDAT 2.2 | A digital, bidirectional interface standard for incremental and absolute position encoders |
| EPWM | Enhanced Pulse-Width Modulation |
| FOC | Field-Oriented Control |
| ICSS | Industrial Communication Subsystem |
| IEP | Industrial Ethernet Peripheral |
| IPC | Inter-Processor Communication |
| ISR | Interrupt Service Routine |
| LP | LaunchPad™ |
| PLC | Programmable Logic Controller |
| PRU | Programmable Real-time Unit |
| PWM | Pulse-Width Modulation |
| RPM | Revolutions Per Minute |
| %SDFM | Sigma-Delta Filter Module |

## System Specifications

1. EtherCAT CiA402 device profile support
2. Single-chip dual-servo motor control
3. 50-kHz FOC loop for current and velocity control
4. Dual-axis (6 channels) 3-phase sigma-delta modulated current feedback
5. Dual-axis absolute encoder position feedback

## System Overview

### Block Diagram

\imageStyle{umc_tidep_01032_block_diagram.png,width:80%}
\image html umc_tidep_01032_block_diagram.png "TIDEP-01032 Block Diagram"

### Key Components

#### Encoder Interface

Encoder signal parameters for the AM243x LaunchPad™ and Booster Pack:

<table>
<tr>
    <th>Encoder</th>
    <th>AM243x LP (PIN NUMBER)</th>
    <th>BP CONNECTORS</th>
    <th>BLDC BP SIGNAL NAME</th>
    <th>Description</th>
</tr>
<tr>
    <td>Encoder 1</td>
    <td>GPIO1_78(C16)</td>
    <td>J8.73</td>
    <td>VSENSOR1_SW_EN</td>
    <td>Encoder 1 enable</td>
</tr>
<tr>
    <td>Encoder 1</td>
    <td>PRG0_PRU1_GPO0(L5)</td>
    <td>J2.11</td>
    <td>ENCODER_CLK1</td>
    <td>Encoder 1 clock</td>
</tr>
<tr>
    <td>Encoder 1</td>
    <td>PRG0_PRU1_GPO2(M2)</td>
    <td>J7.68</td>
    <td>ENCODER_DATA_TX_EN1</td>
    <td>Encoder 1 TX enable</td>
</tr>
<tr>
    <td>Encoder 1</td>
    <td>PRG0_PRU1_GPO1(J2)</td>
    <td>J7.67</td>
    <td>ENCODER_DATA_TX1</td>
    <td>Encoder 1 TX</td>
</tr>
<tr>
    <td>Encoder 1</td>
    <td>PRG0_PRU1_GPO13(T4)</td>
    <td>J8.71</td>
    <td>ENCODER_DATA_RX1</td>
    <td>Encoder 1 RX</td>
</tr>
<tr>
    <td>Encoder 2</td>
    <td>GPIO1_77(B17)</td>
    <td>J8.74</td>
    <td>VSENSOR2_SW_EN</td>
    <td>Encoder 2 enable</td>
</tr>
<tr>
    <td>Encoder 2</td>
    <td>PRG0_PRU1_GPO6(F5)</td>
    <td>J7.69</td>
    <td>ENCODER_CLK2</td>
    <td>Encoder 2 clock</td>
</tr>
<tr>
    <td>Encoder 2</td>
    <td>PRG0_PRU1_GPO8(F4)</td>
    <td>J6.57</td>
    <td>ENCODER_DATA_TX_EN2</td>
    <td>Encoder 2 TX enable</td>
</tr>
<tr>
    <td>Encoder 2</td>
    <td>PRG0_PRU1_GPO12(P2)</td>
    <td>J8.72</td>
    <td>ENCODER_DATA_TX2</td>
    <td>Encoder 2 TX</td>
</tr>
<tr>
    <td>Encoder 2</td>
    <td>PRG0_PRU1_GPO11(P1)</td>
    <td>J7.70</td>
    <td>ENCODER_DATA_RX2</td>
    <td>Encoder 2 RX</td>
</tr>
</table>

The `HAL_setupEncoder` function is used to initialize the encoder interface. The `HAL_getMtrEncoderPosition` function is used to read the position frame from the encoder. The position frame is typically stored in memory by the PRU core, and the `HAL_getMtrEncoderPosition` function converts this value into an electrical angle, which is then used in the control loop during the interrupt service routine.

##### How to modify encoder interface

To enable support for another encoder, first add the encoder module in SysConfig. The encoder initialization code can be added in the `HAL_setupEncoder` function. The position response data can be read and converted in the `HAL_getMtrEncoderPosition` function. Additional encoder parameters and the encoder handle can be defined in the `hal.h` and `hal.c` files, similar to EnDAT.

#### SDFM Interface

%SDFM parameters for the AM243x LaunchPad™ and Booster Pack:

\imageStyle{umc_sdfm_clock_source_distribution.png,width:60%}
\image html umc_sdfm_clock_source_distribution.png "Clock Source Distribution"

%SDFM Clock:
- PRG0_PRU0_GPO19(G2) → SYNC0 - %SDFM CLOCK_OUT1(J5.45) → SDFM_CLOCK_SOURCE[1|2](TP31)
- AMC_CLKIN_A1, AMC_CLKIN_B1, AMC_CLKIN_C1, PR0_PRU0_SD0_CLK(J4.33), PR0_PRU0_SD1_CLK(J4.31), PR0_PRU0_SD2_CLK(J2.17)
- AMC_CLKIN_A2, AMC_CLKIN_B2, AMC_CLKIN_C2, PR0_PRU0_SD3_CLK(J5.48), PR0_PRU0_SD6_CLK(J1.5), PR0_PRU0_SD7_CLK(J2.14)

Signal Mapping Table:

<table>
<tr>
    <th>Motor</th>
    <th>AM243x LP (PIN NUMBER)</th>
    <th>BP CONNECTORS</th>
    <th>BLDC BP SIGNAL NAME</th>
    <th>Description</th>
</tr>
<tr>
    <td rowspan="3">Motor 1</td>
    <td>PRG0_PRU0_GPI1(J4)</td>
    <td>J4.32</td>
    <td>%SDFM Current High A1</td>
    <td>Sigma-Delta current feedback for Phase A, Motor 1</td>
</tr>
<tr>
    <td>PRG0_PRU0_GPI3(H1)</td>
    <td>J2.19</td>
    <td>%SDFM Current High B1</td>
    <td>Sigma-Delta current feedback for Phase B, Motor 1</td>
</tr>
<tr>
    <td>PRG0_PRU0_GPI5(F2)</td>
    <td>J2.13</td>
    <td>%SDFM Current High C1</td>
    <td>Sigma-Delta current feedback for Phase C, Motor 1</td>
</tr>
<tr>
    <td rowspan="3">Motor 2</td>
    <td>PRG0_PRU0_GPI7(E2)</td>
    <td>J5.44</td>
    <td>%SDFM Current High A2</td>
    <td>Sigma-Delta current feedback for Phase A, Motor 2</td>
</tr>
<tr>
    <td>PRG0_PRU0_GPI8(H5)</td>
    <td>J2.15</td>
    <td>%SDFM Current High B2</td>
    <td>Sigma-Delta current feedback for Phase B, Motor 2</td>
</tr>
<tr>
    <td>PRG0_PRU0_GPO11(L1)</td>
    <td>J2.12</td>
    <td>%SDFM Current High C2</td>
    <td>Sigma-Delta current feedback for Phase C, Motor 2</td>
</tr>
</table>

`HAL_setupSDFM` and `HAL_readMtrSdfmData` functions are used to initialize %SDFM and to get %SDFM samples, respectively. Other relevant macros and %SDFM handle are defined in `hal.h` and `hal.c`. The current example uses a common configuration for both motors and PRU0 channel and RTU channels in load share mode for sampling. The IEP SYNC output is used to generate a 20 MHz %SDFM clock. Parameter configuration can be changed in `hal.h` file and %SDFM SysConfig.

#### EPWM Interface

EPWM (Enhanced Pulse Width Modulation) Signals Configuration

EPWM Signals for Motor 1:

<table>
<tr>
    <th>Signal Type</th>
    <th>AM243x LP (PIN NUMBER)</th>
    <th>BP CONNECTORS</th>
    <th>BLDC BP SIGNAL NAME</th>
    <th>Description</th>
</tr>
<tr>
    <td rowspan="2">Motor Enable</td>
    <td>GPIO1_64(B16)</td>
    <td>J5.49</td>
    <td>nPWM_EN_M1</td>
    <td>DRV1 Motor Enable Control</td>
</tr>
<tr>
    <td colspan="4">Active Low Enable Signal for Motor 1 Drive</td>
</tr>
<tr>
    <td rowspan="2">Phase C PWM</td>
    <td>GPMC0_AD8(U18)</td>
    <td>J4.36</td>
    <td>DRV1 EPWM High C</td>
    <td>Phase C High-Side PWM Signal</td>
</tr>
<tr>
    <td>GPMC0_AD9(U20)</td>
    <td>J4.35</td>
    <td>DRV1 EPWM Low C</td>
    <td>Phase C Low-Side PWM Signal</td>
</tr>
<tr>
    <td rowspan="2">Phase B PWM</td>
    <td>GPMC0_AD5(T20)</td>
    <td>J4.38</td>
    <td>DRV1 EPWM High B</td>
    <td>Phase B High-Side PWM Signal</td>
</tr>
<tr>
    <td>GPMC0_AD6(T18)</td>
    <td>J4.37</td>
    <td>DRV1 EPWM Low B</td>
    <td>Phase B Low-Side PWM Signal</td>
</tr>
<tr>
    <td rowspan="2">Phase A PWM</td>
    <td>GPMC0_AD3(V21)</td>
    <td>J4.40</td>
    <td>DRV1 EPWM High A</td>
    <td>Phase A High-Side PWM Signal</td>
</tr>
<tr>
    <td>GPMC0_AD4(U21)</td>
    <td>J4.39</td>
    <td>DRV1 EPWM Low A</td>
    <td>Phase A Low-Side PWM Signal</td>
</tr>
</table>

EPWM Signals for Motor 2:

<table>
<tr>
    <th>Signal Type</th>
    <th>AM243x LP (PIN NUMBER)</th>
    <th>BP CONNECTORS</th>
    <th>BLDC BP SIGNAL NAME</th>
    <th>Description</th>
</tr>
<tr>
    <td rowspan="2">Motor Enable</td>
    <td>GPIO1_65(B15)</td>
    <td>J5.50</td>
    <td>nPWM_EN_M2</td>
    <td>DRV2 Motor Enable Control</td>
</tr>
<tr>
    <td colspan="4">Active Low Enable Signal for Motor 2 Drive</td>
</tr>
<tr>
    <td rowspan="2">Phase C PWM</td>
    <td>FSI_TX0_CLK (P21)</td>
    <td>J8.79</td>
    <td>DRV2 EPWM High C</td>
    <td>Phase C High-Side PWM Signal</td>
</tr>
<tr>
    <td>FSI_TX0_D0(Y18)</td>
    <td>J8.80</td>
    <td>DRV2 EPWM Low C</td>
    <td>Phase C Low-Side PWM Signal</td>
</tr>
<tr>
    <td rowspan="2">Phase B PWM</td>
    <td>TEST_LED3_RED(D1)</td>
    <td>J8.75</td>
    <td>DRV2 EPWM High B</td>
    <td>Phase B High-Side PWM Signal</td>
</tr>
<tr>
    <td>TEST_LED4_GREEN(F3)</td>
    <td>J8.76</td>
    <td>DRV2 EPWM Low B</td>
    <td>Phase B Low-Side PWM Signal</td>
</tr>
<tr>
    <td rowspan="2">Phase A PWM</td>
    <td>TEST_LED1_GREEN(U19)</td>
    <td>J8.77</td>
    <td>DRV2 EPWM High A</td>
    <td>Phase A High-Side PWM Signal</td>
</tr>
<tr>
    <td>FSI_RX0_D1(V20)</td>
    <td>J8.78</td>
    <td>DRV2 EPWM Low A</td>
    <td>Phase A Low-Side PWM Signal</td>
</tr>
</table>

EPWM configuration is handled by `HAL_setupPWMs` function in `hal.c` file. The parameters used in this function are defined in `epwm.h` files.

\note To update the output frequency of the EPWM, the `#define USER_M1_PWM_FREQ_kHz` macro in `user_mtr.h` file needs to be updated. Additionally, the `#define APP_EPWM_OUTPUT_FREQ` in `epwm.h` also needs to be updated.

# Examples

- \ref EXAMPLE_TIDEP_01032_REFERENCE_DESIGN

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.
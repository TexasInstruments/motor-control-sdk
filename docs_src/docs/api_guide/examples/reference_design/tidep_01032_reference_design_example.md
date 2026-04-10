# TIDEP-01032 EtherCAT-Connected Single-Chip Dual-Servo Motor Drive Reference Design {#EXAMPLE_TIDEP_01032_REFERENCE_DESIGN}

[TOC]

This reference design showcases the ability of the AM243x device to support a fully-integrated real-time servo motor drive control and industrial communication path. This path extends from receiving EtherCAT CiA402 target commands for velocity, to performing closed-loop FOC velocity control of dual connected motors, to passing the actual velocity values back to the EtherCAT PLC.

The implementation features dual-axis BLDC motor control where the control loop runs in the Arm® Cortex®-R5F (R5F_0_0) core. %SDFM interrupt is used to trigger the control loop execution, while EPWM is utilized for motor output control. The system employs EnDat 2.2 absolute encoders for position feedback and ICSS %SDFM module for current feedback.

## R5F_0_0 Example Implementation

1. System Initialization
    - Initializes the driver hardware
    - Configures driver parameters through HAL (Hardware Abstraction Layer)

2. Motor Control Setup
    - Configures control parameters for both motors
    - Sets initial reference speeds (default 8.0 Hz)
    - Configures gate drivers after GPIO initialization

3. Offset Calibration
    - Performs offset calibration for Motor 1
    - Performs offset calibration for Motor 2
    - Essential for accurate current measurements

4. Interrupt Configuration
    - Sets up %SDFM interrupts for both motors
    - Configures interrupt handlers for motors

5. System Enable Sequence
    - Waits one second for system enable flag
    - Uses power relay wait time (POWER_RELAY_WAIT_TIME_ms)
    - Sets initialization completion flags for both motors

6. Main Control Loop
    - Runs continuously while system is enabled
    - Operates on 1ms time base for:
        - LED status updates
        - Motor monitoring
        - RMS calculations
        - Protection checks
        - Controller tuning (if enabled)
    - Executes motor control routines for both motors

7. Shutdown Handling
    - Disables PWM outputs
    - Closes drivers and board interfaces

## Supported/Tested Features

- EPWM frequency up to 50KHz
- Both Open loop and closed loop

## Not Supported/Tested Features

- Motor fault detection and protection checks
- Field weakening control, flying start, MTPA, and braking
- RMS calculations (Not tested)
- Controller tuning (Not tested)
- Encoder offset calibration

## Design Considerations

The single-chip dual-axis servo motor drive implementation is architected around a central real-time path comprising:

- ICSSG1
    - EtherCAT client controller firmware
- ICSSG0
    - %SDFM and EnDAT 2.2 decoding
    - Sigma-Delta filtering firmware with Load Sharing between RTU and PRU cores in PRU0 for phase current feedback from two directly connected motors
    - EnDat2.2 decoding firmware with Load Sharing between RTU and TXPRU cores in PRU1 for position feedback from two directly connected absolute encoders
- R5F_1_0
    - EtherCAT client stack implementing CiA402 using FreeRTOS
- R5F_0_0
    - Two independent closed-loop FOC capable of current, speed, or position closed-loop control for two directly connected motors with absolute encoders
    - EPWM
        - Six channels of enhanced PWM peripherals to generate waveforms based on the output of two FOC loops

### FOC System Timing and Trigger Analysis

The timing diagram shows the complete sequence from PWM cycle start through position feedback, current feedback, and FOC algorithm execution

System Configuration:
- EPWM Frequency: 20kHz (50µs cycle)
- %SDFM Settings:
    - Clock: 20MHz
    - OSR (Over Sampling Ratio): 64
    - Filter Type: SINC3
- EnDAT Settings:
    - Command: 8 (Position value)
    - Clock: 8MHz

\imageStyle{umc_am243x_foc_timing_capture1.png,width:60%}
\image html umc_am243x_foc_timing_capture1.png "FOC Motor Control Timing Analysis with 20kHz Control Loop(1)"

\imageStyle{umc_am243x_foc_timing_capture2.png,width:60%}
\image html umc_am243x_foc_timing_capture2.png "FOC Motor Control Timing Analysis with 20kHz Control Loop(2)"
- EPWM Cycle (P0)
- Position feedback sampling (P1-P2)
- Current feedback sampling (P3-P4)
- FOC algorithm execution (P5-P6)

\note Measurement Considerations: Timing measurements were captured using a Logic Analyzer through GPIO pins. Therefore, there will be nanosecond-level jitter with each timing sequence.

### Components

#### EnDat Configuration

The EnDat 2.2 is used for position feedback. It operates in periodic mode and is synchronized with the EPWM SYNC out event. For Motor 1, Channel 0 of PRU slice 1 runs on the RTU core. For Motor 2, Channel 2 of PRU slice 1 runs on the TX PRU core. Both PRUs send commands to the encoder and store the position value in memory, which is used by R5F for position feedback.

All EnDat configuration and parameter settings are configured in `hal.c` and `hal.h` files. For more information about the EnDat design, please refer to \ref ENDAT.

Below are the settings used in the SDK example. These configurations are done in the `HAL_setupEncoder` function in the `hal.c` file.
- EnDat 2.2 command 8 is used for position feedback.
- Frequency: 8MHz.
- Position trigger point: 25us (middle of EPWM cycle).
    - IEP CMP event is configured as 7500 IEP count for 25us.
To change the trigger point and EnDat frequency, the following macros need to be updated in the `hal.h` file:
```c
#define ENDAT_FREQUENCY                    8000000
#define ENDAT_TRIGGER_POINT                7500
```
\note The current implementation uses a fixed 90-degree (0.5*MATH_PI) offset for all EnDat encoder configurations. While this works for many setups, optimal motor performance may require a proper encoder offset calibration process.The relationship between the encoder's zero position and the motor's electrical reference position can vary based on physical mounting. For custom applications or when precise alignment is required, consider implementing a true offset calibration by:
    - Reading the encoder position after the motor alignment phase
    - Using this value as the offset instead of the fixed 90-degree value
    - This calibration would replace the current fixed offset lines in the `HAL_getMtrEncoderPosition` function in the `hal.c`:
```c
obj->thetaElec_rad =  obj->thetaElec_rad + 0.5*MATH_PI;
```

#### SDFM Configuration

The PRU-ICSS %SDFM interface is used for current feedback and operates in normal current trigger mode. Channels 0 to 2 of PRU slice 0 are used for Motor 1 current feedback, running on the RTU core. Channels 3 to 5 of PRU slice 0 are used for Motor 2 current feedback, running on the PRU core. After sampling is complete, both PRUs store the sampled values in memory and trigger interrupts to the R5F core. The IEP0 SYNC0 output is used to generate a 20 MHz SDFM clock, which is configured by `HAL_setupSDFM`.

All %SDFM configuration and parameter settings are configured in `hal.c` and `hal.h` files. For more information about the %SDFM design, please refer to \ref SDFM.

The following settings are used in the SDK example:

- Normal Current OSR: 64, SINC3 filter
- Frequency: 20MHz
- Normal current trigger point: 30us

`HAL_setupSDFM` and `HAL_readMtrSdfmData` functions are used to initialize %SDFM and to get %SDFM samples, respectively. Other relevant macros and %SDFM handle are defined in `hal.h` and `hal.c`. The current example uses a common configuration for both motors and PRU0 channel and RTU channels in load share mode for sampling. The IEP SYNC output is used to generate a 20 MHz %SDFM clock. Parameter configuration can be changed in `hal.h` file and %SDFM SysConfig.

\note
- The following macros should be updated according to the OSR values and SINC filter types when the OSR value and filter type are changed in the SysConfig.
```c
   /* Sigma delta filter output range for SINC3 OSR64: 64*64*64 */
   #define SDFM_FULL_SCALE         262144.0f
   #define SDFM_HALF_SCALE         131072.0f
```
- Overcurrent detection is not enabled.

#### EPWM Configuration

EPWM0-2 are used for Motor1, and EPWM3_A, EPWM5_B, EPWM7, and EPWM8 are used for Motor2. Time synchronization is used to sync all EPWM modules together.

All EPWM configuration and parameter settings are configured in `hal.c`, `hal.h`, `epwm.h`, and `epwm.c` files.

\note To update the output frequency of the EPWM, the `USER_M1_PWM_FREQ_kHz` macro in the `user_mtr.h` file needs to be updated. Additionally, the `APP_EPWM_OUTPUT_FREQ` in the `app_epwm.h` file also needs to be updated. Moreover, the position and current trigger points should also be adjusted according to the EPWM cycle timing.

\note EPWM trip zone is not enabled.

# Steps to Run the Example

Other than the basic EVM setup mentioned in <a href="@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html" target="_blank">EVM Setup</a>, the following additional hardware is required to run this demo:

## Single Motor Operation

The default example requires two motors and two encoders connected to the LP + BP (Axis 1 and Axis 2). To run a single motor, follow the guidelines below.

### Running Single Motor on Axis 1 Only

To run only Motor 1 on Axis 1, undefine the Motor 2 predefined symbols in the project build settings:
- Remove or undefine all `MOTOR2_*` predefined symbols (e.g., `MOTOR2_ENC`, `MOTOR2_CONNECTED`, etc.)
- Rebuild the project
- Only Motor 1 on Axis 1 will be active

### Running Single Motor on Axis 2 Only

To run only a single motor on Axis 2, code changes are required. The Motor 1 code must be eliminated from the build, similar to how Motor 2 code is guarded using `#if defined(MOTOR2_CONNECTED)`. The Motor 1 code paths need to be wrapped with a discrete predefined symbol (e.g., `MOTOR1_CONNECTED`) or removed, so that only Motor 2 on Axis 2 is active. This approach follows the same pattern already used for Motor 2 conditional compilation.

Steps:
- Add `#if defined(MOTOR1_CONNECTED)` / `#endif` guards around Motor 1 specific code (similar to existing `#if defined(MOTOR2_CONNECTED)` guards for Motor 2)
- Undefine or remove the `MOTOR1_*` predefined symbols from the project build settings
- Rebuild the project

## Hardware, Software, Testing Requirements, and Test Results

### Hardware Requirements

The following equipment is required to test this reference design:

- One Microsoft® Windows® personal computer (PC) with TwinCAT automation software installed
- One <a href="https://www.ti.com/tool/LP-AM243" target="_blank">LP-AM243 Evaluation board Rev A</a>
- One <a href="https://www.ti.com/tool/BP-AM2BLDCSERVO" target="_blank">BP-AM2BLDCSERVO</a> — AM2x Brushless-DC (BLDC) Servo Motor BoosterPack
- Two BLY342D-48V-3200 Anaheim Automation 3-phase Brushless DC motors
- Two ROQ-437 EnDat2.2 Encoders with cables

\note
- The PROC109A version of LP supports two channels
- To enable the second channel on LP, SW6 needs to be turned OFF

#### Booster Pack Jumper Configuration

<table>
<tr>
    <th>Designator</th>
    <th>ON/OFF</th>
    <th>Description</th>
</tr>
<tr>
    <td>J11</td>
    <td>OFF</td>
    <td>VSENSE/ISENSE select</td>
</tr>
<tr>
    <td>J13</td>
    <td>OFF</td>
    <td>VSENSE/ISENSE select</td>
</tr>
<tr>
    <td>J17</td>
    <td>Pin 1-2 Connected</td>
    <td>%SDFM Clock Feedback Select</td>
</tr>
<tr>
    <td>J18/J19</td>
    <td>J19 installed: sets VSENSOR1 to 5.0V</td>
    <td>Axis 1: Encoder/Resolver Voltage Select</td>
</tr>
<tr>
    <td>J20/J21</td>
    <td>J21 installed: sets VSENSOR2 to 5.0V</td>
    <td>Axis 2: Encoder/Resolver Voltage Select</td>
</tr>
<tr>
    <td>J22</td>
    <td>OFF</td>
    <td>Axis 1: Manchester Encoding Select</td>
</tr>
<tr>
    <td>J23</td>
    <td>OFF</td>
    <td>Axis 2: Manchester Encoding Select</td>
</tr>
<tr>
    <td>J24</td>
    <td>OFF</td>
    <td>Axis 1: RS485/DSL MUX</td>
</tr>
<tr>
    <td>J25</td>
    <td>OFF</td>
    <td>Axis 2: RS485/DSL MUX</td>
</tr>
<tr>
    <td>J26</td>
    <td>OFF</td>
    <td>VSENSE/ISENSE Select</td>
</tr>
<tr>
    <td>J27</td>
    <td>ON</td>
    <td>Encoder and %SDFM Paths Select</td>
</tr>
<tr>
    <td>J28</td>
    <td>OFF</td>
    <td>AM243/AM263 Mode</td>
</tr>
</table>

\imageStyle{umc_system_hw_configuration.png,width:60%}
\image html umc_system_hw_configuration.png "System Hardware Configuration"

\imageStyle{umc_am243x_lp_rev_A_and_bldc_bp_e2.png,width:60%}
\image html umc_am243x_lp_rev_A_and_bldc_bp_e2.png "AM243x LP Rev A and BLDC BP E2"

\imageStyle{umc_bp_e2_connectors_for_m1_and_m2.png,width:60%}
\image html umc_bp_e2_connectors_for_m1_and_m2.png "BLDC BP E2 Connectors for Motor 1 and Motor 2"

\imageStyle{umc_bp_axis1_and_axis2_motor_connectors.png,width:60%}
\image html umc_bp_axis1_and_axis2_motor_connectors.png "BLDC BP E2 Power and Motors connectors"

\imageStyle{umc_star_config_for_m1_and_m2.png,width:60%}
\image html umc_star_config_for_m1_and_m2.png "Star Configuration for Motor 1 and Motor 2"

\imageStyle{umc_EtherCAT_connected_motor_setup.png,width:60%}
\image html umc_EtherCAT_connected_motor_setup.png "EtherCAT Connected Motor Control Setup"

### Software Requirements

Import and build the system project from:
`C:\ti\motor_control_sdk_am243x_xx\examples\tidep_01032_dual_motor_drive\universal_single_chip_servo\am243x-lp-BP-AM2BLDCSERVO\system_freertos_nortos`

The project has 2 sub-projects:
- `C:\ti\motor_control_sdk_am243x_xx\examples\tidep_01032_dual_motor_drive\universal_single_chip_servo\am243x-lp-BP-AM2BLDCSERVO\r5fss0-0_nortos` (universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang: motor drive project for both axes)
- `C:\ti\motor_control_sdk_am243x_xx\examples\tidep_01032_dual_motor_drive\universal_single_chip_servo\am243x-lp-BP-AM2BLDCSERVO\r5fss1-0_freertos` (universal_single_chip_servo_am243x-lp_r5fss1-0_freertos_ti-arm-clang: EtherCAT CiA402 client code)

### Test Results

The system is gradually tested and verified in multiple stages to ensure the final system can be operated confidently. To select a particular build option, change the value of the DMC_BUILDLEVEL define to the desired DMC_LEVEL_X option in the `sys_settings.h` file. After selecting the build option, compile the project by right-clicking on the project name and clicking Rebuild Project.

After importing and building the system project, the executable binary files for R5F_0_0 and R5F_1_0 appear in the CCS workspace directory:

1. Connect to the target AM243x LP using the target configuration file
2. Load and run R5F_0_0 example

#### Build Level 1

Objectives for this build level:
- Use the HAL object to initialize the peripherals of the MCU for the motor drive hardware
- Verify the PWM and %SDFM modules
- Verify the %SDFM Offset validation

In this build level, the board executes in open loop mode with a fixed PWM duty cycle of 50%. This build level verifies the sensing of feedback values from the power stage and operation of the PWM gate driver to ensure there are no hardware issues. Additionally, calibration of input and output voltage sensing can be performed.

\note During this process, the motor must remain disconnected.

##### Run and Results

1. Load and Run R5F_0_0 example
2. Halt R5F_0_0 and set up the watch window: Click View → Expressions on the menu bar to open an Expressions watch window
3. Import variables into the Expressions window by right-clicking within the window and clicking Import. Browse to the project directory at `<workspace>\universal_motorcontrol_am243x_r5fss0-0_nortos_ti-armclang\src_control\debug\`, select universal_motor1_control_level1.txt and universal_motor2_control_level1.txt files, and click OK to import.

\imageStyle{umc_build_level1_expression_window.png,width:60%}
\image html umc_build_level1_expression_window.png "Build Level 1: Variables in Expressions Window"

4. In the Expressions window, set motorVars_M1.flagEnableRunAndIdentify to 1 after systemVars.flagEnableSystem is automatically set to 1

5. In the watch view, motorVars_M1.flagRunIdentAndOnLine will be set to 1 automatically if there are no faults. The ISRCount should increase continuously.

6. Check the calibration offsets of the motor driver board. The offset values of the motor phase current sensing values should be approximately half of the %SDFM scale current.

7. Using an oscilloscope, probe the PWM outputs used for motor drive control. The duty cycles of the three PWMs are set to 50% in this build level. Verify that the PWM switching frequencies match the value set for USER_M1_PWM_FREQ_kHz in the user_mtr1.h file.

\imageStyle{umc_build_level1_pwm_output.png,width:60%}
\image html umc_build_level1_pwm_output.png "Build Level 1: PWM Output Waveforms"

8. Set motorVars_M1.flagEnableRunAndIdentify to 0 to disable the PWMs.

9. If any of the previous steps provide unexpected results, check the following:
    - Confirm the motor driver board being used matches the board selected in the build configurations
    - Verify the proper predefines are set
    - Check that the jumpers are configured properly on the LaunchPad and BoosterPack

\note The steps for Motor 2 are the same as for Motor 1. Just update and observe the variables of motorVars_M2.

#### Build Level 2

Objectives for this build level:
- Implement simple scalar v/f control of motor to validate current and voltage sensing circuits and gate driver circuit
- Test open loop for motor control

In this build level, the system runs with open-loop control. The %SDFM values are used only for verification and validation, not in the actual control loop of the motor.

- Connect the motor to the appropriate terminals on the motor driver evaluation board.
- Set DMC_BUILDLEVEL to DMC_LEVEL_2 and rebuild the example.

##### Run and Results

1. Load and Run R5F_0_0 example
2. Halt R5F_0_0 and set up the watch window: Click View → Expressions to open an Expressions watch window
3. Import variables by right-clicking within the Expressions window and clicking Import. Browse to `<workspace>\universal_motorcontrol_am243x_r5fss0-0_nortos_ti-armclang\src_control\debug\`, select universal_motor1_control_level2.txt and universal_motor2_control_level2.txt, and click OK

4. Resume execution by clicking the Resume button or Run → Resume. The systemVars.flagEnableSystem will be set to 1 after a fixed time, indicating completion of offset calibration

5. To verify the current sensing circuits, set motorVars_M1.flagEnableRunAndIdentify to 1 in the Expressions window. The motor will run with voltage/frequency (v/f) open loop. If the motor doesn't spin smoothly, tune the v/f profile parameters in user_mtr1.h according to your motor's specifications:

```c
#define USER_MOTOR1_FREQ_LOW_HZ  (5.0)   // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ (400.0) // Hz
#define USER_MOTOR1_VOLT_MIN_V   (1.0)   // Volt
#define USER_MOTOR1_VOLT_MAX_V   (24.0)  // Volt
```

\note Modifying these parameters requires rebuilding the project.

6. The motorVars_M1.speedRef_Hz variable sets the speed reference. Verify that the actual speed (motorVars_M1.speed_Hz) closely matches the reference speed (motorVars_M1.speedRef_Hz)

\note Runtime changes to MotorVars_M1.speedRef_Hz require halting the R5F core. To avoid sudden motor stops, use EtherCAT for speed control as described in the EtherCAT CiA402 Client Example section

7. Validate current sensing, rotor angle estimator, and generator using the DATALOG module for viewing waveforms

8. To use DATALOG with the graph tool:
    - To test the phase currents using the DATALOG module, the following code must be set up in sys_main.c:
    ```c
    // Note this code is already configured by default for Motor 1 in `sys_main.c`
    // (similar configuration is possible for motor2):
    datalogObj->iptr[0] = (float32_t*) &motorVars_M1.sdfmData.I_A.value[0];
    datalogObj->iptr[1] = (float32_t*) &motorVars_M1.sdfmData.I_A.value[1];
    datalogObj->iptr[2] = (float32_t*) &motorVars_M1.sdfmData.I_A.value[2];
    datalogObj->iptr[3] = (float32_t*) &motorVars_M1.angleFOC_rad;
    ```
    - Enable DATALOG in predefined symbols
    - Rebuild the example and import data log file in Graph tool window from `<workspace>\universal_motorcontrol_am243x_r5fss0-0_nortos_ti-armclang\src_control\debug\`
    - sampling signals waveform displayed on the graph tool as shown in Figure

\note With DATALOG enabled, any variables can be observed using the Graph tool. Just the required variables need to be configured in `sys_main.c` as described.

\imageStyle{umc_build_level2_current_readings.png,width:60%}
\image html umc_build_level2_current_readings.png "Build Level 2: Motor Phase Current Waveforms with Graph Tool"

\imageStyle{umc_build_level2_angle_waveforms.png,width:60%}
\image html umc_build_level2_angle_waveforms.png "Build Level 2: Motor Rotor Angle Waveforms With Graph Tool"

9. Set motorVars_M1.flagEnableRunAndIdentify to 0 to stop the motor

\note The steps for Motor 2 are the same as for Motor 1. Just update and observe the variables of motorVars_M2.

#### Build Level 3

Objectives for this build level:
- Evaluate closed current loop operation of the motor
- Verify current sensing parameter settings

In this build level, the motor is controlled using i/f control with rotor angle generated from the ramp generator module.

Connect the motor to the appropriate terminals on the power inverter board and set DMC_BUILDLEVEL to DMC_LEVEL_3.

##### Run and Results

1. Load and Run R5F_0_0 example
2. Halt R5F_0_0 and import universal_motor1_control_level3.txt and universal_motor2_control_level3.txt into the Expressions window
3. Set motorVars_M1.flagEnableRunAndIdentify to 1 to verify closed-loop current control. The motor will run using the angle from the angle generator at the speed set in motorVars_M1.speedRef_Hz
4. If the motor fails to run with current-closed loop control, verify motorVars_M1.sdfmData.current_sf and userParams_M1.current_sf settings match your hardware board
5. Set motorVars_M1.flagEnableRunAndIdentify to 0 to stop the motor
6. To verify the encoder measured angle `angleENC_rad`, enable and configure the DATALOG module as described in Build level2

\note The steps for Motor 2 are the same as for Motor 1. Just update and observe the variables of motorVars_M2.

#### Build Level 4

Objectives for this build level:
- Evaluate complete motor drive with absolute encoder-based FOC
- Test closed outer speed loop with inner current loop using absolute encoder feedback

Required motor parameters must be defined in user_mtr1.h as per motor specification and characteristics:

```c
#define USER_MOTOR1_TYPE              MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS
#define USER_MOTOR1_Rr_Ohm
#define USER_MOTOR1_Rs_Ohm
#define USER_MOTOR1_Ls_d_H
#define USER_MOTOR1_Ls_q_H
#define USER_MOTOR1_RATED_FLUX_VpHz
```

##### Run and Results

1. Build and load the code into the R5F_0_0 core. Wait for systemVars.flagEnableSystem to be set to 1, indicating completion of offset calibration and power relay activation
2. Set `flagEnableRunAndIdentify` to 1 for both motors
3. Control motor speed through the EtherCAT CiA402 Client Example section instructions

#### EtherCAT CiA402 Client Example

This section describes the steps to run the EtherCAT sub-device CiA402 demo example (R5F_1_0) and control the speed of both motors using TwinCAT.

- Connect the R5F_1_0
- Load and run `ethercat_subdevice_cia402_demo_am243x-lp_r5fss1-0_freertos_ti-arm-clang` example
- The EtherCAT CiA402 client device is now ready for TwinCAT detection
- Install and launch TwinCAT on your Windows PC

##### TwinCAT Setup

1. Create an EtherCAT Project in TwinCAT

\imageStyle{umc_am243x_TwinCAT_new_project.png,width:60%}
\image html umc_am243x_TwinCAT_new_project.png "Creating an EtherCAT® Project in TwinCAT"

2. Scan for devices by right-clicking on Devices → Scan

\imageStyle{umc_am243x_TwinCAT_scan_device1.png,width:60%}
\image html umc_am243x_TwinCAT_scan_device1.png "Scan for EtherCAT® device in TwinCAT(1)"

\imageStyle{umc_am243x_TwinCAT_scan_device2.png,width:60%}
\image html umc_am243x_TwinCAT_scan_device2.png "Scan for EtherCAT® device in TwinCAT(2)"

\imageStyle{umc_am243x_TwinCAT_scan_device3.png,width:60%}
\image html umc_am243x_TwinCAT_scan_device3.png "Scan for EtherCAT® device in TwinCAT(3)"

\imageStyle{umc_am243x_TwinCAT_scan_device4.png,width:60%}
\image html umc_am243x_TwinCAT_scan_device4.png "Scan for EtherCAT® device in TwinCAT(4)"

\imageStyle{umc_am243x_TwinCAT_scan_device5.png,width:60%}
\image html umc_am243x_TwinCAT_scan_device5.png "Scan for EtherCAT® device in TwinCAT(5)"

3. Verify Device 1 (TI EtherCAT Toolkit CiA402 for AM243X.R5F) is found

\imageStyle{umc_am243x_TwinCAT_device_found.png,width:60%}
\image html umc_am243x_TwinCAT_device_found.png "An EtherCAT® device is found by TwinCAT"

##### Motor Speed Control

###### MOTOR1

1. Set RxPDO (Motor 1) Target velocity to `240` (240 RPM)

\imageStyle{umc_am243x_TwinCAT_set_target_velocity1.png,width:60%}
\image html umc_am243x_TwinCAT_set_target_velocity1.png "Change the Target Velocity for Motor 1 in TwinCAT(1)"

\imageStyle{umc_am243x_TwinCAT_set_target_velocity2.png,width:60%}
\image html umc_am243x_TwinCAT_set_target_velocity2.png "Change the Target Velocity for Motor 1 in TwinCAT(2)"

2. Set RxPDO (Motor 1) Modes of Operation to `"9"` (Cyclic synchronous velocity mode)
3. Set RxPDO (Motor 1) Controlword to `"15"` (Switch On | Enable Voltage | Quick Stop | Enable Operation)
4. Verify Motor 1 speed changes from `120` RPM to `240` RPM
5. Verify TxPDO (Motor 1) Velocity actual value shows `240` RPM

\imageStyle{umc_am243x_TwinCAT_actual_velocity.png,width:60%}
\image html umc_am243x_TwinCAT_actual_velocity.png "Check the Actual Velocity for Motor 1 in TwinCAT"

###### MOTOR2

1. Set RxPDO_1 (Motor 2) Target velocity to `180` (180 RPM)
2. Set RxPDO_1 (Motor 2) Modes of Operation to `"9"`
3. Set RxPDO_1 (Motor 2) Controlword to `"15"`
4. Verify Motor 2 reaches target speed of `180` RPM
5. Verify TxPDO1 (Motor 2) Velocity actual value shows `180` RPM

\note For instructions on how to enable DC mode, refer to the section titled "Testing DC Synchronization mode" in the document <a href="@VAR_IC_SDK_DOCS_PATH/ETHERCAT_SUBDEVICE_DEMO_TWINCAT.html" target="_blank">@VAR_SOC_NAME EtherCAT SubDevice Setup with TwinCAT</a>.

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

# TIDEP-01032 EtherCAT-Connected Single-Chip Dual-Servo Motor Drive Reference Design {#EXAMPLE_TIDEP_01032_REFERENCE_DESIGN}

This reference design showcases the ability of the AM243x device to support a fully-integrated real-time servo motor drive control and industrial communication path. This path extends from receiving EtherCAT CiA402 target commands for velocity, to performing closed-loop FOC velocity control of dual connected motors, to passing the actual velocity values back up to the EtherCAT PLC.

## Features Supported

- Support EtherCAT CiA402 device profile for motor velocity control
- Single-chip, dual-servo motor control
- BOOST-XL TI BoosterPack Plug-in Module design - 80 digital and analog I/O compatible with AM2x LaunchPad Development Kits
- Two axes of 3-phase BLDC motor drive with the DRV8316R 24 V, 8 A monolithic gate drive and amplifier bridges
- Two axes (6 channels) of 3-phase current feedback through AMC1035D Sigma-Delta modulator and INA241A current sense path
- Two axes of RS-485 based absolute encoder feedback supporting multiple industrial encoder standards

\note Currently the FOC loop used for Closed Speed (Velocity) Control is based on the CMSIS library. We will switch to the RTLib functions and DCL functions in the near future.
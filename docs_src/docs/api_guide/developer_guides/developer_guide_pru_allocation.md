\cond SOC_AM64X || SOC_AM243X
# How to allocate PRUs for different functions within one PRU-ICSSG ? {#DEVELOPER_GUIDE_PRU_ALLOCATION}

[TOC]

Each PRU can perform different functions like \ref POSITION_SENSE, \ref CURRENT_SENSE, \ref PRUICSS_PWM, Industrial Communication Protocol, MDIO emulation, custom interfaces, etc. This page explains how to realize different functions with multiple PRU cores available within the device.

## PRU-ICSSG overview
One PRU-ICSSG is divided into two slices (Slice 0 and Slice 1), each having 2 programmable real-time unit (PRU) cores; namely Programmable Real-time Units (PRU0/PRU1), Real-Time Units (RTU_PRU0/RTU_PRU1), and Transmit Programmable Real-Time Units (TX_PRU0/TX_PRU1). In total, there are 6 cores.

\image html pruicssg_block_diagram.png "PRU-ICSSG Block Diagram"

For more details on PRU-ICSSG, please see section "6.4 Programmable Real-Time Unit and Industrial Communication Subsystem - Gigabit (PRU_ICSSG)" of <a href="https://www.ti.com/lit/ug/spruim2h/spruim2h.pdf" target="_blank">AM243x Technical Reference Manual</a>.

For more details on input/output modes, see \ref PRUICSSG_IO_MODES.

## PRU allocation for different functions

\attention **Pin mode selection is done per PRU-ICSSG slice. If Peripheral IF mode is selected for a slice, all pins from that slice are selected for Peripheral IF mode and cannot be used for other modes like GPIO or SD or MII.**

The following sections show PRU requirements for different functions. Please refer to the module-specific page for detailed resource requirements of each module.

### Position Sense

- 1 PRU for 1 channel of BISS-C/EnDat/HDSL/Nikon A-format/Tamagawa
- 1 PRU for 3 channels of BISS-C/EnDat/Nikon A-format/Tamagawa
- 1 PRU slice (PRU/RTU_PRU/TX_PRU cores in load share mode) for 3 channels of BISS-C/EnDat/HDSL/Nikon A-format.
    - Each core is mapped to one channel.
    - For more details on load share mode, refer to the \ref PRUICSSG_LOAD_SHARE_MODE section.
- Refer to the module-specific page for more details on limitations with different modes and features.
- For multi-channel configuration, 1 PRU slice can only implement the same encoder type (BISS-C/EnDat/HDSL/Nikon A-format/Tamagawa) and the same interface frequency.
    - Each PRU-ICSSG has 2 slices. Different slices can implement different encoder types.

### Current Sense

- 1 PRU for 3 SD channels
- 1 PRU slice (PRU/RTU_PRU/TX_PRU cores in load share mode) for 9 SD channels
    - Each core is mapped to 3 channels.
        - RTU_PRU: Channels 0, 1, 2
        - PRU: Channels 3, 4, 5
        - TX_PRU: Channels 6, 7, 8
    - For more details on load share mode, refer to the \ref PRUICSSG_LOAD_SHARE_MODE section.
- Refer to the module-specific page for more details on limitations with different modes and features.

### PRUICSS PWM

- PRU-ICSS PWM can be used with any PRU-ICSS input/output mode given that the pinmux for PRU-ICSS pins does not have conflicts.
- A PRU core may or may not be needed for programming PRU-ICSS PWM based on required features. Refer to \ref PRUICSS_PWM for more details.

### Industrial Communication Protocol

- All 100M Industrial Communication Protocol (EtherCAT, EtherNet/IP, and Profinet) implementations use one full PRU-ICSSG. In terms of processing cores, they need both PRU0 and PRU1.
- If the silicon is affected by errata <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf" target="_blank">i2329— MDIO interface corruption</a>, then usage of MDIO Manual mode via PRU emulation is required. This takes up one PRU core from the PRU-ICSSG used for industrial communication.

### Summary of PRU allocation

<table>
    <tr>
        <th rowspan="2">Component</th>
        <th rowspan="2">Function</th>
        <th colspan="3">Slice 0</th>
        <th colspan="3">Slice 1</th>
        <th rowspan="2">Notes</th>
    </tr>
    <tr>
        <th>PRU0</th>
        <th>RTU_PRU0</th>
        <th>TX_PRU0</th>
        <th>PRU1</th>
        <th>RTU_PRU1</th>
        <th>TX_PRU1</th>
    </tr>
    <tr>
        <td rowspan="6">Position Sense</td>
        <td>1 channel BISS-C/EnDat/HDSL/Nikon A-format/Tamagawa</td>
        <td></td>
        <td></td>
        <td></td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td><ul><li>Can also use PRU0</li></ul></td>
    </tr>
    <tr>
        <td>3 channels of BISS-C/EnDat/Nikon A-format/Tamagawa</td>
        <td></td>
        <td></td>
        <td></td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td><ul><li>Can also use PRU0</li><li>All channels run with same interface frequency</li><li>Limited features. See encoder-specific page for more details.</li></ul></td>
    </tr>
    <tr>
        <td>3 channels of BISS-C/EnDat/HDSL/Nikon A-format</td>
        <td></td>
        <td></td>
        <td></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td><ul><li>Uses load share mode</li><li>All channels run with same interface frequency</li><li>Can also use Slice 0 (PRU0 + RTU_PRU0 + TX_PRU0)</li></ul></td>
    </tr>
    <tr>
        <td>1 channel of type A and 1 channel of type B<br/>(type A/B can be BISS-C/EnDat/HDSL/Nikon A-format)</td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td></td>
    </tr>
    <tr>
        <td>3 channels of type A and 3 channels of type B<br/>(type A/B can be BISS-C/EnDat/HDSL/Nikon A-format)</td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td><ul><li>All channels on 1 slice run with same interface frequency</li><li>Limited features. See encoder-specific page for more details.</li></ul></td>
    </tr>
    <tr>
        <td>3 channels of type A and 3 channels of type B<br/>(type A/B can be BISS-C/EnDat/HDSL/Nikon A-format)</td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td><ul><li>Uses load share mode</li><li>All channels on 1 slice run with same interface frequency</li></ul></td>
    </tr>
    <tr>
        <td rowspan="4">Current Sense</td>
        <td>Up to 3 channels<br/>(Normal current + overcurrent/fast detect)</td>
        <td></td>
        <td></td>
        <td></td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td><ul><li>Can also use PRU0</li><li>Same channel input used for normal current and overcurrent</li><li>Refer to the module-specific page for more details on limitations with different modes and features</li></ul></td>
    </tr>
    <tr>
        <td>6 channels<br/>(Normal current + overcurrent/fast detect)</td>
        <td></td>
        <td></td>
        <td></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td><ul><li>Can use any 2 cores from the same slice</li><li>Same channel input used for normal current and overcurrent</li><li>Refer to the module-specific page for more details on limitations with different modes and features</li></ul></td>
    </tr>
    <tr>
        <td>9 channels<br/>(Normal current + overcurrent/fast detect)</td>
        <td></td>
        <td></td>
        <td></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td><ul><li>Uses load share mode</li><li>Can also use Slice 0 (PRU0 + RTU_PRU0 + TX_PRU0)</li><li>Same channel input used for normal current and overcurrent</li><li>Refer to the module-specific page for more details on limitations with different modes and features</li></ul></td>
    </tr>
    <tr>
        <td>Up to 9 channels<br/>(Normal current only)</td>
        <td></td>
        <td></td>
        <td></td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td><ul><li>Can also use PRU0</li><li>Example not available in Motor Control SDK</li></ul></td>
    </tr>
    <tr>
        <td rowspan="3">PWM</td>
        <td>18 EPWM signals</td>
        <td></td>
        <td></td>
        <td></td>
        <td></td>
        <td></td>
        <td></td>
        <td><ul><li>No PRU core needed for SoC EPWM</li></ul></td>
    </tr>
    <tr>
        <td>24 PRUICSS PWM signals</td>
        <td></td>
        <td></td>
        <td></td>
        <td></td>
        <td></td>
        <td></td>
        <td><ul><li>No PRU core needed</li><li>Arm®-based core programs PRU-ICSS PWM and PRU-ICSS IEP registers</li></ul></td>
    </tr>
    <tr>
        <td>24 PRUICSS PWM signals</td>
        <td></td>
        <td></td>
        <td></td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td><ul><li>Can use any PRUx/RTU_PRUx/TX_PRUx core</li><li>PRU programs PRU-ICSS PWM and PRU-ICSS IEP registers</li></ul></td>
    </tr>
    <tr>
        <td rowspan="2">Industrial Communication Protocol/Ethernet</td>
        <td>100M Industrial Communication Protocol<br/>(EtherCAT, EtherNet/IP, and Profinet)</td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td bgcolor="#008000">MDIO</td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td><ul><li>MDIO emulation is needed due to <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf" target="_blank">i2329— MDIO interface corruption</a> errata</li><li>MDIO emulation can be done in Arm®-based core or any free PRUx/RTU_PRUx/TX_PRUx core</li><li>IEP SYNC and LATCH needed for time synchronization</li></ul></td>
    </tr>
    <tr>
        <td>1G Ethernet</td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td bgcolor="#008000"></td>
        <td><ul><li>MDIO emulation is done in Arm®-based core</li><li>IEP SYNC and LATCH needed for time synchronization</li></ul></td>
    </tr>
    <tr>
        <td>IOLINK</td>
        <td>8 channels of IOLINK</td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td bgcolor="#008000"></td>
        <td></td>
        <td></td>
        <td><ul><li>See IOLINK-specific page in Industrial Communications SDK for more details.</li></ul></td>
    </tr>
</table>

### Examples of function combinations

The following are a few example combinations. More combinations can be created based on input/output mode selection per PRU-ICSSG slice and conditions mentioned in the sections above.

\note
    - Use SysConfig to check pin feasibility for the desired use case.

#### 6-axis motor control

\image html pruicssg_6_axis.png "6-axis motor control"

\note
    - With 6-axis PWM generation, there is a limit on SD. In total, only 15 out of 18 SDF channels can be supported.
    - More details on the 2-axis reference design are available on the \"\ref EXAMPLE_TIDEP_01032_REFERENCE_DESIGN\" page.

#### 3-axis motor control with industrial communication

\image html pruicssg_3_axis_with_ind_comms.png "3-axis motor control with industrial communication"

#### 9-axis motor control with industrial communication

Two AM243x devices can be used to create a 9-axis motor control solution by combining the "3-axis motor control with industrial communication" and "6-axis motor control" designs mentioned in the sections above.

\endcond


\cond SOC_AM261X || SOC_AM263PX || SOC_AM263X
# How to allocate PRUs for different functions within one PRU-ICSSM ? {#DEVELOPER_GUIDE_PRU_ALLOCATION}

[TOC]

Each PRU can perform different functions like \ref POSITION_SENSE, Industrial Communication Protocol, custom interfaces, etc. This page explains how to realize different functions with multiple PRU cores available within the device.

## PRU-ICSSM overview
One PRU-ICSSM has 2 programmable real-time unit (PRU) cores.

\image html pruicssm_block_diagram.png "PRU-ICSSM Block Diagram"

\cond SOC_AM261X
For more details on PRU-ICSSM, please see section "7.3 Programmable Real-Time Unit Subsystem (PRU-ICSS)" of <a href="https://www.ti.com/lit/ug/sprujb6b/sprujb6b.pdf" target="_blank">AM261x Technical Reference Manual</a>.
\endcond
\cond SOC_AM263PX
For more details on PRU-ICSSM, please see section "7.3 Programmable Real-Time Unit Subsystem (PRU-ICSS)" of <a href="https://www.ti.com/lit/ug/spruj55d/spruj55d.pdf" target="_blank">AM263Px Technical Reference Manual</a>.
\endcond

For more details on input/output modes, see \ref PRUICSSM_IO_MODES.

## PRU allocation for different functions

\attention **Pin mode selection is done per PRU. If Peripheral IF mode is selected for a PRU, all pins from that PRU are selected for Peripheral IF mode and cannot be used for other modes like GPIO or SD or MII.**

The following sections show PRU requirements for different functions. Please refer to the module-specific page for detailed resource requirements of each module.

### Position Sense

- 1 PRU for 1 channel of BISS-C/EnDat/HDSL/Nikon A-format/Tamagawa
- For multi-channel configuration, 1 PRU can only implement the same encoder type (BISS-C/EnDat/HDSL/Nikon A-format/Tamagawa) and the same interface frequency.
    - Each PRU-ICSSM has 2 PRUs. Different PRUs can implement different encoder types.

### Industrial Communication Protocol

- All 100M Industrial Communication Protocol (EtherCAT, EtherNet/IP, and Profinet) implementations use one full PRU-ICSSM. In terms of processing cores, they need both PRU0 and PRU1.

### Example of function combinations

The following is an example combination. More combinations can be created based on input/output mode selection per PRU and conditions mentioned in the sections above.

\note
    - Use SysConfig to check pin feasibility for the desired use case.

#### 2-axis motor control with industrial communication

\image html pruicssm_2_axis_with_ind_comms.png "2-axis motor control with industrial communication"

\endcond

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

\note A-format® is a registered trademark of the Nikon Corporation.
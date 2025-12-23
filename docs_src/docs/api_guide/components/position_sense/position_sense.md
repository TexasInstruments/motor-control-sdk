# Position Sense {#POSITION_SENSE}

[TOC]

Real-time communication with encoders and current sensing is typically handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implement the low-level firmware. The PRU-ICSS frees up the main Arm®-based cores in the device for other functions, such as control and data processing.

Applications and PRU-ICSS firmwares for the following position sense encoders are provided in the SDK:

\cond SOC_AM64X || SOC_AM243X
- \subpage BISS-C
- \subpage ENDAT
- \subpage ENDAT3
- \subpage HDSL
- \subpage NIKON
- \subpage TAMAGAWA
\endcond

\cond (SOC_AM263X || SOC_AM263PX)
- \subpage BISS-C
- \subpage ENDAT
- \subpage ENDAT3
- \subpage NIKON
- \subpage TAMAGAWA
- \subpage TAMAGAWA_OVER_UART (Application only, PRU is not needed)
\endcond

\cond SOC_AM261X
- \subpage BISS-C
- \subpage ENDAT
- \subpage ENDAT3
- \subpage HDSL
- \subpage NIKON
- \subpage TAMAGAWA
\endcond

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

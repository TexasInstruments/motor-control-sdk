# Position Sense {#POSITION_SENSE}

[TOC]

\cond SOC_AM64X || SOC_AM243X

Real-time communication with encoders and current sensing is typically handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implement the low level firmware. The PRU-ICSS frees up the main ARM cores in the device for other functions, such as control and data processing.

Applications and PRU-ICSS firmwares for following position sense encoders are provided in the SDK.

- \subpage ENDAT
- \subpage HDSL
- \subpage TAMAGAWA
\cond SOC_AM243X
- \subpage BISS-C
\endcond
\endcond

\cond SOC_AM263X

Application for following position sense encoder is provided in the SDK.

- \subpage TAMAGAWA_OVER_UART

\endcond

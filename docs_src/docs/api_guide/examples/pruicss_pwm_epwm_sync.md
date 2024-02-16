# PRU-ICSS PWM EPWM SYNC {#EXAMPLE_PRUICSS_PWM_EPWM_SYNC}

[TOC]

# Introduction

This example generates a signal for a specified period and duty cycle using
PRU-ICSS PWM and SOC EPWM. The period and duty cycle can be configured by the user.

The example Uses PRU-ICSSG0 PWM module and does below

- Configures EPWM0_CHANNEL_A to generate a 1KHz signal with 25% duty cycle
- Configures EPWM0_SYNC_OUT to output high when SOC EPWM PERIOD VAL reaches zero
- Configures IEP counter reset on EPWM0_SYNC_OUT event
- Configures PWM0_0_POS(alias signal PWM0_A0) to generate a 1KHz signal with 50% duty cycle 
- Configures PWM3_2_NEG(alias signal PWM3_B2) to generate a 1KHz signal with 75% duty cycle 
- PWM0_0_POS(alias signal PWM0_A0) uses IEP0 CMP1  EVENT to control Duty cycle and EPWM0 SYNC OUT to control output Frequency
- PWM3_2_NEG(alias signal PWM3_B2) uses IEP1 CMP12 EVENT to control Duty cycle and EPWM0 SYNC OUT to control output Frequency
- Configures IEP0 CMP6 value with PWM0_0_POS(alias signal PWM0_A0) output duty cycle value
- Configures IEP1 CMP12 value with PWM3_2_NEG(alias signal PWM3_B2) output duty cycle value
- Configures IEP0 CMP0 value with zero to make state transition to intial on IEP counter reset
- PWM0_0_POS(alias signal PWM0_A0) and PWM3_2_NEG(alias signal PWM3_B2) and EPWM0_CHANNEL_A will be in sync with respect to each other as EPWM0_SYNC_OUT is used to  control output period of these signals


#### AM243X-LP 
- PRG0_PWM0_A0 can be probed on J1.5
- PRG0_PWM3_B2 can be probed on J2.8
- EPWM0_CHANNEL_A can be probed on J4.1

#### AM64X-EVM and AM243X-EVM 
An IO Breakout Board (BB) is required to probe the PWM outputs
- PRG0_PWM0_A0 can be probed on J3.1 of IO Breakout Board
- PRG0_PWM3_B2 can be probed on J2.11 of IO Breakout Board
- EPWM0_CHANNEL_A can be probed on J6.1 of IO Breakout Board

#### Probe Output

\imageStyle{epwm_pruicss_pwm_sync_probe_output.png,width:85%}
\image html epwm_pruicss_pwm_sync_probe_output.png "PRU-ICSS PWM EPWM SYNC PROBE OUTPUT"

# Supported Combinations

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/pruicss_pwm/pruicss_pwm_epwm_sync

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/pruicss_pwm/pruicss_pwm_epwm_sync

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using
  make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>
- To probe the PRUICSS PWM output please refer setup details as mentioned above in Introduction section

# See Also

\ref PRUICSS_PWM_API

# Additional Details

## Steps to sync PRUICSS PWM from SOC EPWM

This example demonstrates synchronization of PRUICSS PWM with SOC EPWM. For synchronization of SOC EPWM with PRUICSS PWM, follow the steps below 

- Compare 0 event which controls period of PRUICSS PWM is mapped to cmp_event_router_input_16
- Configure compare event router to mux cmp_event_router_output_40 with cmp_event_router_input_16.
- Select source of SOC EPWM sync input signal as cmp_event_router_output_40

\code

/* define the unlock and lock values */
#define KICK_LOCK_VAL                           (0x00000000U)
#define KICK0_UNLOCK_VAL                        (0x68EF3490U)
#define KICK1_UNLOCK_VAL                        (0xD172BC5AU)

void unlock_Partition_1_CTRLMMR()
{
    /* EPWMCTRL MMR is present at Proxy0 Offset Range 0x4000 t0 0x5FFF
     * Refer 5.1.3.1.2 of TRM to find Lock register & its unlock value
     * */
    volatile uint32_t  *kickAddr;
    kickAddr = (volatile uint32_t *) (0x43005008);
    CSL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);   /* KICK 0 */
    kickAddr++;
    CSL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);   /* KICK 1 */
}
void lock_Partition_1_CTRLMMR()
{
    volatile uint32_t  *kickAddr;
    kickAddr = (volatile uint32_t *) (0x43005008);
    CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);   /* KICK 0 */
    kickAddr++;
    CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);   /* KICK 1 */
}
void EPWM_config_syncin_event()
{
    unlock_Partition_1_CTRLMMR();
    CSL_REG32_WR(CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL+CSL_CTRL_MMR0_CFG0_BASE,0x300);
    lock_Partition_1_CTRLMMR();
}

void config_compare_event_router_out()
{
    /*Configure compare event router to mux cmp_event_router_output_40 with cmp_event_router_input_16*/
    volatile uint32_t  *Addr ;
    uint32_t CMP_EVENT_INTROUTER_MUXCTRL_ADDR_OFFSET = 0x00000004;
    uint32_t CMP_EVENT_INTROUTER0_OUTP_40_EPWM0_SYNC_IN_ADDR_OFFSET = 0x28 * 4 ;
    Addr =(uint32_t *)( CSL_CMP_EVENT_INTROUTER0_CFG_BASE + CMP_EVENT_INTROUTER_MUXCTRL_ADDR_OFFSET + CMP_EVENT_INTROUTER0_OUTP_40_EPWM0_SYNC_IN_ADDR_OFFSET);
    CSL_REG32_WR(Addr,0x10010);
}

\endcode

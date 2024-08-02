# PRU-ICSS PWM DEADBAND EPWM SYNC {#EXAMPLE_PRUICSS_PWM_DEADBAND_EPWM_SYNC}

[TOC]

# Introduction


This example uses the PRUICSS PWM module to generate a signal with a specified duty cycle and deadband at rise edge and fall edge of PWM, syncing pruicss PWM with EPWM sync cout
 
The default parameters are in the example are: 
  
- Frequency : 16kHz  
- PWM0_0_POS(alias signal PWM0_A0) is configured with duty cycle of 25% , rise edge delay as 0ns and fall edge delay as 0ns 
- PWM0_0_NEG(alias signal PWM0_B0) is configured with duty cycle of 25% , rise edge delay as 200ns and fall edge delay as 400ns 
- PWM2_0_POS(alias signal PWM2_A0) is configured with duty cycle of 25% , rise edge delay as 0ns and fall edge delay as 0ns 
- PWM2_0_NEG(alias signal PWM2_B0) is configured with duty cycle of 25% , rise edge delay as 600ns and fall edge delay as 800ns 

All these parameters are configurable.

## Code Flow  

### PRUICSS IEP configuration

- IEP shadow mode and slave mode are enabled (Refer section 6.4.13 of Technical Reference Manual) and IEP is configured to reset twice on every PWM period as mentioned below

- EPWM0 sync out is configured to generate every PRUICSS PWM period, PRUICSS IEP COMPARE 0 is configured with one IEP cycle delay ((PWM_PERIOD/2)+1). 
- By configuring PRUICSS IEP COMPARE 0 with one IEP cycle delay, IEP COMPARE 0 event is missed at the end of PRUICSS PWM period.
- PRUICSS IEP CMP0 resets IEP couter in middle of PRUICSS PWM period.
- EPWM0 sync out resets IEP counter at end of PRUICSS PWM period.   

### PRUICSS PWM configuration

- PWM signal is configured to low in Intial state, Toggle in Active state, Change of state from Active to Intial is disabled on IEP0 CMP0 event.

- PWM Duty cycle, rise edge delay, fall edge delay can configured or updated using \ref PRUICSS_PWM_config API call.

- PWM Period can be configured or updated using \ref PRUICSS_PWM_pruIcssPwmFrequencyInit API call . 

- PWM0_0_POS, PWM0_0_NEG, PWM2_0_POS, PWM2_0_NEG uses IEP0 CMP0 and EPWM0 sync out signal to achieve PWM synchronization.

- Individual compare event mapped to PWM_x signals controls duty cycle.

Below two steps are executed once in PWM period 

\imageStyle{epwm_pruicss_pwm_sync_dead_band.png, width:100% height:50%}
\image html epwm_pruicss_pwm_sync_dead_band.png "GENERATION OF PRUICSS PWM SIGNAL"

- Step1 : When EPWM0 sync out resets IEP counter, PWM signals are moved to intial state on software reset, shadow compare register values which decides rise edge are moved to active register values, next compare value which decides fall edge of PWM signal is computed and shadow compare register field is updated.

- Step2 : When PRUICSS IEP CMP0 resets IEP counter, shadow compare register values which decides fall edge are moved to active register values, new compare values which decides rise edge of PWM signal are computed from updated duty cycle and shadow compare register field is updated.

The pruicss PWM signal generated is similar to EPWM signal when EPWM counter is configured in up-down mode.

#### Note 
This example uses EPWM0 sync out to reset IEP at the PWM period, PRUICSS IEP CMP0 can also be used to do this when EPWM sync out signal is disabled.


#### AM243X-LP 
- When AM243X-LP PROC109A revision is used to test this example, SW6 should be in open state to probe PRG0_PWM0_B0 on J2.15, Refer <a href="https://www.ti.com/lit/ug/spruj12f/spruj12f.pdf?ts=1711526744464&ref_url=https%253A%252F%252Fwww.ti.com%252Ftool%252FLP-AM243" target="_blank"> AM243X-LP User Guide </a>  Section 4.2.1
- PRG0_PWM0_A0 can be probed on J1.5
- PRG0_PWM0_B0 can be probed on J2.15
- PRG0_PWM2_A0 can be probed on J4.31
- PRG0_PWM2_B0 can be probed on J4.39
- EPWM0_CHANNEL_A can be probed on J4.40

#### AM64X-EVM and AM243X-EVM 
<a href="https://www.ti.com/tool/TMDS64DC01EVM" target="_blank"> An IO Breakout Board </a> is required to probe the PWM outputs
- PRG0_PWM0_A0 can be probed on J3.1
- PRG0_PWM0_B0 can be probed on J3.3
- PRG0_PWM2_A0 can be probed on J2.5
- PRG0_PWM2_B0 can be probed on J6.9
- EPWM0_CHANNEL_A can be probed on J6.7

#### Probe Output

\imageStyle{epwm_pruicss_pwm_sync_dead_band_probe_output_2.png, width:98%}
\image html epwm_pruicss_pwm_sync_dead_band_probe_output_2.png "PRU-ICSS0 PWM0_A0 and PWM0_B0 DUTYCYCLE and FREQUENCY"

\imageStyle{epwm_pruicss_pwm_sync_dead_band_probe_output_1.png, width:98%}
\image html epwm_pruicss_pwm_sync_dead_band_probe_output_1.png "PRU-ICSS0 PWM2_A0 and PWM2_B0 DUTYCYCLE and FREQUENCY"

\imageStyle{epwm_pruicss_pwm_sync_dead_band_probe_output_3.png, width:98%}
\image html epwm_pruicss_pwm_sync_dead_band_probe_output_3.png "PRU-ICSS0 PWM0_A0 and PWM0_B0 DEADBAND and PRU-ICSS0 PWM0_A0 and PWM0_B0 DEADBAND"

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/pruicss_pwm/pruicss_pwm_dead_band_epwm_sync

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/pruicss_pwm/pruicss_pwm_dead_band_epwm_sync

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination and build it using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
- **When using makefiles to build**, note the required combination and build using make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)
- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>
- To probe the PRUICSS PWM output please refer setup details as mentioned above in Introduction section

# See Also

\ref PRUICSS_PWM_API

# Additional Details

## Steps to synchronize PRUICSS PWM from SOC EPWM

This example demonstrates synchronization of PRUICSS PWM with SOC EPWM. For synchronization of SOC EPWM with PRUICSS PWM, follow the steps below 

- Compare 0 event which controls period of PRUICSS PWM is mapped to cmp_event_router_input_16.
- Configure compare event router to mux cmp_event_router_output_40 with cmp_event_router_input_16.
- Select source of SOC EPWM sync input signal as cmp_event_router_output_40.

\code

/* define the unlock and lock values */
#define KICK_LOCK_VAL                           (0x00000000U)
#define KICK0_UNLOCK_VAL                        (0x68EF3490U)
#define KICK1_UNLOCK_VAL                        (0xD172BC5AU)

void unlock_Partition_1_CTRLMMR()
{
    /* EPWMCTRL MMR is present at Proxy0 Offset Range 0x4000 t0 0x5FFF
     * Refer 5.1.3.1.2 of TRM to find Lock register and its unlock value
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

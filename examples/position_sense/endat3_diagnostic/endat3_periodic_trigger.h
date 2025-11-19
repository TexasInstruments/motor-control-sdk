/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ENDAT3_PERIODIC_TRIGGER_H_
#define _ENDAT3_PERIODIC_TRIGGER_H_

#include<stdint.h>
#include<drivers/pruicss.h>

/* External global variables - defined in endat3_periodic_trigger.c */
extern PRUICSS_Handle gPruIcssXHandle;
extern void *gPruss_iep;
extern uint32_t gPruEnDat3IrqCnt0;
extern uint32_t gPruEnDat3IrqCnt1;
extern uint32_t gPruEnDat3IrqCnt2;

/**
 * \brief EnDat3 Periodic Interface Structure
 * 
 * This structure defines the interface for periodic trigger configuration
 * in EnDat3 encoders. It contains pointers to PRU-ICSS resources and
 * compare register values for IEP timer configuration.
 */
struct endat3_periodic_interface
{
  void *pruss_iep;      /**< Pointer to ICSS IEP (Industrial Ethernet Peripheral) timer */
  void *pruss_dmem;     /**< Pointer to PRU data memory */
  void *pruss_cfg;      /**< Pointer to PRU configuration */
  uint8_t load_share;   /**< Load share mode flag (0 or 1) */
  uint64_t cmp0;        /**< Compare register 0 value (64-bit) - counter reset */
  uint64_t cmp3;        /**< Compare register 3 value (64-bit) - periodic trigger */
  uint64_t cmp5;        /**< Compare register 5 value (64-bit) - channel 1 trigger */
  uint64_t cmp6;        /**< Compare register 6 value (64-bit) - channel 2 trigger */
};

/* IEP Timer Configuration Macros */
#define IEP_DEFAULT_INC    0x1                /**< IEP default increment value */
#define IEP_DEFAULT_INC_EN  0x4               /**< IEP default increment enable */
#define IEP_COUNTER_EN      0x1               /**< IEP counter enable bit */
#define IEP_RST_CNT_EN      0x1               /**< IEP reset counter enable */
#define IEP_CMP0_ENABLE     (0x1 << 1)        /**< IEP compare 0 enable */
#define IEP_CMP3_EVNT       (0x1 << 3)        /**< IEP compare 3 event flag */
#define IEP_CMP5_EVNT       (0x1 << 5)        /**< IEP compare 5 event flag */
#define IEP_CMP6_EVNT       (0x1 << 6)        /**< IEP compare 6 event flag */

/* PRU Interrupt Event Definitions */
#define PRU_TRIGGER_HOST_ENDAT3_EVT0   (2+16)    /**< pr0_pru_mst_intr[2]_intr_req - Channel 0 */
#define PRU_TRIGGER_HOST_ENDAT3_EVT1   (3+16)    /**< pr0_pru_mst_intr[3]_intr_req - Channel 1 */
#define PRU_TRIGGER_HOST_ENDAT3_EVT2   (4+16)    /**< pr0_pru_mst_intr[4]_intr_req - Channel 2 */

/**
 * \brief Configure EnDat3 periodic mode
 * 
 * Initializes the EnDat3 interface for periodic triggering using the IEP timer.
 * This function configures the IEP timer with compare registers, initializes
 * the ICSS interrupt controller, and sets up interrupt handlers.
 *
 * \param endat3_periodic_interface Pointer to periodic interface configuration structure
 * \param handle PRUICSS handle for the ICSS instance
 *
 * \return 1 on success, 0 on failure
 * 
 * \note This function must be called after PRUICSS initialization and before
 *       starting periodic communication.
 * 
 * \code
 * struct endat3_periodic_interface periodic_cfg = {
 *     .pruss_iep = iep_base,
 *     .pruss_dmem = dmem_base,
 *     .pruss_cfg = cfg_base,
 *     .load_share = 0,
 *     .cmp0 = 0x100000000,  // 4 seconds at 250MHz
 *     .cmp3 = 0x0FA00000    // 1 second at 250MHz
 * };
 * 
 * if (endat3_config_periodic_mode(&periodic_cfg, pruicss_handle)) {
 *     // Periodic mode configured successfully
 * }
 * \endcode
 */
uint32_t endat3_config_periodic_mode(struct endat3_periodic_interface *endat3_periodic_interface, PRUICSS_Handle handle);

/**
 * \brief Stop EnDat3 periodic mode
 * 
 * Disables the IEP timer and stops periodic triggering. This function
 * resets the IEP counter and disables the counter enable bit.
 *
 * \param endat3_periodic_interface Pointer to periodic interface configuration structure
 *
 * \return void
 * 
 * \code
 * endat3_stop_periodic_continuous_mode(&periodic_cfg);
 * \endcode
 */
void endat3_stop_periodic_continuous_mode(struct endat3_periodic_interface *endat3_periodic_interface);

/**
 * \brief EnDat3 periodic trigger interrupt handler (Channel 0)
 * 
 * Interrupt service routine for channel 0 periodic trigger events.
 * Handles CMP3 event clearing and interrupt acknowledgment.
 *
 * \param args Pointer to arguments (typically NULL)
 *
 * \return void
 * 
 * \note This is an internal function called by the interrupt controller.
 */
static void pruEnDat3IrqHandler0(void *args);

/**
 * \brief EnDat3 periodic trigger interrupt handler (Channel 1)
 * 
 * Interrupt service routine for channel 1 periodic trigger events.
 * Handles CMP5 event clearing and interrupt acknowledgment.
 *
 * \param args Pointer to arguments (typically NULL)
 *
 * \return void
 * 
 * \note This is an internal function called by the interrupt controller.
 */
static void pruEnDat3IrqHandler1(void *args);

/**
 * \brief EnDat3 periodic trigger interrupt handler (Channel 2)
 * 
 * Interrupt service routine for channel 2 periodic trigger events.
 * Handles CMP6 event clearing and interrupt acknowledgment.
 *
 * \param args Pointer to arguments (typically NULL)
 *
 * \return void
 * 
 * \note This is an internal function called by the interrupt controller.
 */
static void pruEnDat3IrqHandler2(void *args);

#endif /* _ENDAT3_PERIODIC_TRIGGER_H_ */

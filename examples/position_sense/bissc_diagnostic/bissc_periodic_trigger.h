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

#ifndef _BISSC_PERIODIC_TRIGGER_H_
#define _BISSC_PERIODIC_TRIGGER_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include<stdint.h>
#include <position_sense/bissc/include/bissc_drv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief IEP counter default increment value (1 per clock cycle) */
#define IEP_DEFAULT_INC     0x1

/** \brief IEP counter enable bit in Global Config register (start counter) */
#define IEP_COUNTER_EN      0x1

/** \brief IEP reset counter on CMP0 event enable bit */
#define IEP_RST_CNT_EN      0x1

/** \brief IEP Compare 0 (CMP0) event enable bit (bit 1 in CMP_CFG_REG) */
#define IEP_CMP0_ENABLE     (0x1 << 1)

/** \brief IEP Compare event number for Channel 0 trigger */
#define IEP_CH0_CMP_EVNT ( 3 )

/** \brief IEP Compare event number for Channel 1 trigger */
#define IEP_CH1_CMP_EVNT ( 5 )

/** \brief IEP Compare event number for Channel 2 trigger */
#define IEP_CH2_CMP_EVNT ( 6 )

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief   Structure defining BiSS-C periodic trigger interface configuration
 *
 * \details Contains BiSS-C driver handle, IEP timer base pointer, and trigger
 *          count values for periodic mode operation. Used to configure IEP timer
 *          for automatic BiSS-C transaction triggering at specified intervals.
 */
typedef struct bissc_periodic_interface_s
{
  bissc_handle handle;
  /**< BiSS-C driver handle obtained from bissc_init().
   *   Used to access driver configuration and PRU-ICSS resources */

  uint64_t ch0_trigger_count;
  /**< IEP counter value for Channel 0 periodic trigger (in IEP clock cycles). */

  uint64_t ch1_trigger_count;
  /**< IEP counter value for Channel 1 periodic trigger (in IEP clock cycles). */

  uint64_t ch2_trigger_count;
  /**< IEP counter value for Channel 2 periodic trigger (in IEP clock cycles). */

  uint64_t iep_reset_count;
  /**< IEP counter reset value (in IEP clock cycles) for CMP0 event.
   *   When IEP counter reaches this value, it resets to 0, creating periodic cycles */
} bissc_periodic_interface;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

uint32_t bissc_config_periodic_mode(bissc_periodic_interface *bissc_periodic_interface);

void bissc_stop_periodic_mode(bissc_periodic_interface *bissc_periodic_interface);

void bissc_periodic_interface_init(bissc_handle handle, bissc_periodic_interface *bissc_periodic_interface_instance, int64_t ch0_trigger_count,
                                    int64_t ch1_trigger_count, int64_t ch2_trigger_count, int64_t iep_reset_count);

#endif /* _BISSC_PERIODIC_TRIGGER_H_ */

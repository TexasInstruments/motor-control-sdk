/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#ifndef SVGEN_CURRENT_H
#define SVGEN_CURRENT_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \addtogroup TRANSFORMS_API_MODULE APIs for Transformation Algorithms
 *
 *  Here is the list of SVGEN_CURRENT function APIs
 *  @{
 *
 *  \file           svgen_current.h
 *  \brief          Contains space vector generation current (SVGENCURRENT) functions implementation
 */

#include <stdlib.h>
#include <math.h>

#include "math_types.h"

// **************************************************************************
// the typedefs

typedef enum
{
    SVGENCURRENT_USE_ALL = 0,     //!< Use all shunt measurements
    SVGENCURRENT_IGNORE_A,        //!< Ignore the A phase shunt measurement
    SVGENCURRENT_IGNORE_B,        //!< Ignore the B phase shunt measurement
    SVGENCURRENT_IGNORE_C,        //!< Ignore the C phase shunt measurement
    SVGENCURRENT_IGNORE_AB,       //!< Ignore the AB phase shunt measurement
    SVGENCURRENT_IGNORE_AC,       //!< Ignore the AC phase shunt measurement
    SVGENCURRENT_IGNORE_BC,       //!< Ignore the BC phase shunt measurement
    SVGENCURRENT_IGNORE_ALL       //!< Ignore the ABC phase shunt measurement
} SVGENCURRENT_IgnoreShunt_e;

typedef enum
{
    SVGENCURRENT_ALL_PHASE_MEASURABLE = 1,  //!< all shunt measurable
    SVGENCURRENT_TWO_PHASE_MEASURABLE,      //!< just two shunt measurable
    SVGENCURRENT_ONE_PHASE_MEASURABLE,      //!< just one shunt measurable
    SVGENCURRENT_IMMEASUREABLE
} SVGENCURRENT_MeasureShunt_e;

typedef enum
{
    SVGENCURRENT_VMID_A=0,        //!< Middle voltage is A phase
    SVGENCURRENT_VMID_B,          //!< Middle voltage is B phase
    SVGENCURRENT_VMID_C           //!< Middle voltage is C phase
} SVGENCURRENT_VmidShunt_e;

//! \brief Defines the Svgen Current object
//!
typedef struct _SVGENCURRENT_Obj_
{
  int16_t                       minWidth;    //!< The maximum width where a valid measurement cannot be taken
  SVGENCURRENT_IgnoreShunt_e    ignoreShunt; //!< Output of what shunt or shunts to ignore
  SVGENCURRENT_MeasureShunt_e   compMode;    //!< Output phase compensation mode
  SVGENCURRENT_VmidShunt_e      Vmid;        //!< The middle amplitude voltage among the three phase voltages
  float32_t                     Vlimit;      //!< The maximum output voltage duty that current can be sampled
  int16_t                       Voffset;     //!< The offset
} SVGENCURRENT_Obj;

//! \brief Defines the Svgen Current handle
//!
typedef struct _SVGENCURRENT_Obj_ *SVGENCURRENT_Handle;


// **************************************************************************
// the function prototypes

//*****************************************************************************
//
//! \brief     Initializes the svgen current object
//! \param[in] *pMemory         Pointer in to the svgen current object
//! \param[in] numBytes         Size of the object
//
//*****************************************************************************
extern SVGENCURRENT_Handle
SVGENCURRENT_init(void *pMemory,const size_t numBytes);

//*****************************************************************************
//
//! \brief     Sets up the PWM minimum width
//! \param[in] handle         The svgen current (SVGEN) handle
//! \param[in] minWidth_usec  The minimum pwm width
//! \param[in] pwmFreq_kHz    The PWM frequency, in MHz
//! \param[in] systemFreq_MHz The system SoC frequency, in MHz
//
//*****************************************************************************
void SVGENCURRENT_setup(SVGENCURRENT_Handle handle, const float32_t minWidth_usec,
                   const float32_t pwmFreq_kHz, const float32_t systemFreq_MHz);

//*****************************************************************************
//
//! \brief     Sets the minimum Duty Cycle width that the lower switch can be on before
//! \brief     the current data is invalid.
//! \param[in] handle           The Svgen Current handle
//! \param[in] minwidth         Integer value of the minimum number of pwm counts
//
//*****************************************************************************
static inline void
SVGENCURRENT_setMinWidth(SVGENCURRENT_Handle handle,
                         const int16_t minwidth)
{
  SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;

  obj->minWidth = minwidth;

  return;
} // end of SVGENCURRENT_setMinWidth() function

//*****************************************************************************
//
//! \brief     Sets the ignore shunt value
//! \param[in] handle         The Svgen Current handle
//! \param[in] ignoreShunt    The ignore shunt value
//
//*****************************************************************************
static inline void
SVGENCURRENT_setIgnoreShunt(SVGENCURRENT_Handle handle,
                            const SVGENCURRENT_IgnoreShunt_e ignoreShunt)
{
  SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;

  obj->ignoreShunt = ignoreShunt;

  return;
} // end of SVGENCURRENT_setIgnoreShunt() function

//*****************************************************************************
//
//! \brief     Sets the compensation mode
//! \param[in] handle     The Svgen Current handle
//! \param[in] compMode   The compensation mode
//
//*****************************************************************************
static inline void
SVGENCURRENT_setMode(SVGENCURRENT_Handle handle,
                     const SVGENCURRENT_MeasureShunt_e compMode)
{
  SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;

  obj->compMode = compMode;

  return;
} // end of SVGENCURRENT_setMode() function

//*****************************************************************************
//
//! \brief     Sets the output voltage limit value for gurrantee a current sampling
//! \param[in] handle  The Svgen Current handle
//! \param[in] Vlimit  The output voltage limit
//
//*****************************************************************************
static inline void
SVGENCURRENT_setVlimit(SVGENCURRENT_Handle handle,
                       const float32_t Vlimit)
{
  SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;

  obj->Vlimit = Vlimit;

  return;
} // end of SVGENCURRENT_setVlimit() function

//*****************************************************************************
//
//! \brief     Gets the ignore shunt value
//! \param[in] handle  The Svgen Current handle
//! \return    Ignore shunt value
//
//*****************************************************************************
static inline SVGENCURRENT_IgnoreShunt_e
SVGENCURRENT_getIgnoreShunt(SVGENCURRENT_Handle handle)
{
  SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;

  return(obj->ignoreShunt);
} // end of SVGENCURRENT_getIgnoreShunt() function

//*****************************************************************************
//
//! \brief     Gets the minimum Duty Cycle width that the lower switch can be on before
//! \brief     the current data is invalid.
//! \param[in] handle           The Svgen Current handle
//! \return    Integer value of the minimum number of pwm counts
//
//*****************************************************************************
static inline int16_t
SVGENCURRENT_getMinWidth(SVGENCURRENT_Handle handle)
{
  SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;

  return(obj->minWidth);
} // end of SVGENCURRENT_getMinWidth() function

//*****************************************************************************
//
//! \brief     Gets the Voltage(Duty) Limit value
//! \param[in] handle  The Svgen Current handle
//! \return    Integer value of the voltage(duty) limit
//
//*****************************************************************************
static inline float32_t
SVGENCURRENT_getVlimit(SVGENCURRENT_Handle handle)
{
  SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;

  return(obj->Vlimit);

} // end of SVGENCURRENT_getVlimit() function

//*****************************************************************************
//
//! \brief     Gets the current reconstruction mode
//! \param[in] handle  The Svgen Current handle
//! \return    CompMode
//
//*****************************************************************************
static inline SVGENCURRENT_MeasureShunt_e
SVGENCURRENT_getMode(SVGENCURRENT_Handle handle)
{
  SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;

  return(obj->compMode);

} // end of SVGENCURRENT_getMode() function

//*****************************************************************************
//
//! \brief     Gets the middle amplitude voltage
//! \param[in] handle  The Svgen Current handle
//! \return    middle voltage
//
//*****************************************************************************
static inline SVGENCURRENT_VmidShunt_e
SVGENCURRENT_getVmid(SVGENCURRENT_Handle handle)
{
  SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;

  return(obj->Vmid);

} // end of SVGENCURRENT_getVmiddle() function

//*****************************************************************************
//
//! \brief     Gets the svgen current module ignore shunt
//! \brief     In the pwm structure, the value variable is the on-time of the low fet.
//! \brief     A low value is a small on-time for the low switch of the bridge and thus a short current window.
//! \param[in] handle              The Svgen Current handle
//! \param[in] cmp1                compare value 1
//! \param[in] cmp2                compare value 2
//! \param[in] cmp3                compare value 3
//! \param[in] cmpM1               active compare value 1, from mirror register
//! \param[in] cmpM2               active compare value 2, from mirror register
//! \param[in] cmpM3               active compare value 3, from mirror register
//
//*****************************************************************************
static __attribute__((always_inline))
void SVGENCURRENT_RunIgnoreShunt(SVGENCURRENT_Handle handle,
                            uint16_t cmp1, uint16_t cmp2, uint16_t cmp3,
                            uint16_t cmpM1, uint16_t cmpM2, uint16_t cmpM3)
{
  SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;
  uint16_t minWidth;

  minWidth = obj->minWidth;

  uint16_t nextPulse1 = (cmp1 + cmpM1)>>1;
  uint16_t nextPulse2 = (cmp2 + cmpM2)>>1;
  uint16_t nextPulse3 = (cmp3 + cmpM3)>>1;

  if(nextPulse1 < minWidth)
    {
      if((nextPulse2 < minWidth) || ((cmp2 - cmp1) < minWidth))
        {
          obj->ignoreShunt = SVGENCURRENT_IGNORE_AB;
        }
      else if((nextPulse3 < minWidth) || ((cmp3 - cmp1) < minWidth))
        {
          obj->ignoreShunt = SVGENCURRENT_IGNORE_AC;
        }
      else
        {
          obj->ignoreShunt = SVGENCURRENT_IGNORE_A;
        }
    }
  else if(nextPulse2 < minWidth)
    {
      if((nextPulse1 < minWidth) || ((cmp1 - cmp2) < minWidth))
        {
          obj->ignoreShunt = SVGENCURRENT_IGNORE_AB;
        }
      else if((nextPulse3 < minWidth) || ((cmp3 - cmp2) < minWidth))
        {
          obj->ignoreShunt = SVGENCURRENT_IGNORE_BC;
        }
      else
        {
          obj->ignoreShunt = SVGENCURRENT_IGNORE_B;
        }
    }
  else if(nextPulse3 < minWidth)
    {
      if((nextPulse1 < minWidth) || ((cmp1 - cmp3) < minWidth))
        {
          obj->ignoreShunt = SVGENCURRENT_IGNORE_AC;
        }
      else if((nextPulse2 < minWidth) || ((cmp2 - cmp3) < minWidth))
        {
          obj->ignoreShunt = SVGENCURRENT_IGNORE_BC;
        }
      else
        {
          obj->ignoreShunt = SVGENCURRENT_IGNORE_C;
        }
    }
  else
    {
      obj->ignoreShunt = SVGENCURRENT_USE_ALL;
    }

  return;

} // end of SVGENCURRENT_RunIgnoreShunt() function

//*****************************************************************************
//
//! \brief     Reconstructs the missed measured currents due to a small sampling window
//! \param[in] handle         The svgen current handle
//! \param[in] pADCData       Pointer to the shunt currents
//! \param[in] pADCDataPrev   Pointer to the previous shunt currents
//
//*****************************************************************************
static __attribute__((always_inline))
void SVGENCURRENT_RunRegenCurrent(SVGENCURRENT_Handle handle,
                             MATH_Vec3 *pADCData, MATH_Vec3 *pADCDataPrev)
{
  SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;

  float32_t Ia = pADCData->value[0];
  float32_t Ib = pADCData->value[1];
  float32_t Ic = pADCData->value[2];

  // select valid shunts and ignore one when needed
  if(obj->ignoreShunt == SVGENCURRENT_IGNORE_A)
  {
      // repair a based on b and c
      Ia = -Ib - Ic;       //Ia = -Ib - Ic;
  }
  else if(obj->ignoreShunt == SVGENCURRENT_IGNORE_B)
  {
      // repair b based on a and c
      Ib = -Ia - Ic;       //Ib = -Ia - Ic;
  }
  else if(obj->ignoreShunt == SVGENCURRENT_IGNORE_C)
  {
      // repair c based on a and b
      Ic = -Ia - Ib;       //Ic = -Ia - Ib;
  }
  else if(obj->ignoreShunt == SVGENCURRENT_IGNORE_AB)
  {
      Ia = -Ic * 0.5f;
      Ib = Ia;
  }
  else if(obj->ignoreShunt == SVGENCURRENT_IGNORE_AC)
  {
      Ia = -Ib * 0.5f;
      Ic = Ia;
  }
  else if(obj->ignoreShunt == SVGENCURRENT_IGNORE_BC)
  {
      Ib = -Ia * 0.5f;
      Ic = Ib;
  }

  pADCData->value[0] = Ia;
  pADCData->value[1] = Ib;
  pADCData->value[2] = Ic;

  pADCDataPrev->value[0] += (pADCData->value[0] - pADCDataPrev->value[0]) * 0.5f;
  pADCDataPrev->value[1] += (pADCData->value[1] - pADCDataPrev->value[1]) * 0.5f;
  pADCDataPrev->value[2] += (pADCData->value[2] - pADCDataPrev->value[2]) * 0.5f;

  if(obj->compMode > SVGENCURRENT_TWO_PHASE_MEASURABLE)
  {
      pADCData->value[0] = pADCDataPrev->value[0];
      pADCData->value[1] = pADCDataPrev->value[1];
      pADCData->value[2] = pADCDataPrev->value[2];
  }

  return;
} // end of SVGENCURRENT_RunRegenCurrent() function

//*****************************************************************************
//
//! \brief     output voltage reconsturction to guarantee min duty in two phase at least
//! \param[in] handle         The svgen current handle
//! \param[in] pPWMData       The pointer of the PWM data
//! \param[in] pPWMData_prev  The pointer of old PWM data
//
//*****************************************************************************
static __attribute__((always_inline)) 
void SVGENCURRENT_compPWMData(SVGENCURRENT_Handle handle,
                         MATH_Vec3 *pPWMData, MATH_Vec3 *pPWMData_prev)
{
    SVGENCURRENT_Obj *obj = (SVGENCURRENT_Obj *)handle;

    float32_t Va_avg = (pPWMData->value[0] + pPWMData_prev->value[0]) * 0.5f;
    float32_t Vb_avg = (pPWMData->value[1] + pPWMData_prev->value[1]) * 0.5f;
    float32_t Vc_avg = (pPWMData->value[2] + pPWMData_prev->value[2]) * 0.5f;

    float32_t Vlimit = obj->Vlimit;
    float32_t Vmid, Vmid_prev, Voffset;

    //define compensation mode
    if(Va_avg > Vlimit)
    {
        if(Vb_avg > Vlimit)
        {
            obj->compMode = SVGENCURRENT_ONE_PHASE_MEASURABLE;

            if(Va_avg > Vb_avg)
            {
                obj->Vmid = SVGENCURRENT_VMID_B;
                Vmid = pPWMData->value[1];
                Vmid_prev = pPWMData_prev->value[1];
            }
            else
            {
                obj->Vmid = SVGENCURRENT_VMID_A;
                Vmid = pPWMData->value[0];
                Vmid_prev = pPWMData_prev->value[0];
            }
        }
        else if(Vc_avg > Vlimit)
        {
            obj->compMode = SVGENCURRENT_ONE_PHASE_MEASURABLE;

            if(Va_avg > Vc_avg)
            {
                obj->Vmid = SVGENCURRENT_VMID_C;
                Vmid = pPWMData->value[2];
                Vmid_prev = pPWMData_prev->value[2];
            }
            else
            {
                obj->Vmid = SVGENCURRENT_VMID_A;
                Vmid = pPWMData->value[0];
                Vmid_prev = pPWMData_prev->value[0];
            }
        }
        else
        {
            obj->compMode = SVGENCURRENT_TWO_PHASE_MEASURABLE;

            if(Vb_avg > Vc_avg)
            {
                obj->Vmid = SVGENCURRENT_VMID_B;
            }
            else
            {
                obj->Vmid = SVGENCURRENT_VMID_C;
            }
        }
    }
    else
    {
        if(Vb_avg > Vlimit)
        {
            if(Vc_avg > Vlimit)
            {
                obj->compMode = SVGENCURRENT_ONE_PHASE_MEASURABLE;

                if(Vb_avg > Vc_avg)
                {
                    obj->Vmid = SVGENCURRENT_VMID_C;
                    Vmid = pPWMData->value[2];
                    Vmid_prev = pPWMData_prev->value[2];
                }
                else
                {
                    obj->Vmid = SVGENCURRENT_VMID_B;
                    Vmid = pPWMData->value[1];
                    Vmid_prev = pPWMData_prev->value[1];
                }
            }
            else
            {
                obj->compMode = SVGENCURRENT_TWO_PHASE_MEASURABLE;

                if(Va_avg > Vc_avg)
                {
                    obj->Vmid = SVGENCURRENT_VMID_A;
                }
                else
                {
                    obj->Vmid = SVGENCURRENT_VMID_C;
                }
            }
        }
        else if(Vc_avg > Vlimit)
        {
            obj->compMode = SVGENCURRENT_TWO_PHASE_MEASURABLE;

            if(Va_avg > Vb_avg)
            {
                obj->Vmid = SVGENCURRENT_VMID_A;
            }
            else
            {
                obj->Vmid = SVGENCURRENT_VMID_B;
            }
        }
        else
        {
            obj->compMode = SVGENCURRENT_ALL_PHASE_MEASURABLE;
            obj->ignoreShunt = SVGENCURRENT_USE_ALL;
        }
    }

    //phase voltage compensator
    if(obj->compMode > SVGENCURRENT_TWO_PHASE_MEASURABLE)
    {
        Voffset = (Vmid + Vmid_prev) * 0.5f - Vlimit;

        if(pPWMData->value[0] > -0.50f)
        {
            pPWMData->value[0] -= Voffset;
        }

        if(pPWMData->value[1] > -0.50f)
        {
            pPWMData->value[1] -= Voffset;
        }

        if(pPWMData->value[2] > -0.50f)
        {
            pPWMData->value[2] -= Voffset;
        }

        obj->Voffset = Voffset;
    }

    // get ignore current
    if(((pPWMData->value[0] + pPWMData_prev->value[0]) * 0.5f) > Vlimit)
    {
        obj->ignoreShunt = SVGENCURRENT_IGNORE_A;
    }
    else if(((pPWMData->value[1] + pPWMData_prev->value[1]) * 0.5f) > Vlimit)
    {
        obj->ignoreShunt = SVGENCURRENT_IGNORE_B;
    }
    else if(((pPWMData->value[2] + pPWMData_prev->value[2]) * 0.5f) > Vlimit)
    {
        obj->ignoreShunt = SVGENCURRENT_IGNORE_C;
    }
    else
    {
        obj->ignoreShunt = SVGENCURRENT_USE_ALL;
    }


    pPWMData_prev->value[0] = pPWMData->value[0];
    pPWMData_prev->value[1] = pPWMData->value[1];
    pPWMData_prev->value[2] = pPWMData->value[2];

    return;
} // end of SVGENCURRENT_compPWMData() function


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of SVGEN_CURRENT_H definition

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
//#############################################################################

#ifndef DATALOG_H
#define DATALOG_H

//! \file   datalog.h
//! \brief  Contains the public interface to the data logging (DATALOG) module routines
//!

// **************************************************************************
// the includes

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <datalog_input.h> //User configured macros as input

#ifndef IEEE754_TYPES
#define IEEE754_TYPES

typedef float float32_t;
typedef double float64_t;

#endif //IEEE754_TYPES

/**
* \defgroup RTLIBS_API APIs for Real Time Library
*/

/**
 *  \defgroup DATALOG_API_MODULE APIs for Data Log Library
 *  \ingroup  RTLIBS_API
 *  
 *  Here is the list of APIs used for Digital Control Library
 *  @{
 */

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

typedef enum
{
    manual,  //Manual trigger when user provides True for flag_enableLogData/flag_enableLogOneShot
    auto_trigger  // when *iptr[0] crosses the trigger value data logging will be enabled
} Trigger_type;



//! \brief Defines the default initialization for the DLOG object
//!
#define DATALOG_DEFAULTS {                          \
                            {NULL,                 \
                             NULL,                 \
                             NULL,                 \
                             NULL},               \
                            true,                  \
                            false,                 \
                            0,                     \
                            DATA_LOG_BUFF_SIZE     \
                         }

// **************************************************************************
// the typedefs

//! \brief Defines the data logging (DATALOG) object
//!
typedef struct _DATALOG_OBJ_
{

#ifdef u_int32_t
    int32_t datalogBuff[DATA_LOG_BUFF_NUM][DATA_LOG_BUFF_SIZE];
    int32_t  *iptr[DATA_LOG_BUFF_NUM];    //!< Input: First input pointer
    int32_t prev_value;
    int32_t trigValue;
#endif

#ifdef u_uint32_t
    uint32_t datalogBuff[DATA_LOG_BUFF_NUM][DATA_LOG_BUFF_SIZE];
    uint32_t  *iptr[DATA_LOG_BUFF_NUM];    //!< Input: First input pointer
    uint32_t prev_value;
    uint32_t trigValue;
#endif

#ifdef u_int64_t
    int64_t datalogBuff[DATA_LOG_BUFF_NUM][DATA_LOG_BUFF_SIZE];
    int64_t  *iptr[DATA_LOG_BUFF_NUM];    //!< Input: First input pointer
    int64_t prev_value;
    int64_t trigValue;
#endif

#ifdef u_float64_t
    float64_t datalogBuff[DATA_LOG_BUFF_NUM][DATA_LOG_BUFF_SIZE];
    float64_t  *iptr[DATA_LOG_BUFF_NUM];    //!< Input: First input pointer
    float64_t prev_value;
    float64_t trigValue;
#endif

#ifdef u_float32_t
    float32_t datalogBuff[DATA_LOG_BUFF_NUM][DATA_LOG_BUFF_SIZE];
    float32_t  *iptr[DATA_LOG_BUFF_NUM];    //!< Input: First input pointer
    float32_t prev_value;
    float32_t trigValue;
#endif

#ifdef u_int16_t
    int16_t datalogBuff[DATA_LOG_BUFF_NUM][DATA_LOG_BUFF_SIZE];
    int16_t  *iptr[DATA_LOG_BUFF_NUM];    //!< Input: First input pointer
    int16_t prev_value;
    int16_t trigValue;
#endif

#ifdef u_uint16_t
    uint16_t datalogBuff[DATA_LOG_BUFF_NUM][DATA_LOG_BUFF_SIZE];
    uint16_t  *iptr[DATA_LOG_BUFF_NUM];    //!< Input: First input pointer
    uint16_t prev_value;
    uint16_t trigValue;
#endif

#ifdef u_int8_t
    int8_t datalogBuff[DATA_LOG_BUFF_NUM][DATA_LOG_BUFF_SIZE];
    int8_t  *iptr[DATA_LOG_BUFF_NUM];    //!< Input: First input pointer
    int8_t prev_value;
    int8_t trigValue;
#endif

#ifdef u_uint8_t
    uint8_t datalogBuff[DATA_LOG_BUFF_NUM][DATA_LOG_BUFF_SIZE];
    uint8_t  *iptr[DATA_LOG_BUFF_NUM];    //!< Input: First input pointer
    uint8_t prev_value;
    uint8_t trigValue;
#endif

    int32_t  cntr;                       //!< Variable:  Data log counter
    int32_t  size;                       //!< Parameter: Maximum data buffer
    int32_t skipCount;                   //!< used for prescalar counter
    int32_t preScalar;                   //!< Parameter: How many datalog update to be skipped
    int32_t status;                      //!< Status of auto trigger type state machine
    int32_t trigger;                     //!< Trigger value used for Auto triger type
    bool  flag_enableLogData;             //!< Manual trigger usage - For datalog to be initiated always to be true 
    bool  flag_enableLogOneShot;          //!< Manual trigger usage - For datalog once this need to be true 
} DATALOG_Obj;

//! \brief Defines the DATALOG handle
//!
typedef struct _DATALOG_Obj_   *DATALOG_Handle;


//! \brief Defines the DATALOG object
//!
extern DATALOG_Obj datalog;
extern DATALOG_Handle datalogHandle;

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

//! \brief     Initializes the data logger
//! \param[in] pMemory  The pointer to memory
//! \param[in] numBytes size of datalog_obj
//! \param[in] trigger manual or auto_triiger
//! \param[in] trig_value Trigger value used for Auto triger type to initiate datalog
//! \param[in] scale How many datalog update to be skipped
extern DATALOG_Handle DATALOG_init(void *pMemory, const size_t numBytes, Trigger_type trigger, float32_t trig_value, int32_t scale);

//! \brief     Updates the data logger
//! \param[in] handle  Datalog handle need to be provided
static inline void DATALOG_update(DATALOG_Handle handle)
{
    DATALOG_Obj *obj = (DATALOG_Obj *)handle;

 if(obj->trigger == manual)
 {
    if(obj->flag_enableLogData == true)
    {
      obj->skipCount++;
      if(obj->skipCount == obj->preScalar)
      {
          obj->skipCount = 0;

          if(obj->cntr >= obj->size)
          {
              obj->cntr = 0;
              if(obj->flag_enableLogOneShot == true)
              {

                  obj->flag_enableLogData = false;
                  obj->flag_enableLogOneShot = false;
              }
          }
          else {
          for(int temp1 = 0; temp1 < DATA_LOG_BUFF_NUM; temp1++)
          {
              obj->datalogBuff[temp1][obj->cntr] = (*obj->iptr[temp1]);
          }
          }
          obj->cntr++;
       }
    }
    else if(obj->flag_enableLogOneShot == true)
    {
        obj->flag_enableLogData = true;
        obj->cntr = 0;
    }

 }

 else if (obj->trigger == auto_trigger)
 {
     switch(obj->status)
     {
         //
         // wait for trigger
         //
         case 1:
             if(*obj->iptr[0] > obj->trigValue && obj->prev_value <= obj->trigValue)
             {
                 // rising edge detected start logging data
                 obj->status=2;
             }
             break;
         case 2:
             obj->skipCount++;
             if(obj->skipCount==obj->preScalar)
             {
                 obj->skipCount=0;
                 for(int temp1 = 0; temp1 < DATA_LOG_BUFF_NUM; temp1++)
                 {
                 obj->datalogBuff[temp1][obj->cntr] = (*obj->iptr[temp1]);
                 }
                 obj->cntr++;
                 if(obj->cntr == obj->size)
                 {
                     obj->cntr = 0;
                     obj->status = 1;
                 }
             }
             break;
     }
 }

 obj-> prev_value = *obj->iptr[0];
}

#ifdef __cplusplus
}
#endif // extern "C"

/** @} */

#endif // end of DATALOG_H definition


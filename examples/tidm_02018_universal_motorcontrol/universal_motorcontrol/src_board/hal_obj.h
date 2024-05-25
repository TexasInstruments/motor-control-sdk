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


//! \file
//! \brief  Defines the structures for the HAL object
//!


#ifndef HAL_OBJ_H
#define HAL_OBJ_H


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup HAL HALL_OBJ
//! @{
//
//*****************************************************************************

// modules
#include "hal_data.h"

// platforms

// Defines for I2C
#define I2C_SLAVE_ADDRESS           0x50


//! \brief Enumeration for the GPIO for SPI_CS
//!
typedef enum
{
  SPI_CS_NSC = 0,    // Not a device connected to the SPI
  SPI_CS_DRV = 1,    // A DRV device is connecting to the SPI
  SPI_CS_DAC = 2     // A DAC device is connecting to the SPI
} SPI_CS_SEL_e;

//! \brief      Defines the HAL object
//!
typedef struct _HAL_MTR_Obj_
{
  Bool           flagEnablePWM;     //<! the pwm enable flag

  MotorNum_e     motorNum;

  uint16_t       numCurrentSensors;  //!< the number of current sensors
  uint16_t       numVoltageSensors;  //!< the number of voltage sensors

  uint32_t       pwmHandle[3];       //<! the PWM handles

#if defined(MOTOR1_DCLINKSS)
  uint32_t       cmpssHandle[1];     //!< the CMPSS handle
#else   // !(MOTOR1_DCLINKSS)
  uint32_t       cmpssHandle[3];     //!< the CMPSS handle
#endif  // !(MOTOR1_DCLINKSS)

#if defined(HVMTRPFC_REV1P1)
  uint32_t       gateEnableGPIO;
  uint32_t       gateEnableGPIOBaseAdd;
  // HVMTRPFC_REV1P1
#elif defined(BSXL3PHGAN_REVA)
  uint32_t       gateEnableGPIO;
  uint32_t       gateEnableGPIOBaseAdd;
  // BSXL3PHGAN_REVA
#else
#error Not select a right board for this project
#endif

#if defined(MOTOR1_HALL)
  uint32_t       capHandle[3];        //<! the CAP handles
#endif  // MOTOR1_HALL

  uint32_t       qepHandle;           //!< the QEP handle

} HAL_MTR_Obj;

//! \brief      Defines the HAL_MTR handle
//! \details    The HAL_MTR handle is a pointer to a HAL_MTR object.  In all
//!             HAL_MTR functions, the HAL_MTR handle is passed so that the
//!             function knows what peripherals are to be accessed.
//!
typedef struct _HAL_MTR_Obj_ *HAL_MTR_Handle;


//! \brief      Defines the hardware abstraction layer (HAL) data
//! \details    The HAL object contains all handles to peripherals.  When accessing a
//!             peripheral on a processor, use a HAL function along with the HAL handle
//!             for that processor to access its peripherals.
//!
typedef struct _HAL_Obj_
{
  uint32_t       adcHandle[5];      //!< the ADC handles
  uint32_t       adcResult[5];      //!< the ADC results

  uint32_t       timerHandle[1];    //<! the timer handle
  uint32_t       uartHandle;         //!< the UART handle
  uint32_t       linHandle;         //!< the LIN handle
  uint32_t       canHandle;         //!< the CAN handle
  uint32_t       i2cHandle;         //!< the I2C handle

  uint32_t       spiHandle[2];      //!< the SPI handle

  uint32_t       dmaHandle;         //!< the DMA handle
  uint32_t       dmaChHandle[4];    //!< the DMA Channel handle
  uint16_t       toggleGPIO[1];     //!< the toggleGPIO handle
#if defined(EPWMDAC_MODE)
#if defined(HVMTRPFC_REV1P1)
  uint32_t       pwmDACHandle[4];   //<! the PWMDAC handles
  // HVMTRPFC_REV1P1
#else
#error EPWMDAC is not supported on this kit!
#endif  // !HVMTRPFC_REV1P1
#endif  // EPWMDAC_MODE

} HAL_Obj;

//! \brief      Defines the HAL handle
//! \details    The HAL handle is a pointer to a HAL object.  In all HAL functions
//!             the HAL handle is passed so that the function knows what peripherals
//!             are to be accessed.
//!
typedef struct _HAL_Obj_ *HAL_Handle;


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#ifdef __cplusplus
}
#endif // extern "C"

#endif // end of HAL_OBJ_H definition


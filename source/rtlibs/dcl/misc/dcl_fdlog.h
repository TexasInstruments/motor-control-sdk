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
 
#ifndef _DCL_FDLOG_H_
#define _DCL_FDLOG_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_fdlog.h
 *  \brief      Defines a 32-bit floating-point data logger strcture 
 *              and related functions.
 *
 *  \details    The FDLOG structure describes a continuous memory block
 *              which is characterized by its start index pointer, 
 *              current index pointer, and size of the buffer.
 *
 *              Overfilling the data logger would result in a warp-around 
 *              in a similar fashion as circular buffers.
 *               
 *  \code       An example of initialization and use is shown below:
 *
 *              uint32_t fdlog_size = 400;
 *              #pragma DATA_SECTION(r_Array, "DataLogSection") //Not necessary but helpful for debugging
 *              float32_t r_Array[fdlog_size];
 *              DCL_FDLOG rBuf;
 *
 *              DCL_initLog(&rBuf, r_Array, fdlog_size);
 *              DCL_clearLog(&rBuf);
 *
 *              DCL_writeFDLOG(&rBuf, some_results);
 *              output = readFDLOG(&rBuf);
 *  \endcode
 */

#include "../dcl_common.h"

//! \brief          Defines the data logger strcture for 32-bit float
//!
typedef _DCL_VOLATILE struct dcl_fdlog 
{
    float32_t *fptr;    //!< Pointer to first buffer element
    float32_t *lptr;    //!< Pointer to last buffer element
    float32_t *dptr;    //!< Current data index pointer 
    uint32_t size;      //!< The size of buffer
} DCL_FDLOG, FDLOG;

/******************** macro definitions ********************/
//! \brief          Default initialization of DCL_FDLOG
//!
#define FDLOG_DEFAULTS     {0x0,0x0,0x0,0}

//! \brief          Obtain the total size of buffer
//!
//! \param[in] buf  Pointer to DCL_FDLOG
//! \return         uint32_t size    
//!
#define DCL_getLogSize(buf)         ((buf)->size)

//! \brief          Index of the current pointer (zero-indexed)
//!
//! \param[in] buf  Pointer to DCL_FDLOG
//! \return         uint32_t index   
//!
#define DCL_getLogIndex(buf)        ((uint32_t)((buf)->dptr - (buf)->fptr))

//! \brief          Remaining space left from indexed pointer to end of buffer
//!
//! \param[in] buf  Pointer to DCL_FDLOG
//! \return         int32_t remain   
//!
#define DCL_getLogRemain(buf)       ((int32_t)((buf)->lptr - (buf)->dptr))

/******************** inline functions ********************/

//! \brief          Sets index of the current pointer (zero-indexed) 
//!
//! \param[in] buf  Pointer to DCL_FDLOG
//! \param[in] idx  Index number
//!
_DCL_CODE_ACCESS
void DCL_setLogIndex(DCL_FDLOG *buf, uint32_t idx)
{
    if(idx < buf->size)
    {
        buf->dptr = buf->fptr + idx;
    } 
}    

//! \brief          Resets all structure pointers to null value
//!
//! \param[in] buf  The DCL_FDLOG structure
//!
_DCL_CODE_ACCESS
void DCL_deleteLog(DCL_FDLOG *buf)  { buf->dptr = buf->fptr = buf->lptr = NULL; buf->size = 0; }

//! \brief          Resets the data index pointer to start of buffer 
//! \param[in] buf  The DCL_FDLOG structure
//!
_DCL_CODE_ACCESS
void DCL_resetLog(DCL_FDLOG *buf)   { buf->dptr = buf->fptr; }

//! \brief          Fills the buffer with a given data value and resets the 
//!                 data index pointer to the start of the buffer
//!
//! \param[in] buf  The DCL_FDLOG structure
//! \param[in] data The fill data value
//!
_DCL_CODE_ACCESS
void DCL_fillLog(DCL_FDLOG *buf, float32_t data)
{
    uint32_t size = DCL_getLogSize(buf);
    float32_t* mem = buf->fptr;
    DCL_resetLog(buf);
    while (size--) *mem++ = data;
}

//! \brief          Clears the buffer contents by writing 0 to all elements and
//!                 resets the data index pointer to the start of the buffer.
//!
#define DCL_clearLog(buf)       DCL_fillLog(buf,0.0f)

//! \brief          Assigns the buffer pointers to a memory block or array and
//!                 sets the data index pointer to the first address
//!
//! \param[in] buf  The DCL_FDLOG structure
//! \param[in] addr The start address of the memory block
//! \param[in] size The length of the memory block in 32-bit words
//!
_DCL_CODE_ACCESS
void DCL_initLog(DCL_FDLOG *buf, float32_t *addr, uint32_t size)
{
    buf->size = size;
    buf->fptr = addr;
    buf->lptr = addr + size - 1;
    DCL_resetLog(buf);
}

//! \brief          Writes a data point into the buffer and advances the
//!                 indexing pointer, wrapping if necessary.
//!                 Returns the over-written data value for delay line or FIFO
//!                 implementation.
//!
//! \param[in] buf  The DCL_FDLOG structure
//! \param[in] data The input data value
//! \return         The over-written data value
//!
_DCL_CODE_ACCESS
float32_t DCL_writeLog(DCL_FDLOG *buf, float32_t data)
{
    // save existing data
    float32_t rv = *(buf->dptr);      

    // write new data to log
    *(buf->dptr++) = data;

    // check for end of buffer & wrap if necessary
    if (buf->dptr > buf->lptr) DCL_resetLog(buf);

    return(rv);
}

//! \brief          Reads a data point from the buffer and then advances the
//!                 indexing pointer, wrapping if necessary
//!
//! \param[in] buf  The DCL_FDLOG structure
//! \return         The indexed data value
//!
_DCL_CODE_ACCESS
float32_t DCL_readLog(DCL_FDLOG *buf)
{
    float32_t rv = *(buf->dptr++);

    // check for end of buffer & wrap if necessary
    if (buf->dptr > buf->lptr) DCL_resetLog(buf);

    return(rv);
}

//! \brief          Copies the contents of one log (src) into another (dst).  
//!                 Both logs must have the same size.
//!
//! \param[in] src  The destination DCL_FDLOG structure
//! \param[in] dst  The source DCL_FDLOG structure
//!
_DCL_CODE_ACCESS
void DCL_copyLog(DCL_FDLOG *src, DCL_FDLOG *dst)
{
    uint32_t size = DCL_getLogSize(src);
    if (size != DCL_getLogSize(dst))
    {
        return;
    }

    float32_t* src_ptr = src->fptr;
    float32_t* dst_ptr = dst->fptr;
    while (size--)  *(dst_ptr++) = *(src_ptr++);
    DCL_setLogIndex(dst, DCL_getLogIndex(src));
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_FDLOG_H_


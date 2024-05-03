/*!
 *  \file nvm_drv_eeprom.c
 *
 *  \brief
 *  EEPROM driver.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \copyright
 *  Copyright (c) 2024, KUNBUS GmbH<br /><br />
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 *  Copyright (c) 2024 KUNBUS GmbH.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer./<li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its contributors
 *  may be used to endorse or promote products derived from this software without
 *  specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *  SUCH DAMAGE.
 *
 */

#include "nvm_drv_eeprom.h"
#include "nvm.h"

#include "ti_drivers_config.h"
#include "ti_board_open_close.h"

/*!
 * \brief
 * Read data from EEPROM.
 *
 * \param[in]     id                        EEPROM ID from sysconfig.
 * \param[in]     offset                    Offset on storage device.
 * \param[in]     length                    Data length in bytes.
 * \param[in]     pData                     Data buffer.
 *
 * \return        NVM_err_t as uint32_t.
 * \retval        NVM_ERR_SUCCESS           Success.
 * \retval        NVM_ERR_INVALID           Something went wrong.
 *
 * \see NVM_DRV_EEPROM_write
 *
 * \ingroup NVM_DRV
 *
 */
uint32_t NVM_DRV_EEPROM_read(
    const uint32_t id,
    const uint32_t offset,
    const uint32_t length,
    void * const pData)
{
    uint32_t error = NVM_ERR_SUCCESS;
    /*README: Following ifdef check is added for TIDEP-01032 as I2C0 EEPROM can not be used because of pin unavailability*/
#ifdef CONFIG_EEPROM_NUM_INSTANCES
    int32_t status = SystemP_SUCCESS;
    if(id < CONFIG_EEPROM_NUM_INSTANCES)
    {
        status = EEPROM_read(gEepromHandle[id], offset, pData, length);
        if(status != SystemP_SUCCESS)
        {
            error = NVM_ERR_FAIL;
        }
    }
    else
#endif
    {
        error = NVM_ERR_REJECT;
    }

    return error;
}

/*!
 * \brief
 * Write data to EEPROM.
 *
 * \param[in]     id                        Device ID from sysconfig.
 * \param[in]     offset                    Offset on storage device.
 * \param[in]     length                    Data length in bytes.
 * \param[in]     pData                     Data buffer.
 *
 * \return        NVM_err_t as uint32_t.
 * \retval        NVM_ERR_SUCCESS           Success.
 * \retval        NVM_ERR_INVALID           Something went wrong.
 *
 * \see NVM_DRV_EEPROM_read
 *
 * \ingroup NVM_DRV
 *
 */
uint32_t NVM_DRV_EEPROM_write(
    const uint32_t id,
    const uint32_t offset,
    const uint32_t length,
    const void * const pData)
{
    uint32_t error = NVM_ERR_SUCCESS;

    /*README: Following ifdef check is added for TIDEP-01032 as I2C0 EEPROM can not be used because of pin unavailability*/
#ifdef CONFIG_EEPROM_NUM_INSTANCES
    int32_t status = SystemP_SUCCESS;
    if(id < CONFIG_EEPROM_NUM_INSTANCES)
    {
        status = EEPROM_write(gEepromHandle[id], offset, pData, length);
        if(status != SystemP_SUCCESS)
        {
            error = NVM_ERR_FAIL;
        }
    }
    else
#endif
    {
        error = NVM_ERR_REJECT;
    }
    return error;
}

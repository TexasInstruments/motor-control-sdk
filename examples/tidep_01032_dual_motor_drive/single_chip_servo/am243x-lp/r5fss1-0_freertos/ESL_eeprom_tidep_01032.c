/*!
 *  \file ESL_eeprom.c
 *
 *  \brief
 *  EEPROM load and store function.
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

#include <defines/ecSlvApiDef.h>
#include <ecSlvApi.h>
#include <ESL_eeprom.h>
#include <osal.h>
#include "ti_board_config.h"
#if !(defined FBTLPROVIDER) || (FBTLPROVIDER==0)
#include "nvm.h"
#endif


#define EEPROM_EMPTY_DATA   (0xFFFFFFFFU)

//README: Following macro is added for TIDEP-01032 as I2C0 EEPROM can not be used because of pin unavailability
//#define EEPROM_DATA_OFFSET  (0x200U)
// Offset at which ESI EEPROM data is written on Flash
#define APP_OSPI_FLASH_OFFSET_BASE  (0x200000U)

/// EEPROM header containing magic key and data length parameters
typedef struct ESL_EEP_header
{
    uint32_t    magicKey;
    uint32_t    dataSize;
} ESL_EEP_header_t;

/// EtherCAT EEPROM header containing identity data. Refer to ETG2010 Table 2
typedef struct ESL_EEP_EC_info
{
    uint16_t pdiControl;
    uint16_t pdiConfiguration;

    uint16_t syncImpulseLen;
    uint16_t pdiConfiguration2;

    uint16_t stationAlias;
    uint16_t reserved[2];
    uint16_t checksum;

    uint32_t vendorID;
    uint32_t productCode;
    uint32_t revisionNo;
    uint32_t serialNo;
}ESL_EEP_EC_info_t;

static ESL_EEP_EC_info_t ESL_EEP_identity = {0};

static bool EC_SLV_APP_EEP_verifyIntegrity(ESL_EEP_EC_info_t *pAppInfo);

/*! <!-- Description: -->
 *
 *  \brief
 *  Initialize EEProm instance
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext  application context
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ESL_eeprom.h>
 *
 *  // required variables
 *  void* pHandle;
 *
 *  // the Call
 *  EC_SLV_APP_EEP_init(pHandle);
 *  \endcode
 *
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
void EC_SLV_APP_EEP_init(void*pContext)
{
    OSALUNREF_PARM(pContext);

#if !(defined FBTLPROVIDER) || (FBTLPROVIDER==0)
    NVM_APP_init(OSAL_TASK_Prio_Normal);
#endif
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Write EEPROM to memory.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext      Application context
 *  \param[in]  pEeprom		Pointer to eeprom memory address.
 *  \param[in]  length    Eeprom length.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  void*       handle;
 *  uint16_t    pEeprom[0x400];
 *  uint32_t    length = 0x400;
 *
 *  // the Call
 *  EC_SLV_APP_EEP_write(handle, pEeprom, length);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
void EC_SLV_APP_EEP_write(void *pContext, void*pEeprom, uint32_t length)
{
    uint32_t magicKey  = (uint32_t)pContext;
    ESL_EEP_header_t* pageHead = NULL;
    bool validIdentity = false;

#if !(defined FBTLPROVIDER) || (FBTLPROVIDER==0)

    if(pEeprom != NULL && length > sizeof(ESL_EEP_EC_info_t))
    {
        //Read current EEPROM content and compare with application data
        validIdentity = EC_SLV_APP_EEP_verifyIntegrity((ESL_EEP_EC_info_t *)pEeprom);
    }
    if(!validIdentity)
    {
        //Allocate memory for header and eeprom data
        pageHead = (ESL_EEP_header_t*)OSAL_MEMORY_calloc(sizeof(ESL_EEP_header_t)+ length,
                                                          sizeof(uint8_t));
        if(pageHead != NULL)
        {
            pageHead[0].magicKey = magicKey;
            pageHead[0].dataSize = length;
            OSAL_MEMORY_memcpy((void*)&pageHead[1], pEeprom, length);


            //README: Following change is done to use FLASH instead of EEPROM for TIDEP-01032 as I2C0 EEPROM can not be used because of pin unavailability
            //If application data is different to EEPROM content, overwrite EEPROM content
            /*NVM_APP_write(NVM_TYPE_EEPROM,
                          CONFIG_EEPROM0,
                          EEPROM_DATA_OFFSET,
                          sizeof(ESL_EEP_header_t) + length,
                          pageHead);*/
            NVM_APP_write(NVM_TYPE_FLASH,
                          CONFIG_FLASH0,
                          APP_OSPI_FLASH_OFFSET_BASE,
                          sizeof(ESL_EEP_header_t) + length,
                          pageHead);
        }

        OSAL_MEMORY_free(pageHead);
    }
#endif
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Load EEPROM from memory.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext      application context
 *  \param[out] pEeprom       Pointer to eeprom memory address.
 *  \param[out] pLength       Eeprom length.
 *
 *  \return     bool            eeprom loaded correctly or not.
 *
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  uint8_t* pEeprom = NULL;;
 *  uint32_t length;
 *
 *  // the Call
 *  pEeprom = (uint8_t*)OSAL_MEMORY_calloc(length, sizeof(uint8_t));
 *  EC_SLV_APP_EEP_read(pEeprom, &length);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
bool EC_SLV_APP_EEP_read(void*pContext, void*pEeprom, uint32_t *pLength)
{
    bool ret = false;
    ESL_EEP_header_t pageProto = {0};
    uint32_t magicKey = (uint32_t)pContext;

#if !(defined FBTLPROVIDER) || (FBTLPROVIDER==0)
    uint32_t error = NVM_ERR_SUCCESS;

    OSAL_MEMORY_memset(&ESL_EEP_identity, 0, sizeof(ESL_EEP_EC_info_t));

    //README: Following change is done to use FLASH instead of EEPROM for TIDEP-01032 as I2C0 EEPROM can not be used because of pin unavailability
    //Read EEPROM header (magic key and eeprom length)
    /*error = NVM_APP_read(NVM_TYPE_EEPROM,
                 CONFIG_EEPROM0,
                 EEPROM_DATA_OFFSET,
                 sizeof(ESL_EEP_header_t),
                 &pageProto);*/
    error = NVM_APP_read(NVM_TYPE_FLASH,
                 CONFIG_FLASH0,
                 APP_OSPI_FLASH_OFFSET_BASE,
                 sizeof(ESL_EEP_header_t),
                 &pageProto);

    if(error == NVM_ERR_SUCCESS &&
        pageProto.magicKey == magicKey &&
        pageProto.dataSize != EEPROM_EMPTY_DATA)
    {
        //README: Following change is done to use FLASH instead of EEPROM for TIDEP-01032 as I2C0 EEPROM can not be used because of pin unavailability
        //Read EEPROM data (EtherCAT EEPROM content)
        /*error = NVM_APP_read(NVM_TYPE_EEPROM,
                             CONFIG_EEPROM0,
                             EEPROM_DATA_OFFSET + sizeof(ESL_EEP_header_t),
                             pageProto.dataSize,
                             pEeprom);*/
        error = NVM_APP_read(NVM_TYPE_FLASH,
                             CONFIG_FLASH0,
                             APP_OSPI_FLASH_OFFSET_BASE + sizeof(ESL_EEP_header_t),
                             pageProto.dataSize,
                             pEeprom);
        if (error == NVM_ERR_SUCCESS && pLength != NULL)
        {
            *pLength = pageProto.dataSize;

            //Copy EtherCAT info for verification
            if(pageProto.dataSize > sizeof(ESL_EEP_EC_info_t))
            {
                OSAL_MEMORY_memcpy(&ESL_EEP_identity, pEeprom, sizeof(ESL_EEP_EC_info_t));
            }
            ret = true;
        }
    }
#endif
    return ret;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Verify EEPROM Cache VendorID, Product Code, Revision Number and CRC.
 *
 *  \details
 *  This function compares the EtherCAT identity data and the CRC between the EEPROM data and
 *  the application data. If this data is the same, then the EEPROM write function does not
 *  write into the EEPROM to prevent physical wearing.

 *  \remarks
 *  The EtherCAT stack calls EEPROM read and therefore the identity data and CRC are known.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppInfo    EtherCAT EEPROM info from application.
 *  \return     bool        EEPROM Identity correct or not.
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
static bool EC_SLV_APP_EEP_verifyIntegrity(ESL_EEP_EC_info_t *pAppInfo)
{
    bool ret = false;
    if(pAppInfo != NULL)
    {
        if(pAppInfo->vendorID == ESL_EEP_identity.vendorID &&
           pAppInfo->productCode == ESL_EEP_identity.productCode &&
           pAppInfo->revisionNo == ESL_EEP_identity.revisionNo &&
           pAppInfo->checksum == ESL_EEP_identity.checksum)
        {
            ret = true;
        }
    }
    return ret;
}

//*************************************************************************************************

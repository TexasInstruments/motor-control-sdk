/*
 *  Copyright (C) 2023-2026 Texas Instruments Incorporated
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


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <position_sense/tamagawa_over_soc_uart/include/tamagawa_soc_uart_interface.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief Maximum transmit frame size in bytes
 */
#define MAX_TX_FRAME (4)

/**
 * \brief Maximum receive frame size in bytes
 */
#define MAX_RX_FRAME (12)

/**
 * \brief Assert macro for UART transfer error checking
 *
 * Validates UART transfer completion status and asserts on failure.
 * Used to catch UART communication errors during development and debug.
 *
 * \param transferOK    Return status from UART_lld_write() or UART_lld_read()
 * \param transaction   UART_Transaction structure containing transfer status
 */
#define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
        { \
            DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
        } \
    } while(0) \

/* ========================================================================== */
/*                       Internal Function Declarations                       */
/* ========================================================================== */

static int32_t tamagawa_handle_rx(volatile struct tamagawa_uart_interface *tamagawa_interface, uint8_t *p);
static int32_t tamagawa_command_build(volatile struct tamagawa_uart_interface *tamagawa_interface, uint8_t *tx, uint8_t *tx_size, uint8_t *rx_size);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * \brief Calculate Tamagawa protocol CRC
 *
 * \param s     Pointer to input data buffer to calculate CRC over
 * \param len   Number of bytes in input data buffer
 *
 * \return      Calculated 8-bit CRC value
 *
 * \note A local copy of input data is made to avoid modifying original buffer
 */
uint8_t tamagawa_calculate_crc(uint8_t *s, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t val;
    uint32_t  i, j;
    uint8_t data[MAX_RX_FRAME];

    memcpy (data, s, len);

    for(i = 0; i < len; i++)
    {
        for(j = 0; j < 8; j++)
        {
            val = (data[i] >> 7) ^ (crc >> 7);

            crc <<= 1;
            data[i] <<= 1;
            crc |= val;
        }
    }
    return crc;
}

/**
 * \brief Parse received Tamagawa response data
 *
 * \details Extracts and populates encoder response data from raw UART receive buffer
 *          into the tamagawa_rx structure based on the current Data ID.

 * \param tamagawa_interface    Pointer to Tamagawa interface structure.
 *                              INPUT: data_id field specifies which command was sent
 *                              OUTPUT: rx structure populated with parsed data
 * \param p                     Pointer to raw receive buffer containing encoder response
 *                              (without TX echo bytes)
 *
 * \retval SystemP_SUCCESS      Data parsed successfully
 * \retval SystemP_FAILURE      Invalid Data ID (parsing failed)
 *
 * \note Caller must ensure buffer pointer 'p' points to valid response data
 * \note CRC byte is extracted but not validated by this function
 */
int32_t tamagawa_handle_rx(volatile struct tamagawa_uart_interface *tamagawa_interface, uint8_t *p)
{
    int32_t ret = SystemP_SUCCESS;
    switch(tamagawa_interface->data_id)
    {
        case DATA_ID_0:
        case DATA_ID_7:
        case DATA_ID_8:
        case DATA_ID_C:
            tamagawa_interface->rx.cf = p[0];
            tamagawa_interface->rx.sf = p[1];
            tamagawa_interface->rx.abs = p[2] | (p[3] << 8) | (p[4] << 16);
            tamagawa_interface->rx.crc = p[5];
            break;

        case DATA_ID_1:
            tamagawa_interface->rx.cf = p[0];
            tamagawa_interface->rx.sf = p[1];
            tamagawa_interface->rx.abm = p[2] | (p[3] << 8) | (p[4] << 16);
            tamagawa_interface->rx.crc = p[5];
            break;

        case DATA_ID_2:
            tamagawa_interface->rx.cf = p[0];
            tamagawa_interface->rx.sf = p[1];
            tamagawa_interface->rx.enid = p[2];
            tamagawa_interface->rx.crc = p[3];
            break;

        case DATA_ID_3:
            tamagawa_interface->rx.cf = p[0];
            tamagawa_interface->rx.sf = p[1];
            tamagawa_interface->rx.abs = p[2] | (p[3] << 8) | (p[4] << 16);
            tamagawa_interface->rx.enid = p[5];
            tamagawa_interface->rx.abm = p[6] | (p[7] << 8) | (p[8] << 16);
            tamagawa_interface->rx.almc = p[9];
            tamagawa_interface->rx.crc = p[10];
            break;

        case DATA_ID_6:
        case DATA_ID_D:
            tamagawa_interface->rx.cf = p[0];
            tamagawa_interface->rx.adf = p[1];
            tamagawa_interface->rx.edf = p[2];
            tamagawa_interface->rx.crc = p[3];
            break;

        default:
            ret = SystemP_FAILURE;
            break;
    }

    return ret;
}

/**
 * \brief Build Tamagawa command frame for transmission
 *
 * \details Constructs the appropriate command frame based on the Data ID
 *          specified in tamagawa_interface->data_id. Each Data ID has a specific
 *          command code and frame format.
 *
 * \param tamagawa_interface    Pointer to Tamagawa interface structure.
 *                              INPUT: data_id specifies which command to build
 *                              INPUT: tx.adf and tx.edf for EEPROM operations
 * \param tx                    Pointer to transmit buffer to populate with command frame.
 *                              Must be at least MAX_TX_FRAME (4) bytes.
 * \param tx_size               Pointer to variable to store transmit frame size in bytes
 * \param rx_size               Pointer to variable to store expected receive frame size
 *
 * \retval SystemP_SUCCESS      Command frame built successfully
 * \retval SystemP_FAILURE      Invalid Data ID (unsupported command)
 *
 * \note For EEPROM operations, ensure tx.adf and tx.edf are set before calling
 * \note The function automatically calculates and appends CRC for applicable commands
 * \note tx_size and rx_size are set according to Data ID requirements
 */
int32_t tamagawa_command_build(volatile struct tamagawa_uart_interface *tamagawa_interface, uint8_t *tx, uint8_t *tx_size, uint8_t *rx_size)
{
    int32_t ret = SystemP_SUCCESS;

    switch(tamagawa_interface->data_id)
    {
        case DATA_ID_0:
            *tx = 0x02;
            *tx_size = 1;
            *rx_size = 6;
            break;

        case DATA_ID_1:
            *tx = 0x8A;
            *tx_size = 1;
            *rx_size = 6;
            break;

        case DATA_ID_2:
            *tx = 0x92;
            *tx_size = 1;
            *rx_size = 4;
            break;

        case DATA_ID_3:
            *tx = 0x1A;
            *tx_size = 1;
            *rx_size = 11;
            break;

        case DATA_ID_7:
            *tx = 0xBA;
            *tx_size = 1;
            *rx_size = 6;
            break;

        case DATA_ID_8:
            *tx = 0xC2;
            *tx_size = 1;
            *rx_size = 6;
            break;

        case DATA_ID_C:
            *tx = 0x62;
            *tx_size = 1;
            *rx_size = 6;
            break;

        case DATA_ID_6:
            *tx++ = 0x32;
            *tx++ = tamagawa_interface->tx.adf;
            *tx++ = tamagawa_interface->tx.edf;
            *tx_size = 4;
            *rx_size = 4;
            *tx = tamagawa_calculate_crc(tx - 3, *tx_size - 1);
            break;

        case DATA_ID_D:
            *tx++ = 0xEA;
            *tx++ = tamagawa_interface->tx.adf;
            *tx_size = 3;
            *rx_size = 4;
            *tx = tamagawa_calculate_crc(tx - 2, *tx_size - 1);
            break;

        default:
            ret = SystemP_FAILURE;
            break;
    }

    return ret;
}

/* ========================================================================== */
/*                      Public Function Implementations                       */
/* ========================================================================== */

int32_t tamagawa_command_process(volatile struct tamagawa_uart_interface *tamagawa_interface, UARTLLD_Handle *gUartHandle, int32_t cmd)
{
    int32_t          transferOK;
    UART_Transaction trans;
    uint8_t          tx[MAX_TX_FRAME];
    uint8_t          rx[MAX_RX_FRAME];
    uint8_t          tx_size = 0, rx_size = 0;

    /* Initialize UART transaction structure */
    UART_lld_Transaction_init(&trans);
    tamagawa_interface->data_id = cmd;

    /* Build command frame based on Data ID */
    cmd = tamagawa_command_build(tamagawa_interface, tx, &tx_size, &rx_size);
    if(cmd != SystemP_SUCCESS)
    {
        return cmd;
    }

    /* Transmit command to encoder via UART */
    trans.buf   = &tx[0U];
    trans.count = tx_size;

    /* Set RTSn (HIGH) before TX start */
    GPIO_pinWriteHigh(tamagawa_interface->gpio_base_address, tamagawa_interface->gpio_pin_number);

    /* Transmit command bytes */
    transferOK = UART_lld_write(gUartHandle[tamagawa_interface->uart_instance], trans.buf, trans.count, trans.timeout, NULL);

    /* Clear RTSn (LOW) after TX completion */
    GPIO_pinWriteLow(tamagawa_interface->gpio_base_address, tamagawa_interface->gpio_pin_number);

    /* Check TX status and assert on failure */
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Receive response from encoder via UART */
    trans.buf   = &rx[0U];
    trans.count = rx_size + tx_size;
    transferOK = UART_lld_read(gUartHandle[tamagawa_interface->uart_instance], trans.buf, trans.count, trans.timeout, NULL);

    /* Check RX status and assert on failure */
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Parse received encoder response (skip TX echo bytes) */
    cmd = tamagawa_handle_rx(tamagawa_interface, rx + tx_size);

    /* Calculate CRC on received data (excluding CRC byte itself) */
    tamagawa_interface->rx_crc = tamagawa_calculate_crc(rx + tx_size, rx_size - 1);

    return cmd;
}

int32_t tamagawa_crc_verify(volatile struct tamagawa_uart_interface *tamagawa_interface)
{
    /* Compare received CRC (from encoder) with calculated CRC (by driver) */
    return (tamagawa_interface->rx.crc == tamagawa_interface->rx_crc) ? 1 : 0;
}

int32_t tamagawa_init(volatile struct tamagawa_uart_interface *tamagawa_interface, uint32_t instance , uint32_t base_address, uint32_t pin_number, uint32_t pin_direction)
{
    /* Validate input parameter */
    if(tamagawa_interface == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Store UART instance number for later use in command processing */
    tamagawa_interface->uart_instance = instance;

    /* Store GPIO configuration for RTSn flow control */
    tamagawa_interface->gpio_base_address = base_address;
    tamagawa_interface->gpio_pin_number = pin_number;

    /* Configure GPIO pin direction for RTSn control */
    GPIO_setDirMode(base_address, pin_number, pin_direction);

    return SystemP_SUCCESS;
}
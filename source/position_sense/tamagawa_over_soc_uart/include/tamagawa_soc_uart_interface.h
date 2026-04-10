/*
 * Copyright (C) 2023-2026 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TAMAGAWA_UART_DRV_H_
#define TAMAGAWA_UART_DRV_H_
 /**
 * \defgroup POSITION_SENSE_API APIs for Position Sense
 *
 * This module contains APIs for device drivers for position sense encoders supported in this SDK.
 */

/**
 *  \defgroup TAMAGAWA_UART_API_MODULE APIs for Tamagawa Encoder Over UART
 *  \ingroup POSITION_SENSE_API
 *
 *  The Tamagawa UART API module provides functions for communicating with
 *  Tamagawa absolute encoders over standard UART interface. This driver supports
 *  various Tamagawa data IDs for reading position data, encoder ID, EEPROM access,
 *  and encoder reset commands.
 *
 *
 *  \section tamagawa_uart_usage Typical Usage Flow
 *
 *  **1. Initialization:**
 *  - Open and configure UART peripheral using UART driver
 *  - Call \ref tamagawa_init() to initialize interface structure and configure GPIO for RTSn
 *
 *  **2. Command Execution:**
 *  - For EEPROM operations, set tx.adf (address) and tx.edf (data) fields
 *  - Call \ref tamagawa_command_process() to execute command desired data_id
 *  - Check return value for success/failure
 *
 *  **3. Data Validation:**
 *  - Call \ref tamagawa_crc_verify() to validate received data CRC
 *  - If CRC passes, read data from rx structure fields
 *
 *  **4. Data Retrieval:**
 *  - Access data via rx structure of tamagawa_uart_interface
 *
 *  @{
 */
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief  Maximum EEPROM address for read/write operations
 *
 *  The Tamagawa encoder EEPROM supports addresses in the range 0-127.
 *  Valid address range: 0 to \ref MAX_EEPROM_ADDRESS (inclusive)
 */
#define MAX_EEPROM_ADDRESS (127)

/**
 *  \brief Maximum data value that can be written to EEPROM
 *
 *  Each EEPROM location stores an 8-bit value (0-255).
 *  Valid data range: 0 to \ref MAX_EEPROM_WRITE_DATA (inclusive)
 */
#define MAX_EEPROM_WRITE_DATA (255)

/**
 *  \brief Tamagawa encoder Data ID codes
 *
 *  Data IDs specify which command/operation to execute on the encoder.
 *  Different Data IDs return different combinations of position data,
 *  encoder status, and configuration information.
 */
enum data_id
{
    DATA_ID_0,  /**< Data readout data in one revolution */
    DATA_ID_1,  /**< Data readout multi-turn data */
    DATA_ID_2,  /**< Data readout encoder ID */
    DATA_ID_3,  /**< Data readout data in one revolution, encoder ID, multi-turn, encoder error */
    DATA_ID_6,  /**< EEPROM write */
    DATA_ID_7,  /**< Reset */
    DATA_ID_8,  /**< Reset */
    DATA_ID_C,  /**< Reset */
    DATA_ID_D,  /**< EEPROM read */
    DATA_ID_NUM /**< Number of Data ID codes */
};
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/**
 * \brief Tamagawa over SoC UART transmit data structure
 *
 * Contains data to be transmitted to the encoder for EEPROM read/write operations.
 * These fields are only used with DATA_ID_6 (EEPROM write) and DATA_ID_D (EEPROM read).
 */
struct tamagawa_tx
{
    uint8_t  adf;   /**< Address field: EEPROM address to read from or write to (0-127, see \ref MAX_EEPROM_ADDRESS) */
    uint8_t  edf;   /**< Encoder data field: Data value to write to EEPROM (0-255, see \ref MAX_EEPROM_WRITE_DATA). Not used for EEPROM read. */
};

/**
 * \brief Tamagawa over SoC UART receive data structure
 *
 * Contains all possible data fields that can be received from the encoder.
 * Fields are populated based on the Data ID used in the command.
 * It is recommended to verify the CRC field using \ref tamagawa_crc_verify() before using received data.
 */
struct tamagawa_rx
{
    uint32_t abs;   /**< Data in one revolution */
    uint32_t abm;   /**< Multi-turn Data */
    uint8_t  cf;    /**< Control Frame */
    uint8_t  sf;    /**< Status Frame */
    uint8_t  enid;  /**< Encoder ID */
    uint8_t  almc;  /**< Encoder error */
    uint8_t  adf;   /**< EEPROM address */
    uint8_t  edf;   /**< EEPROM data */
    uint8_t  crc;   /**< CRC */
};
/**
 * \brief Tamagawa over SoC UART interface structure
 *
 * Main interface structure for Tamagawa encoder communication over UART.
 * This structure maintains all state and configuration needed for encoder operations.
 *
 */
struct tamagawa_uart_interface
{
    uint8_t             data_id;            /**< Data ID code */
    struct tamagawa_tx  tx;                 /**< Transmit data */
    struct tamagawa_rx  rx;                 /**< Received data */
    uint8_t             rx_crc;             /**< Calculated CRC */
    uint32_t            uart_instance;      /**< Uart instance*/
    uint32_t            gpio_base_address;  /**< GPIO pin base address*/
    uint32_t            gpio_pin_number;    /**<GPIO Pin number*/
};

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */
/**
 *  \brief Process and execute a Tamagawa encoder command
 *
 *  \details This function executes a complete Tamagawa command transaction:
 *           1. Builds the command frame based on the specified Data ID
 *           2. Controls RTSn GPIO pin (HIGH) to enable encoder listening mode
 *           3. Transmits command via UART
 *           4. Controls RTSn GPIO pin (LOW) to disable transmit and enable receive
 *           5. Receives response from encoder via UART
 *           6. Parses received data into tamagawa_interface->rx structure
 *           7. Calculates CRC on received data and stores in tamagawa_interface->rx_crc
 *
 *           After this function returns successfully, it is recommended to call
 *           \ref tamagawa_crc_verify() to validate the received data integrity before
 *           using rx structure fields.
 *
 *  \param[in,out]  tamagawa_interface  Pointer to Tamagawa interface structure.
 *                                      - INPUT: data_id field must be set to desired command
 *                                      - INPUT: For DATA_ID_6 or DATA_ID_D, tx.adf and tx.edf must be populated
 *                                      - OUTPUT: rx structure populated with encoder response
 *                                      - OUTPUT: rx_crc calculated from received data
 *  \param[in]      gUartHandle         Array of UART LLD handles. The handle at index
 *                                      tamagawa_interface->uart_instance will be used.
 *  \param[in]      cmd                 Data ID command code from \ref data_id enum
 *                                      (DATA_ID_0, DATA_ID_1, etc.). This parameter
 *                                      updates tamagawa_interface->data_id internally.
 *
 *  \retval         SystemP_SUCCESS     Command executed successfully. Data received and parsed.
 *                                      Call \ref tamagawa_crc_verify() before using rx data.
 *  \retval         SystemP_FAILURE     Command failed due to:
 *                                      - Invalid Data ID
 *                                      - Response parsing error
 *
 *  \note The function asserts (DebugP_assert) on UART transfer failures
 *  \note RTSn GPIO pin must be properly configured before calling this API
 *  \note UART peripheral must be opened and configured before calling this function
 *  \note This function uses UART_lld_write() and UART_lld_read() for communication
 *
 */
int32_t tamagawa_command_process(volatile struct tamagawa_uart_interface *tamagawa_interface, UARTLLD_Handle *gUartHandle, int32_t cmd);

/**
 *  \brief Verify CRC integrity of received encoder data
 *
 *  \param[in]  tamagawa_interface  Pointer to Tamagawa interface structure
 *
 *  \retval     1   CRC verification passed. Received data is valid and can be used safely.
 *  \retval     0   CRC verification failed. Received data is corrupted and should not be used.
 *
 */
int32_t tamagawa_crc_verify(volatile struct tamagawa_uart_interface *tamagawa_interface);

/**
 *  \brief Initialize Tamagawa UART interface and configure RTSn GPIO pin
 *
 *  \details Initializes the Tamagawa interface structure and configures the GPIO pin
 *           used for software flow control (RTSn signal). This function must be called
 *           before using any other Tamagawa driver functions.
 *
 *           The RTSn GPIO pin is used to control half-duplex communication.
 *
 *  \param[in,out]  tamagawa_interface  Pointer to Tamagawa interface structure to initialize.
 *                                      Must be allocated by caller. On success, all fields
 *                                      are initialized with provided parameters.
 *  \param[in]      instance            UART peripheral instance number. This value is used to index
 *                                      into the UART handle array passed to \ref tamagawa_command_process().
 *  \param[in]      base_address        GPIO peripheral base address containing the RTSn pin.
 *  \param[in]      pin_number          GPIO pin number for RTSn control signal within the
 *                                      GPIO port specified by base_address.
 *  \param[in]      pin_direction       GPIO pin direction configuration. Use GPIO driver
 *                                      defines: GPIO_DIRECTION_OUTPUT for RTSn control pin.
 *
 *  \retval         SystemP_SUCCESS     Initialization successful.
 *  \retval         SystemP_FAILURE     Initialization failed due to NULL tamagawa_interface pointer.
 *
 *  \note The GPIO pin is configured as specified direction using GPIO_setDirMode()
 *  \note Typically pin_direction should be GPIO_DIRECTION_OUTPUT for RTSn control
 *  \note UART peripheral must still be opened separately using UART driver APIs
 *  \note The GPIO pin initial state is not set by this function
 *
 */
int32_t tamagawa_init(volatile struct tamagawa_uart_interface *tamagawa_interface, uint32_t instance , uint32_t base_address, uint32_t pin_number, uint32_t pin_direction);

/** @} */

#ifdef __cplusplus
}
#endif

#endif

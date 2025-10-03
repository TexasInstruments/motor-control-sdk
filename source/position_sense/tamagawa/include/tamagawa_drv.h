/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#ifndef TAMAGAWA_DRV_H_
#define TAMAGAWA_DRV_H_

/**
 *  \defgroup TAMAGAWA_API_MODULE APIs for Tamagawa Encoder
 *  \ingroup POSITION_SENSE_API
 *
 * Here is the list of APIs used for Tamagawa encoder communication protocol
 *
 *  @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <drivers/pruicss.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief 3 channel Peripheral clock Source 
 */
#define PRU_UART_CLOCK_SOURCE  (0)
#define PRU_CORE_CLOCK_SOURCE  (1)

/**
 *  \brief  Used to set the value of Tamagawa multi-channel mask based on the whether the Channel 0 is selected or not
 */
#define TAMAGAWA_MULTI_CH0 (1 << 0)

/**
 *  \brief  Used to set the value of Tamagawa multi-channel mask based on the whether the Channel 1 is selected or not
 */
#define TAMAGAWA_MULTI_CH1 (1 << 1)

/**
 *  \brief  Used to set the value of Tamagawa multi-channel mask based on the whether the Channel 2 is selected or not
 */
#define TAMAGAWA_MULTI_CH2 (1 << 2)

/**
 *  \brief  Used to set the maximum channels supported
 */
#define MAX_CHANNELS (3)

/**
 *  \brief  Used to set the maximum address that can be used for EEPROM Read/Write
 */
#define MAX_EEPROM_ADDRESS (127)

/**
 *  \brief  Used to set the maximum value that can be written in EEPROM
 */
#define MAX_EEPROM_WRITE_DATA (255)

/**
 * 
 * \brief  Used to set the Rx oversampling rate
 * 
*/
#define TAMAGAWA_RX_OVERSAMPLING_RATE    (7)

/**    \brief    delay counter increment value */ 
#define  TAMAGAWA_DELAY_COUNTER_INCREMENT  5



/**
*    \brief    Data ID codes
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
    PERIODIC_TRIGGER_CMD, /**< periodic trigger command */
    DATA_ID_NUM /**< Number of Data ID codes */
};

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *    \brief    Structure defining tamagawa per channel interface
 *
 *    \details  Firmware per channel interface
 */
typedef struct Tamagawa_ChInfo_s
{
    volatile uint32_t    pos_word0;          /**<word0 for receiving Rx data  */

    volatile uint32_t   pos_word1;          /**<word1 for receiving Rx data  */

    volatile uint32_t   pos_word2;          /**<word2 for receiving Rx data  */

    volatile uint32_t  cal_crc;             /**<word for receiving the CRC  */

} Tamagawa_ChInfo;

/**
 *    \brief    Structure defining Tamagawa command interface
 *
 *    \details  Firmware command interface
 */
typedef struct Tamagawa_Cmd_s
{
    volatile uint32_t   word0;/**< command,                                                         <br>
                                    [Byte 0] control field <br> **/
    volatile uint32_t   word1;/**< command parameters,                                              <br>
                                    [Byte 0] No. of Tx frames                                        <br>
                                    [Byte 1] No. of Rx frames                                        <br>

                                    */

} Tamagawa_Cmd;

/**
 *    \brief    Structure defining Tamagawa configuration interface
 *
 *    \details  Firmware configuration interface
 */
typedef struct Tamagawa_FwConfig_s
{
    volatile uint8_t  opmode;/**< operation mode selection: 0 - periodic trigger, 1 - host trigger */
    volatile uint8_t  channel;/**< channel mask (1 << channel), 0 < channel < 3. This has to be      <br>
                                        selected before running firmware. Once initialization is complete,<br>
                                        it will reflect the detected channels in the selected mask.       <br>
                                        Multichannel can have upto 3 selected, while single channel only one */
    volatile uint8_t  trigger;/**< command trigger. Set LSB to send cmd, will be cleared upon cmd    <br>
                                        completion. Note that cmd has to be setup before trigger */
    volatile uint8_t  status;/**< initialization status: 1 - upon successful.  */
} Tamagawa_FwConfig;

/**
 * \brief Tamagawa Interface Received data
 **/
typedef struct Tamagawa_RxFrames_s
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
} Tamagawa_RxFrames;
/**
 * \brief Tamagawa Interface
 */
typedef struct Tamagawa_Interface_s
{
    uint8_t ch_mask;   //**< Mask for what channel is required*/
    volatile uint32_t  rx_div_factor;   //**< Rx Divide factor*/

    volatile uint32_t  tx_div_factor;   //**< Tx Divide factor*/

    volatile uint32_t  oversample_rate; //**< Oversampling rate*/

    uint32_t version;  /**< Firmware version */
    uint8_t  data_id;  /**< Data ID code */
    Tamagawa_RxFrames rx_frames_received;      /**< Received data */
    uint8_t tx_frames;  /**< Number of Tx frames */
    uint8_t rx_frames;  /**< Number of Rx frames */
} Tamagawa_Interface;

typedef struct Tamagawa_ChannelConfig_s
{
    uint8_t  ch0;   /**< config for channel 0 */
    uint8_t  ch1;   /**< config for channel 1 */
    uint8_t  ch2;   /**< config for channel 2 */
} Tamagawa_ChannelConfig;

/**
 * \brief Tamagawa EEPROM Interface
 */
typedef struct Tamagawa_EepromInterface_s
{
    volatile uint32_t cmd; /**< holds the value of command id for EEPROM commands */
    volatile uint32_t adf; /**< holds the value of ADF for EEPROM commands */
    volatile uint32_t edf; /**< holds the value of EDF for EEPROM Write command */
    volatile uint32_t crc; /**< holds the value of CRC for EEPROM commands */

    volatile uint32_t word0; /**< used for CRC calculation */
    volatile uint32_t word1; /**< used for CRC calculation */
    volatile uint32_t word2; /**< used for CRC calculation */

    uint64_t eeprom_tx_data; /**< used to store the bits for tx in eeprom read/write */
} Tamagawa_EepromInterface;


/**
 *    \brief    Structure defining Tamagawa interface
 *
 *    \details  Firmware config, command and channel interface
 *
 */
typedef struct Tamagawa_Xchg_s
{
    Tamagawa_FwConfig   config;/**< config interface */
    Tamagawa_Cmd      cmd;/**< command interface */
    Tamagawa_ChInfo  ch[3];/**< per channel interface */

    Tamagawa_Interface tamagawa_interface;/**< tamagawa interface */
    Tamagawa_EepromInterface tamagawa_eeprom_interface[3];/**< tamagawa interface for EEPROM commands */
} Tamagawa_Xchg;

/**
 * \brief   Used to configure the Tamagawa Clock.
 *
 */
typedef struct Tamagawa_ClkCfg_s
{
    uint16_t  rx_div;   /**< Rx Div factor*/
    uint16_t  tx_div;   /**< Tx Div factor*/
    uint16_t  rx_os_rate; /*rx oversample rate*/
    uint8_t   rx_clk_source; /*rx clock source*/
    uint8_t   tx_clk_source; /*tx clock source*/
    uint16_t  rx_en_cnt; /*rx enable counter*/
} Tamagawa_ClkCfg;


/**
 * \brief   Used to store the register offsets depending on different PRU slices.
 *
 */
typedef struct Tamagawa_RegisterOffsets_s
{
    int32_t ICSS_CFG_PRUx_ED_CH0_CFG0;
    int32_t ICSS_CFG_PRUx_ED_CH1_CFG0;
    int32_t ICSS_CFG_PRUx_ED_CH2_CFG0;
    int32_t ICSS_CFG_PRUx_ED_CH0_CFG1;
    int32_t ICSS_CFG_PRUx_ED_CH1_CFG1;
    int32_t ICSS_CFG_PRUx_ED_CH2_CFG1;
    int32_t ICSS_CFG_GPCFGx;
    int32_t ICSS_CFG_PRUx_ED_RXCFG;
    int32_t ICSS_CFG_PRUx_ED_TXCFG;
} Tamagawa_RegisterOffsets;

/**
 *    \brief    Structure defining Tamagawa PRU configuration
 *
 *    \details  Contains configuration parameters for PRU including ID, clock settings, and load sharing
 */
typedef struct Tamagawa_PruConfig_s
{
    volatile uint8_t      pru_slice;           /**< PRU Slice */
    PRUICSS_Handle        pruicss_handle;      /**< PRU ICSS Handle */
    volatile uint32_t     pru_clock;           /**< PRU core clock frequency in Hz */
    volatile uint8_t      load_share_enable;    /**< Enable load sharing between PRUs */
    volatile uint8_t      iep_cmp_event;              /**< IEP CMP event */
    volatile uint8_t      iep_instance;        /**< IEP Instance (0 for IEP0, 1 for IEP1) */
    volatile uint32_t     iep_clock;           /**< PRU iep clock frequency in Hz */
    volatile uint32_t     uart_clock;          /**< PRU UART clock frequency */
} Tamagawa_PruConfig;
/**
 * \brief   Used to structures defining the Tamagawa interface, PRU slice and register offsets.
 *
 */
typedef struct Tamagawa_Config_s
{
    uint8_t instance_index; /**< Holds the index of the current Tamagawa instance */
    uint8_t channel;    /**< Holds the ID of the current channel being used*/
    Tamagawa_PruConfig pru_cfg;    /**< Structure defining Tamagawa PRU configuration*/
    Tamagawa_Xchg *tamagawa_xchg;    /**<Structure defining Tamagawa interface*/
    Tamagawa_RegisterOffsets register_offset_val;    /**< Register offset values based on PRUx slice selection*/
    Tamagawa_ClkCfg clk_cfg;  /**< Tamagawa clock configuration */
}Tamagawa_Config;

typedef Tamagawa_Config *Tamagawa_Handle;

/**
 *    \brief    Structure defining TAMAGAWA initialization parameters.
 *
 */
typedef struct Tamagawa_Params_s
{ 
    Tamagawa_PruConfig pru_cfg;    /**< Structure defining Tamagawa PRU configuration*/
    Tamagawa_ClkCfg clk_cfg;  /**< Tamagawa clock configuration */
}Tamagawa_Params;
/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief      Initialize tamagawa firmware interface address and configure the provided
 *              tamagawa_handle instance
 *
 *  \param[in]  index            Index of tamagawa handle to use in the gTamagawaHandles array
 *  \param[in]  tamagawa_params  Structure containing Tamagawa parameters (firmware interface address, 
 *                              PRU config base, IEP base, slice value)
 *
 *  \retval     handle           Pointer to initialized TAMAGAWA_Handle_s instance
 *
 */
Tamagawa_Handle tamagawa_init(uint32_t index, Tamagawa_Params tamagawa_params);
/**
 *  \brief      send the tamagawa command and wait till firmware acknowledges
 *
 *  \param[in]  handle            cookie returned by tamagawa_init
 *  \param[in]  cmd             tamagawa command number
 *  \param[in]  gTamagawa_multi_ch_mask  Multi-channel mask to keep track of which channels are selected
 *
 *  \retval     0       success
 *  \retval     -EINVAL failure
 *
 */
int32_t tamagawa_command_process(Tamagawa_Handle handle, int32_t cmd, uint8_t gTamagawa_multi_ch_mask);

/**
 *  \brief      setup the tamagawa command in the PRU interface buffer
 *
 *  \param[in]  handle            cookie returned by tamagawa_init
 *  \param[in]  cmd             tamagawa command number
 *  \param[in]  gTamagawa_multi_ch_mask Multi-channel mask to keep track of which channels are selected
 *
 *  \retval     0       success
 *  \retval     -EINVAL failure
 *
 */
int32_t tamagawa_command_build(Tamagawa_Handle handle, int32_t cmd,  uint8_t gTamagawa_multi_ch_mask);

/**
 *  \brief      trigger sending the tamagawa command in PRU
 *
 *  \param[in]  handle     cookie returned by tamagawa_init
 *
 */
void tamagawa_command_send(Tamagawa_Handle handle);

/**
 *  \brief  wait till PRU finishes tamagawa transaction
 *
 *  \param[in]  handle     cookie returned by tamagawa_init
 *
 */
void tamagawa_command_wait(Tamagawa_Handle handle);

/**
 *  \brief  configure tamagawa clock
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *  \param[in]  rx_en_cnt value to be set in global RX auto arm counter register
 *
 */
void tamagawa_config_global_rx_arm_cnt(Tamagawa_Handle handle,  uint16_t  rx_en_cnt);

/**
 *  \brief  configure tamagawa clock
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *  \param[in]  clk_cfg pointer to structure containing clock configuration data
 *
 */
void tamagawa_config_clock(Tamagawa_Handle handle, Tamagawa_ClkCfg *clk_cfg);

/**
 *  \brief      configure tamagawa master for host trigger mode
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *
 */
void tamagawa_config_host_trigger(Tamagawa_Handle handle);

/**
 *  \brief      configure tamagawa master in periodic trigger mode
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *
 */
void tamagawa_config_periodic_trigger(Tamagawa_Handle handle);

/**
 *  \brief      select channel to be used by tamagawa master
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *  \param[in]  ch      channel to be selected
 *
 */
void tamagawa_config_channel(Tamagawa_Handle handle, uint32_t ch);

/**
 *  \brief      select mask of channels to be used in multi channel configuration by tamagawa master
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *  \param[in]  mask    channel mask
 *
 */
void tamagawa_config_multi_channel_mask(Tamagawa_Handle handle, uint8_t mask);

/**
 *  \brief      select channels detected in multi channel configuration by tamagawa master.    <br>
 *              required to be invoked only if firmware indicates initialization failure    <br>
 *              to know the channels that has been detected. Initialization success implies <br>
 *              that all channels indicated has been detected.
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *
 *  \retval     mask    mask of the detected channels
 *
 */
uint8_t tamagawa_multi_channel_detected(Tamagawa_Handle handle);

/**
 *  \brief      In multi channel configuration, select channel before receive processing in <br>
 *              multi channel configuration. After receive is complete, select each channel <br>
 *              and invoke rx API's to parse data recieved in each channel.
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *  \param[in]  ch      channel number to be selected
 *
 */
void tamagawa_multi_channel_set_cur(Tamagawa_Handle handle, uint32_t ch);

/**
 *  \brief      update the current requested command id in tamagawa interface.    <br>
 *
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *  \param[in]  cmd     tamagawa command number
 *
 */

void tamagawa_update_data_id(Tamagawa_Handle handle, int32_t cmd);

/**
 *  \brief      update the adf(address of EEPROM) field entered by user for EEPROM command in tamagawa interface.<br>
 *
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *  \param[in]  val     ADF value to be updated
 *  \param[in]  ch      channel number that is currently selected
 *
 */

void tamagawa_update_adf(Tamagawa_Handle handle, uint32_t val, uint32_t ch);

/**
 *  \brief      update the edf(data for EEPROM) field entered by user for EEPROM command in tamagawa interface.<br>
 *
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *  \param[in]  val     EDF value to be updated
 *  \param[in]  ch      channel number that is currently selected
 *
 */

void tamagawa_update_edf(Tamagawa_Handle handle, uint32_t val, uint32_t ch);


/**
 *  \brief      Parse the data in tamagawa interface.<br>
 *
 *
 *  \param[in]  cmd     tamagawa command number
 *  \param[in]  handle    cookie returned by tamagawa_init
 *
 */
int32_t tamagawa_parse(int32_t cmd, Tamagawa_Handle handle);

/**
 *  \brief      verify the CRC computed with the encoder crc.<br>
 *
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *
 *  \retval     1/0     if verify correctly, return 1 else return 0.
 *
 */

int32_t tamagawa_crc_verify(Tamagawa_Handle handle);

/**
 *  \brief      Pass the values of CF(Control Field), ADF(address of EEPROM) and EDF(data for EEPROM) to the CRC calculator fucntion and update the CRC field.<br>
 *
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *  \param[in]  cmd     tamagawa command number
 *  \param[in]  ch      channel number that is currently selected
 *
 */

void tamagawa_update_crc(Tamagawa_Handle handle, int32_t cmd, uint32_t ch);

/**
 *  \brief      Update the values for oversample rate and division factor for Tx and Rx.<br>
 *
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *  \param[in]  baudrate     baud rate of the tamagawa encoder
 *
 */

void tamagawa_set_baudrate(Tamagawa_Handle handle, double baudrate);

/**
 *  \brief      Reset the values of the variables used in CRC calculation to 0.<br>
 *
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *
 */

void tamagawa_eeprom_crc_reinit(Tamagawa_Handle handle);

/**
 *  \brief      Reverse the bits of a number.<br>
 *
 *
 *  \param[in]  data    8 bit value for any of CF(Control Field), ADF(address of EEPROM) or EDF(data for EEPROM)
 *
 *  \retval     reversed_num    number obtained after reversing the bits of data provided
 *
 */

uint32_t tamagawa_reverse_bits(int8_t data);

/**
 *  \brief      Add the start and the stop bit to the reversed data.<br>
 *
 *
 *  \param[in]  eeprom_tx_data    holds the value of the Tx data to be sent
 *  \param[in]  data    holds the value of CF(Control Field), ADF(address of EEPROM) or EDF(data for EEPROM)
 *
 *  \retval     eeprom_tx_data    64 bit integer that holds the value of the Tx data to be sent
 *
 */

uint64_t tamagawa_prepare_eeprom_tx_data(uint64_t eeprom_tx_data, volatile uint32_t data);

/**
 *  \brief      Prepare the required EEPROM command from the CF(Control Field), ADF(address of EEPROM) and EDF(data for EEPROM).<br>
 *
 *
 *  \param[in]  handle    cookie returned by tamagawa_init
 *  \param[in]  cmd     tamagawa command number
 *  \param[in]  ch      channel number that is currently selected
 *
 */

void tamagawa_prepare_eeprom_command(Tamagawa_Handle handle, int32_t cmd, uint32_t ch);

/** @} */

#ifdef __cplusplus
}
#endif

#endif

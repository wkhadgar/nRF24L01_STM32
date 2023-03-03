//
// Created by paulo on 21/11/2022.
//

#ifndef EQDRIVER_NRF24L01P_H
#define EQDRIVER_NRF24L01P_H

#include "stdint.h"
#include "stdbool.h"

/**
 * @brief nRF24L01 commands.
 */
#define nRF24_CMD_R_REGISTER       (uint8_t)0x00      /**<  Register read. */
#define nRF24_CMD_W_REGISTER       (uint8_t)0x20      /**<  Register write. */
#define nRF24_CMD_ACTIVATE         (uint8_t)0x50      /**<  (De)Activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK features. */
#define nRF24_CMD_R_RX_PL_WID       (uint8_t)0x60     /**<  Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO. */
#define nRF24_CMD_R_RX_PAYLOAD     (uint8_t)0x61      /**<  Read RX payload. */
#define nRF24_CMD_W_TX_PAYLOAD     (uint8_t)0xA0      /**<  Write TX payload. */
#define nRF24_CMD_W_ACK_PAYLOAD    (uint8_t)0xA8      /**<  Write ACK payload. */
#define nRF24_CMD_W_TX_PAYLOAD_NO_ACK (uint8_t) 0xB0  /**<  Write TX payload and disable AUTOACK. */
#define nRF24_CMD_FLUSH_TX         (uint8_t)0xE1      /**<  Flush TX FIFO. */
#define nRF24_CMD_FLUSH_RX         (uint8_t)0xE2      /**<  Flush RX FIFO. */
#define nRF24_CMD_REUSE_TX_PL      (uint8_t)0xE3      /**<  Reuse TX payload. */
#define nRF24_CMD_LOCK_UNLOCK      (uint8_t)0x50      /**<  Lock/unlock exclusive features. */
#define nRF24_CMD_NOP              (uint8_t)0xFF      /**<  No operation (used for reading status register). */

/**
 * @brief nRF24L01 register adresses.
 */
#define nRF24_REG_CONFIG           (uint8_t)0x00  /**< Configuration register. */
#define nRF24_REG_EN_AA            (uint8_t)0x01  /**< Enable "Auto acknowledgment". */
#define nRF24_REG_EN_RXADDR        (uint8_t)0x02  /**< Enable RX addresses. */
#define nRF24_REG_SETUP_AW         (uint8_t)0x03  /**< Setup of address widths. */
#define nRF24_REG_SETUP_RETR       (uint8_t)0x04  /**< Setup of automatic retransmit. */
#define nRF24_REG_RF_CH            (uint8_t)0x05  /**< RF channel. */
#define nRF24_REG_RF_SETUP         (uint8_t)0x06  /**< RF setup register. */
#define nRF24_REG_STATUS           (uint8_t)0x07  /**< Status register. */
#define nRF24_REG_OBSERVE_TX       (uint8_t)0x08  /**< Transmit observe register. */
#define nRF24_REG_RPD              (uint8_t)0x09  /**< Received power detector. */
#define nRF24_REG_RX_ADDR_P0       (uint8_t)0x0A  /**< Receive address data pipe 0. */
#define nRF24_REG_RX_ADDR_P1       (uint8_t)0x0B  /**< Receive address data pipe 1. */
#define nRF24_REG_RX_ADDR_P2       (uint8_t)0x0C  /**< Receive address data pipe 2. */
#define nRF24_REG_RX_ADDR_P3       (uint8_t)0x0D  /**< Receive address data pipe 3. */
#define nRF24_REG_RX_ADDR_P4       (uint8_t)0x0E  /**< Receive address data pipe 4. */
#define nRF24_REG_RX_ADDR_P5       (uint8_t)0x0F  /**< Receive address data pipe 5. */
#define nRF24_REG_TX_ADDR          (uint8_t)0x10  /**< Transmit address. */
#define nRF24_REG_RX_PW_P0         (uint8_t)0x11  /**< Number of bytes in RX payload in data pipe 0. */
#define nRF24_REG_RX_PW_P1         (uint8_t)0x12  /**< Number of bytes in RX payload in data pipe 1. */
#define nRF24_REG_RX_PW_P2         (uint8_t)0x13  /**< Number of bytes in RX payload in data pipe 2. */
#define nRF24_REG_RX_PW_P3         (uint8_t)0x14  /**< Number of bytes in RX payload in data pipe 3. */
#define nRF24_REG_RX_PW_P4         (uint8_t)0x15  /**< Number of bytes in RX payload in data pipe 4. */
#define nRF24_REG_RX_PW_P5         (uint8_t)0x16  /**< Number of bytes in RX payload in data pipe 5. */
#define nRF24_REG_FIFO_STATUS      (uint8_t)0x17  /**< FIFO status register. */
#define nRF24_REG_DYNPD            (uint8_t)0x1C  /**< Enable dynamic payload length. */
#define nRF24_REG_FEATURE          (uint8_t)0x1D  /**< Feature register. */
#define NRF_REG_AMOUNT             (uint8_t)26    /**< Ammount of config registers. */

/**
 * @brief Register bits definitions.
 */
#define nRF24_CONFIG_PRIM_RX       (uint8_t)0x01  /**< PRIM_RX bit in CONFIG register. */
#define nRF24_CONFIG_PWR_UP        (uint8_t)0x02  /**< PWR_UP bit in CONFIG register. */
#define nRF24_FEATURE_EN_DYN_ACK   (uint8_t)0x01  /**< EN_DYN_ACK bit in FEATURE register. */
#define nRF24_FEATURE_EN_ACK_PAY   (uint8_t)0x02  /**< EN_ACK_PAY bit in FEATURE register. */
#define nRF24_FEATURE_EN_DPL       (uint8_t)0x04  /**< EN_DPL bit in FEATURE register. */
#define nRF24_FLAG_RX_DR           (uint8_t)0x40  /**< RX_DR bit (data ready RX FIFO interrupt). */
#define nRF24_FLAG_TX_DS           (uint8_t)0x20  /**< TX_DS bit (data sent TX FIFO interrupt). */
#define nRF24_FLAG_MAX_RT          (uint8_t)0x10  /**< MAX_RT bit (maximum number of TX retransmits interrupt). */
#define nRF24_FLAG_CONT_WAVE       (uint8_t)0x80  /**< CONT_WAVE bit (continuous wave carrier). */
#define nRF24_FLAG_PLL_LOCK        (uint8_t)0x10  /**< PLL_LOCK bit (locks PLL). */

/**
 * @brief Register masks definitions.
 */
#define nRF24_MASK_REG_MAP         (uint8_t)0x1F  /**< Mask bits[4:0] for CMD_RREG and CMD_WREG commands. */
#define nRF24_MASK_CRC             (uint8_t)0x0C  /**< Mask for CRC bits [3:2] in CONFIG register. */
#define nRF24_MASK_STATUS_IRQ      (uint8_t)0x70  /**< Mask for all IRQ bits in STATUS register. */
#define nRF24_MASK_RF_PWR          (uint8_t)0x06  /**< Mask RF_PWR[2:1] bits in RF_SETUP register. */
#define nRF24_MASK_RX_P_NO         (uint8_t)0x0E  /**< Mask RX_P_NO[3:1] bits in STATUS register. */
#define nRF24_MASK_DATARATE        (uint8_t)0x28  /**< Mask RD_DR_[5,3] bits in RF_SETUP register. */
#define nRF24_MASK_EN_RX           (uint8_t)0x3F  /**< Mask ERX_P[5:0] bits in EN_RXADDR register. */
#define nRF24_MASK_RX_PW           (uint8_t)0x3F  /**< Mask [5:0] bits in RX_PW_Px register. */
#define nRF24_MASK_RETR_ARD        (uint8_t)0xF0  /**< Mask for ARD[7:4] bits in SETUP_RETR register. */
#define nRF24_MASK_RETR_ARC        (uint8_t)0x0F  /**< Mask for ARC[3:0] bits in SETUP_RETR register. */
#define nRF24_MASK_RXFIFO          (uint8_t)0x03  /**< Mask for RX FIFO status bits [1:0] in FIFO_STATUS register. */
#define nRF24_MASK_TXFIFO          (uint8_t)0x30  /**< Mask for TX FIFO status bits [5:4] in FIFO_STATUS register. */
#define nRF24_MASK_PLOS_CNT        (uint8_t)0xF0  /**< Mask for PLOS_CNT[7:4] bits in OBSERVE_TX register. */
#define nRF24_MASK_ARC_CNT         (uint8_t) 0x0F /**< Mask for ARC_CNT[3:0] bits in OBSERVE_TX register. */

#define nRF24_WAIT_TIMEOUT         (uint32_t) 100 /**< Timeout value for blocking communication. */
#define PLD_LEN                    (uint8_t) 0x10 /**< Maximum payload size, if the dinamic payload is not used. */
#define NRF24_TRANSF_MAX_LEN       (PLD_LEN - 3)  /**< Standard maximum size for the data tranfer data. */

/**
 * @brief Enumerates the possible retransmit delays.
 */
typedef enum __attribute__((packed)) {
    nRF24_ARD_NONE = (uint8_t) 0x00, /**< Dummy value exclusive for when retransmission is not used. */
    nRF24_ARD_250us = (uint8_t) 0x00,
    nRF24_ARD_500us = (uint8_t) 0x01,
    nRF24_ARD_750us = (uint8_t) 0x02,
    nRF24_ARD_1000us = (uint8_t) 0x03,
    nRF24_ARD_1250us = (uint8_t) 0x04,
    nRF24_ARD_1500us = (uint8_t) 0x05,
    nRF24_ARD_1750us = (uint8_t) 0x06,
    nRF24_ARD_2000us = (uint8_t) 0x07,
    nRF24_ARD_2250us = (uint8_t) 0x08,
    nRF24_ARD_2500us = (uint8_t) 0x09,
    nRF24_ARD_2750us = (uint8_t) 0x0A,
    nRF24_ARD_3000us = (uint8_t) 0x0B,
    nRF24_ARD_3250us = (uint8_t) 0x0C,
    nRF24_ARD_3500us = (uint8_t) 0x0D,
    nRF24_ARD_3750us = (uint8_t) 0x0E,
    nRF24_ARD_4000us = (uint8_t) 0x0F,
} retransmit_delay_t;

/**
 * @brief Enum for the possible data rate values.
 */
typedef enum __attribute__((packed)) {
    nRF24_DR_250kbps = (uint8_t) 0x20,   /**< 250kbps data rate. */
    nRF24_DR_1Mbps = (uint8_t) 0x00,     /**< 1Mbps data rate. */
    nRF24_DR_2Mbps = (uint8_t) 0x08,     /**< 2Mbps data rate. */
} data_rate_t;

/**
 * @brief RF output power in TX mode
 */
typedef enum __attribute__((packed)) {
    nRF24_TXPWR_18dBm = (uint8_t) 0x00, /**< -18dBm. */
    nRF24_TXPWR_12dBm = (uint8_t) 0x02, /**< -12dBm. */
    nRF24_TXPWR_6dBm = (uint8_t) 0x04,  /**<  -6dBm. */
    nRF24_TXPWR_0dBm = (uint8_t) 0x06,  /**<   0dBm. */
} output_power_t;

/**
 * @brief CRC encoding scheme.
 */
typedef enum __attribute__((packed)) {
    nRF24_CRC_off = (uint8_t) 0x00,   /**< CRC disabled. */
    nRF24_CRC_1byte = (uint8_t) 0x08, /**< 1-byte CRC. */
    nRF24_CRC_2byte = (uint8_t) 0x0c, /**< 2-byte CRC. */
} crc_scheme_t;

/**
 * @brief nRF24L01 power control.
 */
typedef enum __attribute__((packed)) {
    nRF24_PWR_DOWN = (uint8_t) 0x00, /**< Power down. */
    nRF24_PWR_UP = (uint8_t) 0x02,   /**< Power up. */
} power_state_t;

/**
 * @brief Transceiver mode.
 */
typedef enum __attribute__((packed)) {
    nRF24_MODE_TX = (uint8_t) 0x00, /**< PTX. */
    nRF24_MODE_RX = (uint8_t) 0x01, /**< PRX. */
} transceiver_mode_t;

/**
 * @brief Dynamic payload mode.
 */
typedef enum __attribute__((packed)) {
    nRF24_DPL_OFF = (uint8_t) 0x00,  /**< PTX. */
    nRF24_DPL_ON = (uint8_t) 0x01,   /**< PRX. */
} dpl_state_t;

/**
 * @brief Enumeration of RX pipe addresses and TX address.
 */
typedef enum __attribute__((packed)) {
    nRF24_PIPE0 = (uint8_t) 0x00,    /**< Pipe 0. */
    nRF24_PIPE1 = (uint8_t) 0x01,    /**< Pipe 1. */
    nRF24_PIPE2 = (uint8_t) 0x02,    /**< Pipe 2. */
    nRF24_PIPE3 = (uint8_t) 0x03,    /**< Pipe 3. */
    nRF24_PIPE4 = (uint8_t) 0x04,    /**< Pipe 4. */
    nRF24_PIPE5 = (uint8_t) 0x05,    /**< Pipe 5. */
    nRF24_PIPETX = (uint8_t) 0x06,   /**< TX address (not a pipe in fact). */
} pipe_addr_t;

/**
 * @brief State of auto acknowledgment for specified pipe.
 */
typedef enum __attribute__((packed)) {
    nRF24_AA_OFF = (uint8_t) 0x00,   /**< Enable Auto-ACK. */
    nRF24_AA_ON = (uint8_t) 0x01,    /**< Disable Auto-ACK. */
} auto_ack_state_t;

/**
 * @brief Width of the pipe adresses.
 */
typedef enum __attribute__((packed)) {
    nRF24_ADDR_3_BITS = 0x01,        /**< 3 byte address. */
    nRF24_ADDR_4_BITS = 0x02,        /**< 4 byte address. */
    nRF24_ADDR_5_BITS = 0x03,        /**< 5 byte address. */
} address_width_t;

/**
 * @brief Status of the RX FIFO.
 */
typedef enum __attribute__((packed)) {
    nRF24_STATUS_RXFIFO_DATA = (uint8_t) 0x00,   /**< The RX FIFO contains data and available locations. */
    nRF24_STATUS_RXFIFO_EMPTY = (uint8_t) 0x01,  /**< The RX FIFO is empty. */
    nRF24_STATUS_RXFIFO_FULL = (uint8_t) 0x02,   /**< The RX FIFO is full. */
    nRF24_STATUS_RXFIFO_ERROR = (uint8_t) 0x03,  /**< Impossible state: RX FIFO cannot be empty and full at the same time. */
} rx_fifo_status_t;

/**
 * @brief Status of the TX FIFO.
 */
typedef enum __attribute__((packed)) {
    nRF24_STATUS_TXFIFO_DATA = (uint8_t) 0x00,   /**< The TX FIFO contains data and available locations. */
    nRF24_STATUS_TXFIFO_EMPTY = (uint8_t) 0x01,  /**< The TX FIFO is empty. */
    nRF24_STATUS_TXFIFO_FULL = (uint8_t) 0x02,   /**< The TX FIFO is full. */
    nRF24_STATUS_TXFIFO_ERROR = (uint8_t) 0x03,  /**< Impossible state: TX FIFO cannot be empty and full at the same time. */
} tx_fifo_status_t;

/**
 * @brief Result of RX FIFO reading.
 */
typedef enum __attribute__((packed)) {
    nRF24_RX_PIPE0 = (uint8_t) 0x00,             /**< Packet received from the PIPE#0. */
    nRF24_RX_PIPE1 = (uint8_t) 0x01,             /**< Packet received from the PIPE#1. */
    nRF24_RX_PIPE2 = (uint8_t) 0x02,             /**< Packet received from the PIPE#2. */
    nRF24_RX_PIPE3 = (uint8_t) 0x03,             /**< Packet received from the PIPE#3. */
    nRF24_RX_PIPE4 = (uint8_t) 0x04,             /**< Packet received from the PIPE#4. */
    nRF24_RX_PIPE5 = (uint8_t) 0x05,             /**< Packet received from the PIPE#5. */
    nRF24_RX_EMPTY = (uint8_t) 0xFF,             /**< The RX FIFO is empty. */
} nRF24_RXResult;

/**
 * @brief Result of packet transmission.
 */
typedef enum __attribute__((packed)) {
    nRF24_TX_ERROR = (uint8_t) 0x00,             /**< Unknown error. */
    nRF24_TX_SUCCESS,                            /**< Packet has been transmitted successfully. */
    nRF24_TX_TIMEOUT,                            /**< It was timeout during packet transmit. */
    nRF24_TX_MAXRT,                              /**< Transmit failed with maximum auto retransmit count. */
} nRF24_TXResult;

/**
 * @brief Mode of communication of one's specific data packet.
 */
typedef enum __attribute__((packed)) {
    REQUEST = (uint8_t) 0, /**< Request packet. */
    RESPONSE,              /**< Response packet. */
    COMMAND,               /**< Command packet. */
} transf_t;

/**
 * @brief Struct of a data packet, used to encapsulate most uses of communication.
 */
typedef struct {
    transf_t kind;
    struct {
        uint8_t data;
        uint8_t payload;
    } size;
    uint8_t data[NRF24_TRANSF_MAX_LEN];
} nrf24_data_t;

/**
 * @brief Initializes the nRF24L01 module, with standard configuration.
 *
 * @param addr Address for the TX and pipe 0 communication.
 * @param channel RF channel of the transceivers.
 */
void nRF24_init(const uint8_t* addr, uint8_t channel);

/**
 * @brief Checks if the module is present by reading the tx pipe address and comparing it to the suposedly previously well-written address.
 *
 * @param curr_addr [in] Current set address, it will be compared with the module's cuurent address.
 * @return bool true if adresses are equal, false otherwise.
 */
bool nRF24_check(const uint8_t* curr_addr);

/**
 * @brief Configures the payload size.
 *
 * @param size Max payload size in bytes.
 */
void nRF24_setPayloadSize(uint8_t size);

/**
 * @brief Turns the module on or off.
 *
 * @param mode Desired state of the module.
 */
void nRF24_SetPowerMode(power_state_t mode);

/**
 * @brief Configures the RF channel for the transceiver.
 *
 * @param channel Desired channel for RF ccommunications.
 *
 * @note The channel must be between 0 and 127, inclusive. This will efectively shift the 2.4GHz RF signal to
 * +(1 * channel)MHz.
 */
void nRF24_SetRFChannel(uint8_t channel);

/**
 * @brief Configures the number of auto-retransmissions and the retransmission delay for the Enhanced ShockBurst.
 *
 * @param ard Delay for the retransmissions.
 * @param arc Number of retransmissons.
 *
 * @note It is recommended to use retransmission delays of more than 1000us.
 */
void nRF24_SetAutoRetr(retransmit_delay_t ard, uint8_t arc);

/**
 * @brief Configures the address width.
 *
 * @param addr_width Width of the adresses used by the pipes.
 */
void nRF24_SetAddrWidth(address_width_t addr_width);

/**
 * @brief Sets the address for the specified pipe.
 *
 * @param pipe Pipe to have the address updated.
 * @param addr [in] Address to set.
 */
void nRF24_SetAddr(pipe_addr_t pipe, const uint8_t* addr);

/**
 * @brief Configures the TX power.
 *
 * @param tx_pwr Power for the RF transmission.
 */
void nRF24_SetTXPower(output_power_t tx_pwr);

/**
 * @brief Configures the data rate for the transmissions.
 *
 * @param data_rate Desired data rate.
 */
void nRF24_SetDataRate(data_rate_t data_rate);

/**
 * @brief Configures the CRC for the packets.
 *
 * @param scheme desired CRC scheme.
 *
 * @note CRC should not be used with copy nR24L01 modules, since they have an error that lends they unusable if CRC is used.
 */
void nRF24_SetCRCScheme(crc_scheme_t scheme);

/**
 * @brief Configure an specific pipe.
 *
 * @param pipe Pipe to be configured.
 * @param aa_state Auto acknowledge state on this pipe.
 * @param payload_len Max payload length on this pipe.
 */
void nRF24_SetRXPipe(pipe_addr_t pipe, auto_ack_state_t aa_state, uint8_t payload_len);

/**
 * @brief Closes the specified pipe.
 *
 * @param pipe Pipe to be closed.
 */
void nRF24_ClosePipe(pipe_addr_t pipe);

/**
 * @brief Enable the auto acknowledge for the specified pipe.
 *
 * @param pipe Desired pipe to enable auto-ack.
 */
void nRF24_EnableAA(pipe_addr_t pipe);

/**
 * @brief Disable the auto acknowledge for the specified pipe.
 *
 * @param pipe Desired pipe to disable auto-ack.
 */
void nRF24_DisableAA(pipe_addr_t pipe);

/**
 * @brief Configures the dinamic payload feature.
 *
 * @param mode Desired DPL configuration.
 */
void nRF24_SetDynamicPayloadLength(dpl_state_t mode);

/**
 * @brief Configures the ack payload feature.
 * @param mode Desired ACK_PAYLOAD configuration.
 */
void nRF24_SetPayloadWithAck(auto_ack_state_t mode);

/**
 * @brief Gets the STATUS register.
 *
 * @return uint8_t STATUS register current status.
 */
uint8_t nRF24_GetStatus(void);

/**
 * @brief Gets the IRQ bits from the STATUS register.
 *
 * @return uint8_t IRQ masked STATUS register current status.
 */
uint8_t nRF24_GetIRQFlags(void);

/**
 * @brief Gets the RX bits from the FIFO register.
 *
 * @return uint8_t RX masked FIFO register current status.
 */
uint8_t nRF24_GetStatus_RXFIFO(void);

/**
 * @brief Gets the TX bits from the FIFO register.
 *
 * @return uint8_t TX masked FIFO register current status.
 */
uint8_t nRF24_GetStatus_TXFIFO(void);

/**
 * @brief Gets the RX_PIPE bits from the STATUS register.
 *
 * @return uint8_t RX_PIPE masked STATUS register current status.
 */
uint8_t nRF24_GetRXSource(void);

/**
 * @brief Gets the OBSERVE_TX register.
 *
 * @return uint8_t OBSERVE_TX register current status.
 */
uint8_t nRF24_GetRetransmitCounters(void);

/**
 * @brief Gets the FEATURE register.
 *
 * @return uint8_t FEATURE register current status.
 */
uint8_t nRF24_GetFeatures(void);

/**
 * @brief Resets the packets lost counter in the OBSERVE_TX register.
 */
void nRF24_ResetPLOS(void);

/**
 * @brief Flushes the TX FIFO.
 */
void nRF24_FlushTX(void);

/**
 * @brief Flushes the RX FIFO.
 */
void nRF24_FlushRX(void);

/**
 * @brief Clear the status of the IRQ bits on the STATUS register.
 */
void nRF24_ClearIRQFlags(void);

/**
 * @brief Activates the features in the FEATURE register.
 *
 * @note This command, when sent multiple times should toggle the FEATURE bits, but this is for some types of nRF24L01
 * modules, there is a variant on which this behaviour is not present.
 */
void nRF24_ActivateFeatures(void);

/**
 * @brief Configures the reuse of the payload in the TX FIFO. Effectively not erasing it after its transmission.
 */
void nRF24_ReuseTX(void);

/**
 * @brief Starts the RF carrier constant output.
 *
 * @param pwr Desired output power.
 * @param channel Desired output channel.
 */
void nRF24_StartCarrier(output_power_t pwr, uint8_t channel);

/**
 * @brief Stops the emission of the carrier wave.
 */
void nRF24_StopCarrier(void);

/**
 * @brief Checks if the carrier wave is detectable.
 *
 * @return bool true if carrier wave is detected, false otherwise.
 */
bool nRF24_CarrierDetect(void);

/**
 * @brief Writes the already prepared data into the TX FIFO.
 *
 * @param nRF24_data [in] Prepared data to be sent.
 */
void nRF24_WritePayload(const nrf24_data_t* nRF24_data);

/**
 * @brief Writes the already prepared data to be an ACK payload.
 *
 * @param pipe Pipe that should have this ACK payload.
 * @param nRF24_data [in] Prepared data to be sent.
 */
void nRF24_WriteAckPayload(nRF24_RXResult pipe, const nrf24_data_t* nRF24_data);

/**
 * @brief Gets the available payload from the RX FIFO, without DPL.
 *
 * @param nRF24_data [out] Pointer to the received data buffer.
 * @return nRF24_RXResult Pipe where the FIFO payoad was read.
 */
nRF24_RXResult nRF24_ReadPayload(nrf24_data_t* nRF24_data);

/**
 * @brief Gets the available payload from the RX FIFO, with DPL.
 *
 * @param nRF24_data [out] Pointer to the received data buffer.
 * @return nRF24_RXResult Pipe where the FIFO payoad was read.
 */
nRF24_RXResult nRF24_ReadPayloadDpl(nrf24_data_t* nRF24_data);

/**
 * @brief Sends the given data through RF to all the listening devices.
 *
 * @param nRF24_data [in] Data to be sent.
 * @return nRF24_TXResult The outcome of this transmission.
 */
nRF24_TXResult nRF24_TransmitPacket(const nrf24_data_t* nRF24_data);

/**
 * @brief Sets the module as RX, able to listen to other modules.
 */
void nRF24_StartListening(void);

/**
 * @brief Sets the module as TX, able to talk to other modules.
 */
void nRF24_StopListening(void);

/**
 * @brief Prepares a data packet for transmission, based on the given data.
 *
 * @param in_data [in] Data that is ment to be transformed in a data packet.
 * @param in_data_len Size of the input data.
 * @param transfer_mode Mode of transmission of this data.
 * @param out_data [out] Prepared data packet.
 */
void nRF24_PrepareData(const void* in_data, uint8_t in_data_len, transf_t transfer_mode, nrf24_data_t* out_data);

/**
 * @brief Retrieves the information from a received data packet.
 *
 * @param in_data Received packet.
 * @param out_data [out] Pointer to where the retrieved data should be placed.
 */
void nRF24_RetrieveData(nrf24_data_t in_data, void* out_data);

/**
 * @brief Transmit to other modules, waiting for an answer if needed, and sets the module to a given mode afterwards.
 *
 * @param question Data to be sent.
 * @param answer [out] Data received as a response.
 * @param after_set Mode of the module after the given communication.
 * @return bool true if everything succeded, false otherwise.
 */
bool nRF24_Talk(nrf24_data_t question, nrf24_data_t* answer, transceiver_mode_t after_set);

/**
 * @brief Gets the received data.
 *
 * @param nRF24_payload [out] Pointer to the data buffer.
 * @return bool true if data was received, false otherwise.
 */
bool nRF24_GetData(nrf24_data_t* nRF24_payload);

#endif //EQDRIVER_NRF24L01P_H
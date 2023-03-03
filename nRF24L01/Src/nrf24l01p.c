//
// Created by paulo on 21/11/2022.
//

#include "nrf24l01p.h"
#include "spi.h"
#include "flags.h"

#define NRF_CS_GPIO NRF_CS_GPIO_Port
#define NRF_CS_PIN NRF_CS_Pin

#define NRF_CE_GPIO NRF_CE_GPIO_Port
#define NRF_CE_PIN NRF_CE_Pin

#define SPI_HANDLE &hspi2

/**
 * @brief Addresses of the RX_PW_P# registers.
 */
static const uint8_t nRF24_RX_PW_PIPE[] = {
        nRF24_REG_RX_PW_P0,
        nRF24_REG_RX_PW_P1,
        nRF24_REG_RX_PW_P2,
        nRF24_REG_RX_PW_P3,
        nRF24_REG_RX_PW_P4,
        nRF24_REG_RX_PW_P5,
};

/**
 * @brief Addresses of the address registers.
 */
static const uint8_t nRF24_ADDR_REGS[] = {
        nRF24_REG_RX_ADDR_P0,
        nRF24_REG_RX_ADDR_P1,
        nRF24_REG_RX_ADDR_P2,
        nRF24_REG_RX_ADDR_P3,
        nRF24_REG_RX_ADDR_P4,
        nRF24_REG_RX_ADDR_P5,
        nRF24_REG_TX_ADDR,
};

/**
 * @brief Structure to encapsulate all the nRF24 config data.
 *
 * @note This has mostly debug purposes. Feel free to change as modify this as pleased.
 */
static struct {
    uint8_t registers[NRF_REG_AMOUNT];
    struct {
        retransmit_delay_t rt_delay;
        uint8_t rt_cnt;
        data_rate_t data_rate;
        output_power_t tx_pwr;
        crc_scheme_t crc;
        power_state_t pwr_state;
        transceiver_mode_t mode;
        dpl_state_t dpl;
        auto_ack_state_t auto_ack;
        address_width_t addr_w;
        uint8_t tx_addr[5];
        uint8_t channel;
    } configs;
    struct {
        rx_fifo_status_t rx_fifo_status;
        tx_fifo_status_t tx_fifo_status;
        bool is_p_variant;
    } info;
} nRF24 = {
        .registers = {0},
        .configs = {
                .rt_delay = nRF24_ARD_1500us,
                .rt_cnt = 15,
                .data_rate = nRF24_DR_1Mbps,
                .tx_pwr = nRF24_TXPWR_0dBm,
                .crc = nRF24_CRC_2byte,
                .pwr_state = nRF24_PWR_DOWN,
                .mode = nRF24_MODE_TX,
                .dpl = nRF24_DPL_OFF,
                .auto_ack = nRF24_AA_OFF,
                .addr_w = nRF24_ADDR_5_BITS,
                .tx_addr = {0},
                .channel = 0,
        },
        .info = {
                .rx_fifo_status = nRF24_STATUS_RXFIFO_EMPTY,
                .tx_fifo_status = nRF24_STATUS_TXFIFO_EMPTY,
                .is_p_variant = false,
        },
};

/**
 * @brief Enables de module by setting CE pin.
 */
static inline void nRF24_enable(void) {
    HAL_GPIO_WritePin(NRF_CE_GPIO, NRF_CE_PIN, GPIO_PIN_SET);
}

/**
 * @brief Disables de module by clearing CE pin.
 */
static inline void nRF24_disable(void) {
    HAL_GPIO_WritePin(NRF_CE_GPIO, NRF_CE_PIN, GPIO_PIN_RESET);
}

/**
 * @brief SPI to the module.
 *
 * @param data SPI data.
 * @return uint8_t Response of the SPI communication.
 */
static inline uint8_t nRF24_SPI_TR(uint8_t data) {
uint8_t result;
if (HAL_SPI_TransmitReceive(SPI_HANDLE, &data, &result, 1, 10) != HAL_OK) {
return 0;
}
return result;
}

/**
 * @brief Read a register value.
 *
 * @param reg Register to read.
 * @return uint8_t Current register status.
 */
static uint8_t nRF24_read_register(const uint8_t reg) {
    uint8_t value;


    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_RESET);

    nRF24_SPI_TR(reg & nRF24_MASK_REG_MAP);
    value = nRF24_SPI_TR(nRF24_CMD_NOP);

    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_SET);

    if (reg <= nRF24_REG_FIFO_STATUS) {
        nRF24.registers[reg] = value;
    } else if ((reg == nRF24_REG_DYNPD) || (reg == nRF24_REG_FEATURE)) {
        nRF24.registers[reg - 0x04] = value;
    }

    return value;
}

/**
 * @brief Write a new value to register.
 *
 * @param reg number of register to write.
 * @param value value to write.
 */
static void nRF24_write_register(uint8_t reg, uint8_t value) {
    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_RESET);

    if (reg < nRF24_CMD_W_REGISTER) {
        // This is a register access
        nRF24_SPI_TR(nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP));
        nRF24_SPI_TR(value);
    } else {
        // This is a single byte command or future command/register
        nRF24_SPI_TR(reg);
        if ((reg != nRF24_CMD_FLUSH_TX) && (reg != nRF24_CMD_FLUSH_RX) && (reg != nRF24_CMD_REUSE_TX_PL) &&
            (reg != nRF24_CMD_NOP)) {
            // Send register value
            nRF24_SPI_TR(value);
        }
    }

    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_SET);

    if (reg <= nRF24_REG_FIFO_STATUS) {
        nRF24.registers[reg] = value;
    } else if ((reg == nRF24_REG_DYNPD) || (reg == nRF24_REG_FEATURE)) {
        nRF24.registers[reg - 4] = value;
    }
}

/**
 * @brief Read a multiple byte register.
 *
 * @param reg number of the register to read.
 * @param pBuf [out] pointer to the buffer for the read data.
 * @param count number of bytes to read.
 */
static void nRF24_read_multi_register(uint8_t reg, uint8_t* pBuf, uint8_t count) {
    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_RESET);

    nRF24_SPI_TR(reg);
    while (count--) {
        *pBuf++ = nRF24_SPI_TR(nRF24_CMD_NOP);
    }

    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief Write a multiple byte register.
 *
 * @param reg number of the register to write.
 * @param pBuf [out] pointer to the buffer for the write data.
 * @param count number of bytes to write.
 */
static void nRF24_write_multi_register(uint8_t reg, const uint8_t* pBuf, uint8_t count) {
    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_RESET);
    nRF24_SPI_TR(reg);
    while (count--) {
        if (nRF24_SPI_TR(*pBuf++) != HAL_OK) {
            break;
        }
    }
    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief Sets the module on RX or TX mode.
 *
 * @param mode Desired transceiver mode.
 */
static void nRF24_SetOperationalMode(transceiver_mode_t mode) {
    uint8_t reg;

    // Configure PRIM_RX bit of the CONFIG register
    reg = nRF24_read_register(nRF24_REG_CONFIG);
    reg &= ~nRF24_CONFIG_PRIM_RX;
    reg |= (mode & nRF24_CONFIG_PRIM_RX);
    nRF24_write_register(nRF24_REG_CONFIG, reg);

    nRF24.configs.mode = mode;
}

void nRF24_init(const uint8_t* addr, uint8_t channel) {
    uint8_t prev_feat;

    HAL_Delay(5);
    nRF24_SetAutoRetr(nRF24_ARD_2500us, 15);
    nRF24_SetDataRate(nRF24_DR_250kbps);
    nRF24_SetTXPower(nRF24_TXPWR_6dBm);

    prev_feat = nRF24_GetFeatures();
    nRF24_ActivateFeatures();
    nRF24_GetFeatures();
    nRF24.info.is_p_variant = (prev_feat == nRF24.registers[nRF24_REG_FEATURE - 0x04]);
    if (nRF24.registers[nRF24_REG_FEATURE - 0x04]) {
        if (nRF24.info.is_p_variant) {
            nRF24_ActivateFeatures();
        }
        nRF24_write_register(nRF24_REG_FEATURE, 0x00);
    }

    nRF24_write_register(nRF24_REG_DYNPD, 0x00);
    nRF24_write_register(nRF24_REG_EN_AA, 0x3F);
    nRF24_write_register(nRF24_REG_EN_RXADDR, 0x01);
    nRF24_setPayloadSize(32);
    nRF24_SetAddrWidth(nRF24_ADDR_5_BITS);
    nRF24_SetRFChannel(channel);
    nRF24_write_multi_register(nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR, addr, 5);
    nRF24_write_multi_register(nRF24_CMD_W_REGISTER | nRF24_REG_RX_ADDR_P0, addr, 5);
    nRF24_ClearIRQFlags();
    nRF24_FlushRX();
    nRF24_FlushTX();

    nRF24_SetCRCScheme(nRF24_CRC_off);
    nRF24_SetPowerMode(nRF24_PWR_UP);
}

bool nRF24_check(const uint8_t* curr_addr) {
    nRF24_read_multi_register(nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR, nRF24.configs.tx_addr, 5);

    // Compare buffers, return error on first mismatch
    for (uint8_t i = 0; i < 5; i++) {
        if (nRF24.configs.tx_addr[i] != *curr_addr++) {
            return false;
        }
    }

    return true;
}

void nRF24_setPayloadSize(uint8_t size) {

    if (size > 32) {
        size = 32;
    } else {
        size = (size < 1) ? 1 : size;
    }

    for (uint8_t i = 0; i < 6; i++) {
        nRF24_write_register(nRF24_REG_RX_PW_P0 + i, size);
    }
}

void nRF24_SetPowerMode(power_state_t mode) {
    uint8_t reg;

    reg = nRF24_read_register(nRF24_REG_CONFIG);
    if (mode == nRF24_PWR_UP) {
        // Set the PWR_UP bit of CONFIG register to wake the transceiver
        // It goes into Stanby-I mode with consumption about 26uA
        reg |= nRF24_CONFIG_PWR_UP;
    } else {
        // Clear the PWR_UP bit of CONFIG register to put the transceiver
        // into power down mode with consumption about 900nA
        reg &= ~nRF24_CONFIG_PWR_UP;
    }
    nRF24_write_register(nRF24_REG_CONFIG, reg);
    HAL_Delay(5);

    nRF24.configs.pwr_state = mode;
}

void nRF24_SetDynamicPayloadLength(dpl_state_t mode) {
    uint8_t reg;
    reg = nRF24_read_register(nRF24_REG_FEATURE);
    if (mode) {
        nRF24_write_register(nRF24_REG_FEATURE, reg | nRF24_FEATURE_EN_DPL);
        nRF24_write_register(nRF24_REG_DYNPD, 0x1F);
    } else {
        nRF24_write_register(nRF24_REG_FEATURE, reg & ~nRF24_FEATURE_EN_DPL);
        nRF24_write_register(nRF24_REG_DYNPD, 0x0);
    }

    nRF24.configs.dpl = mode;
}

void nRF24_SetPayloadWithAck(auto_ack_state_t mode) {
    uint8_t reg;
    reg = nRF24_read_register(nRF24_REG_FEATURE);
    if (mode) {
        nRF24_write_register(nRF24_REG_FEATURE, reg | nRF24_FEATURE_EN_ACK_PAY);
    } else {
        nRF24_write_register(nRF24_REG_FEATURE, reg & ~nRF24_FEATURE_EN_ACK_PAY);
    }

    nRF24.configs.auto_ack = mode;
}

void nRF24_SetCRCScheme(crc_scheme_t scheme) {
    uint8_t reg;

    // Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
    reg = nRF24_read_register(nRF24_REG_CONFIG);
    reg &= ~nRF24_MASK_CRC;
    reg |= (scheme & nRF24_MASK_CRC);
    nRF24_write_register(nRF24_REG_CONFIG, reg);

    nRF24.configs.crc = scheme;
}

void nRF24_SetRFChannel(uint8_t channel) {
    nRF24_write_register(nRF24_REG_RF_CH, channel);
    nRF24.configs.channel = channel;
}

void nRF24_SetAutoRetr(retransmit_delay_t ard, uint8_t arc) {
    // Set auto retransmit settings (SETUP_RETR register)
    nRF24_write_register(nRF24_REG_SETUP_RETR, (uint8_t) ((ard << 4) | (arc & nRF24_MASK_RETR_ARC)));
    nRF24.configs.rt_delay = ard;
    nRF24.configs.rt_cnt = arc;
}

void nRF24_SetAddrWidth(address_width_t addr_width) {
    nRF24_write_register(nRF24_REG_SETUP_AW, addr_width);
    nRF24.configs.addr_w = addr_width;
}

void nRF24_SetAddr(pipe_addr_t pipe, const uint8_t* addr) {
    uint8_t addr_width;

    // RX_ADDR_Px register
    switch (pipe) {
        case nRF24_PIPETX:
        case nRF24_PIPE0:
        case nRF24_PIPE1:
            // Get address width
            addr_width = nRF24_read_register(nRF24_REG_SETUP_AW) + 1;
            // Write address in reverse order (LSByte first)
            addr += addr_width;
            HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_RESET);
            nRF24_SPI_TR(nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe]);
            do {
                nRF24_SPI_TR(*addr--);
            } while (addr_width--);
            HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_SET);
            break;
        case nRF24_PIPE2:
        case nRF24_PIPE3:
        case nRF24_PIPE4:
        case nRF24_PIPE5:
            // Write address LSBbyte (only first byte from the addr buffer)
            nRF24_write_register(nRF24_ADDR_REGS[pipe], *addr);
            break;
        default:
            // Incorrect pipe number -> do nothing
            break;
    }

    nRF24_read_multi_register(nRF24_ADDR_REGS[pipe], nRF24.configs.tx_addr, 5);

}

void nRF24_SetTXPower(output_power_t tx_pwr) {
    uint8_t reg;

    // Configure RF_PWR[2:1] bits of the RF_SETUP register
    reg = nRF24_read_register(nRF24_REG_RF_SETUP);
    reg &= ~nRF24_MASK_RF_PWR;
    reg |= tx_pwr;
    nRF24_write_register(nRF24_REG_RF_SETUP, reg);

    nRF24.configs.tx_pwr = tx_pwr;
}

void nRF24_SetDataRate(data_rate_t data_rate) {
    uint8_t reg;

    // Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
    reg = nRF24_read_register(nRF24_REG_RF_SETUP);
    reg &= ~nRF24_MASK_DATARATE;
    reg |= data_rate;
    nRF24_write_register(nRF24_REG_RF_SETUP, reg);

    nRF24.configs.data_rate = data_rate;
}

void nRF24_SetRXPipe(pipe_addr_t pipe, auto_ack_state_t aa_state, uint8_t payload_len) {
    uint8_t reg;

    // Enable the specified pipe (EN_RXADDR register)
    reg = (nRF24_read_register(nRF24_REG_EN_RXADDR) | (1 << pipe)) & nRF24_MASK_EN_RX;
    nRF24_write_register(nRF24_REG_EN_RXADDR, reg);

    // Set RX payload length (RX_PW_Px register)
    nRF24_write_register(nRF24_RX_PW_PIPE[pipe], payload_len & nRF24_MASK_RX_PW);

    // Set auto acknowledgment for a specified pipe (EN_AA register)
    reg = nRF24_read_register(nRF24_REG_EN_AA);
    if (aa_state == nRF24_AA_ON) {
        reg |= (1 << pipe);
    } else {
        reg &= ~(1 << pipe);
    }
    nRF24_write_register(nRF24_REG_EN_AA, reg);
}

void nRF24_ClosePipe(pipe_addr_t pipe) {
    uint8_t reg;

    reg = nRF24_read_register(nRF24_REG_EN_RXADDR);
    reg &= ~(1 << pipe);
    reg &= nRF24_MASK_EN_RX;
    nRF24_write_register(nRF24_REG_EN_RXADDR, reg);
}

void nRF24_EnableAA(pipe_addr_t pipe) {
    uint8_t reg;

    // Set bit in EN_AA register
    reg = nRF24_read_register(nRF24_REG_EN_AA);
    reg |= (1 << pipe);
    nRF24_write_register(nRF24_REG_EN_AA, reg);
}

void nRF24_DisableAA(pipe_addr_t pipe) {
    uint8_t reg;

    if (pipe > 5) {
        // Disable Auto-ACK for ALL pipes
        nRF24_write_register(nRF24_REG_EN_AA, 0x00);
    } else {
        // Clear bit in the EN_AA register
        reg = nRF24_read_register(nRF24_REG_EN_AA);
        reg &= ~(1 << pipe);
        nRF24_write_register(nRF24_REG_EN_AA, reg);
    }
}

uint8_t nRF24_GetStatus(void) {
    nRF24.registers[nRF24_REG_STATUS] = nRF24_read_register(nRF24_REG_STATUS);
    return nRF24.registers[nRF24_REG_STATUS];
}

uint8_t nRF24_GetIRQFlags(void) {
    return (nRF24_read_register(nRF24_REG_STATUS) & nRF24_MASK_STATUS_IRQ);
}

uint8_t nRF24_GetStatus_RXFIFO(void) {
    nRF24.info.rx_fifo_status = (nRF24_read_register(nRF24_REG_FIFO_STATUS) & nRF24_MASK_RXFIFO);
    return (uint8_t) nRF24.info.rx_fifo_status;
}

uint8_t nRF24_GetStatus_TXFIFO(void) {
    nRF24.info.tx_fifo_status = ((nRF24_read_register(nRF24_REG_FIFO_STATUS) & nRF24_MASK_TXFIFO) >> 4);
    return (uint8_t) nRF24.info.tx_fifo_status;
}

uint8_t nRF24_GetRXSource(void) {
    return ((nRF24_read_register(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1);
}

uint8_t nRF24_GetRetransmitCounters(void) {
    return (nRF24_read_register(nRF24_REG_OBSERVE_TX));
}

uint8_t nRF24_GetFeatures(void) {
    return nRF24_read_register(nRF24_REG_FEATURE);
}

void nRF24_ResetPLOS(void) {
    uint8_t reg;

    // The PLOS counter is reset after write to RF_CH register
    reg = nRF24_read_register(nRF24_REG_RF_CH);
    nRF24_write_register(nRF24_REG_RF_CH, reg);
}

void nRF24_FlushTX(void) {
    nRF24_write_register(nRF24_CMD_FLUSH_TX, nRF24_CMD_NOP);
}

void nRF24_FlushRX(void) {
    nRF24_write_register(nRF24_CMD_FLUSH_RX, nRF24_CMD_NOP);
}

void nRF24_ClearIRQFlags(void) {
    uint8_t reg;

    // Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
    reg = nRF24_read_register(nRF24_REG_STATUS);
    reg |= nRF24_MASK_STATUS_IRQ;
    nRF24_write_register(nRF24_REG_STATUS, reg);
}

void nRF24_ActivateFeatures(void) {
    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_RESET);
    nRF24_SPI_TR(nRF24_CMD_ACTIVATE);
    nRF24_SPI_TR(0x73);
    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_SET);
}

void nRF24_WritePayload(const nrf24_data_t* nRF24_data) {
    nRF24_write_multi_register(nRF24_CMD_W_TX_PAYLOAD, (const uint8_t*) nRF24_data, nRF24_data->size.payload);
}

static uint8_t nRF24_GetRxDplPayloadWidth(void) {
    uint8_t value;

    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_RESET);
    nRF24_SPI_TR(nRF24_CMD_R_RX_PL_WID);
    value = nRF24_SPI_TR(nRF24_CMD_NOP);
    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_SET);

    return value;

}

static nRF24_RXResult nRF24_ReadPayloadGeneric(nrf24_data_t* nRF24_data, uint8_t dpl) {
    uint8_t pipe;
    uint8_t pld_size;

    // Extract a payload pipe number from the STATUS register
    pipe = (nRF24_read_register(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1;

    // RX FIFO empty?
    if (pipe < 6) {
        // Get payload length
        if (dpl) {
            pld_size = nRF24_GetRxDplPayloadWidth();
            if (pld_size > 32) {//broken packet
                pld_size = 0;
                nRF24_FlushRX();
            }
            nRF24_data->size.payload = pld_size;
        } else {
            nRF24_data->size.payload = (nRF24_read_register(nRF24_RX_PW_PIPE[pipe]));
        }

        // Read a payload from the RX FIFO
        if (nRF24_data->size.payload) {
            nRF24_read_multi_register(nRF24_CMD_R_RX_PAYLOAD, (uint8_t*) nRF24_data, nRF24_data->size.payload);
        }

        return ((nRF24_RXResult) pipe);
    }

    // The RX FIFO is empty
    nRF24_data->size.payload = 0;

    return nRF24_RX_EMPTY;
}

nRF24_RXResult nRF24_ReadPayload(nrf24_data_t* nRF24_data) {
    return nRF24_ReadPayloadGeneric(nRF24_data, false);
}

nRF24_RXResult nRF24_ReadPayloadDpl(nrf24_data_t* nRF24_data) {
    return nRF24_ReadPayloadGeneric(nRF24_data, true);
}

nRF24_TXResult nRF24_TransmitPacket(const nrf24_data_t* nRF24_data) {

    uint32_t wait = nRF24_WAIT_TIMEOUT;

    // Transfer a data from the specified buffer to the TX FIFO

    nRF24_WritePayload(nRF24_data);
    HAL_Delay(1);
    nRF24_enable();

    while ((!get_flag(NRF_SENT)) && (!get_flag(NRF_MAX_RT)) && wait) {
        HAL_Delay(1);
        wait--;
    }

    nRF24_disable();

    if (wait == 0) {
        // Timeout
        return nRF24_TX_TIMEOUT;
    }

    // Clear pending IRQ flags
    nRF24_ClearIRQFlags();

    if (get_flag(NRF_MAX_RT)) {
        // Auto retransmit counter exceeds the programmed maximum limit
        clear_flag(NRF_MAX_RT);
        nRF24_FlushTX();
        return nRF24_TX_MAXRT;
    }

    if (get_flag(NRF_SENT)) {
        // Successful transmission
        clear_flag(NRF_SENT);
        return nRF24_TX_SUCCESS;
    }

    nRF24_FlushTX();

    return nRF24_TX_ERROR;
}

void nRF24_WriteAckPayload(nRF24_RXResult pipe, const nrf24_data_t* nRF24_data) {
    uint8_t length = nRF24_data->size.payload;

    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_RESET);
    nRF24_SPI_TR(nRF24_CMD_W_ACK_PAYLOAD | pipe);
    while (length--) {
        nRF24_SPI_TR(*(const uint8_t*) nRF24_data++);
    }
    HAL_GPIO_WritePin(NRF_CS_GPIO, NRF_CS_PIN, GPIO_PIN_SET);
}

void nRF24_ReuseTX(void) {
    uint8_t status = nRF24_read_register(nRF24_REG_STATUS);
    nRF24_write_register(nRF24_REG_STATUS, status | nRF24_FLAG_MAX_RT);
    nRF24_write_register(nRF24_CMD_REUSE_TX_PL, nRF24_CMD_NOP);
    nRF24_disable();
    nRF24_enable();
}

void nRF24_StartCarrier(output_power_t pwr, uint8_t channel) {
    uint8_t rf_status;
    nrf24_data_t dummy = {
            .kind = COMMAND,
            .size = {
                    .data    = NRF24_TRANSF_MAX_LEN,
                    .payload = NRF24_TRANSF_MAX_LEN,
            }};

    nRF24_StopListening();
    rf_status = nRF24_read_register(nRF24_REG_RF_SETUP);
    nRF24_write_register(nRF24_REG_RF_SETUP, rf_status | nRF24_FLAG_CONT_WAVE | nRF24_FLAG_PLL_LOCK);

    if (nRF24.info.is_p_variant) {
        nRF24_write_register(nRF24_REG_EN_AA, 0x00);
        nRF24_SetAutoRetr(0, 0);
        for (uint8_t i = 0; i < PLD_LEN; i++) {
            dummy.data[i] = 0xFF;
        }

        nRF24_write_multi_register(nRF24_REG_TX_ADDR, dummy.data, 5);
        nRF24_FlushTX();
        nRF24_WritePayload(&dummy);
        nRF24_SetCRCScheme(nRF24_CRC_off);
    }
    nRF24_SetTXPower(pwr);
    nRF24_SetRFChannel(channel);
    nRF24_enable();
    if (nRF24.info.is_p_variant) {
        HAL_Delay(1);
        nRF24_disable();
        nRF24_ReuseTX();
    }
}

void nRF24_StopCarrier(void) {
    nRF24_SetPowerMode(nRF24_PWR_DOWN);
    uint8_t rf_status = nRF24_read_register(nRF24_REG_RF_SETUP);
    nRF24_write_register(nRF24_REG_RF_SETUP, (rf_status & ~nRF24_FLAG_PLL_LOCK) & ~nRF24_FLAG_CONT_WAVE);
    nRF24_disable();
}

bool nRF24_CarrierDetect(void) {
    nRF24.registers[nRF24_REG_RPD] = nRF24_read_register(nRF24_REG_RPD);
    return nRF24.registers[nRF24_REG_RPD] & 1;
}

static bool nRF24_SendData(const nrf24_data_t* nRF24_data) {

    nRF24_TXResult tx_res;

    if (nRF24_data->size.payload > 32) {
        return false;
    }

    // Transmit a packet
    tx_res = nRF24_TransmitPacket(nRF24_data);

    switch (tx_res) {
        case nRF24_TX_SUCCESS:
            return true;
        case nRF24_TX_MAXRT:
            nRF24_ResetPLOS();
            break;
        case nRF24_TX_TIMEOUT:
        case nRF24_TX_ERROR:
            break;
    }

    return false;
}

void nRF24_StartListening(void) {
    nRF24_SetPowerMode(nRF24_PWR_UP);
    nRF24_SetOperationalMode(nRF24_MODE_RX);
    nRF24_ClearIRQFlags();
    nRF24_enable();
}

void nRF24_StopListening(void) {
    nRF24_disable();
    HAL_Delay(100);
    nRF24_SetOperationalMode(nRF24_MODE_TX);
}

void nRF24_PrepareData(const void* in_data, const uint8_t in_data_len, const transf_t transfer_mode, nrf24_data_t* out_data) {
    out_data->kind         = transfer_mode;
    out_data->size.payload = PLD_LEN;

    if (in_data_len <= NRF24_TRANSF_MAX_LEN) {
        out_data->size.data = in_data_len;

        for (uint8_t i = 0; i < NRF24_TRANSF_MAX_LEN; i++) {
            out_data->data[i] = (i <= in_data_len) ? ((const uint8_t*) in_data)[i] : 0;
        }
    }
}

void nRF24_RetrieveData(const nrf24_data_t in_data, void* out_data) {

    if ((in_data.size.data > 0) && (in_data.size.data <= NRF24_TRANSF_MAX_LEN)) {
        for (uint8_t i = 0; i < in_data.size.data; i++) {
            ((uint8_t*) out_data)[i] = in_data.data[i];
        }
    }
}

bool nRF24_Talk(const nrf24_data_t question, nrf24_data_t* answer, transceiver_mode_t after_set) {
    uint8_t timeout = nRF24_WAIT_TIMEOUT;
    nRF24_StopListening();

    while (!nRF24_SendData(&question) && timeout) {
        timeout--;
        HAL_Delay(10);
    }

    if (timeout == 0) {
        if (after_set == nRF24_MODE_TX) {
            nRF24_StopListening();
        } else {
            nRF24_StartListening();
        }
        return false;
    }

    if ((answer != NULL) && (question.kind == REQUEST)) {
        timeout = nRF24_WAIT_TIMEOUT;
        nRF24_StartListening();
        while (!get_flag(NRF_RECEIVE) && timeout) {
            timeout--;
            HAL_Delay(10);
        }

        if (timeout == 0) {
            if (after_set == nRF24_MODE_TX) {
                nRF24_StopListening();
            } else {
                nRF24_StartListening();
            }
            return false;
        }

        clear_flag(NRF_RECEIVE);
        nRF24_GetData(answer);
    }

    if (after_set == nRF24_MODE_TX) {
        nRF24_StopListening();
    } else {
        nRF24_StartListening();
    }
    return true;
}

bool nRF24_GetData(nrf24_data_t* nRF24_payload) {
    if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
        // Get a payload from the transceiver
        nRF24_ReadPayload(nRF24_payload);

        return true;
    }

    return false;
}
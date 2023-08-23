/**
 * @file nrf_flags.h
 * @author Paulo Santos (pauloxrms@gmail.com)
 * @brief Declares the IRQ auxiliary functions for the nRF24L01 communication.
 * @version 0.1
 * @date 21-11-2022
 *
 * @copyright Copyright (c) 2023
 *
 */


#ifndef NRF24_FLAGS_H
#define NRF24_FLAGS_H

#include "stdbool.h"

typedef enum {
	NRF_SENT,
	NRF_RECEIVE,
	NRF_MAX_RT,
    IRQ_FLAGS_AMOUNT,
} irq_flag_t;

/**
 * @brief: Gets the boolean value of a flag.
 *
 * @param flag_var: Desired flag.
 * @retval true Flag is set.
 * @retval false Flag is not set.
 */
bool nrf_get_flag(irq_flag_t flag_var);

/**
 * @brief: Sets a given flag to True.
 *
 * @param flag_var: Desired flag.
 */
void nrf_set_flag(irq_flag_t flag_var);

/**
 * @brief: Sets a given flag to False.
 *
 * @param flag_var: Desired flag.
 */
void nrf_clear_flag(irq_flag_t flag_var);

#endif /* NRF24_FLAGS_H */
/**
 * @file nrf_flags.c
 * @author Paulo Santos (pauloxrms@gmail.com)
 * @brief Defines the IRQ auxiliary functions for the nRF24L01 communication.
 * @version 0.1
 * @date 21-11-2022
 *
 * @copyright Copyright (c) 2023
 *
 */


#include "nrf_flags.h"

static volatile bool irq_flags[IRQ_FLAGS_AMOUNT];

void nrf_set_flag(irq_flag_t flag_var) {
	irq_flags[flag_var] = true;
}

void nrf_clear_flag(irq_flag_t flag_var) {
	irq_flags[flag_var] = false;
}

bool nrf_get_flag(irq_flag_t flag_var) {
	return irq_flags[flag_var];
}
/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
 *
 * This is the private file for the a2dp common functionality.
 */

#pragma once

#include "wiced_bt_a2d.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_a2d_sbc.h"

/* Codec related functions */
extern uint8_t wiced_bt_a2dp_sbc_cfg_for_cap(uint8_t *p_peer,
    wiced_bt_a2d_sbc_cie_t *p_cap, wiced_bt_a2d_sbc_cie_t *p_pref);
extern uint8_t wiced_bt_a2dp_sbc_cfg_in_cap(uint8_t *p_cfg,
    wiced_bt_a2d_sbc_cie_t *p_cap);

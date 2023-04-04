/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/**
 * @file cy_asc_config_sbc_encoder.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */
#ifndef __CY_ASC_CONFIG_SBC_ENCODER_H__
#define __CY_ASC_CONFIG_SBC_ENCODER_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                      Structures
 ******************************************************************************/

/**
 * SBC encoder additional config structure
 */
typedef struct
{
    /**
     *  Number of sub bands - (4 or 8)
     */
    uint16_t sub_bands;

    /**
     * Number of sub blocks (4 or 8 or 12 or 16)
     */
    uint16_t sub_blocks;

    /**
     * Allocation method (Loudness:0 (SBC_LOUDNESS) , SNR: 1(SBC_SNR))
     */
    uint16_t allocation_method;

    /**
     * channel mode (0 for mono, 1 for Dual, 2 for stereo, 3 for joint stereo)
     */
    uint16_t channel_mode;

    /**
     * Bit pool, ranging from 2 to 250
     */
    uint16_t bit_pool;

    /**
     * SBC mode,  0 for A2DP mode, 1 for WideBand mode
     *          For WideBand mode - Only uuid is valid all other parameters were not considered
     */
    uint16_t sbc_mode;

    /**
     * UUID to be used. Need only for SBC Wideband
     * Currently UUID 2 value is supported in encoder and decoder
     * which corresponds to the below values,
     *      * Sub Band 8
     *      * Block 15
     *      * Bit Bool 26
     *      * Channel mode to Mono
     *      * Allocation method to Loudness
     *      * Sampling Freq to 16K
     */
    uint16_t uui_id;

}cy_asc_sbc_encode_config_t;

#ifdef __cplusplus
}
#endif

#endif /* __CY_ASC_CONFIG_SBC_ENCODER_H__ */

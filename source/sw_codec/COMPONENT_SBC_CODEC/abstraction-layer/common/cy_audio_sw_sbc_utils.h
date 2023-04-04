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
 * @file cy_asc_config_sbc_utils.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */
#ifndef __CY_AUDIO_SW_SBC_UTILS_H__
#define __CY_AUDIO_SW_SBC_UTILS_H__

/*******************************************************************************
 *                      Includes
 ******************************************************************************/
#include "cy_audio_sw_codecs.h"

/*******************************************************************************
 *                      Macros
 ******************************************************************************/
#define SBC_MODE_A2DP       0
#define SBC_MODE_WIDE_BAND  1

/* In final WB Cr, there is only 1 SBC configuration ie)UUID “2”: 2EV3, TeSCO 12 */
#define SBC_WB_SUPPORTED_UUID 2
#define SBC_DEC_BIT_POOL_MIN 2
#define SBC_DEC_BIT_POOL_MAX 250

/*******************************************************************************
 *                      Constants
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/


/*******************************************************************************
 *                      Structure
 ******************************************************************************/
typedef struct
{
    /* number of sub bands */
    uint16_t num_sub_bands;

    /* number of sub Block */
    uint16_t num_sub_blocks;

    /* model */
    uint16_t allocation_method;

    /* channel mode (0 for mono, 1 for Dual, 2 for stereo, 3 for joint stereo) */
    uint16_t channel_mode;

    /* sampling frequency (0 for 16k, 1 for 32k, 2 for 44.1k, 3 for 48k) */
    uint16_t sampling_freq;

    /* bit pool */
    uint16_t bit_pool;

    /* SBC Mode ( 0 for A2DP mode, 1 for WideBand mode) */
    uint16_t sbc_mode;

    /* UUI Id (for WideBand mode only !) */
    uint16_t uui_id;

}sbc_raw_params;

/*******************************************************************************
 *                      API declaration
 ******************************************************************************/
#if 0
uint16_t cy_asc_sbc_sub_band_enum_to_value(const cy_audio_codec_sbc_sub_bands_t e_sub_band);
uint16_t cy_asc_sbc_sub_block_enum_to_value(const cy_audio_codec_sbc_sub_blocks_t e_sub_block);
uint16_t cy_asc_sbc_enum_to_value_sub_bands(const cy_audio_codec_sbc_model_t e_model);
uint16_t cy_asc_sbc_channel_mode_enum_to_value(const cy_audio_codec_sbc_channel_mode_t e_ch_mode);
uint16_t cy_asc_sbc_mode_enum_to_value(const cy_audio_codec_sbc_mode_t e_mode);
#endif
uint16_t cy_asc_sbc_sample_rate_enum_to_value(const cy_audio_sw_codec_sampling_rate_t e_sample_rate);

#if 0
cy_audio_codec_sbc_sub_bands_t cy_asc_sbc_sub_band_value_to_enum(const uint16_t value);
cy_audio_codec_sbc_sub_blocks_t cy_asc_sbc_sub_block_value_to_enum(const uint16_t value);
cy_audio_codec_sbc_model_t cy_asc_sbc_value_to_enum_sub_bands(const uint16_t value);
cy_audio_codec_sbc_channel_mode_t cy_asc_sbc_channel_mode_value_to_enum(const uint16_t value);
cy_audio_codec_sbc_mode_t cy_asc_sbc_mode_value_to_enum(const uint16_t value);
#endif
cy_audio_sw_codec_sampling_rate_t cy_asc_sbc_sample_rate_value_to_enum(const uint16_t value);

#endif /* __CY_AUDIO_SW_SBC_UTILS_H__ */

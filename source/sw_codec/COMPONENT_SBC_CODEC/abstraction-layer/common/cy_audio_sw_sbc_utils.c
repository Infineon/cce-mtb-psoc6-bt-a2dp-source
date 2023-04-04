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
 * @file cy_asc_config_sbc_utils.c
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

/*******************************************************************************
 *                      Includes
 ******************************************************************************/
#include "cy_audio_sw_sbc_utils.h"
#include "sbc_encoder.h"

/*******************************************************************************
 *                      Macros
 ******************************************************************************/

/*******************************************************************************
 *                      Constants
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                      API declaration
 ******************************************************************************/

/*******************************************************************************
 *                      API definition
 ******************************************************************************/
#if 0
uint16_t cy_asc_sbc_sub_band_enum_to_value(const cy_audio_codec_sbc_sub_bands_t e_sub_band)
{
    uint16_t value = 0;
    switch(e_sub_band)
    {
        case CY_AUDIO_CODEC_SBC_SUB_BAND_4:
        {
            value = 4;
        }
        break;

        case CY_AUDIO_CODEC_SBC_SUB_BAND_8:
        {
            value = 8;
        }
        break;

        default:
        {
            value = 0;
        }
    }

    return value;
}

uint16_t cy_asc_sbc_sub_block_enum_to_value(const cy_audio_codec_sbc_sub_blocks_t e_sub_block)
{
    uint16_t value = 0;
    switch (e_sub_block)
    {
        case     CY_AUDIO_CODEC_SBC_SUB_BLOCK_4:
        {
            value = 4;
        }
        break;

        case CY_AUDIO_CODEC_SBC_SUB_BLOCK_8:
        {
            value = 8;
        }
        break;

        case CY_AUDIO_CODEC_SBC_SUB_BLOCK_12:
        {
            value = 12;
        }
        break;

        case CY_AUDIO_CODEC_SBC_SUB_BLOCK_16:
        {
            value = 16;
        }
        break;

        default:
            break;
    }
    return value;
}

uint16_t cy_asc_sbc_enum_to_value_sub_bands(const cy_audio_codec_sbc_model_t e_model)
{
    uint16_t value = SBC_LOUDNESS;
    switch (e_model)
    {
        case CY_AUDIO_CODEC_SBC_MODEL_LOUDNESS:
        {
            value = SBC_LOUDNESS;
        }
        break;

        case CY_AUDIO_CODEC_SBC_MODEL_SNR:
        {
            value = SBC_SNR;
        }
        break;

        default:
            value = SBC_LOUDNESS;
            break;
    }
    return value;
}


uint16_t cy_asc_sbc_channel_mode_enum_to_value(const cy_audio_codec_sbc_channel_mode_t e_ch_mode)
{
    uint16_t value = 0;

    switch (e_ch_mode)
    {
        case CY_AUDIO_CODEC_SBC_CHANNEL_MODE_MONO:
        {
            value = 0;
        }
        break;

        case CY_AUDIO_CODEC_SBC_CHANNEL_MODE_DUAL:
        {
            value = 1;
        }
        break;

        case CY_AUDIO_CODEC_SBC_CHANNEL_MODE_STEREO:
        {
            value = 2;
        }
        break;

        case CY_AUDIO_CODEC_SBC_CHANNEL_MODE_JOINT_STEREO:
        {
            value = 3;
        }
        break;

        default:
            value = 0;
            break;
    }

    return value;
}

uint16_t cy_asc_sbc_mode_enum_to_value(const cy_audio_codec_sbc_mode_t e_mode)
{
    uint16_t value = 0;

    switch (e_mode)
    {
        case     CY_AUDIO_CODEC_SBC_MODE_A2DP:
        {
            value = 0;
        }
        break;

        case CY_AUDIO_CODEC_SBC_MODE_WIDE_BAND:
        {
            value = 1;
        }
        break;

        default:
            value = 0;
            break;
    }

    return value;
}
#endif
uint16_t cy_asc_sbc_sample_rate_enum_to_value(const cy_audio_sw_codec_sampling_rate_t e_sample_rate)
{
    uint32_t value = 0;

    /* 0 for 16k, 1 for 32k, 2 for 44.1k, 3 for 48k */
     switch (e_sample_rate)
     {
        case  CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ:
        {
            value = 0;
        }
        break;

        case CY_AUDIO_CODEC_SAMPLING_RATE_32000HZ:
        {
            value = 1;
        }
        break;

        case CY_AUDIO_CODEC_SAMPLING_RATE_44100HZ:
        {
            value = 2;
        }
        break;

        case CY_AUDIO_CODEC_SAMPLING_RATE_48000HZ:
        {
            value = 3;
        }
        break;

        default:
            value = 0;
            break;
     }

     return value;
}

#if 0
cy_audio_codec_sbc_sub_bands_t cy_asc_sbc_sub_band_value_to_enum(const uint16_t value)
{
    cy_audio_codec_sbc_sub_bands_t e_sub_band = CY_AUDIO_CODEC_SBC_SUB_BAND_INVALID;

    switch (value)
    {
        case 4:
        {
            e_sub_band = CY_AUDIO_CODEC_SBC_SUB_BAND_4;
            break;
        }

        case 8:
        {
            e_sub_band = CY_AUDIO_CODEC_SBC_SUB_BAND_8;
            break;
        }
    }

    return e_sub_band;
}
cy_audio_codec_sbc_sub_blocks_t cy_asc_sbc_sub_block_value_to_enum(const uint16_t value)
{
    cy_audio_codec_sbc_sub_blocks_t e_sub_block = CY_AUDIO_CODEC_SBC_SUB_BLOCK_INVALID;

    switch (value)
    {
        case 4:
        {
            e_sub_block = CY_AUDIO_CODEC_SBC_SUB_BLOCK_4;
        }
        break;

        case 8:
        {
            e_sub_block = CY_AUDIO_CODEC_SBC_SUB_BLOCK_8;
        }
        break;

        case 12:
        {
            e_sub_block = CY_AUDIO_CODEC_SBC_SUB_BLOCK_12;
        }
        break;

        case 16:
        {
            e_sub_block = CY_AUDIO_CODEC_SBC_SUB_BLOCK_16;
        }
        break;
    }

    return e_sub_block;
}
cy_audio_codec_sbc_model_t cy_asc_sbc_value_to_enum_sub_bands(const uint16_t value)
{
    cy_audio_codec_sbc_model_t e_model = CY_AUDIO_CODEC_SBC_MODEL_INVALID;

    switch (value)
    {
        case SBC_LOUDNESS:
        {
            e_model = CY_AUDIO_CODEC_SBC_MODEL_LOUDNESS;
        }
        break;

        case SBC_SNR:
        {
            e_model = CY_AUDIO_CODEC_SBC_MODEL_SNR;
        }
        break;
    }

return e_model;
}
cy_audio_codec_sbc_channel_mode_t cy_asc_sbc_channel_mode_value_to_enum(const uint16_t value)
{
    cy_audio_codec_sbc_channel_mode_t e_ch_mode = CY_AUDIO_CODEC_SBC_CHANNEL_MODE_INVALID;

    switch (value)
    {
        case 0:
        {
        e_ch_mode = CY_AUDIO_CODEC_SBC_CHANNEL_MODE_MONO;
        }
            break;
        case 1:
        {
            e_ch_mode = CY_AUDIO_CODEC_SBC_CHANNEL_MODE_DUAL;
        }
            break;
        case 2:
        {
            e_ch_mode = CY_AUDIO_CODEC_SBC_CHANNEL_MODE_STEREO;
        }
            break;

        case 3:
        {
            e_ch_mode = CY_AUDIO_CODEC_SBC_CHANNEL_MODE_JOINT_STEREO;
        }
            break;
    }

    return e_ch_mode;
}

cy_audio_codec_sbc_mode_t cy_asc_sbc_mode_value_to_enum(const uint16_t value)
{
    cy_audio_codec_sbc_mode_t e_mode = CY_AUDIO_CODEC_SBC_MODE_INVALID;

    switch (value)
    {
        case 0:
            e_mode = CY_AUDIO_CODEC_SBC_MODE_A2DP;
            break;
        case 1:
            e_mode = CY_AUDIO_CODEC_SBC_MODE_WIDE_BAND;
            break;
    }

    return e_mode;
}
#endif
cy_audio_sw_codec_sampling_rate_t cy_asc_sbc_sample_rate_value_to_enum(const uint16_t value)
{
    cy_audio_sw_codec_sampling_rate_t e_sample_rate = CY_AUDIO_CODEC_SAMPLING_RATE_INVALID;

    switch (value)
    {
        case 0:
        {
            e_sample_rate = CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ;
        }
        break;

        case 1:
        {
            e_sample_rate = CY_AUDIO_CODEC_SAMPLING_RATE_32000HZ;
        }
        break;

        case 2:
        {
            e_sample_rate = CY_AUDIO_CODEC_SAMPLING_RATE_44100HZ;
        }
        break;

        case 3:
        {
            e_sample_rate = CY_AUDIO_CODEC_SAMPLING_RATE_48000HZ;
        }
        break;
    }

    return e_sample_rate;
}

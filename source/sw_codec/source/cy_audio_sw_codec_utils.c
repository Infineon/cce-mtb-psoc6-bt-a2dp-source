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
 * @file cy_audio_sw_codecs_utils.c
 *
 * @brief This file is the source file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

/*******************************************************************************
 *                      Includes
 ******************************************************************************/
#include "cy_audio_sw_codecs.h"
/*******************************************************************************
 *                      Macros
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                      Function decelerations
 ******************************************************************************/

/*******************************************************************************
 *                      Function definations
 ******************************************************************************/
uint32_t cy_audio_sw_codec_get_sampling_rate(cy_audio_sw_codec_sampling_rate_t e_sample_rate)
{
    uint32_t value = 0;
     switch (e_sample_rate)
     {
        case  CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ:
        {
            value = 16000;
        }
        break;

        case CY_AUDIO_CODEC_SAMPLING_RATE_32000HZ:
        {
            value = 32000;
        }
        break;

        case CY_AUDIO_CODEC_SAMPLING_RATE_44100HZ:
        {
            value = 44100;
        }
        break;

        case CY_AUDIO_CODEC_SAMPLING_RATE_48000HZ:
        {
            value = 48000;
        }
        break;

        default:
            break;
     }

     return value;
}

cy_audio_sw_codec_sampling_rate_t cy_audio_sw_codec_get_sampling_rate_enum(uint32_t sample_rate)
{
    cy_audio_sw_codec_sampling_rate_t value = CY_AUDIO_CODEC_SAMPLING_RATE_UNKNOWN;
     switch (sample_rate)
     {
        case  16000:
        {
            value = CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ;
        }
        break;

        case 32000:
        {
            value = CY_AUDIO_CODEC_SAMPLING_RATE_32000HZ;
        }
        break;

        case 44100:
        {
            value = CY_AUDIO_CODEC_SAMPLING_RATE_44100HZ;
        }
        break;

        case 48000:
        {
            value = CY_AUDIO_CODEC_SAMPLING_RATE_48000HZ;
        }
        break;

        default:
        break;
     }

     return value;
}

uint8_t cy_audio_sw_codec_get_channel_num(cy_audio_sw_codec_channel_t e_channel)
{
    uint8_t value = 0;
    switch (e_channel)
    {
        case CY_AUDIO_CODEC_CHANNEL_MONO:
        {
            value = 1;
        }
        break;

        case CY_AUDIO_CODEC_CHANNEL_STEREO:
        {
            value = 2;
        }
        break;

        default:
        {
            value = 1;
        }
        break;
    }
    return value;
}

cy_audio_sw_codec_channel_t cy_audio_sw_codec_get_channel_enum(uint8_t channel)
{
    uint8_t value = CY_AUDIO_CODEC_CHANNEL_UNKNOWN;
    switch (channel)
    {
        case 1:
        {
            value = CY_AUDIO_CODEC_CHANNEL_MONO;
        }
        break;

        case 2:
        {
            value = CY_AUDIO_CODEC_CHANNEL_STEREO;
        }
        break;

        default:
        break;
    }
    return value;
}

uint8_t cy_audio_sw_codec_get_bitwidth(cy_audio_sw_codec_bitwidth_t e_bitwidth)
{
    uint8_t value = 16;
    switch (e_bitwidth)
    {
        case CY_AUDIO_CODEC_BITWIDTH_16:
        {
            value = 16;
        }
        break;

        case CY_AUDIO_CODEC_BITWIDTH_24:
        {
            value = 24;
        }
        break;

        case CY_AUDIO_CODEC_BITWIDTH_32:
        {
            value = 32;
        }
        break;

        default:
        break;
    }

    return value;
}

cy_audio_sw_codec_bitwidth_t cy_audio_sw_codec_get_bitwidth_enum(uint8_t bitwidth)
{
    cy_audio_sw_codec_bitwidth_t e_value = CY_AUDIO_CODEC_BITWIDTH_16;
    switch (bitwidth)
    {
        case 16:
        {
            e_value = CY_AUDIO_CODEC_BITWIDTH_16;
        }
        break;

        case 24:
        {
            e_value = CY_AUDIO_CODEC_BITWIDTH_24;
        }
        break;

        case 32:
        {
            e_value = CY_AUDIO_CODEC_BITWIDTH_32;
        }
        break;

        default:
        break;
    }

    return e_value;
}

uint32_t cy_audio_sw_codec_get_bitrate_num(cy_audio_sw_codec_bitrate_t e_bitrate)
{
    uint32_t value = 0;
     switch (e_bitrate)
     {
        case CY_AUDIO_CODEC_BITRATE_32kbps:
        {
            value = 32000;
        }
        break;

        case CY_AUDIO_CODEC_BITRATE_64kbps:
        {
            value = 64000;
        }
        break;

        case CY_AUDIO_CODEC_BITRATE_96kbps:
        {
            value = 96000;
        }
        break;

        case CY_AUDIO_CODEC_BITRATE_128kbps:
        {
            value = 128000;
        }
        break;

        case CY_AUDIO_CODEC_BITRATE_196kbps:
        {
            value = 196000;
        }
        break;

        case CY_AUDIO_CODEC_BITRATE_320kbps:
        {
            value = 320000;
        }
        break;

        default:
        break;
     }

     return value;
}

cy_audio_sw_codec_bitrate_t cy_audio_sw_codec_get_bitrate_enum(uint32_t bitrate)
{
    cy_audio_sw_codec_bitrate_t ebitrate = CY_AUDIO_CODEC_BITRATE_UNKNOWN;
     switch (bitrate)
     {
        case 36000:
        {
            ebitrate = CY_AUDIO_CODEC_BITRATE_32kbps;
        }
        break;

        case 96000:
        {
            ebitrate = CY_AUDIO_CODEC_BITRATE_96kbps;
        }
        break;

        case 128000:
        {
            ebitrate = CY_AUDIO_CODEC_BITRATE_128kbps;
        }
        break;

        case 196000:
        {
            ebitrate = CY_AUDIO_CODEC_BITRATE_196kbps;
        }
        break;

        case 320000:
        {
            ebitrate = CY_AUDIO_CODEC_BITRATE_320kbps;
        }
        break;

        default:
            break;
     }

     return ebitrate;
}

uint32_t cy_audio_sw_codec_get_frame_ms(cy_audio_sw_codec_frame_ms_t e_frame_ms)
{
    uint32_t value = 10;
    switch (e_frame_ms)
    {
        case CY_AUDIO_CODEC_FRAME_MS_10:
        {
            value = 10;
        }
        break;

        case CY_AUDIO_CODEC_FRAME_MS_20:
        {
            value = 20;
        }
        break;

        default:
        break;
    }

    return value;
}

cy_audio_sw_codec_frame_ms_t cy_audio_sw_codec_get_frame_ms_enum(uint32_t value)
{
    cy_audio_sw_codec_frame_ms_t e_frame_ms = CY_AUDIO_CODEC_FRAME_MS_10;

        switch (value)
    {
        case 10:
        {
            e_frame_ms = CY_AUDIO_CODEC_FRAME_MS_10;
        }
        break;

        case 20:
        {
            e_frame_ms = CY_AUDIO_CODEC_FRAME_MS_20;
        }
        break;

        default:
        break;
    }

    return e_frame_ms;
}
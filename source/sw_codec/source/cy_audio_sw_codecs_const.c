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
 * @file cy_audio_sw_codecs_const.c
 *
 * @brief This file is the source file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

/*******************************************************************************
 *                      Includes
 ******************************************************************************/
#include "cy_audio_sw_codecs_common.h"

/*******************************************************************************
 *                      Macros
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                      Const
 ******************************************************************************/

/* Available codecs */
#ifdef ENABLE_LC3_ENCODE
extern const cy_audio_sw_codec_encoder_ops_t cy_audio_sw_encode_lc3;
#endif /* ENABLE_LC3_CODEC */

#ifdef ENABLE_LC3_DECODE
extern const cy_audio_sw_codec_decoder_ops_t cy_audio_sw_decode_lc3;
#endif /* ENABLE_LC3_DECODE */

#ifdef ENABLE_OPUS_ENCODE
extern const cy_audio_sw_codec_encoder_ops_t cy_audio_sw_encode_opus;
#endif /* ENABLE_OPUS_ENCODE */

#ifdef ENABLE_SBC_DECODE
extern const cy_audio_sw_codec_decoder_ops_t cy_audio_sw_decode_sbc;
#endif /* ENABLE_SBC_DECODE */

#ifdef ENABLE_SBC_ENCODE
extern const cy_audio_sw_codec_encoder_ops_t cy_audio_sw_encode_sbc;
#endif /* ENABLE_SBC_ENCODE */

const cy_audio_sw_codec_encoder_ops_t *const audio_sw_encode_ops[] =
{
#ifdef ENABLE_LC3_ENCODE
   &cy_audio_sw_encode_lc3,
#endif /* ENABLE_LC3_ENCODE */

#ifdef ENABLE_OPUS_ENCODE
    &cy_audio_sw_encode_opus,
#endif /* ENABLE_OPUS_ENCODE */

#ifdef ENABLE_SBC_ENCODE
    &cy_audio_sw_encode_sbc,
#endif /* ENABLE_SBC_ENCODE */

#ifdef ENABLE_MP3_CODEC
   &cy_audio_sw_codec_lc3,
#endif /* ENABLE_MP3_CODEC */

#ifdef ENABLE_AAC_CODEC
   &cy_audio_sw_codec_aac,
#endif /* ENABLE_AAC_CODEC */

#ifdef ENABLE_OGG_CODEC
   &cy_audio_sw_codec_ogg,
#endif /* ENABLE_OGG_CODEC */

   /* End of data */
   NULL
};

const cy_audio_sw_codec_decoder_ops_t *const audio_sw_decode_ops[] =
{
#ifdef ENABLE_LC3_DECODE
   &cy_audio_sw_decode_lc3,
#endif /* ENABLE_LC3_ENCODE */

#ifdef ENABLE_SBC_DECODE
    &cy_audio_sw_decode_sbc,
#endif /* ENABLE_SBC_DECODE */

#ifdef ENABLE_MP3_CODEC
   &cy_audio_sw_codec_lc3,
#endif /* ENABLE_MP3_CODEC */

#ifdef ENABLE_AAC_CODEC
   &cy_audio_sw_codec_aac,
#endif /* ENABLE_AAC_CODEC */

#ifdef ENABLE_OGG_CODEC
   &cy_audio_sw_codec_ogg,
#endif /* ENABLE_OGG_CODEC */

   /* End of data */
   NULL
};
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
 * @file cy_audio_sw_sbc_decode.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

#ifndef __CY_AUDIO_SW_SBC_DECODER_H__
#define __CY_AUDIO_SW_SBC_DECODER_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                      Includes
 ******************************************************************************/

#include "cy_audio_sw_codecs_common.h"
#include "cy_audio_sw_codecs_private.h"

/*******************************************************************************
 *                      Macros
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                      Type Definitions
 ******************************************************************************/
cy_rslt_t cy_audio_sw_sbc_decoder_init(
        cy_audio_sw_codec_decode_config_t *codec_config,
        cy_audio_sw_codec_t *handle);

cy_rslt_t cy_audio_sw_sbc_decode(
        cy_audio_sw_codec_handle_t audio_sw_hdl,
        uint8_t *in_encoded_buff,
        uint32_t *in_encoded_data_len,
        uint8_t *out_buff,
        uint32_t *outbuf_len);

cy_rslt_t cy_audio_sw_sbc_get_decoder_recommended_outbuf_size(
        cy_audio_sw_codec_handle_t audio_sw_hdl,
        cy_audio_sw_codec_stream_info_t *stream_info,
        uint32_t *recommended_out_size);

cy_rslt_t cy_audio_sw_sbc_decode_stream_info_header(
        cy_audio_sw_codec_handle_t audio_sw_hdl,
        uint8_t *in_encoded_buff,
        uint32_t *in_encoded_data_len,
        cy_audio_sw_codec_stream_info_t *stream_info);

cy_rslt_t cy_audio_sw_sbc_decoder_deinit(
        cy_audio_sw_codec_t *handle);

/*******************************************************************************
 *                      Constants
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __CY_AUDIO_SW_SBC_DECODER_H__ */

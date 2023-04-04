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
 * @file cy_audio_sw_sbc_encoder.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

#ifndef __CY_AUDIO_SW_SBC_ENCODER_H__
#define __CY_AUDIO_SW_SBC_ENCODER_H__
#ifdef __cplusplus
extern "C"
{
#endif

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
 *                      Type Definitions
 ******************************************************************************/

/**
 * @brief Initialize the SBC audio encoder
 *
 * @param codec_config [in] encoder config
 * @param handle [out] handle to the sw codec
 * @return cy_rslt_t - CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_audio_sw_sbc_encoder_init(
            cy_audio_sw_codec_encode_config_t *codec_config,
            cy_audio_sw_codec_t *handle);
/**
 * @brief performs frame level encoding operation
 *
 * @param audio_sw_hdl [in] handle to the sw codec
 * @param in_pcm_buff [in] input PCM buffer
 * @param in_pcm_data_len [in] size of the PCM buffer provided
 * @param out_buff [out] encoded output data
 * @param outbuf_len [out] encoded output data length
 * @return cy_rslt_t - CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_audio_sw_sbc_encode(
            cy_audio_sw_codec_handle_t audio_sw_hdl,
            uint8_t *in_pcm_buff,
            uint32_t *in_pcm_data_len,
            uint8_t *out_buff,
            uint32_t *outbuf_len);
/**
 * Get the recommended size of the input buffer for the codec encode
 * operation configured.
 *
 * @param[in] handle                Handle to the software codec instance
 * @param[out] recommended_in_size  Recommended Input buffer size
 * @return cy_rslt_t                on success; an error code on failure.
 */
cy_rslt_t cy_audio_sw_sbc_get_encoder_recommended_inbuf_size(
        cy_audio_sw_codec_handle_t audio_sw_hdl,
        uint32_t *recommended_in_size);

/**
 * Get the recommended size of the output buffer for the codec encoding
 * operation configured. Input buffer size is provided to get the
 * corresponding output buffer recommendation
 *
 * @param[in] handle                Handle to the software codec instance
 * @param[in] in_size               Input buffer size corresponding to the output buffer size
 * @param[out] recommended_out_size Recommended Output buffer size
 * @return cy_rslt_t                on success; an error code on failure.
 */
cy_rslt_t cy_audio_sw_sbc_get_encoder_recommended_outbuf_size(
        cy_audio_sw_codec_handle_t audio_sw_hdl,
        uint32_t in_size,
        uint32_t *recommended_out_size);

/**
 * @brief Deinitialize the SBC audio encoder
 *
 * @param handle [in] handle to the sw codec
 * @return cy_rslt_t - CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_audio_sw_sbc_encoder_deinit(
            cy_audio_sw_codec_t *handle);

/*******************************************************************************
 *                      Constants
 ******************************************************************************/
extern const cy_audio_sw_codec_encoder_ops_t cy_audio_sw_encode_sbc;

#endif /* __CY_AUDIO_SW_SBC_ENCODER_H__ */

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
 * @file cy_audio_sw_codecs_common.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

#ifndef __CY_AUDIO_SW_CODECS_COMMON_H__
#define __CY_AUDIO_SW_CODECS_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                      Includes
 ******************************************************************************/
#include "cy_audio_sw_codecs_log.h"
#include "cy_audio_sw_codecs_errors.h"
#include "cy_audio_sw_codecs.h"
#include "cy_audio_sw_codecs_private.h"

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
 *                    Operations codec fn ptr definitions
 ******************************************************************************/

typedef cy_rslt_t (*cy_audio_sw_encoder_init_t)(
        cy_audio_sw_codec_encode_config_t *codec_config,
        cy_audio_sw_codec_t *handle);

typedef cy_rslt_t (*cy_audio_sw_decoder_init_t)(
      cy_audio_sw_codec_decode_config_t *codec_config,
        cy_audio_sw_codec_t *handle);

typedef cy_rslt_t (*cy_audio_sw_codec_encode_t)(
        cy_audio_sw_codec_handle_t handle,
        uint8_t *in_pcm_buff,
        uint32_t *in_pcm_data_len,
        uint8_t *out_buff,
        uint32_t *outbuf_len);

typedef cy_rslt_t (*cy_audio_sw_codec_decode_t)(
        cy_audio_sw_codec_handle_t handle,
        uint8_t *in_encoded_buff,
        uint32_t *in_encoded_data_len,
        uint8_t *out_buff,
        uint32_t *outbuf_len);

typedef cy_rslt_t (*cy_audio_sw_codec_get_encoder_recommended_inbuf_size_t)(
        cy_audio_sw_codec_handle_t handle,
        uint32_t *recommended_in_size);

typedef cy_rslt_t (*cy_audio_sw_codec_get_encoder_recommended_outbuf_size_t)(
        cy_audio_sw_codec_handle_t handle,
        uint32_t in_size,
        uint32_t *recommended_out_size);

typedef cy_rslt_t (*cy_audio_sw_codec_get_decoder_recommended_outbuf_size_t)(
        cy_audio_sw_codec_handle_t handle,
        cy_audio_sw_codec_stream_info_t * stream_info,
        uint32_t *recommended_out_size);

typedef cy_rslt_t (*cy_audio_sw_codec_decode_stream_info_header_t)(
        cy_audio_sw_codec_handle_t handle,
        uint8_t *in_encoded_buff,
        uint32_t *in_encoded_data_len,
        cy_audio_sw_codec_stream_info_t *stream_info);

typedef cy_rslt_t (*cy_audio_sw_codec_encoder_deinit_t)(
        cy_audio_sw_codec_t *handle);

typedef cy_rslt_t (*cy_audio_sw_codec_decoder_deinit_t)(
        cy_audio_sw_codec_t *handle);

/*******************************************************************************
 *                      API Interface Structures
 ******************************************************************************/

/**
 * struct cy_audio_sw_codec_encode_ops_t - Audio SW encode operation interface API
 *
 * This structure defines the API that each LC3 interface needs to implement
 * for audio codec-mw. All encoder specific functionality is captured
 * in this wrapper.
 */
typedef struct cy_audio_sw_codec_encode_ops_
{
    /**
    * Audio Codec Type
    */
    cy_audio_sw_codec_type_t type;

    /**
    * Initialize api
    */
    cy_audio_sw_encoder_init_t init;

    /**
    * Process decoding
    */
    cy_audio_sw_codec_encode_t encode;

    /**
    * Get recommended input size for buffers
    */
    cy_audio_sw_codec_get_encoder_recommended_inbuf_size_t get_recommended_inbuf_size;

    /**
     * Get recommended output size for buffers
     */
    cy_audio_sw_codec_get_encoder_recommended_outbuf_size_t get_recommended_outbuf_size;

    /**
    * De-initialize api
    */
    cy_audio_sw_codec_encoder_deinit_t deinit;

}cy_audio_sw_codec_encoder_ops_t;


/**
 * struct cy_audio_sw_codec_decode_ops_ - Audio SW encode operation interface API
 *
 * This structure defines the API that each LC3 interface needs to implement
 * for audio codec-mw. All decoder specific functionality is captured
 * in this wrapper.
 */
typedef struct cy_audio_sw_codec_decode_ops_
{
    /**
    * Audio codec type
    */
    cy_audio_sw_codec_type_t type;

    /**
    * Initialize api
    */
    cy_audio_sw_decoder_init_t init;

    /**
    * Process decoding
    */
    cy_audio_sw_codec_decode_t decode;

    /**
    * Get recommended size for buffers
    */
    cy_audio_sw_codec_get_decoder_recommended_outbuf_size_t get_recommended_size;

    /**
     * Decode streaminfo header
     */
    cy_audio_sw_codec_decode_stream_info_header_t decode_stream_info_header;

    /**
    * De-initialize api
    */
    cy_audio_sw_codec_decoder_deinit_t deinit;

}cy_audio_sw_codec_decoder_ops_t;


typedef struct cy_audio_sw_buff_
{
    /**
     * buffer ptr
     */
    uint8_t *buff;

    /**
     * buffer length
     */
    uint32_t buff_len;

    /**
    * Length of occupied/filled buffer.
    */
    uint32_t used_len;

}cy_audio_sw_buff_t;


/*******************************************************************************
 *                      Function decelerations
 ******************************************************************************/

/**
 * Cache the residual data if there were extra input data is provided
 *
 * @param input_buff                Internal Input buffer handle
 * @param user_buff                 User input buffer pointer
 * @param processed_user_buff_len   Bytes processed on user buffer
 * @param tot_user_buff_len         Total user buffer length
 */
void cy_asc_check_and_cache_any_residual_data(cy_audio_sw_buff_t *input_buff,
        uint8_t *user_buff, uint32_t processed_user_buff_len,
        uint32_t tot_user_buff_len);

/**
 * check if sufficient data available to process. if the user data is less,
 * data is appended to the residual cache buffer
 *
 * @param input_buff        Internal Input buffer handle
 * @param user_buffer       User input buffer pointer
 * @param user_buffer_len   Bytes processed on user buffer
 * @return true             if user has provided sufficient data to process
 * @return false            No sufficient data available, and the data is
 *                          cached internally.
 */
bool cy_asc_check_sufficient_buffer_process(cy_audio_sw_buff_t *input_buff,
        uint8_t *user_buffer,uint32_t user_buffer_len);

#ifdef __cplusplus
}
#endif

#endif /* __CY_AUDIO_SW_CODECS_COMMON_H__ */

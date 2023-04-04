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
 * @file cy_audio_sw_codecs.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

#ifndef __CY_AUDIO_SW_CODECS_H__
#define __CY_AUDIO_SW_CODECS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "cy_result.h"
#include "cy_audio_sw_codecs_log.h"
#include "cy_audio_sw_codecs_errors.h"
#if ENABLE_SBC_DECODE
#include "cy_asc_config_sbc_decoder.h"
#endif
#if ENABLE_SBC_ENCODE
#include "cy_asc_config_sbc_encoder.h"
#endif

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 *                      Macros
 ******************************************************************************/

/*******************************************************************************
 *                      Constants
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/**
 * Codec Type
 */
typedef enum
{
    /**
     * Invalid Codec type
     */
    CY_AUDIO_CODEC_TYPE_UNKNOWN,
    /**
     * LC3
     */
    CY_AUDIO_CODEC_TYPE_LC3,
    /**
     * OPUS
     */
    CY_AUDIO_CODEC_TYPE_OPUS,
    /**
     * SBC
     */
    CY_AUDIO_CODEC_TYPE_SBC,
    /**
     * MP3
     */
    CY_AUDIO_CODEC_TYPE_MP3,
    /**
     * AAC
     */
    CY_AUDIO_CODEC_TYPE_AAC,
    /**
     * OGG
     */
    CY_AUDIO_CODEC_TYPE_OGG,
    /**
     * Codec type maximum
     */
    CY_AUDIO_CODEC_TYPE_MAX

} cy_audio_sw_codec_type_t;

/**
 * Codec Operation
 */
typedef enum
{
    /**
     * Codec Operation Invalid
     */
    CY_AUDIO_CODEC_OPERATION_INVALID,
    /**
     * Encode operation
     */
    CY_AUDIO_CODEC_OPERATION_ENCODE,
    /**
     * Decode operation
     */
    CY_AUDIO_CODEC_OPERATION_DECODE,

    /**
     * Codec Operation Max
     */
    CY_AUDIO_CODEC_OPERATION_MAX

} cy_audio_sw_codec_operation_t;

/**
 * Sampling Rate
 */
typedef enum
{
    /**
     * Sampling Rate Invalid
     */
    CY_AUDIO_CODEC_SAMPLING_RATE_INVALID,

    /**
     * Sampling Rate Unknown
     */
    CY_AUDIO_CODEC_SAMPLING_RATE_UNKNOWN,

    /**
     * Sampling Rate 16KHz
     */
    CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ,
    /**
     * Sampling Rate 32KHz
     */
    CY_AUDIO_CODEC_SAMPLING_RATE_32000HZ,
    /**
     * Sampling Rate 44.1KHz
     */
    CY_AUDIO_CODEC_SAMPLING_RATE_44100HZ,
    /**
     * Sampling Rate 48KHz
     */
    CY_AUDIO_CODEC_SAMPLING_RATE_48000HZ,
    /**
     * Sampling Rate Maximum
     */
    CY_AUDIO_CODEC_SAMPLING_RATE_MAX

} cy_audio_sw_codec_sampling_rate_t;

/**
 * Audio channel information
 */
typedef enum
{
    /**
     * Channels Invalid
     */
    CY_AUDIO_CODEC_CHANNEL_INVALID,
    /**
     * Channel information unknown
     */
    CY_AUDIO_CODEC_CHANNEL_UNKNOWN,
    /**
     * Channel Mono
     */
    CY_AUDIO_CODEC_CHANNEL_MONO,
    /**
     * Channels Stereo
     */
    CY_AUDIO_CODEC_CHANNEL_STEREO,
    /**
     * Channels Maximum supported
     */
    CY_AUDIO_CODEC_CHANNEL_MAX

} cy_audio_sw_codec_channel_t;

/**
 * Audio bitwidth information
 */
typedef enum
{
    /**
     * Bitwidth Invalid
     */
    CY_AUDIO_CODEC_BITWIDTH_INVALID,
    /**
     * Bitwidth of audio unknown
     */
    CY_AUDIO_CODEC_BITWIDTH_UNKNOWN,
    /**
     * Bitwidth 16 bit
     */
    CY_AUDIO_CODEC_BITWIDTH_16,
    /**
     * Bitwidth 24 bit
     */
    CY_AUDIO_CODEC_BITWIDTH_24,
    /**
     * Bitwidth 32 bit
     */
    CY_AUDIO_CODEC_BITWIDTH_32,
    /**
     * Bitwidth Maximum
     */
    CY_AUDIO_CODEC_BITWIDTH_MAX

} cy_audio_sw_codec_bitwidth_t;

typedef enum
{
    /**
     * Bitwidth Invalid
     */
    CY_AUDIO_CODEC_BITRATE_INVALID,
    /**
     * Bitwidth of audio unknown
     */
    CY_AUDIO_CODEC_BITRATE_UNKNOWN,
    /**
     * bitrate @ 32kbps
     */
    CY_AUDIO_CODEC_BITRATE_32kbps,
    /**
     * bitrate @ 64kbps
     */
    CY_AUDIO_CODEC_BITRATE_64kbps,
    /**
     * bitrate @ 96kbps
     */
    CY_AUDIO_CODEC_BITRATE_96kbps,
    /**
     * bitrate @ 128kbps
     */
    CY_AUDIO_CODEC_BITRATE_128kbps,
    /**
     * bitrate @ 192kbps
     */
    CY_AUDIO_CODEC_BITRATE_196kbps,
    /**
     * bitrate @ 320kbps
     */
    CY_AUDIO_CODEC_BITRATE_320kbps,
    /**
     * Bitrate Maximum
     */
    CY_AUDIO_CODEC_BITRATE_MAX

} cy_audio_sw_codec_bitrate_t;

typedef enum
{
    /**
     * Frame ms Invalid
     */
    CY_AUDIO_CODEC_FRAME_MS_INVALID,
    /**
     * Frame mso unknown
     */
    CY_AUDIO_CODEC_FRAME_MS_UNKNOWN,
    /**
     * Frame 10 ms
     */
    CY_AUDIO_CODEC_FRAME_MS_10,
    /**
     * Frame 20 ms
     */
    CY_AUDIO_CODEC_FRAME_MS_20,

    /**
     * fame ms Maximum
     */
    CY_AUDIO_CODEC_FRAME_MS_MAX

} cy_audio_sw_codec_frame_ms_t;

/*******************************************************************************
 *                      Type Definitions
 ******************************************************************************/

/**
 * audio software codec instance
 */
typedef void* cy_audio_sw_codec_t;

/*******************************************************************************
 *                      Structures
 ******************************************************************************/

/**
 * Audio Software Codec decode specific configuration.
 */
typedef union
{
    /**
     * To avoid build error in case no additional config
     */
    void *Empty;

#if ENABLE_SBC_DECODE
    /**
     * Additional struct specific to SBC codec
     */
    cy_asc_sbc_decode_config_t sbc;
#endif

}cy_audio_sw_codec_specific_decode_config_t;

/**
 * Audio Software Codec Encode specific configuration.
 */
typedef union
{
    /**
     * To avoid build error in case no additional config
     */
    void *Empty;

#if ENABLE_SBC_ENCODE
    /**
     * Additional struct specific to SBC codec
     */
    cy_asc_sbc_encode_config_t sbc;
#endif

}cy_audio_sw_codec_specific_encode_config_t;


/**
 * Audio Software Codec Encode specific stream info.
 */
typedef union
{
    /**
     * To avoid build error in case no additional config
     */
    void *Empty;

#if ENABLE_SBC_DECODE
    /**
     * Additional struct specific to SBC codec
     */
    cy_asc_sbc_decode_stream_info_t sbc;
#endif
}cy_audio_sw_codec_decode_stream_info_t;

/**
 * Stream information. After decoding the headers, the decoder would use this
 * structure to notify the application about the input stream details.
 */
typedef struct
{
    /**
     * Sampling Rate
     */
    cy_audio_sw_codec_sampling_rate_t sampling_rate;
    /**
     * Audio channel information
     */
    cy_audio_sw_codec_channel_t channel;
    /**
     * Audio bit-width information
     */
    cy_audio_sw_codec_bitwidth_t bitwidth;

    cy_audio_sw_codec_decode_stream_info_t codec_specific_info;

} cy_audio_sw_codec_stream_info_t;


/*******************************************************************************
 *                      Callback Definitions
 ******************************************************************************/
/**
 * Stream information callback. Once the software codec (decoder) instance
 * identifies the stream information  like, sampling rate, channels, bitwidth
 * then, decoder instance will provide callback to the application. Application
 * can understand the decoded stream information.
 *
 * This callback would be called whenever there is a change in the stream
 * information .
 *
 * @param[in] handle                Handle to the software codec (decoder) instance
 *                                  2. Application can hold the buffer pointers
 *                                  and post it to next MW for its processing.
 * @param[in] stream_info           Information about decoded stream.
 * @param[in] callback_user_arg     User argument passed in \ref cy_audio_sw_codec_init
 *                                  using \ref cy_audio_sw_codec_decode_config_t
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
typedef cy_rslt_t (*cy_audio_sw_codec_decode_stream_info_callback_t)(
        cy_audio_sw_codec_t handle,
        cy_audio_sw_codec_stream_info_t stream_info,
        void *callback_user_arg);

/*******************************************************************************
 *                      API Interface Structures
 ******************************************************************************/

/**
 * Audio Software Codec Encode configuration.
 */
typedef struct
{
    /**
     * Codec type used for encoding, Following are the supported codecs for
     * Encoding:
     *      1. CY_AUDIO_CODEC_TYPE_LC3
     *      2. CY_AUDIO_CODEC_TYPE_OPUS
     *  Other codecs are not supported for encoding
     */
    cy_audio_sw_codec_type_t codec_type;

    /**
     * Sampling rate
     */
    cy_audio_sw_codec_sampling_rate_t sampling_rate;

    /**
     * Channel informations
     */
    cy_audio_sw_codec_channel_t channel;

    /**
     * Bit-width informations
     */
    cy_audio_sw_codec_bitwidth_t bitwidth;

    /**
     * bitrate informations
     */
    cy_audio_sw_codec_bitrate_t bitrate;

    /**
     * frame duration informations
     */
    cy_audio_sw_codec_frame_ms_t frame_ms;

    /**
     * Additional config based on codec usage
     */
    cy_audio_sw_codec_specific_encode_config_t codec_specific_config;

} cy_audio_sw_codec_encode_config_t;

/**
 * Audio Software Codec Decode configuration.
 */
typedef struct
{
    /**
     * Codec used for decoding
     */
    cy_audio_sw_codec_type_t codec_type;

    /**
     * Sampling rate of the data.
     *
     * Application needs to provide this parameter compulsorily if the data
     * to be decoded is not having the codec header (example: lc3 header).
     * i.e.,If the codec header is not available in the stream and if the application
     * has not passed this information, then codec-mw will not be able to decode
     * any data.
     *
     * If the data to be decoded has the codec header, then codec-mw will internally
     * parse the codec header and extract this information. In this case,
     * Application can optionally provide this parameter if it is known already,
     * if this information is not known then application can pass
     * CY_AUDIO_CODEC_SAMPLING_RATE_UNKNOWN.
     */
    cy_audio_sw_codec_sampling_rate_t sampling_rate;

    /**
     * Channel informations of the data.
     *
     * Application needs to provide this parameter compulsorily if the data
     * to be decoded is not having the codec header (example: lc3 header).
     * i.e.,If the codec header is not available in the stream and if the application
     * has not passed this information, then codec-mw will not be able to decode
     * any data.
     *
     * If the data to be decoded has the codec header, then codec-mw will internally
     * parse the codec header and extract this information. In this case,
     * Application can optionally provide this parameter if it is known already,
     * if this information is not known then application can pass
     * CY_AUDIO_CODEC_CHANNEL_UNKNOWN.
     */
    cy_audio_sw_codec_channel_t channel;

    /**
     * Bitwidth information
     *
     * Application needs to provide this parameter compulsory if the data
     * to be decoded is not having the codec header (example: lc3 header).
     * i.e.,If the codec header is not available in the stream and if the application
     * has not passed this information, then codec-mw will not be able to decode
     * any data.
     *
     * If the data to be decoded has the codec header, then codec-mw will internally
     * parse the codec header and extract this information. In this case,
     * Application can optionally provide this parameter if it is known already,
     * if this information is not known then application can pass
     * CY_AUDIO_CODEC_BITWIDTH_UNKNOWN.
     *
     */
    cy_audio_sw_codec_bitwidth_t bitwidth;

    /**
     * Application specific user argument, Passed argument will be passed
     * back to application in the MW callbacks
     * \ref: cy_audio_sw_codec_decode_stream_info_callback_t
     */
    void *callback_user_arg;

    /**
     * Decoded stream information callback. Used to notify application about
     * the information about the stream fed through \ref
     * cy_audio_sw_codec_decode.
     *
     * Dynamic change of stream info is not supported. Change in the stream info
     * is notified via this callback and application is expected to do deinit and init
     * and decode new data.
     */
    cy_audio_sw_codec_decode_stream_info_callback_t stream_info;


    /**
     * Additional config based on codec usage
     */
    cy_audio_sw_codec_specific_decode_config_t codec_specific_config;

} cy_audio_sw_codec_decode_config_t;

/*******************************************************************************
 *                      Global Variables
 ******************************************************************************/

/*******************************************************************************
 *                      Function Declarations
 ******************************************************************************/

/**
 * Initializes the software codec for either decoding and encoding.
 *
 * @param[in]  codec_operation      Codec operation to be done (either encode
 *                                  / decode)
 * @param[in]  codec_config         Codec configuration parameter.
 *                                  1) If the codec_operation refers to
 *                                  CY_AUDIO_CODEC_OPERATION_ENCODE, then the
 *                                  codec_config should corresponds to
 *                                  cy_audio_sw_codec_encode_config_t
 *                                  2) If the codec_operation refers to
 *                                  CY_AUDIO_CODEC_OPERATION_DECODE, then the
 *                                  codec_config should corresponds to
 *                                  cy_audio_sw_codec_decode_config_t
 * @param[out]  handle              Pointer to store the audio software codec
 *                                  instance
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_audio_sw_codec_init(
        cy_audio_sw_codec_operation_t codec_operation,
        void *codec_config,
        cy_audio_sw_codec_t *handle);

/**
 * Encodes the input PCM data and returns the encoded data.
 *
 * @param[in]  handle               Handle to the software codec instance
 * @param[in]  in_pcm_buff          Pointer to input PCM data buffer.
 * @param[in/out]  in_pcm_data_len  1) Length of the input data referred by in_pcm_buff.
 *                                  2) Once the API returns, return number of bytes
 *                                  processed/accepted successfully in the codec-mw
 *                                  as part of processing.
 * @param[in]  out_buff             Pointer to output encoded buffer
 * @param[in/out]  outbuf_len       1) Length of the output buffer.
 *                                  2) Once the API returns, return number of bytes
 *                                  filled in the out_buff will be returned in the
 *                                  outbuf_len pointer.
 * @return      uint32_t            On error a negative value is returned. On
 *                                  success, zero or the number of bytes used to
 *                                  encode the data read from the input buffer.
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_audio_sw_codec_encode(
        cy_audio_sw_codec_t handle,
        uint8_t *in_pcm_buff,
        uint32_t *in_pcm_data_len,
        uint8_t *out_buff,
        uint32_t *outbuf_len);

/**
 * Decodes the encoded data and returns the PCM data.
 *
 * @param[in]  handle               Handle to the software codec instance
 * @param[in]  in_encoded_buff      Pointer to input encoded data.
 * @param[in]  in_encoded_data_len  1) Length of the input data referred by in_encoded_data_len.
 *                                  2) Once the API returns, return number of bytes
 *                                  processed/accepted successfully in the codec-mw
 *                                  as part of processing.
 * @param[in]  out_buff             Pointer to output PCM data
 * @param[in/out]  outbuf_len       1) Length of the output buffer.
 *                                  2) Once the API returns, return number of bytes
 *                                  filled in the output buffer will be returned
 *                                  in the outbuf_len pointer.
 * @return      uint32_t            On error a negative value is returned,
 *                                  otherwise the number of bytes used or zero
 *                                  if no frame data was decompressed (used)
 *                                  from the input buffer.
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 *            CY_RSLT_ASC_HEADER_NOT_FOUND - error code if no stream info provided
 *                                           during init and no header found in
 *                                           buffer "in_encoded_buff".
 */
cy_rslt_t cy_audio_sw_codec_decode(
        cy_audio_sw_codec_t handle,
        uint8_t *in_encoded_buff,
        uint32_t *in_encoded_data_len,
        uint8_t *out_buff,
        uint32_t *outbuf_len);

/**
 * Get the recommended size of the input buffer for the codec encode
 * operation configured.
 *
 *  1.  Input size can be any length but suggested to have multiples
 *      of the recommended size.
 *  2.  For optimal, suggested passing multiples of recommended input,
 *      so that internal copy could be avoided.
 *  3.  use api cy_audio_sw_codec_get_encoder_recommended_outbuf_size
 *      to get the recommended output size for the input size.
 *  4.  Since it is PCM data length is fixed for provided config (sample rate,  bit width, channel ),
 *      accurate prediction is possible for input.
 *
 * @param[in] handle                Handle to the software codec instance
 * @param[out] recommended_in_size  Recommended Input buffer size
 * @return cy_rslt_t                on success; an error code on failure.
 */
cy_rslt_t cy_audio_sw_codec_get_encoder_recommended_inbuf_size(
        cy_audio_sw_codec_t handle,
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
cy_rslt_t cy_audio_sw_codec_get_encoder_recommended_outbuf_size(
        cy_audio_sw_codec_t handle,
        uint32_t in_size,
        uint32_t *recommended_out_size);

/**
 * Get the recommended size of the output buffer for the codec decoding
 * operation configured.
 *
 *  1.  if the application knows the stream info prior, this API call can
 *      be invoked at any point in time.
 *
 *  2.  if the application doesn't know the stream info prior, application
 *      can all the cy_audio_sw_codec_decode_stream_info_header and get
 *      the stream info and then call this API to get the right recommended
 *      out size.
 *
 * @param[in] handle                Handle to the software codec instance
 * @param[in] stream_info           Information about decoded stream.
 * @param[out] recommended_out_size Recommended Output buffer size
 * @return cy_rslt_t                on success; an error code on failure.
 */
cy_rslt_t cy_audio_sw_codec_get_decoder_recommended_outbuf_size(
        cy_audio_sw_codec_t handle,
        cy_audio_sw_codec_stream_info_t *stream_info,
        uint32_t *recommended_out_size);

/**
 * Parse the header information from the input encoded data, used with decoder.
 * This API can be used by the application to know the stream information, so that
 *  'cy_audio_sw_codec_get_decoder_recommended_outbuf_size' can give the right
 * output size for 'cy_audio_sw_codec_decode' api.
 *
 * @param[in] handle                    Handle to the software codec instance
 * @param[in] in_encoded_buff           Pointer to input encoded data.
 * @param[in/out] in_encoded_data_len   1) Length of the input data referred by in_encoded_data_len.
 *                                      2) Once the API returns, return number of bytes
 *                                         processed/accepted successfully in the codec-mw
 *                                         as part of processing.
 * @param[out] stream_info
 * @return cy_rslt_t                    on success,
 *                                      in_encoded_data_len             - Offset to the end of header.
 *
 *                                      an error code on failure,
 *                                      1. CY_RSLT_ASC_HEADER_NOT_FOUND - no input data is accepted.
 *                                                                        appl should send next buffers
 *                                      2. CY_RSLT_ASC_PARTIAL_HEADER_IDENTIFIED
 *                                                                      - returns accepted data in in_encoded_data_len,
 *                                                                        appl can provide continuous data
 */
cy_rslt_t cy_audio_sw_codec_decode_stream_info_header(
            cy_audio_sw_codec_t handle,
            uint8_t *in_encoded_buff,
            uint32_t *in_encoded_data_len,
            cy_audio_sw_codec_stream_info_t *stream_info);

/**
 * Deinit the software codec instance.
 *
 * @param[out]  handle     Address of the Handle to audio software codec
 *                         instance created by the \ref cy_audio_sw_codec_init
 *                         API.Once API returns\ref cy_audio_sw_codec_t handle
 *                         will be reset to NULL.
 *
 * @return    CY_RSLT_SUCCESS on success; an error code on failure.
 */
cy_rslt_t cy_audio_sw_codec_deinit(cy_audio_sw_codec_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* __CY_AUDIO_SW_CODECS_H__ */

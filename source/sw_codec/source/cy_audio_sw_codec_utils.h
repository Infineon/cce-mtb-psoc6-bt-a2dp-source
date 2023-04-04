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
 * @file cy_audio_sw_codec_utils.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

#ifndef __CY_AUDIO_SW_CODECS_UTILS_H__
#define __CY_AUDIO_SW_CODECS_UTILS_H__

#ifdef __cplusplus
extern "C"
{
#endif
/*******************************************************************************
 *                      Includes
 ******************************************************************************/
#include "cy_audio_sw_codecs.h"

/*******************************************************************************
 *                      Macros
 ******************************************************************************/
#define IS_SUFFICIENT_BUFF_LEN(_BUFF_LEN_,_REQ_LEN_) (_BUFF_LEN_ >= (_REQ_LEN_))?true:false

/* Macro to get the Max data len can be used */
#define GET_DATA_LEN_TO_COPY(BUFF_MAX_LEN,BUFF_PRE_FILL_LEN, DATA_LEN)  ((BUFF_MAX_LEN-BUFF_PRE_FILL_LEN) < DATA_LEN)?(BUFF_MAX_LEN-BUFF_PRE_FILL_LEN):DATA_LEN;
/*******************************************************************************
 *                      Constants
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/**
 * @brief Convert the enum sampling rate to actual value
 *
 * @param e_sample_rate [in]sampling rate enum
 * @return uint32_t sampling rate value
 */
uint32_t cy_audio_sw_codec_get_sampling_rate(cy_audio_sw_codec_sampling_rate_t e_sample_rate);

/**
 * @brief Convert the actual sampling rate to enum
 *
 * @param sample_rate [in]sampling rate value
 * @return cy_audio_sw_codec_sampling_rate_t - sampling rate enum
 */
cy_audio_sw_codec_sampling_rate_t cy_audio_sw_codec_get_sampling_rate_enum(uint32_t sample_rate);

/**
 * @brief Conver the enum to actual value
 *
 * @param e_channel input enum cy_audio_sw_codec_channel_t
 * @return uint8_t  Number of channels
 */
uint8_t cy_audio_sw_codec_get_channel_num(cy_audio_sw_codec_channel_t e_channel);

/**
 * @brief Conver the actual value to enum
 *
 * @param channel Number of channels
 * @return cy_audio_sw_codec_channel_t out enum cy_audio_sw_codec_channel_t
 */
cy_audio_sw_codec_channel_t cy_audio_sw_codec_get_channel_enum(uint8_t channel);

/**
 * @brief get bitwidth in values
 *
 * @param e_bitwidth enum of cy_audio_sw_codec_bitwidth_t
 * @return uint8_t  value after conversion
 */
uint8_t cy_audio_sw_codec_get_bitwidth(cy_audio_sw_codec_bitwidth_t e_bitwidth);

/**
 * @brief get the enum bitwidth
 *
 * @param bitwidth bandwith value
 * @return cy_audio_sw_codec_bitwidth_t - enum of cy_audio_sw_codec_bitwidth_t
 */
cy_audio_sw_codec_bitwidth_t cy_audio_sw_codec_get_bitwidth_enum(uint8_t bitwidth);

/**
 * @brief convert the bitrate value to enum cy_audio_sw_codec_bitrate_t
 *
 * @param bitrate - bitrate value
 * @return cy_audio_sw_codec_bitrate_t
 */
cy_audio_sw_codec_bitrate_t cy_audio_sw_codec_get_bitrate_enum(uint32_t bitrate);

/**
 * @brief convert the bitrate cy_audio_sw_codec_bitrate_t enum to value
 *
 * @param e_bitrate bitrate value enum
 * @return uint32_t - bitrate value
 */
uint32_t cy_audio_sw_codec_get_bitrate_num(cy_audio_sw_codec_bitrate_t e_bitrate);

/**
 * @brief convert the frame ms cy_audio_sw_codec_frame_ms_t enum to value
 *
 * @param e_bitrate frame ms value enum
 * @return uint32_t - frame ms value
 */
uint32_t cy_audio_sw_codec_get_frame_ms(cy_audio_sw_codec_frame_ms_t e_frame_ms);

/**
 * @brief convert the frame ms value to cy_audio_sw_codec_bitrate_t
 *
 * @param value frame ms value
 * @return cy_audio_sw_codec_frame_ms_t - frame ms enum
 */
cy_audio_sw_codec_frame_ms_t cy_audio_sw_codec_get_frame_ms_enum(uint32_t value);
#ifdef __cplusplus
}
#endif

#endif /* __CY_AUDIO_SW_CODECS_UTILS_H__ */

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
 * @file cy_audio_sw_lc3_decoder_private.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

#ifndef __CY_AUDIO_SW_SBC_DECODER_PVT_H__
#define __CY_AUDIO_SW_SBC_DECODER_PVT_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                      Includes
 ******************************************************************************/
#include "sbc_decoder.h"
#include "sbc_dec_func_declare.h"

#include "cy_audio_sw_codecs_common.h"
#include "cy_audio_sw_codecs_private.h"

/*******************************************************************************
 *                      Macros
 ******************************************************************************/
#define SBC_ENC_HEADER_LEN 4
#define SBC_ENC_HEADER_BUFF_LEN 4*20
/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

typedef enum lc3_dec_stream_info_state
{
    /* stream info not available */
    ASC_SBC_DEC_STRM_INFO_NOT_AVAILABLE,

    /* stream info available */
    ASC_SBC_DEC_STRM_INFO_AVAILABLE,
}sbc_dec_stream_info_state_t;

typedef enum lc3_decode_state
{
    /* Invalid state */
    ASC_SBC_DEC_STATE_INVALID,

    ASC_SBC_DEC_STATE_ALGO_NOT_INITIALIZED,

    /* Read next frame or header */
    ASC_SBC_DEC_PARSE_HEADER,

    /* Decode frame */
    ASC_SBC_DEC_DECODE_FRAME,

    /* Max state */
    ASC_SBC_DEC_DECODE_MAX,

}sbc_decode_state_t;

/*******************************************************************************
 *                      struct
 ******************************************************************************/


typedef struct sbc_decoder_handle_
{

    /* SBC decoder handle */
    SBC_DEC_PARAMS *decoder;

    // /* To hold all the allocated memory */
    // void* memory_alloc[MAX_MEM_ALLOCS_ENC];

    // /* Memory allocated count */
    // uint8_t malloc_count;

    /* Input Buffer Buffer */
    cy_audio_sw_buff_t  input_buff;

    // /* Buffer to hold the header */
    // cy_audio_sw_buff_t  header_buff;

    /* Recommended out size */
    uint32_t recommended_out_size;

    /* Decode state */
    sbc_decode_state_t decode_state;

    /* Stream info */
    cy_audio_sw_codec_stream_info_t stream_info;

    /* Stream info state */
    sbc_dec_stream_info_state_t stream_info_state;

    /* Encoder config */
    cy_audio_sw_codec_decode_config_t dec_config;

    // /* SBC param in raw values */
    // sbc_raw_params raw_param;

    /* Last parsed Frame Length */
    uint32 frame_len;



}sbc_decoder_handle;

/*******************************************************************************
 *                      typedef
 ******************************************************************************/
typedef sbc_decoder_handle * sbc_decoder_handle_t;

#ifdef __cplusplus
}
#endif

#endif /* __CY_AUDIO_SW_SBC_DECODER_PVT_H__ */

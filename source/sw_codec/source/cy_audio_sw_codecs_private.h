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
 * @file cy_audio_sw_codecs_private.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

#ifndef __CY_AUDIO_SW_CODECS_PRIVATE_H__
#define __CY_AUDIO_SW_CODECS_PRIVATE_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                      Includes
 ******************************************************************************/

#include "cy_result.h"
#include <stdbool.h>

/*******************************************************************************
 *                      Macros
 ******************************************************************************/
#define ENABLE_STATS
/*******************************************************************************
 *                      Constants
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/**
 * Audio channel information
 */
typedef enum
{
    /**
    * No Header
    */
    CY_AUDIO_CODEC_NO_HEADER,
    /**
    * Include header once in
    * beginning
    */
    CY_AUDIO_CODEC_HEADER_NO_START,
    /**
    * include Header on all frames
    */
    CY_AUDIO_CODEC_NO_ALL_FRAME,

    /**
    * include Header on all frames
    */
    CY_AUDIO_CODEC_HEADER_ON_EACH_CALL,

} cy_audio_sw_codec_header_schema_t ;

/**
 * Audio channel information
 */
typedef enum cy_audio_sw_codec_state
{
   /**
    * Not initialized
    */
   CY_AUDIO_CODEC_STATE_NOT_INITIALIZED,
   /**
    * Initialized
    */
   CY_AUDIO_CODEC_STATE_INITIALIZED,
   /**
    * include Header on all frames
    */
   CY_AUDIO_CODEC_STATE_DEINITIALIZED,

} cy_audio_sw_codec_state_t ;


typedef struct audio_sw_codec_buffer_
{
   /**
    * allocated buffer pointer
    */
   uint8_t *alloc_buff;

   /**
    * buffer ptr after alignment adjustment
    */
   uint8_t *buff;

   /**
    * buffer length
    */
   uint32_t buff_len;

   /**
    * buffer alignment required
    */
   uint32_t alignment;

}audio_sw_codec_buff_t;

#ifdef ENABLE_STATS
typedef struct cy_audio_sw_encoder_stats_
{
   uint32_t TotalEncodedData;

   uint32_t TotalInputData;

   uint32_t NumberOfFramesEncoded;
}cy_audio_sw_encoder_stats_t;

typedef struct cy_audio_sw_decoder_stats_
{
   uint32_t TotalDecodedData;

   uint32_t TotalInputData;

   uint32_t NumberOfFramesDecoded;

}cy_audio_sw_decoder_stats_t;

typedef struct cy_audio_sw_stats_
{
   union
   {
      cy_audio_sw_encoder_stats_t enc_stat;
      cy_audio_sw_decoder_stats_t dec_stat;
   };
}cy_audio_sw_codec_stats_t;
#endif

typedef struct cy_audio_sw_codec_handle_
{
   /**
    * Based on codec type, encode/decode operations struct is set
    */
   void * pst_audio_sw_codec_ops;

   /**
    * Object to sw codec(Lc3/aac/etc.,)
    */
   void * audio_sw_algo_object;

   /**
    * Codec operation to be done (either encode/decode)
    */
   cy_audio_sw_codec_operation_t codec_operation;
#if 0
   /**
    * Input buffer info - needed by codec layer
    */
   audio_sw_codec_buff_t algo_in_buff;

   /**
    * Output buffer info - needed by codec layer
    */
   audio_sw_codec_buff_t algo_out_buff;

    /**
    * input/changed stream information
    */
   cy_audio_sw_codec_stream_info_t stream_info;
#endif
   /**
    * Header inclusion schema
    */
   cy_audio_sw_codec_header_schema_t header_schema;

   /**
    * Current state
    */
   cy_audio_sw_codec_state_t state;

#ifdef ENABLE_STATS
   cy_audio_sw_codec_stats_t  statistics;
#endif /* ENABLE_STATS */

}cy_audio_sw_codec_handle;

/*******************************************************************************
 *                      typedef
 ******************************************************************************/
typedef cy_audio_sw_codec_handle * cy_audio_sw_codec_handle_t;


#ifdef __cplusplus
}
#endif
#endif /* __CY_AUDIO_SW_CODECS_PRIVATE_H__ */
